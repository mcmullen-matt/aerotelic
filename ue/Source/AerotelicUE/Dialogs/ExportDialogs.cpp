#include "ExportDialogs.h"
#include "Widgets/MessageBox.h"
#include "BodyPicker.h"

#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Framework/Application/SlateApplication.h"
#include "Styling/CoreStyle.h"
#include "Misc/MessageDialog.h"
#include "Misc/Paths.h"
#include "DesktopPlatformModule.h"
#include "IDesktopPlatform.h"

#include "Widgets/CurveTabControls.h"
#include "Widgets/NumericEntry.h"
#include "Export/AerotelicExport.h"
#include "aerotelic/sdf.hpp"
#include "aerotelic/shell.hpp"
#include "aerotelic/tet.hpp"

// Last: OCCT's Handle() macro vs engine headers.
#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicLatticeModel.h"
#include "Entity/AerotelicEntityActor.h"

#define LOCTEXT_NAMESPACE "ExportDialogs"

namespace
{
	FString Thousands(int64 N)
	{
		return FText::AsNumber(N).ToString();
	}

	TSharedRef<SWidget> FormRow(const FText& Label, TSharedRef<SWidget> Field)
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0)
			[ SNew(SBox).WidthOverride(140) [ SNew(STextBlock).Text(Label) ] ]
			+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
			[ Field ];
	}

	TSharedRef<STextBlock> Stat(TSharedPtr<STextBlock>& Out)
	{
		return SAssignNew(Out, STextBlock).Text(FText::FromString(TEXT("Ã¢â‚¬â€")))
			.ColorAndOpacity(FSlateColor(FLinearColor(0.85f, 0.85f, 0.85f)));
	}

	bool AskSavePath(const TSharedRef<SWidget>& Parent, const FString& Title, const FString& Filter, const FString& Extension, FString& OutPath)
	{
		IDesktopPlatform* Desktop = FDesktopPlatformModule::Get();
		if (Desktop == nullptr)
		{
			return false;
		}
		TArray<FString> Files;
		const void* Handle = FSlateApplication::Get().FindBestParentWindowHandleForDialogs(Parent);
		if (!Desktop->SaveFileDialog(Handle, Title, TEXT(""), TEXT(""), Filter, EFileDialogFlags::None, Files) || Files.Num() == 0)
		{
			return false;
		}
		OutPath = FPaths::ConvertRelativePathToFull(Files[0]);
		if (FPaths::GetExtension(OutPath).IsEmpty())
		{
			OutPath += Extension;
		}
		return true;
	}

	TSharedRef<SWindow> MakeWindow(const FText& Title, FVector2D Size)
	{
		return SNew(SWindow).Title(Title).ClientSize(Size).SupportsMaximize(false).SupportsMinimize(false);
	}
}

// ------------------------------------------------------------ skin (VTK)

void ShowAerotelicExportSkinVtkDialog(TFunction<void(const FText&)> Status)
{
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	aerotelic::TetMesh Merged;
	int32 Lattices = 0;
	if (Doc != nullptr)
	{
		for (auto& Weak : Doc->GetEntities())
		{
			if (!Weak.IsValid() || Weak->GetEntityType() != aerotelic::project_yaml::EntityType::Lattice) continue;
			aerotelic::TetMesh S = Weak->GetModelAs<FAerotelicLatticeModel>()->MergedSkinTets();
			const int Offset = static_cast<int>(Merged.nodes.size());
			Merged.nodes.insert(Merged.nodes.end(), S.nodes.begin(), S.nodes.end());
			for (const auto& T : S.tets) Merged.tets.push_back({T[0] + Offset, T[1] + Offset, T[2] + Offset, T[3] + Offset});
			Merged.constrained.insert(Merged.constrained.end(), S.constrained.begin(), S.constrained.end());
			Merged.material.insert(Merged.material.end(), S.material.begin(), S.material.end());
			++Lattices;
		}
	}
	if (Merged.tets.empty())
	{
		AerotelicMessageBox(EAppMsgType::Ok, LOCTEXT("NoSkin", "No lattice skin to export: generate a lattice first (its skin is built with it)."), LOCTEXT("SkinTitle", "Export Skin Tetrahedra"));
		return;
	}
	TSharedPtr<SWindow> Active = FSlateApplication::Get().GetActiveTopLevelWindow();
	TSharedRef<SWidget> Parent = Active.IsValid() ? StaticCastSharedRef<SWidget>(Active.ToSharedRef()) : SNullWidget::NullWidget;
	FString Path;
	if (!AskSavePath(Parent, TEXT("Export Skin Tetrahedra"), TEXT("VTK Files (*.vtk)|*.vtk"), TEXT(".vtk"), Path))
	{
		return;
	}
	Merged.neighbors.assign(Merged.tets.size(), {{-1, -1, -1, -1}});
	if (!aerotelic::WriteTetMeshVtk(Merged, TCHAR_TO_UTF8(*Path)))
	{
		AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(FString::Printf(TEXT("Could not write %s"), *Path)), LOCTEXT("ExportError", "Export Error"));
		return;
	}
	Status(FText::FromString(FString::Printf(TEXT("Exported %s skin tetrahedra of %d lattice(s) to %s"), *Thousands(static_cast<int64>(Merged.tets.size())), Lattices, *Path)));
}

// ------------------------------------------------- skin + core tetrahedra (VTK)

void ShowAerotelicExportTetVtkDialog(TFunction<void(const FText&)> Status)
{
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	// The lattices and their targets.
	struct FJob
	{
		FString Label;
		aerotelic::TriMesh Surface;
		double Skin = 0.0;
		double Cell = 0.0;
	};
	TArray<FJob> Jobs;
	TArray<FString> Missing;
	if (Doc != nullptr)
	{
		for (auto& Weak : Doc->GetEntities())
		{
			if (!Weak.IsValid() || Weak->GetEntityType() != aerotelic::project_yaml::EntityType::Lattice) continue;
			const FAerotelicLatticeModel* Model = Weak->GetModelAs<FAerotelicLatticeModel>();
			// The surface at the lattice's own mesh tolerance (a few voxels finer than its grid).
			const double Voxel = Model->Spec.cell_size / FMath::Max(Model->Spec.resolution, 2);
			const double Tolerance = FMath::Max(Voxel * 0.25, 1e-5);
			for (aerotelic::NamedMesh& Body : Model->TargetBodies(Tolerance, Missing))
			{
				FJob J;
				J.Label = Weak->EntityName + TEXT(": ") + UTF8_TO_TCHAR(Body.name.c_str());
				J.Surface = std::move(Body.mesh);
				J.Skin = Model->Spec.skin_thickness;
				J.Cell = Model->Spec.cell_size;
				Jobs.Add(MoveTemp(J));
			}
		}
	}
	if (Jobs.Num() == 0)
	{
		AerotelicMessageBox(EAppMsgType::Ok, LOCTEXT("NoTetBodies", "No body to mesh: generate a lattice first (its bodies are what is meshed)."), LOCTEXT("TetTitle", "Export Tetrahedral Mesh"));
		return;
	}
	TSharedPtr<SWindow> Active = FSlateApplication::Get().GetActiveTopLevelWindow();
	TSharedRef<SWidget> Parent = Active.IsValid() ? StaticCastSharedRef<SWidget>(Active.ToSharedRef()) : SNullWidget::NullWidget;
	FString Path;
	if (!AskSavePath(Parent, TEXT("Export Tetrahedral Mesh"), TEXT("VTK Files (*.vtk)|*.vtk"), TEXT(".vtk"), Path))
	{
		return;
	}

	// Mesh every body and merge; the materials carry over (1 skin, 2 core).
	aerotelic::TetMesh Merged;
	TArray<FString> Failures, Notes;
	int64 SkinTets = 0, CoreTets = 0, Abandoned = 0;
	double Seconds = 0.0;
	for (FJob& J : Jobs)
	{
		Status(FText::FromString(FString::Printf(TEXT("Meshing %s..."), *J.Label)));
		try
		{
			if (!J.Surface.IsClosedManifold() || !(J.Surface.SignedVolume() > 0.0))
			{
				Failures.Add(J.Label + TEXT(": not a closed, outward surface"));
				continue;
			}
			aerotelic::MeshSdf Sdf(J.Surface);
			aerotelic::SkinCoreOptions Opt;
			Opt.thickness = J.Skin > 0.0 ? J.Skin : 0.01 * J.Surface.Bounds().Diagonal();
			Opt.core_spacing = J.Cell;
			// Delaunay refinement of the interior to a radius-edge ratio of 2,
			// then flips and smoothing (tet.hpp): the mesh is for stress.
			Opt.conforming.refine_radius_edge = 2.0;
			Opt.improve = true;
			aerotelic::SkinCoreStats St;
			aerotelic::TetMesh M = aerotelic::BuildSkinCoreMesh(J.Surface, Sdf, Opt, &St);
			const int Offset = static_cast<int>(Merged.nodes.size());
			Merged.nodes.insert(Merged.nodes.end(), M.nodes.begin(), M.nodes.end());
			for (size_t K = 0; K < M.tets.size(); ++K)
			{
				const auto& T = M.tets[K];
				Merged.tets.push_back({T[0] + Offset, T[1] + Offset, T[2] + Offset, T[3] + Offset});
				Merged.material.push_back(M.material[K]);
			}
			SkinTets += St.n_skin_tets;
			CoreTets += St.n_core_tets;
			Abandoned += St.conforming.n_faces_abandoned;
			Seconds += St.seconds;
			UE_LOG(LogTemp, Log, TEXT("AerotelicTet: %s: %d skin + %d core tets, skin %.5g core %.5g (surface %.5g), %d Steiner on edges, %d on faces, %d faces abandoned, min dihedral skin %.2f core %.2f, %.2f s"),
				*J.Label, St.n_skin_tets, St.n_core_tets, St.skin_volume, St.core_volume, St.conforming.surface_volume, St.conforming.n_edge_steiner, St.conforming.n_face_steiner,
				St.conforming.n_faces_abandoned, St.min_dihedral_skin, St.min_dihedral_core, St.seconds);
			if (St.conforming.n_faces_abandoned > 0)
			{
				Notes.Add(FString::Printf(TEXT("%s: %d surface faces could not be conformed to exactly (the mesh boundary follows the tetrahedra there)"), *J.Label, St.conforming.n_faces_abandoned));
			}
		}
		catch (const std::exception& E)
		{
			Failures.Add(J.Label + TEXT(": ") + UTF8_TO_TCHAR(E.what()));
		}
	}
	if (Merged.tets.empty())
	{
		Status(LOCTEXT("Ready", "Ready"));
		AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(TEXT("No body could be meshed:\n") + FString::Join(Failures, TEXT("\n"))), LOCTEXT("TetTitle", "Export Tetrahedral Mesh"));
		return;
	}
	if (!aerotelic::WriteTetMeshVtk(Merged, TCHAR_TO_UTF8(*Path)))
	{
		Status(LOCTEXT("Ready", "Ready"));
		AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(FString::Printf(TEXT("Could not write %s"), *Path)), LOCTEXT("ExportError", "Export Error"));
		return;
	}
	Status(FText::FromString(FString::Printf(TEXT("Exported %s skin + %s core tetrahedra of %d bodies in %.1f s to %s"), *Thousands(SkinTets), *Thousands(CoreTets), Jobs.Num() - Failures.Num(), Seconds, *Path)));
	if (Failures.Num() > 0 || Notes.Num() > 0 || Missing.Num() > 0)
	{
		FString Text;
		if (Failures.Num() > 0) Text += TEXT("Written without these bodies:\n") + FString::Join(Failures, TEXT("\n")) + TEXT("\n");
		if (Missing.Num() > 0) Text += TEXT("Targets with no geometry: ") + FString::Join(Missing, TEXT(", ")) + TEXT("\n");
		if (Notes.Num() > 0) Text += FString::Join(Notes, TEXT("\n"));
		AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(Text), LOCTEXT("TetTitle", "Export Tetrahedral Mesh"));
	}
}

// -------------------------------------------------------------------- STL

void ShowAerotelicExportStlDialog(TFunction<void(const FText&)> Status)
{
	TArray<FAerotelicBodyChoicePtr> Choices = SceneBodyChoices();
	if (Choices.Num() == 0)
	{
		AerotelicMessageBox(EAppMsgType::Ok, LOCTEXT("NothingStl", "Nothing in the scene to export."), LOCTEXT("StlTitle", "Export STL"));
		return;
	}

	TSharedRef<SWindow> Window = MakeWindow(LOCTEXT("StlTitle", "Export STL"), FVector2D(520, 560));
	TSharedPtr<SAerotelicBodyPicker> Picker;
	TSharedPtr<STextBlock> LblBodies, LblNodes, LblTris;
	TSharedPtr<SCheckBox> AsciiCheck;
	double Deflection = 0.01, AngleDeg = 20.0;
	AerotelicExport::FStlMesh Meshed;

	// The mesh is rebuilt when the body selection changes and re-meshed
	// when a density changes; it is exactly what Save writes out.
	auto Refresh = [&]()
	{
		TArray<FAerotelicBooleanOperand> Operands;
		TArray<FString> Labels;
		for (const FAerotelicBodyChoicePtr& C : Picker->GetChosen())
		{
			Operands.Add(C->Operand);
			Labels.Add(C->Label);
		}
		if (Operands.Num() == 0)
		{
			Meshed = AerotelicExport::FStlMesh();
			LblBodies->SetText(FText::FromString(TEXT("Ã¢â‚¬â€")));
			LblNodes->SetText(FText::FromString(TEXT("Ã¢â‚¬â€")));
			LblTris->SetText(FText::FromString(TEXT("Ã¢â‚¬â€")));
			Status(LOCTEXT("SelectBody", "Select at least one body to export."));
			return;
		}
		Status(LOCTEXT("Meshing", "Meshing..."));
		TArray<FString> Failures;
		Meshed = AerotelicExport::MeshForStl(Operands, Labels, Deflection, AngleDeg, Failures);
		const int32 Bodies = Meshed.Bodies;
		const int64 Nodes = Meshed.Nodes, Tris = Meshed.Triangles.Num();
		if (Meshed.IsEmpty())
		{
			LblBodies->SetText(FText::FromString(TEXT("Ã¢â‚¬â€")));
			LblNodes->SetText(FText::FromString(TEXT("Ã¢â‚¬â€")));
			LblTris->SetText(FText::FromString(TEXT("Ã¢â‚¬â€")));
			Status(FText::FromString(TEXT("No body could be built: ") + FString::Join(Failures, TEXT("; "))));
			return;
		}
		LblBodies->SetText(FText::FromString(Thousands(Bodies)));
		LblNodes->SetText(FText::FromString(Thousands(Nodes)));
		LblTris->SetText(FText::FromString(Thousands(Tris)));
		Status(FText::FromString(FString::Printf(TEXT("%s triangles across %s bodies at %g / %gÃ‚Â°%s"), *Thousands(Tris), *Thousands(Bodies), Deflection, AngleDeg,
			Failures.Num() ? *(TEXT("; skipped: ") + FString::Join(Failures, TEXT("; "))) : TEXT(""))));
	};

	auto Save = [&]()
	{
		if (Meshed.IsEmpty())
		{
			AerotelicMessageBox(EAppMsgType::Ok, LOCTEXT("SelectBodyWarn", "Select at least one body to export."), LOCTEXT("StlTitle", "Export STL"));
			return;
		}
		FString Path;
		if (!AskSavePath(Window, TEXT("Export STL"), TEXT("STL Files (*.stl)|*.stl"), TEXT(".stl"), Path))
		{
			return;
		}
		const bool bAscii = AsciiCheck->IsChecked();
		FString Error;
		if (!AerotelicExport::WriteStl(Meshed, Path, bAscii, Error))
		{
			Status(LOCTEXT("Ready", "Ready"));
			AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(Error), LOCTEXT("ExportError", "Export Error"));
			return;
		}
		Status(FText::FromString(FString::Printf(TEXT("Exported %s triangles Ã¢â€ â€™ %s"), *LblTris->GetText().ToString(), *Path)));
		Window->RequestDestroyWindow();
	};

	auto Spin = [&](double& Var, double Min, double Max, double Delta, const FText& Suffix)
	{
		return SNew(SAerotelicNumericEntry)
			.Value_Lambda([&Var]() { return Var; })
			.MinValue(Min).MaxValue(Max).Delta(Delta).Suffix(Suffix)
			.LiveUpdate(false)
			.OnValueCommitted_Lambda([&Var, &Refresh](double V, ETextCommit::Type) { if (V != Var) { Var = V; Refresh(); } });
	};

	Window->SetContent(
		SNew(SBorder).Padding(12)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
			[
				MakeGroup(LOCTEXT("Bodies", "Bodies to export"),
					SAssignNew(Picker, SAerotelicBodyPicker).Choices(Choices).ListHeight(180.0f)
					.OnChanged(FSimpleDelegate::CreateLambda([&Refresh]() { Refresh(); })))
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
			[
				MakeGroup(LOCTEXT("Density", "Mesh Density"),
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Linear", "Linear deflection:"), Spin(Deflection, 1e-9, 1e9, 0.005, FText::GetEmpty())) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Angular", "Angular deflection:"), Spin(AngleDeg, 1e-9, 180.0, 1.0, FText::FromString(TEXT(" Ã‚Â°")))) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
					[ MakeInfoLabel(LOCTEXT("DensityHint", "Linear: the chordal sag a triangle may have from the true surface. Angular: the most adjacent facet normals may differ. Smaller is denser.")) ])
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
			[
				MakeGroup(LOCTEXT("Mesh", "Mesh"),
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("BodiesN", "Bodies:"), Stat(LblBodies)) ]
					+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("Nodes", "Nodes:"), Stat(LblNodes)) ]
					+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("Triangles", "Triangles:"), Stat(LblTris)) ])
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 10)
			[
				MakeGroup(LOCTEXT("Output", "Output"),
					SAssignNew(AsciiCheck, SCheckBox)
					[ SNew(STextBlock).Text(LOCTEXT("Ascii", "ASCII STL  (larger; binary is written otherwise)")) ])
			]
			+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[ SNew(SButton).Text(LOCTEXT("SaveStl", "Save STL...")).OnClicked_Lambda([&Save]() { Save(); return FReply::Handled(); }) ]
				+ SHorizontalBox::Slot().AutoWidth()
				[ SNew(SButton).Text(LOCTEXT("Close", "Close")).OnClicked_Lambda([Window]() { Window->RequestDestroyWindow(); return FReply::Handled(); }) ]
			]
		]);

	Status(LOCTEXT("SelectBody", "Select at least one body to export."));
	FSlateApplication::Get().AddModalWindow(Window, FSlateApplication::Get().GetActiveTopLevelWindow());
}

// ------------------------------------------------------------------- STEP

void ShowAerotelicExportStepDialog(TFunction<void(const FText&)> Status)
{
	// Solid bodies only: a boolean result and a lattice are triangle meshes
	// with no B-rep (export a boolean's operands and combine them downstream).
	TArray<FAerotelicBodyChoicePtr> Choices = SceneBodyChoices();
	Choices.RemoveAll([](const FAerotelicBodyChoicePtr& C)
	{
		using aerotelic::project_yaml::EntityType;
		return !C->Operand.Entity.IsValid() || C->Operand.Entity->GetEntityType() == EntityType::Boolean || C->Operand.Entity->GetEntityType() == EntityType::Lattice;
	});
	if (Choices.Num() == 0)
	{
		AerotelicMessageBox(EAppMsgType::Ok, LOCTEXT("NothingStep", "Nothing in the scene to export as STEP (solid bodies only; boolean results and lattices are meshes)."), LOCTEXT("StepTitle", "Export STEP"));
		return;
	}
	if (AerotelicExport::FindStepWriter().IsEmpty())
	{
		AerotelicMessageBox(EAppMsgType::Ok,
			LOCTEXT("NoWriter", "The STEP writer (aerotelic_step_writer.exe) is not installed beside the application.\nBuild the core and run ue\\tools\\stage_step_writer.ps1."),
			LOCTEXT("StepTitle", "Export STEP"));
		return;
	}

	TSharedRef<SWindow> Window = MakeWindow(LOCTEXT("StepTitle", "Export STEP"), FVector2D(520, 520));
	TSharedPtr<SAerotelicBodyPicker> Picker;
	TSharedPtr<STextBlock> LblBodies;
	TSharedPtr<SCheckBox> RebuildCheck, NamedCheck;
	double StationTol = AerotelicExport::ExportStationTolerance();

	auto RefreshCount = [&]()
	{
		const int32 N = Picker->GetChosen().Num();
		LblBodies->SetText(FText::FromString(N ? Thousands(N) : TEXT("Ã¢â‚¬â€")));
		Status(N ? FText::FromString(FString::Printf(TEXT("%s bodies selected for STEP export"), *Thousands(N))) : LOCTEXT("SelectBody", "Select at least one body to export."));
	};

	auto Save = [&]()
	{
		const TArray<FAerotelicBodyChoicePtr>& Chosen = Picker->GetChosen();
		if (Chosen.Num() == 0)
		{
			AerotelicMessageBox(EAppMsgType::Ok, LOCTEXT("SelectBodyWarn", "Select at least one body to export."), LOCTEXT("StepTitle", "Export STEP"));
			return;
		}
		FString Path;
		if (!AskSavePath(Window, TEXT("Export STEP"), TEXT("STEP Files (*.step;*.stp)|*.step;*.stp"), TEXT(".step"), Path))
		{
			return;
		}
		const TOptional<double> Tol = RebuildCheck->IsChecked() ? TOptional<double>(StationTol) : TOptional<double>();
		Status(LOCTEXT("Building", "Building bodies for export..."));
		TArray<AerotelicExport::FBody> Bodies;
		TArray<FString> Failures;
		for (const FAerotelicBodyChoicePtr& C : Chosen)
		{
			FString Error;
			TopoDS_Shape Shape = AerotelicExport::StepBodyShape(C->Operand, Tol, Error);
			if (Shape.IsNull())
			{
				Failures.Add(C->Label + TEXT(": ") + (Error.IsEmpty() ? TEXT("produced no shape") : Error));
				continue;
			}
			Bodies.Add({ C->Label, Shape });
		}
		if (Bodies.Num() == 0)
		{
			Status(LOCTEXT("Ready", "Ready"));
			AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(TEXT("No body could be built:\n") + FString::Join(Failures, TEXT("\n"))), LOCTEXT("StepTitle", "Export STEP"));
			return;
		}
		Status(LOCTEXT("Writing", "Writing STEP..."));
		FString Error;
		if (!AerotelicExport::WriteStep(Bodies, Path, NamedCheck->IsChecked(), Error))
		{
			Status(LOCTEXT("Ready", "Ready"));
			AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(Error), LOCTEXT("ExportError", "Export Error"));
			return;
		}
		Status(FText::FromString(FString::Printf(TEXT("Exported %s bodies Ã¢â€ â€™ %s"), *Thousands(Bodies.Num()), *Path)));
		if (Failures.Num() > 0)
		{
			AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(TEXT("Written without these bodies:\n") + FString::Join(Failures, TEXT("\n"))), LOCTEXT("StepTitle", "Export STEP"));
		}
		Window->RequestDestroyWindow();
	};

	Window->SetContent(
		SNew(SBorder).Padding(12)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
			[
				MakeGroup(LOCTEXT("Bodies", "Bodies to export"),
					SAssignNew(Picker, SAerotelicBodyPicker).Choices(Choices).ListHeight(180.0f)
					.OnChanged(FSimpleDelegate::CreateLambda([&RefreshCount]() { RefreshCount(); })))
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
			[
				MakeGroup(LOCTEXT("Geometry", "Geometry"),
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
					[
						FormRow(LOCTEXT("StationTol", "Station tolerance:"),
							SNew(SAerotelicNumericEntry)
							.Value_Lambda([&StationTol]() { return StationTol; })
							.MinValue(1e-9).MaxValue(1e9).Delta(1e-4)
							.LiveUpdate(false)
							.OnValueCommitted_Lambda([&StationTol](double V, ETextCommit::Type) { StationTol = V; }))
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
					[
						SAssignNew(RebuildCheck, SCheckBox).IsChecked(ECheckBoxState::Checked)
						[ SNew(STextBlock).Text(LOCTEXT("Rebuild", "Rebuild bodies at this tolerance")) ]
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
					[ MakeInfoLabel(LOCTEXT("TolHint", "Loft station tolerance, in model units: stations are added until the surface is within this of the parametric definition. The export default is finer than the interactive preview. Unchecked, the geometry in the scene is written as-is; booleans and mirrors are always written as they stand.")) ])
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 10)
			[
				MakeGroup(LOCTEXT("Output", "Output"),
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
					[
						SAssignNew(NamedCheck, SCheckBox).IsChecked(ECheckBoxState::Checked)
						[ SNew(STextBlock).Text(LOCTEXT("Named", "Write named parts (STEP assembly)")) ]
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("BodiesN", "Bodies:"), Stat(LblBodies)) ])
			]
			+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[ SNew(SButton).Text(LOCTEXT("SaveStep", "Save STEP...")).OnClicked_Lambda([&Save]() { Save(); return FReply::Handled(); }) ]
				+ SHorizontalBox::Slot().AutoWidth()
				[ SNew(SButton).Text(LOCTEXT("Close", "Close")).OnClicked_Lambda([Window]() { Window->RequestDestroyWindow(); return FReply::Handled(); }) ]
			]
		]);

	Status(LOCTEXT("SelectBody", "Select at least one body to export."));
	FSlateApplication::Get().AddModalWindow(Window, FSlateApplication::Get().GetActiveTopLevelWindow());
}

#undef LOCTEXT_NAMESPACE
