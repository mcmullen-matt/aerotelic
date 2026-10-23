#include "StructuralFeaDialog.h"
#include "Widgets/MessageBox.h"

#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Framework/Application/SlateApplication.h"
#include "Misc/Paths.h"
#include "DesktopPlatformModule.h"
#include "IDesktopPlatform.h"

#include "Widgets/CurveTabControls.h"
#include "Widgets/NumericEntry.h"
#include "Entity/AerotelicStructuralFea.h"

#include "aerotelic/materials.hpp"
#include "aerotelic/units.hpp"

// Last: OCCT's Handle() macro vs engine headers.
#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"
#include "Entity/AerotelicLatticeModel.h"

#define LOCTEXT_NAMESPACE "StructuralFeaDialog"

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
			[ SNew(SBox).WidthOverride(200) [ SNew(STextBlock).Text(Label) ] ]
			+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
			[ Field ];
	}

	TSharedRef<STextBlock> Stat(TSharedPtr<STextBlock>& Out)
	{
		return SAssignNew(Out, STextBlock).Text(FText::FromString(TEXT("-")))
			.AutoWrapText(true)
			.ColorAndOpacity(FSlateColor(FLinearColor(0.85f, 0.85f, 0.85f)));
	}

	FString DefaultSolutionPath()
	{
		const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
		if (Doc != nullptr && Doc->HasCurrentPath())
		{
			const FString Path = Doc->GetCurrentPath();
			return FPaths::Combine(FPaths::GetPath(Path), FPaths::GetBaseFilename(Path) + TEXT("_fea.vtk"));
		}
		return FPaths::Combine(FPlatformProcess::UserDir(), TEXT("aerotelic_fea.vtk"));
	}

	FString UnitName(const std::string& S) { return FString(UTF8_TO_TCHAR(S.c_str())); }

	// The entity whose placed body a lattice's tetrahedral mesh belongs
	// to, by the label the model gave it: "<entity>", "<entity> (mirror)",
	// either with " / <body>" after it for a multi-body entity.
	AAerotelicEntityActor* EntityForTetLabel(const FAerotelicLatticeModel* Model, const FString& Label)
	{
		for (const FAerotelicBooleanOperand& Op : Model->Targets)
		{
			if (!Op.Entity.IsValid()) continue;
			const FString EntityLabel = Op.Entity->EntityName + (Op.bMirror ? TEXT(" (mirror)") : TEXT(""));
			if (Label == EntityLabel || Label.StartsWith(EntityLabel + TEXT(" / "))) return Op.Entity.Get();
		}
		return nullptr;
	}
}

void ShowAerotelicStructuralFeaDialog(TFunction<void(const FText&)> Status)
{
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	TArray<TWeakObjectPtr<AAerotelicEntityActor>> Lattices;
	if (Doc != nullptr)
	{
		for (auto& Weak : Doc->GetEntities())
		{
			if (Weak.IsValid() && Weak->GetEntityType() == aerotelic::project_yaml::EntityType::Lattice) Lattices.Add(Weak);
		}
	}
	if (Lattices.Num() == 0)
	{
		AerotelicMessageBox(EAppMsgType::Ok,
			LOCTEXT("NoLattice", "Nothing to analyse: the model is a lattice-filled body's skin and core tetrahedra, so generate a lattice first (Lattice > Generate Lattice...)."),
			LOCTEXT("Title", "Structural (Kratos FEA)"));
		return;
	}
	if (AerotelicStructuralFea::FindScript().IsEmpty())
	{
		AerotelicMessageBox(EAppMsgType::Ok,
			LOCTEXT("NoScript", "tools\\structural_fea.py is not installed beside the application."),
			LOCTEXT("Title", "Structural (Kratos FEA)"));
		return;
	}
	const aerotelic::UnitSystem Units = Doc != nullptr ? Doc->GetUnits() : aerotelic::UnitSystem{};
	const FText LengthSuffix = FText::FromString(TEXT(" ") + UnitName(Units.LengthName()));
	const FText StressSuffix = FText::FromString(TEXT(" ") + UnitName(Units.StressName()));

	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(LOCTEXT("Title", "Structural (Kratos FEA)"))
		.ClientSize(FVector2D(640, 860))
		.SupportsMaximize(false)
		.SupportsMinimize(false);

	// Modal: every local below outlives the widgets that point at it.
	TSharedPtr<SEditableTextBox> PathBox;
	TSharedPtr<SCheckBox> SecondOrderCheck, PointCheck;
	TSharedPtr<STextBlock> LblMaterials, LblMesh, LblDisp, LblStress, LblMargins, LblBalance, LblTime;

	TArray<TSharedPtr<FString>> LatticeOptions;
	for (const auto& L : Lattices) LatticeOptions.Add(MakeShared<FString>(L->EntityName));
	int32 LatticeIndex = 0;
	TArray<TSharedPtr<FString>> AxisOptions = { MakeShared<FString>(TEXT("x")), MakeShared<FString>(TEXT("y")), MakeShared<FString>(TEXT("z")) };
	int32 AxisIndex = 1;
	TArray<TSharedPtr<FString>> PlaneOptions = { MakeShared<FString>(TEXT("at the value below")), MakeShared<FString>(TEXT("at the mesh's minimum")), MakeShared<FString>(TEXT("at the mesh's maximum")) };
	int32 PlaneIndex = 0;

	double SupportValue = 0.0, SupportTolerance = 0.0;
	double LoadFactor = 1.0, Pressure = 0.0;
	double PointX = 0.0, PointY = 0.0, PointZ = 0.0, ForceX = 0.0, ForceY = 0.0, ForceZ = -1000.0;

	auto Combo = [](TArray<TSharedPtr<FString>>& Options, int32& Index)
	{
		return SNew(SComboBox<TSharedPtr<FString>>)
			.OptionsSource(&Options)
			.InitiallySelectedItem(Options[Index])
			.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
			.OnSelectionChanged_Lambda([&Options, &Index](TSharedPtr<FString> S, ESelectInfo::Type)
			{
				const int32 I = Options.IndexOfByKey(S);
				if (I != INDEX_NONE) Index = I;
			})
			[ SNew(STextBlock).Text_Lambda([&Options, &Index]() { return FText::FromString(*Options[Index]); }) ];
	};
	auto Spin = [](double& Var, double Min, double Max, double Delta, const FText& Suffix)
	{
		return SNew(SAerotelicNumericEntry)
			.Value_Lambda([&Var]() { return Var; })
			.MinValue(Min).MaxValue(Max).Delta(Delta).Suffix(Suffix)
			.LiveUpdate(false)
			.OnValueCommitted_Lambda([&Var](double V, ETextCommit::Type) { Var = V; });
	};
	auto Button = [](const FText& Label, TFunction<void()> Fn)
	{
		return SNew(SButton).Text(Label).HAlign(HAlign_Center)
			.OnClicked_Lambda([Fn]() { Fn(); return FReply::Handled(); });
	};

	// What the chosen lattice's model is made of, before anything is
	// meshed: one line per region, naming the material or the gap.
	auto DescribeModel = [&]()
	{
		if (!Lattices[LatticeIndex].IsValid()) { LblMaterials->SetText(LOCTEXT("Gone", "(the lattice is gone)")); return; }
		AAerotelicEntityActor* L = Lattices[LatticeIndex].Get();
		const FAerotelicLatticeModel* Model = L->GetModelAs<FAerotelicLatticeModel>();
		TArray<FString> Lines;
		for (const FAerotelicBooleanOperand& Op : Model->Targets)
		{
			if (!Op.Entity.IsValid()) continue;
			const aerotelic::MassSpec& S = Op.Entity->GetMassSpec();
			const bool bOk = S.mode == aerotelic::MassMode::Material && S.material.IsStructural();
			Lines.Add(FString::Printf(TEXT("%s%s skin: %s"), *Op.Entity->EntityName, Op.bMirror ? TEXT(" (mirror)") : TEXT(""),
				bOk ? UTF8_TO_TCHAR(S.material.name.c_str()) : TEXT("NO STRUCTURAL MATERIAL -- set one under Material Properties")));
		}
		const aerotelic::MassSpec& C = L->GetMassSpec();
		const bool bCoreOk = C.mode == aerotelic::MassMode::Material && C.material.IsStructural();
		Lines.Add(FString::Printf(TEXT("core (the lattice's effective material): %s"),
			bCoreOk ? UTF8_TO_TCHAR(C.material.name.c_str()) : TEXT("NO STRUCTURAL MATERIAL -- set one on the lattice under Material Properties")));
		LblMaterials->SetText(FText::FromString(FString::Join(Lines, TEXT("\n"))));
	};

	auto Solve = [&]()
	{
		if (!Lattices[LatticeIndex].IsValid())
		{
			AerotelicMessageBox(EAppMsgType::Ok, LOCTEXT("GoneWarn", "The lattice is no longer in the scene."), LOCTEXT("Title", "Structural (Kratos FEA)"));
			return;
		}
		const FString Path = PathBox->GetText().ToString().TrimStartAndEnd();
		if (Path.IsEmpty())
		{
			AerotelicMessageBox(EAppMsgType::Ok, LOCTEXT("NoPath", "Name the file the solution is to be written to."), LOCTEXT("Title", "Structural (Kratos FEA)"));
			return;
		}
		AAerotelicEntityActor* L = Lattices[LatticeIndex].Get();
		const FAerotelicLatticeModel* Model = L->GetModelAs<FAerotelicLatticeModel>();

		// The model: every filled body's skin and core tetrahedra as one
		// mesh, each region its own material id (body i: skin 2i+1, core
		// 2i+2). The bodies are not connected to one another -- a lattice
		// over separate bodies is separate models solved at once.
		Status(LOCTEXT("Meshing", "Building the skin and core tetrahedra..."));
		const std::vector<FAerotelicLatticeModel::FTetMesh>& Meshes = Model->GetTetMeshes();
		aerotelic::TetMesh Mesh;
		AerotelicStructuralFea::FRequest Request;
		TArray<FString> Problems;
		int32 BodyIndex = 0;
		for (const FAerotelicLatticeModel::FTetMesh& M : Meshes)
		{
			AAerotelicEntityActor* Body = EntityForTetLabel(Model, M.Label);
			if (Body == nullptr) { Problems.Add(M.Label + TEXT(": its entity is gone")); continue; }
			const aerotelic::MassSpec& S = Body->GetMassSpec();
			if (S.mode != aerotelic::MassMode::Material || !S.material.IsStructural())
			{
				Problems.Add(M.Label + TEXT(": no structural material (Material Properties in the Scene tree)"));
				continue;
			}
			const int Offset = static_cast<int>(Mesh.nodes.size());
			const int SkinId = 2 * BodyIndex + 1, CoreId = 2 * BodyIndex + 2;
			Mesh.nodes.insert(Mesh.nodes.end(), M.Result.Mesh.nodes.begin(), M.Result.Mesh.nodes.end());
			for (size_t t = 0; t < M.Result.Mesh.tets.size(); ++t)
			{
				const auto& T = M.Result.Mesh.tets[t];
				Mesh.tets.push_back({T[0] + Offset, T[1] + Offset, T[2] + Offset, T[3] + Offset});
				const int Region = t < M.Result.Mesh.material.size() ? M.Result.Mesh.material[t] : 1;
				Mesh.material.push_back(Region == 2 ? CoreId : SkinId);
			}
			Request.Materials.Add({ SkinId, M.Label + TEXT(" skin"), S.material });
			const aerotelic::MassSpec& C = L->GetMassSpec();
			Request.Materials.Add({ CoreId, M.Label + TEXT(" core (") + L->EntityName + TEXT(")"), C.material });
			++BodyIndex;
		}
		for (const FString& N : Model->GetTetNotes()) Problems.Add(N);
		if (Mesh.tets.empty())
		{
			Status(LOCTEXT("Ready", "Ready"));
			AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(TEXT("No model could be built:\n") + FString::Join(Problems, TEXT("\n"))), LOCTEXT("Title", "Structural (Kratos FEA)"));
			return;
		}
		{
			const aerotelic::MassSpec& C = L->GetMassSpec();
			if (C.mode != aerotelic::MassMode::Material || !C.material.IsStructural())
			{
				Status(LOCTEXT("Ready", "Ready"));
				AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(FString::Printf(TEXT("%s has no structural material for its core: set one on the lattice under Material Properties in the Scene tree (the lattice's EFFECTIVE density and stiffness)."), *L->EntityName)), LOCTEXT("Title", "Structural (Kratos FEA)"));
				return;
			}
		}

		Request.Mesh = &Mesh;
		Request.LengthToSI = Units.LengthToSI();
		Request.bSecondOrder = SecondOrderCheck->IsChecked();
		Request.SupportAxis = (*AxisOptions[AxisIndex])[0];
		Request.SupportValue = SupportValue;
		Request.bSupportAtMin = PlaneIndex == 1;
		Request.bSupportAtMax = PlaneIndex == 2;
		Request.SupportTolerance = SupportTolerance;
		Request.LoadFactor = LoadFactor;
		Request.Pressure = Pressure * Units.StressToSI();
		Request.bPointForce = PointCheck->IsChecked();
		Request.PointAt = aerotelic::Vec3{PointX, PointY, PointZ};
		Request.PointForce = aerotelic::Vec3{ForceX, ForceY, ForceZ};
		Request.VtkPath = Path;

		Status(FText::FromString(FString::Printf(TEXT("Solving %s tetrahedra on Kratos..."), *Thousands((int64)Mesh.tets.size()))));
		AerotelicStructuralFea::FResult R;
		FString Error;
		if (!AerotelicStructuralFea::Run(Request, R, Error))
		{
			Status(LOCTEXT("Ready", "Ready"));
			AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(Error), LOCTEXT("Failed", "Structural Analysis Failed"));
			return;
		}

		const double L2 = Units.LengthToSI(), S2 = Units.StressToSI(), M2 = Units.MassToSI();
		LblMesh->SetText(FText::FromString(FString::Printf(TEXT("%s nodes, %s %s tetrahedra, %s DOF; %s wetted faces; %s nodes on the support"),
			*Thousands(R.Nodes), *Thousands(R.Elements), R.bSecondOrder ? TEXT("10-node") : TEXT("4-node"), *Thousands(R.Dofs), *Thousands(R.WettedFaces), *Thousands(R.SupportNodes))));
		LblDisp->SetText(FText::FromString(FString::Printf(TEXT("%.4g %s at (%.4g, %.4g, %.4g); mean surface displacement %s (u . n = %.3g %s)"),
			R.MaxDisplacement / L2, *UnitName(Units.LengthName()), R.MaxDisplacementAt[0] / L2, R.MaxDisplacementAt[1] / L2, R.MaxDisplacementAt[2] / L2,
			R.MeanNormalDisplacement < 0.0 ? TEXT("inward") : TEXT("outward"), R.MeanNormalDisplacement / L2, *UnitName(Units.LengthName()))));
		FString Peaks;
		for (int32 i = 0; i < Request.Materials.Num() && i < R.MaxVonMisesByMaterial.Num(); ++i)
		{
			const bool bHasAllowable = Request.Materials[i].Material.allowable_stress > 0.0;
			Peaks += FString::Printf(TEXT("%s%s: %.4g %s%s"), i ? TEXT("\n") : TEXT(""), *Request.Materials[i].Label, R.MaxVonMisesByMaterial[i] / S2, *UnitName(Units.StressName()),
				bHasAllowable ? *FString::Printf(TEXT(", margin %+.2f against %.4g"), R.MarginByMaterial[i], Request.Materials[i].Material.allowable_stress / S2) : TEXT(" (no allowable set)"));
		}
		LblStress->SetText(FText::FromString(FString::Printf(TEXT("%.4g %s, in material %d"), R.MaxVonMises / S2, *UnitName(Units.StressName()), R.MaxVonMisesMaterial)));
		LblMargins->SetText(FText::FromString(Peaks));
		LblBalance->SetText(FText::FromString(FString::Printf(
			TEXT("mass %.6g %s. Reactions (%.4g, %.4g, %.4g) N against gravity (%.4g, %.4g, %.4g) + pressure (%.4g, %.4g, %.4g) + point (%.4g, %.4g, %.4g) N"),
			R.Mass / M2, *UnitName(Units.MassName()), R.Reaction[0], R.Reaction[1], R.Reaction[2],
			R.AppliedGravity[0], R.AppliedGravity[1], R.AppliedGravity[2], R.AppliedPressure[0], R.AppliedPressure[1], R.AppliedPressure[2],
			R.AppliedPoint[0], R.AppliedPoint[1], R.AppliedPoint[2])));
		LblTime->SetText(FText::FromString(FString::Printf(TEXT("%.1f s (%.1f solving)"), R.Seconds, R.SecondsSolve)));
		Status(FText::FromString(FString::Printf(TEXT("Structural analysis solved on %s DOF: max displacement %.4g %s, max von Mises %.4g %s -> %s"),
			*Thousands(R.Dofs), R.MaxDisplacement / L2, *UnitName(Units.LengthName()), R.MaxVonMises / S2, *UnitName(Units.StressName()), *R.VtkPath)));
		if (Problems.Num() > 0)
		{
			AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(TEXT("Left out of the model:\n") + FString::Join(Problems, TEXT("\n"))), LOCTEXT("Title", "Structural (Kratos FEA)"));
		}
	};

	auto Browse = [&]()
	{
		IDesktopPlatform* Desktop = FDesktopPlatformModule::Get();
		if (Desktop == nullptr) return;
		TArray<FString> Files;
		const FString Current = PathBox->GetText().ToString();
		const void* Handle = FSlateApplication::Get().FindBestParentWindowHandleForDialogs(Window);
		if (Desktop->SaveFileDialog(Handle, TEXT("Structural Solution"), FPaths::GetPath(Current), FPaths::GetCleanFilename(Current),
			TEXT("VTK Files (*.vtk)|*.vtk"), EFileDialogFlags::None, Files) && Files.Num() > 0)
		{
			PathBox->SetText(FText::FromString(FPaths::ConvertRelativePathToFull(Files[0])));
		}
	};

	Window->SetContent(
		SNew(SBorder).Padding(12)
		[
			SNew(SScrollBox)
			+ SScrollBox::Slot()
			[
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
				[
					MakeGroup(LOCTEXT("ModelGroup", "Model"),
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
						[
							FormRow(LOCTEXT("Lattice", "Lattice-filled body:"),
								SNew(SComboBox<TSharedPtr<FString>>)
								.OptionsSource(&LatticeOptions)
								.InitiallySelectedItem(LatticeOptions[0])
								.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
								.OnSelectionChanged_Lambda([&LatticeOptions, &LatticeIndex, &DescribeModel](TSharedPtr<FString> S, ESelectInfo::Type)
								{
									const int32 I = LatticeOptions.IndexOfByKey(S);
									if (I != INDEX_NONE) { LatticeIndex = I; DescribeModel(); }
								})
								[ SNew(STextBlock).Text_Lambda([&LatticeOptions, &LatticeIndex]() { return FText::FromString(*LatticeOptions[LatticeIndex]); }) ])
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 4) [ Stat(LblMaterials) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
						[
							SAssignNew(SecondOrderCheck, SCheckBox).IsChecked(ECheckBoxState::Checked)
							[ SNew(STextBlock).Text(LOCTEXT("SecondOrder", "Second-order (10-node) tetrahedra")) ]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
						[
							MakeInfoLabel(LOCTEXT("ModelHint",
								"The model is the lattice's skin and core tetrahedra (what Lattice > Export Tetrahedral Mesh writes): the skin made of each filled body's material, the core of the lattice's own -- its EFFECTIVE density and stiffness, set on the lattice under Material Properties. Keep second order on: a skin one tetrahedron thick is a bending problem, and first-order tetrahedra lock."))
						])
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
				[
					MakeGroup(LOCTEXT("SupportGroup", "Support"),
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Axis", "Fix every node on the plane:"), Combo(AxisOptions, AxisIndex)) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Plane", "Where:"), Combo(PlaneOptions, PlaneIndex)) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Value", "Value:"), Spin(SupportValue, -1e9, 1e9, 0.1, LengthSuffix)) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Tolerance", "Within (0 = a millionth of the size):"), Spin(SupportTolerance, 0.0, 1e9, 0.001, LengthSuffix)) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
						[
							MakeInfoLabel(LOCTEXT("SupportHint",
								"Every degree of freedom of the nodes on that plane is fixed. y = 0 is the root of a mirrored half -- the symmetry plane -- which is why it is the default; the mesh's minimum or maximum along an axis clamps a body by its end."))
						])
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
				[
					MakeGroup(LOCTEXT("LoadsGroup", "Loads"),
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("LoadFactor", "Inertial load factor:"), Spin(LoadFactor, -100.0, 100.0, 0.5, FText::FromString(TEXT(" g")))) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Pressure", "Pressure on the surface:"), Spin(Pressure, -1e12, 1e12, 100.0, StressSuffix)) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 2)
						[
							SAssignNew(PointCheck, SCheckBox).IsChecked(ECheckBoxState::Unchecked)
							[ SNew(STextBlock).Text(LOCTEXT("PointForce", "A point force at the nearest node")) ]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
						[
							FormRow(LOCTEXT("PointAt", "At:"),
								SNew(SHorizontalBox)
								+ SHorizontalBox::Slot().FillWidth(1).Padding(0, 0, 4, 0) [ Spin(PointX, -1e9, 1e9, 0.1, LengthSuffix) ]
								+ SHorizontalBox::Slot().FillWidth(1).Padding(0, 0, 4, 0) [ Spin(PointY, -1e9, 1e9, 0.1, LengthSuffix) ]
								+ SHorizontalBox::Slot().FillWidth(1) [ Spin(PointZ, -1e9, 1e9, 0.1, LengthSuffix) ])
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
						[
							FormRow(LOCTEXT("PointF", "Force (N):"),
								SNew(SHorizontalBox)
								+ SHorizontalBox::Slot().FillWidth(1).Padding(0, 0, 4, 0) [ Spin(ForceX, -1e12, 1e12, 100.0, FText::FromString(TEXT(" N"))) ]
								+ SHorizontalBox::Slot().FillWidth(1).Padding(0, 0, 4, 0) [ Spin(ForceY, -1e12, 1e12, 100.0, FText::FromString(TEXT(" N"))) ]
								+ SHorizontalBox::Slot().FillWidth(1) [ Spin(ForceZ, -1e12, 1e12, 100.0, FText::FromString(TEXT(" N"))) ])
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
						[
							MakeInfoLabel(LOCTEXT("LoadsHint",
								"Dummy loads, for now. The load factor puts the body's own mass under that many g along -z (1 is its weight; a 4 g pull-up is a real case). The pressure acts on every wetted face, positive pressing inwards -- the place a pressure field from the potential flow or DUST will plug in. Forces are newtons whatever the project's units, since a force in pounds-mass is a poundal."))
						])
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
				[
					MakeGroup(LOCTEXT("Result", "Result"),
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("MeshStat", "Mesh:"), Stat(LblMesh)) ]
						+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("DispStat", "Max displacement:"), Stat(LblDisp)) ]
						+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("StressStat", "Max von Mises:"), Stat(LblStress)) ]
						+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("MarginStat", "Per material:"), Stat(LblMargins)) ]
						+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("BalanceStat", "Force balance:"), Stat(LblBalance)) ]
						+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("TimeStat", "Took:"), Stat(LblTime)) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
						[
							MakeInfoLabel(LOCTEXT("ResultHint",
								"Linear static, small displacement, linear elastic. The reactions at the support must balance what was applied -- gravity on the mass, the pressure's resultant (zero over a closed surface), the point force. Margin is allowable / peak von Mises - 1 per material. The VTK holds the displacement and the von Mises stress for ParaView, in SI."))
						])
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 10)
				[
					MakeGroup(LOCTEXT("Output", "Solution file"),
						SNew(SHorizontalBox)
						+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
						[ SAssignNew(PathBox, SEditableTextBox).Text(FText::FromString(DefaultSolutionPath())) ]
						+ SHorizontalBox::Slot().AutoWidth().Padding(6, 0, 0, 0)
						[ Button(LOCTEXT("Browse", "Browse..."), [&Browse]() { Browse(); }) ])
				]
				+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
					[ Button(LOCTEXT("SolveBtn", "Solve"), [&Solve]() { Solve(); }) ]
					+ SHorizontalBox::Slot().AutoWidth()
					[ Button(LOCTEXT("Close", "Close"), [Window]() { Window->RequestDestroyWindow(); }) ]
				]
			]
		]);

	DescribeModel();
	Status(LOCTEXT("Ready", "Choose the lattice, the support and the loads, then Solve."));
	FSlateApplication::Get().AddModalWindow(Window, FSlateApplication::Get().GetActiveTopLevelWindow());
}

#undef LOCTEXT_NAMESPACE
