#include "PotentialFlowDialog.h"
#include "Widgets/MessageBox.h"
#include "BodyPicker.h"

#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Framework/Application/SlateApplication.h"
#include "Misc/Paths.h"
#include "DesktopPlatformModule.h"
#include "IDesktopPlatform.h"

#include "Widgets/CurveTabControls.h"
#include "Widgets/NumericEntry.h"
#include "Entity/AerotelicPotentialFlow.h"

// Last: OCCT's Handle() macro vs engine headers.
#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"

#define LOCTEXT_NAMESPACE "PotentialFlowDialog"

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
			[ SNew(SBox).WidthOverride(190) [ SNew(STextBlock).Text(Label) ] ]
			+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
			[ Field ];
	}

	TSharedRef<STextBlock> Stat(TSharedPtr<STextBlock>& Out)
	{
		return SAssignNew(Out, STextBlock).Text(FText::FromString(TEXT("-")))
			.ColorAndOpacity(FSlateColor(FLinearColor(0.85f, 0.85f, 0.85f)));
	}

	// The solution file the dialog starts with: beside the project, named
	// after it, so a user who only presses Solve still gets a file in a
	// sensible place.
	FString DefaultSolutionPath()
	{
		const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
		if (Doc != nullptr && Doc->HasCurrentPath())
		{
			const FString Path = Doc->GetCurrentPath();
			return FPaths::Combine(FPaths::GetPath(Path), FPaths::GetBaseFilename(Path) + TEXT("_flow.vtk"));
		}
		return FPaths::Combine(FPlatformProcess::UserDir(), TEXT("aerotelic_flow.vtk"));
	}
}

void ShowAerotelicPotentialFlowDialog(TFunction<void(const FText&)> Status)
{
	// The same rows the boolean and export dialogs offer: "what can I
	// operate on", "what can I export" and "what can I compute the flow
	// around" are one question.
	TArray<FAerotelicBodyChoicePtr> Choices = SceneBodyChoices();
	if (Choices.Num() == 0)
	{
		AerotelicMessageBox(EAppMsgType::Ok,
			LOCTEXT("Nothing", "Nothing in the scene to compute the flow around."),
			LOCTEXT("Title", "Potential Flow"));
		return;
	}
	if (AerotelicPotentialFlow::FindScript().IsEmpty())
	{
		AerotelicMessageBox(EAppMsgType::Ok,
			LOCTEXT("NoScript", "tools\\potential_flow.py is not installed beside the application."),
			LOCTEXT("Title", "Potential Flow"));
		return;
	}

	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(LOCTEXT("Title", "Potential Flow"))
		.ClientSize(FVector2D(620, 820))
		.SupportsMaximize(false)
		.SupportsMinimize(false);

	TSharedPtr<SAerotelicBodyPicker> Picker;
	TSharedPtr<SEditableTextBox> PathBox;
	TSharedPtr<STextBlock> LblMesh, LblQuality, LblSpeed, LblCp, LblForce, LblTime;
	TArray<TSharedPtr<FString>> Models = { MakeShared<FString>(TEXT("Incompressible")), MakeShared<FString>(TEXT("Compressible")) };
	TSharedPtr<FString> Model = Models[0];

	double Speed = 50.0, AngleDeg = 0.0, Density = 1.225, SpeedOfSound = 340.0;
	double Deflection = 2e-3, FarFactor = 5.0, NearSize = 0.0, FarSize = 0.0;

	auto RefreshCount = [&]()
	{
		const int32 N = Picker->GetChosen().Num();
		Status(N ? FText::FromString(FString::Printf(TEXT("%d selected for the flow analysis"), N))
			: LOCTEXT("SelectBody", "Select the geometry the flow is to be computed around."));
	};

	auto Browse = [&]()
	{
		IDesktopPlatform* Desktop = FDesktopPlatformModule::Get();
		if (Desktop == nullptr)
		{
			return;
		}
		TArray<FString> Files;
		const FString Current = PathBox->GetText().ToString();
		const void* Handle = FSlateApplication::Get().FindBestParentWindowHandleForDialogs(Window);
		if (!Desktop->SaveFileDialog(Handle, TEXT("Potential Flow Solution"), FPaths::GetPath(Current),
			FPaths::GetCleanFilename(Current), TEXT("VTK Files (*.vtk)|*.vtk"), EFileDialogFlags::None, Files) || Files.Num() == 0)
		{
			return;
		}
		FString Path = FPaths::ConvertRelativePathToFull(Files[0]);
		if (FPaths::GetExtension(Path).IsEmpty())
		{
			Path += TEXT(".vtk");
		}
		PathBox->SetText(FText::FromString(Path));
	};

	auto Solve = [&]()
	{
		const TArray<FAerotelicBodyChoicePtr>& Chosen = Picker->GetChosen();
		if (Chosen.Num() == 0)
		{
			AerotelicMessageBox(EAppMsgType::Ok, LOCTEXT("SelectBodyWarn", "Select the geometry the flow is to be computed around."), LOCTEXT("Title", "Potential Flow"));
			return;
		}
		const FString Path = PathBox->GetText().ToString().TrimStartAndEnd();
		if (Path.IsEmpty())
		{
			AerotelicMessageBox(EAppMsgType::Ok, LOCTEXT("NoPath", "Name the file the solution is to be written to."), LOCTEXT("Title", "Potential Flow"));
			return;
		}

		// The bodies, triangulated as they sit in the scene. The meshes
		// have to outlive the call, so they are held here and pointed at.
		Status(LOCTEXT("Building", "Building the bodies..."));
		std::vector<aerotelic::NamedMesh> Held;
		TArray<FString> Labels;
		TArray<FString> Failures;
		for (const FAerotelicBodyChoicePtr& C : Chosen)
		{
			if (!C->Operand.Entity.IsValid())
			{
				Failures.Add(C->Label + TEXT(": the entity is gone"));
				continue;
			}
			std::vector<aerotelic::NamedMesh> Meshes;
			try
			{
				Meshes = C->Operand.Entity->GetPlacedMeshes(C->Operand.bMirror, Deflection);
			}
			catch (const std::exception& E)
			{
				Failures.Add(C->Label + TEXT(": ") + UTF8_TO_TCHAR(E.what()));
				continue;
			}
			bool bAny = false;
			for (aerotelic::NamedMesh& M : Meshes)
			{
				if (M.mesh.IsEmpty()) continue;
				Labels.Add(Meshes.size() > 1 ? C->Label + TEXT("/") + UTF8_TO_TCHAR(M.name.c_str()) : C->Label);
				Held.push_back(std::move(M));
				bAny = true;
			}
			if (!bAny)
			{
				Failures.Add(C->Label + TEXT(": produced no geometry"));
			}
		}
		if (Held.empty())
		{
			Status(LOCTEXT("Ready", "Ready"));
			AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(TEXT("No body could be built:\n") + FString::Join(Failures, TEXT("\n"))), LOCTEXT("Title", "Potential Flow"));
			return;
		}

		AerotelicPotentialFlow::FRequest Request;
		for (int32 i = 0; i < static_cast<int32>(Held.size()); ++i)
		{
			Request.Bodies.Add({ Labels[i], &Held[i].mesh });
		}
		Request.Speed = Speed;
		Request.AngleOfAttackDeg = AngleDeg;
		Request.Density = Density;
		Request.SpeedOfSound = SpeedOfSound;
		Request.ElementType = *Model == TEXT("Compressible") ? TEXT("compressible") : TEXT("incompressible");
		Request.FarFieldFactor = FarFactor;
		Request.NearElementSize = NearSize;
		Request.FarElementSize = FarSize;
		Request.VtkPath = Path;

		Status(FText::FromString(FString::Printf(TEXT("Meshing the air around %d bodies and solving..."), Request.Bodies.Num())));
		AerotelicPotentialFlow::FResult R;
		FString Error;
		if (!AerotelicPotentialFlow::Run(Request, R, Error))
		{
			Status(LOCTEXT("Ready", "Ready"));
			AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(Error), LOCTEXT("Failed", "Potential Flow Failed"));
			return;
		}

		LblMesh->SetText(FText::FromString(FString::Printf(TEXT("%s nodes, %s tetrahedra; %s wetted triangles"),
			*Thousands(R.Nodes), *Thousands(R.Tets), *Thousands(R.WettedTriangles))));
		LblQuality->SetText(FText::FromString(FString::Printf(TEXT("worst %.3f, mean %.3f (%s under 0.1); element size %.4g at the bodies, %.4g far out"),
			R.MinQuality, R.MeanQuality, *Thousands(R.PoorQuality), R.NearElementSize, R.FarElementSize)));
		LblSpeed->SetText(FText::FromString(FString::Printf(TEXT("up to %.4g (free stream %.4g)"), R.MaxSpeed, R.FreeStreamSpeed)));
		LblCp->SetText(FText::FromString(FString::Printf(TEXT("%.4g to %.4g on the bodies (%.4g to %.4g over the field)"),
			R.MinCpBody, R.MaxCpBody, R.MinCp, R.MaxCp)));
		LblForce->SetText(FText::FromString(FString::Printf(TEXT("(%.4g, %.4g, %.4g) = %.2e of q x frontal area %.4g"),
			R.Force[0], R.Force[1], R.Force[2], R.ForceCoefficient, R.FrontalArea)));
		LblTime->SetText(FText::FromString(FString::Printf(TEXT("%.1f s (%.1f meshing, %.1f solving)"), R.Seconds, R.SecondsMesh, R.SecondsSolve)));

		FString Line = FString::Printf(TEXT("Potential flow solved on %s tetrahedra: Cp %.3g to %.3g on the bodies, net force %.1e of q x frontal area (zero in an exact potential flow) -> %s"),
			*Thousands(R.Tets), R.MinCpBody, R.MaxCpBody, R.ForceCoefficient, *R.VtkPath);
		Status(FText::FromString(Line));
		if (Failures.Num() > 0)
		{
			AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(TEXT("Left out of the analysis:\n") + FString::Join(Failures, TEXT("\n"))), LOCTEXT("Title", "Potential Flow"));
		}
	};

	auto Spin = [&](double& Var, double Min, double Max, double Delta, const FText& Suffix)
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

	Window->SetContent(
		SNew(SBorder).Padding(12)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
			[
				MakeGroup(LOCTEXT("Geometry", "Geometry for the analysis"),
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight()
					[
						SAssignNew(Picker, SAerotelicBodyPicker).Choices(Choices).ListHeight(170.0f)
						.OnChanged(FSimpleDelegate::CreateLambda([&RefreshCount]() { RefreshCount(); }))
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
					[
						MakeInfoLabel(LOCTEXT("GeometryHint",
							"The air around these bodies is what gets meshed, so they must not run into one another: fuse intersecting parts with a boolean operation first and select the result. The wetted surface of the mesh is each body's own triangulation, which is what sets how finely the pressures on it are resolved."))
					])
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
			[
				MakeGroup(LOCTEXT("FreeStream", "Free stream"),
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Speed", "Speed:"), Spin(Speed, 1e-9, 1e9, 5.0, LOCTEXT("PerSecond", " units/s"))) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Aoa", "Angle of attack:"), Spin(AngleDeg, -90.0, 90.0, 0.5, FText::FromString(TEXT(" deg")))) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Density", "Air density:"), Spin(Density, 1e-9, 1e9, 0.1, FText::GetEmpty())) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Sound", "Speed of sound:"), Spin(SpeedOfSound, 1e-9, 1e9, 10.0, LOCTEXT("PerSecond2", " units/s"))) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
					[
						FormRow(LOCTEXT("ModelRow", "Flow model:"),
							SNew(SComboBox<TSharedPtr<FString>>)
							.OptionsSource(&Models)
							.InitiallySelectedItem(Model)
							.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
							.OnSelectionChanged_Lambda([&Model](TSharedPtr<FString> S, ESelectInfo::Type) { if (S.IsValid()) Model = S; })
							[ SNew(STextBlock).Text_Lambda([&Model]() { return FText::FromString(Model.IsValid() ? *Model : FString()); }) ])
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
					[
						MakeInfoLabel(LOCTEXT("FlowHint",
							"The flow runs along +x, tilted towards +z by the angle of attack. There is no wake, so there is no circulation and no lift: the pressure distribution is real, the lift is zero by construction. Density and the speed of sound only scale the pressures into forces and set the Mach number for the compressible model."))
					])
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
			[
				MakeGroup(LOCTEXT("MeshGroup", "Mesh"),
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Deflection", "Surface deflection:"), Spin(Deflection, 1e-9, 1e9, 0.001, FText::GetEmpty())) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("FarFactor", "Far field, in model sizes:"), Spin(FarFactor, 0.6, 1000.0, 1.0, FText::GetEmpty())) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("NearSize", "Element size at the bodies:"), Spin(NearSize, 0.0, 1e9, 0.01, FText::GetEmpty())) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("FarSize", "Element size far out:"), Spin(FarSize, 0.0, 1e9, 0.1, FText::GetEmpty())) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
					[
						MakeInfoLabel(LOCTEXT("MeshHint",
							"Zero takes the element size at the bodies from their own triangles and the size far out from the size of the box. The far field is a box of that many bounding-box diagonals to each side; smaller is quicker but crowds the flow."))
					])
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
			[
				MakeGroup(LOCTEXT("Result", "Result"),
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("MeshStat", "Grid:"), Stat(LblMesh)) ]
					+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("QualityStat", "Element quality:"), Stat(LblQuality)) ]
					+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("SpeedStat", "Speed:"), Stat(LblSpeed)) ]
					+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("CpStat", "Pressure coefficient:"), Stat(LblCp)) ]
					+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("ForceStat", "Net force:"), Stat(LblForce)) ]
					+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("TimeStat", "Took:"), Stat(LblTime)) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
					[
						MakeInfoLabel(LOCTEXT("ResultHint",
							"A potential flow carries no force at all (d'Alembert's paradox), so the net force is a measure of the discretisation, not a load: it should be a small fraction of the dynamic pressure on the frontal area. The pressure coefficient reaches 1 wherever the flow comes to rest."))
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
		]);

	Status(LOCTEXT("SelectBody", "Select the geometry the flow is to be computed around."));
	FSlateApplication::Get().AddModalWindow(Window, FSlateApplication::Get().GetActiveTopLevelWindow());
}

#undef LOCTEXT_NAMESPACE
