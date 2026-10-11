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
	TopoDS_Shape Meshed;

	// The compound is rebuilt when the body selection changes and re-meshed
	// when a density changes; it carries whatever triangulation was last
	// computed, which is exactly what Save writes out.
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
			Meshed.Nullify();
			LblBodies->SetText(FText::FromString(TEXT("Ã¢â‚¬â€")));
			LblNodes->SetText(FText::FromString(TEXT("Ã¢â‚¬â€")));
			LblTris->SetText(FText::FromString(TEXT("Ã¢â‚¬â€")));
			Status(LOCTEXT("SelectBody", "Select at least one body to export."));
			return;
		}
		Status(LOCTEXT("Meshing", "Meshing..."));
		int32 Bodies = 0;
		int64 Nodes = 0, Tris = 0;
		TArray<FString> Failures;
		Meshed = AerotelicExport::MeshForStl(Operands, Labels, Deflection, AngleDeg, Bodies, Nodes, Tris, Failures);
		if (Meshed.IsNull())
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
		if (Meshed.IsNull())
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
	TArray<FAerotelicBodyChoicePtr> Choices = SceneBodyChoices();
	if (Choices.Num() == 0)
	{
		AerotelicMessageBox(EAppMsgType::Ok, LOCTEXT("NothingStep", "Nothing in the scene to export."), LOCTEXT("StepTitle", "Export STEP"));
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
