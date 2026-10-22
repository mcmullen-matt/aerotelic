#include "MassPropertiesDialog.h"
#include "Widgets/MessageBox.h"
#include "BodyPicker.h"

#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Framework/Application/SlateApplication.h"
#include "HAL/PlatformApplicationMisc.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Styling/CoreStyle.h"
#include "DesktopPlatformModule.h"
#include "IDesktopPlatform.h"

#include "Widgets/CurveTabControls.h"
#include "Widgets/NumericEntry.h"
#include "Entity/AerotelicMassProperties.h"

// Last: OCCT's Handle() macro vs engine headers.
#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"

#define LOCTEXT_NAMESPACE "MassPropertiesDialog"

namespace
{
	TSharedRef<SWidget> FormRow(const FText& Label, TSharedRef<SWidget> Field)
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0)
			[ SNew(SBox).WidthOverride(150) [ SNew(STextBlock).Text(Label) ] ]
			+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
			[ Field ];
	}

	// Beside the project, named after it, like the flow solution.
	FString DefaultReportPath()
	{
		const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
		if (Doc != nullptr && Doc->HasCurrentPath())
		{
			const FString Path = Doc->GetCurrentPath();
			return FPaths::Combine(FPaths::GetPath(Path), FPaths::GetBaseFilename(Path) + TEXT("_mass.txt"));
		}
		return FPaths::Combine(FPlatformProcess::UserDir(), TEXT("aerotelic_mass.txt"));
	}
}

void ShowAerotelicMassPropertiesDialog(TFunction<void(const FText&)> Status)
{
	TArray<FAerotelicBodyChoicePtr> Choices = SceneBodyChoices();
	if (Choices.Num() == 0)
	{
		AerotelicMessageBox(EAppMsgType::Ok,
			LOCTEXT("Nothing", "Nothing in the scene to weigh."),
			LOCTEXT("Title", "Mass Properties"));
		return;
	}

	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(LOCTEXT("Title", "Mass Properties"))
		.ClientSize(FVector2D(1060, 820))
		.SupportsMaximize(true)
		.SupportsMinimize(false);

	TSharedPtr<SAerotelicBodyPicker> Picker;
	TSharedPtr<STextBlock> ReportText;
	double Tolerance = FAerotelicBooleanModel::kValidationTolerance;
	FString ReportString;
	const UAerotelicDocumentSubsystem* DocForUnits = UAerotelicDocumentSubsystem::Get();
	const aerotelic::UnitSystem Units = DocForUnits != nullptr ? DocForUnits->GetUnits() : aerotelic::UnitSystem{};

	auto RefreshCount = [&]()
	{
		const int32 N = Picker->GetChosen().Num();
		Status(N ? FText::FromString(FString::Printf(TEXT("%d selected to weigh"), N))
			: LOCTEXT("SelectBody", "Select the bodies to weigh."));
	};

	auto Compute = [&]()
	{
		TArray<FAerotelicBooleanOperand> Bodies;
		for (const FAerotelicBodyChoicePtr& C : Picker->GetChosen())
		{
			Bodies.Add(C->Operand);
		}
		if (Bodies.Num() == 0)
		{
			AerotelicMessageBox(EAppMsgType::Ok, LOCTEXT("NoneChosen", "Choose at least one body to weigh."), LOCTEXT("Title", "Mass Properties"));
			return;
		}
		const double Started = FPlatformTime::Seconds();
		const FAerotelicMassReport Report = ComputeAerotelicMassReport(Bodies, Tolerance);
		const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
		ReportString = FormatAerotelicMassReport(Report, Doc != nullptr ? Doc->GetUnits() : aerotelic::UnitSystem{});
		ReportText->SetText(FText::FromString(ReportString));
		const int32 LeftOut = Report.Bodies.Num() - Report.IncludedCount;
		Status(FText::FromString(FString::Printf(TEXT("Weighed %d %s in %.1f s%s"),
			Report.IncludedCount, Report.IncludedCount == 1 ? TEXT("body") : TEXT("bodies"), FPlatformTime::Seconds() - Started,
			LeftOut > 0 ? *FString::Printf(TEXT(" -- %d left out, see the report"), LeftOut) : TEXT(""))));
	};

	auto Copy = [&]()
	{
		if (ReportString.IsEmpty()) return;
		FPlatformApplicationMisc::ClipboardCopy(*ReportString);
		Status(LOCTEXT("Copied", "Report copied to the clipboard."));
	};

	auto Save = [&]()
	{
		if (ReportString.IsEmpty()) return;
		IDesktopPlatform* Desktop = FDesktopPlatformModule::Get();
		if (Desktop == nullptr) return;
		TArray<FString> Files;
		const FString Default = DefaultReportPath();
		const void* Handle = FSlateApplication::Get().FindBestParentWindowHandleForDialogs(Window);
		if (!Desktop->SaveFileDialog(Handle, TEXT("Mass Properties Report"), FPaths::GetPath(Default),
			FPaths::GetCleanFilename(Default), TEXT("Text Files (*.txt)|*.txt"), EFileDialogFlags::None, Files) || Files.Num() == 0)
		{
			return;
		}
		if (FFileHelper::SaveStringToFile(ReportString, *Files[0]))
		{
			Status(FText::FromString(FString::Printf(TEXT("Report saved to %s"), *Files[0])));
		}
		else
		{
			AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(TEXT("Could not write ") + Files[0]), LOCTEXT("Title", "Mass Properties"));
		}
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
				MakeGroup(LOCTEXT("Bodies", "Bodies to weigh"),
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight()
					[
						SAssignNew(Picker, SAerotelicBodyPicker).Choices(Choices).ListHeight(150.0f)
						.OnChanged(FSimpleDelegate::CreateLambda([&RefreshCount]() { RefreshCount(); }))
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
					[
						MakeInfoLabel(LOCTEXT("BodiesHint",
							"Each entity weighs the way its own Mass Properties setting says (right-click it in the Scene tree): a density over the volume it encloses, or an asserted mass spread over its surface. One without a setting is reported as left out rather than counted as zero."))
					])
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
			[
				MakeGroup(LOCTEXT("MeshGroup", "Mesh"),
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
					[
						FormRow(LOCTEXT("Tolerance", "Surface tolerance:"),
							SNew(SAerotelicNumericEntry)
							.Value_Lambda([&Tolerance]() { return Tolerance; })
							.MinValue(1e-9).MaxValue(1e9).Delta(0.001)
							.Suffix(FText::FromString(FString(TEXT(" ")) + UTF8_TO_TCHAR(Units.LengthName())))
							.LiveUpdate(false)
							.OnValueCommitted_Lambda([&Tolerance](double V, ETextCommit::Type) { Tolerance = V; }))
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
					[
						MakeInfoLabel(LOCTEXT("ToleranceHint",
							"The chordal sag the triangles may have from the true surface -- the same tolerance STL export uses. The integrals are exact for the triangles, so this is the accuracy of the volume and the inertia of a curved body. A box is exact at any setting."))
					])
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8).HAlign(HAlign_Left)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[ Button(LOCTEXT("Compute", "Compute"), [&Compute]() { Compute(); }) ]
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[ Button(LOCTEXT("Copy", "Copy"), [&Copy]() { Copy(); }) ]
				+ SHorizontalBox::Slot().AutoWidth()
				[ Button(LOCTEXT("Save", "Save..."), [&Save]() { Save(); }) ]
			]
			+ SVerticalBox::Slot().FillHeight(1).Padding(0, 0, 0, 10)
			[
				SNew(SBorder).Padding(8)
				.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
				.BorderBackgroundColor(FLinearColor(0.06f, 0.06f, 0.07f))
				[
					// A table row is wider than the window can be made on a
					// small screen: scroll both ways rather than wrap, which
					// would break the columns.
					SNew(SScrollBox)
					+ SScrollBox::Slot()
					[
						SNew(SScrollBox).Orientation(Orient_Horizontal)
						+ SScrollBox::Slot()
						[
							SAssignNew(ReportText, STextBlock)
							.Font(FCoreStyle::GetDefaultFontStyle("Mono", 9))
							.Text(LOCTEXT("Empty", "Choose the bodies and press Compute."))
						]
					]
				]
			]
			+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
			[
				Button(LOCTEXT("Close", "Close"), [Window]() { Window->RequestDestroyWindow(); })
			]
		]);

	Status(LOCTEXT("SelectBody", "Select the bodies to weigh."));
	FSlateApplication::Get().AddModalWindow(Window, FSlateApplication::Get().GetActiveTopLevelWindow());
}

#undef LOCTEXT_NAMESPACE
