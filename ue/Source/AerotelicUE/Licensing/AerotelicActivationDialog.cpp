#include "AerotelicActivationDialog.h"
#include "Branding/AerotelicBranding.h"

#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Framework/Application/SlateApplication.h"
#include "Styling/CoreStyle.h"

#define LOCTEXT_NAMESPACE "AerotelicActivationDialog"

using namespace AerotelicLicence;

TOptional<FCheckResult> EnsureAerotelicLicensed()
{
	FCheckResult Result = CheckLicence();
	if (Result.Ok())
	{
		return Result;
	}

	bool bPassed = false;
	TSharedPtr<STextBlock> ReasonText, StatusText;
	TSharedPtr<SEditableTextBox> KeyBox;
	const FLinearColor Red(0.847f, 0.416f, 0.353f); // #d86a5a

	auto ReasonFor = [](const FCheckResult& R)
	{
		FString Text = R.Message;
		if (R.Licence.IsSet())
		{
			Text += FString::Printf(TEXT("\n\nSerial %s"), *R.Licence->Serial);
			if (!R.Licence->Licensee.IsEmpty()) Text += FString::Printf(TEXT(", licensed to %s"), *R.Licence->Licensee);
		}
		return FText::FromString(Text);
	};

	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(FText::FromString(FString::Printf(TEXT("%s — licence"), AerotelicBranding::Product)))
		.SizingRule(ESizingRule::Autosized)
		.MinWidth(480)
		.SupportsMaximize(false)
		.SupportsMinimize(false);

	auto CheckAgain = [&]()
	{
		Result = CheckLicence();
		if (Result.Ok())
		{
			bPassed = true;
			Window->RequestDestroyWindow();
			return;
		}
		ReasonText->SetText(ReasonFor(Result));
		StatusText->SetText(LOCTEXT("StillNotValid", "The licence is still not valid."));
	};
	auto Activate = [&]()
	{
		const FString Key = KeyBox->GetText().ToString();
		if (!LooksLikeKey(Key))
		{
			StatusText->SetText(FText::Format(LOCTEXT("BadKey", "A licence key is {0} letters and digits, usually in five groups of five."), KeyLength));
			return;
		}
		FString LicenceText, Err;
		if (!AerotelicLicence::Activate(Key, LicenceText, Err))
		{
			StatusText->SetText(FText::FromString(Err));
			return;
		}
		if (!InstallLicence(LicenceText, Err))
		{
			StatusText->SetText(FText::FromString(Err));
			return;
		}
		CheckAgain();
	};

	Window->SetContent(
		SNew(SBorder).Padding(16)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
			[
				SNew(STextBlock)
				.Font(FCoreStyle::GetDefaultFontStyle("Bold", 14))
				.Text(FText::FromString(FString::Printf(TEXT("%s needs a valid licence to start."), AerotelicBranding::Product)))
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 12)
			[
				SNew(SBox).MaxDesiredWidth(560)
				[ SAssignNew(ReasonText, STextBlock).AutoWrapText(true).Text(ReasonFor(Result)) ]
			]
			+ SVerticalBox::Slot().AutoHeight()
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0)
				[ SNew(STextBlock).Text(LOCTEXT("KeyLabel", "Licence key:")) ]
				+ SHorizontalBox::Slot().FillWidth(1.0f)
				[
					SAssignNew(KeyBox, SEditableTextBox)
					.HintText(LOCTEXT("KeyHint", "XXXXX-XXXXX-XXXXX-XXXXX-XXXXX"))
					.OnTextCommitted_Lambda([&](const FText&, ETextCommit::Type Commit) { if (Commit == ETextCommit::OnEnter) Activate(); })
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 12)
			[
				SNew(SBox).MaxDesiredWidth(560)
				[ SAssignNew(StatusText, STextBlock).AutoWrapText(true).ColorAndOpacity(FSlateColor(Red)) ]
			]
			+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[ SNew(SButton).Text(LOCTEXT("CheckAgain", "Check again")).OnClicked_Lambda([&]() { CheckAgain(); return FReply::Handled(); }) ]
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[ SNew(SButton).Text(LOCTEXT("Activate", "Activate")).OnClicked_Lambda([&]() { Activate(); return FReply::Handled(); }) ]
				+ SHorizontalBox::Slot().AutoWidth()
				[ SNew(SButton).Text(LOCTEXT("Quit", "Quit")).OnClicked_Lambda([&]() { Window->RequestDestroyWindow(); return FReply::Handled(); }) ]
			]
		]);

	FSlateApplication::Get().AddModalWindow(Window, FSlateApplication::Get().GetActiveTopLevelWindow());
	if (bPassed)
	{
		return Result;
	}
	return TOptional<FCheckResult>();
}

#undef LOCTEXT_NAMESPACE
