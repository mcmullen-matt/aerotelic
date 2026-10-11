#include "MessageBox.h"

#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Framework/Application/SlateApplication.h"
#include "Misc/App.h"

#include "Branding/AerotelicBranding.h"

#define LOCTEXT_NAMESPACE "AerotelicMessageBox"

EAppReturnType::Type AerotelicMessageBox(EAppMsgType::Type Type, const FText& Message, const FText& Title)
{
	if (!FSlateApplication::IsInitialized() || IsRunningCommandlet() || FApp::IsUnattended() || !FSlateApplication::Get().CanDisplayWindows())
	{
		return FMessageDialog::Open(Type, Message, Title);
	}

	// The buttons each kind offers, in order, and what each answers.
	struct FChoice { FText Label; EAppReturnType::Type Answer; };
	TArray<FChoice> Choices;
	switch (Type)
	{
	case EAppMsgType::YesNo:
		Choices = { { LOCTEXT("Yes", "Yes"), EAppReturnType::Yes }, { LOCTEXT("No", "No"), EAppReturnType::No } };
		break;
	case EAppMsgType::OkCancel:
		Choices = { { LOCTEXT("OK", "OK"), EAppReturnType::Ok }, { LOCTEXT("Cancel", "Cancel"), EAppReturnType::Cancel } };
		break;
	case EAppMsgType::YesNoCancel:
		Choices = { { LOCTEXT("Yes", "Yes"), EAppReturnType::Yes }, { LOCTEXT("No", "No"), EAppReturnType::No }, { LOCTEXT("Cancel", "Cancel"), EAppReturnType::Cancel } };
		break;
	default:
		Choices = { { LOCTEXT("OK", "OK"), EAppReturnType::Ok } };
		break;
	}
	// Closing the window with its X is the negative answer.
	EAppReturnType::Type Result = Choices.Last().Answer;

	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(Title.IsEmpty() ? FText::FromString(AerotelicBranding::Product) : Title)
		.SizingRule(ESizingRule::Autosized)
		.SupportsMaximize(false)
		.SupportsMinimize(false)
		.HasCloseButton(true);

	TSharedRef<SHorizontalBox> Buttons = SNew(SHorizontalBox);
	for (const FChoice& C : Choices)
	{
		const EAppReturnType::Type Answer = C.Answer;
		Buttons->AddSlot().AutoWidth().Padding(6, 0, 0, 0)
		[
			SNew(SBox).MinDesiredWidth(72)
			[
				SNew(SButton).Text(C.Label).HAlign(HAlign_Center)
				.OnClicked_Lambda([&Result, Answer, Window]() { Result = Answer; Window->RequestDestroyWindow(); return FReply::Handled(); })
			]
		];
	}

	Window->SetContent(
		SNew(SBorder).Padding(FMargin(16, 14))
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 16)
			[
				SNew(SBox).MinDesiredWidth(260).MaxDesiredWidth(520)
				[ SNew(STextBlock).Text(Message).AutoWrapText(true) ]
			]
			+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
			[ Buttons ]
		]);

	FSlateApplication::Get().AddModalWindow(Window, FSlateApplication::Get().GetActiveTopLevelWindow());
	return Result;
}

#undef LOCTEXT_NAMESPACE
