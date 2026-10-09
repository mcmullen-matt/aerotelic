#include "PromptDialogs.h"

#include "Widgets/SWindow.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SSpinBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Input/SButton.h"
#include "Framework/Application/SlateApplication.h"
#include "Styling/AppStyle.h"

TOptional<double> PromptForDouble(const FText& Title, const FText& Label,
	double InitialValue, double MinValue, double MaxValue, double Delta)
{
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(Title)
		.SizingRule(ESizingRule::Autosized)
		.SupportsMaximize(false)
		.SupportsMinimize(false);

	double Value = InitialValue;
	bool bAccepted = false;
	TSharedPtr<SSpinBox<double>> SpinBox;

	TSharedRef<SWidget> Content =
		SNew(SBorder)
		.Padding(12)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
			[
				SNew(STextBlock).Text(Label).AutoWrapText(true)
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 12)
			[
				SAssignNew(SpinBox, SSpinBox<double>)
				.Value(InitialValue)
				.MinValue(MinValue)
				.MaxValue(MaxValue)
				.Delta(Delta)
				.MinDesiredWidth(140.0f)
				.OnValueChanged_Lambda([&Value](double V) { Value = V; })
			]
			+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[
					SNew(SButton).Text(NSLOCTEXT("PromptForDouble", "OK", "OK"))
					.OnClicked_Lambda([&bAccepted, Window]() { bAccepted = true; Window->RequestDestroyWindow(); return FReply::Handled(); })
				]
				+ SHorizontalBox::Slot().AutoWidth()
				[
					SNew(SButton).Text(NSLOCTEXT("PromptForDouble", "Cancel", "Cancel"))
					.OnClicked_Lambda([Window]() { Window->RequestDestroyWindow(); return FReply::Handled(); })
				]
			]
		];

	Window->SetContent(Content);
	FSlateApplication::Get().AddModalWindow(Window, FSlateApplication::Get().GetActiveTopLevelWindow());

	if (bAccepted)
	{
		return Value;
	}
	return TOptional<double>();
}

TOptional<FString> PromptForString(const FText& Title, const FText& Label, const FString& InitialValue)
{
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(Title)
		.SizingRule(ESizingRule::Autosized)
		.SupportsMaximize(false)
		.SupportsMinimize(false);

	FString Value = InitialValue;
	bool bAccepted = false;
	TSharedPtr<SEditableTextBox> TextBox;

	auto Accept = [&bAccepted, Window]() { bAccepted = true; Window->RequestDestroyWindow(); };

	TSharedRef<SWidget> Content =
		SNew(SBorder)
		.Padding(12)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
			[
				SNew(STextBlock).Text(Label).AutoWrapText(true)
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 12)
			[
				SAssignNew(TextBox, SEditableTextBox)
				.Text(FText::FromString(InitialValue))
				.MinDesiredWidth(240.0f)
				.SelectAllTextWhenFocused(true)
				.OnTextChanged_Lambda([&Value](const FText& T) { Value = T.ToString(); })
				.OnTextCommitted_Lambda([&Value, Accept](const FText& T, ETextCommit::Type Commit)
				{
					Value = T.ToString();
					if (Commit == ETextCommit::OnEnter) { Accept(); }
				})
			]
			+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[
					SNew(SButton).Text(NSLOCTEXT("PromptForString", "OK", "OK"))
					.OnClicked_Lambda([Accept]() { Accept(); return FReply::Handled(); })
				]
				+ SHorizontalBox::Slot().AutoWidth()
				[
					SNew(SButton).Text(NSLOCTEXT("PromptForString", "Cancel", "Cancel"))
					.OnClicked_Lambda([Window]() { Window->RequestDestroyWindow(); return FReply::Handled(); })
				]
			]
		];

	Window->SetContent(Content);
	Window->SetWidgetToFocusOnActivate(TextBox);
	FSlateApplication::Get().AddModalWindow(Window, FSlateApplication::Get().GetActiveTopLevelWindow());

	if (bAccepted)
	{
		return Value;
	}
	return TOptional<FString>();
}
