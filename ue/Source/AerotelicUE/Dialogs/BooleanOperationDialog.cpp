#include "BooleanOperationDialog.h"
#include "BodyPicker.h"

#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Framework/Application/SlateApplication.h"
#include "Misc/MessageDialog.h"

#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"
#include "Entity/AerotelicBooleanModel.h"
#include "Widgets/CurveTabControls.h"

#define LOCTEXT_NAMESPACE "BooleanOperationDialog"

FText ShowAerotelicBooleanOperationDialog()
{
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	if (Doc == nullptr)
	{
		return FText::GetEmpty();
	}
	// The same rows the export dialogs offer (_boolean_operand_choices):
	// "what can I operate on" and "what can I export" are one question.
	TArray<FAerotelicBodyChoicePtr> Choices = SceneBodyChoices();
	if (Choices.Num() < 2)
	{
		FMessageDialog::Open(EAppMsgType::Ok,
			LOCTEXT("NeedTwo", "Need at least two entities in the scene to run a boolean operation."),
			LOCTEXT("Title", "Boolean Operation"));
		return FText::GetEmpty();
	}

	TSharedPtr<SAerotelicBodyPicker> Picker;
	TArray<TSharedPtr<FString>> Operations = { MakeShared<FString>(TEXT("Fuse")), MakeShared<FString>(TEXT("Cut (first − rest)")), MakeShared<FString>(TEXT("Common")) };
	TSharedPtr<FString> Operation = Operations[0];
	const FString DefaultName = Doc->NextBooleanName();
	TSharedPtr<SEditableTextBox> NameBox;
	bool bAccepted = false;

	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(LOCTEXT("Title", "Boolean Operation"))
		.ClientSize(FVector2D(520, 420))
		.SupportsMaximize(false)
		.SupportsMinimize(false);

	auto Button = [](const FText& Label, TFunction<void()> Fn)
	{
		return SNew(SButton).Text(Label).HAlign(HAlign_Center)
			.OnClicked_Lambda([Fn]() { Fn(); return FReply::Handled(); });
	};

	Window->SetContent(
		SNew(SBorder).Padding(12)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().FillHeight(1.0f).Padding(0, 0, 0, 10)
			[
				// An ordered list, not a pair: the operation folds left to
				// right, so for Cut the first entry is the base and the rest
				// are subtracted from it.
				MakeGroup(LOCTEXT("Operands", "Operands  (first is the base; applied top to bottom)"),
					SAssignNew(Picker, SAerotelicBodyPicker).Choices(Choices).AllowReorder(true).ListHeight(230.0f))
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0) [ SNew(STextBlock).Text(LOCTEXT("Operation", "Operation:")) ]
				+ SHorizontalBox::Slot().FillWidth(1.0f)
				[
					SNew(SComboBox<TSharedPtr<FString>>)
					.OptionsSource(&Operations)
					.InitiallySelectedItem(Operation)
					.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
					.OnSelectionChanged_Lambda([&Operation](TSharedPtr<FString> S, ESelectInfo::Type) { if (S.IsValid()) Operation = S; })
					[ SNew(STextBlock).Text_Lambda([&Operation]() { return FText::FromString(Operation.IsValid() ? *Operation : FString()); }) ]
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 12)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0) [ SNew(STextBlock).Text(LOCTEXT("Name", "Name:")) ]
				+ SHorizontalBox::Slot().FillWidth(1.0f) [ SAssignNew(NameBox, SEditableTextBox).Text(FText::FromString(DefaultName)) ]
			]
			+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[ Button(LOCTEXT("OK", "OK"), [&]() { bAccepted = true; Window->RequestDestroyWindow(); }) ]
				+ SHorizontalBox::Slot().AutoWidth()
				[ Button(LOCTEXT("Cancel", "Cancel"), [Window]() { Window->RequestDestroyWindow(); }) ]
			]
		]);

	FSlateApplication::Get().AddModalWindow(Window, FSlateApplication::Get().GetActiveTopLevelWindow());
	if (!bAccepted)
	{
		return FText::GetEmpty();
	}

	const TArray<FAerotelicBodyChoicePtr>& Chosen = Picker->GetChosen();
	if (Chosen.Num() < 2)
	{
		FMessageDialog::Open(EAppMsgType::Ok, LOCTEXT("AtLeastTwo", "Select at least two operands."), LOCTEXT("Title", "Boolean Operation"));
		return FText::GetEmpty();
	}
	const FString OpType = *Operation == TEXT("Fuse") ? TEXT("fuse") : (*Operation == TEXT("Common") ? TEXT("common") : TEXT("cut"));
	FString Name = NameBox.IsValid() ? NameBox->GetText().ToString().TrimStartAndEnd() : FString();
	if (Name.IsEmpty())
	{
		Name = DefaultName;
	}
	if (Doc->IsNameTaken(Name))
	{
		FMessageDialog::Open(EAppMsgType::Ok,
			FText::Format(LOCTEXT("NameTaken", "'{0}' is already in use. Names must be unique in the scene."), FText::FromString(Name)),
			LOCTEXT("Title", "Boolean Operation"));
		return FText::GetEmpty();
	}
	TArray<FAerotelicBooleanOperand> Operands;
	for (const FAerotelicBodyChoicePtr& C : Chosen)
	{
		Operands.Add(C->Operand);
	}
	FString Error;
	AAerotelicEntityActor* Result = Doc->AddBoolean(Name, Operands, OpType, Error);
	if (Result == nullptr)
	{
		FMessageDialog::Open(EAppMsgType::Ok, FText::FromString(Error), LOCTEXT("Failed", "Boolean Operation Failed"));
		return FText::FromString(TEXT("Ready"));
	}
	Doc->SetSelectedEntity(Result);
	return FText::Format(LOCTEXT("Built", "Boolean operation '{0}' built."), FText::FromString(Name));
}

#undef LOCTEXT_NAMESPACE
