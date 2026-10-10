#include "BooleanOperationDialog.h"

#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Views/SListView.h"
#include "Widgets/Views/STableRow.h"
#include "Framework/Application/SlateApplication.h"
#include "Styling/CoreStyle.h"
#include "Misc/MessageDialog.h"

#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"
#include "Entity/AerotelicBooleanModel.h"
#include "MainShell/AerotelicSceneTree.h"
#include "Widgets/CurveTabControls.h"

#define LOCTEXT_NAMESPACE "BooleanOperationDialog"

namespace
{
	// One selectable operand (_boolean_operand_choices): a single node, so
	// selecting two rows operates on two bodies -- to include both halves
	// of a mirrored part, pick both rows.
	struct FChoice
	{
		FString Label;
		FAerotelicBooleanOperand Operand;
	};
	using FChoicePtr = TSharedPtr<FChoice>;

	TArray<FChoicePtr> OperandChoices(UAerotelicDocumentSubsystem* Doc)
	{
		TArray<FChoicePtr> Out;
		for (const TWeakObjectPtr<AAerotelicEntityActor>& Weak : Doc->GetEntities())
		{
			if (!Weak.IsValid())
			{
				continue;
			}
			FChoicePtr C = MakeShared<FChoice>();
			C->Label = Weak->EntityName;
			C->Operand.Entity = Weak;
			Out.Add(C);
			if (Weak->IsMirrored())
			{
				FChoicePtr M = MakeShared<FChoice>();
				M->Label = Weak->EntityName + TEXT(" (mirror)");
				M->Operand.Entity = Weak;
				M->Operand.bMirror = true;
				Out.Add(M);
			}
		}
		return Out;
	}

	TSharedRef<ITableRow> ChoiceRow(FChoicePtr Item, const TSharedRef<STableViewBase>& Owner)
	{
		return SNew(STableRow<FChoicePtr>, Owner).Padding(FMargin(4, 2))
			[ SNew(STextBlock).Text(FText::FromString(Item->Label)) ];
	}
}

FText ShowAerotelicBooleanOperationDialog()
{
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	if (Doc == nullptr)
	{
		return FText::GetEmpty();
	}
	TArray<FChoicePtr> Available = OperandChoices(Doc);
	if (Available.Num() < 2)
	{
		FMessageDialog::Open(EAppMsgType::Ok,
			LOCTEXT("NeedTwo", "Need at least two entities in the scene to run a boolean operation."),
			LOCTEXT("Title", "Boolean Operation"));
		return FText::GetEmpty();
	}

	TArray<FChoicePtr> Chosen;
	TSharedPtr<SListView<FChoicePtr>> AvailableList, ChosenList;
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

	auto AddRows = [&Chosen, &ChosenList](const TArray<FChoicePtr>& Rows)
	{
		for (const FChoicePtr& R : Rows)
		{
			if (!Chosen.Contains(R))
			{
				Chosen.Add(R);
			}
		}
		if (ChosenList.IsValid()) ChosenList->RequestListRefresh();
	};
	auto Shift = [&Chosen, &ChosenList](int32 Delta)
	{
		if (!ChosenList.IsValid()) return;
		TArray<FChoicePtr> Sel = ChosenList->GetSelectedItems();
		if (Sel.Num() == 0) return;
		const int32 Row = Chosen.IndexOfByKey(Sel[0]);
		const int32 New = Row + Delta;
		if (Row < 0 || New < 0 || New >= Chosen.Num()) return;
		Chosen.Swap(Row, New);
		ChosenList->RequestListRefresh();
		ChosenList->SetSelection(Sel[0]);
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
			+ SVerticalBox::Slot().FillHeight(1.0f).Padding(0, 0, 0, 10)
			[
				MakeGroup(LOCTEXT("Operands", "Operands  (first is the base; applied top to bottom)"),
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().FillWidth(1.0f)
					[
						SNew(SBorder).Padding(1).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(FLinearColor(0.06f, 0.06f, 0.07f))
						[
							SAssignNew(AvailableList, SListView<FChoicePtr>)
							.ListItemsSource(&Available)
							.SelectionMode(ESelectionMode::Multi)
							.OnGenerateRow_Static(&ChoiceRow)
						]
					]
					+ SHorizontalBox::Slot().AutoWidth().Padding(6, 0)
					[
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ Button(LOCTEXT("Add", "Add →"), [&]() { if (AvailableList.IsValid()) AddRows(AvailableList->GetSelectedItems()); }) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ Button(LOCTEXT("AddAll", "Select All →"), [&]() { AddRows(Available); }) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ Button(LOCTEXT("Remove", "← Remove"), [&]()
							{
								if (!ChosenList.IsValid()) return;
								for (const FChoicePtr& R : ChosenList->GetSelectedItems()) Chosen.Remove(R);
								ChosenList->RequestListRefresh();
							}) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ Button(LOCTEXT("Up", "Up"), [&]() { Shift(-1); }) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ Button(LOCTEXT("Down", "Down"), [&]() { Shift(1); }) ]
					]
					+ SHorizontalBox::Slot().FillWidth(1.0f)
					[
						SNew(SBorder).Padding(1).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(FLinearColor(0.06f, 0.06f, 0.07f))
						[
							SAssignNew(ChosenList, SListView<FChoicePtr>)
							.ListItemsSource(&Chosen)
							.SelectionMode(ESelectionMode::Single)
							.OnGenerateRow_Static(&ChoiceRow)
						]
					])
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
	for (const FChoicePtr& C : Chosen)
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
