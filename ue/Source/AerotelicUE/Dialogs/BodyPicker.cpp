#include "BodyPicker.h"

#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Views/SListView.h"
#include "Widgets/Views/STableRow.h"
#include "Styling/CoreStyle.h"

#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"

#define LOCTEXT_NAMESPACE "BodyPicker"

TArray<FAerotelicBodyChoicePtr> SceneBodyChoices()
{
	TArray<FAerotelicBodyChoicePtr> Out;
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	if (Doc == nullptr)
	{
		return Out;
	}
	for (const TWeakObjectPtr<AAerotelicEntityActor>& Weak : Doc->GetEntities())
	{
		if (!Weak.IsValid())
		{
			continue;
		}
		FAerotelicBodyChoicePtr C = MakeShared<FAerotelicBodyChoice>();
		C->Label = Weak->EntityName;
		C->Operand.Entity = Weak;
		Out.Add(C);
		if (Weak->IsMirrored())
		{
			FAerotelicBodyChoicePtr M = MakeShared<FAerotelicBodyChoice>();
			M->Label = Weak->EntityName + TEXT(" (mirror)");
			M->Operand.Entity = Weak;
			M->Operand.bMirror = true;
			Out.Add(M);
		}
	}
	return Out;
}

void SAerotelicBodyPicker::Construct(const FArguments& InArgs)
{
	Available = InArgs._Choices;
	OnChanged = InArgs._OnChanged;
	const bool bReorder = InArgs._AllowReorder;

	auto Button = [this](const FText& Label, void (SAerotelicBodyPicker::*Fn)())
	{
		return SNew(SButton).Text(Label).HAlign(HAlign_Center)
			.OnClicked_Lambda([this, Fn]() { (this->*Fn)(); return FReply::Handled(); });
	};
	auto ListBox = [](TSharedRef<SWidget> List)
	{
		return SNew(SBorder).Padding(1).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(FLinearColor(0.06f, 0.06f, 0.07f))
			[ List ];
	};

	TSharedRef<SVerticalBox> Buttons = SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ Button(LOCTEXT("Add", "Add →"), &SAerotelicBodyPicker::AddSelectedFromAvailable) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ Button(LOCTEXT("AddAll", "Select All →"), &SAerotelicBodyPicker::AddAll) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ Button(LOCTEXT("Remove", "← Remove"), &SAerotelicBodyPicker::RemoveSelected) ];
	if (bReorder)
	{
		Buttons->AddSlot().AutoHeight().Padding(0, 2) [ Button(LOCTEXT("Up", "Up"), &SAerotelicBodyPicker::ShiftUp) ];
		Buttons->AddSlot().AutoHeight().Padding(0, 2) [ Button(LOCTEXT("Down", "Down"), &SAerotelicBodyPicker::ShiftDown) ];
	}

	ChildSlot
	[
		SNew(SBox).HeightOverride(InArgs._ListHeight)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().FillWidth(1.0f)
			[
				ListBox(SAssignNew(AvailableList, SListView<FAerotelicBodyChoicePtr>)
					.ListItemsSource(&Available)
					.SelectionMode(ESelectionMode::Multi)
					.OnGenerateRow(this, &SAerotelicBodyPicker::MakeRow))
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(6, 0) [ Buttons ]
			+ SHorizontalBox::Slot().FillWidth(1.0f)
			[
				ListBox(SAssignNew(ChosenList, SListView<FAerotelicBodyChoicePtr>)
					.ListItemsSource(&Chosen)
					.SelectionMode(bReorder ? ESelectionMode::Single : ESelectionMode::Multi)
					.OnGenerateRow(this, &SAerotelicBodyPicker::MakeRow))
			]
		]
	];
}

TSharedRef<ITableRow> SAerotelicBodyPicker::MakeRow(FAerotelicBodyChoicePtr Item, const TSharedRef<STableViewBase>& Owner)
{
	return SNew(STableRow<FAerotelicBodyChoicePtr>, Owner).Padding(FMargin(4, 2))
		[ SNew(STextBlock).Text(FText::FromString(Item->Label)) ];
}

void SAerotelicBodyPicker::AddRows(const TArray<FAerotelicBodyChoicePtr>& Rows)
{
	bool bAdded = false;
	for (const FAerotelicBodyChoicePtr& R : Rows)
	{
		if (!Chosen.Contains(R))
		{
			Chosen.Add(R);
			bAdded = true;
		}
	}
	if (bAdded)
	{
		ChosenList->RequestListRefresh();
		OnChanged.ExecuteIfBound();
	}
}

void SAerotelicBodyPicker::AddSelectedFromAvailable()
{
	AddRows(AvailableList->GetSelectedItems());
}

void SAerotelicBodyPicker::AddAll()
{
	AddRows(Available);
}

void SAerotelicBodyPicker::RemoveSelected()
{
	TArray<FAerotelicBodyChoicePtr> Sel = ChosenList->GetSelectedItems();
	if (Sel.Num() == 0)
	{
		return;
	}
	for (const FAerotelicBodyChoicePtr& R : Sel)
	{
		Chosen.Remove(R);
	}
	ChosenList->RequestListRefresh();
	OnChanged.ExecuteIfBound();
}

void SAerotelicBodyPicker::Shift(int32 Delta)
{
	TArray<FAerotelicBodyChoicePtr> Sel = ChosenList->GetSelectedItems();
	if (Sel.Num() == 0)
	{
		return;
	}
	const int32 Row = Chosen.IndexOfByKey(Sel[0]);
	const int32 New = Row + Delta;
	if (Row < 0 || New < 0 || New >= Chosen.Num())
	{
		return;
	}
	Chosen.Swap(Row, New);
	ChosenList->RequestListRefresh();
	ChosenList->SetSelection(Sel[0]);
	OnChanged.ExecuteIfBound();
}

void SAerotelicBodyPicker::ShiftUp() { Shift(-1); }
void SAerotelicBodyPicker::ShiftDown() { Shift(1); }

#undef LOCTEXT_NAMESPACE
