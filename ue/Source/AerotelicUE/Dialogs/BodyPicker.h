#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"

#include "Entity/AerotelicBooleanModel.h"

template <typename ItemType> class SListView;
class ITableRow;
class STableViewBase;

// One selectable scene body (main_window.py _boolean_operand_choices): a
// single node -- an entity, the mirrored half of a mirrored entity as a
// row of its own, or a boolean result -- never expanded implicitly.
struct FAerotelicBodyChoice
{
	FString Label;
	FAerotelicBooleanOperand Operand;
};
using FAerotelicBodyChoicePtr = TSharedPtr<FAerotelicBodyChoice>;

// Every body the scene offers, in scene order.
TArray<FAerotelicBodyChoicePtr> SceneBodyChoices();

// The two-list body picker shared by the boolean and export dialogs
// (_make_body_picker): available on the left, chosen on the right, with
// Add / Select All / Remove, and Up / Down when order matters (the
// boolean operands; exports deliberately do without).
class SAerotelicBodyPicker : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SAerotelicBodyPicker) : _AllowReorder(false), _ListHeight(220.0f) {}
		SLATE_ARGUMENT(TArray<FAerotelicBodyChoicePtr>, Choices)
		// Rows (of Choices) that start out chosen.
		SLATE_ARGUMENT(TArray<FAerotelicBodyChoicePtr>, InitiallyChosen)
		SLATE_ARGUMENT(bool, AllowReorder)
		SLATE_ARGUMENT(float, ListHeight)
		SLATE_EVENT(FSimpleDelegate, OnChanged)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	const TArray<FAerotelicBodyChoicePtr>& GetChosen() const { return Chosen; }

private:
	TArray<FAerotelicBodyChoicePtr> Available;
	TArray<FAerotelicBodyChoicePtr> Chosen;
	TSharedPtr<SListView<FAerotelicBodyChoicePtr>> AvailableList;
	TSharedPtr<SListView<FAerotelicBodyChoicePtr>> ChosenList;
	FSimpleDelegate OnChanged;

	void AddRows(const TArray<FAerotelicBodyChoicePtr>& Rows);
	void AddSelectedFromAvailable();
	void AddAll();
	void RemoveSelected();
	void Shift(int32 Delta);
	void ShiftUp();
	void ShiftDown();
	TSharedRef<ITableRow> MakeRow(FAerotelicBodyChoicePtr Item, const TSharedRef<STableViewBase>& Owner);
};
