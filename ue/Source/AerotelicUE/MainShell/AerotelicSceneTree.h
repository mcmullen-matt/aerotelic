#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/Views/STreeView.h"

class AAerotelicEntityActor;
class ITableRow;
class STableViewBase;

DECLARE_DELEGATE_OneParam(FOnAerotelicStatus, const FText&);

// One row of the scene tree -- the Slate form of WingDesigner's
// (kind, entity, body) row key. The list is the list of parameterisations:
// each entity is a row, with a child row per solid body it produces (the
// one solid of a fuselage; a propeller's spinner and blades) and, when the
// entity is mirrored, a child row per mirrored body.
struct FAerotelicSceneRow
{
	enum class EKind : uint8 { Entity, Body, Mirror };

	EKind Kind = EKind::Entity;
	TWeakObjectPtr<AAerotelicEntityActor> Entity;
	int32 BodyIndex = INDEX_NONE; // Body / Mirror rows: which body
	TArray<TSharedPtr<FAerotelicSceneRow>> Children;
};

// The scene tree beside the 3D view (main_window.py's _make_entity_tree):
// entity rows with a visibility checkbox, name, colour swatch and kind;
// body and mirror rows beneath with their own checkbox. Click to select
// (a body row selects its entity), double-click to edit, right-click for
// the row's menu: an entity's Hide/Show, Edit, Rename, Transform, Color,
// Mirrored and Delete; a body's Hide/Show and Color.
class SAerotelicSceneTree : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SAerotelicSceneTree) {}
		SLATE_EVENT(FOnAerotelicStatus, OnStatus)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);
	virtual ~SAerotelicSceneTree();

	// Selects Entity and brings its design panel to the front (the tree's
	// Edit / double-click; also what the toolbar's Add buttons do).
	static void EditEntity(AAerotelicEntityActor* Entity);

private:
	using FItem = TSharedPtr<FAerotelicSceneRow>;

	TSharedRef<ITableRow> GenerateRow(FItem Item, const TSharedRef<STableViewBase>& OwnerTable);
	void GetChildren(FItem Item, TArray<FItem>& OutChildren);
	void OnSelectionChanged(FItem Item, ESelectInfo::Type SelectInfo);
	void OnDoubleClick(FItem Item);
	TSharedPtr<SWidget> OnContextMenuOpening();
	TSharedPtr<SWidget> MakeEntityMenu(FItem Item);
	TSharedPtr<SWidget> MakeBodyMenu(FItem Item);

	void RefreshItems();
	void OnBodiesChanged(AAerotelicEntityActor* Entity);
	void SyncSelectionFromDocument();
	void Status(const FText& Text);

	void RenameEntity(AAerotelicEntityActor* Entity);
	void DeleteEntity(AAerotelicEntityActor* Entity);
	void PickEntityColor(AAerotelicEntityActor* Entity);
	void PickBodyColor(AAerotelicEntityActor* Entity, int32 BodyIndex);

	TArray<FItem> Items;
	TSharedPtr<STreeView<FItem>> Tree;
	bool bSyncingSelection = false;
	FOnAerotelicStatus OnStatusDelegate;
	FDelegateHandle ListChangedHandle;
	FDelegateHandle SelectionChangedHandle;
	FDelegateHandle BodiesChangedHandle;
};
