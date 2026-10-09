#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/Views/STreeView.h"

class AAerotelicEntityActor;
class ITableRow;
class STableViewBase;

DECLARE_DELEGATE_OneParam(FOnAerotelicStatus, const FText&);

// The scene tree beside the 3D view (main_window.py's _make_entity_tree):
// one row per entity with a visibility checkbox, its name and kind; click to
// select, double-click to edit, right-click for Hide/Show, Edit, Rename,
// Transform, Color, Mirrored and Delete. Body child rows (a wing parted
// into several solids) are not ported yet -- entities render as one mesh.
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
	using FItem = TWeakObjectPtr<AAerotelicEntityActor>;

	TSharedRef<ITableRow> GenerateRow(FItem Item, const TSharedRef<STableViewBase>& OwnerTable);
	void GetChildren(FItem Item, TArray<FItem>& OutChildren);
	void OnSelectionChanged(FItem Item, ESelectInfo::Type SelectInfo);
	void OnDoubleClick(FItem Item);
	TSharedPtr<SWidget> OnContextMenuOpening();

	void RefreshItems();
	void SyncSelectionFromDocument();
	void Status(const FText& Text);

	void RenameEntity(AAerotelicEntityActor* Entity);
	void DeleteEntity(AAerotelicEntityActor* Entity);

	TArray<FItem> Items;
	TSharedPtr<STreeView<FItem>> Tree;
	bool bSyncingSelection = false;
	FOnAerotelicStatus OnStatusDelegate;
	FDelegateHandle ListChangedHandle;
	FDelegateHandle SelectionChangedHandle;
};
