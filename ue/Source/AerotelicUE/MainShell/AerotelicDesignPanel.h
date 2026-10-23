#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "Dialogs/EntityDesignContent.h"

class AAerotelicEntityActor;
class SBox;

// The content of the design dock's "Design" tab: the design panel of the
// currently selected entity (Wing Design / Fuselage Design / ... in the Qt
// version's right-hand dock column), or a hint when nothing is selected.
// Panels are built once per entity and kept while it exists, so switching
// selection back and forth keeps each panel's tab state.
//
// Every panel edits a DRAFT of its entity (FAerotelicDesignDraft) and ends
// in the same footer: Apply hands the draft to the entity -- which is when
// the viewport, the booleans over it and the lattices over it change --
// and Revert throws the draft away and rebuilds the panel on a fresh copy.
// Nothing in the scene changes as fields are typed into.
class SAerotelicDesignPanel : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SAerotelicDesignPanel) {}
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);
	virtual ~SAerotelicDesignPanel();

private:
	TSharedPtr<SBox> Host;
	TMap<TWeakObjectPtr<AAerotelicEntityActor>, FAerotelicDesignContent> Panels;
	TMap<TWeakObjectPtr<AAerotelicEntityActor>, TSharedPtr<SWidget>> Composed;
	TWeakObjectPtr<AAerotelicEntityActor> Shown;
	FDelegateHandle SelectionHandle;
	FDelegateHandle ListHandle;

	void Refresh();
	void PruneDead();
	// The footer's buttons.
	void ApplyDraft(TWeakObjectPtr<AAerotelicEntityActor> Entity);
	void RevertDraft(TWeakObjectPtr<AAerotelicEntityActor> Entity);
};
