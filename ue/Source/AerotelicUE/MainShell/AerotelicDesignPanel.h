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
};
