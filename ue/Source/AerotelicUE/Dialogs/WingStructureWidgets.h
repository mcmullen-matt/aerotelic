#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"

namespace aerotelic { class WingModel; }

// The Structure tab's tables (2026-11-09): the rib rules, one row per
// spar and one per stringer of the wing (WingModel::structure, see
// WingStructureSpec in wing_model.hpp), each with Remove, and "+ Add"
// buttons. Edits write the panel's draft model directly and raise
// OnChanged; the scene follows on the panel's Apply, as for every other
// tab. The model refuses a bad set (spars that cross, a spar through a
// flap's pocket) when it builds, and the panel reports that then.
class SWingStructureWidget : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SWingStructureWidget) {}
		SLATE_EVENT(FSimpleDelegate, OnChanged)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs, aerotelic::WingModel* InModel);

private:
	aerotelic::WingModel* Model = nullptr;
	FSimpleDelegate OnChangedDelegate;
	TSharedPtr<class SVerticalBox> SparRows, StringerRows;

	TSharedRef<SWidget> BuildRibRules();
	void RebuildSparRows();
	void RebuildStringerRows();
	void AddSpar();
	void RemoveSpar(int32 Index);
	void AddStringer();
	void RemoveStringer(int32 Index);
	void Changed();
};
