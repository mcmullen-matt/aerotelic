#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"

namespace aerotelic { class WingModel; }

// The Control Surfaces tab's flap table: one row per trailing-edge flap of
// the wing (WingModel::flaps, see FlapSpec in wing_model.hpp) -- name, span
// extent (eta at root and tip), hinge chord fraction at each end, gap --
// plus Remove per row and "+ Add Flap". Edits write the panel's draft model
// directly and raise OnChanged; the scene follows on the panel's Apply, as
// for every other tab. The model refuses a bad set (overlapping flaps, a gap
// the section cannot hold) when it builds, and the panel reports that then,
// so nothing here needs to validate beyond each field's own range.
class SFlapListWidget : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SFlapListWidget) {}
		SLATE_EVENT(FSimpleDelegate, OnChanged)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs, aerotelic::WingModel* InModel);

private:
	aerotelic::WingModel* Model = nullptr;
	FSimpleDelegate OnChangedDelegate;
	TSharedPtr<class SVerticalBox> RowsBox;

	void RebuildRows();
	void AddFlap();
	void RemoveFlap(int32 Index);
	void Changed();
};
