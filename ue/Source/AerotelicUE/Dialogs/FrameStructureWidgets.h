#pragma once

#include "CoreMinimal.h"
#include "Widgets/SWidget.h"
#include "Widgets/SCompoundWidget.h"

namespace aerotelic { struct FrameStructureSpec; }

// The Structure tab of a body lofted along its length -- a fuselage, a
// nacelle (2026-11-10): the frame rules and one row per stringer
// (structure_plan.hpp FrameStructureSpec), each with Remove, and "+ Add".
// Edits write the panel's draft spec directly and raise OnChanged; the
// scene follows on the panel's Apply, as for every other tab.
class SFrameStructureWidget : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SFrameStructureWidget) {}
		SLATE_EVENT(FSimpleDelegate, OnChanged)
		// What the positions round the section mean for this body, shown
		// above the stringer table.
		SLATE_ARGUMENT(FText, AroundHint)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs, aerotelic::FrameStructureSpec* InSpec);

private:
	aerotelic::FrameStructureSpec* Spec = nullptr;
	FSimpleDelegate OnChangedDelegate;
	TSharedPtr<class SVerticalBox> StringerRows;

	TSharedRef<SWidget> BuildFrameRules();
	void RebuildStringerRows();
	void AddStringer();
	void RemoveStringer(int32 Index);
	void Changed();
};

// The Structure tab's content for a fuselage or a nacelle: the widget in
// a scroll box with its description, and the Help paragraph for the
// panel's Help tab.
TSharedRef<SWidget> MakeFrameStructureTab(aerotelic::FrameStructureSpec* Spec, FSimpleDelegate OnChanged, const FText& AroundHint);
FText FrameStructureHelpText();
