#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"

// One tab: its header label and its content, built once up front (every
// dialog's tabs are cheap Slate trees over an already-live model, not
// something worth lazily constructing).
struct FAerotelicTab
{
	FText Label;
	TSharedRef<SWidget> Content;
};

// A minimal tab strip: a row of header buttons over an SWidgetSwitcher.
// Shared by every entity dialog instead of each rolling its own, and by the
// Propeller dialog, which concatenates the Wing dialog's four tabs with its
// own two into a single strip (see WingDialog.h's BuildWingTabs()).
class SAerotelicTabStrip : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SAerotelicTabStrip) {}
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs, TArray<FAerotelicTab> InTabs);

private:
	TSharedPtr<class SWidgetSwitcher> Switcher;
};
