#pragma once

#include "CoreMinimal.h"
#include "Widgets/SWidget.h"

#include "aerotelic/wing_model.hpp"

// Builds the Planform / Dihedral / Sections / Twist tab contents every
// wing-shaped entity edits, factored out so the Propeller dialog can insert
// its own Propeller/Spinner/Help tabs around the same four (matching
// propeller_panel.py's structural trick of inserting into
// NURBSPlanformPanel's own QTabWidget). Not a widget itself -- just builds
// widgets; the composing dialog (WingDialog/PropellerDialog) owns the
// actual tab strip and parents these into it.
//
// The Planform tab's content depends on which concrete Planform the model
// was built with (fixed for the model's lifetime -- this port does not
// support switching a wing between Standard/Cranked/NURBS after creation,
// matching how "Add Wing" offers three separate actions). The Standard
// tab shows only the Tip-Chord representation (TrapPlanform) -- the
// Taper-Ratio alternate representation (TrapTaperPlanform) some Python
// panels also offer is a different *input* convention for the same data,
// not a different capability, and is left out of this pass's scope.
class FWingDesignTabs : public TSharedFromThis<FWingDesignTabs>
{
public:
	FWingDesignTabs(aerotelic::WingModel* InModel, FSimpleDelegate InOnChanged);

	const TArray<TPair<FText, TSharedRef<SWidget>>>& GetTabs() const { return Tabs; }

private:
	aerotelic::WingModel* Model = nullptr;
	FSimpleDelegate OnChangedDelegate;
	TArray<TPair<FText, TSharedRef<SWidget>>> Tabs;

	void NotifyChanged();
	TSharedRef<SWidget> BuildPlanformTab();
	TSharedRef<SWidget> BuildStandardPlanformTab(aerotelic::TrapPlanform* Trap);
	TSharedRef<SWidget> BuildCrankedPlanformTab(aerotelic::DoubleTrapPlanform* Dtp);
	TSharedRef<SWidget> BuildNurbsPlanformTab(aerotelic::NURBSPlanform* Nurbs);
	TSharedRef<SWidget> BuildDihedralTab();
	TSharedRef<SWidget> BuildTwistTab();
	TSharedRef<SWidget> BuildSectionsTab();
};
