#pragma once

#include "CoreMinimal.h"
#include "Widgets/SWidget.h"

#include "aerotelic/wing_model.hpp"

class SCurveEditor;

// Builds the Planform / Dihedral / Sections / Twist (/ Control Surfaces)
// tab contents every wing-shaped entity edits -- the Slate port of
// WingDesigner's TrapPlanformPanel / DoubleTrapPlanformPanel /
// NURBSPlanformPanel with their embedded DihedralPanel / SectionPanel /
// TwistPanel -- factored out so the Propeller panel can insert its own
// Propeller/Spinner/Help tabs around the same four (propeller_panel.py's
// structural trick of inserting into NURBSPlanformPanel's own QTabWidget).
// Not a widget itself -- just builds widgets; the composing panel owns the
// tab strip.
//
// The Planform tab's content depends on which concrete Planform the model
// was built with (fixed for the model's lifetime -- a wing is not switched
// between Standard/Cranked/NURBS after creation, matching how "Add Wing"
// offers three separate actions). The Standard tab offers the Qt panel's
// Type switch between the Tip Chord and Taper Ratio input conventions.
//
// bBlade: a propeller blade (NURBSPlanformPanel(blade=True)): no Semi-Span
// row (the blade's span is the propeller's radius R, typed on the Propeller
// tab), no Derived Metrics, no Control Surfaces tab.
class FWingDesignTabs : public TSharedFromThis<FWingDesignTabs>
{
public:
	FWingDesignTabs(aerotelic::WingModel* InModel, FSimpleDelegate InOnChanged, bool bInBlade = false);

	const TArray<TPair<FText, TSharedRef<SWidget>>>& GetTabs() const { return Tabs; }

	// After the semi-span changed from outside (the propeller's radius):
	// rescales the planform editor's chordwise axis and redraws the outline,
	// as NURBSPlanformPanel.span_changed() does.
	void SpanChanged();

private:
	aerotelic::WingModel* Model = nullptr;
	FSimpleDelegate OnChangedDelegate;
	bool bBlade = false;
	TArray<TPair<FText, TSharedRef<SWidget>>> Tabs;

	TSharedPtr<SCurveEditor> PlanformEditor;  // NURBS planform: the editable LE/TE editor
	TSharedPtr<SCurveEditor> OutlineView;     // Standard/Cranked: the read-only outline graph

	void NotifyChanged();
	TSharedRef<SWidget> BuildPlanformTab();
	TSharedRef<SWidget> BuildStandardPlanformTab(aerotelic::TrapPlanform* Trap);
	TSharedRef<SWidget> BuildCrankedPlanformTab(aerotelic::DoubleTrapPlanform* Dtp);
	TSharedRef<SWidget> BuildNurbsPlanformTab(aerotelic::NURBSPlanform* Nurbs);
	TSharedRef<SWidget> BuildOutlineView(const FText& Title);
	TSharedRef<SWidget> BuildDerivedMetrics(const FText& TaperLabel);
	TSharedRef<SWidget> BuildDihedralTab();
	TSharedRef<SWidget> BuildTwistTab();
	TSharedRef<SWidget> BuildSectionsTab();
	TSharedRef<SWidget> BuildControlSurfacesTab();
};
