#pragma once

#include "CoreMinimal.h"
#include "Input/Reply.h"

#include "aerotelic/nurbs.hpp"

class SWidget;
class SCurveEditor;

// The controls every WingDesigner curve tab has along its bottom edge
// (planform_panel.py, dihedral_panel.py, fuselage_panel.py, ...): a NURBS
// degree spin box per curve, "Reset Knots", and a reset-to-something
// button -- plus the shared pieces those tabs are built from (grey info
// text, right-aligned metric read-outs, a Help tab). Shared here so every
// panel's chrome is the same rather than each rolling its own.

// Curve degree (1 = piecewise-linear, 2 = quadratic, 3 = cubic ...), ranged
// 1..max_degree (= number of poles - 1) and re-read every frame, so adding or
// removing poles elsewhere keeps it honest. Commits live, like the Qt
// QSpinBox with keyboard tracking on. A degree the curve refuses (a periodic
// curve whose knot multiplicity forbids it) is ignored, as the Python
// panels' try/except ValueError does.
TSharedRef<SWidget> MakeDegreeSpin(const FText& Label, TFunction<aerotelic::NurbsCurve*()> GetCurve,
	TSharedPtr<SCurveEditor> Editor, FSimpleDelegate OnChanged, FText ToolTip = FText());

TSharedRef<SWidget> MakeControlButton(const FText& Label, const FText& ToolTip, FOnClicked OnClicked);

// Leading widgets left-aligned, a stretch, then the trailing (button) widgets.
TSharedRef<SWidget> MakeControlsRow(const TArray<TSharedRef<SWidget>>& Leading, const TArray<TSharedRef<SWidget>>& Trailing);

// Grey, word-wrapped explanatory text (the Qt panels' "color: #888" labels).
TSharedRef<SWidget> MakeInfoLabel(const FText& Text);

// "Label:" left, value right-aligned (the Qt metric_label rows).
TSharedRef<SWidget> MakeMetricRow(const FText& Label, TAttribute<FText> Value);

// A group box: bold title over content.
TSharedRef<SWidget> MakeGroup(const FText& Title, TSharedRef<SWidget> Content);

// A read-only Help tab: wrapped text in a scroll box.
TSharedRef<SWidget> MakeHelpTab(const FText& HelpText);

// The nacelle panels' "_reset_curve": poles from Spec scaled by Scale,
// weights from Spec or ones, then reset knots -> set degree -> reset knots
// (the first reset is needed because a repeated knot can refuse the degree).
void ResetCurveFromSpec(aerotelic::NurbsCurve& Curve, const aerotelic::NurbsSpec& Spec, FVector2D Scale = FVector2D(1.0, 1.0));

// Two poles (0, Y0)-(1, Y1), unit weights, degree 1, default knots: the
// dihedral panel's "Reset to Flat", the twist panel's "Reset to Initial",
// and (per edge, keeping the current end heights) the NURBS planform's
// "Reset to Straight".
void ResetCurveToLine(aerotelic::NurbsCurve& Curve, double Y0, double Y1);

// Formats as Python's f"{v:.4g}" / f"{v:.3f}" etc.
FText FormatG(double V, int32 Significant = 4);
FText FormatF(double V, int32 Decimals);
// The universal "not applicable" read-out.
FText DashText();
