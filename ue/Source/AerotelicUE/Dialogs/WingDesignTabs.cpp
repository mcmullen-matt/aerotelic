#include "WingDesignTabs.h"
#include "SectionListDialog.h"
#include "Widgets/NumericEntry.h"
#include "Widgets/CurveEditor.h"
#include "Widgets/CurveTabControls.h"

#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/SBoxPanel.h"

#include "Entity/AerotelicDocumentSubsystem.h"

#define LOCTEXT_NAMESPACE "WingDesign"

using namespace aerotelic;

namespace
{
	const FLinearColor kLeColor(0.88f, 0.31f, 0.31f);   // #e05050
	const FLinearColor kTeColor(0.25f, 0.5f, 0.88f);    // #4080e0
	const FLinearColor kChordColor(0.6f, 0.63f, 0.67f); // #9aa0aa
	const FLinearColor kKinkColor(0.94f, 0.63f, 0.19f);

	TSharedRef<SWidget> FormRow(const FText& Label, TSharedRef<SWidget> Field, TAttribute<EVisibility> Visibility = EVisibility::Visible)
	{
		return SNew(SHorizontalBox)
			.Visibility(Visibility)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 2, 8, 2)
			[ SNew(SBox).MinDesiredWidth(110) [ SNew(STextBlock).Text(Label) ] ]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 2)
			[ Field ];
	}

	// A live-committing numeric field (every FloatSpin on these panels
	// commits per keystroke), reading the model each frame.
	TSharedRef<SWidget> NumericRow(const FText& Label, TFunction<double()> Get, double Lo, double Hi, double Delta,
		TFunction<void(double)> Set, TAttribute<EVisibility> Visibility = EVisibility::Visible)
	{
		return FormRow(Label,
			SNew(SAerotelicNumericEntry)
			.Value_Lambda([Get]() { return Get(); })
			.MinValue(Lo).MaxValue(Hi).Delta(Delta)
			.OnValueCommitted_Lambda([Get, Set](double V, ETextCommit::Type) { if (!FMath::IsNearlyEqual(V, Get())) Set(V); }),
			Visibility);
	}

	const FText kNurbsInfo = LOCTEXT("NurbsInfo",
		"Leading and trailing edges are two independent curves in one graph, laid out like the fuselage profile: the leading edge's knot graph above the pole graph, the trailing edge's below.  "
		"Each curve's first pole is held at the root (η = 0) and its last at the tip (η = 1); those move only chordwise.  All other poles move freely.  "
		"Axes are non-dimensional — span fraction y/L across, chordwise x/L up, L the semi-span — and drawn at the same scale both ways, so changing the semi-span rescales the drawing instead of moving the control points.  "
		"The view stays where you put it — wheel to zoom, or right-click for Fit view to contents.  "
		"Middle: drag poles; double-click near a curve to add a pole to it; right-click for weight / remove; click a pole to highlight its blending function.  "
		"Knot graphs: drag the green knots, push one onto a neighbour to repeat it (a corner needs degree-many); right-click the line to insert a knot without changing the shape, or a knot to delete it.");

	const FText kDihedralInfo = LOCTEXT("DihedralInfo",
		"Top: drag control points to shape the dihedral (root and tip stay at their span ends); double-click to add a point; right-click a point to set its weight or delete it; click one to highlight its blending function.  "
		"The view only moves when you move it: wheel to zoom, or right-click for Fit view to contents.  "
		"Bottom: drag the green knots along the line — push one onto a neighbour to repeat it (a crease needs degree-many); right-click the line to insert a knot without changing the shape, or a knot to delete it.");

	const FText kTwistInfo = LOCTEXT("TwistInfo",
		"Top: drag control points to shape the twist distribution (root and tip stay at their span ends); double-click to add a point; right-click a point to set its weight or delete it; click one to highlight its blending function.  "
		"Bottom: drag the green knots along the line — push one onto a neighbour to repeat it (a crease needs degree-many); right-click the line to insert a knot without changing the shape, or a knot to delete it.");
}

FWingDesignTabs::FWingDesignTabs(WingModel* InModel, FSimpleDelegate InOnChanged, bool bInBlade)
	: Model(InModel), OnChangedDelegate(InOnChanged), bBlade(bInBlade)
{
	Tabs.Add({ LOCTEXT("Planform", "Planform"), BuildPlanformTab() });
	Tabs.Add({ LOCTEXT("Dihedral", "Dihedral"), BuildDihedralTab() });
	Tabs.Add({ LOCTEXT("Sections", "Sections"), BuildSectionsTab() });
	Tabs.Add({ LOCTEXT("Twist", "Twist"), BuildTwistTab() });
	if (!bBlade)
	{
		Tabs.Add({ LOCTEXT("ControlSurfaces", "Control Surfaces"), BuildControlSurfacesTab() });
		// A blade's help goes into the propeller panel's own Help tab.
		Tabs.Add({ LOCTEXT("Help", "Help"), MakeHelpTab(GetHelpText()) });
	}
}

FText FWingDesignTabs::GetHelpText() const
{
	// The per-tab descriptions, gathered here rather than shown above each
	// tab's editor so the editing tabs hold only the controls.
	const std::string Mode = Model->planform().Serialize().type;
	FText PlanformHelp;
	if (Mode == "nurbs")
	{
		PlanformHelp = FText::Format(LOCTEXT("HelpPlanformNurbs", "PLANFORM (NURBS)\n{0}"), kNurbsInfo);
	}
	else if (Mode == "double_trap")
	{
		PlanformHelp = LOCTEXT("HelpPlanformCranked",
			"PLANFORM (CRANKED)\nTwo trapezoids joined at the kink: the Planform group sets the semi-span, the kink's span fraction and the root, kink and tip chords; "
			"the Sweep group sets each panel's sweep angle and the chord fraction the sweep is measured at (0 = leading edge, 1 = trailing edge). "
			"The outline below follows the inputs; the derived metrics are the aspect ratio, overall taper (tip over root) and wetted area.");
	}
	else
	{
		PlanformHelp = LOCTEXT("HelpPlanformStandard",
			"PLANFORM (STANDARD)\nA single trapezoid. Type picks whether the tip is given as a chord (Ct/Cr) or as a taper ratio λ; switching keeps the same wing. "
			"Semi-span and root chord set the size, the sweep angle is measured at the chord fraction given (0 = leading edge, 1 = trailing edge). "
			"The outline below follows the inputs; the derived metrics are the aspect ratio, taper ratio and wetted area.");
	}
	const FText Dihedral = FText::Format(LOCTEXT("HelpDihedral", "DIHEDRAL\n{0}"), kDihedralInfo);
	const FText Twist = FText::Format(LOCTEXT("HelpTwist", "TWIST\n{0}"), kTwistInfo);
	const FText Sections = LOCTEXT("HelpSections",
		"SECTIONS\nAirfoil sections at span stations s (0 = root, 1 = tip), blended between. Add at s inserts a station; pick each station's airfoil from the project's airfoils; Remove deletes the row.");
	const FText Axes = LOCTEXT("HelpAxes",
		"AXES\nEvery editor is non-dimensional: span fraction y/L across, with L the semi-span; chordwise x/L and height z/L up at the same scale; twist is in degrees.");
	return FText::Join(FText::FromString(TEXT("\n\n")), PlanformHelp, Dihedral, Twist, Sections, Axes);
}

void FWingDesignTabs::NotifyChanged()
{
	if (OutlineView.IsValid()) OutlineView->RequestRedraw();
	OnChangedDelegate.ExecuteIfBound();
}

void FWingDesignTabs::SpanChanged()
{
	if (PlanformEditor.IsValid())
	{
		PlanformEditor->SetScales(1.0, 1.0 / FMath::Max(Model->planform().SemiSpan(), 1e-9));
	}
	if (OutlineView.IsValid())
	{
		OutlineView->SetScales(1.0, 1.0 / FMath::Max(Model->planform().SemiSpan(), 1e-9));
		OutlineView->RequestRedraw();
	}
}

TSharedRef<SWidget> FWingDesignTabs::BuildPlanformTab()
{
	Planform& Pf = Model->planform();
	const std::string Mode = Pf.Serialize().type;
	if (Mode == "double_trap")
	{
		return BuildCrankedPlanformTab(static_cast<DoubleTrapPlanform*>(&Pf));
	}
	if (Mode == "nurbs")
	{
		return BuildNurbsPlanformTab(static_cast<NURBSPlanform*>(&Pf));
	}
	return BuildStandardPlanformTab(static_cast<TrapPlanform*>(&Pf));
}

// The read-only outline graph under the Standard/Cranked inputs
// (planform_outline.py's PlanformOutlineView): LE, TE, root and tip chords,
// and the kink chord on a cranked wing.
TSharedRef<SWidget> FWingDesignTabs::BuildOutlineView(const FText& Title)
{
	WingModel* M = Model;
	TSharedRef<SCurveEditor> View = SNew(SCurveEditor)
		.XLabel(LOCTEXT("SpanLabel", "Span fraction (y/L)"))
		.YLabel(LOCTEXT("ChordLabel", "Chordwise (x/L)"))
		.Title(Title)
		.XScale(1.0).YScale(1.0 / FMath::Max(M->planform().SemiSpan(), 1e-9)).EqualAspect(true);
	View->SetSlots({});
	View->SetOutlineProvider(FOnGetOutlinePaths::CreateLambda([M]()
	{
		TArray<FCurveEditorOutlinePath> Paths;
		try
		{
			const Planform& P = M->planform();
			Mat O = P.PlanformOutline(P.IsDoubleTrap() ? 3 : 2);
			if (O.rows() < 2 || O.cols() < 3) return Paths;
			FCurveEditorOutlinePath Le, Te;
			for (int i = 0; i < O.rows(); ++i)
			{
				Le.Points.Add(FVector2D(O(i, 0), O(i, 1)));
				Te.Points.Add(FVector2D(O(i, 0), O(i, 2)));
			}
			Le.Color = kLeColor; Le.Label = LOCTEXT("LeadingEdge", "Leading edge"); Le.LineWidth = 1.8f;
			Te.Color = kTeColor; Te.Label = LOCTEXT("TrailingEdge", "Trailing edge"); Te.LineWidth = 1.8f;
			Paths.Add(Le);
			Paths.Add(Te);
			// Root and tip chords.
			for (int i : { 0, O.rows() - 1 })
			{
				FCurveEditorOutlinePath Chord;
				Chord.Points = { FVector2D(O(i, 0), O(i, 1)), FVector2D(O(i, 0), O(i, 2)) };
				Chord.Color = kChordColor; Chord.LineWidth = 1.4f;
				Paths.Add(Chord);
			}
			// Kink chord (cranked wing): the interior station of the outline.
			if (P.IsDoubleTrap() && O.rows() >= 3)
			{
				const int i = 1;
				FCurveEditorOutlinePath Kink;
				Kink.Points = { FVector2D(O(i, 0), O(i, 1)), FVector2D(O(i, 0), O(i, 2)) };
				Kink.Color = kKinkColor; Kink.LineWidth = 1.2f;
				Kink.Label = FText::FromString(FString::Printf(TEXT("Kink (η = %.3f)"), P.KinkFraction()));
				Paths.Add(Kink);
			}
		}
		catch (const std::exception&) {}
		return Paths;
	}));
	OutlineView = View;
	return SNew(SBox).MinDesiredHeight(220) [ View ];
}

TSharedRef<SWidget> FWingDesignTabs::BuildDerivedMetrics(const FText& TaperLabel)
{
	WingModel* M = Model;
	auto Metric = [M](TFunction<double(const Planform&)> Get, int32 Decimals)
	{
		return TAttribute<FText>::CreateLambda([M, Get, Decimals]()
		{
			try { return FormatF(Get(M->planform()), Decimals); }
			catch (const std::exception&) { return DashText(); }
		});
	};
	return MakeGroup(LOCTEXT("DerivedMetrics", "Derived Metrics"),
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("AspectRatio", "Aspect Ratio:"), Metric([](const Planform& P) { return P.AspectRatio(); }, 3)) ]
		+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(TaperLabel, Metric([](const Planform& P) { return P.TaperRatio(); }, 4)) ]
		+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("WettedArea", "Wetted Area:"), Metric([](const Planform& P) { return 2.0 * P.PlanformArea(); }, 3)) ]);
}

TSharedRef<SWidget> FWingDesignTabs::BuildStandardPlanformTab(TrapPlanform* Trap)
{
	// "Type:" -- the Tip Chord (Ct/Cr) or Taper Ratio (lambda) input
	// convention for the same data (planform_panel.py's sub-mode combo).
	struct FTypeState { int32 Mode = 0; TArray<TSharedPtr<FString>> Options; };
	TSharedRef<FTypeState> State = MakeShared<FTypeState>();
	State->Options.Add(MakeShared<FString>(TEXT("Tip Chord  (Ct/Cr)")));
	State->Options.Add(MakeShared<FString>(TEXT("Taper Ratio  (λ)")));
	auto TipVisible = TAttribute<EVisibility>::CreateLambda([State]() { return State->Mode == 0 ? EVisibility::Visible : EVisibility::Collapsed; });
	auto TaperVisible = TAttribute<EVisibility>::CreateLambda([State]() { return State->Mode == 1 ? EVisibility::Visible : EVisibility::Collapsed; });

	TSharedRef<SWidget> Inputs = SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[ FormRow(LOCTEXT("Type", "Type:"),
			SNew(SBox).MinDesiredWidth(160)
			[
				SNew(SComboBox<TSharedPtr<FString>>)
				.OptionsSource(&State->Options)
				.InitiallySelectedItem(State->Options[0])
				.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
				.OnSelectionChanged_Lambda([State](TSharedPtr<FString> S, ESelectInfo::Type)
				{
					if (S.IsValid()) State->Mode = State->Options.IndexOfByKey(S);
				})
				[ SNew(STextBlock).Text_Lambda([State]() { return FText::FromString(*State->Options[FMath::Clamp(State->Mode, 0, 1)]); }) ]
			]) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(LOCTEXT("SemiSpan", "Semi-Span:"), [Trap]() { return Trap->SemiSpan(); }, 1e-9, 1e9, 0.25,
			[this, Trap](double V) { Trap->SetSemiSpan(V); SpanChanged(); NotifyChanged(); }) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(LOCTEXT("RootChord", "Root Chord:"), [Trap]() { return Trap->RootChord(); }, 1e-9, 1e9, 0.1,
			[this, Trap, State](double V)
			{
				// In taper mode the tip chord follows the root chord.
				const double Taper = Trap->TaperRatio();
				Trap->SetRootChord(V);
				if (State->Mode == 1) Trap->SetTipChord(V * Taper);
				NotifyChanged();
			}) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(LOCTEXT("TipChord", "Tip Chord:"), [Trap]() { return Trap->TipChord(); }, 1e-9, 1e9, 0.05,
			[this, Trap](double V) { Trap->SetTipChord(V); NotifyChanged(); }, TipVisible) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(LOCTEXT("TaperRatio", "Taper Ratio λ:"), [Trap]() { return Trap->TaperRatio(); }, 1e-9, 1e9, 0.01,
			[this, Trap](double V) { Trap->SetTipChord(Trap->RootChord() * V); NotifyChanged(); }, TaperVisible) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(LOCTEXT("SweepAngle", "Sweep Angle (°):"), [Trap]() { return Trap->sweep_angle(); }, -89.0, 89.0, 0.5,
			[this, Trap](double V) { Trap->SetSweepAngle(V); NotifyChanged(); }) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(LOCTEXT("SweepFraction", "Sweep at chord (0=LE · 1=TE):"), [Trap]() { return Trap->sweep_fraction(); }, 0.0, 1.0, 0.05,
			[this, Trap](double V) { Trap->SetSweepFraction(V); NotifyChanged(); }) ];

	return SNew(SScrollBox)
		+ SScrollBox::Slot().Padding(8)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight() [ MakeGroup(LOCTEXT("TrapGroup", "Trapezoidal Planform"), Inputs) ]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 8)
			[ BuildOutlineView(LOCTEXT("TrapOutlineTitle", "Trapezoidal planform  ·  outline follows the inputs above")) ]
			+ SVerticalBox::Slot().AutoHeight() [ BuildDerivedMetrics(LOCTEXT("TaperRatioMetric", "Taper Ratio:")) ]
		];
}

TSharedRef<SWidget> FWingDesignTabs::BuildCrankedPlanformTab(DoubleTrapPlanform* Dtp)
{
	TSharedRef<SWidget> PlanformInputs = SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(LOCTEXT("SemiSpan", "Semi-Span:"), [Dtp]() { return Dtp->SemiSpan(); }, 1e-9, 1e9, 0.25,
			[this, Dtp](double V) { Dtp->SetSemiSpan(V); SpanChanged(); NotifyChanged(); }) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(LOCTEXT("RootChord", "Root Chord:"), [Dtp]() { return Dtp->RootChord(); }, 1e-9, 1e9, 0.1,
			[this, Dtp](double V) { Dtp->SetRootChord(V); NotifyChanged(); }) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(LOCTEXT("TipChord", "Tip Chord:"), [Dtp]() { return Dtp->TipChord(); }, 1e-9, 1e9, 0.05,
			[this, Dtp](double V) { Dtp->SetTipChord(V); NotifyChanged(); }) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(LOCTEXT("KinkFraction", "Kink Position (η, 0=root · 1=tip):"), [Dtp]() { return Dtp->KinkFraction(); }, 0.05, 0.95, 0.05,
			[this, Dtp](double V) { Dtp->SetKinkFraction(V); NotifyChanged(); }) ];

	TSharedRef<SWidget> SweepInputs = SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(LOCTEXT("LeSweep", "Leading Edge Sweep (°):"), [Dtp]() { return Dtp->LeadingEdgeSweepAngle(); }, -89.0, 89.0, 0.5,
			[this, Dtp](double V) { Dtp->SetLeadingEdgeSweepAngle(V); NotifyChanged(); }) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(LOCTEXT("TeSweep", "Trailing Edge Sweep (°):"), [Dtp]() { return Dtp->TrailingEdgeSweepAngle(); }, -89.0, 89.0, 0.5,
			[this, Dtp](double V) { Dtp->SetTrailingEdgeSweepAngle(V); NotifyChanged(); }) ];

	return SNew(SScrollBox)
		+ SScrollBox::Slot().Padding(8)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight() [ MakeGroup(LOCTEXT("PlanformGroup", "Planform"), PlanformInputs) ]
			+ SVerticalBox::Slot().AutoHeight() [ MakeGroup(LOCTEXT("SweepGroup", "Sweep"), SweepInputs) ]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 8)
			[ BuildOutlineView(LOCTEXT("CrankedOutlineTitle", "Cranked planform  ·  outline follows the inputs above  ·  kink chord is derived")) ]
			+ SVerticalBox::Slot().AutoHeight() [ BuildDerivedMetrics(LOCTEXT("OverallTaper", "Overall Taper:")) ]
		];
}

TSharedRef<SWidget> FWingDesignTabs::BuildNurbsPlanformTab(NURBSPlanform* Nurbs)
{
	FSimpleDelegate OnChanged = FSimpleDelegate::CreateLambda([this]() { NotifyChanged(); });

	TSharedRef<SCurveEditor> Editor = SNew(SCurveEditor)
		.XLabel(LOCTEXT("SpanLabel", "Span fraction (y/L)"))
		.YLabel(LOCTEXT("ChordLabel", "Chordwise (x/L)"))
		.Title(LOCTEXT("PlanformTitle", "Planform  ·  leading and trailing edge  ·  root and tip poles pinned in η"))
		.XScale(1.0)
		.YScale(1.0 / FMath::Max(Nurbs->SemiSpan(), 1e-9))
		.EqualAspect(true)
		.SupportsUserLinks(true)
		.LeadingName(LOCTEXT("Root", "root")).TrailingName(LOCTEXT("Tip", "tip"))
		.OnChanged(OnChanged);

	FCurveEditorSlot LeSlot, TeSlot;
	LeSlot.Curve = &Nurbs->le_curve_mut();
	LeSlot.Color = kLeColor;
	LeSlot.Label = LOCTEXT("LE", "LE");
	LeSlot.Basis = ECurveBasisPlacement::Above;
	TeSlot.Curve = &Nurbs->te_curve_mut();
	TeSlot.Color = kTeColor;
	TeSlot.Label = LOCTEXT("TE", "TE");
	TeSlot.Basis = ECurveBasisPlacement::Below;
	Editor->SetSlots({ LeSlot, TeSlot });
	PlanformEditor = Editor;

	TSharedRef<SWidget> Controls = MakeControlsRow(
		{
			MakeDegreeSpin(LOCTEXT("LeDeg", "LE deg:"), [Nurbs]() { return &Nurbs->le_curve_mut(); }, Editor, OnChanged, LOCTEXT("LeDegTip", "NURBS degree for leading edge")),
			MakeDegreeSpin(LOCTEXT("TeDeg", "TE deg:"), [Nurbs]() { return &Nurbs->te_curve_mut(); }, Editor, OnChanged, LOCTEXT("TeDegTip", "NURBS degree for trailing edge")),
		},
		{
			MakeControlButton(LOCTEXT("ResetKnots", "Reset Knots"), LOCTEXT("ResetKnotsEdgesTip", "Put both edges' knots back at their default spacing (poles unchanged)"),
				FOnClicked::CreateLambda([Nurbs, Editor, OnChanged]()
				{
					Nurbs->le_curve_mut().ResetKnotsPublic();
					Nurbs->te_curve_mut().ResetKnotsPublic();
					Editor->RequestRedraw();
					OnChanged.ExecuteIfBound();
					return FReply::Handled();
				})),
			MakeControlButton(LOCTEXT("ResetStraight", "Reset to Straight"), LOCTEXT("ResetStraightTip", "Straight leading and trailing edges between their current root and tip poles"),
				FOnClicked::CreateLambda([Nurbs, Editor, OnChanged]()
				{
					for (NurbsCurve* C : { &Nurbs->le_curve_mut(), &Nurbs->te_curve_mut() })
					{
						const Mat P = C->poles();
						ResetCurveToLine(*C, P(0, 1), P(P.rows() - 1, 1));
					}
					Editor->RequestRedraw();
					OnChanged.ExecuteIfBound();
					return FReply::Handled();
				})),
		});

	TSharedRef<SVerticalBox> Tab = SNew(SVerticalBox)
		+ SVerticalBox::Slot().FillHeight(1.0f).Padding(4, 2) [ Editor ]
		+ SVerticalBox::Slot().AutoHeight().Padding(8, 2) [ Controls ];

	if (!bBlade)
	{
		Tab->AddSlot().AutoHeight().Padding(8, 2)
		[ NumericRow(LOCTEXT("SemiSpan", "Semi-Span:"), [Nurbs]() { return Nurbs->SemiSpan(); }, 1e-9, 1e9, 0.25,
			[this, Nurbs](double V) { Nurbs->SetSemiSpan(V); SpanChanged(); NotifyChanged(); }) ];
		Tab->AddSlot().AutoHeight().Padding(8, 2)
		[ BuildDerivedMetrics(LOCTEXT("TaperRatioMetric", "Taper Ratio:")) ];
	}
	return Tab;
}

TSharedRef<SWidget> FWingDesignTabs::BuildDihedralTab()
{
	FSimpleDelegate OnChanged = FSimpleDelegate::CreateLambda([this]() { NotifyChanged(); });
	TSharedRef<SCurveEditor> Editor = SNew(SCurveEditor)
		.XLabel(LOCTEXT("SpanLabel", "Span fraction (y/L)"))
		.YLabel(LOCTEXT("HeightLabel", "Height (z/L)"))
		.Title(LOCTEXT("DihedralTitle", "Dihedral NURBS  ·  drag poles  ·  dbl-click to add  ·  right-click for options"))
		.XScale(1.0).YScale(1.0).EqualAspect(true)
		.OnChanged(OnChanged);
	FCurveEditorSlot Slot;
	Slot.Curve = &Model->planform().DihedralCurveMut();
	Slot.Color = FLinearColor(0.75f, 0.63f, 0.19f);
	Slot.Basis = ECurveBasisPlacement::Below;
	Editor->SetSlots({ Slot });
	WingModel* M = Model;

	return SNew(SVerticalBox)
		+ SVerticalBox::Slot().FillHeight(1.0f).Padding(4, 2) [ Editor ]
		+ SVerticalBox::Slot().AutoHeight().Padding(8, 2)
		[
			MakeControlsRow(
				{ MakeDegreeSpin(LOCTEXT("NurbsDegree", "NURBS Degree:"), [M]() { return &M->planform().DihedralCurveMut(); }, Editor, OnChanged) },
				{
					MakeControlButton(LOCTEXT("ResetKnots", "Reset Knots"), LOCTEXT("ResetKnotsTip", "Put the knots back at their default spacing (poles unchanged)"),
						FOnClicked::CreateLambda([M, Editor, OnChanged]()
						{
							M->planform().DihedralCurveMut().ResetKnotsPublic();
							Editor->RequestRedraw();
							OnChanged.ExecuteIfBound();
							return FReply::Handled();
						})),
					MakeControlButton(LOCTEXT("ResetFlat", "Reset to Flat"), FText(),
						FOnClicked::CreateLambda([M, Editor, OnChanged]()
						{
							ResetCurveToLine(M->planform().DihedralCurveMut(), 0.0, 0.0);
							Editor->RequestRedraw();
							OnChanged.ExecuteIfBound();
							return FReply::Handled();
						})),
				})
		];
}

TSharedRef<SWidget> FWingDesignTabs::BuildTwistTab()
{
	FSimpleDelegate OnChanged = FSimpleDelegate::CreateLambda([this]() { NotifyChanged(); });
	TSharedRef<SCurveEditor> Editor = SNew(SCurveEditor)
		.XLabel(LOCTEXT("SpanLabel", "Span fraction (y/L)"))
		.YLabel(LOCTEXT("TwistLabel", "Twist (degrees)"))
		.Title(LOCTEXT("TwistTitle", "Twist NURBS  ·  drag poles  ·  dbl-click to add  ·  right-click for options"))
		.XScale(1.0).YScale(1.0).EqualAspect(false)
		.OnChanged(OnChanged);
	FCurveEditorSlot Slot;
	Slot.Curve = &Model->planform().TwistCurveMut();
	Slot.Color = FLinearColor(0.44f, 0.75f, 0.44f);
	Slot.Basis = ECurveBasisPlacement::Below;
	Editor->SetSlots({ Slot });
	WingModel* M = Model;

	return SNew(SVerticalBox)
		+ SVerticalBox::Slot().FillHeight(1.0f).Padding(4, 2) [ Editor ]
		+ SVerticalBox::Slot().AutoHeight().Padding(8, 2)
		[
			MakeControlsRow(
				{ MakeDegreeSpin(LOCTEXT("NurbsDegree", "NURBS Degree:"), [M]() { return &M->planform().TwistCurveMut(); }, Editor, OnChanged) },
				{
					MakeControlButton(LOCTEXT("ResetKnots", "Reset Knots"), LOCTEXT("ResetKnotsTip", "Put the knots back at their default spacing (poles unchanged)"),
						FOnClicked::CreateLambda([M, Editor, OnChanged]()
						{
							M->planform().TwistCurveMut().ResetKnotsPublic();
							Editor->RequestRedraw();
							OnChanged.ExecuteIfBound();
							return FReply::Handled();
						})),
					MakeControlButton(LOCTEXT("ResetInitial", "Reset to Initial"), FText(),
						FOnClicked::CreateLambda([M, Editor, OnChanged]()
						{
							// 0 degrees across the entire span (the default twist curve).
							ResetCurveToLine(M->planform().TwistCurveMut(), 0.0, 0.0);
							Editor->RequestRedraw();
							OnChanged.ExecuteIfBound();
							return FReply::Handled();
						})),
				})
		];
}

TSharedRef<SWidget> FWingDesignTabs::BuildSectionsTab()
{
	TAttribute<TArray<std::shared_ptr<Airfoil>>> ProjectAirfoilsAttr = TAttribute<TArray<std::shared_ptr<Airfoil>>>::CreateLambda([]()
	{
		if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
		{
			return Doc->GatherProjectAirfoils();
		}
		return TArray<std::shared_ptr<Airfoil>>();
	});
	return SNew(SSectionListWidget, Model)
		.ProjectAirfoils(ProjectAirfoilsAttr)
		.OnChanged(FSimpleDelegate::CreateLambda([this]() { NotifyChanged(); }));
}

TSharedRef<SWidget> FWingDesignTabs::BuildControlSurfacesTab()
{
	// The geometry backend of this port does not carry control surfaces yet
	// (see wing_model.hpp's file header), so the tab is a placeholder.
	return SNew(SBorder).Padding(12)
	[
		MakeInfoLabel(LOCTEXT("ControlSurfacesPlaceholder",
			"Control surfaces are not available yet in this version.\n\n"
			"In WingDesigner this tab places a hinge line in plan view (two handles setting the span station and chord fraction at each end), "
			"with the hinge gap, edge gap, nose fill, deflection and deflection limit of the selected surface. "
			"It will appear here once the geometry backend carries control surfaces."))
	];
}

#undef LOCTEXT_NAMESPACE
