#include "PropellerDialog.h"
#include "WingDesignTabs.h"
#include "EntityDesignContent.h"
#include "Widgets/TabStrip.h"
#include "Widgets/NumericEntry.h"
#include "Widgets/CurveEditor.h"
#include "Widgets/CurveTabControls.h"
#include "Widgets/ReferenceImageBar.h"
#include "Widgets/ReferenceImageMath.h"

#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SSpinBox.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SComboBox.h"

#include "Entity/AerotelicEntityActor.h"
#include "aerotelic/propeller_model.hpp"

#define LOCTEXT_NAMESPACE "PropellerDialog"

using namespace aerotelic;

namespace
{
	TSharedRef<SWidget> FormRow(const FText& Label, TSharedRef<SWidget> Field, const FText& ToolTip = FText())
	{
		return SNew(SHorizontalBox)
			.ToolTipText(ToolTip)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 2, 8, 2)
			[ SNew(SBox).MinDesiredWidth(150) [ SNew(STextBlock).Text(Label) ] ]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 2)
			[ Field ];
	}

	// The Propeller tab's derived read-outs, from one Metrics() call; any
	// exception blanks every label.
	struct FDerived
	{
		TOptional<PropellerMetrics> M;
		void Refresh(PropellerModel* Model)
		{
			try { M = Model->Metrics(); }
			catch (const std::exception&) { M.Reset(); }
		}
	};

	const FText kPropellerNote = LOCTEXT("PropellerNote",
		"R is the length scale: the blade's span fraction is r/R, its planform is drawn in x/R, and the spinner in x/R and r/R. "
		"Changing R rescales the whole propeller. Spin direction sets which way it turns — positive is clockwise seen from behind. See Help.");

	const FText kSpinnerInfo = LOCTEXT("SpinnerInfo",
		"The spinner's radius along its length, revolved about the axis. Both ends stay on the axis (r = 0) — the tip at the front, the closure at the back — and move along it only with Spinner length. "
		"Drag the other points freely; double-click to add one, right-click for weight or to remove. The dashed line is the blades' axial position. Axes are x/R and r/R at equal scale.");

	const FText kHelp = LOCTEXT("Help",
		"ONE LENGTH\n"
		"The tip radius R is the only length typed in; every other input and every editor axis is a fraction of it, so changing R rescales the whole propeller and moves no control point.\n\n"
		"THE BLADE\n"
		"Planform, Dihedral, Sections and Twist are the NURBS wing's tabs editing one blade whose semi-span is R, so span fraction is r/R and the planform is drawn in x/R. "
		"The blade is drawn from the axis out, but only the part beyond the blade root radius is built, and that radius should stay inside the spinner. "
		"Planform: leading and trailing edges in the plane of rotation, the pitch-change axis at x = 0; moving both edges together skews the blade. "
		"Dihedral: the blade's rake, forward positive, aft negative, in units of R. Sections: airfoils at r/R stations, blended between. "
		"Twist: the blade angle β from the plane of rotation; for a constant geometric pitch P, tan β = (P/D) / (π r/R). "
		"The suction side faces forward.\n\n"
		"SPIN DIRECTION\n"
		"Which way the propeller turns, by the right-hand rule with the thumb along the shaft pointing from the propeller towards the spinner — that is, forward, the spinner's tip being the front of the hub. "
		"Positive, the default, is clockwise seen from behind: the usual right-hand tractor propeller, and the only hand this model had before the setting existed, so older projects open turning as they did. "
		"Negative is the mirror image. The blade is reflected in the plane of the axis and the span, which reverses its camber and twist, so it still leads with the leading edge and still thrusts forward while turning the other way. "
		"Only the blades change — the spinner is a body of revolution — and the blade angle β and every derived figure read the same either way, the hand being in the geometry rather than in the sign of β. "
		"Setting it is independent of mirroring the entity: a mirrored placement also reverses the hand, so a mirrored negative-spin propeller turns the same way as an unmirrored positive one.\n\n"
		"THE SPINNER\n"
		"One curve, radius against length, revolved about the axis. Both ends are held on the axis so the body is closed; they move along the axis only via Spinner length. "
		"Every other point moves freely as long as it stays off the axis. The dashed line marks the blades' axial position.\n"
		"The spinner is optional: clear Build the spinner on the Propeller tab and only the blades are built — a bare hub, or a rotor whose hub is modelled by other entities. "
		"The profile is kept while it is off, so switching it back on restores the shape unchanged, and the Spinner tab stays editable either way. "
		"With it off the body list starts at Blade 1, the blade root radius is no longer required to sit inside the spinner (the roots are meant to be bare), and the derived spinner figures read none.\n\n"
		"CURVE EDITORS\n"
		"Drag to move, double-click to add, right-click for weight or remove. In the knot graph drag the green knots, push one onto a neighbour to repeat it, right-click to insert or delete. "
		"The wheel zooms; the right-click menu's Fit view to contents refits.\n\n"
		"SOLID\n"
		"While editing, the spinner and blades are drawn as separate overlapping bodies, which is quick; export joins them into one solid. "
		"Without a spinner the blades only meet at the hub, so a fused export may come back as several solids rather than one.");
}

void BuildPropellerDesignContent(AAerotelicEntityActor* Entity, FAerotelicDesignContent& Out)
{
	if (Entity == nullptr || Entity->GetEntityType() != project_yaml::EntityType::Propeller)
	{
		return;
	}

	PropellerModel* Model = Entity->GetModelAs<PropellerModel>();
	TWeakObjectPtr<AAerotelicEntityActor> WeakEntity = Entity;
	FSimpleDelegate Rebuild = FSimpleDelegate::CreateLambda([WeakEntity]()
	{
		if (WeakEntity.IsValid()) WeakEntity->RefreshProxy();
	});

	TSharedRef<FDerived> Derived = MakeShared<FDerived>();
	Derived->Refresh(Model);

	// ---- Spinner editor (built first: the layout router redraws it) ----
	FCurveEditorSlot SpinnerSlot;
	SpinnerSlot.Curve = &Model->spinner_curve_mut();
	SpinnerSlot.Color = FLinearColor(0.75f, 0.78f, 0.85f);
	SpinnerSlot.Label = LOCTEXT("Spinner", "spinner");
	SpinnerSlot.YClampLo = 0.0;
	SpinnerSlot.FixedPoles.Add(0, FVector2D(0.0, 0.0));
	SpinnerSlot.FixedPoles.Add(-1, FVector2D(1.0, 0.0));
	SpinnerSlot.Basis = ECurveBasisPlacement::Below;
	TSharedRef<SCurveEditor> SpinnerEditor = SNew(SCurveEditor)
		.XLabel(LOCTEXT("XR", "Axial (x/R)"))
		.YLabel(LOCTEXT("RadiusR", "Radius (r/R)"))
		.Title(LOCTEXT("SpinnerTitle", "Spinner  ·  revolved about the axis  ·  both ends on the axis"))
		.DefaultYMax(0.25)
		.XScale(FMath::Max(Model->spinner_length, 1e-9)).YScale(1.0).EqualAspect(true);
	SpinnerEditor->SetSlots({ SpinnerSlot });
	// The blades' axial position, dashed.
	SpinnerEditor->SetOutlineProvider(FOnGetOutlinePaths::CreateLambda([Model]()
	{
		TArray<FCurveEditorOutlinePath> Paths;
		if (Model->spinner_length <= 0.0) return Paths;
		const double X = Model->blade_x / Model->spinner_length;
		double Top = Model->hub_radius + 0.05;
		try { Top = FMath::Max(1.4 * Model->SpinnerRadius(), Top); } catch (const std::exception&) {}
		FCurveEditorOutlinePath P;
		P.Points = { FVector2D(X, Model->hub_radius), FVector2D(X, Top) };
		P.Color = FLinearColor(0.88f, 0.31f, 0.31f);
		P.Label = LOCTEXT("BladeAxis", "blade axis");
		P.LineWidth = 1.2f;
		Paths.Add(P);
		return Paths;
	}));

	// ---- Change routers (propeller_panel.py's _blade_changed / radius_changed / layout_changed / spinner_changed) ----
	FSimpleDelegate OnBladeChanged = FSimpleDelegate::CreateLambda([Derived, Model, Rebuild]()
	{
		Derived->Refresh(Model);
		Rebuild.ExecuteIfBound();
	});
	FSimpleDelegate OnLayoutChanged = FSimpleDelegate::CreateLambda([Derived, Model, SpinnerEditor, Rebuild]()
	{
		Derived->Refresh(Model);
		SpinnerEditor->RequestRedraw(); // the blade-axis marker moves
		Rebuild.ExecuteIfBound();
	});
	FSimpleDelegate OnSpinnerChanged = OnBladeChanged;
	SpinnerEditor->SetOnChanged(OnSpinnerChanged);

	TSharedRef<FWingDesignTabs> BladeTabs = MakeShared<FWingDesignTabs>(&Model->blade(), OnBladeChanged, /*bBlade*/ true);
	FSimpleDelegate OnRadiusChanged = FSimpleDelegate::CreateLambda([Derived, Model, BladeTabs, Rebuild]()
	{
		BladeTabs->SpanChanged();
		Derived->Refresh(Model);
		Rebuild.ExecuteIfBound();
	});

	// ---- Propeller tab ----
	auto Metric = [Derived](TFunction<FText(const PropellerMetrics&)> Get)
	{
		return TAttribute<FText>::CreateLambda([Derived, Get]() { return Derived->M.IsSet() ? Get(Derived->M.GetValue()) : DashText(); });
	};
	// The same, for the two read-outs that describe a body which may not
	// be built at all.
	auto SpinnerMetric = [Derived, Model](TFunction<FText(const PropellerMetrics&)> Get)
	{
		return TAttribute<FText>::CreateLambda([Derived, Model, Get]()
		{
			if (!Model->spinner) return LOCTEXT("SpinnerNone", "none");
			return Derived->M.IsSet() ? Get(Derived->M.GetValue()) : DashText();
		});
	};

	// Spin direction: the option strings have to outlive this function (the
	// panel is a persistent tab, not a modal), so they ride a shared state
	// the combo's own lambdas keep alive -- WingDesignTabs' planform-type
	// combo does the same.
	struct FSpinState { TArray<TSharedPtr<FString>> Options; };
	TSharedRef<FSpinState> SpinState = MakeShared<FSpinState>();
	SpinState->Options.Add(MakeShared<FString>(TEXT("Positive")));
	SpinState->Options.Add(MakeShared<FString>(TEXT("Negative")));
	auto SpinItem = [SpinState](SpinDirection D) { return SpinState->Options[D == SpinDirection::Negative ? 1 : 0]; };

	TSharedRef<SWidget> PropellerInputs = SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[ FormRow(LOCTEXT("Radius", "Radius R:"),
			SNew(SAerotelicNumericEntry).Value_Lambda([Model]() { return Model->radius; })
				.MinValue(1e-9).MaxValue(1e9).Delta(0.01).LiveUpdate(false)
				.OnValueCommitted_Lambda([Model, OnRadiusChanged](double V, ETextCommit::Type)
				{ if (!FMath::IsNearlyEqual(V, Model->radius)) { Model->SetRadius(V); OnRadiusChanged.ExecuteIfBound(); } }),
			LOCTEXT("RadiusTip", "Tip radius R — the length scale: every other input and every editor\naxis on this panel is a fraction of it.")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ FormRow(LOCTEXT("BladeCount", "Number of blades:"),
			SNew(SBox).WidthOverride(90)
			[
				SNew(SSpinBox<int32>)
				.MinValue(1).MaxValue(kPropellerMaxBlades).MinSliderValue(1).MaxSliderValue(kPropellerMaxBlades).Delta(1)
				.Value_Lambda([Model]() { return Model->blade_count; })
				.OnValueCommitted_Lambda([Model, OnLayoutChanged](int32 V, ETextCommit::Type)
				{ if (V != Model->blade_count) { Model->blade_count = V; OnLayoutChanged.ExecuteIfBound(); } })
			],
			LOCTEXT("BladeCountTip", "Number of blades, spaced evenly around the axis.")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ FormRow(LOCTEXT("BladeX", "Blade position (x/R):"),
			SNew(SAerotelicNumericEntry).Value_Lambda([Model]() { return Model->blade_x; })
				.MinValue(-1e9).MaxValue(1e9).Delta(0.01).LiveUpdate(false)
				.OnValueCommitted_Lambda([Model, OnLayoutChanged](double V, ETextCommit::Type)
				{ if (!FMath::IsNearlyEqual(V, Model->blade_x)) { Model->blade_x = V; OnLayoutChanged.ExecuteIfBound(); } }),
			LOCTEXT("BladeXTip", "Axial position of the blades' pitch-change axis, aft of the spinner tip,\nas a fraction of R (x/R).")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ FormRow(LOCTEXT("HubRadius", "Blade root radius (r/R):"),
			SNew(SAerotelicNumericEntry).Value_Lambda([Model]() { return Model->hub_radius; })
				.MinValue(0.0).MaxValue(1.0).Delta(0.01).LiveUpdate(false)
				.OnValueCommitted_Lambda([Model, OnLayoutChanged](double V, ETextCommit::Type)
				{ if (!FMath::IsNearlyEqual(V, Model->hub_radius)) { Model->hub_radius = V; OnLayoutChanged.ExecuteIfBound(); } }),
			LOCTEXT("HubRadiusTip", "Where each blade is cut off, as r/R. The blade is drawn from the axis\nout, so its axes are r/R; only the part beyond this radius is built,\nand it should be inside the spinner.")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ FormRow(LOCTEXT("Spin", "Spin direction:"),
			SNew(SBox).MinDesiredWidth(120)
			[
				SNew(SComboBox<TSharedPtr<FString>>)
				.OptionsSource(&SpinState->Options)
				.InitiallySelectedItem(SpinItem(Model->spin_direction))
				.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
				.OnSelectionChanged_Lambda([Model, SpinState, OnLayoutChanged](TSharedPtr<FString> S, ESelectInfo::Type)
				{
					if (!S.IsValid()) return;
					const SpinDirection D = SpinState->Options.IndexOfByKey(S) == 1 ? SpinDirection::Negative : SpinDirection::Positive;
					if (D == Model->spin_direction) return;
					Model->spin_direction = D;
					OnLayoutChanged.ExecuteIfBound();
				})
				[ SNew(STextBlock).Text_Lambda([Model, SpinItem]() { return FText::FromString(*SpinItem(Model->spin_direction)); }) ]
			],
			LOCTEXT("SpinTip", "Which way the propeller turns, by the right-hand rule with the thumb\nalong the shaft pointing from the propeller towards the spinner --\nthat is, forward. Positive (the default) is clockwise seen from\nbehind, the usual right-hand tractor propeller; Negative is the\nmirror image, turning the other way and still thrusting forward.\nOnly the blades change: the blade angle beta and every derived\nfigure are the same either way, the hand being in the geometry.")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ FormRow(LOCTEXT("SpinnerOn", "Spinner:"),
			SNew(SCheckBox)
				.IsChecked_Lambda([Model]() { return Model->spinner ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
				.OnCheckStateChanged_Lambda([Model, SpinnerEditor, OnLayoutChanged](ECheckBoxState S)
				{
					const bool bOn = S == ECheckBoxState::Checked;
					if (bOn == Model->spinner) return;
					Model->spinner = bOn;
					// The Spinner tab keeps editing the profile either way,
					// so it only needs redrawing because the blade-axis
					// marker is drawn against the spinner's outline.
					SpinnerEditor->RequestRedraw();
					OnLayoutChanged.ExecuteIfBound();
				})
				[ SNew(STextBlock).Text(LOCTEXT("SpinnerOnLabel", "Build the spinner")) ],
			LOCTEXT("SpinnerOnTip", "Whether the spinner is built at all. On by default. Off leaves the\nblades on their own -- a bare hub, or a rotor whose hub is modelled\nby other entities -- and the body list starts at \"Blade 1\".\nThe profile on the Spinner tab is kept either way, so switching it\nback on restores the shape unchanged.")) ];

	TSharedRef<SWidget> DerivedGroup = MakeGroup(LOCTEXT("DerivedGroup", "Derived"),
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("Diameter", "Diameter:"), Metric([](const PropellerMetrics& M) { return FormatG(M.diameter, 4); })) ]
		+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("Solidity", "Solidity σ:"), Metric([](const PropellerMetrics& M) { return FormatF(M.solidity, 4); })) ]
		+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("ActivityFactor", "Activity factor (per blade):"), Metric([](const PropellerMetrics& M) { return FormatF(M.activity_factor, 1); })) ]
		+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("Beta75", "Blade angle β at 0.75 R:"), Metric([](const PropellerMetrics& M) { return FText::FromString(FString::Printf(TEXT("%.2f °"), M.beta_75)); })) ]
		+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("PitchRatio", "Geometric pitch P/D at 0.75 R:"), Metric([](const PropellerMetrics& M) { return FormatF(M.pitch_ratio, 3); })) ]
		// Reported as "none" rather than as a number when the spinner is
		// switched off: the profile still has a size, but nothing is built
		// from it, and a live figure there reads as a body that exists.
		+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("SpinnerDiameter", "Spinner diameter:"), SpinnerMetric([](const PropellerMetrics& M) { return FormatG(M.spinner_diameter, 4); })) ]
		+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("SpinnerLength", "Spinner length:"), SpinnerMetric([](const PropellerMetrics& M) { return FormatG(M.spinner_length, 4); })) ]);

	TSharedRef<SWidget> PropellerTab = SNew(SScrollBox)
		+ SScrollBox::Slot().Padding(8)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight() [ MakeGroup(LOCTEXT("PropellerGroup", "Propeller"), PropellerInputs) ]
			+ SVerticalBox::Slot().AutoHeight() [ DerivedGroup ]
		];

	// ---- Spinner tab ----
	TSharedRef<SWidget> SpinnerTab = SNew(SVerticalBox)
		+ SVerticalBox::Slot().FillHeight(1.0f).Padding(4, 2) [ SpinnerEditor ]
		+ SVerticalBox::Slot().AutoHeight().Padding(8, 2)
		[
			MakeControlsRow(
				{
					FormRow(LOCTEXT("SpinnerLengthLabel", "Spinner length (l/R):"),
						SNew(SAerotelicNumericEntry).Value_Lambda([Model]() { return Model->spinner_length; })
							.MinValue(1e-9).MaxValue(1e9).Delta(0.01).LiveUpdate(false)
							.OnValueCommitted_Lambda([Model, SpinnerEditor, OnSpinnerChanged](double V, ETextCommit::Type)
							{
								if (FMath::IsNearlyEqual(V, Model->spinner_length)) return;
								Model->spinner_length = V;
								SpinnerEditor->SetScales(Model->spinner_length, 1.0);
								OnSpinnerChanged.ExecuteIfBound();
							}),
						LOCTEXT("SpinnerLengthTip", "Spinner length, tip to back, as a fraction of R.")),
					MakeDegreeSpin(LOCTEXT("Degree", "Degree:"), [Model]() { return &Model->spinner_curve_mut(); }, SpinnerEditor, OnSpinnerChanged,
						LOCTEXT("PolyDegreeTip", "Curve degree  (1 = polygon, 2 = quadratic, 3 = cubic ...).")),
				},
				{
					MakeControlButton(LOCTEXT("ResetKnots", "Reset Knots"), LOCTEXT("ResetKnotsTip", "Put the knots back at their default spacing (points unchanged)"),
						FOnClicked::CreateLambda([Model, SpinnerEditor, OnSpinnerChanged]()
						{
							Model->spinner_curve_mut().ResetKnotsPublic();
							SpinnerEditor->RequestRedraw();
							OnSpinnerChanged.ExecuteIfBound();
							return FReply::Handled();
						})),
					MakeControlButton(LOCTEXT("ResetDefault", "Reset to Default"), LOCTEXT("ResetDefaultTip", "The default general-aviation spinner profile, at the current length"),
						FOnClicked::CreateLambda([Model, SpinnerEditor, OnSpinnerChanged]()
						{
							const PropellerSpec Def;
							NurbsSpec Spec = Def.spinner_curve;
							Spec.weights.reset(); // unit weights, as the Python reset forces
							ResetCurveFromSpec(Model->spinner_curve_mut(), Spec);
							SpinnerEditor->RequestRedraw();
							OnSpinnerChanged.ExecuteIfBound();
							return FReply::Handled();
						})),
				})
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(8, 2)
		[
			// The side-view underlay spans the spinner's length (x/R).
			SNew(SReferenceImageBar).Editor(SpinnerEditor).Images(&Model->reference_images)
			.Key(TEXT("spinner")).ViewLabel(LOCTEXT("SideView", "side view")).Units(FText::FromString(TEXT("x/R")))
			.Placement(SReferenceImageBar::FPlaceReferenceImage::CreateLambda([Model](aerotelic::ReferenceImageSpec& S, FIntPoint Size)
			{
				ReferenceImageMath::FitWidth(S, Size, FMath::Max(Model->spinner_length, 1e-6), 0.0, 0.0);
			}))
		];

	TArray<FAerotelicTab> Tabs;
	Tabs.Add({ LOCTEXT("PropellerTab", "Propeller"), PropellerTab });
	for (const auto& Tab : BladeTabs->GetTabs())
	{
		Tabs.Add({ Tab.Key, Tab.Value });
	}
	Tabs.Add({ LOCTEXT("SpinnerTab", "Spinner"), SpinnerTab });
	// The Propeller and Spinner tabs' descriptions and the blade tabs' help
	// live here rather than above the controls.
	const FText HelpText = FText::Join(FText::FromString(TEXT("\n\n")),
		FText::Format(LOCTEXT("HelpPropellerTab", "PROPELLER TAB\n{0}"), kPropellerNote),
		FText::Format(LOCTEXT("HelpSpinnerTab", "SPINNER TAB\n{0}"), kSpinnerInfo),
		kHelp,
		FText::Format(LOCTEXT("HelpBladeTabs", "BLADE TABS\n\n{0}"), BladeTabs->GetHelpText()));
	Tabs.Add({ LOCTEXT("HelpTab", "Help"), MakeHelpTab(HelpText) });

	Out.Title = FText::Format(LOCTEXT("Title", "Propeller -- {0}"), FText::FromString(Entity->EntityName));
	// Keeps BladeTabs (and its model-bound widgets) alive with the panel.
	Out.WingTabs = BladeTabs;
	Out.Widget = SNew(SAerotelicTabStrip, Tabs);
}

#undef LOCTEXT_NAMESPACE
