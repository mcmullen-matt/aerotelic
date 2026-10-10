#include "NacelleDialog.h"
#include "EntityDesignContent.h"
#include "Widgets/TabStrip.h"
#include "Widgets/NumericEntry.h"
#include "Widgets/CurveEditor.h"
#include "Widgets/CurveTabControls.h"
#include "Widgets/ReferenceImageBar.h"
#include "Widgets/ReferenceImageMath.h"
#include "Widgets/PromptDialogs.h"

#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SSpinBox.h"
#include "Misc/MessageDialog.h"

#include "Entity/AerotelicEntityActor.h"
#include "aerotelic/nacelle_model.hpp"

#define LOCTEXT_NAMESPACE "NacelleDialog"

using namespace aerotelic;

namespace
{
	TSharedRef<SWidget> FormRow(const FText& Label, TSharedRef<SWidget> Field)
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 2, 8, 2)
			[ SNew(STextBlock).Text(Label) ]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 2)
			[ Field ];
	}

	TSharedRef<SWidget> MakeLengthField(NacelleModel* Model, FSimpleDelegate OnLength)
	{
		return FormRow(LOCTEXT("Length", "Length:"),
			SNew(SAerotelicNumericEntry)
			.ToolTipText(LOCTEXT("LengthTip", "Nacelle length along its axis, lip to trailing edge"))
			.Value_Lambda([Model]() { return Model->length; })
			.MinValue(1e-9).MaxValue(1e9).Delta(0.1).LiveUpdate(false)
			.OnValueCommitted_Lambda([Model, OnLength](double V, ETextCommit::Type)
			{
				if (FMath::IsNearlyEqual(V, Model->length)) return;
				Model->SetLength(V);
				OnLength.ExecuteIfBound();
			}));
	}

	const FText kRadiusPrompt = LOCTEXT("RadiusPrompt",
		"The front curve is the inlet lip exactly as drawn, and every other ring of the cowl is it scaled in proportion to the side view's radius there. "
		"This is therefore the lip's own radius, and it sets the size of the whole body.");

	const FText kHelp = LOCTEXT("Help",
		"HOW THE NACELLE IS MADE\n"
		"The Side view is the section through the cowl wall: the upper curve is the outer skin, the lower the inner (duct) skin, both as radii from the axis (the axis is y/L = 0). "
		"They meet at the inlet lip (x/L = 0) and at a sharp trailing edge (x/L = 1). The Front view is the lip ring as drawn, looking aft (starboard on the left); "
		"every other ring is it scaled about the axis in proportion to the section radius there, so a circular lip gives a true body of revolution.\n\n"
		"SIDE VIEW\n"
		"Dragging the lip or trailing-edge point moves it vertically and brings the other skin's end point with it; other points move freely. "
		"Double-click near a skin to add a point to it; right-click a point to set its weight or remove it (the shared end points cannot be removed). "
		"A second point straight above or below the lip gives a round lip.\n\n"
		"FRONT VIEW\n"
		"Points move anywhere; the curve is closed, so there are no end points, and the point marked seam is where its parameter starts. "
		"Double-click adds a point on the nearest side of the control polygon. Reset to Circle... asks for a radius in units of the nacelle length, "
		"opening on the side view's lip point, and sets the size of the whole body.\n\n"
		"KNOT GRAPHS\n"
		"Drag the green knots between their neighbours; push one onto a neighbour to repeat it. Grey knots are the clamped ends of a skin and the seam of the lip curve. "
		"Right-click the line to insert a knot without changing the shape; right-click a knot to raise its multiplicity or delete it.\n\n"
		"CHEVRONS\n"
		"Triangular notches around the trailing edge, spaced evenly by arc length with the first tooth tip at the top. Depth is how far forward they reach, "
		"marked by a dashed line in the side view; 0 is a plain trailing edge. The tip angle beside it is the included angle at a tooth tip, reported so a body drawn either way can be matched.\n\n"
		"VIEW\n"
		"The wheel zooms about the cursor; the right-click menu's Fit view to contents fits everything; nothing else moves the view.");
}

void BuildNacelleDesignContent(AAerotelicEntityActor* Entity, FAerotelicDesignContent& Out)
{
	if (Entity == nullptr || Entity->GetEntityType() != project_yaml::EntityType::Nacelle)
	{
		return;
	}

	NacelleModel* Model = Entity->GetModelAs<NacelleModel>();
	TWeakObjectPtr<AAerotelicEntityActor> WeakEntity = Entity;
	FSimpleDelegate Rebuild = FSimpleDelegate::CreateLambda([WeakEntity]()
	{
		if (WeakEntity.IsValid()) WeakEntity->RefreshProxy();
	});

	const double InvLength = 1.0 / FMath::Max(Model->length, 1e-9);

	// ---- Side view: outer/inner skins, lip + trailing-edge poles shared rigidly ----
	FCurveEditorSlot OuterSlot, InnerSlot;
	OuterSlot.Curve = &Model->upper_curve_mut();
	OuterSlot.Color = FLinearColor(0.23f, 0.62f, 0.84f);
	OuterSlot.Label = LOCTEXT("Outer", "outer skin");
	OuterSlot.Basis = ECurveBasisPlacement::Above;
	InnerSlot.Curve = &Model->lower_curve_mut();
	InnerSlot.Color = FLinearColor(0.84f, 0.54f, 0.23f);
	InnerSlot.Label = LOCTEXT("Inner", "inner skin");
	InnerSlot.Basis = ECurveBasisPlacement::Below;

	TSharedRef<SCurveEditor> SideEditor = SNew(SCurveEditor)
		.XLabel(LOCTEXT("XL", "Nacelle fraction (x/L)"))
		.YLabel(LOCTEXT("YL", "Radius from axis (y/L)"))
		.Title(LOCTEXT("SideTitle", "Side view  ·  outer and inner skin  ·  lip and trailing edge shared"))
		.DefaultYMax(0.25)
		.XScale(1.0).YScale(InvLength).EqualAspect(true)
		.OnChanged(Rebuild);
	SideEditor->SetSlots({ OuterSlot, InnerSlot });
	SideEditor->SetLinks({ { FIntPoint(0, 0), FIntPoint(1, 0) }, { FIntPoint(0, -1), FIntPoint(1, -1) } });
	// Chevron-valley marker: a dashed red line at x = 1 - depth/L across the wall.
	SideEditor->SetOutlineProvider(FOnGetOutlinePaths::CreateLambda([Model]()
	{
		TArray<FCurveEditorOutlinePath> Paths;
		const double Depth = Model->chevron_count > 0 ? Model->chevron_depth : 0.0;
		if (Depth <= 0.0 || Depth >= Model->length) return Paths;
		const double X = 1.0 - Depth / Model->length;
		try
		{
			const double YOut = Model->upper_curve().ZAtEta(X)[1];
			const double YIn = Model->lower_curve().ZAtEta(X)[1];
			FCurveEditorOutlinePath P;
			P.Points = { FVector2D(X, YIn), FVector2D(X, YOut) };
			P.Color = FLinearColor(0.88f, 0.31f, 0.31f);
			P.Label = LOCTEXT("Valleys", "chevron valleys");
			P.LineWidth = 1.2f;
			Paths.Add(P);
		}
		catch (const std::exception&) {}
		return Paths;
	}));

	// ---- Front view: closed, free-x lip ring ----
	auto MakeFrontSlot = [Model]()
	{
		FCurveEditorSlot FrontSlot;
		FrontSlot.Curve = &Model->front_curve_mut();
		FrontSlot.Color = FLinearColor(0.31f, 0.78f, 0.5f);
		FrontSlot.Label = LOCTEXT("Lip", "inlet lip");
		FrontSlot.bClosedFill = true;
		FrontSlot.bFreeX = true;
		FrontSlot.Basis = ECurveBasisPlacement::Below;
		return FrontSlot;
	};
	TSharedRef<SCurveEditor> FrontEditor = SNew(SCurveEditor)
		.XLabel(LOCTEXT("LateralL", "Lateral (y/L)   ← starboard  ·  port →"))
		.YLabel(LOCTEXT("VerticalL", "Vertical (z/L)"))
		.Title(LOCTEXT("FrontTitle", "Front view  ·  inlet lip, looking aft  ·  the axis is the crosshair"))
		.XScale(InvLength).YScale(InvLength).EqualAspect(true)
		.OnChanged(Rebuild);
	FrontEditor->SetSlots({ MakeFrontSlot() });

	// ---- Chevrons tab read-outs ----
	TSharedRef<STextBlock> ToothWidthLabel = SNew(STextBlock);
	TSharedRef<STextBlock> DepthRatioLabel = SNew(STextBlock);
	TSharedRef<STextBlock> TipAngleLabel = SNew(STextBlock);
	TSharedRef<STextBlock> TePerimeterLabel = SNew(STextBlock);
	TFunction<void()> RefreshChevronReadouts = [Model, ToothWidthLabel, DepthRatioLabel, TipAngleLabel, TePerimeterLabel]()
	{
		const int32 N = Model->chevron_count;
		double Perim = std::numeric_limits<double>::quiet_NaN();
		try { Perim = Model->TePerimeter(); } catch (const std::exception&) {}
		ToothWidthLabel->SetText(N > 0 ? FormatG(Perim / N, 4) : DashText());
		DepthRatioLabel->SetText(N > 0 ? FormatF(Model->chevron_depth / FMath::Max(Model->length, 1e-9), 3) : DashText());
		double Angle = 0.0;
		try { Angle = N > 0 ? Model->ChevronTipAngle() : 0.0; } catch (const std::exception&) {}
		TipAngleLabel->SetText(N > 0 && Angle > 0.0 ? FText::FromString(FString::Printf(TEXT("%.1f °"), Angle)) : DashText());
		TePerimeterLabel->SetText(FormatG(Perim, 4));
	};
	RefreshChevronReadouts();

	// Every curve edit also changes the trailing-edge perimeter.
	FSimpleDelegate OnCurveChanged = FSimpleDelegate::CreateLambda([RefreshChevronReadouts, Rebuild]()
	{
		RefreshChevronReadouts();
		Rebuild.ExecuteIfBound();
	});
	FSimpleDelegate OnChevronsChanged = FSimpleDelegate::CreateLambda([RefreshChevronReadouts, SideEditor, Rebuild]()
	{
		RefreshChevronReadouts();
		SideEditor->RequestRedraw(); // the valley marker moves
		Rebuild.ExecuteIfBound();
	});
	// Length: rescales both editors' axes and re-seeds the chevron depth
	// (SetLength scales it along with the curves).
	FSimpleDelegate OnLength = FSimpleDelegate::CreateLambda([Model, SideEditor, FrontEditor, OnChevronsChanged]()
	{
		const double Inv = 1.0 / FMath::Max(Model->length, 1e-9);
		SideEditor->SetScales(1.0, Inv);
		FrontEditor->SetScales(Inv, Inv);
		OnChevronsChanged.ExecuteIfBound();
	});
	auto LengthField = [Model, OnLength]() { return MakeLengthField(Model, OnLength); };

	auto CurveTab = [&](TSharedRef<SCurveEditor> Editor, TArray<TSharedRef<SWidget>> Leading, TArray<TSharedRef<SWidget>> Trailing, TSharedPtr<SWidget> Extra = nullptr)
	{
		Leading.Insert(LengthField(), 0);
		return SNew(SVerticalBox)
			+ SVerticalBox::Slot().FillHeight(1.0f) [ Editor ]
			+ SVerticalBox::Slot().AutoHeight().Padding(4, 2) [ MakeControlsRow(Leading, Trailing) ]
			+ SVerticalBox::Slot().AutoHeight().Padding(4, 2) [ Extra.IsValid() ? Extra.ToSharedRef() : SNullWidget::NullWidget ];
	};
	// Reference images: the side view spans the length from x/L = 0; the
	// front view is centred on the axis, three lip radii wide.
	TSharedRef<SWidget> SideImageBar = SNew(SReferenceImageBar).Editor(SideEditor).Images(&Model->reference_images)
		.Key(TEXT("side")).ViewLabel(LOCTEXT("SideView", "side view"));
	TSharedRef<SWidget> FrontImageBar = SNew(SReferenceImageBar).Editor(FrontEditor).Images(&Model->reference_images)
		.Key(TEXT("front")).ViewLabel(LOCTEXT("FrontView", "front view"))
		.Placement(SReferenceImageBar::FPlaceReferenceImage::CreateLambda([Model](aerotelic::ReferenceImageSpec& S, FIntPoint Size)
		{
			double R = 0.0;
			const Mat Poles = Model->front_curve().poles();
			for (int i = 0; i < Poles.rows(); ++i) R = FMath::Max(R, FMath::Max(FMath::Abs(Poles(i, 0)), FMath::Abs(Poles(i, 1))));
			R /= FMath::Max(Model->length, 1e-9);
			const double W = 3.0 * FMath::Max(R, 1e-3);
			ReferenceImageMath::FitWidth(S, Size, W, 0.0, -0.5 * W);
		}));

	// Side view controls.
	TSharedRef<SWidget> SideTab = CurveTab(SideEditor,
		{
			MakeDegreeSpin(LOCTEXT("OuterDegree", "Outer degree:"), [Model]() { return &Model->upper_curve_mut(); }, SideEditor, OnCurveChanged),
			MakeDegreeSpin(LOCTEXT("InnerDegree", "Inner degree:"), [Model]() { return &Model->lower_curve_mut(); }, SideEditor, OnCurveChanged),
		},
		{
			MakeControlButton(LOCTEXT("ResetKnots", "Reset Knots"), LOCTEXT("ResetKnotsBothTip", "Put both skins' knots back at their default spacing (points unchanged)"),
				FOnClicked::CreateLambda([Model, SideEditor, OnCurveChanged]()
				{
					Model->upper_curve_mut().ResetKnotsPublic();
					Model->lower_curve_mut().ResetKnotsPublic();
					SideEditor->RequestRedraw();
					OnCurveChanged.ExecuteIfBound();
					return FReply::Handled();
				})),
			MakeControlButton(LOCTEXT("ResetDefault", "Reset to Default"), FText(),
				FOnClicked::CreateLambda([Model, SideEditor, OnCurveChanged]()
				{
					const double K = Model->length / kNacelleDefaultLength;
					const NacelleSpec Def;
					ResetCurveFromSpec(Model->upper_curve_mut(), Def.upper_curve, FVector2D(1.0, K));
					ResetCurveFromSpec(Model->lower_curve_mut(), Def.lower_curve, FVector2D(1.0, K));
					SideEditor->RequestRedraw();
					OnCurveChanged.ExecuteIfBound();
					return FReply::Handled();
				})),
		},
		SideImageBar);

	// Front view controls.
	TSharedRef<SWidget> FrontTab = CurveTab(FrontEditor,
		{ MakeDegreeSpin(LOCTEXT("Degree", "Degree:"), [Model]() { return &Model->front_curve_mut(); }, FrontEditor, OnCurveChanged,
			LOCTEXT("PolyDegreeTip", "Curve degree  (1 = polygon, 2 = quadratic, 3 = cubic ...).")) },
		{
			MakeControlButton(LOCTEXT("ResetKnots", "Reset Knots"), LOCTEXT("ResetKnotsTip", "Put the knots back at their default spacing (points unchanged)"),
				FOnClicked::CreateLambda([Model, FrontEditor, OnCurveChanged]()
				{
					Model->front_curve_mut().ResetKnotsPublic();
					FrontEditor->RequestRedraw();
					OnCurveChanged.ExecuteIfBound();
					return FReply::Handled();
				})),
			MakeControlButton(LOCTEXT("ResetCircle", "Reset to Circle..."),
				LOCTEXT("ResetCircleTip", "A round lip ring, at a radius you type in units of the nacelle length.\nStarts from the lip point of the side view, so accepting it unchanged\ngives a circle through that point."),
				FOnClicked::CreateLambda([Model, FrontEditor, MakeFrontSlot, OnCurveChanged]()
				{
					const double Length = FMath::Max(Model->length, 1e-9);
					double Current = FMath::Abs(Model->LipRadius()) / Length;
					if (!(Current > 0.0)) Current = 0.25;
					TOptional<double> R = PromptCircleRadius(Current, Length, kRadiusPrompt);
					if (!R.IsSet()) return FReply::Handled();
					try
					{
						Model->SetFrontCurve(DefaultFrontCurveSpec(R.GetValue() * Length));
					}
					catch (const std::exception& e)
					{
						FMessageDialog::Open(EAppMsgType::Ok, FText::FromString(FString::Printf(TEXT("That radius was rejected:\n%s"), UTF8_TO_TCHAR(e.what()))));
						return FReply::Handled();
					}
					FrontEditor->SetSlots({ MakeFrontSlot() }); // SetFrontCurve replaces the curve object
					FrontEditor->RequestRedraw();
					OnCurveChanged.ExecuteIfBound();
					return FReply::Handled();
				})),
		},
		FrontImageBar);

	// ---- Chevrons ----
	TSharedRef<SWidget> ChevronsTab = SNew(SScrollBox)
		+ SScrollBox::Slot().Padding(8)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight()
			[
				MakeGroup(LOCTEXT("ChevronsGroup", "Trailing-edge chevrons"),
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight()
					[ FormRow(LOCTEXT("ChevronCount", "Number of chevrons:"),
						SNew(SBox).WidthOverride(90)
						[
							SNew(SSpinBox<int32>)
							.ToolTipText(LOCTEXT("CountTip", "How many notches around the trailing edge. 0 = a plain trailing edge."))
							.MinValue(0).MaxValue(kMaxChevrons).MinSliderValue(0).MaxSliderValue(kMaxChevrons).Delta(1)
							.Value_Lambda([Model]() { return Model->chevron_count; })
							.OnValueCommitted_Lambda([Model, OnChevronsChanged](int32 V, ETextCommit::Type)
							{
								if (V == Model->chevron_count) return;
								Model->chevron_count = V;
								OnChevronsChanged.ExecuteIfBound();
							})
						]) ]
					+ SVerticalBox::Slot().AutoHeight()
					[ FormRow(LOCTEXT("ChevronDepth", "Chevron depth:"),
						SNew(SAerotelicNumericEntry)
						.ToolTipText(LOCTEXT("DepthTip", "How far forward the notches reach: the distance from the trailing edge\nto the forwardmost point of a notch."))
						.Value_Lambda([Model]() { return Model->chevron_depth; })
						.MinValue(0.0).MaxValue(1e9).Delta(0.05).LiveUpdate(false)
						.OnValueCommitted_Lambda([Model, OnChevronsChanged](double V, ETextCommit::Type)
						{
							if (FMath::IsNearlyEqual(V, Model->chevron_depth)) return;
							Model->chevron_depth = V;
							OnChevronsChanged.ExecuteIfBound();
						})) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0) [ MakeMetricRow(LOCTEXT("ToothWidth", "Tooth width:"), TAttribute<FText>::CreateLambda([ToothWidthLabel]() { return ToothWidthLabel->GetText(); })) ]
					+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("DepthRatio", "Depth / length:"), TAttribute<FText>::CreateLambda([DepthRatioLabel]() { return DepthRatioLabel->GetText(); })) ]
					+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("TipAngle", "Tip angle:"), TAttribute<FText>::CreateLambda([TipAngleLabel]() { return TipAngleLabel->GetText(); })) ]
					+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("TePerimeter", "Trailing-edge perimeter:"), TAttribute<FText>::CreateLambda([TePerimeterLabel]() { return TePerimeterLabel->GetText(); })) ])
			]
			// (The chevron description is in the Help tab's CHEVRONS section.)
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
			[ LengthField() ]
		];

	TArray<FAerotelicTab> Tabs;
	Tabs.Add({ LOCTEXT("SideTab", "Side view"), SideTab });
	Tabs.Add({ LOCTEXT("FrontTab", "Front view"), FrontTab });
	Tabs.Add({ LOCTEXT("ChevronsTab", "Chevrons"), ChevronsTab });
	Tabs.Add({ LOCTEXT("HelpTab", "Help"), MakeHelpTab(kHelp) });

	Out.Title = FText::Format(LOCTEXT("Title", "Nacelle -- {0}"), FText::FromString(Entity->EntityName));
	Out.Widget = SNew(SAerotelicTabStrip, Tabs);
}

#undef LOCTEXT_NAMESPACE
