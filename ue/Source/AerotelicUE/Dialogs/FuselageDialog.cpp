#include "FuselageDialog.h"
#include "EntityDesignContent.h"
#include "Widgets/TabStrip.h"
#include "Widgets/NumericEntry.h"
#include "Widgets/CurveEditor.h"
#include "Widgets/CurveTabControls.h"

#include "Widgets/SLeafWidget.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SGridPanel.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SSlider.h"
#include "Rendering/DrawElements.h"
#include "Styling/CoreStyle.h"
#include "Framework/Application/SlateApplication.h"
#include "Fonts/FontMeasure.h"

#include "Entity/AerotelicEntityActor.h"
#include "aerotelic/fuselage_model.hpp"

#define LOCTEXT_NAMESPACE "FuselageDialog"

using namespace aerotelic;

namespace
{
	// ---- Sections tab state: one query per station change / curve edit,
	// shared by the preview, the parameter read-outs and the summary table.
	struct FSectionCache
	{
		FuselageModel* Model = nullptr;
		double Station = 0.5;
		TOptional<FuselageSection> Section;
		TArray<std::array<double, 5>> Summary; // 11 rows: SectionParams at linspace(0, 1, 11)

		void Refresh()
		{
			Section.Reset();
			Summary.Reset();
			if (Model == nullptr) return;
			try
			{
				Section = Model->QuerySection(Station, 120);
			}
			catch (const std::exception&) {}
			for (int32 i = 0; i < 11; ++i)
			{
				try
				{
					Summary.Add(Model->SectionParams(i / 10.0));
				}
				catch (const std::exception&)
				{
					Summary.Add({ 0, 0, 0, 0, 0 });
				}
			}
		}
	};

	// The cross-section preview (the Sections tab's matplotlib figure): the
	// contour at the chosen station, the symmetry plane, and markers at the
	// top, bottom and widest points. Model units, not divided by L.
	class SFuselageSectionPreview : public SLeafWidget
	{
	public:
		SLATE_BEGIN_ARGS(SFuselageSectionPreview) {}
		SLATE_END_ARGS()

		void Construct(const FArguments& InArgs, TSharedRef<FSectionCache> InCache) { Cache = InCache; }

		virtual FVector2D ComputeDesiredSize(float) const override { return FVector2D(300.0f, 220.0f); }

		virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& Geo, const FSlateRect& Cull,
			FSlateWindowElementList& Out, int32 Layer, const FWidgetStyle& Style, bool bEnabled) const override
		{
			const FVector2D Size = Geo.GetLocalSize();
			FSlateDrawElement::MakeBox(Out, Layer, Geo.ToPaintGeometry(), FCoreStyle::Get().GetBrush("WhiteBrush"),
				ESlateDrawEffect::None, FLinearColor(0.051f, 0.067f, 0.09f));
			const FSlateFontInfo Font = FCoreStyle::GetDefaultFontStyle("Regular", 8);
			if (!Cache->Section.IsSet() || Cache->Section->contour_2d.rows() < 2)
			{
				FSlateDrawElement::MakeText(Out, Layer + 1, Geo.ToOffsetPaintGeometry(FVector2D(8, 8)),
					LOCTEXT("NoSection", "Section unavailable"), Font, ESlateDrawEffect::None, FLinearColor(0.53f, 0.53f, 0.53f));
				return Layer + 2;
			}
			const FuselageSection& S = Cache->Section.GetValue();
			const Mat& C = S.contour_2d;

			// Fit the contour (and the symmetry axis) with a margin, equal aspect.
			double MinX = 0, MaxX = 0, MinY = S.z_bot, MaxY = S.z_top;
			for (int i = 0; i < C.rows(); ++i)
			{
				MinX = FMath::Min(MinX, C(i, 0)); MaxX = FMath::Max(MaxX, C(i, 0));
				MinY = FMath::Min(MinY, C(i, 1)); MaxY = FMath::Max(MaxY, C(i, 1));
			}
			const double Span = FMath::Max(FMath::Max(MaxX - MinX, MaxY - MinY), 1e-6) * 1.2;
			const double Cx = 0.5 * (MinX + MaxX), Cy = 0.5 * (MinY + MaxY);
			const double Scale = FMath::Min(Size.X, Size.Y - 18.0) / Span;
			auto ToLocal = [&](double X, double Y)
			{
				return FVector2f((float)(Size.X * 0.5 + (X - Cx) * Scale), (float)(18.0 + (Size.Y - 18.0) * 0.5 - (Y - Cy) * Scale));
			};

			// Symmetry plane.
			{
				TArray<FVector2f> Axis = { ToLocal(0.0, MinY - 0.1 * Span), ToLocal(0.0, MaxY + 0.1 * Span) };
				FSlateDrawElement::MakeLines(Out, Layer + 1, Geo.ToPaintGeometry(), Axis, ESlateDrawEffect::None, FLinearColor(0.2f, 0.2f, 0.2f), true, 1.0f);
			}
			// Contour, closed.
			TArray<FVector2f> Pts;
			for (int i = 0; i < C.rows(); ++i) Pts.Add(ToLocal(C(i, 0), C(i, 1)));
			const FVector2f First = Pts[0];
			Pts.Add(First);
			FSlateDrawElement::MakeLines(Out, Layer + 2, Geo.ToPaintGeometry(), Pts, ESlateDrawEffect::None, FLinearColor(0.31f, 0.78f, 0.5f), true, 1.5f);

			// Markers: top/bottom (blue), widest (orange).
			auto Marker = [&](double X, double Y, const FLinearColor& Col)
			{
				const FVector2f P = ToLocal(X, Y);
				FSlateDrawElement::MakeBox(Out, Layer + 3, Geo.ToPaintGeometry(FVector2f(7, 7), FSlateLayoutTransform(P - FVector2f(3.5f, 3.5f))),
					FCoreStyle::Get().GetBrush("WhiteBrush"), ESlateDrawEffect::None, Col);
			};
			const FLinearColor Blue(0.49f, 0.75f, 1.0f), Orange(0.94f, 0.63f, 0.19f);
			Marker(0.0, S.z_top, Blue);
			Marker(0.0, S.z_bot, Blue);
			const double Mid = 0.5 * (S.z_top + S.z_bot);
			Marker(S.y_half, Mid, Orange);
			Marker(-S.y_half, Mid, Orange);

			const FText Title = FText::FromString(FString::Printf(TEXT("x/L = %.3f  |  x = %.2f"), S.x_frac, S.x_pos));
			FSlateDrawElement::MakeText(Out, Layer + 3, Geo.ToOffsetPaintGeometry(FVector2D(6, 3)), Title, Font, ESlateDrawEffect::None, FLinearColor(0.8f, 0.8f, 0.8f));
			return Layer + 4;
		}

	private:
		TSharedPtr<FSectionCache> Cache;
	};

	TSharedRef<SCurveEditor> MakeEditor(FText XLabel, FText YLabel, FText Title, double YScale, bool bEqualAspect,
		TArray<FCurveEditorSlot> Slots, FSimpleDelegate OnChanged, bool bUserLinks = false, double DefaultYMax = 0.1)
	{
		TSharedRef<SCurveEditor> Editor = SNew(SCurveEditor)
			.XLabel(XLabel).YLabel(YLabel).Title(Title)
			.XScale(1.0).YScale(YScale).EqualAspect(bEqualAspect).DefaultYMax(DefaultYMax)
			.SupportsUserLinks(bUserLinks)
			.LeadingName(LOCTEXT("Nose", "nose")).TrailingName(LOCTEXT("Tail", "tail"))
			.OnChanged(OnChanged);
		Editor->SetSlots(MoveTemp(Slots));
		return Editor;
	}

	TSharedRef<SWidget> MakeLengthField(FuselageModel* Model, FSimpleDelegate OnLength)
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 6, 0)
			[ SNew(STextBlock).Text(LOCTEXT("Length", "Length:")) ]
			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SAerotelicNumericEntry)
				.ToolTipText(LOCTEXT("LengthTip", "Total fuselage length along the X axis"))
				.Value_Lambda([Model]() { return Model->length; })
				.MinValue(1e-9).MaxValue(1e9).Delta(0.5).LiveUpdate(true)
				.OnValueCommitted_Lambda([Model, OnLength](double V, ETextCommit::Type)
				{
					if (FMath::IsNearlyEqual(V, Model->length)) return;
					Model->SetLength(V);
					OnLength.ExecuteIfBound();
				})
			];
	}

	FText RhoTitle(const TCHAR* Which)
	{
		return FText::FromString(FString::Printf(TEXT("%s  ·  0.4142 = ellipse (circle if width = height)  ·  0.5 = parabola  ·  > 0.5 hyperbola (→ 1 squares the corner)"), Which));
	}

	const FText kHelp = LOCTEXT("Help",
		"CONTROL POINTS\n"
		"Left-drag a control point to move it; the end points keep their station (x/L = 0 or 1) and move only vertically. "
		"Both nose points sit at x/L = 0 at any height; the lower one follows the upper one by default and is drawn ringed. "
		"Double-click near a curve to add a point to it; right-click a point to set its weight or remove it. "
		"Right-click an end point to link it to, or unlink it from, the other curve's end point (links last for the session). "
		"Clicking a point highlights its blending function (upper above, lower below).\n\n"
		"KNOT GRAPHS\n"
		"Each knot graph shows the blending functions R_i(s) over the curve parameter, with the knot number line below. "
		"Drag the green knots between their neighbours; push one onto a neighbour to repeat it (a corner needs as many repeats as the degree). "
		"Grey knots are the pinned ends. Right-click the line to insert a knot without changing the shape; right-click a knot to raise its multiplicity or delete it (the shape changes).\n\n"
		"DEGREE AND RESETS\n"
		"The degree boxes change each curve's NURBS degree (1 = piecewise-linear, 2 = quadratic, 3 = cubic ...); changing it puts the knots back at their default spacing. "
		"Reset Knots restores the default spacing with the poles unchanged. Reset to Default restores the default curve.\n\n"
		"AXES AND LENGTH\n"
		"Profile and Width are drawn in fractions of the length on both axes at equal scale; changing Length leaves the points where they are and redraws the body bigger. "
		"The ρ tabs keep ordinary axes (ρ is a pure number).\n\n"
		"VIEW\n"
		"The view never moves on its own: it is framed once, then the wheel zooms about the cursor and the right-click menu's Fit view to contents fits everything back in.");
}

void BuildFuselageDesignContent(AAerotelicEntityActor* Entity, FAerotelicDesignContent& Out)
{
	if (Entity == nullptr || Entity->GetEntityType() != project_yaml::EntityType::Fuselage)
	{
		return;
	}

	FuselageModel* Model = Entity->GetModelAs<FuselageModel>();
	TWeakObjectPtr<AAerotelicEntityActor> WeakEntity = Entity;

	TSharedRef<FSectionCache> Cache = MakeShared<FSectionCache>();
	Cache->Model = Model;
	Cache->Refresh();

	// Every edit: re-query the Sections tab, then rebuild the solid.
	FSimpleDelegate OnChanged = FSimpleDelegate::CreateLambda([WeakEntity, Cache]()
	{
		Cache->Refresh();
		if (WeakEntity.IsValid()) WeakEntity->RefreshProxy();
	});

	const double InvLength = 1.0 / FMath::Max(Model->length, 1e-9);

	// ---- Profile ----
	FCurveEditorSlot UpperSlot, LowerSlot;
	UpperSlot.Curve = &Model->z_upper_curve_mut();
	UpperSlot.Color = FLinearColor(0.23f, 0.62f, 0.84f);
	UpperSlot.Label = LOCTEXT("Upper", "upper");
	UpperSlot.Basis = ECurveBasisPlacement::Above;
	LowerSlot.Curve = &Model->z_lower_curve_mut();
	LowerSlot.Color = FLinearColor(0.84f, 0.54f, 0.23f);
	LowerSlot.Label = LOCTEXT("Lower", "lower");
	LowerSlot.Basis = ECurveBasisPlacement::Below;
	TSharedRef<SCurveEditor> ProfileEditor = MakeEditor(
		LOCTEXT("XL", "Fuselage fraction (x/L)"), LOCTEXT("ZL", "Height (z/L)"),
		LOCTEXT("ProfileTitle", "Side profile  ·  upper and lower outline  ·  right-click an end point to link it"),
		InvLength, true, { UpperSlot, LowerSlot }, OnChanged, /*bUserLinks*/ true);
	{
		// The model's default pole link: the lower curve's nose follows the upper's.
		FCurveEditorUserLink NoseLink;
		NoseLink.Follower = FIntPoint(1, 0);
		NoseLink.Target = FIntPoint(0, 0);
		ProfileEditor->SetUserLinks({ NoseLink });
	}

	// ---- Width ----
	FCurveEditorSlot WidthSlot;
	WidthSlot.Curve = &Model->y_half_curve_mut();
	WidthSlot.Color = FLinearColor(0.75f, 0.56f, 0.19f);
	WidthSlot.YClampLo = 0.0;
	WidthSlot.Basis = ECurveBasisPlacement::Below;
	TSharedRef<SCurveEditor> WidthEditor = MakeEditor(
		LOCTEXT("XL", "Fuselage fraction (x/L)"), LOCTEXT("YL", "Half-width (y/L)"),
		LOCTEXT("WidthTitle", "Width profile  ·  fuselage half-width  ·  nose at x/L = 0"),
		InvLength, true, { WidthSlot }, OnChanged);

	// ---- rho upper / lower ----
	FCurveEditorSlot RhoUpperSlot;
	RhoUpperSlot.Curve = &Model->rho_upper_curve_mut();
	RhoUpperSlot.Color = FLinearColor(0.63f, 0.31f, 0.82f);
	RhoUpperSlot.YClampLo = 0.01; RhoUpperSlot.YClampHi = 0.99;
	RhoUpperSlot.Basis = ECurveBasisPlacement::Below;
	TSharedRef<SCurveEditor> RhoUpperEditor = MakeEditor(
		LOCTEXT("XL", "Fuselage fraction (x/L)"), LOCTEXT("RhoUpperY", "Conic ρ (upper arcs)"),
		RhoTitle(TEXT("ρ upper")), 1.0, false, { RhoUpperSlot }, OnChanged, /*bUserLinks*/ false, /*DefaultYMax*/ 1.0);

	FCurveEditorSlot RhoLowerSlot;
	RhoLowerSlot.Curve = &Model->rho_lower_curve_mut();
	RhoLowerSlot.Color = FLinearColor(0.82f, 0.44f, 0.63f);
	RhoLowerSlot.YClampLo = 0.01; RhoLowerSlot.YClampHi = 0.99;
	RhoLowerSlot.Basis = ECurveBasisPlacement::Below;
	TSharedRef<SCurveEditor> RhoLowerEditor = MakeEditor(
		LOCTEXT("XL", "Fuselage fraction (x/L)"), LOCTEXT("RhoLowerY", "Conic ρ (lower arcs)"),
		RhoTitle(TEXT("ρ lower")), 1.0, false, { RhoLowerSlot }, OnChanged, /*bUserLinks*/ false, /*DefaultYMax*/ 1.0);

	// Length: rescales the Profile/Width editors' y axis (the curves hold
	// lengths; the axes are fractions of L), then everything refreshes.
	FSimpleDelegate OnLength = FSimpleDelegate::CreateLambda([Model, ProfileEditor, WidthEditor, OnChanged]()
	{
		const double Inv = 1.0 / FMath::Max(Model->length, 1e-9);
		ProfileEditor->SetScales(1.0, Inv);
		WidthEditor->SetScales(1.0, Inv);
		OnChanged.ExecuteIfBound();
	});
	auto LengthField = [Model, OnLength]() { return MakeLengthField(Model, OnLength); };

	auto ResetKnotsButton = [OnChanged](TArray<NurbsCurve*> Curves, TSharedRef<SCurveEditor> Editor, FText Tip)
	{
		return MakeControlButton(LOCTEXT("ResetKnots", "Reset Knots"), Tip,
			FOnClicked::CreateLambda([Curves, Editor, OnChanged]()
			{
				for (NurbsCurve* C : Curves) C->ResetKnotsPublic();
				Editor->RequestRedraw();
				OnChanged.ExecuteIfBound();
				return FReply::Handled();
			}));
	};

	// "Reset to Default": the default curve's poles and degree (fuselage_panel.py's _DEFAULTS).
	auto ResetDefaultButton = [OnChanged](TArray<TPair<NurbsCurve*, NurbsSpec>> Targets, TSharedRef<SCurveEditor> Editor)
	{
		return MakeControlButton(LOCTEXT("ResetDefault", "Reset to Default"), FText(),
			FOnClicked::CreateLambda([Targets, Editor, OnChanged]()
			{
				for (const auto& T : Targets) ResetCurveFromSpec(*T.Key, T.Value);
				Editor->RequestRedraw();
				OnChanged.ExecuteIfBound();
				return FReply::Handled();
			}));
	};
	const FuselageSpec Defaults;

	auto CurveTab = [&](TSharedRef<SCurveEditor> Editor, TArray<TSharedRef<SWidget>> Leading, TArray<TSharedRef<SWidget>> Trailing)
	{
		Leading.Insert(LengthField(), 0);
		return SNew(SVerticalBox)
			+ SVerticalBox::Slot().FillHeight(1.0f) [ Editor ]
			+ SVerticalBox::Slot().AutoHeight().Padding(4, 2) [ MakeControlsRow(Leading, Trailing) ];
	};

	TSharedRef<SWidget> ProfileTab = CurveTab(ProfileEditor,
		{
			MakeDegreeSpin(LOCTEXT("UpperDegree", "Upper degree:"), [Model]() { return &Model->z_upper_curve_mut(); }, ProfileEditor, OnChanged),
			MakeDegreeSpin(LOCTEXT("LowerDegree", "Lower degree:"), [Model]() { return &Model->z_lower_curve_mut(); }, ProfileEditor, OnChanged),
		},
		{
			ResetKnotsButton({ &Model->z_upper_curve_mut(), &Model->z_lower_curve_mut() }, ProfileEditor,
				LOCTEXT("ResetKnotsBothTip", "Put both curves' knots back at their default spacing (poles unchanged)")),
			ResetDefaultButton({ { &Model->z_upper_curve_mut(), Defaults.z_upper_curve }, { &Model->z_lower_curve_mut(), Defaults.z_lower_curve } }, ProfileEditor),
		});

	TSharedRef<SWidget> WidthTab = CurveTab(WidthEditor,
		{ MakeDegreeSpin(LOCTEXT("NurbsDegree", "NURBS Degree:"), [Model]() { return &Model->y_half_curve_mut(); }, WidthEditor, OnChanged) },
		{
			ResetKnotsButton({ &Model->y_half_curve_mut() }, WidthEditor, LOCTEXT("ResetKnotsTip", "Put the knots back at their default spacing (poles unchanged)")),
			ResetDefaultButton({ { &Model->y_half_curve_mut(), Defaults.y_half_curve } }, WidthEditor),
		});

	TSharedRef<SWidget> RhoUpperTab = CurveTab(RhoUpperEditor,
		{ MakeDegreeSpin(LOCTEXT("NurbsDegree", "NURBS Degree:"), [Model]() { return &Model->rho_upper_curve_mut(); }, RhoUpperEditor, OnChanged) },
		{
			ResetKnotsButton({ &Model->rho_upper_curve_mut() }, RhoUpperEditor, LOCTEXT("ResetKnotsTip", "Put the knots back at their default spacing (poles unchanged)")),
			ResetDefaultButton({ { &Model->rho_upper_curve_mut(), Defaults.rho_upper_curve } }, RhoUpperEditor),
		});

	TSharedRef<SWidget> RhoLowerTab = CurveTab(RhoLowerEditor,
		{ MakeDegreeSpin(LOCTEXT("NurbsDegree", "NURBS Degree:"), [Model]() { return &Model->rho_lower_curve_mut(); }, RhoLowerEditor, OnChanged) },
		{
			ResetKnotsButton({ &Model->rho_lower_curve_mut() }, RhoLowerEditor, LOCTEXT("ResetKnotsTip", "Put the knots back at their default spacing (poles unchanged)")),
			ResetDefaultButton({ { &Model->rho_lower_curve_mut(), Defaults.rho_lower_curve } }, RhoLowerEditor),
		});

	// ---- Sections (read-only preview + parameters + summary table) ----
	auto Param = [Cache](TFunction<double(const FuselageSection&)> Get, int32 Decimals)
	{
		return TAttribute<FText>::CreateLambda([Cache, Get, Decimals]()
		{
			return Cache->Section.IsSet() ? FormatF(Get(Cache->Section.GetValue()), Decimals) : DashText();
		});
	};
	TSharedRef<SGridPanel> Table = SNew(SGridPanel);
	{
		const TCHAR* Headers[] = { TEXT("x/L"), TEXT("Z top"), TEXT("Z bot"), TEXT("½-width"), TEXT("ρ up"), TEXT("ρ low") };
		for (int32 c = 0; c < 6; ++c)
		{
			Table->AddSlot(c, 0).Padding(6, 2).HAlign(HAlign_Center)
			[ SNew(STextBlock).Text(FText::FromString(Headers[c])).ColorAndOpacity(FSlateColor(FLinearColor(0.53f, 0.53f, 0.53f))).Font(FCoreStyle::GetDefaultFontStyle("Bold", 8)) ];
		}
		for (int32 r = 0; r < 11; ++r)
		{
			Table->AddSlot(0, r + 1).Padding(6, 1).HAlign(HAlign_Center)
			[ SNew(STextBlock).Text(FormatF(r / 10.0, 2)).Font(FCoreStyle::GetDefaultFontStyle("Regular", 8)) ];
			for (int32 c = 0; c < 5; ++c)
			{
				Table->AddSlot(c + 1, r + 1).Padding(6, 1).HAlign(HAlign_Center)
				[
					SNew(STextBlock).Font(FCoreStyle::GetDefaultFontStyle("Regular", 8))
					.Text_Lambda([Cache, r, c]()
					{
						return Cache->Summary.IsValidIndex(r) ? FormatF(Cache->Summary[r][c], 3) : DashText();
					})
				];
			}
		}
	}

	TSharedRef<SWidget> SectionsTab = SNew(SScrollBox)
		+ SScrollBox::Slot().Padding(6)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight()
			[
				MakeGroup(LOCTEXT("StationGroup", "Station"),
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight()
					[
						SNew(SHorizontalBox)
						+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0)
						[ SNew(STextBlock).Text(LOCTEXT("StationLabel", "x/L :")) ]
						+ SHorizontalBox::Slot().AutoWidth()
						[
							SNew(SAerotelicNumericEntry)
							.Value_Lambda([Cache]() { return Cache->Station; })
							.MinValue(0.0).MaxValue(1.0).Delta(0.01).LiveUpdate(true)
							.OnValueCommitted_Lambda([Cache](double V, ETextCommit::Type) { Cache->Station = FMath::Clamp(V, 0.0, 1.0); Cache->Refresh(); })
						]
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
					[
						SNew(SSlider)
						.Value_Lambda([Cache]() { return (float)Cache->Station; })
						.OnValueChanged_Lambda([Cache](float V) { Cache->Station = FMath::Clamp((double)V, 0.0, 1.0); Cache->Refresh(); })
					])
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 6)
			[ SNew(SBox).HeightOverride(220) [ SNew(SFuselageSectionPreview, Cache) ] ]
			+ SVerticalBox::Slot().AutoHeight()
			[
				MakeGroup(LOCTEXT("ParamsGroup", "Section Parameters"),
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("XPos", "x position:"), Param([](const FuselageSection& S) { return S.x_pos; }, 3)) ]
					+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("ZTop", "Z top:"), Param([](const FuselageSection& S) { return S.z_top; }, 4)) ]
					+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("ZBot", "Z bottom:"), Param([](const FuselageSection& S) { return S.z_bot; }, 4)) ]
					+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("Height", "Height:"), Param([](const FuselageSection& S) { return S.height; }, 4)) ]
					+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("HalfWidth", "Half-width:"), Param([](const FuselageSection& S) { return S.y_half; }, 4)) ]
					+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("Width", "Width:"), Param([](const FuselageSection& S) { return S.width; }, 4)) ]
					+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("RhoU", "Conic ρ upper:"), Param([](const FuselageSection& S) { return S.rho_upper; }, 4)) ]
					+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("RhoL", "Conic ρ lower:"), Param([](const FuselageSection& S) { return S.rho_lower; }, 4)) ])
			]
			+ SVerticalBox::Slot().AutoHeight()
			[ MakeGroup(LOCTEXT("SummaryGroup", "Station Summary"), Table) ]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
			[ LengthField() ]
		];

	TArray<FAerotelicTab> Tabs;
	Tabs.Add({ LOCTEXT("Profile", "Profile"), ProfileTab });
	Tabs.Add({ LOCTEXT("WidthTab", "Width"), WidthTab });
	Tabs.Add({ LOCTEXT("RhoUpper", "ρ upper"), RhoUpperTab });
	Tabs.Add({ LOCTEXT("RhoLower", "ρ lower"), RhoLowerTab });
	Tabs.Add({ LOCTEXT("Sections", "Sections"), SectionsTab });
	Tabs.Add({ LOCTEXT("HelpTab", "Help"), MakeHelpTab(kHelp) });

	Out.Title = FText::Format(LOCTEXT("Title", "Fuselage -- {0}"), FText::FromString(Entity->EntityName));
	Out.Widget = SNew(SAerotelicTabStrip, Tabs);
}

#undef LOCTEXT_NAMESPACE
