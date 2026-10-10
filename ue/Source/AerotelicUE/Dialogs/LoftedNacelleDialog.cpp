#include "LoftedNacelleDialog.h"
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
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SSpinBox.h"
#include "Widgets/Input/SComboBox.h"
#include "Misc/MessageDialog.h"

#include "Entity/AerotelicEntityActor.h"
#include "aerotelic/lofted_nacelle_model.hpp"

#define LOCTEXT_NAMESPACE "LoftedNacelleDialog"

using namespace aerotelic;

namespace
{
	const FLinearColor kOuterColor(0.23f, 0.62f, 0.84f);
	const FLinearColor kInnerColor(0.84f, 0.54f, 0.23f);
	const FLinearColor kSkinColor(0.54f, 0.58f, 0.65f);
	const FLinearColor kMeanColor(0.31f, 0.78f, 0.5f);
	const FLinearColor kCentrelineColor(0.75f, 0.5f, 1.0f);
	const FLinearColor kGuideColor(0.31f, 0.78f, 0.5f);
	const FLinearColor kOtherGuideColor(0.29f, 0.34f, 0.41f);
	const FLinearColor kValleyColor(0.88f, 0.31f, 0.31f);

	TSharedRef<SWidget> FormRow(const FText& Label, TSharedRef<SWidget> Field)
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 2, 8, 2)
			[ SNew(STextBlock).Text(Label) ]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 2)
			[ Field ];
	}

	TSharedRef<SWidget> MakeLengthField(LoftedNacelleModel* Model, FSimpleDelegate OnLength)
	{
		return FormRow(LOCTEXT("Length", "Length:"),
			SNew(SAerotelicNumericEntry)
			.ToolTipText(LOCTEXT("LengthTip", "Nacelle length along its axis, inlet to trailing edge"))
			.Value_Lambda([Model]() { return Model->length; })
			.MinValue(1e-9).MaxValue(1e9).Delta(0.1).LiveUpdate(false)
			.OnValueCommitted_Lambda([Model, OnLength](double V, ETextCommit::Type)
			{
				if (FMath::IsNearlyEqual(V, Model->length)) return;
				Model->SetLength(V);
				OnLength.ExecuteIfBound();
			}));
	}

	TArray<FVector2D> MatPoints(const Mat& M)
	{
		TArray<FVector2D> Pts;
		Pts.Reserve(M.rows());
		for (int i = 0; i < M.rows(); ++i) Pts.Add(FVector2D(M(i, 0), M(i, 1)));
		return Pts;
	}

	const FText kRadiusPrompt = LOCTEXT("RadiusPrompt",
		"The guide curve is the radius the cowl is revolved around, so this is how far the mean line stands off the centreline at this station. "
		"The skins are then offset from it.");

	// The Guide-curves tab (lofted_nacelle_panel.py's _SectionsTab): a combo of
	// the guide curves, the selected one's station, Add/Remove, and one curve
	// editor rebound to whichever guide is selected. Tracks the selection by
	// the curve's pointer identity, not index -- indices shift whenever
	// sections re-sort by station.
	class SGuideCurvesTab : public SCompoundWidget
	{
	public:
		SLATE_BEGIN_ARGS(SGuideCurvesTab) {}
			SLATE_EVENT(FSimpleDelegate, OnChanged)
			SLATE_ARGUMENT(TSharedPtr<SWidget>, LengthField)
		SLATE_END_ARGS()

		void Construct(const FArguments& InArgs, LoftedNacelleModel* InModel)
		{
			Model = InModel;
			OnChangedDelegate = InArgs._OnChanged;
			LengthField = InArgs._LengthField;
			if (!Model->sections().empty())
			{
				SelectedCurve = Model->sections()[0].curve.get();
			}

			Editor = SNew(SCurveEditor)
				.XLabel(LOCTEXT("LateralL", "Lateral (y/L)   ← starboard  ·  port →"))
				.YLabel(LOCTEXT("VerticalL", "Vertical (z/L)"))
				.Title(LOCTEXT("GuideTitle", "Guide curve  ·  front view  ·  the mean ring the cowl is revolved around"))
				.XScale(1.0 / FMath::Max(Model->length, 1e-9)).YScale(1.0 / FMath::Max(Model->length, 1e-9)).EqualAspect(true)
				.OnChanged(FSimpleDelegate::CreateSP(this, &SGuideCurvesTab::NotifyChanged));
			Editor->SetOutlineProvider(FOnGetOutlinePaths::CreateSP(this, &SGuideCurvesTab::ContextOutlines));

			// The front-view underlay: centred on the axis, three radii wide.
			LoftedNacelleModel* M = Model;
			TSharedRef<SWidget> ImageBar = SNew(SReferenceImageBar).Editor(Editor).Images(&Model->reference_images)
				.Key(TEXT("section")).ViewLabel(LOCTEXT("FrontView", "front view")).Units(FText::FromString(TEXT("y/L")))
				.Placement(SReferenceImageBar::FPlaceReferenceImage::CreateLambda([M](aerotelic::ReferenceImageSpec& S, FIntPoint Size)
				{
					double R = 0.0;
					for (const GuideSection& Sec : M->sections())
					{
						const Mat Poles = Sec.curve->poles();
						for (int i = 0; i < Poles.rows(); ++i) R = FMath::Max(R, FMath::Max(FMath::Abs(Poles(i, 0)), FMath::Abs(Poles(i, 1))));
					}
					R /= FMath::Max(M->length, 1e-9);
					const double W = 3.0 * FMath::Max(R, 1e-3);
					ReferenceImageMath::FitWidth(S, Size, W, 0.0, -0.5 * W);
				}));

			ChildSlot
			[
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().FillHeight(1.0f) [ Editor.ToSharedRef() ]
				+ SVerticalBox::Slot().AutoHeight().Padding(4, 2)
				[ MakeGroup(LOCTEXT("GuidesGroup", "Guide curves"), SAssignNew(GuidesRow, SHorizontalBox)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(4, 2)
				[ SAssignNew(ControlsHost, SBox) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(4, 2)
				[ ImageBar ]
			];
			Rebuild();
		}

		// Called by the panel when another tab changed the body (the rings
		// drawn here follow the centreline and skins).
		void Redraw() { if (Editor.IsValid()) Editor->RequestRedraw(); }
		void SetScales(double Inv) { if (Editor.IsValid()) Editor->SetScales(Inv, Inv); }

	private:
		LoftedNacelleModel* Model = nullptr;
		FSimpleDelegate OnChangedDelegate;
		TSharedPtr<SWidget> LengthField;
		TSharedPtr<SCurveEditor> Editor;
		TSharedPtr<SHorizontalBox> GuidesRow;
		TSharedPtr<SBox> ControlsHost;
		PeriodicNurbsCurve* SelectedCurve = nullptr;
		TArray<TSharedPtr<int32>> GuideOptions;

		void NotifyChanged() { OnChangedDelegate.ExecuteIfBound(); }

		GuideSection* FindSelected() const
		{
			if (SelectedCurve == nullptr) return nullptr;
			return Model->FindSectionByCurve(SelectedCurve);
		}

		int32 SelectedIndex() const
		{
			const auto& Secs = Model->sections();
			for (int32 i = 0; i < (int32)Secs.size(); ++i)
			{
				if (Secs[i].curve.get() == SelectedCurve) return i;
			}
			return INDEX_NONE;
		}

		// Midpoint of the largest gap along the body (0.5 if there are none).
		double NextStation() const
		{
			TArray<double> S = { 0.0, 1.0 };
			for (double St : Model->SectionStations()) S.Add(St);
			S.Sort();
			double Best = 0.0, At = 0.5;
			for (int32 i = 1; i < S.Num(); ++i)
			{
				if (S[i] - S[i - 1] > Best) { Best = S[i] - S[i - 1]; At = 0.5 * (S[i] + S[i - 1]); }
			}
			return Best > 0.0 ? At : 0.5;
		}

		TArray<FCurveEditorOutlinePath> ContextOutlines()
		{
			TArray<FCurveEditorOutlinePath> Paths;
			const GuideSection* Sel = FindSelected();
			if (Sel == nullptr) return Paths;
			// The other guide curves, faint.
			for (const auto& S : Model->sections())
			{
				if (S.curve.get() == SelectedCurve) continue;
				FCurveEditorOutlinePath P;
				P.Points = MatPoints(S.curve->Sample(241));
				P.Color = kOtherGuideColor;
				P.Label = FText::FromString(FString::Printf(TEXT("x/L %.3f"), S.station));
				P.bClosed = true;
				P.LineWidth = 1.0f;
				Paths.Add(P);
			}
			// The skins as they will be built at this station, about the guide's own origin.
			try
			{
				const double Cz = Model->CentrelineZ(Sel->station);
				auto Rings = Model->RingsYZ(Sel->station, 241);
				auto Shifted = [Cz](const Mat& M)
				{
					TArray<FVector2D> Pts = MatPoints(M);
					for (FVector2D& P : Pts) P.Y -= Cz;
					return Pts;
				};
				FCurveEditorOutlinePath Outer, Inner;
				Outer.Points = Shifted(Rings.first); Outer.Color = kOuterColor; Outer.Label = LOCTEXT("OuterSkin", "outer skin"); Outer.LineWidth = 1.3f;
				Inner.Points = Shifted(Rings.second); Inner.Color = kInnerColor; Inner.Label = LOCTEXT("InnerSkin", "inner skin"); Inner.LineWidth = 1.3f;
				Paths.Add(Outer);
				Paths.Add(Inner);
			}
			catch (const std::exception&) {}
			return Paths;
		}

		void Select(PeriodicNurbsCurve* Curve)
		{
			SelectedCurve = Curve;
			Rebuild();
		}

		void Rebuild()
		{
			// Fall back to the first guide if the selected one is gone.
			if (FindSelected() == nullptr && !Model->sections().empty())
			{
				SelectedCurve = Model->sections()[0].curve.get();
			}
			GuideSection* Sel = FindSelected();

			// Editor slot.
			if (Sel != nullptr)
			{
				FCurveEditorSlot Slot;
				Slot.Curve = Sel->curve.get();
				Slot.Color = kGuideColor;
				Slot.Label = LOCTEXT("GuideCurve", "guide curve");
				Slot.bClosedFill = true;
				Slot.bFreeX = true;
				Slot.Basis = ECurveBasisPlacement::Below;
				Editor->SetSlots({ Slot });
			}
			else
			{
				Editor->SetSlots({});
			}
			Editor->RequestRedraw();

			// Guides row: combo, station, Add, Remove.
			GuideOptions.Reset();
			const auto& Secs = Model->sections();
			for (int32 i = 0; i < (int32)Secs.size(); ++i) GuideOptions.Add(MakeShared<int32>(i));
			const int32 SelIdx = SelectedIndex();
			const int32 N = (int32)Secs.size();

			GuidesRow->ClearChildren();
			GuidesRow->AddSlot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 6, 0)
			[ SNew(STextBlock).Text(LOCTEXT("Editing", "Editing:")) ];
			GuidesRow->AddSlot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 10, 0)
			[
				SNew(SBox).MinDesiredWidth(130)
				[
					SNew(SComboBox<TSharedPtr<int32>>)
					.ToolTipText(LOCTEXT("EditingTip", "Which guide curve the graph above is editing"))
					.OptionsSource(&GuideOptions)
					.InitiallySelectedItem(GuideOptions.IsValidIndex(SelIdx) ? GuideOptions[SelIdx] : nullptr)
					.OnGenerateWidget_Lambda([this](TSharedPtr<int32> Item)
					{
						return SNew(STextBlock).Text(GuideLabel(*Item));
					})
					.OnSelectionChanged_Lambda([this](TSharedPtr<int32> Item, ESelectInfo::Type Info)
					{
						if (!Item.IsValid() || Info == ESelectInfo::Direct) return;
						const auto& S = Model->sections();
						if (*Item >= 0 && *Item < (int32)S.size() && S[*Item].curve.get() != SelectedCurve)
						{
							Select(S[*Item].curve.get());
						}
					})
					[
						SNew(STextBlock).Text_Lambda([this]()
						{
							const int32 I = SelectedIndex();
							return I == INDEX_NONE ? LOCTEXT("NoGuide", "(none)") : GuideLabel(I);
						})
					]
				]
			];
			GuidesRow->AddSlot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 6, 0)
			[ SNew(STextBlock).Text(LOCTEXT("Station", "Station:")) ];
			GuidesRow->AddSlot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 10, 0)
			[
				SNew(SAerotelicNumericEntry)
				.ToolTipText(LOCTEXT("StationTip", "Where along the body this guide curve applies (x/L)"))
				.IsEnabled(Sel != nullptr)
				.Value_Lambda([this]() { GuideSection* S = FindSelected(); return S ? S->station : 0.0; })
				.MinValue(0.0).MaxValue(1.0).Delta(0.05).LiveUpdate(false)
				.OnValueCommitted_Lambda([this](double V, ETextCommit::Type)
				{
					GuideSection* S = FindSelected();
					if (S == nullptr || FMath::IsNearlyEqual(S->station, V)) return;
					S->station = V;
					Model->ResortSections();
					Rebuild();
					NotifyChanged();
				})
			];
			GuidesRow->AddSlot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 4, 0)
			[
				SNew(SButton).Text(LOCTEXT("Add", "Add"))
				.ToolTipText(LOCTEXT("AddTip", "Add a guide curve in the largest gap along the body, copied from its nearest neighbour"))
				.IsEnabled(N < kLoftedNacelleMaxSections)
				.OnClicked_Lambda([this]()
				{
					try
					{
						Model->AddSection(NextStation());
					}
					catch (const std::exception& e)
					{
						FMessageDialog::Open(EAppMsgType::Ok, FText::FromString(UTF8_TO_TCHAR(e.what())), LOCTEXT("GuidesTitle", "Guide curves"));
						return FReply::Handled();
					}
					// The new section is the one that was not present before.
					PeriodicNurbsCurve* NewCurve = nullptr;
					for (auto& S : Model->sections()) if (!Known.Contains(S.curve.get())) NewCurve = S.curve.get();
					Select(NewCurve ? NewCurve : SelectedCurve);
					NotifyChanged();
					return FReply::Handled();
				})
			];
			GuidesRow->AddSlot().AutoWidth().VAlign(VAlign_Center)
			[
				SNew(SButton).Text(LOCTEXT("Remove", "Remove"))
				.ToolTipText(LOCTEXT("RemoveTip", "Remove the selected guide curve"))
				.IsEnabled(N > 1)
				.OnClicked_Lambda([this]()
				{
					const int32 Index = SelectedIndex();
					if (Index == INDEX_NONE) return FReply::Handled();
					try
					{
						Model->RemoveSection(Index);
					}
					catch (const std::exception& e)
					{
						FMessageDialog::Open(EAppMsgType::Ok, FText::FromString(UTF8_TO_TCHAR(e.what())), LOCTEXT("GuidesTitle", "Guide curves"));
						return FReply::Handled();
					}
					const auto& S = Model->sections();
					Select(S.empty() ? nullptr : S[FMath::Min(Index, (int32)S.size() - 1)].curve.get());
					NotifyChanged();
					return FReply::Handled();
				})
			];
			// Remember the current curves so Add can tell which one is new.
			Known.Reset();
			for (auto& S : Model->sections()) Known.Add(S.curve.get());
			NextStationBefore = NextStation();

			// Bottom controls: Length, Degree, Reset Knots, Reset to Circle...
			FSimpleDelegate OnCurve = FSimpleDelegate::CreateSP(this, &SGuideCurvesTab::NotifyChanged);
			TArray<TSharedRef<SWidget>> Leading;
			if (LengthField.IsValid()) Leading.Add(LengthField.ToSharedRef());
			Leading.Add(MakeDegreeSpin(LOCTEXT("Degree", "Degree:"), [this]() -> NurbsCurve* { GuideSection* S = FindSelected(); return S ? S->curve.get() : nullptr; },
				Editor, OnCurve, LOCTEXT("PolyDegreeTip", "Curve degree  (1 = polygon, 2 = quadratic, 3 = cubic ...).")));
			TArray<TSharedRef<SWidget>> Trailing;
			Trailing.Add(MakeControlButton(LOCTEXT("ResetKnots", "Reset Knots"), LOCTEXT("ResetKnotsTip", "Put the knots back at their default spacing (points unchanged)"),
				FOnClicked::CreateLambda([this]()
				{
					if (GuideSection* S = FindSelected())
					{
						S->curve->ResetKnotsPublic();
						Editor->RequestRedraw();
						NotifyChanged();
					}
					return FReply::Handled();
				})));
			Trailing.Add(MakeControlButton(LOCTEXT("ResetCircle", "Reset to Circle..."),
				LOCTEXT("ResetCircleTip", "A round ring, at a radius you type in units of the nacelle length.\nStarts from the radius this guide already averages, so accepting it\nunchanged keeps the body's size and only rounds off its shape."),
				FOnClicked::CreateLambda([this]()
				{
					GuideSection* S = FindSelected();
					if (S == nullptr) return FReply::Handled();
					const double Length = FMath::Max(Model->length, 1e-9);
					// The radius this guide already averages.
					Mat Pts = S->curve->Sample(721);
					double Sum = 0.0;
					for (int i = 0; i + 1 < Pts.rows(); ++i) Sum += FMath::Sqrt(Pts(i, 0) * Pts(i, 0) + Pts(i, 1) * Pts(i, 1));
					double Current = Pts.rows() > 1 ? Sum / (Pts.rows() - 1) / Length : 0.0;
					if (!(Current > 0.0)) Current = 0.25;
					TOptional<double> R = PromptCircleRadius(Current, Length, kRadiusPrompt);
					if (!R.IsSet()) return FReply::Handled();
					try
					{
						ResetCurveFromSpec(*S->curve, DefaultSectionCurveSpec(R.GetValue() * Length));
					}
					catch (const std::exception& e)
					{
						FMessageDialog::Open(EAppMsgType::Ok, FText::FromString(FString::Printf(TEXT("That radius was rejected:\n%s"), UTF8_TO_TCHAR(e.what()))));
						return FReply::Handled();
					}
					Editor->RequestRedraw();
					NotifyChanged();
					return FReply::Handled();
				})));
			ControlsHost->SetContent(MakeControlsRow(Leading, Trailing));
		}

		FText GuideLabel(int32 Index) const
		{
			const auto& S = Model->sections();
			if (Index < 0 || Index >= (int32)S.size()) return FText::GetEmpty();
			return FText::FromString(FString::Printf(TEXT("%d  ·  x/L %.3f"), Index + 1, S[Index].station));
		}

		TSet<PeriodicNurbsCurve*> Known;
		double NextStationBefore = 0.5;
	};

	const FText kHelp = LOCTEXT("Help",
		"HOW THIS NACELLE IS MADE\n"
		"This is the second nacelle parameterisation. It carries the cowl wall on a mean line you draw: the Centreline is its route against x/L (straight by default); "
		"the Guide curves give the radius at each face angle of the revolved ring (several at different x/L are blended); the Skins say how far outside and inside the mean line each skin sits, "
		"the gap between them being the wall. Both ends of every side-view curve are held at x/L = 0 and 1.\n\n"
		"CENTRELINE AND SKINS\n"
		"End points move vertically only; other points move freely. Double-click near a curve to add a point; right-click a point to set its weight or remove it. "
		"On Skins the outer curve cannot go below the mean line and the inner above it. Both on the mean line at an end closes the wall to a sharp edge; a gap leaves a blunt end.\n\n"
		"GUIDE CURVES\n"
		"Pick the guide to edit from the box. Add puts one in the largest gap along the body, copied from its nearest neighbour; Station moves the selected one; the last cannot be removed. "
		"A guide is used at the size you draw it. The two outlines drawn over it are the skins as they will be built; the faint ones are the other guides. "
		"Two guides give a straight body; three or more become the control points of a blend, so the body is pulled towards interior guides rather than passing through them. "
		"Put the first guide at x/L = 0 and the last at 1 for a crease-free body, and keep their seams at the same place around the ring.\n\n"
		"KNOT GRAPHS\n"
		"Drag the green knots between their neighbours; push one onto a neighbour to repeat it. Grey knots are the clamped ends of a side-view curve and the seam of a guide. "
		"Right-click the line to insert a knot without changing the shape; right-click a knot to raise its multiplicity or delete it.\n\n"
		"CHEVRONS\n"
		"Notches around the trailing edge, spaced evenly by arc length with the first tip at the top. Depth is typed in directly and marked by a dashed line on the Skins tab; 0 is a plain trailing edge. "
		"The tip angle read-out is what the other nacelle parameterisation asks for instead of a depth; it is shown so a body drawn either way can be matched to the other.\n\n"
		"VIEW\n"
		"The wheel zooms about the cursor; the right-click menu's Fit view to contents fits everything; nothing else moves the view.");
}

void BuildLoftedNacelleDesignContent(AAerotelicEntityActor* Entity, FAerotelicDesignContent& Out)
{
	if (Entity == nullptr || Entity->GetEntityType() != project_yaml::EntityType::LoftedNacelle)
	{
		return;
	}

	LoftedNacelleModel* Model = Entity->GetModelAs<LoftedNacelleModel>();
	TWeakObjectPtr<AAerotelicEntityActor> WeakEntity = Entity;
	FSimpleDelegate Rebuild = FSimpleDelegate::CreateLambda([WeakEntity]()
	{
		if (WeakEntity.IsValid()) WeakEntity->RefreshProxy();
	});

	const double InvLength = 1.0 / FMath::Max(Model->length, 1e-9);

	// ---- Centreline ----
	FCurveEditorSlot CentrelineSlot;
	CentrelineSlot.Curve = &Model->centreline_curve_mut();
	CentrelineSlot.Color = kCentrelineColor;
	CentrelineSlot.Label = LOCTEXT("Centreline", "centreline");
	CentrelineSlot.Basis = ECurveBasisPlacement::Below;
	TSharedRef<SCurveEditor> CentrelineEditor = SNew(SCurveEditor)
		.XLabel(LOCTEXT("XL", "Nacelle fraction (x/L)"))
		.YLabel(LOCTEXT("OffsetL", "Vertical offset (y/L)"))
		.Title(LOCTEXT("CentrelineTitle", "Centreline  ·  side view  ·  inlet and trailing edge pinned in x/L"))
		.DefaultYMax(0.1)
		.XScale(1.0).YScale(InvLength).EqualAspect(true);
	CentrelineEditor->SetSlots({ CentrelineSlot });
	// Body outline: the outer skin silhouette and the mean line, read-only.
	CentrelineEditor->SetOutlineProvider(FOnGetOutlinePaths::CreateLambda([Model]()
	{
		TArray<FCurveEditorOutlinePath> Paths;
		try
		{
			Mat Z = Model->OutlineZs(160);
			FCurveEditorOutlinePath Skin;
			for (int i = 0; i < Z.rows(); ++i) Skin.Points.Add(FVector2D(Z(i, 0), Z(i, 1)));
			for (int i = Z.rows() - 1; i >= 0; --i) Skin.Points.Add(FVector2D(Z(i, 0), Z(i, 2)));
			Skin.Color = kSkinColor; Skin.Label = LOCTEXT("OuterSkin", "outer skin"); Skin.bClosed = true; Skin.LineWidth = 1.0f;
			Paths.Add(Skin);
			FCurveEditorOutlinePath MeanTop, MeanBot;
			for (int i = 0; i < Z.rows(); ++i)
			{
				MeanTop.Points.Add(FVector2D(Z(i, 0), Z(i, 3)));
				MeanBot.Points.Add(FVector2D(Z(i, 0), Z(i, 4)));
			}
			MeanTop.Color = kMeanColor; MeanTop.Label = LOCTEXT("MeanLine", "mean line"); MeanTop.LineWidth = 1.0f;
			MeanBot.Color = kMeanColor; MeanBot.LineWidth = 1.0f;
			Paths.Add(MeanTop);
			Paths.Add(MeanBot);
		}
		catch (const std::exception&) {}
		return Paths;
	}));

	// ---- Skins ----
	FCurveEditorSlot OuterSlot, InnerSlot;
	OuterSlot.Curve = &Model->outer_curve_mut();
	OuterSlot.Color = kOuterColor;
	OuterSlot.Label = LOCTEXT("OuterSkin", "outer skin");
	OuterSlot.YClampLo = 0.0;
	OuterSlot.Basis = ECurveBasisPlacement::Above;
	InnerSlot.Curve = &Model->inner_curve_mut();
	InnerSlot.Color = kInnerColor;
	InnerSlot.Label = LOCTEXT("InnerSkin", "inner skin");
	InnerSlot.YClampHi = 0.0;
	InnerSlot.Basis = ECurveBasisPlacement::Below;
	TSharedRef<SCurveEditor> SkinsEditor = SNew(SCurveEditor)
		.XLabel(LOCTEXT("XL", "Nacelle fraction (x/L)"))
		.YLabel(LOCTEXT("SkinOffsetL", "Offset from mean line (y/L)"))
		.Title(LOCTEXT("SkinsTitle", "Skins  ·  outer and inner offsets  ·  the gap between them is the wall"))
		.DefaultYMax(0.1)
		.XScale(1.0).YScale(InvLength).EqualAspect(true)
		.SupportsUserLinks(true)
		.LeadingName(LOCTEXT("LipEnd", "lip")).TrailingName(LOCTEXT("TeEnd", "trailing-edge"));
	SkinsEditor->SetSlots({ OuterSlot, InnerSlot });
	// Chevron-valley marker across the whole graph.
	SkinsEditor->SetOutlineProvider(FOnGetOutlinePaths::CreateLambda([Model]()
	{
		TArray<FCurveEditorOutlinePath> Paths;
		const double Depth = Model->chevron_count > 0 ? Model->chevron_depth : 0.0;
		if (Depth <= 0.0 || Depth >= Model->length) return Paths;
		const double X = 1.0 - Depth / Model->length;
		double Lo = 0.0, Hi = 0.0;
		Mat In = Model->inner_curve().Sample(200), Ou = Model->outer_curve().Sample(200);
		for (int i = 0; i < In.rows(); ++i) Lo = FMath::Min(Lo, In(i, 1));
		for (int i = 0; i < Ou.rows(); ++i) Hi = FMath::Max(Hi, Ou(i, 1));
		FCurveEditorOutlinePath P;
		P.Points = { FVector2D(X, Lo), FVector2D(X, Hi) };
		P.Color = kValleyColor; P.Label = LOCTEXT("Valleys", "chevron valleys"); P.LineWidth = 1.2f;
		Paths.Add(P);
		return Paths;
	}));

	// ---- Chevrons read-outs ----
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

	// Guide-curves tab (needs the shared change router, built below).
	TSharedPtr<SGuideCurvesTab> GuideTab;

	// Every curve/station change: chevron read-outs, every tab redraws (each
	// draws another tab's shape), then the solid rebuilds.
	TSharedRef<TSharedPtr<SGuideCurvesTab>> GuideTabRef = MakeShared<TSharedPtr<SGuideCurvesTab>>();
	FSimpleDelegate OnCurveChanged = FSimpleDelegate::CreateLambda([RefreshChevronReadouts, CentrelineEditor, SkinsEditor, GuideTabRef, Rebuild]()
	{
		RefreshChevronReadouts();
		CentrelineEditor->RequestRedraw();
		SkinsEditor->RequestRedraw();
		if (GuideTabRef->IsValid()) (*GuideTabRef)->Redraw();
		Rebuild.ExecuteIfBound();
	});
	// The editors' own OnChanged is bound after construction via SetOnChanged?
	// SCurveEditor takes it at construction; rebuild them with the router.
	FSimpleDelegate OnChevronsChanged = FSimpleDelegate::CreateLambda([RefreshChevronReadouts, SkinsEditor, Rebuild]()
	{
		RefreshChevronReadouts();
		SkinsEditor->RequestRedraw();
		Rebuild.ExecuteIfBound();
	});
	FSimpleDelegate OnLength = FSimpleDelegate::CreateLambda([Model, CentrelineEditor, SkinsEditor, GuideTabRef, OnChevronsChanged]()
	{
		const double Inv = 1.0 / FMath::Max(Model->length, 1e-9);
		CentrelineEditor->SetScales(1.0, Inv);
		SkinsEditor->SetScales(1.0, Inv);
		if (GuideTabRef->IsValid()) (*GuideTabRef)->SetScales(Inv);
		OnChevronsChanged.ExecuteIfBound();
	});
	auto LengthField = [Model, OnLength]() { return MakeLengthField(Model, OnLength); };

	CentrelineEditor->SetOnChanged(OnCurveChanged);
	SkinsEditor->SetOnChanged(OnCurveChanged);

	GuideTab = SNew(SGuideCurvesTab, Model).OnChanged(OnCurveChanged).LengthField(LengthField());
	*GuideTabRef = GuideTab;

	auto CurveTab = [&](TSharedRef<SCurveEditor> Editor, TArray<TSharedRef<SWidget>> Leading, TArray<TSharedRef<SWidget>> Trailing, TSharedPtr<SWidget> Extra = nullptr)
	{
		Leading.Insert(LengthField(), 0);
		return SNew(SVerticalBox)
			+ SVerticalBox::Slot().FillHeight(1.0f) [ Editor ]
			+ SVerticalBox::Slot().AutoHeight().Padding(4, 2) [ MakeControlsRow(Leading, Trailing) ]
			+ SVerticalBox::Slot().AutoHeight().Padding(4, 2) [ Extra.IsValid() ? Extra.ToSharedRef() : SNullWidget::NullWidget ];
	};
	auto ImageBar = [Model](TSharedRef<SCurveEditor> Editor, const TCHAR* Key)
	{
		return SNew(SReferenceImageBar).Editor(Editor).Images(&Model->reference_images).Key(Key).ViewLabel(LOCTEXT("SideView", "side view"));
	};

	TSharedRef<SWidget> CentrelineTab = CurveTab(CentrelineEditor,
		{ MakeDegreeSpin(LOCTEXT("Degree", "Degree:"), [Model]() { return &Model->centreline_curve_mut(); }, CentrelineEditor, OnCurveChanged) },
		{
			MakeControlButton(LOCTEXT("ResetKnots", "Reset Knots"), LOCTEXT("ResetKnotsTip", "Put the knots back at their default spacing (points unchanged)"),
				FOnClicked::CreateLambda([Model, CentrelineEditor, OnCurveChanged]()
				{
					Model->centreline_curve_mut().ResetKnotsPublic();
					CentrelineEditor->RequestRedraw();
					OnCurveChanged.ExecuteIfBound();
					return FReply::Handled();
				})),
			MakeControlButton(LOCTEXT("ResetStraight", "Reset to Straight"), LOCTEXT("ResetStraightTip", "A straight centreline on the nacelle axis"),
				FOnClicked::CreateLambda([Model, CentrelineEditor, OnCurveChanged]()
				{
					const LoftedNacelleSpec Def;
					ResetCurveFromSpec(Model->centreline_curve_mut(), Def.centreline_curve);
					CentrelineEditor->RequestRedraw();
					OnCurveChanged.ExecuteIfBound();
					return FReply::Handled();
				})),
		},
		ImageBar(CentrelineEditor, TEXT("centreline")));

	TSharedRef<SWidget> SkinsTab = CurveTab(SkinsEditor,
		{
			MakeDegreeSpin(LOCTEXT("OuterDegree", "Outer degree:"), [Model]() { return &Model->outer_curve_mut(); }, SkinsEditor, OnCurveChanged),
			MakeDegreeSpin(LOCTEXT("InnerDegree", "Inner degree:"), [Model]() { return &Model->inner_curve_mut(); }, SkinsEditor, OnCurveChanged),
		},
		{
			MakeControlButton(LOCTEXT("ResetKnots", "Reset Knots"), LOCTEXT("ResetKnotsBothTip", "Put both skins' knots back at their default spacing (points unchanged)"),
				FOnClicked::CreateLambda([Model, SkinsEditor, OnCurveChanged]()
				{
					Model->outer_curve_mut().ResetKnotsPublic();
					Model->inner_curve_mut().ResetKnotsPublic();
					SkinsEditor->RequestRedraw();
					OnCurveChanged.ExecuteIfBound();
					return FReply::Handled();
				})),
			MakeControlButton(LOCTEXT("ResetDefault", "Reset to Default"), LOCTEXT("ResetDefaultTip", "The default wall at the current length"),
				FOnClicked::CreateLambda([Model, SkinsEditor, OnCurveChanged]()
				{
					const double K = Model->length / kLoftedNacelleDefaultLength;
					const LoftedNacelleSpec Def;
					ResetCurveFromSpec(Model->outer_curve_mut(), Def.outer_curve, FVector2D(1.0, K));
					ResetCurveFromSpec(Model->inner_curve_mut(), Def.inner_curve, FVector2D(1.0, K));
					SkinsEditor->RequestRedraw();
					OnCurveChanged.ExecuteIfBound();
					return FReply::Handled();
				})),
		},
		ImageBar(SkinsEditor, TEXT("skins")));

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
							.MinValue(0).MaxValue(kLoftedNacelleMaxChevrons).MinSliderValue(0).MaxSliderValue(kLoftedNacelleMaxChevrons).Delta(1)
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
	Tabs.Add({ LOCTEXT("CentrelineTab", "Centreline"), CentrelineTab });
	Tabs.Add({ LOCTEXT("GuideTab", "Guide curves"), GuideTab.ToSharedRef() });
	Tabs.Add({ LOCTEXT("SkinsTab", "Skins"), SkinsTab });
	Tabs.Add({ LOCTEXT("ChevronsTab", "Chevrons"), ChevronsTab });
	Tabs.Add({ LOCTEXT("HelpTab", "Help"), MakeHelpTab(kHelp) });

	Out.Title = FText::Format(LOCTEXT("Title", "Lofted Nacelle -- {0}"), FText::FromString(Entity->EntityName));
	Out.Widget = SNew(SAerotelicTabStrip, Tabs);
}

#undef LOCTEXT_NAMESPACE
