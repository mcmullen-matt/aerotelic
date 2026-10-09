#include "LoftedNacelleDialog.h"
#include "EntityDesignContent.h"
#include "Widgets/TabStrip.h"
#include "Widgets/NumericEntry.h"
#include "Widgets/CurveEditor.h"
#include "Widgets/PromptDialogs.h"

#include "Widgets/Layout/SBorder.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"

#include "Entity/AerotelicEntityActor.h"
#include "aerotelic/lofted_nacelle_model.hpp"

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

	// The Guide-curves tab: a row of "N" selector buttons (tracking the
	// selected section by its curve's pointer identity, not index -- indices
	// shift whenever sections re-sort by station, same note as
	// LoftedNacelleModel::FindSectionByCurve's own comment), a station field,
	// Add/Remove, and one curve editor rebound to whichever section is
	// selected.
	class SGuideCurvesTab : public SCompoundWidget
	{
	public:
		SLATE_BEGIN_ARGS(SGuideCurvesTab) {}
			SLATE_EVENT(FSimpleDelegate, OnChanged)
		SLATE_END_ARGS()

		void Construct(const FArguments& InArgs, LoftedNacelleModel* InModel)
		{
			Model = InModel;
			OnChangedDelegate = InArgs._OnChanged;
			if (!Model->sections().empty())
			{
				SelectedCurve = Model->sections()[0].curve.get();
			}
			ChildSlot
			[
				SAssignNew(Root, SVerticalBox)
			];
			Rebuild();
		}

	private:
		LoftedNacelleModel* Model = nullptr;
		FSimpleDelegate OnChangedDelegate;
		TSharedPtr<SVerticalBox> Root;
		PeriodicNurbsCurve* SelectedCurve = nullptr;

		void NotifyChanged() { OnChangedDelegate.ExecuteIfBound(); }

		GuideSection* FindSelected() const
		{
			if (SelectedCurve == nullptr) return nullptr;
			return Model->FindSectionByCurve(SelectedCurve);
		}

		void Rebuild()
		{
			Root->ClearChildren();

			TSharedRef<SHorizontalBox> Selector = SNew(SHorizontalBox);
			const auto& Sections = Model->sections();
			for (int32 i = 0; i < (int32)Sections.size(); ++i)
			{
				PeriodicNurbsCurve* Curve = Sections[i].curve.get();
				const bool bSel = (Curve == SelectedCurve);
				Selector->AddSlot().AutoWidth().Padding(2, 0)
				[
					SNew(SButton)
					.Text(FText::AsNumber(i + 1))
					.ButtonColorAndOpacity(bSel ? FLinearColor(0.3f, 0.5f, 0.8f) : FLinearColor::White)
					.OnClicked_Lambda([this, Curve]() { SelectedCurve = Curve; Rebuild(); return FReply::Handled(); })
				];
			}
			Root->AddSlot().AutoHeight().Padding(0, 0, 0, 4)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0)
				[ SNew(STextBlock).Text(NSLOCTEXT("LoftedNacelleDialog", "Editing", "Editing:")) ]
				+ SHorizontalBox::Slot().AutoWidth()
				[ Selector ]
			];

			GuideSection* Sel = FindSelected();
			if (Sel == nullptr)
			{
				Root->AddSlot().AutoHeight()
				[ SNew(STextBlock).Text(NSLOCTEXT("LoftedNacelleDialog", "NoSections", "No guide curves.")) ];
				return;
			}

			Root->AddSlot().AutoHeight().Padding(0, 0, 0, 4)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0)
				[ SNew(STextBlock).Text(NSLOCTEXT("LoftedNacelleDialog", "Station", "Station (x/L):")) ]
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 8, 0)
				[
					SNew(SAerotelicNumericEntry)
					.Value_Lambda([this]() { GuideSection* S = FindSelected(); return S ? S->station : 0.0; })
					.MinValue(0.0).MaxValue(1.0).Delta(0.05).LiveUpdate(false)
					.OnValueCommitted_Lambda([this](double V, ETextCommit::Type)
					{
						if (GuideSection* S = FindSelected()) { S->station = V; Model->ResortSections(); }
						NotifyChanged();
						Rebuild();
					})
				]
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 4, 0)
				[
					SNew(SButton).Text(NSLOCTEXT("LoftedNacelleDialog", "Add", "Add"))
					.OnClicked_Lambda([this]()
					{
						GuideSection* S = FindSelected();
						const double Station = S ? FMath::Clamp(S->station + 0.1, 0.0, 1.0) : 0.5;
						try
						{
							Model->AddSection(Station);
							SelectedCurve = Model->sections().back().curve.get();
						}
						catch (const std::exception&) {}
						NotifyChanged();
						Rebuild();
						return FReply::Handled();
					})
				]
				+ SHorizontalBox::Slot().AutoWidth()
				[
					SNew(SButton).Text(NSLOCTEXT("LoftedNacelleDialog", "Remove", "Remove"))
					.IsEnabled(Model->sections().size() > 1)
					.OnClicked_Lambda([this]()
					{
						const auto& Secs = Model->sections();
						for (int32 i = 0; i < (int32)Secs.size(); ++i)
						{
							if (Secs[i].curve.get() == SelectedCurve)
							{
								try { Model->RemoveSection(i); } catch (const std::exception&) {}
								break;
							}
						}
						SelectedCurve = Model->sections().empty() ? nullptr : Model->sections()[0].curve.get();
						NotifyChanged();
						Rebuild();
						return FReply::Handled();
					})
				]
			];

			FCurveEditorSlot Slot;
			Slot.Curve = Sel->curve.get();
			Slot.Color = FLinearColor(0.31f, 0.78f, 0.5f);
			Slot.bClosedFill = true;
			Slot.bFreeX = true;
			Slot.Basis = ECurveBasisPlacement::Below;
			TSharedRef<SCurveEditor> Editor = SNew(SCurveEditor)
				.XLabel(NSLOCTEXT("LoftedNacelleDialog", "LateralL", "Lateral (y/L)"))
				.YLabel(NSLOCTEXT("LoftedNacelleDialog", "VerticalL", "Vertical (z/L)"))
				.Title(NSLOCTEXT("LoftedNacelleDialog", "GuideTitle", "Guide curve"))
				.XScale(1.0 / FMath::Max(Model->length, 1e-9)).YScale(1.0 / FMath::Max(Model->length, 1e-9)).EqualAspect(true)
				.OnChanged(OnChangedDelegate);
			Editor->SetSlots({ Slot });
			Root->AddSlot().FillHeight(1.0f) [ Editor ];
		}
	};
}

void BuildLoftedNacelleDesignContent(AAerotelicEntityActor* Entity, FAerotelicDesignContent& Out)
{
	if (Entity == nullptr || Entity->GetEntityType() != project_yaml::EntityType::LoftedNacelle)
	{
		return;
	}

	LoftedNacelleModel* Model = Entity->GetModelAs<LoftedNacelleModel>();
	TWeakObjectPtr<AAerotelicEntityActor> WeakEntity = Entity;
	FSimpleDelegate OnChanged = FSimpleDelegate::CreateLambda([WeakEntity]()
	{
		if (WeakEntity.IsValid()) WeakEntity->RefreshProxy();
	});

	// ---- Centreline ----
	FCurveEditorSlot CentrelineSlot;
	CentrelineSlot.Curve = &Model->centreline_curve_mut();
	CentrelineSlot.Color = FLinearColor(0.63f, 0.31f, 0.82f);
	CentrelineSlot.Basis = ECurveBasisPlacement::Below;
	TSharedRef<SCurveEditor> CentrelineEditor = SNew(SCurveEditor)
		.XLabel(NSLOCTEXT("LoftedNacelleDialog", "XL", "Nacelle fraction (x/L)"))
		.YLabel(NSLOCTEXT("LoftedNacelleDialog", "OffsetL", "Vertical offset (y/L)"))
		.Title(NSLOCTEXT("LoftedNacelleDialog", "CentrelineTitle", "Centreline"))
		.XScale(1.0).YScale(1.0 / FMath::Max(Model->length, 1e-9)).EqualAspect(true)
		.OnChanged(OnChanged);
	CentrelineEditor->SetSlots({ CentrelineSlot });

	// ---- Guide curves ----
	TSharedRef<SGuideCurvesTab> GuideTab = SNew(SGuideCurvesTab, Model).OnChanged(OnChanged);

	// ---- Skins ----
	FCurveEditorSlot OuterSlot, InnerSlot;
	OuterSlot.Curve = &Model->outer_curve_mut();
	OuterSlot.Color = FLinearColor(0.23f, 0.62f, 0.84f);
	OuterSlot.YClampLo = 0.0;
	OuterSlot.Basis = ECurveBasisPlacement::Above;
	InnerSlot.Curve = &Model->inner_curve_mut();
	InnerSlot.Color = FLinearColor(0.84f, 0.54f, 0.23f);
	InnerSlot.YClampHi = 0.0;
	InnerSlot.Basis = ECurveBasisPlacement::Below;
	TSharedRef<SCurveEditor> SkinsEditor = SNew(SCurveEditor)
		.XLabel(NSLOCTEXT("LoftedNacelleDialog", "XL", "Nacelle fraction (x/L)"))
		.YLabel(NSLOCTEXT("LoftedNacelleDialog", "SkinOffsetL", "Offset from mean line (y/L)"))
		.Title(NSLOCTEXT("LoftedNacelleDialog", "SkinsTitle", "Skins"))
		.XScale(1.0).YScale(1.0 / FMath::Max(Model->length, 1e-9)).EqualAspect(true)
		.OnChanged(OnChanged);
	SkinsEditor->SetSlots({ OuterSlot, InnerSlot });

	// ---- Chevrons ----
	TSharedRef<STextBlock> TipAngleLabel = SNew(STextBlock);
	TSharedRef<STextBlock> TePerimeterLabel = SNew(STextBlock);
	TFunction<void()> RefreshChevronReadouts = [Model, TipAngleLabel, TePerimeterLabel]()
	{
		TipAngleLabel->SetText(Model->chevron_count > 0 ? FText::AsNumber(Model->ChevronTipAngle()) : FText::FromString(TEXT("--")));
		TePerimeterLabel->SetText(FText::AsNumber(Model->TePerimeter()));
	};
	TSharedRef<SWidget> ChevronsTab = SNew(SBorder).Padding(8)
	[
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[ FormRow(NSLOCTEXT("LoftedNacelleDialog", "ChevronCount", "Number of chevrons"),
			SNew(SAerotelicNumericEntry).Value_Lambda([Model]() { return (double)Model->chevron_count; })
				.MinValue(0).MaxValue((double)kLoftedNacelleMaxChevrons).Delta(1.0).LiveUpdate(false)
				.OnValueCommitted_Lambda([Model, OnChanged, RefreshChevronReadouts](double V, ETextCommit::Type)
				{
					Model->chevron_count = FMath::RoundToInt(V);
					RefreshChevronReadouts();
					OnChanged.ExecuteIfBound();
				})) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ FormRow(NSLOCTEXT("LoftedNacelleDialog", "ChevronDepth", "Chevron depth"),
			SNew(SAerotelicNumericEntry).Value_Lambda([Model]() { return Model->chevron_depth; })
				.MinValue(0.0).MaxValue(1e9).Delta(0.05).LiveUpdate(false)
				.OnValueCommitted_Lambda([Model, OnChanged, RefreshChevronReadouts](double V, ETextCommit::Type)
				{
					Model->chevron_depth = V;
					RefreshChevronReadouts();
					OnChanged.ExecuteIfBound();
				})) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 2)
		[ FormRow(NSLOCTEXT("LoftedNacelleDialog", "TipAngle", "Tip angle"), TipAngleLabel) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ FormRow(NSLOCTEXT("LoftedNacelleDialog", "TePerimeter", "Trailing-edge perimeter"), TePerimeterLabel) ]
	];
	RefreshChevronReadouts();

	TSharedRef<SWidget> LengthRow = FormRow(NSLOCTEXT("LoftedNacelleDialog", "Length", "Length"),
		SNew(SAerotelicNumericEntry).Value_Lambda([Model]() { return Model->length; })
			.MinValue(1e-9).MaxValue(1e9).Delta(0.1).LiveUpdate(false)
			.OnValueCommitted_Lambda([Model, OnChanged, CentrelineEditor, SkinsEditor](double V, ETextCommit::Type)
			{
				Model->SetLength(V);
				const double Inv = 1.0 / FMath::Max(Model->length, 1e-9);
				CentrelineEditor->SetScales(1.0, Inv);
				SkinsEditor->SetScales(1.0, Inv);
				OnChanged.ExecuteIfBound();
			}));

	TArray<FAerotelicTab> Tabs;
	Tabs.Add({ NSLOCTEXT("LoftedNacelleDialog", "CentrelineTab", "Centreline"), CentrelineEditor });
	Tabs.Add({ NSLOCTEXT("LoftedNacelleDialog", "GuideTab", "Guide curves"), GuideTab });
	Tabs.Add({ NSLOCTEXT("LoftedNacelleDialog", "SkinsTab", "Skins"), SkinsEditor });
	Tabs.Add({ NSLOCTEXT("LoftedNacelleDialog", "ChevronsTab", "Chevrons"), ChevronsTab });

	Out.Title = FText::Format(NSLOCTEXT("LoftedNacelleDialog", "Title", "Lofted Nacelle -- {0}"), FText::FromString(Entity->EntityName));
	Out.Widget =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(8) [ LengthRow ]
		+ SVerticalBox::Slot().FillHeight(1.0f) [ SNew(SAerotelicTabStrip, Tabs) ];
}
