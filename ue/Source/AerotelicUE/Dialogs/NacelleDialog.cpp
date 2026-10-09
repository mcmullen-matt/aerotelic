#include "NacelleDialog.h"
#include "EntityDesignContent.h"
#include "Widgets/TabStrip.h"
#include "Widgets/NumericEntry.h"
#include "Widgets/CurveEditor.h"
#include "Widgets/PromptDialogs.h"

#include "Widgets/Layout/SBorder.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Misc/MessageDialog.h"

#include "Entity/AerotelicEntityActor.h"
#include "aerotelic/nacelle_model.hpp"

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
}

void BuildNacelleDesignContent(AAerotelicEntityActor* Entity, FAerotelicDesignContent& Out)
{
	if (Entity == nullptr || Entity->GetEntityType() != project_yaml::EntityType::Nacelle)
	{
		return;
	}

	NacelleModel* Model = Entity->GetModelAs<NacelleModel>();
	TWeakObjectPtr<AAerotelicEntityActor> WeakEntity = Entity;
	FSimpleDelegate OnChanged = FSimpleDelegate::CreateLambda([WeakEntity]()
	{
		if (WeakEntity.IsValid()) WeakEntity->RefreshProxy();
	});

	// ---- Side view: outer/inner skins, lip + trailing-edge poles shared rigidly ----
	FCurveEditorSlot OuterSlot, InnerSlot;
	OuterSlot.Curve = &Model->upper_curve_mut();
	OuterSlot.Color = FLinearColor(0.23f, 0.62f, 0.84f);
	OuterSlot.Label = NSLOCTEXT("NacelleDialog", "Outer", "outer skin");
	OuterSlot.Basis = ECurveBasisPlacement::Above;
	InnerSlot.Curve = &Model->lower_curve_mut();
	InnerSlot.Color = FLinearColor(0.84f, 0.54f, 0.23f);
	InnerSlot.Label = NSLOCTEXT("NacelleDialog", "Inner", "inner skin");
	InnerSlot.Basis = ECurveBasisPlacement::Below;

	TSharedRef<SCurveEditor> SideEditor = SNew(SCurveEditor)
		.XLabel(NSLOCTEXT("NacelleDialog", "XL", "Nacelle fraction (x/L)"))
		.YLabel(NSLOCTEXT("NacelleDialog", "YL", "Radius from axis (y/L)"))
		.Title(NSLOCTEXT("NacelleDialog", "SideTitle", "Side view"))
		.XScale(1.0).YScale(1.0 / FMath::Max(Model->length, 1e-9)).EqualAspect(true)
		.OnChanged(OnChanged);
	SideEditor->SetSlots({ OuterSlot, InnerSlot });
	SideEditor->SetLinks({ { FIntPoint(0, 0), FIntPoint(1, 0) }, { FIntPoint(0, -1), FIntPoint(1, -1) } });

	// ---- Front view: closed, free-x lip ring ----
	FCurveEditorSlot FrontSlot;
	FrontSlot.Curve = &Model->front_curve_mut();
	FrontSlot.Color = FLinearColor(0.31f, 0.78f, 0.5f);
	FrontSlot.bClosedFill = true;
	FrontSlot.bFreeX = true;
	FrontSlot.Basis = ECurveBasisPlacement::Below;

	TSharedRef<SCurveEditor> FrontEditor = SNew(SCurveEditor)
		.XLabel(NSLOCTEXT("NacelleDialog", "LateralL", "Lateral (y/L)"))
		.YLabel(NSLOCTEXT("NacelleDialog", "VerticalL", "Vertical (z/L)"))
		.Title(NSLOCTEXT("NacelleDialog", "FrontTitle", "Front view (inlet lip)"))
		.XScale(1.0 / FMath::Max(Model->length, 1e-9)).YScale(1.0 / FMath::Max(Model->length, 1e-9)).EqualAspect(true)
		.OnChanged(OnChanged);
	FrontEditor->SetSlots({ FrontSlot });

	TSharedRef<SButton> ResetCircleButton = SNew(SButton)
		.Text(NSLOCTEXT("NacelleDialog", "ResetCircle", "Reset to Circle..."))
		.OnClicked_Lambda([Model, FrontEditor, OnChanged]()
		{
			const double Current = Model->LipRadius();
			TOptional<double> R = PromptForDouble(
				NSLOCTEXT("NacelleDialog", "ResetCircleTitle", "Reset to Circle"),
				NSLOCTEXT("NacelleDialog", "ResetCircleLabel", "Radius, as a fraction of the nacelle length (y/L):"),
				Current, 1e-9, 1e9, 0.01);
			if (R.IsSet())
			{
				try
				{
					Model->SetFrontCurve(DefaultFrontCurveSpec(R.GetValue() * Model->length));
				}
				catch (const std::exception& e)
				{
					FMessageDialog::Open(EAppMsgType::Ok,
						FText::FromString(FString::Printf(TEXT("That radius was rejected:\n%s"), UTF8_TO_TCHAR(e.what()))));
					return FReply::Handled();
				}
				FCurveEditorSlot NewSlot;
				NewSlot.Curve = &Model->front_curve_mut();
				NewSlot.Color = FLinearColor(0.31f, 0.78f, 0.5f);
				NewSlot.bClosedFill = true;
				NewSlot.bFreeX = true;
				NewSlot.Basis = ECurveBasisPlacement::Below;
				FrontEditor->SetSlots({ NewSlot });
				OnChanged.ExecuteIfBound();
			}
			return FReply::Handled();
		});

	TSharedRef<SWidget> FrontTab = SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(4) [ ResetCircleButton ]
		+ SVerticalBox::Slot().FillHeight(1.0f) [ FrontEditor ];

	// ---- Chevrons ----
	// Built before the refresh lambda so it can capture these by value (a
	// capture-by-reference to a function-local TSharedPtr would dangle the
	// moment OpenNacelleDialog returns -- the lambda outlives this stack
	// frame, kept alive by the SAerotelicNumericEntry widgets below).
	TSharedRef<STextBlock> ToothWidthLabel = SNew(STextBlock);
	TSharedRef<STextBlock> TipAngleLabel = SNew(STextBlock);
	TSharedRef<STextBlock> TePerimeterLabel = SNew(STextBlock);
	TFunction<void()> RefreshChevronReadouts = [Model, ToothWidthLabel, TipAngleLabel, TePerimeterLabel]()
	{
		ToothWidthLabel->SetText(FText::AsNumber(Model->ToothWidth()));
		TipAngleLabel->SetText(Model->chevron_count > 0 ? FText::AsNumber(Model->ChevronTipAngle()) : FText::FromString(TEXT("--")));
		TePerimeterLabel->SetText(FText::AsNumber(Model->TePerimeter()));
	};

	TSharedRef<SWidget> ChevronsTab = SNew(SBorder).Padding(8)
	[
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[ FormRow(NSLOCTEXT("NacelleDialog", "ChevronCount", "Number of chevrons"),
			SNew(SAerotelicNumericEntry).Value_Lambda([Model]() { return (double)Model->chevron_count; })
				.MinValue(0).MaxValue((double)kMaxChevrons).Delta(1.0).LiveUpdate(false)
				.OnValueCommitted_Lambda([Model, OnChanged, SideEditor, RefreshChevronReadouts](double V, ETextCommit::Type)
				{
					Model->chevron_count = FMath::RoundToInt(V);
					SideEditor->RequestRedraw();
					RefreshChevronReadouts();
					OnChanged.ExecuteIfBound();
				})) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ FormRow(NSLOCTEXT("NacelleDialog", "ChevronDepth", "Chevron depth"),
			SNew(SAerotelicNumericEntry).Value_Lambda([Model]() { return Model->chevron_depth; })
				.MinValue(0.0).MaxValue(1e9).Delta(0.05).LiveUpdate(false)
				.OnValueCommitted_Lambda([Model, OnChanged, SideEditor, RefreshChevronReadouts](double V, ETextCommit::Type)
				{
					Model->chevron_depth = V;
					SideEditor->RequestRedraw();
					RefreshChevronReadouts();
					OnChanged.ExecuteIfBound();
				})) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 2)
		[ FormRow(NSLOCTEXT("NacelleDialog", "ToothWidth", "Tooth width"), ToothWidthLabel) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ FormRow(NSLOCTEXT("NacelleDialog", "TipAngle", "Tip angle"), TipAngleLabel) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ FormRow(NSLOCTEXT("NacelleDialog", "TePerimeter", "Trailing-edge perimeter"), TePerimeterLabel) ]
	];
	RefreshChevronReadouts();

	// ---- Length (shared) ----
	TSharedRef<SWidget> LengthRow = FormRow(NSLOCTEXT("NacelleDialog", "Length", "Length"),
		SNew(SAerotelicNumericEntry).Value_Lambda([Model]() { return Model->length; })
			.MinValue(1e-9).MaxValue(1e9).Delta(0.1).LiveUpdate(false)
			.OnValueCommitted_Lambda([Model, OnChanged, SideEditor, FrontEditor](double V, ETextCommit::Type)
			{
				Model->SetLength(V);
				const double Inv = 1.0 / FMath::Max(Model->length, 1e-9);
				SideEditor->SetScales(1.0, Inv);
				FrontEditor->SetScales(Inv, Inv);
				OnChanged.ExecuteIfBound();
			}));

	TArray<FAerotelicTab> Tabs;
	Tabs.Add({ NSLOCTEXT("NacelleDialog", "SideTab", "Side view"), SideEditor });
	Tabs.Add({ NSLOCTEXT("NacelleDialog", "FrontTab", "Front view"), FrontTab });
	Tabs.Add({ NSLOCTEXT("NacelleDialog", "ChevronsTab", "Chevrons"), ChevronsTab });

	Out.Title = FText::Format(NSLOCTEXT("NacelleDialog", "Title", "Nacelle -- {0}"), FText::FromString(Entity->EntityName));
	Out.Widget =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(8) [ LengthRow ]
		+ SVerticalBox::Slot().FillHeight(1.0f) [ SNew(SAerotelicTabStrip, Tabs) ];
}
