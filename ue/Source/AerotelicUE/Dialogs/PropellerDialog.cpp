#include "PropellerDialog.h"
#include "WingDesignTabs.h"
#include "EntityDesignContent.h"
#include "Widgets/TabStrip.h"
#include "Widgets/NumericEntry.h"
#include "Widgets/CurveEditor.h"

#include "Widgets/Layout/SBorder.h"
#include "Widgets/Text/STextBlock.h"

#include "Entity/AerotelicEntityActor.h"
#include "aerotelic/propeller_model.hpp"

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

	TSharedRef<SWidget> BuildPropellerTab(PropellerModel* Model, FSimpleDelegate OnChanged)
	{
		TSharedRef<STextBlock> Derived = SNew(STextBlock).AutoWrapText(true);
		TFunction<void()> Refresh = [Model, Derived]()
		{
			try
			{
				PropellerMetrics M = Model->Metrics();
				Derived->SetText(FText::Format(NSLOCTEXT("PropellerDialog", "Derived",
					"Diameter: {0}    Solidity: {1}    Activity Factor: {2}\nBeta @0.75R: {3}    Pitch/D @0.75R: {4}\nSpinner diameter: {5}    Spinner length: {6}"),
					FText::AsNumber(M.diameter), FText::AsNumber(M.solidity), FText::AsNumber(M.activity_factor),
					FText::AsNumber(M.beta_75), FText::AsNumber(M.pitch_ratio), FText::AsNumber(M.spinner_diameter), FText::AsNumber(M.spinner_length)));
			}
			catch (const std::exception&)
			{
				Derived->SetText(NSLOCTEXT("PropellerDialog", "DerivedNA", "--"));
			}
		};
		Refresh();

		return SNew(SBorder).Padding(8)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight()
			[ FormRow(NSLOCTEXT("PropellerDialog", "Radius", "Radius R"),
				SNew(SAerotelicNumericEntry).Value_Lambda([Model]() { return Model->radius; })
					.MinValue(1e-9).MaxValue(1e9).Delta(0.01).LiveUpdate(false)
					.OnValueCommitted_Lambda([Model, OnChanged, Refresh](double V, ETextCommit::Type)
					{ Model->SetRadius(V); Refresh(); OnChanged.ExecuteIfBound(); })) ]
			+ SVerticalBox::Slot().AutoHeight()
			[ FormRow(NSLOCTEXT("PropellerDialog", "BladeCount", "Number of blades"),
				SNew(SAerotelicNumericEntry).Value_Lambda([Model]() { return (double)Model->blade_count; })
					.MinValue(1).MaxValue((double)kPropellerMaxBlades).Delta(1.0).LiveUpdate(false)
					.OnValueCommitted_Lambda([Model, OnChanged, Refresh](double V, ETextCommit::Type)
					{ Model->blade_count = FMath::RoundToInt(V); Refresh(); OnChanged.ExecuteIfBound(); })) ]
			+ SVerticalBox::Slot().AutoHeight()
			[ FormRow(NSLOCTEXT("PropellerDialog", "BladeX", "Blade position (x/R)"),
				SNew(SAerotelicNumericEntry).Value_Lambda([Model]() { return Model->blade_x; })
					.MinValue(-1e9).MaxValue(1e9).Delta(0.01).LiveUpdate(false)
					.OnValueCommitted_Lambda([Model, OnChanged, Refresh](double V, ETextCommit::Type)
					{ Model->blade_x = V; Refresh(); OnChanged.ExecuteIfBound(); })) ]
			+ SVerticalBox::Slot().AutoHeight()
			[ FormRow(NSLOCTEXT("PropellerDialog", "HubRadius", "Blade root radius (r/R)"),
				SNew(SAerotelicNumericEntry).Value_Lambda([Model]() { return Model->hub_radius; })
					.MinValue(0.0).MaxValue(1.0).Delta(0.01).LiveUpdate(false)
					.OnValueCommitted_Lambda([Model, OnChanged, Refresh](double V, ETextCommit::Type)
					{ Model->hub_radius = V; Refresh(); OnChanged.ExecuteIfBound(); })) ]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
			[ Derived ]
		];
	}
}

void BuildPropellerDesignContent(AAerotelicEntityActor* Entity, FAerotelicDesignContent& Out)
{
	if (Entity == nullptr || Entity->GetEntityType() != project_yaml::EntityType::Propeller)
	{
		return;
	}

	PropellerModel* Model = Entity->GetModelAs<PropellerModel>();
	TWeakObjectPtr<AAerotelicEntityActor> WeakEntity = Entity;
	FSimpleDelegate OnChanged = FSimpleDelegate::CreateLambda([WeakEntity]()
	{
		if (WeakEntity.IsValid()) WeakEntity->RefreshProxy();
	});

	TSharedRef<FWingDesignTabs> BladeTabs = MakeShared<FWingDesignTabs>(&Model->blade(), OnChanged);

	// ---- Spinner ----
	FCurveEditorSlot SpinnerSlot;
	SpinnerSlot.Curve = &Model->spinner_curve_mut();
	SpinnerSlot.Color = FLinearColor(0.75f, 0.78f, 0.85f);
	SpinnerSlot.YClampLo = 0.0;
	SpinnerSlot.FixedPoles.Add(0, FVector2D(0.0, 0.0));
	SpinnerSlot.FixedPoles.Add(-1, FVector2D(1.0, 0.0));
	SpinnerSlot.Basis = ECurveBasisPlacement::Below;
	TSharedRef<SCurveEditor> SpinnerEditor = SNew(SCurveEditor)
		.XLabel(NSLOCTEXT("PropellerDialog", "XR", "Axial position (x/R)"))
		.YLabel(NSLOCTEXT("PropellerDialog", "RadiusR", "Radius (/R)"))
		.Title(NSLOCTEXT("PropellerDialog", "SpinnerTitle", "Spinner"))
		.XScale(Model->spinner_length).YScale(1.0).EqualAspect(false)
		.OnChanged(OnChanged);
	SpinnerEditor->SetSlots({ SpinnerSlot });

	TSharedRef<SWidget> SpinnerTab = SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(8)
		[ FormRow(NSLOCTEXT("PropellerDialog", "SpinnerLength", "Spinner length (l/R)"),
			SNew(SAerotelicNumericEntry).Value_Lambda([Model]() { return Model->spinner_length; })
				.MinValue(1e-9).MaxValue(1e9).Delta(0.01).LiveUpdate(false)
				.OnValueCommitted_Lambda([Model, OnChanged, SpinnerEditor](double V, ETextCommit::Type)
				{
					Model->spinner_length = V;
					SpinnerEditor->SetScales(Model->spinner_length, 1.0);
					OnChanged.ExecuteIfBound();
				})) ]
		+ SVerticalBox::Slot().FillHeight(1.0f) [ SpinnerEditor ];

	TArray<FAerotelicTab> Tabs;
	Tabs.Add({ NSLOCTEXT("PropellerDialog", "PropellerTab", "Propeller"), BuildPropellerTab(Model, OnChanged) });
	for (const auto& Tab : BladeTabs->GetTabs())
	{
		Tabs.Add({ Tab.Key, Tab.Value });
	}
	Tabs.Add({ NSLOCTEXT("PropellerDialog", "SpinnerTab", "Spinner"), SpinnerTab });

	Out.Title = FText::Format(NSLOCTEXT("PropellerDialog", "Title", "Propeller -- {0}"), FText::FromString(Entity->EntityName));
	// Keeps BladeTabs (and its model-bound widgets) alive with the panel.
	Out.WingTabs = BladeTabs;
	Out.Widget = SNew(SAerotelicTabStrip, Tabs);
}
