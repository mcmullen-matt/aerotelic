#include "FuselageDialog.h"
#include "EntityDesignContent.h"
#include "Widgets/TabStrip.h"
#include "Widgets/NumericEntry.h"
#include "Widgets/CurveEditor.h"

#include "Widgets/Layout/SBorder.h"
#include "Widgets/Text/STextBlock.h"

#include "Entity/AerotelicEntityActor.h"
#include "aerotelic/fuselage_model.hpp"

using namespace aerotelic;

namespace
{
	TSharedRef<SCurveEditor> MakeCurveTab(FText XLabel, FText YLabel, FText Title, double YScale, bool bEqualAspect,
		TArray<FCurveEditorSlot> Slots, FSimpleDelegate OnChanged)
	{
		TSharedRef<SCurveEditor> Editor = SNew(SCurveEditor)
			.XLabel(XLabel).YLabel(YLabel).Title(Title)
			.XScale(1.0).YScale(YScale).EqualAspect(bEqualAspect)
			.OnChanged(OnChanged);
		Editor->SetSlots(MoveTemp(Slots));
		return Editor;
	}
}

void BuildFuselageDesignContent(AAerotelicEntityActor* Entity, FAerotelicDesignContent& Out)
{
	if (Entity == nullptr || Entity->GetEntityType() != project_yaml::EntityType::Fuselage)
	{
		return;
	}

	FuselageModel* Model = Entity->GetModelAs<FuselageModel>();
	TWeakObjectPtr<AAerotelicEntityActor> WeakEntity = Entity;
	FSimpleDelegate OnChanged = FSimpleDelegate::CreateLambda([WeakEntity]()
	{
		if (WeakEntity.IsValid()) WeakEntity->RefreshProxy();
	});

	FCurveEditorSlot UpperSlot, LowerSlot;
	UpperSlot.Curve = &Model->z_upper_curve_mut();
	UpperSlot.Color = FLinearColor(0.23f, 0.62f, 0.84f);
	UpperSlot.Label = NSLOCTEXT("FuselageDialog", "Upper", "upper");
	UpperSlot.Basis = ECurveBasisPlacement::Above;
	LowerSlot.Curve = &Model->z_lower_curve_mut();
	LowerSlot.Color = FLinearColor(0.84f, 0.54f, 0.23f);
	LowerSlot.Label = NSLOCTEXT("FuselageDialog", "Lower", "lower");
	LowerSlot.Basis = ECurveBasisPlacement::Below;

	FCurveEditorSlot WidthSlot;
	WidthSlot.Curve = &Model->y_half_curve_mut();
	WidthSlot.Color = FLinearColor(0.75f, 0.56f, 0.19f);
	WidthSlot.YClampLo = 0.0;
	WidthSlot.Basis = ECurveBasisPlacement::Below;

	FCurveEditorSlot RhoUpperSlot;
	RhoUpperSlot.Curve = &Model->rho_upper_curve_mut();
	RhoUpperSlot.Color = FLinearColor(0.63f, 0.31f, 0.82f);
	RhoUpperSlot.YClampLo = 0.01;
	RhoUpperSlot.YClampHi = 0.99;
	RhoUpperSlot.Basis = ECurveBasisPlacement::Below;

	FCurveEditorSlot RhoLowerSlot;
	RhoLowerSlot.Curve = &Model->rho_lower_curve_mut();
	RhoLowerSlot.Color = FLinearColor(0.82f, 0.44f, 0.63f);
	RhoLowerSlot.YClampLo = 0.01;
	RhoLowerSlot.YClampHi = 0.99;
	RhoLowerSlot.Basis = ECurveBasisPlacement::Below;

	const double InvLength = 1.0 / FMath::Max(Model->length, 1e-9);

	TSharedRef<SCurveEditor> ProfileEditor = MakeCurveTab(
		NSLOCTEXT("FuselageDialog", "XL", "Fuselage fraction (x/L)"), NSLOCTEXT("FuselageDialog", "ZL", "Height (z/L)"),
		NSLOCTEXT("FuselageDialog", "ProfileTitle", "Profile"), InvLength, true, { UpperSlot, LowerSlot }, OnChanged);
	TSharedRef<SCurveEditor> WidthEditor = MakeCurveTab(
		NSLOCTEXT("FuselageDialog", "XL", "Fuselage fraction (x/L)"), NSLOCTEXT("FuselageDialog", "YL", "Half-width (y/L)"),
		NSLOCTEXT("FuselageDialog", "WidthTitle", "Width"), InvLength, true, { WidthSlot }, OnChanged);

	TArray<FAerotelicTab> Tabs;
	Tabs.Add({ NSLOCTEXT("FuselageDialog", "Profile", "Profile"), ProfileEditor });
	Tabs.Add({ NSLOCTEXT("FuselageDialog", "Width", "Width"), WidthEditor });
	Tabs.Add({ NSLOCTEXT("FuselageDialog", "RhoUpper", "rho upper"),
		MakeCurveTab(NSLOCTEXT("FuselageDialog", "XL", "Fuselage fraction (x/L)"), NSLOCTEXT("FuselageDialog", "Rho", "rho"),
			NSLOCTEXT("FuselageDialog", "RhoUpperTitle", "rho upper (0.414=ellipse, 0.5=parabola, >0.5=hyperbola)"), 1.0, false, { RhoUpperSlot }, OnChanged) });
	Tabs.Add({ NSLOCTEXT("FuselageDialog", "RhoLower", "rho lower"),
		MakeCurveTab(NSLOCTEXT("FuselageDialog", "XL", "Fuselage fraction (x/L)"), NSLOCTEXT("FuselageDialog", "Rho", "rho"),
			NSLOCTEXT("FuselageDialog", "RhoLowerTitle", "rho lower (0.414=ellipse, 0.5=parabola, >0.5=hyperbola)"), 1.0, false, { RhoLowerSlot }, OnChanged) });

	Out.Title = FText::Format(NSLOCTEXT("FuselageDialog", "Title", "Fuselage -- {0}"), FText::FromString(Entity->EntityName));
	Out.Widget =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(8)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0)
			[ SNew(STextBlock).Text(NSLOCTEXT("FuselageDialog", "Length", "Length")) ]
			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SAerotelicNumericEntry)
				.Value_Lambda([Model]() { return Model->length; })
				.MinValue(1e-9).MaxValue(1e9).Delta(0.5).LiveUpdate(false)
				.OnValueCommitted_Lambda([Model, OnChanged, ProfileEditor, WidthEditor](double V, ETextCommit::Type)
				{
					Model->SetLength(V);
					const double NewInvLength = 1.0 / FMath::Max(Model->length, 1e-9);
					ProfileEditor->SetScales(1.0, NewInvLength);
					WidthEditor->SetScales(1.0, NewInvLength);
					OnChanged.ExecuteIfBound();
				})
			]
		]
		+ SVerticalBox::Slot().FillHeight(1.0f)
		[ SNew(SAerotelicTabStrip, Tabs) ];
}
