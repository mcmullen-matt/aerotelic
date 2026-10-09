#include "WingDesignTabs.h"
#include "SectionListDialog.h"
#include "Widgets/NumericEntry.h"
#include "Widgets/CurveEditor.h"

#include "Widgets/Layout/SBorder.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/SBoxPanel.h"

#include "Entity/AerotelicDocumentSubsystem.h"

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

	TSharedRef<SWidget> NumericRow(const FText& Label, double Value, double Lo, double Hi, double Delta, FOnAerotelicValueCommitted OnCommitted)
	{
		return FormRow(Label, SNew(SAerotelicNumericEntry).Value(Value).MinValue(Lo).MaxValue(Hi).Delta(Delta).OnValueCommitted(OnCommitted));
	}
}

FWingDesignTabs::FWingDesignTabs(WingModel* InModel, FSimpleDelegate InOnChanged)
	: Model(InModel), OnChangedDelegate(InOnChanged)
{
	Tabs.Add({ NSLOCTEXT("WingDesign", "Planform", "Planform"), BuildPlanformTab() });
	Tabs.Add({ NSLOCTEXT("WingDesign", "Dihedral", "Dihedral"), BuildDihedralTab() });
	Tabs.Add({ NSLOCTEXT("WingDesign", "Sections", "Sections"), BuildSectionsTab() });
	Tabs.Add({ NSLOCTEXT("WingDesign", "Twist", "Twist"), BuildTwistTab() });
}

void FWingDesignTabs::NotifyChanged()
{
	OnChangedDelegate.ExecuteIfBound();
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

TSharedRef<SWidget> FWingDesignTabs::BuildStandardPlanformTab(TrapPlanform* Trap)
{
	return SNew(SBorder).Padding(8)
	[
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(NSLOCTEXT("WingDesign", "SemiSpan", "Semi-Span"), Trap->SemiSpan(), 1e-9, 1e9, 0.25,
			FOnAerotelicValueCommitted::CreateLambda([this, Trap](double V, ETextCommit::Type) { Trap->SetSemiSpan(V); NotifyChanged(); })) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(NSLOCTEXT("WingDesign", "RootChord", "Root Chord"), Trap->RootChord(), 1e-9, 1e9, 0.1,
			FOnAerotelicValueCommitted::CreateLambda([this, Trap](double V, ETextCommit::Type) { Trap->SetRootChord(V); NotifyChanged(); })) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(NSLOCTEXT("WingDesign", "TipChord", "Tip Chord"), Trap->TipChord(), 1e-9, 1e9, 0.05,
			FOnAerotelicValueCommitted::CreateLambda([this, Trap](double V, ETextCommit::Type) { Trap->SetTipChord(V); NotifyChanged(); })) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(NSLOCTEXT("WingDesign", "SweepAngle", "Sweep Angle (deg)"), Trap->sweep_angle(), -89.0, 89.0, 0.5,
			FOnAerotelicValueCommitted::CreateLambda([this, Trap](double V, ETextCommit::Type) { Trap->SetSweepAngle(V); NotifyChanged(); })) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(NSLOCTEXT("WingDesign", "SweepFraction", "Sweep at chord (0=LE, 1=TE)"), Trap->sweep_fraction(), 0.0, 1.0, 0.05,
			FOnAerotelicValueCommitted::CreateLambda([this, Trap](double V, ETextCommit::Type) { Trap->SetSweepFraction(V); NotifyChanged(); })) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
		[
			SNew(STextBlock).Text_Lambda([Trap]()
			{
				return FText::Format(NSLOCTEXT("WingDesign", "DerivedAR", "Aspect Ratio: {0}    Taper Ratio: {1}"),
					FText::AsNumber(Trap->AspectRatio()), FText::AsNumber(Trap->TaperRatio()));
			})
		]
	];
}

TSharedRef<SWidget> FWingDesignTabs::BuildCrankedPlanformTab(DoubleTrapPlanform* Dtp)
{
	return SNew(SBorder).Padding(8)
	[
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(NSLOCTEXT("WingDesign", "SemiSpan", "Semi-Span"), Dtp->SemiSpan(), 1e-9, 1e9, 0.25,
			FOnAerotelicValueCommitted::CreateLambda([this, Dtp](double V, ETextCommit::Type) { Dtp->SetSemiSpan(V); NotifyChanged(); })) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(NSLOCTEXT("WingDesign", "RootChord", "Root Chord"), Dtp->RootChord(), 1e-9, 1e9, 0.1,
			FOnAerotelicValueCommitted::CreateLambda([this, Dtp](double V, ETextCommit::Type) { Dtp->SetRootChord(V); NotifyChanged(); })) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(NSLOCTEXT("WingDesign", "TipChord", "Tip Chord"), Dtp->TipChord(), 1e-9, 1e9, 0.05,
			FOnAerotelicValueCommitted::CreateLambda([this, Dtp](double V, ETextCommit::Type) { Dtp->SetTipChord(V); NotifyChanged(); })) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(NSLOCTEXT("WingDesign", "KinkFraction", "Kink Position (eta)"), Dtp->KinkFraction(), 0.05, 0.95, 0.05,
			FOnAerotelicValueCommitted::CreateLambda([this, Dtp](double V, ETextCommit::Type) { Dtp->SetKinkFraction(V); NotifyChanged(); })) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(NSLOCTEXT("WingDesign", "LeSweep", "Leading Edge Sweep (deg)"), Dtp->LeadingEdgeSweepAngle(), -89.0, 89.0, 0.5,
			FOnAerotelicValueCommitted::CreateLambda([this, Dtp](double V, ETextCommit::Type) { Dtp->SetLeadingEdgeSweepAngle(V); NotifyChanged(); })) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumericRow(NSLOCTEXT("WingDesign", "TeSweep", "Trailing Edge Sweep (deg)"), Dtp->TrailingEdgeSweepAngle(), -89.0, 89.0, 0.5,
			FOnAerotelicValueCommitted::CreateLambda([this, Dtp](double V, ETextCommit::Type) { Dtp->SetTrailingEdgeSweepAngle(V); NotifyChanged(); })) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
		[
			SNew(STextBlock).Text_Lambda([Dtp]()
			{
				return FText::Format(NSLOCTEXT("WingDesign", "DerivedAR", "Aspect Ratio: {0}    Taper Ratio: {1}"),
					FText::AsNumber(Dtp->AspectRatio()), FText::AsNumber(Dtp->TaperRatio()));
			})
		]
	];
}

TSharedRef<SWidget> FWingDesignTabs::BuildNurbsPlanformTab(NURBSPlanform* Nurbs)
{
	TSharedRef<SCurveEditor> Editor = SNew(SCurveEditor)
		.XLabel(NSLOCTEXT("WingDesign", "EtaLabel", "Span fraction (eta)"))
		.YLabel(NSLOCTEXT("WingDesign", "ChordLabel", "Chordwise (x/L)"))
		.Title(NSLOCTEXT("WingDesign", "PlanformTitle", "Planform"))
		.XScale(1.0)
		.YScale(1.0 / FMath::Max(Nurbs->SemiSpan(), 1e-9))
		.EqualAspect(true)
		.SupportsUserLinks(false)
		.OnChanged(FSimpleDelegate::CreateLambda([this]() { NotifyChanged(); }));

	FCurveEditorSlot LeSlot, TeSlot;
	LeSlot.Curve = &Nurbs->le_curve_mut();
	LeSlot.Color = FLinearColor(0.88f, 0.31f, 0.31f);
	LeSlot.Label = NSLOCTEXT("WingDesign", "LE", "LE");
	LeSlot.Basis = ECurveBasisPlacement::Above;
	TeSlot.Curve = &Nurbs->te_curve_mut();
	TeSlot.Color = FLinearColor(0.25f, 0.5f, 0.88f);
	TeSlot.Label = NSLOCTEXT("WingDesign", "TE", "TE");
	TeSlot.Basis = ECurveBasisPlacement::Below;
	Editor->SetSlots({ LeSlot, TeSlot });

	return SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(8)
		[ NumericRow(NSLOCTEXT("WingDesign", "SemiSpan", "Semi-Span"), Nurbs->SemiSpan(), 1e-9, 1e9, 0.25,
			FOnAerotelicValueCommitted::CreateLambda([this, Nurbs, Editor](double V, ETextCommit::Type)
			{
				Nurbs->SetSemiSpan(V);
				Editor->SetScales(1.0, 1.0 / FMath::Max(Nurbs->SemiSpan(), 1e-9));
				NotifyChanged();
			})) ]
		+ SVerticalBox::Slot().FillHeight(1.0f)
		[ Editor ];
}

TSharedRef<SWidget> FWingDesignTabs::BuildDihedralTab()
{
	TSharedRef<SCurveEditor> Editor = SNew(SCurveEditor)
		.XLabel(NSLOCTEXT("WingDesign", "EtaLabel", "Span fraction (eta)"))
		.YLabel(NSLOCTEXT("WingDesign", "HeightLabel", "Height (z/L)"))
		.Title(NSLOCTEXT("WingDesign", "DihedralTitle", "Dihedral"))
		.XScale(1.0).YScale(1.0).EqualAspect(true)
		.OnChanged(FSimpleDelegate::CreateLambda([this]() { NotifyChanged(); }));
	FCurveEditorSlot Slot;
	Slot.Curve = &Model->planform().DihedralCurveMut();
	Slot.Color = FLinearColor(0.75f, 0.63f, 0.19f);
	Slot.Basis = ECurveBasisPlacement::Below;
	Editor->SetSlots({ Slot });
	return Editor;
}

TSharedRef<SWidget> FWingDesignTabs::BuildTwistTab()
{
	TSharedRef<SCurveEditor> Editor = SNew(SCurveEditor)
		.XLabel(NSLOCTEXT("WingDesign", "EtaLabel", "Span fraction (eta)"))
		.YLabel(NSLOCTEXT("WingDesign", "TwistLabel", "Twist (deg)"))
		.Title(NSLOCTEXT("WingDesign", "TwistTitle", "Twist"))
		.XScale(1.0).YScale(1.0).EqualAspect(false)
		.OnChanged(FSimpleDelegate::CreateLambda([this]() { NotifyChanged(); }));
	FCurveEditorSlot Slot;
	Slot.Curve = &Model->planform().TwistCurveMut();
	Slot.Color = FLinearColor(0.44f, 0.75f, 0.44f);
	Slot.Basis = ECurveBasisPlacement::Below;
	Editor->SetSlots({ Slot });
	return Editor;
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
