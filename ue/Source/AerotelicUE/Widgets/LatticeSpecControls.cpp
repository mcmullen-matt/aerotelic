#include "LatticeSpecControls.h"

#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SComboBox.h"

#include "Widgets/CurveTabControls.h"
#include "Widgets/NumericEntry.h"

#define LOCTEXT_NAMESPACE "AerotelicLatticeSpecControls"

namespace
{
	TSharedRef<SWidget> FormRow(const FText& Label, TSharedRef<SWidget> Field)
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0)
			[ SNew(SBox).WidthOverride(150) [ SNew(STextBlock).Text(Label) ] ]
			+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
			[ Field ];
	}

	TArray<TSharedPtr<FString>> Options(const std::vector<std::string>& Names)
	{
		TArray<TSharedPtr<FString>> Out;
		for (const std::string& N : Names) Out.Add(MakeShared<FString>(UTF8_TO_TCHAR(N.c_str())));
		return Out;
	}

	TSharedPtr<FString> Find(const TArray<TSharedPtr<FString>>& List, const FString& Value)
	{
		for (const TSharedPtr<FString>& S : List)
		{
			if (S.IsValid() && *S == Value) return S;
		}
		return List.Num() ? List[0] : nullptr;
	}
}

FAerotelicLatticeSpecDraft::FAerotelicLatticeSpecDraft()
{
	Families = { MakeShared<FString>(TEXT("TPMS")), MakeShared<FString>(TEXT("Strut-based")), MakeShared<FString>(TEXT("Auxetic")), MakeShared<FString>(TEXT("Honeycomb")) };
	Phases = { MakeShared<FString>(TEXT("Network")), MakeShared<FString>(TEXT("Sheet (matrix)")), MakeShared<FString>(TEXT("Iso-surface")) };
	Axes = { MakeShared<FString>(TEXT("X")), MakeShared<FString>(TEXT("Y")), MakeShared<FString>(TEXT("Z")) };
	Sizings = { MakeShared<FString>(TEXT("From volume fraction")), MakeShared<FString>(TEXT("Wall thickness")) };
	SyncFromSpec();
}

void FAerotelicLatticeSpecDraft::SyncFromSpec()
{
	Types = Options(aerotelic::LatticeTypeNames(Spec.family));
	Family = Families[FMath::Clamp(static_cast<int32>(Spec.family), 0, Families.Num() - 1)];
	Phase = Phases[FMath::Clamp(static_cast<int32>(Spec.phase), 0, Phases.Num() - 1)];
	Axis = Axes[FMath::Clamp(static_cast<int32>(Spec.axis), 0, Axes.Num() - 1)];
	Sizing = Sizings[FMath::Clamp(static_cast<int32>(Spec.wall_sizing), 0, Sizings.Num() - 1)];
	Type = Find(Types, UTF8_TO_TCHAR(Spec.type.c_str()));
	Resolution = Spec.resolution;
	if (TypeCombo.IsValid()) { TypeCombo->RefreshOptions(); TypeCombo->SetSelectedItem(Type); }
	if (PhaseCombo.IsValid()) PhaseCombo->SetSelectedItem(Phase);
	UpdateRows();
}

void FAerotelicLatticeSpecDraft::SyncToSpec()
{
	Spec.family = static_cast<aerotelic::LatticeFamily>(FMath::Max(0, Families.IndexOfByKey(Family)));
	Spec.phase = static_cast<aerotelic::TpmsPhase>(FMath::Max(0, Phases.IndexOfByKey(Phase)));
	Spec.axis = static_cast<aerotelic::PrismAxis>(FMath::Max(0, Axes.IndexOfByKey(Axis)));
	Spec.wall_sizing = static_cast<aerotelic::WallSizing>(FMath::Max(0, Sizings.IndexOfByKey(Sizing)));
	Spec.type = TCHAR_TO_UTF8(*(Type.IsValid() ? *Type : FString()));
	Spec.resolution = FMath::Clamp(FMath::RoundToInt(Resolution), 2, 64);
	Spec.skin_thickness = 0.0;   // the body's, never the lattice's
}

void FAerotelicLatticeSpecDraft::UpdateRows()
{
	const int32 Fam = Families.IndexOfByKey(Family);
	const bool bTpms = Fam == 0;
	const bool bComb = Fam == 3;
	const bool bIso = bTpms && Phases.IndexOfByKey(Phase) == 2;
	const bool bByThickness = bComb && Sizings.IndexOfByKey(Sizing) == 1;
	const bool bSquare = bComb && Type.IsValid() && *Type == TEXT("Square");
	if (PhaseCombo.IsValid()) PhaseCombo->SetEnabled(bTpms);
	if (FractionRow.IsValid()) FractionRow->SetEnabled(!bIso && !bByThickness);
	if (WallRow.IsValid()) WallRow->SetEnabled(bIso || bByThickness);
	if (AspectRow.IsValid()) AspectRow->SetEnabled(bComb && !bSquare);
	if (AxisRow.IsValid()) AxisRow->SetEnabled(bComb);
	if (SizingRow.IsValid()) SizingRow->SetEnabled(bComb);
}

TSharedRef<SWidget> MakeAerotelicLatticeSpecControls(TSharedRef<FAerotelicLatticeSpecDraft> Draft, FSimpleDelegate OnChanged)
{
	auto Changed = [Draft, OnChanged]()
	{
		Draft->SyncToSpec();
		Draft->UpdateRows();
		OnChanged.ExecuteIfBound();
	};
	auto Combo = [Draft, Changed](TArray<TSharedPtr<FString>>* Source, TSharedPtr<FString>* Current)
	{
		return SNew(SComboBox<TSharedPtr<FString>>)
			.OptionsSource(Source)
			.InitiallySelectedItem(*Current)
			.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
			.OnSelectionChanged_Lambda([Draft, Current, Changed](TSharedPtr<FString> S, ESelectInfo::Type) { if (S.IsValid()) { *Current = S; Changed(); } })
			[ SNew(STextBlock).Text_Lambda([Current]() { return FText::FromString(Current->IsValid() ? **Current : FString()); }) ];
	};
	auto Spin = [Draft, Changed](double aerotelic::LatticeSpec::* Member, double Min, double Max, double Delta)
	{
		return SNew(SAerotelicNumericEntry)
			.Value_Lambda([Draft, Member]() { return Draft->Spec.*Member; })
			.MinValue(Min).MaxValue(Max).Delta(Delta)
			.LiveUpdate(false)
			.OnValueCommitted_Lambda([Draft, Member, Changed](double V, ETextCommit::Type) { Draft->Spec.*Member = V; Changed(); });
	};
	auto OffsetSpin = [Draft, Changed](int32 Component)
	{
		return SNew(SAerotelicNumericEntry)
			.Value_Lambda([Draft, Component]() { return Draft->Spec.origin[Component]; })
			.MinValue(-1e9).MaxValue(1e9).Delta(0.01)
			.LiveUpdate(false)
			.OnValueCommitted_Lambda([Draft, Component, Changed](double V, ETextCommit::Type) { Draft->Spec.origin[Component] = V; Changed(); });
	};

	// The family combo refills the type list.
	TSharedRef<SWidget> FamilyCombo = SNew(SComboBox<TSharedPtr<FString>>)
		.OptionsSource(&Draft->Families)
		.InitiallySelectedItem(Draft->Family)
		.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
		.OnSelectionChanged_Lambda([Draft, Changed](TSharedPtr<FString> S, ESelectInfo::Type)
		{
			if (!S.IsValid()) return;
			Draft->Family = S;
			Draft->Types = Options(aerotelic::LatticeTypeNames(static_cast<aerotelic::LatticeFamily>(FMath::Max(0, Draft->Families.IndexOfByKey(S)))));
			Draft->Type = Draft->Types.Num() ? Draft->Types[0] : nullptr;
			if (Draft->TypeCombo.IsValid()) { Draft->TypeCombo->RefreshOptions(); Draft->TypeCombo->SetSelectedItem(Draft->Type); }
			Changed();
		})
		[ SNew(STextBlock).Text_Lambda([Draft]() { return FText::FromString(Draft->Family.IsValid() ? *Draft->Family : FString()); }) ];

	TSharedRef<SWidget> Controls =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Family", "Family:"), FamilyCombo) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
		[ FormRow(LOCTEXT("Type", "Type:"), SAssignNew(Draft->TypeCombo, SComboBox<TSharedPtr<FString>>)
			.OptionsSource(&Draft->Types)
			.InitiallySelectedItem(Draft->Type)
			.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
			.OnSelectionChanged_Lambda([Draft, Changed](TSharedPtr<FString> S, ESelectInfo::Type) { if (S.IsValid()) { Draft->Type = S; Changed(); } })
			[ SNew(STextBlock).Text_Lambda([Draft]() { return FText::FromString(Draft->Type.IsValid() ? *Draft->Type : FString()); }) ]) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
		[ FormRow(LOCTEXT("Phase", "TPMS phase:"), SAssignNew(Draft->PhaseCombo, SComboBox<TSharedPtr<FString>>)
			.OptionsSource(&Draft->Phases)
			.InitiallySelectedItem(Draft->Phase)
			.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
			.OnSelectionChanged_Lambda([Draft, Changed](TSharedPtr<FString> S, ESelectInfo::Type) { if (S.IsValid()) { Draft->Phase = S; Changed(); } })
			[ SNew(STextBlock).Text_Lambda([Draft]() { return FText::FromString(Draft->Phase.IsValid() ? *Draft->Phase : FString()); }) ]) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
		[ SAssignNew(Draft->AxisRow, SBox) [ FormRow(LOCTEXT("Axis", "Prism axis:"), Combo(&Draft->Axes, &Draft->Axis)) ] ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("CellSize", "Unit cell size:"), Spin(&aerotelic::LatticeSpec::cell_size, 1e-6, 1e9, 0.01)) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ SAssignNew(Draft->AspectRow, SBox) [ FormRow(LOCTEXT("Aspect", "Cell aspect:"), Spin(&aerotelic::LatticeSpec::cell_aspect, 0.05, 20.0, 0.1)) ] ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ SAssignNew(Draft->SizingRow, SBox) [ FormRow(LOCTEXT("Sizing", "Wall set by:"), Combo(&Draft->Sizings, &Draft->Sizing)) ] ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ SAssignNew(Draft->FractionRow, SBox) [ FormRow(LOCTEXT("Fraction", "Volume fraction:"), Spin(&aerotelic::LatticeSpec::volume_fraction, 0.02, 0.95, 0.05)) ] ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ SAssignNew(Draft->WallRow, SBox) [ FormRow(LOCTEXT("Wall", "Wall thickness:"), Spin(&aerotelic::LatticeSpec::wall_thickness, 1e-6, 1e9, 0.001)) ] ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
		[ FormRow(LOCTEXT("Resolution", "Voxels per cell:"), SNew(SAerotelicNumericEntry)
			.Value_Lambda([Draft]() { return Draft->Resolution; })
			.MinValue(2).MaxValue(64).Delta(1)
			.LiveUpdate(false)
			.OnValueCommitted_Lambda([Draft, Changed](double V, ETextCommit::Type) { Draft->Resolution = V; Changed(); })) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("OffsetX", "Offset x (fore/aft):"), OffsetSpin(0)) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("OffsetY", "Offset y (left/right):"), OffsetSpin(1)) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("OffsetZ", "Offset z (up/down):"), OffsetSpin(2)) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
		[ MakeInfoLabel(LOCTEXT("Hint", "TPMS: a level-set surface (27 types, after OpenLattice's library) -- Network keeps one side of it, Sheet thickens it to the volume fraction, Iso-surface to a wall thickness. Strut-based and auxetic cells are unions of cylindrical struts. Honeycomb cells are a 2D tiling's walls extruded along the prism axis, stiff along it and soft across it; their wall can be set by thickness. Sizes are in model units. The volume fraction is the lattice's share of the interior. Voxels per cell is the sampling resolution of the surface extraction (8-16 is usual). The offsets add a constant to the coordinates the pattern is built on, sliding it through the body; a whole cell of offset changes nothing.")) ];

	Draft->UpdateRows();
	return Controls;
}

#undef LOCTEXT_NAMESPACE
