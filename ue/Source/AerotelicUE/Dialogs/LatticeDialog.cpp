#include "LatticeDialog.h"
#include "Widgets/MessageBox.h"
#include "BodyPicker.h"

#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Framework/Application/SlateApplication.h"

#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"
#include "Entity/AerotelicLatticeModel.h"
#include "Widgets/CurveTabControls.h"
#include "Widgets/NumericEntry.h"

#include "aerotelic/lattice.hpp"

#define LOCTEXT_NAMESPACE "LatticeDialog"

namespace
{
	using aerotelic::project_yaml::EntityType;

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
		for (const std::string& N : Names)
		{
			Out.Add(MakeShared<FString>(UTF8_TO_TCHAR(N.c_str())));
		}
		return Out;
	}

	TSharedPtr<FString> Find(const TArray<TSharedPtr<FString>>& List, const FString& Value)
	{
		for (const TSharedPtr<FString>& S : List)
		{
			if (S.IsValid() && *S == Value)
			{
				return S;
			}
		}
		return List.Num() ? List[0] : nullptr;
	}

	FString Thousands(int64 N)
	{
		return FText::AsNumber(N).ToString();
	}

	bool SameOperand(const FAerotelicBooleanOperand& A, const FAerotelicBooleanOperand& B)
	{
		return A.Entity.Get() == B.Entity.Get() && A.bMirror == B.bMirror;
	}
}

void ShowAerotelicLatticeDialog(AAerotelicEntityActor* Existing, TFunction<void(const FText&)> Status)
{
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	if (Doc == nullptr)
	{
		return;
	}
	const bool bEdit = Existing != nullptr && Existing->GetEntityType() == EntityType::Lattice;
	const FAerotelicLatticeModel* ExistingModel = bEdit ? Existing->GetModelAs<FAerotelicLatticeModel>() : nullptr;

	// The bodies on offer: everything that is not a primitive or a lattice
	// can be filled; the primitives are the reserved volumes.
	TArray<FAerotelicBodyChoicePtr> All = SceneBodyChoices();
	TArray<FAerotelicBodyChoicePtr> Fillable, Reserved;
	for (const FAerotelicBodyChoicePtr& C : All)
	{
		if (!C->Operand.Entity.IsValid())
		{
			continue;
		}
		const EntityType Type = C->Operand.Entity->GetEntityType();
		if (Type == EntityType::Lattice)
		{
			continue;
		}
		(Type == EntityType::Primitive ? Reserved : Fillable).Add(C);
	}
	if (Fillable.Num() == 0)
	{
		AerotelicMessageBox(EAppMsgType::Ok, LOCTEXT("NothingToFill", "Nothing in the scene to fill: add a body first (a wing, fuselage, nacelle, propeller or a boolean result)."),
			LOCTEXT("Title", "Generate Lattice"));
		return;
	}
	// Everything fillable and every primitive start chosen for a new
	// lattice; an existing lattice's own lists otherwise.
	TArray<FAerotelicBodyChoicePtr> InitialTargets, InitialReserved;
	auto PickInitial = [](const TArray<FAerotelicBodyChoicePtr>& From, const TArray<FAerotelicBooleanOperand>& Wanted, TArray<FAerotelicBodyChoicePtr>& Out)
	{
		for (const FAerotelicBodyChoicePtr& C : From)
		{
			for (const FAerotelicBooleanOperand& W : Wanted)
			{
				if (SameOperand(C->Operand, W))
				{
					Out.Add(C);
					break;
				}
			}
		}
	};
	if (bEdit)
	{
		PickInitial(Fillable, ExistingModel->Targets, InitialTargets);
		PickInitial(Reserved, ExistingModel->Exclusions, InitialReserved);
	}
	else
	{
		InitialTargets = Fillable;
		InitialReserved = Reserved;
	}

	aerotelic::LatticeSpec Spec = bEdit ? ExistingModel->Spec : aerotelic::LatticeSpec();
	TArray<FAerotelicBooleanOperand> FillableOperands;
	for (const FAerotelicBodyChoicePtr& C : Fillable)
	{
		FillableOperands.Add(C->Operand);
	}
	if (!bEdit)
	{
		// A cell a tenth of the largest body's extent: a visible lattice at
		// a few hundred thousand samples, whatever the model's size.
		FBox Bounds(ForceInit);
		for (const FAerotelicBodyChoicePtr& C : Fillable)
		{
			Bounds += C->Operand.Entity->GetPlacedBounds(C->Operand.bMirror);
		}
		if (Bounds.IsValid)
		{
			const double Largest = Bounds.GetSize().GetMax();
			Spec.cell_size = FMath::Max(Largest / 10.0, 1e-4);
		}
	}
	// Every lattice has a solid skin (the part's outer shell, its own body
	// "Skin"): one percent of the largest root chord to start with. A
	// lattice saved without one gets the same default.
	if (!(Spec.skin_thickness > 0.0))
	{
		Spec.skin_thickness = UAerotelicDocumentSubsystem::DefaultSkinThickness(bEdit ? ExistingModel->Targets : FillableOperands);
	}

	// ---- option lists ----
	TArray<TSharedPtr<FString>> Families = { MakeShared<FString>(TEXT("TPMS")), MakeShared<FString>(TEXT("Strut-based")), MakeShared<FString>(TEXT("Auxetic")), MakeShared<FString>(TEXT("Honeycomb")) };
	TArray<TSharedPtr<FString>> Phases = { MakeShared<FString>(TEXT("Network")), MakeShared<FString>(TEXT("Sheet (matrix)")), MakeShared<FString>(TEXT("Iso-surface")) };
	TArray<TSharedPtr<FString>> Axes = { MakeShared<FString>(TEXT("X")), MakeShared<FString>(TEXT("Y")), MakeShared<FString>(TEXT("Z")) };
	TArray<TSharedPtr<FString>> Sizings = { MakeShared<FString>(TEXT("From volume fraction")), MakeShared<FString>(TEXT("Wall thickness")) };
	TArray<TSharedPtr<FString>> Types = Options(aerotelic::LatticeTypeNames(Spec.family));
	TSharedPtr<FString> Family = Families[static_cast<int32>(Spec.family)];
	TSharedPtr<FString> Phase = Phases[static_cast<int32>(Spec.phase)];
	TSharedPtr<FString> Axis = Axes[static_cast<int32>(Spec.axis)];
	TSharedPtr<FString> Sizing = Sizings[static_cast<int32>(Spec.wall_sizing)];
	TSharedPtr<FString> Type = Find(Types, UTF8_TO_TCHAR(Spec.type.c_str()));
	TSharedPtr<SComboBox<TSharedPtr<FString>>> TypeCombo, PhaseCombo;
	TSharedPtr<SAerotelicBodyPicker> TargetPicker, ReservedPicker;
	TSharedPtr<STextBlock> EstimateLabel;
	TSharedPtr<SEditableTextBox> NameBox;
	TSharedPtr<SWidget> FractionRow, WallRow, AspectRow, AxisRow, SizingRow;
	const FString DefaultName = bEdit ? Existing->EntityName : Doc->NextLatticeName();
	bool bAccepted = false;

	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(bEdit ? FText::Format(LOCTEXT("EditTitle", "Lattice -- {0}"), FText::FromString(Existing->EntityName)) : LOCTEXT("Title", "Generate Lattice"))
		.ClientSize(FVector2D(620, 760))
		.SupportsMaximize(false)
		.SupportsMinimize(false);

	auto Button = [](const FText& Label, TFunction<void()> Fn)
	{
		return SNew(SButton).Text(Label).HAlign(HAlign_Center)
			.OnClicked_Lambda([Fn]() { Fn(); return FReply::Handled(); });
	};
	auto Combo = [](TArray<TSharedPtr<FString>>* Source, TSharedPtr<FString>& Current, TFunction<void(TSharedPtr<FString>)> OnPick)
	{
		return SNew(SComboBox<TSharedPtr<FString>>)
			.OptionsSource(Source)
			.InitiallySelectedItem(Current)
			.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
			.OnSelectionChanged_Lambda([&Current, OnPick](TSharedPtr<FString> S, ESelectInfo::Type) { if (S.IsValid()) { Current = S; OnPick(S); } })
			[ SNew(STextBlock).Text_Lambda([&Current]() { return FText::FromString(Current.IsValid() ? *Current : FString()); }) ];
	};

	// The current choices as a spec (the family/type/phase from the combos,
	// the numbers live in Spec already).
	auto CurrentSpec = [&]() -> aerotelic::LatticeSpec
	{
		aerotelic::LatticeSpec S = Spec;
		S.family = static_cast<aerotelic::LatticeFamily>(Families.IndexOfByKey(Family));
		S.phase = static_cast<aerotelic::TpmsPhase>(Phases.IndexOfByKey(Phase));
		S.axis = static_cast<aerotelic::PrismAxis>(Axes.IndexOfByKey(Axis));
		S.wall_sizing = static_cast<aerotelic::WallSizing>(Sizings.IndexOfByKey(Sizing));
		S.type = TCHAR_TO_UTF8(*(Type.IsValid() ? *Type : FString()));
		return S;
	};
	auto ChosenOperands = [](const TSharedPtr<SAerotelicBodyPicker>& Picker)
	{
		TArray<FAerotelicBooleanOperand> Out;
		if (Picker.IsValid())
		{
			for (const FAerotelicBodyChoicePtr& C : Picker->GetChosen()) Out.Add(C->Operand);
		}
		return Out;
	};

	// The grid the generator would use, from the chosen bodies' bounds.
	auto UpdateEstimate = [&]()
	{
		if (!EstimateLabel.IsValid())
		{
			return;
		}
		FBox Bounds(ForceInit);
		for (const FAerotelicBooleanOperand& Op : ChosenOperands(TargetPicker))
		{
			if (Op.Entity.IsValid()) Bounds += Op.Entity->GetPlacedBounds(Op.bMirror);
		}
		if (!Bounds.IsValid || !(Spec.cell_size > 0.0))
		{
			EstimateLabel->SetText(LOCTEXT("NoEstimate", "Grid: choose at least one body to fill."));
			return;
		}
		aerotelic::Box3 B;
		B.Add(aerotelic::Vec3{Bounds.Min.X, Bounds.Min.Y, Bounds.Min.Z});
		B.Add(aerotelic::Vec3{Bounds.Max.X, Bounds.Max.Y, Bounds.Max.Z});
		const aerotelic::LatticeSpec S = CurrentSpec();
		FString Text, Trouble;
		try
		{
			const aerotelic::LatticeGrid G = aerotelic::PlanLatticeGrid(S, B);
			const double Cells = Bounds.GetSize().X / Spec.cell_size * Bounds.GetSize().Y / Spec.cell_size * Bounds.GetSize().Z / Spec.cell_size;
			Text = FString::Printf(TEXT("Grid: %d x %d x %d = %s samples (%.3g unit cells across the bounds)"),
				G.nx, G.ny, G.nz, *Thousands(static_cast<int64>(G.Nodes())), Cells);
			// A honeycomb's wall is set apart from its cell, so say how thick
			// it came out and whether the grid can carry it.
			if (S.family == aerotelic::LatticeFamily::Honeycomb)
			{
				const double Wall = aerotelic::HoneycombWallThickness(S);
				Text += FString::Printf(TEXT("\nWall: %.4g thick, %.1f voxels across"), Wall, Wall / G.h);
			}
			if (G.Nodes() > 40000000)
			{
				Trouble = TEXT("That is a lot: expect a long wait and a very large mesh. A larger cell or a lower resolution is cheaper.");
			}
			const FString Why = UTF8_TO_TCHAR(aerotelic::LatticeResolutionWarning(S, G).c_str());
			if (!Why.IsEmpty())
			{
				Trouble = Why;
			}
		}
		catch (const std::exception& E)
		{
			Trouble = UTF8_TO_TCHAR(E.what());
		}
		if (!Trouble.IsEmpty())
		{
			Text += (Text.IsEmpty() ? TEXT("") : TEXT("\n"));
			Text += Trouble;
		}
		EstimateLabel->SetText(FText::FromString(Text));
		EstimateLabel->SetColorAndOpacity(FSlateColor(Trouble.IsEmpty() ? FLinearColor(0.85f, 0.85f, 0.85f) : FLinearColor(1.0f, 0.6f, 0.3f)));
	};

	// Which parameter rows apply: the TPMS phase and the honeycomb's own
	// three, and whether the thickness or the fraction is the live knob.
	auto UpdateRows = [&]()
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
	};

	auto Spin = [&](double& Var, double Min, double Max, double Delta, const FText& Suffix)
	{
		return SNew(SAerotelicNumericEntry)
			.Value_Lambda([&Var]() { return Var; })
			.MinValue(Min).MaxValue(Max).Delta(Delta).Suffix(Suffix)
			.LiveUpdate(false)
			.OnValueCommitted_Lambda([&Var, &UpdateEstimate](double V, ETextCommit::Type) { Var = V; UpdateEstimate(); });
	};
	double Resolution = Spec.resolution;

	Window->SetContent(
		SNew(SBorder).Padding(12)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().FillHeight(1.0f)
			[
				SNew(SScrollBox)
				+ SScrollBox::Slot().Padding(0, 0, 0, 8)
				[
					MakeGroup(LOCTEXT("LatticeGroup", "Lattice"),
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
						[ FormRow(LOCTEXT("Family", "Family:"), Combo(&Families, Family, [&](TSharedPtr<FString>)
							{
								Types = Options(aerotelic::LatticeTypeNames(static_cast<aerotelic::LatticeFamily>(Families.IndexOfByKey(Family))));
								Type = Types.Num() ? Types[0] : nullptr;
								if (TypeCombo.IsValid()) { TypeCombo->RefreshOptions(); TypeCombo->SetSelectedItem(Type); }
								UpdateRows();
								UpdateEstimate();
							})) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
						[ FormRow(LOCTEXT("Type", "Type:"), SAssignNew(TypeCombo, SComboBox<TSharedPtr<FString>>)
							.OptionsSource(&Types)
							.InitiallySelectedItem(Type)
							.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
							.OnSelectionChanged_Lambda([&Type, &UpdateRows, &UpdateEstimate](TSharedPtr<FString> S, ESelectInfo::Type)
								{ if (S.IsValid()) { Type = S; UpdateRows(); UpdateEstimate(); } })
							[ SNew(STextBlock).Text_Lambda([&Type]() { return FText::FromString(Type.IsValid() ? *Type : FString()); }) ]) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
						[ FormRow(LOCTEXT("Phase", "TPMS phase:"), SAssignNew(PhaseCombo, SComboBox<TSharedPtr<FString>>)
							.OptionsSource(&Phases)
							.InitiallySelectedItem(Phase)
							.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
							.OnSelectionChanged_Lambda([&Phase, &UpdateRows, &UpdateEstimate](TSharedPtr<FString> S, ESelectInfo::Type)
								{ if (S.IsValid()) { Phase = S; UpdateRows(); UpdateEstimate(); } })
							[ SNew(STextBlock).Text_Lambda([&Phase]() { return FText::FromString(Phase.IsValid() ? *Phase : FString()); }) ]) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
						[ SAssignNew(AxisRow, SBox) [ FormRow(LOCTEXT("Axis", "Prism axis:"), Combo(&Axes, Axis, [&](TSharedPtr<FString>) { UpdateEstimate(); })) ] ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
						[ MakeInfoLabel(LOCTEXT("FamilyHint", "TPMS: a level-set surface (27 types, after OpenLattice's library). Network keeps one side of it; Sheet thickens it to the volume fraction; Iso-surface thickens it to a wall thickness. Strut-based and auxetic cells are unions of cylindrical struts. Honeycomb cells are a 2D tiling's walls extruded along the prism axis -- a sandwich core, stiff along that axis and soft across it; the other families are much the same in every direction. The prism axis applies to honeycombs only.")) ])
				]
				+ SScrollBox::Slot().Padding(0, 0, 0, 8)
				[
					MakeGroup(LOCTEXT("Parameters", "Parameters"),
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("CellSize", "Unit cell size:"), Spin(Spec.cell_size, 1e-6, 1e9, 0.01, FText::GetEmpty())) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ SAssignNew(AspectRow, SBox) [ FormRow(LOCTEXT("Aspect", "Cell aspect:"), Spin(Spec.cell_aspect, 0.05, 20.0, 0.1, FText::GetEmpty())) ] ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ SAssignNew(SizingRow, SBox) [ FormRow(LOCTEXT("Sizing", "Wall set by:"), Combo(&Sizings, Sizing, [&](TSharedPtr<FString>) { UpdateRows(); UpdateEstimate(); })) ] ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ SAssignNew(FractionRow, SBox) [ FormRow(LOCTEXT("Fraction", "Volume fraction:"), Spin(Spec.volume_fraction, 0.02, 0.95, 0.05, FText::GetEmpty())) ] ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ SAssignNew(WallRow, SBox) [ FormRow(LOCTEXT("Wall", "Wall thickness:"), Spin(Spec.wall_thickness, 1e-6, 1e9, 0.001, FText::GetEmpty())) ] ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Resolution", "Voxels per cell:"), Spin(Resolution, 2, 64, 1, FText::GetEmpty())) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ FormRow(LOCTEXT("Skin", "Solid skin thickness:"), Spin(Spec.skin_thickness, 1e-6, 1e9, 0.001, FText::GetEmpty())) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
						[ MakeInfoLabel(LOCTEXT("ParamHint", "Sizes are in model units. Volume fraction is the lattice's share of the filled volume (its level-set parameter, strut radius or honeycomb wall thickness is calibrated to it). For a honeycomb, the unit cell size is the side of a Square or Triangle, the width of a Rectangle or a Hexagon's size across the flats, and the cell aspect stretches the cross-section's second dimension by that factor (1 is the regular cell; Square ignores it). A honeycomb wall can be set by thickness instead of volume fraction, which is what a printed or bonded core is actually specified by -- whole bead widths or foil gauge -- and the achieved fraction then follows from it. Voxels per cell is the sampling resolution of the surface extraction (OpenLattice's mesh density); 8-16 is usual, more is finer and slower. The skin is the part's solid outer shell to that depth, always present (one percent of the largest root chord by default): each body is hollowed by MeshLib's inward offset of its surface (so any thickness is fine, and a concave corner is rounded from the inside), giving its own body, 'Skin', beside the 'Lattice' body -- the lattice cut to the hollowed interior by a mesh boolean -- so the two can be coloured, exported and given materials separately. Lattice > Export Skin Tetrahedra / Export Tetrahedral Mesh mesh each body with gmsh into skin and core tetrahedra whose outer faces are the body's own triangles and whose volumes sum to the body's exactly. A region thinner than two skins comes out solid. Needs MeshLib and gmsh in the WingDesigner Python environment.")) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
						[ SAssignNew(EstimateLabel, STextBlock).AutoWrapText(true) ])
				]
				+ SScrollBox::Slot().Padding(0, 0, 0, 8)
				[
					MakeGroup(LOCTEXT("Targets", "Bodies to fill"),
						SAssignNew(TargetPicker, SAerotelicBodyPicker).Choices(Fillable).InitiallyChosen(InitialTargets).ListHeight(150.0f)
						.OnChanged(FSimpleDelegate::CreateLambda([&UpdateEstimate]() { UpdateEstimate(); })))
				]
				+ SScrollBox::Slot().Padding(0, 0, 0, 8)
				[
					MakeGroup(LOCTEXT("Reserved", "Reserved volumes (left solid-free)"),
						Reserved.Num() == 0
							? StaticCastSharedRef<SWidget>(MakeInfoLabel(LOCTEXT("NoPrimitives", "No primitives in the scene. Add a sphere, box or cylinder to mark a payload bay or other volume the lattice must leave alone.")))
							: StaticCastSharedRef<SWidget>(SAssignNew(ReservedPicker, SAerotelicBodyPicker).Choices(Reserved).InitiallyChosen(InitialReserved).ListHeight(110.0f)))
				]
				+ SScrollBox::Slot().Padding(0, 0, 0, 4)
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0) [ SNew(STextBlock).Text(LOCTEXT("Name", "Name:")) ]
					+ SHorizontalBox::Slot().FillWidth(1.0f) [ SAssignNew(NameBox, SEditableTextBox).Text(FText::FromString(DefaultName)).IsEnabled(!bEdit) ]
				]
			]
			+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right).Padding(0, 8, 0, 0)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[ Button(bEdit ? LOCTEXT("Regenerate", "Regenerate") : LOCTEXT("Generate", "Generate"), [&]() { bAccepted = true; Window->RequestDestroyWindow(); }) ]
				+ SHorizontalBox::Slot().AutoWidth()
				[ Button(LOCTEXT("Cancel", "Cancel"), [Window]() { Window->RequestDestroyWindow(); }) ]
			]
		]);

	UpdateRows();
	UpdateEstimate();
	FSlateApplication::Get().AddModalWindow(Window, FSlateApplication::Get().GetActiveTopLevelWindow());
	if (!bAccepted)
	{
		return;
	}

	aerotelic::LatticeSpec Final = CurrentSpec();
	Final.resolution = FMath::Clamp(FMath::RoundToInt(Resolution), 2, 64);
	TArray<FAerotelicBooleanOperand> Targets = ChosenOperands(TargetPicker);
	TArray<FAerotelicBooleanOperand> Exclusions = ChosenOperands(ReservedPicker);
	if (Targets.Num() == 0)
	{
		AerotelicMessageBox(EAppMsgType::Ok, LOCTEXT("NeedTarget", "Select at least one body to fill."), LOCTEXT("Title", "Generate Lattice"));
		return;
	}
	if (!(Final.skin_thickness > 0.0))
	{
		AerotelicMessageBox(EAppMsgType::Ok, LOCTEXT("NeedSkin", "The solid skin thickness must be greater than zero: the skin is the part's outer shell."), LOCTEXT("Title", "Generate Lattice"));
		return;
	}
	FString Name = NameBox.IsValid() ? NameBox->GetText().ToString().TrimStartAndEnd() : FString();
	if (Name.IsEmpty())
	{
		Name = DefaultName;
	}
	if (!bEdit && Doc->IsNameTaken(Name))
	{
		AerotelicMessageBox(EAppMsgType::Ok,
			FText::Format(LOCTEXT("NameTaken", "'{0}' is already in use. Names must be unique in the scene."), FText::FromString(Name)),
			LOCTEXT("Title", "Generate Lattice"));
		return;
	}

	Status(LOCTEXT("Generating", "Generating lattice..."));
	FString Error;
	AAerotelicEntityActor* Result = Existing;
	if (bEdit)
	{
		if (!Doc->UpdateLattice(Existing, Final, Targets, Exclusions, Error))
		{
			Result = nullptr;
		}
	}
	else
	{
		Result = Doc->AddLattice(Name, Final, Targets, Exclusions, Error);
	}
	if (Result == nullptr)
	{
		Status(LOCTEXT("Ready", "Ready"));
		AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(Error), LOCTEXT("Failed", "Lattice Generation Failed"));
		return;
	}
	Doc->SetSelectedEntity(Result);
	const FAerotelicLatticeModel::FStats& S = Result->GetModelAs<FAerotelicLatticeModel>()->GetStats();
	Status(FText::FromString(FString::Printf(TEXT("%s '%s': %s %s, %s triangles, volume fraction %.1f%% achieved, %d x %d x %d samples in %.1f s"),
		bEdit ? TEXT("Regenerated") : TEXT("Generated"), *Result->EntityName, *(Family.IsValid() ? *Family : FString()), *(Type.IsValid() ? *Type : FString()),
		*Thousands(S.Triangles), 100.0 * S.AchievedFraction, S.GridX, S.GridY, S.GridZ, S.Seconds)));
}

#undef LOCTEXT_NAMESPACE
