#include "MaterialLibraryDialog.h"

#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Input/SMultiLineEditableTextBox.h"
#include "Widgets/Views/SListView.h"
#include "Widgets/Views/STableRow.h"
#include "Widgets/Views/SHeaderRow.h"
#include "Framework/Application/SlateApplication.h"
#include "Styling/CoreStyle.h"

#include "Widgets/TabStrip.h"
#include "Widgets/CurveTabControls.h"
#include "Widgets/NumericEntry.h"
#include "Widgets/AerotelicDialogWindow.h"
#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicMaterialLibrary.h"

#include "aerotelic/materials.hpp"
#include "aerotelic/units.hpp"

#define LOCTEXT_NAMESPACE "MaterialLibraryDialog"

using aerotelic::Material;

namespace
{
	struct FRow
	{
		Material M;
		bool bCustom = false;
	};
	using FRowPtr = TSharedPtr<FRow>;

	const FName ColName("Name"), ColCategory("Category"), ColDensity("Density"), ColModulus("Modulus"),
		ColPoisson("Poisson"), ColAllowable("Allowable"), ColSource("Source");

	FString Str(const std::string& S) { return FString(UTF8_TO_TCHAR(S.c_str())); }
	std::string Std(const FString& S) { return std::string(TCHAR_TO_UTF8(*S)); }

	FText WithUnit(const FText& Label, const std::string& Unit)
	{
		return FText::Format(LOCTEXT("Labelled", "{0} ({1})"), Label, FText::FromString(Str(Unit)));
	}

	// Everything the three tabs share. The dialog is modal, so one State
	// outlives every widget that captures it.
	struct FState : public TSharedFromThis<FState>
	{
		aerotelic::UnitSystem Units;
		TFunction<void(const FText&)> Status;
		TSharedPtr<SAerotelicTabStrip> Tabs;

		// Materials tab: every material, sorted as the headers say. None
		// means the library's own order (the presets by family, then yours).
		TArray<FRowPtr> Rows;
		TSharedPtr<SListView<FRowPtr>> Table;
		FName SortColumn = ColName;
		EColumnSortMode::Type SortMode = EColumnSortMode::None;

		// Custom tab: the user's materials and the one being typed.
		TArray<FRowPtr> CustomRows;
		TSharedPtr<SListView<FRowPtr>> CustomList;
		Material Draft;
		TSharedPtr<SEditableTextBox> NameBox, CategoryBox;
		TSharedPtr<SMultiLineEditableTextBox> SourceBox;
		FText Note;

		FState()
		{
			ClearDraft();
		}

		void ClearDraft()
		{
			Draft = Material{};
			Draft.name.clear();
			Draft.category = "Custom";
			PushDraftToBoxes();
		}

		void LoadDraft(const Material& M, bool bAsCopy)
		{
			Draft = M;
			if (bAsCopy)
			{
				Draft.name = M.name + " (custom)";
			}
			PushDraftToBoxes();
		}

		void PushDraftToBoxes()
		{
			if (NameBox.IsValid()) NameBox->SetText(FText::FromString(Str(Draft.name)));
			if (CategoryBox.IsValid()) CategoryBox->SetText(FText::FromString(Str(Draft.category)));
			if (SourceBox.IsValid()) SourceBox->SetText(FText::FromString(Str(Draft.source)));
		}

		void RebuildRows()
		{
			Rows.Reset();
			CustomRows.Reset();
			const int32 PresetCount = (int32)aerotelic::MaterialPresets().size();
			int32 Index = 0;
			for (const Material& M : AerotelicMaterialLibrary::All())
			{
				FRowPtr R = MakeShared<FRow>();
				R->M = M;
				R->bCustom = Index++ >= PresetCount;
				Rows.Add(R);
				if (R->bCustom) CustomRows.Add(R);
			}
			SortRows();
			if (CustomList.IsValid()) CustomList->RequestListRefresh();
		}

		static int32 CompareText(const std::string& A, const std::string& B)
		{
			return Str(A).Compare(Str(B), ESearchCase::IgnoreCase);
		}
		static int32 CompareNumber(double A, double B)
		{
			return A < B ? -1 : (A > B ? 1 : 0);
		}
		int32 Compare(const FRow& A, const FRow& B) const
		{
			if (SortColumn == ColName) return CompareText(A.M.name, B.M.name);
			if (SortColumn == ColCategory) return CompareText(A.M.category, B.M.category);
			if (SortColumn == ColDensity) return CompareNumber(A.M.density, B.M.density);
			if (SortColumn == ColModulus) return CompareNumber(A.M.youngs_modulus, B.M.youngs_modulus);
			if (SortColumn == ColPoisson) return CompareNumber(A.M.poisson_ratio, B.M.poisson_ratio);
			if (SortColumn == ColAllowable) return CompareNumber(A.M.allowable_stress, B.M.allowable_stress);
			if (SortColumn == ColSource) return CompareText(A.M.source, B.M.source);
			return 0;
		}

		void SortRows()
		{
			if (SortMode == EColumnSortMode::None)
			{
				// Back to the library's order.
				TArray<FRowPtr> Ordered;
				for (const FRowPtr& R : Rows) if (!R->bCustom) Ordered.Add(R);
				for (const FRowPtr& R : Rows) if (R->bCustom) Ordered.Add(R);
				Rows = Ordered;
			}
			else
			{
				const bool bDescending = SortMode == EColumnSortMode::Descending;
				Rows.StableSort([this, bDescending](const FRowPtr& A, const FRowPtr& B)
				{
					int32 C = Compare(*A, *B);
					if (C == 0) C = CompareText(A->M.name, B->M.name);   // ties by name, always ascending
					return bDescending ? C > 0 : C < 0;
				});
			}
			if (Table.IsValid()) Table->RequestListRefresh();
		}

		void Say(const FText& What)
		{
			Note = What;
			if (Status) Status(What);
		}
	};

	class SMaterialRow : public SMultiColumnTableRow<FRowPtr>
	{
	public:
		SLATE_BEGIN_ARGS(SMaterialRow) {}
		SLATE_END_ARGS()

		void Construct(const FArguments& InArgs, const TSharedRef<STableViewBase>& Owner, FRowPtr InRow, const aerotelic::UnitSystem& InUnits)
		{
			Row = InRow;
			Units = InUnits;
			SMultiColumnTableRow<FRowPtr>::Construct(FSuperRowType::FArguments().Padding(FMargin(2, 1)), Owner);
		}

		virtual TSharedRef<SWidget> GenerateWidgetForColumn(const FName& Column) override
		{
			const Material& M = Row->M;
			FText Text, Tip;
			bool bRight = false;
			if (Column == ColName) { Text = FText::FromString(Str(M.name)); Tip = Text; }
			else if (Column == ColCategory) Text = FText::FromString(Str(M.category));
			else if (Column == ColDensity) { Text = FormatG(M.density / Units.DensityToSI(), 4); bRight = true; }
			else if (Column == ColModulus) { Text = FormatG(M.youngs_modulus / Units.StressToSI(), 4); bRight = true; }
			else if (Column == ColPoisson) { Text = FormatG(M.poisson_ratio, 3); bRight = true; }
			else if (Column == ColAllowable) { Text = FormatG(M.allowable_stress / Units.StressToSI(), 4); bRight = true; }
			else if (Column == ColSource) { Text = FText::FromString(Str(M.source)); Tip = Text; }
			return SNew(SBox).Padding(FMargin(4, 1)).HAlign(bRight ? HAlign_Right : HAlign_Left).VAlign(VAlign_Center)
				[
					SNew(STextBlock)
					.Text(Text)
					.ToolTipText(Tip)
					.OverflowPolicy(ETextOverflowPolicy::Ellipsis)
					.Font(FCoreStyle::GetDefaultFontStyle(Row->bCustom ? "Bold" : "Regular", 9))
					.ColorAndOpacity(Row->bCustom ? FSlateColor(FLinearColor(0.72f, 0.88f, 1.0f)) : FSlateColor::UseForeground())
				];
		}

	private:
		FRowPtr Row;
		aerotelic::UnitSystem Units;
	};

	TSharedRef<SWidget> FormRow(const FText& Label, TSharedRef<SWidget> Control, float LabelWidth = 190.0f)
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 2, 8, 2)
			[ SNew(SBox).WidthOverride(LabelWidth) [ SNew(STextBlock).Text(Label) ] ]
			+ SHorizontalBox::Slot().FillWidth(1).Padding(1) [ Control ];
	}

	TSharedRef<SWidget> ListFrame(TSharedRef<SWidget> List)
	{
		return SNew(SBorder).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(FLinearColor(0.05f, 0.05f, 0.06f)).Padding(2)
			[ List ];
	}

	const FText kHelp = LOCTEXT("Help",
		"MATERIAL PROPERTIES\n"
		"\n"
		"A material is what a solid body is made of: its density, Young's modulus, Poisson's ratio and the allowable stress a margin is reported against. "
		"Analysis > Mass Properties weighs a body by its density (over the volume it encloses, or over its skin), and Analysis > Structural (Kratos FEA) analyses it with the other three. "
		"A material is a property of the stuff, not of the project's units: every figure is kept in SI (kg/m^3, Pa) and shown here in the project's units (Model > Units), so changing the units changes the numbers on screen and nothing underneath.\n"
		"\n"
		"THE MATERIALS TAB\n"
		"\n"
		"One row per material: the presets the application ships with, then your own (in bold). Click a column header to sort the table by that column, ascending; click it again for descending. "
		"The Source column says where a preset's figures came from and what convention they follow; hover a row to read the whole of it. "
		"Double-click a row to take it to the Custom tab as the starting point for a material of your own.\n"
		"\n"
		"THE CUSTOM TAB\n"
		"\n"
		"Define your own materials. Pick a material under 'Start from' to copy its figures into the fields (or select one of yours on the left to edit it), then type a name, a category of your choosing, the four figures in the project's units, and where the figures came from. "
		"'Add as new' adds the material under the name typed; 'Update selected' overwrites the material selected on the left with the fields (renaming it if the name was changed); 'Remove selected' deletes it. "
		"Names must be unique and may not be a preset's. A material needs a positive density and modulus and a Poisson's ratio in [0, 0.5); an allowable of zero means no margin is reported.\n"
		"\n"
		"Your materials are written at once to one file per user (shown at the bottom of the Custom tab), so they are offered in every project you open on this machine. "
		"An entity is given a COPY of the material it is assigned (the name and the figures, saved in the project file), so editing or removing a library material changes nothing already assigned: pick it again on the entity to take the new figures. "
		"A project file carries its own materials, so it opens the same on a machine without your library.\n"
		"\n"
		"ASSIGNING A MATERIAL\n"
		"\n"
		"Right-click a parameterisation in the Scene tree and choose Material Properties: pick the material from the list, give the skin thickness (0 for a solid body; a positive thickness makes every body of the entity a skin of the material that thick, which is also what a lattice hollows it by and what the structural analysis gives its shells), and read off the enclosed volume, the surface area and what it would weigh. "
		"A boolean result or a lattice inherits the material of what it is built from.\n"
		"\n"
		"CONVENTIONS OF THE PRESETS\n"
		"\n"
		"Metals: room-temperature handbook values. The seven original rows (2024-T3, 7075-T6, 6061-T6, Ti-6Al-4V annealed, 4130 normalised, 17-4PH H900, AZ31B-H24) are TYPICAL values whose allowable is the typical tensile yield. "
		"The alloy tables added after them (at least ten aluminium, steel, magnesium and titanium alloys) are MIL-HDBK-5J design values: their allowable is the handbook's MINIMUM tensile yield -- A-basis (99% of material exceeds it, 95% confidence) where the handbook gives one, else S-basis (the procurement specification's minimum) -- longitudinal, for the product form, condition and thickness range the Source column names, so it is a design minimum rather than a typical figure and sits 5-15% below the typical yield of the same alloy. "
		"Magnesium casting rows are the handbook's 'any area' minimums, which are well below separately cast test-bar figures. Where the handbook gives no Poisson's ratio (some titanium alloys) a literature value is used and the Source column says so.\n"
		"Laminates: representative quasi-isotropic in-plane figures for a carbon/epoxy and an E-glass/epoxy laminate, not any one supplier's; the allowable is a representative laminate strength.\n"
		"Woods: the USDA Wood Handbook's clear-wood figures along the grain at 12% moisture, density from its specific gravity, Poisson's ratio nu_LR from its Table 5-1; the allowable is the compression strength parallel to the grain (the lower of the two, what a spar cap buckles at). Balsa is sold by weight, so three grades are given -- contest-light, medium, hard -- with the Handbook's figures scaled in proportion to density (its stiffness and strength go roughly as its density: Borrega & Gibson 2015). The two plywoods (aircraft birch, poplar lite-ply) take the solid wood at two-thirds along the face grain of a three-ply sheet, ANC-18's approach, and an approximate in-plane Poisson's ratio of 0.2. Weigh a sheet: hobby stock varies more than any handbook.\n"
		"Foam: the data-sheet figure for the nominal grade; the allowable sits between the compressive and tensile strengths.\n"
		"SLA / LCD / DLP resins: the manufacturers' technical data sheets, POST-CURED figures (ASTM D638 tensile modulus and strength); the allowable is the tensile yield where the sheet gives one, else the strength at break; densities from the sheet or Formlabs' post-cured density table. Poisson's ratio is not on the sheets: 0.35 is the literature figure for cured acrylates (0.30 glass-filled, 0.38 for the soft polypropylene-like and ductile resins, 0.42 for the near-elastomeric one). "
		"Anycubic's sheets give ranges, no post-cure and no tensile modulus (the store pages add a Young's modulus range): the LOWER bound of the modulus and of the tensile strength and the UPPER bound of the density were taken, a conservative card.\n"
		"FDM thermoplastics: the filament makers' technical data sheets, AS PRINTED, flat (XY / XZ) orientation, dry; the allowable is the tensile yield where given, else the tensile strength. A printed part is weaker across its layers (Z strength is typically 60-85% of XY on these sheets) and its stiffness depends on walls and infill: these are the figures for a solid part loaded along the layers, an upper bound for an isotropic analysis. Poisson's ratios are literature values for printed parts.\n"
		"\n"
		"SOURCES\n"
		"\n"
		"Metals, the seven original rows: MMPDS-01 (Metallic Materials Properties Development and Standardization, DOT/FAA/AR-MMPDS-01, FAA/Battelle, 2003) and ASM Handbook vol. 1 (Irons, Steels and High-Performance Alloys) and vol. 2 (Nonferrous Alloys), ASM International; AK Steel 17-4 PH stainless steel product data bulletin.\n"
		"Metals, the alloy tables: MIL-HDBK-5J, Metallic Materials and Elements for Aerospace Vehicle Structures, US Department of Defense, 31 January 2003 (Distribution Statement A: approved for public release, distribution unlimited; a US Government work in the public domain) -- the 'Design Mechanical and Physical Properties' table of each alloy, cited by table number in the Source column: Chapter 2 steels, Chapter 3 aluminium, Chapter 4 magnesium, Chapter 5 titanium, Chapter 6 A-286. MMPDS-01 is the FAA's technically equivalent re-issue of it; later MMPDS editions are copyrighted by Battelle.\n"
		"Wood: Forest Products Laboratory, Wood Handbook -- Wood as an Engineering Material, General Technical Report FPL-GTR-190, USDA, 2010: Table 5-3 (Sitka spruce, American basswood, Eastern white pine, yellow-poplar, Coast Douglas-fir, yellow birch), Table 5-5 (balsa), Table 5-1 (Poisson's ratios), all at 12% moisture; "
		"M. Borrega and L. J. Gibson, 'Mechanics of balsa (Ochroma pyramidale) wood', Mechanics of Materials 84 (2015) 75-90, for the density scaling of the balsa grades; "
		"ANC-18, Design of Wood Aircraft Structures, ANC Bulletin, US Army-Navy-Civil Committee on Aircraft Design Criteria, 1951, for the plywood approach.\n"
		"Foam: Evonik Industries, ROHACELL IG / IG-F technical data sheet (grade 71).\n"
		"Resins: Formlabs technical data sheets -- Grey Resin V4, Clear Resin V4.1, Tough 2000 Resin V2, Durable Resin, High Temp Resin V2, Rigid 10K Resin -- and Formlabs support, 'Density of selected Formlabs SLA resins' (post-cured); Siraya Tech, Blu resin technical data sheet; "
		"Anycubic technical data sheets (store.anycubic.com, resin user manual index) and store pages for Standard, Standard V2, 16K Standard, ABS-Like 2.0, ABS-Like V2, ABS-Like 3.0, ABS-Like Pro 2, Water-Wash ABS-Like 3.0, Water-Wash 2.0, Tough 2.0, Tough Ultra, High Clear, High Speed 2.0, Plant-Based 2.0, DLP Craftsman, 14K Texture and Bio resins.\n"
		"FDM: Prusa Research, Prusament PLA, PETG, ASA and PC Blend technical data sheets; Bambu Lab, ABS and PAHT-CF technical data sheets; Polymaker, PolyMide CoPA technical data sheet; Markforged, composites material data sheet (Onyx); Stratasys, ULTEM 9085 resin data sheet (XZ orientation); 3DXTech, ThermaX PEEK technical data sheet.\n"
		"Poisson's ratios of printed polymers: literature values for cured acrylate resins and printed PLA, ABS, PETG, nylon, PC, PEI and PEEK; 0.35 for fibre-filled filaments.\n"
		"\n"
		"The figures are the suppliers' and the handbooks' typical values, not design allowables for any particular part: a printed or moulded part's properties depend on its process, orientation, cure and environment. Check them against your own coupons before trusting a margin.");
}

void ShowAerotelicMaterialLibraryDialog(TFunction<void(const FText&)> Status)
{
	TSharedRef<FState> State = MakeShared<FState>();
	State->Status = Status;
	if (const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		State->Units = Doc->GetUnits();
	}
	const aerotelic::UnitSystem& Units = State->Units;
	State->RebuildRows();

	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(LOCTEXT("Title", "Material Properties"))
		.ClientSize(FVector2D(1180, 660))
		.SupportsMaximize(true)
		.SupportsMinimize(false);
	TWeakPtr<SWindow> WeakWindow = Window;

	auto Button = [](const FText& Label, const FText& Tip, TFunction<void()> Fn)
	{
		return SNew(SButton).Text(Label).ToolTipText(Tip).HAlign(HAlign_Center).ContentPadding(FMargin(12, 3))
			.OnClicked_Lambda([Fn]() { Fn(); return FReply::Handled(); });
	};

	// ------------------------------------------------------------ Materials
	// By value: the setters return a reference to the temporary FArguments,
	// which must not be what the lambda hands back.
	auto Column = [State](const FName& Id, const FText& Label, const FText& Tip) -> SHeaderRow::FColumn::FArguments
	{
		return SHeaderRow::Column(Id)
			.DefaultLabel(Label)
			.DefaultTooltip(Tip)
			.SortMode_Lambda([State, Id]() { return State->SortColumn == Id ? State->SortMode : EColumnSortMode::None; })
			.OnSort_Lambda([State](EColumnSortPriority::Type, const FName& Col, EColumnSortMode::Type Mode)
			{
				State->SortColumn = Col;
				State->SortMode = Mode;
				State->SortRows();
			});
	};

	TSharedRef<SWidget> MaterialsTab =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
		[
			MakeInfoLabel(FText::Format(LOCTEXT("TableHint",
				"Every material a body can be made of: the presets, then your own in bold (Custom tab). Click a column header to sort by it, again to reverse. Figures are in the project's units -- density in {0}, stresses in {1} (Model > Units) -- kept in SI underneath. Hover a row for its full source; double-click one to start a material of your own from it."),
				FText::FromString(Str(Units.DensityName())), FText::FromString(Str(Units.StressName()))))
		]
		+ SVerticalBox::Slot().FillHeight(1.0f)
		[
			ListFrame(
				SAssignNew(State->Table, SListView<FRowPtr>)
				.ListItemsSource(&State->Rows)
				.SelectionMode(ESelectionMode::Single)
				.OnGenerateRow_Lambda([State](FRowPtr Row, const TSharedRef<STableViewBase>& Owner) { return SNew(SMaterialRow, Owner, Row, State->Units); })
				.OnMouseButtonDoubleClick_Lambda([State](FRowPtr Row)
				{
					if (!Row.IsValid()) return;
					State->LoadDraft(Row->M, /*bAsCopy*/ !Row->bCustom);
					if (State->CustomList.IsValid())
					{
						if (Row->bCustom) State->CustomList->SetSelection(Row); else State->CustomList->ClearSelection();
					}
					if (State->Tabs.IsValid()) State->Tabs->SetActiveTab(1);
				})
				.HeaderRow
				(
					SNew(SHeaderRow)
					+ Column(ColName, LOCTEXT("ColName", "Material"), LOCTEXT("ColNameTip", "The material's name, as written to the project file when assigned")).FillWidth(2.4f)
					+ Column(ColCategory, LOCTEXT("ColCategory", "Category"), LOCTEXT("ColCategoryTip", "The family: Metal, Composite, Wood, Foam, SLA resin, FDM thermoplastic, or your own")).ManualWidth(125)
					+ Column(ColDensity, WithUnit(LOCTEXT("ColDensity", "Density"), Units.DensityName()), LOCTEXT("ColDensityTip", "Mass per unit volume, in the project's units")).ManualWidth(110)
					+ Column(ColModulus, WithUnit(LOCTEXT("ColModulus", "Young's modulus"), Units.StressName()), LOCTEXT("ColModulusTip", "Tensile modulus of elasticity, in the project's stress unit")).ManualWidth(135)
					+ Column(ColPoisson, LOCTEXT("ColPoisson", "Poisson's ratio"), LOCTEXT("ColPoissonTip", "Lateral over axial strain; 0.5 would be incompressible")).ManualWidth(105)
					+ Column(ColAllowable, WithUnit(LOCTEXT("ColAllowable", "Allowable stress"), Units.StressName()), LOCTEXT("ColAllowableTip", "The stress a margin is reported against (the yield, or the strength where there is no yield), in the project's stress unit")).ManualWidth(135)
					+ Column(ColSource, LOCTEXT("ColSource", "Source"), LOCTEXT("ColSourceTip", "Where the figures came from and the convention they follow; hover a row for the whole of it")).FillWidth(3.2f)
				))
		];

	// --------------------------------------------------------------- Custom
	auto Field = [State](double Material::* Member, double ToSI, double Delta, double Max)
	{
		return SNew(SAerotelicNumericEntry)
			.Value_Lambda([State, Member, ToSI]() { return (State->Draft.*Member) / ToSI; })
			.MinValue(0.0).MaxValue(Max).Delta(Delta)
			.LiveUpdate(true)
			.OnValueCommitted_Lambda([State, Member, ToSI](double V, ETextCommit::Type) { State->Draft.*Member = V * ToSI; });
	};

	TArray<TSharedPtr<FString>> StartOptions;
	for (const Material& M : AerotelicMaterialLibrary::All()) StartOptions.Add(MakeShared<FString>(Str(M.name)));

	auto SelectedCustom = [State]() -> FRowPtr
	{
		if (!State->CustomList.IsValid()) return nullptr;
		TArray<FRowPtr> Sel = State->CustomList->GetSelectedItems();
		return Sel.Num() == 1 ? Sel[0] : nullptr;
	};
	auto TrimmedDraft = [State]()
	{
		Material M = State->Draft;
		M.name = Std(Str(M.name).TrimStartAndEnd());
		M.category = Std(Str(M.category).TrimStartAndEnd());
		M.source = Std(Str(M.source).TrimStartAndEnd());
		return M;
	};
	auto Commit = [State](const TArray<Material>& NewCustom, const FText& Done, const FString& SelectName)
	{
		FString Error;
		if (!AerotelicMaterialLibrary::SetCustom(NewCustom, Error))
		{
			State->Say(FText::Format(LOCTEXT("Refused", "Not saved: {0}"), FText::FromString(Error)));
			return;
		}
		State->RebuildRows();
		if (State->CustomList.IsValid())
		{
			State->CustomList->ClearSelection();
			for (const FRowPtr& R : State->CustomRows)
			{
				if (Str(R->M.name) == SelectName) { State->CustomList->SetSelection(R); break; }
			}
		}
		State->Say(Done);
	};

	auto OnAdd = [State, Commit, TrimmedDraft]()
	{
		const Material M = TrimmedDraft();
		TArray<Material> New = AerotelicMaterialLibrary::Custom();
		for (const Material& Existing : New)
		{
			if (Existing.name == M.name)
			{
				State->Say(FText::Format(LOCTEXT("AlreadyThere", "Not saved: you already have a material named '{0}' -- select it on the left and Update it, or type another name"), FText::FromString(Str(M.name))));
				return;
			}
		}
		New.Add(M);
		Commit(New, FText::Format(LOCTEXT("Added", "Added '{0}' to your materials ({1})"), FText::FromString(Str(M.name)), FText::FromString(AerotelicMaterialLibrary::FilePath())), Str(M.name));
	};
	auto OnUpdate = [State, Commit, TrimmedDraft, SelectedCustom]()
	{
		const FRowPtr Sel = SelectedCustom();
		if (!Sel.IsValid())
		{
			State->Say(LOCTEXT("NoneSelectedUpdate", "Select one of your materials on the left to update it (or use Add as new)"));
			return;
		}
		const Material M = TrimmedDraft();
		TArray<Material> New = AerotelicMaterialLibrary::Custom();
		for (Material& Existing : New)
		{
			if (Existing.name == Sel->M.name) { Existing = M; break; }
		}
		Commit(New, FText::Format(LOCTEXT("Updated", "Updated '{0}'. Entities already assigned it keep the figures they were given until it is picked again on them."), FText::FromString(Str(M.name))), Str(M.name));
	};
	auto OnRemove = [State, Commit, SelectedCustom]()
	{
		const FRowPtr Sel = SelectedCustom();
		if (!Sel.IsValid())
		{
			State->Say(LOCTEXT("NoneSelectedRemove", "Select one of your materials on the left to remove it"));
			return;
		}
		TArray<Material> New = AerotelicMaterialLibrary::Custom();
		New.RemoveAll([&Sel](const Material& M) { return M.name == Sel->M.name; });
		Commit(New, FText::Format(LOCTEXT("Removed", "Removed '{0}' from your materials. Entities assigned it keep their copy."), FText::FromString(Str(Sel->M.name))), FString());
	};

	TSharedRef<SWidget> CustomTab =
		SNew(SHorizontalBox)
		// Your materials
		+ SHorizontalBox::Slot().FillWidth(0.36f).Padding(0, 0, 10, 0)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 2)
			[ SNew(STextBlock).Text(LOCTEXT("Yours", "Your materials")).Font(FCoreStyle::GetDefaultFontStyle("Bold", 9)) ]
			+ SVerticalBox::Slot().FillHeight(1.0f)
			[
				ListFrame(
					SAssignNew(State->CustomList, SListView<FRowPtr>)
					.ListItemsSource(&State->CustomRows)
					.SelectionMode(ESelectionMode::Single)
					.OnGenerateRow_Lambda([](FRowPtr Row, const TSharedRef<STableViewBase>& Owner)
					{
						return SNew(STableRow<FRowPtr>, Owner).Padding(FMargin(4, 2))
							[
								SNew(STextBlock).Text(FText::FromString(Str(Row->M.name)))
								.ToolTipText(FText::FromString(Str(Row->M.source)))
							];
					})
					.OnSelectionChanged_Lambda([State](FRowPtr Row, ESelectInfo::Type Info)
					{
						if (Row.IsValid() && Info != ESelectInfo::Direct) State->LoadDraft(Row->M, /*bAsCopy*/ false);
					}))
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
			[
				MakeInfoLabel(LOCTEXT("YoursHint", "Select one to edit it on the right. Nothing here yet? Fill in the fields and Add as new."))
			]
		]
		// The form
		+ SHorizontalBox::Slot().FillWidth(0.64f)
		[
			SNew(SScrollBox)
			+ SScrollBox::Slot()
			[
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight()
				[
					FormRow(LOCTEXT("StartFrom", "Start from:"),
						SNew(SComboBox<TSharedPtr<FString>>)
						.OptionsSource(&StartOptions)
						.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
						.OnSelectionChanged_Lambda([State](TSharedPtr<FString> S, ESelectInfo::Type Info)
						{
							if (!S.IsValid() || Info == ESelectInfo::Direct) return;
							if (const Material* M = AerotelicMaterialLibrary::Find(Std(*S)))
							{
								const bool bPreset = aerotelic::FindMaterialPreset(M->name) != nullptr;
								State->LoadDraft(*M, /*bAsCopy*/ bPreset);
								if (State->CustomList.IsValid()) State->CustomList->ClearSelection();
								State->Say(FText::Format(LOCTEXT("StartedFrom", "Fields filled from '{0}'; change what you like, then Add as new"), FText::FromString(*S)));
							}
						})
						[ SNew(STextBlock).Text(LOCTEXT("StartFromPrompt", "Copy a material's figures into the fields...")) ])
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0)
				[
					FormRow(LOCTEXT("Name", "Name:"),
						SAssignNew(State->NameBox, SEditableTextBox)
						.Text(FText::FromString(Str(State->Draft.name)))
						.HintText(LOCTEXT("NameHint", "e.g. Balsa, end grain  or  Formlabs Tough 1500 (my coupons)"))
						.OnTextChanged_Lambda([State](const FText& T) { State->Draft.name = Std(T.ToString()); }))
				]
				+ SVerticalBox::Slot().AutoHeight()
				[
					FormRow(LOCTEXT("Category", "Category:"),
						SAssignNew(State->CategoryBox, SEditableTextBox)
						.Text(FText::FromString(Str(State->Draft.category)))
						.HintText(LOCTEXT("CategoryHint", "Metal, Composite, Wood, Foam, SLA resin, FDM thermoplastic, or your own"))
						.OnTextChanged_Lambda([State](const FText& T) { State->Draft.category = Std(T.ToString()); }))
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0)
				[ FormRow(WithUnit(LOCTEXT("Density", "Density"), Units.DensityName()), Field(&Material::density, Units.DensityToSI(), 10.0, 1.0e9)) ]
				+ SVerticalBox::Slot().AutoHeight()
				[ FormRow(WithUnit(LOCTEXT("Young", "Young's modulus"), Units.StressName()), Field(&Material::youngs_modulus, Units.StressToSI(), 1.0e9 / Units.StressToSI(), 1.0e15)) ]
				+ SVerticalBox::Slot().AutoHeight()
				[ FormRow(LOCTEXT("Poisson", "Poisson's ratio:"), Field(&Material::poisson_ratio, 1.0, 0.01, 0.4999)) ]
				+ SVerticalBox::Slot().AutoHeight()
				[ FormRow(WithUnit(LOCTEXT("Allowable", "Allowable stress"), Units.StressName()), Field(&Material::allowable_stress, Units.StressToSI(), 1.0e6 / Units.StressToSI(), 1.0e15)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0)
				[
					FormRow(LOCTEXT("Source", "Source / notes:"),
						SNew(SBox).HeightOverride(64)
						[
							SAssignNew(State->SourceBox, SMultiLineEditableTextBox)
							.Text(FText::FromString(Str(State->Draft.source)))
							.HintText(LOCTEXT("SourceHint", "Where the figures came from: the data sheet, the handbook, your own coupons; the condition and orientation; what the allowable is"))
							.AutoWrapText(true)
							.OnTextChanged_Lambda([State](const FText& T) { State->Draft.source = Std(T.ToString()); })
						])
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
					[ Button(LOCTEXT("Add", "Add as new"), LOCTEXT("AddTip", "Add the fields as a new material of yours, under the name typed"), OnAdd) ]
					+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
					[ Button(LOCTEXT("Update", "Update selected"), LOCTEXT("UpdateTip", "Overwrite the material selected on the left with the fields (a changed name renames it)"), OnUpdate) ]
					+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
					[ Button(LOCTEXT("Remove", "Remove selected"), LOCTEXT("RemoveTip", "Delete the material selected on the left from your library"), OnRemove) ]
					+ SHorizontalBox::Slot().AutoWidth()
					[ Button(LOCTEXT("Clear", "Clear fields"), LOCTEXT("ClearTip", "Empty the fields to start a material from nothing"), [State]() { State->ClearDraft(); if (State->CustomList.IsValid()) State->CustomList->ClearSelection(); State->Say(FText::GetEmpty()); }) ]
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0)
				[
					SNew(STextBlock).AutoWrapText(true)
					.Text_Lambda([State]() { return State->Note; })
					.ColorAndOpacity(FSlateColor(FLinearColor(0.95f, 0.85f, 0.55f)))
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
				[
					MakeInfoLabel(FText::Format(LOCTEXT("CustomHint",
						"Figures are typed in the project's units (Model > Units) and kept in SI, as every material is. A name must be unique and may not be a preset's; a material needs a positive density and modulus and a Poisson's ratio below 0.5; an allowable of zero reports no margin. "
						"Add, Update and Remove write your library at once to {0}, and the Scene tree's Material Properties offers it in every project from then on. An entity keeps a copy of the material it was assigned (the project file carries the figures), so editing one here changes nothing already assigned until it is picked again."),
						FText::FromString(AerotelicMaterialLibrary::FilePath())))
				]
			]
		];

	// ------------------------------------------------------------------ tabs
	TArray<FAerotelicTab> Tabs;
	Tabs.Add({ LOCTEXT("MaterialsTab", "Materials"), MaterialsTab });
	Tabs.Add({ LOCTEXT("CustomTab", "Custom"), CustomTab });
	Tabs.Add({ LOCTEXT("HelpTab", "Help"), MakeHelpTab(kHelp) });

	Window->SetContent(
		SNew(SBorder).Padding(12)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().FillHeight(1.0f).Padding(0, 0, 0, 10)
			[ SAssignNew(State->Tabs, SAerotelicTabStrip, Tabs) ]
			+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
			[
				Button(LOCTEXT("Close", "Close"), LOCTEXT("CloseTip", "Close; everything added or changed is already saved"),
					[WeakWindow]() { if (TSharedPtr<SWindow> W = WeakWindow.Pin()) W->RequestDestroyWindow(); })
			]
		]);

	FSlateApplication::Get().AddModalWindow(Window, FindAerotelicMainWindow());
}

#undef LOCTEXT_NAMESPACE
