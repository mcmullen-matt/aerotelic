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

#include "aerotelic/laminate.hpp"
#include "aerotelic/materials.hpp"
#include "aerotelic/units.hpp"

#define LOCTEXT_NAMESPACE "MaterialLibraryDialog"

using aerotelic::Material;
using aerotelic::Lamina;

namespace
{
	struct FRow
	{
		Material M;
		bool bCustom = false;
	};
	using FRowPtr = TSharedPtr<FRow>;

	const FName ColName("Name"), ColCategory("Category"), ColDensity("Density"), ColModulus("Modulus"),
		ColPoisson("Poisson"), ColAllowable("Allowable"), ColSpecE("SpecE"), ColSpecRootE("SpecRootE"),
		ColSpecG("SpecG"), ColSpecRootG("SpecRootG"), ColSpecStrength("SpecStrength"), ColSource("Source");

	FString Str(const std::string& S) { return FString(UTF8_TO_TCHAR(S.c_str())); }
	std::string Std(const FString& S) { return std::string(TCHAR_TO_UTF8(*S)); }

	// A material's stiffness- and strength-to-weight ratios (Ashby's
	// material indices), in the project's units: E/rho ranks materials for
	// a tie of least mass at a given axial stiffness, sqrt(E)/rho for a beam
	// of given length and bending stiffness, G/rho and sqrt(G)/rho for a
	// shaft in torsion (section fixed, radius free), sigma/rho for a tie of
	// least mass at a given load. G is the isotropic E / (2 (1 + nu)). A
	// ratio's ranking is the same in any units.
	// The figures the table shows in the modulus, Poisson and allowable
	// columns: a material's own; a LAMINA's E1, nu12 and T1 (its fibre-
	// direction figures), since it has no isotropic ones; a laminate's
	// effective constants are already in its fields. The shear modulus for
	// the ratio columns: the isotropic E / (2 (1 + nu)), or a lamina's G12.
	struct FShown { double Modulus = 0.0, Poisson = 0.0, Allowable = 0.0, Shear = 0.0; };
	FShown ShownOf(const Material& M)
	{
		FShown S;
		if (M.IsLamina())
		{
			S.Modulus = M.lamina->e1;
			S.Poisson = M.lamina->nu12;
			S.Allowable = M.lamina->t1;
			S.Shear = M.lamina->g12;
			return S;
		}
		S.Modulus = M.youngs_modulus;
		S.Poisson = M.poisson_ratio;
		S.Allowable = M.allowable_stress;
		S.Shear = M.youngs_modulus / (2.0 * (1.0 + M.poisson_ratio));
		return S;
	}

	struct FIndices { double E = 0.0, RootE = 0.0, G = 0.0, RootG = 0.0, Strength = 0.0; };
	FIndices IndicesOf(const Material& M, const aerotelic::UnitSystem& U)
	{
		FIndices I;
		const double Rho = M.density / U.DensityToSI();
		if (Rho <= 0.0) return I;
		const FShown S = ShownOf(M);
		const double E = S.Modulus / U.StressToSI();
		const double G = S.Shear / U.StressToSI();
		I.E = E / Rho;
		I.RootE = FMath::Sqrt(FMath::Max(E, 0.0)) / Rho;
		I.G = G / Rho;
		I.RootG = FMath::Sqrt(FMath::Max(G, 0.0)) / Rho;
		I.Strength = (S.Allowable / U.StressToSI()) / Rho;
		return I;
	}

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

		// Custom tab: the user's materials and the one being typed. The
		// draft is one of three KINDS (2026-11-12): an isotropic material
		// (the four figures), a lamina (one ply's orthotropic figures) or a
		// laminate (a stack of a lamina, built from the stacking notation).
		enum class EKind { Isotropic, Lamina, Laminate };
		TArray<FRowPtr> CustomRows;
		TSharedPtr<SListView<FRowPtr>> CustomList;
		Material Draft;
		EKind Kind = EKind::Isotropic;
		FString Notation;                 // the laminate draft's stacking, as typed
		FString LaminaName;               // ...and the lamina its plies are cut from
		TArray<TSharedPtr<FString>> LaminaOptions;
		TSharedPtr<SEditableTextBox> NameBox, CategoryBox, NotationBox;
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
			Kind = EKind::Isotropic;
			Notation.Empty();
			PushDraftToBoxes();
		}

		void LoadDraft(const Material& M, bool bAsCopy)
		{
			Draft = M;
			if (bAsCopy)
			{
				Draft.name = M.name + " (custom)";
			}
			Kind = M.IsLamina() ? EKind::Lamina : (M.IsLaminate() ? EKind::Laminate : EKind::Isotropic);
			if (M.IsLaminate())
			{
				Notation = Str(aerotelic::StackingNotation(M));
				LaminaName = Str(M.plies[0].lamina);
			}
			PushDraftToBoxes();
		}

		// The draft's lamina figures, there to edit (made when the kind is
		// switched to Lamina).
		Lamina& LaminaDraft()
		{
			if (!Draft.lamina) Draft.lamina = Lamina{};
			return *Draft.lamina;
		}

		void SetKind(EKind K)
		{
			if (K == Kind) return;
			Kind = K;
			const bool bGenericCategory = Draft.category.empty() || Draft.category == "Custom" || Draft.category == "Lamina" || Draft.category == "Laminate";
			if (K == EKind::Lamina)
			{
				Draft.plies.clear();
				LaminaDraft();
				if (bGenericCategory) Draft.category = "Lamina";
			}
			else if (K == EKind::Laminate)
			{
				Draft.lamina.reset();
				if (bGenericCategory) Draft.category = "Laminate";
			}
			else
			{
				Draft.lamina.reset();
				Draft.plies.clear();
				if (bGenericCategory) Draft.category = "Custom";
			}
			PushDraftToBoxes();
		}

		// The laminae a laminate can be cut from: every lamina row of the
		// library (presets and the user's).
		void RebuildLaminaOptions()
		{
			LaminaOptions.Reset();
			for (const Material& M : AerotelicMaterialLibrary::All())
			{
				if (M.IsLamina()) LaminaOptions.Add(MakeShared<FString>(Str(M.name)));
			}
			if (LaminaName.IsEmpty() && LaminaOptions.Num() > 0) LaminaName = *LaminaOptions[0];
		}

		// Builds the laminate draft's plies from the notation and the chosen
		// lamina, keeping its name, category and source. False with a reason.
		bool BuildStack(FString& OutError)
		{
			const Material* L = AerotelicMaterialLibrary::Find(Std(LaminaName));
			if (L == nullptr || !L->IsLamina())
			{
				OutError = TEXT("pick the lamina the plies are cut from");
				return false;
			}
			try
			{
				Material Built = aerotelic::MakeLaminate(Draft.name, Std(Notation), *L, Draft.source);
				Built.category = Draft.category.empty() ? "Laminate" : Draft.category;
				Draft = Built;
				Kind = EKind::Laminate;
				return true;
			}
			catch (const std::exception& e)
			{
				OutError = UTF8_TO_TCHAR(e.what());
				return false;
			}
		}

		// What the laminate draft amounts to, for the Custom tab's summary.
		FText LaminateSummary(const aerotelic::UnitSystem& U) const
		{
			if (!Draft.IsLaminate()) return LOCTEXT("NoStack", "No stack yet: pick a lamina, type the stacking and Build.");
			const std::vector<aerotelic::Ply> Stack = aerotelic::ExpandedStack(Draft);
			const aerotelic::LaminateStiffness S = aerotelic::ComputeStiffness(Stack);
			const aerotelic::LaminateConstants C = aerotelic::EngineeringConstants(S);
			const double St = U.StressToSI(), Lt = U.LengthToSI();
			FString Text = FString::Printf(TEXT("%d plies %s of '%s': %.4g %s thick, density %.4g %s.\nIn-plane (curvatures free): Ex %.4g, Ey %.4g, Gxy %.4g %s, nu_xy %.3f; flexural Ex %.4g, Ey %.4g %s.\nFirst-ply-failure tension along x (Tsai-Wu): %s."),
				(int32)Stack.size(), *Str(aerotelic::StackingNotation(Draft)), *Str(Draft.plies[0].lamina), C.thickness / Lt, *Str(U.LengthName()), C.density / U.DensityToSI(), *Str(U.DensityName()),
				C.ex / St, C.ey / St, C.gxy / St, *Str(U.StressName()), C.nuxy, C.ex_flexural / St, C.ey_flexural / St, *Str(U.StressName()),
				Draft.allowable_stress > 0.0 ? *FString::Printf(TEXT("%.4g %s"), Draft.allowable_stress / St, *Str(U.StressName())) : TEXT("none (the lamina has no strengths, so no margin will be reported)"));
			if (C.symmetric && C.balanced) Text += TEXT("\nSymmetric and balanced: no extension-bending coupling (B = 0) and no shear-extension coupling (A16 = A26 = 0).");
			if (!C.symmetric) Text += TEXT("\nWARNING -- UNSYMMETRIC: B != 0, so in-plane loads bend and twist it and it warps on cure; the effective moduli above hold with the curvatures free.");
			if (!C.balanced) Text += TEXT("\nWARNING -- UNBALANCED: A16 or A26 != 0, so it shears under tension along the reference direction.");
			if (C.symmetric && C.balanced && !C.orthotropic) Text += TEXT("\nBend-twist coupled (D16, D26 != 0): the +/- plies sit at different depths, as in every [0/±45/90]s.");
			return FText::FromString(Text);
		}

		void PushDraftToBoxes()
		{
			if (NameBox.IsValid()) NameBox->SetText(FText::FromString(Str(Draft.name)));
			if (CategoryBox.IsValid()) CategoryBox->SetText(FText::FromString(Str(Draft.category)));
			if (SourceBox.IsValid()) SourceBox->SetText(FText::FromString(Str(Draft.source)));
			if (NotationBox.IsValid()) NotationBox->SetText(FText::FromString(Notation));
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
			if (SortColumn == ColModulus) return CompareNumber(ShownOf(A.M).Modulus, ShownOf(B.M).Modulus);
			if (SortColumn == ColPoisson) return CompareNumber(ShownOf(A.M).Poisson, ShownOf(B.M).Poisson);
			if (SortColumn == ColAllowable) return CompareNumber(ShownOf(A.M).Allowable, ShownOf(B.M).Allowable);
			if (SortColumn == ColSource) return CompareText(A.M.source, B.M.source);
			const FIndices IA = IndicesOf(A.M, Units), IB = IndicesOf(B.M, Units);
			if (SortColumn == ColSpecE) return CompareNumber(IA.E, IB.E);
			if (SortColumn == ColSpecRootE) return CompareNumber(IA.RootE, IB.RootE);
			if (SortColumn == ColSpecG) return CompareNumber(IA.G, IB.G);
			if (SortColumn == ColSpecRootG) return CompareNumber(IA.RootG, IB.RootG);
			if (SortColumn == ColSpecStrength) return CompareNumber(IA.Strength, IB.Strength);
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
			const FShown S = ShownOf(M);
			FText Text, Tip;
			bool bRight = false;
			if (Column == ColName)
			{
				Text = FText::FromString(Str(M.name));
				Tip = Text;
				if (M.IsLamina())
				{
					const Lamina& L = *M.lamina;
					Tip = FText::FromString(FString::Printf(TEXT("%s\nA LAMINA (one ply, %s): ply %.4g %s; E1 %.4g, E2 %.4g, G12 %.4g %s, nu12 %.3f; T1 %.4g, C1 %.4g, T2 %.4g, C2 %.4g, S12 %.4g %s%s"),
						*Str(M.name), *Str(L.form), L.ply_thickness / Units.LengthToSI(), *Str(Units.LengthName()), L.e1 / Units.StressToSI(), L.e2 / Units.StressToSI(), L.g12 / Units.StressToSI(), *Str(Units.StressName()), L.nu12,
						L.t1 / Units.StressToSI(), L.c1 / Units.StressToSI(), L.t2 / Units.StressToSI(), L.c2 / Units.StressToSI(), L.s12 / Units.StressToSI(), *Str(Units.StressName()),
						L.locking_angle > 0.0 ? *FString::Printf(TEXT("; locking angle %.0f deg"), L.locking_angle) : TEXT("")));
				}
				else if (M.IsLaminate())
				{
					const std::vector<aerotelic::Ply> Stack = aerotelic::ExpandedStack(M);
					Tip = FText::FromString(FString::Printf(TEXT("%s\nA LAMINATE: %d plies %s of '%s', %.4g %s thick; the figures are the stack's effective constants along the reference direction (classical lamination theory) and the allowable its first-ply-failure tension (Tsai-Wu)"),
						*Str(M.name), (int32)Stack.size(), *Str(aerotelic::StackingNotation(M)), *Str(M.plies[0].lamina), aerotelic::StackThickness(Stack) / Units.LengthToSI(), *Str(Units.LengthName())));
				}
			}
			else if (Column == ColCategory) Text = FText::FromString(Str(M.category));
			else if (Column == ColDensity) { Text = FormatG(M.density / Units.DensityToSI(), 4); bRight = true; }
			else if (Column == ColModulus) { Text = FormatG(S.Modulus / Units.StressToSI(), 4); bRight = true; if (M.IsLamina()) Tip = LOCTEXT("LaminaE1", "A lamina: E1, along the fibre (E2 and G12 in the name's tooltip)"); }
			else if (Column == ColPoisson) { Text = FormatG(S.Poisson, 3); bRight = true; if (M.IsLamina()) Tip = LOCTEXT("LaminaNu12", "A lamina: nu12, the major Poisson's ratio"); }
			else if (Column == ColAllowable) { Text = FormatG(S.Allowable / Units.StressToSI(), 4); bRight = true; if (M.IsLamina()) Tip = LOCTEXT("LaminaT1", "A lamina: T1, the tensile strength along the fibre"); }
			else if (Column == ColSpecE) { Text = FormatG(IndicesOf(M, Units).E, 4); bRight = true; }
			else if (Column == ColSpecRootE) { Text = FormatG(IndicesOf(M, Units).RootE, 4); bRight = true; }
			else if (Column == ColSpecG) { Text = FormatG(IndicesOf(M, Units).G, 4); bRight = true; }
			else if (Column == ColSpecRootG) { Text = FormatG(IndicesOf(M, Units).RootG, 4); bRight = true; }
			else if (Column == ColSpecStrength) { Text = FormatG(IndicesOf(M, Units).Strength, 4); bRight = true; }
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
		"A material is what a solid body is made of: its density, Young's modulus, Poisson's ratio and the allowable stress a margin is reported against -- or, since 2026-11-12, a composite LAMINATE, a stack of plies whose figures are computed from the plies (see COMPOSITE LAMINAE AND LAMINATES below). "
		"Analysis > Mass Properties weighs a body by its density (over the volume it encloses, or over its skin), and Analysis > Structural (Kratos FEA) analyses it with the other three. "
		"A material is a property of the stuff, not of the project's units: every figure is kept in SI (kg/m^3, Pa) and shown here in the project's units (Model > Units), so changing the units changes the numbers on screen and nothing underneath.\n"
		"\n"
		"THE MATERIALS TAB\n"
		"\n"
		"One row per material: the presets the application ships with, then your own (in bold). Click a column header to sort the table by that column, ascending; click it again for descending. "
		"The Source column says where a preset's figures came from and what convention they follow; hover a row to read the whole of it. "
		"Double-click a row to take it to the Custom tab as the starting point for a material of your own.\n"
		"\n"
		"The five ratio columns are the stiffness- and strength-to-weight indices that rank materials for a part of least mass (Ashby's material indices), computed from the four figures in the project's units -- so their numbers change with the units but their order never does. "
		"E/ρ ranks materials for a tie, an axial member of given stiffness; √E/ρ for a beam of given length and bending stiffness, where a lighter, thicker section wins (a plate's index would be the cube root); "
		"G/ρ and √G/ρ do the same for a shaft in torsion, with the section fixed and with the radius free, the shear modulus taken as the isotropic G = E / (2 (1 + ν)); σ/ρ is the specific strength, the allowable stress over density, for a tie of given load. "
		"Sort by one to see why spruce and balsa beat aluminium for a beam while aluminium beats them for a tie.\n"
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
		"THE FILES\n"
		"\n"
		"Both libraries are plain YAML a person can read and edit. The presets are core/data/materials_presets.yaml in the source tree, built into the application (edit it and rebuild); your own are the file named at the bottom of the Custom tab, written the moment you add, update or remove a material here, and read when the application starts -- so you may also edit it by hand while the application is closed. "
		"The two share one schema: a 'materials' list of name, category, density (kg/m^3), youngs_modulus (Pa), poisson_ratio, allowable_stress (Pa; 0 reports no margin) and source. "
		"The application ships only materials whose sources permit public and commercial redistribution. Materials whose sources' terms restrict it -- the ASM Handbook's typical figures for the first seven metals and every manufacturer's data sheet (the resins, the filaments, the PMI foam, AK Steel's 17-4PH) -- were split out on 2026-11-07 into core/data/materials_restricted.yaml, which is not built in, and into the library file of the user who held them, where they appear in bold like any material of theirs.\n"
		"\n"
		"ASSIGNING A MATERIAL\n"
		"\n"
		"Right-click a body in the Scene tree and choose Structure: pick the material from the list, give the skin thickness (0 for a solid body; a positive thickness makes the body's surface a skin of the material that thick, which is also what the structural analysis gives its shells), and read off the enclosed volume, the surface area and what it would weigh. "
		"A boolean result's bodies own their structure; Copy Structure From Operand starts them from an operand's.\n"
		"\n"
		"CONVENTIONS OF THE PRESETS\n"
		"\n"
		"Metals: MIL-HDBK-5J design values, room temperature, for at least ten aluminium, steel, magnesium and titanium alloys each. The allowable is the handbook's MINIMUM tensile yield -- A-basis (99% of material exceeds it, 95% confidence) where the handbook gives one, else S-basis (the procurement specification's minimum) -- longitudinal, for the product form, condition and thickness range the Source column names: a design minimum rather than a typical figure, 5-15% below the typical yield of the same alloy. "
		"Magnesium casting rows are the handbook's 'any area' minimums, which are well below separately cast test-bar figures. Where the handbook gives no Poisson's ratio (some titanium alloys) a literature value is used and the Source column says so. "
		"(The seven metal rows the application used to ship -- 2024-T3, 7075-T6, 6061-T6, Ti-6Al-4V, 4130, 17-4PH H900, AZ31B -- were the ASM Handbook's TYPICAL values with the typical yield as allowable; they are in the restricted file and in the library of a user who held them, and have handbook rows of the same alloys here.)\n"
		"Laminates (the two isotropic 'quasi-isotropic' rows): representative in-plane figures for a carbon/epoxy and an E-glass/epoxy laminate, not any one supplier's; the allowable is a representative laminate strength.\n"
		"\n"
		"COMPOSITE LAMINAE AND LAMINATES (2026-11-12)\n"
		"\n"
		"A LAMINA (category Lamina) is ONE PLY's material -- a unidirectional tape (UD) or a woven cloth -- in the ply's own axes: 1 along the fibre (a cloth's warp), 2 across it in the plane. Its row carries the density, the cured ply thickness, E1, E2, nu12, G12 (and G13, G23 for thick shells; the thin shell used here does not need them), "
		"the strengths T1 and C1 (tension and compression along the fibre), T2 and C2 (across) and S12 (in-plane shear), and a cloth's shear LOCKING ANGLE. A lamina is never assigned to a body; the table shows its E1, nu12 and T1 in the modulus, Poisson and allowable columns.\n"
		"A LAMINATE (category Laminate) is a stack of plies of a lamina, written in the usual stacking notation: [0/±45/90]s is 0, +45, -45, 90 and the same again in reverse (s mirrors the listed half about the midplane; ~ on the last angle makes it the single centre ply); [0/90]2s repeats the group; 0_2 repeats a ply; +- may be typed for ±. Plies are listed from the OUTER face -- the one the body's outward normal leaves, where a laminator lays the first ply on a female tool -- inward. "
		"Its density, Young's modulus, Poisson's ratio and allowable are COMPUTED from the plies by classical lamination theory and never typed: the density is the thickness-weighted mean; the modulus and Poisson's ratio are the stack's effective in-plane constants along the REFERENCE DIRECTION with the curvatures free (the inverse of [A B; B D] -- for a symmetric stack simply A^-1); the allowable is the FIRST-PLY-FAILURE tensile strength along that direction by the Tsai-Wu criterion (0 when the lamina has no strengths). "
		"These effective figures let a laminate be weighed, sorted and ranked like any material; the structural analysis itself is given the plies.\n"
		"SYMMETRIC OR GENERAL. A symmetric stack has no extension-bending coupling (B = 0): in-plane loads stretch it and moments bend it, separately, and it comes off the tool flat. A general (unsymmetric) stack couples the two -- tension bends and twists it, and it warps as it cures -- so its effective modulus is a figure with the curvatures free, not a material constant; the Custom tab's summary warns of it, and of an UNBALANCED stack (A16, A26 != 0: it shears under tension). Both are analysed alike: Kratos's layered shell integrates the full A, B, D.\n"
		"THE REFERENCE DIRECTION AND THE DRAPE. A 0 degree ply runs along the body's loft -- the span of a wing, the length of a fuselage, the along-the-loft direction in every triangle (a laminator's rosette); on a planar rib or spar, along its longest extent. A positive angle turns counter-clockwise about the outward normal. "
		"Before the analysis each ply is DRAPED over the body as a pin-jointed net (Mack & Taylor 1956; Van Der Weeen 1991): where the surface is developable (a cylinder, a nearly flat wing skin) the cloth lies as cut, where it has double curvature (a nose, a tip, a fillet) warp and weft shear closer than 90 degrees and the fibres turn from their nominal direction. "
		"The analysis then turns each ply by its deviation and thickens a WOVEN ply by 1 / cos(shear) (areal weight is conserved as a cloth shears); a UD tape cannot shear in its plane and keeps its thickness -- on double curvature it is laid in strips, and the net's shear says how much gap or overlap they would need. Faces where a cloth is sheared past its locking angle are counted and reported: they will wrinkle, not drape. "
		"Not modelled: the change of a sheared cloth's stiffness with the angle between its fibres (Hofstee & van Keulen 2001), which grows past about 20 degrees of shear.\n"
		"MARGINS. A laminate region's margin is Tsai-Wu's: the reserve factor by which the stresses of every ply, at every point, could be scaled before the criterion reaches one (Tsai & Wu 1971, plane stress, F12 = -1/2 sqrt(F11 F22)); the margin is that factor less one. It needs all five strengths on the lamina; without them the analysis reports no margin and says so. Von Mises is still reported for a laminate as a stress magnitude, not as a criterion.\n"
		"THE FILE. A lamina row has a 'lamina' map (form, ply_thickness, e1, e2, nu12, g12, g13, g23, t1, c1, t2, c2, s12, s13, s23, locking_angle; SI: Pa, m); a laminate row has 'plies' (a list of {angle, thickness (optional), lamina (by name)}), 'symmetric' and 'symmetric_odd', its plies resolved against the laminae of the file and of the presets when read. A project file writes a laminate's plies with the lamina's figures, so it stands alone. To mix laminae in one stack or give a ply its own thickness, edit the file.\n"
		"Woods: the USDA Wood Handbook's clear-wood figures along the grain at 12% moisture, density from its specific gravity, Poisson's ratio nu_LR from its Table 5-1; the allowable is the compression strength parallel to the grain (the lower of the two, what a spar cap buckles at). Balsa is sold by weight, so three grades are given -- contest-light, medium, hard -- with the Handbook's figures scaled in proportion to density (its stiffness and strength go roughly as its density: Borrega & Gibson 2015). The two plywoods (aircraft birch, poplar lite-ply) take the solid wood at two-thirds along the face grain of a three-ply sheet, ANC-18's approach, and an approximate in-plane Poisson's ratio of 0.2. Weigh a sheet: hobby stock varies more than any handbook.\n"
		"\n"
		"CONVENTIONS OF THE DATA-SHEET MATERIALS (not shipped; in your library if you hold them)\n"
		"\n"
		"Foam: the data-sheet figure for the nominal grade; the allowable sits between the compressive and tensile strengths.\n"
		"SLA / LCD / DLP resins: the manufacturers' technical data sheets, POST-CURED figures (ASTM D638 tensile modulus and strength); the allowable is the tensile yield where the sheet gives one, else the strength at break; densities from the sheet or Formlabs' post-cured density table. Poisson's ratio is not on the sheets: 0.35 is the literature figure for cured acrylates (0.30 glass-filled, 0.38 for the soft polypropylene-like and ductile resins, 0.42 for the near-elastomeric one). "
		"Anycubic's sheets give ranges, no post-cure and no tensile modulus (the store pages add a Young's modulus range): the LOWER bound of the modulus and of the tensile strength and the UPPER bound of the density were taken, a conservative card.\n"
		"FDM thermoplastics: the filament makers' technical data sheets, AS PRINTED, flat (XY / XZ) orientation, dry; the allowable is the tensile yield where given, else the tensile strength. A printed part is weaker across its layers (Z strength is typically 60-85% of XY on these sheets) and its stiffness depends on walls and infill: these are the figures for a solid part loaded along the layers, an upper bound for an isotropic analysis. Poisson's ratios are literature values for printed parts.\n"
		"\n"
		"SOURCES\n"
		"\n"
		"Metals: MIL-HDBK-5J, Metallic Materials and Elements for Aerospace Vehicle Structures, US Department of Defense, 31 January 2003 (Distribution Statement A: approved for public release, distribution unlimited; a US Government work in the public domain) -- the 'Design Mechanical and Physical Properties' table of each alloy, cited by table number in the Source column: Chapter 2 steels, Chapter 3 aluminium, Chapter 4 magnesium, Chapter 5 titanium, Chapter 6 A-286. MMPDS-01 (DOT/FAA/AR-MMPDS-01, 2003) is the FAA's technically equivalent re-issue of it; later MMPDS editions are copyrighted by Battelle and are not used.\n"
		"Wood: Forest Products Laboratory, Wood Handbook -- Wood as an Engineering Material, General Technical Report FPL-GTR-190, USDA, 2010: Table 5-3 (Sitka spruce, American basswood, Eastern white pine, yellow-poplar, Coast Douglas-fir, yellow birch), Table 5-5 (balsa), Table 5-1 (Poisson's ratios), all at 12% moisture; "
		"M. Borrega and L. J. Gibson, 'Mechanics of balsa (Ochroma pyramidale) wood', Mechanics of Materials 84 (2015) 75-90, for the density scaling of the balsa grades; "
		"ANC-18, Design of Wood Aircraft Structures, ANC Bulletin, US Army-Navy-Civil Committee on Aircraft Design Criteria, 1951, for the plywood approach.\n"
		"Composite laminae: MIL-HDBK-17-2F, Composite Materials Handbook, Volume 2, Polymer Matrix Composites: Materials Properties, US Department of Defense, 17 June 2002 (Distribution Statement A: approved for public release, distribution unlimited; no copyright notice) -- section 4.2.8 AS4 12k/3502 tape (B-basis strengths), 4.2.6 T-300 3k/934 plain weave, 6.2.4 E-glass 7781/EA 9396 8-harness satin, 6.2.1 S2-449/SP381 tape; "
		"each row's Source names its tables, the fibre volume the figures are normalised to, the basis, and which figures (a cloth's nu12, G12, S12; a tape's nu12) are literature values the handbook lacks. The handbook's successor CMH-17 is copyrighted and is not used. "
		"Theory: R. M. Jones, Mechanics of Composite Materials, 2nd ed., Taylor & Francis, 1999, ch. 4 (classical lamination theory); A. T. Nettles, Basic Mechanics of Laminated Composite Plates, NASA RP-1351, 1994 (whose worked examples the code's checks reproduce); S. W. Tsai and E. M. Wu, A General Theory of Strength for Anisotropic Materials, J. Composite Materials 5 (1971) 58-80. "
		"Drape: C. Mack and H. M. Taylor, The fitting of woven cloth to surfaces, J. Textile Institute 47 (1956) T477; F. Van Der Weeen, Algorithms for draping fabrics on doubly-curved surfaces, Int. J. Numer. Meth. Eng. 31 (1991) 1415; A. P. Hancock and K. D. Potter, Composites Part A 37 (2006) 413; P. Boisse et al., Composites Part A 42 (2011) 1156 (locking angles); H. J. Hofstee and F. van Keulen, Composite Structures 54 (2001) 179.\n"
		"\n"
		"Sources of the data-sheet materials, whose terms of use restrict redistribution (so they are not shipped): ASM Handbook vol. 1 (Irons, Steels and High-Performance Alloys) and vol. 2 (Nonferrous Alloys), ASM International, and the AK Steel 17-4 PH product data bulletin, for the seven former metal rows. "
		"Foam: Evonik Industries, ROHACELL IG / IG-F technical data sheet (grade 71).\n"
		"Resins: Formlabs technical data sheets -- Grey Resin V4, Clear Resin V4.1, Tough 2000 Resin V2, Durable Resin, High Temp Resin V2, Rigid 10K Resin -- and Formlabs support, 'Density of selected Formlabs SLA resins' (post-cured); Siraya Tech, Blu resin technical data sheet; "
		"Anycubic technical data sheets (store.anycubic.com, resin user manual index) and store pages for Standard, Standard V2, 16K Standard, ABS-Like 2.0, ABS-Like V2, ABS-Like 3.0, ABS-Like Pro 2, Water-Wash ABS-Like 3.0, Water-Wash 2.0, Tough 2.0, Tough Ultra, High Clear, High Speed 2.0, Plant-Based 2.0, DLP Craftsman, 14K Texture and Bio resins.\n"
		"FDM: Prusa Research, Prusament PLA, PETG, ASA and PC Blend technical data sheets; Bambu Lab, ABS and PAHT-CF technical data sheets; Polymaker, PolyMide CoPA technical data sheet; Markforged, composites material data sheet (Onyx); Stratasys, ULTEM 9085 resin data sheet (XZ orientation); 3DXTech, ThermaX PEEK technical data sheet.\n"
		"Poisson's ratios of printed polymers: literature values for cured acrylate resins and printed PLA, ABS, PETG, nylon, PC, PEI and PEEK; 0.35 for fibre-filled filaments.\n"
		"\n"
		"The metal presets are handbook design minimums for the product form named, the woods clear-wood averages, the data-sheet materials their suppliers' typical values: none is a design allowable for any particular part. A printed or moulded part's properties depend on its process, orientation, cure and environment. Check them against your own coupons before trusting a margin.");
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
		.ClientSize(FVector2D(1640, 680))
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

	// The ratio columns' units: stress over density, and root-stress over
	// density, in the project's names.
	const std::string RatioUnit = std::string(Units.StressName()) + "/(" + Units.DensityName() + ")";
	const std::string RootRatioUnit = std::string(Units.StressName()) + "^0.5/(" + Units.DensityName() + ")";

	TSharedRef<SWidget> MaterialsTab =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
		[
			MakeInfoLabel(FText::Format(LOCTEXT("TableHint",
				"Every material a body can be made of: the presets, then your own in bold (Custom tab). Click a column header to sort by it, again to reverse. Figures are in the project's units -- density in {0}, stresses in {1} (Model > Units) -- kept in SI underneath. "
				"A Lamina row is one ply's figures (E1, nu12 and T1 shown; hover its name for the rest) and is never assigned to a body; a Laminate row is a stack of plies whose figures are computed from them (hover its name). "
				"The five ratio columns rank stiffness and strength against weight: E/ρ for a tie, √E/ρ for a beam, G/ρ and √G/ρ for a shaft, σ/ρ for strength (G from E and ν); their order is the same in any units. Hover a row for its full source; double-click one to start a material of your own from it."),
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
					+ Column(ColSpecE, WithUnit(LOCTEXT("ColSpecE", "E/ρ"), RatioUnit), LOCTEXT("ColSpecETip", "Specific stiffness: Young's modulus over density, in the project's units. The index of a tie (axial member) of least mass at a given stiffness; the ranking is the same in any units")).ManualWidth(100)
					+ Column(ColSpecRootE, WithUnit(LOCTEXT("ColSpecRootE", "√E/ρ"), RootRatioUnit), LOCTEXT("ColSpecRootETip", "Square root of Young's modulus over density: the index of a beam of given length and bending stiffness and least mass (a plate's would be the cube root)")).ManualWidth(100)
					+ Column(ColSpecG, WithUnit(LOCTEXT("ColSpecG", "G/ρ"), RatioUnit), LOCTEXT("ColSpecGTip", "Shear modulus over density, G = E / (2 (1 + ν)) for an isotropic material: the index of a shaft of given section and least mass at a given torsional stiffness")).ManualWidth(100)
					+ Column(ColSpecRootG, WithUnit(LOCTEXT("ColSpecRootG", "√G/ρ"), RootRatioUnit), LOCTEXT("ColSpecRootGTip", "Square root of the shear modulus over density: the index of a shaft of given length and torsional stiffness, radius free, and least mass")).ManualWidth(100)
					+ Column(ColSpecStrength, WithUnit(LOCTEXT("ColSpecStrength", "σ/ρ"), RatioUnit), LOCTEXT("ColSpecStrengthTip", "Specific strength: the allowable stress over density, the index of a tie of least mass at a given load")).ManualWidth(100)
					+ Column(ColSource, LOCTEXT("ColSource", "Source"), LOCTEXT("ColSourceTip", "Where the figures came from and the convention they follow; hover a row for the whole of it")).FillWidth(2.6f)
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
	// A lamina figure, the same way.
	auto LField = [State](double Lamina::* Member, double ToSI, double Delta, double Max)
	{
		return SNew(SAerotelicNumericEntry)
			.Value_Lambda([State, Member, ToSI]() { return (State->LaminaDraft().*Member) / ToSI; })
			.MinValue(0.0).MaxValue(Max).Delta(Delta)
			.LiveUpdate(true)
			.OnValueCommitted_Lambda([State, Member, ToSI](double V, ETextCommit::Type) { State->LaminaDraft().*Member = V * ToSI; });
	};
	// A part of the form shown for one kind of draft only.
	auto OnlyFor = [State](FState::EKind K, TSharedRef<SWidget> W)
	{
		return SNew(SBox).Visibility_Lambda([State, K]() { return State->Kind == K ? EVisibility::Visible : EVisibility::Collapsed; })[W];
	};
	const double StressDelta = 1.0e9 / Units.StressToSI(), StrengthDelta = 1.0e6 / Units.StressToSI();
	const double PlyDelta = 1.0e-5 / Units.LengthToSI();

	TArray<TSharedPtr<FString>> StartOptions;
	for (const Material& M : AerotelicMaterialLibrary::All()) StartOptions.Add(MakeShared<FString>(Str(M.name)));
	TArray<TSharedPtr<FString>> KindOptions = { MakeShared<FString>(TEXT("Isotropic material")), MakeShared<FString>(TEXT("Lamina (one ply's orthotropic figures)")), MakeShared<FString>(TEXT("Laminate (a stack of plies)")) };
	TArray<TSharedPtr<FString>> FormOptions = { MakeShared<FString>(TEXT("UD")), MakeShared<FString>(TEXT("Woven")) };
	State->RebuildLaminaOptions();
	auto KindLabel = [State, KindOptions]() { return FText::FromString(*KindOptions[State->Kind == FState::EKind::Isotropic ? 0 : (State->Kind == FState::EKind::Lamina ? 1 : 2)]); };

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
		if (State->Kind == FState::EKind::Lamina)
		{
			// A lamina carries no isotropic figures.
			M.plies.clear();
			if (!M.lamina) M.lamina = Lamina{};
			M.youngs_modulus = 0.0;
			M.poisson_ratio = 0.0;
			M.allowable_stress = 0.0;
		}
		else if (State->Kind == FState::EKind::Laminate)
		{
			M.lamina.reset();
			aerotelic::RefreshLaminate(M);   // the effective figures from the plies
		}
		else
		{
			M.lamina.reset();
			M.plies.clear();
		}
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
				[
					FormRow(LOCTEXT("Kind", "Kind:"),
						SNew(SComboBox<TSharedPtr<FString>>)
						.OptionsSource(&KindOptions)
						.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
						.OnSelectionChanged_Lambda([State, KindOptions](TSharedPtr<FString> S, ESelectInfo::Type Info)
						{
							if (!S.IsValid() || Info == ESelectInfo::Direct) return;
							const int32 I = KindOptions.IndexOfByKey(S);
							State->SetKind(I == 1 ? FState::EKind::Lamina : (I == 2 ? FState::EKind::Laminate : FState::EKind::Isotropic));
							if (I == 2) State->RebuildLaminaOptions();
						})
						[ SNew(STextBlock).Text_Lambda(KindLabel) ])
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0)
				[ FormRow(WithUnit(LOCTEXT("Density", "Density"), Units.DensityName()), SNew(SBox).IsEnabled_Lambda([State]() { return State->Kind != FState::EKind::Laminate; }) [ Field(&Material::density, Units.DensityToSI(), 10.0, 1.0e9) ]) ]
				// ---- isotropic ----
				+ SVerticalBox::Slot().AutoHeight()
				[
					OnlyFor(FState::EKind::Isotropic,
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight()
						[ FormRow(WithUnit(LOCTEXT("Young", "Young's modulus"), Units.StressName()), Field(&Material::youngs_modulus, Units.StressToSI(), StressDelta, 1.0e15)) ]
						+ SVerticalBox::Slot().AutoHeight()
						[ FormRow(LOCTEXT("Poisson", "Poisson's ratio:"), Field(&Material::poisson_ratio, 1.0, 0.01, 0.4999)) ]
						+ SVerticalBox::Slot().AutoHeight()
						[ FormRow(WithUnit(LOCTEXT("Allowable", "Allowable stress"), Units.StressName()), Field(&Material::allowable_stress, Units.StressToSI(), StrengthDelta, 1.0e15)) ])
				]
				// ---- lamina ----
				+ SVerticalBox::Slot().AutoHeight()
				[
					OnlyFor(FState::EKind::Lamina,
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight()
						[
							FormRow(LOCTEXT("Form", "Form:"),
								SNew(SComboBox<TSharedPtr<FString>>)
								.OptionsSource(&FormOptions)
								.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
								.OnSelectionChanged_Lambda([State](TSharedPtr<FString> S, ESelectInfo::Type Info) { if (S.IsValid() && Info != ESelectInfo::Direct) State->LaminaDraft().form = Std(*S); })
								[ SNew(STextBlock).Text_Lambda([State]() { return FText::FromString(Str(State->LaminaDraft().form)); }) ])
						]
						+ SVerticalBox::Slot().AutoHeight()
						[ FormRow(WithUnit(LOCTEXT("PlyThickness", "Cured ply thickness"), Units.LengthName()), LField(&Lamina::ply_thickness, Units.LengthToSI(), PlyDelta, 1.0e9)) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
						[ FormRow(WithUnit(LOCTEXT("E1", "E1, along the fibre"), Units.StressName()), LField(&Lamina::e1, Units.StressToSI(), StressDelta, 1.0e15)) ]
						+ SVerticalBox::Slot().AutoHeight()
						[ FormRow(WithUnit(LOCTEXT("E2", "E2, across"), Units.StressName()), LField(&Lamina::e2, Units.StressToSI(), StressDelta, 1.0e15)) ]
						+ SVerticalBox::Slot().AutoHeight()
						[ FormRow(LOCTEXT("Nu12", "nu12, major Poisson's ratio:"), LField(&Lamina::nu12, 1.0, 0.01, 2.0)) ]
						+ SVerticalBox::Slot().AutoHeight()
						[ FormRow(WithUnit(LOCTEXT("G12", "G12, in-plane shear"), Units.StressName()), LField(&Lamina::g12, Units.StressToSI(), StressDelta, 1.0e15)) ]
						+ SVerticalBox::Slot().AutoHeight()
						[ FormRow(WithUnit(LOCTEXT("G13", "G13 (0 = G12)"), Units.StressName()), LField(&Lamina::g13, Units.StressToSI(), StressDelta, 1.0e15)) ]
						+ SVerticalBox::Slot().AutoHeight()
						[ FormRow(WithUnit(LOCTEXT("G23", "G23 (0 = G12 / 2)"), Units.StressName()), LField(&Lamina::g23, Units.StressToSI(), StressDelta, 1.0e15)) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
						[ FormRow(WithUnit(LOCTEXT("T1", "T1, tension along the fibre"), Units.StressName()), LField(&Lamina::t1, Units.StressToSI(), StrengthDelta, 1.0e15)) ]
						+ SVerticalBox::Slot().AutoHeight()
						[ FormRow(WithUnit(LOCTEXT("C1", "C1, compression along"), Units.StressName()), LField(&Lamina::c1, Units.StressToSI(), StrengthDelta, 1.0e15)) ]
						+ SVerticalBox::Slot().AutoHeight()
						[ FormRow(WithUnit(LOCTEXT("T2", "T2, tension across"), Units.StressName()), LField(&Lamina::t2, Units.StressToSI(), StrengthDelta, 1.0e15)) ]
						+ SVerticalBox::Slot().AutoHeight()
						[ FormRow(WithUnit(LOCTEXT("C2", "C2, compression across"), Units.StressName()), LField(&Lamina::c2, Units.StressToSI(), StrengthDelta, 1.0e15)) ]
						+ SVerticalBox::Slot().AutoHeight()
						[ FormRow(WithUnit(LOCTEXT("S12", "S12, in-plane shear strength"), Units.StressName()), LField(&Lamina::s12, Units.StressToSI(), StrengthDelta, 1.0e15)) ]
						+ SVerticalBox::Slot().AutoHeight()
						[ FormRow(WithUnit(LOCTEXT("S13", "S13 (0 = S12)"), Units.StressName()), LField(&Lamina::s13, Units.StressToSI(), StrengthDelta, 1.0e15)) ]
						+ SVerticalBox::Slot().AutoHeight()
						[ FormRow(WithUnit(LOCTEXT("S23", "S23 (0 = S12)"), Units.StressName()), LField(&Lamina::s23, Units.StressToSI(), StrengthDelta, 1.0e15)) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
						[ FormRow(LOCTEXT("Locking", "Locking angle, deg (cloth; 0 = none):"), LField(&Lamina::locking_angle, 1.0, 1.0, 89.0)) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
						[
							MakeInfoLabel(LOCTEXT("LaminaHint", "One ply's figures in the ply's own axes: 1 along the fibre (a cloth's warp), 2 across it in the plane. Give all five of T1, C1, T2, C2 and S12 for Tsai-Wu margins, or none. "
								"A lamina is not assigned to a body: build a laminate of it (Kind: Laminate). The transverse shear moduli and strengths may be left 0 (the thin shell does not use them)."))
						])
				]
				// ---- laminate ----
				+ SVerticalBox::Slot().AutoHeight()
				[
					OnlyFor(FState::EKind::Laminate,
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight()
						[
							FormRow(LOCTEXT("LaminaOf", "Plies cut from:"),
								SNew(SComboBox<TSharedPtr<FString>>)
								.OptionsSource(&State->LaminaOptions)
								.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
								.OnComboBoxOpening_Lambda([State]() { State->RebuildLaminaOptions(); })
								.OnSelectionChanged_Lambda([State](TSharedPtr<FString> S, ESelectInfo::Type Info) { if (S.IsValid() && Info != ESelectInfo::Direct) State->LaminaName = *S; })
								[ SNew(STextBlock).Text_Lambda([State]() { return State->LaminaName.IsEmpty() ? LOCTEXT("PickLamina", "pick a lamina...") : FText::FromString(State->LaminaName); }) ])
						]
						+ SVerticalBox::Slot().AutoHeight()
						[
							FormRow(LOCTEXT("Stacking", "Stacking:"),
								SNew(SHorizontalBox)
								+ SHorizontalBox::Slot().FillWidth(1.0f)
								[
									SAssignNew(State->NotationBox, SEditableTextBox)
									.Text(FText::FromString(State->Notation))
									.HintText(LOCTEXT("StackingHint", "[0/±45/90]s   [0/90]2s   [45/-45/0_2/90~]s   (outer face first; +- for ±; _n repeats; ]s mirrors; ~ marks the single centre ply)"))
									.OnTextChanged_Lambda([State](const FText& T) { State->Notation = T.ToString(); })
								]
								+ SHorizontalBox::Slot().AutoWidth().Padding(6, 0, 0, 0)
								[
									Button(LOCTEXT("Build", "Build stack"), LOCTEXT("BuildTip", "Make the plies from the stacking notation and the lamina, each at the lamina's ply thickness; the figures below follow"),
										[State]() { FString Err; if (State->BuildStack(Err)) State->Say(LOCTEXT("Built", "Stack built; the figures below are the laminate's. Add as new or Update selected to keep it.")); else State->Say(FText::Format(LOCTEXT("NotBuilt", "No stack: {0}"), FText::FromString(Err))); })
								])
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
						[
							SNew(STextBlock).AutoWrapText(true)
							.Text_Lambda([State]() { return State->LaminateSummary(State->Units); })
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
						[
							MakeInfoLabel(LOCTEXT("LaminateHint", "A laminate's density, modulus, Poisson's ratio and allowable are computed from its plies by classical lamination theory (Jones 1999 ch. 4) and are not typed. Plies are listed from the OUTER face inward; a positive angle turns counter-clockwise about the outward normal from the reference direction, "
								"which on a body is the loft's own direction (a wing's span, a fuselage's length). The analysis drapes each ply over the body and reports Tsai-Wu margins. To change a ply's thickness or mix laminae, edit the stack in the library file (plies: [{angle, thickness, lamina}])."))
						])
				]
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
