#include "MeshTab.h"

#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Text/STextBlock.h"

#include "Widgets/NumericEntry.h"
#include "Widgets/CurveTabControls.h"

#include "aerotelic/solid_model.hpp"
#include "aerotelic/primitive_model.hpp"

#define LOCTEXT_NAMESPACE "MeshTab"

using aerotelic::ISolidModel;
using aerotelic::MeshSpec;
using aerotelic::PrimitiveModel;
using aerotelic::PrimitiveShape;
using aerotelic::project_yaml::EntityType;

namespace
{
	// The state the rows read and write: the model's own MeshSpec, the
	// change delegate, and the triangle-count read-out.
	struct FMeshTabState : public TSharedFromThis<FMeshTabState>
	{
		ISolidModel* Model = nullptr;
		EntityType Type = EntityType::Primitive;
		FSimpleDelegate OnChanged;
		TSharedPtr<STextBlock> CountLabel;

		MeshSpec& Spec() { return Model->mesh; }

		void Changed()
		{
			RefreshCount();
			OnChanged.ExecuteIfBound();
		}

		// What the shape is, for a primitive's rows (the Shape tab may change
		// it under us, so this is read live).
		PrimitiveShape Shape() const
		{
			return Type == EntityType::Primitive ? static_cast<const PrimitiveModel*>(Model)->shape : PrimitiveShape::Sphere;
		}
		bool IsLofted() const { return Type != EntityType::Primitive || Shape() == PrimitiveShape::Sphere; }
		bool IsBox() const { return Type == EntityType::Primitive && Shape() == PrimitiveShape::Box; }
		bool IsCylinder() const { return Type == EntityType::Primitive && Shape() == PrimitiveShape::Cylinder; }
		bool IsWingLike() const { return Type == EntityType::Wing || Type == EntityType::Propeller; }

		void RefreshCount()
		{
			if (!CountLabel.IsValid()) return;
			try
			{
				// The viewport's tolerance rule (AAerotelicEntityActor::DisplayTolerance):
				// 0.15% of the body's diagonal, from a first coarse build.
				double Tolerance = 2e-3;
				std::vector<aerotelic::NamedMesh> Meshes = Model->BuildMeshes(Tolerance);
				aerotelic::Box3 Bounds;
				for (const aerotelic::NamedMesh& M : Meshes) for (const aerotelic::Vec3& V : M.mesh.vertices) Bounds.Add(V);
				if (!Meshes.empty())
				{
					Tolerance = FMath::Clamp(Bounds.Diagonal() * 0.0015, 1e-4, 1e9);
					Meshes = Model->BuildMeshes(Tolerance);
				}
				int64 Triangles = 0, Vertices = 0;
				for (const aerotelic::NamedMesh& M : Meshes)
				{
					Triangles += static_cast<int64>(M.mesh.triangles.size());
					Vertices += static_cast<int64>(M.mesh.vertices.size());
				}
				if (Meshes.empty())
				{
					CountLabel->SetText(LOCTEXT("NoMeshBuilder", "This model has no mesh builder of its own (it is tessellated from its solid); the settings above do not apply."));
				}
				else
				{
					CountLabel->SetText(FText::FromString(FString::Printf(TEXT("As drafted: %lld triangles on %lld vertices over %d bod%s (at the viewport's tolerance %.3g). The FEA puts a shell on every triangle; the flow solvers wet every one."),
						Triangles, Vertices, static_cast<int32>(Meshes.size()), Meshes.size() == 1 ? TEXT("y") : TEXT("ies"), Tolerance)));
				}
			}
			catch (const std::exception& E)
			{
				CountLabel->SetText(FText::FromString(FString::Printf(TEXT("The draft cannot be built as it stands: %s"), UTF8_TO_TCHAR(E.what()))));
			}
		}
	};

	TSharedRef<SWidget> Row(const FText& Label, TSharedRef<SWidget> Field, TAttribute<EVisibility> Vis, const FText& Tip, const FText& Note)
	{
		return SNew(SHorizontalBox)
			.Visibility(Vis)
			.ToolTipText(Tip)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 2, 8, 2)
			[ SNew(SBox).MinDesiredWidth(170) [ SNew(STextBlock).Text(Label) ] ]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 2)
			[ SNew(SBox).MinDesiredWidth(90) [ Field ] ]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(8, 2, 0, 2)
			[ SNew(STextBlock).Text(Note).ColorAndOpacity(FSlateColor::UseSubduedForeground()) ];
	}

	// An integer count: 0 shown and typed as automatic.
	TSharedRef<SWidget> CountRow(TSharedRef<FMeshTabState> S, const FText& Label, TFunction<int&()> Field, int32 Max, TAttribute<EVisibility> Vis, const FText& Tip, const FText& Note)
	{
		return Row(Label,
			SNew(SAerotelicNumericEntry)
			.Value_Lambda([Field]() { return static_cast<double>(Field()); })
			.MinValue(0.0).MaxValue(static_cast<double>(Max)).Delta(1.0).LiveUpdate(false)
			.OnValueCommitted_Lambda([S, Field](double V, ETextCommit::Type)
			{
				const int32 N = FMath::Clamp(FMath::RoundToInt(V), 0, 100000);
				if (N != Field()) { Field() = N; S->Changed(); }
			}),
			Vis, Tip, Note);
	}

	TSharedRef<SWidget> RealRow(TSharedRef<FMeshTabState> S, const FText& Label, TFunction<double&()> Field, double Min, double Max, double Delta, TAttribute<EVisibility> Vis, const FText& Tip, const FText& Note)
	{
		return Row(Label,
			SNew(SAerotelicNumericEntry)
			.Value_Lambda([Field]() { return Field(); })
			.MinValue(Min).MaxValue(Max).Delta(Delta).LiveUpdate(false)
			.OnValueCommitted_Lambda([S, Field](double V, ETextCommit::Type)
			{
				if (!FMath::IsNearlyEqual(V, Field())) { Field() = V; S->Changed(); }
			}),
			Vis, Tip, Note);
	}
}

TSharedRef<SWidget> MakeMeshTab(ISolidModel* Model, EntityType Type, FSimpleDelegate OnChanged)
{
	TSharedRef<FMeshTabState> S = MakeShared<FMeshTabState>();
	S->Model = Model;
	S->Type = Type;
	S->OnChanged = OnChanged;

	auto Lofted = TAttribute<EVisibility>::CreateLambda([S]() { return S->IsLofted() ? EVisibility::Visible : EVisibility::Collapsed; });
	auto WingLike = TAttribute<EVisibility>::CreateLambda([S]() { return S->IsWingLike() ? EVisibility::Visible : EVisibility::Collapsed; });
	auto Box = TAttribute<EVisibility>::CreateLambda([S]() { return S->IsBox() ? EVisibility::Visible : EVisibility::Collapsed; });
	auto Cylinder = TAttribute<EVisibility>::CreateLambda([S]() { return S->IsCylinder() ? EVisibility::Visible : EVisibility::Collapsed; });
	const FText Auto = LOCTEXT("Auto", "0 = automatic");

	// ---- the grid of a lofted body ----
	TSharedRef<SWidget> GridGroup = MakeGroup(LOCTEXT("GridGroup", "Grid"),
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[ CountRow(S, LOCTEXT("Stations", "Panels along the body:"), [S]() -> int& { return S->Spec().stations; }, 400, Lofted,
			LOCTEXT("StationsTip", "Uniform intervals along the loft, on top of its own sections, chord-law breakpoints and ends: the stations the triangulation starts from and the rows of the panel method's network. 0: the model's own minimum (2 to 8 by kind) and whatever the refinement adds."), Auto) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ CountRow(S, LOCTEXT("Around", "Panels round a section:"), [S]() -> int& { return S->Spec().around; }, 400, Lofted,
			LOCTEXT("AroundTip", "Uniform intervals round each section, on top of its corners: the points of every ring and the columns of the panel method's network. 0: the model's own minimum (12 to 24 by kind) and whatever the refinement adds."), Auto) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ Row(LOCTEXT("Refine", "Refine to tolerance:"),
			SNew(SCheckBox)
			.IsChecked_Lambda([S]() { return S->Spec().refine ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
			.OnCheckStateChanged_Lambda([S](ECheckBoxState State) { S->Spec().refine = State == ECheckBoxState::Checked; S->Changed(); }),
			Lofted,
			LOCTEXT("RefineTip", "On: points and stations are added wherever a straight chord misses the curve by more than the tolerance, and strips much longer than their rings' spacing are split (the viewport's rule). Off: the grid above is used as it stands -- the panel method's quads split in two, so the FEA shells, the wetted faces and the panels are ONE grid."),
			LOCTEXT("RefineNote", "off = the grid as it stands, shared with the panel method")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ RealRow(S, LOCTEXT("Tolerance", "Tolerance:"), [S]() -> double& { return S->Spec().tolerance; }, 0.0, 1e9, 0.0005, Lofted,
			LOCTEXT("ToleranceTip", "The chordal sag the refinement allows, in the project's length unit. 0: automatic, 0.15% of the body's diagonal (the viewport's)."), Auto) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ RealRow(S, LOCTEXT("MaxAspect", "Longest strip, times spacing:"), [S]() -> double& { return S->Spec().max_aspect; }, -1.0, 1e3, 1.0, Lofted,
			LOCTEXT("MaxAspectTip", "A strip between two stations longer than this times its rings' point spacing is split: no needle triangles into an apex or a knife edge, which a tetrahedral mesh cannot conform to. 0: the default, 6. -1: off."), LOCTEXT("AspectNote", "0 = 6, -1 = off")) ]);

	// ---- the panel method's counts for a wing ----
	TSharedRef<SWidget> PanelGroup = MakeGroup(LOCTEXT("PanelGroup", "Panel method (wing)"),
		SNew(SVerticalBox)
		.Visibility(WingLike)
		+ SVerticalBox::Slot().AutoHeight()
		[ CountRow(S, LOCTEXT("Chordwise", "Panels per surface, chordwise:"), [S]() -> int& { return S->Spec().chordwise; }, 200, WingLike,
			LOCTEXT("ChordwiseTip", "The high-order panel method's panels along the upper and along the lower surface, cosine-spaced. 0: the Panel Method dialog's figure."), LOCTEXT("AutoDialog", "0 = the dialog's")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ CountRow(S, LOCTEXT("Spanwise", "Panels over the semi-span:"), [S]() -> int& { return S->Spec().spanwise; }, 200, WingLike,
			LOCTEXT("SpanwiseTip", "The high-order panel method's panels along the span, cosine-spaced. 0: the Panel Method dialog's figure."), LOCTEXT("AutoDialog", "0 = the dialog's")) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
		[ MakeInfoLabel(LOCTEXT("WingNote", "A wing's panel networks are laid on its planform (cosine-spaced), not on the triangulation above; the two grids do not coincide yet.")) ]);

	// ---- a box's cells ----
	TSharedRef<SWidget> BoxGroup = MakeGroup(LOCTEXT("BoxGroup", "Cells"),
		SNew(SVerticalBox)
		.Visibility(Box)
		+ SVerticalBox::Slot().AutoHeight()
		[ CountRow(S, LOCTEXT("CellsX", "Cells along X:"), [S]() -> int& { return S->Spec().cells[0]; }, 1000, Box,
			LOCTEXT("CellsXTip", "Cells along the box's length. Each face is a grid of quads, every quad two triangles (so two shells). 0: automatic -- cells about as long as the shortest side."), Auto) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ CountRow(S, LOCTEXT("CellsY", "Cells along Y:"), [S]() -> int& { return S->Spec().cells[1]; }, 1000, Box, LOCTEXT("CellsYTip", "Cells along the box's width. 0: automatic."), Auto) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ CountRow(S, LOCTEXT("CellsZ", "Cells along Z:"), [S]() -> int& { return S->Spec().cells[2]; }, 1000, Box, LOCTEXT("CellsZTip", "Cells along the box's height. 0: automatic."), Auto) ]);

	// ---- a cylinder's counts ----
	TSharedRef<SWidget> CylinderGroup = MakeGroup(LOCTEXT("CylinderGroup", "Cells"),
		SNew(SVerticalBox)
		.Visibility(Cylinder)
		+ SVerticalBox::Slot().AutoHeight()
		[ CountRow(S, LOCTEXT("CylAround", "Points round the axis:"), [S]() -> int& { return S->Spec().around; }, 400, Cylinder,
			LOCTEXT("CylAroundTip", "Points round the circle: the side's polygon and the caps' rings. 0: automatic, from the chordal tolerance (at least 12)."), Auto) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ CountRow(S, LOCTEXT("CylAlong", "Cells along the axis:"), [S]() -> int& { return S->Spec().stations; }, 400, Cylinder,
			LOCTEXT("CylAlongTip", "Rows of cells along the cylinder's length. 0: automatic -- cells about as long as they are wide."), Auto) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ CountRow(S, LOCTEXT("CapRings", "Rings on each end cap:"), [S]() -> int& { return S->Spec().cap_rings; }, 200, Cylinder,
			LOCTEXT("CapRingsTip", "Concentric rings of cells on each end cap, closing on the axis point. 0: automatic -- rings about as wide as the side's cells."), Auto) ]);

	S->CountLabel = SNew(STextBlock).AutoWrapText(true);

	TSharedRef<SWidget> Tab = SNew(SScrollBox)
		+ SScrollBox::Slot().Padding(8)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
			[ MakeInfoLabel(LOCTEXT("Intro", "The density of this body's grid. One grid serves the viewport, the structural analysis (a shell on every triangle; the tetrahedra bounded by them), the flow solvers' wetted surface and, through the same counts, the panel method. Saved with the entity. Apply to rebuild.")) ]
			+ SVerticalBox::Slot().AutoHeight() [ GridGroup ]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0) [ PanelGroup ]
			+ SVerticalBox::Slot().AutoHeight() [ BoxGroup ]
			+ SVerticalBox::Slot().AutoHeight() [ CylinderGroup ]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0) [ S->CountLabel.ToSharedRef() ]
		];
	// The lofted group's rows hide themselves for a box / cylinder; the
	// group title would stay, so the whole group follows.
	GridGroup->SetVisibility(TAttribute<EVisibility>(Lofted));
	S->RefreshCount();
	return Tab;
}

FText MeshTabHelpText(EntityType Type)
{
	FString Text = TEXT("MESH TAB\n")
		TEXT("The density of the body's grid, saved with it. One grid serves everything: the viewport, the structural analysis (a shell on every triangle, or tetrahedra bounded by them), the flow solvers' wetted surface and, through the same counts, the panel method. ")
		TEXT("Every field at 0 is automatic -- what the model did before the tab existed.\n");
	if (Type == EntityType::Primitive)
	{
		Text += TEXT("A box has cells along X, Y and Z (automatic: cells about as long as its shortest side -- a 1 x 10 x 1 bar is 1 x 10 x 1 cells; the single cell of old was 12 triangles, which no analysis could use). ")
			TEXT("A cylinder has points round the axis, cells along it and rings on each cap. A sphere is a loft like the bodies below.\n");
	}
	Text += TEXT("A lofted body: Panels along and Panels round are uniform intervals on top of the loft's own sections and corners -- the stations and ring points the triangulation starts from and, with the same numbers, the rows and columns of the panel method's network. ")
		TEXT("Refine to tolerance adds points and stations wherever a straight chord misses the curve by more than the tolerance (automatic: 0.15% of the body's diagonal) and splits strips longer than the aspect limit times their rings' spacing; switched off, the grid stands as given and the triangles are the panel method's quads split in two -- the FEA and the flow then share one grid exactly.");
	if (Type == EntityType::Wing || Type == EntityType::Propeller)
	{
		Text += TEXT("\nA wing's panel-method counts (chordwise per surface, spanwise) are set here too; its networks are laid on the planform, cosine-spaced, and do not yet coincide with the triangulation.");
	}
	return FText::FromString(Text);
}

#undef LOCTEXT_NAMESPACE
