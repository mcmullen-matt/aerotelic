#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/SLeafWidget.h"

#include "aerotelic/nurbs.hpp"

// The shared dual-curve/single-curve editing tool -- the Slate port of
// aerotelic/ui/nurbs_editor.py's NURBSEditorCanvas. One or two NurbsCurves
// ("slots") drawn and dragged in a curve pane, with an optional blending-
// function/knot pane per slot above or below it, stacked in a resizable
// SSplitter exactly as the Python tool does.
//
// Deliberately NOT ported (see the approved plan's scope cuts): reference-
// image display/drag/rotate (the hook points matplotlib's canvas reserved
// for it have no Slate equivalent here at all), and `pole_links` persistence
// -- the C++ backend never carried pole_links through (see wing_model.hpp's
// file header: "UI-editor display/bookkeeping that no geometry reads"), so
// a "follow" link made here is a live, in-session convenience only; it is
// not saved with the project and must be re-made after a reload. Everything
// else -- pole drag/add/remove/weight, knot drag/insert/delete, the fit
// view, static (rigid) pairings, outline-only display panes -- is ported.

// Where a curve slot's knot track goes relative to the shared curve pane.
enum class ECurveBasisPlacement : uint8 { None, Above, Below };

// One curve being edited (or displayed) in the editor. Curve is a
// non-owning pointer into the model the dialog is bound to -- it must
// outlive this struct's use (the dialog owns the model; the editor widget
// is rebound via SCurveEditor::SetSlots whenever the dialog switches models).
struct FCurveEditorSlot
{
	aerotelic::NurbsCurve* Curve = nullptr;
	FLinearColor Color = FLinearColor(0.227f, 0.482f, 0.835f); // #3a7bd5
	FText Label;
	ECurveBasisPlacement Basis = ECurveBasisPlacement::Below;
	// Poles that cannot move or be removed; a negative index counts from
	// the end (survives poles being added/removed), matching CurveSlot.
	TMap<int32, FVector2D> FixedPoles;
	TOptional<double> YClampLo, YClampHi;
	bool bClosedFill = false;
	// The curve is a shape in the plane, not a function of x (e.g. a
	// nacelle's front-view lip ring): no pole is held to an x, and nothing
	// is kept inside 0..1.
	bool bFreeX = false;

	TOptional<FVector2D> FixedPole(int32 Index) const;
};

// A read-only polyline drawn in the curve pane -- a shape that follows
// numeric inputs rather than poles (the trapezoidal planform outline), or
// context drawn alongside editable curves (a lofted nacelle's other guide
// curves, a nacelle's chevron-valley marker).
struct FCurveEditorOutlinePath
{
	TArray<FVector2D> Points; // in curve units; the pane applies XScale/YScale
	FLinearColor Color = FLinearColor::White;
	FText Label;
	bool bClosed = false;
	bool bFill = false;
	float FillAlpha = 0.12f;
	float LineWidth = 1.5f;
};

// A rigid pairing: dragging either pole moves both, and neither can be
// removed (the nacelle's shared lip/trailing-edge points). Indices are
// (slot, pole); a negative pole index counts from the end.
struct FCurveEditorLink
{
	FIntPoint A = FIntPoint::ZeroValue;
	FIntPoint B = FIntPoint::ZeroValue;
};

// A one-way "follow" pairing the user makes/breaks from a pole's right-
// click menu: Follower moves when Target is dragged; grabbing Follower
// drags Target. In-session only -- see this file's header note on why.
struct FCurveEditorUserLink
{
	FIntPoint Follower = FIntPoint::ZeroValue;
	FIntPoint Target = FIntPoint::ZeroValue;
};

DECLARE_DELEGATE_RetVal(TArray<FCurveEditorOutlinePath>, FOnGetOutlinePaths);

class SCurveEditor : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SCurveEditor)
		: _XScale(1.0)
		, _YScale(1.0)
		, _EqualAspect(false)
		, _SupportsUserLinks(false)
		, _LeadingName(FText::FromString(TEXT("leading")))
		, _TrailingName(FText::FromString(TEXT("trailing")))
		{}
		SLATE_ARGUMENT(FText, XLabel)
		SLATE_ARGUMENT(FText, YLabel)
		SLATE_ARGUMENT(FText, Title)
		SLATE_ARGUMENT(double, XScale)
		SLATE_ARGUMENT(double, YScale)
		// One display unit is the same number of pixels on both axes --
		// only meaningful when both axes mean the same kind of quantity.
		SLATE_ARGUMENT(bool, EqualAspect)
		SLATE_ARGUMENT(bool, SupportsUserLinks)
		SLATE_ARGUMENT(FText, LeadingName)
		SLATE_ARGUMENT(FText, TrailingName)
		SLATE_EVENT(FSimpleDelegate, OnChanged)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	// Rebinds every slot, e.g. when the dialog is pointed at a different
	// entity. The view is kept (the axes are non-dimensional; see
	// nurbs_editor.py's set_curves() docstring for the same call).
	void SetSlots(TArray<FCurveEditorSlot> InSlots);
	void SetLinks(TArray<FCurveEditorLink> InLinks) { Links = MoveTemp(InLinks); }
	void SetOutlineProvider(FOnGetOutlinePaths InProvider) { OutlineProvider = InProvider; }
	void SetScales(double NewXScale, double NewYScale);

	// The live, in-session follow-link list (see FCurveEditorUserLink).
	const TArray<FCurveEditorUserLink>& GetUserLinks() const { return UserLinks; }
	void SetUserLinks(TArray<FCurveEditorUserLink> InLinks) { UserLinks = MoveTemp(InLinks); RequestRedraw(); }

	void FitView();
	// Call after any out-of-band edit (Reset Knots, Reset to Default, degree
	// change) so the panes repaint and the splitter/basis panes rebuild.
	void RequestRedraw();

private:
	friend class SCurveEditorPane;

	TArray<FCurveEditorSlot> Slots;
	TArray<FCurveEditorLink> Links;
	TArray<FCurveEditorUserLink> UserLinks;
	bool bSupportsUserLinks = false;
	FOnGetOutlinePaths OutlineProvider;
	FSimpleDelegate OnChangedDelegate;

	FText XLabel, YLabel, Title, LeadingName, TrailingName;
	double XScale = 1.0, YScale = 1.0;
	bool bEqualAspect = false;

	TSharedPtr<class SSplitter> Splitter;
	TSharedPtr<class SCurveEditorPane> CurvePane;
	TArray<TSharedPtr<class SCurveEditorPane>> BasisPanes; // parallel to Slots, null where Basis==None

	// ---- shared editing state (mirrors nurbs_editor.py's instance state) ----
	TOptional<FIntPoint> Selected;         // (slot, pole) or (slot, basis function)
	TOptional<FIntPoint> DraggingPole;     // (slot, pole)
	bool bDragMoved = false;
	TOptional<FIntPoint> DraggingKnot;     // (slot, knot edit index)
	TOptional<FIntPoint> LastPoleHit;      // for cycling through coincident poles
	bool bNotifying = false;

	// View box (x_lo, x_hi, y_lo, y_hi) in display units. Unset means "not
	// framed yet" -- the next draw with content adopts the fit box, exactly
	// once; after that only the wheel and Fit View move it.
	TOptional<FBox2D> ViewRequest;
	FBox2D FitBox = FBox2D(FVector2D(0, 0), FVector2D(1, 1));
	TOptional<FBox2D> LimLock; // pins the box for the duration of a drag

	void RebuildPanes();
	void NotifyChanged();

	// ---- topology / constraints (mirrors the Python methods of the same
	// name/intent) ----
	FVector2D ConstrainPole(const FCurveEditorSlot& Slot, int32 PoleIndex, FVector2D Requested) const;
	static bool IsEndPole(const FCurveEditorSlot& Slot, int32 PoleIndex);
	TArray<FIntPoint> Partners(int32 SlotIndex, int32 PoleIndex) const;
	bool CanRemovePole(const FCurveEditorSlot& Slot, int32 SlotIndex, int32 PoleIndex) const;
	void MovePole(int32 SlotIndex, int32 PoleIndex, FVector2D Requested);
	bool Linkable(int32 SlotIndex) const;
	TOptional<FIntPoint> TargetOf(FIntPoint FollowerSlotPole) const;
	TArray<FIntPoint> FollowersOf(FIntPoint TargetSlotPole) const;
	TOptional<int32> EndOf(int32 SlotIndex, int32 PoleIndex) const; // 0, -1 (last), or unset
	bool EnforceLinks();
	void LinkPole(int32 SlotIndex, int32 PoleIndex, int32 TargetSlot, int32 TargetPole);
	void UnlinkPole(int32 SlotIndex, int32 PoleIndex);

	FBox2D ComputeFitBox() const;
	FBox2D Equalized(const FBox2D& Box, FVector2D PaneSizePx) const;

	void OnPoleWeightEdited(int32 SlotIndex, int32 PoleIndex, double NewWeight);
	void RemovePole(int32 SlotIndex, int32 PoleIndex);
	void InsertKnot(int32 SlotIndex, double Value);
	void DeleteKnot(int32 SlotIndex, int32 EditIndex);
	void ResetKnotsForSlot(int32 SlotIndex);

	// ---- pane-facing API (called by SCurveEditorPane, a friend; the pane
	// owns pixel-space hit-testing/painting, this class owns curve-space
	// semantics) ----
	FBox2D GetOrComputeViewBoxForPane(FVector2D PaneSizePx);
	TOptional<FIntPoint> PoleAt(FVector2D Local, const FBox2D& Box, FVector2D SizePx) const;
	TOptional<int32> KnotAt(int32 SlotIndex, FVector2D Local, const FBox2D& Box, FVector2D SizePx) const;
	void SelectBasisFunction(int32 SlotIndex, FVector2D Local, const FBox2D& Box, FVector2D SizePx);
	void AddPoleAtLocal(FVector2D Local, const FBox2D& Box, FVector2D SizePx);
	void DeselectAll();

	bool IsDraggingPole() const { return DraggingPole.IsSet(); }
	void BeginDragPole(FIntPoint Hit, const FBox2D& CurrentBox);
	void DragPoleTo(FVector2D Local, const FBox2D& Box, FVector2D SizePx);
	void ReleasePole();

	bool IsDraggingKnot(int32 SlotIndex) const { return DraggingKnot.IsSet() && DraggingKnot->X == SlotIndex; }
	void BeginDragKnot(int32 SlotIndex, int32 EditIndex) { DraggingKnot = FIntPoint(SlotIndex, EditIndex); }
	void DragKnotTo(int32 SlotIndex, FVector2D Local, const FBox2D& Box, FVector2D SizePx);
	void ReleaseKnot();

	void ZoomAboutLocal(FVector2D Local, const FBox2D& Box, FVector2D SizePx, float WheelDelta);

	void BuildPoleMenu(class FMenuBuilder& MenuBuilder, FIntPoint Hit);
	void BuildKnotMenu(class FMenuBuilder& MenuBuilder, int32 SlotIndex, TOptional<int32> KnotEditIndex, double ClickS);

	// Index into a curve's poles the add/insert position for a double-click
	// at DataPos should take (open: by x; periodic: nearest polygon edge).
	int32 InsertIndexFor(const FCurveEditorSlot& Slot, FVector2D DataPos, FVector2D Local, const FBox2D& Box, FVector2D SizePx) const;
};
