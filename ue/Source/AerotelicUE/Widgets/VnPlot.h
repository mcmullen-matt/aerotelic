#pragma once

#include "CoreMinimal.h"
#include "Widgets/SLeafWidget.h"

#include "aerotelic/load_cases.hpp"

// A load case's point on the V-n diagram.
struct FAerotelicVnPoint
{
	double Speed = 0.0;        // m/s
	double LoadFactor = 1.0;
	FString Name;
	bool bSolved = false;      // drawn filled when its pressures have been solved
};

// The V-n diagram (Model > Load Cases, aero tab, 2026-11-15): the flight
// envelope of a V-n diagram painted with Slate -- the positive stall
// boundary n = q CL_max S / W rising from the origin to the structural
// limit n_max at the manoeuvre speed V_A, the limit out to the never-exceed
// speed V_ne, the vertical at V_ne down to n_min, the negative limit back to
// where the negative stall boundary (CL_min) meets it, and that boundary
// back to the origin; the stall speed V_S (n = 1) and V_A marked -- with
// the load cases as points on it. Interactive: a click inside the plot
// adds a point there (OnAddPoint), a press on a point selects it and a drag
// moves it (OnSelectPoint, OnMovePoint); the dialog's table is the other
// view of the same list. Points may lie outside the envelope (the diagram
// says so by colour); the dialog does not refuse them.
class SAerotelicVnPlot : public SLeafWidget
{
public:
	SLATE_BEGIN_ARGS(SAerotelicVnPlot) : _DesiredSize(FVector2D(560.0, 360.0)) {}
		SLATE_ARGUMENT(FVector2D, DesiredSize)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	// The envelope's figures (SI) with the weight (N) and area (m^2) resolved.
	void SetEnvelope(const aerotelic::VnDiagramSpec& InSpec, double InWeight, double InArea);
	void SetPoints(const TArray<FAerotelicVnPoint>& InPoints);
	void SetSelected(int32 Index);
	int32 GetSelected() const { return Selected; }

	TFunction<void(double Speed, double LoadFactor)> OnAddPoint;
	TFunction<void(int32 Index)> OnSelectPoint;
	TFunction<void(int32 Index, double Speed, double LoadFactor)> OnMovePoint;
	// Fired when a drag ends (the dialog commits the moved point then).
	TFunction<void(int32 Index)> OnMoveEnd;

	// The envelope's corners, for the caption: V_S at n = 1, V_A where the
	// positive stall boundary meets n_max, the negative stall speed at n = -1
	// and the negative corner speed; 0 where undefined.
	double StallSpeed() const;
	double ManoeuvreSpeed() const;
	double NegativeStallSpeed() const;
	double NegativeCornerSpeed() const;
	// Whether (V, n) lies inside the envelope.
	bool Inside(double Speed, double LoadFactor) const;

	virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
		FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const override;
	virtual FVector2D ComputeDesiredSize(float) const override { return DesiredSize; }
	virtual FReply OnMouseButtonDown(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent) override;
	virtual FReply OnMouseButtonUp(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent) override;
	virtual FReply OnMouseMove(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent) override;
	virtual void OnMouseLeave(const FPointerEvent& MouseEvent) override;
	virtual FCursorReply OnCursorQuery(const FGeometry& MyGeometry, const FPointerEvent& CursorEvent) const override;

private:
	aerotelic::VnDiagramSpec Spec;
	double Weight = 0.0, Area = 0.0;
	TArray<FAerotelicVnPoint> Points;
	int32 Selected = -1;
	int32 Hover = -1;
	int32 Dragging = -1;
	FVector2D DesiredSize;

	// The plot's data ranges and the mapping to pixels.
	struct FFrame
	{
		double VMin = 0.0, VMax = 1.0, NMin = -1.0, NMax = 1.0;
		FVector2D Origin, Size;   // the plot area in local pixels
		FVector2D ToPixel(double V, double N) const;
		void FromPixel(const FVector2D& P, double& V, double& N) const;
	};
	FFrame Frame(const FVector2D& LocalSize) const;
	int32 PointAt(const FGeometry& Geometry, const FVector2D& Pixel) const;
	double StallN(double CL, double V) const { return aerotelic::VnDiagramSpec::StallLoadFactor(CL, V, Spec.density, Weight, Area); }
};
