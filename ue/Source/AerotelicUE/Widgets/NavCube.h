#pragma once

#include "CoreMinimal.h"
#include "Widgets/SLeafWidget.h"

// The navigation cube (core/view_gizmo.py NavigationCube, WingDesigner's
// AIS_ViewCube): a small shaded cube in the top-right corner of the 3D
// view, turning with the camera, its faces labelled FRONT / BACK / LEFT /
// RIGHT / TOP / BOTTOM after the application's own view presets (FRONT is
// the face the Front preset looks at, from -X), with a little XYZ triad at
// its corner. Hovering lights the face, edge or corner under the cursor;
// clicking swings the camera to look at it: the centre of a face gives a
// face-on view, near an edge a two-axis view, a corner the three-axis
// isometric-like view (classify_point with face_zone = 0.72). The swing
// is animated by the camera pawn.
//
// Drawn entirely with Slate (filled quads + lines + text); no scene
// objects, so the section plane and the styles never touch it.
class SAerotelicNavCube : public SLeafWidget
{
public:
	SLATE_BEGIN_ARGS(SAerotelicNavCube) {}
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
		FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const override;
	virtual FVector2D ComputeDesiredSize(float) const override;
	virtual FReply OnMouseButtonDown(const FGeometry& Geometry, const FPointerEvent& Event) override;
	virtual FReply OnMouseButtonUp(const FGeometry& Geometry, const FPointerEvent& Event) override;
	virtual FReply OnMouseMove(const FGeometry& Geometry, const FPointerEvent& Event) override;
	virtual void OnMouseEnter(const FGeometry& Geometry, const FPointerEvent& Event) override;
	virtual void OnMouseLeave(const FPointerEvent& Event) override;
	virtual FReply OnMouseWheel(const FGeometry& Geometry, const FPointerEvent& Event) override { return FReply::Handled(); }
	virtual FCursorReply OnCursorQuery(const FGeometry& Geometry, const FPointerEvent& Event) const override;

private:
	// The camera axes read at paint/hit time; false when there is no
	// camera (the Editor host).
	bool GetBasis(FVector& Forward, FVector& Right, FVector& Up) const;
	// Signs (-1/0/+1 per axis) of the view direction the cursor picks, or
	// false when the cursor misses the cube.
	bool ClassifyCursor(const FGeometry& Geometry, const FVector2D& ScreenPos, FIntVector& OutSigns) const;

	// The signs under the cursor, (0,0,0) when none.
	FIntVector Hover = FIntVector::ZeroValue;
	bool bHovering = false;
};
