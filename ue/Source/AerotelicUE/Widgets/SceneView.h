#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "Styling/SlateBrush.h"
#include "UObject/StrongObjectPtr.h"
// Complete, not forward-declared: TStrongObjectPtr checks the type.
#include "Engine/TextureRenderTarget2D.h"

class AAerotelicSceneViewActor;
class SImage;

// A 3D view inside a Slate dialog: the engine renders a chosen set of
// actors -- and only those -- through a scene capture into a texture this
// widget shows, lit by the scene's own lighting rig, and the widget drives
// the camera with the main view's conventions (AerotelicCameraPawn): left-
// drag orbits about the focus point, right- or middle-drag pans, the wheel
// zooms, Home (or F) frames what is shown. Overlays (a colour bar, a case
// selector) sit on top of the picture through the Overlay slot.
//
// Reusable: the post-processing of any analysis that wants a picture of a
// field on the geometry (AAerotelicColoredMeshActor) puts the actor in the
// view with SetShownActors and frames it. The capture renders only when
// something changed (the camera, the size, the content), so an idle view
// costs nothing.
class SAerotelicSceneView : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SAerotelicSceneView) : _FieldOfViewDeg(60.0f) {}
		SLATE_ARGUMENT(float, FieldOfViewDeg)
		SLATE_NAMED_SLOT(FArguments, Overlay)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);
	virtual ~SAerotelicSceneView() override;

	// The actors the view draws. Nothing else in the world appears.
	void SetShownActors(const TArray<AActor*>& Actors);
	// Frames a box as the main view's camera does (bounding sphere inside
	// the vertical field of view).
	void FrameBox(const FBox& Box);
	// Frames the shown actors' bounds.
	void FrameShown();
	// Camera as yaw/pitch (degrees) about the focus point.
	void SetView(double Yaw, double Pitch);
	// Re-renders over the next few frames (after the shown content changed).
	void RequestRedraw() { bDirty = true; }
	// The backdrop colour; unset, it follows the theme's viewport colour.
	void SetBackdropColor(const FLinearColor& Color);

	// SWidget
	virtual void Tick(const FGeometry& AllottedGeometry, const double InCurrentTime, const float InDeltaTime) override;
	virtual FReply OnMouseButtonDown(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent) override;
	virtual FReply OnMouseButtonUp(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent) override;
	virtual FReply OnMouseMove(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent) override;
	virtual FReply OnMouseWheel(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent) override;
	virtual FReply OnKeyDown(const FGeometry& MyGeometry, const FKeyEvent& InKeyEvent) override;
	virtual bool SupportsKeyboardFocus() const override { return true; }
	virtual FCursorReply OnCursorQuery(const FGeometry& MyGeometry, const FPointerEvent& CursorEvent) const override;

private:
	TWeakObjectPtr<AAerotelicSceneViewActor> ViewActor;
	TStrongObjectPtr<UTextureRenderTarget2D> RenderTarget;
	FSlateBrush Brush;
	TSharedPtr<SImage> Image;
	TArray<TWeakObjectPtr<AActor>> Shown;

	float FieldOfViewDeg = 60.0f;
	FVector FocusPoint = FVector::ZeroVector;
	double Distance = 30.0;
	double Yaw = -135.0;
	double Pitch = -25.0;

	bool bDragging = false;
	bool bDragIsLeft = false;
	bool bDirty = true;
	// Capture every frame until this time (FPlatformTime seconds) after the
	// last change. A change is captured for a while, not once: a procedural
	// mesh created this frame reaches the capture before its vertex colour
	// stream does -- for several frames the colours come out white, then
	// right -- so the picture is retaken for a second to be safe. Idle, the
	// view captures nothing.
	double CaptureUntilSeconds = 0.0;
	FIntPoint TargetSize = FIntPoint::ZeroValue;
	TOptional<FLinearColor> BackdropColor;

	// Spawns the capture actor in the game world the first time the view is
	// ticked (there is none to spawn into in a commandlet, where the widget
	// then shows nothing).
	bool EnsureActor();
	void EnsureTarget(const FIntPoint& Size);
	void Orbit(double DeltaX, double DeltaY);
	void Pan(double DeltaX, double DeltaY, const FVector2D& ViewSize);
	void Zoom(double Factor);
};
