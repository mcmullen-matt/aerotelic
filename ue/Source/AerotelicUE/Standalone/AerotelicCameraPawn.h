#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

#include "AerotelicCameraPawn.generated.h"

class UCameraComponent;

// The View menu / Orientation dropdown presets (occ_viewer.py's set_view).
enum class EAerotelicViewPreset : uint8
{
	Isometric,
	Top,
	Front,
	Side,
};

// The standalone application's 3D-view camera: a CAD-style orbit camera
// (the Editor viewport's own navigation, which the standalone app no longer
// has). Left-drag orbits about the focus point, right/middle-drag pans,
// the wheel zooms, F frames the selected entity and Home frames the whole
// scene. A click (press/release without dragging) on an entity selects it.
//
// Input is polled from the player controller every tick rather than bound
// through an input-mapping asset: there is nothing to configure and nothing
// to author in the Editor for it.
UCLASS()
class AAerotelicCameraPawn : public APawn
{
	GENERATED_BODY()

public:
	AAerotelicCameraPawn();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

	// The camera pawn of the running standalone application, or null when
	// this module is hosted by the Editor (which has no game world).
	static AAerotelicCameraPawn* FindInGameWorld();

	void FrameBox(const FBox& Box);
	void FrameAll();
	void FrameSelectedOrAll();

	// Swings the camera to a preset direction and refits the whole scene,
	// as occ_viewer.py's set_view() does.
	void SetPresetView(EAerotelicViewPreset Preset);

	// Rebuilds the selected entity's mesh (View -> Rebuild, F5). Returns
	// false when nothing is selected.
	static bool RebuildSelectedEntity();

private:
	UPROPERTY(VisibleAnywhere, Category = "Aerotelic")
	TObjectPtr<UCameraComponent> Camera;

	// Orbit state. The camera sits Distance away from FocusPoint, looking at
	// it, along the direction given by Yaw/Pitch (degrees).
	FVector FocusPoint = FVector::ZeroVector;
	double Distance = 30.0;
	double Yaw = -135.0;
	double Pitch = -25.0;

	// Drag tracking (screen pixels).
	bool bDragging = false;
	bool bDragIsLeftButton = false;
	// A left press on the section gizmo's arrow moves the section plane
	// instead of orbiting (and never counts as a click-select).
	bool bDraggingSection = false;
	FVector2D LastMouse = FVector2D::ZeroVector;
	double DragTravel = 0.0;

	// Set when the scene goes from empty to non-empty (first Add, or Open
	// Project); consumed on the next tick, once the new meshes' bounds exist.
	bool bPendingFrameAll = false;
	int32 LastKnownEntityCount = 0;
	FDelegateHandle ListChangedHandle;

	void OnEntityListChanged();
	void UpdateCameraTransform();
	void Orbit(double DeltaX, double DeltaY);
	void Pan(double DeltaX, double DeltaY, const FVector2D& ViewportSize);
	void Zoom(double Factor);
	void ClickSelect(class APlayerController* PC);
	// True (and the drag begun) if the cursor is over the section gizmo's
	// arrow handle.
	bool TryBeginSectionDrag(class APlayerController* PC);
	void ContinueSectionDrag(class APlayerController* PC);
};
