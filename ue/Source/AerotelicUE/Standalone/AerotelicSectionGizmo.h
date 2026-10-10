#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Entity/AerotelicViewStyle.h"

#include "AerotelicSectionGizmo.generated.h"

class UStaticMeshComponent;
class UPrimitiveComponent;
class UMaterialInstanceDynamic;

// The section plane's handle in the 3D view (core/view_gizmo.py
// SectionPlane's gizmo): an amber rectangle outlining the plane, padded
// around the scene's extent, and a double-headed arrow along the axis
// that the user drags to move the plane. The arrow draws on top of the
// bodies so it stays reachable when the plane is inside one. Hidden while
// no section axis is chosen.
//
// The camera pawn does the picking and dragging (it owns the mouse); this
// actor supplies the hit test, the drag parameter and the clamp.
UCLASS()
class AAerotelicSectionGizmo : public AActor
{
	GENERATED_BODY()

public:
	AAerotelicSectionGizmo();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

	// The gizmo of the running application, or null in the Editor host.
	static AAerotelicSectionGizmo* FindInGameWorld();

	// Whether the cursor (viewport pixels) is on the arrow handle: within a
	// few pixels of the arrow's projected segment. Done in screen space
	// rather than with a physics trace, because the shaft is only a few
	// pixels thick and the handle must be reachable even when a body is in
	// front of it (it draws on top).
	bool IsCursorOnHandle(const class APlayerController* PC, float MouseX, float MouseY) const;

	// Parameter along the plane's axis of the point on the arrow's line
	// closest to the view ray; false when the ray runs nearly along the
	// axis (view_gizmo.py axis_drag_parameter, ~0.6 degrees).
	bool DragParameter(const FVector& RayOrigin, const FVector& RayDir, double& OutT) const;
	// Relative drag: the plane keeps its offset from the cursor's parameter
	// at the press, and is clamped to the scene box plus a 2% margin.
	void BeginDrag(double T);
	void DragTo(double T);

	static FString FrameMaterialPackage();
	static FString HandleMaterialPackage();

private:
	UPROPERTY()
	TObjectPtr<USceneComponent> SceneRoot;

	// Four thin bars: the plane's outline.
	UPROPERTY()
	TArray<TObjectPtr<UStaticMeshComponent>> Frame;

	// The arrow: a shaft and two heads.
	UPROPERTY()
	TObjectPtr<UStaticMeshComponent> Shaft;

	UPROPERTY()
	TObjectPtr<UStaticMeshComponent> HeadA;

	UPROPERTY()
	TObjectPtr<UStaticMeshComponent> HeadB;

	UPROPERTY()
	TObjectPtr<UMaterialInstanceDynamic> FrameMID;

	UPROPERTY()
	TObjectPtr<UMaterialInstanceDynamic> HandleMID;

	FBox SceneBox = FBox(ForceInit);
	FAerotelicSectionPlane Plane;
	// The arrow's end points (world), kept for the screen-space hit test.
	FVector ArrowA = FVector::ZeroVector;
	FVector ArrowB = FVector::ZeroVector;
	double DragT0 = 0.0;
	double DragPos0 = 0.0;
	FDelegateHandle SectionHandle;
	FDelegateHandle ListHandle;

	void OnSectionChanged();
	void Refit();
	void Rebuild();
	void SetShown(bool bShown);
	// The arrow's line: through the scene centre, along the axis.
	FVector AxisLineOrigin() const;
};
