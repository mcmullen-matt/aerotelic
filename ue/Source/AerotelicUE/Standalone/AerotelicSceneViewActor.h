#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "AerotelicSceneViewActor.generated.h"

class USceneCaptureComponent2D;
class UStaticMeshComponent;
class UMaterialInstanceDynamic;
class UTextureRenderTarget2D;

// The scene-side half of SAerotelicSceneView: a scene capture that renders
// a chosen set of actors -- and only those -- into a texture the widget
// shows, with its own backdrop (the main view's grey sphere again, seen by
// captures only) so the picture has the same ground as the 3D view rather
// than a black void. The capture lives in the one game world, so the
// studio lighting rig lights the result exactly as it lights the scene.
// Transient; spawned and destroyed by the widget.
UCLASS()
class AAerotelicSceneViewActor : public AActor
{
	GENERATED_BODY()

public:
	AAerotelicSceneViewActor();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	USceneCaptureComponent2D* GetCapture() const { return Capture; }

	// The actors the capture draws (plus its own backdrop). Everything else
	// in the world -- the scene's bodies, the ground grid, the gizmos -- is
	// left out.
	void SetShownActors(const TArray<AActor*>& Actors);

	void SetRenderTarget(UTextureRenderTarget2D* Target);

	// Places the camera: Distance back from Focus along the direction given
	// by Yaw/Pitch (degrees), the orbit-camera convention of the main view.
	void SetCamera(const FVector& Focus, double Distance, double Yaw, double Pitch, float FieldOfViewDeg);

	// Renders the next frame into the target (deferred to the next scene
	// render, so one call per frame at most costs one extra view).
	void RequestCapture();

	// A fixed backdrop colour in place of the theme's.
	void SetBackdropColor(const FLinearColor& Color);

private:
	UPROPERTY()
	TObjectPtr<USceneComponent> Root;

	UPROPERTY()
	TObjectPtr<USceneCaptureComponent2D> Capture;

	UPROPERTY()
	TObjectPtr<UStaticMeshComponent> Backdrop;

	UPROPERTY()
	TObjectPtr<UMaterialInstanceDynamic> BackdropMaterial;

	FDelegateHandle ThemeHandle;
	TOptional<FLinearColor> FixedBackdrop;

	void ApplyTheme();
};
