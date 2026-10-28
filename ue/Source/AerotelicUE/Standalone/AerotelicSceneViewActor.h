#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "AerotelicSceneViewActor.generated.h"

class USceneCaptureComponent2D;
class UDirectionalLightComponent;
class UStaticMeshComponent;
class UMaterialInstanceDynamic;
class UTextureRenderTarget2D;

// The scene-side half of SAerotelicSceneView: a scene capture that renders
// a chosen set of actors -- and only those -- into a texture the widget
// shows, with its own backdrop (the main view's grey sphere again, seen by
// captures only) so the picture has the same ground as the 3D view rather
// than a black void.
//
// Lighting: the capture carries its own rig, fixed to the eye -- a key
// light along the line of sight and four fills from either side, above and
// below it -- so every surface the picture shows is lit whichever way the
// result is turned; the main view's studio rig, fixed in the world, left
// the far side of a body in shadow. The rig lives on lighting channel 2
// alone (see LightingChannel) and so lights nothing in the main scene,
// whose bodies are on channel 0; a result mesh is put on channel 2 alone
// (AAerotelicColoredMeshActor does) and so sees nothing of the studio rig.
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

	// The lighting channel the eye-fixed rig lights and a shown mesh must
	// be on (0 is the scene's, 1 is left free).
	static constexpr int32 LightingChannel = 2;

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

	// The eye-fixed rig: children of the capture, so they turn with it.
	UPROPERTY()
	TArray<TObjectPtr<UDirectionalLightComponent>> Lights;

	FDelegateHandle ThemeHandle;
	TOptional<FLinearColor> FixedBackdrop;

	void ApplyTheme();
};
