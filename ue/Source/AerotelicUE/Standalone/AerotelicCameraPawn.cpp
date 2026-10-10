#include "AerotelicCameraPawn.h"

#include "Camera/CameraComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "Engine/LocalPlayer.h"
#include "InputCoreTypes.h"

#include "AerotelicSectionGizmo.h"
#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"

DEFINE_LOG_CATEGORY_STATIC(LogAerotelicCamera, Log, All);

namespace
{
	constexpr double kOrbitDegreesPerPixel = 0.35;
	constexpr double kZoomStep = 0.85;          // wheel notch: distance *= / /= this
	constexpr double kClickTravelPixels = 4.0;  // press/release moving less than this is a click, not a drag
	constexpr double kMinDistance = 0.05;
	constexpr double kMaxDistance = 1.0e6;
	constexpr float kFieldOfViewDeg = 60.0f;
}

AAerotelicCameraPawn::AAerotelicCameraPawn()
{
	PrimaryActorTick.bCanEverTick = true;

	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = Root;

	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->SetupAttachment(Root);
	Camera->FieldOfView = kFieldOfViewDeg;
	Camera->bUsePawnControlRotation = false;

	bUseControllerRotationPitch = false;
	bUseControllerRotationYaw = false;
	bUseControllerRotationRoll = false;
}

void AAerotelicCameraPawn::BeginPlay()
{
	Super::BeginPlay();
	UpdateCameraTransform();

	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		LastKnownEntityCount = Doc->GetEntities().Num();
		// A project opened from the command line is loaded by the game mode's
		// BeginPlay, which runs before this one -- frame it now.
		bPendingFrameAll = LastKnownEntityCount > 0;
		ListChangedHandle = Doc->OnEntityListChanged.AddUObject(this, &AAerotelicCameraPawn::OnEntityListChanged);
	}
}

void AAerotelicCameraPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		Doc->OnEntityListChanged.Remove(ListChangedHandle);
	}
	Super::EndPlay(EndPlayReason);
}

AAerotelicCameraPawn* AAerotelicCameraPawn::FindInGameWorld()
{
	if (GEngine == nullptr)
	{
		return nullptr;
	}
	for (const FWorldContext& Context : GEngine->GetWorldContexts())
	{
		UWorld* World = Context.World();
		if (Context.WorldType != EWorldType::Game || World == nullptr)
		{
			continue;
		}
		if (APlayerController* PC = World->GetFirstPlayerController())
		{
			return Cast<AAerotelicCameraPawn>(PC->GetPawn());
		}
	}
	return nullptr;
}

void AAerotelicCameraPawn::OnEntityListChanged()
{
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	const int32 Count = Doc ? Doc->GetEntities().Num() : 0;
	if (LastKnownEntityCount == 0 && Count > 0)
	{
		bPendingFrameAll = true;
	}
	LastKnownEntityCount = Count;
}

void AAerotelicCameraPawn::UpdateCameraTransform()
{
	const FRotator Rotation(Pitch, Yaw, 0.0);
	const FVector Location = FocusPoint - Rotation.Vector() * Distance;
	SetActorLocationAndRotation(Location, Rotation);
}

void AAerotelicCameraPawn::Orbit(double DeltaX, double DeltaY)
{
	Yaw = FMath::Fmod(Yaw + DeltaX * kOrbitDegreesPerPixel, 360.0);
	// Dragging upward (negative DeltaY) lifts the camera over the model.
	Pitch = FMath::Clamp(Pitch + DeltaY * kOrbitDegreesPerPixel, -89.0, 89.0);
	UpdateCameraTransform();
}

void AAerotelicCameraPawn::Pan(double DeltaX, double DeltaY, const FVector2D& ViewportSize)
{
	// World units per screen pixel at the focus distance, so the model
	// tracks the cursor 1:1 while panning.
	const double ViewHeightWorld = 2.0 * Distance * FMath::Tan(FMath::DegreesToRadians(kFieldOfViewDeg * 0.5));
	const double UnitsPerPixel = ViewHeightWorld / FMath::Max(ViewportSize.Y, 1.0);

	const FRotationMatrix Frame(FRotator(Pitch, Yaw, 0.0));
	const FVector Right = Frame.GetUnitAxis(EAxis::Y);
	const FVector Up = Frame.GetUnitAxis(EAxis::Z);
	FocusPoint += (-Right * DeltaX + Up * DeltaY) * UnitsPerPixel;
	UpdateCameraTransform();
}

void AAerotelicCameraPawn::Zoom(double Factor)
{
	Distance = FMath::Clamp(Distance * Factor, kMinDistance, kMaxDistance);
	UpdateCameraTransform();
}

void AAerotelicCameraPawn::FrameBox(const FBox& Box)
{
	if (!Box.IsValid)
	{
		return;
	}
	FocusPoint = Box.GetCenter();
	const double Radius = FMath::Max(Box.GetExtent().Size(), 1e-3);
	// Fit the bounding sphere inside the vertical field of view, with a
	// little margin.
	Distance = FMath::Clamp(Radius / FMath::Sin(FMath::DegreesToRadians(kFieldOfViewDeg * 0.5)) * 1.1, kMinDistance, kMaxDistance);
	UpdateCameraTransform();
}

void AAerotelicCameraPawn::FrameAll()
{
	FBox All(ForceInit);
	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		for (const TWeakObjectPtr<AAerotelicEntityActor>& Entity : Doc->GetEntities())
		{
			if (Entity.IsValid())
			{
				const FBox Box = Entity->GetVisibleWorldBounds();
				UE_LOG(LogAerotelicCamera, Verbose, TEXT("FrameAll: '%s' bounds %s"), *Entity->EntityName, Box.IsValid ? *Box.ToString() : TEXT("(none)"));
				All += Box;
			}
		}
	}
	UE_LOG(LogAerotelicCamera, Log, TEXT("FrameAll: scene bounds %s"), All.IsValid ? *All.ToString() : TEXT("(none)"));
	if (All.IsValid)
	{
		FrameBox(All);
	}
	else
	{
		FocusPoint = FVector::ZeroVector;
		Distance = 30.0;
		UpdateCameraTransform();
	}
}

void AAerotelicCameraPawn::SetPresetView(EAerotelicViewPreset Preset)
{
	// occ_viewer.py gives the eye direction (focus -> eye); the camera's
	// forward is its negation. iso: eye at (-0.943, -0.333, 0.2).
	FVector Forward;
	switch (Preset)
	{
	case EAerotelicViewPreset::Top:   Forward = FVector(0.0, 0.0, -1.0); break;
	case EAerotelicViewPreset::Front: Forward = FVector(1.0, 0.0, 0.0); break;
	case EAerotelicViewPreset::Side:  Forward = FVector(0.0, 1.0, 0.0); break;
	case EAerotelicViewPreset::Isometric:
	default:                          Forward = FVector(2.8284 / 3.0, 1.0 / 3.0, -0.2).GetSafeNormal(); break;
	}
	const FRotator R = Forward.Rotation();
	Yaw = R.Yaw;
	Pitch = FMath::Clamp(R.Pitch, -89.0, 89.0);
	UE_LOG(LogAerotelicCamera, Log, TEXT("camera: preset view -> yaw %.1f pitch %.1f"), Yaw, Pitch);
	FrameAll();
}

bool AAerotelicCameraPawn::RebuildSelectedEntity()
{
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	AAerotelicEntityActor* Selected = Doc ? Doc->GetSelectedEntity() : nullptr;
	if (Selected == nullptr)
	{
		return false;
	}
	Selected->RefreshProxy();
	return true;
}

void AAerotelicCameraPawn::FrameSelectedOrAll()
{
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	AAerotelicEntityActor* Selected = Doc ? Doc->GetSelectedEntity() : nullptr;
	if (Selected != nullptr)
	{
		const FBox Box = Selected->GetVisibleWorldBounds();
		if (Box.IsValid)
		{
			FrameBox(Box);
			return;
		}
	}
	FrameAll();
}

void AAerotelicCameraPawn::ClickSelect(APlayerController* PC)
{
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	if (Doc == nullptr)
	{
		return;
	}
	FHitResult Hit;
	AAerotelicEntityActor* HitEntity = nullptr;
	const bool bHit = PC->GetHitResultUnderCursor(ECC_Visibility, true, Hit);
	if (bHit)
	{
		HitEntity = Cast<AAerotelicEntityActor>(Hit.GetActor());
	}
	{
		float MX = 0.f, MY = 0.f;
		PC->GetMousePosition(MX, MY);
		FVector O, D;
		PC->DeprojectScreenPositionToWorld(MX, MY, O, D);
		const ULocalPlayer* LP = PC->GetLocalPlayer();
		UE_LOG(LogAerotelicCamera, Verbose, TEXT("camera: click at (%.0f, %.0f) view origin (%.3f, %.3f) size (%.3f, %.3f) ray from %s dir %s -> %s"),
			MX, MY, LP ? LP->Origin.X : -1.0, LP ? LP->Origin.Y : -1.0, LP ? LP->Size.X : -1.0, LP ? LP->Size.Y : -1.0,
			*O.ToString(), *D.ToString(), HitEntity ? *HitEntity->EntityName : (bHit ? *GetNameSafe(Hit.GetActor()) : TEXT("(nothing)")));
	}
	// Clicking empty space clears the selection, as in any CAD view.
	Doc->SetSelectedEntity(HitEntity);
}

bool AAerotelicCameraPawn::TryBeginSectionDrag(APlayerController* PC)
{
	AAerotelicSectionGizmo* Gizmo = AAerotelicSectionGizmo::FindInGameWorld();
	if (Gizmo == nullptr)
	{
		return false;
	}
	float MX = 0.f, MY = 0.f;
	FVector O, D;
	if (!PC->GetMousePosition(MX, MY) || !Gizmo->IsCursorOnHandle(PC, MX, MY) || !PC->DeprojectScreenPositionToWorld(MX, MY, O, D))
	{
		return false;
	}
	double T = 0.0;
	if (!Gizmo->DragParameter(O, D, T))
	{
		return false;
	}
	Gizmo->BeginDrag(T);
	UE_LOG(LogAerotelicCamera, Verbose, TEXT("camera: section drag start, t0 %.3f"), T);
	return true;
}

void AAerotelicCameraPawn::ContinueSectionDrag(APlayerController* PC)
{
	AAerotelicSectionGizmo* Gizmo = AAerotelicSectionGizmo::FindInGameWorld();
	float MX = 0.f, MY = 0.f;
	FVector O, D;
	if (Gizmo == nullptr || !PC->GetMousePosition(MX, MY) || !PC->DeprojectScreenPositionToWorld(MX, MY, O, D))
	{
		return;
	}
	double T = 0.0;
	if (Gizmo->DragParameter(O, D, T))
	{
		Gizmo->DragTo(T);
	}
}

void AAerotelicCameraPawn::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (bPendingFrameAll)
	{
		bPendingFrameAll = false;
		FrameAll();
	}

	APlayerController* PC = Cast<APlayerController>(GetController());
	if (PC == nullptr)
	{
		return;
	}

	// Periodic frame-time diagnostic (every ~5 s) -- cheap, and the only way
	// to see the real frame rate of a packaged run from its log.
	{
		static double LastReport = 0.0;
		static int32 FramesSinceReport = 0;
		++FramesSinceReport;
		const double Now = FPlatformTime::Seconds();
		if (Now - LastReport > 5.0)
		{
			if (LastReport > 0.0)
			{
				UE_LOG(LogAerotelicCamera, Verbose, TEXT("camera: %.1f fps (dt %.1f ms)"), FramesSinceReport / (Now - LastReport), DeltaSeconds * 1000.0);
			}
			LastReport = Now;
			FramesSinceReport = 0;
		}
	}

	// Keyboard shortcuts.
	if (PC->WasInputKeyJustPressed(EKeys::F))
	{
		UE_LOG(LogAerotelicCamera, Log, TEXT("camera: F pressed -> frame selected"));
		FrameSelectedOrAll();
	}
	if (PC->WasInputKeyJustPressed(EKeys::Home))
	{
		UE_LOG(LogAerotelicCamera, Log, TEXT("camera: Home pressed -> frame all"));
		FrameAll();
	}

	// Wheel zoom. These arrive as press events whenever the wheel turns over
	// the 3D view (Slate routes them by hover, so no click-to-focus needed).
	if (PC->WasInputKeyJustPressed(EKeys::MouseScrollUp))
	{
		UE_LOG(LogAerotelicCamera, Verbose, TEXT("camera: wheel up"));
		Zoom(kZoomStep);
	}
	if (PC->WasInputKeyJustPressed(EKeys::MouseScrollDown))
	{
		UE_LOG(LogAerotelicCamera, Verbose, TEXT("camera: wheel down"));
		Zoom(1.0 / kZoomStep);
	}

	// Mouse drags. Button state only reaches the player controller for
	// presses that began over the 3D view (presses over the shell or a
	// dialog are consumed by Slate), which is exactly the desired scoping.
	const bool bLeft = PC->IsInputKeyDown(EKeys::LeftMouseButton);
	const bool bRight = PC->IsInputKeyDown(EKeys::RightMouseButton);
	const bool bMiddle = PC->IsInputKeyDown(EKeys::MiddleMouseButton);
	const bool bAnyButton = bLeft || bRight || bMiddle;

	float MouseX = 0.f, MouseY = 0.f;
	const bool bHaveMouse = PC->GetMousePosition(MouseX, MouseY);
	const FVector2D Mouse(MouseX, MouseY);

	if (bAnyButton && bHaveMouse)
	{
		if (!bDragging)
		{
			bDragging = true;
			bDragIsLeftButton = bLeft && !bRight && !bMiddle;
			// A left press on the section arrow drags the plane (the press
			// is swallowed: it neither orbits nor selects).
			bDraggingSection = bDragIsLeftButton && TryBeginSectionDrag(PC);
			LastMouse = Mouse;
			DragTravel = 0.0;
			UE_LOG(LogAerotelicCamera, Verbose, TEXT("camera: drag start L=%d R=%d M=%d at (%.0f, %.0f)"), bLeft, bRight, bMiddle, Mouse.X, Mouse.Y);
		}
		else
		{
			const FVector2D Delta = Mouse - LastMouse;
			LastMouse = Mouse;
			DragTravel += FMath::Abs(Delta.X) + FMath::Abs(Delta.Y);
			if (bDraggingSection)
			{
				ContinueSectionDrag(PC);
			}
			else if (bLeft && !bRight && !bMiddle)
			{
				Orbit(Delta.X, Delta.Y);
			}
			else
			{
				int32 SizeX = 0, SizeY = 0;
				PC->GetViewportSize(SizeX, SizeY);
				// The 3D view is confined to the frame's centre (see
				// AerotelicPlayerController), so pan speed follows the view
				// rect's height, not the whole window's.
				double ViewHeight = SizeY;
				if (const ULocalPlayer* LP = PC->GetLocalPlayer())
				{
					ViewHeight *= FMath::Max(LP->Size.Y, 0.05);
				}
				Pan(Delta.X, Delta.Y, FVector2D(SizeX, ViewHeight));
			}
		}
	}
	else if (bDragging)
	{
		bDragging = false;
		UE_LOG(LogAerotelicCamera, Verbose, TEXT("camera: drag end, travel %.0f px"), DragTravel);
		if (bDraggingSection)
		{
			bDraggingSection = false;
		}
		else if (bDragIsLeftButton && DragTravel < kClickTravelPixels)
		{
			ClickSelect(PC);
		}
	}
}
