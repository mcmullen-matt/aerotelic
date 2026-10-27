#include "AerotelicSceneViewActor.h"

#include "Components/SceneCaptureComponent2D.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h"

#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

#include "Branding/AerotelicTheme.h"

AAerotelicSceneViewActor::AAerotelicSceneViewActor()
{
	PrimaryActorTick.bCanEverTick = false;

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = Root;

	Capture = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("Capture"));
	Capture->SetupAttachment(Root);
	// The main view's tonemapped colour, so the picture matches it.
	Capture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
	// -AerotelicSceneViewEveryFrame: a developer comparison against the
	// demand-driven capture below.
	Capture->bCaptureEveryFrame = FParse::Param(FCommandLine::Get(), TEXT("AerotelicSceneViewEveryFrame"));
	Capture->bCaptureOnMovement = false;
	Capture->bAlwaysPersistRenderingState = true;
	Capture->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;
	Capture->FOVAngle = 60.0f;
	Capture->ShowFlags.SetAtmosphere(false);
	Capture->ShowFlags.SetFog(false);
	Capture->ShowFlags.SetMotionBlur(false);

	// The backdrop: the main view's inside-out grey sphere (see
	// AerotelicGameMode::SpawnBackdrop), seen by captures only. Its mesh
	// and material are loaded at BeginPlay, not here: this also runs for
	// the class default object at module start-up, before assets load.
	Backdrop = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Backdrop"));
	Backdrop->SetupAttachment(Root);
	Backdrop->SetMobility(EComponentMobility::Movable);
	Backdrop->SetCastShadow(false);
	Backdrop->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Backdrop->bAffectDistanceFieldLighting = false;
	Backdrop->SetVisibleInSceneCaptureOnly(true);
	// Absolute: the sphere stays on the world origin while the capture moves.
	Backdrop->SetUsingAbsoluteLocation(true);
	Backdrop->SetUsingAbsoluteRotation(true);
	Backdrop->SetUsingAbsoluteScale(true);
}

void AAerotelicSceneViewActor::BeginPlay()
{
	Super::BeginPlay();
	if (UStaticMesh* Sphere = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere")))
	{
		Backdrop->SetStaticMesh(Sphere);
		// Engine sphere is 100 units across; this makes it ~2 km across,
		// inside out (negative scale flips the faces inward).
		Backdrop->SetWorldScale3D(FVector(-2000.0, -2000.0, -2000.0));
	}
	static const TCHAR* UnlitCandidates[] = {
		TEXT("/Engine/EngineMaterials/EmissiveMeshMaterial.EmissiveMeshMaterial"),
		TEXT("/Engine/EngineDebugMaterials/LevelColorationUnlitMaterial.LevelColorationUnlitMaterial"),
	};
	for (const TCHAR* Path : UnlitCandidates)
	{
		UMaterialInterface* Candidate = LoadObject<UMaterialInterface>(nullptr, Path);
		FLinearColor Unused;
		if (Candidate != nullptr && Candidate->GetVectorParameterValue(FHashedMaterialParameterInfo(TEXT("Color")), Unused))
		{
			BackdropMaterial = UMaterialInstanceDynamic::Create(Candidate, this);
			Backdrop->SetMaterial(0, BackdropMaterial);
			break;
		}
	}
	ApplyTheme();
	ThemeHandle = AerotelicTheme::OnChanged().AddUObject(this, &AAerotelicSceneViewActor::ApplyTheme);
	Capture->ShowOnlyComponent(Backdrop);
}

void AAerotelicSceneViewActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	AerotelicTheme::OnChanged().Remove(ThemeHandle);
	Super::EndPlay(EndPlayReason);
}

void AAerotelicSceneViewActor::ApplyTheme()
{
	if (BackdropMaterial != nullptr)
	{
		if (FixedBackdrop.IsSet())
		{
			BackdropMaterial->SetVectorParameterValue(TEXT("Color"), FixedBackdrop.GetValue());
			return;
		}
		const FAerotelicPalette& P = AerotelicTheme::Current();
		const FLinearColor Mid = (P.ViewportBgTop + P.ViewportBgBot) * 0.5f;
		BackdropMaterial->SetVectorParameterValue(TEXT("Color"), FLinearColor(Mid.R + 0.06f, Mid.G + 0.06f, Mid.B + 0.07f));
	}
}

void AAerotelicSceneViewActor::SetBackdropColor(const FLinearColor& Color)
{
	FixedBackdrop = Color;
	ApplyTheme();
}

void AAerotelicSceneViewActor::SetShownActors(const TArray<AActor*>& Actors)
{
	Capture->ClearShowOnlyComponents();
	Capture->ShowOnlyComponent(Backdrop);
	for (AActor* A : Actors)
	{
		if (A != nullptr)
		{
			Capture->ShowOnlyActorComponents(A, true);
		}
	}
}

void AAerotelicSceneViewActor::SetRenderTarget(UTextureRenderTarget2D* Target)
{
	Capture->TextureTarget = Target;
}

void AAerotelicSceneViewActor::SetCamera(const FVector& Focus, double Distance, double Yaw, double Pitch, float FieldOfViewDeg)
{
	const FRotator Rotation(Pitch, Yaw, 0.0);
	const FVector Location = Focus - Rotation.Vector() * Distance;
	Capture->FOVAngle = FieldOfViewDeg;
	Capture->SetWorldLocationAndRotation(Location, Rotation);
}

void AAerotelicSceneViewActor::RequestCapture()
{
	if (Capture->TextureTarget != nullptr)
	{
		Capture->CaptureSceneDeferred();
	}
}
