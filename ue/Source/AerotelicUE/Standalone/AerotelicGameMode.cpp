#include "AerotelicGameMode.h"

#include "Engine/World.h"
#include "Engine/DirectionalLight.h"
#include "Engine/SkyLight.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/SkyLightComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

#include "AerotelicCameraPawn.h"
#include "AerotelicGridGround.h"
#include "AerotelicPlayerController.h"
#include "AerotelicSectionGizmo.h"
#include "Branding/AerotelicTheme.h"
#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"

AAerotelicGameMode::AAerotelicGameMode()
{
	DefaultPawnClass = AAerotelicCameraPawn::StaticClass();
	PlayerControllerClass = AAerotelicPlayerController::StaticClass();
}

void AAerotelicGameMode::BeginPlay()
{
	Super::BeginPlay();
	SpawnLighting();
	SpawnBackdrop();
	SpawnGround();
	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		ListChangedHandle = Doc->OnEntityListChanged.AddUObject(this, &AAerotelicGameMode::FitGroundToScene);
	}
	OpenProjectFromCommandLine();
}

void AAerotelicGameMode::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		Doc->OnEntityListChanged.Remove(ListChangedHandle);
	}
	AerotelicTheme::OnChanged().Remove(ThemeHandle);
	Super::EndPlay(EndPlayReason);
}

void AAerotelicGameMode::SpawnGround()
{
	UWorld* World = GetWorld();
	if (World == nullptr)
	{
		return;
	}
	FActorSpawnParameters Params;
	Params.ObjectFlags |= RF_Transient;
	Ground = World->SpawnActor<AAerotelicGridGround>(FVector::ZeroVector, FRotator::ZeroRotator, Params);
	FitGroundToScene();
	// The section plane's handle; hidden until a Section axis is chosen.
	World->SpawnActor<AAerotelicSectionGizmo>(FVector::ZeroVector, FRotator::ZeroRotator, Params);
}

void AAerotelicGameMode::FitGroundToScene()
{
	if (Ground == nullptr)
	{
		return;
	}
	FBox Content(ForceInit);
	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		for (const TWeakObjectPtr<AAerotelicEntityActor>& Weak : Doc->GetEntities())
		{
			if (Weak.IsValid())
			{
				const FBox Box = Weak->GetVisibleWorldBounds();
				if (Box.IsValid)
				{
					Content += Box;
				}
			}
		}
	}
	Ground->FitToContent(Content);
}

void AAerotelicGameMode::SpawnLighting()
{
	UWorld* World = GetWorld();
	if (World == nullptr)
	{
		return;
	}

	FActorSpawnParameters Params;
	Params.ObjectFlags |= RF_Transient;

	// Key light. Everything here is Movable -- nothing in this scene is ever
	// baked. ForwardShadingPriority makes this unambiguously the "main"
	// directional light (otherwise the renderer warns on screen that two
	// directional lights are competing for that role).
	if (ADirectionalLight* Sun = World->SpawnActor<ADirectionalLight>(FVector::ZeroVector, FRotator(-42.0f, 35.0f, 0.0f), Params))
	{
		if (UDirectionalLightComponent* Light = Cast<UDirectionalLightComponent>(Sun->GetLightComponent()))
		{
			Light->ForwardShadingPriority = 1;
			Light->SetMobility(EComponentMobility::Movable);
			Light->SetIntensity(3.0f);
			Light->SetLightColor(FLinearColor(1.0f, 0.97f, 0.92f));
			Light->SetAtmosphereSunLight(false);
			Light->SetCastShadows(true);
		}
	}

	// Fill light from the opposite side, lower, so undersides and shadowed
	// faces still read.
	if (ADirectionalLight* Fill = World->SpawnActor<ADirectionalLight>(FVector::ZeroVector, FRotator(-15.0f, -140.0f, 0.0f), Params))
	{
		if (UDirectionalLightComponent* Light = Cast<UDirectionalLightComponent>(Fill->GetLightComponent()))
		{
			Light->ForwardShadingPriority = 0;
			Light->SetMobility(EComponentMobility::Movable);
			Light->SetIntensity(0.8f);
			Light->SetLightColor(FLinearColor(0.85f, 0.9f, 1.0f));
			Light->SetAtmosphereSunLight(false);
			Light->SetCastShadows(false);
		}
	}

	// Underside fill, from below, so a model viewed from beneath is not a
	// silhouette. (No sky light: with no sky atmosphere in the scene a
	// real-time-capture sky light has nothing to capture and warns on screen.)
	if (ADirectionalLight* Under = World->SpawnActor<ADirectionalLight>(FVector::ZeroVector, FRotator(35.0f, 100.0f, 0.0f), Params))
	{
		if (UDirectionalLightComponent* Light = Cast<UDirectionalLightComponent>(Under->GetLightComponent()))
		{
			Light->ForwardShadingPriority = 0;
			Light->SetMobility(EComponentMobility::Movable);
			Light->SetIntensity(0.5f);
			Light->SetLightColor(FLinearColor(0.9f, 0.9f, 0.9f));
			Light->SetAtmosphereSunLight(false);
			Light->SetCastShadows(false);
		}
	}
}

void AAerotelicGameMode::SpawnBackdrop()
{
	// A huge inside-out sphere (negative scale flips the faces inward) in a
	// flat neutral grey: a plain backdrop instead of a black void or a
	// landscape/sky-sphere template. The material must be *unlit*: the
	// sphere's normals point outward, so from the inside a lit material only
	// ever sees the lights from behind and renders black. It casts no shadows
	// (it would otherwise shadow everything inside it) and has no collision
	// (so click-to-select ignores it).
	UWorld* World = GetWorld();
	if (World == nullptr)
	{
		return;
	}
	UStaticMesh* SphereMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere"));
	if (SphereMesh == nullptr)
	{
		return;
	}
	// Engine-provided unlit materials exposing a "Color" vector parameter;
	// the first one that loads and really has the parameter wins.
	static const TCHAR* UnlitCandidates[] = {
		TEXT("/Engine/EngineMaterials/EmissiveMeshMaterial.EmissiveMeshMaterial"),
		TEXT("/Engine/EngineDebugMaterials/LevelColorationUnlitMaterial.LevelColorationUnlitMaterial"),
	};
	UMaterialInterface* BaseMaterial = nullptr;
	for (const TCHAR* Path : UnlitCandidates)
	{
		UMaterialInterface* Candidate = LoadObject<UMaterialInterface>(nullptr, Path);
		FLinearColor Unused;
		if (Candidate != nullptr && Candidate->GetVectorParameterValue(FHashedMaterialParameterInfo(TEXT("Color")), Unused))
		{
			BaseMaterial = Candidate;
			break;
		}
	}
	if (BaseMaterial == nullptr)
	{
		UE_LOG(LogTemp, Warning, TEXT("AerotelicGameMode: no unlit backdrop material found; the 3D view background stays black."));
	}

	FActorSpawnParameters Params;
	Params.ObjectFlags |= RF_Transient;
	AStaticMeshActor* Backdrop = World->SpawnActor<AStaticMeshActor>(FVector::ZeroVector, FRotator::ZeroRotator, Params);
	if (Backdrop == nullptr)
	{
		return;
	}
	UStaticMeshComponent* Mesh = Backdrop->GetStaticMeshComponent();
	Mesh->SetMobility(EComponentMobility::Movable);
	Mesh->SetStaticMesh(SphereMesh);
	// Engine sphere is 100 units across; this makes it ~2 km across.
	Mesh->SetRelativeScale3D(FVector(-2000.0, -2000.0, -2000.0));
	Mesh->SetCastShadow(false);
	Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Mesh->bAffectDistanceFieldLighting = false;
	if (BaseMaterial != nullptr)
	{
		UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(BaseMaterial, Backdrop);
		BackdropMID = Mid;
		Mesh->SetMaterial(0, Mid);
		ApplyThemeToBackdrop();
		ThemeHandle = AerotelicTheme::OnChanged().AddUObject(this, &AAerotelicGameMode::ApplyThemeToBackdrop);
	}
}

void AAerotelicGameMode::ApplyThemeToBackdrop()
{
	// The theme's viewport background (themes.py viewport_bg_top/bot, a
	// vertical gradient in the Qt viewer): the sphere takes the midpoint,
	// lifted a little so the models keep reading against it.
	if (BackdropMID != nullptr)
	{
		const FAerotelicPalette& P = AerotelicTheme::Current();
		const FLinearColor Mid = (P.ViewportBgTop + P.ViewportBgBot) * 0.5f;
		BackdropMID->SetVectorParameterValue(TEXT("Color"), FLinearColor(Mid.R + 0.06f, Mid.G + 0.06f, Mid.B + 0.07f));
	}
}

void AAerotelicGameMode::OpenProjectFromCommandLine()
{
	FString Path;
	if (!FParse::Value(FCommandLine::Get(), TEXT("AerotelicProject="), Path))
	{
		return;
	}
	Path = Path.TrimQuotes();
	if (Path.IsEmpty())
	{
		return;
	}
	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		// The scene is empty at startup, so this never asks for confirmation.
		Doc->OpenProjectPath(Path);
	}
}
