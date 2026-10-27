#include "AerotelicColoredMeshActor.h"

#include "ProceduralMeshComponent.h"
#include "Engine/Engine.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/PackageName.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

namespace
{
	UMaterialInterface* VertexColorBaseMaterial()
	{
		static TWeakObjectPtr<UMaterialInterface> Cached;
		if (Cached.IsValid())
		{
			return Cached.Get();
		}
		FString Package = AAerotelicColoredMeshActor::VertexColorMaterialPackage();
		// Developer checks of the colour path: -AerotelicEngineVertexColorMaterial
		// takes the engine's vertex colour material instead of ours;
		// -AerotelicResultSurfaceMaterial takes the bodies' own surface
		// material (a plain colour: shows whether a material reaches the
		// mesh at all).
		if (FParse::Param(FCommandLine::Get(), TEXT("AerotelicResultSurfaceMaterial"))) Package = TEXT("/Game/Materials/M_AerotelicSurface");
		UMaterialInterface* M = FParse::Param(FCommandLine::Get(), TEXT("AerotelicEngineVertexColorMaterial")) ? nullptr
			: LoadObject<UMaterialInterface>(nullptr, *FString::Printf(TEXT("%s.%s"), *Package, *FPackageName::GetLongPackageAssetName(Package)));
		if (M == nullptr)
		{
			UE_LOG(LogTemp, Warning, TEXT("AerotelicColoredMeshActor: %s not found (run setup_materials.bat); coloured results use the engine's unlit vertex colour material."), *Package);
			M = GEngine != nullptr ? GEngine->VertexColorMaterial.Get() : nullptr;
		}
		Cached = M;
		return M;
	}
}

AAerotelicColoredMeshActor::AAerotelicColoredMeshActor()
{
	PrimaryActorTick.bCanEverTick = false;
	Mesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Mesh"));
	RootComponent = Mesh;
	Mesh->SetMobility(EComponentMobility::Movable);
	Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Mesh->SetCastShadow(true);
	Mesh->bUseAsyncCooking = false;
	Mesh->SetVisibleInSceneCaptureOnly(true);
}

FString AAerotelicColoredMeshActor::VertexColorMaterialPackage()
{
	return TEXT("/Game/Materials/M_AerotelicVertexColor");
}

FColor AAerotelicColoredMeshActor::VertexColorFromSRGB(float R, float G, float B)
{
	// FColor -> FLinearColor decodes sRGB; ToFColor(false) stores the linear
	// value as the 8-bit the vertex stream carries.
	const FColor Srgb(
		static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(R * 255.0f), 0, 255)),
		static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(G * 255.0f), 0, 255)),
		static_cast<uint8>(FMath::Clamp(FMath::RoundToInt(B * 255.0f), 0, 255)));
	return FLinearColor(Srgb).ToFColor(false);
}

void AAerotelicColoredMeshActor::SetMesh(const TArray<FVector>& InVertices, const TArray<int32>& Triangles, const TArray<FVector>& InNormals, const TArray<FColor>& Colors)
{
	Vertices = InVertices;
	Normals = InNormals;
	if (Normals.Num() != Vertices.Num())
	{
		Normals.SetNumZeroed(Vertices.Num());
	}
	TArray<FColor> Cols = Colors;
	if (Cols.Num() != Vertices.Num())
	{
		Cols.Init(FColor::White, Vertices.Num());
	}
	Mesh->ClearAllMeshSections();
	if (Vertices.Num() == 0 || Triangles.Num() < 3)
	{
		return;
	}
	UE_LOG(LogTemp, Verbose, TEXT("AerotelicColoredMeshActor: %d vertices, %d triangles, %d colours given (first %s, middle %s, last %s)"),
		Vertices.Num(), Triangles.Num() / 3, Colors.Num(),
		*Cols[0].ToString(), *Cols[Cols.Num() / 2].ToString(), *Cols.Last().ToString());
	Mesh->CreateMeshSection(0, Vertices, Triangles, Normals, TArray<FVector2D>(), Cols, TArray<FProcMeshTangent>(), false);
	if (Material == nullptr)
	{
		if (UMaterialInterface* Base = VertexColorBaseMaterial())
		{
			Material = UMaterialInstanceDynamic::Create(Base, this);
		}
	}
	if (Material != nullptr)
	{
		if (FParse::Param(FCommandLine::Get(), TEXT("AerotelicResultSurfaceMaterial")))
		{
			Material->SetVectorParameterValue(TEXT("Color"), FLinearColor(0.9f, 0.1f, 0.1f)); // the check's plain colour
		}
		Mesh->SetMaterial(0, Material);
	}
}

void AAerotelicColoredMeshActor::SetColors(const TArray<FColor>& Colors)
{
	if (Vertices.Num() == 0 || Colors.Num() != Vertices.Num())
	{
		return;
	}
	Mesh->UpdateMeshSection(0, Vertices, Normals, TArray<FVector2D>(), Colors, TArray<FProcMeshTangent>());
}

void AAerotelicColoredMeshActor::Clear()
{
	Vertices.Reset();
	Normals.Reset();
	Mesh->ClearAllMeshSections();
}

FBox AAerotelicColoredMeshActor::GetMeshBounds() const
{
	FBox Box(ForceInit);
	for (const FVector& V : Vertices)
	{
		Box += V;
	}
	return Box;
}

void AAerotelicColoredMeshActor::SetVisibleInCapturesOnly(bool bCapturesOnly)
{
	Mesh->SetVisibleInSceneCaptureOnly(bCapturesOnly);
}
