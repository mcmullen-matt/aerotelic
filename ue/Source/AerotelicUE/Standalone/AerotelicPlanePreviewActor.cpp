#include "AerotelicPlanePreviewActor.h"

#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "Misc/PackageName.h"
#include "ProceduralMeshComponent.h"
#include "UObject/ConstructorHelpers.h"

#include "AerotelicSectionGizmo.h"
#include "Entity/AerotelicEntityActor.h"

#include "aerotelic/body_structure.hpp"

namespace
{
	constexpr float kBasicShapeSize = 100.0f;
	// The rectangle reaches this far past the body's bounds.
	constexpr double kPad = 0.06;
	// Outlines are drawn for up to this many planes; beyond, the quads alone.
	constexpr int32 kMaxOutlinedPlanes = 64;
	const FLinearColor kActive(1.0f, 0.7f, 0.1f, 1.0f);
	const FLinearColor kOther(0.75f, 0.75f, 0.8f, 1.0f);

	UMaterialInterface* LoadMaterial(const FString& Package)
	{
		return LoadObject<UMaterialInterface>(nullptr, *FString::Printf(TEXT("%s.%s"), *Package, *FPackageName::GetLongPackageAssetName(Package)));
	}
}

FString AAerotelicPlanePreviewActor::MaterialPackage()
{
	return TEXT("/Game/Materials/M_AerotelicPlanePreview");
}

AAerotelicPlanePreviewActor::AAerotelicPlanePreviewActor()
{
	PrimaryActorTick.bCanEverTick = true;
	SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("SceneRoot"));
	SceneRoot->SetMobility(EComponentMobility::Movable);
	RootComponent = SceneRoot;

	Quads = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("Quads"));
	Quads->SetupAttachment(SceneRoot);
	Quads->SetMobility(EComponentMobility::Movable);
	Quads->bUseAsyncCooking = false;
	Quads->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Quads->SetCastShadow(false);

	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeFinder(TEXT("/Engine/BasicShapes/Cube.Cube"));
	CubeMesh = CubeFinder.Succeeded() ? CubeFinder.Object : nullptr;
}

void AAerotelicPlanePreviewActor::BeginPlay()
{
	Super::BeginPlay();
	if (UMaterialInterface* BarMat = LoadMaterial(AAerotelicSectionGizmo::FrameMaterialPackage()))
	{
		BarMID = UMaterialInstanceDynamic::Create(BarMat, this);
		BarMID->SetVectorParameterValue(TEXT("Color"), kActive);
	}
	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		PreviewHandle = Doc->OnStructurePreviewChanged.AddUObject(this, &AAerotelicPlanePreviewActor::OnPreviewChanged);
		Preview = Doc->GetStructurePreview();
	}
	GeometryHandle = AAerotelicEntityActor::OnGeometryChanged.AddUObject(this, &AAerotelicPlanePreviewActor::OnGeometryChanged);
	Rebuild();
}

void AAerotelicPlanePreviewActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		Doc->OnStructurePreviewChanged.Remove(PreviewHandle);
	}
	AAerotelicEntityActor::OnGeometryChanged.Remove(GeometryHandle);
	Super::EndPlay(EndPlayReason);
}

void AAerotelicPlanePreviewActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	// A dragged or re-placed body moves its planes with it; the bounds are
	// cheap to re-read every frame while a preview is up.
	if (Preview.IsActive())
	{
		const FBox Box = Preview.Entity->GetPlacedBounds(false);
		if (Box.IsValid != LastBounds.IsValid || (Box.IsValid && !(Box.Min.Equals(LastBounds.Min, 1e-6) && Box.Max.Equals(LastBounds.Max, 1e-6))))
		{
			Rebuild();
		}
	}
}

void AAerotelicPlanePreviewActor::OnPreviewChanged()
{
	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		Preview = Doc->GetStructurePreview();
	}
	Rebuild();
}

void AAerotelicPlanePreviewActor::OnGeometryChanged(AAerotelicEntityActor* Entity)
{
	if (Preview.IsActive() && Preview.Entity.Get() == Entity)
	{
		Rebuild();
	}
}

UStaticMeshComponent* AAerotelicPlanePreviewActor::Bar(int32 Index)
{
	while (Bars.Num() <= Index)
	{
		UStaticMeshComponent* C = NewObject<UStaticMeshComponent>(this, *FString::Printf(TEXT("Bar%d"), Bars.Num()));
		C->SetMobility(EComponentMobility::Movable);
		C->SetStaticMesh(CubeMesh);
		C->SetCastShadow(false);
		C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		if (BarMID != nullptr) C->SetMaterial(0, BarMID);
		C->SetupAttachment(SceneRoot);
		C->RegisterComponent();
		Bars.Add(C);
	}
	return Bars[Index];
}

void AAerotelicPlanePreviewActor::PlaceBar(UStaticMeshComponent* B, const FVector& A, const FVector& End, double Thickness)
{
	const FVector D = End - A;
	const double Len = D.Size();
	if (Len <= 0.0)
	{
		B->SetVisibility(false);
		return;
	}
	B->SetWorldLocation((A + End) * 0.5);
	B->SetWorldRotation(FRotationMatrix::MakeFromX(D / Len).Rotator());
	B->SetWorldScale3D(FVector(Len / kBasicShapeSize, Thickness / kBasicShapeSize, Thickness / kBasicShapeSize));
	B->SetVisibility(true);
}

void AAerotelicPlanePreviewActor::Rebuild()
{
	Quads->ClearAllMeshSections();
	for (UStaticMeshComponent* B : Bars) B->SetVisibility(false);
	if (!Preview.IsActive())
	{
		LastBounds = FBox(ForceInit);
		return;
	}
	AAerotelicEntityActor* Entity = Preview.Entity.Get();
	const FBox Bounds = Entity->GetPlacedBounds(false);
	LastBounds = Bounds;
	if (!Bounds.IsValid)
	{
		return;
	}
	const double Diag = Bounds.GetSize().Size();
	const FBox Padded = Bounds.ExpandBy(kPad * Diag);
	aerotelic::Box3 Box;
	Box.Add(aerotelic::Vec3{Padded.Min.X, Padded.Min.Y, Padded.Min.Z});
	Box.Add(aerotelic::Vec3{Padded.Max.X, Padded.Max.Y, Padded.Max.Z});
	aerotelic::Box3 MirrorBox;
	MirrorBox.Add(aerotelic::Vec3{Padded.Min.X, -Padded.Max.Y, Padded.Min.Z});
	MirrorBox.Add(aerotelic::Vec3{Padded.Max.X, -Padded.Min.Y, Padded.Max.Z});
	std::array<double, 9> R;
	aerotelic::Vec3 T;
	double Scale = 1.0;
	Entity->GetPlacementMatrix(R, T, Scale);
	const bool bMirrored = Entity->IsMirrored();
	const double BarThickness = FMath::Clamp(0.003 * Diag, 0.002, 0.5);

	UMaterialInterface* QuadMat = LoadMaterial(MaterialPackage());
	if (QuadMat == nullptr) QuadMat = LoadMaterial(AAerotelicSectionGizmo::HandleMaterialPackage());

	int32 Section = 0;
	int32 BarIndex = 0;
	for (size_t n = 0; n < Preview.Networks.size(); ++n)
	{
		const aerotelic::PlaneNetwork& Net = Preview.Networks[n];
		const bool bActive = static_cast<int32>(n) == Preview.ActiveIndex;
		while (NetworkMIDs.Num() <= static_cast<int32>(n))
		{
			NetworkMIDs.Add(QuadMat != nullptr ? UMaterialInstanceDynamic::Create(QuadMat, this) : nullptr);
		}
		UMaterialInstanceDynamic* MID = NetworkMIDs[static_cast<int32>(n)];
		if (MID != nullptr)
		{
			MID->SetVectorParameterValue(TEXT("Color"), bActive ? kActive : kOther);
			MID->SetScalarParameterValue(TEXT("Opacity"), bActive ? 0.30f : 0.10f);
		}
		if (!aerotelic::PlaneNetworkProblem(Net).empty()) continue;
		for (int k = 0; k < Net.count; ++k)
		{
			const aerotelic::Plane Local = aerotelic::NetworkPlane(Net, k);
			for (int Half = 0; Half < (bMirrored ? 2 : 1); ++Half)
			{
				aerotelic::Plane P = aerotelic::PlacedPlane(Local, R, T, Scale);
				if (Half == 1) P = aerotelic::MirrorPlaneY(P);
				std::array<aerotelic::Vec3, 4> C;
				if (!aerotelic::PlaneBoxRectangle(P, Half == 0 ? Box : MirrorBox, &C)) continue;
				TArray<FVector> V;
				for (const aerotelic::Vec3& Q : C) V.Add(FVector(Q.x, Q.y, Q.z));
				// Both windings, so the quad shows from either side.
				const TArray<int32> Tri = { 0, 1, 2, 0, 2, 3, 0, 2, 1, 0, 3, 2 };
				TArray<FVector> N;
				const FVector Normal(P.normal.x, P.normal.y, P.normal.z);
				for (int32 i = 0; i < 4; ++i) N.Add(Normal);
				Quads->CreateMeshSection(Section, V, Tri, N, TArray<FVector2D>(), TArray<FColor>(), TArray<FProcMeshTangent>(), false);
				if (MID != nullptr) Quads->SetMaterial(Section, MID);
				++Section;
				if (BarIndex / 4 < kMaxOutlinedPlanes)
				{
					for (int32 e = 0; e < 4; ++e)
					{
						PlaceBar(Bar(BarIndex++), V[e], V[(e + 1) % 4], bActive ? BarThickness : 0.5 * BarThickness);
					}
				}
			}
		}
	}
}
