#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "Entity/AerotelicDocumentSubsystem.h"

#include "AerotelicPlanePreviewActor.generated.h"

class UProceduralMeshComponent;
class UStaticMeshComponent;
class UMaterialInstanceDynamic;

// The Structure dialog's cutting planes in the 3D view: every plane of the
// body's networks as a translucent rectangle covering the body's bounds,
// with an outline, the network being edited amber and the others grey, the
// mirrored half's planes with it. Follows the document's structure preview
// (UAerotelicDocumentSubsystem::OnStructurePreviewChanged) and the body's
// geometry; shows nothing while no dialog is editing planes. Collides with
// nothing, so the entity pick passes through it.
UCLASS()
class AAerotelicPlanePreviewActor : public AActor
{
	GENERATED_BODY()

public:
	AAerotelicPlanePreviewActor();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

	// "/Game/Materials/M_AerotelicPlanePreview": unlit, two sided,
	// translucent, Color and Opacity parameters (authored by
	// AerotelicMaterialSetupCommandlet). Falls back to the gizmo's on-top
	// material (opaque) when missing.
	static FString MaterialPackage();

private:
	UPROPERTY()
	TObjectPtr<USceneComponent> SceneRoot;

	// One section per plane.
	UPROPERTY()
	TObjectPtr<UProceduralMeshComponent> Quads;

	// One material per network (its tone), reused across rebuilds.
	UPROPERTY()
	TArray<TObjectPtr<UMaterialInstanceDynamic>> NetworkMIDs;

	// The outlines: four thin bars per plane from a pool, hidden when unused.
	UPROPERTY()
	TArray<TObjectPtr<UStaticMeshComponent>> Bars;

	UPROPERTY()
	TObjectPtr<UMaterialInstanceDynamic> BarMID;

	UPROPERTY()
	TObjectPtr<UStaticMesh> CubeMesh;

	FAerotelicStructurePreview Preview;
	FBox LastBounds = FBox(ForceInit);
	FDelegateHandle PreviewHandle;
	FDelegateHandle GeometryHandle;

	void OnPreviewChanged();
	void OnGeometryChanged(AAerotelicEntityActor* Entity);
	void Rebuild();
	UStaticMeshComponent* Bar(int32 Index);
	void PlaceBar(UStaticMeshComponent* Bar, const FVector& A, const FVector& B, double Thickness);
};
