#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "AerotelicGridGround.generated.h"

class UInstancedStaticMeshComponent;
class UMaterialInstanceDynamic;
class UMaterialInterface;
class UStaticMeshComponent;

// The ground plane: the grid lattice from the GridCube-UE5.8 reference
// project (C:\Users\Matt\unreal_tutorial\GridCube-UE5.8), ported here.
//
// Structure is the reference's: thin cube instances in one
// UInstancedStaticMeshComponent for the lines, plus a solid base plane just
// below them, both using only engine basic shapes and BasicShapeMaterial's
// "Color" parameter -- no project assets.
//
// What differs is scale. The reference is a game-scale world of 100-unit
// cells; here one model unit is one Unreal unit and an aircraft is a few
// tens of units across (the P-38's semi-span is 7.9), so the defaults are
// the reference's divided by a hundred: 1-unit cells, 0.03 line width,
// 0.02 line height. FitToContent() then resizes the grid to whatever the
// scene actually holds, so a model of any size lands on a sensible grid.
//
// Neither component collides: the 3D view picks entities with a Visibility
// trace and a blocking floor would swallow every click meant for empty
// space (which is what clears the selection).
UCLASS()
class AAerotelicGridGround : public AActor
{
	GENERATED_BODY()

public:
	AAerotelicGridGround();

	virtual void OnConstruction(const FTransform& Transform) override;

	// Resizes the grid to run from -L to +L in X and Y, symmetric about the
	// origin, with L = 1.2 x the content's largest |X| or |Y|; the cell
	// size is L over a whole number of cells so the lines fall exactly on 0
	// and +/-L. An empty scene keeps a 20 x 20 unit grid. Call after the
	// scene changes.
	void FitToContent(const FBox& Content);

	// Top of the lattice, for anything that wants to sit on the ground.
	float GetSurfaceHeight() const { return LineHeight; }

	UPROPERTY(EditAnywhere, Category = "Grid")
	int32 CellsX = 20;

	UPROPERTY(EditAnywhere, Category = "Grid")
	int32 CellsY = 20;

	UPROPERTY(EditAnywhere, Category = "Grid")
	float CellSize = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Grid")
	float LineWidth = 0.03f;

	UPROPERTY(EditAnywhere, Category = "Grid")
	float LineHeight = 0.02f;

	UPROPERTY(EditAnywhere, Category = "Grid")
	bool bShowBasePlane = true;

	// The reference project's cyan lines over a near-black base.
	UPROPERTY(EditAnywhere, Category = "Grid")
	FLinearColor LineColor = FLinearColor(0.05f, 0.65f, 1.0f, 1.0f);

	UPROPERTY(EditAnywhere, Category = "Grid")
	FLinearColor BaseColor = FLinearColor(0.02f, 0.02f, 0.025f, 1.0f);

private:
	UPROPERTY()
	TObjectPtr<USceneComponent> SceneRoot;

	UPROPERTY()
	TObjectPtr<UInstancedStaticMeshComponent> GridLines;

	UPROPERTY()
	TObjectPtr<UStaticMeshComponent> BasePlane;

	UPROPERTY()
	TObjectPtr<UMaterialInterface> ShapeMaterial;

	UPROPERTY()
	TObjectPtr<UMaterialInstanceDynamic> LineMID;

	UPROPERTY()
	TObjectPtr<UMaterialInstanceDynamic> BaseMID;

	void RebuildGrid();
	void ApplyMaterials();
};
