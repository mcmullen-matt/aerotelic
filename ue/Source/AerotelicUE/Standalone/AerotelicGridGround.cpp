#include "AerotelicGridGround.h"

#include "Components/InstancedStaticMeshComponent.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "UObject/ConstructorHelpers.h"

namespace
{
	// Engine basic shapes are 100 x 100 (x 100) units, so scale = size / 100.
	constexpr float kBasicShapeSize = 100.0f;
	const FName kColorParam(TEXT("Color"));
	// Roughly how many cells the grid should span, whatever the content's
	// size -- the cell size is rounded to a 1/2/5 x 10^k step to suit.
	constexpr int32 kTargetCells = 20;
}

AAerotelicGridGround::AAerotelicGridGround()
{
	PrimaryActorTick.bCanEverTick = false;

	SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("SceneRoot"));
	SceneRoot->SetMobility(EComponentMobility::Movable);
	RootComponent = SceneRoot;

	GridLines = CreateDefaultSubobject<UInstancedStaticMeshComponent>(TEXT("GridLines"));
	GridLines->SetupAttachment(SceneRoot);
	GridLines->SetMobility(EComponentMobility::Movable);
	// No collision on either component: the 3D view selects entities with a
	// Visibility trace, and a blocking ground would absorb the clicks on
	// empty space that are meant to clear the selection.
	GridLines->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	GridLines->SetCastShadow(false);

	BasePlane = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BasePlane"));
	BasePlane->SetupAttachment(SceneRoot);
	BasePlane->SetMobility(EComponentMobility::Movable);
	BasePlane->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	BasePlane->SetCastShadow(false);

	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> PlaneMesh(TEXT("/Engine/BasicShapes/Plane.Plane"));
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> Material(TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));

	if (CubeMesh.Succeeded())  { GridLines->SetStaticMesh(CubeMesh.Object); }
	if (PlaneMesh.Succeeded()) { BasePlane->SetStaticMesh(PlaneMesh.Object); }
	if (Material.Succeeded())  { ShapeMaterial = Material.Object; }
}

void AAerotelicGridGround::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	RebuildGrid();
}

void AAerotelicGridGround::FitToContent(const FBox& Content)
{
	// The grid is symmetric about the origin and square: it runs from -L to
	// +L in both X and Y, with L = 1.2 x the content's largest |X| or |Y|
	// (its height plays no part in a floor's limits). An empty scene keeps
	// a 20 x 20 unit grid so the app does not open onto a degenerate floor.
	double Limit = 10.0;
	if (Content.IsValid)
	{
		const double MaxExtent = FMath::Max(
			FMath::Max(FMath::Abs(Content.Min.X), FMath::Abs(Content.Max.X)),
			FMath::Max(FMath::Abs(Content.Min.Y), FMath::Abs(Content.Max.Y)));
		Limit = FMath::Max(1.2 * MaxExtent, 0.5);
	}

	// The limit is exact, so the cell size is L divided by a whole number
	// of cells per half: lines always fall on 0 and on +/-L. The count per
	// half comes from a 1/2/5 x 10^k step near L / (cells / 2), so the
	// spacing stays close to a round number.
	const double Raw = Limit / (0.5 * kTargetCells);
	const double Mag = FMath::Pow(10.0, FMath::FloorToDouble(FMath::LogX(10.0, FMath::Max(Raw, 1e-9))));
	double Step = 10.0 * Mag;
	for (double Mantissa : { 1.0, 2.0, 5.0 })
	{
		if (Mantissa * Mag >= Raw * (1.0 - 1e-9)) { Step = Mantissa * Mag; break; }
	}
	const int32 CellsPerHalf = FMath::Clamp(FMath::CeilToInt(Limit / Step), 2, 100);

	CellSize = (float)(Limit / CellsPerHalf);
	CellsX = 2 * CellsPerHalf;
	CellsY = CellsX;
	// Lines keep the reference project's proportions: 3% and 2% of a cell.
	LineWidth = CellSize * 0.03f;
	LineHeight = CellSize * 0.02f;
	RebuildGrid();
}

void AAerotelicGridGround::RebuildGrid()
{
	if (GridLines == nullptr || BasePlane == nullptr)
	{
		return;
	}

	GridLines->ClearInstances();

	const float SizeX = CellsX * CellSize;
	const float SizeY = CellsY * CellSize;
	const float HalfX = SizeX * 0.5f;
	const float HalfY = SizeY * 0.5f;
	const float Z = LineHeight * 0.5f; // the engine cube's pivot is its centre

	TArray<FTransform> Instances;
	Instances.Reserve((CellsX + 1) + (CellsY + 1));

	// Bars running along X, one per Y grid line. Over-long by one line
	// width so the corners close cleanly.
	const FVector XBarScale((SizeX + LineWidth) / kBasicShapeSize, LineWidth / kBasicShapeSize, LineHeight / kBasicShapeSize);
	for (int32 j = 0; j <= CellsY; ++j)
	{
		Instances.Emplace(FRotator::ZeroRotator, FVector(0.0f, -HalfY + j * CellSize, Z), XBarScale);
	}

	// Bars running along Y, one per X grid line.
	const FVector YBarScale(LineWidth / kBasicShapeSize, (SizeY + LineWidth) / kBasicShapeSize, LineHeight / kBasicShapeSize);
	for (int32 i = 0; i <= CellsX; ++i)
	{
		Instances.Emplace(FRotator::ZeroRotator, FVector(-HalfX + i * CellSize, 0.0f, Z), YBarScale);
	}

	GridLines->AddInstances(Instances, /*bShouldReturnIndices*/ false);

	// A solid base just under the lattice, so the lines read against
	// something rather than against the backdrop.
	BasePlane->SetVisibility(bShowBasePlane);
	BasePlane->SetRelativeLocation(FVector(0.0f, 0.0f, -LineHeight * 0.05f));
	BasePlane->SetRelativeScale3D(FVector((SizeX + LineWidth) / kBasicShapeSize, (SizeY + LineWidth) / kBasicShapeSize, 1.0f));

	ApplyMaterials();
}

void AAerotelicGridGround::ApplyMaterials()
{
	if (ShapeMaterial == nullptr)
	{
		return;
	}
	if (LineMID == nullptr)
	{
		LineMID = UMaterialInstanceDynamic::Create(ShapeMaterial, this);
	}
	if (BaseMID == nullptr)
	{
		BaseMID = UMaterialInstanceDynamic::Create(ShapeMaterial, this);
	}
	LineMID->SetVectorParameterValue(kColorParam, LineColor);
	BaseMID->SetVectorParameterValue(kColorParam, BaseColor);
	GridLines->SetMaterial(0, LineMID);
	BasePlane->SetMaterial(0, BaseMID);
}
