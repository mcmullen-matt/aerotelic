#include "AerotelicEntityActor.h"

#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "UObject/ConstructorHelpers.h"
#include "ProceduralMeshComponent.h"

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

namespace
{
	// Neutral light grey for the surfaces; a warm orange tint for the one
	// selected entity (the standalone app's stand-in for the Editor's
	// selection outline).
	const FLinearColor kSurfaceColor(0.72f, 0.74f, 0.78f);
	const FLinearColor kSelectedColor(1.0f, 0.55f, 0.12f);
}

AAerotelicEntityActor::AAerotelicEntityActor()
{
	PrimaryActorTick.bCanEverTick = false;

	ProxyRoot = CreateDefaultSubobject<USceneComponent>(TEXT("ProxyRoot"));
	RootComponent = ProxyRoot;

	// A component created here via CreateDefaultSubobject is registered
	// automatically by SpawnActor() itself, before RefreshProxy()'s own
	// IsRunningCommandlet() guard ever runs -- which is exactly what made
	// the 12 StaticMeshComponent proxy edges hang under `-run=` the first
	// time (see EnsureProxyEdges()'s own note), and ProceduralMeshComponent
	// registration hangs the same way. Skip creating it at all under a
	// commandlet rather than relying on a guard that runs too late.
	if (!IsRunningCommandlet())
	{
		ProxyMesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("ProxyMesh"));
		ProxyMesh->SetupAttachment(ProxyRoot);
		ProxyMesh->SetMobility(EComponentMobility::Movable);
		// Query-only collision on the Visibility channel: this is what lets
		// the standalone app's click-to-select cursor trace hit the mesh.
		// Nothing simulates physics against it.
		ProxyMesh->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
		ProxyMesh->SetCollisionResponseToAllChannels(ECR_Ignore);
		ProxyMesh->SetCollisionResponseToChannel(ECC_Visibility, ECR_Block);
		ProxyMesh->bUseComplexAsSimpleCollision = true;
	}
}

void AAerotelicEntityActor::InitializeFrom(aerotelic::project_yaml::EntityType InType,
	std::unique_ptr<aerotelic::ISolidModel> InModel, const FString& InName)
{
	EntityType = InType;
	Model = std::move(InModel);
	EntityName = InName;
	RefreshProxy();
}

void AAerotelicEntityActor::EnsureProxyEdges()
{
	if (ProxyEdges.Num() == 12)
	{
		return;
	}
	UStaticMesh* CubeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	for (int32 i = 0; i < 12; ++i)
	{
		UStaticMeshComponent* Edge = NewObject<UStaticMeshComponent>(this, *FString::Printf(TEXT("ProxyEdge%d"), i));
		Edge->SetStaticMesh(CubeMesh);
		Edge->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Edge->SetMobility(EComponentMobility::Movable);
		Edge->SetupAttachment(ProxyRoot);
		Edge->RegisterComponent();
		ProxyEdges.Add(Edge);
	}
}

void AAerotelicEntityActor::EnsureMeshMaterial()
{
	if (MeshMaterial != nullptr)
	{
		return;
	}
	UMaterialInterface* Base = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial"));
	if (Base == nullptr)
	{
		return;
	}
	MeshMaterial = UMaterialInstanceDynamic::Create(Base, this);
	ApplySelectionTint();
}

void AAerotelicEntityActor::ApplySelectionTint()
{
	if (MeshMaterial != nullptr)
	{
		// BasicShapeMaterial exposes its base colour as the "Color" vector
		// parameter.
		MeshMaterial->SetVectorParameterValue(TEXT("Color"), bSelected ? kSelectedColor : kSurfaceColor);
	}
}

void AAerotelicEntityActor::SetSelected(bool bInSelected)
{
	if (bSelected == bInSelected)
	{
		return;
	}
	bSelected = bInSelected;
	ApplySelectionTint();
}

void AAerotelicEntityActor::SetEntityVisible(bool bVisible)
{
	SetActorHiddenInGame(!bVisible);
	SetActorEnableCollision(bVisible);
}

FBox AAerotelicEntityActor::GetVisibleWorldBounds() const
{
	FBox Box(ForceInit);
	if (IsHidden())
	{
		return Box;
	}
	if (ProxyMesh != nullptr && ProxyMesh->IsVisible())
	{
		Box += ProxyMesh->Bounds.GetBox();
	}
	for (const UStaticMeshComponent* Edge : ProxyEdges)
	{
		if (Edge != nullptr && Edge->IsVisible())
		{
			Box += Edge->Bounds.GetBox();
		}
	}
	return Box;
}

void AAerotelicEntityActor::RefreshProxy()
{
	if (Model == nullptr)
	{
		return;
	}
	// Creating/registering components hung indefinitely the one time this
	// ran under `-run=` (a commandlet's world has no real rendering context,
	// which RegisterComponent()'s render-state setup apparently blocks on).
	// Real interactive use (the only place either visual matters) always
	// runs inside the application or the full Editor, never a commandlet.
	if (IsRunningCommandlet())
	{
		return;
	}
	EnsureProxyEdges();

	TopoDS_Shape Shape;
	try
	{
		Shape = Model->BuildSolid();
	}
	catch (const std::exception& e)
	{
		// A model in a state BuildSolid() can't loft (e.g. a wing with no
		// sections) must not leave either visual in a half-built state --
		// hide both rather than show something stale or wrong.
		UE_LOG(LogTemp, Warning, TEXT("AerotelicEntityActor: BuildSolid failed for '%s': %s"), *EntityName, UTF8_TO_TCHAR(e.what()));
		ProxyMesh->SetVisibility(false);
		for (UStaticMeshComponent* Edge : ProxyEdges)
		{
			Edge->SetVisibility(false);
		}
		return;
	}

	if (RefreshTessellatedMesh(Shape))
	{
		ProxyMesh->SetVisibility(true);
		for (UStaticMeshComponent* Edge : ProxyEdges)
		{
			Edge->SetVisibility(false);
		}
	}
	else
	{
		ProxyMesh->SetVisibility(false);
		ShowBoxFallback(Shape);
	}
}

bool AAerotelicEntityActor::RefreshTessellatedMesh(const TopoDS_Shape& Shape)
{
	Bnd_Box Box;
	BRepBndLib::AddOptimal(Shape, Box);
	if (Box.IsVoid())
	{
		return false;
	}
	double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
	Box.Get(Xmin, Ymin, Zmin, Xmax, Ymax, Zmax);
	const double Diag = FMath::Max(FVector(Xmax - Xmin, Ymax - Ymin, Zmax - Zmin).Size(), 1e-6);
	// Linear deflection relative to the shape's own size, so a span-10 wing
	// and a span-1 blade both tessellate to a comparable visual fidelity.
	const double Deflection = FMath::Clamp(Diag * 0.0015, 1e-4, 1e9);

	BRepMesh_IncrementalMesh Mesher(Shape, Deflection);
	if (!Mesher.IsDone())
	{
		return false;
	}

	TArray<FVector> Vertices;
	TArray<int32> Triangles;

	for (TopExp_Explorer FaceExp(Shape, TopAbs_FACE); FaceExp.More(); FaceExp.Next())
	{
		const TopoDS_Face& Face = TopoDS::Face(FaceExp.Current());
		TopLoc_Location Loc;
		Handle(Poly_Triangulation) Tri = BRep_Tool::Triangulation(Face, Loc);
		if (Tri.IsNull())
		{
			continue;
		}
		const gp_Trsf& Trsf = Loc.Transformation();
		const bool bReversed = (Face.Orientation() == TopAbs_REVERSED);

		const int32 BaseIndex = Vertices.Num();
		for (int32 i = 1; i <= Tri->NbNodes(); ++i)
		{
			gp_Pnt P = Tri->Node(i);
			P.Transform(Trsf);
			// 1 model unit = 1 Unreal unit (cm), same convention the proxy
			// box used -- no axis conversion, so OCCT's right-handed
			// winding needs correcting for UE's left-handed one below.
			Vertices.Add(FVector(P.X(), P.Y(), P.Z()));
		}
		for (int32 i = 1; i <= Tri->NbTriangles(); ++i)
		{
			int32 n1, n2, n3;
			Tri->Triangle(i).Get(n1, n2, n3);
			if (bReversed)
			{
				Swap(n1, n2);
			}
			// Emit both windings: OCCT's right-handed convention mapped
			// straight into UE's left-handed one flips which winding is
			// "front", and getting that chirality fix exactly right without
			// being able to look at it is a coin flip not worth getting
			// wrong silently. Normals (below) are computed once from this
			// same winding and stay correct for both; the duplicate index
			// triple costs triangles, not correctness.
			Triangles.Add(BaseIndex + n1 - 1);
			Triangles.Add(BaseIndex + n2 - 1);
			Triangles.Add(BaseIndex + n3 - 1);
			Triangles.Add(BaseIndex + n1 - 1);
			Triangles.Add(BaseIndex + n3 - 1);
			Triangles.Add(BaseIndex + n2 - 1);
		}
	}

	if (Triangles.Num() == 0)
	{
		return false;
	}

	// Smooth per-vertex normals, accumulated from one winding direction
	// only (the first of each emitted pair) so the two duplicate triangles
	// above don't cancel each other's contribution to zero.
	TArray<FVector> Normals;
	Normals.SetNumZeroed(Vertices.Num());
	for (int32 t = 0; t < Triangles.Num(); t += 6)
	{
		const FVector& A = Vertices[Triangles[t]];
		const FVector& B = Vertices[Triangles[t + 1]];
		const FVector& C = Vertices[Triangles[t + 2]];
		const FVector FaceNormal = FVector::CrossProduct(B - A, C - A);
		Normals[Triangles[t]] += FaceNormal;
		Normals[Triangles[t + 1]] += FaceNormal;
		Normals[Triangles[t + 2]] += FaceNormal;
	}
	for (FVector& N : Normals)
	{
		N = N.GetSafeNormal();
	}

	// bCreateCollision = true: the triangle mesh doubles as the pick
	// geometry for the standalone app's click-to-select.
	ProxyMesh->CreateMeshSection(0, Vertices, Triangles, Normals, TArray<FVector2D>(), TArray<FColor>(), TArray<FProcMeshTangent>(), true);
	EnsureMeshMaterial();
	if (MeshMaterial != nullptr)
	{
		ProxyMesh->SetMaterial(0, MeshMaterial);
	}
	return true;
}

void AAerotelicEntityActor::ShowBoxFallback(const TopoDS_Shape& Shape)
{
	Bnd_Box Box;
	// AddOptimal (not the cheaper, pole-based Add): several of these models'
	// surfaces are NURBS, whose convex hull of poles can noticeably overstate
	// the true extent -- the same distinction primitive_fixture_check.cpp's
	// bbox check already relies on.
	BRepBndLib::AddOptimal(Shape, Box);
	if (Box.IsVoid())
	{
		for (UStaticMeshComponent* Edge : ProxyEdges)
		{
			Edge->SetVisibility(false);
		}
		return;
	}
	double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
	Box.Get(Xmin, Ymin, Zmin, Xmax, Ymax, Zmax);

	// 1 model unit = 1 Unreal unit (cm), same simple convention the plan's
	// "entities become real Unreal Actors" uses throughout.
	const double Diag = FMath::Max(FVector(Xmax - Xmin, Ymax - Ymin, Zmax - Zmin).Size(), 1e-6);
	const double Thickness = FMath::Clamp(Diag * 0.006, 0.05, 5.0);

	struct FEdgeSpec { FVector Mid; FVector Scale; };
	TArray<FEdgeSpec> Edges;
	const double Xc = 0.5 * (Xmin + Xmax), Yc = 0.5 * (Ymin + Ymax), Zc = 0.5 * (Zmin + Zmax);
	const double Lx = FMath::Max(Xmax - Xmin, 1e-6), Ly = FMath::Max(Ymax - Ymin, 1e-6), Lz = FMath::Max(Zmax - Zmin, 1e-6);
	// Engine cube is 100x100x100 units; scale is length/100 along the running
	// axis and Thickness/100 on the other two.
	for (double Y : {Ymin, Ymax})
	{
		for (double Z : {Zmin, Zmax})
		{
			Edges.Add({ FVector(Xc, Y, Z), FVector(Lx / 100.0, Thickness / 100.0, Thickness / 100.0) });
		}
	}
	for (double X : {Xmin, Xmax})
	{
		for (double Z : {Zmin, Zmax})
		{
			Edges.Add({ FVector(X, Yc, Z), FVector(Thickness / 100.0, Ly / 100.0, Thickness / 100.0) });
		}
	}
	for (double X : {Xmin, Xmax})
	{
		for (double Y : {Ymin, Ymax})
		{
			Edges.Add({ FVector(X, Y, Zc), FVector(Thickness / 100.0, Thickness / 100.0, Lz / 100.0) });
		}
	}

	check(Edges.Num() == 12 && ProxyEdges.Num() == 12);
	for (int32 i = 0; i < 12; ++i)
	{
		ProxyEdges[i]->SetVisibility(true);
		ProxyEdges[i]->SetRelativeLocation(Edges[i].Mid);
		ProxyEdges[i]->SetRelativeScale3D(Edges[i].Scale);
	}
}

aerotelic::EntityTransform AAerotelicEntityActor::GetEntityTransform() const
{
	const FVector Loc = GetActorLocation();
	FVector Axis;
	float AngleRad;
	GetActorQuat().ToAxisAndAngle(Axis, AngleRad);
	if (!Axis.IsNearlyZero())
	{
		return { Loc.X, Loc.Y, Loc.Z, Axis.X, Axis.Y, Axis.Z, FMath::RadiansToDegrees(AngleRad) };
	}
	return { Loc.X, Loc.Y, Loc.Z, 0.0, 0.0, 1.0, 0.0 };
}

void AAerotelicEntityActor::ApplyPlacement(const aerotelic::Placement& Placement)
{
	const FVector Loc(Placement.translation[0], Placement.translation[1], Placement.translation[2]);
	const FVector Axis(Placement.rotation_axis[0], Placement.rotation_axis[1], Placement.rotation_axis[2]);
	FQuat Quat = FQuat::Identity;
	if (!Axis.IsNearlyZero() && !FMath::IsNearlyZero(Placement.rotation_deg))
	{
		Quat = FQuat(Axis.GetSafeNormal(), FMath::DegreesToRadians(Placement.rotation_deg));
	}
	SetActorLocationAndRotation(Loc, Quat);
}

aerotelic::Placement AAerotelicEntityActor::GetPlacement() const
{
	aerotelic::EntityTransform T = GetEntityTransform();
	return { {T.tx, T.ty, T.tz}, {T.ax, T.ay, T.az}, T.angle_deg };
}
