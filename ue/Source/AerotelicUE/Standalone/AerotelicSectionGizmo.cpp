#include "AerotelicSectionGizmo.h"

#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/Engine.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "Misc/PackageName.h"
#include "UObject/ConstructorHelpers.h"

#include "Entity/AerotelicDocumentSubsystem.h"

namespace
{
	constexpr float kBasicShapeSize = 100.0f;
	// view_gizmo.py: frame padding 8% of the scene diagonal, arrow half
	// length 7%, travel margin 2%.
	constexpr double kFramePad = 0.08;
	constexpr double kArrowHalf = 0.07;
	constexpr double kTravelMargin = 0.02;
	const FLinearColor kAmber(1.0f, 0.7f, 0.1f, 1.0f);

	UMaterialInterface* LoadMaterial(const FString& Package)
	{
		UMaterialInterface* M = LoadObject<UMaterialInterface>(nullptr, *FString::Printf(TEXT("%s.%s"), *Package, *FPackageName::GetLongPackageAssetName(Package)));
		if (M == nullptr)
		{
			// Without the authored asset the gizmo still shows, in the
			// engine's lit basic material (depth-tested, so the handle can
			// hide inside a body).
			M = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial"));
		}
		return M;
	}
}

FString AAerotelicSectionGizmo::FrameMaterialPackage()
{
	return TEXT("/Game/Materials/M_AerotelicGizmo");
}

FString AAerotelicSectionGizmo::HandleMaterialPackage()
{
	return TEXT("/Game/Materials/M_AerotelicGizmoTop");
}

AAerotelicSectionGizmo::AAerotelicSectionGizmo()
{
	PrimaryActorTick.bCanEverTick = true;

	SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("SceneRoot"));
	SceneRoot->SetMobility(EComponentMobility::Movable);
	RootComponent = SceneRoot;

	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeFinder(TEXT("/Engine/BasicShapes/Cube.Cube"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CylinderFinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> ConeFinder(TEXT("/Engine/BasicShapes/Cone.Cone"));
	UStaticMesh* Cube = CubeFinder.Succeeded() ? CubeFinder.Object : nullptr;
	UStaticMesh* Cylinder = CylinderFinder.Succeeded() ? CylinderFinder.Object : nullptr;
	UStaticMesh* Cone = ConeFinder.Succeeded() ? ConeFinder.Object : nullptr;

	// Nothing here collides: the entity pick (a Visibility trace) must not
	// hit the gizmo, and the handle is picked in screen space.
	auto Make = [this](const TCHAR* Name, UStaticMesh* Mesh)
	{
		UStaticMeshComponent* C = CreateDefaultSubobject<UStaticMeshComponent>(Name);
		C->SetupAttachment(SceneRoot);
		C->SetMobility(EComponentMobility::Movable);
		C->SetStaticMesh(Mesh);
		C->SetCastShadow(false);
		C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		C->SetVisibility(false);
		return C;
	};
	for (int32 i = 0; i < 4; ++i)
	{
		Frame.Add(Make(*FString::Printf(TEXT("Frame%d"), i), Cube));
	}
	Shaft = Make(TEXT("Shaft"), Cylinder);
	HeadA = Make(TEXT("HeadA"), Cone);
	HeadB = Make(TEXT("HeadB"), Cone);
}

void AAerotelicSectionGizmo::BeginPlay()
{
	Super::BeginPlay();
	if (UMaterialInterface* FrameMat = LoadMaterial(FrameMaterialPackage()))
	{
		FrameMID = UMaterialInstanceDynamic::Create(FrameMat, this);
		FrameMID->SetVectorParameterValue(TEXT("Color"), kAmber);
		for (UStaticMeshComponent* Bar : Frame) Bar->SetMaterial(0, FrameMID);
	}
	if (UMaterialInterface* HandleMat = LoadMaterial(HandleMaterialPackage()))
	{
		HandleMID = UMaterialInstanceDynamic::Create(HandleMat, this);
		HandleMID->SetVectorParameterValue(TEXT("Color"), kAmber);
		Shaft->SetMaterial(0, HandleMID);
		HeadA->SetMaterial(0, HandleMID);
		HeadB->SetMaterial(0, HandleMID);
	}
	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		SectionHandle = Doc->OnSectionChanged.AddUObject(this, &AAerotelicSectionGizmo::OnSectionChanged);
		ListHandle = Doc->OnEntityListChanged.AddUObject(this, &AAerotelicSectionGizmo::Refit);
		Plane = Doc->GetSectionPlane();
	}
	Refit();
}

void AAerotelicSectionGizmo::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		Doc->OnSectionChanged.Remove(SectionHandle);
		Doc->OnEntityListChanged.Remove(ListHandle);
	}
	Super::EndPlay(EndPlayReason);
}

AAerotelicSectionGizmo* AAerotelicSectionGizmo::FindInGameWorld()
{
	UWorld* World = (GEngine != nullptr && GEngine->GameViewport != nullptr) ? GEngine->GameViewport->GetWorld() : nullptr;
	if (World == nullptr)
	{
		return nullptr;
	}
	for (TActorIterator<AAerotelicSectionGizmo> It(World); It; ++It)
	{
		return *It;
	}
	return nullptr;
}

void AAerotelicSectionGizmo::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	// The scene box moves whenever an entity is edited or dragged, which
	// no single event announces; the bounds are cached per component, so
	// re-reading them every frame is cheap (view_gizmo.py refit()).
	if (Plane.IsActive())
	{
		Refit();
	}
}

void AAerotelicSectionGizmo::OnSectionChanged()
{
	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		Plane = Doc->GetSectionPlane();
	}
	Rebuild();
}

void AAerotelicSectionGizmo::Refit()
{
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	const FBox Box = Doc ? Doc->GetSceneBounds() : FBox(ForceInit);
	if (Box.IsValid != SceneBox.IsValid || (Box.IsValid && !(Box.Min.Equals(SceneBox.Min, 1e-6) && Box.Max.Equals(SceneBox.Max, 1e-6))))
	{
		SceneBox = Box;
		Rebuild();
	}
}

void AAerotelicSectionGizmo::SetShown(bool bShown)
{
	for (UStaticMeshComponent* Bar : Frame) Bar->SetVisibility(bShown);
	Shaft->SetVisibility(bShown);
	HeadA->SetVisibility(bShown);
	HeadB->SetVisibility(bShown);
}

FVector AAerotelicSectionGizmo::AxisLineOrigin() const
{
	// The arrow's line runs through the scene centre; the plane's
	// coordinate along the axis is Plane.Position.
	FVector Centre = SceneBox.IsValid ? SceneBox.GetCenter() : FVector::ZeroVector;
	const int32 Axis = FAerotelicSectionPlane::AxisIndex(Plane.Axis);
	if (Axis >= 0)
	{
		Centre[Axis] = 0.0;
	}
	return Centre;
}

void AAerotelicSectionGizmo::Rebuild()
{
	if (!Plane.IsActive())
	{
		SetShown(false);
		return;
	}
	const int32 Axis = FAerotelicSectionPlane::AxisIndex(Plane.Axis);
	const FVector U = Plane.Unit();
	// The two in-plane axes.
	const int32 I = (Axis + 1) % 3;
	const int32 J = (Axis + 2) % 3;

	FBox Box = SceneBox;
	if (!Box.IsValid)
	{
		Box = FBox(FVector(-5.0), FVector(5.0));
	}
	const double Diag = FMath::Max(Box.GetSize().Size(), 1e-3);
	const double Pad = kFramePad * Diag;
	const FVector Centre = AxisLineOrigin() + U * Plane.Position;

	// Frame: four bars around the padded extent in the plane.
	const double HalfI = 0.5 * Box.GetSize()[I] + Pad;
	const double HalfJ = 0.5 * Box.GetSize()[J] + Pad;
	const double Bar = FMath::Clamp(0.004 * Diag, 0.005, 0.5);
	FVector EI = FVector::ZeroVector; EI[I] = 1.0;
	FVector EJ = FVector::ZeroVector; EJ[J] = 1.0;
	FVector PlaneCentre = Centre;
	PlaneCentre[I] = Box.GetCenter()[I];
	PlaneCentre[J] = Box.GetCenter()[J];
	struct FBarSpec { FVector Mid; FVector Scale; };
	FVector ScaleI = FVector(Bar) / kBasicShapeSize; ScaleI[I] = (2.0 * HalfI + Bar) / kBasicShapeSize;
	FVector ScaleJ = FVector(Bar) / kBasicShapeSize; ScaleJ[J] = (2.0 * HalfJ + Bar) / kBasicShapeSize;
	const FBarSpec Bars[4] = {
		{ PlaneCentre + EJ * HalfJ, ScaleI },
		{ PlaneCentre - EJ * HalfJ, ScaleI },
		{ PlaneCentre + EI * HalfI, ScaleJ },
		{ PlaneCentre - EI * HalfI, ScaleJ },
	};
	for (int32 b = 0; b < 4; ++b)
	{
		Frame[b]->SetWorldLocationAndRotation(Bars[b].Mid, FQuat::Identity);
		Frame[b]->SetWorldScale3D(Bars[b].Scale);
	}

	// Arrow along the axis through the scene centre: shaft between the
	// heads, cones pointing outward. Engine cylinder/cone are 100 tall on
	// Z, 50 radius, pivot at the centre.
	const double Half = kArrowHalf * Diag;
	const double HeadLen = 0.35 * Half;
	const double ShaftLen = 2.0 * (Half - HeadLen);
	const double ShaftR = 0.08 * Half;
	const double HeadR = 0.22 * Half;
	const FQuat Along = FRotationMatrix::MakeFromZ(U).ToQuat();
	const FQuat Against = FRotationMatrix::MakeFromZ(-U).ToQuat();
	Shaft->SetWorldLocationAndRotation(Centre, Along);
	Shaft->SetWorldScale3D(FVector(2.0 * ShaftR, 2.0 * ShaftR, ShaftLen) / kBasicShapeSize);
	HeadA->SetWorldLocationAndRotation(Centre + U * (Half - 0.5 * HeadLen), Along);
	HeadA->SetWorldScale3D(FVector(2.0 * HeadR, 2.0 * HeadR, HeadLen) / kBasicShapeSize);
	HeadB->SetWorldLocationAndRotation(Centre - U * (Half - 0.5 * HeadLen), Against);
	HeadB->SetWorldScale3D(FVector(2.0 * HeadR, 2.0 * HeadR, HeadLen) / kBasicShapeSize);
	ArrowA = Centre + U * Half;
	ArrowB = Centre - U * Half;

	SetShown(true);
}

bool AAerotelicSectionGizmo::IsCursorOnHandle(const APlayerController* PC, float MouseX, float MouseY) const
{
	if (!Plane.IsActive() || PC == nullptr || !Shaft->IsVisible())
	{
		return false;
	}
	FVector2D A, B;
	if (!PC->ProjectWorldLocationToScreen(ArrowA, A) || !PC->ProjectWorldLocationToScreen(ArrowB, B))
	{
		return false;
	}
	// Distance from the cursor to the projected segment, with a grab
	// margin generous enough for the thin shaft.
	const FVector2D P(MouseX, MouseY);
	const FVector2D AB = B - A;
	const double Len2 = AB.SizeSquared();
	const double T = Len2 > 1e-6 ? FMath::Clamp(FVector2D::DotProduct(P - A, AB) / Len2, 0.0, 1.0) : 0.0;
	const double Dist = (P - (A + AB * T)).Size();
	constexpr double kGrabPixels = 14.0;
	return Dist <= kGrabPixels;
}

bool AAerotelicSectionGizmo::DragParameter(const FVector& RayOrigin, const FVector& RayDir, double& OutT) const
{
	if (!Plane.IsActive())
	{
		return false;
	}
	// Closest point between the axis line P(t) = O + t U and the view ray
	// R(s) = RayOrigin + s D (view_gizmo.py axis_drag_parameter).
	const FVector U = Plane.Unit();
	const FVector D = RayDir.GetSafeNormal();
	const FVector W = AxisLineOrigin() - RayOrigin;
	const double B = FVector::DotProduct(U, D);
	const double Denom = 1.0 - B * B;
	// Within ~0.6 degrees of parallel the parameter is meaningless.
	if (Denom < 1.1e-4)
	{
		return false;
	}
	const double DU = FVector::DotProduct(W, U);
	const double DD = FVector::DotProduct(W, D);
	OutT = (B * DD - DU) / Denom;
	return true;
}

void AAerotelicSectionGizmo::BeginDrag(double T)
{
	DragT0 = T;
	DragPos0 = Plane.Position;
}

void AAerotelicSectionGizmo::DragTo(double T)
{
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	if (Doc == nullptr || !Plane.IsActive())
	{
		return;
	}
	double Position = DragPos0 + (T - DragT0);
	// Clamped to the scene box plus a margin (view_gizmo.py section_range).
	if (SceneBox.IsValid)
	{
		const int32 Axis = FAerotelicSectionPlane::AxisIndex(Plane.Axis);
		const double Margin = kTravelMargin * SceneBox.GetSize().Size();
		Position = FMath::Clamp(Position, SceneBox.Min[Axis] - Margin, SceneBox.Max[Axis] + Margin);
	}
	if (!FMath::IsNearlyEqual(Position, Plane.Position, 1e-9))
	{
		Doc->SetSectionPosition(Position);
	}
}
