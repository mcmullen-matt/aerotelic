#include "AerotelicEntityActor.h"

#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "UObject/ConstructorHelpers.h"
#include "ProceduralMeshComponent.h"

#include "AerotelicDocumentSubsystem.h"
#include "AerotelicBooleanModel.h"
#include "AerotelicMeshing.h"
#include "Misc/PackageName.h"

#include "aerotelic/feature_tree.hpp"

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

using aerotelic::project_yaml::EntityType;

namespace
{
	// A warm orange tint for the one selected entity (the standalone app's
	// stand-in for the Editor's selection outline).
	const FLinearColor kSelectedColor(1.0f, 0.55f, 0.12f);
	// occ_viewer.py _C_MESH_EDGE: the near-black of "Shaded with Edges".
	const FLinearColor kMeshEdgeColor(0.02f, 0.02f, 0.03f);

	UMaterialInterface* BaseMaterial()
	{
		static TWeakObjectPtr<UMaterialInterface> Cached;
		if (Cached.IsValid())
		{
			return Cached.Get();
		}
		const FString Package = AAerotelicEntityActor::SurfaceMaterialPackage();
		UMaterialInterface* M = LoadObject<UMaterialInterface>(nullptr, *FString::Printf(TEXT("%s.%s"), *Package, *FPackageName::GetLongPackageAssetName(Package)));
		if (M == nullptr)
		{
			UE_LOG(LogTemp, Warning, TEXT("AerotelicEntityActor: %s not found (run setup_materials.bat); bodies use BasicShapeMaterial, so View -> Surface has no effect."), *Package);
			M = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial"));
		}
		Cached = M;
		return M;
	}

	UMaterialInterface* WireMaterial()
	{
		static TWeakObjectPtr<UMaterialInterface> Cached;
		static bool bWarned = false;
		if (Cached.IsValid())
		{
			return Cached.Get();
		}
		const FString Package = AAerotelicEntityActor::WireMaterialPackage();
		UMaterialInterface* M = LoadObject<UMaterialInterface>(nullptr, *FString::Printf(TEXT("%s.%s"), *Package, *FPackageName::GetLongPackageAssetName(Package)));
		if (M == nullptr && !bWarned)
		{
			bWarned = true;
			UE_LOG(LogTemp, Warning, TEXT("AerotelicEntityActor: %s not found (run setup_materials.bat); the Shaded with Edges and Wireframe styles draw no edges."), *Package);
		}
		Cached = M;
		return M;
	}

	void ApplyStyleTo(UMaterialInstanceDynamic* Mid, const FAerotelicSurfaceStyle& Style)
	{
		if (Mid != nullptr)
		{
			Mid->SetScalarParameterValue(TEXT("Metallic"), Style.Metallic);
			Mid->SetScalarParameterValue(TEXT("Specular"), Style.Specular);
			Mid->SetScalarParameterValue(TEXT("Roughness"), Style.Roughness);
			Mid->SetScalarParameterValue(TEXT("FresnelIntensity"), Style.FresnelIntensity);
			Mid->SetScalarParameterValue(TEXT("FresnelExponent"), Style.FresnelExponent);
		}
	}

	void ApplyClipTo(UMaterialInstanceDynamic* Mid, const FAerotelicSectionPlane& Plane)
	{
		if (Mid != nullptr)
		{
			const FVector P = Plane.Point();
			const FVector N = Plane.KeptNormal();
			Mid->SetVectorParameterValue(TEXT("ClipPoint"), FLinearColor((float)P.X, (float)P.Y, (float)P.Z, 0.0f));
			Mid->SetVectorParameterValue(TEXT("ClipNormal"), FLinearColor((float)N.X, (float)N.Y, (float)N.Z, 0.0f));
		}
	}

	// The top-level solids of a built shape, in build order: a propeller's
	// compound lists its spinner then each blade; anything else is one
	// solid. (TopoDS_Iterator keeps a compound's insertion order, which is
	// what the role names below rely on.)
	TArray<TopoDS_Shape> TopLevelSolids(const TopoDS_Shape& Shape)
	{
		TArray<TopoDS_Shape> Out;
		if (Shape.IsNull())
		{
			return Out;
		}
		if (Shape.ShapeType() == TopAbs_COMPOUND)
		{
			for (TopoDS_Iterator It(Shape); It.More(); It.Next())
			{
				if (!It.Value().IsNull())
				{
					Out.Add(It.Value());
				}
			}
		}
		if (Out.Num() == 0)
		{
			Out.Add(Shape);
		}
		return Out;
	}

	// WingDesigner's role names: "Main" for a single solid, "Spinner" and
	// "Blade n" for a propeller's parts, "Body n" for a boolean result's
	// solids (_sync_result_bodies).
	FString RoleName(EntityType Type, int32 Index, int32 Count)
	{
		if (Type == EntityType::Propeller && Count > 1)
		{
			return Index == 0 ? FString(TEXT("Spinner")) : FString::Printf(TEXT("Blade %d"), Index);
		}
		if (Type == EntityType::Boolean)
		{
			return FString::Printf(TEXT("Body %d"), Index + 1);
		}
		if (Type == EntityType::Lattice)
		{
			return FString(TEXT("Lattice"));
		}
		return Count > 1 ? FString::Printf(TEXT("Part %d"), Index + 1) : FString(TEXT("Main"));
	}
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
		auto MakeMesh = [this](const TCHAR* Name)
		{
			UProceduralMeshComponent* Mesh = CreateDefaultSubobject<UProceduralMeshComponent>(Name);
			Mesh->SetupAttachment(ProxyRoot);
			Mesh->SetMobility(EComponentMobility::Movable);
			// Query-only collision on the Visibility channel: this is what
			// lets the standalone app's click-to-select cursor trace hit the
			// mesh. Nothing simulates physics against it.
			Mesh->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
			Mesh->SetCollisionResponseToAllChannels(ECR_Ignore);
			Mesh->SetCollisionResponseToChannel(ECC_Visibility, ECR_Block);
			Mesh->bUseComplexAsSimpleCollision = true;
			return Mesh;
		};
		ProxyMesh = MakeMesh(TEXT("ProxyMesh"));
		MirrorMesh = MakeMesh(TEXT("MirrorMesh"));
		// The mirror holds world-space geometry (see the header): pin it to
		// the world, not to the actor's transform.
		MirrorMesh->SetUsingAbsoluteLocation(true);
		MirrorMesh->SetUsingAbsoluteRotation(true);
		MirrorMesh->SetUsingAbsoluteScale(true);
		MirrorMesh->SetVisibility(false);

		// The edge copies: no collision (the surface mesh is the pick
		// geometry even when the style hides it), hidden until a style that
		// draws edges is chosen.
		auto MakeEdgeMesh = [this](const TCHAR* Name)
		{
			UProceduralMeshComponent* Mesh = CreateDefaultSubobject<UProceduralMeshComponent>(Name);
			Mesh->SetupAttachment(ProxyRoot);
			Mesh->SetMobility(EComponentMobility::Movable);
			Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
			Mesh->SetCastShadow(false);
			Mesh->SetVisibility(false);
			return Mesh;
		};
		EdgeMesh = MakeEdgeMesh(TEXT("EdgeMesh"));
		MirrorEdgeMesh = MakeEdgeMesh(TEXT("MirrorEdgeMesh"));
		MirrorEdgeMesh->SetUsingAbsoluteLocation(true);
		MirrorEdgeMesh->SetUsingAbsoluteRotation(true);
		MirrorEdgeMesh->SetUsingAbsoluteScale(true);
	}
}

void AAerotelicEntityActor::InitializeFrom(aerotelic::project_yaml::EntityType InType,
	std::unique_ptr<aerotelic::ISolidModel> InModel, const FString& InName)
{
	EntityType = InType;
	Model = std::move(InModel);
	EntityName = InName;
	// The mirror is a reflection of the *placed* shape, so it follows every
	// move of the actor.
	if (ProxyRoot != nullptr && !TransformHandle.IsValid())
	{
		TransformHandle = ProxyRoot->TransformUpdated.AddUObject(this, &AAerotelicEntityActor::OnRootTransformed);
	}
	// A boolean result follows its operands.
	if (EntityType == aerotelic::project_yaml::EntityType::Boolean && !GeometryHandle.IsValid())
	{
		GeometryHandle = OnGeometryChanged.AddUObject(this, &AAerotelicEntityActor::OnOtherGeometryChanged);
	}
	RefreshProxy();
}

void AAerotelicEntityActor::OnRootTransformed(USceneComponent*, EUpdateTransformFlags, ETeleportType)
{
	if (bMirrored)
	{
		RefreshMirror();
	}
	OnGeometryChanged.Broadcast(this);
}

void AAerotelicEntityActor::OnOtherGeometryChanged(AAerotelicEntityActor* Other)
{
	if (Other == this || Model == nullptr || EntityType != aerotelic::project_yaml::EntityType::Boolean)
	{
		return;
	}
	if (static_cast<const FAerotelicBooleanModel*>(Model.get())->DependsOn(Other))
	{
		RefreshProxy();
	}
}

TopoDS_Shape AAerotelicEntityActor::GetPlacedShape(bool bMirror) const
{
	if (Model == nullptr)
	{
		return TopoDS_Shape();
	}
	TopoDS_Shape Solid = LastSolid;
	if (Solid.IsNull())
	{
		try
		{
			Solid = Model->BuildSolid();
		}
		catch (const std::exception&)
		{
			return TopoDS_Shape();
		}
	}
	const aerotelic::EntityTransform T = GetEntityTransform();
	TopoDS_Shape Placed = aerotelic::Place(Solid, { T.tx, T.ty, T.tz }, { T.ax, T.ay, T.az }, T.angle_deg);
	if (bMirror)
	{
		Placed = aerotelic::ApplyMirror(Placed, { 0.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 });
	}
	return Placed;
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

// ---------------------------------------------------------------- colour

FLinearColor AAerotelicEntityActor::DefaultColorFor(aerotelic::project_yaml::EntityType Type)
{
	// WingDesigner's _kind_default (core/occ_viewer.py), linear RGB as the
	// Qt app fed OCCT. (Qualified: inside this class the bare name
	// EntityType is the data member.)
	using EType = aerotelic::project_yaml::EntityType;
	switch (Type)
	{
	case EType::Fuselage:      return FLinearColor(0.20f, 0.72f, 0.38f); // fuselage green
	case EType::Nacelle:       return FLinearColor(0.78f, 0.60f, 0.28f); // nacelle bronze
	case EType::LoftedNacelle: return FLinearColor(0.82f, 0.42f, 0.55f); // nacelle rose
	case EType::Propeller:     return FLinearColor(0.66f, 0.68f, 0.74f); // propeller aluminium
	case EType::Primitive:     return FLinearColor(0.85f, 0.80f, 0.30f); // primitive ochre
	case EType::Boolean:       return FLinearColor(0.62f, 0.56f, 0.78f); // boolean result lilac
	case EType::Lattice:       return FLinearColor(0.42f, 0.76f, 0.70f); // lattice teal
	case EType::Wing:
	default:                   return FLinearColor(0.20f, 0.50f, 0.90f); // steel blue
	}
}

FLinearColor AAerotelicEntityActor::GetDisplayColor() const
{
	return ColorOverride.IsSet() ? ColorOverride.GetValue() : DefaultColorFor(EntityType);
}

FLinearColor AAerotelicEntityActor::GetBodyDisplayColor(int32 Index) const
{
	if (Bodies.IsValidIndex(Index) && Bodies[Index].bHasColor)
	{
		return Bodies[Index].Color;
	}
	return GetDisplayColor();
}

void AAerotelicEntityActor::SetColorOverride(TOptional<FLinearColor> InColor)
{
	ColorOverride = InColor;
	// Every body takes the entity's colour, the mirrored ones included,
	// and any colour a body had on its own is dropped (the Qt app's
	// _set_entity_color).
	for (FAerotelicBody& Body : Bodies)
	{
		Body.bHasColor = false;
	}
	ApplyColors();
}

void AAerotelicEntityActor::SetBodyColor(int32 Index, TOptional<FLinearColor> InColor)
{
	if (!Bodies.IsValidIndex(Index))
	{
		return;
	}
	Bodies[Index].bHasColor = InColor.IsSet();
	if (InColor.IsSet())
	{
		Bodies[Index].Color = InColor.GetValue();
	}
	ApplyColors();
}

void AAerotelicEntityActor::ApplyColors()
{
	// The surface always shows the body's own colour (WingDesigner draws
	// the selected entity no differently; a colour picked for it must be
	// visible at once). The selection shows on the edge copy instead: the
	// selected entity's triangulation edges in orange over its surface.
	for (int32 i = 0; i < BodyMaterials.Num(); ++i)
	{
		if (BodyMaterials[i] != nullptr)
		{
			BodyMaterials[i]->SetVectorParameterValue(TEXT("Color"), GetBodyDisplayColor(i));
		}
	}
	// The mirror inherits its source body's colour (REQ-VIZ-020).
	for (int32 i = 0; i < MirrorMaterials.Num(); ++i)
	{
		if (MirrorMaterials[i] != nullptr)
		{
			MirrorMaterials[i]->SetVectorParameterValue(TEXT("Color"), GetBodyDisplayColor(i));
		}
	}
	// Edges: orange while selected; else near-black over a shaded surface,
	// or the body's own colour when they are all there is (occ_viewer.py
	// _apply_style's SetEdgeColor).
	const bool bWire = RenderStyle == EAerotelicRenderStyle::Wireframe;
	for (int32 i = 0; i < EdgeMaterials.Num(); ++i)
	{
		if (EdgeMaterials[i] != nullptr)
		{
			EdgeMaterials[i]->SetVectorParameterValue(TEXT("Color"), bSelected ? kSelectedColor : (bWire ? GetBodyDisplayColor(i) : kMeshEdgeColor));
		}
	}
	for (int32 i = 0; i < MirrorEdgeMaterials.Num(); ++i)
	{
		if (MirrorEdgeMaterials[i] != nullptr)
		{
			MirrorEdgeMaterials[i]->SetVectorParameterValue(TEXT("Color"), bSelected ? kSelectedColor : (bWire ? GetBodyDisplayColor(i) : kMeshEdgeColor));
		}
	}
}

FString AAerotelicEntityActor::SurfaceMaterialPackage()
{
	return TEXT("/Game/Materials/M_AerotelicSurface");
}

FString AAerotelicEntityActor::WireMaterialPackage()
{
	return TEXT("/Game/Materials/M_AerotelicWire");
}

void AAerotelicEntityActor::ApplySurfaceStyle(const FAerotelicSurfaceStyle& Style)
{
	for (UMaterialInstanceDynamic* Mid : BodyMaterials) ApplyStyleTo(Mid, Style);
	for (UMaterialInstanceDynamic* Mid : MirrorMaterials) ApplyStyleTo(Mid, Style);
}

void AAerotelicEntityActor::ApplyRenderStyle(EAerotelicRenderStyle Style)
{
	RenderStyle = Style;
	ApplyMeshVisibility();
	ApplyColors();
}

void AAerotelicEntityActor::ApplySectionPlane(const FAerotelicSectionPlane& Plane)
{
	for (UMaterialInstanceDynamic* Mid : BodyMaterials) ApplyClipTo(Mid, Plane);
	for (UMaterialInstanceDynamic* Mid : MirrorMaterials) ApplyClipTo(Mid, Plane);
	for (UMaterialInstanceDynamic* Mid : EdgeMaterials) ApplyClipTo(Mid, Plane);
	for (UMaterialInstanceDynamic* Mid : MirrorEdgeMaterials) ApplyClipTo(Mid, Plane);
}

void AAerotelicEntityActor::ApplyMeshVisibility()
{
	// Which meshes the style shows. The surface mesh stays registered and
	// collidable while hidden, so a wireframe body is still click-selectable.
	// The edge copy also carries the selection highlight, so it shows for
	// the selected entity in every style.
	const bool bSurface = RenderStyle != EAerotelicRenderStyle::Wireframe;
	const bool bEdges = RenderStyle != EAerotelicRenderStyle::Shaded || bSelected;
	if (ProxyMesh != nullptr)      ProxyMesh->SetVisibility(bHasTessellation && bSurface);
	if (EdgeMesh != nullptr)       EdgeMesh->SetVisibility(bHasTessellation && bEdges);
	if (MirrorMesh != nullptr)     MirrorMesh->SetVisibility(bMirrorHasTessellation && bSurface);
	if (MirrorEdgeMesh != nullptr) MirrorEdgeMesh->SetVisibility(bMirrorHasTessellation && bEdges);
}

void AAerotelicEntityActor::SetSelected(bool bInSelected)
{
	if (bSelected == bInSelected)
	{
		return;
	}
	bSelected = bInSelected;
	ApplyMeshVisibility();
	ApplyColors();
}

// ------------------------------------------------------------ visibility

void AAerotelicEntityActor::SetEntityVisible(bool bVisible)
{
	SetActorHiddenInGame(!bVisible);
	SetActorEnableCollision(bVisible);
}

FString AAerotelicEntityActor::GetBodyDisplayName(int32 Index) const
{
	return FString::Printf(TEXT("%s-%d"), *EntityName, Index + 1);
}

void AAerotelicEntityActor::SetBodyVisible(int32 Index, bool bVisible)
{
	if (!Bodies.IsValidIndex(Index))
	{
		return;
	}
	// A body and its mirrored copy are shown or hidden independently, as
	// WingDesigner's two rows are; only hiding the whole entity takes
	// both away.
	Bodies[Index].bVisible = bVisible;
	if (ProxyMesh != nullptr)
	{
		ProxyMesh->SetMeshSectionVisible(Index, bVisible);
	}
	if (EdgeMesh != nullptr)
	{
		EdgeMesh->SetMeshSectionVisible(Index, bVisible);
	}
}

bool AAerotelicEntityActor::IsBodyVisible(int32 Index) const
{
	return Bodies.IsValidIndex(Index) && Bodies[Index].bVisible;
}

void AAerotelicEntityActor::SetMirrorBodyVisible(int32 Index, bool bVisible)
{
	if (!Bodies.IsValidIndex(Index))
	{
		return;
	}
	Bodies[Index].bMirrorVisible = bVisible;
	if (MirrorMesh != nullptr)
	{
		MirrorMesh->SetMeshSectionVisible(Index, bVisible);
	}
	if (MirrorEdgeMesh != nullptr)
	{
		MirrorEdgeMesh->SetMeshSectionVisible(Index, bVisible);
	}
}

bool AAerotelicEntityActor::IsMirrorBodyVisible(int32 Index) const
{
	return Bodies.IsValidIndex(Index) && Bodies[Index].bMirrorVisible;
}

FString AAerotelicEntityActor::GetMirrorBodyDisplayName(int32 Index) const
{
	return FString::Printf(TEXT("%s-%d-mirror"), *EntityName, Index + 1);
}

AAerotelicEntityActor::FOnAerotelicBodiesChanged AAerotelicEntityActor::OnBodiesChanged;
AAerotelicEntityActor::FOnAerotelicGeometryChanged AAerotelicEntityActor::OnGeometryChanged;

FBox AAerotelicEntityActor::GetVisibleWorldBounds() const
{
	FBox Box(ForceInit);
	if (IsHidden())
	{
		return Box;
	}
	// The surface meshes' bounds stand for both copies, whichever the style
	// is showing (in Wireframe the surface is hidden but its edges are not).
	if (ProxyMesh != nullptr && bHasTessellation)
	{
		Box += ProxyMesh->Bounds.GetBox();
	}
	if (MirrorMesh != nullptr && bMirrorHasTessellation)
	{
		Box += MirrorMesh->Bounds.GetBox();
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

// ------------------------------------------------------------- mirroring

void AAerotelicEntityActor::SetMirrored(bool bInMirrored)
{
	if (bMirrored == bInMirrored)
	{
		return;
	}
	bMirrored = bInMirrored;
	if (bMirrored)
	{
		// The mirror is the only thing drawing that half now, so it starts
		// visible.
		for (FAerotelicBody& Body : Bodies)
		{
			Body.bMirrorVisible = true;
		}
	}
	RefreshMirror();
	OnBodiesChanged.Broadcast(this);
}

void AAerotelicEntityActor::RefreshMirror()
{
	if (MirrorMesh == nullptr || IsRunningCommandlet())
	{
		return;
	}
	if (!bMirrored || Model == nullptr)
	{
		MirrorMesh->ClearAllMeshSections();
		MirrorEdgeMesh->ClearAllMeshSections();
		bMirrorHasTessellation = false;
		ApplyMeshVisibility();
		MirrorMaterials.Reset();
		MirrorEdgeMaterials.Reset();
		return;
	}

	TArray<FString> Roles;
	if (!LastMeshes.empty())
	{
		// The model's own meshes, placed and reflected about Y = 0 (what
		// MirrorFeature does to EntityShapeFeature's result): an
		// off-centreline boom lands on the other side, and a rotated
		// propeller comes out the opposite hand.
		std::vector<aerotelic::NamedMesh> Mirrored = LastMeshes;
		const FTransform Transform = GetActorTransform();
		for (aerotelic::NamedMesh& M : Mirrored)
		{
			AerotelicMeshing::PlaceMesh(M.mesh, Transform);
			AerotelicMeshing::MirrorY(M.mesh);
		}
		Roles = RefreshMeshFromTriangles(Mirrored, MirrorMesh, MirrorMaterials, MirrorEdgeMesh, MirrorEdgeMaterials);
	}
	else
	{
		TopoDS_Shape Placed;
		try
		{
			// The OCCT path: the solid RefreshProxy() cached, placed and
			// reflected.
			const aerotelic::EntityTransform T = GetEntityTransform();
			Placed = aerotelic::Place(LastSolid.IsNull() ? Model->BuildSolid() : LastSolid, { T.tx, T.ty, T.tz }, { T.ax, T.ay, T.az }, T.angle_deg);
			Placed = aerotelic::ApplyMirror(Placed, { 0.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 });
		}
		catch (const std::exception& e)
		{
			UE_LOG(LogTemp, Warning, TEXT("AerotelicEntityActor: mirror of '%s' failed: %s"), *EntityName, UTF8_TO_TCHAR(e.what()));
			MirrorMesh->ClearAllMeshSections();
			MirrorEdgeMesh->ClearAllMeshSections();
			bMirrorHasTessellation = false;
			ApplyMeshVisibility();
			return;
		}
		Roles = RefreshTessellatedMesh(Placed, MirrorMesh, MirrorMaterials, MirrorEdgeMesh, MirrorEdgeMaterials);
	}
	bMirrorHasTessellation = Roles.Num() > 0;
	ApplyMeshVisibility();
	for (int32 i = 0; i < Roles.Num(); ++i)
	{
		const bool bVisible = !Bodies.IsValidIndex(i) || Bodies[i].bMirrorVisible;
		MirrorMesh->SetMeshSectionVisible(i, bVisible);
		MirrorEdgeMesh->SetMeshSectionVisible(i, bVisible);
	}
	ApplyColors();
}

// ---------------------------------------------------------------- build

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

	// The model's own meshes first (every lofted body and primitive builds
	// them straight from its section curves, in milliseconds); the OCCT
	// solid, tessellated, only for a model without a mesh builder (a
	// boolean result, a nacelle with chevrons).
	std::vector<aerotelic::NamedMesh> Meshes;
	TopoDS_Shape Shape;
	const double Tolerance = DisplayTolerance();
	try
	{
		Meshes = Model->BuildMeshes(Tolerance);
		if (Meshes.empty())
		{
			Shape = Model->BuildSolid();
		}
	}
	catch (const std::exception& e)
	{
		// A model in a state that can't be built (e.g. a wing with no
		// sections) must not leave either visual in a half-built state --
		// hide both rather than show something stale or wrong.
		UE_LOG(LogTemp, Warning, TEXT("AerotelicEntityActor: build failed for '%s': %s"), *EntityName, UTF8_TO_TCHAR(e.what()));
		LastBuildError = UTF8_TO_TCHAR(e.what());
		LastSolid.Nullify();
		LastMeshes.clear();
		bHasTessellation = false;
		bMirrorHasTessellation = false;
		ApplyMeshVisibility();
		for (UStaticMeshComponent* Edge : ProxyEdges)
		{
			Edge->SetVisibility(false);
		}
		OnGeometryChanged.Broadcast(this);
		return;
	}

	LastBuildError.Empty();

	// Keep each body's display state across a rebuild by role name, so a
	// hidden or recoloured blade stays so when the propeller is edited.
	const TArray<FAerotelicBody> Previous = Bodies;
	TArray<FString> Roles;
	LastLocalBounds = FBox(ForceInit);
	if (!Meshes.empty())
	{
		LastMeshes = Meshes;
		LastMeshTolerance = Tolerance;
		LastSolid.Nullify();
		for (const aerotelic::NamedMesh& M : Meshes)
		{
			LastLocalBounds += AerotelicMeshing::BoundsOf(M.mesh);
		}
		Roles = RefreshMeshFromTriangles(Meshes, ProxyMesh, BodyMaterials, EdgeMesh, EdgeMaterials);
	}
	else
	{
		LastSolid = Shape;
		LastMeshes.clear();
		Bnd_Box Box;
		BRepBndLib::AddOptimal(Shape, Box);
		if (!Box.IsVoid())
		{
			double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
			Box.Get(Xmin, Ymin, Zmin, Xmax, Ymax, Zmax);
			LastLocalBounds = FBox(FVector(Xmin, Ymin, Zmin), FVector(Xmax, Ymax, Zmax));
		}
		Roles = RefreshTessellatedMesh(Shape, ProxyMesh, BodyMaterials, EdgeMesh, EdgeMaterials);
	}
	Bodies.Reset();
	for (const FString& BodyRole : Roles)
	{
		FAerotelicBody Body;
		Body.Name = BodyRole;
		if (const FAerotelicBody* Old = Previous.FindByPredicate([&BodyRole](const FAerotelicBody& B) { return B.Name == BodyRole; }))
		{
			Body.bVisible = Old->bVisible;
			Body.bMirrorVisible = Old->bMirrorVisible;
			Body.bHasColor = Old->bHasColor;
			Body.Color = Old->Color;
		}
		Bodies.Add(Body);
	}
	bool bBodiesChanged = Previous.Num() != Bodies.Num();
	for (int32 i = 0; !bBodiesChanged && i < Bodies.Num(); ++i)
	{
		bBodiesChanged = Previous[i].Name != Bodies[i].Name;
	}

	bHasTessellation = Roles.Num() > 0;
	ApplyMeshVisibility();
	if (bHasTessellation)
	{
		for (int32 i = 0; i < Bodies.Num(); ++i)
		{
			ProxyMesh->SetMeshSectionVisible(i, Bodies[i].bVisible);
			EdgeMesh->SetMeshSectionVisible(i, Bodies[i].bVisible);
		}
		for (UStaticMeshComponent* Edge : ProxyEdges)
		{
			Edge->SetVisibility(false);
		}
	}
	else
	{
		ShowBoxFallback(Shape);
	}
	ApplyColors();
	RefreshMirror();
	if (bBodiesChanged)
	{
		OnBodiesChanged.Broadcast(this);
	}
	OnGeometryChanged.Broadcast(this);
}

TArray<FString> AAerotelicEntityActor::RefreshTessellatedMesh(const TopoDS_Shape& Shape,
	UProceduralMeshComponent* Target, TArray<TObjectPtr<UMaterialInstanceDynamic>>& Materials,
	UProceduralMeshComponent* EdgeTarget, TArray<TObjectPtr<UMaterialInstanceDynamic>>& EdgeMaterialsOut)
{
	TArray<FString> Roles;
	if (Target == nullptr)
	{
		return Roles;
	}
	Target->ClearAllMeshSections();
	if (EdgeTarget != nullptr)
	{
		EdgeTarget->ClearAllMeshSections();
	}

	Bnd_Box Box;
	BRepBndLib::AddOptimal(Shape, Box);
	if (Box.IsVoid())
	{
		return Roles;
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
		return Roles;
	}

	const TArray<TopoDS_Shape> Solids = TopLevelSolids(Shape);
	TArray<TObjectPtr<UMaterialInstanceDynamic>> NewMaterials;
	TArray<TObjectPtr<UMaterialInstanceDynamic>> NewEdgeMaterials;

	for (int32 s = 0; s < Solids.Num(); ++s)
	{
		TArray<FVector> Vertices;
		TArray<int32> Triangles;

		for (TopExp_Explorer FaceExp(Solids[s], TopAbs_FACE); FaceExp.More(); FaceExp.Next())
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
			continue;
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

		CreateBodySection(Roles.Num(), Vertices, Triangles, Normals, Target, Materials, EdgeTarget, EdgeMaterialsOut, NewMaterials, NewEdgeMaterials);
		Roles.Add(RoleName(EntityType, s, Solids.Num()));
	}
	Materials = MoveTemp(NewMaterials);
	EdgeMaterialsOut = MoveTemp(NewEdgeMaterials);
	return Roles;
}

TArray<FString> AAerotelicEntityActor::RefreshMeshFromTriangles(const std::vector<aerotelic::NamedMesh>& Meshes,
	UProceduralMeshComponent* Target, TArray<TObjectPtr<UMaterialInstanceDynamic>>& Materials,
	UProceduralMeshComponent* EdgeTarget, TArray<TObjectPtr<UMaterialInstanceDynamic>>& EdgeMaterialsOut)
{
	TArray<FString> Roles;
	if (Target == nullptr)
	{
		return Roles;
	}
	Target->ClearAllMeshSections();
	if (EdgeTarget != nullptr)
	{
		EdgeTarget->ClearAllMeshSections();
	}
	TArray<TObjectPtr<UMaterialInstanceDynamic>> NewMaterials;
	TArray<TObjectPtr<UMaterialInstanceDynamic>> NewEdgeMaterials;
	for (const aerotelic::NamedMesh& M : Meshes)
	{
		if (M.mesh.IsEmpty())
		{
			continue;
		}
		TArray<FVector> Vertices;
		TArray<int32> Triangles;
		TArray<FVector> Normals;
		AerotelicMeshing::ToProceduralMesh(M.mesh, Vertices, Triangles, Normals);
		CreateBodySection(Roles.Num(), Vertices, Triangles, Normals, Target, Materials, EdgeTarget, EdgeMaterialsOut, NewMaterials, NewEdgeMaterials);
		Roles.Add(UTF8_TO_TCHAR(M.name.c_str()));
	}
	Materials = MoveTemp(NewMaterials);
	EdgeMaterialsOut = MoveTemp(NewEdgeMaterials);
	return Roles;
}

void AAerotelicEntityActor::CreateBodySection(int32 Section, const TArray<FVector>& Vertices, const TArray<int32>& Triangles, const TArray<FVector>& Normals,
	UProceduralMeshComponent* Target, TArray<TObjectPtr<UMaterialInstanceDynamic>>& Materials,
	UProceduralMeshComponent* EdgeTarget, TArray<TObjectPtr<UMaterialInstanceDynamic>>& EdgeMaterialsOut,
	TArray<TObjectPtr<UMaterialInstanceDynamic>>& NewMaterials, TArray<TObjectPtr<UMaterialInstanceDynamic>>& NewEdgeMaterials)
{
	UMaterialInterface* Base = BaseMaterial();
	UMaterialInterface* Wire = WireMaterial();
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();

	// bCreateCollision = true: the triangle mesh doubles as the pick
	// geometry for the standalone app's click-to-select. A lattice (often
	// millions of triangles) does without: cooking its collision would
	// take far longer than generating it, and it is selected in the tree.
	const bool bCollision = EntityType != aerotelic::project_yaml::EntityType::Lattice;
	Target->CreateMeshSection(Section, Vertices, Triangles, Normals, TArray<FVector2D>(), TArray<FColor>(), TArray<FProcMeshTangent>(), bCollision);

	// Reuse the section's material instance across rebuilds; make one
	// the first time. A new instance picks up the scene-wide finish and
	// section plane itself.
	UMaterialInstanceDynamic* Mid = Materials.IsValidIndex(Section) ? Materials[Section].Get() : nullptr;
	if (Mid == nullptr && Base != nullptr)
	{
		Mid = UMaterialInstanceDynamic::Create(Base, this);
		if (Doc != nullptr)
		{
			ApplyStyleTo(Mid, Doc->GetSurfaceStyle());
			ApplyClipTo(Mid, Doc->GetSectionPlane());
		}
	}
	if (Mid != nullptr)
	{
		Target->SetMaterial(Section, Mid);
	}
	NewMaterials.Add(Mid);

	// The edge copy: same triangles, wireframe material, no collision.
	if (EdgeTarget != nullptr)
	{
		EdgeTarget->CreateMeshSection(Section, Vertices, Triangles, Normals, TArray<FVector2D>(), TArray<FColor>(), TArray<FProcMeshTangent>(), false);
		UMaterialInstanceDynamic* EdgeMid = EdgeMaterialsOut.IsValidIndex(Section) ? EdgeMaterialsOut[Section].Get() : nullptr;
		if (EdgeMid == nullptr && Wire != nullptr)
		{
			EdgeMid = UMaterialInstanceDynamic::Create(Wire, this);
			if (Doc != nullptr)
			{
				ApplyClipTo(EdgeMid, Doc->GetSectionPlane());
			}
		}
		if (EdgeMid != nullptr)
		{
			EdgeTarget->SetMaterial(Section, EdgeMid);
		}
		NewEdgeMaterials.Add(EdgeMid);
	}
}

double AAerotelicEntityActor::DisplayTolerance() const
{
	// Relative to the body's own size, so a span-10 wing and a span-1 blade
	// both come out at a comparable visual fidelity (the same 0.15% of the
	// diagonal the OCCT deflection used). Before the first build the size
	// is unknown: build fine, and the next build takes its size from this.
	if (LastLocalBounds.IsValid)
	{
		const double Diag = FMath::Max(LastLocalBounds.GetSize().Size(), 1e-6);
		return FMath::Clamp(Diag * 0.0015, 1e-4, 1e9);
	}
	return 2e-3;
}

std::vector<aerotelic::NamedMesh> AAerotelicEntityActor::GetPlacedMeshes(bool bMirror, double Tolerance) const
{
	std::vector<aerotelic::NamedMesh> Meshes;
	if (Model == nullptr)
	{
		return Meshes;
	}
	try
	{
		// The viewport's own meshes when they are at least as fine as asked.
		if (!LastMeshes.empty() && LastMeshTolerance <= Tolerance)
		{
			Meshes = LastMeshes;
		}
		else
		{
			Meshes = Model->BuildMeshes(Tolerance);
		}
	}
	catch (const std::exception& e)
	{
		UE_LOG(LogTemp, Warning, TEXT("AerotelicEntityActor: meshes of '%s' failed: %s"), *EntityName, UTF8_TO_TCHAR(e.what()));
		return {};
	}
	if (Meshes.empty())
	{
		// No mesh builder: tessellate the OCCT solid, already placed.
		return AerotelicMeshing::MeshesFromShape(GetPlacedShape(bMirror), Tolerance);
	}
	const FTransform Transform = GetActorTransform();
	for (aerotelic::NamedMesh& M : Meshes)
	{
		AerotelicMeshing::PlaceMesh(M.mesh, Transform);
		if (bMirror)
		{
			AerotelicMeshing::MirrorY(M.mesh);
		}
	}
	return Meshes;
}

FBox AAerotelicEntityActor::GetPlacedBounds(bool bMirror) const
{
	FBox Box(ForceInit);
	if (!LastMeshes.empty())
	{
		// The eight corners of the local bounds, placed.
		const FTransform Transform = GetActorTransform();
		FBox Local(ForceInit);
		for (const aerotelic::NamedMesh& M : LastMeshes)
		{
			Local += AerotelicMeshing::BoundsOf(M.mesh);
		}
		if (!Local.IsValid)
		{
			return Box;
		}
		for (int32 i = 0; i < 8; ++i)
		{
			FVector Corner((i & 1) ? Local.Max.X : Local.Min.X, (i & 2) ? Local.Max.Y : Local.Min.Y, (i & 4) ? Local.Max.Z : Local.Min.Z);
			Corner = Transform.TransformPositionNoScale(Corner);
			if (bMirror)
			{
				Corner.Y = -Corner.Y;
			}
			Box += Corner;
		}
		return Box;
	}
	for (const aerotelic::NamedMesh& M : GetPlacedMeshes(bMirror, DisplayTolerance()))
	{
		Box += AerotelicMeshing::BoundsOf(M.mesh);
	}
	return Box;
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

// ------------------------------------------------------------- placement

aerotelic::EntityTransform AAerotelicEntityActor::GetEntityTransform() const
{
	const aerotelic::Placement& P = Placement;
	// A zero-length axis is no rotation whatever the angle reads: OCC's
	// gp_Dir, which Place() hands this to, throws on a null direction.
	const bool bHasAxis = P.rotation_axis[0] != 0.0 || P.rotation_axis[1] != 0.0 || P.rotation_axis[2] != 0.0;
	if (!bHasAxis)
	{
		return { P.translation[0], P.translation[1], P.translation[2], 0.0, 0.0, 1.0, 0.0 };
	}
	return { P.translation[0], P.translation[1], P.translation[2],
		P.rotation_axis[0], P.rotation_axis[1], P.rotation_axis[2], P.rotation_deg };
}

void AAerotelicEntityActor::ApplyPlacement(const aerotelic::Placement& InPlacement)
{
	// The placement is stored verbatim, never normalised and never read
	// back off the transform below. An FQuat holds a rotation, not the
	// parameters that describe one: at zero angle it is the identity and
	// has forgotten the axis entirely, and it cannot hold an axis that was
	// typed as (0, 2, 1) rather than a unit vector. Normalising is also
	// unnecessary -- gp_Dir (core) and FQuat (here) both do it themselves.
	Placement = InPlacement;
	aerotelic::SyncPlacementRotation(Placement);

	const FVector Loc(Placement.translation[0], Placement.translation[1], Placement.translation[2]);
	const FVector Axis(Placement.rotation_axis[0], Placement.rotation_axis[1], Placement.rotation_axis[2]);
	FQuat Quat = FQuat::Identity;
	if (!Axis.IsNearlyZero() && !FMath::IsNearlyZero(Placement.rotation_deg))
	{
		Quat = FQuat(Axis.GetSafeNormal(), FMath::DegreesToRadians(Placement.rotation_deg));
	}
	// Moving the root is what refreshes the mirror and rebuilds any
	// boolean over this entity -- see OnRootTransformed. An edit that
	// leaves the transform alone (an axis typed while the angle is zero)
	// deliberately rebuilds nothing: the geometry really is unchanged.
	SetActorLocationAndRotation(Loc, Quat);
}
