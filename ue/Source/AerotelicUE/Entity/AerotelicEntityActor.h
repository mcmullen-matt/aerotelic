#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/SceneComponent.h"
#include <memory>

#include "aerotelic/solid_model.hpp"
#include "aerotelic/project_yaml.hpp"

#include "AerotelicEntityActor.generated.h"

// The finish every body is drawn with -- View -> Surface... sets it for the
// whole scene. Metallic 0..1; Specular 0..1 (how strong the highlights
// are, "glossy"); Roughness 0..1 (how blurred the reflections are, 0 is a
// mirror polish). The defaults are the surface material's own.
struct FAerotelicSurfaceStyle
{
	float Metallic = 0.0f;
	float Specular = 0.5f;
	float Roughness = 0.5f;
};

// One solid the entity's parameterisation produces. Most entities make a
// single "Main" body; a propeller makes a Spinner and one Blade per blade.
// Name is the role (what the body IS); the scene tree shows the display
// name "<entity>-<n>" beside it, as WingDesigner's body rows do.
USTRUCT()
struct FAerotelicBody
{
	GENERATED_BODY()

	// Role name: "Main", "Spinner", "Blade 1", ...
	UPROPERTY()
	FString Name;

	UPROPERTY()
	bool bVisible = true;

	// The mirrored copy of this body, when the entity is mirrored: shown or
	// hidden on its own, independently of the body (WingDesigner's two
	// rows). Hiding the whole entity hides both.
	UPROPERTY()
	bool bMirrorVisible = true;

	// Per-body colour override; unset means "take the entity's colour".
	UPROPERTY()
	bool bHasColor = false;

	UPROPERTY()
	FLinearColor Color = FLinearColor::White;
};

// One scene entity: a live aerotelic model (WingModel/FuselageModel/...)
// rendered as a real tessellated mesh (BRepMesh_IncrementalMesh ->
// UProceduralMeshComponent), with a wireframe-box fallback for the rare
// case BuildSolid() succeeds but tessellation produces nothing. The actor's
// Unreal transform *is* its placement, round-tripping through
// entities[].placement exactly as EntityShapeFeature::get_transform expects;
// the standalone shell's Placement fields (and, in the Editor, the viewport
// gizmo) edit it.
UCLASS()
class AAerotelicEntityActor : public AActor
{
	GENERATED_BODY()

public:
	AAerotelicEntityActor();

	// Takes ownership of InModel. Must be called once, right after SpawnActor,
	// before the model is used for anything else.
	void InitializeFrom(aerotelic::project_yaml::EntityType InType,
		std::unique_ptr<aerotelic::ISolidModel> InModel, const FString& InName);

	aerotelic::project_yaml::EntityType GetEntityType() const { return EntityType; }
	aerotelic::ISolidModel* GetModel() const { return Model.get(); }

	// Caller must already know GetEntityType() matches T (e.g. Wing ->
	// WingModel) -- same duck-typed contract the Python entity dict's
	// "model"/"type" pair relies on.
	template <typename T>
	T* GetModelAs() const { return static_cast<T*>(Model.get()); }

	// Rebuilds the tessellated mesh (or the wireframe-box fallback) from the
	// model's current BuildSolid() geometry. Called by a dialog's OnChanged
	// after every edit.
	void RefreshProxy();

	// Selection highlight (the standalone app's equivalent of the Editor's
	// selection outline): tints the mesh. Driven by the document subsystem.
	void SetSelected(bool bInSelected);
	bool IsSelectedEntity() const { return bSelected; }

	// World-space bounds of whatever is currently visible for this entity,
	// for the camera's Frame Selected / Frame All. Empty while hidden.
	FBox GetVisibleWorldBounds() const;

	// The scene tree's visibility checkbox (main_window.py's per-entity
	// Hide/Show). Hidden entities are neither drawn nor click-selectable.
	void SetEntityVisible(bool bVisible);
	bool IsEntityVisible() const { return !IsHidden(); }

	// ---- bodies (the solids this parameterisation produces) ----

	const TArray<FAerotelicBody>& GetBodies() const { return Bodies; }
	// "<entity name>-<n>", the label WingDesigner's body rows carry.
	FString GetBodyDisplayName(int32 Index) const;
	void SetBodyVisible(int32 Index, bool bVisible);
	bool IsBodyVisible(int32 Index) const;
	// Per-body colour. Clearing it falls back to the entity's colour.
	void SetBodyColor(int32 Index, TOptional<FLinearColor> InColor);
	// What body Index is actually drawn in: its own colour, else the
	// entity's, else the per-type default.
	FLinearColor GetBodyDisplayColor(int32 Index) const;

	// ---- colour ----

	// The entity's colour override; unset means the per-entity-type default
	// (WingDesigner's _kind_default). Setting one drops every per-body
	// override, as the Qt app's _set_entity_color does.
	void SetColorOverride(TOptional<FLinearColor> InColor);
	TOptional<FLinearColor> GetColorOverride() const { return ColorOverride; }
	// Override if set, else the type default -- what is on screen.
	FLinearColor GetDisplayColor() const;
	static FLinearColor DefaultColorFor(aerotelic::project_yaml::EntityType Type);

	// ---- surface finish ----

	// Re-applies the scene-wide finish to every body and mirror material.
	// The document subsystem calls this on all entities when the style
	// changes; a freshly built body picks the current style up itself.
	void ApplySurfaceStyle(const FAerotelicSurfaceStyle& Style);
	// "/Game/Materials/M_AerotelicSurface" -- the asset
	// AerotelicMaterialSetupCommandlet authors. Falls back to the engine's
	// BasicShapeMaterial (colour only) if the asset is missing.
	static FString SurfaceMaterialPackage();

	// ---- mirroring ----

	// A port-side copy reflected about world Y = 0, derived from the placed
	// shape exactly as WingDesigner's MirrorFeature is: never edited on its
	// own, it always follows this entity (including its placement, so a
	// mirrored propeller comes out the opposite hand).
	void SetMirrored(bool bInMirrored);
	bool IsMirrored() const { return bMirrored; }
	void SetMirrorBodyVisible(int32 Index, bool bVisible);
	bool IsMirrorBodyVisible(int32 Index) const;
	FString GetMirrorBodyDisplayName(int32 Index) const;

	// Fired after the set of bodies (or the mirrored state) of an entity
	// changed, so the scene tree can re-sync its child rows. Static: the
	// tree outlives any one entity.
	DECLARE_MULTICAST_DELEGATE_OneParam(FOnAerotelicBodiesChanged, AAerotelicEntityActor*);
	static FOnAerotelicBodiesChanged OnBodiesChanged;

	// This actor's placement as the backend's feature tree wants it.
	aerotelic::EntityTransform GetEntityTransform() const;
	// Applies a saved placement (from a loaded project file) to this actor.
	// Placement is declared in the base `aerotelic` namespace (feature_tree.hpp),
	// not `aerotelic::project_yaml` -- project_yaml.hpp only (de)serializes it.
	void ApplyPlacement(const aerotelic::Placement& Placement);
	aerotelic::Placement GetPlacement() const;

	UPROPERTY(VisibleAnywhere, Category = "Aerotelic")
	FString EntityName;

private:
	UPROPERTY(VisibleAnywhere, Category = "Aerotelic")
	TObjectPtr<USceneComponent> ProxyRoot;

	// Twelve thin cube StaticMeshComponents, one per box edge -- the
	// fallback visual when there's a real shape but nothing to tessellate
	// (e.g. BuildSolid() returned a degenerate/faceless shape). Built
	// lazily on first use.
	UPROPERTY()
	TArray<TObjectPtr<UStaticMeshComponent>> ProxyEdges;

	// The real visual: BuildSolid()'s tessellated surface, one mesh section
	// per body so each can carry its own colour and visibility.
	UPROPERTY(VisibleAnywhere, Category = "Aerotelic")
	TObjectPtr<class UProceduralMeshComponent> ProxyMesh;

	// The mirrored copy. Its transform is absolute (not the actor's), so it
	// can hold world-space geometry: the reflection is of the *placed*
	// shape, which no relative transform under a moving actor expresses.
	UPROPERTY(VisibleAnywhere, Category = "Aerotelic")
	TObjectPtr<class UProceduralMeshComponent> MirrorMesh;

	// One dynamic material per body per mesh, so a body's colour and the
	// selection tint can be set without touching its siblings.
	UPROPERTY()
	TArray<TObjectPtr<class UMaterialInstanceDynamic>> BodyMaterials;

	UPROPERTY()
	TArray<TObjectPtr<class UMaterialInstanceDynamic>> MirrorMaterials;

	UPROPERTY()
	TArray<FAerotelicBody> Bodies;

	aerotelic::project_yaml::EntityType EntityType = aerotelic::project_yaml::EntityType::Primitive;
	std::unique_ptr<aerotelic::ISolidModel> Model;
	bool bSelected = false;
	bool bMirrored = false;
	TOptional<FLinearColor> ColorOverride;
	FDelegateHandle TransformHandle;

	void EnsureProxyEdges();
	void ApplyColors();
	void ShowBoxFallback(const TopoDS_Shape& Shape);
	// Tessellates Shape into Target, one section per top-level solid.
	// Returns the role names of the sections written, empty if there was
	// nothing to tessellate (caller falls back to ShowBoxFallback()).
	TArray<FString> RefreshTessellatedMesh(const TopoDS_Shape& Shape,
		class UProceduralMeshComponent* Target, TArray<TObjectPtr<class UMaterialInstanceDynamic>>& Materials);
	// Rebuilds MirrorMesh from the placed-then-reflected shape. Cheap no-op
	// when the entity is not mirrored.
	void RefreshMirror();
	void OnRootTransformed(class USceneComponent* Component, EUpdateTransformFlags Flags, ETeleportType Teleport);
};
