#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/SceneComponent.h"
#include <memory>

#include "aerotelic/solid_model.hpp"
#include "aerotelic/project_yaml.hpp"
#include "AerotelicViewStyle.h"

#include "AerotelicEntityActor.generated.h"

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
	// Why the last RefreshProxy() could not build the model (its exception's
	// message, e.g. a wing with a negative chord), empty after a successful
	// build. The entity shows nothing while this is set; the project loader
	// and the editors report it to the user.
	const FString& GetBuildError() const { return LastBuildError; }

	// Selection highlight (the standalone app's equivalent of the Editor's
	// selection outline): the entity's triangulation edges drawn in orange
	// over its surface, which keeps its own colour (WingDesigner draws the
	// selected entity no differently, so a colour picked for it shows at
	// once). Driven by the document subsystem.
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

	// ---- mass ----

	// How this parameterisation weighs (the Scene tree's Mass Properties
	// dialog): an asserted total spread over its surface, or a density
	// times its enclosed volume. Per entity, not per body; saved with it.
	// Unset means Analysis -> Mass Properties leaves it out and says so.
	void SetMassSpec(const aerotelic::MassSpec& InSpec) { MassSpec = InSpec; }
	const aerotelic::MassSpec& GetMassSpec() const { return MassSpec; }

	// ---- surface finish, style, section ----

	// Re-applies the scene-wide finish to every body and mirror material.
	// The document subsystem calls this on all entities when the style
	// changes; a freshly built body picks the current style up itself.
	void ApplySurfaceStyle(const FAerotelicSurfaceStyle& Style);
	// Shaded / Shaded with Edges / Wireframe: which of the surface and the
	// edge meshes are shown and what colour the edges take.
	void ApplyRenderStyle(EAerotelicRenderStyle Style);
	// The section plane, as the materials' ClipPoint/ClipNormal parameters
	// (world space, so a moved entity needs nothing re-done).
	void ApplySectionPlane(const FAerotelicSectionPlane& Plane);
	// "/Game/Materials/M_AerotelicSurface" -- the asset
	// AerotelicMaterialSetupCommandlet authors. Falls back to the engine's
	// BasicShapeMaterial (colour only) if the asset is missing.
	static FString SurfaceMaterialPackage();
	// "/Game/Materials/M_AerotelicWire": the unlit wireframe material the
	// edge meshes use (same commandlet). No fallback: without it the
	// Shaded with Edges / Wireframe styles draw no edges.
	static FString WireMaterialPackage();

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

	// Fired after an entity's placed geometry changed (a rebuild after an
	// edit, or a move): boolean results that use it as an operand rebuild
	// themselves (feature_tree.py's dirty propagation).
	DECLARE_MULTICAST_DELEGATE_OneParam(FOnAerotelicGeometryChanged, AAerotelicEntityActor*);
	static FOnAerotelicGeometryChanged OnGeometryChanged;

	// This entity's shape as placed in the world (the model's solid under
	// the actor's transform), or its reflection about Y = 0 -- what a
	// boolean operation consumes (EntityShapeFeature / MirrorFeature).
	// Uses the solid cached by the last RefreshProxy(), building it if
	// there is none yet (the viewport itself no longer needs the OCCT
	// solid for a lofted body, so this is the first build for most). Null
	// if the model cannot be built.
	TopoDS_Shape GetPlacedShape(bool bMirror) const;

	// This entity's bodies as placed (or mirrored) triangle meshes within
	// Tolerance of the surface: the model's own OCCT-free meshes
	// (ISolidModel::BuildMeshes, reusing the viewport's when they are fine
	// enough), else the OCCT solid tessellated. What STL export and the
	// lattice generator consume. Empty if nothing can be built.
	std::vector<aerotelic::NamedMesh> GetPlacedMeshes(bool bMirror, double Tolerance) const;
	// World bounds of the placed (or mirrored) bodies; invalid if none.
	FBox GetPlacedBounds(bool bMirror) const;

	// This actor's placement as the backend's feature tree wants it.
	aerotelic::EntityTransform GetEntityTransform() const;
	// Applies a placement -- from a loaded project file, the clipboard, or
	// the Transform dialog -- and makes it this entity's stored one.
	// Placement is declared in the base `aerotelic` namespace (feature_tree.hpp),
	// not `aerotelic::project_yaml` -- project_yaml.hpp only (de)serializes it.
	void ApplyPlacement(const aerotelic::Placement& InPlacement);
	// The placement as authored, NOT as read back off the actor transform.
	// The two are not interchangeable: a quaternion cannot hold an axis at
	// zero angle (it is the identity whatever the axis was) nor an axis
	// that was typed un-normalised, so reading the dialog's fields back out
	// of one overwrote them -- see the comment on Placement in ApplyPlacement.
	aerotelic::Placement GetPlacement() const { return Placement; }

	UPROPERTY(VisibleAnywhere, Category = "Aerotelic")
	FString EntityName;

private:
	// This entity's placement, as authored. The actor's own transform is
	// derived from it (ApplyPlacement) for the viewport's benefit and is
	// never read back as the placement: that round-trip is lossy.
	aerotelic::Placement Placement{{0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, 0.0};

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

	// The same triangles again, drawn with the wireframe material: the
	// edges of "Shaded with Edges" and all of "Wireframe". No collision --
	// picking goes through the surface mesh even while it is hidden.
	UPROPERTY(VisibleAnywhere, Category = "Aerotelic")
	TObjectPtr<class UProceduralMeshComponent> EdgeMesh;

	UPROPERTY(VisibleAnywhere, Category = "Aerotelic")
	TObjectPtr<class UProceduralMeshComponent> MirrorEdgeMesh;

	// One dynamic material per body per mesh, so a body's colour and the
	// selection tint can be set without touching its siblings.
	UPROPERTY()
	TArray<TObjectPtr<class UMaterialInstanceDynamic>> BodyMaterials;

	UPROPERTY()
	TArray<TObjectPtr<class UMaterialInstanceDynamic>> MirrorMaterials;

	UPROPERTY()
	TArray<TObjectPtr<class UMaterialInstanceDynamic>> EdgeMaterials;

	UPROPERTY()
	TArray<TObjectPtr<class UMaterialInstanceDynamic>> MirrorEdgeMaterials;

	UPROPERTY()
	TArray<FAerotelicBody> Bodies;

	aerotelic::project_yaml::EntityType EntityType = aerotelic::project_yaml::EntityType::Primitive;
	std::unique_ptr<aerotelic::ISolidModel> Model;
	bool bSelected = false;
	bool bMirrored = false;
	// Whether the surface/mirror meshes hold any triangles (else the box
	// fallback, or nothing, is on screen).
	bool bHasTessellation = false;
	bool bMirrorHasTessellation = false;
	EAerotelicRenderStyle RenderStyle = EAerotelicRenderStyle::Shaded;
	TOptional<FLinearColor> ColorOverride;
	aerotelic::MassSpec MassSpec;
	FDelegateHandle TransformHandle;
	FDelegateHandle GeometryHandle;
	// The model's solid from the last build that needed OCCT (unplaced):
	// a boolean result, or a body whose model has no mesh builder. Null
	// while the viewport shows the model's own meshes; GetPlacedShape()
	// builds it on demand then.
	TopoDS_Shape LastSolid;
	// The model's own meshes from the last RefreshProxy() (unplaced), at
	// LastMeshTolerance; the mirror, export and the lattice generator reuse
	// them.
	std::vector<aerotelic::NamedMesh> LastMeshes;
	double LastMeshTolerance = 0.0;
	FString LastBuildError;
	// Local bounds of what was last shown, for the next build's tolerance.
	FBox LastLocalBounds;

	void OnOtherGeometryChanged(AAerotelicEntityActor* Other);

	void EnsureProxyEdges();
	void ApplyColors();
	void ApplyMeshVisibility();
	void ShowBoxFallback(const TopoDS_Shape& Shape);
	// The chordal tolerance the viewport builds the model's meshes at:
	// relative to the body's own size, as the OCCT deflection was.
	double DisplayTolerance() const;
	// Tessellates Shape into Target (surface material) and EdgeTarget
	// (wireframe material), one section per top-level solid. Returns the
	// role names of the sections written, empty if there was nothing to
	// tessellate (caller falls back to ShowBoxFallback()).
	TArray<FString> RefreshTessellatedMesh(const TopoDS_Shape& Shape,
		class UProceduralMeshComponent* Target, TArray<TObjectPtr<class UMaterialInstanceDynamic>>& Materials,
		class UProceduralMeshComponent* EdgeTarget, TArray<TObjectPtr<class UMaterialInstanceDynamic>>& EdgeMaterialsOut);
	// The same for the model's own meshes (one section per body, named
	// as the meshes are).
	TArray<FString> RefreshMeshFromTriangles(const std::vector<aerotelic::NamedMesh>& Meshes,
		class UProceduralMeshComponent* Target, TArray<TObjectPtr<class UMaterialInstanceDynamic>>& Materials,
		class UProceduralMeshComponent* EdgeTarget, TArray<TObjectPtr<class UMaterialInstanceDynamic>>& EdgeMaterialsOut);
	// Writes one body's triangles as section Section of Target and
	// EdgeTarget, reusing or creating its material instances.
	void CreateBodySection(int32 Section, const TArray<FVector>& Vertices, const TArray<int32>& Triangles, const TArray<FVector>& Normals,
		class UProceduralMeshComponent* Target, TArray<TObjectPtr<class UMaterialInstanceDynamic>>& Materials,
		class UProceduralMeshComponent* EdgeTarget, TArray<TObjectPtr<class UMaterialInstanceDynamic>>& EdgeMaterialsOut,
		TArray<TObjectPtr<class UMaterialInstanceDynamic>>& NewMaterials, TArray<TObjectPtr<class UMaterialInstanceDynamic>>& NewEdgeMaterials);
	// Rebuilds MirrorMesh from the placed-then-reflected shape. Cheap no-op
	// when the entity is not mirrored.
	void RefreshMirror();
	void OnRootTransformed(class USceneComponent* Component, EUpdateTransformFlags Flags, ETeleportType Teleport);
};
