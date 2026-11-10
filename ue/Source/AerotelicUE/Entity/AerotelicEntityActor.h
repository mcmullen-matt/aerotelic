#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/SceneComponent.h"
#include <memory>

#include "aerotelic/solid_model.hpp"
#include "aerotelic/project_yaml.hpp"
#include "AerotelicBodyStructure.h"
#include "AerotelicViewStyle.h"

#include "AerotelicEntityActor.generated.h"

// The plane an entity's mirrored copy is reflected in (the scene tree's
// Mirror dialog, 2026-11-10): a normal (any direction; kept as typed) and
// an offset along it from the origin, in project units -- the plane is
// normal . p = offset, placed with the model scale. The default is the
// aircraft's symmetry plane y = 0. Saved as `mirror_plane:` when it is
// not the default.
USTRUCT()
struct FAerotelicMirrorPlane
{
	GENERATED_BODY()

	UPROPERTY()
	FVector Normal = FVector(0.0, 1.0, 0.0);

	UPROPERTY()
	double Offset = 0.0;

	bool IsDefault() const { return Normal.Equals(FVector(0.0, 1.0, 0.0), 0.0) && Offset == 0.0; }
	bool operator==(const FAerotelicMirrorPlane& O) const { return Normal.Equals(O.Normal, 0.0) && Offset == O.Offset; }
	bool operator!=(const FAerotelicMirrorPlane& O) const { return !(*this == O); }
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

	// What the body is made of and built with (body_structure.hpp): its
	// material, skin, plane networks, asserted mass. Per body since
	// 2026-11-08; kept by role name across rebuilds like the display state,
	// saved under the entity's `bodies:`. Not reflected (a plain C++ value).
	aerotelic::BodyStructure Structure;

	// The structure's GENERATED geometry -- the skin and the rib discs, as
	// shells (AerotelicBodyStructure.h) -- once the document has generated
	// it; null until then. Kept by role across rebuilds and marked stale by
	// them (the document regenerates); not saved (regenerated on load for
	// a body with an interior). Not reflected.
	TSharedPtr<FAerotelicBodyStructure> Generated;
};

// One layer of generated structure as the actor draws and lists it: which
// body's, which of its layers, and the layer's own display state (shown or
// hidden on its own and on the mirrored half, a colour of its own). The
// Skin layer IS the body's surface -- it has no mesh and no section of its
// own -- so its flag shows or hides the body's own section, and hiding it
// leaves the ribs alone on screen (IsBodySurfaceShown).
struct FAerotelicLayerView
{
	FString BodyRole;
	int32 BodyIndex = INDEX_NONE;
	int32 LayerIndex = INDEX_NONE;
	FString Name;
	FAerotelicStructureLayer::EKind Kind = FAerotelicStructureLayer::EKind::Skin;
	bool bVisible = true;
	bool bMirrorVisible = true;
	bool bHasColor = false;
	FLinearColor Color = FLinearColor::White;
	// The section of the structure meshes this layer is drawn as; INDEX_NONE
	// for an empty layer (nothing to draw, still listed).
	int32 Section = INDEX_NONE;
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
	// The model's own meshes from the last RefreshProxy(), unplaced, one
	// per body by role: the body's own grid (what a wing's structure plan
	// indexes). Empty for a model without a mesh builder.
	const std::vector<aerotelic::NamedMesh>& GetLastMeshes() const { return LastMeshes; }
	// The structure plans a wing's model made for THOSE meshes (copied at
	// RefreshProxy: the model's own cache is overwritten by every later
	// build at another tolerance -- a boolean's, an analysis's -- whose
	// plans index a different grid). Empty for anything but a wing.
	const std::vector<aerotelic::WingBodyStructurePlan>& GetLastStructurePlans() const { return LastStructurePlans; }

	// Caller must already know GetEntityType() matches T (e.g. Wing ->
	// WingModel) -- same duck-typed contract the Python entity dict's
	// "model"/"type" pair relies on.
	template <typename T>
	T* GetModelAs() const { return static_cast<T*>(Model.get()); }

	// Swaps in a new model -- a design panel's draft, applied -- and rebuilds
	// from it. Each body's display state (hidden, colour) is kept by role
	// name across the swap, as across any rebuild. If the new model cannot
	// be built the old one is put back, rebuilt, and false returned with
	// the reason: the scene never shows a model that refused to build.
	bool ReplaceModel(std::unique_ptr<aerotelic::ISolidModel> NewModel, FString& OutError);

	// Rebuilds the tessellated mesh (or the wireframe-box fallback) from the
	// model's current BuildSolid() geometry. Called when a design panel's
	// draft is applied (ReplaceModel), when a boolean result over it is
	// rebuilt, and on load.
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

	// ---- structure (per body, 2026-11-08) ----

	// What each body is made of and built with -- its material, skin
	// thickness, plane networks, exclusions, asserted mass
	// (aerotelic::BodyStructure) -- addressed by the body's role name. Kept
	// by role across rebuilds like the display state; a role that vanishes
	// (a flap removed, a blade count lowered) parks its structure and gets
	// it back if the role returns. Every entity's bodies carry their own,
	// boolean results included (no inheritance any more: a fused airframe's
	// body is given its structure directly).
	//
	// SetBodyStructure broadcasts OnBodyStructureChanged with bGeometry true
	// when a regeneration would give different geometry (the skin, a
	// network's planes, the exclusions -- see SameStructureGeometry); a
	// material or an asserted mass alone is read
	// on demand by the analyses and the mass report and broadcasts false.
	int32 FindBodyIndex(const FString& BodyRole) const;
	// Null when there is no such body (or the role is empty and the entity
	// has more than one body; an empty role on a single-body entity is that
	// body).
	const aerotelic::BodyStructure* GetBodyStructure(const FString& BodyRole) const;
	const aerotelic::BodyStructure& GetBodyStructure(int32 Index) const;
	// False when there is no such body.
	bool SetBodyStructure(const FString& BodyRole, const aerotelic::BodyStructure& InStructure);
	DECLARE_MULTICAST_DELEGATE_ThreeParams(FOnAerotelicBodyStructureChanged, AAerotelicEntityActor*, const FString& /*Role*/, bool /*bGeometry*/);
	static FOnAerotelicBodyStructureChanged OnBodyStructureChanged;
	// The placement as the plane geometry wants it: a point x of the
	// parameterisation's frame sits at OutScale * R x + T in the world
	// (feature_tree.hpp RotationMatrix; T the actor's location).
	void GetPlacementMatrix(std::array<double, 9>& OutR, aerotelic::Vec3& OutT, double& OutScale) const;

	// ---- generated structure (the layers) ----

	// The generated geometry of a body's structure, null until the document
	// has generated it (UAerotelicDocumentSubsystem::GenerateBodyStructure);
	// Set hands a fresh generation over and redraws the layers, Clear drops
	// them. Both broadcast OnBodiesChanged (the tree's rows change).
	const FAerotelicBodyStructure* GetGeneratedStructure(const FString& BodyRole) const;
	void SetGeneratedStructure(const FString& BodyRole, TSharedPtr<FAerotelicBodyStructure> InGenerated);
	void ClearGeneratedStructure(const FString& BodyRole);
	// Every generated layer of every body, in body order then layer order:
	// what the Scene tree lists under the body rows and the pickers offer.
	const TArray<FAerotelicLayerView>& GetLayers() const { return Layers; }
	int32 FindLayer(const FString& BodyRole, const FString& LayerName) const;
	// "<entity>-<n> / <layer>", and with "-mirror" for the mirrored half.
	FString GetLayerDisplayName(int32 Layer) const;
	FString GetMirrorLayerDisplayName(int32 Layer) const;
	void SetLayerVisible(int32 Layer, bool bVisible);
	bool IsLayerVisible(int32 Layer) const;
	void SetMirrorLayerVisible(int32 Layer, bool bVisible);
	bool IsMirrorLayerVisible(int32 Layer) const;
	void SetLayerColor(int32 Layer, TOptional<FLinearColor> InColor);
	// Its own colour, else the layer kind's default (the skin the body's
	// colour, ribs steel blue).
	FLinearColor GetLayerDisplayColor(int32 Layer) const;
	// A body's generated layers that have a mesh -- the Ribs layers, open
	// discs -- as placed (or mirrored) meshes, named by layer: what STL
	// export and the mass report take. The Skin layer has no mesh (the
	// body's own placed mesh is the skin) and is not listed. Empty when the
	// body has none.
	std::vector<aerotelic::NamedMesh> GetPlacedLayerMeshes(const FString& BodyRole, bool bMirror) const;

	// ---- the old per-entity setting, as a bridge ----
	// The one MassSpec a parameterisation used to carry, read and written
	// through its bodies: Set gives every body the structure the spec
	// amounts to (an asserted total split between them by area), Get is the
	// first body's. Kept for the commandlets and the FEA dialog while they
	// move to the per-body API.
	void SetMassSpec(const aerotelic::MassSpec& InSpec);
	aerotelic::MassSpec GetMassSpec() const;
	// Built on other entities (a boolean result).
	bool IsDerived() const;
	// The spec that applies to the first body: its own when set; for a
	// boolean result whose bodies carry nothing, the first operand's.
	aerotelic::MassSpec GetEffectiveMassSpec() const;
	// The entity the effective spec comes from (this one when its own), or
	// null.
	const AAerotelicEntityActor* GetMassSpecSource() const;

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

	// A reflected copy of the placed shape in the entity's MIRROR PLANE,
	// derived from it exactly as WingDesigner's MirrorFeature is: never
	// edited on its own, it always follows this entity (including its
	// placement, so a mirrored propeller comes out the opposite hand). The
	// plane is the scene tree's Mirror dialog's (2026-11-10): any normal and
	// an offset along it, in project units; the default is the aircraft's
	// symmetry plane y = 0, the only plane there was before.
	void SetMirrored(bool bInMirrored);
	bool IsMirrored() const { return bMirrored; }
	const FAerotelicMirrorPlane& GetMirrorPlane() const { return MirrorPlane; }
	// Sets the plane (and whether the mirror is on): the mirror redraws and
	// every boolean over it rebuilds.
	void SetMirrorPlane(const FAerotelicMirrorPlane& Plane, bool bInMirrored);
	// The plane in WORLD (placed) coordinates: the normal, unit, and a point
	// on it (the offset scaled with the model).
	aerotelic::Vec3 MirrorNormal() const;
	aerotelic::Vec3 MirrorPoint() const;
	// Reflect placed geometry in the plane.
	void ReflectMesh(aerotelic::TriMesh& Mesh) const;
	TopoDS_Shape ReflectShape(const TopoDS_Shape& Placed) const;
	FVector ReflectPoint(const FVector& P) const;
	void SetMirrorBodyVisible(int32 Index, bool bVisible);
	bool IsMirrorBodyVisible(int32 Index) const;
	FString GetMirrorBodyDisplayName(int32 Index) const;

	// Fired after the set of bodies (or the mirrored state) of an entity
	// changed, so the scene tree can re-sync its child rows. Static: the
	// tree outlives any one entity.
	DECLARE_MULTICAST_DELEGATE_OneParam(FOnAerotelicBodiesChanged, AAerotelicEntityActor*);
	static FOnAerotelicBodiesChanged OnBodiesChanged;

	// Fired after an entity's placed geometry changed (a rebuild after an
	// applied edit, or a move). The document subsystem queues every boolean
	// result and generated structure over it and rebuilds them one per tick
	// (feature_tree.py's dirty propagation, paced so the window repaints).
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
	// enough), else the OCCT solid tessellated. What STL export, the mass
	// report and the structure's generation consume. Empty if nothing can
	// be built.
	//
	// A Tolerance of 0 or less asks for THE BODY'S OWN GRID (2026-11-05):
	// the mesh the viewport shows, built by the entity's Mesh tab spec at
	// the viewport's tolerance rule (DisplayTolerance, 0.15% of the body's
	// diagonal) -- so an analysis that asks for it solves on exactly the
	// triangles the user sees and set. The structural analysis, the
	// structure's generation (its ribs' rims are this grid's vertices),
	// mass properties and the material dialog ask for it; a caller with a
	// tolerance of its own (STL export) gets a private copy at that
	// tolerance and must not hand it to a solver.
	std::vector<aerotelic::NamedMesh> GetPlacedMeshes(bool bMirror, double Tolerance = 0.0) const;
	// World bounds of the placed (or mirrored) bodies; invalid if none.
	FBox GetPlacedBounds(bool bMirror) const;

	// This actor's placement as the backend's feature tree wants it: the
	// full-scale translation and rotation as authored, and the project's
	// model scale (GetModelScale) for Place() to apply.
	aerotelic::EntityTransform GetEntityTransform() const;
	// The project's model scale (Model > Scale; the document's). The
	// model stays the full-scale aircraft: the actor is placed at the
	// scaled translation with a uniform scale, so everything read off the
	// actor's transform -- the viewport, GetPlacedMeshes, GetPlacedBounds,
	// GetPlacedShape -- is the replica. 1 outside a document.
	static double GetModelScale();
	// Applies a placement -- from a loaded project file, the clipboard, or
	// the Transform dialog -- and makes it this entity's stored one. The
	// placement is FULL-SCALE; the actor's transform is it scaled by the
	// model scale, so the document re-applies every placement when the
	// scale changes.
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

	// The generated structure's layers (skin shells, ribs), one
	// section per layer, in the body's frame under the actor like ProxyMesh
	// -- the layers are generated in the body's own frame, so a move needs
	// no rebuild -- and the mirrored copy in world space like MirrorMesh.
	// No collision: the body's own surface is the pick geometry.
	UPROPERTY(VisibleAnywhere, Category = "Aerotelic")
	TObjectPtr<class UProceduralMeshComponent> StructureMesh;

	UPROPERTY(VisibleAnywhere, Category = "Aerotelic")
	TObjectPtr<class UProceduralMeshComponent> MirrorStructureMesh;

	UPROPERTY(VisibleAnywhere, Category = "Aerotelic")
	TObjectPtr<class UProceduralMeshComponent> StructureEdgeMesh;

	UPROPERTY(VisibleAnywhere, Category = "Aerotelic")
	TObjectPtr<class UProceduralMeshComponent> MirrorStructureEdgeMesh;

	UPROPERTY()
	TArray<TObjectPtr<class UMaterialInstanceDynamic>> LayerMaterials;

	UPROPERTY()
	TArray<TObjectPtr<class UMaterialInstanceDynamic>> MirrorLayerMaterials;

	UPROPERTY()
	TArray<TObjectPtr<class UMaterialInstanceDynamic>> LayerEdgeMaterials;

	UPROPERTY()
	TArray<TObjectPtr<class UMaterialInstanceDynamic>> MirrorLayerEdgeMaterials;

	UPROPERTY()
	TArray<FAerotelicBody> Bodies;

	// The layers as drawn and listed (see FAerotelicLayerView), rebuilt by
	// RefreshStructureLayers from the bodies' generated structures; their
	// display state survives a regeneration by "<role>|<layer>".
	TArray<FAerotelicLayerView> Layers;
	TMap<FString, FAerotelicLayerView> LayerDisplayStates;
	bool bHasLayers = false;
	bool bMirrorHasLayers = false;

	aerotelic::project_yaml::EntityType EntityType = aerotelic::project_yaml::EntityType::Primitive;
	std::unique_ptr<aerotelic::ISolidModel> Model;
	bool bSelected = false;
	bool bMirrored = false;
	FAerotelicMirrorPlane MirrorPlane;
	// Whether the surface/mirror meshes hold any triangles (else the box
	// fallback, or nothing, is on screen).
	bool bHasTessellation = false;
	bool bMirrorHasTessellation = false;
	EAerotelicRenderStyle RenderStyle = EAerotelicRenderStyle::Shaded;
	TOptional<FLinearColor> ColorOverride;
	// Structures of body roles that are not currently produced (a removed
	// flap), by role, restored if the role comes back. Not saved.
	TMap<FString, aerotelic::BodyStructure> OrphanedStructures;
	FDelegateHandle TransformHandle;
	// The model's solid from the last build that needed OCCT (unplaced):
	// a boolean result, or a body whose model has no mesh builder. Null
	// while the viewport shows the model's own meshes; GetPlacedShape()
	// builds it on demand then.
	TopoDS_Shape LastSolid;
	// The model's own meshes from the last RefreshProxy() (unplaced), at
	// LastMeshTolerance; the mirror, export and the structure's generation
	// reuse them.
	std::vector<aerotelic::NamedMesh> LastMeshes;
	std::vector<aerotelic::WingBodyStructurePlan> LastStructurePlans;
	double LastMeshTolerance = 0.0;
	FString LastBuildError;
	// Local bounds of what was last shown, for the next build's tolerance.
	FBox LastLocalBounds;

	void EnsureProxyEdges();
	void ApplyColors();
	void ApplyMeshVisibility();
	// Every section's visibility -- the bodies' and the layers', on the
	// surface, edge and mirror meshes -- from the bodies' and layers' flags
	// (a body's section hidden while its Skin layer is).
	void ApplySectionVisibility();
	// Rebuilds the layer list and the structure meshes (the rib discs, both
	// sides) from the bodies' generated structures (and the mirrored copy).
	void RefreshStructureLayers();
	void RefreshStructureMirror();
	// True when body Index's own surface is to be drawn (on the mirror when
	// bMirror): the body's flag, and -- when the body has a generated Skin
	// layer, found by kind since it has no section -- that layer's flag.
	bool IsBodySurfaceShown(int32 BodyIndex, bool bMirror) const;
	// The wireframe material's nudge towards the camera, a fraction of this
	// body's PLACED size (so a replica's edges sit on its surface as the
	// full-size vehicle's do), on every edge material. After every rebuild
	// and every move or rescale.
	void ApplyEdgeOffset();
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
	// Writes one body's (or layer's) triangles as section Section of Target
	// and EdgeTarget, reusing or creating its material instances. The
	// bodies' sections carry collision (the pick geometry); the layers' do
	// not (often millions of triangles, and the body underneath is picked).
	void CreateBodySection(int32 Section, const TArray<FVector>& Vertices, const TArray<int32>& Triangles, const TArray<FVector>& Normals,
		class UProceduralMeshComponent* Target, TArray<TObjectPtr<class UMaterialInstanceDynamic>>& Materials,
		class UProceduralMeshComponent* EdgeTarget, TArray<TObjectPtr<class UMaterialInstanceDynamic>>& EdgeMaterialsOut,
		TArray<TObjectPtr<class UMaterialInstanceDynamic>>& NewMaterials, TArray<TObjectPtr<class UMaterialInstanceDynamic>>& NewEdgeMaterials,
		bool bCollision = true);
	// Rebuilds MirrorMesh from the placed-then-reflected shape. Cheap no-op
	// when the entity is not mirrored.
	void RefreshMirror();
	void OnRootTransformed(class USceneComponent* Component, EUpdateTransformFlags Flags, ETeleportType Teleport);
};
