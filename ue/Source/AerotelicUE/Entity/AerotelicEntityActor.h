#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include <memory>

#include "aerotelic/solid_model.hpp"
#include "aerotelic/project_yaml.hpp"

#include "AerotelicEntityActor.generated.h"

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

	// The real visual: BuildSolid()'s tessellated surface.
	UPROPERTY(VisibleAnywhere, Category = "Aerotelic")
	TObjectPtr<class UProceduralMeshComponent> ProxyMesh;

	// The mesh's material, instanced so the selection tint can be set per
	// entity. Created on first use from the engine's BasicShapeMaterial.
	UPROPERTY()
	TObjectPtr<class UMaterialInstanceDynamic> MeshMaterial;

	aerotelic::project_yaml::EntityType EntityType = aerotelic::project_yaml::EntityType::Primitive;
	std::unique_ptr<aerotelic::ISolidModel> Model;
	bool bSelected = false;

	void EnsureProxyEdges();
	void EnsureMeshMaterial();
	void ApplySelectionTint();
	void ShowBoxFallback(const TopoDS_Shape& Shape);
	// Returns false (and leaves ProxyMesh empty) if the shape has nothing
	// to tessellate -- caller falls back to ShowBoxFallback().
	bool RefreshTessellatedMesh(const TopoDS_Shape& Shape);
};
