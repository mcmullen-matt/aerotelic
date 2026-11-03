#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "AerotelicColoredMeshActor.generated.h"

class UProceduralMeshComponent;
class UMaterialInstanceDynamic;

// A triangle mesh with a colour at every vertex -- a scalar field painted
// on a surface (a pressure coefficient, a stress, a Mach number), drawn by
// the engine through M_AerotelicVertexColor and lit by the result view's
// eye-fixed rig (lighting channel 2, AAerotelicSceneViewActor) rather than
// the scene's studio rig, so no side of it is ever in shadow.
// Nothing about it is specific to one analysis: the post-processing views
// (SAerotelicSceneView) show one of these per result and recolour it when
// the colour map or the range changes, without rebuilding the geometry.
//
// By default it is visible only to scene captures (the result views), never
// in the main 3D view, so a result never masquerades as scene geometry.
// Transient: never saved with a project.
UCLASS()
class AAerotelicColoredMeshActor : public AActor
{
	GENERATED_BODY()

public:
	AAerotelicColoredMeshActor();

	// Replaces the mesh (one section). Vertices in world units; one colour
	// per vertex; triangles in the engine's index convention. No collision.
	void SetMesh(const TArray<FVector>& Vertices, const TArray<int32>& Triangles, const TArray<FVector>& Normals, const TArray<FColor>& Colors);
	// Recolours the mesh in place (Colors.Num() must match the vertices).
	void SetColors(const TArray<FColor>& Colors);
	void Clear();

	bool HasMesh() const { return Vertices.Num() > 0; }
	// World bounds of the mesh (invalid when empty).
	FBox GetMeshBounds() const;
	UProceduralMeshComponent* GetMesh() const { return Mesh; }

	// Visible to scene captures only (default) or in the main view as well.
	void SetVisibleInCapturesOnly(bool bCapturesOnly);

	// Draws the triangles' edges over the surface (the main view's "Shaded
	// with Edges": the same triangles again through the wireframe material
	// M_AerotelicWire, unlit, in the colour given) -- a finite-element
	// mesh wants its elements seen. Off by default; survives SetMesh.
	void SetEdges(bool bShow, const FLinearColor& Color = FLinearColor(0.02f, 0.02f, 0.03f));

	// "/Game/Materials/M_AerotelicVertexColor", authored by the material
	// setup commandlet: lit, two sided, base colour from the vertex colour.
	// Falls back to the engine's unlit VertexColorMaterial when missing.
	static FString VertexColorMaterialPackage();

	// Vertex colours are read by the GPU as linear values (no sRGB decode
	// happens on a vertex stream), so a display colour has to be linearised
	// before it is stored. This does that for a colour given in sRGB 0..1.
	static FColor VertexColorFromSRGB(float R, float G, float B);

private:
	UPROPERTY()
	TObjectPtr<UProceduralMeshComponent> Mesh;

	UPROPERTY()
	TObjectPtr<UMaterialInstanceDynamic> Material;

	// The edge copy (SetEdges): the same section through the wire material.
	UPROPERTY()
	TObjectPtr<UProceduralMeshComponent> Edges;

	UPROPERTY()
	TObjectPtr<UMaterialInstanceDynamic> EdgeMaterial;

	bool bEdges = false;
	FLinearColor EdgeColor = FLinearColor(0.02f, 0.02f, 0.03f);

	// Kept for UpdateMeshSection, which wants the positions again.
	TArray<FVector> Vertices;
	TArray<FVector> Normals;
	TArray<int32> Triangles;

	void RefreshEdges();
};
