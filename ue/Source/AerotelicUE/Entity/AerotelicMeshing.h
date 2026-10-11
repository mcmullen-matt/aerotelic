#pragma once

#include "CoreMinimal.h"
#include <vector>

#include "aerotelic/mesh.hpp"

#include <TopoDS_Shape.hxx>

// Conversions between the core's triangle meshes (mesh.hpp -- the OCCT-free
// loft triangulation every lofted model builds), OCCT shapes (for the
// bodies that still need OCCT: boolean results, nacelles with chevrons)
// and the engine's procedural-mesh arrays.
namespace AerotelicMeshing
{
	// Tessellates Shape with OCCT (BRepMesh at the given linear deflection
	// in model units and angular deflection in degrees), one welded mesh per
	// top-level solid, in the shape's own order. Names are empty; the caller
	// assigns roles.
	std::vector<aerotelic::NamedMesh> MeshesFromShape(const TopoDS_Shape& Shape, double LinearDeflection, double AngularDeflectionDeg = 20.0);

	// Applies an actor's placement (rotation then translation; scale is
	// never used for entities) to every vertex.
	void PlaceMesh(aerotelic::TriMesh& Mesh, const FTransform& Transform);
	// Reflects about the world Y = 0 plane (the mirrored half of an entity).
	void MirrorY(aerotelic::TriMesh& Mesh);

	// The arrays UProceduralMeshComponent::CreateMeshSection takes: every
	// triangle emitted in both windings (so the engine's left-handed
	// convention shows the surface from either side) and smooth normals
	// accumulated from one winding.
	void ToProceduralMesh(const aerotelic::TriMesh& Mesh, TArray<FVector>& OutVertices, TArray<int32>& OutTriangles, TArray<FVector>& OutNormals);

	FBox BoundsOf(const aerotelic::TriMesh& Mesh);
}
