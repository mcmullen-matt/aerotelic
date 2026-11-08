#pragma once

// Cutting a closed triangle mesh by planes so that each plane's section
// becomes a set of mesh EDGES (2026-11-08): what a rib embedded in a
// finite-element model needs. The rib is a planar shell inside the body;
// its rim must lie on the skin's nodes so the two share degrees of
// freedom, and that is only possible if the surface mesh has vertices
// exactly along the plane. CutMeshByPlanes puts them there: every
// triangle crossing a plane is split (into two or three) at the crossing
// points of its edges, one point per (edge, plane) so neighbours share
// it exactly, and a vertex within `tol` of a plane is snapped onto it
// rather than split a hair away from it. The surface stays closed and
// its volume changes by no more than the snaps move it (a few `tol`).
//
// SectionLoopsOnPlane then reads the rim back: the on-plane edges that
// separate the two sides, chained into loops of vertex ids, each loop
// counter-clockwise about the plane's normal where it bounds material
// (the same orientation rule as body_structure.hpp SectionByPlane, with a
// vertex exactly on the plane counted as positive).

#include <vector>

#include "aerotelic/body_structure.hpp"
#include "aerotelic/mesh.hpp"

namespace aerotelic {

struct MeshCutStats {
	int split_triangles = 0;    // input triangles that were split (summed over the planes)
	int snapped_vertices = 0;   // vertices moved onto a plane
	int new_vertices = 0;       // crossing points added
};

// `parent`, when given, receives the index of the input triangle each
// output triangle came from (a per-face quantity of the input -- a pressure
// from a flow solution -- is carried through it).
TriMesh CutMeshByPlanes(const TriMesh& mesh, const std::vector<Plane>& planes, double tol,
	std::vector<int>* parent = nullptr, MeshCutStats* stats = nullptr);

// The loops on plane `p` of a mesh cut by it (vertices within `tol` of the
// plane count as on it): each a closed chain of vertex ids, every
// consecutive pair an edge of the mesh. Empty when the plane misses the
// mesh or the mesh was not cut by it.
std::vector<std::vector<int>> SectionLoopsOnPlane(const TriMesh& cut, const Plane& p, double tol);

// The signed area of a loop of the mesh's vertices in the plane (positive
// counter-clockwise about the normal: an outer loop).
double LoopArea(const TriMesh& mesh, const std::vector<int>& loop, const Plane& p);

} // namespace aerotelic
