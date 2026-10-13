#pragma once

// Boolean operations on closed triangle meshes, without OpenCascade: the
// union, difference and intersection of two bodies as one closed surface,
// built from the bodies' own triangles.
//
//   1. Coincident faces. A triangle of A whose three vertices are, to the
//      bit, the three vertices of a triangle of B, wound the opposite way,
//      is a shared wall between the two bodies -- the root caps of a wing
//      and its mirror image on the symmetry plane. Both are dropped and the
//      bodies join along the cap's rim; this is the union of two halves
//      that share a face, and it costs no geometry. The same vertices
//      wound the same way are two bodies overlapping on a common plane,
//      which is refused (see 5).
//   2. Intersections. Every pair of triangles whose boxes overlap (a
//      bounding-volume hierarchy over B) is tested with Shewchuk's exact
//      orientation predicates (Guigue & Devillers' method): the vertices of
//      each triangle against the other's plane, then the two chords where
//      each triangle crosses the other's plane, whose overlap along the
//      common line is the intersection segment. The chords' ends are
//      computed once per (edge, triangle) pair and reused, so the segments
//      of neighbouring triangles meet in the same points to the bit.
//   3. Cutting. Every intersected triangle is re-triangulated in its own
//      plane: a Delaunay triangulation of its corners and its segments'
//      ends (exact 2-D predicates on the coordinates with the plane's
//      dominant axis dropped), then the segments and the triangle's edges
//      recovered as constrained edges by flips. The pieces tile the
//      triangle exactly, and every point on the intersection curve is
//      shared, to the bit, by the pieces of both bodies that meet there.
//   4. Classification and assembly. Each piece (and each uncut triangle)
//      is inside or outside the other body by the sign of that body's
//      distance field at its centroid (sdf.hpp; the pieces never straddle
//      the other surface, so the centroid decides). Fuse keeps A outside B
//      and B outside A; Common keeps A inside B and B inside A; Cut (A
//      minus B) keeps A outside B and B inside A with its winding reversed.
//      Vertices with the same coordinates are then one vertex, and the
//      result must come out a closed manifold.
//   5. Refused configurations, reported as MeshBooleanError with where:
//      two triangles overlapping in a common plane; a vertex of one body
//      exactly on a face of the other; an edge of one exactly crossing an
//      edge of the other; and an intersection segment shorter than the
//      length floor. The exact predicates make the first three exact
//      findings. No vertex is nudged.
//
// Chained operations fold left to right: (A op B) op C.

#include <stdexcept>
#include <string>

#include "aerotelic/mesh.hpp"
#include "aerotelic/vec3.hpp"

namespace aerotelic {

enum class MeshBooleanOp { Fuse, Cut, Common };

// "fuse", "cut", "common" (the project file's spellings); throws
// std::invalid_argument on anything else.
MeshBooleanOp ParseMeshBooleanOp(const std::string& name);
const char* MeshBooleanOpName(MeshBooleanOp op);

// A degenerate contact between the two surfaces (5 above). `where` is a
// point of it.
class MeshBooleanError : public std::runtime_error {
public:
	MeshBooleanError(const std::string& what, const Vec3& where_) : std::runtime_error(what), where(where_) {}
	Vec3 where;
};

struct MeshBooleanStats {
	int n_coincident_pairs = 0;    // triangle pairs dropped as a shared wall (1)
	int n_pairs_tested = 0;        // triangle pairs whose boxes overlapped
	int n_pairs_intersecting = 0;
	int n_segments = 0;
	int n_triangles_cut = 0;       // triangles of either body re-triangulated
	int n_pieces = 0;              // pieces they were cut into
	int n_kept_a = 0, n_kept_b = 0; // triangles (pieces included) kept from each body
	double volume_a = 0.0, volume_b = 0.0, volume_result = 0.0;
	double seconds = 0.0;
};

// `a` and `b` must be closed, outward-oriented meshes (TriMesh::
// IsClosedManifold, positive SignedVolume); std::invalid_argument
// otherwise. Throws MeshBooleanError on a degenerate contact and
// std::runtime_error if the result does not close. An empty result (a
// Common of disjoint bodies, a Cut that removes everything) is an empty
// mesh, not an error.
TriMesh MeshBoolean(const TriMesh& a, const TriMesh& b, MeshBooleanOp op, MeshBooleanStats* stats = nullptr);

} // namespace aerotelic
