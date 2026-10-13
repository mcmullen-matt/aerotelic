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
//   4. Classification and assembly. The pieces of each body joined across
//      any edge that is not a seam segment lie on one side of the other
//      body; each such region is inside or outside by the sign of that
//      body's distance field (sdf.hpp) at the centroid of its piece
//      farthest from that body's surface, so a sliver hugging the seam
//      cannot be misjudged (a region whose pieces disagree beyond rounding
//      falls back to judging each piece by itself). Fuse keeps A outside B
//      and B outside A; Common keeps A inside B and B inside A; Cut (A
//      minus B) keeps A outside B and B inside A with its winding reversed.
//      Vertices with the same coordinates are then one vertex, and the
//      result must come out a closed manifold.
//   5. Seam slivers. Where the intersection curve passes close to a vertex
//      or an edge of a triangle it cuts, the cut leaves a sliver piece --
//      the one kind of triangle a tetrahedral mesh of the result cannot
//      conform to (tet.hpp). Sliver triangles touching the seam are
//      removed worst first: a needle by collapsing its shortest edge onto
//      its seam end (an original vertex onto the seam point on its own
//      edge, which moves nothing off either surface and moves the body's
//      surface by that edge's length; two seam points onto one, which
//      moves the seam by that length), a cap by flipping its long edge
//      within its own sheet (no vertex moves; the surface changes by the
//      quad's non-planarity). Every operation is bounded by a movement
//      budget and refused if it would invert a triangle, break the
//      manifold or exceed the budget.
//   6. Refused configurations, reported as MeshBooleanError with where:
//      two triangles overlapping in a common plane; a vertex of one body
//      exactly on a face of the other; an edge of one exactly in a face of
//      the other. The exact predicates make these exact findings. Chords
//      that only touch (an edge of one body crossing an edge of the other
//      exactly, as mirror-image edges do on a symmetry plane) are point
//      contacts and not refused. No vertex is nudged.
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

struct MeshBooleanOptions {
	// EXPERIMENTAL, off by default (0). Delaunay refinement of the pieces
	// of the cut triangles (Ruppert's algorithm with off-centres, splitting
	// constrained sub-edges shared with the neighbours) until none has an
	// angle under this many degrees: it removes the fans a far corner makes
	// to a dense seam, which no tetrahedral Delaunay mesh can hold, and
	// makes Boom + VT and VT + HT of the P-38 conform exactly -- but the
	// refinement points fall on skinny loft strips and thin regions whose
	// rows the tetrahedral mesher needs matched, so other junctions get
	// worse, and on the mirror-symmetric wing root (a seam a hair from a
	// metre-long cap edge) the splitting cascades and the 2-D recovery
	// fails. No point is put within refine_floor_fraction of the diagonal
	// of another; at most refine_max_points go in one triangle; a piece is
	// refined only if its shortest edge is under refine_grade times its
	// distance from the seam (0: no such limit).
	double refine_min_angle_deg = 0.0;
	double refine_floor_fraction = 1e-5;
	int refine_max_points = 400;
	double refine_grade = 0.5;
	// A triangle touching the seam with a smaller angle (degrees) is a
	// sliver the seam pass works on (0 disables the pass).
	double seam_sliver_deg = 1.0;
	// No collapse moves a vertex, and no flip changes the surface, by more
	// than this fraction of the bounding-box diagonal.
	double seam_move_fraction = 1e-4;
};

struct MeshBooleanStats {
	int n_coincident_pairs = 0;    // triangle pairs dropped as a shared wall (1)
	int n_pairs_tested = 0;        // triangle pairs whose boxes overlapped
	int n_pairs_intersecting = 0;
	int n_segments = 0;
	int n_triangles_cut = 0;       // triangles of either body re-triangulated
	int n_pieces = 0;              // pieces they were cut into
	int n_refine_points = 0;       // refinement points inside cut triangles
	int n_refine_splits = 0;       // refinement midpoints on constrained sub-edges
	int n_refine_rounds = 0;       // rounds of re-cutting until no shared sub-edge changed
	int n_kept_a = 0, n_kept_b = 0; // triangles (pieces included) kept from each body
	int n_seam_slivers = 0;        // sliver triangles touching the seam before the pass
	int n_seam_collapses = 0, n_seam_flips = 0, n_seam_refused = 0;
	int n_seam_slivers_left = 0;   // after it
	double max_seam_move = 0.0;    // the longest edge collapsed or the largest flip deviation
	double volume_a = 0.0, volume_b = 0.0, volume_result = 0.0;
	double seconds = 0.0;
};

// `a` and `b` must be closed, outward-oriented meshes (TriMesh::
// IsClosedManifold, positive SignedVolume); std::invalid_argument
// otherwise. Throws MeshBooleanError on a degenerate contact and
// std::runtime_error if the result does not close. An empty result (a
// Common of disjoint bodies, a Cut that removes everything) is an empty
// mesh, not an error.
TriMesh MeshBoolean(const TriMesh& a, const TriMesh& b, MeshBooleanOp op, MeshBooleanStats* stats = nullptr, const MeshBooleanOptions& options = {});

} // namespace aerotelic
