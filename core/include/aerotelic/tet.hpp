#pragma once

// Tetrahedral meshing of the bodies' interiors: a constrained Delaunay
// tetrahedralization (CDT) that conforms exactly to a body's surface
// triangulation, and the skin / core partition of it a finite-element
// model wants (the skin as its own material, the core as the volume the
// lattice's orthotropic material card applies to).
//
// The machinery, in the order the pipeline runs it:
//
//   1. Delaunay tetrahedralization (DelaunayTetrahedralize). Incremental
//      Bowyer-Watson: the points are inserted in Morton order into a
//      tetrahedron enclosing them all; each insertion walks from the last
//      tetrahedron made to the one holding the point, grows the cavity of
//      tetrahedra whose circumsphere contains the point breadth-first
//      across faces, and replaces it by the fan from the point to the
//      cavity's boundary faces. Every geometric decision -- which side of
//      a face, inside which sphere -- is made by Shewchuk's adaptive exact
//      predicates (third_party/predicates), so cospherical and coplanar
//      configurations (a loft's rings, mirrored halves, planar caps) are
//      decided consistently and never produce an inverted tetrahedron; a
//      point on an existing sphere counts as inside it.
//
//   1b. Planar regions. Groups of adjacent coplanar surface triangles (the
//      planar end caps of a loft, which are ear-clipped: needle triangles
//      whose long diagonals pass within microns of other cap vertices, so
//      no sphere through them can be empty and recovery would split them
//      without end) are re-triangulated in their plane by edge flips to
//      the region's constrained Delaunay triangulation
//      (RetriangulatePlanarRegions). A planar region is then treated as a
//      facet: only its rim is a constraint, its interior edges are free,
//      and once the segments are in, the triangulation the mesh itself
//      draws across the region's plane is adopted wherever it tiles the
//      region (a symmetric section's cap is cocircular everywhere, so the
//      Delaunay mesh may pick either diagonal of each quad; insisting on
//      one would split segments for ever). Where the mesh does not tile a
//      region, points are put inside it where tetrahedra cross it. The
//      region's boundary and its vertices are unchanged, so the surface is
//      the same set of points; only the triangles drawn across a flat
//      region differ from the input's.
//
//   2. Segment recovery. Every edge of the surface must be an edge of the
//      mesh. An edge that is not is split (the point is inserted, the two
//      surface triangles on the edge are split with it), and its pieces
//      re-checked; a segment short enough against the surrounding
//      vertices is strongly Delaunay and always present (Shewchuk 2002).
//      The split point is at a power-of-two distance from the edge's
//      surface vertex rather than at the midpoint (concentric shells), so
//      that where two sheets of the surface meet at a small angle -- a
//      trailing edge, a knife-edged tail -- the Steiner points of both
//      sheets lie on common spheres about the shared vertex and stop
//      encroaching each other. Steiner points therefore lie on surface
//      edges only, and the outer faces of the result are sub-triangles of
//      the input triangles, in their planes.
//
//   3. Face recovery. With all edges present, a surface triangle F that is
//      not a face of the mesh is crossed by mesh edges. The cavity of
//      tetrahedra crossed by F is removed; F cuts it into two halves; each
//      half is re-tetrahedralized by the Delaunay tetrahedralization of
//      its own vertices, kept where it reproduces every face of the half's
//      boundary and enlarged by the tetrahedron across any face it misses
//      until it does (Si & Gartner 2005). This is where a flip-based
//      recovery would fail -- a half may be a polyhedron no flip sequence
//      can reach -- and it needs no interior Steiner point as long as the
//      segments are strongly Delaunay. Where it still cannot succeed (a
//      coplanar neighbour inside F's circumcircle, which no sphere through
//      F can exclude), the point where a crossing edge meets F is inserted
//      on F, which is split into sub-triangles, and the pieces are
//      recovered in turn.
//
//   4. Exterior removal. Starting from the enclosing tetrahedron, regions
//      of tetrahedra are flooded without crossing surface faces; crossing
//      one steps the depth, and the tetrahedra at odd depth are the
//      interior (so a hollow body's cavity is exterior). The sum of the
//      kept tetrahedra's volumes equals the surface's enclosed volume
//      exactly -- the conformity check the tests apply.
//
//   Degradation. A surface can be locally degenerate (a zero-thickness
//   wedge whose sliver triangles fold onto their neighbours) so that no
//   splitting rule conforms to it; a segment whose pieces reach the length
//   floor is then abandoned, the triangles on it are left unrecovered,
//   both are counted in the stats, and the interior is classified by the
//   surface's distance field instead of by crossing parity, so the mesh
//   boundary deviates from the surface only around the abandoned faces.
//
//   5. Skin / core (BuildSkinCoreMesh). The surface's inward offset nodes
//      (shell.hpp's field-guided walk, at depth -t) and a body-centred
//      cubic lattice of core points are added to the point set before
//      step 1. After step 4, every tetrahedron edge whose ends straddle the
//      iso-surface b = -t of the body's distance field is split where it
//      crosses (the crossing located on the field by bisection), or an
//      interior end near the crossing is snapped onto it when that
//      inverts nothing; afterwards no edge straddles, so every tetrahedron
//      lies wholly in the skin (b >= -t at its nodes) or the core, and the
//      faces between the two classes form a conforming interface on the
//      iso-surface. Surface nodes never move; the skin's outer faces are
//      the surface itself.
//
// Conventions. Tetrahedron (a, b, c, d) is stored with Dot(b-a, Cross(c-a,
// d-a)) > 0 (positive volume, as shell.hpp). Face i is the face opposite
// node i; TetMesh::kFace gives its nodes in outward (counter-clockwise
// from outside) order. neighbors[t][i] is the tetrahedron across face i,
// -1 on the mesh boundary.

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "aerotelic/mesh.hpp"
#include "aerotelic/sdf.hpp"
#include "aerotelic/vec3.hpp"

namespace aerotelic {

struct TetMesh {
	std::vector<Vec3> nodes;
	std::vector<std::array<int, 4>> tets;
	std::vector<std::array<int, 4>> neighbors;
	// Bit i set: face i lies on a constraint surface (the body surface).
	std::vector<uint8_t> constrained;
	// Per tetrahedron: 0 unclassified, 1 skin, 2 core.
	std::vector<int> material;

	// Nodes of face i of a tetrahedron, outward-oriented.
	static constexpr int kFace[4][3] = {{1, 2, 3}, {0, 3, 2}, {0, 1, 3}, {0, 2, 1}};

	bool IsEmpty() const { return tets.empty(); }
	double TetVolume(int t) const;
	double Volume() const;
	double Volume(int material) const;
	int Count(int material) const;
	// The faces with no neighbour, or whose neighbour has another material
	// (material < 0: the whole mesh's boundary), outward-oriented.
	TriMesh BoundaryFaces(int material = -1) const;
	// The faces between two materials (both orientations once: as seen
	// from material a).
	TriMesh InterfaceFaces(int material_a, int material_b) const;
	// Smallest dihedral angle (degrees) and worst radius ratio
	// (3 * inradius / circumradius, 1 for the regular tetrahedron) over
	// the tetrahedra of a material (< 0: all).
	void Quality(int material, double* min_dihedral_deg, double* min_radius_ratio, int* n_below_5deg) const;
	// Re-derives `neighbors` from the tetrahedra (every face shared by at
	// most two); returns false if a face is shared by three or more.
	bool RebuildNeighbors();
	// True if every tetrahedron has positive volume and the neighbour
	// table is consistent with the faces.
	bool IsValid(std::string* why = nullptr) const;
};

struct TetStats {
	int n_points = 0;
	int n_tets = 0;
	int n_walk_steps = 0;        // point-location steps in total
	int n_walk_fallbacks = 0;    // locations that had to scan every tetrahedron
	int n_cavity_repairs = 0;    // tetrahedra dropped from cavities to keep them star-shaped
	double seconds = 0.0;
};

// Delaunay tetrahedralization of the convex hull of `points` (duplicate
// points are skipped: a node with no tetrahedron). Throws
// std::invalid_argument on fewer than four non-coplanar points.
TetMesh DelaunayTetrahedralize(const std::vector<Vec3>& points, TetStats* stats = nullptr);

struct ConformingOptions {
	// Planar regions: triangles whose vertices all lie within this fraction
	// of the surface's bounding-box diagonal of a neighbour's plane are
	// re-triangulated together (0 disables).
	double planar_tolerance = 1e-9;
	// Segment recovery: no surface edge is split into a piece shorter than
	// this fraction of the surface's bounding-box diagonal (a runaway
	// cascade of splits is reported as a failure instead).
	double min_segment_fraction = 1e-6;
	// A surface node within this fraction of the bounding-box diagonal of a
	// missing segment's line (projecting well inside it) is made a node of
	// the segment instead of being split away from: the surface moves by
	// less than this, and no near-twin node is planted beside it.
	double segment_snap_fraction = 1e-4;
	// Face recovery: how far a half-cavity may be enlarged (tetrahedra)
	// before a Steiner point on the face is used instead.
	int max_cavity_tets = 400;
	// Hard cap on Steiner points on faces per input triangle.
	int max_face_steiner = 64;
	// Budget for Steiner points of all kinds, as a multiple of the number
	// of surface vertices; once spent, what is still missing is abandoned.
	double max_steiner_factor = 10.0;
	// Time budget for boundary recovery in seconds (0: none); once spent,
	// what is still missing is abandoned.
	double max_seconds = 0.0;
	// Interior points to include in the point set (inserted before the
	// surface vertices are recovered; must lie strictly inside the body).
	std::vector<Vec3> interior_points;
};

struct ConformingStats {
	int n_surface_vertices = 0;
	int n_surface_triangles = 0;
	int n_interior_points = 0;
	int n_planar_regions = 0;        // regions of two or more coplanar triangles
	int n_planar_flips = 0;          // edge flips made re-triangulating them
	int n_regions_adopted = 0;       // regions whose triangulation was taken from the mesh's own faces
	int n_region_steiner = 0;        // points put inside planar regions
	int n_segments_missing = 0;      // after the Delaunay tetrahedralization
	int n_edge_steiner = 0;          // points put on surface edges
	int n_segment_snaps = 0;         // segments bent through a node lying on them
	int n_segment_flips = 0;         // sliver surface triangles flipped away from a segment
	int n_flips_refused = 0;         // slivers that could not be flipped (the new edge exists)
	int n_faces_missing = 0;         // after segment recovery
	int n_cavity_recoveries = 0;     // faces recovered by cavity re-tetrahedralization
	int n_cavity_enlargements = 0;
	int n_face_steiner = 0;          // points put on surface faces
	int n_segments_abandoned = 0;    // segments given up on (split down to the floor); the surface is not exactly conformed there
	int n_faces_abandoned = 0;       // surface triangles not recovered (on an abandoned segment, or a recovery that failed)
	int n_surface_faces = 0;         // constraint faces in the result (sub-triangles)
	int n_tets_interior = 0;
	int n_tets_exterior = 0;
	double surface_volume = 0.0;     // of the input surface
	double mesh_volume = 0.0;        // of the interior tetrahedra
	double seconds_delaunay = 0.0, seconds_segments = 0.0, seconds_faces = 0.0, seconds_total = 0.0;
	TetStats delaunay;
};

// The constrained Delaunay tetrahedralization of the interior of a
// closed, outward-oriented surface mesh, conforming to it (steps 1-4
// above). `surface` may have several shells (a hollow body); nodes 0 ..
// n-1 of the result are the surface's vertices in order, then the
// interior points, then the Steiner points. Throws std::invalid_argument
// on an open surface, std::runtime_error when recovery fails (with where).
TetMesh TetrahedralizeInterior(const TriMesh& surface, const ConformingOptions& options = {}, ConformingStats* stats = nullptr);

struct SkinCoreOptions {
	// Skin thickness t, model units.
	double thickness = 0.02;
	// Spacing of the body-centred cubic core points (0: none; then the
	// core is meshed from the surface and offset nodes alone).
	double core_spacing = 0.0;
	// An interior node within this fraction of an edge's length of the
	// crossing is snapped onto the iso-surface instead of splitting the
	// edge.
	double snap_fraction = 0.3;
	// Bisection tolerance for the crossing, as a fraction of t.
	double crossing_tolerance = 1e-4;
	// Whether to add the surface's inward offset nodes (shell.hpp walk) to
	// the point set.
	bool offset_nodes = true;
	ConformingOptions conforming;
};

struct SkinCoreStats {
	ConformingStats conforming;
	int n_offset_nodes = 0;
	int n_core_points = 0;
	int n_edges_split = 0;
	int n_nodes_snapped = 0;
	int n_skin_tets = 0, n_core_tets = 0;
	double skin_volume = 0.0, core_volume = 0.0;
	double min_dihedral_skin = 0.0, min_dihedral_core = 0.0;
	double min_radius_ratio_skin = 0.0, min_radius_ratio_core = 0.0;
	int n_slivers_skin = 0, n_slivers_core = 0; // dihedral below 5 degrees
	double seconds = 0.0;
};

// Steps 1-5: the skin/core tetrahedral mesh of a closed surface over its
// distance field (MeshSdf of the same surface). material 1 = skin, 2 =
// core. Throws as TetrahedralizeInterior.
TetMesh BuildSkinCoreMesh(const TriMesh& surface, const MeshSdf& sdf, const SkinCoreOptions& options, SkinCoreStats* stats = nullptr);

// Step 1b on its own: flips the edges inside each planar region of a
// closed surface mesh to the region's constrained Delaunay triangulation
// (in the region's plane). Returns the number of flips; `n_regions` the
// number of multi-triangle regions found.
int RetriangulatePlanarRegions(TriMesh& surface, double plane_tolerance, int* n_regions = nullptr, std::vector<int>* region_of_triangle = nullptr);

// Body-centred cubic points of the given spacing inside the box, kept
// where keep(p) is true.
std::vector<Vec3> BccPoints(const Box3& box, double spacing, const std::function<bool(const Vec3&)>& keep);

// Legacy ASCII VTK unstructured grid with the material as cell data.
bool WriteTetMeshVtk(const TetMesh& mesh, const std::string& path);

// Exact predicates on Vec3 (positive volume / inside sphere conventions
// of this header), for tests.
double ExactOrient(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d);
double ExactInSphere(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d, const Vec3& e);

} // namespace aerotelic
