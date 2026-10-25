#pragma once

// OCCT-free surface triangulation for the lofted bodies.
//
// Every lofted body in this code base (wing, fuselage, both nacelles, the
// propeller's blades and spinner, and the sphere/cylinder primitives) is a
// chain of ruled surfaces between consecutive section curves. This module
// triangulates such a chain directly from the section curves, without
// building an OCCT solid first:
//
//   1. The section curves are discretised at ordered sets of along-curve
//      parameters v. A shared set, refined against every station, drives
//      the station refinement; each ring is then sampled at its OWN set,
//      refined against its own curve alone, so a small ring (a tail, a
//      blade tip) is not sampled as densely as the largest one.
//   2. Parameters are added wherever the straight chord between two
//      consecutive points deviates from the actual curve by more than the
//      tolerance (the chordal sag) -- so the linear representation the
//      triangles use is within tolerance of the curved surface along both
//      end curves of every ruled strip, which is where a ruled surface's
//      error lives.
//   3. Stations are added between two consecutive ones wherever the ruled
//      strip deviates from the surface (the section half way between them
//      is not the average of the two) by more than the tolerance, and
//      wherever the strip is longer than LoftOptions::max_aspect times its
//      rings' point spacing (needle triangles into an apex or knife edge).
//   4. The strip between two rings is zig-zagged by parameter: equal
//      parameters make a quadrilateral split along its shorter diagonal,
//      otherwise the ring whose next parameter is smaller advances with a
//      triangle; a collapsed edge (a ring shrunk to an apex) makes fans.
//   5. The open ends are capped with planar polygons (ear clipping), so
//      the result is a closed, consistently outward-oriented triangle
//      mesh -- what the viewport, the STL writer and the signed-distance
//      field (sdf.hpp) all consume.
//
// Tolerances are absolute, in model units (metres), like an STL linear
// deflection. The mesh is NOT the exact solid OCCT would build (no exact
// B-spline faces), but for a ruled loft the deviation is bounded by the
// tolerance by construction, and it is built in milliseconds rather than
// seconds.

#include <array>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "aerotelic/mat.hpp"
#include "aerotelic/vec3.hpp"

namespace aerotelic {

// An indexed triangle mesh. Triangles are counter-clockwise seen from
// outside once FlipIfInsideOut()/the loft builder has oriented them.
struct TriMesh {
	std::vector<Vec3> vertices;
	std::vector<std::array<int, 3>> triangles;

	bool IsEmpty() const { return triangles.empty(); }
	void Clear() { vertices.clear(); triangles.clear(); }
	// Appends another mesh (its vertex indices offset).
	void Append(const TriMesh& other);
	Box3 Bounds() const;
	// Divergence-theorem volume; positive when the triangles face outward.
	double SignedVolume() const;
	double SurfaceArea() const;
	void FlipWinding();
	// Reverses every triangle if SignedVolume() < 0.
	void FlipIfInsideOut();
	// Applies a 3x3 linear map (row-major) then a translation to every vertex;
	// reverses the winding when the map has a negative determinant (a mirror).
	void Transform(const std::array<double, 9>& linear, const Vec3& translation);
	void Translate(const Vec3& t);
	// Reflects through the plane of the given normal through `point`
	// (winding reversed so the result stays outward-facing).
	void Mirror(const Vec3& point, const Vec3& normal);
	// Merges vertices closer than `tol` and drops the degenerate triangles
	// this leaves. The loft builder shares vertices by construction and
	// needs none of this; it is for meshes assembled from independent faces
	// (OCCT's per-face triangulations) before they go into an SDF.
	void WeldVertices(double tol);
	// Drops triangles with a repeated vertex index or (near) zero area.
	void RemoveDegenerateTriangles(double area_eps = 0.0);
	// True if every edge is shared by exactly two triangles, traversed in
	// opposite directions -- a closed, consistently oriented surface.
	bool IsClosedManifold() const;
	Vec3 TriangleNormal(int t) const;
};

// A mesh with the role it plays in its entity ("Main", "Spinner", "Blade 1").
struct NamedMesh {
	std::string name;
	TriMesh mesh;
};

// A chain of ruled patches, described by the section curve at any station.
struct LoftSource {
	// Points of the section at station u, at the along-curve parameters v
	// (each in [0, 1]): an (n, 3) matrix, one row per v. The map must be
	// continuous in u over [u_min, u_max]; it may collapse a whole section
	// to one point (an apex: a fuselage nose, a spinner tip).
	std::function<Mat(double u, const std::vector<double>& v)> section;
	double u_min = 0.0, u_max = 1.0;
	// Stations that must be present (the user's sections, chord-law
	// breakpoints, the ends). The ends are added if missing.
	std::vector<double> u_seeds;
	// Along-curve parameters that must be present (0 and 1 are added if
	// missing; a corner of the section, e.g. a conic quadrant boundary).
	std::vector<double> v_seeds;
	// True when the section curve itself closes (v = 1 coincides with
	// v = 0, a periodic ring). False for an open curve such as an airfoil,
	// whose two ends are then joined by a straight segment (the trailing
	// edge) to close the section polygon.
	bool closed_curve = true;
	// True when the chain of stations closes back on itself (u_max
	// coincides with u_min, e.g. a cowl modelled as outer skin -> rim ->
	// inner skin -> rim). No end caps are then made.
	bool periodic_u = false;
	// Whether to cap the two ends (ignored for periodic_u, and for an end
	// whose section is an apex).
	bool cap_start = true, cap_end = true;

	// Thin shells. A body made of two sheets that meet at a knife edge --
	// an airfoil's upper and lower surfaces at the trailing edge, a cowl's
	// outer and inner skins at the lip and the trailing edge -- can only be
	// given a tetrahedral mesh that conforms to it (tet.hpp) if the two
	// sheets' vertex rows face each other exactly. A vertex of one sheet
	// under the middle of an edge of the other, nearer than that edge's
	// own sag, leaves the edge no empty circumsphere, and the mesher then
	// splits it without end down the wedge. So the two sides are sampled at
	// MATCHED parameters:
	//   mirror_v(u, v): the parameter of the point across the section from
	//     (u, v) -- for an airfoil, the lower-surface parameter at the same
	//     chordwise station as upper-surface parameter v (and back). Every
	//     ring's parameter set is closed under it. null: none.
	//   mirror_u: the stations are mirrored about this value -- u and
	//     mirror_u - u (wrapped into the period) are the two sides of the
	//     wall, e.g. a cowl parametrised by x fraction out along the outer
	//     skin and back along the inner -- and the two paired stations share
	//     one ring parameter set. NaN: none.
	std::function<double(double u, double v)> mirror_v;
	double mirror_u = std::numeric_limits<double>::quiet_NaN();

	// Parameters the first / last ring must contain in addition to its own
	// refined set (added after the mirror closure, not mirrored themselves):
	// how TriangulateLoftChain gives the two rings that meet at a break the
	// same vertices along their shared stretch. Ignored for periodic_u.
	std::vector<double> v_extra_start, v_extra_end;
};

// The first and last rings of a loft: the mesh's vertex indices, in ring
// order (ascending v), and their along-curve parameters.
struct LoftEndRings {
	std::vector<int> start, end;
	std::vector<double> v_start, v_end;
};

// A break between two consecutive segments of a chain (TriangulateLoftChain):
// the section changes shape at one station -- a wing's full section
// becoming the notched main element where a flap begins -- and the two
// rings there, the previous segment's last and the next's first, share a
// stretch (the skin forward of the cut) and differ elsewhere. The shared
// stretch is [va0, va1] on the previous ring and [vb0, vb1] on the next
// (inclusive, in each ring's own parameter), and a_to_b / b_to_a map one
// parameter onto the other (exact inverses). The two rings' remaining
// stretches, together, bound a planar polygon -- the pocket -- which closes
// the body across the break.
struct LoftBreak {
	double va0 = 0.0, va1 = 1.0, vb0 = 0.0, vb1 = 1.0;
	std::function<double(double)> a_to_b, b_to_a;
};

struct LoftOptions {
	// Maximum chordal deviation, in model units.
	double tolerance = 1e-3;
	// Minimum numbers of points around a section and of stations, before
	// error-driven refinement (so a near-straight curve still gets a
	// sensible polygon).
	int min_v = 8;
	int min_u = 2;
	// Hard caps on the shared sets (refinement stops when reached).
	int max_v = 1200;
	int max_u = 400;
	// Stations are also added wherever a ruled strip is longer than this
	// times the mean spacing of its rings' points: the fans into an apex
	// and the sheets converging on a knife edge otherwise come out as
	// needle triangles hundreds of times longer than wide, which a
	// tetrahedral mesh cannot conform to (0 disables).
	double max_aspect = 6.0;
	// Below this, a section is treated as collapsed to an apex.
	double apex_tolerance = 1e-9;
	// Reverse the triangles if the result's signed volume is negative. Off
	// for the segments of a chain, which are open (their signed volume
	// means nothing) and must keep the one winding their shared
	// parametrisation gives them; the assembled body is oriented once.
	bool orient_outward = true;
};

struct LoftStats {
	int n_stations = 0;
	int n_ring_points = 0;
	int v_rounds = 0, u_rounds = 0;
	bool v_capped = false, u_capped = false;
};

// Triangulates the loft. Throws std::invalid_argument on a malformed
// source (fewer than two stations, no section points).
TriMesh TriangulateLoft(const LoftSource& source, const LoftOptions& options, LoftStats* stats = nullptr, LoftEndRings* ends = nullptr);

// Triangulates a chain of segments (consecutive in u, each its own
// LoftSource with its own section) joined at breaks, as ONE closed body:
// each segment is lofted without caps at the breaks, the two rings at a
// break are given the same vertices along their shared stretch (by
// parameter correspondence, not by position), and the pocket between
// their remaining stretches is capped. The outer ends are capped as the
// first / last segment's cap_start / cap_end say. breaks.size() must be
// segments.size() - 1; no segment may be periodic_u. Throws
// std::runtime_error if a ring lacks the counterpart of a shared point
// (which the extra-parameter mechanism is meant to make impossible).
TriMesh TriangulateLoftChain(std::vector<LoftSource> segments, const std::vector<LoftBreak>& breaks, const LoftOptions& options, std::vector<LoftStats>* stats = nullptr);

// The ruled strip between two rings sharing their point count (and
// correspondence), without refinement: quads split along the shorter
// diagonal. `closed` joins the last point back to the first.
void AppendRuledStrip(TriMesh& mesh, const std::vector<int>& ring_a, const std::vector<int>& ring_b, bool closed);

// The strip between two rings sampled at their own along-curve parameters
// (ascending, both starting at v = 0): walking both by parameter, the ring
// whose next parameter is smaller advances with a triangle, equal
// parameters advance both with a quad (split as AppendRuledStrip does).
// Rings of equal parameter lists give AppendRuledStrip's result.
void AppendZigZagStrip(TriMesh& mesh, const std::vector<int>& ring_a, const std::vector<double>& va, const std::vector<int>& ring_b, const std::vector<double>& vb, bool closed);

// Triangulates a closed, simple (non-self-intersecting), near-planar
// polygon given by vertex indices into `mesh`, by ear clipping in its
// best-fit plane. Winding follows the polygon's order. Returns false
// (adding nothing) if the polygon is degenerate.
bool AppendPolygonCap(TriMesh& mesh, const std::vector<int>& ring);

// Refines a sorted parameter set in place so that the chordal sag of
// `evaluate` over every interval is within `tol` on every curve: the
// building block TriangulateLoft uses for both v and u. `evaluate(params)`
// returns one (n, 3) point set per curve. Returns the number of rounds.
int RefineChordal(std::vector<double>& params, double tol, int max_points,
	const std::function<std::vector<Mat>(const std::vector<double>&)>& evaluate, bool* capped = nullptr);

// The box, sphere/ellipsoid and cylinder primitives (centred on the origin,
// cylinder axis +X, as PrimitiveModel::BuildSolid makes them).
TriMesh BoxMesh(double lx, double ly, double lz);
TriMesh EllipsoidMesh(double a, double b, double c, double tolerance);
TriMesh CylinderMesh(double radius, double length, double tolerance);

} // namespace aerotelic
