#pragma once

// The panel method ON THE BODY TRIANGULATION (2026-11-15): incompressible
// potential flow about closed bodies given as the triangle meshes the
// viewport draws, the structural shells are built on and the mass report
// weighs -- one mesh for every analysis, which is the point of it. A
// boolean result is a triangle mesh like any other, so a fused airframe
// is paneled as the one surface it is; nothing of one body lies inside
// another. The higher-order method of panel_method.hpp (Johnson's curved
// networks) stays as the validation reference.
//
// FORMULATION. Constant-strength source and doublet on every triangle,
// the Dirichlet (interior-potential) form of Morino: the perturbation
// potential is made zero at a point just inside every panel, the source
// strength is prescribed, sigma = -(n . V_inf), and the doublet strengths
// are the unknowns, mu = the perturbation potential on the outer surface
// (Katz & Plotkin, Low-Speed Aerodynamics, chapter 12; Morino & Kuo, AIAA
// J. 12 (1974) 191). The influence of a flat polygon is in closed form
// (Hess & Smith 1967): the solid angle for the doublet (Van Oosterom &
// Strackee's triangle formula), the edge logarithms and the solid angle
// for the source. Panels far from the point take the point source and
// dipole instead.
//
// WAKE AND KUTTA CONDITION. A body may name trailing-edge polylines (a
// wing's, a cowl rim's); every mesh edge lying on one with a triangle on
// each side sheds a flat wake strip along the stream of doublet strength
// mu_upper - mu_lower (the circulation of that section), folded into the
// two panels' columns. An edge with one triangle (a fin cut at the
// symmetry plane: its image is the other side) sheds nothing, which is the
// symmetric flow's answer. An edge whose wake would run into another body
// sheds nothing either (the buried-edge rule of wing_panels.hpp).
//
// SYMMETRY. With Options::symmetry_y the image of every panel and wake
// about that plane is added to its own column, and a body straddling the
// plane is cut there (mesh_cut.hpp) and its y < plane side dropped; the
// image closes it.
//
// VELOCITY AND PRESSURE. On the outer side the tangential velocity is the
// free stream's tangential part plus the surface gradient of mu, which is
// fitted by weighted least squares over the panel's neighbours across
// edges and vertices (neighbours facing more than 60 degrees away -- the
// other side of a sharp trailing edge -- left out). Cp = 1 - |V|^2 / V^2.
// Forces are -sum Cp n A over the panels (and their images).
//
// SOLVE. Dense LU with partial pivoting up to Options::direct_max_panels;
// beyond, restarted GMRES on the Jacobi-scaled system (the Dirichlet
// matrix is strongly diagonal, -1/2 on the diagonal, and converges in a
// few dozen iterations). The matrix is N x N doubles: 8000 panels is half
// a gigabyte, and Options::max_panels refuses more with a message that
// says to coarsen the grid on the Mesh tab.

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "aerotelic/mesh.hpp"
#include "aerotelic/vec3.hpp"

namespace aerotelic::tripanel {

// One body to panel: a closed, outward-oriented triangle mesh placed in
// the world, with the polylines (world coordinates) along which it sheds
// a wake. A body with no trailing edge is a non-lifting closed body.
struct Body {
	std::string name;
	TriMesh mesh;
	std::vector<std::vector<Vec3>> trailing_edges;
	// Edge endpoints within this distance of a trailing-edge polyline count
	// as on it (0: a millionth of the mesh's diagonal... too tight for a
	// boolean's seam points; the caller sets it, typically 1e-6 x extent).
	double trailing_edge_tolerance = 0.0;
};

struct Options {
	// Mirror every panel in the plane y = *symmetry_y; bodies straddling it
	// are cut there and their y < plane side dropped.
	std::optional<double> symmetry_y;
	double reference_area = 1.0;
	Vec3 moment_reference{0.0, 0.0, 0.0};
	// Wake length (0: twenty times the configuration's extent).
	double wake_length = 0.0;
	// The direction Solution::lift_from_wake is resolved along (unset: normal
	// to the stream in the plane of the stream and +z).
	std::optional<Vec3> lift_direction;
	// Beyond this many panel radii (sqrt of the area) the point source and
	// dipole stand in for the closed-form integrals.
	double far_field_radii = 8.0;
	// Direct LU up to this many unknowns, GMRES beyond (the Dirichlet matrix
	// converges in a dozen or two iterations: 6000 unknowns take 20 s by LU
	// and well under a second by GMRES on the same matrix).
	int direct_max_panels = 1500;
	// Refused beyond this many unknowns (the matrix would not fit).
	int max_panels = 20000;
	double gmres_tolerance = 1e-9;
	int gmres_restart = 60;
	int gmres_max_iterations = 600;
	// Threads for the influence matrix and the matrix-vector products (0: all).
	int threads = 0;
	// The buried-edge rule: a trailing edge whose midpoint stepped `probe`
	// downstream lies inside another body, or within `tolerance` of its
	// surface, sheds no wake (0: 2% and 5% of the configuration's extent).
	double buried_probe = 0.0;
	double buried_tolerance = 0.0;
	bool buried_rule = true;
	// Neighbours for the gradient fit must face within this many degrees.
	double gradient_max_angle_deg = 60.0;
	// Wake routing (see WakeStrip): a sheet whose straight path would enter a
	// body -- any body, the shedding one included: a fused configuration is
	// one body -- is kinked around it, `wake_clearance_panels` of that body's
	// typical panel size off its surface, sampled at `wake_route_samples`
	// stations over the body's extent along the stream.
	bool wake_routing = true;
	double wake_clearance_panels = 2.0;
	int wake_route_samples = 6;
	// A panel's surface speed is capped at this many times the free stream
	// (0: no cap). No potential flow about a closed body goes much beyond 2
	// (a cylinder's crosswise peak); a fit on neighbours lying in a line can
	// give anything. See BodyResult::capped_panels.
	double max_speed_ratio = 4.0;
};

struct PanelResult {
	Vec3 centre, normal;   // the centroid and the outward unit normal
	double area = 0.0;
	double sigma = 0.0;    // prescribed source strength
	double mu = 0.0;       // doublet strength = the outer perturbation potential
	Vec3 grad_mu;          // its surface gradient
	Vec3 velocity;         // on the outer side
	double cp = 0.0;
};

// One trailing-edge edge's wake: a sheet of constant doublet strength from
// the edge (a, b) downstream. Its two long edges are the polylines a_path
// (from a) and b_path (from b), of equal length; segment k is the quad
// (b_path[k], a_path[k], a_path[k+1], b_path[k+1]). A sheet whose straight
// path along the stream would enter a body, or pass within a clearance of
// it, is ROUTED around it (Options::wake_routing): kinked to pass the body
// on the side it approached from, the clearance off its surface, and back
// to the stream line beyond -- a doublet sheet stopped short would close
// its vortex loop with a filament at that body's leading edge; through the
// body it is a potential jump across the body's panels; a fraction of a
// panel off the surface it is nearly as bad. A straight strip has two
// points per path.
struct WakeStrip {
	std::vector<Vec3> a_path, b_path;
	std::array<Vec3, 4> corners; // the FIRST segment: b, a, a_path[1], b_path[1] (kept for readers of the shed edge)
	double mu = 0.0;
	int body = -1;
	int upper = -1, lower = -1;  // the two panels of the body at the edge
	int routed_around = -1;      // the body the sheet was kinked around (-1: none)
	bool stopped = false;        // the sheet stayed within a clearance of a body to the end of the wake length
	size_t Segments() const { return a_path.size() >= 2 ? a_path.size() - 1 : 0; }
};

struct BodyResult {
	std::string name;
	// The mesh actually paneled: the input, or its y >= plane half when it
	// was cut. One PanelResult per triangle of it.
	TriMesh mesh;
	// Per paneled triangle, the index of the input triangle it came from.
	std::vector<int> parent;
	std::vector<PanelResult> panels;
	bool cut_at_plane = false;
	int wake_edges = 0;          // trailing-edge edges that shed a wake
	int rim_edges = 0;           // trailing-edge edges on the plane's rim (one triangle: no wake)
	// Trailing-edge edges lying in y = 0 with the free stream in that plane
	// and no symmetry option (a centreline fin): no wake. A sheet there would
	// run along the centreline geometry (a tailplane's root) within a
	// fraction of a panel of its panels and couple the two sides; by the
	// configuration's symmetry its strength is zero, and the plane on gives
	// the same through its rim rule. A sideslip sheds it.
	int centreline_edges = 0;
	int buried_edges = 0;        // trailing-edge edges whose wake would run into another body
	int routed_edges = 0;        // trailing-edge edges whose sheet was kinked around a body downstream
	int stopped_edges = 0;       // ... or ended short of one it never left (see WakeStrip::stopped)
	int dropped_in_plane = 0;    // triangles lying in the symmetry plane, left out
	// The gradient fit: panels whose neighbours lay in a line (the gradient
	// taken along it only), and panels whose surface speed exceeded
	// Options::max_speed_ratio and was capped, the first of them.
	int rank1_gradients = 0;
	int capped_panels = 0;
	Vec3 first_capped_at;
	// -sum Cp n A / S_ref over this body (the image included when mirrored).
	Vec3 force_coefficient;
	Vec3 moment_coefficient;
	std::vector<std::string> notes;
};

struct Solution {
	std::vector<BodyResult> bodies;
	std::vector<WakeStrip> wakes;
	Vec3 free_stream;
	Vec3 force_coefficient;
	Vec3 moment_coefficient;
	int unknowns = 0;
	bool direct = true;          // LU, else GMRES
	int iterations = 0;          // GMRES iterations (0 for LU)
	double residual = 0.0;       // GMRES relative residual reached
	double pivot_ratio = 0.0;    // LU: smallest over largest pivot
	double seconds_influence = 0.0, seconds_solve = 0.0;
	// C_L from the wake doublet strengths: (2 / V S) sum mu_j (V_hat x dl_j) . L_hat
	// over the shed edges (images included). Unset without a wake.
	std::optional<double> lift_from_wake;
	// Every body's notes together, for the report.
	std::vector<std::string> notes;
};

// Solves the flow. Throws std::invalid_argument on an empty or unusable
// mesh (not closed, inside out), on too many panels (Options::max_panels),
// and std::runtime_error if the system cannot be solved.
Solution Solve(const std::vector<Body>& bodies, const Vec3& free_stream, const Options& options = {});

// ---- mesh edges and statistics ----

// The mesh edges (vertex pairs, each once) whose two endpoints and
// midpoint lie within `tol` of one of the polylines.
std::vector<std::array<int, 2>> EdgesOnPolylines(const TriMesh& mesh, const std::vector<std::vector<Vec3>>& polylines, double tol);

// What an analysis dialog shows about a mesh before anything is solved.
struct MeshStats {
	int triangles = 0, vertices = 0;
	double min_edge = 0.0, median_edge = 0.0, max_edge = 0.0;
	// The worst ratio of the longest edge to the shortest altitude (1.15
	// for an equilateral triangle; needles run to hundreds).
	double worst_aspect = 0.0;
	int slivers = 0;             // triangles with an aspect over 20
	double area = 0.0, volume = 0.0;
	bool closed = false;         // a closed, consistently oriented surface
	bool outward = false;        // positive signed volume
};
MeshStats ComputeMeshStats(const TriMesh& mesh);

// How many triangles of `a` have their centroid within `distance` of the
// surface of `b` (an SDF over b; both placed): the proximity warning.
int TrianglesNear(const TriMesh& a, const TriMesh& b, double distance);

// ---- post-processing on triangles ----

// The section of a paneled body by the plane Dot(P, axis) = s: every
// triangle the plane crosses contributes its crossing segment with the
// panel's Cp, and the force per unit length along the axis is
//   dF/ds / q = sum -Cp (t x axis) |dl|   (t the unit tangent of the cut,
// oriented so t x axis points along the panel's outward normal).
struct SectionCut {
	bool found = false;
	Vec3 force_per_span;
	double length = 0.0;
	std::vector<Vec3> points;    // segment midpoints
	std::vector<double> cp;
};
// `mask`, when given (one flag per panel), restricts the cut to the panels
// flagged -- one entity's share of a fused body.
SectionCut CutSection(const BodyResult& body, const Vec3& axis, double s, bool mirror_y = false, const std::vector<char>* mask = nullptr);

// The panels' loads summed into bins along `axis` (edges: n + 1 increasing
// values; n forces returned, / q): each triangle's force -Cp n A spread
// over the bins its extent along the axis overlaps, in proportion.
// `mirror_y` adds every body's image about y = 0 as well.
std::vector<Vec3> BinnedLoads(const Solution& sol, const Vec3& axis, const std::vector<double>& edges, bool mirror_y);

} // namespace aerotelic::tripanel
