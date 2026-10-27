#pragma once

// The higher-order panel method of Johnson, NASA CR-3079 (1980), for
// incompressible potential flow: networks of curved panels carrying linear
// source or quadratic doublet distributions, singularity splines over each
// network, boundary conditions at panel centres and network edges, closed-
// form influence coefficients (panel_integrals.hpp), a dense solve, and
// the surface velocities, pressures and loads.
//
// What is implemented, by the report's appendices:
//   A  Corner-point grids; the flat-panel frame (A.1)-(A.7) and the curved
//      panel ζ = a ξ² + b η² by weighted least squares with the iterated
//      rotations that remove the linear terms (A.8)-(A.20).
//   B  Network types 1 (source/analysis), 2 (doublet/analysis), 8 and 10
//      (doublet wakes) with the least-squares splines of B.2: singularity
//      parameters at the points of figure B.1, each panel's distribution
//      fitted to its own and its neighbours' parameters.
//   C  Control points of figure C.1; panel-centre conditions on velocity
//      (C.7) or potential (C.14); the network-edge conditions of C.3 as the
//      conditions they enforce -- doublet strength and gradient continuity
//      across an abutment (which, against a wake, is the Kutta condition),
//      zero strength on a free edge, zero normal gradient on a symmetry
//      plane -- imposed on the splines directly rather than through the
//      edge-induced velocity of the pilot code.
//   D  The influence coefficients, near and far field.
//   E  LU with partial pivoting (in core; the report's out-of-core block
//      scheme is not needed here).
//   F  Average velocity at panel centres, the difference velocity σ n + ∇μ
//      (C.10), upper/lower velocities and pressures, forces by panel and by
//      trailing-edge doublet strength.
// Not implemented: the design networks (types 3-6) and the continuous
// doublet spline of B.3.

#include <array>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "aerotelic/panel_integrals.hpp"
#include "aerotelic/vec3.hpp"

namespace aerotelic::panel {

enum class NetworkType {
	SourceAnalysis = 1,  // type 1: linear sources, parameters and control points at panel centres
	DoubletAnalysis = 2, // type 2: quadratic doublets, parameters/control points at centres, edge midpoints and corners
	DoubletWake = 8,     // type 8: one doublet parameter per column, constant along the column (streamwise)
	DoubletWakeConstant = 10, // type 10: one parameter for the whole network
};

// A panel-centre boundary condition (C.7)/(C.14) in average/difference form:
//   Velocity:  c_A (n·V_A) + c_D (n·V_D) = β,   V_A the average total velocity,
//                                                V_D = σ n + ∇μ the difference
//   Potential: k_A φ_A + k_D φ_D = β,            φ_D = μ
// Upper/lower coefficients convert by c_A = c_U + c_L, c_D = (c_U - c_L)/2.
struct BoundaryCondition {
	enum class Kind { Velocity, Potential } kind = Kind::Velocity;
	double c_average = 1.0, c_difference = 0.5, beta = 0.0; // c_U = 1, c_L = 0, β = 0: an impermeable upper side

	static BoundaryCondition UpperLower(Kind kind, double c_upper, double c_lower, double beta) {
		BoundaryCondition bc;
		bc.kind = kind;
		bc.c_average = c_upper + c_lower;
		bc.c_difference = 0.5 * (c_upper - c_lower);
		bc.beta = beta;
		return bc;
	}
	// No flow through the surface on its upper (+ζ) side.
	static BoundaryCondition Impermeable() { return UpperLower(Kind::Velocity, 1.0, 0.0, 0.0); }
	// Zero perturbation potential on the lower (interior) side: the
	// Dirichlet formulation of figure 8, with the sources prescribed.
	static BoundaryCondition InteriorPotentialZero() { return UpperLower(Kind::Potential, 0.0, 1.0, 0.0); }
};

// How a network edge is treated when its points carry no centre-type
// condition. Edges are numbered 0: I = 1 (first row), 1: J = N (last
// column), 2: I = M (last row), 3: J = 1 (first column).
enum class EdgeKind {
	Auto,     // abutting another doublet network: continuity; on the symmetry plane: zero normal gradient; else free
	Free,     // doublet strength zero
	Symmetry, // zero gradient normal to the edge
	Abut,     // continuity with whatever doublet network edge coincides (error if none)
};

struct Network {
	NetworkType type = NetworkType::DoubletAnalysis;
	std::string name;
	int rows = 0, cols = 0;    // corner grid M x N; corners[(I - 1) * cols + (J - 1)]
	std::vector<Vec3> corners;
	BoundaryCondition bc;
	std::array<EdgeKind, 4> edges = {EdgeKind::Auto, EdgeKind::Auto, EdgeKind::Auto, EdgeKind::Auto};
	// False keeps this network's panels flat whatever Options::curved_panels
	// says: a surface of planes meeting at edges (a box) has no curvature
	// worth fitting, and a fit across an edge is wrong.
	bool curved = true;
	// A source distribution of KNOWN strength carried on the same panels
	// (figure 8: σ = -V∞·n with the interior potential zero): fitted by the
	// type-1 spline from these values at the panel centres and added to the
	// right-hand side and to the difference velocity.
	std::function<double(const Vec3& centre, const Vec3& normal)> prescribed_source;

	Vec3 Corner(int I, int J) const { return corners[static_cast<size_t>(I - 1) * cols + (J - 1)]; }
	bool IsDoublet() const { return type != NetworkType::SourceAnalysis; }
	bool IsWake() const { return type == NetworkType::DoubletWake || type == NetworkType::DoubletWakeConstant; }
};

struct Options {
	bool curved_panels = true;
	IntegralOptions integrals;
	// Mirror every network in the plane y = *symmetry_y (the paper's sphere
	// and wings: half the configuration paneled, the image added).
	std::optional<double> symmetry_y;
	double reference_area = 1.0;
	Vec3 moment_reference{0, 0, 0};
	// Edges whose points all lie within this distance of another network's
	// edge abut it; points within it of the symmetry plane lie on it.
	double abutment_tolerance = 1e-6;
	// Spline weights (B.2): the heavy weight on a panel's own parameters.
	double own_weight = 1e8;
	// Edge control point displacement (C.1), (C.2), as a fraction of the
	// panel diameter. Keep it in 1e-4 .. 1e-3: at 1e-5 and below the
	// displaced point sits so close to the edge of the NEIGHBOURING curved
	// panel (the image across a symmetry plane, or an abutting network)
	// that its near-field influence loses all precision, and single columns
	// of the solution oscillate (seen 2026-10-26 on the default wing at 12 x
	// 10 and 14 x 10: Cp of -300 on the root strip, lift of the wrong sign,
	// with the wake lift still right); at 3e-3 and above the displaced
	// points no longer register on the wake abutment and the lift is lost.
	double edge_epsilon = 1e-4;
};

struct PanelResult {
	Vec3 centre, normal;       // the control point (on the curved surface) and ζ̂ there
	double area = 0.0;         // of the projected quadrilateral Σ
	double sigma = 0.0;        // source strength at the centre
	double mu = 0.0;           // doublet strength at the centre
	Vec3 grad_mu;              // surface gradient of μ at the centre
	Vec3 v_average, v_upper, v_lower; // total velocities (C.5), (C.9)
	double cp_upper = 0.0, cp_lower = 0.0;
	double delta = 0.0;        // the curvature measure δ of (A.21)
};

struct NetworkResult {
	std::string name;
	NetworkType type;
	int panel_rows = 0, panel_cols = 0; // panels[(i) * panel_cols + j], 0-based
	std::vector<PanelResult> panels;
	// Force coefficient on the reference area: -∮ Cp n dA for a one-sided
	// (closed-body) network, ∬ (Cp_lower - Cp_upper) n dA for a sheet
	// with both sides wetted.
	Vec3 force_coefficient;
	Vec3 moment_coefficient;
};

struct Solution {
	std::vector<NetworkResult> networks;
	Vec3 free_stream;
	Vec3 force_coefficient;  // sum over networks (the image side included when mirrored)
	Vec3 moment_coefficient;
	int unknowns = 0;
	double pivot_ratio = 0.0; // smallest over largest LU pivot: a conditioning hint
	double max_delta = 0.0;  // the largest panel curvature measure; (A.23) wants < 0.066
	double seconds_influence = 0.0, seconds_solve = 0.0;
	// Lift coefficient from the trailing-edge doublet strength of the wakes,
	// C_L = (2 / V∞ S_R) ∫ μ_TE dy (section 6.3.1), when there is a wake.
	std::optional<double> lift_from_wake;
	// Internal: the singularity parameters and the panel splines, for VelocityAt().
	std::vector<double> parameters;
	struct Internal;
	std::shared_ptr<Internal> internal;
};

// Solves the flow. Throws std::invalid_argument on a malformed network
// (fewer than 2 x 2 corners, a wake without a column head, an abutting
// edge with nothing to abut) and std::runtime_error if the system is
// singular.
Solution Solve(const std::vector<Network>& networks, const Vec3& free_stream, const Options& options = {});

// The total velocity and the perturbation potential at a field point off
// the surfaces (on a surface they give the average values).
Vec3 VelocityAt(const Solution& solution, const Vec3& point);
double PotentialAt(const Solution& solution, const Vec3& point);

// ---- network builders for the standard shapes ----

// A grid of corner points from a parametric surface f(u, v), u along the
// rows (I) and v along the columns (J).
Network GridNetwork(NetworkType type, const std::string& name, int rows, int cols, const std::function<Vec3(double u, double v)>& surface, const std::vector<double>* u_values = nullptr, const std::vector<double>* v_values = nullptr);

// A flat wake network behind an edge polyline (the trailing edge, in
// column order), extended `length` along `direction`, one panel row.
Network WakeNetwork(const std::string& name, const std::vector<Vec3>& trailing_edge, const Vec3& direction, double length, NetworkType type = NetworkType::DoubletWake);

// Reverses the column order, which turns the panel normals
// ζ = (∂P/∂J) x (∂P/∂I) the other way (edges 1 and 3 swap).
void FlipNormals(Network& net);

// Cosine spacing on [0, 1]: n + 1 values clustered at both ends.
std::vector<double> CosineSpacing(int n);

} // namespace aerotelic::panel
