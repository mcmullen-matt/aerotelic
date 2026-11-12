#pragma once

// The coupled inviscid / integral-boundary-layer airfoil solver of XFOIL
// in Fidkowski's mfoil form (sections IV-V and VII of the paper), on the
// vortex-panel method of vortex_panel.hpp:
//
//   - four states per node [theta, delta*, n~ or sqrt(c_tau), u_e] on the
//     airfoil and wake nodes, three boundary-layer residuals per node
//     (ibl_closures.hpp) plus the edge-velocity residual
//     u_e = u_e,inv + D (u_e delta*) that couples the layer back to the
//     potential flow through transpiration sources (eqs. 23-27);
//   - the stagnation point extrapolated from the first two nodes of each
//     surface (IV.C), the wake started from the two trailing-edge states
//     (IV.D), transition by the e^N method inside a panel (IV.E) with the
//     transition residual's derivatives through the implicit function
//     theorem (eq. 19), forced transition at a chord fraction optional;
//   - initialisation by direct/inverse marching on the inviscid edge
//     velocity (V.A), then a global Newton with the paper's update
//     limiting (V.C), the stagnation point moved and the transition
//     interval re-marched after every update;
//   - Jacobians by forward-mode automatic differentiation (dual.hpp), the
//     global system solved densely.
//
// Incompressible (Mach 0). The solve is a few seconds at 200 nodes.

#include <string>
#include <vector>

#include "aerotelic/mat.hpp"

namespace aerotelic {

struct IblOptions {
	double reynolds = 1.0e6;   // on the chord
	double ncrit = 9.0;        // e^N critical amplification
	double xft_upper = 1.0;    // forced transition x/c on each surface (1 = natural only)
	double xft_lower = 1.0;
	int max_newton = 50;
	double rtol = 1e-10;       // residual norm tolerance
	double wake_length = 1.0;  // chords
	int n_wake = 0;            // 0: the paper's N/10 + 10 d_w
	double h_sep_lam = 3.83;   // laminar separation cue: H_k at which the laminar c_f closure vanishes
	double h_sep_turb = 2.8;   // turbulent incipient-detachment cue (Kline 2.7, Castillo-George 2.76)
	bool continuation = true;  // on failure, retry from a lower incidence in steps
	bool verbose = false;
};

struct IblNode {
	double x = 0.0, y = 0.0; // position
	double xi = 0.0;         // distance from stagnation along the surface (or wake)
	double theta = 0.0, delta_star = 0.0, sa = 0.0, ue = 0.0;
	double hk = 0.0, cf = 0.0, ret = 0.0, cp = 0.0;
	bool turb = false;
};

struct IblResult {
	bool ok = false;        // a solution was produced (converged or not)
	bool converged = false;
	std::string error;
	std::vector<std::string> warnings;
	int iterations = 0;
	double residual_norm = 0.0;
	double alpha_deg = 0.0;

	double cl = 0.0, cm_c4 = 0.0, cm_le = 0.0;
	double cd = 0.0;        // Squire-Young total drag
	double cdf = 0.0;       // skin friction
	double cdp = 0.0;       // cd - cdf
	double cdpi = 0.0;      // near-field pressure drag by Cp integration (a residual)
	double cl_inviscid = 0.0;

	// Transition x/c per surface (chord fraction), negative when the layer
	// is laminar to the trailing edge (transition forced at the wake).
	double xt_lower = -1.0, xt_upper = -1.0;
	bool xt_lower_forced = false, xt_upper_forced = false;

	int n_airfoil = 0, n_wake = 0;
	std::vector<IblNode> nodes;         // airfoil nodes then wake nodes
	std::vector<int> lower, upper, wake; // node indices of each surface from stagnation / the TE
	double s_stag = 0.0;
	double x_stag = 0.0, y_stag = 0.0;
	double chord = 1.0, x_min = 0.0;
	double solve_ms = 0.0;
};

// `closed_contour`: (M, 2) lower TE -> LE -> upper TE with coincident end
// points (resample it to the node count wanted beforehand with
// vortexpanel::ResampleContour). Never throws for a solver failure: ok =
// false with the reason, or ok = true and converged = false with the state
// as it stands.
IblResult SolveIblAirfoil(const Mat& closed_contour, double alpha_deg, const IblOptions& options);

// Development check: after initialisation, compares the assembled global
// Jacobian with central differences of the residual over a sample of
// columns (every `stride`-th state). Returns the worst relative error.
double IblJacobianCheck(const Mat& closed_contour, double alpha_deg, const IblOptions& options, int stride = 7, bool verbose = false);

} // namespace aerotelic
