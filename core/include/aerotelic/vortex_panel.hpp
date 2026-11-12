#pragma once

// The inviscid half of an XFOIL-class airfoil solver, after Fidkowski,
// "A Coupled Inviscid-Viscous Airfoil Analysis Solver, Revisited" (mfoil),
// sections II-III and appendix A, which in turn follow Drela's XFOIL:
//
//   - N airfoil nodes ordered lower TE -> LE -> upper TE (clockwise in
//     x-right, y-up coordinates), N-1 panels each carrying a LINEAR vortex
//     sheet and a CONSTANT source sheet; a wake of Nw nodes carrying source
//     sheets linear over each half panel.
//   - Flow tangency as a constant streamfunction Psi0 at the airfoil nodes,
//     the Kutta condition gamma_1 + gamma_N = 0, and for a closed (sharp)
//     trailing edge, whose nodes 1 and N coincide, the N-th node row replaced
//     by the extrapolation gamma_N - gamma_1 = 2(gamma_2 - gamma_N-1) -
//     (gamma_3 - gamma_N-2).
//   - Because the interior is stagnant, gamma_i is the tangential velocity at
//     node i in the clockwise (node-ordering) direction; the edge velocity
//     away from stagnation is u_e,i = d_i gamma_i with d_i = -1 on the lower
//     surface.
//   - The solution is stored as the 0 and 90 degree reference vectors,
//     gamma = gamma_0 cos(alpha) + gamma_90 sin(alpha), and the LU of the
//     augmented system, which the viscous coupling reuses for B' = -A^-1 B.
//   - The wake is the inviscid streamline from the trailing edge, marched by
//     predictor-corrector with geometric spacing to a length of d_w chords.
//
// Sign conventions (derived in src/vortex_panel.cpp and checked against
// quadrature in tests/vortex_panel_check.cpp): Psi = +(1/2pi) Int gamma ln r
// for the vortex sheet, Psi = (1/2pi) Int sigma theta for the source sheet,
// velocity u = dPsi/dy, v = -dPsi/dx. Constant-source branch cuts are placed
// along the outward normal through each panel's midpoint so that no airfoil
// node sees a cut (the paper's figure 3).

#include <vector>

#include "aerotelic/dense_lu.hpp"
#include "aerotelic/mat.hpp"

namespace aerotelic {
namespace vortexpanel {

struct Vec2 {
	double x = 0.0, y = 0.0;
	Vec2() = default;
	Vec2(double x_, double y_) : x(x_), y(y_) {}
	Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
	Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
	Vec2 operator*(double s) const { return {x * s, y * s}; }
	double dot(const Vec2& o) const { return x * o.x + y * o.y; }
	double norm() const;
};

// Panel-local geometry of an evaluation point (the paper's figure 10): the
// panel runs from p1 to p2 (length d, tangent t, left/outward normal n); a is
// the point's coordinate along t from p1, h its distance along n; r1, r2 the
// distances to the ends; th1, th2 the angles of the vectors from p1 and p2
// to the point, from the t axis.
struct PanelFrame {
	Vec2 t, n;
	double d = 0.0, a = 0.0, h = 0.0, r1 = 0.0, r2 = 0.0, th1 = 0.0, th2 = 0.0;
};
PanelFrame FrameOf(const Vec2& p1, const Vec2& p2, const Vec2& x);

// Streamfunction influence coefficients: Psi = c1 * q1 + c2 * q2 for the
// end strengths q1, q2 of a linear sheet; a constant sheet is c1 + c2.
struct Coef2 { double c1 = 0.0, c2 = 0.0; };
Coef2 PsiLinearVortex(const PanelFrame& f);
Coef2 PsiLinearSource(const PanelFrame& f); // cut along +n from the midpoint
double PsiConstSource(const PanelFrame& f);  // = PsiLinearSource summed

// Velocity influence coefficients, global components: v = v1 * q1 + v2 * q2.
struct VelCoef2 { Vec2 v1, v2; };
VelCoef2 VelLinearVortex(const PanelFrame& f);
VelCoef2 VelLinearSource(const PanelFrame& f);

struct Paneling {
	std::vector<Vec2> nodes;   // N airfoil nodes, nodes.front() == nodes.back() (closed TE)
	std::vector<double> s;     // arc length at the airfoil nodes from node 1
	std::vector<Vec2> wake;    // Nw wake nodes, from just behind the TE
	std::vector<double> sw;    // arc length along the wake from the TE midpoint
	double chord = 1.0, x_min = 0.0;
	Vec2 te;                   // trailing-edge point (midpoint of the two ends)
	Vec2 te_bisector;          // unit direction leaving the trailing edge downstream
	int n_airfoil() const { return static_cast<int>(nodes.size()); }
	int n_wake() const { return static_cast<int>(wake.size()); }
};

struct InviscidOptions {
	double wake_length = 1.0;  // d_w, chords
	int n_wake = 0;            // 0: the paper's N/10 + 10 d_w
	double eps_wake = 1e-5;    // first wake node, chords behind the TE
	double alpha_wake_deg = 0.0; // the wake streamline is traced at this incidence
};

struct InviscidSolution {
	Paneling geom;
	DenseLU lu;                       // of the (N+1) x (N+1) augmented system
	std::vector<double> gamma0, gamma90; // the reference vortex strengths (N)
	double psi0_0 = 0.0, psi0_90 = 0.0;
	std::vector<int> direction;       // d_i: -1 on the lower surface, +1 on the upper (and wake)
	int stagnation_lower = 0;         // the stagnation interval's lower-surface node index at alpha_wake

	std::vector<double> GammaAt(double alpha_rad) const;
	std::vector<double> CpAt(double alpha_rad) const; // 1 - gamma^2 at the nodes
};

// Solves the inviscid problem on a closed contour (M, 2) lower TE -> LE ->
// upper TE with coincident ends; the contour is used as given (resample it
// first, see ResampleContour). Throws std::invalid_argument on bad input.
InviscidSolution SolveInviscid(const Mat& closed_contour, const InviscidOptions& options = {});

// The velocity of the airfoil's vortex sheet plus the free stream at a field
// point (no sources): what the wake streamline is traced with.
Vec2 InviscidVelocityAt(const InviscidSolution& sol, const std::vector<double>& gamma, const Vec2& x, double alpha_rad);

// Lift, drag and moment coefficients by panel-wise linear Cp (eqs. 33-35),
// the moment nose-up positive about x_ref, chord from the paneling.
struct PanelLoads { double cl = 0.0, cd = 0.0, cm = 0.0; };
PanelLoads IntegrateLoads(const Paneling& geom, const std::vector<double>& cp, double alpha_rad, const Vec2& x_ref);

// The stagnation interval (gamma changes sign from negative on the lower
// surface to positive on the upper) nearest the leading edge, and the
// interpolated stagnation arc length s_stag (eq. in IV.C).
int StagnationInterval(const Paneling& geom, const std::vector<double>& gamma, double* s_stag = nullptr);

// Resamples a closed contour polyline to n nodes by arc length, clustered at
// the leading edge (cosine) and, by `te_cluster` in [0, 1], at the trailing
// edge: 1 is the full cosine (nodes as dense at the TE as at the LE), 0 none
// (uniform spacing arriving at the TE). The boundary-layer solver wants a
// low value: tiny trailing-edge panels make the transpiration source
// strength dm/ds there ill-conditioned (mfoil's low TE resolution factor).
// Ends coincide.
Mat ResampleContour(const Mat& closed_contour, int n, double te_cluster = 1.0);

} // namespace vortexpanel
} // namespace aerotelic
