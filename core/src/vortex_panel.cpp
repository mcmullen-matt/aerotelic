#include "aerotelic/vortex_panel.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aerotelic {
namespace vortexpanel {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double SafeLog(double r) { return r > 0.0 ? std::log(r) : 0.0; }

} // namespace

double Vec2::norm() const { return std::sqrt(x * x + y * y); }

PanelFrame FrameOf(const Vec2& p1, const Vec2& p2, const Vec2& x) {
	PanelFrame f;
	const Vec2 e = p2 - p1;
	f.d = e.norm();
	if (!(f.d > 0.0)) throw std::invalid_argument("vortex panel: zero-length panel");
	f.t = e * (1.0 / f.d);
	f.n = Vec2(-f.t.y, f.t.x);
	const Vec2 q = x - p1;
	f.a = q.dot(f.t);
	f.h = q.dot(f.n);
	f.r1 = q.norm();
	f.r2 = (x - p2).norm();
	f.th1 = std::atan2(f.h, f.a);
	f.th2 = std::atan2(f.h, f.a - f.d);
	// At the panel's own end points atan2(0, 0) is 0, but the angles the
	// exterior-side limit has are pi, pi at the start node and 0, 0 at the
	// end node (mfoil's panel_info convention); the logs of r = 0 are taken
	// as 0, their factors vanishing there.
	const double ep = 1e-9 * f.d;
	if (f.r1 < ep) { f.th1 = kPi; f.th2 = kPi; }
	else if (f.r2 < ep) { f.th1 = 0.0; f.th2 = 0.0; }
	return f;
}

// ---- vortex sheet -----------------------------------------------------------
// Psi = (1/2pi) Int gamma(s') ln r ds' (a clockwise element dGamma = -gamma ds'
// has Psi = +(gamma ds'/2pi) ln r), so that a sheet with the stagnant interior
// on its right carries the velocity +gamma t on its outer side.
//   Int_0^d ln r ds'    = a ln r1 - (a - d) ln r2 - d + h (th2 - th1)           (eq. 37)
//   Int_0^d s' ln r ds' = a Int ln r + 1/2 (r2^2 ln r2 - r1^2 ln r1) - 1/4 (r2^2 - r1^2) (eq. 38)
Coef2 PsiLinearVortex(const PanelFrame& f) {
	const double l1 = SafeLog(f.r1), l2 = SafeLog(f.r2);
	const double bar = (f.a * l1 - (f.a - f.d) * l2 - f.d + f.h * (f.th2 - f.th1)) / kTwoPi;
	const double tilde = (f.a / f.d) * bar + (f.r2 * f.r2 * l2 - f.r1 * f.r1 * l1 - 0.5 * f.r2 * f.r2 + 0.5 * f.r1 * f.r1) / (2.0 * kTwoPi * f.d);
	return {bar - tilde, tilde};
}

// v_t = (1/2pi) Int gamma h / r^2 ds', v_n = -(1/2pi) Int gamma (a - s') / r^2 ds'  (eqs. 39-40)
VelCoef2 VelLinearVortex(const PanelFrame& f) {
	const double lr = SafeLog(f.r2) - SafeLog(f.r1); // ln(r2/r1)
	const double dth = f.th2 - f.th1;
	const double vt_bar = dth / kTwoPi;
	const double vt_tilde = (f.h * lr + f.a * dth) / (kTwoPi * f.d);
	const double vn_bar = lr / kTwoPi;
	const double vn_tilde = (f.a * lr + f.d - f.h * dth) / (kTwoPi * f.d);
	VelCoef2 v;
	const double t1 = vt_bar - vt_tilde, n1 = vn_bar - vn_tilde;
	v.v1 = f.t * t1 + f.n * n1;
	v.v2 = f.t * vt_tilde + f.n * vn_tilde;
	return v;
}

// ---- source sheet -------------------------------------------------------------
// Psi = (1/2pi) Int sigma(s') theta(s') ds', theta = atan2(h, a - s').
//   Int_0^d theta ds'    = a (th1 - th2) + d th2 + h (ln r1 - ln r2)          (eq. 41)
//   Int_0^d s' theta ds' = a Int theta + 1/2 (r2^2 th2 - r1^2 th1) - 1/2 h d   (eq. 42)
// The atan2 cut of the integral lies along the -t ray from node 1; the whole
// sheet's strength d (or (s1 + s2) d / 2) is subtracted for points above the
// panel and left of its perpendicular bisector (th1 + th2 > pi), which moves
// the cut to the +n ray from the midpoint, where no airfoil node lies.
Coef2 PsiLinearSource(const PanelFrame& f) {
	const double l1 = SafeLog(f.r1), l2 = SafeLog(f.r2);
	const double bar = (f.a * (f.th1 - f.th2) + f.d * f.th2 + f.h * (l1 - l2)) / kTwoPi;
	const double tilde = (f.a / f.d) * bar + (f.r2 * f.r2 * f.th2 - f.r1 * f.r1 * f.th1 - f.h * f.d) / (2.0 * kTwoPi * f.d);
	Coef2 c{bar - tilde, tilde};
	if (f.th1 + f.th2 > kPi) { c.c1 -= 0.5 * f.d; c.c2 -= 0.5 * f.d; }
	return c;
}

double PsiConstSource(const PanelFrame& f) {
	const Coef2 c = PsiLinearSource(f);
	return c.c1 + c.c2;
}

// v_t = (1/2pi) Int sigma (a - s') / r^2 ds', v_n = (1/2pi) Int sigma h / r^2 ds'  (eqs. 43-44)
VelCoef2 VelLinearSource(const PanelFrame& f) {
	const double lr = SafeLog(f.r1) - SafeLog(f.r2); // ln(r1/r2)
	const double dth = f.th2 - f.th1;
	const double vt_bar = lr / kTwoPi;
	const double vt_tilde = (f.a * lr - f.d + f.h * dth) / (kTwoPi * f.d);
	const double vn_bar = dth / kTwoPi;
	const double vn_tilde = (-f.h * lr + f.a * dth) / (kTwoPi * f.d);
	VelCoef2 v;
	v.v1 = f.t * (vt_bar - vt_tilde) + f.n * (vn_bar - vn_tilde);
	v.v2 = f.t * vt_tilde + f.n * vn_tilde;
	return v;
}

// ---- paneling ---------------------------------------------------------------------

Mat ResampleContour(const Mat& contour, int n, double te_cluster) {
	const int m = contour.rows();
	if (m < 4 || contour.cols() != 2) throw std::invalid_argument("ResampleContour: need an (M, 2) contour");
	if (n < 8) throw std::invalid_argument("ResampleContour: need at least 8 nodes");
	std::vector<double> s(m, 0.0);
	for (int i = 1; i < m; ++i) s[i] = s[i - 1] + std::hypot(contour(i, 0) - contour(i - 1, 0), contour(i, 1) - contour(i - 1, 1));
	const double S = s.back();
	if (!(S > 0.0)) throw std::invalid_argument("ResampleContour: degenerate contour");
	// The leading edge (minimum x) splits the arc; each half gets cosine
	// spacing so the nodes cluster at both the TE and the LE.
	int i_le = 0;
	for (int i = 1; i < m; ++i) if (contour(i, 0) < contour(i_le, 0)) i_le = i;
	const double s_le = s[i_le];
	const int n_lower = std::max(3, static_cast<int>(std::lround((n - 1) * s_le / S)));
	const int n_upper = n - 1 - n_lower;
	if (n_upper < 3) throw std::invalid_argument("ResampleContour: too few nodes for the upper surface");
	Mat out(n, 2);
	auto at = [&](double target, int row) {
		target = std::clamp(target, 0.0, S);
		auto it = std::upper_bound(s.begin(), s.end(), target);
		int k = static_cast<int>(it - s.begin()) - 1;
		k = std::clamp(k, 0, m - 2);
		const double w = (s[k + 1] > s[k]) ? (target - s[k]) / (s[k + 1] - s[k]) : 0.0;
		out(row, 0) = contour(k, 0) + w * (contour(k + 1, 0) - contour(k, 0));
		out(row, 1) = contour(k, 1) + w * (contour(k + 1, 1) - contour(k, 1));
	};
	// Fraction of a half's arc at fraction t of its nodes, measured from the
	// TE (lower half) or from the LE (upper half): a blend of the full cosine
	// (clustered at both ends) and a quarter cosine (clustered at the LE only).
	const double w = std::clamp(te_cluster, 0.0, 1.0);
	auto from_te = [&](double t) { return w * 0.5 * (1.0 - std::cos(kPi * t)) + (1.0 - w) * std::sin(0.5 * kPi * t); };
	auto from_le = [&](double t) { return w * 0.5 * (1.0 - std::cos(kPi * t)) + (1.0 - w) * (1.0 - std::cos(0.5 * kPi * t)); };
	for (int k = 0; k <= n_lower; ++k) at(s_le * from_te(static_cast<double>(k) / n_lower), k);
	for (int k = 1; k <= n_upper; ++k) at(s_le + (S - s_le) * from_le(static_cast<double>(k) / n_upper), n_lower + k);
	// Exact closure and the exact leading-edge point.
	out(n - 1, 0) = out(0, 0) = 0.5 * (out(0, 0) + out(n - 1, 0));
	out(n - 1, 1) = out(0, 1) = 0.5 * (out(0, 1) + out(n - 1, 1));
	return out;
}

int StagnationInterval(const Paneling& g, const std::vector<double>& gamma, double* s_stag) {
	const int N = g.n_airfoil();
	// The sign change nearest the leading edge (minimum x).
	int best = -1;
	double best_x = 1e300;
	for (int i = 0; i + 1 < N; ++i) {
		if (gamma[i] < 0.0 && gamma[i + 1] >= 0.0) {
			const double xm = 0.5 * (g.nodes[i].x + g.nodes[i + 1].x);
			if (xm < best_x) { best_x = xm; best = i; }
		}
	}
	if (best < 0) {
		// No clean sign change: take the node of smallest |gamma| near the LE.
		int i_le = 0;
		for (int i = 1; i < N; ++i) if (g.nodes[i].x < g.nodes[i_le].x) i_le = i;
		best = std::clamp(i_le, 0, N - 2);
	}
	if (s_stag) {
		// u_e = -gamma on the lower node, +gamma on the upper: both positive
		// away from stagnation; s_stag = (ue2 s1 + ue1 s2) / (ue1 + ue2).
		const double ue1 = std::fabs(gamma[best]), ue2 = std::fabs(gamma[best + 1]);
		const double den = ue1 + ue2;
		*s_stag = den > 0.0 ? (ue2 * g.s[best] + ue1 * g.s[best + 1]) / den : g.s[best];
	}
	return best;
}

Vec2 InviscidVelocityAt(const InviscidSolution& sol, const std::vector<double>& gamma, const Vec2& x, double alpha_rad) {
	Vec2 v(std::cos(alpha_rad), std::sin(alpha_rad));
	const Paneling& g = sol.geom;
	for (int k = 0; k + 1 < g.n_airfoil(); ++k) {
		const VelCoef2 c = VelLinearVortex(FrameOf(g.nodes[k], g.nodes[k + 1], x));
		v = v + c.v1 * gamma[k] + c.v2 * gamma[k + 1];
	}
	return v;
}

std::vector<double> InviscidSolution::GammaAt(double alpha_rad) const {
	std::vector<double> g(gamma0.size());
	const double c = std::cos(alpha_rad), s = std::sin(alpha_rad);
	for (size_t i = 0; i < g.size(); ++i) g[i] = gamma0[i] * c + gamma90[i] * s;
	return g;
}

std::vector<double> InviscidSolution::CpAt(double alpha_rad) const {
	std::vector<double> g = GammaAt(alpha_rad);
	for (double& v : g) v = 1.0 - v * v;
	return g;
}

PanelLoads IntegrateLoads(const Paneling& g, const std::vector<double>& cp, double alpha_rad, const Vec2& x_ref) {
	// Clockwise polygon: outward n ds = (-dy, dx); dF = -Cp n ds = Cp (dy, -dx)
	// per unit dynamic pressure, with Cp linear over the panel (the paper's
	// eqs. 33-35, the moment with the [2 1; 1 2]/6 weights).
	const int N = g.n_airfoil();
	const double ca = std::cos(alpha_rad), sa = std::sin(alpha_rad);
	double fx = 0.0, fy = 0.0, m = 0.0;
	for (int i = 0; i + 1 < N; ++i) {
		const double dx = g.nodes[i + 1].x - g.nodes[i].x, dy = g.nodes[i + 1].y - g.nodes[i].y;
		const double cbar = 0.5 * (cp[i] + cp[i + 1]);
		fx += cbar * dy;
		fy += -cbar * dx;
		// torque (counter-clockwise) of the linearly varying load: weights 2/6, 1/6
		const double rx1 = g.nodes[i].x - x_ref.x, ry1 = g.nodes[i].y - x_ref.y;
		const double rx2 = g.nodes[i + 1].x - x_ref.x, ry2 = g.nodes[i + 1].y - x_ref.y;
		// dF per unit length of the panel at its ends: (dy, -dx) cp / d, integrated with the cp weights
		const double w11 = 2.0 / 6.0, w12 = 1.0 / 6.0;
		const double c1 = cp[i], c2 = cp[i + 1];
		const double f1 = w11 * c1 + w12 * c2, f2 = w12 * c1 + w11 * c2; // effective loads attached to the ends
		m += (rx1 * (-dx) - ry1 * dy) * f1 + (rx2 * (-dx) - ry2 * dy) * f2;
	}
	PanelLoads L;
	L.cl = (fy * ca - fx * sa) / g.chord;
	L.cd = (fx * ca + fy * sa) / g.chord;
	L.cm = -m / (g.chord * g.chord); // nose-up positive
	return L;
}

InviscidSolution SolveInviscid(const Mat& contour, const InviscidOptions& opt) {
	InviscidSolution sol;
	Paneling& g = sol.geom;
	const int N = contour.rows();
	if (N < 8 || contour.cols() != 2) throw std::invalid_argument("SolveInviscid: need an (N, 2) contour");
	g.nodes.resize(N);
	g.s.assign(N, 0.0);
	double xmin = 1e300, xmax = -1e300;
	for (int i = 0; i < N; ++i) {
		g.nodes[i] = Vec2(contour(i, 0), contour(i, 1));
		xmin = std::min(xmin, g.nodes[i].x);
		xmax = std::max(xmax, g.nodes[i].x);
		if (i > 0) g.s[i] = g.s[i - 1] + (g.nodes[i] - g.nodes[i - 1]).norm();
	}
	g.chord = xmax - xmin;
	g.x_min = xmin;
	if ((g.nodes.front() - g.nodes.back()).norm() > 1e-9 * g.chord) throw std::invalid_argument("SolveInviscid: the trailing edge must be closed (coincident end nodes)");
	g.te = g.nodes.front() * 0.5 + g.nodes.back() * 0.5;
	{
		const Vec2 d_lower = g.nodes[1] - g.nodes[0];
		const Vec2 d_upper = g.nodes[N - 2] - g.nodes[N - 1];
		Vec2 inward = d_lower * (1.0 / d_lower.norm()) + d_upper * (1.0 / d_upper.norm());
		const double nrm = inward.norm();
		g.te_bisector = nrm > 1e-12 ? inward * (-1.0 / nrm) : Vec2(1.0, 0.0);
	}

	// ---- the augmented streamfunction system ---------------------------------
	// Unknowns gamma_1..gamma_N, Psi0. Rows: nodes 1..N-1 (Psi_i - Psi0 = -Psi_inf,i),
	// the sharp-TE extrapolation row, the Kutta row.
	const int n = N + 1;
	std::vector<double> A(static_cast<size_t>(n) * n, 0.0);
	std::vector<double> rhs0(n, 0.0), rhs90(n, 0.0);
	for (int i = 0; i < N - 1; ++i) {
		for (int k = 0; k + 1 < N; ++k) {
			const Coef2 c = PsiLinearVortex(FrameOf(g.nodes[k], g.nodes[k + 1], g.nodes[i]));
			A[i * n + k] += c.c1;
			A[i * n + k + 1] += c.c2;
		}
		A[i * n + N] = -1.0;
		// Psi_inf = y cos(alpha) - x sin(alpha): alpha = 0 -> y, alpha = 90 -> -x
		rhs0[i] = -g.nodes[i].y;
		rhs90[i] = g.nodes[i].x;
	}
	{
		// Sharp trailing edge: the difference d_k = gamma_k - gamma_{N-1-k}
		// between the lower and upper values extrapolated to the end,
		// d_0 = 2 d_1 - d_2 (XFOIL / mfoil's row [1 -2 1 -1 2 -1]; the paper's
		// typeset equation has the left-hand side's sign the other way).
		// With the Kutta row this gives the trailing-edge speed as the
		// extrapolation of the neighbours', finite, as the viscous flow has.
		const int r = N - 1;
		A[r * n + 0] += 1.0;
		A[r * n + 1] -= 2.0;
		A[r * n + 2] += 1.0;
		A[r * n + (N - 3)] -= 1.0;
		A[r * n + (N - 2)] += 2.0;
		A[r * n + (N - 1)] -= 1.0;
	}
	{
		const int r = N;
		A[r * n + 0] = 1.0;
		A[r * n + (N - 1)] = 1.0;
	}
	sol.lu = DenseLU(A, n);
	if (!sol.lu.ok()) throw std::runtime_error("SolveInviscid: singular streamfunction system");
	sol.lu.Solve(rhs0);
	sol.lu.Solve(rhs90);
	sol.gamma0.assign(rhs0.begin(), rhs0.begin() + N);
	sol.gamma90.assign(rhs90.begin(), rhs90.begin() + N);
	sol.psi0_0 = rhs0[N];
	sol.psi0_90 = rhs90[N];

	// ---- direction factors and the stagnation interval at the wake incidence ---
	const double alpha_w = opt.alpha_wake_deg * kPi / 180.0;
	const std::vector<double> gamma_w = sol.GammaAt(alpha_w);
	sol.stagnation_lower = StagnationInterval(g, gamma_w);
	sol.direction.assign(N, 1);
	for (int i = 0; i <= sol.stagnation_lower; ++i) sol.direction[i] = -1;

	// ---- the wake streamline ------------------------------------------------------
	const double L = opt.wake_length * g.chord;
	int Nw = opt.n_wake > 0 ? opt.n_wake : static_cast<int>(std::lround(N / 10.0 + 10.0 * opt.wake_length));
	Nw = std::max(Nw, 4);
	g.wake.resize(Nw);
	g.sw.resize(Nw);
	const double s0 = opt.eps_wake * g.chord;
	// Geometric spacing: first step the TE panel length, ratio r such that
	// s0 + h (r^(Nw-1) - 1) / (r - 1) = L.
	const double h0 = std::max(0.5 * ((g.nodes[1] - g.nodes[0]).norm() + (g.nodes[N - 1] - g.nodes[N - 2]).norm()), 1e-4 * g.chord);
	double r = 1.0;
	{
		auto total = [&](double rr) { return rr == 1.0 ? h0 * (Nw - 1) : h0 * (std::pow(rr, Nw - 1) - 1.0) / (rr - 1.0); };
		double lo = 1.0, hi = 2.0;
		if (total(lo) >= L - s0) r = 1.0;
		else {
			while (total(hi) < L - s0) hi *= 1.5;
			for (int it = 0; it < 80; ++it) { const double mid = 0.5 * (lo + hi); if (total(mid) < L - s0) lo = mid; else hi = mid; }
			r = 0.5 * (lo + hi);
		}
	}
	Vec2 p = g.te + g.te_bisector * s0;
	g.wake[0] = p;
	g.sw[0] = s0;
	double step = h0;
	for (int k = 1; k < Nw; ++k) {
		// predictor-corrector along the unit velocity
		Vec2 v1 = InviscidVelocityAt(sol, gamma_w, p, alpha_w);
		double n1 = v1.norm();
		if (!(n1 > 1e-12)) v1 = g.te_bisector; else v1 = v1 * (1.0 / n1);
		const Vec2 pp = p + v1 * step;
		Vec2 v2 = InviscidVelocityAt(sol, gamma_w, pp, alpha_w);
		double n2 = v2.norm();
		if (!(n2 > 1e-12)) v2 = v1; else v2 = v2 * (1.0 / n2);
		Vec2 dir = v1 * 0.5 + v2 * 0.5;
		dir = dir * (1.0 / std::max(dir.norm(), 1e-12));
		p = p + dir * step;
		g.wake[k] = p;
		g.sw[k] = g.sw[k - 1] + step;
		step *= r;
	}
	return sol;
}

} // namespace vortexpanel
} // namespace aerotelic
