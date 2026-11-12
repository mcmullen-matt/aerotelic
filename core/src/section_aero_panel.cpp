// The vortex-panel registry entries of section_aero.hpp: the inviscid
// streamfunction panel method (vortex_panel.hpp). The coupled integral
// boundary-layer method builds on the same paneling.

#include "aerotelic/section_aero.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "aerotelic/ibl_airfoil.hpp"
#include "aerotelic/vortex_panel.hpp"

namespace aerotelic {

namespace {

constexpr double kPi = 3.14159265358979323846;
using namespace vortexpanel;

// Fills the two surface distributions of a result from node values of the
// edge speed on a paneling: from the interpolated stagnation point along
// the lower surface to node 1 and along the upper surface to node N.
void FillSurfaces(SectionAeroResult& r, const Paneling& g, const std::vector<double>& gamma) {
	double s_stag = 0.0;
	const int i_lo = StagnationInterval(g, gamma, &s_stag);
	const int N = g.n_airfoil();
	// Interpolated stagnation point.
	const double w = (g.s[i_lo + 1] > g.s[i_lo]) ? (s_stag - g.s[i_lo]) / (g.s[i_lo + 1] - g.s[i_lo]) : 0.0;
	const Vec2 p_stag = g.nodes[i_lo] * (1.0 - w) + g.nodes[i_lo + 1] * w;
	auto fill = [&](SectionSurface& sf, int from, int step, int count) {
		sf.x.clear(); sf.y.clear(); sf.s.clear(); sf.q.clear(); sf.cp.clear();
		auto push = [&](const Vec2& p, double q, double s_arc) {
			sf.x.push_back((p.x - g.x_min) / g.chord);
			sf.y.push_back(p.y / g.chord);
			sf.s.push_back(s_arc / g.chord);
			sf.q.push_back(q);
			sf.cp.push_back(1.0 - q * q);
		};
		push(p_stag, 0.0, 0.0);
		for (int k = 0, i = from; k < count; ++k, i += step) push(g.nodes[i], std::fabs(gamma[i]), std::fabs(g.s[i] - s_stag));
	};
	fill(r.lower, i_lo, -1, i_lo + 1);
	fill(r.upper, i_lo + 1, +1, N - 1 - i_lo);
}

class VortexPanelInviscidMethod : public SectionAeroMethod {
public:
	std::string Name() const override { return "Vortex panel (inviscid)"; }
	std::string Description() const override {
		return "Linear-vortex / constant-source streamfunction panel method of XFOIL as documented by Fidkowski (mfoil), "
			"inviscid: flow tangency at the nodes, Kutta condition, sharp trailing edge. The potential-flow half of the coupled solver.";
	}

	SectionAeroResult Solve(const SectionAeroRequest& request) const override {
		SectionAeroResult r;
		r.method = Name();
		r.alpha_deg = request.alpha_deg;
		r.reynolds = request.reynolds;
		const auto t0 = std::chrono::steady_clock::now();
		try {
			const Mat closed = CloseTrailingEdge(request.contour, request.te_blend_start, &r.te_gap_closed);
			r.contour = ResampleContour(closed, std::max(40, request.panel_nodes));
			InviscidOptions opt;
			opt.alpha_wake_deg = request.alpha_deg;
			const InviscidSolution sol = SolveInviscid(r.contour, opt);
			const Paneling& g = sol.geom;
			const double alpha = request.alpha_deg * kPi / 180.0;
			const std::vector<double> gamma = sol.GammaAt(alpha);
			const std::vector<double> cp = sol.CpAt(alpha);
			const PanelLoads le = IntegrateLoads(g, cp, alpha, Vec2(g.x_min, 0.0));
			const PanelLoads c4 = IntegrateLoads(g, cp, alpha, Vec2(g.x_min + 0.25 * g.chord, 0.0));
			r.cl = le.cl;
			r.cd_pressure = le.cd;
			r.cm_le = le.cm;
			r.cm_c4 = c4.cm;
			r.cl_exact = std::numeric_limits<double>::quiet_NaN();
			// cl(alpha) = cl0 cos + cl90 sin from the reference solutions.
			const PanelLoads l0 = IntegrateLoads(g, sol.CpAt(0.0), 0.0, Vec2(g.x_min, 0.0));
			const PanelLoads l90 = IntegrateLoads(g, sol.CpAt(0.5 * kPi), 0.5 * kPi, Vec2(g.x_min, 0.0));
			// (Cp is quadratic in gamma, so cl is not exactly cl0 cos + cl90 sin; the
			// zero-lift angle and slope are taken from a central difference instead.)
			(void)l0; (void)l90;
			const double da = 0.5 * kPi / 180.0;
			const double clp = IntegrateLoads(g, sol.CpAt(alpha + da), alpha + da, Vec2(g.x_min, 0.0)).cl;
			const double clm = IntegrateLoads(g, sol.CpAt(alpha - da), alpha - da, Vec2(g.x_min, 0.0)).cl;
			r.cl_alpha = (clp - clm) / (2.0 * da);
			r.alpha0_deg = (r.cl_alpha != 0.0) ? (request.alpha_deg - r.cl / r.cl_alpha) : std::numeric_limits<double>::quiet_NaN();
			r.cp_min = *std::min_element(cp.begin(), cp.end());
			// Trailing-edge angle from the paneling's end tangents.
			{
				const Vec2 d_l = g.nodes[1] - g.nodes[0], d_u = g.nodes[g.n_airfoil() - 2] - g.nodes[g.n_airfoil() - 1];
				const double c = std::clamp(d_l.dot(d_u) / (d_l.norm() * d_u.norm()), -1.0, 1.0);
				r.te_angle_deg = std::acos(c) * 180.0 / kPi;
			}
			FillSurfaces(r, g, gamma);
			r.iterations = 0;
			r.residual = 0.0;
			r.convergence_margin = sol.lu.pivot_ratio();
			r.ok = true;
		} catch (const std::exception& e) {
			r.ok = false;
			r.error = e.what();
		}
		r.solve_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
		return r;
	}
};

// ---- the coupled boundary-layer method ----------------------------------------
class VortexPanelIblMethod : public SectionAeroMethod {
public:
	std::string Name() const override { return "Vortex panel + boundary layer (mfoil)"; }
	std::string Description() const override {
		return "XFOIL-class viscous analysis as documented by Fidkowski (mfoil): the vortex-panel potential flow coupled "
			"through transpiration sources to Drela's integral boundary layer on the airfoil and wake, e^N transition "
			"(n_crit, or forced), the coupled Newton solver with update limiting, Squire-Young drag. Incompressible.";
	}
	bool Fast() const override { return false; }
	bool Viscous() const override { return true; }

	SectionAeroResult Solve(const SectionAeroRequest& request) const override {
		SectionAeroResult r;
		r.method = Name();
		r.alpha_deg = request.alpha_deg;
		r.reynolds = request.reynolds;
		const auto t0 = std::chrono::steady_clock::now();
		try {
			const Mat closed = CloseTrailingEdge(request.contour, request.te_blend_start, &r.te_gap_closed);
			// Low trailing-edge clustering: dense TE panels make the transpiration
			// source dm/ds ill-conditioned and the Newton iteration erratic
			// (measured 2026-11-12: full cosine clustering converged for 2 of 7
			// node counts, 0.2 for all of them).
			r.contour = ResampleContour(closed, std::max(40, request.panel_nodes), 0.2);
			IblOptions opt;
			opt.reynolds = request.reynolds;
			opt.ncrit = request.ncrit;
			opt.xft_upper = request.xft_upper;
			opt.xft_lower = request.xft_lower;
			opt.h_sep_lam = request.h_sep_lam;
			opt.h_sep_turb = request.h_sep_turb;
			const IblResult ib = SolveIblAirfoil(r.contour, request.alpha_deg, opt);
			if (!ib.ok) { r.error = ib.error; r.solve_ms = ib.solve_ms; return r; }
			r.converged = ib.converged;
			r.warnings = ib.warnings;
			if (!ib.converged) r.warnings.push_back("coupled solver did not converge in " + std::to_string(ib.iterations) + " iterations (|R| = " + std::to_string(ib.residual_norm) + "); result is the last iterate");
			r.iterations = ib.iterations;
			r.residual = ib.residual_norm;
			r.cl = ib.cl;
			r.cm_le = ib.cm_le;
			r.cm_c4 = ib.cm_c4;
			r.cd_total = ib.cd;
			r.cd_friction = ib.cdf;
			r.cd_pressure = ib.cdp;
			r.cl_exact = std::numeric_limits<double>::quiet_NaN();
			r.cl_alpha = std::numeric_limits<double>::quiet_NaN();
			r.alpha0_deg = std::numeric_limits<double>::quiet_NaN();
			r.cp_min = 1.0;
			for (int i = 0; i < ib.n_airfoil; ++i) r.cp_min = std::min(r.cp_min, ib.nodes[i].cp);
			{
				const int N = ib.n_airfoil;
				const double ax = ib.nodes[1].x - ib.nodes[0].x, ay = ib.nodes[1].y - ib.nodes[0].y;
				const double bx = ib.nodes[N - 2].x - ib.nodes[N - 1].x, by = ib.nodes[N - 2].y - ib.nodes[N - 1].y;
				const double c = std::clamp((ax * bx + ay * by) / (std::hypot(ax, ay) * std::hypot(bx, by)), -1.0, 1.0);
				r.te_angle_deg = std::acos(c) * 180.0 / kPi;
			}
			auto fill = [&](SectionSurface& s, const std::vector<int>& idx, double xt, bool forced, bool upper) {
				s = SectionSurface();
				auto push = [&](double x, double y, double xi, double ue, double cp, double th, double ds, double hk, double cf, double sa, bool tb) {
					s.x.push_back((x - ib.x_min) / ib.chord);
					s.y.push_back(y / ib.chord);
					s.s.push_back(xi / ib.chord);
					s.q.push_back(ue);
					s.cp.push_back(cp);
					s.theta.push_back(th / ib.chord);
					s.delta_star.push_back(ds / ib.chord);
					s.hk.push_back(hk);
					s.cf.push_back(cf);
					s.amp.push_back(sa);
					s.turb.push_back(tb ? 1 : 0);
				};
				push(ib.x_stag, ib.y_stag, 0.0, 0.0, 1.0, ib.nodes[idx.front()].theta, ib.nodes[idx.front()].delta_star, ib.nodes[idx.front()].hk, 0.0, 0.0, false);
				for (int i : idx) {
					const IblNode& n = ib.nodes[i];
					push(n.x, n.y, n.xi, n.ue, n.cp, n.theta, n.delta_star, n.hk, n.cf, n.sa, n.turb);
				}
				s.stratford.assign(s.x.size(), 0.0);
				int ip = 0;
				for (size_t k = 1; k < s.q.size(); ++k) if (s.q[k] > s.q[ip]) ip = static_cast<int>(k);
				s.peak_index = ip;
				// transition cue
				if (xt >= 0.0) {
					s.transition.found = true;
					s.transition.x = xt;
					// y and s at that x along the surface
					for (size_t k = 1; k < s.x.size(); ++k) {
						if ((s.x[k - 1] <= xt && xt <= s.x[k]) || (s.x[k] <= xt && xt <= s.x[k - 1])) {
							const double t = (s.x[k] == s.x[k - 1]) ? 0.0 : (xt - s.x[k - 1]) / (s.x[k] - s.x[k - 1]);
							s.transition.y = s.y[k - 1] + t * (s.y[k] - s.y[k - 1]);
							s.transition.s = s.s[k - 1] + t * (s.s[k] - s.s[k - 1]);
							break;
						}
					}
					s.transition.note = forced ? "forced transition" : "e^N amplification reached n_crit = " + std::to_string(request.ncrit);
				} else {
					s.transition.note = "laminar to the trailing edge (transition forced at the wake)";
				}
				// separation by shape factor, detachment by cf <= 0
				for (size_t k = 1; k < s.x.size(); ++k) {
					const bool tb = s.turb[k] != 0;
					const double limit = tb ? request.h_sep_turb : request.h_sep_lam;
					if (!s.separation.found && s.hk[k] >= limit) {
						s.separation.found = true;
						s.separation.x = s.x[k]; s.separation.y = s.y[k]; s.separation.s = s.s[k];
						s.separation.note = (tb ? "turbulent H_k reached " : "laminar H_k reached ") + std::to_string(limit);
					}
					if (!s.detachment.found && tb && s.cf[k] <= 0.0 && s.x[k] < 0.999) {
						s.detachment.found = true;
						s.detachment.x = s.x[k]; s.detachment.y = s.y[k]; s.detachment.s = s.s[k];
						s.detachment.note = "turbulent c_f fell to zero (closure detachment)";
					}
				}
				if (!s.separation.found) s.separation.note = "H_k below the thresholds everywhere";
				if (!s.detachment.found) s.detachment.note = "c_f positive to the trailing edge";
				(void)upper;
			};
			fill(r.lower, ib.lower, ib.xt_lower, ib.xt_lower_forced, false);
			fill(r.upper, ib.upper, ib.xt_upper, ib.xt_upper_forced, true);
			// skin-friction shares by surface are not separated by the solver; keep the total on the result
			r.upper.cd_friction = 0.5 * ib.cdf;
			r.lower.cd_friction = 0.5 * ib.cdf;
			r.ok = true;
		} catch (const std::exception& e) {
			r.ok = false;
			r.error = e.what();
		}
		r.solve_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
		return r;
	}
};

} // namespace

std::unique_ptr<SectionAeroMethod> MakeVortexPanelInviscidMethod() {
	return std::make_unique<VortexPanelInviscidMethod>();
}

std::unique_ptr<SectionAeroMethod> MakeVortexPanelIblMethod() {
	return std::make_unique<VortexPanelIblMethod>();
}

} // namespace aerotelic
