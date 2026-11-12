// Checks the streamfunction vortex-panel method (vortex_panel.hpp):
// the panel influence coefficients against Gauss quadrature of the point
// kernels, the branch-cut placement, the inviscid solution against the
// exact conformal map on the same contour, the wake streamline, and the
// registry entry through the shared section layer.

#include <cmath>
#include <cstdio>
#include <vector>

#include "aerotelic/airfoil.hpp"
#include "aerotelic/section_aero.hpp"
#include "aerotelic/vortex_panel.hpp"

using namespace aerotelic;
using namespace aerotelic::vortexpanel;

namespace {

int g_failures = 0;
constexpr double kPi = 3.14159265358979323846;

void Check(bool ok, const char* what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what);
	if (!ok) ++g_failures;
}

// 64-point Gauss-Legendre on [0, 1].
void Gauss(std::vector<double>& x, std::vector<double>& w) {
	const int n = 64;
	x.resize(n); w.resize(n);
	for (int i = 0; i < n; ++i) {
		double z = std::cos(kPi * (i + 0.75) / (n + 0.5)), pp = 0.0;
		for (int it = 0; it < 100; ++it) {
			double p1 = 1.0, p2 = 0.0;
			for (int j = 1; j <= n; ++j) { const double p3 = p2; p2 = p1; p1 = ((2.0 * j - 1.0) * z * p2 - (j - 1.0) * p3) / j; }
			pp = n * (z * p1 - p2) / (z * z - 1.0);
			const double z1 = z; z = z1 - p1 / pp;
			if (std::fabs(z - z1) < 1e-15) break;
		}
		x[i] = 0.5 * (1.0 - z);
		w[i] = 1.0 / ((1.0 - z * z) * pp * pp);
	}
}

double InterpSurface(const SectionSurface& s, double x) {
	for (size_t k = 1; k < s.x.size(); ++k) {
		if ((s.x[k - 1] <= x && x <= s.x[k]) || (s.x[k] <= x && x <= s.x[k - 1])) {
			const double t = (s.x[k] == s.x[k - 1]) ? 0.0 : (x - s.x[k - 1]) / (s.x[k] - s.x[k - 1]);
			return s.cp[k - 1] + t * (s.cp[k] - s.cp[k - 1]);
		}
	}
	return std::nan("");
}

} // namespace

int main() {
	setvbuf(stdout, nullptr, _IONBF, 0);

	// ---- influence coefficients vs quadrature ------------------------------------
	{
		std::vector<double> gx, gw;
		Gauss(gx, gw);
		const Vec2 p1(0.1, 0.2), p2(0.9, 0.35);
		const Vec2 e = p2 - p1;
		const double d = e.norm();
		const Vec2 t = e * (1.0 / d), n(-t.y, t.x);
		const Vec2 pts[] = { Vec2(0.5, 0.6), Vec2(1.5, 0.2), Vec2(0.4, -0.4), Vec2(-0.3, -0.2), Vec2(-0.3, 0.5) };
		double worst_psi_v = 0.0, worst_vel_v = 0.0, worst_vel_s = 0.0, worst_psi_s = 0.0;
		for (const Vec2& x : pts) {
			const PanelFrame f = FrameOf(p1, p2, x);
			// quadrature of the kernels
			double psi1 = 0.0, psi2 = 0.0, vt1 = 0.0, vt2 = 0.0, vn1 = 0.0, vn2 = 0.0;
			double st1 = 0.0, st2 = 0.0, sn1 = 0.0, sn2 = 0.0, sp1 = 0.0, sp2 = 0.0;
			for (size_t q = 0; q < gx.size(); ++q) {
				const double sp = gx[q] * d, ww = gw[q] * d;
				const double w1 = 1.0 - sp / d, w2 = sp / d;
				const double dx = f.a - sp, r2 = dx * dx + f.h * f.h, r = std::sqrt(r2);
				psi1 += ww * w1 * std::log(r) / (2 * kPi); psi2 += ww * w2 * std::log(r) / (2 * kPi);
				vt1 += ww * w1 * f.h / r2 / (2 * kPi); vt2 += ww * w2 * f.h / r2 / (2 * kPi);
				vn1 += -ww * w1 * dx / r2 / (2 * kPi); vn2 += -ww * w2 * dx / r2 / (2 * kPi);
				st1 += ww * w1 * dx / r2 / (2 * kPi); st2 += ww * w2 * dx / r2 / (2 * kPi);
				sn1 += ww * w1 * f.h / r2 / (2 * kPi); sn2 += ww * w2 * f.h / r2 / (2 * kPi);
				const double th = std::atan2(f.h, dx);
				sp1 += ww * w1 * th / (2 * kPi); sp2 += ww * w2 * th / (2 * kPi);
			}
			const Coef2 cv = PsiLinearVortex(f);
			worst_psi_v = std::fmax(worst_psi_v, std::fmax(std::fabs(cv.c1 - psi1), std::fabs(cv.c2 - psi2)));
			const VelCoef2 vv = VelLinearVortex(f);
			worst_vel_v = std::fmax(worst_vel_v, std::fmax(std::fabs(vv.v1.dot(t) - vt1), std::fabs(vv.v2.dot(t) - vt2)));
			worst_vel_v = std::fmax(worst_vel_v, std::fmax(std::fabs(vv.v1.dot(n) - vn1), std::fabs(vv.v2.dot(n) - vn2)));
			const VelCoef2 vs = VelLinearSource(f);
			worst_vel_s = std::fmax(worst_vel_s, std::fmax(std::fabs(vs.v1.dot(t) - st1), std::fabs(vs.v2.dot(t) - st2)));
			worst_vel_s = std::fmax(worst_vel_s, std::fmax(std::fabs(vs.v1.dot(n) - sn1), std::fabs(vs.v2.dot(n) - sn2)));
			// source streamfunction: the quadrature carries atan2's own cut; compare
			// where the panel's cut shift does not apply (th1 + th2 <= pi)
			if (f.th1 + f.th2 <= kPi) {
				const Coef2 cs = PsiLinearSource(f);
				worst_psi_s = std::fmax(worst_psi_s, std::fmax(std::fabs(cs.c1 - sp1), std::fabs(cs.c2 - sp2)));
			}
		}
		std::printf("kernel vs quadrature: vortex psi %.2e, vortex vel %.2e, source psi %.2e, source vel %.2e\n", worst_psi_v, worst_vel_v, worst_psi_s, worst_vel_s);
		Check(worst_psi_v < 1e-9, "linear vortex streamfunction coefficients (eqs. 37-38)");
		Check(worst_vel_v < 1e-9, "linear vortex velocity coefficients (eqs. 39-40)");
		Check(worst_psi_s < 1e-9, "linear source streamfunction coefficients (eqs. 41-42)");
		Check(worst_vel_s < 1e-9, "linear source velocity coefficients (eqs. 43-44)");

		// The constant-source cut lies along +n from the midpoint: continuous
		// across the panel's line behind node 1 and beyond node 2, discontinuous
		// by d across the outward normal.
		const Vec2 behind = p1 - t * 0.5, beyond = p2 + t * 0.5, mid = p1 * 0.5 + p2 * 0.5;
		const double eps = 1e-7;
		const double b_up = PsiConstSource(FrameOf(p1, p2, behind + n * eps)), b_dn = PsiConstSource(FrameOf(p1, p2, behind - n * eps));
		const double y_up = PsiConstSource(FrameOf(p1, p2, beyond + n * eps)), y_dn = PsiConstSource(FrameOf(p1, p2, beyond - n * eps));
		const double c_l = PsiConstSource(FrameOf(p1, p2, mid + n * 0.5 - t * eps)), c_r = PsiConstSource(FrameOf(p1, p2, mid + n * 0.5 + t * eps));
		std::printf("source cut: behind %.2e beyond %.2e, jump across +n ray %.6f (d = %.6f)\n", std::fabs(b_up - b_dn), std::fabs(y_up - y_dn), std::fabs(c_l - c_r), d);
		Check(std::fabs(b_up - b_dn) < 1e-5 && std::fabs(y_up - y_dn) < 1e-5, "no cut along the panel's line");
		Check(std::fabs(std::fabs(c_l - c_r) - d) < 1e-5, "the cut of strength d lies along the outward normal");
	}

	// ---- inviscid solution vs the exact conformal map -------------------------------
	const auto& methods = SectionAeroMethods();
	const SectionAeroMethod* conformal = FindSectionAeroMethod("Conformal map (Theodorsen/Ives)");
	const SectionAeroMethod* panel = FindSectionAeroMethod("Vortex panel (inviscid)");
	Check(conformal && panel && methods.size() >= 2, "both methods registered");
	if (conformal && panel) {
		struct Case { const char* code; double alpha; double tol_cp; double tol_cl; };
		const Case cases[] = { {"0012", 0.0, 1.5e-2, 1e-3}, {"0012", 6.0, 3e-2, 1.5e-2}, {"2412", 2.0, 2e-2, 1e-2}, {"4415", 4.0, 3e-2, 1.5e-2} };
		for (const Case& c : cases) {
			std::unique_ptr<Airfoil> af = Airfoil::Naca4(c.code, 81);
			SectionAeroRequest req = SectionRequestFor(*af);
			req.alpha_deg = c.alpha;
			req.panel_nodes = 200;
			const SectionAeroResult rc = SolveSection(*conformal, req);
			const SectionAeroResult rp = SolveSection(*panel, req);
			double worst = 0.0, worst_x = 0.0;
			int compared = 0;
			for (int side = 0; side < 2; ++side) {
				const SectionSurface& sp = side == 0 ? rp.upper : rp.lower;
				const SectionSurface& sc = side == 0 ? rc.upper : rc.lower;
				for (size_t k = 0; k < sp.x.size(); ++k) {
					if (sp.x[k] < 0.01 || sp.x[k] > 0.97) continue; // leading-edge peak and inviscid TE recovery excluded
					const double ref = InterpSurface(sc, sp.x[k]);
					if (std::isnan(ref)) continue;
					++compared;
					const double e = std::fabs(ref - sp.cp[k]);
					if (e > worst) { worst = e; worst_x = sp.x[k]; }
				}
			}
			std::printf("NACA %s alpha %.0f: panel ok=%d (%s) cl %.4f cm_c4 %.4f cd %.5f cp_min %.3f %.1f ms | conformal cl %.4f cm_c4 %.4f cp_min %.3f | %d pts, worst dCp %.2e at x/c %.3f\n",
				c.code, c.alpha, rp.ok, rp.error.c_str(), rp.cl, rp.cm_c4, rp.cd_pressure, rp.cp_min, rp.solve_ms, rc.cl, rc.cm_c4, rc.cp_min, compared, worst, worst_x);
			Check(rp.ok, "panel method solves");
			Check(compared > 150, "surfaces compared");
			Check(worst < c.tol_cp, "panel Cp matches the exact map");
			Check(std::fabs(rp.cl - rc.cl) < c.tol_cl, "panel lift matches the exact map");
			Check(std::fabs(rp.cm_c4 - rc.cm_c4) < 6e-3, "panel moment matches the exact map");
			Check(std::fabs(rp.cd_pressure) < 3e-3, "panel pressure drag is a small residual");
			Check(rp.upper.cp.front() > 0.999 && rp.lower.cp.front() > 0.999, "surfaces start at the stagnation point");
		}
	}

	// ---- paneling details: Kutta, stagnation, wake -------------------------------------
	{
		std::unique_ptr<Airfoil> af = Airfoil::Naca4("2412", 81);
		const Mat closed = CloseTrailingEdge(af->ContourPoints(600), 0.7);
		const Mat nodes = ResampleContour(closed, 160);
		Check(nodes.rows() == 160 && nodes(0, 0) == nodes(159, 0) && nodes(0, 1) == nodes(159, 1), "resampled contour closed with 160 nodes");
		InviscidOptions opt;
		opt.alpha_wake_deg = 3.0;
		const InviscidSolution sol = SolveInviscid(nodes, opt);
		const std::vector<double> gamma = sol.GammaAt(3.0 * kPi / 180.0);
		Check(std::fabs(gamma.front() + gamma.back()) < 1e-10, "Kutta condition gamma_1 + gamma_N = 0");
		Check(gamma.back() > 0.0 && gamma.front() < 0.0, "clockwise gamma positive on the upper surface, negative on the lower, to the trailing-edge nodes");
		std::printf("trailing-edge gamma %.4f / %.4f, neighbours %.4f / %.4f\n", gamma.front(), gamma.back(), gamma[1], gamma[gamma.size() - 2]);
		Check(std::fabs(gamma.back() - gamma[gamma.size() - 2]) < 0.15, "the extrapolated trailing-edge speed is close to its neighbour's");
		double s_stag = 0.0;
		const int i_lo = StagnationInterval(sol.geom, gamma, &s_stag);
		std::printf("stagnation interval %d (x %.4f), s_stag %.4f; wake %d nodes, first (%.5f, %.5f), last (%.4f, %.4f)\n",
			i_lo, sol.geom.nodes[i_lo].x, s_stag, sol.geom.n_wake(), sol.geom.wake.front().x, sol.geom.wake.front().y, sol.geom.wake.back().x, sol.geom.wake.back().y);
		Check(sol.geom.nodes[i_lo].x < 0.02 && sol.geom.nodes[i_lo].y < 0.0, "stagnation at positive incidence sits just under the leading edge");
		Check(sol.direction[0] == -1 && sol.direction[sol.geom.n_airfoil() - 1] == 1, "direction factors");
		const Paneling& g = sol.geom;
		Check(g.n_wake() == static_cast<int>(std::lround(160 / 10.0 + 10.0)), "wake node count N/10 + 10 d_w");
		Check((g.wake.front() - g.te).norm() < 2e-5 && g.wake.front().x > g.te.x, "wake starts just behind the trailing edge");
		const double L = (g.wake.back() - g.te).norm();
		Check(std::fabs(g.sw.back() - 1.0) < 1e-6 && L > 0.9 && L < 1.05, "wake is one chord long");
		bool monotonic = true;
		for (int k = 1; k < g.n_wake(); ++k) if (g.wake[k].x <= g.wake[k - 1].x) monotonic = false;
		Check(monotonic, "wake marches downstream");
		// Far downstream the wake runs with the free stream.
		const Vec2 tail = g.wake.back() - g.wake[g.n_wake() - 2];
		const double ang = std::atan2(tail.y, tail.x) * 180.0 / kPi;
		std::printf("wake tail direction %.2f deg (alpha 3)\n", ang);
		Check(std::fabs(ang - 3.0) < 2.0, "wake tail within 2 deg of the free stream");
	}

	std::printf("RESULT: %d failure(s)\n", g_failures);
	return g_failures == 0 ? 0 : 1;
}
