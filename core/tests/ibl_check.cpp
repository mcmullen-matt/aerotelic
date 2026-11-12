// Checks the integral boundary-layer solver (ibl_airfoil.hpp): the
// automatic derivatives of the closures and station residual against
// finite differences (mfoil's "ping" test states), then coupled solutions
// of NACA sections against XFOIL-class expectations and the paper's
// Table 1 trends, and the registry entry through the section layer.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "aerotelic/airfoil.hpp"
#include "aerotelic/dual.hpp"
#include "aerotelic/ibl_airfoil.hpp"
#include "aerotelic/ibl_closures.hpp"
#include "aerotelic/section_aero.hpp"
#include "aerotelic/vortex_panel.hpp"

using namespace aerotelic;
using namespace aerotelic::ibl;

namespace {

int g_failures = 0;

void Check(bool ok, const char* what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what);
	if (!ok) ++g_failures;
}

using D = Dual<12>;
using SD = State<D>;
using S4 = State<double>;

SD Seed(const S4& U, int base) { SD r; for (int k = 0; k < 4; ++k) r[k] = D::Var(U[k], base + k); return r; }

// Compares the dual derivatives of f(U) (slots 0..3) with central differences.
template <class F> double PingState(const char* name, F f, const S4& U0) {
	const SD Ud = Seed(U0, 0);
	const D v = f(Ud);
	double worst = 0.0;
	for (int k = 0; k < 4; ++k) {
		const double h = 1e-6 * std::fabs(U0[k]) + 1e-9;
		S4 Up = U0, Um = U0;
		Up[k] += h; Um[k] -= h;
		const double fd = (f(SD{D(Up[0]), D(Up[1]), D(Up[2]), D(Up[3])}).v - f(SD{D(Um[0]), D(Um[1]), D(Um[2]), D(Um[3])}).v) / (2.0 * h);
		const double scale = std::fmax(std::fabs(v.d[k]), std::fabs(fd)) + 1e-12 * std::fabs(v.v) + 1e-300;
		const double rel = std::fabs(v.d[k] - fd) / scale;
		if (std::fabs(v.d[k] - fd) > 1e-7 * (1.0 + std::fabs(v.v)) ) worst = std::fmax(worst, rel);
	}
	std::printf("  %-10s value %.6e  worst rel. derivative error %.2e\n", name, v.v, worst);
	return worst;
}

void PrintResult(const char* label, const IblResult& r) {
	std::printf("%s: ok=%d conv=%d it=%d |R|=%.2e cl %.4f cm_c4 %.4f cd %.5f cdf %.5f cdp %.5f cdpi %.5f xtr up %.4f%s low %.4f%s  %.0f ms\n",
		label, r.ok, r.converged, r.iterations, r.residual_norm, r.cl, r.cm_c4, r.cd, r.cdf, r.cdp, r.cdpi,
		r.xt_upper, r.xt_upper_forced ? "(f)" : "", r.xt_lower, r.xt_lower_forced ? "(f)" : "", r.solve_ms);
	for (const std::string& w : r.warnings) std::printf("   warning: %s\n", w.c_str());
	if (!r.ok) std::printf("   error: %s\n", r.error.c_str());
}

} // namespace

int main(int argc, char** argv) {
	setvbuf(stdout, nullptr, _IONBF, 0);

	if (argc > 1 && std::string(argv[1]) == "probe") {
		// Convergence sweep: node counts and incidences. argv: probe [nodes alpha code te_cluster]
		const double tec = argc > 5 ? std::atof(argv[5]) : 1.0;
		auto contour_of = [tec](const char* code, int nodes) {
			std::unique_ptr<Airfoil> af = Airfoil::Naca4(code, 81);
			const Mat closed = CloseTrailingEdge(af->ContourPoints(600), 0.7);
			return vortexpanel::ResampleContour(closed, nodes, tec);
		};
		std::printf("te_cluster %.2f\n", tec);
		IblOptions opt;
		opt.reynolds = 1e6;
		opt.continuation = false;
		for (int nodes : {100, 120, 140, 160, 180, 200, 240}) {
			const IblResult r = SolveIblAirfoil(contour_of("2412", nodes), 2.0, opt);
			std::printf("2412 a=2 N=%3d: conv=%d it=%2d |R|=%.1e cl %.4f cd %.5f xtr %.3f/%.3f\n", nodes, r.converged, r.iterations, r.residual_norm, r.cl, r.cd, r.xt_upper, r.xt_lower);
		}
		for (double a : {0.0, 0.3, 1.0, 3.0}) {
			const IblResult r = SolveIblAirfoil(contour_of("0012", 160), a, opt);
			std::printf("0012 a=%.1f N=160: conv=%d it=%2d |R|=%.1e cl %.4f cd %.5f xtr %.3f/%.3f\n", a, r.converged, r.iterations, r.residual_norm, r.cl, r.cd, r.xt_upper, r.xt_lower);
		}
		if (argc > 2) {
			opt.verbose = true;
			const int nodes = std::atoi(argv[2]);
			const double a = argc > 3 ? std::atof(argv[3]) : 2.0;
			const IblResult r = SolveIblAirfoil(contour_of(argc > 4 ? argv[4] : "2412", nodes), a, opt);
			PrintResult("verbose case", r);
		}
		return 0;
	}

	// ---- closure derivatives (mfoil's ping states, Re 1e5) ----------------------------
	{
		IblConstants c;
		c.mu0 = 1.0 / 1e5;
		const S4 Ulam{0.01, 0.02, 8.4, 0.9}, Uturb{0.023, 0.05, 0.031, 1.1};
		double worst = 0.0;
		for (int which = 0; which < 3; ++which) {
			IblFlags f;
			f.turb = which >= 1;
			f.wake = which == 2;
			const S4& U = which == 0 ? Ulam : Uturb;
			std::printf("state %s:\n", which == 0 ? "laminar" : (which == 1 ? "turbulent" : "wake"));
			worst = std::fmax(worst, PingState("Hs", [&](const SD& u) { return Hs(u, f, c); }, U));
			worst = std::fmax(worst, PingState("cf", [&](const SD& u) { return Cf(u, f, c); }, U));
			worst = std::fmax(worst, PingState("cDi", [&](const SD& u) { return CDi(u, f, c); }, U));
			worst = std::fmax(worst, PingState("Us", [&](const SD& u) { return Us(u, f, c); }, U));
			worst = std::fmax(worst, PingState("de", [&](const SD& u) { return De(u, f); }, U));
			worst = std::fmax(worst, PingState("cteq", [&](const SD& u) { return Cteq(u, f, c); }, U));
			if (which == 0) worst = std::fmax(worst, PingState("damp", [&](const SD& u) { return Damp(u, f, c); }, U));
			if (which == 1) worst = std::fmax(worst, PingState("cttr", [&](const SD& u) { return Cttr(u, f, c); }, U));
		}
		Check(worst < 1e-5, "closure derivatives by duals match central differences");

		// station residual derivatives w.r.t. the second state and the x's
		for (int which = 0; which < 2; ++which) {
			IblFlags f;
			f.turb = which == 1;
			const S4 U1 = which == 0 ? Ulam : Uturb;
			S4 U2 = U1;
			U2[0] *= 1.1; U2[1] *= 1.12; U2[2] *= 1.05; U2[3] *= 0.98;
			const double x1 = 0.3, x2 = 0.35;
			auto eval = [&](const S4& a, const S4& b, double xa, double xb) {
				return ResidualStation(f, c, D::Var(xa, 8), D::Var(xb, 9), Seed(a, 0), Seed(b, 4), D(0.0), D(0.0));
			};
			const std::array<D, 3> R = eval(U1, U2, x1, x2);
			double w = 0.0;
			for (int slot = 0; slot < 10; ++slot) {
				S4 ap = U1, am = U1, bp = U2, bm = U2;
				double xap = x1, xam = x1, xbp = x2, xbm = x2;
				double h;
				if (slot < 4) { h = 1e-6 * std::fabs(U1[slot]); ap[slot] += h; am[slot] -= h; }
				else if (slot < 8) { h = 1e-6 * std::fabs(U2[slot - 4]); bp[slot - 4] += h; bm[slot - 4] -= h; }
				else if (slot == 8) { h = 1e-6 * x1; xap += h; xam -= h; }
				else { h = 1e-6 * x2; xbp += h; xbm -= h; }
				const std::array<D, 3> Rp = eval(ap, bp, xap, xbp), Rm = eval(am, bm, xam, xbm);
				for (int k = 0; k < 3; ++k) {
					const double fd = (Rp[k].v - Rm[k].v) / (2.0 * h);
					const double err = std::fabs(fd - R[k].d[slot]) / (std::fabs(fd) + std::fabs(R[k].d[slot]) + 1e-6);
					w = std::fmax(w, err);
				}
			}
			std::printf("station residual (%s) derivative worst rel. error %.2e; R = %.3e %.3e %.3e\n", which ? "turbulent" : "laminar", w, R[0].v, R[1].v, R[2].v);
			Check(w < 1e-5, "station residual derivatives match finite differences");
		}
	}

	// ---- coupled solutions ----------------------------------------------------------------
	// (0.2: the boundary-layer method's trailing-edge clustering, see section_aero_panel.cpp)
	auto contour_of = [](const char* code, int nodes) {
		std::unique_ptr<Airfoil> af = Airfoil::Naca4(code, 81);
		const Mat closed = CloseTrailingEdge(af->ContourPoints(600), 0.7);
		return vortexpanel::ResampleContour(closed, nodes, 0.2);
	};
	{
		IblOptions opt;
		opt.reynolds = 1e6;
		const double worst = IblJacobianCheck(contour_of("2412", 60), 2.0, opt, 5, false);
		std::printf("Jacobian check (NACA 2412, 60 nodes): worst rel. error %.2e\n", worst);
		Check(worst >= 0.0 && worst < 1e-3, "assembled global Jacobian matches finite differences (NACA 2412, 60 nodes)");
		const double worst0 = IblJacobianCheck(contour_of("0012", 80), 0.0, opt, 3, false);
		std::printf("Jacobian check (NACA 0012 at 0 deg, 80 nodes, stagnation on a node): worst rel. error %.2e\n", worst0);
		Check(worst0 >= 0.0 && worst0 < 1e-3, "Jacobian with the stagnation point on a node (NACA 0012 at 0 deg, 80 nodes)");
		IblOptions optf = opt;
		optf.xft_upper = 0.10;
		const double worstf = IblJacobianCheck(contour_of("2412", 60), 2.0, optf, 5, false);
		std::printf("Jacobian check (NACA 2412, forced transition at 10%%): worst rel. error %.2e\n", worstf);
		Check(worstf >= 0.0 && worstf < 1e-3, "Jacobian with a forced transition (NACA 2412, 60 nodes)");
	}
	{
		IblOptions opt;
		opt.reynolds = 1e6;
		const IblResult r = SolveIblAirfoil(contour_of("0012", 160), 0.0, opt);
		PrintResult("NACA 0012 Re 1e6 a=0", r);
		Check(r.ok && r.converged, "NACA 0012 at 0 deg converges");
		Check(std::fabs(r.cl) < 2e-3, "symmetric: no lift");
		Check(r.iterations < 30, "NACA 0012 at 0 deg converges in under 30 iterations");
		Check(r.cd > 0.0050 && r.cd < 0.0075, "NACA 0012 cd at Re 1e6 between 0.0050 and 0.0075 (XFOIL ~0.006)");
		Check(r.cdf > 0.0030 && r.cdf < r.cd, "friction drag positive and below the total");
		Check(r.xt_upper > 0.40 && r.xt_upper < 0.85 && std::fabs(r.xt_upper - r.xt_lower) < 0.02, "transition on both surfaces between 40% and 85% chord, symmetric");
		Check(r.cdpi > -2e-3 && r.cdpi < 4e-3, "near-field pressure drag small");
		Check(r.solve_ms < 20000.0, "solve under 20 s");
	}
	{
		IblOptions opt;
		opt.reynolds = 1e6;
		const IblResult r = SolveIblAirfoil(contour_of("2412", 200), 2.0, opt);
		PrintResult("NACA 2412 Re 1e6 a=2 (paper Table 1 at M 0.4: cl 0.4889 cm -0.0501 cd 0.00617 cdf 0.00421 xtr 0.4908/0.9487)", r);
		Check(r.ok && r.converged, "NACA 2412 at 2 deg converges");
		Check(r.cl > 0.42 && r.cl < 0.52, "cl between 0.42 and 0.52 (0.489 at M 0.4)");
		Check(r.cm_c4 > -0.065 && r.cm_c4 < -0.040, "cm c/4 between -0.065 and -0.040 (-0.050 at M 0.4)");
		Check(r.cd > 0.0055 && r.cd < 0.0075, "cd between 0.0055 and 0.0075 (0.00617 at M 0.4)");
		Check(r.cdf > 0.0035 && r.cdf < 0.0050, "cdf between 0.0035 and 0.0050 (0.00421 at M 0.4)");
		Check(r.xt_upper > 0.40 && r.xt_upper < 0.60, "upper transition between 40% and 60% (0.49 at M 0.4)");
		Check(r.xt_lower < 0.0 || r.xt_lower > 0.85, "lower transition aft of 85% or in the wake (0.95 at M 0.4)");
		Check(r.cl < r.cl_inviscid, "viscous lift below the inviscid lift");

		IblOptions forced = opt;
		forced.xft_upper = 0.10;
		const IblResult rf = SolveIblAirfoil(contour_of("2412", 200), 2.0, forced);
		PrintResult("NACA 2412 forced transition at 10% upper", rf);
		Check(rf.ok && rf.converged && rf.xt_upper_forced && std::fabs(rf.xt_upper - 0.10) < 0.03, "forced transition lands at 10% chord");
		Check(rf.cd > r.cd, "forced early transition raises the drag");
	}
	{
		IblOptions opt;
		opt.reynolds = 5e5;
		const IblResult r = SolveIblAirfoil(contour_of("4415", 200), 8.0, opt);
		PrintResult("NACA 4415 Re 5e5 a=8", r);
		Check(r.ok, "NACA 4415 at 8 deg returns a solution");
		if (r.ok) Check(r.cl > 1.0 && r.cl < 1.45, "cl between 1.0 and 1.45");
	}

	// ---- registry entry through the section layer ------------------------------------------
	{
		const SectionAeroMethod* m = FindSectionAeroMethod("Vortex panel + boundary layer (mfoil)");
		Check(m != nullptr && !m->Fast() && m->Viscous(), "boundary-layer method registered as slow and viscous");
		if (m) {
			std::unique_ptr<Airfoil> af = Airfoil::Naca4("2412", 81);
			SectionAeroRequest req = SectionRequestFor(*af);
			req.alpha_deg = 2.0;
			req.reynolds = 1e6;
			req.panel_nodes = 160;
			const SectionAeroResult r = SolveSection(*m, req);
			std::printf("[%s] ok=%d conv=%d cl %.4f cd_total %.5f cd_f %.5f cd_p %.5f cm_c4 %.4f %.0f ms; upper: tr %s %.3f (%s) sep %s %.3f (%s) det %s %.3f; lower: tr %s %.3f sep %s %.3f\n",
				r.method.c_str(), r.ok, r.converged, r.cl, r.cd_total, r.cd_friction, r.cd_pressure, r.cm_c4, r.solve_ms,
				r.upper.transition.found ? "at" : "none", r.upper.transition.x, r.upper.transition.note.c_str(),
				r.upper.separation.found ? "at" : "none", r.upper.separation.x, r.upper.separation.note.c_str(),
				r.upper.detachment.found ? "at" : "none", r.upper.detachment.x,
				r.lower.transition.found ? "at" : "none", r.lower.transition.x, r.lower.separation.found ? "at" : "none", r.lower.separation.x);
			Check(r.ok && r.converged, "section layer result ok and converged");
			if (r.ok) {
				Check(r.upper.transition.found && r.upper.transition.x > 0.4 && r.upper.transition.x < 0.6, "e^N transition cue on the upper surface");
				Check(r.upper.hk.size() == r.upper.x.size() && r.upper.cf.size() == r.upper.x.size(), "boundary-layer distributions filled");
				Check(!r.upper.cp.empty() && r.upper.cp.front() > 0.999 && r.upper.x.front() < 0.01, "surface starts at the stagnation point");
				Check(std::fabs(r.cd_friction + r.cd_pressure - r.cd_total) < 1e-9, "drag components sum to the total");
			} else {
				std::printf("   error: %s\n", r.error.c_str());
			}
		}
	}

	std::printf("RESULT: %d failure(s)\n", g_failures);
	return g_failures == 0 ? 0 : 1;
}
