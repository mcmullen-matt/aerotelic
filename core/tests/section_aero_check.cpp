// Checks the section-aerodynamics layer (section_aero.hpp) end to end on
// NACA four-digit sections through the method registry: lift slope and
// zero-lift angle against the thin-airfoil / Abbott & von Doenhoff values,
// the trailing-edge closure, the transition and Stratford cues, the
// skin-friction drag and the pole sensitivities.

#include <cmath>
#include <cstdio>
#include <vector>

#include "aerotelic/airfoil.hpp"
#include "aerotelic/section_aero.hpp"

using namespace aerotelic;

namespace {

int g_failures = 0;

void Check(bool ok, const char* what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what);
	if (!ok) ++g_failures;
}

void Print(const SectionAeroResult& r) {
	std::printf("[%s] alpha %.1f Re %.2e: ok=%d cl %.4f (exact %.4f) cd_p %.5f cd_f %.5f cm_le %.4f cm_c4 %.4f alpha0 %.3f cl_a %.3f cp_min %.3f | it %d res %.1e margin %.3f te gap %.2e tau %.1f | %.2f ms %s\n",
		r.method.c_str(), r.alpha_deg, r.reynolds, r.ok, r.cl, r.cl_exact, r.cd_pressure, r.cd_friction, r.cm_le, r.cm_c4,
		r.alpha0_deg, r.cl_alpha, r.cp_min, r.iterations, r.residual, r.convergence_margin, r.te_gap_closed, r.te_angle_deg, r.solve_ms, r.error.c_str());
	for (const std::string& w : r.warnings) std::printf("   warning: %s\n", w.c_str());
	auto cue = [](const char* name, const SectionSurface& s) {
		std::printf("   %s: %zu pts, peak q %.3f at x/c %.3f; transition %s x/c %.3f (%s); separation %s x/c %.3f (%s); 4/7 at %.3f; cd_f %.5f x_EL %.4f\n",
			name, s.x.size(), s.q.empty() ? 0.0 : s.q[s.peak_index], s.x.empty() ? 0.0 : s.x[s.peak_index],
			s.transition.found ? "at" : "none", s.transition.x, s.transition.note.c_str(),
			s.separation.found ? "at" : "none", s.separation.x, s.separation.note.c_str(), s.stratford_range_end_x, s.cd_friction, s.cf_integral_x_equiv);
	};
	cue("upper", r.upper);
	cue("lower", r.lower);
}

} // namespace

int main() {
	setvbuf(stdout, nullptr, _IONBF, 0);
	const auto& methods = SectionAeroMethods();
	Check(!methods.empty() && FindSectionAeroMethod(methods[0]->Name()) == methods[0], "registry has the conformal map first");
	const SectionAeroMethod& cm = *methods[0];
	std::printf("method: %s -- %s\n", cm.Name().c_str(), cm.Description().c_str());

	// ---- NACA 0012 ------------------------------------------------------------------
	{
		std::unique_ptr<Airfoil> af = Airfoil::Naca4("0012", 81);
		SectionAeroRequest req = SectionRequestFor(*af);
		req.reynolds = 1e6;
		SectionAeroResult r0 = SolveSection(cm, req);
		Print(r0);
		Check(r0.ok, "NACA 0012 solves at alpha 0");
		Check(std::fabs(r0.cl) < 2e-3 && std::fabs(r0.cl_exact) < 1e-6, "symmetric: no lift at alpha 0");
		Check(std::fabs(r0.cm_c4) < 2e-3, "symmetric: no moment at alpha 0");
		Check(std::fabs(r0.cp_min + 0.43) < 0.03, "NACA 0012 Cp_min about -0.43 at alpha 0 (v/V max 1.196, Abbott & von Doenhoff)");
		Check(r0.cl_alpha > 6.6 && r0.cl_alpha < 7.2, "NACA 0012 lift slope between 6.6 and 7.2 per radian (thin-airfoil 6.28 plus thickness)");
		Check(std::fabs(r0.cd_pressure) < 2e-3, "pressure drag is a discretisation residual (d'Alembert)");
		Check(r0.cd_friction > 0.008 && r0.cd_friction < 0.014, "turbulent flat-plate friction drag about 0.0094 at Re 1e6");
		Check(r0.upper.transition.found && r0.upper.transition.x > 0.05 && r0.upper.transition.x < 0.3, "transition cue at the suction peak (first adverse gradient)");
		Check(r0.upper.x.front() < 1e-3 && std::fabs(r0.upper.x.back() - 1.0) < 1e-6 && r0.upper.cp.front() > 0.999, "upper surface runs from the stagnation point (Cp 1) to the trailing edge");
		Check(r0.upper.cp.back() > 0.999, "finite trailing-edge angle: inviscid stagnation at the trailing edge");
		Check(r0.te_gap_closed < 1e-9, "closed NACA trailing edge needs no closure");

		req.alpha_deg = 8.0;
		SectionAeroResult r8 = SolveSection(cm, req);
		Print(r8);
		Check(r8.ok && r8.cl > 0.85 && r8.cl < 1.0, "NACA 0012 at 8 deg: Cl between 0.85 and 1.0");
		Check(std::fabs(r8.cl - r8.cl_exact) < 5e-3, "integrated lift agrees with Kutta-Joukowski");
		Check(r8.upper.transition.found && r8.upper.transition.x < 0.05, "at 8 deg the upper suction peak (transition cue) is near the leading edge");
		Check(r8.lower.cd_friction > 0.0 && r8.upper.cd_friction > 0.0, "both surfaces contribute friction");

		req.alpha_deg = 0.0;
		req.reynolds = 5e7;
		SectionAeroResult rh = SolveSection(cm, req);
		Print(rh);
		Check(rh.upper.transition.found && rh.upper.transition.note.find("Re_x") != std::string::npos, "at Re 5e7 the Re_x rule fires before the peak");
	}

	// ---- NACA 2412 --------------------------------------------------------------------
	{
		std::unique_ptr<Airfoil> af = Airfoil::Naca4("2412", 81);
		SectionAeroRequest req = SectionRequestFor(*af);
		req.alpha_deg = 2.0;
		SectionAeroResult r = SolveSection(cm, req);
		Print(r);
		Check(r.ok, "NACA 2412 solves");
		Check(r.alpha0_deg > -2.7 && r.alpha0_deg < -1.6, "NACA 2412 zero-lift angle about -2 deg");
		Check(r.cm_c4 > -0.08 && r.cm_c4 < -0.03, "NACA 2412 Cm c/4 about -0.05");
		Check(r.cl > 0.4 && r.cl < 0.55, "NACA 2412 at 2 deg: Cl about 0.47");
		Check(SectionAeroScalarValue(r, SectionAeroScalar::CpUpperAtX, 0.3) < SectionAeroScalarValue(r, SectionAeroScalar::CpLowerAtX, 0.3), "upper Cp below lower Cp at 30% chord");

		// Sensitivity: raising an upper-surface pole raises the lift.
		std::unique_ptr<Airfoil> fit = Airfoil::Refit(*af, 3, 16);
		const NurbsSpec spec = fit->Serialize();
		const PoleSensitivity sens = ComputePoleSensitivity(spec, cm, req, SectionAeroScalar::Cl, 0.3);
		if (!sens.error.empty()) std::printf("sensitivity note: %s\n", sens.error.c_str());
		Check(sens.ok, "sensitivity computed");
		if (sens.ok) {
			std::printf("dCl/dy per pole:");
			for (double v : sens.d_dy) std::printf(" %.2f", v);
			std::printf("\n");
			const int n = static_cast<int>(sens.d_dy.size());
			Check(sens.d_dy[(3 * n) / 4] > 0.0 && sens.d_dy[n / 4] > 0.0, "raising a mid-chord pole on either surface raises Cl");
			Check(std::fabs(sens.base - r.cl) < 0.05, "sensitivity base matches the refit's lift (refit error aside)");
		}
	}

	// ---- open trailing edge: closure ---------------------------------------------------
	{
		// A NACA 0012 with the classic open TE (0.1015 coefficient) has a gap of 0.0025.
		const int n = 61;
		Mat pts(2 * n - 1, 2);
		for (int i = 0; i < n; ++i) {
			const double u = 3.14159265358979323846 * i / (n - 1);
			const double x = 0.5 * (1.0 - std::cos(u));
			const double yt = 0.6 * (0.2969 * std::sqrt(x) - 0.1260 * x - 0.3516 * x * x + 0.2843 * x * x * x - 0.1015 * x * x * x * x);
			pts(n - 1 - i, 0) = x; pts(n - 1 - i, 1) = -yt;
			if (i > 0) { pts(n - 1 + i, 0) = x; pts(n - 1 + i, 1) = yt; }
		}
		double gap = 0.0;
		const Mat closed = CloseTrailingEdge(pts, 0.7, &gap);
		std::printf("open TE gap %.5f; closed first (%.4f, %.5f) last (%.4f, %.5f); y at x=0.85: %.5f -> %.5f\n", gap,
			closed(0, 0), closed(0, 1), closed(closed.rows() - 1, 0), closed(closed.rows() - 1, 1), pts(2 * n - 2 - 8, 1), closed(2 * n - 2 - 8, 1));
		Check(std::fabs(gap - 0.00252) < 2e-4, "gap measured");
		Check(closed(0, 0) == closed(closed.rows() - 1, 0) && closed(0, 1) == closed(closed.rows() - 1, 1), "closed contour's ends coincide");
		Check(closed(n - 1, 1) == pts(n - 1, 1) && closed(30, 1) == pts(30, 1), "points ahead of the blend start are untouched");

		std::unique_ptr<Airfoil> af = Airfoil::FromPoints(pts, 3, "0012 open");
		Check(af->te_line() != nullptr, "open-TE airfoil carries a trailing-edge line");
		SectionAeroRequest req = SectionRequestFor(*af);
		req.alpha_deg = 4.0;
		SectionAeroResult r = SolveSection(cm, req);
		Print(r);
		Check(r.ok && r.te_gap_closed > 0.002 && r.te_gap_closed < 0.003, "open trailing edge closed before analysis and reported");
		Check(r.cl > 0.4 && r.cl < 0.55, "closed open-TE 0012 at 4 deg lifts about as the closed one");
	}

	std::printf("RESULT: %d failure(s)\n", g_failures);
	return g_failures == 0 ? 0 : 1;
}
