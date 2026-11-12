// Checks the least-squares ApproximateCurve, NurbsCurve::ClosestParameter,
// Airfoil::Refit and CompareAirfoilFit.

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>

#include "aerotelic/airfoil.hpp"
#include "aerotelic/nurbs.hpp"

using namespace aerotelic;

namespace {

int g_failures = 0;

void Check(bool ok, const char* what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what);
	if (!ok) ++g_failures;
}

} // namespace

int main() {
	setvbuf(stdout, nullptr, _IONBF, 0);
	try {
		// ---- the solve is a least-squares minimum: nudging any interior pole
		// raises the sum of squared residuals at the assigned parameters ----
		{
			NurbsSpec spec;
			spec.degree = 3;
			spec.poles = Mat::FromRows({{0, 0}, {0.1, 0.3}, {0.4, 0.5}, {0.7, 0.2}, {1.0, 0.4}, {1.3, 0.0}, {1.6, -0.3}, {2.0, 0.0}});
			spec.interior_knots = std::vector<double>{0.2, 0.4, 0.6, 0.8};
			ClampedNurbsCurve ref(spec);
			Mat pts = ref.Sample(120);
			unsigned seed = 11u;
			for (int i = 1; i < pts.rows() - 1; ++i) { seed = seed * 1664525u + 1013904223u; pts(i, 1) += 0.02 * ((seed >> 8) / 16777216.0 - 0.5); }
			const std::vector<double> knots{0.3, 0.6};
			ApproximateCurve fit(pts, 3, 6, ApproximationKnots::Given, &knots, 0);
			const std::vector<double> t = fit.data_parameters();
			auto ssr = [&](const NurbsCurve& c) {
				double s = 0.0;
				for (int k = 0; k < pts.rows(); ++k) { const double d = VecNorm(VecSub(c.Evaluate(t[k]), pts.row(k))); s += d * d; }
				return s;
			};
			const double s0 = ssr(fit);
			bool minimum = true;
			for (int i = 1; i < 5; ++i) for (int c = 0; c < 2; ++c) for (double sign : {-1.0, 1.0}) {
				NurbsSpec p = fit.Serialize();
				p.poles(i, c) += sign * 1e-3;
				ClampedNurbsCurve nudged(p);
				if (ssr(nudged) < s0 - 1e-12) minimum = false;
			}
			std::printf("least-squares SSR %.4e; nudging interior poles raises it: %d\n", s0, minimum);
			Check(minimum, "ApproximateCurve is a least-squares minimum (normal equations solved)");
			ApproximateCurve fit3(pts, 3, 6, ApproximationKnots::Given, &knots, 3);
			double worst0 = 0.0, worst3 = 0.0;
			for (int k = 0; k < pts.rows(); ++k) {
				worst0 = std::fmax(worst0, fit.ClosestParameter(pts.row(k), 400).second);
				worst3 = std::fmax(worst3, fit3.ClosestParameter(pts.row(k), 400).second);
			}
			std::printf("max normal distance: without parameter correction %.3e, with 3 rounds %.3e\n", worst0, worst3);
			Check(worst3 <= worst0 * 1.05, "parameter correction does not worsen the normal distance");
		}

		// ---- ClosestParameter on a straight segment --------------------------------
		{
			NurbsSpec line;
			line.degree = 1;
			line.poles = Mat::FromRows({{0, 0}, {2, 0}});
			ClampedNurbsCurve c(line);
			const auto [t, d] = c.ClosestParameter({0.5, 0.3});
			std::printf("closest point on segment: t %.12f d %.12f\n", t, d);
			Check(std::fabs(t - 0.25) < 1e-7 && std::fabs(d - 0.3) < 1e-9, "closest point on a segment");
		}

		// ---- refitting a NACA 2412 with fewer poles -----------------------------------
		{
			std::unique_ptr<Airfoil> src = Airfoil::Naca4("2412", 81);
			std::printf("source: %d points, %d poles\n", src->fit_points().rows(), src->nurbs_curve().n_poles());
			for (int n : {10, 14, 20, 30}) {
				std::unique_ptr<Airfoil> fit = Airfoil::Refit(*src, 3, n);
				const AirfoilFitReport rep = CompareAirfoilFit(*src, *fit);
				// The least-squares residual at the assigned parameters, for comparison.
				const auto& ac = dynamic_cast<const ApproximateCurve&>(fit->nurbs_curve());
				const std::vector<double> t = ac.data_parameters();
				double res = 0.0;
				for (int k = 0; k < src->fit_points().rows(); ++k) res = std::fmax(res, VecNorm(VecSub(ac.Evaluate(t[k]), src->fit_points().row(k))));
				std::printf("refit %2d poles: max %.2e at x/c %.3f (%s) rms %.2e [LS residual %.2e] dt %.2e dc %.2e te %.1e->%.1e\n",
					n, rep.max_distance, rep.max_at_x, rep.max_on_upper ? "upper" : "lower", rep.rms_distance, res,
					rep.delta_max_thickness, rep.delta_max_camber, rep.te_gap_source, rep.te_gap_fitted);
				Check(rep.n_poles == n && rep.degree == 3, "report carries the representation");
				// Measured 2026-11-11 (parameter correction on, averaging knots):
				// 1.1e-2 / 1.9e-3 / 5.8e-4, the worst point always the leading
				// edge, where the knot density (by data count) is lowest relative
				// to the curvature. Curvature-adapted knots would improve this.
				if (n == 10) Check(rep.max_distance < 2e-2, "10 poles within 2e-2 of the source");
				if (n == 20) Check(rep.max_distance < 3e-3, "20 poles within 3e-3 of the source");
				if (n == 30) Check(rep.max_distance < 1e-3, "30 poles within 1e-3 of the source");
				Check(std::fabs(res - rep.max_distance) < 0.2 * rep.max_distance + 1e-5, "after parameter correction the residual at the data parameters is the normal distance");
				Check(std::fabs(fit->MaxThickness().second - 0.12) < 0.01, "refit keeps the thickness");
				Check(fit->te_line() == nullptr, "closed trailing edge stays closed (end points interpolated)");
				Check(fit->LeadingEdgeParameter() > 0.3 && fit->LeadingEdgeParameter() < 0.7, "leading-edge parameter located");
			}
			std::unique_ptr<Airfoil> uni = Airfoil::Refit(*src, 3, 16, ApproximationKnots::Uniform);
			const AirfoilFitReport ru = CompareAirfoilFit(*src, *uni);
			std::printf("uniform knots, 16 poles: max %.2e at x/c %.3f\n", ru.max_distance, ru.max_at_x);
			Check(ru.max_distance < 3e-2, "uniform knots also fit (worse at the leading edge, as expected)");
			std::unique_ptr<Airfoil> p2 = Airfoil::Refit(*src, 2, 16);
			const AirfoilFitReport r2 = CompareAirfoilFit(*src, *p2);
			std::printf("degree 2, 16 poles: max %.2e at x/c %.3f\n", r2.max_distance, r2.max_at_x);
			Check(p2->nurbs_curve().degree() == 2 && r2.max_distance < 1e-2, "degree 2 refit");

			bool threw = false;
			try { Airfoil::Refit(*src, 3, 500); } catch (const std::invalid_argument&) { threw = true; }
			Check(threw, "more poles than points is refused");

			// Editing a pole in place and re-deriving the leading edge.
			std::unique_ptr<Airfoil> ed = Airfoil::Refit(*src, 3, 16);
			NurbsCurve& c = ed->mutable_curve();
			std::vector<double> p = c.poles().row(11);
			p[1] += 0.02;
			c.UpdatePole(11, p);
			ed->CurveEdited();
			const AirfoilFitReport rep = CompareAirfoilFit(*src, *ed);
			Check(rep.max_distance > 1e-3 && rep.max_on_upper, "a moved upper pole shows up in the report on the upper surface");
		}
	} catch (const std::exception& e) {
		std::printf("FAIL: exception: %s\n", e.what());
		++g_failures;
	}
	std::printf("RESULT: %d failure(s)\n", g_failures);
	return g_failures == 0 ? 0 : 1;
}
