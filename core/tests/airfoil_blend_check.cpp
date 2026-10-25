// Checks Airfoil::Blend / MakeCommonKnots: the blend of two airfoils in
// B-spline form is exactly the pointwise blend of the two curves at equal
// parameter, shares one knot vector at every weight, reproduces the ends,
// finds the true leading edge, and refuses incompatible curves.

#include <cmath>
#include <cstdio>
#include <string>

#include "aerotelic/airfoil.hpp"
#include "aerotelic/nurbs.hpp"

using namespace aerotelic;

namespace {

int g_pass = 0, g_fail = 0;

void Check(bool ok, const std::string& what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what.c_str());
	if (ok) ++g_pass; else ++g_fail;
}

double MaxPointError(const Airfoil& blend, const Airfoil& a, const Airfoil& b, double w, int n = 500) {
	double worst = 0.0;
	for (int i = 0; i <= n; ++i) {
		double t = static_cast<double>(i) / n;
		auto p = blend.nurbs_curve().Evaluate(t), pa = a.nurbs_curve().Evaluate(t), pb = b.nurbs_curve().Evaluate(t);
		for (size_t c = 0; c < p.size(); ++c) worst = std::max(worst, std::abs(p[c] - ((1.0 - w) * pa[c] + w * pb[c])));
	}
	return worst;
}

} // namespace

int main() {
	auto a = Airfoil::Naca4("2412");
	auto b = Airfoil::Naca4("0010");

	// ---- MakeCommonKnots on its own ----
	{
		ClampedNurbsCurve ca(a->nurbs_curve().Serialize()), cb(b->nurbs_curve().Serialize());
		int na = ca.n_poles(), nb = cb.n_poles();
		Check(MakeCommonKnots(ca, cb), "common knots: succeeds for two cubic fits");
		Check(ca.interior_knots() == cb.interior_knots(), "common knots: identical knot vectors afterwards");
		Check(ca.n_poles() == cb.n_poles() && ca.n_poles() >= std::max(na, nb), "common knots: same pole count, no fewer than before (" + std::to_string(ca.n_poles()) + ")");
		double worst = 0.0;
		for (int i = 0; i <= 400; ++i) {
			double t = i / 400.0;
			worst = std::max(worst, VecNorm(VecSub(ca.Evaluate(t), a->nurbs_curve().Evaluate(t))));
			worst = std::max(worst, VecNorm(VecSub(cb.Evaluate(t), b->nurbs_curve().Evaluate(t))));
		}
		char buf[160];
		std::snprintf(buf, sizeof buf, "common knots: shape preserved (worst %.3g)", worst);
		Check(worst < 1e-9, buf);
		// A second call has nothing to do and changes nothing.
		std::vector<double> before = ca.interior_knots();
		Check(MakeCommonKnots(ca, cb) && ca.interior_knots() == before, "common knots: idempotent");
	}

	// ---- the blend ----
	for (double w : {0.0, 0.25, 0.5, 1.0}) {
		auto m = Airfoil::Blend(*a, *b, w);
		Check(m != nullptr, "blend w=" + std::to_string(w) + ": built");
		if (!m) continue;
		double err = MaxPointError(*m, *a, *b, w);
		char buf[160];
		std::snprintf(buf, sizeof buf, "blend w=%.2f: pointwise blend at equal parameter (worst %.3g)", w, err);
		Check(err < 1e-10, buf);
		// The leading edge is the minimum of x on the blended curve.
		double t_le = m->LeadingEdgeParameter();
		double x_le = m->nurbs_curve().Evaluate(t_le)[0];
		bool is_min = true;
		for (int i = 0; i <= 2000; ++i) is_min = is_min && m->nurbs_curve().Evaluate(i / 2000.0)[0] >= x_le - 1e-12;
		Check(is_min, "blend w=" + std::to_string(w) + ": leading-edge parameter is the x minimum");
		Check(std::abs(m->nurbs_curve().Derivative(t_le)[0]) < 1e-6, "blend w=" + std::to_string(w) + ": dx/dt vanishes at the leading edge");
		Check(m->te_line() == nullptr, "blend w=" + std::to_string(w) + ": closed trailing edge stays closed");
	}
	{
		auto m1 = Airfoil::Blend(*a, *b, 0.3), m2 = Airfoil::Blend(*a, *b, 0.7);
		auto [k1, c1] = m1->nurbs_curve().GetUniqueKnots();
		auto [k2, c2] = m2->nurbs_curve().GetUniqueKnots();
		Check(k1 == k2 && c1 == c2, "blend: the same knot vector at every weight");
		Check(m1->name == "NACA 2412 -> NACA 0010", "blend: default name (" + m1->name + ")");
		// The thickness of the blend is the blend of the thicknesses, to the
		// accuracy of the sampled thickness query.
		auto ta = a->Thickness({0.3}), tb = b->Thickness({0.3}), tm = m1->Thickness({0.3});
		Check(std::abs(tm[0] - (0.7 * ta[0] + 0.3 * tb[0])) < 2e-3, "blend: thickness blends (" + std::to_string(tm[0]) + ")");
	}

	// ---- incompatible curves ----
	{
		NurbsSpec quad = a->nurbs_curve().Serialize();
		quad.degree = 2;
		quad.interior_knots.reset(); // let the curve lay out knots for the new degree
		auto q = Airfoil::FromCurve(quad, "quadratic");
		Check(Airfoil::Blend(*a, *q, 0.5) == nullptr, "blend: different degrees refused");
	}

	std::printf("\nRESULT: %d passed, %d failed\n", g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}
