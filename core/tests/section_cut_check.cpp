// Checks the 2-D flap cut (section_cut.hpp): the nose circle is tangent to
// both skins, the cavity crossings are a cavity radius from the hinge, the
// deflection limits follow from the geometry, and the section loops lie on
// their curves with a parameter mirror that is an exact involution.

#include <cmath>
#include <cstdio>
#include <string>

#include "aerotelic/section_cut.hpp"

using namespace aerotelic;

namespace {

int g_pass = 0, g_fail = 0;

void Check(bool ok, const std::string& what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what.c_str());
	if (ok) ++g_pass; else ++g_fail;
}

std::string F(const std::string& fmt, double a, double b = 0.0, double c = 0.0) {
	char buf[240];
	std::snprintf(buf, sizeof buf, fmt.c_str(), a, b, c);
	return buf;
}

double Dist(const std::vector<double>& p, double cx, double cy) { return std::hypot(p[0] - cx, p[1] - cy); }

void CheckCut(const std::string& label, const Airfoil& af, double x_hinge, double gap) {
	const NurbsCurve& c = af.nurbs_curve();
	FlapCut cut = SolveFlapCut(af, x_hinge, gap);
	std::printf("%s: hinge (%.4f, %.5f) nose %.5f cavity %.5f; limits down %.1f up %.1f deg; lips %.1f / %.1f deg\n", label.c_str(), cut.x_hinge, cut.y_hinge,
		cut.r_nose, cut.r_cavity, cut.deflection_down_limit_deg, cut.deflection_up_limit_deg, cut.lower_lip_angle_deg, cut.upper_lip_angle_deg);

	// Tangency: a radius from the centre, normal to the curve.
	for (auto [t, name] : {std::make_pair(cut.t_upper_tangent, "upper"), std::make_pair(cut.t_lower_tangent, "lower")}) {
		auto p = c.Evaluate(t), d = c.Derivative(t);
		double r = Dist(p, cut.x_hinge, cut.y_hinge);
		double dot = ((p[0] - cut.x_hinge) * d[0] + (p[1] - cut.y_hinge) * d[1]) / std::hypot(d[0], d[1]);
		Check(std::fabs(r - cut.r_nose) < 1e-9, F(label + ": " + name + " tangency point is a nose radius from the hinge (%.3g)", r - cut.r_nose));
		Check(std::fabs(dot) < 1e-9, F(label + ": " + name + " radius is normal to the skin (%.3g)", dot));
	}
	Check(cut.t_upper_tangent > af.LeadingEdgeParameter() && cut.t_lower_tangent < af.LeadingEdgeParameter(), label + ": tangency points on their own surfaces");
	// The circle is the largest that fits: no smaller than the half chord
	// at the hinge would allow by much, and inside the section.
	{
		double x_le = c.Evaluate(af.LeadingEdgeParameter())[0];
		Check(cut.r_nose > 0.0 && cut.r_nose < cut.x_hinge - x_le, label + ": nose radius positive and inside the section");
		double half = 0.5 * af.Thickness({x_hinge})[0];
		Check(cut.r_nose <= half * 1.05 && cut.r_nose >= half * 0.6, F(label + ": nose radius close to the half thickness at the hinge (%.4f vs %.4f)", cut.r_nose, half));
	}
	// Lips: a cavity radius from the hinge, forward of the tangency points.
	for (auto [t, name] : {std::make_pair(cut.t_upper_lip, "upper"), std::make_pair(cut.t_lower_lip, "lower")}) {
		double r = Dist(c.Evaluate(t), cut.x_hinge, cut.y_hinge);
		Check(std::fabs(r - cut.r_cavity) < 1e-9, F(label + ": " + name + " lip is a cavity radius from the hinge (%.3g)", r - cut.r_cavity));
	}
	Check(cut.t_upper_lip < cut.t_upper_tangent && cut.t_lower_lip > cut.t_lower_tangent, label + ": lips forward of the tangency points");
	Check(c.Evaluate(cut.t_upper_lip)[0] < cut.x_hinge && c.Evaluate(cut.t_lower_lip)[0] < cut.x_hinge, label + ": lips forward of the hinge");
	// Aft crossings, when the flap is long enough.
	if (!std::isnan(cut.t_upper_aft)) {
		double r = Dist(c.Evaluate(cut.t_upper_aft), cut.x_hinge, cut.y_hinge);
		Check(std::fabs(r - cut.r_cavity) < 1e-9 && cut.t_upper_aft > cut.t_upper_tangent, label + ": upper aft crossing a cavity radius out, aft of the tangency");
	}
	if (!std::isnan(cut.t_lower_aft)) {
		double r = Dist(c.Evaluate(cut.t_lower_aft), cut.x_hinge, cut.y_hinge);
		Check(std::fabs(r - cut.r_cavity) < 1e-9 && cut.t_lower_aft < cut.t_lower_tangent, label + ": lower aft crossing a cavity radius out, aft of the tangency");
	}
	// Deflection limits: positive, and near the thin-skin estimate
	// 2 sqrt(2 gap / R) radians (the skin is not straight, so loosely).
	{
		double est = 2.0 * std::sqrt(2.0 * gap / cut.r_nose) * 180.0 / 3.14159265358979323846;
		Check(cut.deflection_down_limit_deg > 0.0 && cut.deflection_up_limit_deg > 0.0, label + ": deflection limits positive");
		Check(std::fabs(cut.deflection_down_limit_deg - est) < 0.5 * est && std::fabs(cut.deflection_up_limit_deg - est) < 0.5 * est,
			F(label + ": deflection limits near the thin-skin estimate %.1f deg (%.1f, %.1f)", est, cut.deflection_down_limit_deg, cut.deflection_up_limit_deg));
	}
	Check(cut.lower_lip_angle_deg > 0.0 && cut.lower_lip_angle_deg < 90.0 && cut.upper_lip_angle_deg > 0.0 && cut.upper_lip_angle_deg < 90.0, label + ": lip corners are acute wedges");

	// ---- the loops ----
	SectionLoop flap = SectionLoop::Flap(af, cut), main = SectionLoop::MainElement(af, cut), full = SectionLoop::Full(af);
	Check(!flap.closed() && main.closed() && !full.closed(), label + ": flap and full loops close through the trailing edge, the main element on itself");
	// Flap: skin pieces on the airfoil, the arc a nose radius out, the
	// ends at the trailing edge.
	{
		double worst_skin = 0.0, worst_arc = 0.0;
		for (int i = 0; i <= 400; ++i) {
			double v = i / 400.0;
			auto p = flap.Point(v);
			double t = flap.AirfoilParameter(v);
			if (!std::isnan(t)) {
				auto q = c.Evaluate(t);
				worst_skin = std::max(worst_skin, std::hypot(p[0] - q[0], p[1] - q[1]));
				Check(std::fabs(flap.ParameterAtAirfoil(t) - v) < 1e-12 || std::fabs(v - 0.35) < 1e-12 || std::fabs(v - 0.65) < 1e-12, F(label + ": flap v <-> t round trip at v = %.4f", v));
			} else {
				worst_arc = std::max(worst_arc, std::fabs(Dist(p, cut.x_hinge, cut.y_hinge) - cut.r_nose));
			}
		}
		Check(worst_skin < 1e-12, F(label + ": flap skin pieces lie on the airfoil (%.3g)", worst_skin));
		Check(worst_arc < 1e-12, F(label + ": flap nose arc a nose radius from the hinge (%.3g)", worst_arc));
		auto p0 = flap.Point(0.0), p1 = flap.Point(1.0), a0 = c.Evaluate(0.0), a1 = c.Evaluate(1.0);
		Check(std::hypot(p0[0] - a0[0], p0[1] - a0[1]) < 1e-12 && std::hypot(p1[0] - a1[0], p1[1] - a1[1]) < 1e-12, label + ": flap loop ends at the trailing edge");
		// The arc joins the skin at the tangency points.
		auto t_l = flap.Point(0.35), t_u = flap.Point(0.65), tl = c.Evaluate(cut.t_lower_tangent), tu = c.Evaluate(cut.t_upper_tangent);
		Check(std::hypot(t_l[0] - tl[0], t_l[1] - tl[1]) < 1e-12 && std::hypot(t_u[0] - tu[0], t_u[1] - tu[1]) < 1e-12, label + ": flap arc meets the skin at the tangency points");
		// Forward extreme of the arc at v = 0.5.
		auto fe = flap.Point(0.5);
		Check(std::fabs(fe[0] - (cut.x_hinge - cut.r_nose)) < 1e-12 && std::fabs(fe[1] - cut.y_hinge) < 1e-12, label + ": flap arc's forward extreme at v = 0.5");
	}
	// Main element: skin on the airfoil between the lips, the arc a cavity
	// radius out, closed.
	{
		double worst_skin = 0.0, worst_arc = 0.0;
		for (int i = 0; i < 400; ++i) {
			double v = i / 400.0;
			auto p = main.Point(v);
			double t = main.AirfoilParameter(v);
			if (!std::isnan(t)) {
				auto q = c.Evaluate(t);
				worst_skin = std::max(worst_skin, std::hypot(p[0] - q[0], p[1] - q[1]));
				Check(t >= cut.t_lower_lip - 1e-12 && t <= cut.t_upper_lip + 1e-12, F(label + ": main skin stays between the lips at v = %.4f", v));
			} else {
				worst_arc = std::max(worst_arc, std::fabs(Dist(p, cut.x_hinge, cut.y_hinge) - cut.r_cavity));
			}
		}
		Check(worst_skin < 1e-12, F(label + ": main skin pieces lie on the airfoil (%.3g)", worst_skin));
		Check(worst_arc < 1e-12, F(label + ": main cavity arc a cavity radius from the hinge (%.3g)", worst_arc));
		auto p0 = main.Point(0.0), p1 = main.Point(1.0);
		Check(std::hypot(p0[0] - p1[0], p0[1] - p1[1]) < 1e-12, label + ": main loop closes on itself");
		auto le = main.Point(0.25), le_ref = c.Evaluate(af.LeadingEdgeParameter());
		Check(std::hypot(le[0] - le_ref[0], le[1] - le_ref[1]) < 1e-12, label + ": main loop's v = 0.25 is the leading edge");
	}
	// Mirrors: same x, an involution, and on the full loop the airfoil's own.
	for (auto [loop, name] : {std::make_pair(&flap, "flap"), std::make_pair(&main, "main"), std::make_pair(&full, "full")}) {
		double worst_x = 0.0, worst_inv = 0.0, worst_af = 0.0;
		int self_twins = 0;
		for (int i = 1; i < 300; ++i) {
			double v = i / 300.0;
			double m = loop->Mirror(v);
			if (std::fabs(m - v) < 1e-12) { ++self_twins; continue; }
			auto p = loop->Point(v), q = loop->Point(m);
			worst_x = std::max(worst_x, std::fabs(p[0] - q[0]));
			double mm = loop->Mirror(m);
			double d = std::fabs(mm - v);
			if (loop->closed()) d = std::min(d, std::fabs(d - 1.0));
			if (d > 1e-6 || std::fabs(p[0] - q[0]) > 1e-9) {
				auto r = loop->Point(mm);
				std::printf("    %s mirror trouble: v %.6f (%.5f, %.5f) -> m %.6f (%.5f, %.5f) -> mm %.6f (%.5f, %.5f)\n", name, v, p[0], p[1], m, q[0], q[1], mm, r[0], r[1]);
			}
			worst_inv = std::max(worst_inv, d);
			if (loop == &full) worst_af = std::max(worst_af, std::fabs(m - af.MirrorParameter(v)));
		}
		Check(worst_x < 1e-9, F(label + ": " + name + " mirror keeps x (%.3g)", worst_x));
		// Near an x extreme a roundoff in x maps to ~sqrt(roundoff) in v, so
		// the involution holds to ~1e-8 there, not to the root's own 1e-13.
		Check(worst_inv < 1e-6, F(label + ": " + name + " mirror is an involution (%.3g)", worst_inv));
		// Own twins: the leading edge (v = 0.25 on the main element), the
		// arcs' forward extreme (0.5 on the flap, 0.75 on the main), and on
		// the main element whichever lip corner is the loop's aft extreme.
		Check(self_twins <= 3, label + ": " + name + " mirror: at most the extremes are their own twins (" + std::to_string(self_twins) + ")");
		if (loop == &full) Check(worst_af < 1e-9, F(label + ": full mirror agrees with Airfoil::MirrorParameter (%.3g)", worst_af));
	}
	// On the main element, a point of the upper skin in the lip mirrors onto
	// the cavity arc, not the lower skin.
	{
		double t_mid = 0.5 * (cut.t_upper_lip + af.LeadingEdgeParameter());
		double x_mid = c.Evaluate(t_mid)[0];
		if (x_mid > cut.x_hinge - cut.r_cavity) {
			double v = main.ParameterAtAirfoil(t_mid), m = main.Mirror(v);
			Check(std::isnan(main.AirfoilParameter(m)) && m > 0.5, F(label + ": lip skin point mirrors onto the cavity arc (v %.4f -> %.4f)", v, m));
		}
	}
}

} // namespace

int Run();

int main() {
	setvbuf(stdout, nullptr, _IONBF, 0);
	try {
		return Run();
	} catch (const std::exception& e) {
		std::printf("FAIL: exception: %s\nRESULT: %d passed, %d failed\n", e.what(), g_pass, g_fail + 1);
		return 1;
	}
}

int Run() {
	auto n2412 = Airfoil::Naca4("2412");
	auto n0012 = Airfoil::Naca4("0012");
	auto n4415 = Airfoil::Naca4("4415");

	CheckCut("2412 x=0.70 g=0.002", *n2412, 0.70, 0.002);
	CheckCut("0012 x=0.75 g=0.003", *n0012, 0.75, 0.003);
	CheckCut("4415 x=0.65 g=0.004", *n4415, 0.65, 0.004);
	CheckCut("2412 x=0.90 g=0.001", *n2412, 0.90, 0.001);

	// A symmetric section: the hinge sits on the chord line and the two
	// tangency points share an x.
	{
		FlapCut cut = SolveFlapCut(*n0012, 0.7, 0.002);
		Check(std::fabs(cut.y_hinge) < 1e-9, F("0012: hinge on the chord line (y %.3g)", cut.y_hinge));
		auto pu = n0012->nurbs_curve().Evaluate(cut.t_upper_tangent), pl = n0012->nurbs_curve().Evaluate(cut.t_lower_tangent);
		Check(std::fabs(pu[0] - pl[0]) < 1e-7 && std::fabs(pu[1] + pl[1]) < 1e-7, "0012: tangency points mirror each other");
		Check(std::fabs(cut.deflection_down_limit_deg - cut.deflection_up_limit_deg) < 1e-6, "0012: equal deflection limits up and down");
	}
	// Zero gap: the cavity is the nose; the lips coincide with the tangency
	// points (to the crossing solver's resolution).
	{
		FlapCut cut = SolveFlapCut(*n2412, 0.7, 0.0);
		Check(std::fabs(cut.r_cavity - cut.r_nose) == 0.0, "zero gap: cavity is the nose");
		Check(std::fabs(cut.t_upper_lip - cut.t_upper_tangent) < 1e-5 && std::fabs(cut.t_lower_lip - cut.t_lower_tangent) < 1e-5, "zero gap: lips at the tangency points");
	}

	// ---- refusals ----
	auto refuses = [](const char* what, auto fn) {
		bool threw = false;
		std::string msg;
		try { fn(); } catch (const std::invalid_argument& e) { threw = true; msg = e.what(); }
		Check(threw, std::string("refuses ") + what + (threw ? " (" + msg + ")" : ""));
	};
	refuses("a hinge off the section", [&] { SolveFlapCut(*n2412, 1.2, 0.002); });
	refuses("a hinge at the leading edge", [&] { SolveFlapCut(*n2412, 0.0, 0.002); });
	// (A cavity radius beyond the hinge's distance to the leading edge: the
	// circle encloses the whole forward section and crosses no skin.)
	refuses("a gap wider than the section can hold", [&] { SolveFlapCut(*n2412, 0.3, 0.5); });
	refuses("a negative gap", [&] { SolveFlapCut(*n2412, 0.7, -0.01); });

	std::printf("\nRESULT: %d passed, %d failed\n", g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}
