// The panel method on a WingModel (wing_panels.hpp): the default tapered,
// swept planform with NACA sections, thick and as a mean surface, at a
// small angle of attack. Lift slopes are checked against lifting-line
// theory and the two formulations against each other.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

#include "aerotelic/fuselage_model.hpp"
#include "aerotelic/lofted_nacelle_model.hpp"
#include "aerotelic/primitive_model.hpp"
#include "aerotelic/wing_panels.hpp"

using namespace aerotelic;
using namespace aerotelic::panel;

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

} // namespace

int main() {
	setvbuf(stdout, nullptr, _IONBF, 0);
	try {
		PlanformSpec ps; // semi-span 10, root 3.1, tip 0.5, sweep 20 degrees
		WingModel wing(ps, "Wing");
		wing.AddSection(Airfoil::Naca4("0012"), 0.0);
		wing.AddSection(Airfoil::Naca4("0012"), 1.0);
		const double area = 2.0 * wing.planform().PlanformArea() / 2.0; // PlanformArea() is the full wing's
		const double aspect = wing.planform().AspectRatio();
		const double alpha = 4.0 * 3.14159265358979323846 / 180.0;
		const Vec3 vinf{std::cos(alpha), 0.0, std::sin(alpha)};
		// Helmbold / lifting-line: C_Lα ≈ 2π AR / (AR + 2) for a swept, tapered wing, roughly.
		const double cl_ll = 2.0 * 3.14159265358979323846 * aspect / (aspect + 2.0) * alpha * std::cos(20.0 * 3.14159265358979323846 / 180.0);

		Options opt;
		opt.symmetry_y = 0.0;
		opt.reference_area = wing.planform().PlanformArea();
		(void)area;

		WingPaneling thick;
		thick.chordwise = 10; thick.spanwise = 10;
		auto t0 = std::chrono::steady_clock::now();
		std::vector<Network> nets = WingNetworks(wing, thick, vinf);
		Solution s_thick = Solve(nets, vinf, opt);
		double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
		double cl_thick = s_thick.force_coefficient.z * std::cos(alpha) - s_thick.force_coefficient.x * std::sin(alpha);
		std::printf("thick wing: %d unknowns in %.2f s (influence %.2f, solve %.3f), max delta %.3f; C_L wake %.4f, C_L forces %.4f, lifting line %.4f\n",
			s_thick.unknowns, secs, s_thick.seconds_influence, s_thick.seconds_solve, s_thick.max_delta, s_thick.lift_from_wake.value_or(0.0), cl_thick, cl_ll);
		Check(nets.size() == 2 && nets[0].rows == 21 && nets[0].cols == 11 && nets[1].rows == 2, "thick wing: one body network of 20 x 10 panels and its wake");
		Check(s_thick.max_delta < 0.1, F("thick wing: panel curvature measure %.3f (report wants < 0.066)", s_thick.max_delta));
		Check(std::fabs(s_thick.lift_from_wake.value_or(0.0) / cl_ll - 1.0) < 0.2, F("thick wing: lift from the wake %.4f within 20%% of lifting line %.4f", s_thick.lift_from_wake.value_or(0.0), cl_ll));
		Check(std::fabs(cl_thick / s_thick.lift_from_wake.value_or(1.0) - 1.0) < 0.2, F("thick wing: lift from the pressures %.4f agrees with the wake", cl_thick));
		// Symmetric sections at incidence: the root station loads like an airfoil.
		const NetworkResult& b = s_thick.networks[0];
		double cp_min = 1.0;
		for (int i = 0; i < b.panel_rows; ++i) cp_min = std::min(cp_min, b.panels[static_cast<size_t>(i) * b.panel_cols + 1].cp_upper);
		Check(cp_min < -0.5 && cp_min > -4.0, F("thick wing: suction peak Cp %.3f near the root", cp_min));

		WingPaneling thin = thick;
		thin.thick = false;
		t0 = std::chrono::steady_clock::now();
		std::vector<Network> nets_thin = WingNetworks(wing, thin, vinf);
		Solution s_thin = Solve(nets_thin, vinf, opt);
		secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
		std::printf("mean-surface wing: %d unknowns in %.2f s; C_L wake %.4f\n", s_thin.unknowns, secs, s_thin.lift_from_wake.value_or(0.0));
		Check(nets_thin[0].rows == 11 && nets_thin[0].cols == 11, "mean-surface wing: one sheet of 10 x 10 panels");
		Check(std::fabs(s_thin.lift_from_wake.value_or(0.0) / cl_ll - 1.0) < 0.2, F("mean-surface wing: lift from the wake %.4f within 20%% of lifting line %.4f", s_thin.lift_from_wake.value_or(0.0), cl_ll));
		Check(std::fabs(s_thin.lift_from_wake.value_or(0.0) / s_thick.lift_from_wake.value_or(1.0) - 1.0) < 0.15, "thin and thick formulations agree on the lift to 15%");
		// Zero incidence on symmetric sections: no lift.
		Solution s0 = Solve(WingNetworks(wing, thick, {1.0, 0.0, 0.0}), {1.0, 0.0, 0.0}, opt);
		Check(std::fabs(s0.lift_from_wake.value_or(1.0)) < 1e-3, F("thick wing at zero incidence: no lift (%.2e)", s0.lift_from_wake.value_or(1.0)));

		// ---- bodies from their lofts ----
		// A sphere primitive as a source network: 1.5 sin θ again, now through
		// BodyNetworks and the primitive's own loft (poles as collapsed rows).
		{
			PrimitiveModel sphere;
			sphere.shape = PrimitiveShape::Sphere;
			sphere.radius = 1.0;
			BodyPaneling bp;
			bp.along = 12; bp.around = 16;
			std::vector<Network> nets = BodyNetworks(sphere, "sphere", bp, {1.0, 0.0, 0.0});
			Check(nets.size() == 1 && nets[0].type == NetworkType::SourceAnalysis && nets[0].curved, "sphere primitive: one curved source network");
			Options o;
			o.reference_area = 3.14159265358979323846;
			Solution s = Solve(nets, {1.0, 0.0, 0.0}, o);
			double worst = 0.0, vn = 0.0;
			const NetworkResult& nr = s.networks[0];
			for (int i = 0; i < nr.panel_rows; ++i) for (int j = 0; j < nr.panel_cols; ++j) {
				const PanelResult& p = nr.panels[static_cast<size_t>(i) * nr.panel_cols + j];
				double th = std::acos(std::clamp(p.centre.x / Length(p.centre), -1.0, 1.0));
				double err = std::fabs(Length(p.v_upper) - 1.5 * std::sin(th));
				if (err > 0.05) std::printf("  sphere panel (%d, %d): centre (%.3f, %.3f, %.3f) |c| %.3f n (%.3f, %.3f, %.3f) |V| %.3f want %.3f sigma %.3f area %.4f\n", i, j, p.centre.x, p.centre.y, p.centre.z, Length(p.centre), p.normal.x, p.normal.y, p.normal.z, Length(p.v_upper), 1.5 * std::sin(th), p.sigma, p.area);
				worst = std::max(worst, err);
				vn = std::max(vn, std::fabs(Dot(p.v_upper, p.normal)));
			}
			Check(worst < 0.05, F("sphere primitive: |V| = 1.5 sin θ (worst %.4f)", worst));
			Check(vn < 1e-6 && Length(s.force_coefficient) < 0.03, F("sphere primitive: impermeable, no net force (|C_F| %.4f)", Length(s.force_coefficient)));
			// The same by the interior-potential formulation.
			bp.sources = false;
			Solution sd = Solve(BodyNetworks(sphere, "sphere", bp, {1.0, 0.0, 0.0}), {1.0, 0.0, 0.0}, o);
			double worst_d = 0.0;
			for (const PanelResult& p : sd.networks[0].panels) {
				double th = std::acos(std::clamp(p.centre.x / Length(p.centre), -1.0, 1.0));
				worst_d = std::max(worst_d, std::fabs(Length(p.v_upper) - 1.5 * std::sin(th)));
			}
			Check(worst_d < 0.08, F("sphere primitive, interior potential: |V| = 1.5 sin θ (worst %.4f)", worst_d));
		}
		// A box: flat panels, runs, no flow through the faces.
		{
			PrimitiveModel box;
			box.shape = PrimitiveShape::Box;
			box.size = {2.0, 1.0, 1.0};
			BodyPaneling bp;
			bp.along = 8; bp.around = 16;
			std::vector<Network> nets = BodyNetworks(box, "box", bp, {1.0, 0.0, 0.0});
			Check(nets.size() == 1 && !nets[0].curved, "box primitive: one flat source network");
			Solution s = Solve(nets, {1.0, 0.0, 0.0});
			double vn = 0.0;
			for (const PanelResult& p : s.networks[0].panels) vn = std::max(vn, std::fabs(Dot(p.v_upper, p.normal)));
			Check(vn < 1e-6 && s.max_delta == 0.0, F("box primitive: impermeable with flat panels (normal velocity %.2e)", vn));
		}
		// The default fuselage: a pointed nose and tail, impermeable, little net force.
		{
			FuselageSpec fs;
			FuselageModel fus(fs);
			BodyPaneling bp;
			bp.along = 20; bp.around = 16;
			std::vector<Network> nets = BodyNetworks(fus, "fuselage", bp, {1.0, 0.0, 0.0});
			Check(nets.size() == 1 && nets[0].rows > 20 && nets[0].cols > 16, "fuselage: one network on the loft's stations plus the uniform ones");
			Options o;
			o.reference_area = 1.0;
			Solution s = Solve(nets, {1.0, 0.0, 0.0}, o);
			double vn = 0.0, cpmin = 1.0;
			{
				const NetworkResult& nr = s.networks[0];
				for (int i = 0; i < nr.panel_rows; ++i) for (int j = 0; j < nr.panel_cols; ++j) {
					const PanelResult& p = nr.panels[static_cast<size_t>(i) * nr.panel_cols + j];
					if (p.cp_upper < -1.0 || p.cp_upper > 1.5) std::printf("  fuselage panel (%d, %d): centre (%.3f, %.3f, %.3f) n (%.3f, %.3f, %.3f) Cp %.3f sigma %.3f area %.3e delta %.3f\n", i, j, p.centre.x, p.centre.y, p.centre.z, p.normal.x, p.normal.y, p.normal.z, p.cp_upper, p.sigma, p.area, p.delta);
					vn = std::max(vn, std::fabs(Dot(p.v_upper, p.normal))); cpmin = std::min(cpmin, p.cp_upper);
				}
			}
			std::printf("fuselage: %d unknowns, Cp min %.3f, |C_F| %.4f, max delta %.3f\n", s.unknowns, cpmin, Length(s.force_coefficient), s.max_delta);
			Check(vn < 1e-6, F("fuselage: impermeable (normal velocity %.2e)", vn));
			Check(cpmin < 0.0 && cpmin > -1.0, F("fuselage: a sensible suction (Cp min %.3f)", cpmin));
			// The same body cut at the symmetry plane and solved with its image:
			// half the panels, the same pressures.
			{
				BodyPaneling hp = bp;
				hp.half_plane_y = 0.0;
				std::vector<Network> hn = BodyNetworks(fus, "fuselage", hp, {1.0, 0.0, 0.0});
				double ymin = 1e300;
				for (const Vec3& p : hn[0].corners) ymin = std::min(ymin, p.y);
				Check(hn[0].cols < nets[0].cols && ymin > -1e-9, F("fuselage half: %.0f columns of the ring, none below y = 0", static_cast<double>(hn[0].cols)));
				Options oh = o;
				oh.symmetry_y = 0.0;
				Solution sh = Solve(hn, {1.0, 0.0, 0.0}, oh);
				double vnh = 0.0, cpminh = 1.0;
				for (const PanelResult& p : sh.networks[0].panels) { vnh = std::max(vnh, std::fabs(Dot(p.v_upper, p.normal))); cpminh = std::min(cpminh, p.cp_upper); }
				std::printf("fuselage half: %d unknowns, Cp min %.3f, |C_F| %.4f\n", sh.unknowns, cpminh, Length(sh.force_coefficient));
				Check(vnh < 1e-6 && std::fabs(cpminh - cpmin) < 0.02, F("fuselage half with its image: impermeable, Cp min %.3f against %.3f whole", cpminh, cpmin));
				Check(std::fabs(sh.force_coefficient.y) < 1e-12 && std::fabs(sh.force_coefficient.x - s.force_coefficient.x) < 0.01, F("fuselage half: the force agrees with the whole body (C_X %.4f vs %.4f)", sh.force_coefficient.x, s.force_coefficient.x));
			}
			// A nacelle is a ring: the periodic chain of stations closes on a seam.
			{
				LoftedNacelleSpec ns;
				LoftedNacelleModel nac(ns);
				BodyPaneling np;
				np.along = 8; np.around = 12;
				std::vector<Network> nn = BodyNetworks(nac, "nacelle", np, {1.0, 0.0, 0.0});
				Check(nn.size() == 1 && nn[0].rows > 8, "lofted nacelle: one network round the periodic chain");
				Solution sn = Solve(nn, {1.0, 0.0, 0.0}, o);
				double vnn = 0.0, cpn = 1.0;
				for (const PanelResult& p : sn.networks[0].panels) { vnn = std::max(vnn, std::fabs(Dot(p.v_upper, p.normal))); cpn = std::min(cpn, p.cp_upper); }
				std::printf("lofted nacelle: %d unknowns, Cp min %.3f, |C_F| %.4f, max delta %.3f\n", sn.unknowns, cpn, Length(sn.force_coefficient), sn.max_delta);
				Check(vnn < 1e-6 && cpn < 0.0 && cpn > -3.0, F("lofted nacelle: impermeable, Cp min %.3f", cpn));
			}
			// Together with the wing: one system, both bodies impermeable, the wing still lifting.
			std::vector<Network> all = WingNetworks(wing, thick, vinf);
			for (Network& n : BodyNetworks(fus, "fuselage", bp, vinf)) all.push_back(n);
			Solution sa = Solve(all, vinf, opt);
			Check(sa.networks.size() == 3 && sa.lift_from_wake.has_value() && *sa.lift_from_wake > 0.2, F("wing and fuselage together: the wing lifts (C_L %.4f)", sa.lift_from_wake.value_or(0.0)));
		}
	} catch (const std::exception& e) {
		std::printf("FAIL: exception: %s\n", e.what());
		++g_fail;
	}
	std::printf("\nRESULT: %d passed, %d failed\n", g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}
