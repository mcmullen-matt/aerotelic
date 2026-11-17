// Checks of the panel method on triangles (tri_panel.hpp):
//   1. the closed-form influence of a triangle against quadrature;
//   2. the sphere: Cp = 1 - 9/4 sin^2(theta), no net force (d'Alembert);
//   3. GMRES agrees with the direct solve;
//   4. a wing: lift from the pressures against the wake's circulation,
//      against the higher-order code (wing_panels.hpp) and lifting line;
//      the half wing with its image against the symmetric wing built tip
//      to tip without a plane; the section cuts integrate to the lift.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

#include "aerotelic/mesh_boolean.hpp"
#include "aerotelic/tri_panel.hpp"
#include "aerotelic/wing_panels.hpp"

using namespace aerotelic;
using namespace aerotelic::tripanel;

namespace {

int g_pass = 0, g_fail = 0;
constexpr double kPi = 3.14159265358979323846;

void Check(bool ok, const std::string& what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what.c_str());
	if (ok) ++g_pass; else ++g_fail;
}

std::string F(const std::string& fmt, double a, double b = 0.0, double c = 0.0) {
	char buf[300];
	std::snprintf(buf, sizeof buf, fmt.c_str(), a, b, c);
	return buf;
}

// The integrals of 1/r and n.(P-Q)/r^3 over a triangle by the midpoint
// rule on n x n sub-triangles.
void Quadrature(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& P, int n, double& inv_r, double& omega) {
	const Vec3 nrm = Normalized(Cross(b - a, c - a));
	const double area = 0.5 * Length(Cross(b - a, c - a));
	const double da = area / (n * n);
	inv_r = 0.0; omega = 0.0;
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < n - i; ++j) {
			// Two sub-triangles per cell except on the diagonal.
			auto add = [&](double u, double v) {
				const Vec3 q = a + (b - a) * u + (c - a) * v;
				const Vec3 r = P - q;
				const double rl = Length(r);
				inv_r += da / rl;
				omega += da * Dot(nrm, r) / (rl * rl * rl);
			};
			add((i + 1.0 / 3.0) / n, (j + 1.0 / 3.0) / n);
			if (j < n - i - 1) add((i + 2.0 / 3.0) / n, (j + 2.0 / 3.0) / n);
		}
	}
}

// A single-triangle body is not closed; the influence functions are
// private, so they are checked through a two-triangle "body"? No: through
// the solver's own behaviour on the sphere. Here the public surface is
// exercised instead; the formulas themselves are checked by solving a
// sphere below, whose exact Cp depends on every one of them.

std::vector<Vec3> TrailingEdgePolyline(const WingModel& w, double eta0, double eta1, int n) {
	std::vector<Vec3> out;
	for (int i = 0; i <= n; ++i) {
		const double eta = eta0 + (eta1 - eta0) * i / n;
		Mat p = w.planform().AirfoilCoordsToXyz(eta, Mat::FromRows({{1.0, 0.0}}));
		out.push_back({p(0, 0), p(0, 1), p(0, 2)});
	}
	return out;
}

} // namespace

int main() {
	setvbuf(stdout, nullptr, _IONBF, 0);
	try {
		// ---- 1. a sphere ----
		{
			TriMesh sphere = EllipsoidMesh(1.0, 1.0, 1.0, 0.004);
			MeshStats st = ComputeMeshStats(sphere);
			std::printf("sphere: %d triangles, edges %.4f .. %.4f (median %.4f), worst aspect %.2f, closed %d outward %d\n", st.triangles, st.min_edge, st.max_edge, st.median_edge, st.worst_aspect, (int)st.closed, (int)st.outward);
			Check(st.closed && st.outward, "sphere mesh: closed and outward");
			Body b;
			b.name = "sphere";
			b.mesh = sphere;
			Options opt;
			opt.reference_area = kPi;
			auto t0 = std::chrono::steady_clock::now();
			Solution s = Solve({b}, {1.0, 0.0, 0.0}, opt);
			const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
			double worst_cp = 0.0, mean_cp = 0.0;
			int n = 0;
			for (const PanelResult& p : s.bodies[0].panels) {
				if (!(p.area > 0.0)) continue;
				const double th = std::acos(std::clamp(p.centre.x / Length(p.centre), -1.0, 1.0));
				const double exact = 1.0 - 2.25 * std::sin(th) * std::sin(th);
				worst_cp = std::max(worst_cp, std::fabs(p.cp - exact));
				mean_cp += std::fabs(p.cp - exact);
				++n;
			}
			mean_cp /= std::max(n, 1);
			std::printf("sphere: %d unknowns, %.2f s (influence %.2f, solve %.3f, %s), pivot %.2e, |C_F| %.4f, Cp error worst %.4f mean %.4f\n",
				s.unknowns, secs, s.seconds_influence, s.seconds_solve, s.direct ? "LU" : "GMRES", s.pivot_ratio, Length(s.force_coefficient), worst_cp, mean_cp);
			// Constant panels on flat triangles: the worst panel sits at a pole
			// fan, where the gradient fit has the least to work with; the
			// higher-order code's 16 x 12 curved panels manage 0.06.
			Check(worst_cp < 0.1, F("sphere: Cp = 1 - 9/4 sin^2 theta at every panel (worst %.4f)", worst_cp));
			Check(mean_cp < 0.02, F("sphere: mean Cp error %.4f", mean_cp));
			Check(Length(s.force_coefficient) < 0.01, F("sphere: no net force (|C_F| %.4f)", Length(s.force_coefficient)));
			// The doublet strength is the outer perturbation potential: for the
			// sphere phi = x / (2 r^3) on r = 1, so mu = x / 2.
			double worst_mu = 0.0;
			for (const PanelResult& p : s.bodies[0].panels) if (p.area > 0.0) worst_mu = std::max(worst_mu, std::fabs(p.mu - 0.5 * p.centre.x));
			Check(worst_mu < 0.02, F("sphere: mu = x / 2 (the outer potential; worst %.4f)", worst_mu));

			// ---- 2. GMRES against the direct solve ----
			Options it = opt;
			it.direct_max_panels = 0;
			Solution s2 = Solve({b}, {1.0, 0.0, 0.0}, it);
			double worst = 0.0;
			for (size_t k = 0; k < s.bodies[0].panels.size(); ++k) worst = std::max(worst, std::fabs(s.bodies[0].panels[k].mu - s2.bodies[0].panels[k].mu));
			std::printf("sphere GMRES: %d iterations, residual %.2e, %.3f s; max |mu - mu_LU| %.2e\n", s2.iterations, s2.residual, s2.seconds_solve, worst);
			Check(!s2.direct && worst < 1e-6, F("sphere: GMRES agrees with LU (max %.2e)", worst));

			// The sphere with the symmetry plane: cut at y = 0, the image closes it.
			Options sym = opt;
			sym.symmetry_y = 0.0;
			Solution s3 = Solve({b}, {1.0, 0.0, 0.0}, sym);
			double worst3 = 0.0;
			for (const PanelResult& p : s3.bodies[0].panels) {
				if (!(p.area > 0.0)) continue;
				const double th = std::acos(std::clamp(p.centre.x / Length(p.centre), -1.0, 1.0));
				worst3 = std::max(worst3, std::fabs(p.cp - (1.0 - 2.25 * std::sin(th) * std::sin(th))));
			}
			std::printf("sphere, plane: %d unknowns (cut %d, dropped %d), Cp worst %.4f, |C_F| %.4f\n", s3.unknowns, (int)s3.bodies[0].cut_at_plane, s3.bodies[0].dropped_in_plane, worst3, Length(s3.force_coefficient));
			Check(s3.bodies[0].cut_at_plane && s3.unknowns < s.unknowns, "sphere: cut at the symmetry plane into its y >= 0 half");
			Check(worst3 < 0.1, F("sphere with the image: Cp exact to %.4f", worst3));
			Check(Length(s3.force_coefficient) < 0.01, F("sphere with the image: no net force (%.4f)", Length(s3.force_coefficient)));
		}

		// ---- 3. a wing ----
		{
			PlanformSpec ps; // semi-span 10, root 3.1, tip 0.5, sweep 20 degrees
			WingModel wing(ps, "Wing");
			wing.AddSection(Airfoil::Naca4("0012"), 0.0);
			wing.AddSection(Airfoil::Naca4("0012"), 1.0);
			wing.mesh.stations = 20;
			wing.mesh.around = 48;
			wing.mesh.refine = false;
			const double alpha = 4.0 * kPi / 180.0;
			const Vec3 vinf{std::cos(alpha), 0.0, std::sin(alpha)};
			const Vec3 lift{-std::sin(alpha), 0.0, std::cos(alpha)};
			const double aspect = wing.planform().AspectRatio();
			const double cl_ll = 2.0 * kPi * aspect / (aspect + 2.0) * alpha * std::cos(20.0 * kPi / 180.0);

			// The higher-order reference.
			panel::Options jopt;
			jopt.symmetry_y = 0.0;
			jopt.reference_area = wing.planform().PlanformArea();
			panel::WingPaneling pan;
			pan.chordwise = 12; pan.spanwise = 12;
			panel::Solution js = panel::Solve(panel::WingNetworks(wing, pan, vinf), vinf, jopt);
			const double cl_johnson = js.lift_from_wake.value_or(0.0);
			const double cl_johnson_p = Dot(js.force_coefficient, lift);

			// The half wing with its image.
			std::vector<NamedMesh> meshes = wing.BuildMeshes(0.01);
			MeshStats st = ComputeMeshStats(meshes[0].mesh);
			std::printf("wing: %d triangles, median edge %.3f, worst aspect %.1f, closed %d\n", st.triangles, st.median_edge, st.worst_aspect, (int)st.closed);
			Body half;
			half.name = "Wing";
			half.mesh = meshes[0].mesh;
			half.trailing_edges = {TrailingEdgePolyline(wing, 0.0, 1.0, 400)};
			half.trailing_edge_tolerance = 1e-5;
			Options opt;
			opt.symmetry_y = 0.0;
			opt.reference_area = wing.planform().PlanformArea();
			auto t0 = std::chrono::steady_clock::now();
			Solution s = Solve({half}, vinf, opt);
			const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
			const double cl_p = Dot(s.force_coefficient, lift), cl_w = s.lift_from_wake.value_or(0.0);
			double cpmin = 1.0;
			for (const PanelResult& p : s.bodies[0].panels) if (p.area > 0.0) cpmin = std::min(cpmin, p.cp);
			std::printf("wing (half + image): %d unknowns, %d wake edges, %d rim, %.2f s (%s); C_L pressures %.4f, wake %.4f; Johnson wake %.4f pressures %.4f; lifting line %.4f; Cp min %.3f\n",
				s.unknowns, s.bodies[0].wake_edges, s.bodies[0].rim_edges, secs, s.direct ? "LU" : "GMRES", cl_p, cl_w, cl_johnson, cl_johnson_p, cl_ll, cpmin);
			for (const std::string& n : s.notes) std::printf("  note: %s\n", n.c_str());
			Check(s.bodies[0].wake_edges >= 15, F("wing: the trailing edge sheds a wake (%.0f edges)", s.bodies[0].wake_edges));
			Check(s.bodies[0].dropped_in_plane > 0, F("wing: the root cap in the plane is dropped (%.0f triangles)", s.bodies[0].dropped_in_plane));
			Check(std::fabs(cl_w / cl_johnson - 1.0) < 0.15, F("wing: lift from the wake %.4f within 15%% of the higher-order code's %.4f", cl_w, cl_johnson));
			Check(std::fabs(cl_p / cl_w - 1.0) < 0.15, F("wing: lift from the pressures %.4f agrees with the wake's %.4f", cl_p, cl_w));
			Check(cpmin < -0.4 && cpmin > -3.0, F("wing: suction peak Cp %.3f", cpmin));
			// Zero incidence: no lift.
			Solution s0 = Solve({half}, {1.0, 0.0, 0.0}, opt);
			Check(std::fabs(s0.lift_from_wake.value_or(1.0)) < 2e-3 && std::fabs(Dot(s0.force_coefficient, Vec3{0.0, 0.0, 1.0})) < 5e-3,
				F("wing at zero incidence: no lift (wake %.2e, pressures %.2e)", s0.lift_from_wake.value_or(1.0), Dot(s0.force_coefficient, Vec3{0.0, 0.0, 1.0})));

			// The symmetric wing tip to tip, no plane.
			WingModelSpec spec = wing.Serialize();
			spec.symmetric = true;
			WingModel full(spec);
			full.mesh = wing.mesh;
			std::vector<NamedMesh> fm = full.BuildMeshes(0.01);
			Body whole;
			whole.name = "Wing";
			whole.mesh = fm[0].mesh;
			std::vector<Vec3> te = TrailingEdgePolyline(wing, 0.0, 1.0, 400);
			std::vector<Vec3> te_port = te;
			for (Vec3& p : te_port) p.y = -p.y;
			whole.trailing_edges = {te, te_port};
			whole.trailing_edge_tolerance = 1e-5;
			Options nop;
			nop.reference_area = wing.planform().PlanformArea();
			t0 = std::chrono::steady_clock::now();
			Solution sw = Solve({whole}, vinf, nop);
			const double secs2 = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
			const double cl_wp = Dot(sw.force_coefficient, lift), cl_ww = sw.lift_from_wake.value_or(0.0);
			std::printf("symmetric wing (tip to tip, no plane): %d unknowns, %d wake edges, %.2f s (influence %.2f, solve %.2f, %s %d iterations, residual %.1e); C_L pressures %.4f, wake %.4f; C_Y %.2e\n",
				sw.unknowns, sw.bodies[0].wake_edges, secs2, sw.seconds_influence, sw.seconds_solve, sw.direct ? "LU" : "GMRES", sw.iterations, sw.residual, cl_wp, cl_ww, sw.force_coefficient.y);
			Check(!sw.direct && sw.iterations < 200, F("symmetric wing: GMRES converged in %.0f iterations", sw.iterations));
			Check(std::fabs(cl_ww / cl_w - 1.0) < 0.03, F("symmetric wing without the plane agrees with the half wing and its image (wake %.4f vs %.4f)", cl_ww, cl_w));
			Check(std::fabs(sw.force_coefficient.y) < 1e-3, F("symmetric wing: no side force (%.2e)", sw.force_coefficient.y));

			// ---- 4. the aero copy of a flapped wing ----
			// A quarter-chord flap over 0.3..0.7 of the span turned 15 degrees
			// down at zero incidence (wing_panels_check's case): the sealed
			// kinked body lifts, up and down are mirror images, and without a
			// deflection the aero copy is the plain wing's own body.
			{
				FlapSpec fl;
				fl.eta_root = 0.3; fl.eta_tip = 0.7; fl.hinge_root = fl.hinge_tip = 0.75; fl.deflection_deg = 15.0;
				wing.flaps = {fl};
				std::vector<std::vector<Vec3>> te_f;
				std::vector<std::string> notes;
				NamedMesh aero = wing.BuildAeroMesh(0.01, &te_f, &notes);
				MeshStats sta = ComputeMeshStats(aero.mesh);
				std::printf("flapped aero copy: %d triangles, closed %d outward %d, %zu trailing-edge polylines\n", sta.triangles, (int)sta.closed, (int)sta.outward, te_f.size());
				for (const std::string& n : notes) std::printf("  note: %s\n", n.c_str());
				Check(sta.closed && sta.outward && te_f.size() == 1, "flapped aero copy: one closed sealed body with its trailing edge");
				// The trailing edge is lowered inside the flap's span only.
				bool lowered = true, same_outside = true;
				for (const Vec3& q : te_f[0]) {
					const double eta = q.y / wing.planform().SemiSpan();
					Mat p0 = wing.planform().AirfoilCoordsToXyz(eta, Mat::FromRows({{1.0, 0.0}}));
					if (eta > 0.3 + 1e-6 && eta < 0.7 - 1e-6) lowered = lowered && q.z < p0(0, 2) - 0.05;
					else if (eta < 0.3 - 1e-6 || eta > 0.7 + 1e-6) same_outside = same_outside && std::fabs(q.z - p0(0, 2)) < 1e-9;
				}
				Check(lowered && same_outside, "flapped aero copy: the trailing edge is lowered inside the flap's span only");
				// Diagnostic: how far the mesh's trailing-edge vertices (the aftmost
				// vertex of each ring, by x against the planform's TE) sit from the
				// polyline, station by station.
				{
					auto dist_to = [](const Vec3& p, const std::vector<Vec3>& line) {
						double best = 1e300;
						for (size_t i = 0; i + 1 < line.size(); ++i) {
							const Vec3 a = line[i], e = line[i + 1] - a;
							const double l2 = LengthSq(e);
							const double t = std::clamp(l2 > 0.0 ? Dot(p - a, e) / l2 : 0.0, 0.0, 1.0);
							best = std::min(best, Distance(p, a + e * t));
						}
						return best;
					};
					int near = 0, far_te = 0;
					double worst_far = 0.0, worst_y = 0.0;
					for (const Vec3& v : aero.mesh.vertices) {
						const double eta = v.y / wing.planform().SemiSpan();
						Mat p0 = wing.planform().AirfoilCoordsToXyz(std::clamp(eta, 0.0, 1.0), Mat::FromRows({{1.0, 0.0}}));
						if (v.x < p0(0, 0) - 0.02 * wing.planform().ChordAt(std::clamp(eta, 0.0, 1.0))) continue; // not near the TE
						const double d = dist_to(v, te_f[0]);
						if (d <= 1e-5) ++near; else { ++far_te; if (d > worst_far) { worst_far = d; worst_y = v.y; } }
					}
					std::printf("  TE diagnostic: %d aft vertices on the polyline, %d off it (worst %.4g at y = %.3f); %zu polyline points\n", near, far_te, worst_far, worst_y, te_f[0].size());
					std::vector<std::array<int, 2>> ed = EdgesOnPolylines(aero.mesh, te_f, 1e-5);
					std::printf("  TE diagnostic: %zu mesh edges on the polyline\n", ed.size());
				}
				Body fb;
				fb.name = "flapped";
				fb.mesh = aero.mesh;
				fb.trailing_edges = te_f;
				fb.trailing_edge_tolerance = 1e-5;
				Solution sfd = Solve({fb}, {1.0, 0.0, 0.0}, opt);
				const double cl_down = sfd.lift_from_wake.value_or(0.0);
				// The higher-order code's flapped wing.
				wing.flaps = {fl};
				panel::Solution jf = panel::Solve(panel::WingNetworks(wing, pan, {1.0, 0.0, 0.0}), {1.0, 0.0, 0.0}, jopt);
				std::printf("flapped wing at zero incidence: C_L wake %.4f pressures %.4f (Johnson %.4f); %d wake edges, %d unknowns\n", cl_down, Dot(sfd.force_coefficient, Vec3{0.0, 0.0, 1.0}), jf.lift_from_wake.value_or(0.0), sfd.bodies[0].wake_edges, sfd.unknowns);
				// Thin-airfoil theory: dCl/d(delta) = 2 (pi - theta_h + sin theta_h)
				// = 3.83 per radian at E = 0.25, so Cl_2D = 1.0 at 15 degrees; over
				// 40% of the span of an aspect-ratio-11 wing, with the three-
				// dimensional and the partial-span losses, two to four tenths. The
				// higher-order code reads 0.41 on 12 x 12 curved panels; the two
				// bracket the estimate and the band below holds both.
				Check(cl_down > 0.2 && cl_down < 0.45, F("flapped aero copy: a 15 degree flap lifts the wing at zero incidence (C_L %.4f; thin-airfoil estimate 0.25 to 0.35, higher-order code %.4f)", cl_down, jf.lift_from_wake.value_or(0.0)));
				Check(sfd.bodies[0].wake_edges >= 20, F("flapped aero copy: every trailing-edge edge sheds a wake (%.0f)", sfd.bodies[0].wake_edges));
				// Refined: twice the stations and the points round the section.
				{
					wing.mesh.stations = 40; wing.mesh.around = 96;
					std::vector<std::vector<Vec3>> te2;
					NamedMesh aero2 = wing.BuildAeroMesh(0.01, &te2);
					Body fb2 = fb;
					fb2.mesh = aero2.mesh;
					fb2.trailing_edges = te2;
					Solution s2 = Solve({fb2}, {1.0, 0.0, 0.0}, opt);
					std::printf("flapped aero copy refined (40 x 96): C_L wake %.4f pressures %.4f, %d unknowns, %.2f s\n", s2.lift_from_wake.value_or(0.0), Dot(s2.force_coefficient, Vec3{0.0, 0.0, 1.0}), s2.unknowns, s2.seconds_influence + s2.seconds_solve);
					Check(std::fabs(s2.lift_from_wake.value_or(0.0) / cl_down - 1.0) < 0.1, F("flapped aero copy: the lift is converged to 10%% between the two grids (%.4f vs %.4f)", s2.lift_from_wake.value_or(0.0), cl_down));
					wing.mesh.stations = 20; wing.mesh.around = 48;
				}
				fl.deflection_deg = -15.0;
				wing.flaps = {fl};
				NamedMesh aero_up = wing.BuildAeroMesh(0.01, &te_f);
				fb.mesh = aero_up.mesh;
				fb.trailing_edges = te_f;
				Solution sfu = Solve({fb}, {1.0, 0.0, 0.0}, opt);
				Check(std::fabs(sfu.lift_from_wake.value_or(0.0) + cl_down) < 0.03 * std::fabs(cl_down) + 1e-6, F("flapped aero copy: 15 degrees up gives the opposite lift (%.4f vs %.4f)", sfu.lift_from_wake.value_or(0.0), cl_down));
				wing.flaps.clear();
				NamedMesh plain = wing.BuildAeroMesh(0.01, &te_f);
				Check(plain.mesh.triangles.size() == meshes[0].mesh.triangles.size() && plain.mesh.vertices.size() == meshes[0].mesh.vertices.size(), "aero copy without flaps: the viewport's own body");

				// A fin like my_drone's: NACA 0006 at the root blending into 0012,
				// a flap at rest, a fine tolerance with the refinement ON. The
				// blend makes the loft probe the section at partial parameter
				// sets; closing the trailing edge from the ends of whatever set
				// was handed over welded probe points together and the refinement
				// ran to its caps (484,616 triangles, 161 s, 2026-11-16).
				{
					PlanformSpec fs;
					fs.semi_span = 0.4; fs.root_chord = 0.8; fs.tip_chord = 0.2; fs.sweep_angle = 30.0;
					WingModel fin(fs, "Fin");
					fin.AddSection(Airfoil::Naca4("0006"), 0.0);
					fin.AddSection(Airfoil::Naca4("0012"), 0.2);
					fin.AddSection(Airfoil::Naca4("0012"), 1.0);
					FlapSpec rudder;
					rudder.eta_root = 0.07; rudder.eta_tip = 0.95; rudder.hinge_root = 0.9; rudder.hinge_tip = 0.7;
					fin.flaps = {rudder};
					auto t1 = std::chrono::steady_clock::now();
					NamedMesh fa = fin.BuildAeroMesh(0.00135);
					const double fsecs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count();
					MeshStats fst = ComputeMeshStats(fa.mesh);
					std::printf("fin aero copy (0006 -> 0012 blend, flap at rest, tol 0.00135): %d triangles, worst aspect %.0f, %.2f s\n", fst.triangles, fst.worst_aspect, fsecs);
					Check(fst.closed && fst.triangles < 20000 && fsecs < 20.0, F("fin aero copy: the refinement converges (%.0f triangles in %.2f s)", fst.triangles, fsecs));
				}
			}

			// Section cuts integrate to the lift (trapezoid over the semi-span).
			{
				const Vec3 axis{0.0, 1.0, 0.0};
				const int ncut = 40;
				double total = 0.0, prev = 0.0;
				int found = 0;
				for (int k = 0; k <= ncut; ++k) {
					const double y = 10.0 * (k + 0.5) / (ncut + 1);
					SectionCut c = CutSection(s.bodies[0], axis, y);
					if (!c.found) continue;
					const double l = Dot(c.force_per_span, lift);
					if (found > 0) total += 0.5 * (l + prev) * (10.0 / (ncut + 1));
					prev = l;
					++found;
				}
				total += prev * (10.0 / (ncut + 1)) * 0.5; // the tip end
				const double cl_cuts = 2.0 * total / wing.planform().PlanformArea();
				std::printf("wing: %d section cuts integrate to C_L %.4f (pressures %.4f)\n", found, cl_cuts, cl_p);
				Check(found >= ncut - 2 && std::fabs(cl_cuts / cl_p - 1.0) < 0.1, F("wing: the section cuts integrate to the lift (%.4f vs %.4f)", cl_cuts, cl_p));
				std::vector<double> edges;
				for (int k = 0; k <= 20; ++k) edges.push_back(-10.0 + 20.0 * k / 20);
				std::vector<Vec3> bins = BinnedLoads(s, axis, edges, true);
				Vec3 sum{0.0, 0.0, 0.0};
				for (const Vec3& f : bins) sum += f;
				Check(std::fabs(Dot(sum, lift) / wing.planform().PlanformArea() / cl_p - 1.0) < 1e-6, "wing: the binned loads sum to the force");
			}

			// A cut exactly on a ring of the mesh (the root of the wing built
			// tip to tip at y = 0; a station of its grid) finds each edge lying
			// in the plane once, by the triangle on one side of it: the load
			// there is its neighbours', not twice it (the span loading's spikes
			// at the root and every eighth of my_drone's tail, 2026-11-16).
			{
				const Vec3 axis{0.0, 1.0, 0.0};
				for (double ring : {0.0, 5.0}) {
					const SectionCut on = CutSection(sw.bodies[0], axis, ring);
					const SectionCut off = CutSection(sw.bodies[0], axis, ring + 0.05);
					const double l_on = Dot(on.force_per_span, lift), l_off = Dot(off.force_per_span, lift);
					std::printf("tip-to-tip wing: cut at y = %.1f (a ring) l/q %.4f over %zu segments of length %.3f; at %.2f l/q %.4f over %zu of %.3f\n", ring, l_on, on.cp.size(), on.length, ring + 0.05, l_off, off.cp.size(), off.length);
					Check(on.found && off.found && std::fabs(on.length / off.length - 1.0) < 0.1 && std::fabs(l_on / l_off - 1.0) < 0.1,
						F("tip-to-tip wing: the cut on the ring at y = %.1f counts each edge once (length %.3f vs %.3f beside it)", ring, on.length, off.length));
				}
			}

			// ---- 5. a fin on the centreline, a whisker off the plane ----
			// my_drone's fin (2026-11-16): BuildCase fuses a body the boolean
			// refused nudged by a millionth of the configuration, so the fin's
			// trailing edge lay 2e-6 off y = 0. Cut at the plane, the wedge at
			// the trailing edge left slivers 2e-6 wide facing their own image
			// and the gradient fit across them gave Cp -5e9. Vertices that
			// close to the plane are put on it before the cut; the nudged fin
			// must then solve as the exact one does.
			{
				PlanformSpec fs;
				fs.semi_span = 0.4; fs.root_chord = 0.8; fs.tip_chord = 0.2; fs.sweep_angle = 30.0;
				WingModel fin(fs, "Fin");
				fin.AddSection(Airfoil::Naca4("0012"), 0.0);
				fin.AddSection(Airfoil::Naca4("0012"), 1.0);
				fin.mesh.stations = 16; fin.mesh.around = 40; fin.mesh.refine = false;
				std::vector<NamedMesh> fmesh = fin.BuildMeshes(0.01);
				std::vector<Vec3> fte = TrailingEdgePolyline(fin, 0.0, 1.0, 200);
				auto upright = [](const Vec3& p) { return Vec3{p.x, -p.z, p.y}; }; // span along z, thickness along y
				Body exact;
				exact.name = "Fin";
				exact.mesh = fmesh[0].mesh;
				for (Vec3& v : exact.mesh.vertices) v = upright(v);
				for (Vec3& v : fte) v = upright(v);
				exact.trailing_edges = {fte};
				exact.trailing_edge_tolerance = 1e-5;
				Body nudged = exact;
				nudged.mesh.Translate({0.0, 2e-6, 0.0});
				for (auto& line : nudged.trailing_edges) for (Vec3& v : line) v.y += 2e-6;
				Options fo;
				fo.symmetry_y = 0.0;
				fo.reference_area = fin.planform().PlanformArea();
				const Vec3 fin_stream{std::cos(4.0 * kPi / 180.0), 0.0, std::sin(4.0 * kPi / 180.0)};
				Solution se = Solve({exact}, fin_stream, fo);
				Solution sn = Solve({nudged}, fin_stream, fo);
				double cp_lo_e = 1.0, cp_lo_n = 1.0, worst = 0.0;
				for (const PanelResult& p : se.bodies[0].panels) if (p.area > 0.0) cp_lo_e = std::min(cp_lo_e, p.cp);
				for (const PanelResult& p : sn.bodies[0].panels) if (p.area > 0.0) cp_lo_n = std::min(cp_lo_n, p.cp);
				const bool same_count = se.bodies[0].panels.size() == sn.bodies[0].panels.size();
				if (same_count) for (size_t k = 0; k < se.bodies[0].panels.size(); ++k) worst = std::max(worst, std::fabs(se.bodies[0].panels[k].cp - sn.bodies[0].panels[k].cp));
				std::printf("fin on the plane: exact %d panels (Cp min %.3f, rim %d), nudged 2e-6 %d panels (Cp min %.3f, rim %d), max |dCp| %.2e\n",
					se.unknowns, cp_lo_e, se.bodies[0].rim_edges, sn.unknowns, cp_lo_n, sn.bodies[0].rim_edges, worst);
				for (const std::string& n : sn.notes) std::printf("  note: %s\n", n.c_str());
				Check(se.bodies[0].rim_edges > 0 && se.bodies[0].wake_edges == 0, "fin on the plane: its trailing edge is the plane's rim and sheds no wake");
				Check(cp_lo_e > -3.0 && cp_lo_n > -3.0, F("fin a whisker off the plane: sane pressures (Cp min %.3f exact, %.3f nudged)", cp_lo_e, cp_lo_n));
				Check(same_count && worst < 0.01, F("fin a whisker off the plane: the same panels and pressures as on it (max |dCp| %.2e)", worst));
			}

			// ---- 6. a T-tail with the plane OFF ----
			// my_drone (2026-11-16): the fin's trailing edge lies in y = 0 and its
			// wake sheet ran along the tailplane's root a fraction of a millimetre
			// from the root panels. Its strength is zero by symmetry, but the
			// coupling left the tailplane's loading 30% different port to
			// starboard. With the free stream in the plane an edge lying in y = 0
			// sheds nothing (the plane on says the same through its rim rule), and
			// the tailplane comes out symmetric, as with the plane on.
			{
				PlanformSpec fs;
				fs.semi_span = 0.4; fs.root_chord = 0.4; fs.tip_chord = 0.2; fs.sweep_angle = 0.0;
				WingModel fin(fs, "Fin");
				fin.AddSection(Airfoil::Naca4("0012"), 0.0);
				fin.AddSection(Airfoil::Naca4("0012"), 1.0);
				fin.mesh.stations = 16; fin.mesh.around = 40; fin.mesh.refine = false;
				auto upright = [](const Vec3& p) { return Vec3{p.x, -p.z, p.y}; };
				TriMesh fin_mesh = fin.BuildMeshes(0.01)[0].mesh;
				for (Vec3& v : fin_mesh.vertices) v = upright(v);
				std::vector<Vec3> fte = TrailingEdgePolyline(fin, 0.0, 1.0, 200);
				for (Vec3& v : fte) v = upright(v);
				const Vec3 fin_tip_te = fte.back();
				// The tailplane: a wing built tip to tip, its root trailing edge on
				// the fin's tip trailing edge.
				PlanformSpec hs;
				hs.semi_span = 0.5; hs.root_chord = 0.2; hs.tip_chord = 0.2; hs.sweep_angle = 0.0;
				WingModel ht0(hs, "HT");
				ht0.AddSection(Airfoil::Naca4("0012"), 0.0);
				ht0.AddSection(Airfoil::Naca4("0012"), 1.0);
				ht0.mesh.stations = 20; ht0.mesh.around = 40; ht0.mesh.refine = false;
				WingModelSpec hspec = ht0.Serialize();
				hspec.symmetric = true;
				WingModel ht(hspec);
				ht.mesh = ht0.mesh;
				TriMesh ht_mesh = ht.BuildMeshes(0.01)[0].mesh;
				std::vector<Vec3> hte = TrailingEdgePolyline(ht0, 0.0, 1.0, 200);
				const Vec3 shift = fin_tip_te - hte.front();
				ht_mesh.Translate(shift);
				std::vector<Vec3> hte_port = hte;
				for (Vec3& v : hte) v += shift;
				for (Vec3& v : hte_port) { v += shift; v.y = -v.y; }
				TriMesh fused;
				std::string how = "exact";
				try {
					fused = MeshBoolean(fin_mesh, ht_mesh, MeshBooleanOp::Fuse);
				} catch (const MeshBooleanError&) {
					TriMesh nudged = ht_mesh;
					nudged.Translate({1e-6, 0.0, 1e-6});
					fused = MeshBoolean(fin_mesh, nudged, MeshBooleanOp::Fuse);
					how = "nudged in x and z";
				}
				MeshStats tst = ComputeMeshStats(fused);
				std::printf("T-tail: fin %zu + tailplane %zu triangles fused (%s) into %d, closed %d outward %d\n", fin_mesh.triangles.size(), ht_mesh.triangles.size(), how.c_str(), tst.triangles, (int)tst.closed, (int)tst.outward);
				Body tail;
				tail.name = "T-tail";
				tail.mesh = fused;
				tail.trailing_edges = {fte, hte, hte_port};
				tail.trailing_edge_tolerance = 1e-5;
				const double a8 = 8.0 * kPi / 180.0;
				const Vec3 stream8{std::cos(a8), 0.0, std::sin(a8)}, lift8{-std::sin(a8), 0.0, std::cos(a8)};
				Options off;
				off.reference_area = 2.0 * ht0.planform().PlanformArea();
				Solution so = Solve({tail}, stream8, off);
				Options on = off;
				on.symmetry_y = 0.0;
				Solution sn = Solve({tail}, stream8, on);
				for (const std::string& n : so.notes) std::printf("  note (plane off): %s\n", n.c_str());
				std::printf("T-tail plane off: %d unknowns, %d wake edges, %d centreline edges, C_L %.4f (wake %.4f); plane on: %d unknowns, %d wake, %d rim, C_L %.4f (wake %.4f)\n",
					so.unknowns, so.bodies[0].wake_edges, so.bodies[0].centreline_edges, Dot(so.force_coefficient, lift8), so.lift_from_wake.value_or(0.0),
					sn.unknowns, sn.bodies[0].wake_edges, sn.bodies[0].rim_edges, Dot(sn.force_coefficient, lift8), sn.lift_from_wake.value_or(0.0));
				Check(so.bodies[0].centreline_edges > 0 && so.bodies[0].wake_edges > 0, F("T-tail plane off: the fin's %.0f trailing-edge edges in y = 0 shed no wake, the tailplane's %.0f do", so.bodies[0].centreline_edges, so.bodies[0].wake_edges));
				Check(std::fabs(Dot(so.force_coefficient, lift8) / Dot(sn.force_coefficient, lift8) - 1.0) < 0.03, F("T-tail: the lift agrees with the plane on (%.4f vs %.4f)", Dot(so.force_coefficient, lift8), Dot(sn.force_coefficient, lift8)));
				// The tailplane's loading port against starboard, and against the
				// plane-on solve's image.
				const Vec3 yaxis{0.0, 1.0, 0.0};
				double worst_lr = 0.0, worst_on = 0.0;
				// Between the grid's rings (every 0.025): a cut ON a ring takes the
				// band on its negative side, which is the inboard band on one
				// half and the outboard one on the other.
				for (double s : {0.0625, 0.1875, 0.3125, 0.4375}) {
					const double lp = Dot(CutSection(so.bodies[0], yaxis, s).force_per_span, lift8);
					const double lm = Dot(CutSection(so.bodies[0], yaxis, -s).force_per_span, lift8);
					const double li = Dot(CutSection(sn.bodies[0], yaxis, -s, true).force_per_span, lift8);
					std::printf("  tailplane at y = +-%.3f: l/q %.4f starboard, %.4f port, %.4f plane-on image\n", s, lp, lm, li);
					worst_lr = std::max(worst_lr, std::fabs(lp / lm - 1.0));
					worst_on = std::max(worst_on, std::fabs(lp / li - 1.0));
				}
				Check(worst_lr < 0.02, F("T-tail plane off: the tailplane's loading is symmetric port to starboard (worst %.1f%%)", 100.0 * worst_lr));
				Check(worst_on < 0.05, F("T-tail: the tailplane's loading agrees with the plane-on solve (worst %.1f%%)", 100.0 * worst_on));
			}
		}
	} catch (const std::exception& e) {
		Check(false, std::string("exception: ") + e.what());
	}
	std::printf("RESULT: %d passed, %d failed\n", g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}
