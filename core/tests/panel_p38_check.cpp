// The P-38 wing of ue/Projects/p38.yaml -- a dihedral slope at the root
// (which tilts the root section off the symmetry plane) and a tip that
// closes to 0.6% of the root chord -- paneled as the High Order Panel
// dialog panels it (thick, 12 x 10, the image about y = 0, 4 degrees): the
// span loading (-Cp n dA strip by strip on the closed surface) must be
// smooth and positive to the tip, the root column must not spike, the
// image must agree with the explicit full wing, and the pressure lift
// with the wake lift. Found 2026-10-27 as Cp -30 on the
// root and tip columns and a negative tip load; the fixes are the root
// pinned to the plane, the symmetry/abutment edge conditions and the
// inset collocation on potential networks, the collapsed pointed tip, and
// the interior-velocity fallback (see panel_method.hpp, wing_panels.hpp).
// The default wing at the dialog's 12 x 10 is checked the same way.

#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>

#include "aerotelic/panel_loads.hpp"
#include "aerotelic/project_yaml.hpp"
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

Network Mirrored(const Network& N) {
	Network M = N;
	M.name = N.name + " (mirror)";
	for (Vec3& P : M.corners) P.y = -P.y;
	FlipNormals(M);
	return M;
}

struct Strip { double s, width, chord, l; };

std::vector<Strip> LiftStrips(const Network& net, const NetworkResult& res, const Vec3& lift) {
	StripOptions so;
	so.strips_are_columns = true;
	std::vector<Strip> out;
	for (const StripLoad& st : StripLoads(net, res, so)) out.push_back({st.s, st.width, st.chord, Dot(st.force, lift) / st.width});
	return out;
}

// The checks on one wing at the dialog's defaults.
void CheckWing(const std::string& label, const WingModel& wing, double cl_tolerance) {
	const Planform& pf = wing.planform();
	const double alpha = 4.0 * 3.14159265358979323846 / 180.0;
	const Vec3 vinf{std::cos(alpha), 0.0, std::sin(alpha)};
	const Vec3 lift{-std::sin(alpha), 0.0, std::cos(alpha)};
	WingPaneling pan;
	pan.chordwise = 12;
	pan.spanwise = 10;
	pan.wake_length = 20.0 * pf.RootChord();
	pan.wake_direction = vinf;
	std::vector<Network> nets = WingNetworks(wing, pan, vinf);
	double root_y = 0.0;
	for (int I = 1; I <= nets[0].rows; ++I) root_y = std::max(root_y, std::fabs(nets[0].Corner(I, 1).y));
	Check(root_y == 0.0, label + F(": the root station lies in the plane y = 0 (max |y| %.1e)", root_y));

	Options opt;
	opt.symmetry_y = 0.0;
	opt.reference_area = pf.PlanformArea();
	Solution a = Solve(nets, vinf, opt);
	const double cl_a = Dot(a.force_coefficient, lift), wake_a = a.lift_from_wake.value_or(0.0);
	std::printf("%s: thick 12 x 10 with the image: C_L %.4f, wake %.4f, %d unknowns, pivot %.2e, max delta %.3f\n", label.c_str(), cl_a, wake_a, a.unknowns, a.pivot_ratio, a.max_delta);
	Check(wake_a > 0.2 && std::fabs(cl_a - wake_a) < cl_tolerance * wake_a, label + F(": pressure lift %.4f within %.0f%% of the wake lift %.4f", cl_a, 100.0 * cl_tolerance, wake_a));

	std::vector<Strip> strips = LiftStrips(nets[0], a.networks[0], lift);
	std::printf("   k        s    width    chord        l       cl\n");
	double l_max = 0.0;
	bool positive = true;
	for (size_t k = 0; k < strips.size(); ++k) {
		const Strip& st = strips[k];
		std::printf("  %2zu %8.4f %8.4f %8.4f %8.4f %8.4f\n", k, st.s, st.width, st.chord, st.l, st.l / st.chord);
		l_max = std::max(l_max, st.l);
		positive = positive && st.l > 0.0;
	}
	Check(positive, label + ": every strip's lift per span is positive");
	Check(strips.back().l < 0.35 * l_max && strips.back().l < strips[strips.size() - 2].l, label + F(": the tip strip carries the least (%.4f of a peak %.4f)", strips.back().l, l_max));
	Check(std::fabs(strips[0].l - strips[1].l) < 0.2 * strips[1].l, label + F(": the root strip %.4f within 20%% of its neighbour %.4f (no spike)", strips[0].l, strips[1].l));
	Check(strips.front().chord > strips.back().chord && strips.front().chord > 0.9 * pf.RootChord(), label + F(": strip chords taper %.3f -> %.3f", strips.front().chord, strips.back().chord));
	double cp_min_ends = 1.0;
	const NetworkResult& r = a.networks[0];
	for (int i = 0; i < r.panel_rows; ++i) {
		cp_min_ends = std::min(cp_min_ends, r.panels[static_cast<size_t>(i) * r.panel_cols].cp_upper);
		cp_min_ends = std::min(cp_min_ends, r.panels[static_cast<size_t>(i) * r.panel_cols + r.panel_cols - 1].cp_upper);
	}
	Check(cp_min_ends > -5.0, label + F(": Cp on the root and tip columns stays above -5 (min %.2f)", cp_min_ends));

	// The explicit full wing without the plane agrees with the image.
	std::vector<Network> full = nets;
	full.push_back(Mirrored(nets[0]));
	full.push_back(Mirrored(nets[1]));
	Options opt_full = opt;
	opt_full.symmetry_y.reset();
	Solution b = Solve(full, vinf, opt_full);
	const double cl_b = Dot(b.force_coefficient, lift);
	std::vector<Strip> strips_b = LiftStrips(full[0], b.networks[0], lift);
	Check(std::fabs(cl_b - cl_a) < 0.01 * cl_a && std::fabs(strips_b[0].l - strips[0].l) < 0.02 * strips[0].l,
		label + F(": the explicit full wing agrees with the image (C_L %.4f vs %.4f,", cl_b, cl_a) + F(" root strip %.4f vs %.4f)", strips_b[0].l, strips[0].l));

	// Plane cuts every 5% of the semi-span: smooth, positive, zero at the
	// collapsed tip, and integrating to the strips' total.
	{
		double s_root = 1e300, s_tip = -1e300;
		for (const Vec3& p : nets[0].corners) { s_root = std::min(s_root, p.y); s_tip = std::max(s_tip, p.y); }
		double integral = 0.0, prev = 0.0, l_max = 0.0, l_tip = 0.0;
		bool positive = true, all_found = true;
		std::printf("   cuts: eta  l/q\n");
		for (int k = 0; k <= 20; ++k) {
			const double s = s_root + (s_tip - s_root) * k / 20.0;
			SectionCut c = CutSection(nets[0], a.networks[0], {0.0, 1.0, 0.0}, s);
			const double l = Dot(c.force_per_span, lift);
			std::printf("        %.2f %8.4f%s\n", k / 20.0, l, c.found ? "" : " (not found)");
			all_found = all_found && c.found;
			if (k > 0) integral += 0.5 * (l + prev) * (s_tip - s_root) / 20.0;
			prev = l;
			if (k < 20) positive = positive && l > 0.0;
			l_max = std::max(l_max, l);
			if (k == 20) l_tip = l;
		}
		double strips_total = 0.0;
		for (const Strip& st : strips) strips_total += st.l * st.width;
		Check(all_found && positive, label + ": every cut found, positive lift per span inboard of the tip");
		if (pf.ChordAt(1.0) < 0.02 * pf.RootChord()) Check(std::fabs(l_tip) < 1e-6 * l_max, label + F(": the cut at the collapsed tip carries nothing (%.2e of a peak %.4f)", l_tip, l_max));
		else Check(l_tip > 0.0 && l_tip < 0.5 * l_max, label + F(": the cut at the square tip carries a little (%.4f of a peak %.4f)", l_tip, l_max));
		Check(std::fabs(integral - strips_total) < 0.03 * strips_total, label + F(": the cuts integrate to the strips' lift (%.4f vs %.4f)", integral, strips_total));
	}

	// The total over both halves in uniform bins: symmetric, conserving the strips' total.
	{
		std::vector<LoadSource> sources = {{&nets[0], &a.networks[0], false}, {&nets[0], &a.networks[0], true}};
		double hi = 0.0;
		for (const Vec3& p : nets[0].corners) hi = std::max(hi, std::fabs(p.y));
		std::vector<double> edges;
		const int n = 20;
		for (int k = 0; k <= n; ++k) edges.push_back(-hi + 2.0 * hi * k / n);
		std::vector<Vec3> bins = BinnedLoads(sources, {0.0, 1.0, 0.0}, edges);
		double total = 0.0, asym = 0.0, strips_total = 0.0;
		for (int k = 0; k < n; ++k) {
			total += Dot(bins[k], lift);
			asym = std::max(asym, std::fabs(Dot(bins[k] - bins[n - 1 - k], lift)));
		}
		for (const Strip& st : strips) strips_total += st.l * st.width;
		Check(std::fabs(total - 2.0 * strips_total) < 1e-9 * (1.0 + std::fabs(total)) && asym < 1e-9 * (1.0 + std::fabs(total)),
			label + F(": binned over both halves: total %.4f = twice the strips' %.4f, symmetric", total, strips_total));
	}
}

} // namespace

int main(int argc, char** argv) {
	setvbuf(stdout, nullptr, _IONBF, 0);
	try {
		// The P-38 project, wherever the test is run from.
		std::vector<std::string> candidates;
		if (argc > 1) candidates.push_back(argv[1]);
		for (const char* rel : {"ue/Projects/p38.yaml", "../ue/Projects/p38.yaml", "../../ue/Projects/p38.yaml", "../../../ue/Projects/p38.yaml"}) candidates.push_back(rel);
		std::string path;
		for (const std::string& c : candidates) if (std::ifstream(c).good()) { path = c; break; }
		if (path.empty()) {
			Check(false, "ue/Projects/p38.yaml not found from the working directory (pass its path as the first argument)");
		} else {
			project_yaml::ProjectFile pf = project_yaml::LoadProjectYaml(path);
			const project_yaml::ProjectEntity* wing_entity = nullptr;
			for (const auto& e : pf.entities) if (e.type == project_yaml::EntityType::Wing && e.name == "Wing") wing_entity = &e;
			Check(wing_entity != nullptr, "the P-38 project has its Wing");
			if (wing_entity) {
				WingModel wing(std::get<WingModelSpec>(wing_entity->model));
				Check(wing.planform().ChordAt(1.0) < 0.02 * wing.planform().RootChord(), F("the P-38 tip chord %.4f is under 2%% of the root %.4f (collapsed to a point)", wing.planform().ChordAt(1.0), wing.planform().RootChord()));
				CheckWing("P-38 wing", wing, 0.05);
			}
		}
		PlanformSpec ps; // semi-span 10, root 3.1, tip 0.5, sweep 20 degrees
		WingModel wing(ps, "Wing");
		wing.AddSection(Airfoil::Naca4("0012"), 0.0);
		wing.AddSection(Airfoil::Naca4("0012"), 1.0);
		CheckWing("default wing", wing, 0.05);
	} catch (const std::exception& e) {
		Check(false, std::string("exception: ") + e.what());
	}
	std::printf("RESULT: %d passed, %d failed\n", g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}
