// The strip loads of a panel solution (panel_loads.hpp) and the colour map
// catalogue (colormaps.hpp): the strips of the default wing sum to the
// network's own force, run root to tip, carry an elliptic-looking lift,
// and the elliptic distribution integrates back to its total.

#include <cmath>
#include <cstdio>
#include <string>

#include "aerotelic/colormaps.hpp"
#include "aerotelic/panel_loads.hpp"
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

void CheckStrips(const char* label, const Network& net, const NetworkResult& res, const StripOptions& so, double s_ref, const Vec3& lift_dir, double semi_span) {
	std::vector<StripLoad> strips = StripLoads(net, res, so);
	Check(static_cast<int>(strips.size()) == (so.strips_are_columns ? res.panel_cols : res.panel_rows), std::string(label) + ": one strip per span station");
	Vec3 sum;
	double width = 0.0;
	bool ordered = true;
	for (size_t k = 0; k < strips.size(); ++k) {
		sum += strips[k].force;
		width += strips[k].width;
		if (k > 0 && strips[k].s <= strips[k - 1].s) ordered = false;
	}
	const Vec3 want = res.force_coefficient * s_ref;
	// The solver halves nothing per network when mirrored? It doubles x and z
	// and zeroes y for the image -- compare the unmirrored half, so x, z / 2.
	const double err = std::fabs(sum.x - 0.5 * want.x) + std::fabs(sum.z - 0.5 * want.z);
	Check(err < 1e-9 * (1.0 + std::fabs(want.z)), F(std::string(label) + ": strip forces sum to the network force (|diff| %.2e)", err));
	Check(ordered, std::string(label) + ": strips ordered root to tip");
	Check(std::fabs(width - semi_span) < 0.02 * semi_span, F(std::string(label) + ": strip widths span the semi-span (%.3f of %.3f)", width, semi_span));
	// Lift per unit span falls to the tip and is largest inboard.
	const double l_root = Dot(strips.front().force, lift_dir) / strips.front().width;
	const double l_tip = Dot(strips.back().force, lift_dir) / strips.back().width;
	Check(l_root > 0.0 && l_tip < 0.5 * l_root, F(std::string(label) + ": lift per span %.4f at the root, %.4f at the tip", l_root, l_tip));
	// The elliptic distribution with the same total integrates back to it.
	const double total = 2.0 * Dot(sum, lift_dir); // both halves
	double integral = 0.0;
	const int n = 4000;
	for (int i = 0; i < n; ++i) {
		const double s = -semi_span + (i + 0.5) * 2.0 * semi_span / n;
		integral += EllipticLoad(total, 0.0, semi_span, s) * 2.0 * semi_span / n;
	}
	Check(std::fabs(integral / total - 1.0) < 1e-3, F(std::string(label) + ": elliptic integral %.5f of total %.5f", integral, total));
	Check(EllipticLoad(total, 0.0, semi_span, 0.0) > l_root * 0.5 && EllipticLoad(total, 0.0, semi_span, 0.0) < l_root * 2.0, "elliptic peak is of the order of the root load");
	Check(EllipticLoad(total, 0.0, semi_span, semi_span * 1.01) == 0.0, "elliptic load is zero beyond the tip");
}

} // namespace

int main() {
	setvbuf(stdout, nullptr, _IONBF, 0);
	try {
		PlanformSpec ps; // semi-span 10, root 3.1, tip 0.5, sweep 20 degrees
		WingModel wing(ps, "Wing");
		wing.AddSection(Airfoil::Naca4("0012"), 0.0);
		wing.AddSection(Airfoil::Naca4("0012"), 1.0);
		const double alpha = 4.0 * 3.14159265358979323846 / 180.0;
		const Vec3 vinf{std::cos(alpha), 0.0, std::sin(alpha)};
		const Vec3 lift{-std::sin(alpha), 0.0, std::cos(alpha)};
		Options opt;
		opt.symmetry_y = 0.0;
		opt.reference_area = wing.planform().PlanformArea();
		const double semi = wing.planform().SemiSpan();

		{
			WingPaneling thick;
			thick.chordwise = 8; thick.spanwise = 8;
			std::vector<Network> nets = WingNetworks(wing, thick, vinf);
			Solution s = Solve(nets, vinf, opt);
			StripOptions so;
			so.strips_are_columns = true;
			so.two_sided = false;
			CheckStrips("thick wing", nets[0], s.networks[0], so, opt.reference_area, lift, semi * (1.0 - thick.tip_margin));
		}
		{
			WingPaneling thin;
			thin.chordwise = 8; thin.spanwise = 8; thin.thick = false;
			std::vector<Network> nets = WingNetworks(wing, thin, vinf);
			Solution s = Solve(nets, vinf, opt);
			StripOptions so;
			so.strips_are_columns = false;
			so.two_sided = true;
			CheckStrips("mean-surface wing", nets[0], s.networks[0], so, opt.reference_area, lift, semi * (1.0 - thin.tip_margin));
		}

		// ---- colour maps ----
		const std::vector<Colormap>& maps = Colormaps();
		Check(maps.size() >= 15, F("%.0f colour maps in the catalogue", static_cast<double>(maps.size())));
		bool all_ok = true;
		for (const Colormap& m : maps) {
			if (m.stops.size() < 9) all_ok = false;
			for (double t : {-1.0, 0.0, 0.25, 0.5, 0.75, 1.0, 2.0}) {
				Rgb c = m.Sample(t);
				if (!(c.r >= 0.0f && c.r <= 1.0f && c.g >= 0.0f && c.g <= 1.0f && c.b >= 0.0f && c.b <= 1.0f)) all_ok = false;
			}
			Rgb a = m.Sample(0.0), b = m.Sample(1.0);
			if (std::fabs(a.r - b.r) + std::fabs(a.g - b.g) + std::fabs(a.b - b.b) < 0.05 && m.name != "Twilight") all_ok = false;
		}
		Check(all_ok, "every map has at least 9 stops, samples inside [0, 1] and distinct ends");
		Check(FindColormap("Jet") != nullptr && FindColormap("Viridis") != nullptr && FindColormap("nope") == nullptr, "maps found by name");
		const Rgb jet_mid = FindColormap("Jet")->Sample(0.5);
		Check(jet_mid.g > 0.95 && jet_mid.r < 0.6 && jet_mid.b < 0.6, F("jet is green at the middle (%.2f %.2f %.2f)", jet_mid.r, jet_mid.g, jet_mid.b));
		const Rgb v0 = FindColormap("Viridis")->Sample(0.0);
		Check(std::fabs(v0.r - 0.267f) < 0.01 && std::fabs(v0.b - 0.329f) < 0.01, "viridis starts at its purple");
		const Rgb cw = FindColormap("Cool-Warm")->Sample(0.5);
		Check(std::fabs(cw.r - cw.g) < 0.01 && cw.r > 0.8, "cool-warm is light grey at the middle");
		Check(DefaultColormap().name == "Turbo", "the default map is turbo");
	} catch (const std::exception& e) {
		Check(false, std::string("exception: ") + e.what());
	}
	std::printf("RESULT: %d passed, %d failed\n", g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}
