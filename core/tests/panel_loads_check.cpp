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
	// The chord tapers from the root (3.1) to the tip (0.5) of the default planform.
	Check(strips.front().chord > 2.8 && strips.front().chord < 3.1 && strips.back().chord > 0.5 && strips.back().chord < 0.8 && strips.front().chord > strips.back().chord,
		F(std::string(label) + ": strip chords taper %.3f -> %.3f", strips.front().chord, strips.back().chord));
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

	// Binned on the strips' own edges the binned loads are the strips (the
	// default planform has no dihedral, so every panel of a strip spans
	// exactly the strip's stations); with the image both halves appear and
	// the total doubles; a bin cutting a strip in two gets its share.
	{
		std::vector<double> edges;
		for (const StripLoad& st : strips) edges.push_back(st.s_min);
		edges.push_back(strips.back().s_max);
		LoadSource own{&net, &res, false};
		std::vector<Vec3> bins = BinnedLoads({own}, so.span_axis, edges);
		double diff = 0.0;
		for (size_t k = 0; k < strips.size(); ++k) diff += Length(bins[k] - strips[k].force);
		Check(diff < 1e-9 * (1.0 + std::fabs(total)), F(std::string(label) + ": binned on the strip edges reproduces the strips (|diff| %.2e)", diff));
		LoadSource image{&net, &res, true};
		const double s_tip = strips.back().s_max;
		std::vector<double> both = {-s_tip, 0.0, s_tip};
		std::vector<Vec3> halves = BinnedLoads({own, image}, so.span_axis, both);
		Check(std::fabs(Dot(halves[0], lift_dir) - Dot(halves[1], lift_dir)) < 1e-9 * (1.0 + std::fabs(total)) && std::fabs(Dot(halves[0] + halves[1], lift_dir) - total) < 1e-9 * (1.0 + std::fabs(total)),
			F(std::string(label) + ": the image half mirrors the own half, together the total (%.5f + %.5f)", Dot(halves[0], lift_dir), Dot(halves[1], lift_dir)));
		const double cut = 0.5 * (strips[3].s_min + strips[3].s_max);
		std::vector<Vec3> split = BinnedLoads({own}, so.span_axis, {strips[3].s_min, cut, strips[3].s_max});
		Check(Length(split[0] + split[1] - strips[3].force) < 1e-9 * (1.0 + std::fabs(total)) && Length(split[0] - split[1]) < 1e-9 * (1.0 + std::fabs(total)),
			std::string(label) + ": a strip cut in two bins gives each half the same share");
	}

	// Plane cuts: at a strip's centre the cut's lift per span is the strip's
	// (within the spanwise variation over the strip); integrated over fine
	// cuts the lift is the network's; a cut on a station is taken once; the
	// chord-line length of the cut is the section's perimeter; nothing
	// outside the span.
	{
		double worst = 0.0;
		for (size_t k = 1; k + 1 < strips.size(); ++k) {
			SectionCut c = CutSection(net, res, so.span_axis, strips[k].s, so.strips_are_columns);
			const double l_cut = Dot(c.force_per_span, lift_dir), l_strip = Dot(strips[k].force, lift_dir) / strips[k].width;
			worst = std::max(worst, std::fabs(l_cut - l_strip) / std::fabs(l_strip));
			if (k == 3) Check(c.found && c.length > 1.9 * strips[k].chord && c.length < 2.2 * strips[k].chord, F(std::string(label) + ": the cut's perimeter %.3f is the section's (chord %.3f)", c.length, strips[k].chord));
		}
		Check(worst < 0.03, F(std::string(label) + ": cuts at the strip centres match the strips' lift per span (worst %.2f%%)", 100.0 * worst));
		const double s_root = strips.front().s_min, s_tip = strips.back().s_max;
		const int n = 400;
		double integral = 0.0, prev = 0.0;
		for (int i = 0; i <= n; ++i) {
			const double s = s_root + (s_tip - s_root) * i / n;
			const double l = Dot(CutSection(net, res, so.span_axis, s, so.strips_are_columns).force_per_span, lift_dir);
			if (i > 0) integral += 0.5 * (l + prev) * (s_tip - s_root) / n;
			prev = l;
		}
		const double want = Dot(sum, lift_dir);
		Check(std::fabs(integral - want) < 0.02 * std::fabs(want), F(std::string(label) + ": fine cuts integrate to the network's lift (%.4f vs %.4f)", integral, want));
		SectionCut on_station = CutSection(net, res, so.span_axis, strips[2].s_max, so.strips_are_columns);
		Check(on_station.found && on_station.cp.size() == static_cast<size_t>(res.panel_rows), F(std::string(label) + ": a cut on a station is taken once (%.0f segments)", static_cast<double>(on_station.cp.size())));
		Check(!CutSection(net, res, so.span_axis, s_tip + 0.1, so.strips_are_columns).found, std::string(label) + ": no cut beyond the tip");
		// The network reflected in y = 0 as the dialog reflects the image
		// (corners, centres and normals mirrored, the column order kept, so
		// the columns run the other way along the axis): the cut at -s is
		// the mirror of the cut at s.
		{
			Network rn = net;
			for (Vec3& c : rn.corners) c.y = -c.y;
			NetworkResult rr = res;
			for (PanelResult& p : rr.panels) { p.centre.y = -p.centre.y; p.normal.y = -p.normal.y; }
			double worst = 0.0;
			for (int i = 1; i < 20; ++i) {
				const double s = s_root + (s_tip - s_root) * i / 20.0;
				const double l = Dot(CutSection(net, res, so.span_axis, s, so.strips_are_columns).force_per_span, lift_dir);
				const double lr = Dot(CutSection(rn, rr, so.span_axis, -s, so.strips_are_columns).force_per_span, lift_dir);
				worst = std::max(worst, std::fabs(l - lr));
			}
			Check(worst < 1e-9 * (1.0 + std::fabs(want)), F(std::string(label) + ": the reflected network's cuts mirror the own (worst |diff| %.2e)", worst));
		}
	}
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
			CheckStrips("thick wing", nets[0], s.networks[0], so, opt.reference_area, lift, semi * (1.0 - thick.tip_margin));
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
