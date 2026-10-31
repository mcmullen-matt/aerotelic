// Checks a wing with trailing-edge flaps (WingModel::flaps): the main
// element and each flap come out as closed, outward manifolds, the flap
// sits clear of the main element by about the gap, the bodies together
// hold a little less than the plain wing, flaps at the root and tip and
// two flaps on one wing work, bad flap sets are refused, and the flaps
// survive a YAML round trip.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

#include "aerotelic/mesh.hpp"
#include "aerotelic/project_yaml.hpp"
#include "aerotelic/sdf.hpp"
#include "aerotelic/wing_model.hpp"

using namespace aerotelic;
using namespace aerotelic::project_yaml;

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

WingModel MakeWing() {
	PlanformSpec ps; // the default tapered, swept planform
	WingModel wing(ps, "Wing");
	wing.AddSection(Airfoil::Naca4("2412"), 0.0);
	wing.AddSection(Airfoil::Naca4("0012"), 1.0);
	return wing;
}

void CheckBodies(const std::string& label, const WingModel& wing, double tol, double v_full, size_t n_flaps) {
	auto t0 = std::chrono::steady_clock::now();
	std::vector<NamedMesh> meshes = wing.BuildMeshes(tol);
	double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
	std::printf("%s: %zu bodies in %.3f s\n", label.c_str(), meshes.size(), secs);
	Check(meshes.size() == n_flaps + 1 && meshes[0].name == "Main", label + ": main element plus one body per flap");
	double sum = 0.0;
	for (const NamedMesh& m : meshes) {
		Check(!m.mesh.IsEmpty(), label + " " + m.name + ": has triangles (" + std::to_string(m.mesh.triangles.size()) + ")");
		Check(m.mesh.IsClosedManifold(), label + " " + m.name + ": closed manifold");
		double v = m.mesh.SignedVolume();
		Check(v > 0.0, label + " " + m.name + ": outward (volume " + std::to_string(v) + ")");
		sum += v;
	}
	Check(sum < v_full && sum > 0.97 * v_full, F(label + ": bodies hold a little less than the plain wing (%.5f vs %.5f)", sum, v_full));
	// Each flap clears the main element: no flap vertex is inside it; across
	// the flap's span the nearest approach is about the (chordwise) gap,
	// within the chordal tolerance of both bodies; at its end caps it is the
	// side gap, exactly (both faces are planar) -- or, with no side gap, the
	// caps lie in the pocket faces and touch.
	if (meshes.size() > 1) {
		MeshSdf main_sdf(meshes[0].mesh);
		std::vector<FlapSpec> sorted = wing.flaps;
		std::sort(sorted.begin(), sorted.end(), [](const FlapSpec& a, const FlapSpec& b) { return a.eta_root < b.eta_root; });
		const double semi = wing.planform().SemiSpan();
		for (size_t k = 1; k < meshes.size(); ++k) {
			const FlapSpec& f = sorted[k - 1];
			double nearest_mid = 1e30, nearest_cap = 1e30, deepest = 0.0, y_min = 1e30, y_max = -1e30;
			for (const Vec3& p : meshes[k].mesh.vertices) {
				double d = main_sdf.Distance(p);
				deepest = std::min(deepest, d);
				y_min = std::min(y_min, p.y);
				y_max = std::max(y_max, p.y);
				bool at_cap = p.y < f.eta_root * semi + 1e-6 || p.y > f.eta_tip * semi - 1e-6;
				(at_cap ? nearest_cap : nearest_mid) = std::min(at_cap ? nearest_cap : nearest_mid, d);
			}
			Check(deepest > -1e-9, F(label + " " + meshes[k].name + ": no vertex inside the main element (deepest %.3g)", deepest));
			Check(nearest_mid > f.gap - 2.5 * tol && nearest_mid < f.gap + 2.5 * tol + 1e-9, F(label + " " + meshes[k].name + ": nearest approach across the span %.4f vs gap %.4f", nearest_mid, f.gap));
			// The flap keeps its own extent; the pocket is wider.
			Check(std::fabs(y_min - f.eta_root * semi) < 1e-6 && std::fabs(y_max - f.eta_tip * semi) < 1e-6, F(label + " " + meshes[k].name + ": spans its own extent (y %.4f to %.4f)", y_min, y_max));
			// The end caps: no closer than the smaller of the two clearances
			// (a cap vertex on the nose arc is a gap from the cavity, the cap
			// itself a side gap from the pocket face).
			const double least = std::min(f.gap, f.side_gap);
			Check(nearest_cap > least - 2.5 * tol - 1e-9, F(label + " " + meshes[k].name + ": end caps no closer than the smaller clearance (%.4f vs %.4f)", nearest_cap, least));
			// The pocket runs from eta_root - side_gap to eta_tip + side_gap:
			// its faces (which carry the aft skin) sit exactly there, and
			// between them the main element has nothing aft of the hinge.
			const auto [lo, hi] = wing.PocketExtent(f);
			auto hinge_x = [&](double eta) { return wing.planform().AirfoilCoordsToXyz(eta, Mat::FromRows({{wing.HingeFraction(f, eta), 0.0}}))(0, 0); };
			bool face_lo = false, face_hi = false, band_clear = true;
			for (const Vec3& p : meshes[0].mesh.vertices) {
				const double eta = p.y / semi;
				if (eta < lo - 1e-9 || eta > hi + 1e-9) continue;
				const bool aft = p.x > hinge_x(eta) + 1e-6;
				if (std::fabs(eta - lo) < 1e-9) { face_lo = face_lo || aft; continue; }
				if (std::fabs(eta - hi) < 1e-9) { face_hi = face_hi || aft; continue; }
				band_clear = band_clear && !aft;
			}
			Check(band_clear, label + " " + meshes[k].name + ": nothing of the main element aft of the hinge inside the pocket");
			Check((lo <= 1e-9 || face_lo) && (hi >= 1.0 - 1e-9 || face_hi), F(label + " " + meshes[k].name + ": pocket faces at eta %.4f and %.4f carry the aft skin", lo, hi));
		}
	}
}

} // namespace

int main() {
	setvbuf(stdout, nullptr, _IONBF, 0);
	try {
		const double tol = 1e-3;
		WingModel wing = MakeWing();
		double v_full = wing.BuildMesh(tol).SignedVolume();
		Check(wing.BuildMeshes(tol).size() == 1, "no flaps: one body");

		// One outboard flap, with and without a side gap.
		FlapSpec f;
		f.name = "Aileron";
		f.eta_root = 0.55; f.eta_tip = 0.95;
		f.hinge_root = 0.70; f.hinge_tip = 0.70;
		f.gap = 0.004;
		f.side_gap = 0.012;
		wing.flaps = {f};
		CheckBodies("one flap", wing, tol, v_full, 1);
		{
			auto [lo, hi] = wing.PocketExtent(f);
			double semi = wing.planform().SemiSpan();
			Check(std::fabs(lo - (f.eta_root - f.side_gap / semi)) < 1e-12 && std::fabs(hi - (f.eta_tip + f.side_gap / semi)) < 1e-12, "pocket extent: the flap's span widened by the side gap at each end");
		}
		FlapSpec touching = f;
		touching.side_gap = 0.0;
		wing.flaps = {touching};
		CheckBodies("one flap, no side gap", wing, tol, v_full, 1);
		wing.flaps = {f};
		Check(wing.flaps[0].name == "Aileron" && wing.BuildMeshes(tol)[1].name == "Aileron", "one flap: the flap body carries the flap's name");
		// The hinge line is straight in plan view: the hinge points of
		// stations across the flap are collinear in (x, y).
		{
			FlapSpec g = f;
			g.hinge_root = 0.74; g.hinge_tip = 0.66;
			Check(std::fabs(wing.HingeFraction(g, g.eta_root) - 0.74) < 1e-9 && std::fabs(wing.HingeFraction(g, g.eta_tip) - 0.66) < 1e-9, "hinge fraction: exact at the flap's ends");
			auto plan = [&](double eta) {
				Mat p = wing.planform().AirfoilCoordsToXyz(eta, Mat::FromRows({{wing.HingeFraction(g, eta), 0.0}}));
				return std::array<double, 2>{p(0, 0), p(0, 1)};
			};
			auto r = plan(g.eta_root), t = plan(g.eta_tip);
			double worst = 0.0;
			for (double eta : {0.6, 0.7, 0.75, 0.85, 0.9}) {
				auto p = plan(eta);
				double cross = (t[0] - r[0]) * (p[1] - r[1]) - (t[1] - r[1]) * (p[0] - r[0]);
				worst = std::max(worst, std::fabs(cross) / std::hypot(t[0] - r[0], t[1] - r[1]));
			}
			Check(worst < 1e-9, F("hinge line: straight in plan view (worst offset %.3g)", worst));
			// On this tapered wing a straight plan line is not a linear
			// interpolation of the chord fraction in eta.
			double h_mid = wing.HingeFraction(g, 0.75);
			Check(std::fabs(h_mid - 0.70) > 1e-4, F("hinge fraction: mid-flap %.4f, not the eta-interpolated 0.70", h_mid));
		}

		// A flap over the whole span: the main element is notched from root
		// to tip, no full stretch at all.
		FlapSpec whole = f;
		whole.name = "Full-span flap";
		whole.eta_root = 0.0; whole.eta_tip = 1.0;
		wing.flaps = {whole};
		CheckBodies("full-span flap", wing, tol, v_full, 1);

		// Two flaps with plain wing between them, one reaching the root.
		FlapSpec inner = f, outer = f;
		inner.name = "Flap"; inner.eta_root = 0.0; inner.eta_tip = 0.45; inner.hinge_root = 0.72; inner.hinge_tip = 0.68;
		outer.name = "Aileron"; outer.eta_root = 0.55; outer.eta_tip = 0.92;
		wing.flaps = {outer, inner}; // out of order on purpose
		CheckBodies("two flaps", wing, tol, v_full, 2);
		{
			std::vector<NamedMesh> m = wing.BuildMeshes(tol);
			Check(m[1].name == "Flap" && m[2].name == "Aileron", "two flaps: bodies ordered root to tip");
		}

		// ---- refusals ----
		auto refuses = [&](const char* what, std::vector<FlapSpec> flaps) {
			wing.flaps = std::move(flaps);
			bool threw = false;
			std::string msg;
			try { wing.BuildMeshes(tol); } catch (const std::invalid_argument& e) { threw = true; msg = e.what(); }
			Check(threw, std::string("refuses ") + what + (threw ? " (" + msg + ")" : ""));
		};
		FlapSpec a = f, b = f;
		b.eta_root = 0.80; b.eta_tip = 0.99;
		refuses("overlapping flaps", {a, b});
		b.eta_root = a.eta_tip;
		refuses("flaps sharing an edge station", {a, b});
		b.eta_root = a.eta_tip + 0.5 * (a.side_gap + b.side_gap) / wing.planform().SemiSpan();
		refuses("flaps whose pockets overlap once the side gaps are allowed for", {a, b});
		a = f; a.side_gap = -0.001;
		refuses("a negative side gap", {a});
		a.eta_tip = a.eta_root;
		refuses("a flap of zero span", {a});
		a = f; a.hinge_root = 0.0;
		refuses("a hinge at the leading edge", {a});
		a = f; a.gap = 3.0; // about the root chord: the cavity encloses the leading edge
		refuses("a gap the section cannot hold", {a});
		a = f; a.gap = -0.001;
		refuses("a negative gap", {a});

		// ---- YAML round trip ----
		wing.flaps = {inner, outer};
		WingModelSpec spec = wing.Serialize();
		YAML::Node node = Encode(spec);
		WingModelSpec back = DecodeWingModelSpec(node);
		bool same = back.flaps.size() == 2;
		for (size_t i = 0; same && i < 2; ++i) {
			const FlapSpec& p = spec.flaps[i];
			const FlapSpec& q = back.flaps[i];
			same = p.name == q.name && p.eta_root == q.eta_root && p.eta_tip == q.eta_tip && p.hinge_root == q.hinge_root && p.hinge_tip == q.hinge_tip && p.gap == q.gap && p.side_gap == q.side_gap;
		}
		Check(same, "yaml: flaps survive a round trip");
		wing.flaps.clear();
		Check(!Encode(wing.Serialize())["flaps"], "yaml: a wing without flaps writes no flaps key");
		WingModel again(back);
		Check(again.flaps.size() == 2 && again.BuildMeshes(tol).size() == 3, "yaml: a wing rebuilt from the round-tripped spec has its flaps");
	} catch (const std::exception& e) {
		std::printf("FAIL: exception: %s\n", e.what());
		++g_fail;
	}
	std::printf("\nRESULT: %d passed, %d failed\n", g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}
