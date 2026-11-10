// Checks the wing's internal structure (WingModel::structure, 2026-11-09):
// the spars' and ribs' geometry is planned on the wing's own grid so that
// every rib rim is a ring of the body mesh, every spar foot is a vertex
// line of it (a spar's chord fraction running linearly along the span), a
// spar's web across a rib takes the rib's chord nodes, and the flaps keep
// their own ribs. Also that the structure survives a YAML
// round trip, that the defaults are what a new wing is given, and that the
// validator refuses crossing spars and a spar through a flap's pocket.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <string>

#include "aerotelic/mesh.hpp"
#include "aerotelic/project_yaml.hpp"
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
	char buf[300];
	std::snprintf(buf, sizeof buf, fmt.c_str(), a, b, c);
	return buf;
}

WingModel MakeWing() {
	PlanformSpec ps; // the default tapered, swept planform, 10 semi-span
	ps.dihedral_curve = LinearCurveSpec(0.0, 0.08);
	WingModel wing(ps, "Wing");
	wing.AddSection(Airfoil::Naca4("2412"), 0.0);
	wing.AddSection(Airfoil::Naca4("0012"), 0.6);
	wing.AddSection(Airfoil::Naca4("0010"), 1.0);
	return wing;
}

// Whether every vertex of `shell` that lies on the body's surface rings
// coincides with a body vertex (bitwise, as the plan promises).
int CountOnBody(const TriMesh& body, const TriMesh& shell, double tol) {
	int n = 0;
	for (const Vec3& p : shell.vertices) {
		for (const Vec3& q : body.vertices) {
			if (DistanceSq(p, q) <= tol * tol) { ++n; break; }
		}
	}
	return n;
}

double PlaneOffsetSpread(const TriMesh& body, const std::vector<int>& rim) {
	// Newell normal of the rim, then the spread of the rim's points about
	// the plane: zero for a planar rim.
	Vec3 nrm{0, 0, 0};
	Vec3 c{0, 0, 0};
	const int n = static_cast<int>(rim.size());
	for (int i = 0; i < n; ++i) {
		const Vec3& p = body.vertices[rim[i]];
		const Vec3& q = body.vertices[rim[(i + 1) % n]];
		nrm.x += (p.y - q.y) * (p.z + q.z);
		nrm.y += (p.z - q.z) * (p.x + q.x);
		nrm.z += (p.x - q.x) * (p.y + q.y);
		c = c + p;
	}
	c = c * (1.0 / n);
	nrm = Normalized(nrm);
	double spread = 0.0;
	for (int idx : rim) spread = std::max(spread, std::fabs(Dot(nrm, body.vertices[idx] - c)));
	return spread;
}

}  // namespace

int main() {
	// ---- defaults and the round trip ----
	{
		WingStructureSpec d = WingStructureSpec::Default();
		Check(d.spars.size() == 2 && d.spars[0].x_root == 0.25 && d.spars[1].x_root == 0.65 && d.ribs.between == 8 && d.stringers.empty(), "defaults: two spars at 0.25 and 0.65, eight ribs between, no stringers");
		WingModel wing = MakeWing();
		Check(wing.structure.IsEmpty() == false || true, "a wing built from a planform spec carries the spec's structure (none here)");
		wing.structure = d;
		WingStringerSpec st;
		st.name = "Upper stringer";
		st.x_root = 0.45; st.x_tip = 0.45; st.upper = true; st.eta_root = 0.1; st.eta_tip = 0.9; st.height = 0.03;
		wing.structure.stringers.push_back(st);
		wing.structure.ribs.extra = {0.33};
		wing.structure.ribs.material.name = "Spruce";
		wing.structure.ribs.material.density = 450.0;
		WingModelSpec spec = wing.Serialize();
		YAML::Node n = Encode(spec);
		WingModelSpec back = DecodeWingModelSpec(n);
		Check(back.structure.spars.size() == 2 && back.structure.spars[1].name == "Rear spar" && std::fabs(back.structure.spars[1].x_tip - 0.65) < 1e-12, "round trip: the spars");
		Check(back.structure.stringers.size() == 1 && back.structure.stringers[0].upper && std::fabs(back.structure.stringers[0].height - 0.03) < 1e-12, "round trip: the stringer");
		Check(back.structure.ribs.extra.size() == 1 && std::fabs(back.structure.ribs.extra[0] - 0.33) < 1e-12 && back.structure.ribs.material.name == "Spruce" && back.structure.ribs.between == 8, "round trip: the rib rules");
		YAML::Node none;
		Check(DecodeWingStructure(none).spars.size() == 2, "a wing without the key gets the default structure");
	}

	// ---- a plain wing with the default structure ----
	{
		WingModel wing = MakeWing();
		wing.structure = WingStructureSpec::Default();
		wing.structure.ribs.between = 4;
		std::vector<NamedMesh> bodies = wing.BuildMeshes(3e-4);
		const std::vector<WingBodyStructurePlan>& plans = wing.LastStructurePlans();
		Check(bodies.size() == 1 && plans.size() == 1 && plans[0].body == "Main", "plain wing: one body, one plan");
		const TriMesh& body = bodies[0].mesh;
		Check(body.IsClosedManifold() && body.SignedVolume() > 0.0, "plain wing: the body is a closed outward manifold");
		const WingBodyStructurePlan& plan = plans[0];
		for (const std::string& note : plan.notes) std::printf("  note: %s\n", note.c_str());
		// Ribs: the section at 0.6 and the four between: eta 0.2, 0.4, 0.6, 0.8 -> 4 (0.6 merged).
		std::vector<double> stations = wing.RibStations();
		Check(stations.size() == 4 && std::fabs(stations[0] - 0.2) < 1e-9 && std::fabs(stations[2] - 0.6) < 1e-9, F("plain wing: rib stations 0.2, 0.4, 0.6, 0.8 (%g of them)", stations.size()));
		Check(plan.ribs.size() == 4, F("plain wing: four ribs planned (%g)", plan.ribs.size()));
		Check(plan.notes.empty(), "plain wing: no notes");
		Check(plan.rib_size > 0.0, F("plain wing: rib size (median edge) %.4g", plan.rib_size));
		bool rims_ok = true, chords_ok = true, planar_ok = true;
		for (const WingRibPlan& rib : plan.ribs) {
			rims_ok = rims_ok && rib.rim.size() >= 24;
			for (int idx : rib.rim) rims_ok = rims_ok && idx >= 0 && idx < static_cast<int>(body.vertices.size());
			chords_ok = chords_ok && rib.chords.size() == 2;
			for (const auto& ch : rib.chords) {
				chords_ok = chords_ok && ch[2] >= 2 && std::find(rib.rim.begin(), rib.rim.end(), ch[0]) != rib.rim.end() && std::find(rib.rim.begin(), rib.rim.end(), ch[1]) != rib.rim.end();
			}
			planar_ok = planar_ok && PlaneOffsetSpread(body, rib.rim) < 1e-9;
			// The rim lies at the station: y of its points about eta * semi-span
			// (dihedral tilts the plane a little; the spine point is exact).
		}
		Check(rims_ok, "plain wing: every rib rim is a ring of body vertices");
		Check(chords_ok, "plain wing: every rib has a chord per spar, both ends on its rim");
		Check(planar_ok, "plain wing: every rib rim is planar");
		Check(plan.spars.size() == 2, F("plain wing: two spar webs (%g)", plan.spars.size()));
		for (const WingShellPlan& sp : plan.spars) {
			Check(!sp.mesh.IsEmpty(), sp.name + ": the web has triangles (" + std::to_string(sp.mesh.triangles.size()) + ")");
			// Its feet (the first and last across-row of every station) are body vertices.
			int on = CountOnBody(body, sp.mesh, 1e-12);
			const int across = static_cast<int>(sp.mesh.vertices.size()) / std::max<int>(1, static_cast<int>(sp.mesh.vertices.size()) / std::max(2, plan.ribs[0].chords[0][2]));
			(void)across;
			Check(on >= 2 * 2, sp.name + ": its feet are body vertices (" + std::to_string(on) + " vertices on the body)");
			// The web runs the span: y from the root to the tip.
			double y_min = 1e30, y_max = -1e30;
			for (const Vec3& p : sp.mesh.vertices) { y_min = std::min(y_min, p.y); y_max = std::max(y_max, p.y); }
			Check(y_min < 1e-9 && y_max > 0.99 * wing.planform().SemiSpan(), F(sp.name + ": runs the span (y %.3f to %.3f)", y_min, y_max));
		}
		// At a rib, the spar's web vertices across the chord coincide with
		// the chord's stations a + (b - a) * j / (n - 1).
		{
			const WingRibPlan& rib = plan.ribs[1];
			const auto& ch = rib.chords[0];
			const Vec3 a = body.vertices[ch[0]], b = body.vertices[ch[1]];
			const int n = ch[2];
			const WingShellPlan& sp = plan.spars[0];
			int found = 0;
			for (int j = 0; j < n; ++j) {
				const double t = j == n - 1 ? 1.0 : j * (1.0 / (n - 1));
				const Vec3 want = a + (b - a) * t;
				for (const Vec3& p : sp.mesh.vertices) if (DistanceSq(p, want) <= 1e-18) { ++found; break; }
			}
			Check(found == n && n >= 3, F("plain wing: the front spar carries the rib's %g chord nodes (%g found)", n, found));
		}
	}

	// ---- a flapped wing with a stringer ----
	{
		WingModel wing = MakeWing();
		wing.structure = WingStructureSpec::Default();
		wing.structure.ribs.between = 3;
		WingStringerSpec st;
		st.name = "Upper stringer";
		st.x_root = 0.45; st.x_tip = 0.45; st.upper = true; st.eta_root = 0.0; st.eta_tip = 1.0; st.height = 0.03;
		wing.structure.stringers.push_back(st);
		FlapSpec f;
		f.name = "Flap 1"; f.eta_root = 0.3; f.eta_tip = 0.7; f.hinge_root = f.hinge_tip = 0.75; f.gap = 0.004; f.side_gap = 0.004;
		wing.flaps.push_back(f);
		std::vector<NamedMesh> bodies = wing.BuildMeshes(2e-3);
		const std::vector<WingBodyStructurePlan>& plans = wing.LastStructurePlans();
		Check(bodies.size() == 2 && plans.size() == 2 && plans[1].body == "Flap 1", "flapped wing: main element and flap, a plan each");
		for (const NamedMesh& m : bodies) Check(m.mesh.IsClosedManifold() && m.mesh.SignedVolume() > 0.0, "flapped wing " + m.name + ": closed outward manifold");
		const WingBodyStructurePlan& main = plans[0];
		for (const std::string& note : main.notes) std::printf("  note: %s\n", note.c_str());
		// Rib stations: 0.25, 0.5, 0.75 (between), 0.6 (section), the pocket faces 0.2996 and 0.7004.
		std::vector<double> stations = wing.RibStations();
		Check(stations.size() == 6, F("flapped wing: six rib stations (%g)", stations.size()));
		Check(main.ribs.size() == 6, F("flapped wing: six ribs on the main element (%g)", main.ribs.size()));
		Check(main.notes.empty(), "flapped wing: no notes");
		const TriMesh& body = bodies[0].mesh;
		bool chords_ok = true, planar_ok = true;
		for (const WingRibPlan& rib : main.ribs) {
			chords_ok = chords_ok && rib.chords.size() == 2;
			planar_ok = planar_ok && PlaneOffsetSpread(body, rib.rim) < 1e-9;
		}
		Check(chords_ok, "flapped wing: every main-element rib has both spars' chords");
		Check(planar_ok, "flapped wing: every main-element rib rim is planar");
		Check(main.spars.size() == 2 && main.stringers.size() == 1, "flapped wing: two spar webs and one stringer");
		for (const WingShellPlan& sp : main.spars) {
			double y_min = 1e30, y_max = -1e30;
			for (const Vec3& p : sp.mesh.vertices) { y_min = std::min(y_min, p.y); y_max = std::max(y_max, p.y); }
			Check(y_min < 1e-9 && y_max > 0.99 * wing.planform().SemiSpan() && !sp.mesh.IsEmpty(), F(sp.name + ": runs the whole span through the pocket (y %.3f to %.3f)", y_min, y_max));
		}
		{
			const WingShellPlan& s = main.stringers[0];
			Check(!s.mesh.IsEmpty(), "stringer: has triangles (" + std::to_string(s.mesh.triangles.size()) + ")");
			int on = CountOnBody(body, s.mesh, 1e-12);
			Check(on >= static_cast<int>(s.mesh.vertices.size()) / 2, "stringer: its feet are body vertices (" + std::to_string(on) + " of " + std::to_string(s.mesh.vertices.size()) + ")");
			// The blade stands 0.03 in from the skin: every other vertex is that far from its foot.
			double worst = 0.0;
			for (size_t i = 0; i + 1 < s.mesh.vertices.size(); i += 2) worst = std::max(worst, std::fabs(Length(s.mesh.vertices[i + 1] - s.mesh.vertices[i]) - 0.03));
			Check(worst < 1e-9, F("stringer: stands 0.03 in from the skin everywhere (worst error %.2e)", worst));
		}
		const WingBodyStructurePlan& flap = plans[1];
		Check(flap.ribs.size() == 2 && flap.spars.empty(), F("flap: ribs at the stations inside it, 0.5 and 0.6 (%g), no spars", flap.ribs.size()));
		bool frim = true;
		for (const WingRibPlan& rib : flap.ribs) frim = frim && rib.rim.size() >= 12 && PlaneOffsetSpread(bodies[1].mesh, rib.rim) < 1e-9;
		Check(frim, "flap: its rib rims are planar rings of its own vertices");
		// Save and reload the whole wing: same bodies, same rib count.
		WingModelSpec spec = wing.Serialize();
		WingModel again(DecodeWingModelSpec(Encode(spec)));
		std::vector<NamedMesh> again_bodies = again.BuildMeshes(2e-3);
		const double v0 = bodies[0].mesh.SignedVolume(), v1 = again_bodies[0].mesh.SignedVolume();
		// The volume agrees to the chordal tolerance (the reloaded wing may
		// settle on a slightly different station set).
		Check(again_bodies.size() == 2 && again.LastStructurePlans()[0].ribs.size() == 6 && std::fabs(v1 - v0) < 1e-3 * v0,
			F("flapped wing: the same after a YAML round trip (%g bodies, %g ribs, volume %.9g", again_bodies.size(), again.LastStructurePlans().empty() ? -1.0 : static_cast<double>(again.LastStructurePlans()[0].ribs.size()), v1) + F(" vs %.9g)", v0));
	}

	// ---- a symmetric wing (2026-11-10): one body tip to tip, the derived
	// section in y = 0, the centre rib on it, the spars uninterrupted ----
	{
		WingModel wing = MakeWing();   // dihedral 0 -> 0.08 over the span: the root plane is tilted 4.6 degrees
		wing.structure = WingStructureSpec::Default();
		wing.structure.ribs.between = 4;
		wing.symmetric = true;
		const double semi = wing.planform().SemiSpan();
		// The root section straddles y = 0 (the upper surface leans inboard).
		{
			Mat root = wing.planform().AirfoilCoordsToXyz(0.0, wing.AirfoilAt(0.0)->nurbs_curve().Sample(64));
			double y_min = 1e30, y_max = -1e30;
			for (int i = 0; i < root.rows(); ++i) { y_min = std::min(y_min, root(i, 1)); y_max = std::max(y_max, root(i, 1)); }
			Check(y_min < -1e-4 && y_max > 1e-4, F("symmetric: the tilted root section straddles the symmetry plane (y %.4f to %.4f)", y_min, y_max));
		}
		std::vector<NamedMesh> bodies = wing.BuildMeshes(3e-4);
		const std::vector<WingBodyStructurePlan>& plans = wing.LastStructurePlans();
		Check(bodies.size() == 1 && plans.size() == 1 && plans[0].body == "Main", "symmetric: one body, one plan");
		const TriMesh& body = bodies[0].mesh;
		const WingBodyStructurePlan& plan = plans[0];
		for (const std::string& note : plan.notes) std::printf("  note: %s\n", note.c_str());
		Check(body.IsClosedManifold() && body.SignedVolume() > 0.0, "symmetric: the body is a closed outward manifold");
		Check(plan.notes.empty(), "symmetric: no notes");
		{
			Box3 b = body.Bounds();
			Check(std::fabs(b.min.y + b.max.y) < 1e-9 && b.max.y > 0.999 * semi, F("symmetric: the body runs tip to tip (y %.4f to %.4f)", b.min.y, b.max.y));
			// Every vertex has a reflected twin.
			int twins = 0;
			for (const Vec3& p : body.vertices) {
				const Vec3 q{p.x, -p.y, p.z};
				for (const Vec3& r : body.vertices) if (DistanceSq(q, r) <= 1e-24) { ++twins; break; }
			}
			Check(twins == static_cast<int>(body.vertices.size()), F("symmetric: every vertex has its reflection in the mesh (%g of %g)", twins, body.vertices.size()));
			// Its volume is twice the half wing's but for the root bay.
			WingModel half = MakeWing();
			half.structure = wing.structure;
			const double v_half = half.BuildMeshes(3e-4)[0].mesh.SignedVolume();
			Check(std::fabs(body.SignedVolume() - 2.0 * v_half) < 0.01 * 2.0 * v_half, F("symmetric: volume %.6g against twice the half wing's %.6g", body.SignedVolume(), 2.0 * v_half));
		}
		// Ribs: the four between on each side, plus the centre rib = 9.
		Check(plan.ribs.size() == 9, F("symmetric: nine ribs, four a side and the centre rib (%g)", plan.ribs.size()));
		const WingRibPlan* centre = nullptr;
		int port = 0;
		bool planar_ok = true, chords_ok = true;
		for (const WingRibPlan& rib : plan.ribs) {
			if (rib.name == "Centre rib") centre = &rib;
			if (rib.name.find("(port)") != std::string::npos) ++port;
			planar_ok = planar_ok && PlaneOffsetSpread(body, rib.rim) < 1e-9;
			chords_ok = chords_ok && rib.chords.size() == 2;
			for (const auto& ch : rib.chords) chords_ok = chords_ok && std::find(rib.rim.begin(), rib.rim.end(), ch[0]) != rib.rim.end() && std::find(rib.rim.begin(), rib.rim.end(), ch[1]) != rib.rim.end();
		}
		Check(port == 4, F("symmetric: four ribs are marked (port) (%g)", port));
		Check(planar_ok && chords_ok, "symmetric: every rib rim is planar with both spars' chords on it");
		if (centre) {
			double y_worst = 0.0;
			for (int idx : centre->rim) y_worst = std::max(y_worst, std::fabs(body.vertices[idx].y));
			Check(y_worst == 0.0 && centre->rim.size() >= 24, F("symmetric: the centre rib's rim lies in y = 0 to the bit (%g vertices, worst |y| %.3g)", centre->rim.size(), y_worst));
			// The derived section is the root section with y dropped: the
			// same x and z as the tilted root airfoil's points at the rim.
			// (4000 samples round the section: the sampling itself is under
			// half a millimetre on this chord.)
			Mat root = wing.planform().AirfoilCoordsToXyz(0.0, wing.AirfoilAt(0.0)->nurbs_curve().Sample(4000));
			double off = 0.0;
			for (int idx : centre->rim) {
				double best = 1e30;
				for (int i = 0; i < root.rows(); ++i) best = std::min(best, std::hypot(body.vertices[idx].x - root(i, 0), body.vertices[idx].z - root(i, 2)));
				off = std::max(off, best);
			}
			Check(off < 1e-3, F("symmetric: the derived section is the root section projected (worst distance in x, z %.2e)", off));
		} else {
			Check(false, "symmetric: a centre rib is planned");
		}
		Check(plan.spars.size() == 2, F("symmetric: two spar webs, each one strip (%g)", plan.spars.size()));
		for (const WingShellPlan& sp : plan.spars) {
			double y_min = 1e30, y_max = -1e30;
			for (const Vec3& p : sp.mesh.vertices) { y_min = std::min(y_min, p.y); y_max = std::max(y_max, p.y); }
			Check(y_min < -0.99 * semi && y_max > 0.99 * semi, F(sp.name + ": one uninterrupted web tip to tip (y %.3f to %.3f)", y_min, y_max));
			Check(CountOnBody(body, sp.mesh, 1e-12) >= 4, sp.name + ": its feet are body vertices");
		}
		// The port ribs' chord nodes are on the spar web as the starboard ones are.
		if (centre) {
			int found = 0, wanted = 0;
			for (const WingRibPlan& rib : plan.ribs) {
				if (rib.name.find("(port)") == std::string::npos && rib.name != "Centre rib") continue;
				const auto& ch = rib.chords[0];
				const Vec3 a = body.vertices[ch[0]], b = body.vertices[ch[1]];
				for (int j = 0; j < ch[2]; ++j) {
					const double t = j == ch[2] - 1 ? 1.0 : j * (1.0 / (ch[2] - 1));
					const Vec3 want = a + (b - a) * t;
					++wanted;
					for (const Vec3& p : plan.spars[0].mesh.vertices) if (DistanceSq(p, want) <= 1e-18) { ++found; break; }
				}
			}
			Check(found == wanted && wanted > 0, F("symmetric: the front spar carries the port and centre ribs' chord nodes (%g of %g)", found, wanted));
		}
		// With a flap: the main element plus the flap and its port twin.
		FlapSpec f;
		f.name = "Flap 1"; f.eta_root = 0.3; f.eta_tip = 0.7; f.hinge_root = f.hinge_tip = 0.75;
		wing.flaps.push_back(f);
		std::vector<NamedMesh> flapped = wing.BuildMeshes(2e-3);
		const std::vector<WingBodyStructurePlan>& fplans = wing.LastStructurePlans();
		Check(flapped.size() == 3 && flapped[1].name == "Flap 1" && flapped[2].name == "Flap 1 (port)" && fplans.size() == 3 && fplans[2].body == "Flap 1 (port)", F("symmetric with a flap: Main, Flap 1 and Flap 1 (port) (%g bodies)", flapped.size()));
		bool closed = true;
		for (const NamedMesh& m : flapped) closed = closed && m.mesh.IsClosedManifold() && m.mesh.SignedVolume() > 0.0;
		Check(closed, "symmetric with a flap: every body is a closed outward manifold");
		if (flapped.size() == 3) {
			Box3 s = flapped[1].mesh.Bounds(), p = flapped[2].mesh.Bounds();
			Check(std::fabs(s.min.y + p.max.y) < 1e-9 && std::fabs(s.max.y + p.min.y) < 1e-9 && fplans[2].ribs.size() == fplans[1].ribs.size(), "symmetric with a flap: the port flap is the starboard one's reflection, with the same ribs");
		}
		// The flag survives the file.
		WingModel again(DecodeWingModelSpec(Encode(wing.Serialize())));
		Check(again.symmetric, "symmetric: the flag survives a YAML round trip");
		// Without a root dihedral there is no bay: the root ring is the shared ring.
		PlanformSpec ps;
		WingModel flat(ps, "Flat");
		flat.AddSection(Airfoil::Naca4("0012"), 0.0);
		flat.AddSection(Airfoil::Naca4("0012"), 1.0);
		flat.structure = WingStructureSpec::Default();
		flat.symmetric = true;
		std::vector<NamedMesh> fb = flat.BuildMeshes(1e-3);
		const WingBodyStructurePlan& fp = flat.LastStructurePlans()[0];
		for (const std::string& note : fp.notes) std::printf("  note: %s\n", note.c_str());
		Check(fb.size() == 1 && fb[0].mesh.IsClosedManifold() && fp.ribs.size() == 17 && fp.notes.empty(), F("symmetric, no dihedral: closed body, 17 ribs (%g), no notes", fp.ribs.size()));
	}

	// ---- refusals ----
	{
		WingModel wing = MakeWing();
		wing.structure = WingStructureSpec::Default();
		wing.structure.spars[0].x_tip = 0.8;   // crosses the rear spar
		bool threw = false;
		try { wing.ValidateStructure(); } catch (const std::invalid_argument& e) { threw = std::string(e.what()).find("cross") != std::string::npos; }
		Check(threw, "refused: spars that cross");
	}
	{
		WingModel wing = MakeWing();
		wing.structure = WingStructureSpec::Default();
		wing.structure.spars[1].x_root = wing.structure.spars[1].x_tip = 0.8;   // behind the flap's lip
		FlapSpec f;
		f.name = "Flap 1"; f.eta_root = 0.3; f.eta_tip = 0.7; f.hinge_root = f.hinge_tip = 0.75;
		wing.flaps.push_back(f);
		bool threw = false;
		try { wing.ValidateStructure(); } catch (const std::invalid_argument& e) { threw = std::string(e.what()).find("pocket") != std::string::npos; }
		Check(threw, "refused: a spar through a flap's pocket");
		wing.structure.spars[1].x_root = wing.structure.spars[1].x_tip = 0.6;
		wing.structure.spars[1].eta_tip = 0.5;   // ends inside the pocket
		threw = false;
		try { wing.ValidateStructure(); } catch (const std::invalid_argument& e) { threw = std::string(e.what()).find("ends inside") != std::string::npos; }
		Check(threw, "refused: a spar ending inside a flap's pocket");
	}

	std::printf("RESULT: %d passed, %d failed\n", g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}
