// Checks the frames and stringers of the bodies lofted along their length
// (structure_plan.hpp FrameStructureSpec, 2026-11-10): a fuselage's frames
// are planar rings of its own mesh at the stations asked for, bands of
// the given height; its longerons stand the given height in from the skin
// on the mesh's own vertices; a nacelle's frames are annuli (the inner
// skin's ring as a hole); chevrons leave no plan; the spec survives a YAML
// round trip and a bad one keeps the skin.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "aerotelic/fuselage_model.hpp"
#include "aerotelic/lofted_nacelle_model.hpp"
#include "aerotelic/nacelle_model.hpp"
#include "aerotelic/project_yaml.hpp"

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

double PlaneSpread(const TriMesh& body, const std::vector<int>& rim, double& x_mean) {
	double x_min = 1e30, x_max = -1e30;
	x_mean = 0.0;
	for (int idx : rim) { x_min = std::min(x_min, body.vertices[idx].x); x_max = std::max(x_max, body.vertices[idx].x); x_mean += body.vertices[idx].x; }
	x_mean /= std::max<size_t>(rim.size(), 1);
	return x_max - x_min;
}

int CountOnBody(const TriMesh& body, const TriMesh& shell) {
	int n = 0;
	for (const Vec3& p : shell.vertices) {
		for (const Vec3& q : body.vertices) {
			if (DistanceSq(p, q) <= 1e-24) { ++n; break; }
		}
	}
	return n;
}

}  // namespace

int main() {
	// ---- the default fuselage ----
	{
		FuselageSpec fs;
		FuselageModel fuselage(fs);
		Check(fuselage.structure.frames.between == 8 && fuselage.structure.stringers.size() == 4 && fuselage.structure.stringers[0].name == "Longeron 1", "defaults: eight frames, four longerons");
		std::vector<NamedMesh> bodies = fuselage.BuildMeshes(2e-3);
		const std::vector<BodyStructurePlan>& plans = fuselage.LastStructurePlans();
		Check(bodies.size() == 1 && plans.size() == 1 && plans[0].body == "Main" && plans[0].rib_layer == "Frames", "fuselage: one body, one plan, its ribs' layer called Frames");
		const TriMesh& body = bodies[0].mesh;
		Check(body.IsClosedManifold() && body.SignedVolume() > 0.0, "fuselage: the body is a closed outward manifold");
		const BodyStructurePlan& plan = plans[0];
		for (const std::string& note : plan.notes) std::printf("  note: %s\n", note.c_str());
		Check(plan.notes.empty(), "fuselage: no notes");
		Check(plan.ribs.size() == 8, F("fuselage: eight frames planned (%g)", plan.ribs.size()));
		bool rims_ok = true, planar_ok = true, stations_ok = true, height_ok = true;
		for (size_t k = 0; k < plan.ribs.size(); ++k) {
			const RibPlan& rib = plan.ribs[k];
			rims_ok = rims_ok && rib.rim.size() >= 16 && rib.holes.empty();
			for (int idx : rib.rim) rims_ok = rims_ok && idx >= 0 && idx < static_cast<int>(body.vertices.size());
			double x_mean = 0.0;
			planar_ok = planar_ok && PlaneSpread(body, rib.rim, x_mean) < 1e-9;
			stations_ok = stations_ok && std::fabs(x_mean - (k + 1) / 9.0 * fuselage.length) < 1e-9 && std::fabs(rib.eta - (k + 1) / 9.0) < 1e-9;
			height_ok = height_ok && rib.height == 0.1;
		}
		Check(rims_ok, "fuselage: every frame's rim is a ring of body vertices, no holes");
		Check(planar_ok, "fuselage: every frame lies in a plane x = const");
		Check(stations_ok, "fuselage: the frames sit at x/L = 1/9 .. 8/9");
		Check(height_ok, "fuselage: the frames are bands 0.1 in from the skin");
		Check(plan.spars.empty() && plan.stringers.size() == 4, F("fuselage: no spars, four longerons (%g)", plan.stringers.size()));
		for (const ShellPlan& st : plan.stringers) {
			Check(!st.mesh.IsEmpty(), st.name + ": has triangles (" + std::to_string(st.mesh.triangles.size()) + ")");
			const int on = CountOnBody(body, st.mesh);
			Check(on >= static_cast<int>(st.mesh.vertices.size()) / 2 && on >= 10, st.name + ": its feet are body vertices (" + std::to_string(on) + " of " + std::to_string(st.mesh.vertices.size()) + ")");
			double worst = 0.0, x_min = 1e30, x_max = -1e30;
			for (size_t i = 0; i + 1 < st.mesh.vertices.size(); i += 2) {
				worst = std::max(worst, std::fabs(Length(st.mesh.vertices[i + 1] - st.mesh.vertices[i]) - 0.03));
				x_min = std::min(x_min, st.mesh.vertices[i].x);
				x_max = std::max(x_max, st.mesh.vertices[i].x);
			}
			Check(worst < 1e-9, F(st.name + ": stands 0.03 in from the skin everywhere (worst error %.2e)", worst));
			Check(x_min < 0.1 * fuselage.length && x_max > 0.9 * fuselage.length, F(st.name + ": runs the length (x %.3f to %.3f)", x_min, x_max));
		}
		// The longerons stand INSIDE: their inner points are nearer the
		// section's centre than their feet.
		{
			const ShellPlan& st = plan.stringers[0];
			int inside = 0, total = 0;
			for (size_t i = 0; i + 1 < st.mesh.vertices.size(); i += 2) {
				const Vec3& foot = st.mesh.vertices[i];
				const Vec3& in = st.mesh.vertices[i + 1];
				const double rf = std::hypot(foot.y, foot.z - 0.5 * (fuselage.ProfileZ(foot.x / fuselage.length).first + fuselage.ProfileZ(foot.x / fuselage.length).second));
				const double ri = std::hypot(in.y, in.z - 0.5 * (fuselage.ProfileZ(in.x / fuselage.length).first + fuselage.ProfileZ(in.x / fuselage.length).second));
				++total;
				if (ri < rf) ++inside;
			}
			Check(inside == total && total > 0, F("fuselage: the longeron's blade points inward at every station (%g of %g)", inside, total));
		}
		// A frame station typed in, and the rules edited.
		fuselage.structure.frames.between = 2;
		fuselage.structure.frames.extra = {0.42};
		fuselage.structure.frames.height = 0.0;
		fuselage.structure.stringers.clear();
		std::vector<NamedMesh> again = fuselage.BuildMeshes(2e-3);
		const BodyStructurePlan& p2 = fuselage.LastStructurePlans()[0];
		Check(again.size() == 1 && again[0].mesh.IsClosedManifold() && p2.ribs.size() == 3 && std::fabs(p2.ribs[1].eta - 0.42) < 1e-9 && p2.ribs[1].height == 0.0 && p2.stringers.empty(),
			F("fuselage: two frames between plus one at 0.42, whole bulkheads, no longerons (%g frames)", p2.ribs.size()));
		// The spec survives the file.
		fuselage.structure.frames.material.name = "Spruce";
		fuselage.structure.frames.material.density = 450.0;
		FrameStringerSpec st;
		st.name = "Keel";
		st.around = 0.5;
		st.x_start = 0.1;
		st.x_end = 0.8;
		st.height = 0.05;
		fuselage.structure.stringers.push_back(st);
		FuselageSpec spec = fuselage.Serialize();
		FuselageSpec back = DecodeFuselageSpec(Encode(spec));
		Check(back.structure.frames.between == 2 && back.structure.frames.extra.size() == 1 && back.structure.frames.material.name == "Spruce" && back.structure.stringers.size() == 1
			&& back.structure.stringers[0].name == "Keel" && back.structure.stringers[0].around == 0.5 && back.structure.stringers[0].x_end == 0.8,
			"fuselage: the structure survives a YAML round trip");
		YAML::Node without = Encode(spec);
		without.remove("structure");
		Check(DecodeFuselageSpec(without).structure.frames.between == 8 && DecodeFuselageSpec(without).structure.stringers.size() == 4, "fuselage: a file without the key gets the defaults");
		// The keel runs 0.1 to 0.8 of the length and no further.
		FuselageModel keeled(back);
		keeled.BuildMeshes(2e-3);
		const BodyStructurePlan& p3 = keeled.LastStructurePlans()[0];
		for (const std::string& note : p3.notes) std::printf("  note: %s\n", note.c_str());
		double x_min = 1e30, x_max = -1e30;
		if (!p3.stringers.empty()) for (const Vec3& v : p3.stringers[0].mesh.vertices) { x_min = std::min(x_min, v.x); x_max = std::max(x_max, v.x); }
		Check(p3.stringers.size() == 1 && std::fabs(x_min - 0.1 * keeled.length) < 1e-9 && std::fabs(x_max - 0.8 * keeled.length) < 1e-9, F("fuselage: the keel runs x/L 0.1 to 0.8 exactly (x %.4f to %.4f)", x_min, x_max));
		// A bad spec keeps the skin and says so.
		keeled.structure.stringers[0].around = 1.5;
		std::vector<NamedMesh> bad = keeled.BuildMeshes(2e-3);
		const BodyStructurePlan& p4 = keeled.LastStructurePlans()[0];
		Check(bad.size() == 1 && bad[0].mesh.IsClosedManifold() && p4.ribs.empty() && p4.stringers.empty() && !p4.notes.empty() && p4.notes[0].find("not built") != std::string::npos,
			"fuselage: a bad structure keeps the skin and leaves the reason in the notes");
	}

	// ---- the lofted nacelle: frames are annuli ----
	{
		LoftedNacelleSpec ns;
		LoftedNacelleModel nacelle(ns);
		nacelle.structure.frames.between = 3;
		nacelle.structure.frames.height = 0.0;
		std::vector<NamedMesh> bodies = nacelle.BuildMeshes(2e-3);
		const std::vector<BodyStructurePlan>& plans = nacelle.LastStructurePlans();
		Check(bodies.size() == 1 && plans.size() == 1 && plans[0].rib_layer == "Frames", "lofted nacelle: one body, one plan of frames");
		const TriMesh& body = bodies[0].mesh;
		Check(body.IsClosedManifold() && body.SignedVolume() > 0.0, "lofted nacelle: the body is a closed outward manifold");
		const BodyStructurePlan& plan = plans[0];
		for (const std::string& note : plan.notes) std::printf("  note: %s\n", note.c_str());
		Check(plan.notes.empty(), "lofted nacelle: no notes");
		Check(plan.ribs.size() == 3, F("lofted nacelle: three frames (%g)", plan.ribs.size()));
		bool annuli = true, planar = true;
		for (const RibPlan& rib : plan.ribs) {
			annuli = annuli && rib.holes.size() == 1 && rib.holes[0].size() >= 16 && rib.rim.size() >= 16;
			double xo = 0.0, xi = 0.0;
			planar = planar && PlaneSpread(body, rib.rim, xo) < 1e-9 && (rib.holes.empty() || PlaneSpread(body, rib.holes[0], xi) < 1e-9) && std::fabs(xo - xi) < 1e-9 && std::fabs(xo - rib.eta * nacelle.length) < 1e-9;
			// The hole is inside the rim: smaller mean radius about the centreline.
			if (!rib.holes.empty()) {
				double ro = 0.0, ri = 0.0;
				const double cz = nacelle.CentrelineZ(rib.eta);
				for (int idx : rib.rim) ro += std::hypot(body.vertices[idx].y, body.vertices[idx].z - cz);
				for (int idx : rib.holes[0]) ri += std::hypot(body.vertices[idx].y, body.vertices[idx].z - cz);
				annuli = annuli && ri / rib.holes[0].size() < ro / rib.rim.size();
			}
		}
		Check(annuli, "lofted nacelle: every frame is the annulus between the outer rim and the inner skin's ring");
		Check(planar, "lofted nacelle: both rings of every frame lie in the plane x = station x length");
		Check(plan.stringers.size() == 4, F("lofted nacelle: four stringers along the outer skin (%g)", plan.stringers.size()));
		for (const ShellPlan& st : plan.stringers) {
			const int on = CountOnBody(body, st.mesh);
			Check(!st.mesh.IsEmpty() && on >= static_cast<int>(st.mesh.vertices.size()) / 2, st.name + ": feet on the body (" + std::to_string(on) + " of " + std::to_string(st.mesh.vertices.size()) + ")");
		}
		// With chevrons there is no mesh of its own and no plan.
		nacelle.chevron_count = 6;
		Check(nacelle.BuildMeshes(2e-3).empty() && nacelle.LastStructurePlans().empty(), "lofted nacelle: chevrons leave no mesh and no plan");
		// Round trip.
		nacelle.chevron_count = 0;
		LoftedNacelleSpec back = DecodeLoftedNacelleSpec(Encode(nacelle.Serialize()));
		Check(back.structure.frames.between == 3 && back.structure.frames.height == 0.0 && back.structure.stringers.size() == 4, "lofted nacelle: the structure survives a YAML round trip");
	}

	// ---- the simple nacelle ----
	{
		NacelleSpec ns;
		NacelleModel nacelle(ns);
		nacelle.structure.frames.between = 2;
		std::vector<NamedMesh> bodies = nacelle.BuildMeshes(2e-3);
		const std::vector<BodyStructurePlan>& plans = nacelle.LastStructurePlans();
		Check(bodies.size() == 1 && plans.size() == 1 && bodies[0].mesh.IsClosedManifold(), "nacelle: one closed body, one plan");
		const BodyStructurePlan& plan = plans[0];
		for (const std::string& note : plan.notes) std::printf("  note: %s\n", note.c_str());
		bool annuli = plan.ribs.size() == 2;
		for (const RibPlan& rib : plan.ribs) annuli = annuli && rib.holes.size() == 1 && rib.rim.size() >= 16 && rib.height == 0.1;
		Check(annuli, F("nacelle: two annular frames with the default band height (%g frames)", plan.ribs.size()));
		Check(plan.stringers.size() == 4 && plan.notes.empty(), F("nacelle: four stringers, no notes (%g stringers)", plan.stringers.size()));
		NacelleSpec back = DecodeNacelleSpec(Encode(nacelle.Serialize()));
		Check(back.structure.frames.between == 2 && back.structure.stringers.size() == 4, "nacelle: the structure survives a YAML round trip");
	}

	std::printf("RESULT: %d passed, %d failed\n", g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}
