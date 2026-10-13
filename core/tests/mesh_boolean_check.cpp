// Checks the mesh boolean (mesh_boolean.hpp): fuse / common / cut of a
// sphere and a box (volumes consistent with inclusion-exclusion, results
// closed), disjoint bodies, mirrored halves welded along their shared cap,
// refused coplanar overlaps, and the P-38's airframe fused into one body.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "aerotelic/fuselage_model.hpp"
#include "aerotelic/mesh.hpp"
#include "aerotelic/mesh_boolean.hpp"
#include "aerotelic/project_yaml.hpp"
#include "aerotelic/shell.hpp"
#include "aerotelic/wing_model.hpp"

using namespace aerotelic;
using namespace aerotelic::project_yaml;

namespace {

int g_failures = 0;
void Check(bool ok, const std::string& what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what.c_str());
	if (!ok) ++g_failures;
}
template <class F>
void Case(const char* name, F&& fn) {
	try { fn(); } catch (const std::exception& e) { Check(false, std::string(name) + ": exception: " + e.what()); }
}
std::string Fmt(const char* fmt, double a, double b = 0.0, double c = 0.0) {
	char buf[300];
	std::snprintf(buf, sizeof buf, fmt, a, b, c);
	return buf;
}

void Report(const char* label, const MeshBooleanStats& s, const TriMesh& r) {
	std::printf("%s: %d coincident pairs, %d pairs tested, %d intersecting, %d segments, %d triangles cut into %d pieces, kept %d + %d; volumes %.6g op %.6g -> %.6g; result %zu vertices %zu triangles; %.3f s\n",
		label, s.n_coincident_pairs, s.n_pairs_tested, s.n_pairs_intersecting, s.n_segments, s.n_triangles_cut, s.n_pieces, s.n_kept_a, s.n_kept_b, s.volume_a, s.volume_b, s.volume_result, r.vertices.size(), r.triangles.size(), s.seconds);
}

// Rotation about an axis by an angle, as the row-major 3x3 TriMesh::Transform takes.
std::array<double, 9> Rotation(const std::array<double, 3>& axis, double deg) {
	Vec3 u = Normalized(Vec3(axis));
	double c = std::cos(deg * 3.14159265358979323846 / 180.0), s = std::sin(deg * 3.14159265358979323846 / 180.0), t = 1.0 - c;
	if (LengthSq(u) == 0.0 || deg == 0.0) return {1, 0, 0, 0, 1, 0, 0, 0, 1};
	return {t * u.x * u.x + c, t * u.x * u.y - s * u.z, t * u.x * u.z + s * u.y,
		t * u.x * u.y + s * u.z, t * u.y * u.y + c, t * u.y * u.z - s * u.x,
		t * u.x * u.z - s * u.y, t * u.y * u.z + s * u.x, t * u.z * u.z + c};
}

} // namespace

int main() {
	setvbuf(stdout, nullptr, _IONBF, 0);

	// ---- sphere and box, overlapping ----
	Case("sphere+box", [] {
		TriMesh sph = EllipsoidMesh(1.0, 1.0, 1.0, 2e-3);
		TriMesh box = BoxMesh(2.0, 1.0, 0.5);
		box.Translate({0.8, 0.1, 0.05});
		MeshBooleanStats sf, sc, su;
		TriMesh fuse = MeshBoolean(sph, box, MeshBooleanOp::Fuse, &sf);
		TriMesh common = MeshBoolean(sph, box, MeshBooleanOp::Common, &sc);
		TriMesh cut = MeshBoolean(sph, box, MeshBooleanOp::Cut, &su);
		Report("sphere+box fuse", sf, fuse);
		Report("sphere+box common", sc, common);
		Report("sphere+box cut", su, cut);
		Check(fuse.IsClosedManifold() && common.IsClosedManifold() && cut.IsClosedManifold(), "sphere+box: all three results are closed manifolds");
		double va = sph.SignedVolume(), vb = box.SignedVolume();
		Check(std::fabs(fuse.SignedVolume() + common.SignedVolume() - va - vb) <= 1e-9 * (va + vb), Fmt("sphere+box: fuse + common = a + b (%.9g vs %.9g)", fuse.SignedVolume() + common.SignedVolume(), va + vb));
		Check(std::fabs(cut.SignedVolume() + common.SignedVolume() - va) <= 1e-9 * va, Fmt("sphere+box: cut + common = a (%.9g vs %.9g)", cut.SignedVolume() + common.SignedVolume(), va));
		Check(common.SignedVolume() > 0.1 && common.SignedVolume() < vb, Fmt("sphere+box: common volume %.6g is a proper part of the box's %.6g", common.SignedVolume(), vb));
		Check(sf.n_pairs_intersecting > 0 && sf.n_segments == sf.n_pairs_intersecting, "sphere+box: one segment per intersecting pair");
		WriteStlAscii(fuse, "C:\\Users\\Matt\\devel_1\\aerotelic\\core\\build\\bool_sphere_box_fuse.stl", "fuse");
		WriteStlAscii(cut, "C:\\Users\\Matt\\devel_1\\aerotelic\\core\\build\\bool_sphere_box_cut.stl", "cut");
	});

	// ---- disjoint ----
	Case("disjoint", [] {
		TriMesh sph = EllipsoidMesh(1.0, 1.0, 1.0, 2e-3);
		TriMesh box = BoxMesh(1.0, 1.0, 1.0);
		box.Translate({5.0, 0.0, 0.0});
		MeshBooleanStats s;
		TriMesh fuse = MeshBoolean(sph, box, MeshBooleanOp::Fuse, &s);
		Check(fuse.IsClosedManifold() && std::fabs(fuse.SignedVolume() - sph.SignedVolume() - box.SignedVolume()) <= 1e-9, "disjoint: fuse is both bodies");
		TriMesh common = MeshBoolean(sph, box, MeshBooleanOp::Common);
		Check(common.IsEmpty(), "disjoint: common is empty");
		TriMesh cut = MeshBoolean(sph, box, MeshBooleanOp::Cut);
		Check(cut.IsClosedManifold() && std::fabs(cut.SignedVolume() - sph.SignedVolume()) <= 1e-9, "disjoint: cut is the first body");
	});

	// ---- a body inside the other ----
	Case("nested", [] {
		TriMesh sph = EllipsoidMesh(1.0, 1.0, 1.0, 2e-3);
		TriMesh box = BoxMesh(0.5, 0.5, 0.5);
		MeshBooleanStats s;
		TriMesh fuse = MeshBoolean(sph, box, MeshBooleanOp::Fuse, &s);
		Check(std::fabs(fuse.SignedVolume() - sph.SignedVolume()) <= 1e-9 && s.n_pairs_intersecting == 0, "nested: fuse is the sphere");
		TriMesh common = MeshBoolean(sph, box, MeshBooleanOp::Common);
		Check(std::fabs(common.SignedVolume() - box.SignedVolume()) <= 1e-9, "nested: common is the box");
		TriMesh cut = MeshBoolean(sph, box, MeshBooleanOp::Cut);
		Check(cut.IsClosedManifold() && std::fabs(cut.SignedVolume() - (sph.SignedVolume() - box.SignedVolume())) <= 1e-9, "nested: cut is a hollow sphere (two shells)");
	});

	// ---- coplanar overlap refused ----
	Case("coplanar", [] {
		TriMesh a = BoxMesh(2.0, 1.0, 0.5);
		TriMesh b = BoxMesh(2.0, 1.0, 0.5);
		b.Translate({0.5, 0.0, 0.5}); // b's bottom face lies in a's top face plane, overlapping
		bool refused = false;
		try { MeshBoolean(a, b, MeshBooleanOp::Fuse); } catch (const MeshBooleanError& e) { refused = true; std::printf("coplanar: refused as expected: %s\n", e.what()); }
		Check(refused, "coplanar: two boxes sharing a face plane are refused");
		TriMesh c = BoxMesh(2.0, 1.0, 0.5);
		refused = false;
		try { MeshBoolean(a, c, MeshBooleanOp::Fuse); } catch (const MeshBooleanError& e) { refused = true; std::printf("coplanar: refused as expected: %s\n", e.what()); }
		Check(refused, "coplanar: two identical boxes are refused");
	});

	// ---- mirrored halves welded along the shared cap: the tailplane (no
	// dihedral, so its root cap lies in the symmetry plane) ----
	Case("mirror weld", [] {
		ProjectFile project = LoadProjectYaml("C:\\Users\\Matt\\devel_1\\WingDesigner\\projects\\p38.yaml");
		const ProjectEntity* ht = nullptr;
		for (auto& e : project.entities) if (e.name == "HT") ht = &e;
		Check(ht != nullptr, "mirror weld: HT found");
		if (!ht) return;
		TriMesh half = WingModel(*std::get_if<WingModelSpec>(&ht->model)).BuildMesh(2e-3);
		half.Transform(Rotation(ht->placement.rotation_axis, ht->placement.rotation_deg), Vec3(ht->placement.translation));
		TriMesh other = half;
		other.Mirror({0, 0, 0}, {0, 1, 0});
		MeshBooleanStats s;
		TriMesh whole = MeshBoolean(half, other, MeshBooleanOp::Fuse, &s);
		Report("mirror weld", s, whole);
		Check(s.n_coincident_pairs > 0, Fmt("mirror weld: the root caps were recognised as a shared wall (%.0f pairs)", (double)s.n_coincident_pairs));
		Check(whole.IsClosedManifold(), "mirror weld: the whole tailplane is a closed manifold");
		Check(std::fabs(whole.SignedVolume() - 2.0 * half.SignedVolume()) <= 1e-6 * half.SignedVolume(), Fmt("mirror weld: volume %.9g = twice the half's %.9g", whole.SignedVolume(), half.SignedVolume()));
	});

	// ---- mirrored halves that overshoot the plane: the wing (its section
	// frame follows the dihedral curve, so the root section is tilted and
	// crosses the symmetry plane by the thickness times the dihedral's
	// sine; every edge crossing the plane crosses its own mirror image
	// there exactly) ----
	Case("mirror overlap", [] {
		ProjectFile project = LoadProjectYaml("C:\\Users\\Matt\\devel_1\\WingDesigner\\projects\\p38.yaml");
		const WingModelSpec* ws = nullptr;
		for (auto& e : project.entities) if (e.name == "Wing") ws = std::get_if<WingModelSpec>(&e.model);
		Check(ws != nullptr, "mirror overlap: Wing found");
		if (!ws) return;
		TriMesh half = WingModel(*ws).BuildMesh(2e-3);
		half.Translate({3.1, 0.0, 0.1});
		TriMesh other = half;
		other.Mirror({0, 0, 0}, {0, 1, 0});
		MeshBooleanStats s;
		TriMesh whole = MeshBoolean(half, other, MeshBooleanOp::Fuse, &s);
		Report("mirror overlap", s, whole);
		Check(s.n_pairs_intersecting > 0, "mirror overlap: the halves intersect along the root");
		Check(whole.IsClosedManifold(), "mirror overlap: the whole wing is a closed manifold");
		double v2 = 2.0 * half.SignedVolume();
		Check(whole.SignedVolume() < v2 && whole.SignedVolume() > 0.99 * v2, Fmt("mirror overlap: volume %.9g is just under twice the half's (%.9g)", whole.SignedVolume(), v2));
	});

	// ---- the P-38 airframe: fuselage, wing, booms, tails, both halves ----
	Case("p38 airframe", [] {
		ProjectFile project = LoadProjectYaml("C:\\Users\\Matt\\devel_1\\WingDesigner\\projects\\p38.yaml");
		std::vector<std::pair<std::string, TriMesh>> bodies;
		for (const auto& e : project.entities) {
			TriMesh m;
			if (auto* w = std::get_if<WingModelSpec>(&e.model)) m = WingModel(*w).BuildMesh(2e-3);
			else if (auto* f = std::get_if<FuselageSpec>(&e.model)) m = FuselageModel(*f).BuildMeshes(2e-3)[0].mesh;
			else continue;
			m.Transform(Rotation(e.placement.rotation_axis, e.placement.rotation_deg), Vec3(e.placement.translation));
			bodies.push_back({e.name, m});
			if (e.mirror) {
				TriMesh mm = m;
				mm.Mirror({0, 0, 0}, {0, 1, 0});
				bodies.push_back({e.name + " (mirror)", mm});
			}
		}
		std::printf("p38 airframe: %zu bodies\n", bodies.size());
		TriMesh result = bodies[0].second;
		double sum = result.SignedVolume();
		auto t0 = std::chrono::steady_clock::now();
		for (size_t i = 1; i < bodies.size(); ++i) {
			MeshBooleanStats s;
			TriMesh next = MeshBoolean(result, bodies[i].second, MeshBooleanOp::Fuse, &s);
			Report(("  + " + bodies[i].first).c_str(), s, next);
			sum += bodies[i].second.SignedVolume();
			result = std::move(next);
		}
		double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
		std::printf("p38 airframe: fused volume %.6g (sum of bodies %.6g, so %.4g in overlaps), %zu triangles, %.2f s\n", result.SignedVolume(), sum, sum - result.SignedVolume(), result.triangles.size(), dt);
		Check(result.IsClosedManifold(), "p38 airframe: the fused airframe is a closed manifold");
		Check(result.SignedVolume() < sum && result.SignedVolume() > 0.5 * sum, "p38 airframe: fused volume is below the sum of the bodies and above half of it");
		WriteStlAscii(result, "C:\\Users\\Matt\\devel_1\\aerotelic\\core\\build\\bool_p38_airframe.stl", "p38");
		std::printf("p38 airframe: wrote core\\build\\bool_p38_airframe.stl\n");
	});

	std::printf("RESULT: %d failure(s)\n", g_failures);
	return g_failures == 0 ? 0 : 1;
}
