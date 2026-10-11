// Checks the explicit skin layer (shell.hpp): on a sphere against the
// analytic shell, on a thin plate (thinner than two skins: all skin), and
// on the P-38's port wing against the shell volume integrated from the
// wing's own distance field.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include "aerotelic/mesh.hpp"
#include "aerotelic/project_yaml.hpp"
#include "aerotelic/sdf.hpp"
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

constexpr double kPi = 3.14159265358979323846;

// Monte Carlo volume of {-t < b < 0} over the mesh's box.
double FieldShellVolume(const MeshSdf& sdf, const Box3& box, double t, int n, unsigned seed = 11) {
	std::mt19937 rng(seed);
	std::uniform_real_distribution<double> U(0.0, 1.0);
	int inside = 0;
	Vec3 ext = box.Extent();
	for (int i = 0; i < n; ++i) {
		Vec3 p{box.min.x + U(rng) * ext.x, box.min.y + U(rng) * ext.y, box.min.z + U(rng) * ext.z};
		double b = sdf.Distance(p);
		if (b < 0.0 && b > -t) ++inside;
	}
	return ext.x * ext.y * ext.z * inside / n;
}

void Report(const char* label, const ShellResult& s) {
	std::printf("%s: %zu surface nodes, %zu nodes, %zu tets, skin volume %.6g, min tet %.3g, collapsed %d, bad normals %d, lipschitz-limited %d, degenerate tets %d, inverted prisms' tets %d, cancelled pairs %d\n",
		label, (size_t)s.n_surface_nodes, s.nodes.size(), s.tets.size(), s.skin_volume, s.min_tet_volume, s.n_collapsed_vertices, s.n_bad_normals,
		s.n_lipschitz_limited, s.n_degenerate_tets, s.n_negative_tets, s.n_cancelled_pairs);
}

// Where things went wrong, for the eye.
void Diagnose(const TriMesh& surface, const ShellResult& s, double t) {
	// Offset triangles whose normal turned against their surface triangle's:
	// a tilted top (steep inner face) or a real fold -- the generator's own
	// tet-sign test decides which (n_negative_tets); this is just a map.
	int n_turned = 0;
	Box3 turned_box;
	for (size_t ti = 0; ti < surface.triangles.size(); ++ti) {
		const auto& it = s.inner_triangles[ti];
		if (it[0] < 0) continue;
		const auto& tri = surface.triangles[ti];
		Vec3 nb = Cross(surface.vertices[tri[1]] - surface.vertices[tri[0]], surface.vertices[tri[2]] - surface.vertices[tri[0]]);
		Vec3 nt = Cross(s.nodes[it[1]] - s.nodes[it[0]], s.nodes[it[2]] - s.nodes[it[0]]);
		if (Dot(nb, nt) >= 0.0) continue;
		++n_turned;
		turned_box.Add((surface.vertices[tri[0]] + surface.vertices[tri[1]] + surface.vertices[tri[2]]) / 3.0);
	}
	if (n_turned) std::printf("    %d offset triangles turned against their surface triangle, within x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]; folded prisms' tets: %d\n", n_turned,
		turned_box.min.x, turned_box.max.x, turned_box.min.y, turned_box.max.y, turned_box.min.z, turned_box.max.z, s.n_negative_tets);
	// Deepest node.
	int worst = -1;
	for (int i = 0; i < s.n_surface_nodes; ++i) if (worst < 0 || s.depth[i] < s.depth[worst]) worst = i;
	if (worst >= 0) {
		const Vec3& v = surface.vertices[worst];
		std::printf("    deepest node: vertex (%.3f, %.3f, %.3f) walk %.5f depth %.5f (skin %.5f) collapsed %d\n", v.x, v.y, v.z, s.walk_distance[worst], s.depth[worst], t, (int)s.collapsed[worst]);
	}
	// Non-manifold edges of the full inner copy.
	std::unordered_map<uint64_t, std::pair<int, int>> edges;
	for (const auto& it : s.inner_triangles) {
		if (it[0] < 0) continue;
		for (int k = 0; k < 3; ++k) {
			int a = it[k], b = it[(k + 1) % 3];
			uint64_t key = (static_cast<uint64_t>(std::min(a, b)) << 32) | static_cast<uint64_t>(std::max(a, b));
			auto& e = edges[key];
			e.first += 1;
			e.second += (a < b) ? 1 : -1;
		}
	}
	int bad = 0;
	Box3 bad_box;
	for (auto& [key, e] : edges) {
		if (e.first == 2 && e.second == 0) continue;
		++bad;
		bad_box.Add(s.nodes[static_cast<int>(key >> 32)]);
		if (bad <= 4) {
			const Vec3& a = s.nodes[static_cast<int>(key >> 32)];
			std::printf("    non-manifold inner edge (count %d, net %d) at (%.3f, %.3f, %.3f)\n", e.first, e.second, a.x, a.y, a.z);
		}
	}
	if (bad) std::printf("    %d non-manifold inner edges within x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]\n", bad, bad_box.min.x, bad_box.max.x, bad_box.min.y, bad_box.max.y, bad_box.min.z, bad_box.max.z);

	// The interior domain's non-manifold edges, with the triangles on them.
	{
		std::unordered_map<uint64_t, std::vector<int>> on_edge;
		for (size_t i = 0; i < s.inner_triangles.size(); ++i) {
			const auto& it = s.inner_triangles[i];
			if (it[0] < 0 || s.inner_collapsed[i]) continue;
			for (int k = 0; k < 3; ++k) {
				int a = it[k], b = it[(k + 1) % 3];
				uint64_t key = (static_cast<uint64_t>(std::min(a, b)) << 32) | static_cast<uint64_t>(std::max(a, b));
				on_edge[key].push_back(static_cast<int>(i));
			}
		}
		int shown_edges = 0, n_bad = 0;
		for (auto& [key, tris] : on_edge) {
			if (tris.size() == 2) continue;
			++n_bad;
			if (shown_edges++ >= 3) continue;
			int a = static_cast<int>(key >> 32), b = static_cast<int>(key & 0xffffffffu);
			std::printf("    domain edge %d-%d at (%.3f,%.3f,%.3f)-(%.3f,%.3f,%.3f) has %zu triangles:\n", a, b, s.nodes[a].x, s.nodes[a].y, s.nodes[a].z, s.nodes[b].x,
				s.nodes[b].y, s.nodes[b].z, tris.size());
			for (int ti : tris) {
				const auto& st = surface.triangles[ti];
				Vec3 c = (surface.vertices[st[0]] + surface.vertices[st[1]] + surface.vertices[st[2]]) / 3.0;
				std::printf("      surface tri %d centroid (%.3f,%.3f,%.3f) offsets %d/%d/%d collapsed %d%d%d walks %.4f/%.4f/%.4f\n", ti, c.x, c.y, c.z, s.offset_node[st[0]],
					s.offset_node[st[1]], s.offset_node[st[2]], (int)s.collapsed[st[0]], (int)s.collapsed[st[1]], (int)s.collapsed[st[2]], s.walk_distance[st[0]],
					s.walk_distance[st[1]], s.walk_distance[st[2]]);
			}
		}
		std::printf("    interior domain before sealing: %d open edges along the pinch\n", n_bad);
	}

	// Where the skin is thinner than asked: the edge-slope bound and the
	// relaxation, by amount.
	{
		int limited = 0;
		double worst_cut = 0.0;
		Vec3 worst_at;
		for (int i = 0; i < s.n_surface_nodes; ++i) {
			double cut = s.walk_unlimited[i] - s.walk_distance[i];
			if (cut > 1e-9) { ++limited; if (cut > worst_cut) { worst_cut = cut; worst_at = surface.vertices[i]; } }
		}
		std::printf("    walks cut by the bound/relaxation: %d, worst by %.4f (of %.4f) at (%.3f, %.3f, %.3f)\n", limited, worst_cut, t, worst_at.x, worst_at.y, worst_at.z);
		// Volume deficit per region: prism volume vs area x min(skin, half thickness proxy = -depth).
		double ideal_root = 0.0, got_root = 0.0, ideal_mid = 0.0, got_mid = 0.0, ideal_tip = 0.0, got_tip = 0.0;
		std::vector<double> prism_vol(surface.triangles.size(), 0.0);
		for (size_t k = 0; k < s.tets.size(); ++k) {
			const auto& tet = s.tets[k];
			prism_vol[s.tet_triangle[k]] += Dot(s.nodes[tet[1]] - s.nodes[tet[0]], Cross(s.nodes[tet[2]] - s.nodes[tet[0]], s.nodes[tet[3]] - s.nodes[tet[0]])) / 6.0;
		}
		for (size_t ti = 0; ti < surface.triangles.size(); ++ti) {
			const auto& st = surface.triangles[ti];
			double area = 0.5 * Length(Cross(surface.vertices[st[1]] - surface.vertices[st[0]], surface.vertices[st[2]] - surface.vertices[st[0]]));
			double want = 0.0;
			for (int k = 0; k < 3; ++k) want += std::min(t, -s.depth[st[k]]) / 3.0; // what the walks could reach
			double ideal = area * want;
			Vec3 c = (surface.vertices[st[0]] + surface.vertices[st[1]] + surface.vertices[st[2]]) / 3.0;
			if (c.y > -1.0) { ideal_root += ideal; got_root += prism_vol[ti]; }
			else if (c.y > -7.0) { ideal_mid += ideal; got_mid += prism_vol[ti]; }
			else { ideal_tip += ideal; got_tip += prism_vol[ti]; }
		}
		std::printf("    prism volume vs area x reached depth: root %.4f/%.4f, mid %.4f/%.4f, tip %.4f/%.4f\n", got_root, ideal_root, got_mid, ideal_mid, got_tip, ideal_tip);
	}
}

void CommonChecks(const char* label, const ShellResult& s, const MeshSdf& sdf, double t) {
	std::string L = label;
	Check(s.n_negative_tets == 0, L + ": no inverted prism");
	Check(s.n_bad_normals == 0, L + ": every walk entered the body");
	// No offset node outside the body, none deeper than the skin.
	double worst_out = -1e300, worst_deep = 0.0;
	for (int i = 0; i < s.n_surface_nodes; ++i) {
		worst_out = std::max(worst_out, s.depth[i]);
		worst_deep = std::max(worst_deep, -s.depth[i] - t);
	}
	char buf[200];
	std::snprintf(buf, sizeof buf, "%s: no offset node outside the body (max field %.3g)", label, worst_out);
	Check(worst_out <= 1e-6 * t, buf);
	std::snprintf(buf, sizeof buf, "%s: no offset node deeper than the skin (max excess %.3g)", label, worst_deep);
	Check(worst_deep <= 2e-3 * t, buf);
	// Every tet positive and every one of its nodes inside or on the body.
	double min_vol = 1e300;
	int outside_nodes = 0;
	for (const auto& tet : s.tets) {
		const Vec3& a = s.nodes[tet[0]];
		const Vec3& b = s.nodes[tet[1]];
		const Vec3& c = s.nodes[tet[2]];
		const Vec3& d = s.nodes[tet[3]];
		min_vol = std::min(min_vol, Dot(b - a, Cross(c - a, d - a)) / 6.0);
		Vec3 centroid = (a + b + c + d) * 0.25;
		if (sdf.Distance(centroid) > 1e-6 * t) ++outside_nodes;
	}
	Check(min_vol > 0.0, L + ": every tetrahedron has positive volume");
	std::snprintf(buf, sizeof buf, "%s: every tetrahedron's centroid is inside the body (%d outside)", label, outside_nodes);
	Check(outside_nodes == 0, buf);
	// The interior domain handed to a tetrahedral mesher: the offset copy
	// without the collapsed sheet, sealed along the pinch where the two
	// sides share nodes.
	TriMesh domain = s.InnerSurface(true);
	TriMesh full = s.InnerSurface(false);
	std::printf("    inner surface: full copy %zu triangles (closed manifold %d); interior domain %zu triangles (closed manifold %d); relaxation rounds %d\n", full.triangles.size(),
		(int)full.IsClosedManifold(), domain.triangles.size(), (int)domain.IsClosedManifold(), s.n_relaxation_rounds);
	Check(domain.IsEmpty() || domain.IsClosedManifold(), L + ": the interior domain (collapsed sheet dropped) is a closed manifold or empty");
}

} // namespace

int main() {
	setvbuf(stdout, nullptr, _IONBF, 0);
	try {
		// ---- sphere: analytic shell ----
		{
			const double R = 1.0, t = 0.05;
			TriMesh sph = EllipsoidMesh(R, R, R, 2e-4);
			MeshSdf sdf(sph);
			ShellOptions opt;
			opt.thickness = t;
			ShellResult s = BuildOffsetShell(sph, sdf, opt);
			Report("sphere", s);
			CommonChecks("sphere", s, sdf, t);
			double expected = 4.0 / 3.0 * kPi * (R * R * R - std::pow(R - t, 3));
			char buf[200];
			std::snprintf(buf, sizeof buf, "sphere: skin volume %.5f within 1%% of the shell's %.5f", s.skin_volume, expected);
			Check(std::fabs(s.skin_volume - expected) < 0.01 * expected, buf);
			Check(s.n_collapsed_vertices == 0, "sphere: nothing collapsed");
			TriMesh inner = s.InnerSurface();
			double worst = 0.0;
			for (const auto& p : inner.vertices) worst = std::max(worst, std::fabs(Length(p) - (R - t)));
			std::snprintf(buf, sizeof buf, "sphere: inner surface on radius %.3f (worst deviation %.2e)", R - t, worst);
			Check(worst < 1e-3 * t, buf);
			double inner_vol = inner.SignedVolume();
			std::snprintf(buf, sizeof buf, "sphere: inner surface encloses %.5f (4/3 pi (R-t)^3 = %.5f)", inner_vol, 4.0 / 3.0 * kPi * std::pow(R - t, 3));
			Check(std::fabs(inner_vol - 4.0 / 3.0 * kPi * std::pow(R - t, 3)) < 0.01 * inner_vol, buf);
		}

		// ---- thin plate: thinner than two skins everywhere -> all skin ----
		{
			const double t = 0.05;
			TriMesh plate = BoxMesh(2.0, 1.0, 0.06); // 0.06 thick < 2 t
			MeshSdf sdf(plate);
			ShellOptions opt;
			opt.thickness = t;
			ShellResult s = BuildOffsetShell(plate, sdf, opt);
			Report("plate", s);
			CommonChecks("plate", s, sdf, t);
			char buf[200];
			std::snprintf(buf, sizeof buf, "plate: skin volume %.5f is the whole plate (%.5f) within 5%%", s.skin_volume, 2.0 * 1.0 * 0.06);
			Check(std::fabs(s.skin_volume - 0.12) < 0.05 * 0.12, buf);
			Check(s.n_collapsed_vertices == s.n_surface_nodes, "plate: every vertex collapsed onto the mid-surface");
			Check(s.InnerSurface().IsEmpty(), "plate: no interior domain is left (the whole plate is skin)");
		}

		// ---- the P-38's port wing ----
		{
			ProjectFile project = LoadProjectYaml("C:\\Users\\Matt\\devel_1\\WingDesigner\\projects\\p38.yaml");
			const WingModelSpec* ws = nullptr;
			for (auto& e : project.entities) if (e.name == "Wing") ws = std::get_if<WingModelSpec>(&e.model);
			Check(ws != nullptr, "p38: Wing entity found");
			if (ws) {
				WingModel wing(*ws);
				const double t = 0.01 * wing.planform().RootChord(); // the app's default skin
				TriMesh m = wing.BuildMesh(2e-3);
				m.Translate({3.1, 0.0, 0.1});          // the entity's placement
				m.Mirror({0, 0, 0}, {0, 1, 0});       // the port half
				MeshSdf sdf(m);
				ShellOptions opt;
				opt.thickness = t;
				auto t0 = std::chrono::steady_clock::now();
				ShellResult s = BuildOffsetShell(m, sdf, opt);
				double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
				std::printf("port wing: %zu triangles, skin %.4f, built in %.2f s\n", m.triangles.size(), t, dt);
				Report("port wing", s);
				Diagnose(m, s, t);
				CommonChecks("port wing", s, sdf, t);
				double field_vol = FieldShellVolume(sdf, m.Bounds(), t, 4000000);
				char buf[240];
				// The prisms stop short of the field's shell where the twist
				// relaxation and the slope bound shorten walks beside the pinch
				// and the tip cap: a few percent on this wing.
				std::snprintf(buf, sizeof buf, "port wing: skin volume %.5f within 5%% of the field-integrated shell %.5f (wing volume %.4f)", s.skin_volume, field_vol, m.SignedVolume());
				Check(std::fabs(s.skin_volume - field_vol) < 0.05 * field_vol, buf);
				std::snprintf(buf, sizeof buf, "port wing: %d of %d vertices collapsed (trailing edge and tip)", s.n_collapsed_vertices, s.n_surface_nodes);
				Check(s.n_collapsed_vertices > 0 && s.n_collapsed_vertices < s.n_surface_nodes, buf);
				std::printf("port wing: %d coincident opposite inner triangle pairs cancelled\n", s.n_cancelled_pairs);
				// The interior domain: the inner surface without the collapsed
				// sheet is open along the pinch (documented); the full copy is
				// closed and encloses wing volume minus the skin.
				TriMesh inner = s.InnerSurface(true);
				double inner_vol = inner.SignedVolume();
				std::snprintf(buf, sizeof buf, "port wing: interior domain encloses %.4f = wing %.4f - skin %.4f (within 5%% of the wing)", inner_vol, m.SignedVolume(), s.skin_volume);
				Check(std::fabs(inner_vol - (m.SignedVolume() - s.skin_volume)) < 0.05 * m.SignedVolume(), buf);
				const std::string out = "C:\\Users\\Matt\\devel_1\\aerotelic\\core\\build\\";
				WriteShellVtk(s, out + "p38_port_wing_skin.vtk");
				WriteStlAscii(s.InnerSurface(true), out + "p38_port_wing_inner.stl", "inner");
				WriteStlAscii(m, out + "p38_port_wing_surface.stl", "surface");
				std::printf("port wing: wrote %sp38_port_wing_skin.vtk, _inner.stl, _surface.stl\n", out.c_str());
			}
		}
	} catch (const std::exception& e) {
		std::printf("FAIL: exception: %s\n", e.what());
		++g_failures;
	}
	std::printf("RESULT: %d failure(s)\n", g_failures);
	return g_failures == 0 ? 0 : 1;
}
