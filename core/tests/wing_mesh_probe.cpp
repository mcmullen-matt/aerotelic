// Diagnostic (2026-11-09): the triangle quality of every wing-type entity
// of a project file, built with its structure (the spar feet and rib
// stations seeded into the loft) and without, side by side. Run with the
// project path; prints per body the triangle count, the closed-manifold
// check, the smallest angle and edge and how many triangles are thinner
// than 1 and 5 degrees.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <variant>

#include "aerotelic/mesh.hpp"
#include "aerotelic/project_yaml.hpp"
#include "aerotelic/wing_model.hpp"

using namespace aerotelic;
using namespace aerotelic::project_yaml;

namespace {

void Report(const std::string& label, const TriMesh& m) {
	double min_angle = 180.0, min_edge = 1e30, max_edge = 0.0;
	int under1 = 0, under5 = 0;
	for (const auto& t : m.triangles) {
		const Vec3& a = m.vertices[t[0]];
		const Vec3& b = m.vertices[t[1]];
		const Vec3& c = m.vertices[t[2]];
		const double ab = Length(b - a), bc = Length(c - b), ca = Length(a - c);
		min_edge = std::min({min_edge, ab, bc, ca});
		max_edge = std::max({max_edge, ab, bc, ca});
		auto angle = [](double opp, double s1, double s2) {
			double cosv = (s1 * s1 + s2 * s2 - opp * opp) / std::max(2.0 * s1 * s2, 1e-300);
			return std::acos(std::clamp(cosv, -1.0, 1.0)) * 180.0 / 3.14159265358979323846;
		};
		const double amin = std::min({angle(bc, ab, ca), angle(ca, ab, bc), angle(ab, bc, ca)});
		min_angle = std::min(min_angle, amin);
		if (amin < 1.0) ++under1;
		if (amin < 5.0) ++under5;
	}
	std::printf("  %-28s %6zu tris %6zu verts  closed %d  vol %.5f  min angle %.3f deg  (<1: %d, <5: %d)  edges %.2e .. %.3f\n",
		label.c_str(), m.triangles.size(), m.vertices.size(), m.IsClosedManifold() ? 1 : 0, m.SignedVolume(), min_angle, under1, under5, min_edge, max_edge);
	// The three shortest edges, with their end points, to see what they
	// run along (a station pair, a chordwise pair).
	std::vector<std::pair<double, std::pair<int, int>>> edges;
	for (const auto& t : m.triangles) {
		for (int k = 0; k < 3; ++k) {
			int a = t[k], b = t[(k + 1) % 3];
			if (a != b) edges.push_back({Length(m.vertices[a] - m.vertices[b]), {std::min(a, b), std::max(a, b)}});
		}
	}
	std::sort(edges.begin(), edges.end());
	int shown = 0;
	std::pair<int, int> last{-1, -1};
	for (const auto& e : edges) {
		if (e.second == last) continue;
		last = e.second;
		const Vec3& p = m.vertices[e.second.first];
		const Vec3& q = m.vertices[e.second.second];
		std::printf("      edge %.3e: (%.6f, %.6f, %.6f) - (%.6f, %.6f, %.6f)\n", e.first, p.x, p.y, p.z, q.x, q.y, q.z);
		if (shown == 0) {
			// The ring this edge lies on: every vertex at the same y, in x order.
			std::vector<Vec3> ring;
			for (const Vec3& v : m.vertices) if (std::fabs(v.y - p.y) < 1e-9) ring.push_back(v);
			std::sort(ring.begin(), ring.end(), [](const Vec3& a, const Vec3& b) { return a.x < b.x; });
			std::printf("      ring at y = %.6f: %zu vertices; x from %.6f to %.6f\n", p.y, ring.size(), ring.front().x, ring.back().x);
			int tiny = 0;
			for (size_t i = 1; i < ring.size(); ++i) if (Length(ring[i] - ring[i - 1]) < 1e-4) ++tiny;
			std::printf("      %d consecutive pairs closer than 1e-4\n", tiny);
			// Also the neighbouring rings' y values (the stations around it).
			std::vector<double> ys;
			for (const Vec3& v : m.vertices) ys.push_back(v.y);
			std::sort(ys.begin(), ys.end());
			ys.erase(std::unique(ys.begin(), ys.end(), [](double a, double b) { return std::fabs(a - b) < 1e-9; }), ys.end());
			std::printf("      %zu distinct station y values; the last six:", ys.size());
			for (size_t i = ys.size() >= 6 ? ys.size() - 6 : 0; i < ys.size(); ++i) std::printf(" %.6f", ys[i]);
			std::printf("\n");
		}
		if (++shown >= 3) break;
	}
}

}  // namespace

// The thin triangles in a band of y (an analysis station), to compare two
// builds of the same wing there.
void BandReport(const std::string& label, const TriMesh& m, double y0, double half) {
	int n = 0, thin = 0;
	double min_angle = 180.0, min_area = 1e30;
	for (const auto& t : m.triangles) {
		const Vec3& a = m.vertices[t[0]];
		const Vec3& b = m.vertices[t[1]];
		const Vec3& c = m.vertices[t[2]];
		const double yc = (a.y + b.y + c.y) / 3.0;
		if (std::fabs(yc - y0) > half) continue;
		++n;
		const double ab = Length(b - a), bc = Length(c - b), ca = Length(a - c);
		auto angle = [](double opp, double s1, double s2) {
			double cosv = (s1 * s1 + s2 * s2 - opp * opp) / std::max(2.0 * s1 * s2, 1e-300);
			return std::acos(std::clamp(cosv, -1.0, 1.0)) * 180.0 / 3.14159265358979323846;
		};
		const double amin = std::min({angle(bc, ab, ca), angle(ca, ab, bc), angle(ab, bc, ca)});
		min_angle = std::min(min_angle, amin);
		min_area = std::min(min_area, 0.5 * Length(Cross(b - a, c - a)));
		if (amin < 2.0) {
			++thin;
			std::printf("      thin %.2f deg: (%.4f, %.4f, %.4f) (%.4f, %.4f, %.4f) (%.4f, %.4f, %.4f)\n", amin, a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);
		}
	}
	std::printf("  %s: band y = %.3f +- %.3f: %d triangles, %d thinner than 2 deg, min angle %.3f, min area %.2e\n", label.c_str(), y0, half, n, thin, min_angle, min_area);
}

int main(int argc, char** argv) {
	if (argc < 2) { std::printf("usage: wing_mesh_probe <project.yaml> [tolerance] [band_y band_half]\n"); return 2; }
	const double tol = argc > 2 ? std::atof(argv[2]) : 2e-3;
	const double band_y = argc > 4 ? std::atof(argv[3]) : -1.0, band_half = argc > 4 ? std::atof(argv[4]) : 0.0;
	ProjectFile pf = LoadProjectYaml(argv[1]);
	for (const ProjectEntity& e : pf.entities) {
		if (e.type != EntityType::Wing) continue;
		const WingModelSpec& spec = std::get<WingModelSpec>(e.model);
		std::printf("%s: %zu spars, %d ribs between, %zu stringers\n", e.name.c_str(), spec.structure.spars.size(), spec.structure.ribs.between, spec.structure.stringers.size());
		{
			WingModel wing(spec);
			std::vector<NamedMesh> bodies = wing.BuildMeshes(tol);
			for (const NamedMesh& nm : bodies) Report("with structure / " + nm.name, nm.mesh);
			if (band_y >= 0.0) for (const NamedMesh& nm : bodies) BandReport("with structure / " + nm.name, nm.mesh, band_y, band_half);
			for (const WingBodyStructurePlan& p : wing.LastStructurePlans()) {
				std::printf("    plan %s: %zu ribs, %zu spars, rib size %.4f\n", p.body.c_str(), p.ribs.size(), p.spars.size(), p.rib_size);
				for (const std::string& n : p.notes) std::printf("    note: %s\n", n.c_str());
			}
		}
		{
			WingModelSpec bare = spec;
			bare.structure = WingStructureSpec();
			bare.structure.spars.clear();
			bare.structure.ribs.at_sections = false;
			bare.structure.ribs.at_flap_ends = false;
			bare.structure.ribs.between = 0;
			WingModel wing(bare);
			std::vector<NamedMesh> bodies = wing.BuildMeshes(tol);
			for (const NamedMesh& nm : bodies) Report("bare / " + nm.name, nm.mesh);
			if (band_y >= 0.0) for (const NamedMesh& nm : bodies) BandReport("bare / " + nm.name, nm.mesh, band_y, band_half);
		}
	}
	return 0;
}
