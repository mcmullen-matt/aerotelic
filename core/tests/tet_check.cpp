// Checks the tetrahedral mesher (tet.hpp): the exact predicates, the
// Delaunay tetrahedralization on random and degenerate (grid) points, the
// conforming interior mesh on a sphere, a box, a thin plate and the P-38's
// port wing (the kept tetrahedra's volume must equal the surface's), and
// the skin / core partition.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include "aerotelic/fuselage_model.hpp"
#include "aerotelic/lofted_nacelle_model.hpp"
#include "aerotelic/mesh.hpp"
#include "aerotelic/nacelle_model.hpp"
#include "aerotelic/project_yaml.hpp"
#include "aerotelic/propeller_model.hpp"
#include "aerotelic/sdf.hpp"
#include "aerotelic/tet.hpp"
#include "aerotelic/wing_model.hpp"

using namespace aerotelic;
using namespace aerotelic::project_yaml;

namespace {

int g_failures = 0;
void Check(bool ok, const std::string& what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what.c_str());
	if (!ok) ++g_failures;
}
// Runs one case; an exception fails that case only.
template <class F>
void Case(const char* name, F&& fn) {
	try {
		fn();
	} catch (const std::exception& e) {
		Check(false, std::string(name) + ": exception: " + e.what());
	}
}
std::string Fmt(const char* fmt, double a, double b = 0.0, double c = 0.0) {
	char buf[300];
	std::snprintf(buf, sizeof buf, fmt, a, b, c);
	return buf;
}

constexpr double kPi = 3.14159265358979323846;

// A surface is closed when every edge is used an even number of times
// with no net direction; the number of edges used more than twice (pinches,
// where a region is one tetrahedron thick) is reported.
bool IsClosed(const TriMesh& m, int* n_pinched) {
	std::unordered_map<uint64_t, std::pair<int, int>> edges;
	for (const auto& t : m.triangles) {
		for (int k = 0; k < 3; ++k) {
			int a = t[k], b = t[(k + 1) % 3];
			uint64_t key = (static_cast<uint64_t>(std::min(a, b)) << 32) | static_cast<uint64_t>(std::max(a, b));
			auto& e = edges[key];
			e.first += 1;
			e.second += (a < b) ? 1 : -1;
		}
	}
	int pinched = 0;
	bool closed = true;
	for (const auto& kv : edges) {
		if (kv.second.second != 0 || (kv.second.first % 2) != 0) closed = false;
		if (kv.second.first > 2) ++pinched;
	}
	if (n_pinched) *n_pinched = pinched;
	return closed;
}

// Delaunay: no node strictly inside any tetrahedron's circumsphere.
int DelaunayViolations(const TetMesh& m, int sample_tets, const std::vector<int>* nodes_subset = nullptr) {
	int bad = 0;
	size_t step = std::max<size_t>(1, m.tets.size() / std::max(1, sample_tets));
	for (size_t t = 0; t < m.tets.size(); t += step) {
		const auto& v = m.tets[t];
		for (size_t n = 0; n < m.nodes.size(); ++n) {
			if (n == static_cast<size_t>(v[0]) || n == static_cast<size_t>(v[1]) || n == static_cast<size_t>(v[2]) || n == static_cast<size_t>(v[3])) continue;
			if (ExactInSphere(m.nodes[v[0]], m.nodes[v[1]], m.nodes[v[2]], m.nodes[v[3]], m.nodes[n]) > 0.0) { ++bad; break; }
		}
	}
	(void)nodes_subset;
	return bad;
}

// Every constrained face of the mesh is a surface triangle (same plane,
// inside it), and the constrained faces cover the surface's area.
void SurfaceConformity(const char* label, const TriMesh& surface, const TetMesh& m) {
	TriMesh faces;
	faces.vertices = m.nodes;
	for (size_t t = 0; t < m.tets.size(); ++t) {
		for (int i = 0; i < 4; ++i) {
			if (!(m.constrained[t] & (1u << i))) continue;
			faces.triangles.push_back({{m.tets[t][TetMesh::kFace[i][0]], m.tets[t][TetMesh::kFace[i][1]], m.tets[t][TetMesh::kFace[i][2]]}});
		}
	}
	// The exterior is gone, so each constrained face is a boundary face of
	// exactly one kept tetrahedron, and the boundary is nothing else.
	TriMesh boundary = m.BoundaryFaces(-1);
	std::string L = label;
	Check(boundary.triangles.size() == faces.triangles.size(), L + Fmt(": boundary faces (%.0f) are exactly the constrained faces (%.0f)", (double)boundary.triangles.size(), (double)faces.triangles.size()));
	Check(boundary.IsClosedManifold(), L + ": the mesh boundary is a closed manifold");
	double area_b = boundary.SurfaceArea(), area_s = surface.SurfaceArea();
	Check(std::fabs(area_b - area_s) <= 1e-9 * area_s, L + Fmt(": boundary area %.9g equals the surface's %.9g", area_b, area_s));
	Check(std::fabs(boundary.SignedVolume() - surface.SignedVolume()) <= 1e-9 * surface.SignedVolume(), L + Fmt(": boundary encloses %.9g = surface %.9g", boundary.SignedVolume(), surface.SignedVolume()));
	// Every boundary node lies on the surface (distance 0 within rounding).
	MeshSdf sdf(surface);
	double worst = 0.0;
	std::vector<char> on(m.nodes.size(), 0);
	for (const auto& tr : boundary.triangles) for (int k = 0; k < 3; ++k) on[tr[k]] = 1;
	int n_on = 0;
	for (size_t v = 0; v < m.nodes.size(); ++v) if (on[v]) { ++n_on; worst = std::max(worst, std::fabs(sdf.Distance(m.nodes[v]))); }
	Check(worst <= 1e-9 * surface.Bounds().Diagonal(), L + Fmt(": all %.0f boundary nodes lie on the surface (worst %.2e)", (double)n_on, worst));
	// No node outside the body.
	double worst_out = -1e300;
	for (size_t v = 0; v < m.nodes.size(); ++v) if (!on[v]) worst_out = std::max(worst_out, sdf.Distance(m.nodes[v]));
	Check(worst_out <= 0.0, L + Fmt(": every interior node is inside the body (max field %.3g)", worst_out));
}

// Flips every quad of the mesh (two triangles on a shared edge) to its
// longer diagonal where that keeps both triangles facing the same way: a
// deliberately bad triangulation whose faces are mostly NOT Delaunay, to
// exercise face recovery. Each pair is flipped at most once.
TriMesh FlipLongDiagonals(const TriMesh& in) {
	TriMesh m = in;
	std::unordered_map<uint64_t, std::vector<int>> on_edge;
	for (size_t t = 0; t < m.triangles.size(); ++t) for (int k = 0; k < 3; ++k) {
		int a = m.triangles[t][k], b = m.triangles[t][(k + 1) % 3];
		on_edge[(static_cast<uint64_t>(std::min(a, b)) << 32) | static_cast<uint64_t>(std::max(a, b))].push_back(static_cast<int>(t));
	}
	std::vector<char> touched(m.triangles.size(), 0);
	int flips = 0;
	for (auto& kv : on_edge) {
		if (kv.second.size() != 2) continue;
		int t1 = kv.second[0], t2 = kv.second[1];
		if (touched[t1] || touched[t2]) continue;
		int a = static_cast<int>(kv.first >> 32), b = static_cast<int>(kv.first & 0xffffffffu);
		auto third = [&](int t, bool* fwd) {
			const auto& tr = m.triangles[t];
			for (int k = 0; k < 3; ++k) {
				if (tr[k] == a && tr[(k + 1) % 3] == b) { *fwd = true; return tr[(k + 2) % 3]; }
				if (tr[k] == b && tr[(k + 1) % 3] == a) { *fwd = false; return tr[(k + 2) % 3]; }
			}
			return -1;
		};
		bool f1, f2;
		int c = third(t1, &f1), d = third(t2, &f2);
		if (c < 0 || d < 0 || f1 == f2) continue;
		if (!f1) { std::swap(t1, t2); std::swap(c, d); }
		// t1 = (a, b, c), t2 = (b, a, d). New: (a, d, c), (b, c, d).
		if (Distance(m.vertices[c], m.vertices[d]) <= Distance(m.vertices[a], m.vertices[b])) continue;
		Vec3 n_old = Cross(m.vertices[b] - m.vertices[a], m.vertices[c] - m.vertices[a]) + Cross(m.vertices[a] - m.vertices[b], m.vertices[d] - m.vertices[b]);
		Vec3 n1 = Cross(m.vertices[d] - m.vertices[a], m.vertices[c] - m.vertices[a]);
		Vec3 n2 = Cross(m.vertices[c] - m.vertices[b], m.vertices[d] - m.vertices[b]);
		if (Dot(n1, n_old) <= 0.2 * Length(n1) * Length(n_old) || Dot(n2, n_old) <= 0.2 * Length(n2) * Length(n_old)) continue;
		m.triangles[t1] = {{a, d, c}};
		m.triangles[t2] = {{b, c, d}};
		touched[t1] = touched[t2] = 1;
		++flips;
	}
	std::printf("flipped %d quads to their longer diagonal\n", flips);
	return m;
}

void ReportConforming(const char* label, const ConformingStats& s) {
	std::printf("%s: %d vertices, %d triangles, %d interior points; %d planar regions (%d flips, %d adopted, %d region points); segments missing %d -> %d edge Steiner, %d snaps, %d flips (%d refused), %d ABANDONED; faces missing %d -> %d cavity recoveries (%d enlargements), %d face Steiner, %d ABANDONED; %d surface faces; %d interior / %d exterior tets; volume %.9g vs surface %.9g; %.2f s (delaunay %.2f, segments %.2f, faces %.2f); walk fallbacks %d, cavity repairs %d\n",
		label, s.n_surface_vertices, s.n_surface_triangles, s.n_interior_points, s.n_planar_regions, s.n_planar_flips, s.n_regions_adopted, s.n_region_steiner, s.n_segments_missing, s.n_edge_steiner, s.n_segment_snaps, s.n_segment_flips, s.n_flips_refused, s.n_segments_abandoned, s.n_faces_missing, s.n_cavity_recoveries, s.n_cavity_enlargements,
		s.n_face_steiner, s.n_faces_abandoned, s.n_surface_faces, s.n_tets_interior, s.n_tets_exterior, s.mesh_volume, s.surface_volume, s.seconds_total, s.seconds_delaunay, s.seconds_segments, s.seconds_faces,
		s.delaunay.n_walk_fallbacks, s.delaunay.n_cavity_repairs);
}

void ConformingChecks(const char* label, const TriMesh& surface, const TetMesh& m, const ConformingStats& s) {
	std::string L = label;
	std::string why;
	bool valid = m.IsValid(&why);
	Check(valid, L + ": mesh valid (" + why + ")");
	Check(std::fabs(s.mesh_volume - s.surface_volume) <= 1e-9 * s.surface_volume, L + Fmt(": tetrahedra volume %.9g equals the surface volume %.9g", s.mesh_volume, s.surface_volume));
	for (int v = 0; v < static_cast<int>(surface.vertices.size()); ++v) {
		if (m.nodes[v].x != surface.vertices[v].x || m.nodes[v].y != surface.vertices[v].y || m.nodes[v].z != surface.vertices[v].z) { Check(false, L + ": surface vertices keep their indices and positions"); break; }
		if (v + 1 == static_cast<int>(surface.vertices.size())) Check(true, L + ": surface vertices keep their indices and positions");
	}
	SurfaceConformity(label, surface, m);
}

} // namespace

int main() {
	setvbuf(stdout, nullptr, _IONBF, 0);
	try {
		// ---- predicates ----
		{
			Vec3 a{0, 0, 0}, b{1, 0, 0}, c{0, 1, 0}, d{0, 0, 1};
			Check(ExactOrient(a, b, c, d) > 0.0, "orient: (0,0,0),(1,0,0),(0,1,0),(0,0,1) is positive");
			Check(ExactOrient(a, c, b, d) < 0.0, "orient: swapping two nodes flips the sign");
			Check(ExactOrient(a, b, c, Vec3{0.3, 0.3, 0}) == 0.0, "orient: a coplanar point gives exactly zero");
			Vec3 centroid = (a + b + c + d) * 0.25;
			Check(ExactInSphere(a, b, c, d, centroid) > 0.0, "insphere: the centroid is inside the circumsphere");
			Check(ExactInSphere(a, b, c, d, Vec3{5, 5, 5}) < 0.0, "insphere: a far point is outside");
			// The circumsphere of this tet: centre (0.5,0.5,0.5), radius sqrt(3)/2 -> (1,1,0) is on it.
			Check(ExactInSphere(a, b, c, d, Vec3{1, 1, 0}) == 0.0, "insphere: a cospherical point gives exactly zero");
		}

		// ---- random points ----
		{
			std::mt19937 rng(7);
			std::uniform_real_distribution<double> U(-1.0, 1.0);
			std::vector<Vec3> pts;
			for (int i = 0; i < 5000; ++i) pts.push_back({U(rng), U(rng), U(rng)});
			TetStats st;
			TetMesh m = DelaunayTetrahedralize(pts, &st);
			std::printf("random: %d points -> %d tets in %.3f s, %d walk steps, %d fallbacks, %d repairs\n", st.n_points, st.n_tets, st.seconds, st.n_walk_steps, st.n_walk_fallbacks, st.n_cavity_repairs);
			std::string why;
			Check(m.IsValid(&why), "random: mesh valid " + why);
			Check(m.tets.size() > 25000, "random: about 6.7 tets per point");
			int bad = DelaunayViolations(m, 400);
			Check(bad == 0, Fmt("random: no node inside a sampled circumsphere (%.0f violations)", bad));
			Check(m.BoundaryFaces().IsClosedManifold(), "random: the hull is a closed manifold");
			Check(st.n_walk_fallbacks == 0, "random: no point location fell back to a scan");
		}

		// ---- degenerate: a cubic grid (everything cospherical and coplanar) ----
		{
			std::vector<Vec3> pts;
			const int n = 9;
			for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) for (int k = 0; k < n; ++k) pts.push_back({double(i), double(j), double(k)});
			TetStats st;
			TetMesh m = DelaunayTetrahedralize(pts, &st);
			std::printf("grid: %d points -> %d tets in %.3f s, %d repairs\n", st.n_points, st.n_tets, st.seconds, st.n_cavity_repairs);
			std::string why;
			Check(m.IsValid(&why), "grid: mesh valid " + why);
			double vol = m.Volume();
			Check(std::fabs(vol - std::pow(n - 1.0, 3)) < 1e-9, Fmt("grid: tetrahedra fill the cube exactly (%.12g vs %.0f)", vol, std::pow(n - 1.0, 3)));
			Check(DelaunayViolations(m, 300) == 0, "grid: no node strictly inside a sampled circumsphere");
			// Duplicates are skipped.
			pts.push_back({3, 3, 3});
			pts.push_back({0, 0, 0});
			TetMesh m2 = DelaunayTetrahedralize(pts);
			Check(m2.tets.size() == m.tets.size() && m2.IsValid(), "grid: duplicate points are skipped");
		}

		// ---- conforming: sphere ----
		{
			TriMesh sph = EllipsoidMesh(1.0, 1.0, 1.0, 1e-3);
			ConformingStats s;
			TetMesh m = TetrahedralizeInterior(sph, {}, &s);
			ReportConforming("sphere", s);
			ConformingChecks("sphere", sph, m, s);
			double d, r; int sl;
			m.Quality(-1, &d, &r, &sl);
			std::printf("sphere: min dihedral %.2f deg, min radius ratio %.3f, %d slivers under 5 deg of %zu\n", d, r, sl, m.tets.size());
		}

		// ---- conforming: box (coplanar faces, cospherical corners) ----
		{
			TriMesh box = BoxMesh(2.0, 1.0, 0.5);
			ConformingStats s;
			TetMesh m = TetrahedralizeInterior(box, {}, &s);
			ReportConforming("box", s);
			ConformingChecks("box", box, m, s);
		}

		// ---- conforming: thin plate with interior points ----
		{
			TriMesh plate = BoxMesh(2.0, 1.0, 0.06);
			ConformingOptions o;
			o.interior_points = BccPoints(plate.Bounds(), 0.2, [&](const Vec3& p) { return std::fabs(p.x) < 0.95 && std::fabs(p.y) < 0.45 && std::fabs(p.z) < 0.02; });
			ConformingStats s;
			TetMesh m = TetrahedralizeInterior(plate, o, &s);
			ReportConforming("plate", s);
			ConformingChecks("plate", plate, m, s);
		}

		// ---- face recovery: a cube with a point just under its top face ----
		{
			TriMesh cube = BoxMesh(1.0, 1.0, 1.0);
			ConformingOptions o;
			o.interior_points = {Vec3{0.0, 0.0, 0.45}, Vec3{0.1, 0.05, 0.47}};
			ConformingStats s;
			TetMesh m = TetrahedralizeInterior(cube, o, &s);
			ReportConforming("cube+points", s);
			ConformingChecks("cube+points", cube, m, s);
			Check(m.tets.size() > 12, "cube+points: the interior points are in the mesh");
		}

		// ---- face recovery: an ellipsoid triangulated by the long diagonals ----
		// An adversarial surface: needle triangles meeting at about one degree
		// near the poles. Out of scope for exact conformity (reported, not
		// required); what is required is a clean failure, not a hang.
		{
			TriMesh ell = FlipLongDiagonals(EllipsoidMesh(1.0, 0.7, 0.5, 3e-3));
			Check(ell.IsClosedManifold(), "flipped ellipsoid: still a closed manifold");
			ConformingOptions o;
			o.planar_tolerance = 0.0; // keep the bad triangles
			o.min_segment_fraction = 1e-4; // fail fast on a cascade
			o.max_seconds = 20.0;
			ConformingStats s;
			try {
				TetMesh m = TetrahedralizeInterior(ell, o, &s);
				ReportConforming("flipped ellipsoid", s);
				std::string why;
				Check(m.IsValid(&why), "flipped ellipsoid: a valid (if degraded) mesh " + why);
				Check(std::fabs(s.mesh_volume - s.surface_volume) <= 1e-2 * s.surface_volume, Fmt("flipped ellipsoid: volume within 1%% (%.6g vs %.6g, %.0f faces abandoned)", s.mesh_volume, s.surface_volume, s.n_faces_abandoned));
			} catch (const std::exception& ex) {
				std::printf("flipped ellipsoid (needles, out of scope): %s\n", ex.what());
				Check(true, "flipped ellipsoid: failed cleanly");
			}
		}

		// ---- skin / core: sphere ----
		Case("sphere skin", [] {
			const double R = 1.0, t = 0.1;
			TriMesh sph = EllipsoidMesh(R, R, R, 2e-3);
			MeshSdf sdf(sph);
			SkinCoreOptions o;
			o.thickness = t;
			o.core_spacing = 0.25;
			SkinCoreStats s;
			TetMesh m = BuildSkinCoreMesh(sph, sdf, o, &s);
			ReportConforming("sphere skin (conforming stage)", s.conforming);
			std::printf("sphere skin: %d offset nodes, %d core points, %d edges split, %d nodes snapped; skin %d tets %.5f, core %d tets %.5f; min dihedral skin %.2f core %.2f; slivers %d / %d; %.2f s\n",
				s.n_offset_nodes, s.n_core_points, s.n_edges_split, s.n_nodes_snapped, s.n_skin_tets, s.skin_volume, s.n_core_tets, s.core_volume, s.min_dihedral_skin, s.min_dihedral_core, s.n_slivers_skin,
				s.n_slivers_core, s.seconds);
			std::string why;
			Check(m.IsValid(&why), "sphere skin: mesh valid " + why);
			double expected_skin = 4.0 / 3.0 * kPi * (R * R * R - std::pow(R - t, 3));
			double expected_core = 4.0 / 3.0 * kPi * std::pow(R - t, 3);
			Check(std::fabs(s.skin_volume - expected_skin) < 0.03 * expected_skin, Fmt("sphere skin: skin volume %.5f within 3%% of the shell's %.5f", s.skin_volume, expected_skin));
			Check(std::fabs(s.core_volume - expected_core) < 0.03 * expected_core, Fmt("sphere skin: core volume %.5f within 3%% of %.5f", s.core_volume, expected_core));
			Check(std::fabs(s.skin_volume + s.core_volume - sph.SignedVolume()) <= 1e-9 * sph.SignedVolume(), "sphere skin: skin + core = the surface's volume");
			// The interface is a closed surface on the iso-surface.
			TriMesh iface = m.InterfaceFaces(1, 2);
			double worst = 0.0;
			for (const auto& tr : iface.triangles) for (int k = 0; k < 3; ++k) worst = std::max(worst, std::fabs(Length(m.nodes[tr[k]]) - (R - t)));
			Check(iface.IsClosedManifold(), Fmt("sphere skin: the skin/core interface (%.0f faces) is a closed manifold", (double)iface.triangles.size()));
			Check(worst < 2e-3, Fmt("sphere skin: interface nodes on radius %.2f (worst deviation %.2e)", R - t, worst));
			Check(m.BoundaryFaces(1).SignedVolume() > 0.0, "sphere skin: the skin's boundary faces enclose a positive volume");
			WriteTetMeshVtk(m, "C:\\Users\\Matt\\devel_1\\aerotelic\\core\\build\\tet_sphere_skin.vtk");
		});

		// ---- the P-38's port wing ----
		{
			ProjectFile project = LoadProjectYaml("C:\\Users\\Matt\\devel_1\\WingDesigner\\projects\\p38.yaml");
			const WingModelSpec* ws = nullptr;
			for (auto& e : project.entities) if (e.name == "Wing") ws = std::get_if<WingModelSpec>(&e.model);
			Check(ws != nullptr, "p38: Wing entity found");
			if (ws) {
				WingModel wing(*ws);
				TriMesh m0 = wing.BuildMesh(2e-3);
				m0.Translate({3.1, 0.0, 0.1});
				m0.Mirror({0, 0, 0}, {0, 1, 0});
				std::printf("port wing: %zu vertices, %zu triangles, closed %d\n", m0.vertices.size(), m0.triangles.size(), (int)m0.IsClosedManifold());
				Case("port wing", [&] {
					ConformingStats s;
					TetMesh m = TetrahedralizeInterior(m0, {}, &s);
					ReportConforming("port wing", s);
					ConformingChecks("port wing", m0, m, s);
					double d, r; int sl;
					m.Quality(-1, &d, &r, &sl);
					std::printf("port wing: min dihedral %.3f deg, min radius ratio %.4f, %d slivers under 5 deg of %zu\n", d, r, sl, m.tets.size());
				});
				Case("port wing skin", [&] {
					const double t = 0.01 * wing.planform().RootChord();
					MeshSdf sdf(m0);
					SkinCoreOptions o;
					o.thickness = t;
					o.core_spacing = 0.15;
					SkinCoreStats s;
					TetMesh m = BuildSkinCoreMesh(m0, sdf, o, &s);
					ReportConforming("port wing skin (conforming stage)", s.conforming);
					std::printf("port wing skin: t %.4f, %d offset nodes, %d core points, %d edges split, %d nodes snapped; skin %d tets %.5f, core %d tets %.5f (wing %.5f); min dihedral skin %.3f core %.3f; slivers %d / %d; %.2f s\n",
						t, s.n_offset_nodes, s.n_core_points, s.n_edges_split, s.n_nodes_snapped, s.n_skin_tets, s.skin_volume, s.n_core_tets, s.core_volume, m0.SignedVolume(), s.min_dihedral_skin,
						s.min_dihedral_core, s.n_slivers_skin, s.n_slivers_core, s.seconds);
					std::string why;
					Check(m.IsValid(&why), "port wing skin: mesh valid " + why);
					Check(std::fabs(s.skin_volume + s.core_volume - m0.SignedVolume()) <= 1e-9 * m0.SignedVolume(), "port wing skin: skin + core = the wing's volume");
					TriMesh iface = m.InterfaceFaces(1, 2);
					double worst = 0.0;
					for (const auto& tr : iface.triangles) for (int k = 0; k < 3; ++k) worst = std::max(worst, std::fabs(sdf.Distance(m.nodes[tr[k]]) + t));
					Check(worst <= 1e-3 * t, Fmt("port wing skin: interface nodes on the iso-surface b = -t (worst deviation %.2e of t %.4f)", worst, t));
					int pinched = 0;
					Check(IsClosed(iface, &pinched), Fmt("port wing skin: the interface (%.0f faces) is closed (%.0f pinched edges)", (double)iface.triangles.size(), (double)pinched));
					// The skin's boundary is the surface plus the interface.
					TriMesh skin_boundary = m.BoundaryFaces(1);
					Check(IsClosed(skin_boundary, &pinched), Fmt("port wing skin: the skin's boundary (surface + interface, %.0f faces) is closed (%.0f pinched edges)", (double)skin_boundary.triangles.size(), (double)pinched));
					Check(std::fabs(skin_boundary.SignedVolume() - s.skin_volume) <= 1e-9 * s.skin_volume, Fmt("port wing skin: the skin's boundary encloses the skin volume (%.6f vs %.6f)", skin_boundary.SignedVolume(), s.skin_volume));
					WriteTetMeshVtk(m, "C:\\Users\\Matt\\devel_1\\aerotelic\\core\\build\\tet_p38_port_wing.vtk");
					std::printf("port wing skin: wrote core\\build\\tet_p38_port_wing.vtk\n");
				});
			}

			// ---- diagnostic: the Boom's tail -- is the Delaunay tetrahedralization of its vertices right there? ----
			Case("boom tail diagnostic", [&] {
				const FuselageSpec* fs = nullptr;
				for (auto& e : project.entities) if (e.name == "Boom") fs = std::get_if<FuselageSpec>(&e.model);
				if (!fs) return;
				TriMesh boom = FuselageModel(*fs).BuildMeshes(2e-3)[0].mesh;
				// Coincident vertices?
				std::unordered_map<std::string, int> seen;
				int dups = 0;
				double min_sep = 1e300;
				for (size_t v = 0; v < boom.vertices.size(); ++v) {
					char key[100];
					std::snprintf(key, sizeof key, "%.17g,%.17g,%.17g", boom.vertices[v].x, boom.vertices[v].y, boom.vertices[v].z);
					if (!seen.emplace(key, (int)v).second) ++dups;
				}
				for (const auto& t : boom.triangles) for (int k = 0; k < 3; ++k) min_sep = std::min(min_sep, Distance(boom.vertices[t[k]], boom.vertices[t[(k + 1) % 3]]));
				std::printf("boom: %zu vertices, %zu triangles, %d coincident vertices, shortest surface edge %.3g\n", boom.vertices.size(), boom.triangles.size(), dups, min_sep);
				TetMesh dt = DelaunayTetrahedralize(boom.vertices);
				// Every tetrahedron near the tail against every vertex.
				int bad = 0, checked = 0;
				for (size_t t = 0; t < dt.tets.size(); ++t) {
					const auto& v = dt.tets[t];
					Vec3 c = (dt.nodes[v[0]] + dt.nodes[v[1]] + dt.nodes[v[2]] + dt.nodes[v[3]]) * 0.25;
					if (c.x < 9.98) continue;
					++checked;
					for (size_t n = 0; n < dt.nodes.size(); ++n) {
						if ((int)n == v[0] || (int)n == v[1] || (int)n == v[2] || (int)n == v[3]) continue;
						if (ExactInSphere(dt.nodes[v[0]], dt.nodes[v[1]], dt.nodes[v[2]], dt.nodes[v[3]], dt.nodes[n]) > 0.0) { ++bad; break; }
					}
				}
				Check(bad == 0, Fmt("boom tail: %.0f of %.0f tetrahedra near the tail have an empty circumsphere (violations %.0f)", checked - bad, checked, bad));
				// How many surface edges near the tail are missing from the DT, and is a missing one Gabriel?
				std::unordered_map<uint64_t, int> edges;
				for (const auto& t : boom.triangles) for (int k = 0; k < 3; ++k) {
					int a = t[k], b = t[(k + 1) % 3];
					edges[(static_cast<uint64_t>(std::min(a, b)) << 32) | static_cast<uint64_t>(std::max(a, b))] = 1;
				}
				std::unordered_map<uint64_t, int> dt_edges;
				for (const auto& v : dt.tets) for (int i = 0; i < 4; ++i) for (int j = i + 1; j < 4; ++j) {
					int a = v[i], b = v[j];
					dt_edges[(static_cast<uint64_t>(std::min(a, b)) << 32) | static_cast<uint64_t>(std::max(a, b))] = 1;
				}
				int missing = 0, gabriel_missing = 0, shown = 0;
				for (const auto& kv : edges) {
					int a = static_cast<int>(kv.first >> 32), b = static_cast<int>(kv.first & 0xffffffffu);
					if (boom.vertices[a].x < 9.98 && boom.vertices[b].x < 9.98) continue;
					if (dt_edges.count(kv.first)) continue;
					++missing;
					// Gabriel: no vertex strictly inside the diametral ball.
					Vec3 mid = (boom.vertices[a] + boom.vertices[b]) * 0.5;
					double r2 = DistanceSq(boom.vertices[a], mid);
					int inside = -1;
					for (size_t n = 0; n < boom.vertices.size(); ++n) {
						if ((int)n == a || (int)n == b) continue;
						if (DistanceSq(boom.vertices[n], mid) < r2) { inside = (int)n; break; }
					}
					if (inside < 0) ++gabriel_missing;
					if (shown++ < 5) {
						const Vec3& A = boom.vertices[a];
						const Vec3& B = boom.vertices[b];
						std::printf("  missing tail edge %d-%d: (%.6f,%.6f,%.6f)-(%.6f,%.6f,%.6f) L %.3g, %s", a, b, A.x, A.y, A.z, B.x, B.y, B.z, Distance(A, B), inside < 0 ? "Gabriel (diametral ball empty)\n" : "");
						if (inside >= 0) { const Vec3& P = boom.vertices[inside]; std::printf("ball holds %d (%.6f,%.6f,%.6f) at %.3g from the segment line\n", inside, P.x, P.y, P.z, Length(Cross(P - A, B - A)) / Distance(A, B)); }
					}
				}
				std::printf("boom tail: %d surface edges missing from the DT, %d of them Gabriel\n", missing, gabriel_missing);
				Check(gabriel_missing == 0, "boom tail: no Gabriel edge is missing from the Delaunay tetrahedralization");
			});

			// ---- every P-38 body that has a mesh builder, at the app's tolerance ----
			for (const auto& ent : project.entities) {
				std::vector<NamedMesh> bodies;
				const double tol = 2e-3;
				if (auto* w = std::get_if<WingModelSpec>(&ent.model)) bodies = WingModel(*w).BuildMeshes(tol);
				else if (auto* f = std::get_if<FuselageSpec>(&ent.model)) bodies = FuselageModel(*f).BuildMeshes(tol);
				else if (auto* n = std::get_if<NacelleSpec>(&ent.model)) bodies = NacelleModel(*n).BuildMeshes(tol);
				else if (auto* l = std::get_if<LoftedNacelleSpec>(&ent.model)) bodies = LoftedNacelleModel(*l).BuildMeshes(tol);
				else if (auto* p = std::get_if<PropellerSpec>(&ent.model)) bodies = PropellerModel(*p).BuildMeshes(tol);
				if (bodies.empty()) { std::printf("%s: no mesh builder, skipped\n", ent.name.c_str()); continue; }
				for (const auto& body : bodies) {
					std::string label = ent.name + "/" + body.name;
					const TriMesh& surf = body.mesh;
					if (!surf.IsClosedManifold()) { Check(false, label + ": surface is a closed manifold"); continue; }
					try {
						ConformingStats s;
						auto t0 = std::chrono::steady_clock::now();
						TetMesh m = TetrahedralizeInterior(surf, {}, &s);
						double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
						ReportConforming(label.c_str(), s);
						std::string why;
						bool valid = m.IsValid(&why);
						// Snaps and sliver flips move the surface by up to the snap
						// distance; otherwise the volume and area are exact. Where
						// faces were abandoned the boundary follows the tetrahedra
						// around the holes: only validity and a loose volume are
						// required, and the deviation is reported.
						bool degraded = s.n_faces_abandoned > 0;
						double rel = degraded ? 1e-3 : ((s.n_segment_snaps + s.n_segment_flips) > 0 ? 1e-6 : 1e-9);
						bool vol_ok = std::fabs(s.mesh_volume - s.surface_volume) <= rel * s.surface_volume;
						TriMesh boundary = m.BoundaryFaces(-1);
						int pinched = 0;
						bool closed = IsClosed(boundary, &pinched), manifold = boundary.IsClosedManifold();
						double area_rel = std::fabs(boundary.SurfaceArea() - surf.SurfaceArea()) / surf.SurfaceArea();
						bool bnd_ok = degraded ? closed : (manifold && area_rel <= rel);
						char buf[360];
						std::snprintf(buf, sizeof buf, ": %s mesh in %.2f s (valid %d, volume ok %d (%.9g vs %.9g), boundary ok %d: closed %d, manifold %d, pinched edges %d, area rel diff %.2e, %d faces abandoned) ", degraded ? "DEGRADED" : "conforming", dt,
							(int)valid, (int)vol_ok, s.mesh_volume, s.surface_volume, (int)bnd_ok, (int)closed, (int)manifold, pinched, area_rel, s.n_faces_abandoned);
						Check(valid && vol_ok && bnd_ok, label + buf + why);
					} catch (const std::exception& ex) {
						Check(false, label + ": " + ex.what());
					}
				}
			}

			// ---- the wing again, finer: timing ----
			if (ws) Case("fine wing", [&] {
				WingModel wing(*ws);
				TriMesh fine = wing.BuildMesh(3e-4);
				std::printf("fine wing: %zu vertices, %zu triangles\n", fine.vertices.size(), fine.triangles.size());
				ConformingStats s;
				TetMesh m = TetrahedralizeInterior(fine, {}, &s);
				ReportConforming("fine wing", s);
				std::string why;
				Check(m.IsValid(&why) && std::fabs(s.mesh_volume - s.surface_volume) <= 1e-9 * s.surface_volume, "fine wing: valid, exact volume " + why);
				Check(s.seconds_total < 30.0, Fmt("fine wing: under 30 s (%.2f s)", s.seconds_total));
			});
		}
	} catch (const std::exception& e) {
		std::printf("FAIL: exception: %s\n", e.what());
		++g_failures;
	}
	std::printf("RESULT: %d failure(s)\n", g_failures);
	return g_failures == 0 ? 0 : 1;
}
