// Step 5 of tet.hpp: the skin / core partition of the interior mesh, the
// body-centred cubic core points, and the VTK writer.
#include "aerotelic/tet.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <unordered_set>

#include "aerotelic/shell.hpp"

namespace aerotelic {

namespace {

inline uint64_t EdgeKey(int a, int b) {
	if (a > b) std::swap(a, b);
	return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) | static_cast<uint32_t>(b);
}

// The interface-splitting state over a TetMesh.
class Splitter {
public:
	TetMesh& m;
	const MeshSdf& sdf;
	const SkinCoreOptions& opt;
	const double t;
	const double eps;
	std::vector<double> b;          // field at each node (0 on the surface by definition)
	std::vector<char> on_surface;   // node of a constrained face
	std::vector<char> fixed;        // placed on the iso-surface: never moved again
	std::vector<std::vector<int>> vt; // node -> tetrahedra
	std::vector<char> dead;
	int n_split = 0, n_snapped = 0;

	Splitter(TetMesh& mesh, const MeshSdf& s, const SkinCoreOptions& o) : m(mesh), sdf(s), opt(o), t(o.thickness), eps(o.crossing_tolerance * o.thickness) {
		size_t n = m.nodes.size();
		on_surface.assign(n, 0);
		for (size_t k = 0; k < m.tets.size(); ++k) {
			for (int i = 0; i < 4; ++i) {
				if (!(m.constrained[k] & (1u << i))) continue;
				for (int j = 0; j < 3; ++j) on_surface[m.tets[k][TetMesh::kFace[i][j]]] = 1;
			}
		}
		b.resize(n);
		for (size_t v = 0; v < n; ++v) b[v] = on_surface[v] ? 0.0 : sdf.Distance(m.nodes[v]);
		fixed.assign(n, 0);
		dead.assign(m.tets.size(), 0);
		vt.assign(n, {});
		for (size_t k = 0; k < m.tets.size(); ++k) for (int i = 0; i < 4; ++i) vt[m.tets[k][i]].push_back(static_cast<int>(k));
	}

	int Side(int v) const {
		if (fixed[v]) return 0;
		if (b[v] > -t + eps) return 1;
		if (b[v] < -t - eps) return -1;
		return 0;
	}

	// The point of the segment u-v where the field crosses -t.
	Vec3 Crossing(int u, int v, double* s_out) const {
		const Vec3& A = m.nodes[u];
		const Vec3& B = m.nodes[v];
		double lo = 0.0, hi = 1.0; // field + t positive at lo, negative at hi
		if (b[u] < b[v]) std::swap(lo, hi);
		for (int it = 0; it < 60; ++it) {
			double mid = 0.5 * (lo + hi);
			double f = sdf.Distance(Lerp(A, B, mid)) + t;
			if (std::fabs(f) <= 0.1 * eps) { lo = hi = mid; break; }
			if (f > 0.0) lo = mid; else hi = mid;
			if (std::fabs(hi - lo) * Distance(A, B) < 1e-12 * t) break;
		}
		double s = 0.5 * (lo + hi);
		*s_out = s;
		return Lerp(A, B, s);
	}

	bool CanMove(int v, const Vec3& to) const {
		for (int k : vt[v]) {
			if (dead[k]) continue;
			std::array<Vec3, 4> p;
			for (int i = 0; i < 4; ++i) p[i] = (m.tets[k][i] == v) ? to : m.nodes[m.tets[k][i]];
			if (ExactOrient(p[0], p[1], p[2], p[3]) <= 0.0) return false;
		}
		return true;
	}

	void RemoveIncidence(int v, int k) {
		auto& lst = vt[v];
		lst.erase(std::remove(lst.begin(), lst.end(), k), lst.end());
	}

	int AddTet(const std::array<int, 4>& v, uint8_t cons) {
		int k = static_cast<int>(m.tets.size());
		m.tets.push_back(v);
		m.constrained.push_back(cons);
		m.material.push_back(0);
		dead.push_back(0);
		for (int i = 0; i < 4; ++i) vt[v[i]].push_back(k);
		return k;
	}

	void SplitEdge(int u, int v, const Vec3& x) {
		int xi = static_cast<int>(m.nodes.size());
		m.nodes.push_back(x);
		b.push_back(-t);
		on_surface.push_back(0);
		fixed.push_back(1);
		vt.push_back({});
		std::vector<int> around;
		for (int k : vt[u]) {
			if (dead[k]) continue;
			const auto& tv = m.tets[k];
			if (tv[0] == v || tv[1] == v || tv[2] == v || tv[3] == v) around.push_back(k);
		}
		for (int k : around) {
			std::array<int, 4> tv = m.tets[k];
			uint8_t cons = m.constrained[k];
			int iu = -1, iv = -1;
			for (int i = 0; i < 4; ++i) { if (tv[i] == u) iu = i; if (tv[i] == v) iv = i; }
			dead[k] = 1;
			for (int i = 0; i < 4; ++i) RemoveIncidence(tv[i], k);
			std::array<int, 4> t1 = tv, t2 = tv;
			t1[iu] = xi; // keeps face iu (the one without u) and its bit
			t2[iv] = xi; // keeps face iv
			AddTet(t1, static_cast<uint8_t>(cons & (1u << iu)));
			AddTet(t2, static_cast<uint8_t>(cons & (1u << iv)));
		}
		++n_split;
	}

	// One pass over every straddling edge. Returns how many were handled.
	int Pass() {
		std::unordered_set<uint64_t> seen;
		std::vector<std::pair<int, int>> edges;
		static const int kEdges[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
		for (size_t k = 0; k < m.tets.size(); ++k) {
			if (dead[k]) continue;
			for (const auto& ed : kEdges) {
				int u = m.tets[k][ed[0]], v = m.tets[k][ed[1]];
				if (Side(u) * Side(v) >= 0) continue;
				if (seen.insert(EdgeKey(u, v)).second) edges.push_back({u, v});
			}
		}
		int handled = 0;
		for (const auto& e : edges) {
			int u = e.first, v = e.second;
			if (Side(u) * Side(v) >= 0) continue; // a snap resolved it meanwhile
			double s;
			Vec3 x = Crossing(u, v, &s);
			bool done = false;
			if (s < opt.snap_fraction && !on_surface[u] && !fixed[u] && CanMove(u, x)) {
				m.nodes[u] = x; b[u] = -t; fixed[u] = 1; ++n_snapped; done = true;
			} else if (1.0 - s < opt.snap_fraction && !on_surface[v] && !fixed[v] && CanMove(v, x)) {
				m.nodes[v] = x; b[v] = -t; fixed[v] = 1; ++n_snapped; done = true;
			}
			if (!done) SplitEdge(u, v, x);
			++handled;
		}
		return handled;
	}

	void Compact() {
		std::vector<std::array<int, 4>> tets;
		std::vector<uint8_t> cons;
		for (size_t k = 0; k < m.tets.size(); ++k) {
			if (dead[k]) continue;
			tets.push_back(m.tets[k]);
			cons.push_back(m.constrained[k]);
		}
		m.tets = std::move(tets);
		m.constrained = std::move(cons);
		m.material.assign(m.tets.size(), 0);
		m.RebuildNeighbors();
		dead.assign(m.tets.size(), 0);
	}

	void Classify() {
		for (size_t k = 0; k < m.tets.size(); ++k) {
			int plus = 0, minus = 0;
			for (int i = 0; i < 4; ++i) { int s = Side(m.tets[k][i]); if (s > 0) ++plus; else if (s < 0) ++minus; }
			if (plus && minus) throw std::logic_error("tet: a tetrahedron still straddles the skin iso-surface");
			if (plus) m.material[k] = 1;
			else if (minus) m.material[k] = 2;
			else {
				Vec3 c = (m.nodes[m.tets[k][0]] + m.nodes[m.tets[k][1]] + m.nodes[m.tets[k][2]] + m.nodes[m.tets[k][3]]) * 0.25;
				m.material[k] = sdf.Distance(c) > -t ? 1 : 2;
			}
		}
	}
};

} // namespace

std::vector<Vec3> BccPoints(const Box3& box, double h, const std::function<bool(const Vec3&)>& keep) {
	std::vector<Vec3> out;
	if (h <= 0.0 || box.IsEmpty()) return out;
	Vec3 ext = box.Extent();
	int nx = static_cast<int>(std::floor(ext.x / h)) + 1, ny = static_cast<int>(std::floor(ext.y / h)) + 1, nz = static_cast<int>(std::floor(ext.z / h)) + 1;
	for (int i = 0; i <= nx; ++i) for (int j = 0; j <= ny; ++j) for (int k = 0; k <= nz; ++k) {
		Vec3 p = box.min + Vec3{i * h, j * h, k * h};
		if (keep(p)) out.push_back(p);
		Vec3 q = p + Vec3{0.5 * h, 0.5 * h, 0.5 * h};
		if (keep(q)) out.push_back(q);
	}
	return out;
}

TetMesh BuildSkinCoreMesh(const TriMesh& surface, const MeshSdf& sdf, const SkinCoreOptions& opt, SkinCoreStats* stats_out) {
	if (opt.thickness <= 0.0) throw std::invalid_argument("BuildSkinCoreMesh: the skin thickness must be positive");
	auto t_start = std::chrono::steady_clock::now();
	SkinCoreStats st;
	const double t = opt.thickness;
	ConformingOptions conf = opt.conforming;
	conf.interior_points.clear();
	// Offset nodes.
	if (opt.offset_nodes) {
		ShellOptions so;
		so.thickness = t;
		ShellResult shell = BuildOffsetShell(surface, sdf, so);
		std::vector<char> taken(shell.nodes.size(), 0);
		for (int v = 0; v < shell.n_surface_nodes; ++v) {
			int o = shell.offset_node[v];
			if (o < shell.n_surface_nodes || taken[o]) continue;
			if (!(shell.depth[v] < 0.0)) continue;
			taken[o] = 1;
			conf.interior_points.push_back(shell.nodes[o]);
		}
		st.n_offset_nodes = static_cast<int>(conf.interior_points.size());
	}
	// Core points.
	if (opt.core_spacing > 0.0) {
		double h = opt.core_spacing;
		std::vector<Vec3> core = BccPoints(surface.Bounds(), h, [&](const Vec3& p) { return sdf.Distance(p) < -(t + 0.5 * h); });
		st.n_core_points = static_cast<int>(core.size());
		conf.interior_points.insert(conf.interior_points.end(), core.begin(), core.end());
	}
	// Steps 1-4.
	TetMesh m = TetrahedralizeInterior(surface, conf, &st.conforming);
	// Step 5.
	Splitter sp(m, sdf, opt);
	for (int pass = 0; pass < 8; ++pass) {
		if (sp.Pass() == 0) break;
		if (pass == 7) throw std::runtime_error("tet: interface splitting did not converge");
	}
	sp.Compact();
	sp.Classify();
	st.n_edges_split = sp.n_split;
	st.n_nodes_snapped = sp.n_snapped;
	st.n_skin_tets = m.Count(1);
	st.n_core_tets = m.Count(2);
	st.skin_volume = m.Volume(1);
	st.core_volume = m.Volume(2);
	m.Quality(1, &st.min_dihedral_skin, &st.min_radius_ratio_skin, &st.n_slivers_skin);
	m.Quality(2, &st.min_dihedral_core, &st.min_radius_ratio_core, &st.n_slivers_core);
	st.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
	if (stats_out) *stats_out = st;
	return m;
}

bool WriteTetMeshVtk(const TetMesh& m, const std::string& path) {
	FILE* f = std::fopen(path.c_str(), "w");
	if (!f) return false;
	std::fprintf(f, "# vtk DataFile Version 3.0\nAerotelic tetrahedral mesh\nASCII\nDATASET UNSTRUCTURED_GRID\n");
	std::fprintf(f, "POINTS %zu double\n", m.nodes.size());
	for (const auto& p : m.nodes) std::fprintf(f, "%.17g %.17g %.17g\n", p.x, p.y, p.z);
	std::fprintf(f, "CELLS %zu %zu\n", m.tets.size(), m.tets.size() * 5);
	for (const auto& t : m.tets) std::fprintf(f, "4 %d %d %d %d\n", t[0], t[1], t[2], t[3]);
	std::fprintf(f, "CELL_TYPES %zu\n", m.tets.size());
	for (size_t i = 0; i < m.tets.size(); ++i) std::fprintf(f, "10\n");
	std::fprintf(f, "CELL_DATA %zu\nSCALARS material int 1\nLOOKUP_TABLE default\n", m.tets.size());
	for (size_t i = 0; i < m.tets.size(); ++i) std::fprintf(f, "%d\n", m.material.empty() ? 0 : m.material[i]);
	std::fclose(f);
	return true;
}

} // namespace aerotelic
