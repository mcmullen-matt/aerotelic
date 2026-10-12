// Mesh improvement (tet.hpp, ImproveTetMesh): sliver removal by 2-3 and
// 3-2 flips, and optimisation smoothing of the free nodes, on a finished
// TetMesh. Everything here is local: a flip replaces two or three
// tetrahedra by three or two that tile the same region, relinking only
// the faces around them; a smoothing step moves one node and re-checks
// its own tetrahedra. Constrained faces (the body surface) and the faces
// between two materials (the skin/core interface) are never flipped
// across, and their nodes never move, so the surface and the interface
// are exactly what the mesher produced.
#include "aerotelic/tet.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

namespace aerotelic {

namespace {

constexpr double kPi = 3.14159265358979323846;
const int kEdges[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
const int kOpp[6][2] = {{2, 3}, {1, 3}, {1, 2}, {0, 3}, {0, 2}, {0, 1}};

struct FaceKey {
	int a, b, c;
	FaceKey(int x, int y, int z) {
		if (x > y) std::swap(x, y);
		if (y > z) std::swap(y, z);
		if (x > y) std::swap(x, y);
		a = x; b = y; c = z;
	}
	bool operator==(const FaceKey& o) const { return a == o.a && b == o.b && c == o.c; }
};
struct FaceKeyHash {
	size_t operator()(const FaceKey& k) const {
		uint64_t h = static_cast<uint64_t>(static_cast<uint32_t>(k.a));
		h = h * 0x9E3779B97F4A7C15ull ^ static_cast<uint64_t>(static_cast<uint32_t>(k.b));
		h = h * 0x9E3779B97F4A7C15ull ^ static_cast<uint64_t>(static_cast<uint32_t>(k.c));
		return static_cast<size_t>(h ^ (h >> 29));
	}
};

// Smallest dihedral angle (degrees) of the tetrahedron with these nodes.
double MinDihedral(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3) {
	const Vec3 p[4] = {p0, p1, p2, p3};
	double worst = 180.0;
	for (int k = 0; k < 6; ++k) {
		const Vec3& a = p[kEdges[k][0]];
		const Vec3& b = p[kEdges[k][1]];
		const Vec3& c = p[kOpp[k][0]];
		const Vec3& d = p[kOpp[k][1]];
		Vec3 n1 = Cross(b - a, c - a), n2 = Cross(b - a, d - a);
		double l1 = Length(n1), l2 = Length(n2);
		if (l1 == 0.0 || l2 == 0.0) return 0.0;
		double cosang = std::clamp(Dot(n1, n2) / (l1 * l2), -1.0, 1.0);
		worst = std::min(worst, std::acos(cosang) * 180.0 / kPi);
	}
	return worst;
}

class Improver {
public:
	TetMesh& m;
	const ImproveOptions& opt;
	ImproveStats& st;
	std::vector<std::vector<int>> vt; // node -> live tetrahedra
	std::vector<char> fixed;          // node on the surface or an interface
	std::vector<char> dead;

	Improver(TetMesh& mesh, const ImproveOptions& o, ImproveStats& s) : m(mesh), opt(o), st(s) {
		const size_t n = m.nodes.size();
		fixed.assign(n, 0);
		vt.assign(n, {});
		dead.assign(m.tets.size(), 0);
		for (size_t t = 0; t < m.tets.size(); ++t) {
			for (int k = 0; k < 4; ++k) vt[m.tets[t][k]].push_back(static_cast<int>(t));
			for (int i = 0; i < 4; ++i) if (FaceLocked(static_cast<int>(t), i)) for (int k = 0; k < 3; ++k) fixed[m.tets[t][TetMesh::kFace[i][k]]] = 1;
		}
	}

	// A face that must stay: on the boundary, constrained, or between materials.
	bool FaceLocked(int t, int i) const {
		int nb = m.neighbors[t][i];
		if (nb < 0) return true;
		if (m.constrained[t] & (1u << i)) return true;
		return m.material[t] != m.material[nb];
	}

	double Dihedral(int t) const {
		const auto& v = m.tets[t];
		return MinDihedral(m.nodes[v[0]], m.nodes[v[1]], m.nodes[v[2]], m.nodes[v[3]]);
	}
	double Dihedral(const std::array<int, 4>& v) const { return MinDihedral(m.nodes[v[0]], m.nodes[v[1]], m.nodes[v[2]], m.nodes[v[3]]); }

	// Orients (a, b, c, d) positively by swapping c and d if needed; false if flat.
	bool Oriented(std::array<int, 4>& v) const {
		double o = ExactOrient(m.nodes[v[0]], m.nodes[v[1]], m.nodes[v[2]], m.nodes[v[3]]);
		if (o > 0.0) return true;
		if (o < 0.0) { std::swap(v[2], v[3]); return true; }
		return false;
	}

	int FaceIndex(int t, int a, int b, int c) const {
		const auto& v = m.tets[t];
		for (int i = 0; i < 4; ++i) {
			int o = v[i];
			if (o == a || o == b || o == c) continue;
			return i;
		}
		return -1;
	}

	// Replaces the tetrahedra `old` by `fresh` (tiling the same region, with
	// the same material), relinking the faces around them.
	void Replace(const std::vector<int>& old, const std::vector<std::array<int, 4>>& fresh, int material) {
		std::unordered_map<FaceKey, std::pair<int, int>, FaceKeyHash> outer; // key -> (outer tet, its face), or (-1, bit) for a boundary
		std::unordered_map<FaceKey, uint8_t, FaceKeyHash> outer_bit;
		for (int o : old) {
			for (int i = 0; i < 4; ++i) {
				int nb = m.neighbors[o][i];
				bool interior = false;
				for (int p : old) if (p == nb) interior = true;
				if (interior) continue;
				const auto& v = m.tets[o];
				FaceKey key(v[TetMesh::kFace[i][0]], v[TetMesh::kFace[i][1]], v[TetMesh::kFace[i][2]]);
				int j = -1;
				if (nb >= 0) for (int f = 0; f < 4; ++f) if (m.neighbors[nb][f] == o) { j = f; break; }
				outer[key] = {nb, j};
				outer_bit[key] = static_cast<uint8_t>((m.constrained[o] >> i) & 1u);
			}
		}
		for (int o : old) {
			dead[o] = 1;
			for (int k = 0; k < 4; ++k) {
				auto& lst = vt[m.tets[o][k]];
				lst.erase(std::remove(lst.begin(), lst.end(), o), lst.end());
			}
		}
		std::vector<int> made;
		for (const auto& f : fresh) {
			int slot = static_cast<int>(m.tets.size());
			m.tets.push_back(f);
			m.neighbors.push_back({{-1, -1, -1, -1}});
			m.constrained.push_back(0);
			m.material.push_back(material);
			dead.push_back(0);
			for (int k = 0; k < 4; ++k) vt[f[k]].push_back(slot);
			made.push_back(slot);
		}
		std::unordered_map<FaceKey, std::pair<int, int>, FaceKeyHash> inner;
		for (int t : made) {
			const auto& v = m.tets[t];
			for (int i = 0; i < 4; ++i) {
				FaceKey key(v[TetMesh::kFace[i][0]], v[TetMesh::kFace[i][1]], v[TetMesh::kFace[i][2]]);
				auto it = outer.find(key);
				if (it != outer.end()) {
					int nb = it->second.first, j = it->second.second;
					m.neighbors[t][i] = nb;
					if (nb >= 0 && j >= 0) m.neighbors[nb][j] = t;
					if (outer_bit[key]) m.constrained[t] |= static_cast<uint8_t>(1u << i);
					continue;
				}
				auto in = inner.find(key);
				if (in == inner.end()) inner[key] = {t, i};
				else {
					m.neighbors[t][i] = in->second.first;
					m.neighbors[in->second.first][in->second.second] = t;
				}
			}
		}
	}

	// The best flip of tetrahedron t, if any improves its worst dihedral.
	bool TryFlips(int t) {
		double old_min = Dihedral(t);
		double best = old_min + 1e-9;
		std::vector<int> best_old;
		std::vector<std::array<int, 4>> best_fresh;
		int material = m.material[t];
		// 2-3 flips across each face.
		for (int i = 0; i < 4; ++i) {
			if (FaceLocked(t, i)) continue;
			int t2 = m.neighbors[t][i];
			if (dead[t2]) continue;
			const auto& v = m.tets[t];
			int x = v[TetMesh::kFace[i][0]], y = v[TetMesh::kFace[i][1]], z = v[TetMesh::kFace[i][2]], a = v[i];
			int j = FaceIndex(t2, x, y, z);
			if (j < 0) continue;
			int b = m.tets[t2][j];
			std::array<int, 4> n1{{a, b, x, y}}, n2{{a, b, y, z}}, n3{{a, b, z, x}};
			if (!Oriented(n1) || !Oriented(n2) || !Oriented(n3)) continue;
			// The segment a-b must pass through the face: the three new
			// tetrahedra then have the same total volume as the old two.
			double vol_new = std::fabs(Dot(m.nodes[n1[1]] - m.nodes[n1[0]], Cross(m.nodes[n1[2]] - m.nodes[n1[0]], m.nodes[n1[3]] - m.nodes[n1[0]]))) +
				std::fabs(Dot(m.nodes[n2[1]] - m.nodes[n2[0]], Cross(m.nodes[n2[2]] - m.nodes[n2[0]], m.nodes[n2[3]] - m.nodes[n2[0]]))) +
				std::fabs(Dot(m.nodes[n3[1]] - m.nodes[n3[0]], Cross(m.nodes[n3[2]] - m.nodes[n3[0]], m.nodes[n3[3]] - m.nodes[n3[0]])));
			double vol_old = std::fabs(m.TetVolume(t)) * 6.0 + std::fabs(m.TetVolume(t2)) * 6.0;
			if (std::fabs(vol_new - vol_old) > 1e-9 * vol_old) continue;
			double q = std::min({Dihedral(n1), Dihedral(n2), Dihedral(n3)});
			double old2 = std::min(old_min, Dihedral(t2));
			if (q > std::max(best, old2 + 1e-9)) { best = q; best_old = {t, t2}; best_fresh = {n1, n2, n3}; }
		}
		// 3-2 flips around each edge with exactly three tetrahedra.
		for (int k = 0; k < 6; ++k) {
			int u = m.tets[t][kEdges[k][0]], w = m.tets[t][kEdges[k][1]];
			std::vector<int> around;
			for (int s : vt[u]) {
				if (dead[s]) continue;
				const auto& sv = m.tets[s];
				if (sv[0] == w || sv[1] == w || sv[2] == w || sv[3] == w) around.push_back(s);
			}
			if (around.size() != 3) continue;
			bool locked = false;
			for (int q = 0; q < 3 && !locked; ++q) {
				int s = around[q];
				// The two faces of s holding the edge must be free, and the
				// material the same.
				if (m.material[s] != material) { locked = true; break; }
				for (int i = 0; i < 4; ++i) {
					int o = m.tets[s][i];
					if (o == u || o == w) continue; // face i holds the edge
					if (FaceLocked(s, i)) { locked = true; break; }
				}
			}
			if (locked) continue;
			// The three other nodes a, b, c (each appears in two of the tets).
			std::vector<int> abc;
			for (int s : around) for (int i = 0; i < 4; ++i) { int o = m.tets[s][i]; if (o != u && o != w && std::find(abc.begin(), abc.end(), o) == abc.end()) abc.push_back(o); }
			if (abc.size() != 3) continue;
			std::array<int, 4> n1{{abc[0], abc[1], abc[2], u}}, n2{{abc[0], abc[1], abc[2], w}};
			if (!Oriented(n1) || !Oriented(n2)) continue;
			double vol_new = std::fabs(Dot(m.nodes[n1[1]] - m.nodes[n1[0]], Cross(m.nodes[n1[2]] - m.nodes[n1[0]], m.nodes[n1[3]] - m.nodes[n1[0]]))) +
				std::fabs(Dot(m.nodes[n2[1]] - m.nodes[n2[0]], Cross(m.nodes[n2[2]] - m.nodes[n2[0]], m.nodes[n2[3]] - m.nodes[n2[0]])));
			double vol_old = 0.0, old3 = old_min;
			for (int s : around) { vol_old += std::fabs(m.TetVolume(s)) * 6.0; old3 = std::min(old3, Dihedral(s)); }
			if (std::fabs(vol_new - vol_old) > 1e-9 * vol_old) continue;
			double q = std::min(Dihedral(n1), Dihedral(n2));
			if (q > std::max(best, old3 + 1e-9)) { best = q; best_old = around; best_fresh = {n1, n2}; }
		}
		if (best_old.empty()) return false;
		if (best_fresh.size() == 3) ++st.n_flips_23; else ++st.n_flips_32;
		Replace(best_old, best_fresh, material);
		return true;
	}

	// Moves node v toward the centroid of its neighbours as far as that
	// improves the worst dihedral of its tetrahedra without inverting any.
	bool Smooth(int v) {
		const std::vector<int>& star = vt[v];
		if (star.empty()) return false;
		double old_min = 180.0;
		Vec3 centroid{0, 0, 0};
		int count = 0;
		std::vector<int> seen;
		for (int t : star) {
			old_min = std::min(old_min, Dihedral(t));
			for (int k = 0; k < 4; ++k) {
				int o = m.tets[t][k];
				if (o == v || std::find(seen.begin(), seen.end(), o) != seen.end()) continue;
				seen.push_back(o);
				centroid += m.nodes[o];
				++count;
			}
		}
		if (count == 0 || old_min >= opt.target_dihedral_deg) return false;
		centroid = centroid / static_cast<double>(count);
		const Vec3 p0 = m.nodes[v];
		for (double alpha : {1.0, 0.5, 0.25, 0.125}) {
			m.nodes[v] = Lerp(p0, centroid, alpha);
			double new_min = 180.0;
			bool ok = true;
			for (int t : star) {
				const auto& tv = m.tets[t];
				if (ExactOrient(m.nodes[tv[0]], m.nodes[tv[1]], m.nodes[tv[2]], m.nodes[tv[3]]) <= 0.0) { ok = false; break; }
				new_min = std::min(new_min, Dihedral(t));
			}
			if (ok && new_min > old_min + 1e-9) { ++st.n_smoothed; return true; }
		}
		m.nodes[v] = p0;
		return false;
	}

	void Compact() {
		std::vector<std::array<int, 4>> tets;
		std::vector<uint8_t> cons;
		std::vector<int> mat;
		for (size_t t = 0; t < m.tets.size(); ++t) {
			if (dead[t]) continue;
			tets.push_back(m.tets[t]);
			cons.push_back(m.constrained[t]);
			mat.push_back(m.material[t]);
		}
		m.tets = std::move(tets);
		m.constrained = std::move(cons);
		m.material = std::move(mat);
		m.RebuildNeighbors();
	}
};

} // namespace

void ImproveTetMesh(TetMesh& m, const ImproveOptions& opt, ImproveStats* stats_out) {
	auto t_start = std::chrono::steady_clock::now();
	ImproveStats st;
	if (m.tets.empty()) { if (stats_out) *stats_out = st; return; }
	if (m.neighbors.size() != m.tets.size()) m.RebuildNeighbors();
	if (m.constrained.size() != m.tets.size()) m.constrained.assign(m.tets.size(), 0);
	if (m.material.size() != m.tets.size()) m.material.assign(m.tets.size(), 0);
	Improver imp(m, opt, st);
	st.min_dihedral_before = 180.0;
	for (size_t t = 0; t < m.tets.size(); ++t) {
		double d = imp.Dihedral(static_cast<int>(t));
		st.min_dihedral_before = std::min(st.min_dihedral_before, d);
		if (d < opt.target_dihedral_deg) ++st.n_bad_before;
	}
	for (int round = 0; round < opt.rounds; ++round) {
		int changed = 0;
		if (opt.flips) {
			const size_t n0 = m.tets.size(); // new tetrahedra are visited next round
			for (size_t t = 0; t < n0; ++t) {
				if (imp.dead[t] || imp.Dihedral(static_cast<int>(t)) >= opt.target_dihedral_deg) continue;
				if (imp.TryFlips(static_cast<int>(t))) ++changed;
			}
		}
		if (opt.smoothing) {
			for (size_t v = 0; v < m.nodes.size(); ++v) {
				if (imp.fixed[v]) continue;
				if (imp.Smooth(static_cast<int>(v))) ++changed;
			}
		}
		if (changed == 0) break;
	}
	imp.Compact();
	st.min_dihedral_after = 180.0;
	for (size_t t = 0; t < m.tets.size(); ++t) {
		double d = imp.Dihedral(static_cast<int>(t));
		st.min_dihedral_after = std::min(st.min_dihedral_after, d);
		if (d < opt.target_dihedral_deg) ++st.n_bad_after;
	}
	st.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
	if (stats_out) *stats_out = st;
}

} // namespace aerotelic
