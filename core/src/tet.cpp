#include "aerotelic/tet.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "aerotelic/sdf.hpp"

#include "../third_party/predicates/predicates.h"

namespace aerotelic {

// ---------------------------------------------------------------------------
// Exact predicates in this header's conventions
// ---------------------------------------------------------------------------

namespace {

static_assert(sizeof(Vec3) == 3 * sizeof(double), "Vec3 must be three packed doubles for the predicates");
inline const double* P(const Vec3& v) { return &v.x; }

void EnsureExactInit() {
	static std::once_flag flag;
	std::call_once(flag, [] { exactinit(); });
}

// 6 x signed volume of (a, b, c, d): positive when d is on the side of
// the normal Cross(b-a, c-a). orient3d(pa,pb,pc,pd) is the determinant of
// the rows pa-pd, pb-pd, pc-pd, so orient3d(b, c, d, a) = (b-a).((c-a)x(d-a)).
inline double Orient(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
	return orient3d(P(b), P(c), P(d), P(a));
}

// Positive when e is inside the circumsphere of the positively oriented
// (Orient > 0) tetrahedron (a, b, c, d). insphere() wants its first four
// points positively oriented in orient3d's sense, which (b, c, d, a) is.
inline double InSphere(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d, const Vec3& e) {
	return insphere(P(b), P(c), P(d), P(a), P(e));
}

inline int Sign(double v) { return v > 0.0 ? 1 : (v < 0.0 ? -1 : 0); }

double Volume6(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
	return Dot(b - a, Cross(c - a, d - a));
}

// ---------------------------------------------------------------------------
// Keys
// ---------------------------------------------------------------------------

struct FaceKey {
	int a, b, c; // sorted
	FaceKey() : a(-1), b(-1), c(-1) {}
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
		h ^= h >> 29;
		return static_cast<size_t>(h * 0xBF58476D1CE4E5B9ull);
	}
};
using FaceSet = std::unordered_set<FaceKey, FaceKeyHash>;

inline uint64_t EdgeKey(int a, int b) {
	if (a > b) std::swap(a, b);
	return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) | static_cast<uint32_t>(b);
}

// ---------------------------------------------------------------------------
// The incremental engine
// ---------------------------------------------------------------------------

class Engine {
public:
	struct Tet {
		std::array<int, 4> v{{-1, -1, -1, -1}};
		std::array<int, 4> nbr{{-1, -1, -1, -1}};
		uint8_t cons = 0;
	};

	std::vector<Vec3> nodes;
	std::vector<Tet> tets;
	std::vector<int> vtet;   // a live tetrahedron holding the node, -1 if none
	std::array<int, 4> bbox{{-1, -1, -1, -1}};
	// Faces that are constraints (the surface tracker's current triangles);
	// a new tetrahedron face whose key is here gets its constrained bit.
	const FaceSet* tracked = nullptr;
	// Which edges (by EdgeKey) are constraints: a constrained insertion never
	// lets one become interior to its cavity (it would vanish from the mesh).
	std::function<bool(uint64_t)> is_segment;
	TetStats stats;
	int last_tet = -1;

	// The bounding tetrahedron encloses `points` with a wide margin; the
	// points are stored (nodes 0 .. n-1, then the four bounding nodes) but
	// not yet inserted.
	explicit Engine(const std::vector<Vec3>& points) {
		EnsureExactInit();
		nodes = points;
		vtet.assign(points.size(), -1);
		Box3 box;
		for (const auto& p : points) box.Add(p);
		if (box.IsEmpty()) box.Add({0, 0, 0});
		Vec3 c = box.Center();
		double R = std::max(0.5 * box.Diagonal(), 1e-9) * 1000.0 * 1.7320508075688772;
		bbox = {{AddNode(c + Vec3{R, R, R}), AddNode(c + Vec3{R, -R, -R}), AddNode(c + Vec3{-R, -R, R}), AddNode(c + Vec3{-R, R, -R})}};
		Tet t;
		t.v = bbox;
		tets.push_back(t);
		for (int k = 0; k < 4; ++k) vtet[bbox[k]] = 0;
		last_tet = 0;
		if (Orient(nodes[bbox[0]], nodes[bbox[1]], nodes[bbox[2]], nodes[bbox[3]]) <= 0.0) throw std::logic_error("bounding tetrahedron is not positively oriented");
	}

	int AddNode(const Vec3& p) {
		nodes.push_back(p);
		vtet.push_back(-1);
		return static_cast<int>(nodes.size()) - 1;
	}

	bool Alive(int t) const { return tets[t].v[0] >= 0; }
	bool IsBbox(int v) const { return v == bbox[0] || v == bbox[1] || v == bbox[2] || v == bbox[3]; }

	int AnyLive() const {
		for (int t = static_cast<int>(tets.size()) - 1; t >= 0; --t) if (Alive(t)) return t;
		return -1;
	}

	// Orientation of p against face i of t (> 0: outside across that face).
	double FaceOrient(int t, int i, const Vec3& p) const {
		const Tet& T = tets[t];
		return Orient(nodes[T.v[TetMesh::kFace[i][0]]], nodes[T.v[TetMesh::kFace[i][1]]], nodes[T.v[TetMesh::kFace[i][2]]], p);
	}

	bool Contains(int t, const Vec3& p) const {
		for (int i = 0; i < 4; ++i) if (FaceOrient(t, i, p) > 0.0) return false;
		return true;
	}

	double InSphereTet(int t, const Vec3& p) const {
		const Tet& T = tets[t];
		return InSphere(nodes[T.v[0]], nodes[T.v[1]], nodes[T.v[2]], nodes[T.v[3]], p);
	}

	// The tetrahedron whose closure holds p, by a stochastic visibility
	// walk from `start`; -1 if p is outside the mesh.
	int Locate(const Vec3& p, int start) {
		int t = (start >= 0 && start < static_cast<int>(tets.size()) && Alive(start)) ? start : (last_tet >= 0 && Alive(last_tet) ? last_tet : AnyLive());
		if (t < 0) return -1;
		const int max_steps = 64 + 8 * static_cast<int>(std::cbrt(static_cast<double>(tets.size()) + 1.0)) * 16;
		for (int step = 0; step < max_steps; ++step) {
			++stats.n_walk_steps;
			rng_ = rng_ * 6364136223846793005ull + 1442695040888963407ull;
			int first = static_cast<int>(rng_ >> 62);
			int out = -1;
			for (int k = 0; k < 4; ++k) {
				int i = (first + k) & 3;
				if (FaceOrient(t, i, p) > 0.0) { out = i; break; }
			}
			if (out < 0) return t;
			int n = tets[t].nbr[out];
			if (n < 0) return -1;
			t = n;
		}
		// The walk did not settle (a non-Delaunay region can cycle it): scan.
		++stats.n_walk_fallbacks;
		for (int i = 0; i < static_cast<int>(tets.size()); ++i) if (Alive(i) && Contains(i, p)) return i;
		return -1;
	}

	struct InsertOptions {
		// Cavity growth stops at constrained faces (except those listed).
		bool constrained = false;
		// Constrained faces the point lies on (passable).
		const std::vector<std::array<int, 3>>* split_faces = nullptr;
		// Edges x-y such that the new face (p, x, y) is a constraint.
		const std::vector<std::array<int, 2>>* split_edges = nullptr;
	};

	// Inserts node pi (already in `nodes`). Returns a tetrahedron holding
	// it, -1 if it coincides with an existing node or lies outside the mesh.
	int Insert(int pi, int start, const InsertOptions& opt) {
		const Vec3& p = nodes[pi];
		int t0 = Locate(p, start);
		if (t0 < 0) return -1;
		for (int k = 0; k < 4; ++k) {
			const Vec3& q = nodes[tets[t0].v[k]];
			if (q.x == p.x && q.y == p.y && q.z == p.z) return -1;
		}
		NextStamp();
		std::vector<int>& cavity = cavity_;
		std::vector<int>& queue = queue_;
		std::vector<int>& seeds = seeds_;
		cavity.clear(); queue.clear(); seeds.clear();

		// Seeds: every tetrahedron whose closure holds p.
		seeds.push_back(t0);
		SetIn(t0);
		cavity.push_back(t0);
		for (size_t s = 0; s < seeds.size(); ++s) {
			int t = seeds[s];
			for (int i = 0; i < 4; ++i) {
				if (FaceOrient(t, i, p) != 0.0) continue;
				int n = tets[t].nbr[i];
				if (n < 0 || IsIn(n)) continue;
				if (!Contains(n, p)) continue;
				SetIn(n);
				seeds.push_back(n);
				cavity.push_back(n);
			}
		}
		for (int t : seeds) queue.push_back(t);

		auto passable = [&](int t, int i) {
			if (!opt.constrained) return true;
			if (!(tets[t].cons & (1u << i))) return true;
			if (!opt.split_faces) return false;
			const Tet& T = tets[t];
			FaceKey key(T.v[TetMesh::kFace[i][0]], T.v[TetMesh::kFace[i][1]], T.v[TetMesh::kFace[i][2]]);
			for (const auto& f : *opt.split_faces) if (FaceKey(f[0], f[1], f[2]) == key) return true;
			return false;
		};

		// Breadth-first growth by the circumsphere test.
		for (size_t qi = 0; qi < queue.size(); ++qi) {
			int t = queue[qi];
			for (int i = 0; i < 4; ++i) {
				int n = tets[t].nbr[i];
				if (n < 0 || IsIn(n) || IsOut(n)) continue;
				if (!passable(t, i)) continue;
				if (InSphereTet(n, p) >= 0.0) {
					SetIn(n);
					cavity.push_back(n);
					queue.push_back(n);
				} else {
					SetOut(n);
				}
			}
		}

		// Repair: the cavity must be star-shaped from p (every boundary face
		// strictly visible), must not swallow a constrained face, and must
		// keep every one of its nodes on its boundary (a node whose
		// tetrahedra are all in the cavity would vanish from the mesh --
		// impossible in a Delaunay mesh, possible once constraints have
		// bent it).
		auto is_seed = [&](int t) { return std::find(seeds.begin(), seeds.end(), t) != seeds.end(); };
		for (bool changed = true; changed;) {
			changed = false;
			for (size_t ci = 0; ci < cavity.size(); ++ci) {
				int t = cavity[ci];
				if (!IsIn(t)) continue;
				bool drop = false;
				for (int i = 0; i < 4 && !drop; ++i) {
					int n = tets[t].nbr[i];
					bool boundary = (n < 0 || !IsIn(n));
					if (boundary) {
						if (FaceOrient(t, i, p) >= 0.0) drop = true;
					} else if (opt.constrained && (tets[t].cons & (1u << i)) && !passable(t, i)) {
						// Both sides of a constraint in the cavity: drop the
						// later-discovered one (the neighbour, unless it is a seed).
						int later = n;
						if (is_seed(n)) later = t;
						if (later == t) drop = true;
						else { SetOut(n); changed = true; ++stats.n_cavity_repairs; }
					}
				}
				if (drop) {
					if (is_seed(t)) throw std::runtime_error("tet: a seed tetrahedron of the cavity is not visible from the point (point on a blocked constraint?)");
					SetOut(t);
					changed = true;
					++stats.n_cavity_repairs;
				}
			}
			if (changed) continue;
			// Orphans: nodes of cavity tetrahedra on no boundary face.
			std::unordered_map<int, int>& on_boundary = orphan_count_;
			on_boundary.clear();
			for (int t : cavity) {
				if (!IsIn(t)) continue;
				for (int k = 0; k < 4; ++k) on_boundary.try_emplace(tets[t].v[k], 0);
				for (int i = 0; i < 4; ++i) {
					int n = tets[t].nbr[i];
					if (n >= 0 && IsIn(n)) continue;
					for (int k = 0; k < 3; ++k) on_boundary[tets[t].v[TetMesh::kFace[i][k]]] = 1;
				}
			}
			// Drops from the cavity the non-seed tetrahedron holding nodes u (and
			// v, if >= 0) whose centroid is farthest from p.
			auto give_back = [&](int u, int v) {
				int worst = -1;
				double worst_d = -1.0;
				for (int t : cavity) {
					if (!IsIn(t) || is_seed(t)) continue;
					const auto& tv = tets[t].v;
					bool hu = false, hv = (v < 0);
					for (int k = 0; k < 4; ++k) { hu |= tv[k] == u; hv |= tv[k] == v; }
					if (!hu || !hv) continue;
					Vec3 c = (nodes[tv[0]] + nodes[tv[1]] + nodes[tv[2]] + nodes[tv[3]]) * 0.25;
					double d = DistanceSq(c, p);
					if (d > worst_d) { worst_d = d; worst = t; }
				}
				if (worst < 0) throw std::runtime_error("tet: node " + std::to_string(u) + (v >= 0 ? "-" + std::to_string(v) : std::string()) + " would be lost by inserting node " + std::to_string(pi));
				SetOut(worst);
				changed = true;
				++stats.n_cavity_repairs;
			};
			for (const auto& kv : on_boundary) {
				if (kv.second) continue;
				give_back(kv.first, -1);
				break; // re-run the visibility repair before looking again
			}
			if (changed) continue;
			// Constrained edges interior to the cavity (every tetrahedron
			// around them in it) would vanish too.
			if (opt.constrained && is_segment) {
				static const int kEdges[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
				std::unordered_set<uint64_t>& bedges = boundary_edges_;
				bedges.clear();
				for (int t : cavity) {
					if (!IsIn(t)) continue;
					for (int i = 0; i < 4; ++i) {
						int n = tets[t].nbr[i];
						if (n >= 0 && IsIn(n)) continue;
						for (int k = 0; k < 3; ++k) bedges.insert(EdgeKey(tets[t].v[TetMesh::kFace[i][k]], tets[t].v[TetMesh::kFace[i][(k + 1) % 3]]));
					}
				}
				bool fixed = false;
				for (int t : cavity) {
					if (!IsIn(t) || fixed) continue;
					for (const auto& ed : kEdges) {
						int u = tets[t].v[ed[0]], v = tets[t].v[ed[1]];
						uint64_t key = EdgeKey(u, v);
						if (bedges.count(key) || !is_segment(key)) continue;
						give_back(u, v);
						fixed = true;
						break;
					}
				}
			}
		}
		std::vector<int> live;
		live.reserve(cavity.size());
		for (int t : cavity) if (IsIn(t)) live.push_back(t);

		// New tetrahedra: the fan from p to the boundary faces.
		std::vector<std::array<int, 4>>& fresh = fresh_;
		fresh.clear();
		FaceSet& extra = extra_cons_;
		extra.clear();
		for (int t : live) {
			const Tet& T = tets[t];
			for (int i = 0; i < 4; ++i) {
				int n = T.nbr[i];
				if (n >= 0 && IsIn(n)) continue;
				int x = T.v[TetMesh::kFace[i][0]], y = T.v[TetMesh::kFace[i][1]], z = T.v[TetMesh::kFace[i][2]];
				fresh.push_back({{pi, x, y, z}});
				if (T.cons & (1u << i)) extra.insert(FaceKey(x, y, z));
			}
		}
		if (opt.split_edges) for (const auto& e : *opt.split_edges) extra.insert(FaceKey(pi, e[0], e[1]));
		int made = ReplaceCavity(live, fresh, &extra);
		return made;
	}

	// Replaces the tetrahedra of `cavity` by `fresh` (which must tile the
	// same region, every face on its boundary matching a boundary face of
	// the cavity). Returns one of the new tetrahedra. Constrained bits: a
	// new face is constrained if its key is in `extra` or in `tracked`.
	int ReplaceCavity(const std::vector<int>& cavity, const std::vector<std::array<int, 4>>& fresh, const FaceSet* extra) {
		NextStamp();
		for (int t : cavity) SetIn(t);
		// Boundary faces of the cavity -> the outer tetrahedron and its face.
		std::unordered_map<FaceKey, std::pair<int, int>, FaceKeyHash>& outer = outer_;
		outer.clear();
		for (int t : cavity) {
			const Tet& T = tets[t];
			for (int i = 0; i < 4; ++i) {
				int n = T.nbr[i];
				if (n >= 0 && IsIn(n)) continue;
				int oj = -1;
				if (n >= 0) for (int j = 0; j < 4; ++j) if (tets[n].nbr[j] == t) { oj = j; break; }
				outer[FaceKey(T.v[TetMesh::kFace[i][0]], T.v[TetMesh::kFace[i][1]], T.v[TetMesh::kFace[i][2]])] = {n, oj};
			}
		}
		// Release the cavity.
		for (int t : cavity) {
			tets[t].v = {{-1, -1, -1, -1}};
			tets[t].nbr = {{-1, -1, -1, -1}};
			tets[t].cons = 0;
			free_.push_back(t);
		}
		// Allocate the new ones.
		std::vector<int>& made = made_;
		made.clear();
		for (const auto& f : fresh) {
			if (Orient(nodes[f[0]], nodes[f[1]], nodes[f[2]], nodes[f[3]]) <= 0.0)
				throw std::runtime_error("tet: a new tetrahedron would be inverted or flat");
			int slot;
			if (!free_.empty()) { slot = free_.back(); free_.pop_back(); }
			else { slot = static_cast<int>(tets.size()); tets.emplace_back(); GrowMarks(); }
			Tet& T = tets[slot];
			T.v = f;
			T.nbr = {{-1, -1, -1, -1}};
			T.cons = 0;
			for (int k = 0; k < 4; ++k) vtet[f[k]] = slot;
			made.push_back(slot);
		}
		// Link.
		std::unordered_map<FaceKey, std::pair<int, int>, FaceKeyHash>& inner = inner_;
		inner.clear();
		size_t outer_used = 0;
		for (int t : made) {
			Tet& T = tets[t];
			for (int i = 0; i < 4; ++i) {
				int x = T.v[TetMesh::kFace[i][0]], y = T.v[TetMesh::kFace[i][1]], z = T.v[TetMesh::kFace[i][2]];
				FaceKey key(x, y, z);
				auto it = outer.find(key);
				if (it != outer.end()) {
					int n = it->second.first, oj = it->second.second;
					T.nbr[i] = n;
					if (n >= 0) tets[n].nbr[oj] = t;
					if (n >= 0 && (tets[n].cons & (1u << oj))) T.cons |= static_cast<uint8_t>(1u << i);
					++outer_used;
				} else {
					auto in = inner.find(key);
					if (in == inner.end()) {
						inner[key] = {t, i};
					} else {
						if (in->second.first < 0) throw std::runtime_error("tet: cavity boundary is not a manifold (a face would be shared by three tetrahedra)");
						int u = in->second.first, ui = in->second.second;
						T.nbr[i] = u;
						tets[u].nbr[ui] = t;
						in->second.first = -1;
					}
				}
				if ((extra && extra->count(key)) || (tracked && tracked->count(key))) T.cons |= static_cast<uint8_t>(1u << i);
			}
		}
		for (const auto& kv : inner) if (kv.second.first >= 0) throw std::runtime_error("tet: the new tetrahedra do not close the cavity");
		if (outer_used != outer.size()) throw std::runtime_error("tet: the new tetrahedra do not cover every cavity boundary face");
		// A constrained face must be constrained on both sides.
		for (int t : made) {
			for (int i = 0; i < 4; ++i) {
				int n = tets[t].nbr[i];
				if (n < 0 || !(tets[t].cons & (1u << i))) continue;
				for (int j = 0; j < 4; ++j) if (tets[n].nbr[j] == t) { tets[n].cons |= static_cast<uint8_t>(1u << j); break; }
			}
		}
		last_tet = made.empty() ? AnyLive() : made.back();
		return last_tet;
	}

	// Every live tetrahedron holding node v.
	void Star(int v, std::vector<int>& out) {
		out.clear();
		int t0 = vtet[v];
		if (t0 < 0 || !Alive(t0)) {
			// Repair the entry by scanning (rare: only after external edits).
			t0 = -1;
			for (int t = 0; t < static_cast<int>(tets.size()); ++t) {
				if (!Alive(t)) continue;
				const auto& tv = tets[t].v;
				if (tv[0] == v || tv[1] == v || tv[2] == v || tv[3] == v) { t0 = t; break; }
			}
			vtet[v] = t0;
			if (t0 < 0) return;
		}
		NextStamp();
		SetIn(t0);
		out.push_back(t0);
		for (size_t k = 0; k < out.size(); ++k) {
			int t = out[k];
			const Tet& T = tets[t];
			for (int i = 0; i < 4; ++i) {
				if (T.v[i] == v) continue; // the face opposite v does not hold v
				int n = T.nbr[i];
				if (n < 0 || IsIn(n)) continue;
				SetIn(n);
				out.push_back(n);
			}
		}
	}

	bool HasEdge(int a, int b) {
		Star(a, star_);
		for (int t : star_) {
			const auto& v = tets[t].v;
			if (v[0] == b || v[1] == b || v[2] == b || v[3] == b) return true;
		}
		return false;
	}

	// Which face of t has exactly the nodes {a, b, c}; -1 if none.
	int FaceIndex(int t, int a, int b, int c) const {
		const auto& v = tets[t].v;
		for (int i = 0; i < 4; ++i) {
			int o = v[i];
			if (o != a && o != b && o != c) {
				bool ha = false, hb = false, hc = false;
				for (int k = 0; k < 4; ++k) { if (k == i) continue; ha |= v[k] == a; hb |= v[k] == b; hc |= v[k] == c; }
				if (ha && hb && hc) return i;
			}
		}
		return -1;
	}

	// A tetrahedron having face {a, b, c}; -1 if the face is not in the mesh.
	int FindFace(int a, int b, int c, int* face = nullptr) {
		Star(a, star_);
		for (int t : star_) {
			int i = FaceIndex(t, a, b, c);
			if (i >= 0) { if (face) *face = i; return t; }
		}
		return -1;
	}

	void SetConstrained(int t, int i, bool on) {
		if (on) tets[t].cons |= static_cast<uint8_t>(1u << i);
		else tets[t].cons &= static_cast<uint8_t>(~(1u << i));
	}

	int LiveCount() const {
		int n = 0;
		for (size_t t = 0; t < tets.size(); ++t) if (Alive(static_cast<int>(t))) ++n;
		return n;
	}

	// Marks (per-call visited sets without clearing).
	void NextStamp() {
		if (++stamp_ == 0) { std::fill(mark_in_.begin(), mark_in_.end(), 0u); std::fill(mark_out_.begin(), mark_out_.end(), 0u); stamp_ = 1; }
		GrowMarks();
	}
	void GrowMarks() {
		if (mark_in_.size() < tets.size()) { mark_in_.resize(tets.size(), 0u); mark_out_.resize(tets.size(), 0u); }
	}
	bool IsIn(int t) const { return mark_in_[t] == stamp_; }
	bool IsOut(int t) const { return mark_out_[t] == stamp_; }
	void SetIn(int t) { mark_in_[t] = stamp_; mark_out_[t] = 0u; }
	void SetOut(int t) { mark_out_[t] = stamp_; mark_in_[t] = 0u; }
	void ClearIn(int t) { mark_in_[t] = 0u; }

	// Exports the live tetrahedra for which keep[t] (or all when null),
	// dropping the bounding nodes and renumbering.
	TetMesh Export(const std::vector<char>* keep) const {
		TetMesh m;
		std::vector<int> remap(nodes.size(), -1);
		int n = 0;
		for (size_t v = 0; v < nodes.size(); ++v) {
			if (IsBbox(static_cast<int>(v))) continue;
			remap[v] = n++;
			m.nodes.push_back(nodes[v]);
		}
		std::vector<int> tmap(tets.size(), -1);
		for (size_t t = 0; t < tets.size(); ++t) {
			if (!Alive(static_cast<int>(t))) continue;
			if (keep && !(*keep)[t]) continue;
			const auto& v = tets[t].v;
			if (IsBbox(v[0]) || IsBbox(v[1]) || IsBbox(v[2]) || IsBbox(v[3])) continue;
			tmap[t] = static_cast<int>(m.tets.size());
			m.tets.push_back({{remap[v[0]], remap[v[1]], remap[v[2]], remap[v[3]]}});
			m.constrained.push_back(tets[t].cons);
		}
		m.neighbors.resize(m.tets.size());
		m.material.assign(m.tets.size(), 0);
		for (size_t t = 0; t < tets.size(); ++t) {
			if (tmap[t] < 0) continue;
			auto& nb = m.neighbors[tmap[t]];
			for (int i = 0; i < 4; ++i) {
				int n2 = tets[t].nbr[i];
				nb[i] = (n2 >= 0 && tmap[n2] >= 0) ? tmap[n2] : -1;
			}
		}
		return m;
	}

private:
	std::vector<int> free_;
	std::vector<uint32_t> mark_in_, mark_out_;
	uint32_t stamp_ = 0;
	uint64_t rng_ = 0x2545F4914F6CDD1Dull;
	// Scratch, reused across insertions.
	std::vector<int> cavity_, queue_, seeds_, made_, star_;
	std::vector<std::array<int, 4>> fresh_;
	FaceSet extra_cons_;
	std::unordered_map<FaceKey, std::pair<int, int>, FaceKeyHash> outer_, inner_;
	std::unordered_map<int, int> orphan_count_;
	std::unordered_set<uint64_t> boundary_edges_;
};

// ---------------------------------------------------------------------------
// Morton order
// ---------------------------------------------------------------------------

uint64_t Spread21(uint64_t v) {
	v &= 0x1fffff;
	v = (v | (v << 32)) & 0x1f00000000ffffull;
	v = (v | (v << 16)) & 0x1f0000ff0000ffull;
	v = (v | (v << 8)) & 0x100f00f00f00f00full;
	v = (v | (v << 4)) & 0x10c30c30c30c30c3ull;
	v = (v | (v << 2)) & 0x1249249249249249ull;
	return v;
}

std::vector<int> MortonOrder(const std::vector<Vec3>& pts, int first, int count) {
	Box3 box;
	for (int i = first; i < first + count; ++i) box.Add(pts[i]);
	Vec3 ext = box.Extent();
	double s = std::max({ext.x, ext.y, ext.z, 1e-300});
	std::vector<std::pair<uint64_t, int>> keyed;
	keyed.reserve(count);
	for (int i = first; i < first + count; ++i) {
		Vec3 q = (pts[i] - box.min) / s;
		auto grid = [](double v) { return static_cast<uint64_t>(std::clamp(v, 0.0, 1.0) * 2097151.0); };
		uint64_t key = Spread21(grid(q.x)) | (Spread21(grid(q.y)) << 1) | (Spread21(grid(q.z)) << 2);
		keyed.push_back({key, i});
	}
	std::sort(keyed.begin(), keyed.end());
	std::vector<int> order;
	order.reserve(count);
	for (const auto& k : keyed) order.push_back(k.second);
	return order;
}

void InsertAll(Engine& e, int first, int count, const Engine::InsertOptions& opt) {
	std::vector<int> order = MortonOrder(e.nodes, first, count);
	for (int v : order) {
		int t = e.Insert(v, e.last_tet, opt);
		if (t < 0) {
			if (e.Locate(e.nodes[v], e.last_tet) < 0) throw std::runtime_error("tet: a point lies outside the bounding tetrahedron");
			// A duplicate node: leave it without a tetrahedron.
		}
	}
}

// ---------------------------------------------------------------------------
// The surface tracker: the constraint triangles as they are split
// ---------------------------------------------------------------------------

class SurfaceTracker {
public:
	std::vector<std::array<int, 3>> tris;
	std::vector<char> alive;
	std::vector<int> origin;            // input triangle each piece came from
	std::vector<int> region;            // planar region of the piece, -1 if none
	std::unordered_map<uint64_t, std::vector<int>> edge_tris;
	std::deque<std::string> recent_splits; // the last few segment splits, for failure reports
	std::vector<std::string> first_splits;  // and the first few
	FaceSet keys;
	std::unordered_set<uint64_t> abandoned; // segments given up on

	bool HasAbandonedEdge(int idx) const {
		const auto& t = tris[idx];
		for (int k = 0; k < 3; ++k) if (abandoned.count(EdgeKey(t[k], t[(k + 1) % 3]))) return true;
		return false;
	}

	void Add(const std::array<int, 3>& t, int from, int reg) {
		if (t[0] == t[1] || t[1] == t[2] || t[2] == t[0]) return; // degenerate: nothing to track
		int idx = static_cast<int>(tris.size());
		tris.push_back(t);
		alive.push_back(1);
		origin.push_back(from);
		region.push_back(reg);
		for (int k = 0; k < 3; ++k) edge_tris[EdgeKey(t[k], t[(k + 1) % 3])].push_back(idx);
		keys.insert(FaceKey(t[0], t[1], t[2]));
	}
	// An edge inside a planar region (both its triangles in the same region)
	// is no constraint: the region is a facet whose triangulation is free.
	bool IsFree(uint64_t key) const {
		auto it = edge_tris.find(key);
		if (it == edge_tris.end() || it->second.size() != 2) return false;
		int r = region[it->second[0]];
		return r >= 0 && region[it->second[1]] == r;
	}
	bool IsSegment(uint64_t key) const { return edge_tris.count(key) && !IsFree(key); }
	void Remove(int idx) {
		alive[idx] = 0;
		const auto& t = tris[idx];
		for (int k = 0; k < 3; ++k) {
			auto& lst = edge_tris[EdgeKey(t[k], t[(k + 1) % 3])];
			lst.erase(std::remove(lst.begin(), lst.end(), idx), lst.end());
			if (lst.empty()) edge_tris.erase(EdgeKey(t[k], t[(k + 1) % 3]));
		}
		keys.erase(FaceKey(t[0], t[1], t[2]));
	}
	// Splits edge a-b at node m in every triangle on it.
	void SplitEdge(int a, int b, int m) {
		auto it = edge_tris.find(EdgeKey(a, b));
		if (it == edge_tris.end()) return;
		std::vector<int> on = it->second;
		for (int idx : on) {
			std::array<int, 3> t = tris[idx];
			int from = origin[idx], reg = region[idx];
			Remove(idx);
			// Rotate so t = (a, b, c) or (b, a, c).
			int c = -1;
			for (int k = 0; k < 3; ++k) if (t[k] != a && t[k] != b) c = t[k];
			// Keep the winding: find the position of a and b.
			int ia = -1, ib = -1;
			for (int k = 0; k < 3; ++k) { if (t[k] == a) ia = k; if (t[k] == b) ib = k; }
			bool ab_forward = ((ia + 1) % 3 == ib);
			if (ab_forward) { Add({{a, m, c}}, from, reg); Add({{m, b, c}}, from, reg); }
			else { Add({{b, m, c}}, from, reg); Add({{m, a, c}}, from, reg); }
		}
	}
	// Splits triangle idx at node m inside it.
	void SplitFace(int idx, int m) {
		std::array<int, 3> t = tris[idx];
		int from = origin[idx], reg = region[idx];
		Remove(idx);
		Add({{t[0], t[1], m}}, from, reg);
		Add({{t[1], t[2], m}}, from, reg);
		Add({{t[2], t[0], m}}, from, reg);
	}
	int Find(int a, int b, int c) const {
		auto it = edge_tris.find(EdgeKey(a, b));
		if (it == edge_tris.end()) return -1;
		for (int idx : it->second) {
			const auto& t = tris[idx];
			if (t[0] == c || t[1] == c || t[2] == c) return idx;
		}
		return -1;
	}
};

// ---------------------------------------------------------------------------
// Face recovery
// ---------------------------------------------------------------------------

struct SplitChoice {
	double s = 0.5;
	int ref = -1;
	double ref_perp = 0.0, ref_s = 0.5;
};
SplitChoice SplitFraction(Engine& e, int a, int b, std::vector<int>& star);
int CoincidentNode(Engine& e, int self, int start);
bool SnapSegmentThrough(Engine& e, SurfaceTracker& surf, int a, int b, const SplitChoice& c, double snap, int n_input_vertices, int n_interior_points, ConformingStats& st);

struct Recovery {
	Engine& e;
	SurfaceTracker& surf;
	const ConformingOptions& opt;
	ConformingStats& st;
	std::vector<int> star;
	double snap = 0.0;
	int n_input_vertices = 0, n_interior_points = 0;

	Recovery(Engine& e_, SurfaceTracker& s_, const ConformingOptions& o_, ConformingStats& st_) : e(e_), surf(s_), opt(o_), st(st_) {}

	// Does edge p-q of a tetrahedron cross the interior of triangle (a, b, c)?
	bool EdgeCrosses(int p, int q, int a, int b, int c) const {
		if (p == a || p == b || p == c || q == a || q == b || q == c) return false;
		const Vec3& A = e.nodes[a];
		const Vec3& B = e.nodes[b];
		const Vec3& C = e.nodes[c];
		const Vec3& Pp = e.nodes[p];
		const Vec3& Q = e.nodes[q];
		int sp = Sign(Orient(A, B, C, Pp)), sq = Sign(Orient(A, B, C, Q));
		if (sp == 0 || sq == 0 || sp == sq) return false;
		int s1 = Sign(Orient(Pp, Q, A, B)), s2 = Sign(Orient(Pp, Q, B, C)), s3 = Sign(Orient(Pp, Q, C, A));
		return s1 != 0 && s1 == s2 && s2 == s3;
	}

	// The crossing edge of t, if any (as node pair).
	bool CrossingEdge(int t, int a, int b, int c, int* p, int* q) const {
		static const int kEdges[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
		const auto& v = e.tets[t].v;
		for (const auto& ed : kEdges) {
			if (EdgeCrosses(v[ed[0]], v[ed[1]], a, b, c)) { *p = v[ed[0]]; *q = v[ed[1]]; return true; }
		}
		return false;
	}

	// The tetrahedron on edge a-b whose dihedral wedge holds c.
	int WedgeTet(int a, int b, int c) {
		e.Star(a, star);
		for (int t : star) {
			const auto& v = e.tets[t].v;
			if (v[0] != b && v[1] != b && v[2] != b && v[3] != b) continue;
			int p = -1, q = -1;
			for (int k = 0; k < 4; ++k) { if (v[k] == a || v[k] == b) continue; if (p < 0) p = v[k]; else q = v[k]; }
			const Vec3& A = e.nodes[a];
			const Vec3& B = e.nodes[b];
			const Vec3& C = e.nodes[c];
			const Vec3& Pp = e.nodes[p];
			const Vec3& Q = e.nodes[q];
			int s1 = Sign(Orient(A, B, Pp, C)), r1 = Sign(Orient(A, B, Pp, Q));
			int s2 = Sign(Orient(A, B, Q, C)), r2 = Sign(Orient(A, B, Q, Pp));
			if (s1 != 0 && s1 == r1 && s2 != 0 && s2 == r2) return t;
		}
		return -1;
	}

	// Recovers tracked triangle idx. Returns true when it is now a face.
	bool RecoverFace(int idx) {
		const std::array<int, 3> F = surf.tris[idx];
		int a = F[0], b = F[1], c = F[2];
		if (e.FindFace(a, b, c) >= 0) return true;
		int t0 = WedgeTet(a, b, c);
		if (t0 < 0) throw std::runtime_error("tet: no tetrahedron on edge " + std::to_string(a) + "-" + std::to_string(b) + " holds the face's third node " + std::to_string(c) + " (overlapping coplanar faces?)");
		int p0, q0;
		if (!CrossingEdge(t0, a, b, c, &p0, &q0)) throw std::runtime_error("tet: the wedge tetrahedron of a missing face has no edge crossing it");

		// The cavity: tetrahedra with an edge through F, grown across
		// unconstrained faces.
		std::vector<int> cav;
		{
			e.NextStamp();
			e.SetIn(t0);
			cav.push_back(t0);
			for (size_t k = 0; k < cav.size(); ++k) {
				int t = cav[k];
				for (int i = 0; i < 4; ++i) {
					int n = e.tets[t].nbr[i];
					if (n < 0 || e.IsIn(n) || e.IsOut(n)) continue;
					if (e.tets[t].cons & (1u << i)) continue;
					int p, q;
					if (CrossingEdge(n, a, b, c, &p, &q)) { e.SetIn(n); cav.push_back(n); }
					else e.SetOut(n);
				}
			}
		}
		if (TryCavity(cav, a, b, c)) { ++st.n_cavity_recoveries; return e.FindFace(a, b, c) >= 0; }

		// Fallback: a Steiner point where a crossing edge meets F.
		return SteinerOnFace(idx, t0, p0, q0);
	}

	struct Half {
		std::unordered_map<FaceKey, std::pair<int, int>, FaceKeyHash> faces; // boundary faces of the half: key -> (tet, face) in the main mesh
	};

	// Is the key one of F's edges?
	static bool IsFEdge(uint64_t k, int a, int b, int c) { return k == EdgeKey(a, b) || k == EdgeKey(b, c) || k == EdgeKey(c, a); }

	// From boundary face (t, i) of the cavity holding edge e0-e1, the next
	// boundary face around that edge through the cavity's tetrahedra.
	std::pair<int, int> NextBoundaryFace(int t, int i, int e0, int e1, const std::vector<char>& in_cav) const {
		int cur = t, from_face = i;
		for (int guard = 0; guard < 100000; ++guard) {
			// The other face of cur holding e0 and e1.
			int j = -1;
			for (int f = 0; f < 4; ++f) {
				if (f == from_face) continue;
				int o = e.tets[cur].v[f];
				if (o != e0 && o != e1) { j = f; break; }
			}
			int n = e.tets[cur].nbr[j];
			if (n < 0 || !in_cav[n]) return {cur, j};
			int back = -1;
			for (int f = 0; f < 4; ++f) if (e.tets[n].nbr[f] == cur) { back = f; break; }
			cur = n;
			from_face = back;
		}
		throw std::runtime_error("tet: fan walk around a cavity edge did not close");
	}

	// Re-tetrahedralizes the cavity with F = (a, b, c) as a face, or
	// returns false leaving the mesh untouched.
	bool TryCavity(std::vector<int> cav, int a, int b, int c) {
		std::vector<char> in_cav(e.tets.size(), 0);
		for (int t : cav) in_cav[t] = 1;
		// Interior constrained faces would be lost: not handled here.
		for (int t : cav) for (int i = 0; i < 4; ++i) {
			int n = e.tets[t].nbr[i];
			if (n >= 0 && in_cav[n] && (e.tets[t].cons & (1u << i))) return false;
		}
		// Boundary faces, split into two sides of F by walking around
		// edges other than F's.
		std::vector<std::pair<int, int>> bfaces;
		std::unordered_map<uint64_t, int> face_id; // (t<<2|i) -> index
		std::unordered_map<int, int> on_boundary;
		for (int t : cav) for (int i = 0; i < 4; ++i) {
			for (int k = 0; k < 4; ++k) on_boundary.try_emplace(e.tets[t].v[k], 0);
			int n = e.tets[t].nbr[i];
			if (n >= 0 && in_cav[n]) continue;
			face_id[(static_cast<uint64_t>(t) << 2) | i] = static_cast<int>(bfaces.size());
			bfaces.push_back({t, i});
			for (int k = 0; k < 3; ++k) on_boundary[e.tets[t].v[TetMesh::kFace[i][k]]] = 1;
		}
		// A node, or a constrained edge, interior to the cavity would be
		// lost: not handled here.
		for (const auto& kv : on_boundary) if (!kv.second) return false;
		{
			static const int kEdges[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
			std::unordered_set<uint64_t> bedges;
			for (const auto& bf : bfaces) for (int k = 0; k < 3; ++k)
				bedges.insert(EdgeKey(e.tets[bf.first].v[TetMesh::kFace[bf.second][k]], e.tets[bf.first].v[TetMesh::kFace[bf.second][(k + 1) % 3]]));
			for (int t : cav) for (const auto& ed : kEdges) {
				uint64_t key = EdgeKey(e.tets[t].v[ed[0]], e.tets[t].v[ed[1]]);
				if (!bedges.count(key) && surf.IsSegment(key)) return false;
			}
		}
		std::vector<int> comp(bfaces.size(), -1);
		int n_comp = 0;
		for (size_t s = 0; s < bfaces.size(); ++s) {
			if (comp[s] >= 0) continue;
			std::vector<int> stack{static_cast<int>(s)};
			comp[s] = n_comp;
			while (!stack.empty()) {
				int fi = stack.back(); stack.pop_back();
				int t = bfaces[fi].first, i = bfaces[fi].second;
				for (int k = 0; k < 3; ++k) {
					int e0 = e.tets[t].v[TetMesh::kFace[i][k]], e1 = e.tets[t].v[TetMesh::kFace[i][(k + 1) % 3]];
					if (IsFEdge(EdgeKey(e0, e1), a, b, c)) continue;
					auto nx = NextBoundaryFace(t, i, e0, e1, in_cav);
					auto it = face_id.find((static_cast<uint64_t>(nx.first) << 2) | nx.second);
					if (it == face_id.end()) throw std::logic_error("tet: fan walk left the cavity boundary");
					if (comp[it->second] < 0) { comp[it->second] = n_comp; stack.push_back(it->second); }
				}
			}
			++n_comp;
		}
		if (n_comp != 2) return false;

		Half halves[2];
		for (size_t s = 0; s < bfaces.size(); ++s) {
			int t = bfaces[s].first, i = bfaces[s].second;
			const auto& v = e.tets[t].v;
			halves[comp[s]].faces[FaceKey(v[TetMesh::kFace[i][0]], v[TetMesh::kFace[i][1]], v[TetMesh::kFace[i][2]])] = {t, i};
		}

		std::vector<std::array<int, 4>> fresh;
		int enlargements = 0;
		for (int h = 0; h < 2; ++h) {
			for (;;) {
				std::vector<std::array<int, 4>> part;
				int missing_t = -1, missing_i = -1;
				int r = TetrahedralizeHalf(halves[h], a, b, c, part, &missing_t, &missing_i);
				if (r == 1) { fresh.insert(fresh.end(), part.begin(), part.end()); break; }
				if (r < 0) return false;
				// Enlarge across the missing face.
				if (static_cast<int>(cav.size()) >= opt.max_cavity_tets) return false;
				int n = e.tets[missing_t].nbr[missing_i];
				if (n < 0 || in_cav[n] || (e.tets[missing_t].cons & (1u << missing_i))) return false;
				// n's faces: the one shared with missing_t vanishes; the others
				// join the half unless already there (then they turn interior).
				in_cav[n] = 1;
				cav.push_back(n);
				halves[h].faces.erase(FaceKey(e.tets[missing_t].v[TetMesh::kFace[missing_i][0]], e.tets[missing_t].v[TetMesh::kFace[missing_i][1]], e.tets[missing_t].v[TetMesh::kFace[missing_i][2]]));
				for (int j = 0; j < 4; ++j) {
					if (e.tets[n].nbr[j] == missing_t) continue;
					const auto& v = e.tets[n].v;
					FaceKey key(v[TetMesh::kFace[j][0]], v[TetMesh::kFace[j][1]], v[TetMesh::kFace[j][2]]);
					auto it = halves[h].faces.find(key);
					if (it != halves[h].faces.end()) {
						if (e.tets[n].cons & (1u << j)) return false; // a constraint would become interior
						halves[h].faces.erase(it);
					} else {
						halves[h].faces[key] = {n, j};
					}
				}
				++enlargements;
				++st.n_cavity_enlargements;
			}
		}
		e.ReplaceCavity(cav, fresh, nullptr);
		return true;
	}

	// Delaunay-tetrahedralizes the nodes of a half-cavity. Returns 1 with
	// the tetrahedra inside it (global node ids), 0 with a boundary face
	// that the local tetrahedralization lacks, -1 when F itself is missing
	// or the half is not a closed surface.
	int TetrahedralizeHalf(const Half& half, int a, int b, int c, std::vector<std::array<int, 4>>& out, int* missing_t, int* missing_i) {
		// Nodes.
		std::vector<int> gnodes;
		std::unordered_map<int, int> local;
		auto add = [&](int g) { if (!local.count(g)) { local[g] = static_cast<int>(gnodes.size()); gnodes.push_back(g); } };
		for (const auto& kv : half.faces) { add(kv.first.a); add(kv.first.b); add(kv.first.c); }
		add(a); add(b); add(c);
		// The half's closed boundary: its faces (outward from the cavity) plus
		// F, oriented so the enclosed volume is positive.
		std::vector<std::array<int, 3>> shell;
		for (const auto& kv : half.faces) {
			int t = kv.second.first, i = kv.second.second;
			const auto& v = e.tets[t].v;
			shell.push_back({{v[TetMesh::kFace[i][0]], v[TetMesh::kFace[i][1]], v[TetMesh::kFace[i][2]]}});
		}
		auto shell_volume = [&](const std::array<int, 3>& f) {
			double vol = 0.0;
			for (const auto& s : shell) vol += Dot(e.nodes[s[0]], Cross(e.nodes[s[1]], e.nodes[s[2]]));
			vol += Dot(e.nodes[f[0]], Cross(e.nodes[f[1]], e.nodes[f[2]]));
			return vol / 6.0;
		};
		std::array<int, 3> Fo{{a, b, c}};
		double vol = shell_volume(Fo);
		if (vol < 0.0) { Fo = {{a, c, b}}; vol = -vol; }
		shell.push_back(Fo);
		// Closed and consistently oriented?
		{
			std::unordered_map<uint64_t, std::pair<int, int>> edges;
			for (const auto& s : shell) for (int k = 0; k < 3; ++k) {
				int x = s[k], y = s[(k + 1) % 3];
				auto& ee = edges[EdgeKey(x, y)];
				ee.first += 1;
				ee.second += (x < y) ? 1 : -1;
			}
			for (const auto& kv : edges) if (kv.second.first != 2 || kv.second.second != 0) return -1;
		}
		// Local Delaunay tetrahedralization.
		std::vector<Vec3> pts;
		pts.reserve(gnodes.size());
		for (int g : gnodes) pts.push_back(e.nodes[g]);
		Engine loc(pts);
		Engine::InsertOptions plain;
		InsertAll(loc, 0, static_cast<int>(pts.size()), plain);
		// Every shell face present?
		FaceSet shell_keys;
		std::vector<int> seeds;
		for (const auto& s : shell) {
			int x = local[s[0]], y = local[s[1]], z = local[s[2]];
			shell_keys.insert(FaceKey(x, y, z));
			int fi = -1;
			int t = loc.FindFace(x, y, z, &fi);
			if (t < 0) {
				if (s == Fo) return -1;
				auto it = half.faces.find(FaceKey(s[0], s[1], s[2]));
				*missing_t = it->second.first;
				*missing_i = it->second.second;
				return 0;
			}
			// The inner tetrahedron: its fourth node on the inner side of the
			// outward face (x, y, z).
			int w = loc.tets[t].v[fi];
			int inner = t;
			if (Orient(loc.nodes[x], loc.nodes[y], loc.nodes[z], loc.nodes[w]) > 0.0) inner = loc.tets[t].nbr[fi];
			if (inner < 0) return -1;
			seeds.push_back(inner);
		}
		// Flood the inside.
		std::vector<char> inside(loc.tets.size(), 0);
		std::vector<int> stack;
		for (int s : seeds) if (!inside[s]) { inside[s] = 1; stack.push_back(s); }
		while (!stack.empty()) {
			int t = stack.back(); stack.pop_back();
			for (int i = 0; i < 4; ++i) {
				const auto& v = loc.tets[t].v;
				if (shell_keys.count(FaceKey(v[TetMesh::kFace[i][0]], v[TetMesh::kFace[i][1]], v[TetMesh::kFace[i][2]]))) continue;
				int n = loc.tets[t].nbr[i];
				if (n < 0) return -1; // leaked to the hull: the shell is not closed
				if (!inside[n]) { inside[n] = 1; stack.push_back(n); }
			}
		}
		double sum = 0.0;
		out.clear();
		for (size_t t = 0; t < loc.tets.size(); ++t) {
			if (!inside[t] || !loc.Alive(static_cast<int>(t))) continue;
			const auto& v = loc.tets[t].v;
			if (loc.IsBbox(v[0]) || loc.IsBbox(v[1]) || loc.IsBbox(v[2]) || loc.IsBbox(v[3])) return -1;
			out.push_back({{gnodes[v[0]], gnodes[v[1]], gnodes[v[2]], gnodes[v[3]]}});
			sum += Volume6(loc.nodes[v[0]], loc.nodes[v[1]], loc.nodes[v[2]], loc.nodes[v[3]]) / 6.0;
		}
		if (std::fabs(sum - vol) > 1e-9 * vol + 1e-300) return -1;
		return 1;
	}

	// A planar region as a facet. Returns true when every one of its
	// triangles is a face; adopts the mesh's own tiling of the region's
	// plane when there is one; otherwise puts a point inside the region
	// where a tetrahedron edge crosses it (and returns false: the caller's
	// next round looks again).
	bool RecoverRegion(int reg, const std::vector<int>& members) {
		bool all_present = true;
		for (int idx : members) {
			const auto& t = surf.tris[idx];
			if (e.FindFace(t[0], t[1], t[2]) < 0) { all_present = false; break; }
		}
		if (all_present) return true;
		// The region's nodes, plane and area.
		std::unordered_set<int> rnodes;
		double area = 0.0;
		Vec3 normal{0, 0, 0};
		for (int idx : members) {
			const auto& t = surf.tris[idx];
			for (int k = 0; k < 3; ++k) rnodes.insert(t[k]);
			Vec3 n = Cross(e.nodes[t[1]] - e.nodes[t[0]], e.nodes[t[2]] - e.nodes[t[0]]);
			area += 0.5 * Length(n);
			normal += n;
		}
		normal = Normalized(normal);
		// Rim edges: on exactly one member triangle.
		std::unordered_map<uint64_t, int> edge_use;
		for (int idx : members) {
			const auto& t = surf.tris[idx];
			for (int k = 0; k < 3; ++k) ++edge_use[EdgeKey(t[k], t[(k + 1) % 3])];
		}
		// Mesh faces with all three nodes in the region, facing the region's way.
		FaceSet seen;
		std::vector<std::array<int, 3>> faces;
		std::vector<int> star_nodes(rnodes.begin(), rnodes.end());
		for (int v : star_nodes) {
			e.Star(v, star);
			for (int t : star) {
				for (int i = 0; i < 4; ++i) {
					int x = e.tets[t].v[TetMesh::kFace[i][0]], y = e.tets[t].v[TetMesh::kFace[i][1]], z = e.tets[t].v[TetMesh::kFace[i][2]];
					if (!rnodes.count(x) || !rnodes.count(y) || !rnodes.count(z)) continue;
					FaceKey key(x, y, z);
					if (!seen.insert(key).second) continue;
					Vec3 n = Cross(e.nodes[y] - e.nodes[x], e.nodes[z] - e.nodes[x]);
					if (Dot(n, normal) < 0.0) std::swap(y, z);
					faces.push_back({{x, y, z}});
				}
			}
		}
		// Do they tile the region? Same total area, every rim edge used once,
		// no other edge used other than twice.
		double tiled = 0.0;
		std::unordered_map<uint64_t, int> use;
		for (const auto& f : faces) {
			tiled += 0.5 * Length(Cross(e.nodes[f[1]] - e.nodes[f[0]], e.nodes[f[2]] - e.nodes[f[0]]));
			for (int k = 0; k < 3; ++k) ++use[EdgeKey(f[k], f[(k + 1) % 3])];
		}
		bool tiles = !faces.empty() && std::fabs(tiled - area) <= 1e-9 * area;
		if (tiles) {
			for (const auto& kv : edge_use) {
				if (kv.second != 1) continue; // interior to the region's own triangulation
				auto it = use.find(kv.first);
				if (it == use.end() || it->second != 1) { tiles = false; break; }
			}
		}
		if (tiles) {
			for (const auto& kv : use) {
				auto it = edge_use.find(kv.first);
				bool rim = it != edge_use.end() && it->second == 1;
				if (!rim && kv.second != 2) { tiles = false; break; }
			}
		}
		if (tiles) {
			int from = surf.origin[members.front()];
			for (int idx : members) surf.Remove(idx);
			for (const auto& f : faces) surf.Add(f, from, reg);
			++st.n_regions_adopted;
			return true;
		}
		// Not tiled: fall back to recovering the region's own triangulation,
		// triangle by triangle, as for any other face (a free edge that is
		// missing is split like a segment first).
		int progressed = 0;
		for (int idx : members) {
			if (!surf.alive[idx]) continue;
			std::array<int, 3> t = surf.tris[idx];
			if (e.FindFace(t[0], t[1], t[2]) >= 0) continue;
			bool edges_ok = true;
			for (int k = 0; k < 3 && edges_ok; ++k) {
				int a = t[k], b = t[(k + 1) % 3];
				if (e.HasEdge(a, b)) continue;
				edges_ok = false;
				SplitEdge(a, b);
				++st.n_region_steiner;
				++progressed;
			}
			if (!edges_ok) continue;
			RecoverFace(idx);
			++progressed;
		}
		return progressed == 0 && CountMissing(members) == 0;
	}

	int CountMissing(const std::vector<int>& members) {
		int n = 0;
		for (int idx : members) {
			if (!surf.alive[idx]) continue;
			const auto& t = surf.tris[idx];
			if (e.FindFace(t[0], t[1], t[2]) < 0) ++n;
		}
		return n;
	}

	// Splits the (free) edge a-b by the reference-point rule with a
	// constrained insertion, as the segment stage would.
	void SplitEdge(int a, int b) {
		SplitChoice choice = SplitFraction(e, a, b, star);
		if (SnapSegmentThrough(e, surf, a, b, choice, snap, n_input_vertices, n_interior_points, st)) return;
		double split = choice.s;
		Vec3 m = Lerp(e.nodes[a], e.nodes[b], split);
		int mi = e.AddNode(m);
		Engine::InsertOptions io;
		io.constrained = true;
		if (e.Insert(mi, e.vtet[a], io) < 0) {
			int q = CoincidentNode(e, mi, e.vtet[a]);
			e.nodes.pop_back();
			e.vtet.pop_back();
			if (q < 0) throw std::runtime_error("tet: an edge split point coincides with no locatable node");
			surf.SplitEdge(a, b, q);
			++st.n_segment_snaps;
			return;
		}
		surf.SplitEdge(a, b, mi);
	}

	bool SteinerOnFace(int idx, int t0, int p, int q) {
		const std::array<int, 3> F = surf.tris[idx];
		if (opt.max_face_steiner > 0) {
			int from = surf.origin[idx];
			if (face_steiner_count_[from] >= opt.max_face_steiner) throw std::runtime_error("tet: face recovery needs more than " + std::to_string(opt.max_face_steiner) + " Steiner points on input triangle " + std::to_string(from));
			++face_steiner_count_[from];
		}
		const Vec3& A = e.nodes[F[0]];
		Vec3 n = Cross(e.nodes[F[1]] - A, e.nodes[F[2]] - A);
		const Vec3& Pp = e.nodes[p];
		const Vec3& Q = e.nodes[q];
		double dp = Dot(n, Pp - A), dq = Dot(n, Q - A);
		double s = dp / (dp - dq);
		s = std::clamp(s, 1e-6, 1.0 - 1e-6);
		Vec3 x = Lerp(Pp, Q, s);
		int xi = e.AddNode(x);
		std::vector<std::array<int, 3>> split_faces{F};
		std::vector<std::array<int, 2>> split_edges{{{F[0], F[1]}}, {{F[1], F[2]}}, {{F[2], F[0]}}};
		Engine::InsertOptions io;
		io.constrained = true;
		io.split_faces = &split_faces;
		io.split_edges = &split_edges;
		surf.SplitFace(idx, xi);
		int made = e.Insert(xi, t0, io);
		if (made < 0) throw std::runtime_error("tet: a Steiner point on a face coincides with an existing node");
		++st.n_face_steiner;
		return false; // the pieces are checked by the caller's next pass
	}

	std::unordered_map<int, int> face_steiner_count_;
};

// ---------------------------------------------------------------------------
// Segment recovery
// ---------------------------------------------------------------------------

// Does segment a-b pass through face (x, y, z) (closed: touching counts)?
bool SegmentCrossesFace(Engine& e, int a, int b, int x, int y, int z) {
	const Vec3& A = e.nodes[a];
	const Vec3& B = e.nodes[b];
	const Vec3& X = e.nodes[x];
	const Vec3& Y = e.nodes[y];
	const Vec3& Z = e.nodes[z];
	int sa = Sign(Orient(X, Y, Z, A)), sb = Sign(Orient(X, Y, Z, B));
	if (sa != 0 && sb != 0 && sa == sb) return false;
	int s1 = Sign(Orient(A, B, X, Y)), s2 = Sign(Orient(A, B, Y, Z)), s3 = Sign(Orient(A, B, Z, X));
	bool pos = s1 >= 0 && s2 >= 0 && s3 >= 0, neg = s1 <= 0 && s2 <= 0 && s3 <= 0;
	return pos || neg;
}

// Where to split the missing segment a-b (Si 2015, TetGen, with the
// reference point taken over every tetrahedron the segment crosses): the
// reference point p is the node nearest the segment's line among those
// tetrahedra's nodes whose projection falls inside the segment -- the
// node that blocks it. If p is nearer a than the middle, the split is at
// distance |ap| from a (so the new node lies on the sphere about a
// through p, which p therefore no longer encroaches); likewise from b;
// otherwise at p's projection, kept away from the ends. Midpoint when
// nothing is found. `ref`, `ref_perp` and `ref_s` report p, its distance
// from the segment's line and its projection's fraction (ref -1 if none).
SplitChoice SplitFraction(Engine& e, int a, int b, std::vector<int>& star) {
	SplitChoice out;
	e.Star(a, star);
	const Vec3& A = e.nodes[a];
	const Vec3& B = e.nodes[b];
	const Vec3 d = B - A;
	const double L2 = LengthSq(d);
	const double L = std::sqrt(L2);
	// The tetrahedron the segment leaves a through.
	int t0 = -1;
	for (int t : star) {
		int ia = -1;
		for (int k = 0; k < 4; ++k) if (e.tets[t].v[k] == a) ia = k;
		bool enters = true;
		for (int i = 0; i < 4 && enters; ++i) {
			if (i == ia) continue;
			if (e.FaceOrient(t, i, B) > 0.0) enters = false;
		}
		if (enters) { t0 = t; break; }
	}
	if (t0 < 0) return out;
	// The pipe of tetrahedra the segment crosses.
	std::vector<int> pipe{t0};
	std::unordered_set<int> seen{t0};
	for (size_t k = 0; k < pipe.size() && pipe.size() < 2000; ++k) {
		int t = pipe[k];
		const auto& v = e.tets[t].v;
		if (v[0] == b || v[1] == b || v[2] == b || v[3] == b) continue;
		for (int i = 0; i < 4; ++i) {
			int n = e.tets[t].nbr[i];
			if (n < 0 || seen.count(n)) continue;
			if (!SegmentCrossesFace(e, a, b, v[TetMesh::kFace[i][0]], v[TetMesh::kFace[i][1]], v[TetMesh::kFace[i][2]])) continue;
			seen.insert(n);
			pipe.push_back(n);
		}
	}
	int p = -1;
	double best = 1e300, best_s = 0.5;
	for (int t : pipe) {
		for (int k = 0; k < 4; ++k) {
			int v = e.tets[t].v[k];
			if (v == a || v == b || e.IsBbox(v)) continue;
			double s = Dot(e.nodes[v] - A, d) / L2;
			if (s <= 0.0 || s >= 1.0) continue;
			double dist = Length(Cross(e.nodes[v] - A, d)) / L;
			if (dist < best) { best = dist; p = v; best_s = s; }
		}
	}
	if (p < 0) return out;
	out.ref = p;
	out.ref_perp = best;
	out.ref_s = best_s;
	double da = Distance(A, e.nodes[p]), db = Distance(B, e.nodes[p]);
	const double lo = 0.02 * L;
	if (da < 0.5 * L) out.s = std::max(da / L, lo / L);
	else if (db < 0.5 * L) out.s = 1.0 - std::max(db / L, lo / L);
	else out.s = std::clamp(best_s, 0.25, 0.75);
	return out;
}

// One pass over every tracked edge; returns the number of splits made.
// A missing edge is split where its reference point dictates (Si 2015):
// at the distance from an end node of the vertex that blocks the segment
// there, when that is short of the middle, else at the midpoint. The new
// node then lies on the sphere about the end node through the blocking
// vertex, which therefore no longer encroaches the piece -- the rule
// adapts to whatever vertex is in the way, input or Steiner, where a
// midpoint rule ping-pongs without end between nearby segments (two
// sheets at a knife edge, needle triangles at a Steiner vertex).
// The node (other than `self`) at exactly the position of node `self`, or -1.
int CoincidentNode(Engine& e, int self, int start) {
	const Vec3& p = e.nodes[self];
	int t = e.Locate(p, start);
	if (t < 0) return -1;
	for (int k = 0; k < 4; ++k) {
		int v = e.tets[t].v[k];
		if (v == self) continue;
		const Vec3& q = e.nodes[v];
		if (q.x == p.x && q.y == p.y && q.z == p.z) return v;
	}
	return -1;
}

// A node that lies (almost) on a missing segment -- within `snap` of its
// line, projecting well inside it -- cannot be split away from: every
// piece would still pass through it, and a split beside it would plant a
// near-twin node that blocks the next segment the same way. The segment
// is bent through the node instead (its triangles split by it), which
// moves the surface by less than `snap`. Only surface nodes qualify.
bool SnapSegmentThrough(Engine& e, SurfaceTracker& surf, int a, int b, const SplitChoice& c, double snap, int n_input_vertices, int n_interior_points, ConformingStats& st) {
	if (c.ref < 0 || c.ref_perp >= snap) return false;
	if (c.ref >= n_input_vertices && c.ref < n_input_vertices + n_interior_points) return false; // an interior point
	auto it = surf.edge_tris.find(EdgeKey(a, b));
	if (it == surf.edge_tris.end()) return false;
	// Is the node the third vertex of a triangle on the edge? Then that
	// triangle is a sliver lying along the edge, and the edge is flipped
	// away instead: (a, b, ref) + (b, a, y) -> (a, y, ref) + (b, ref, y),
	// which tiles the same surface when ref lies on a-b.
	int sliver = -1, other = -1;
	for (int idx : it->second) {
		const auto& t = surf.tris[idx];
		if (t[0] == c.ref || t[1] == c.ref || t[2] == c.ref) sliver = idx; else other = idx;
	}
	if (sliver >= 0) {
		if (other < 0 || it->second.size() != 2) return false;
		std::array<int, 3> t1 = surf.tris[sliver], t2 = surf.tris[other];
		int y = -1;
		for (int k = 0; k < 3; ++k) if (t2[k] != a && t2[k] != b) y = t2[k];
		if (y < 0 || y == c.ref || surf.edge_tris.count(EdgeKey(c.ref, y))) { ++st.n_flips_refused; return false; }
		// Orient: does t1 run a -> b?
		bool ab = false;
		for (int k = 0; k < 3; ++k) if (t1[k] == a && t1[(k + 1) % 3] == b) ab = true;
		int from = surf.origin[sliver];
		surf.Remove(sliver);
		surf.Remove(other);
		if (ab) { surf.Add({{a, y, c.ref}}, from, -1); surf.Add({{b, c.ref, y}}, from, -1); }
		else { surf.Add({{b, y, c.ref}}, from, -1); surf.Add({{a, c.ref, y}}, from, -1); }
		++st.n_segment_flips;
		return true;
	}
	surf.SplitEdge(a, b, c.ref);
	++st.n_segment_snaps;
	return true;
}

int RecoverSegmentsPass(Engine& e, SurfaceTracker& surf, int n_input_vertices, int n_interior_points, double min_length, double snap, const std::function<bool()>& over_budget, ConformingStats& st, int* n_missing_initially) {
	std::vector<uint64_t> edges;
	edges.reserve(surf.edge_tris.size());
	for (const auto& kv : surf.edge_tris) edges.push_back(kv.first);
	std::sort(edges.begin(), edges.end());
	std::vector<int> star;
	int splits = 0;
	for (uint64_t key : edges) {
		if (!surf.edge_tris.count(key)) continue; // split away meanwhile
		if (surf.IsFree(key)) continue;           // inside a planar region: no constraint
		if (surf.abandoned.count(key)) continue;  // given up on
		int a = static_cast<int>(key >> 32), b = static_cast<int>(key & 0xffffffffu);
		if (e.HasEdge(a, b)) continue;
		if (n_missing_initially) ++*n_missing_initially;
		if (over_budget() && !std::getenv("AEROTELIC_TET_STRICT")) {
			// The budget is spent: nothing more is tried, not even a snap.
			surf.abandoned.insert(key);
			++st.n_segments_abandoned;
			continue;
		}
		double L = Distance(e.nodes[a], e.nodes[b]);
		bool ia = a < n_input_vertices, ib = b < n_input_vertices;
		SplitChoice choice = SplitFraction(e, a, b, star);
		if (SnapSegmentThrough(e, surf, a, b, choice, snap, n_input_vertices, n_interior_points, st)) { ++splits; continue; }
		double split = choice.s;
		Vec3 m = Lerp(e.nodes[a], e.nodes[b], split);
		if (std::min(split, 1.0 - split) * L < min_length && !std::getenv("AEROTELIC_TET_STRICT")) {
			// Give up on this segment: the surface is not conformed here.
			surf.abandoned.insert(key);
			++st.n_segments_abandoned;
			continue;
		}
		if (std::min(split, 1.0 - split) * L < min_length) {
			// Diagnose: which node sits closest to the segment.
			int nearest = -1;
			double best = 1e300;
			for (int v = 0; v < static_cast<int>(e.nodes.size()); ++v) {
				if (v == a || v == b || e.IsBbox(v)) continue;
				Vec3 d = e.nodes[b] - e.nodes[a];
				double s = std::clamp(Dot(e.nodes[v] - e.nodes[a], d) / LengthSq(d), 0.0, 1.0);
				double dist = Distance(e.nodes[v], e.nodes[a] + d * s);
				if (dist < best) { best = dist; nearest = v; }
			}
			char buf[600];
			std::snprintf(buf, sizeof buf, "tet: segment recovery would split a surface edge below %.3g: edge %d%s-%d%s length %.3g at (%.6f, %.6f, %.6f); nearest other node %d%s at %.3g from the segment; %d Steiner points, %d snaps, %d flips (%d refused) so far; reference node %d at perp %.3g s %.3f, chosen split %.3f",
				min_length, a, ia ? "(input)" : "(steiner)", b, ib ? "(input)" : "(steiner)", L, m.x, m.y, m.z, nearest, nearest < n_input_vertices ? "(input)" : "(steiner)", best, st.n_edge_steiner, st.n_segment_snaps, st.n_segment_flips,
				st.n_flips_refused, choice.ref, choice.ref_perp, choice.ref_s, split);
			std::string msg = buf;
			// The nearest node's relation to the edge's triangles.
			if (nearest >= 0) {
				auto it = surf.edge_tris.find(EdgeKey(a, b));
				bool on_tri = false;
				if (it != surf.edge_tris.end()) for (int idx : it->second) for (int k = 0; k < 3; ++k) if (surf.tris[idx][k] == nearest) on_tri = true;
				Vec3 d = e.nodes[b] - e.nodes[a];
				double s_n = Dot(e.nodes[nearest] - e.nodes[a], d) / LengthSq(d);
				std::snprintf(buf, sizeof buf, "; nearest node projects at s %.4f, %s a vertex of the edge's triangles, edges at it: %d", s_n, on_tri ? "IS" : "is not", (int)std::count_if(surf.edge_tris.begin(), surf.edge_tris.end(), [&](const auto& kv) { return (int)(kv.first >> 32) == nearest || (int)(kv.first & 0xffffffffu) == nearest; }));
				msg += buf;
			}
			if (std::getenv("AEROTELIC_TET_TRACE")) {
				msg += "\n  first splits:";
				for (const auto& s : surf.first_splits) msg += "\n    " + s;
				msg += "\n  last splits:";
				for (const auto& s : surf.recent_splits) msg += "\n    " + s;
			}
			throw std::runtime_error(msg);
		}
		if (std::getenv("AEROTELIC_TET_TRACE")) {
			double da = choice.ref_perp, db = choice.ref_s;
			int nearest = -1;
			double best = 1e300;
			for (int v = 0; v < static_cast<int>(e.nodes.size()); ++v) {
				if (v == a || v == b || e.IsBbox(v)) continue;
				Vec3 d = e.nodes[b] - e.nodes[a];
				double s = std::clamp(Dot(e.nodes[v] - e.nodes[a], d) / LengthSq(d), 0.0, 1.0);
				double dist = Distance(e.nodes[v], e.nodes[a] + d * s);
				if (dist < best) { best = dist; nearest = v; }
			}
			char buf[400];
			std::snprintf(buf, sizeof buf, "split %d%s-%d%s L %.3g ref %d perp %.3g at s %.3f -> at %.3f, node %d (%.7f, %.7f, %.7f); nearest %d%s at %.3g (%.7f, %.7f, %.7f)", a, ia ? "i" : "s", b, ib ? "i" : "s", L, choice.ref, da, db, split,
				(int)e.nodes.size(), m.x, m.y, m.z, nearest, nearest < n_input_vertices ? "i" : "s", best, e.nodes[nearest].x, e.nodes[nearest].y, e.nodes[nearest].z);
			if (surf.first_splits.size() < 60) surf.first_splits.push_back(buf);
			surf.recent_splits.push_back(buf);
			if (surf.recent_splits.size() > 30) surf.recent_splits.pop_front();
		}
		int mi = e.AddNode(m);
		// A plain Delaunay insertion: this stage keeps the whole mesh Delaunay,
		// which is what makes a short enough piece certain to appear.
		Engine::InsertOptions io;
		int made = e.Insert(mi, e.vtet[a], io);
		if (made < 0) {
			// The split point is an existing node (the reference node sits
			// exactly on the segment): bend the segment through it.
			int q = CoincidentNode(e, mi, e.vtet[a]);
			e.nodes.pop_back();
			e.vtet.pop_back();
			if (q < 0) throw std::runtime_error("tet: a segment split point coincides with no locatable node");
			surf.SplitEdge(a, b, q);
			++st.n_segment_snaps;
			++splits;
			continue;
		}
		surf.SplitEdge(a, b, mi);
		++st.n_edge_steiner;
		++splits;
	}
	return splits;
}

// Sets the constrained bit on every mesh face that is a tracked triangle.
void MarkTrackedFaces(Engine& e, const SurfaceTracker& surf) {
	for (size_t idx = 0; idx < surf.tris.size(); ++idx) {
		if (!surf.alive[idx]) continue;
		const auto& t = surf.tris[idx];
		int fi = -1;
		int tt = e.FindFace(t[0], t[1], t[2], &fi);
		if (tt < 0) continue;
		e.SetConstrained(tt, fi, true);
		int n = e.tets[tt].nbr[fi];
		if (n >= 0) for (int j = 0; j < 4; ++j) if (e.tets[n].nbr[j] == tt) { e.SetConstrained(n, j, true); break; }
	}
}

int CountMissingFaces(Engine& e, const SurfaceTracker& surf) {
	int n = 0;
	for (size_t idx = 0; idx < surf.tris.size(); ++idx) {
		if (!surf.alive[idx]) continue;
		const auto& t = surf.tris[idx];
		if (e.FindFace(t[0], t[1], t[2]) < 0) ++n;
	}
	return n;
}
// Missing faces not on an abandoned segment: what recovery still works on.
int CountRecoverableMissing(Engine& e, const SurfaceTracker& surf) {
	int n = 0;
	for (size_t idx = 0; idx < surf.tris.size(); ++idx) {
		if (!surf.alive[idx] || surf.HasAbandonedEdge(static_cast<int>(idx))) continue;
		const auto& t = surf.tris[idx];
		if (e.FindFace(t[0], t[1], t[2]) < 0) ++n;
	}
	return n;
}

// Interior / exterior by the parity of surface crossings from the
// bounding tetrahedron outward.
std::vector<char> InteriorMask(Engine& e) {
	std::vector<int> depth(e.tets.size(), -1);
	std::deque<int> dq;
	for (size_t t = 0; t < e.tets.size(); ++t) {
		if (!e.Alive(static_cast<int>(t))) continue;
		const auto& v = e.tets[t].v;
		if (e.IsBbox(v[0]) || e.IsBbox(v[1]) || e.IsBbox(v[2]) || e.IsBbox(v[3])) { depth[t] = 0; dq.push_back(static_cast<int>(t)); }
	}
	// 0-1 BFS: crossing a constrained face costs one.
	while (!dq.empty()) {
		int t = dq.front(); dq.pop_front();
		for (int i = 0; i < 4; ++i) {
			int n = e.tets[t].nbr[i];
			if (n < 0) continue;
			int w = (e.tets[t].cons & (1u << i)) ? 1 : 0;
			int d = depth[t] + w;
			if (depth[n] < 0 || d < depth[n]) {
				depth[n] = d;
				if (w) dq.push_back(n); else dq.push_front(n);
			}
		}
	}
	std::vector<char> keep(e.tets.size(), 0);
	for (size_t t = 0; t < e.tets.size(); ++t) if (e.Alive(static_cast<int>(t)) && depth[t] > 0 && (depth[t] & 1)) keep[t] = 1;
	return keep;
}

std::vector<char> InteriorMaskByField(Engine& e, const TriMesh& surface) {
	MeshSdf sdf(surface);
	std::vector<char> keep(e.tets.size(), 0);
	for (size_t t = 0; t < e.tets.size(); ++t) {
		if (!e.Alive(static_cast<int>(t))) continue;
		const auto& v = e.tets[t].v;
		if (e.IsBbox(v[0]) || e.IsBbox(v[1]) || e.IsBbox(v[2]) || e.IsBbox(v[3])) continue;
		Vec3 c = (e.nodes[v[0]] + e.nodes[v[1]] + e.nodes[v[2]] + e.nodes[v[3]]) * 0.25;
		if (sdf.Distance(c) < 0.0) keep[t] = 1;
	}
	return keep;
}

} // namespace

// ---------------------------------------------------------------------------
// Public: predicates, Delaunay
// ---------------------------------------------------------------------------

double ExactOrient(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
	EnsureExactInit();
	return Orient(a, b, c, d);
}

double ExactInSphere(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d, const Vec3& e) {
	EnsureExactInit();
	return InSphere(a, b, c, d, e);
}

TetMesh DelaunayTetrahedralize(const std::vector<Vec3>& points, TetStats* stats) {
	if (points.size() < 4) throw std::invalid_argument("DelaunayTetrahedralize: fewer than four points");
	auto t0 = std::chrono::steady_clock::now();
	Engine e(points);
	Engine::InsertOptions plain;
	InsertAll(e, 0, static_cast<int>(points.size()), plain);
	TetMesh m = e.Export(nullptr);
	if (m.tets.empty()) throw std::invalid_argument("DelaunayTetrahedralize: the points are coplanar");
	e.stats.n_points = static_cast<int>(points.size());
	e.stats.n_tets = static_cast<int>(m.tets.size());
	e.stats.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
	if (stats) *stats = e.stats;
	return m;
}

// ---------------------------------------------------------------------------
// Public: planar regions
// ---------------------------------------------------------------------------

int RetriangulatePlanarRegions(TriMesh& m, double plane_tol_fraction, int* n_regions_out, std::vector<int>* region_of) {
	EnsureExactInit();
	int n_regions = 0, n_flips = 0;
	if (n_regions_out) *n_regions_out = 0;
	if (region_of) region_of->assign(m.triangles.size(), -1);
	if (plane_tol_fraction <= 0.0 || m.triangles.size() < 2) return 0;
	const double tol = plane_tol_fraction * m.Bounds().Diagonal();
	// Edge -> the triangles on it.
	std::unordered_map<uint64_t, std::array<int, 2>> on_edge;
	on_edge.reserve(m.triangles.size() * 2);
	for (size_t t = 0; t < m.triangles.size(); ++t) {
		for (int k = 0; k < 3; ++k) {
			auto& e = on_edge.try_emplace(EdgeKey(m.triangles[t][k], m.triangles[t][(k + 1) % 3]), std::array<int, 2>{{-1, -1}}).first->second;
			if (e[0] < 0) e[0] = static_cast<int>(t); else e[1] = static_cast<int>(t);
		}
	}
	auto other = [&](uint64_t key, int t) {
		const auto& e = on_edge[key];
		return e[0] == t ? e[1] : e[0];
	};
	std::vector<int> region(m.triangles.size(), -1);
	for (size_t seed = 0; seed < m.triangles.size(); ++seed) {
		if (region[seed] >= 0) continue;
		const auto& s = m.triangles[seed];
		Vec3 n = Normalized(Cross(m.vertices[s[1]] - m.vertices[s[0]], m.vertices[s[2]] - m.vertices[s[0]]));
		if (LengthSq(n) == 0.0) { region[seed] = -2; continue; }
		Vec3 p0 = m.vertices[s[0]];
		auto in_plane = [&](int t) {
			for (int k = 0; k < 3; ++k) if (std::fabs(Dot(m.vertices[m.triangles[t][k]] - p0, n)) > tol) return false;
			Vec3 nt = Cross(m.vertices[m.triangles[t][1]] - m.vertices[m.triangles[t][0]], m.vertices[m.triangles[t][2]] - m.vertices[m.triangles[t][0]]);
			return Dot(nt, n) > 0.0;
		};
		std::vector<int> members{static_cast<int>(seed)};
		region[seed] = static_cast<int>(seed);
		for (size_t k = 0; k < members.size(); ++k) {
			int t = members[k];
			for (int e = 0; e < 3; ++e) {
				int o = other(EdgeKey(m.triangles[t][e], m.triangles[t][(e + 1) % 3]), t);
				if (o < 0 || region[o] >= 0 || !in_plane(o)) continue;
				region[o] = static_cast<int>(seed);
				members.push_back(o);
			}
		}
		if (members.size() < 2) continue;
		if (region_of) for (int t : members) (*region_of)[t] = n_regions;
		++n_regions;
		// A 2D frame in the plane (right-handed with n, so the triangles stay
		// counter-clockwise).
		Vec3 u = std::fabs(n.x) < 0.9 ? Cross(n, Vec3{1, 0, 0}) : Cross(n, Vec3{0, 1, 0});
		u = Normalized(u);
		Vec3 v = Cross(n, u);
		std::unordered_map<int, std::array<double, 2>> uv;
		for (int t : members) for (int k = 0; k < 3; ++k) {
			int vi = m.triangles[t][k];
			if (!uv.count(vi)) uv[vi] = {{Dot(m.vertices[vi] - p0, u), Dot(m.vertices[vi] - p0, v)}};
		}
		std::vector<char> in_region(m.triangles.size(), 0);
		for (int t : members) in_region[t] = 1;
		// Lawson flips on the interior edges.
		std::vector<uint64_t> stack;
		for (int t : members) for (int e = 0; e < 3; ++e) {
			uint64_t key = EdgeKey(m.triangles[t][e], m.triangles[t][(e + 1) % 3]);
			int o = other(key, t);
			if (o >= 0 && in_region[o] && o > t) stack.push_back(key);
		}
		int guard = 0;
		while (!stack.empty() && guard++ < 100 * static_cast<int>(members.size()) * static_cast<int>(members.size()) + 1000) {
			uint64_t key = stack.back();
			stack.pop_back();
			auto it = on_edge.find(key);
			if (it == on_edge.end()) continue;
			int t1 = it->second[0], t2 = it->second[1];
			if (t1 < 0 || t2 < 0 || !in_region[t1] || !in_region[t2]) continue;
			int a = static_cast<int>(key >> 32), b = static_cast<int>(key & 0xffffffffu);
			// Orient so t1 = (a, b, c) in that cyclic order and t2 = (b, a, d).
			auto third = [&](int t, int x, int y, bool* forward) {
				const auto& tr = m.triangles[t];
				for (int k = 0; k < 3; ++k) {
					if (tr[k] == x && tr[(k + 1) % 3] == y) { *forward = true; return tr[(k + 2) % 3]; }
					if (tr[k] == y && tr[(k + 1) % 3] == x) { *forward = false; return tr[(k + 2) % 3]; }
				}
				return -1;
			};
			bool f1, f2;
			int c = third(t1, a, b, &f1), d = third(t2, a, b, &f2);
			if (c < 0 || d < 0 || f1 == f2) continue; // not a consistently oriented pair
			if (!f1) { std::swap(t1, t2); std::swap(c, d); }
			// Now t1 = (a, b, c), t2 = (b, a, d), both counter-clockwise in (u, v).
			const double* A = uv[a].data();
			const double* B = uv[b].data();
			const double* C = uv[c].data();
			const double* D = uv[d].data();
			if (orient2d(A, B, C) <= 0.0) continue;
			if (incircle(A, B, C, D) <= 0.0) continue;   // locally Delaunay (or cocircular)
			if (orient2d(A, D, C) <= 0.0 || orient2d(B, C, D) <= 0.0) continue; // the quad is not convex
			// Flip a-b to c-d: t1 := (a, d, c), t2 := (b, c, d).
			m.triangles[t1] = {{a, d, c}};
			m.triangles[t2] = {{b, c, d}};
			on_edge.erase(key);
			on_edge[EdgeKey(c, d)] = {{t1, t2}};
			auto reassign = [&](int x, int y, int from, int to) {
				auto& e = on_edge[EdgeKey(x, y)];
				if (e[0] == from) e[0] = to; else if (e[1] == from) e[1] = to;
			};
			reassign(b, c, t1, t2);
			reassign(a, d, t2, t1);
			++n_flips;
			for (uint64_t k2 : {EdgeKey(a, c), EdgeKey(b, c), EdgeKey(a, d), EdgeKey(b, d)}) {
				const auto& e = on_edge[k2];
				if (e[0] >= 0 && e[1] >= 0 && in_region[e[0]] && in_region[e[1]]) stack.push_back(k2);
			}
		}
	}
	if (n_regions_out) *n_regions_out = n_regions;
	return n_flips;
}

// ---------------------------------------------------------------------------
// Public: the conforming interior mesh
// ---------------------------------------------------------------------------

TetMesh TetrahedralizeInterior(const TriMesh& input, const ConformingOptions& opt, ConformingStats* stats_out) {
	ConformingStats st;
	auto clock = std::chrono::steady_clock::now;
	auto t_start = clock();
	if (input.triangles.empty()) throw std::invalid_argument("TetrahedralizeInterior: empty surface");
	if (!input.IsClosedManifold()) throw std::invalid_argument("TetrahedralizeInterior: the surface is not a closed manifold");
	TriMesh surface = input;
	std::vector<int> region_of;
	st.n_planar_flips = RetriangulatePlanarRegions(surface, opt.planar_tolerance, &st.n_planar_regions, &region_of);
	st.n_surface_vertices = static_cast<int>(surface.vertices.size());
	st.n_surface_triangles = static_cast<int>(surface.triangles.size());
	st.n_interior_points = static_cast<int>(opt.interior_points.size());
	st.surface_volume = surface.SignedVolume();
	if (st.surface_volume <= 0.0) throw std::invalid_argument("TetrahedralizeInterior: the surface is inside out (negative volume)");

	// Points: surface vertices, then interior points.
	std::vector<Vec3> pts = surface.vertices;
	pts.insert(pts.end(), opt.interior_points.begin(), opt.interior_points.end());
	Engine e(pts);

	SurfaceTracker surf;
	for (size_t i = 0; i < surface.triangles.size(); ++i) surf.Add(surface.triangles[i], static_cast<int>(i), region_of[i]);
	e.tracked = &surf.keys;
	e.is_segment = [&surf](uint64_t key) { return surf.IsSegment(key); };
	const int n_input_vertices = static_cast<int>(surface.vertices.size());
	const int n_interior_points = static_cast<int>(opt.interior_points.size());
	const double diagonal = surface.Bounds().Diagonal();
	const double min_length = opt.min_segment_fraction * diagonal;
	const double snap = opt.segment_snap_fraction * diagonal;
	const int steiner_budget = static_cast<int>(opt.max_steiner_factor * n_input_vertices) + 100;
	auto elapsed = [&]() { return std::chrono::duration<double>(clock() - t_start).count(); };
	// Steiner points, snaps and flips all count against the budget (snaps
	// and flips add no node but can go on for ever on a degenerate surface).
	std::function<bool()> over_budget = [&]() {
		int work = static_cast<int>(e.nodes.size()) - n_input_vertices - n_interior_points - 4 + st.n_segment_snaps + st.n_segment_flips;
		return work > steiner_budget || (opt.max_seconds > 0.0 && elapsed() > opt.max_seconds);
	};
	const bool trace = std::getenv("AEROTELIC_TET_TRACE") != nullptr;

	// 1. Delaunay.
	{
		auto t0 = clock();
		Engine::InsertOptions plain;
		InsertAll(e, 0, static_cast<int>(pts.size()), plain);
		MarkTrackedFaces(e, surf);
		st.seconds_delaunay = std::chrono::duration<double>(clock() - t0).count();
	}

	// 2. Segments, on a Delaunay mesh throughout.
	{
		auto ts = clock();
		for (int pass = 0; pass < 256; ++pass) {
			int splits = RecoverSegmentsPass(e, surf, n_input_vertices, n_interior_points, min_length, snap, over_budget, st, pass == 0 ? &st.n_segments_missing : nullptr);
			if (trace) std::printf("  [tet] segment pass %d: %d splits, %d nodes, %d abandoned, %.1f s\n", pass, splits, (int)e.nodes.size(), st.n_segments_abandoned, elapsed());
			if (splits == 0) break;
			if (pass == 255) throw std::runtime_error("tet: segment recovery did not converge in 256 passes");
		}
		st.seconds_segments = std::chrono::duration<double>(clock() - ts).count();
		MarkTrackedFaces(e, surf);
		st.n_faces_missing = CountMissingFaces(e, surf);
	}

	// 3. Faces. Every insertion from here on is constrained and keeps the
	// segments and the faces already present.
	{
		auto ts = clock();
		Recovery rec(e, surf, opt, st);
		rec.snap = snap;
		rec.n_input_vertices = n_input_vertices;
		rec.n_interior_points = n_interior_points;
		bool done = false;
		for (int round = 0; round < 64 && !done; ++round) {
			int handled = 0;
			// Planar regions first, as facets.
			{
				std::unordered_map<int, std::vector<int>> by_region;
				for (size_t idx = 0; idx < surf.tris.size(); ++idx) if (surf.alive[idx] && surf.region[idx] >= 0 && !surf.HasAbandonedEdge(static_cast<int>(idx))) by_region[surf.region[idx]].push_back(static_cast<int>(idx));
				for (const auto& kv : by_region) {
					if (over_budget()) break;
					try {
						if (!rec.RecoverRegion(kv.first, kv.second)) ++handled;
					} catch (const std::runtime_error&) {
						for (int idx : kv.second) {
							if (!surf.alive[idx]) continue;
							const auto& t = surf.tris[idx];
							for (int k = 0; k < 3; ++k) surf.abandoned.insert(EdgeKey(t[k], t[(k + 1) % 3]));
						}
					}
				}
				MarkTrackedFaces(e, surf);
			}
			size_t n_tracked = surf.tris.size(); // pieces added during the round are checked next round
			for (size_t idx = 0; idx < n_tracked; ++idx) {
				if (!surf.alive[idx] || surf.region[idx] >= 0 || surf.HasAbandonedEdge(static_cast<int>(idx))) continue;
				const auto& t = surf.tris[idx];
				if (e.FindFace(t[0], t[1], t[2]) >= 0) continue;
				if (over_budget()) {
					// The budget is spent: abandon what is left.
					for (int k = 0; k < 3; ++k) surf.abandoned.insert(EdgeKey(t[k], t[(k + 1) % 3]));
					continue;
				}
				if (!e.HasEdge(t[0], t[1]) || !e.HasEdge(t[1], t[2]) || !e.HasEdge(t[2], t[0]))
					throw std::runtime_error("tet: a surface edge was lost during face recovery (triangle " + std::to_string(idx) + ")");
				try {
					rec.RecoverFace(static_cast<int>(idx));
				} catch (const std::runtime_error& ex) {
					// Give up on this triangle: mark its edges abandoned so its
					// pieces are not retried.
					for (int k = 0; k < 3; ++k) surf.abandoned.insert(EdgeKey(t[k], t[(k + 1) % 3]));
					(void)ex;
				}
				++handled;
			}
			MarkTrackedFaces(e, surf);
			if (trace) std::printf("  [tet] face round %d: %d handled, %d missing, %d nodes, %.1f s\n", round, handled, CountMissingFaces(e, surf), (int)e.nodes.size(), elapsed());
			if (handled == 0 || CountRecoverableMissing(e, surf) == 0) done = true;
			if (over_budget()) {
				for (size_t idx = 0; idx < surf.tris.size(); ++idx) {
					if (!surf.alive[idx]) continue;
					const auto& t = surf.tris[idx];
					if (e.FindFace(t[0], t[1], t[2]) >= 0) continue;
					for (int k = 0; k < 3; ++k) surf.abandoned.insert(EdgeKey(t[k], t[(k + 1) % 3]));
				}
				done = true;
			}
		}
		if (!done) throw std::runtime_error("tet: face recovery did not converge in 64 rounds (" + std::to_string(CountMissingFaces(e, surf)) + " faces still missing; " + std::to_string(st.n_cavity_recoveries) +
			" cavity recoveries, " + std::to_string(st.n_face_steiner) + " face Steiner, " + std::to_string(st.n_region_steiner) + " region points, " + std::to_string(st.n_regions_adopted) + " regions adopted)");
		st.seconds_faces = std::chrono::duration<double>(clock() - ts).count();
		st.n_faces_abandoned = CountMissingFaces(e, surf);
	}

	// 4. Interior: by crossing parity when the surface is whole, by the
	// surface's distance field where faces were abandoned (parity would
	// leak through the holes).
	std::vector<char> keep = st.n_faces_abandoned == 0 ? InteriorMask(e) : InteriorMaskByField(e, input);
	TetMesh m = e.Export(&keep);
	int n_live = e.LiveCount();
	st.n_tets_interior = static_cast<int>(m.tets.size());
	st.n_tets_exterior = n_live - st.n_tets_interior;
	st.n_surface_faces = 0;
	for (size_t idx = 0; idx < surf.tris.size(); ++idx) if (surf.alive[idx]) ++st.n_surface_faces;
	st.mesh_volume = m.Volume();
	st.delaunay = e.stats;
	st.seconds_total = std::chrono::duration<double>(clock() - t_start).count();
	if (stats_out) *stats_out = st;
	return m;
}

// ---------------------------------------------------------------------------
// TetMesh
// ---------------------------------------------------------------------------

double TetMesh::TetVolume(int t) const {
	const auto& v = tets[t];
	return Volume6(nodes[v[0]], nodes[v[1]], nodes[v[2]], nodes[v[3]]) / 6.0;
}

double TetMesh::Volume() const {
	double s = 0.0;
	for (size_t t = 0; t < tets.size(); ++t) s += TetVolume(static_cast<int>(t));
	return s;
}

double TetMesh::Volume(int mat) const {
	double s = 0.0;
	for (size_t t = 0; t < tets.size(); ++t) if (material[t] == mat) s += TetVolume(static_cast<int>(t));
	return s;
}

int TetMesh::Count(int mat) const {
	int n = 0;
	for (size_t t = 0; t < tets.size(); ++t) if (material[t] == mat) ++n;
	return n;
}

TriMesh TetMesh::BoundaryFaces(int mat) const {
	TriMesh m;
	m.vertices = nodes;
	for (size_t t = 0; t < tets.size(); ++t) {
		if (mat >= 0 && material[t] != mat) continue;
		for (int i = 0; i < 4; ++i) {
			int n = neighbors[t][i];
			bool boundary = n < 0 || (mat >= 0 && material[n] != mat);
			if (!boundary) continue;
			m.triangles.push_back({{tets[t][kFace[i][0]], tets[t][kFace[i][1]], tets[t][kFace[i][2]]}});
		}
	}
	return m;
}

TriMesh TetMesh::InterfaceFaces(int a, int b) const {
	TriMesh m;
	m.vertices = nodes;
	for (size_t t = 0; t < tets.size(); ++t) {
		if (material[t] != a) continue;
		for (int i = 0; i < 4; ++i) {
			int n = neighbors[t][i];
			if (n < 0 || material[n] != b) continue;
			m.triangles.push_back({{tets[t][kFace[i][0]], tets[t][kFace[i][1]], tets[t][kFace[i][2]]}});
		}
	}
	return m;
}

void TetMesh::Quality(int mat, double* min_dihedral_deg, double* min_radius_ratio, int* n_below_5deg) const {
	static const int kEdges[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
	static const int kOpp[6][2] = {{2, 3}, {1, 3}, {1, 2}, {0, 3}, {0, 2}, {0, 1}};
	double min_d = 180.0, min_r = 1.0;
	int slivers = 0;
	const double kPi = 3.14159265358979323846;
	for (size_t t = 0; t < tets.size(); ++t) {
		if (mat >= 0 && material[t] != mat) continue;
		const auto& v = tets[t];
		Vec3 p[4] = {nodes[v[0]], nodes[v[1]], nodes[v[2]], nodes[v[3]]};
		double worst = 180.0;
		for (int k = 0; k < 6; ++k) {
			const Vec3& a = p[kEdges[k][0]];
			const Vec3& b = p[kEdges[k][1]];
			const Vec3& c = p[kOpp[k][0]];
			const Vec3& d = p[kOpp[k][1]];
			Vec3 n1 = Cross(b - a, c - a), n2 = Cross(b - a, d - a);
			double l1 = Length(n1), l2 = Length(n2);
			if (l1 == 0.0 || l2 == 0.0) { worst = 0.0; break; }
			// The dihedral angle at edge ab between faces abc and abd: n1 points
			// into the tetrahedron (towards d), n2 out of it (away from c), so
			// the angle between them is the dihedral angle itself.
			double cosang = Dot(n1, n2) / (l1 * l2);
			cosang = std::clamp(cosang, -1.0, 1.0);
			worst = std::min(worst, std::acos(cosang) * 180.0 / kPi);
		}
		min_d = std::min(min_d, worst);
		if (worst < 5.0) ++slivers;
		double vol = Volume6(p[0], p[1], p[2], p[3]) / 6.0;
		double area = 0.0;
		for (int i = 0; i < 4; ++i) area += 0.5 * Length(Cross(p[kFace[i][1]] - p[kFace[i][0]], p[kFace[i][2]] - p[kFace[i][0]]));
		double r_in = area > 0.0 ? 3.0 * vol / area : 0.0;
		// Circumradius: |a|^2 (b x c) + ... formula with d at the origin.
		Vec3 a = p[0] - p[3], b = p[1] - p[3], c = p[2] - p[3];
		double det = Dot(a, Cross(b, c));
		double R = 0.0;
		if (det != 0.0) {
			Vec3 o = (Cross(b, c) * LengthSq(a) + Cross(c, a) * LengthSq(b) + Cross(a, b) * LengthSq(c)) / (2.0 * det);
			R = Length(o);
		}
		double ratio = R > 0.0 ? 3.0 * r_in / R : 0.0;
		min_r = std::min(min_r, ratio);
	}
	if (min_dihedral_deg) *min_dihedral_deg = min_d;
	if (min_radius_ratio) *min_radius_ratio = min_r;
	if (n_below_5deg) *n_below_5deg = slivers;
}

bool TetMesh::RebuildNeighbors() {
	neighbors.assign(tets.size(), {{-1, -1, -1, -1}});
	std::unordered_map<FaceKey, std::pair<int, int>, FaceKeyHash> seen;
	seen.reserve(tets.size() * 2);
	bool ok = true;
	for (size_t t = 0; t < tets.size(); ++t) {
		for (int i = 0; i < 4; ++i) {
			FaceKey key(tets[t][kFace[i][0]], tets[t][kFace[i][1]], tets[t][kFace[i][2]]);
			auto it = seen.find(key);
			if (it == seen.end()) { seen[key] = {static_cast<int>(t), i}; continue; }
			if (it->second.first < 0) { ok = false; continue; }
			neighbors[t][i] = it->second.first;
			neighbors[it->second.first][it->second.second] = static_cast<int>(t);
			it->second.first = -1;
		}
	}
	return ok;
}

bool TetMesh::IsValid(std::string* why) const {
	EnsureExactInit();
	for (size_t t = 0; t < tets.size(); ++t) {
		const auto& v = tets[t];
		for (int k = 0; k < 4; ++k) if (v[k] < 0 || v[k] >= static_cast<int>(nodes.size())) { if (why) *why = "node index out of range in tet " + std::to_string(t); return false; }
		if (Orient(nodes[v[0]], nodes[v[1]], nodes[v[2]], nodes[v[3]]) <= 0.0) { if (why) *why = "tet " + std::to_string(t) + " is not positively oriented"; return false; }
	}
	if (neighbors.size() != tets.size()) { if (why) *why = "neighbour table size"; return false; }
	TetMesh copy;
	copy.nodes = nodes;
	copy.tets = tets;
	if (!copy.RebuildNeighbors()) { if (why) *why = "a face is shared by three tetrahedra"; return false; }
	for (size_t t = 0; t < tets.size(); ++t) {
		if (copy.neighbors[t] != neighbors[t]) { if (why) *why = "neighbour table of tet " + std::to_string(t) + " disagrees with the faces"; return false; }
	}
	return true;
}

} // namespace aerotelic
