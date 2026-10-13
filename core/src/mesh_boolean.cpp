#include "aerotelic/mesh_boolean.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "aerotelic/sdf.hpp"

#include "../third_party/predicates/predicates.h"

namespace aerotelic {

namespace {

void EnsureExact() {
	static std::once_flag flag;
	std::call_once(flag, [] { exactinit(); });
}

inline const double* P(const Vec3& v) { return &v.x; }
inline int Sign(double v) { return v > 0.0 ? 1 : (v < 0.0 ? -1 : 0); }

// ---------------------------------------------------------------------------
// Positions as keys (to the bit)
// ---------------------------------------------------------------------------

struct PosKey {
	double x, y, z;
	bool operator==(const PosKey& o) const { return x == o.x && y == o.y && z == o.z; }
};
struct PosHash {
	size_t operator()(const PosKey& k) const {
		auto h = [](double d) { uint64_t b; std::memcpy(&b, &d, sizeof b); return b * 0x9E3779B97F4A7C15ull; };
		return static_cast<size_t>(h(k.x) ^ (h(k.y) >> 7) ^ (h(k.z) >> 13));
	}
};
inline PosKey KeyOf(const Vec3& v) { return {v.x == 0.0 ? 0.0 : v.x, v.y == 0.0 ? 0.0 : v.y, v.z == 0.0 ? 0.0 : v.z}; } // -0 == +0

// The output vertex table: one vertex per distinct position. Input
// vertices may also be registered with a tolerance (IdNear): a vertex
// within it of one already registered is that vertex -- the root cap of a
// wing and of its mirror image differ by the rounding of the loft's
// rotations (1e-19 in y at the symmetry plane), and the wall between them
// can only be recognised if the two rims are the same vertices.
class VertexTable {
public:
	std::vector<Vec3> points;
	int Id(const Vec3& p) {
		auto it = ids_.find(KeyOf(p));
		if (it != ids_.end()) return it->second;
		int id = static_cast<int>(points.size());
		points.push_back(p);
		ids_.emplace(KeyOf(p), id);
		Cell(p, true).push_back(id);
		return id;
	}
	int IdNear(const Vec3& p, double tol) {
		auto it = ids_.find(KeyOf(p));
		if (it != ids_.end()) return it->second;
		if (tol > 0.0) {
			const double tol2 = tol * tol;
			std::array<int64_t, 3> k = CellKey(p);
			for (int dx = -1; dx <= 1; ++dx) for (int dy = -1; dy <= 1; ++dy) for (int dz = -1; dz <= 1; ++dz) {
				auto c = grid_.find({{k[0] + dx, k[1] + dy, k[2] + dz}});
				if (c == grid_.end()) continue;
				for (int id : c->second) if (DistanceSq(points[id], p) <= tol2) return id;
			}
		}
		return Id(p);
	}
	void SetCell(double cell) { cell_ = std::max(cell, 1e-300); }
private:
	struct CellHash { size_t operator()(const std::array<int64_t, 3>& k) const { return static_cast<size_t>(k[0] * 73856093LL ^ k[1] * 19349663LL ^ k[2] * 83492791LL); } };
	std::array<int64_t, 3> CellKey(const Vec3& p) const {
		return {{static_cast<int64_t>(std::floor(p.x / cell_)), static_cast<int64_t>(std::floor(p.y / cell_)), static_cast<int64_t>(std::floor(p.z / cell_))}};
	}
	std::vector<int>& Cell(const Vec3& p, bool) { return grid_[CellKey(p)]; }
	std::unordered_map<PosKey, int, PosHash> ids_;
	std::unordered_map<std::array<int64_t, 3>, std::vector<int>, CellHash> grid_;
	double cell_ = 1e-6;
};

// ---------------------------------------------------------------------------
// A bounding-volume hierarchy over triangles, for box queries
// ---------------------------------------------------------------------------

class TriTree {
public:
	explicit TriTree(const TriMesh& m) : m_(m) {
		order_.resize(m.triangles.size());
		boxes_.resize(m.triangles.size());
		std::vector<Vec3> centroids(m.triangles.size());
		for (size_t t = 0; t < m.triangles.size(); ++t) {
			order_[t] = static_cast<int>(t);
			const auto& tr = m.triangles[t];
			for (int k = 0; k < 3; ++k) boxes_[t].Add(m.vertices[tr[k]]);
			centroids[t] = boxes_[t].Center();
		}
		if (!m.triangles.empty()) Build(centroids, 0, static_cast<int>(m.triangles.size()));
	}
	const Box3& TriangleBox(int t) const { return boxes_[t]; }
	void Query(const Box3& box, std::vector<int>& out) const {
		out.clear();
		if (nodes_.empty()) return;
		int stack[128];
		int sp = 0;
		stack[sp++] = 0;
		while (sp > 0) {
			const Node& n = nodes_[stack[--sp]];
			if (!Overlaps(n.box, box)) continue;
			if (n.count > 0) {
				for (int i = n.first; i < n.first + n.count; ++i) if (Overlaps(boxes_[order_[i]], box)) out.push_back(order_[i]);
				continue;
			}
			stack[sp++] = n.left;
			stack[sp++] = n.right;
		}
	}
private:
	struct Node { Box3 box; int left = -1, right = -1, first = 0, count = 0; };
	static bool Overlaps(const Box3& a, const Box3& b) {
		return a.min.x <= b.max.x && b.min.x <= a.max.x && a.min.y <= b.max.y && b.min.y <= a.max.y && a.min.z <= b.max.z && b.min.z <= a.max.z;
	}
	int Build(std::vector<Vec3>& centroids, int begin, int end) {
		Node node;
		for (int i = begin; i < end; ++i) node.box.Add(boxes_[order_[i]]);
		int index = static_cast<int>(nodes_.size());
		nodes_.push_back(node);
		if (end - begin <= 4) { nodes_[index].first = begin; nodes_[index].count = end - begin; return index; }
		Box3 cb;
		for (int i = begin; i < end; ++i) cb.Add(centroids[order_[i]]);
		Vec3 ext = cb.Extent();
		int axis = ext.y > ext.x ? 1 : 0;
		if (ext.z > ext[axis]) axis = 2;
		int mid = (begin + end) / 2;
		std::nth_element(order_.begin() + begin, order_.begin() + mid, order_.begin() + end, [&](int a, int b) { return centroids[a][axis] < centroids[b][axis]; });
		int left = Build(centroids, begin, mid), right = Build(centroids, mid, end);
		nodes_[index].left = left;
		nodes_[index].right = right;
		return index;
	}
	const TriMesh& m_;
	std::vector<int> order_;
	std::vector<Box3> boxes_;
	std::vector<Node> nodes_;
};

// ---------------------------------------------------------------------------
// In-plane tests on exact coordinates (the plane's dominant axis dropped)
// ---------------------------------------------------------------------------

struct Frame2 {
	int ax = 0, ay = 1; // the two kept axes, ordered so the triangle is counter-clockwise
	std::array<double, 2> Of(const Vec3& p) const { return {{p[ax], p[ay]}}; }
};

Frame2 FrameOf(const Vec3& a, const Vec3& b, const Vec3& c) {
	Vec3 n = Cross(b - a, c - a);
	int drop = 0;
	if (std::fabs(n.y) > std::fabs(n[drop])) drop = 1;
	if (std::fabs(n.z) > std::fabs(n[drop])) drop = 2;
	Frame2 f;
	f.ax = (drop + 1) % 3;
	f.ay = (drop + 2) % 3;
	if (n[drop] < 0.0) std::swap(f.ax, f.ay);
	return f;
}

// p (in the plane of a, b, c) inside the triangle or on its boundary.
bool InsideOrOn2(const Frame2& f, const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& p) {
	auto A = f.Of(a), B = f.Of(b), C = f.Of(c), Q = f.Of(p);
	double o1 = orient2d(A.data(), B.data(), Q.data()), o2 = orient2d(B.data(), C.data(), Q.data()), o3 = orient2d(C.data(), A.data(), Q.data());
	return (o1 >= 0.0 && o2 >= 0.0 && o3 >= 0.0) || (o1 <= 0.0 && o2 <= 0.0 && o3 <= 0.0);
}

// Segments p-q and r-s (all in one plane) cross or touch.
bool SegmentsMeet2(const Frame2& f, const Vec3& p, const Vec3& q, const Vec3& r, const Vec3& s) {
	auto A = f.Of(p), B = f.Of(q), C = f.Of(r), D = f.Of(s);
	double o1 = orient2d(A.data(), B.data(), C.data()), o2 = orient2d(A.data(), B.data(), D.data());
	double o3 = orient2d(C.data(), D.data(), A.data()), o4 = orient2d(C.data(), D.data(), B.data());
	if (o1 * o2 < 0.0 && o3 * o4 < 0.0) return true;
	// Touching: an endpoint of one on the other.
	auto on = [](const std::array<double, 2>& x, const std::array<double, 2>& y, const std::array<double, 2>& z) {
		if (orient2d(x.data(), y.data(), z.data()) != 0.0) return false;
		return z[0] >= std::min(x[0], y[0]) && z[0] <= std::max(x[0], y[0]) && z[1] >= std::min(x[1], y[1]) && z[1] <= std::max(x[1], y[1]);
	};
	return on(A, B, C) || on(A, B, D) || on(C, D, A) || on(C, D, B);
}

// Two coplanar triangles overlap with positive area (touching along an
// edge or at a point does not count).
bool CoplanarOverlap(const Frame2& f, const Vec3* t1, const Vec3* t2) {
	std::array<double, 2> A[3], B[3];
	for (int k = 0; k < 3; ++k) { A[k] = f.Of(t1[k]); B[k] = f.Of(t2[k]); }
	// Both counter-clockwise in the frame (t1 is by construction; t2 may not be).
	if (orient2d(B[0].data(), B[1].data(), B[2].data()) < 0.0) std::swap(B[1], B[2]);
	auto separated_by = [](const std::array<double, 2>* X, const std::array<double, 2>* Y) {
		for (int k = 0; k < 3; ++k) {
			const auto& a = X[k];
			const auto& b = X[(k + 1) % 3];
			bool all_out = true;
			for (int j = 0; j < 3 && all_out; ++j) all_out = orient2d(a.data(), b.data(), Y[j].data()) <= 0.0;
			if (all_out) return true;
		}
		return false;
	};
	return !(separated_by(A, B) || separated_by(B, A));
}

// ---------------------------------------------------------------------------
// The 2-D constrained Delaunay triangulation of one cut triangle
// ---------------------------------------------------------------------------

class Cdt2 {
public:
	std::vector<std::array<double, 2>> pts;
	std::vector<std::array<int, 3>> tris; // counter-clockwise; -1 marks a dead one

	// Builds the Delaunay triangulation of pts, recovers every constraint
	// edge, then keeps the triangles inside the boundary loop: `boundary`
	// lists the loop's edges (the cut triangle's outline through the points
	// on its edges), and a flood from the super triangle across every edge
	// that is not a constraint marks what is outside. A point on an edge
	// of the cut triangle is a hair off the straight edge in exact
	// arithmetic, so the outline is taken as the polyline through it rather
	// than the straight edge -- both triangles sharing that edge see the
	// same outline, which is what keeps the pieces of the two watertight.
	void Build(const std::vector<std::array<int, 2>>& boundary, const std::vector<std::array<int, 2>>& segments) {
		double lo = std::numeric_limits<double>::infinity(), hi = -lo;
		for (const auto& p : pts) { lo = std::min({lo, p[0], p[1]}); hi = std::max({hi, p[0], p[1]}); }
		double span = std::max(hi - lo, 1e-300), c = 0.5 * (lo + hi);
		const int n = static_cast<int>(pts.size());
		pts.push_back({{c - 20.0 * span, c - 20.0 * span}});
		pts.push_back({{c + 20.0 * span, c - 20.0 * span}});
		pts.push_back({{c, c + 20.0 * span}});
		tris.push_back({{n, n + 1, n + 2}});
		for (int i = 0; i < n; ++i) Insert(i);
		std::unordered_set<uint64_t> locked;
		auto key = [](int a, int b) { return (static_cast<uint64_t>(static_cast<uint32_t>(std::min(a, b))) << 32) | static_cast<uint32_t>(std::max(a, b)); };
		for (const auto& e : boundary) { Recover(e[0], e[1]); locked.insert(key(e[0], e[1])); }
		for (const auto& e : segments) { Recover(e[0], e[1]); locked.insert(key(e[0], e[1])); }
		// Outside: flood from the super vertices' triangles across unlocked edges.
		std::vector<std::array<int, 3>> live;
		for (const auto& t : tris) if (t[0] >= 0) live.push_back(t);
		tris = std::move(live);
		std::unordered_map<uint64_t, std::array<int, 2>> on_edge;
		for (size_t t = 0; t < tris.size(); ++t) for (int k = 0; k < 3; ++k) {
			auto& e = on_edge.try_emplace(key(tris[t][k], tris[t][(k + 1) % 3]), std::array<int, 2>{{-1, -1}}).first->second;
			if (e[0] < 0) e[0] = static_cast<int>(t); else e[1] = static_cast<int>(t);
		}
		std::vector<char> outside(tris.size(), 0);
		std::vector<int> stack;
		for (size_t t = 0; t < tris.size(); ++t) if (tris[t][0] >= n || tris[t][1] >= n || tris[t][2] >= n) { outside[t] = 1; stack.push_back(static_cast<int>(t)); }
		while (!stack.empty()) {
			int t = stack.back();
			stack.pop_back();
			for (int k = 0; k < 3; ++k) {
				uint64_t ek = key(tris[t][k], tris[t][(k + 1) % 3]);
				if (locked.count(ek)) continue;
				const auto& e = on_edge[ek];
				int o = e[0] == t ? e[1] : e[0];
				if (o >= 0 && !outside[o]) { outside[o] = 1; stack.push_back(o); }
			}
		}
		std::vector<std::array<int, 3>> kept;
		for (size_t t = 0; t < tris.size(); ++t) if (!outside[t]) kept.push_back(tris[t]);
		tris = std::move(kept);
		pts.resize(n);
	}

	// Inserts point p (already in pts) into the triangulation built above,
	// keeping the locked edges: the cavity grows from the triangle holding p
	// across unlocked edges only. False, with nothing changed, if p lies in
	// no triangle or the cavity would not be star-shaped from it.
	bool InsertConstrained(int p, const std::unordered_set<uint64_t>& locked) {
		int start = -1;
		for (size_t t = 0; t < tris.size() && start < 0; ++t) {
			const auto& tr = tris[t];
			if (tr[0] < 0) continue;
			if (Orient(tr[0], tr[1], p) >= 0.0 && Orient(tr[1], tr[2], p) >= 0.0 && Orient(tr[2], tr[0], p) >= 0.0) start = static_cast<int>(t);
		}
		if (start < 0) return false;
		return Cavity({start}, p, locked, -1, -1);
	}

	// Inserts point p lying on the locked edge u-v, which becomes the locked
	// edges u-p and p-v.
	bool InsertOnEdge(int p, int u, int v, std::unordered_set<uint64_t>& locked) {
		std::vector<int> seeds;
		for (size_t t = 0; t < tris.size(); ++t) {
			const auto& tr = tris[t];
			if (tr[0] < 0) continue;
			bool has_u = tr[0] == u || tr[1] == u || tr[2] == u, has_v = tr[0] == v || tr[1] == v || tr[2] == v;
			if (has_u && has_v) seeds.push_back(static_cast<int>(t));
		}
		if (seeds.empty() || !Cavity(seeds, p, locked, u, v)) return false;
		locked.erase(Key(u, v));
		locked.insert(Key(u, p));
		locked.insert(Key(p, v));
		return true;
	}

	static uint64_t Key(int a, int b) { return (static_cast<uint64_t>(static_cast<uint32_t>(std::min(a, b))) << 32) | static_cast<uint32_t>(std::max(a, b)); }
	double Orient(int a, int b, int c) const { return orient2d(pts[a].data(), pts[b].data(), pts[c].data()); }

private:
	// Grows the cavity from the seed triangles across unlocked edges into
	// triangles whose circumcircle holds p, and re-triangulates it around p.
	// Edge su-sv (the one p lies on, if any) is never a cavity boundary.
	bool Cavity(std::vector<int> cavity, int p, const std::unordered_set<uint64_t>& locked, int su, int sv) {
		std::unordered_map<uint64_t, std::array<int, 2>> on_edge;
		for (size_t t = 0; t < tris.size(); ++t) {
			if (tris[t][0] < 0) continue;
			for (int k = 0; k < 3; ++k) {
				auto& e = on_edge.try_emplace(Key(tris[t][k], tris[t][(k + 1) % 3]), std::array<int, 2>{{-1, -1}}).first->second;
				if (e[0] < 0) e[0] = static_cast<int>(t); else e[1] = static_cast<int>(t);
			}
		}
		std::vector<char> in(tris.size(), 0);
		for (int t : cavity) in[t] = 1;
		for (size_t i = 0; i < cavity.size(); ++i) {
			int t = cavity[i];
			for (int k = 0; k < 3; ++k) {
				uint64_t ek = Key(tris[t][k], tris[t][(k + 1) % 3]);
				if (locked.count(ek)) continue;
				const auto& e = on_edge[ek];
				int o = e[0] == t ? e[1] : e[0];
				if (o < 0 || in[o]) continue;
				const auto& tr = tris[o];
				if (incircle(pts[tr[0]].data(), pts[tr[1]].data(), pts[tr[2]].data(), pts[p].data()) > 0.0) { in[o] = 1; cavity.push_back(o); }
			}
		}
		std::unordered_map<uint64_t, int> use;
		for (int t : cavity) for (int k = 0; k < 3; ++k) ++use[Key(tris[t][k], tris[t][(k + 1) % 3])];
		std::vector<std::array<int, 2>> boundary;
		const uint64_t skip = su >= 0 ? Key(su, sv) : ~0ull;
		for (int t : cavity) for (int k = 0; k < 3; ++k) {
			int a = tris[t][k], b = tris[t][(k + 1) % 3];
			uint64_t ek = Key(a, b);
			if (use[ek] != 1 || ek == skip) continue;
			if (Orient(a, b, p) <= 0.0) return false;
			boundary.push_back({{a, b}});
		}
		for (int t : cavity) tris[t] = {{-1, -1, -1}};
		for (const auto& e : boundary) tris.push_back({{e[0], e[1], p}});
		return true;
	}

	void Insert(int p) {
		// Cavity: every live triangle whose circumcircle holds p.
		std::vector<int> cavity;
		for (size_t t = 0; t < tris.size(); ++t) {
			const auto& tr = tris[t];
			if (tr[0] < 0) continue;
			if (incircle(pts[tr[0]].data(), pts[tr[1]].data(), pts[tr[2]].data(), pts[p].data()) > 0.0) cavity.push_back(static_cast<int>(t));
		}
		if (cavity.empty()) throw std::logic_error("mesh boolean: a cut point lies in no circumcircle");
		// Boundary edges: those used once by the cavity.
		std::unordered_map<uint64_t, int> use;
		auto key = [](int a, int b) { return (static_cast<uint64_t>(static_cast<uint32_t>(std::min(a, b))) << 32) | static_cast<uint32_t>(std::max(a, b)); };
		for (int t : cavity) for (int k = 0; k < 3; ++k) ++use[key(tris[t][k], tris[t][(k + 1) % 3])];
		std::vector<std::array<int, 2>> boundary;
		for (int t : cavity) for (int k = 0; k < 3; ++k) {
			int a = tris[t][k], b = tris[t][(k + 1) % 3];
			if (use[key(a, b)] == 1) boundary.push_back({{a, b}});
		}
		for (int t : cavity) tris[t] = {{-1, -1, -1}};
		for (const auto& e : boundary) {
			double o = Orient(e[0], e[1], p);
			if (o > 0.0) tris.push_back({{e[0], e[1], p}});
			else if (o < 0.0) tris.push_back({{e[1], e[0], p}});
			else throw std::logic_error("mesh boolean: a cut point lies on a cavity boundary edge");
		}
	}

	bool HasEdge(int a, int b) const {
		for (const auto& t : tris) {
			if (t[0] < 0) continue;
			for (int k = 0; k < 3; ++k) if ((t[k] == a && t[(k + 1) % 3] == b) || (t[k] == b && t[(k + 1) % 3] == a)) return true;
		}
		return false;
	}

	// Flips the diagonal c-d of the two triangles on it to the other
	// diagonal when the quad is convex; false otherwise.
	bool Flip(int c, int d) {
		int t1 = -1, t2 = -1, e = -1, f = -1;
		for (size_t t = 0; t < tris.size(); ++t) {
			const auto& tr = tris[t];
			if (tr[0] < 0) continue;
			for (int k = 0; k < 3; ++k) {
				if (tr[k] == c && tr[(k + 1) % 3] == d) { t1 = static_cast<int>(t); e = tr[(k + 2) % 3]; }
				if (tr[k] == d && tr[(k + 1) % 3] == c) { t2 = static_cast<int>(t); f = tr[(k + 2) % 3]; }
			}
		}
		if (t1 < 0 || t2 < 0) return false;
		// Convex: e and f on opposite sides of c-d (given), and c, d on
		// opposite sides of e-f.
		if (Orient(e, f, c) * Orient(e, f, d) >= 0.0) return false;
		// t1 = (c, d, e), t2 = (d, c, f) -> (c, f, e)?? Keep counter-clockwise:
		// new triangles (e, c, f) and (e, f, d).
		std::array<int, 3> n1{{e, c, f}}, n2{{e, f, d}};
		if (Orient(n1[0], n1[1], n1[2]) <= 0.0 || Orient(n2[0], n2[1], n2[2]) <= 0.0) return false;
		tris[t1] = n1;
		tris[t2] = n2;
		return true;
	}

	// Recovers edge a-b by flipping the edges that cross it (Sloan).
	void Recover(int a, int b) {
		for (int guard = 0; guard < 10000; ++guard) {
			if (HasEdge(a, b)) return;
			bool flipped = false;
			for (size_t t = 0; t < tris.size() && !flipped; ++t) {
				const auto& tr = tris[t];
				if (tr[0] < 0) continue;
				for (int k = 0; k < 3 && !flipped; ++k) {
					int c = tr[k], d = tr[(k + 1) % 3];
					if (c == a || c == b || d == a || d == b) continue;
					if (Orient(a, b, c) * Orient(a, b, d) >= 0.0) continue;
					if (Orient(c, d, a) * Orient(c, d, b) >= 0.0) continue;
					flipped = Flip(c, d);
				}
			}
			if (!flipped) throw std::logic_error("mesh boolean: a constraint edge of a cut triangle could not be recovered");
		}
		throw std::logic_error("mesh boolean: constraint recovery did not converge");
	}
};

// ---------------------------------------------------------------------------
// Triangle-triangle intersection
// ---------------------------------------------------------------------------

// For each triangle, the representative of its exactly coplanar group
// (adjacent triangles whose far vertex lies, to the bit, in its plane: the
// two halves of a planar quad, a planar cap). Every plane test and every
// edge crossing against a triangle uses its representative's vertices, so
// two coplanar triangles cut a crossing edge in the same point to the bit;
// computed each from its own vertices, the points differ by rounding, and
// the two collinear segments overlap by a hair instead of meeting.
std::vector<int> PlaneRepresentatives(const TriMesh& m) {
	const size_t n = m.triangles.size();
	std::vector<int> rep(n);
	for (size_t t = 0; t < n; ++t) rep[t] = static_cast<int>(t);
	std::function<int(int)> find = [&](int t) { while (rep[t] != t) { rep[t] = rep[rep[t]]; t = rep[t]; } return t; };
	std::unordered_map<uint64_t, std::array<int, 2>> on_edge;
	on_edge.reserve(n * 2);
	for (size_t t = 0; t < n; ++t) {
		for (int k = 0; k < 3; ++k) {
			int x = m.triangles[t][k], y = m.triangles[t][(k + 1) % 3];
			auto& e = on_edge.try_emplace((static_cast<uint64_t>(static_cast<uint32_t>(std::min(x, y))) << 32) | static_cast<uint32_t>(std::max(x, y)), std::array<int, 2>{{-1, -1}}).first->second;
			if (e[0] < 0) e[0] = static_cast<int>(t); else e[1] = static_cast<int>(t);
		}
	}
	for (const auto& kv : on_edge) {
		int t1 = kv.second[0], t2 = kv.second[1];
		if (t1 < 0 || t2 < 0) continue;
		const auto& a = m.triangles[t1];
		const auto& b = m.triangles[t2];
		int far = -1;
		for (int k = 0; k < 3; ++k) if (b[k] != a[0] && b[k] != a[1] && b[k] != a[2]) far = b[k];
		if (far < 0) continue;
		if (orient3d(P(m.vertices[a[0]]), P(m.vertices[a[1]]), P(m.vertices[a[2]]), P(m.vertices[far])) != 0.0) continue;
		int r1 = find(t1), r2 = find(t2);
		if (r1 != r2) rep[std::max(r1, r2)] = std::min(r1, r2);
	}
	for (size_t t = 0; t < n; ++t) rep[t] = find(static_cast<int>(t));
	return rep;
}

class Intersector {
public:
	const TriMesh& a;
	const TriMesh& b;
	VertexTable& table;
	const std::vector<int>& ida;   // vertex -> table id
	const std::vector<int>& idb;
	const std::vector<int> rep_a, rep_b;
	const double floor;
	MeshBooleanStats& st;
	// Segments per triangle of each mesh, as vertex-table ids.
	std::vector<std::vector<std::array<int, 2>>> seg_a, seg_b;

	Intersector(const TriMesh& a_, const TriMesh& b_, VertexTable& t_, const std::vector<int>& ida_, const std::vector<int>& idb_, double floor_, MeshBooleanStats& s_)
		: a(a_), b(b_), table(t_), ida(ida_), idb(idb_), rep_a(PlaneRepresentatives(a_)), rep_b(PlaneRepresentatives(b_)), floor(floor_), st(s_) {
		seg_a.resize(a.triangles.size());
		seg_b.resize(b.triangles.size());
	}

	static uint64_t EdgeKey(int u, int v) { return (static_cast<uint64_t>(static_cast<uint32_t>(std::min(u, v))) << 32) | static_cast<uint32_t>(std::max(u, v)); }

	// The point where edge u-v of mesh `m` (0: a, 1: b) crosses the plane of
	// triangle t of the other mesh, from the orientation values du, dv
	// (opposite signs), computed once per (edge, triangle).
	int Crossing(int m, int u, int v, int t, double du, double dv) {
		if (u > v) { std::swap(u, v); std::swap(du, dv); }
		uint64_t key = EdgeKey(u, v);
		auto& cache = m == 0 ? cross_a_ : cross_b_;
		PairKey pk{key, t};
		auto it = cache.find(pk);
		if (it != cache.end()) return it->second;
		const TriMesh& mesh = m == 0 ? a : b;
		double s = du / (du - dv);
		Vec3 p = Lerp(mesh.vertices[u], mesh.vertices[v], s);
		// Within the floor of a point already there, it IS that point: the
		// crossing of edge e with the other body's plane and the crossing of
		// that body's edge e' with this one's plane, where e and e' cross
		// each other (a mirror-image pair on the symmetry plane), are the same
		// point computed twice.
		int id = table.IdNear(p, floor);
		cache.emplace(pk, id);
		auto& origins = origin_[id];
		bool known = false;
		for (const auto& o : origins) known = known || (o.mesh == m && o.u == u && o.v == v);
		if (!known) origins.push_back(Origin{m, u, v, s});
		return id;
	}

	// Records a segment end that lies on an edge (or edges) of either mesh,
	// so the cut of both triangles on that edge runs its outline through it.
	void RegisterEdgePoint(int id) {
		auto it = origin_.find(id);
		if (it == origin_.end()) return;
		for (const Origin& o : it->second) {
			uint64_t ek = EdgeKey(o.u, o.v);
			auto& list = edge_points_[o.mesh][ek];
			bool present = false;
			for (const auto& sp : list) present = present || sp.second == id;
			if (!present) list.push_back({o.s, id});
		}
	}

	// Whether vertex-table id is a crossing point (a point of the seam).
	bool IsSeam(int id) const { return origin_.count(id) > 0; }

	// The points on edge u-v of mesh m, with their fraction from u to v.
	std::vector<std::pair<double, int>> EdgePoints(int m, int u, int v) const {
		std::vector<std::pair<double, int>> out;
		auto it = edge_points_[m].find(EdgeKey(u, v));
		if (it == edge_points_[m].end()) return out;
		out = it->second;
		if (u > v) for (auto& sp : out) sp.first = 1.0 - sp.first;
		return out;
	}

	[[noreturn]] void Refuse(const char* what, const Vec3& where) {
		char buf[300];
		std::snprintf(buf, sizeof buf, "%s at (%.6g, %.6g, %.6g)", what, where.x, where.y, where.z);
		throw MeshBooleanError(buf, where);
	}

	// Processes one pair; returns true if they intersect in a segment.
	bool Process(int ta, int tb) {
		const auto& A = a.triangles[ta];
		const auto& B = b.triangles[tb];
		// Triangles sharing a vertex meet there by construction (the rim of
		// a welded wall, where the two halves share every vertex) and are
		// not a contact to refuse nor a pair to cut.
		for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) if (ida[A[i]] == idb[B[j]]) return false;
		Vec3 pa[3] = {a.vertices[A[0]], a.vertices[A[1]], a.vertices[A[2]]};
		Vec3 pb[3] = {b.vertices[B[0]], b.vertices[B[1]], b.vertices[B[2]]};
		// Each vertex against the other's plane, the plane taken through
		// the other's representative (see PlaneRepresentatives).
		const auto& RA = a.triangles[rep_a[ta]];
		const auto& RB = b.triangles[rep_b[tb]];
		const Vec3* ra[3] = {&a.vertices[RA[0]], &a.vertices[RA[1]], &a.vertices[RA[2]]};
		const Vec3* rb[3] = {&b.vertices[RB[0]], &b.vertices[RB[1]], &b.vertices[RB[2]]};
		double da[3], db[3];
		int sa[3], sb[3];
		for (int k = 0; k < 3; ++k) { da[k] = orient3d(P(*rb[0]), P(*rb[1]), P(*rb[2]), P(pa[k])); sa[k] = Sign(da[k]); }
		for (int k = 0; k < 3; ++k) { db[k] = orient3d(P(*ra[0]), P(*ra[1]), P(*ra[2]), P(pb[k])); sb[k] = Sign(db[k]); }
		int za = (sa[0] == 0) + (sa[1] == 0) + (sa[2] == 0), zb = (sb[0] == 0) + (sb[1] == 0) + (sb[2] == 0);
		if (za == 3) {
			// Coplanar.
			Frame2 f = FrameOf(pa[0], pa[1], pa[2]);
			if (CoplanarOverlap(f, pa, pb)) Refuse("two triangles overlap in a common plane", (pa[0] + pa[1] + pa[2]) / 3.0);
			return false;
		}
		if (sa[0] == sa[1] && sa[1] == sa[2]) return false; // A entirely on one side
		if (sb[0] == sb[1] && sb[1] == sb[2]) return false;
		// Exact contacts.
		Frame2 fb = FrameOf(pb[0], pb[1], pb[2]), fa = FrameOf(pa[0], pa[1], pa[2]);
		for (int k = 0; k < 3; ++k) if (sa[k] == 0 && InsideOrOn2(fb, pb[0], pb[1], pb[2], pa[k])) Refuse("a vertex of one body lies exactly on a face of the other", pa[k]);
		for (int k = 0; k < 3; ++k) if (sb[k] == 0 && InsideOrOn2(fa, pa[0], pa[1], pa[2], pb[k])) Refuse("a vertex of one body lies exactly on a face of the other", pb[k]);
		if (za == 2) {
			int u, v;
			if (sa[0] != 0) { u = 1; v = 2; } else if (sa[1] != 0) { u = 0; v = 2; } else { u = 0; v = 1; }
			for (int k = 0; k < 3; ++k) if (SegmentsMeet2(fb, pa[u], pa[v], pb[k], pb[(k + 1) % 3])) Refuse("an edge of one body lies in a face of the other", pa[u]);
			return false;
		}
		if (zb == 2) {
			int u, v;
			if (sb[0] != 0) { u = 1; v = 2; } else if (sb[1] != 0) { u = 0; v = 2; } else { u = 0; v = 1; }
			for (int k = 0; k < 3; ++k) if (SegmentsMeet2(fa, pb[u], pb[v], pa[k], pa[(k + 1) % 3])) Refuse("an edge of one body lies in a face of the other", pb[u]);
			return false;
		}
		// Chords: where each triangle crosses the other's plane (keyed by the
		// plane's representative, so coplanar neighbours share the points).
		int ca[2], cb[2];
		if (!Chord(0, A, pa, da, sa, rep_b[tb], ca)) return false;
		if (!Chord(1, B, pb, db, sb, rep_a[ta], cb)) return false;
		// Overlap of the two chords along their common line.
		const Vec3& A0 = table.points[ca[0]];
		Vec3 d = table.points[ca[1]] - A0;
		double L2 = LengthSq(d);
		if (L2 == 0.0) return false;
		auto tof = [&](int id) { return Dot(table.points[id] - A0, d) / L2; };
		int ids[4] = {ca[0], ca[1], cb[0], cb[1]};
		double ts[4] = {0.0, 1.0, tof(cb[0]), tof(cb[1])};
		int lo_a = ts[0] <= ts[1] ? 0 : 1, hi_a = 1 - lo_a;
		int lo_b = ts[2] <= ts[3] ? 2 : 3, hi_b = 5 - lo_b;
		int lo = ts[lo_a] >= ts[lo_b] ? lo_a : lo_b, hi = ts[hi_a] <= ts[hi_b] ? hi_a : hi_b;
		double len = (ts[hi] - ts[lo]) * std::sqrt(L2);
		// Chords that only touch (an edge of one body crossing an edge of the
		// other, as mirror-image edges do on the symmetry plane) meet in a
		// point, not a segment: that point is where the neighbouring pairs'
		// segments end, and it is one vertex for all of them (IdNear).
		if (len <= floor) return false;
		if (ids[lo] == ids[hi]) return false;
		std::array<int, 2> seg{{ids[lo], ids[hi]}};
		seg_a[ta].push_back(seg);
		seg_b[tb].push_back(seg);
		RegisterEdgePoint(ids[lo]);
		RegisterEdgePoint(ids[hi]);
		++st.n_segments;
		return true;
	}

private:
	// The chord of triangle T of mesh m across the other mesh's triangle t's
	// plane; false if the chord is a single point (a touch at a vertex
	// outside the other triangle, already checked).
	bool Chord(int m, const std::array<int, 3>& T, const Vec3* p, const double* d, const int* s, int t, int* out) {
		int n = 0;
		int zero = -1;
		for (int k = 0; k < 3; ++k) if (s[k] == 0) zero = k;
		if (zero >= 0) {
			int u = (zero + 1) % 3, v = (zero + 2) % 3;
			if (s[u] == s[v]) return false; // touches the plane at the vertex only
			out[n++] = table.Id(p[zero]);
			out[n++] = Crossing(m, T[u], T[v], t, d[u], d[v]);
			return true;
		}
		for (int k = 0; k < 3; ++k) {
			int u = k, v = (k + 1) % 3;
			if (s[u] != s[v]) out[n++] = Crossing(m, T[u], T[v], t, d[u], d[v]);
		}
		return n == 2;
	}

	struct PairKey {
		uint64_t edge; int t;
		bool operator==(const PairKey& o) const { return edge == o.edge && t == o.t; }
	};
	struct PairHash { size_t operator()(const PairKey& k) const { return std::hash<uint64_t>()(k.edge * 0x9E3779B97F4A7C15ull ^ static_cast<uint64_t>(static_cast<uint32_t>(k.t))); } };
	std::unordered_map<PairKey, int, PairHash> cross_a_, cross_b_;
	struct Origin { int mesh, u, v; double s; };
	std::unordered_map<int, std::vector<Origin>> origin_;
	std::unordered_map<uint64_t, std::vector<std::pair<double, int>>> edge_points_[2];
};

// ---------------------------------------------------------------------------
// Cutting a triangle into pieces along its segments
// ---------------------------------------------------------------------------

// Midpoint splits of constrained sub-edges (a stretch of a triangle's
// outline between two of its points, or a seam segment), shared by every
// triangle whose cut uses the sub-edge, so the pieces on both sides of it
// stay watertight. A sub-edge is split at most once; its halves may be
// split again.
class SubEdgeSplits {
public:
	explicit SubEdgeSplits(VertexTable& t) : table(t) {}
	VertexTable& table;

	static uint64_t Key(int a, int b) { return (static_cast<uint64_t>(static_cast<uint32_t>(std::min(a, b))) << 32) | static_cast<uint32_t>(std::max(a, b)); }

	// Appends the sub-edges of u-v, in order from u to v.
	void Expand(int u, int v, std::vector<std::array<int, 2>>& out) const {
		auto it = mid_.find(Key(u, v));
		if (it == mid_.end()) { out.push_back({{u, v}}); return; }
		Expand(u, it->second, out);
		Expand(it->second, v, out);
	}
	// Splits u-v at its midpoint; the new id, or -1 if it was split already.
	int Split(int u, int v, bool on_seam) {
		uint64_t k = Key(u, v);
		if (mid_.count(k)) return -1;
		int id = table.Id((table.points[u] + table.points[v]) * 0.5);
		mid_[k] = id;
		if (on_seam) seam_.insert(id);
		return id;
	}
	bool IsSeam(int id) const { return seam_.count(id) > 0; }

private:
	std::unordered_map<uint64_t, int> mid_;
	std::unordered_set<int> seam_;
};

struct CutRefine {
	double min_angle_deg = 0.0; // 0: no refinement
	double floor = 0.0;         // no edge shorter than this is made
	int max_points = 0;         // per triangle
	// Grading: a bad piece is refined only if a seam point lies within
	// (shortest edge / grade) of its centroid, so refinement is confined
	// to where pieces are small against their distance from the seam and
	// does not spread over the surface.
	const std::vector<Vec3>* seam_points = nullptr;
	double grade = 0.5;
};

// Cuts a triangle (corners as vertex-table ids) along its segments, with
// `edge_points[k]` the points on edge k = (corner k, corner k+1), each with
// its fraction along the edge, then refines the pieces (Ruppert's algorithm
// with Ungor's off-centres): a piece with an angle under the bound, whose
// shortest edge is still short against the triangle, gets a point on the
// bisector of that edge as near it as the bound allows -- so the pieces
// grade from the seam's spacing up to the triangle's own, and the far
// corners are no longer fanned to a dense, curved seam (the one shape a
// Delaunay tetrahedralization cannot hold: no empty sphere passes through a
// far corner and a point of a tight curve). When that point would encroach
// a constrained sub-edge (lie in its diametral circle, which would leave
// the sub-edge without an empty circle of its own), the sub-edge is split
// at its midpoint instead, through `splits`, which the neighbours on it
// will see. Pieces whose small angle is between two constrained edges, or
// whose point would fall outside the triangle, on top of another, or
// nearer the floor, are left alone. New points lie in the triangle's plane
// to rounding.
void CutTriangle(const std::array<int, 3>& corner_ids, const std::vector<std::array<int, 2>>& segs, const std::vector<std::pair<double, int>>* edge_points, VertexTable& table, SubEdgeSplits& splits, const CutRefine& ro, MeshBooleanStats& st, std::vector<std::array<int, 3>>& out) {
	std::vector<int> global;
	std::unordered_map<int, int> local;
	auto add = [&](int g) { auto it = local.find(g); if (it != local.end()) return it->second; int l = static_cast<int>(global.size()); global.push_back(g); local[g] = l; return l; };
	for (int k = 0; k < 3; ++k) add(corner_ids[k]);
	// The outline: each edge as the polyline through its points, in order,
	// each stretch through its splits.
	std::vector<std::array<int, 2>> boundary, segments, expanded;
	for (int k = 0; k < 3; ++k) {
		std::vector<std::pair<double, int>> on = edge_points[k];
		std::sort(on.begin(), on.end());
		std::vector<int> chain{corner_ids[k]};
		for (const auto& sp : on) if (sp.second != chain.back()) chain.push_back(sp.second);
		if (chain.back() != corner_ids[(k + 1) % 3]) chain.push_back(corner_ids[(k + 1) % 3]);
		for (size_t i = 0; i + 1 < chain.size(); ++i) {
			expanded.clear();
			splits.Expand(chain[i], chain[i + 1], expanded);
			for (const auto& e : expanded) { int u = add(e[0]), v = add(e[1]); if (u != v) boundary.push_back({{u, v}}); }
		}
	}
	for (const auto& s : segs) {
		if (s[0] == s[1]) continue;
		expanded.clear();
		splits.Expand(s[0], s[1], expanded);
		for (const auto& e : expanded) { int u = add(e[0]), v = add(e[1]); if (u != v) segments.push_back({{u, v}}); }
	}
	const Vec3 c0 = table.points[corner_ids[0]], c1 = table.points[corner_ids[1]], c2 = table.points[corner_ids[2]];
	Frame2 f = FrameOf(c0, c1, c2);
	Cdt2 cdt;
	for (int g : global) cdt.pts.push_back(f.Of(table.points[g]));
	try {
		cdt.Build(boundary, segments);
	} catch (const std::exception& e) {
		if (std::getenv("AEROTELIC_BOOL_TRACE")) {
			std::printf("[bool] cut failed: %s\n", e.what());
			for (size_t i = 0; i < global.size(); ++i) std::printf("   point %zu: (%.17g, %.17g, %.17g) 2d (%.17g, %.17g)\n", i, table.points[global[i]].x, table.points[global[i]].y, table.points[global[i]].z, f.Of(table.points[global[i]])[0], f.Of(table.points[global[i]])[1]);
			for (const auto& c : boundary) std::printf("   boundary %d-%d\n", c[0], c[1]);
			for (const auto& c : segments) std::printf("   segment %d-%d\n", c[0], c[1]);
		}
		throw;
	}

	if (ro.min_angle_deg > 0.0 && (!segments.empty() || boundary.size() > 3)) {
		std::unordered_set<uint64_t> locked, on_seam;
		for (const auto& e : boundary) locked.insert(Cdt2::Key(e[0], e[1]));
		for (const auto& e : segments) { locked.insert(Cdt2::Key(e[0], e[1])); on_seam.insert(Cdt2::Key(e[0], e[1])); }
		// Lifting a 2-D point back to the plane: the dropped coordinate
		// from the plane equation (the normal's largest component).
		const Vec3 n = Cross(c1 - c0, c2 - c0);
		const int drop = 3 - f.ax - f.ay;
		auto lift = [&](const std::array<double, 2>& q) {
			Vec3 p;
			p[f.ax] = q[0];
			p[f.ay] = q[1];
			p[drop] = c0[drop] - (n[f.ax] * (q[0] - c0[f.ax]) + n[f.ay] * (q[1] - c0[f.ay])) / n[drop];
			return p;
		};
		auto dist2 = [&](int a, int b) { double dx = cdt.pts[a][0] - cdt.pts[b][0], dy = cdt.pts[a][1] - cdt.pts[b][1]; return dx * dx + dy * dy; };
		const double sin_min = std::sin(ro.min_angle_deg * 3.14159265358979323846 / 180.0);
		const double beta = 1.0 / (2.0 * sin_min);
		const double floor2 = ro.floor * ro.floor;
		std::unordered_set<uint64_t> skipped; // triangles given up on, by their sorted corners
		auto tri_key = [](std::array<int, 3> t) { std::sort(t.begin(), t.end()); return (static_cast<uint64_t>(static_cast<uint32_t>(t[0])) << 42) | (static_cast<uint64_t>(static_cast<uint32_t>(t[1])) << 21) | static_cast<uint64_t>(static_cast<uint32_t>(t[2])); };
		int why_h0 = 0, why_floor = 0, why_split_taken = 0, why_onedge = 0, why_outside = 0, why_crowded = 0, why_insert = 0, inserted = 0;
		// Whether a piece is near enough to the seam for its size to be
		// refined (CutRefine::grade), once per piece.
		std::unordered_map<uint64_t, bool> near_seam;
		auto near_enough = [&](const std::array<int, 3>& tr, double shortest2) {
			if (!ro.seam_points || ro.grade <= 0.0) return true;
			uint64_t k = tri_key(tr);
			auto it = near_seam.find(k);
			if (it != near_seam.end()) return it->second;
			const Vec3 g = (table.points[global[tr[0]]] + table.points[global[tr[1]]] + table.points[global[tr[2]]]) / 3.0;
			const double reach2 = shortest2 / (ro.grade * ro.grade);
			bool near = false;
			for (const Vec3& s : *ro.seam_points) if (DistanceSq(s, g) < reach2) { near = true; break; }
			near_seam[k] = near;
			return near;
		};
		for (; inserted < ro.max_points;) {
			// The worst piece that can be improved: smallest sine of its
			// smallest angle (at the corner opposite the shortest edge).
			int worst = -1, worst_k = 0;
			double worst_sin = sin_min;
			for (size_t t = 0; t < cdt.tris.size(); ++t) {
				const auto& tr = cdt.tris[t];
				if (tr[0] < 0 || skipped.count(tri_key(tr))) continue;
				int ks = 0;
				double ls2 = 1e300;
				for (int k = 0; k < 3; ++k) { double l2 = dist2(tr[k], tr[(k + 1) % 3]); if (l2 < ls2) { ls2 = l2; ks = k; } }
				if (ls2 < 4.0 * floor2) continue; // nothing shorter is made
				const int p = tr[ks], q = tr[(ks + 1) % 3], r = tr[(ks + 2) % 3];
				// Angle at r: sin = 2 area / (|rp| |rq|).
				double area2 = cdt.Orient(p, q, r);
				double s = area2 / std::sqrt(dist2(r, p) * dist2(r, q));
				if (s >= worst_sin) continue;
				if (locked.count(Cdt2::Key(r, p)) && locked.count(Cdt2::Key(r, q))) continue; // between two constraints: unfixable
				if (!near_enough(tr, ls2)) continue;
				worst = static_cast<int>(t);
				worst_k = ks;
				worst_sin = s;
			}
			if (worst < 0) break;
			const std::array<int, 3> tr = cdt.tris[worst];
			const int p = tr[worst_k], q = tr[(worst_k + 1) % 3], r = tr[(worst_k + 2) % 3];
			// The off-centre: on the bisector of p-q, towards r, where the
			// triangle p q c has circumradius beta |pq| (angle at c = bound),
			// or the circumcentre if that is nearer.
			const std::array<double, 2> P = cdt.pts[p], Q = cdt.pts[q], R = cdt.pts[r];
			const double L = std::sqrt(dist2(p, q));
			std::array<double, 2> m{{0.5 * (P[0] + Q[0]), 0.5 * (P[1] + Q[1])}};
			std::array<double, 2> dir{{-(Q[1] - P[1]) / L, (Q[0] - P[0]) / L}}; // left normal of p->q (r is on the left: counter-clockwise)
			double h_off = L * (beta + std::sqrt(beta * beta - 0.25));
			// Circumcentre height above p-q along dir.
			double bx = Q[0] - P[0], by = Q[1] - P[1], cx = R[0] - P[0], cy = R[1] - P[1];
			double d = 2.0 * (bx * cy - by * cx);
			double h_cc = h_off;
			if (d != 0.0) {
				double ux = (cy * (bx * bx + by * by) - by * (cx * cx + cy * cy)) / d, uy = (bx * (cx * cx + cy * cy) - cx * (bx * bx + by * by)) / d;
				h_cc = (P[0] + ux - m[0]) * dir[0] + (P[1] + uy - m[1]) * dir[1];
			}
			double h = std::min(h_off, h_cc);
			if (h <= 0.0) { skipped.insert(tri_key(tr)); ++why_h0; continue; }
			std::array<double, 2> c{{m[0] + dir[0] * h, m[1] + dir[1] * h}};
			// Encroachment: c inside the diametral circle of a constrained
			// sub-edge -> split that sub-edge (the nearest such) instead.
			auto encroached = [&](const std::array<double, 2>& x, int* eu, int* ev) {
				*eu = *ev = -1;
				double best = 1e300;
				for (uint64_t ek : locked) {
					int u = static_cast<int>(ek >> 32), v = static_cast<int>(ek & 0xffffffffu);
					double ax = cdt.pts[u][0] - x[0], ay = cdt.pts[u][1] - x[1], bx2 = cdt.pts[v][0] - x[0], by2 = cdt.pts[v][1] - x[1];
					if (ax * bx2 + ay * by2 >= 0.0) continue;
					double mx = 0.5 * (cdt.pts[u][0] + cdt.pts[v][0]) - x[0], my = 0.5 * (cdt.pts[u][1] + cdt.pts[v][1]) - x[1];
					double dm = mx * mx + my * my;
					if (dm < best) { best = dm; *eu = u; *ev = v; }
				}
				return *eu >= 0;
			};
			int eu = -1, ev = -1;
			encroached(c, &eu, &ev);
			std::array<double, 2> C0 = cdt.pts[0], C1 = cdt.pts[1], C2 = cdt.pts[2];
			const bool outside = orient2d(C0.data(), C1.data(), c.data()) <= 0.0 || orient2d(C1.data(), C2.data(), c.data()) <= 0.0 || orient2d(C2.data(), C0.data(), c.data()) <= 0.0;
			if (eu < 0 && outside) {
				// Off the triangle without encroaching anything: the
				// circumcentre does encroach what it crosses (Ruppert's
				// lemma); failing that, split the outline sub-edge the way
				// out crosses.
				std::array<double, 2> cc{{m[0] + dir[0] * h_cc, m[1] + dir[1] * h_cc}};
				if (!encroached(cc, &eu, &ev)) {
					std::array<double, 2> g{{(P[0] + Q[0] + R[0]) / 3.0, (P[1] + Q[1] + R[1]) / 3.0}};
					double nearest = 1e300;
					for (uint64_t ek : locked) {
						int u = static_cast<int>(ek >> 32), v = static_cast<int>(ek & 0xffffffffu);
						const auto& U = cdt.pts[u];
						const auto& V = cdt.pts[v];
						double o1 = orient2d(g.data(), c.data(), U.data()), o2 = orient2d(g.data(), c.data(), V.data());
						double o3 = orient2d(U.data(), V.data(), g.data()), o4 = orient2d(U.data(), V.data(), c.data());
						if (!(o1 * o2 < 0.0 && o3 * o4 < 0.0)) continue;
						// The crossing's distance from g along g->c, by the ratio of the orientations.
						double t = o3 / (o3 - o4);
						if (t < nearest) { nearest = t; eu = u; ev = v; }
					}
				}
			}
			if (eu >= 0) {
				if (dist2(eu, ev) < 4.0 * floor2) { skipped.insert(tri_key(tr)); ++why_floor; continue; }
				const bool seam_edge = on_seam.count(Cdt2::Key(eu, ev)) > 0;
				int gid = splits.Split(global[eu], global[ev], seam_edge);
				if (gid < 0) { skipped.insert(tri_key(tr)); ++why_split_taken; continue; } // split by a neighbour meanwhile: this triangle is cut again
				int lid = add(gid);
				cdt.pts.push_back(f.Of(table.points[gid]));
				if (!cdt.InsertOnEdge(lid, eu, ev, locked)) { skipped.insert(tri_key(tr)); ++why_onedge; continue; }
				if (seam_edge) { on_seam.erase(Cdt2::Key(eu, ev)); on_seam.insert(Cdt2::Key(eu, lid)); on_seam.insert(Cdt2::Key(lid, ev)); }
				++st.n_refine_splits;
				++inserted;
				continue;
			}
			// Inside the triangle, and not on top of a point.
			if (outside) { skipped.insert(tri_key(tr)); ++why_outside; continue; }
			bool crowded = false;
			for (const auto& pt : cdt.pts) { double dx = pt[0] - c[0], dy = pt[1] - c[1]; if (dx * dx + dy * dy < floor2) { crowded = true; break; } }
			if (crowded) { skipped.insert(tri_key(tr)); ++why_crowded; continue; }
			int gid = table.Id(lift(c));
			int lid = add(gid);
			cdt.pts.push_back(f.Of(table.points[gid]));
			if (!cdt.InsertConstrained(lid, locked)) { skipped.insert(tri_key(tr)); ++why_insert; continue; }
			++st.n_refine_points;
			++inserted;
		}
		if (std::getenv("AEROTELIC_BOOL_TRACE")) {
			// What was left unrefined, and why.
			int left = 0;
			for (const auto& tr : cdt.tris) {
				if (tr[0] < 0) continue;
				int ks = 0;
				double ls2 = 1e300;
				for (int k = 0; k < 3; ++k) { double l2 = dist2(tr[k], tr[(k + 1) % 3]); if (l2 < ls2) { ls2 = l2; ks = k; } }
				const int p = tr[ks], q = tr[(ks + 1) % 3], r = tr[(ks + 2) % 3];
				double s = cdt.Orient(p, q, r) / std::sqrt(dist2(r, p) * dist2(r, q));
				if (s < sin_min && !(locked.count(Cdt2::Key(r, p)) && locked.count(Cdt2::Key(r, q)))) ++left;
			}
			if (left > 0 || inserted >= ro.max_points) std::printf("[bool] cut triangle at (%.4f %.4f %.4f): %d bad pieces left, %d inserted%s; skipped: %d no height, %d floor, %d split taken, %d on-edge insert failed, %d outside, %d crowded, %d insert failed\n", (c0.x + c1.x + c2.x) / 3.0, (c0.y + c1.y + c2.y) / 3.0, (c0.z + c1.z + c2.z) / 3.0, left, inserted, inserted >= ro.max_points ? " (budget spent)" : "", why_h0, why_floor, why_split_taken, why_onedge, why_outside, why_crowded, why_insert);
		}
	}
	for (const auto& t : cdt.tris) if (t[0] >= 0) out.push_back({{global[t[0]], global[t[1]], global[t[2]]}});
}

// ---------------------------------------------------------------------------
// Seam slivers (step 5): collapses for needles, flips for caps
// ---------------------------------------------------------------------------

class SeamPass {
public:
	std::vector<Vec3>& pts;                 // vertex positions (vertex-table ids)
	std::vector<std::array<int, 3>>& tris;  // -1 marks a dead triangle
	const std::vector<char>& seam;          // per vertex
	const double angle_deg, max_move;
	MeshBooleanStats& st;
	std::vector<std::vector<int>> vt;       // vertex -> live triangles

	SeamPass(std::vector<Vec3>& p, std::vector<std::array<int, 3>>& t, const std::vector<char>& s, double deg, double move, MeshBooleanStats& st_)
		: pts(p), tris(t), seam(s), angle_deg(deg), max_move(move), st(st_) {
		vt.assign(pts.size(), {});
		for (size_t i = 0; i < tris.size(); ++i) for (int k = 0; k < 3; ++k) vt[tris[i][k]].push_back(static_cast<int>(i));
	}

	bool Alive(int t) const { return tris[t][0] >= 0; }
	bool TouchesSeam(int t) const { return seam[tris[t][0]] || seam[tris[t][1]] || seam[tris[t][2]]; }

	// Smallest angle of a triangle (degrees).
	double MinAngleOf(const std::array<int, 3>& tr) const {
		double worst = 180.0;
		for (int k = 0; k < 3; ++k) {
			Vec3 e1 = pts[tr[(k + 1) % 3]] - pts[tr[k]], e2 = pts[tr[(k + 2) % 3]] - pts[tr[k]];
			double l1 = Length(e1), l2 = Length(e2);
			double a = (l1 == 0.0 || l2 == 0.0) ? 0.0 : std::acos(std::clamp(Dot(e1, e2) / (l1 * l2), -1.0, 1.0)) * 180.0 / 3.14159265358979323846;
			worst = std::min(worst, a);
		}
		return worst;
	}
	double MinAngle(int t) const { return MinAngleOf(tris[t]); }
	bool IsSliver(int t) const { return Alive(t) && TouchesSeam(t) && MinAngle(t) < angle_deg; }

	Vec3 Normal(const std::array<int, 3>& tr) const { return Cross(pts[tr[1]] - pts[tr[0]], pts[tr[2]] - pts[tr[0]]); }

	// The triangles on edge u-v.
	std::vector<int> OnEdge(int u, int v) const {
		std::vector<int> out;
		for (int t : vt[u]) { if (!Alive(t)) continue; const auto& tr = tris[t]; if (tr[0] == v || tr[1] == v || tr[2] == v) out.push_back(t); }
		return out;
	}

	const char* why = "";  // the last refusal's reason

	// Collapses u onto v (u disappears) if that inverts nothing, keeps the
	// surface a manifold and moves the surface by at most max_move: the
	// surface moves by how far u is from the planes of its triangles once
	// they are re-attached to v (its distance along the edge is not a
	// movement when u, v and the triangles' other corners lie on the same
	// sheet, which is the needle case). False if refused.
	bool Collapse(int u, int v) {
		if (u == v) { why = "same vertex"; return false; }
		std::vector<int> shared = OnEdge(u, v);
		if (shared.size() != 2) { why = "edge not manifold"; return false; }
		// Link condition: the vertices adjacent to both u and v must be
		// exactly the two opposite the edge.
		std::unordered_set<int> link_u, link_v;
		for (int t : vt[u]) if (Alive(t)) for (int k = 0; k < 3; ++k) if (tris[t][k] != u) link_u.insert(tris[t][k]);
		for (int t : vt[v]) if (Alive(t)) for (int k = 0; k < 3; ++k) if (tris[t][k] != v) link_v.insert(tris[t][k]);
		int common = 0;
		for (int x : link_u) if (x != v && link_v.count(x)) ++common;
		if (common != 2) { why = "link condition"; return false; }
		// No inversion among u's surviving triangles, and the movement.
		double dev = 0.0;
		for (int t : vt[u]) {
			if (!Alive(t)) continue;
			if (std::find(shared.begin(), shared.end(), t) != shared.end()) continue;
			std::array<int, 3> tr = tris[t];
			Vec3 n0 = Normal(tr);
			for (int k = 0; k < 3; ++k) if (tr[k] == u) tr[k] = v;
			Vec3 n1 = Normal(tr);
			if (LengthSq(n1) == 0.0 || Dot(n0, n1) <= 0.0) { why = "would invert"; return false; }
			dev = std::max(dev, std::fabs(Dot(pts[u] - pts[v], n1)) / Length(n1));
		}
		if (dev > max_move) { why = "collapse moves the surface too far"; return false; }
		st.max_seam_move = std::max(st.max_seam_move, dev);
		// Apply.
		for (int t : shared) { tris[t] = {{-1, -1, -1}}; }
		for (int t : vt[u]) {
			if (!Alive(t)) continue;
			for (int k = 0; k < 3; ++k) if (tris[t][k] == u) tris[t][k] = v;
			vt[v].push_back(t);
		}
		vt[u].clear();
		return true;
	}

	// Flips edge a-b (triangles (a, b, c) and (b, a, d)) to c-d if the quad
	// is convex, c-d is a new edge, the worse of the two triangles improves,
	// and the surface moves by at most max_move. The old and new pairs are
	// the two pairs of faces of the tetrahedron a b c d, which lies in a slab
	// as thick as the distance between the skew edges a-b and c-d: that
	// distance bounds the movement (for a sliver whose far corner is a hair
	// off the long edge it is that hair, however the sheet folds along a-b).
	bool Flip(int a, int b) {
		std::vector<int> on = OnEdge(a, b);
		if (on.size() != 2) { why = "edge not manifold"; return false; }
		auto third = [&](int t, int x, int y, bool* fwd) {
			const auto& tr = tris[t];
			for (int k = 0; k < 3; ++k) {
				if (tr[k] == x && tr[(k + 1) % 3] == y) { *fwd = true; return tr[(k + 2) % 3]; }
				if (tr[k] == y && tr[(k + 1) % 3] == x) { *fwd = false; return tr[(k + 2) % 3]; }
			}
			return -1;
		};
		bool f1, f2;
		int t1 = on[0], t2 = on[1];
		int c = third(t1, a, b, &f1), d = third(t2, a, b, &f2);
		if (c < 0 || d < 0 || f1 == f2 || c == d) { why = "flip: inconsistent pair"; return false; }
		if (!f1) { std::swap(t1, t2); std::swap(c, d); }
		if (!OnEdge(c, d).empty()) { why = "flip: edge exists"; return false; }
		std::array<int, 3> n1{{a, d, c}}, n2{{b, c, d}};
		Vec3 old = Normal(tris[t1]) + Normal(tris[t2]);
		Vec3 m1 = Normal(n1), m2 = Normal(n2);
		if (LengthSq(m1) == 0.0 || LengthSq(m2) == 0.0 || Dot(m1, old) <= 0.0 || Dot(m2, old) <= 0.0) { why = "flip: not convex"; return false; }
		if (std::min(MinAngleOf(n1), MinAngleOf(n2)) <= std::min(MinAngle(t1), MinAngle(t2))) { why = "flip: no improvement"; return false; }
		Vec3 n = Cross(pts[b] - pts[a], pts[d] - pts[c]);
		if (LengthSq(n) == 0.0) { why = "flip: parallel diagonals"; return false; }
		double dev = std::fabs(Dot(pts[c] - pts[a], n)) / Length(n);
		if (dev > max_move) { why = "flip moves the surface too far"; return false; }
		st.max_seam_move = std::max(st.max_seam_move, dev);
		// Apply.
		for (int t : {t1, t2}) for (int k = 0; k < 3; ++k) { auto& lst = vt[tris[t][k]]; lst.erase(std::remove(lst.begin(), lst.end(), t), lst.end()); }
		tris[t1] = n1;
		tris[t2] = n2;
		for (int t : {t1, t2}) for (int k = 0; k < 3; ++k) vt[tris[t][k]].push_back(t);
		return true;
	}

	// One sliver: a needle is collapsed along its shortest edge (an
	// original vertex onto a seam point, never a seam point off the seam;
	// between two seam points or two original vertices, whichever way is
	// valid), a cap flipped on its longest edge; each falls back to the
	// other.
	bool Treat(int t) {
		const std::array<int, 3> tr = tris[t];
		int shortest = 0, longest = 0;
		double ls = 1e300, ll = -1.0;
		for (int k = 0; k < 3; ++k) {
			double l = Distance(pts[tr[k]], pts[tr[(k + 1) % 3]]);
			if (l < ls) { ls = l; shortest = k; }
			if (l > ll) { ll = l; longest = k; }
		}
		std::string reasons;
		auto collapse = [&]() {
			int u = tr[shortest], v = tr[(shortest + 1) % 3];
			if (seam[u] && !seam[v]) std::swap(u, v);
			if (Collapse(u, v)) { ++st.n_seam_collapses; return true; }
			reasons += std::string(" collapse: ") + why;
			if (seam[u] == seam[v] && Collapse(v, u)) { ++st.n_seam_collapses; return true; }
			if (seam[u] == seam[v]) reasons += std::string(" / ") + why;
			return false;
		};
		auto flip = [&]() {
			if (Flip(tr[longest], tr[(longest + 1) % 3])) { ++st.n_seam_flips; return true; }
			reasons += std::string(" flip: ") + why;
			return false;
		};
		const bool needle = ls < 0.25 * ll;
		if (needle ? (collapse() || flip()) : (flip() || collapse())) return true;
		++st.n_seam_refused;
		if (trace && n_traced++ < 60) {
			const Vec3 c = (pts[tr[0]] + pts[tr[1]] + pts[tr[2]]) / 3.0;
			std::printf("[bool] seam sliver refused: %.4f deg, edges %.3g..%.3g, seam %d%d%d, at (%.4f %.4f %.4f):%s\n", MinAngle(t), ls, ll, seam[tr[0]] ? 1 : 0, seam[tr[1]] ? 1 : 0, seam[tr[2]] ? 1 : 0, c.x, c.y, c.z, reasons.c_str());
		}
		return false;
	}

	const bool trace = std::getenv("AEROTELIC_BOOL_TRACE") != nullptr;
	int n_traced = 0;

	// Worst first; every collapse removes two triangles and every flip
	// raises the pair's smaller angle, so the passes terminate.
	void Run() {
		std::vector<std::pair<double, int>> queue;
		for (size_t t = 0; t < tris.size(); ++t) if (IsSliver(static_cast<int>(t))) queue.push_back({MinAngle(static_cast<int>(t)), static_cast<int>(t)});
		st.n_seam_slivers = static_cast<int>(queue.size());
		for (int pass = 0; pass < 8 && !queue.empty(); ++pass) {
			std::sort(queue.begin(), queue.end());
			int changed = 0;
			std::unordered_set<int> refused;
			for (const auto& q : queue) {
				int t = q.second;
				if (!IsSliver(t)) continue;
				if (Treat(t)) ++changed; else refused.insert(t);
			}
			// Re-collect: neighbours may have become slivers, treated ones are gone.
			queue.clear();
			for (size_t t = 0; t < tris.size(); ++t) if (IsSliver(static_cast<int>(t)) && !refused.count(static_cast<int>(t))) queue.push_back({MinAngle(static_cast<int>(t)), static_cast<int>(t)});
			if (changed == 0) break;
		}
		st.n_seam_slivers_left = 0;
		for (size_t t = 0; t < tris.size(); ++t) if (IsSliver(static_cast<int>(t))) ++st.n_seam_slivers_left;
	}
};

} // namespace

// ---------------------------------------------------------------------------
// Public
// ---------------------------------------------------------------------------

MeshBooleanOp ParseMeshBooleanOp(const std::string& name) {
	if (name == "fuse") return MeshBooleanOp::Fuse;
	if (name == "cut") return MeshBooleanOp::Cut;
	if (name == "common") return MeshBooleanOp::Common;
	throw std::invalid_argument("unknown boolean operation '" + name + "' (fuse, cut or common)");
}

const char* MeshBooleanOpName(MeshBooleanOp op) {
	switch (op) {
	case MeshBooleanOp::Fuse: return "fuse";
	case MeshBooleanOp::Cut: return "cut";
	default: return "common";
	}
}

TriMesh MeshBoolean(const TriMesh& a, const TriMesh& b, MeshBooleanOp op, MeshBooleanStats* stats_out, const MeshBooleanOptions& options) {
	EnsureExact();
	auto t_start = std::chrono::steady_clock::now();
	MeshBooleanStats st;
	if (!a.IsClosedManifold() || !(a.SignedVolume() > 0.0)) throw std::invalid_argument("MeshBoolean: the first body is not a closed, outward surface");
	if (!b.IsClosedManifold() || !(b.SignedVolume() > 0.0)) throw std::invalid_argument("MeshBoolean: the second body is not a closed, outward surface");
	st.volume_a = a.SignedVolume();
	st.volume_b = b.SignedVolume();

	// One vertex table over both bodies: coincident positions are one
	// vertex, and a vertex of B within the weld tolerance of a vertex of A
	// is that vertex (the rims of a welded wall).
	const double diagonal = std::max(a.Bounds().Diagonal(), b.Bounds().Diagonal());
	const double weld = 1e-9 * diagonal;
	VertexTable table;
	table.SetCell(std::max(weld * 4.0, 1e-12));
	std::vector<int> ida(a.vertices.size()), idb(b.vertices.size());
	for (size_t v = 0; v < a.vertices.size(); ++v) ida[v] = table.Id(a.vertices[v]);
	for (size_t v = 0; v < b.vertices.size(); ++v) idb[v] = table.IdNear(b.vertices[v], weld);
	auto tri_ids = [&](const std::vector<int>& ids, const std::array<int, 3>& t) { return std::array<int, 3>{{ids[t[0]], ids[t[1]], ids[t[2]]}}; };

	// 1. Coincident faces: the same three vertices.
	std::vector<char> dropped_a(a.triangles.size(), 0), dropped_b(b.triangles.size(), 0);
	{
		struct TriKey { int a, b, c; bool operator==(const TriKey& o) const { return a == o.a && b == o.b && c == o.c; } };
		struct TriHash { size_t operator()(const TriKey& k) const { return std::hash<uint64_t>()((static_cast<uint64_t>(k.a) * 0x9E3779B97F4A7C15ull) ^ (static_cast<uint64_t>(k.b) * 0xBF58476D1CE4E5B9ull) ^ static_cast<uint64_t>(k.c)); } };
		auto key_of = [](std::array<int, 3> t) { std::sort(t.begin(), t.end()); return TriKey{t[0], t[1], t[2]}; };
		std::unordered_map<TriKey, int, TriHash> faces_a;
		for (size_t t = 0; t < a.triangles.size(); ++t) faces_a.emplace(key_of(tri_ids(ida, a.triangles[t])), static_cast<int>(t));
		for (size_t t = 0; t < b.triangles.size(); ++t) {
			auto tb = tri_ids(idb, b.triangles[t]);
			auto it = faces_a.find(key_of(tb));
			if (it == faces_a.end()) continue;
			auto ta = tri_ids(ida, a.triangles[it->second]);
			// Same cyclic order -> same winding.
			bool same = false;
			for (int r = 0; r < 3 && !same; ++r) same = ta[0] == tb[r] && ta[1] == tb[(r + 1) % 3] && ta[2] == tb[(r + 2) % 3];
			const Vec3 c = (table.points[ta[0]] + table.points[ta[1]] + table.points[ta[2]]) / 3.0;
			if (same) throw MeshBooleanError("two triangles overlap in a common plane (the same face, wound the same way)", c);
			dropped_a[it->second] = 1;
			dropped_b[t] = 1;
			++st.n_coincident_pairs;
		}
	}

	// 2. Intersections.
	const double floor = 1e-9 * diagonal;
	Intersector inter(a, b, table, ida, idb, floor, st);
	{
		TriTree tree_b(b);
		std::vector<int> hits;
		TriTree tree_a(a);
		for (size_t ta = 0; ta < a.triangles.size(); ++ta) {
			if (dropped_a[ta]) continue;
			tree_b.Query(tree_a.TriangleBox(static_cast<int>(ta)), hits);
			for (int tb : hits) {
				if (dropped_b[tb]) continue;
				++st.n_pairs_tested;
				if (inter.Process(static_cast<int>(ta), tb)) ++st.n_pairs_intersecting;
			}
		}
	}

	// 3 + 4. Pieces, classified and kept.
	MeshSdf sdf_a(a), sdf_b(b);
	TriMesh out;
	auto emit = [&](const std::array<int, 3>& t, bool reversed) {
		if (reversed) out.triangles.push_back({{t[0], t[2], t[1]}});
		else out.triangles.push_back(t);
	};
	auto centroid = [&](const std::array<int, 3>& t) { return (table.points[t[0]] + table.points[t[1]] + table.points[t[2]]) / 3.0; };
	// Keep rule per body: side 0 = pieces of A (inside B?), side 1 = pieces of B (inside A?).
	auto keep = [&](int side, bool inside_other) {
		switch (op) {
		case MeshBooleanOp::Fuse: return !inside_other;
		case MeshBooleanOp::Common: return inside_other;
		default: return side == 0 ? !inside_other : inside_other; // Cut: A outside B, B inside A
		}
	};
	// The pieces of every triangle. A triangle's cut depends on the points
	// on its edges, its segments and the splits of those (shared with its
	// neighbours and with the other body's triangles on the same seam), so
	// the cuts are repeated, for the triangles whose constraints changed,
	// until a round changes nothing.
	SubEdgeSplits splits(table);
	std::vector<Vec3> seam_points;
	for (size_t v = 0; v < table.points.size(); ++v) if (inter.IsSeam(static_cast<int>(v))) seam_points.push_back(table.points[v]);
	CutRefine ro;
	ro.min_angle_deg = options.refine_min_angle_deg;
	ro.floor = options.refine_floor_fraction * diagonal;
	ro.max_points = options.refine_max_points;
	ro.seam_points = &seam_points;
	ro.grade = options.refine_grade;
	struct Cut { std::vector<std::array<int, 3>> pieces; std::vector<int> signature; bool built = false; };
	std::vector<Cut> cuts[2] = {std::vector<Cut>(a.triangles.size()), std::vector<Cut>(b.triangles.size())};
	std::vector<int> signature;
	std::vector<std::array<int, 2>> expanded;
	const int max_rounds = 100;
	for (int round = 0; round < max_rounds; ++round) {
		bool changed = false;
		for (int side = 0; side < 2; ++side) {
			const TriMesh& m = side == 0 ? a : b;
			const std::vector<int>& ids = side == 0 ? ida : idb;
			const std::vector<char>& dropped = side == 0 ? dropped_a : dropped_b;
			const auto& segs = side == 0 ? inter.seg_a : inter.seg_b;
			for (size_t t = 0; t < m.triangles.size(); ++t) {
				if (dropped[t]) continue;
				Cut& cut = cuts[side][t];
				const auto corners = tri_ids(ids, m.triangles[t]);
				const auto& tri = m.triangles[t];
				std::vector<std::pair<double, int>> edge_points[3];
				bool plain = segs[t].empty();
				signature.clear();
				for (int k = 0; k < 3; ++k) {
					edge_points[k] = inter.EdgePoints(side, tri[k], tri[(k + 1) % 3]);
					std::sort(edge_points[k].begin(), edge_points[k].end());
					plain = plain && edge_points[k].empty();
					std::vector<int> chain{corners[k]};
					for (const auto& sp : edge_points[k]) chain.push_back(sp.second);
					chain.push_back(corners[(k + 1) % 3]);
					for (size_t i = 0; i + 1 < chain.size(); ++i) {
						expanded.clear();
						splits.Expand(chain[i], chain[i + 1], expanded);
						for (const auto& e : expanded) signature.push_back(e[0]);
						plain = plain && expanded.size() == 1;
					}
				}
				for (const auto& s : segs[t]) {
					expanded.clear();
					splits.Expand(s[0], s[1], expanded);
					for (const auto& e : expanded) { signature.push_back(e[0]); signature.push_back(e[1]); }
				}
				if (cut.built && signature == cut.signature) continue;
				const bool first = !cut.built;
				cut.pieces.clear();
				if (plain) cut.pieces.push_back(corners);
				else CutTriangle(corners, segs[t], edge_points, table, splits, ro, st, cut.pieces);
				if (first && !plain) ++st.n_triangles_cut;
				cut.signature = signature;
				cut.built = true;
				changed = true;
			}
		}
		st.n_refine_rounds = round + 1;
		if (!changed) break;
		if (round + 1 == max_rounds) throw std::runtime_error("MeshBoolean: the refinement of the cut triangles did not settle in " + std::to_string(max_rounds) + " rounds");
	}

	// 4. Classification, by connected regions: pieces joined across any
	// edge that is not a seam segment are on the same side of the other
	// body, so a region takes the sign of the piece farthest from that
	// body's surface -- a sliver piece hugging the seam cannot be
	// misjudged by a distance of rounding size. A region whose pieces
	// disagree beyond rounding (a seam with a gap in it) falls back to
	// judging each piece by itself.
	const double agree = 1e-7 * diagonal;
	for (int side = 0; side < 2; ++side) {
		const TriMesh& m = side == 0 ? a : b;
		const std::vector<char>& dropped = side == 0 ? dropped_a : dropped_b;
		const auto& segs = side == 0 ? inter.seg_a : inter.seg_b;
		const MeshSdf& other = side == 0 ? sdf_b : sdf_a;
		const bool reversed = op == MeshBooleanOp::Cut && side == 1;
		int& kept = side == 0 ? st.n_kept_a : st.n_kept_b;
		std::vector<std::array<int, 3>> all;
		std::unordered_set<uint64_t> barrier;
		for (size_t t = 0; t < m.triangles.size(); ++t) {
			if (dropped[t]) continue;
			const Cut& cut = cuts[side][t];
			if (cut.pieces.size() > 1) st.n_pieces += static_cast<int>(cut.pieces.size());
			all.insert(all.end(), cut.pieces.begin(), cut.pieces.end());
			for (const auto& s : segs[t]) {
				expanded.clear();
				splits.Expand(s[0], s[1], expanded);
				for (const auto& e : expanded) barrier.insert(SubEdgeSplits::Key(e[0], e[1]));
			}
		}
		std::unordered_map<uint64_t, std::array<int, 2>> on_edge;
		on_edge.reserve(all.size() * 2);
		for (size_t p = 0; p < all.size(); ++p) for (int k = 0; k < 3; ++k) {
			auto& e = on_edge.try_emplace(SubEdgeSplits::Key(all[p][k], all[p][(k + 1) % 3]), std::array<int, 2>{{-1, -1}}).first->second;
			if (e[0] < 0) e[0] = static_cast<int>(p); else e[1] = static_cast<int>(p);
		}
		std::vector<double> dist(all.size());
		for (size_t p = 0; p < all.size(); ++p) dist[p] = other.Distance(centroid(all[p]));
		std::vector<int> region(all.size(), -1);
		std::vector<int> stack, members;
		for (size_t seed = 0; seed < all.size(); ++seed) {
			if (region[seed] >= 0) continue;
			const int r = static_cast<int>(seed);
			members.clear();
			stack.assign(1, r);
			region[seed] = r;
			double most_in = 0.0, most_out = 0.0; // the farthest inside (negative) and outside
			while (!stack.empty()) {
				int p = stack.back();
				stack.pop_back();
				members.push_back(p);
				most_in = std::min(most_in, dist[p]);
				most_out = std::max(most_out, dist[p]);
				for (int k = 0; k < 3; ++k) {
					uint64_t ek = SubEdgeSplits::Key(all[p][k], all[p][(k + 1) % 3]);
					if (barrier.count(ek)) continue;
					const auto& e = on_edge[ek];
					int o = e[0] == p ? e[1] : e[0];
					if (o >= 0 && region[o] < 0) { region[o] = r; stack.push_back(o); }
				}
			}
			const bool conflict = -most_in > agree && most_out > agree;
			const bool region_inside = -most_in > most_out;
			for (int p : members) {
				bool inside = conflict ? dist[p] < 0.0 : region_inside;
				if (!keep(side, inside)) continue;
				emit(all[p], reversed);
				++kept;
			}
		}
	}

	// 5. Seam slivers.
	if (options.seam_sliver_deg > 0.0 && !out.triangles.empty()) {
		std::vector<char> seam(table.points.size(), 0);
		for (size_t v = 0; v < seam.size(); ++v) seam[v] = (inter.IsSeam(static_cast<int>(v)) || splits.IsSeam(static_cast<int>(v))) ? 1 : 0;
		SeamPass pass(table.points, out.triangles, seam, options.seam_sliver_deg, options.seam_move_fraction * diagonal, st);
		pass.Run();
		std::vector<std::array<int, 3>> live;
		for (const auto& t : out.triangles) if (t[0] >= 0) live.push_back(t);
		out.triangles = std::move(live);
	}

	// Assemble: only the vertices used, in table order.
	std::vector<int> remap(table.points.size(), -1);
	for (const auto& t : out.triangles) for (int k = 0; k < 3; ++k) remap[t[k]] = 0;
	int n = 0;
	for (size_t v = 0; v < remap.size(); ++v) if (remap[v] == 0) { remap[v] = n++; out.vertices.push_back(table.points[v]); }
	for (auto& t : out.triangles) for (int k = 0; k < 3; ++k) t[k] = remap[t[k]];
	out.RemoveDegenerateTriangles();
	if (!out.triangles.empty()) {
		if (!out.IsClosedManifold()) {
			// Count the open edges for the message.
			std::unordered_map<uint64_t, int> use;
			for (const auto& t : out.triangles) for (int k = 0; k < 3; ++k) {
				int x = t[k], y = t[(k + 1) % 3];
				++use[(static_cast<uint64_t>(static_cast<uint32_t>(std::min(x, y))) << 32) | static_cast<uint32_t>(std::max(x, y))];
			}
			int open = 0;
			std::string where;
			const bool trace = std::getenv("AEROTELIC_BOOL_TRACE") != nullptr;
			// Table id of each output vertex, for the trace.
			std::vector<int> table_of(out.vertices.size(), -1);
			for (size_t v = 0; v < remap.size(); ++v) if (remap[v] >= 0) table_of[remap[v]] = static_cast<int>(v);
			for (const auto& kv : use) {
				if (kv.second == 2) continue;
				if (open++ < 5) {
					const int xi = static_cast<int>(kv.first >> 32), yi = static_cast<int>(kv.first & 0xffffffffu);
					const Vec3& x = out.vertices[xi];
					const Vec3& y = out.vertices[yi];
					char buf[160];
					std::snprintf(buf, sizeof buf, " [%d uses, (%.6g %.6g %.6g)-(%.6g %.6g %.6g)]", kv.second, x.x, x.y, x.z, y.x, y.y, y.z);
					where += buf;
					if (trace) {
						auto flag = [&](int ov) { int tid = table_of[ov]; return std::string(inter.IsSeam(tid) ? "seam" : (splits.IsSeam(tid) ? "seam-split" : (tid < static_cast<int>(a.vertices.size() + b.vertices.size()) ? "input" : "new"))) + "#" + std::to_string(tid); };
						std::printf("[bool] open edge %s - %s, %d uses:\n", flag(xi).c_str(), flag(yi).c_str(), kv.second);
						for (const auto& t : out.triangles) {
							bool has_x = t[0] == xi || t[1] == xi || t[2] == xi, has_y = t[0] == yi || t[1] == yi || t[2] == yi;
							if (has_x && has_y) std::printf("   triangle %s %s %s\n", flag(t[0]).c_str(), flag(t[1]).c_str(), flag(t[2]).c_str());
						}
					}
				}
			}
			throw std::runtime_error("MeshBoolean: the result is not a closed surface (" + std::to_string(open) + " edges not shared by exactly two triangles)" + where);
		}
		st.volume_result = out.SignedVolume();
	}
	st.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
	if (stats_out) *stats_out = st;
	return out;
}

} // namespace aerotelic
