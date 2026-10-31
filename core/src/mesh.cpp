#include "aerotelic/mesh.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace aerotelic {

namespace {

constexpr double kPi = 3.14159265358979323846;

uint64_t EdgeKey(int a, int b) {
	uint32_t lo = static_cast<uint32_t>(std::min(a, b)), hi = static_cast<uint32_t>(std::max(a, b));
	return (static_cast<uint64_t>(lo) << 32) | hi;
}

Vec3 RowVec(const Mat& m, int i) { return {m(i, 0), m(i, 1), m(i, 2)}; }

// Distance from p to the segment ab.
double PointSegmentDistance(const Vec3& p, const Vec3& a, const Vec3& b) {
	Vec3 ab = b - a;
	double l2 = LengthSq(ab);
	if (l2 <= 0.0) return Distance(p, a);
	double t = std::clamp(Dot(p - a, ab) / l2, 0.0, 1.0);
	return Distance(p, a + ab * t);
}

std::vector<double> SortedUnique(std::vector<double> v, double eps = 1e-12) {
	std::sort(v.begin(), v.end());
	std::vector<double> out;
	for (double x : v) {
		if (out.empty() || x - out.back() > eps) out.push_back(x);
	}
	return out;
}

// Ensures at least `min_count` entries by inserting uniform points in the
// largest gaps.
// `period` is the wrap-around length of a closed parameter set (the last
// interval runs from params.back() to params.front() + period).
void EnsureMinimum(std::vector<double>& params, int min_count, bool closed, double period) {
	while (static_cast<int>(params.size()) < min_count) {
		int worst = -1;
		double worst_gap = -1.0;
		int n = static_cast<int>(params.size());
		int n_intervals = closed ? n : n - 1;
		for (int i = 0; i < n_intervals; ++i) {
			double b = (i + 1 < n) ? params[i + 1] : params[0] + period;
			double gap = b - params[i];
			if (gap > worst_gap) { worst_gap = gap; worst = i; }
		}
		if (worst < 0 || worst_gap <= 1e-9) break;
		double b = (worst + 1 < n) ? params[worst + 1] : params[0] + period;
		double mid = 0.5 * (params[worst] + b);
		if (closed && mid >= params[0] + period) mid -= period;
		params.push_back(mid);
		std::sort(params.begin(), params.end());
	}
}

// The angle at `at` between the directions to x and y.
double AngleAt(const Vec3& at, const Vec3& x, const Vec3& y) {
	Vec3 u = x - at, v = y - at;
	double lu = Length(u), lv = Length(v);
	if (lu == 0.0 || lv == 0.0) return 0.0;
	return std::acos(std::clamp(Dot(u, v) / (lu * lv), -1.0, 1.0));
}

// Is the quad p -> q -> r -> s convex (its four turns all the same way in
// its own mean plane)? A strip quad between two rings is not when the
// rings twist or lean a lot relative to the point spacing.
bool QuadConvex(const Vec3& p, const Vec3& q, const Vec3& r, const Vec3& s) {
	Vec3 n = Cross(r - p, s - q);
	if (LengthSq(n) == 0.0) return false;
	const Vec3* c[4] = {&p, &q, &r, &s};
	int sign = 0;
	for (int i = 0; i < 4; ++i) {
		const Vec3& a = *c[i];
		const Vec3& b = *c[(i + 1) % 4];
		const Vec3& d = *c[(i + 2) % 4];
		double turn = Dot(Cross(b - a, d - b), n);
		int sg = turn > 0.0 ? 1 : (turn < 0.0 ? -1 : 0);
		if (sg == 0) return false;
		if (sign == 0) sign = sg;
		else if (sg != sign) return false;
	}
	return true;
}

// For the convex quad p -> q -> r -> s: is the diagonal p-r the locally
// Delaunay one? It is when the angles at the two corners it does not touch
// (q and s) sum to at most 180 degrees -- the circumcircle of p, q, r then
// excludes s and that of p, r, s excludes q. A cocircular quad (both
// diagonals Delaunay, the ruled quads of a surface of revolution) takes the
// shorter diagonal. On a non-planar quad this is the usual approximation,
// and it is what makes the strip's own triangulation Delaunay within its
// sheet, so that a tetrahedral mesh (tet.hpp) finds every strip edge in
// the Delaunay tetrahedralization of the vertices instead of having to
// split it: a diagonal that is not locally Delaunay has a vertex of its
// own sheet inside its circumcircle, and no sphere through it can be empty.
bool DiagonalDelaunay(const Vec3& p, const Vec3& q, const Vec3& r, const Vec3& s) {
	double sum = AngleAt(q, p, r) + AngleAt(s, p, r);
	if (std::fabs(sum - kPi) <= 1e-9) return DistanceSq(p, r) <= DistanceSq(q, s);
	return sum < kPi;
}

// 2-D ear clipping helpers.
double Cross2(double ax, double ay, double bx, double by) { return ax * by - ay * bx; }

bool PointInTriangle2(double px, double py, double ax, double ay, double bx, double by, double cx, double cy) {
	double d1 = Cross2(bx - ax, by - ay, px - ax, py - ay);
	double d2 = Cross2(cx - bx, cy - by, px - bx, py - by);
	double d3 = Cross2(ax - cx, ay - cy, px - cx, py - cy);
	bool has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
	bool has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
	return !(has_neg && has_pos);
}

} // namespace

// ---------------------------------------------------------------------------
// TriMesh
// ---------------------------------------------------------------------------

void TriMesh::Append(const TriMesh& other) {
	int offset = static_cast<int>(vertices.size());
	vertices.insert(vertices.end(), other.vertices.begin(), other.vertices.end());
	triangles.reserve(triangles.size() + other.triangles.size());
	for (const auto& t : other.triangles) triangles.push_back({t[0] + offset, t[1] + offset, t[2] + offset});
}

Box3 TriMesh::Bounds() const {
	Box3 b;
	for (const auto& v : vertices) b.Add(v);
	return b;
}

double TriMesh::SignedVolume() const {
	double v = 0.0;
	for (const auto& t : triangles) {
		const Vec3& a = vertices[t[0]];
		const Vec3& b = vertices[t[1]];
		const Vec3& c = vertices[t[2]];
		v += Dot(a, Cross(b, c));
	}
	return v / 6.0;
}

double TriMesh::SurfaceArea() const {
	double a = 0.0;
	for (const auto& t : triangles) {
		a += 0.5 * Length(Cross(vertices[t[1]] - vertices[t[0]], vertices[t[2]] - vertices[t[0]]));
	}
	return a;
}

void TriMesh::FlipWinding() {
	for (auto& t : triangles) std::swap(t[1], t[2]);
}

void TriMesh::FlipIfInsideOut() {
	if (SignedVolume() < 0.0) FlipWinding();
}

void TriMesh::Transform(const std::array<double, 9>& m, const Vec3& t) {
	for (auto& v : vertices) {
		Vec3 r{m[0] * v.x + m[1] * v.y + m[2] * v.z, m[3] * v.x + m[4] * v.y + m[5] * v.z, m[6] * v.x + m[7] * v.y + m[8] * v.z};
		v = r + t;
	}
	double det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
	if (det < 0.0) FlipWinding();
}

void TriMesh::Translate(const Vec3& t) {
	for (auto& v : vertices) v += t;
}

void TriMesh::Mirror(const Vec3& point, const Vec3& normal) {
	Vec3 n = Normalized(normal);
	for (auto& v : vertices) v = v - n * (2.0 * Dot(v - point, n));
	FlipWinding();
}

void TriMesh::WeldVertices(double tol) {
	if (vertices.empty()) return;
	const double cell = std::max(tol, 1e-12);
	struct CellHash {
		size_t operator()(const std::array<int64_t, 3>& k) const {
			return static_cast<size_t>(k[0] * 73856093LL ^ k[1] * 19349663LL ^ k[2] * 83492791LL);
		}
	};
	std::unordered_map<std::array<int64_t, 3>, std::vector<int>, CellHash> grid;
	std::vector<int> remap(vertices.size(), -1);
	std::vector<Vec3> kept;
	kept.reserve(vertices.size());
	auto key_of = [&](const Vec3& p) {
		return std::array<int64_t, 3>{static_cast<int64_t>(std::floor(p.x / cell)), static_cast<int64_t>(std::floor(p.y / cell)),
			static_cast<int64_t>(std::floor(p.z / cell))};
	};
	const double tol2 = tol * tol;
	for (size_t i = 0; i < vertices.size(); ++i) {
		const Vec3& p = vertices[i];
		auto k = key_of(p);
		int found = -1;
		for (int dx = -1; dx <= 1 && found < 0; ++dx)
			for (int dy = -1; dy <= 1 && found < 0; ++dy)
				for (int dz = -1; dz <= 1 && found < 0; ++dz) {
					auto it = grid.find({k[0] + dx, k[1] + dy, k[2] + dz});
					if (it == grid.end()) continue;
					for (int j : it->second) {
						if (DistanceSq(kept[j], p) <= tol2) { found = j; break; }
					}
				}
		if (found < 0) {
			found = static_cast<int>(kept.size());
			kept.push_back(p);
			grid[k].push_back(found);
		}
		remap[i] = found;
	}
	vertices = std::move(kept);
	for (auto& t : triangles) {
		t = {remap[t[0]], remap[t[1]], remap[t[2]]};
	}
	RemoveDegenerateTriangles();
}

void TriMesh::RemoveDegenerateTriangles(double area_eps) {
	std::vector<std::array<int, 3>> kept;
	kept.reserve(triangles.size());
	for (const auto& t : triangles) {
		if (t[0] == t[1] || t[1] == t[2] || t[0] == t[2]) continue;
		if (area_eps > 0.0) {
			double a2 = Length(Cross(vertices[t[1]] - vertices[t[0]], vertices[t[2]] - vertices[t[0]]));
			if (a2 <= 2.0 * area_eps) continue;
		}
		kept.push_back(t);
	}
	triangles = std::move(kept);
}

bool TriMesh::IsClosedManifold() const {
	if (triangles.empty()) return false;
	// For each undirected edge: how many times it appears, and the net
	// direction (+1 for a->b with a<b, -1 for the reverse).
	std::unordered_map<uint64_t, std::pair<int, int>> edges;
	edges.reserve(triangles.size() * 3);
	for (const auto& t : triangles) {
		for (int k = 0; k < 3; ++k) {
			int a = t[k], b = t[(k + 1) % 3];
			if (a == b) return false;
			auto& e = edges[EdgeKey(a, b)];
			e.first += 1;
			e.second += (a < b) ? 1 : -1;
		}
	}
	for (const auto& [key, e] : edges) {
		if (e.first != 2 || e.second != 0) return false;
	}
	return true;
}

Vec3 TriMesh::TriangleNormal(int t) const {
	const auto& tri = triangles[t];
	return Normalized(Cross(vertices[tri[1]] - vertices[tri[0]], vertices[tri[2]] - vertices[tri[0]]));
}

// ---------------------------------------------------------------------------
// Strips and caps
// ---------------------------------------------------------------------------

void AppendRuledStrip(TriMesh& mesh, const std::vector<int>& ring_a, const std::vector<int>& ring_b, bool closed) {
	if (ring_a.size() != ring_b.size()) throw std::invalid_argument("AppendRuledStrip: rings differ in size");
	int n = static_cast<int>(ring_a.size());
	int n_quads = closed ? n : n - 1;
	// How often each vertex occurs in its ring. A ring squeezed flat (a
	// knife edge: a fuselage end of zero width) retraces itself, so every
	// vertex of it but the two ends occurs twice; the quads on either side
	// of an end then mirror each other across the flat section, and the
	// diagonal from the repeated vertex next to the end lies IN the flat
	// section, so both quads would split along the same segment and leave
	// a coincident, oppositely wound pair of slivers behind (an edge with
	// four triangles). Preferring the diagonal from the unrepeated end
	// vertex keeps both quads' triangles off the section and the mesh a
	// manifold. Elsewhere the counts tie and the shorter diagonal decides.
	std::unordered_map<int, int> count;
	count.reserve(2 * ring_a.size());
	for (int idx : ring_a) ++count[idx];
	for (int idx : ring_b) ++count[idx];
	auto repeated = [&](int idx) { return count[idx] > 1 ? 1 : 0; };
	for (int j = 0; j < n_quads; ++j) {
		int j1 = (j + 1) % n;
		int a0 = ring_a[j], a1 = ring_a[j1], b1 = ring_b[j1], b0 = ring_b[j];
		// Quad a0 -> a1 -> b1 -> b0. Collapsed edges (repeated indices) turn
		// it into a triangle or nothing.
		int ids[4] = {a0, a1, b1, b0};
		int distinct = 0;
		int uniq[4];
		for (int k = 0; k < 4; ++k) {
			bool seen = false;
			for (int m = 0; m < distinct; ++m) {
				if (uniq[m] == ids[k]) { seen = true; break; }
			}
			if (!seen) uniq[distinct++] = ids[k];
		}
		if (distinct < 3) continue;
		if (distinct == 3) {
			// Keep the cyclic order of the three distinct corners.
			mesh.triangles.push_back({uniq[0], uniq[1], uniq[2]});
			continue;
		}
		const Vec3& pa0 = mesh.vertices[a0];
		const Vec3& pa1 = mesh.vertices[a1];
		const Vec3& pb1 = mesh.vertices[b1];
		const Vec3& pb0 = mesh.vertices[b0];
		// Split along the diagonal touching fewer repeated vertices (see
		// above), then the shorter one: a0-b1 or a1-b0.
		int rep_a0b1 = repeated(a0) + repeated(b1), rep_a1b0 = repeated(a1) + repeated(b0);
		bool use_a0b1 = rep_a0b1 != rep_a1b0 ? rep_a0b1 < rep_a1b0 : DistanceSq(pa0, pb1) <= DistanceSq(pa1, pb0);
		if (use_a0b1) {
			mesh.triangles.push_back({a0, a1, b1});
			mesh.triangles.push_back({a0, b1, b0});
		} else {
			mesh.triangles.push_back({a0, a1, b0});
			mesh.triangles.push_back({a1, b1, b0});
		}
	}
}

void AppendZigZagStrip(TriMesh& mesh, const std::vector<int>& ring_a, const std::vector<double>& va, const std::vector<int>& ring_b, const std::vector<double>& vb, bool closed) {
	const int na = static_cast<int>(ring_a.size()), nb = static_cast<int>(ring_b.size());
	if (na == 0 || nb == 0) return;
	if (static_cast<int>(va.size()) != na || static_cast<int>(vb.size()) != nb) throw std::invalid_argument("AppendZigZagStrip: parameter lists do not match the rings");
	// Repeated-vertex counts, for the quad diagonal rule of AppendRuledStrip.
	std::unordered_map<int, int> count;
	for (int idx : ring_a) ++count[idx];
	for (int idx : ring_b) ++count[idx];
	auto repeated = [&](int idx) { return count[idx] > 1 ? 1 : 0; };
	auto emit = [&](int x, int y, int z) {
		if (x == y || y == z || z == x) return;
		mesh.triangles.push_back({x, y, z});
	};
	// Which diagonal of the quad a0 -> a1 -> b1 -> b0: the one touching fewer
	// repeated (knife-edge) vertices, else the locally Delaunay one when the
	// quad is convex, else the shorter. True: a0-b1.
	auto prefer_a0b1 = [&](int a0, int a1, int b1, int b0) {
		int rep_a0b1 = repeated(a0) + repeated(b1), rep_a1b0 = repeated(a1) + repeated(b0);
		if (rep_a0b1 != rep_a1b0) return rep_a0b1 < rep_a1b0;
		const Vec3& A0 = mesh.vertices[a0];
		const Vec3& A1 = mesh.vertices[a1];
		const Vec3& B1 = mesh.vertices[b1];
		const Vec3& B0 = mesh.vertices[b0];
		if (QuadConvex(A0, A1, B1, B0)) return DiagonalDelaunay(A0, A1, B1, B0);
		return DistanceSq(A0, B1) <= DistanceSq(A1, B0);
	};
	auto quad = [&](int a0, int a1, int b1, int b0) {
		int ids[4] = {a0, a1, b1, b0};
		int distinct = 0, uniq[4];
		for (int k = 0; k < 4; ++k) {
			bool seen = false;
			for (int m = 0; m < distinct; ++m) if (uniq[m] == ids[k]) { seen = true; break; }
			if (!seen) uniq[distinct++] = ids[k];
		}
		if (distinct < 3) return;
		if (distinct == 3) { mesh.triangles.push_back({uniq[0], uniq[1], uniq[2]}); return; }
		if (prefer_a0b1(a0, a1, b1, b0)) { emit(a0, a1, b1); emit(a0, b1, b0); }
		else { emit(a0, a1, b0); emit(a1, b1, b0); }
	};
	// Walk both rings by parameter from v = 0. Equal next parameters advance
	// both (a quad, split by the rules above). Otherwise one ring advances
	// with a triangle: the one whose next parameter is smaller, which keeps
	// the two rings in step and the triangles on the ruled surface the
	// stations were refined for; but where the two candidate triangles are
	// the halves of a convex quad whose parameters are within one local
	// step of each other, the locally Delaunay one is laid instead -- the
	// parametric rule alone makes fans of needles between a sparse ring and
	// a dense one wherever the strip leans, and a needle has the dense
	// ring's next vertex inside its circumcircle. (A free Delaunay merge of
	// the two rings is NOT wanted: on a sheared strip it leaves the ruled
	// surface and cuts the corners of the section.) Closed rings wrap to
	// their first node at v = 1.
	const int steps_a = closed ? na : na - 1, steps_b = closed ? nb : nb - 1;
	int i = 0, j = 0;
	auto next_v = [&](const std::vector<double>& v, int k, int steps) { return k + 1 < static_cast<int>(v.size()) ? v[k + 1] : (closed ? v[0] + 1.0 : 2.0); };
	auto cur_v = [&](const std::vector<double>& v, int k) { return k < static_cast<int>(v.size()) ? v[k] : v[0] + 1.0; };
	auto node = [&](const std::vector<int>& ring, int k) { return ring[k % static_cast<int>(ring.size())]; };
	int guard = 0;
	while ((i < steps_a || j < steps_b) && guard++ < na + nb + 2) {
		bool can_a = i < steps_a, can_b = j < steps_b;
		double nva = can_a ? next_v(va, i, steps_a) : 2.0, nvb = can_b ? next_v(vb, j, steps_b) : 2.0;
		if (can_a && can_b && std::fabs(nva - nvb) <= 1e-12) {
			quad(node(ring_a, i), node(ring_a, i + 1), node(ring_b, j + 1), node(ring_b, j));
			++i; ++j;
			continue;
		}
		bool advance_a = can_a && (!can_b || nva < nvb);
		if (can_a && can_b) {
			double step = std::max(nva - cur_v(va, i), nvb - cur_v(vb, j));
			if (std::fabs(nva - nvb) <= step) {
				int a0 = node(ring_a, i), a1 = node(ring_a, i + 1), b1 = node(ring_b, j + 1), b0 = node(ring_b, j);
				if (a0 != a1 && a1 != b1 && b1 != b0 && b0 != a0 && a0 != b1 && a1 != b0 && !repeated(a0) && !repeated(a1) && !repeated(b0) && !repeated(b1)) {
					const Vec3& A0 = mesh.vertices[a0];
					const Vec3& A1 = mesh.vertices[a1];
					const Vec3& B1 = mesh.vertices[b1];
					const Vec3& B0 = mesh.vertices[b0];
					// Advancing a lays (a0, a1, b0), the diagonal a1-b0; advancing
					// b lays (a0, b1, b0), the diagonal a0-b1.
					if (QuadConvex(A0, A1, B1, B0)) advance_a = !DiagonalDelaunay(A0, A1, B1, B0);
				}
			}
		}
		if (advance_a) {
			emit(node(ring_a, i), node(ring_a, i + 1), node(ring_b, j));
			++i;
		} else {
			emit(node(ring_a, i), node(ring_b, j + 1), node(ring_b, j));
			++j;
		}
	}
}

bool AppendPolygonCap(TriMesh& mesh, const std::vector<int>& ring_in) {
	// Drop consecutive repeats (and a closing repeat of the first).
	std::vector<int> ring;
	for (int idx : ring_in) {
		if (ring.empty() || ring.back() != idx) ring.push_back(idx);
	}
	while (ring.size() > 1 && ring.back() == ring.front()) ring.pop_back();
	int n = static_cast<int>(ring.size());
	if (n < 3) return false;
	if (n == 3) {
		mesh.triangles.push_back({ring[0], ring[1], ring[2]});
		return true;
	}

	// Newell's normal, then project onto the plane dropping the dominant axis.
	// A polygon with no area -- a section squeezed flat into a line, traced
	// out and back -- must add nothing: judged against its own perimeter,
	// not against exact zero, since the rounding in the sums leaves ~1e-17
	// behind and ear clipping that would emit collinear triangles that
	// break the manifold along the flat section's edge.
	Vec3 normal{0, 0, 0};
	double perimeter2 = 0.0;
	for (int i = 0; i < n; ++i) {
		const Vec3& p = mesh.vertices[ring[i]];
		const Vec3& q = mesh.vertices[ring[(i + 1) % n]];
		normal.x += (p.y - q.y) * (p.z + q.z);
		normal.y += (p.z - q.z) * (p.x + q.x);
		normal.z += (p.x - q.x) * (p.y + q.y);
		perimeter2 += DistanceSq(p, q);
	}
	// |normal| is twice the area; an area below 1e-9 of the squared
	// perimeter is a line (a real sliver cap, e.g. a thin wing tip, is
	// orders of magnitude above this).
	if (LengthSq(normal) <= 4e-18 * perimeter2 * perimeter2) return false;
	int drop = 0;
	if (std::fabs(normal.y) > std::fabs(normal[drop])) drop = 1;
	if (std::fabs(normal.z) > std::fabs(normal[drop])) drop = 2;
	int ax = (drop + 1) % 3, ay = (drop + 2) % 3;

	std::vector<double> px(n), py(n);
	for (int i = 0; i < n; ++i) {
		const Vec3& p = mesh.vertices[ring[i]];
		px[i] = p[ax];
		py[i] = p[ay];
	}
	double area2 = 0.0;
	for (int i = 0; i < n; ++i) {
		int j = (i + 1) % n;
		area2 += Cross2(px[i], py[i], px[j], py[j]);
	}
	if (std::fabs(area2) <= 2e-9 * perimeter2) return false;
	// Work on a counter-clockwise copy; remember whether we reversed so the
	// emitted triangles keep the caller's winding.
	std::vector<int> order(n);
	for (int i = 0; i < n; ++i) order[i] = i;
	bool reversed = area2 < 0.0;
	if (reversed) std::reverse(order.begin(), order.end());

	auto emit = [&](int i, int j, int k) {
		if (reversed) mesh.triangles.push_back({ring[k], ring[j], ring[i]});
		else mesh.triangles.push_back({ring[i], ring[j], ring[k]});
	};

	std::vector<int> poly = order; // indices into px/py
	const double eps = 1e-12 * std::fabs(area2);
	int guard = 0;
	while (poly.size() > 3 && guard < 4 * n * n) {
		++guard;
		int m = static_cast<int>(poly.size());
		bool clipped = false;
		for (int i = 0; i < m; ++i) {
			int ip = poly[(i + m - 1) % m], ic = poly[i], in = poly[(i + 1) % m];
			double cr = Cross2(px[ic] - px[ip], py[ic] - py[ip], px[in] - px[ic], py[in] - py[ic]);
			if (cr <= eps) continue; // reflex or flat corner
			bool empty = true;
			for (int k = 0; k < m && empty; ++k) {
				int v = poly[k];
				if (v == ip || v == ic || v == in) continue;
				if (PointInTriangle2(px[v], py[v], px[ip], py[ip], px[ic], py[ic], px[in], py[in])) empty = false;
			}
			if (!empty) continue;
			emit(ip, ic, in);
			poly.erase(poly.begin() + i);
			clipped = true;
			break;
		}
		if (!clipped) {
			// Numerically stuck (e.g. a slightly self-touching outline): clip
			// the most convex corner regardless and carry on.
			int best = -1;
			double best_cr = -std::numeric_limits<double>::infinity();
			for (int i = 0; i < m; ++i) {
				int ip = poly[(i + m - 1) % m], ic = poly[i], in = poly[(i + 1) % m];
				double cr = Cross2(px[ic] - px[ip], py[ic] - py[ip], px[in] - px[ic], py[in] - py[ic]);
				if (cr > best_cr) { best_cr = cr; best = i; }
			}
			int ip = poly[(best + m - 1) % m], ic = poly[best], in = poly[(best + 1) % m];
			emit(ip, ic, in);
			poly.erase(poly.begin() + best);
		}
	}
	if (poly.size() == 3) emit(poly[0], poly[1], poly[2]);
	return true;
}

// ---------------------------------------------------------------------------
// Chordal refinement
// ---------------------------------------------------------------------------

int RefineChordal(std::vector<double>& params, double tol, int max_points,
	const std::function<std::vector<Mat>(const std::vector<double>&)>& evaluate, bool* capped) {
	if (capped) *capped = false;
	params = SortedUnique(params);
	if (params.size() < 2) return 0;
	static const double kProbes[3] = {0.25, 0.5, 0.75};
	int rounds = 0;
	for (;;) {
		int n = static_cast<int>(params.size());
		if (n >= max_points) { if (capped) *capped = true; break; }
		std::vector<double> probes;
		probes.reserve(3 * (n - 1));
		for (int i = 0; i + 1 < n; ++i) {
			for (double f : kProbes) probes.push_back(params[i] + f * (params[i + 1] - params[i]));
		}
		std::vector<Mat> at_params = evaluate(params);
		std::vector<Mat> at_probes = evaluate(probes);
		std::vector<std::pair<double, double>> bad; // (error, new param)
		for (int i = 0; i + 1 < n; ++i) {
			double worst = 0.0;
			for (size_t c = 0; c < at_params.size(); ++c) {
				Vec3 a = RowVec(at_params[c], i), b = RowVec(at_params[c], i + 1);
				for (int k = 0; k < 3; ++k) {
					Vec3 p = RowVec(at_probes[c], 3 * i + k);
					worst = std::max(worst, PointSegmentDistance(p, a, b));
				}
			}
			if (worst > tol) bad.push_back({worst, 0.5 * (params[i] + params[i + 1])});
		}
		++rounds;
		if (bad.empty()) break;
		std::sort(bad.begin(), bad.end(), [](const auto& x, const auto& y) { return x.first > y.first; });
		int room = max_points - n;
		int n_add = std::min(room, static_cast<int>(bad.size()));
		if (n_add < static_cast<int>(bad.size()) && capped) *capped = true;
		for (int i = 0; i < n_add; ++i) params.push_back(bad[i].second);
		params = SortedUnique(params);
		if (n_add <= 0) break;
	}
	return rounds;
}

// ---------------------------------------------------------------------------
// The loft
// ---------------------------------------------------------------------------

TriMesh TriangulateLoft(const LoftSource& src, const LoftOptions& opt, LoftStats* stats, LoftEndRings* ends) {
	if (!src.section) throw std::invalid_argument("TriangulateLoft: no section function");
	if (!(src.u_max > src.u_min)) throw std::invalid_argument("TriangulateLoft: empty station range");
	const double tol = std::max(opt.tolerance, 1e-12);

	// ---- initial parameter sets ----
	std::vector<double> us = src.u_seeds;
	us.push_back(src.u_min);
	us.push_back(src.u_max);
	us = SortedUnique(us);
	us.erase(std::remove_if(us.begin(), us.end(), [&](double u) { return u < src.u_min - 1e-12 || u > src.u_max + 1e-12; }), us.end());
	if (src.periodic_u) us.pop_back(); // u_max coincides with u_min
	EnsureMinimum(us, std::max(opt.min_u, src.periodic_u ? 3 : 2), src.periodic_u, src.u_max - src.u_min);

	std::vector<double> vs = src.v_seeds;
	vs.push_back(0.0);
	vs.push_back(1.0);
	vs = SortedUnique(vs);
	vs.erase(std::remove_if(vs.begin(), vs.end(), [](double v) { return v < -1e-12 || v > 1.0 + 1e-12; }), vs.end());
	if (src.closed_curve) vs.pop_back(); // v = 1 coincides with v = 0
	EnsureMinimum(vs, std::max(opt.min_v, 3), src.closed_curve, 1.0);

	// Evaluates the section at u for the v list, with the closing interval
	// of a closed curve / periodic chain appended as an explicit extra point
	// so refinement can measure it too.
	auto section_at = [&](double u, const std::vector<double>& v) -> Mat {
		double uu = u;
		if (src.periodic_u) {
			double period = src.u_max - src.u_min;
			uu = src.u_min + std::fmod(u - src.u_min, period);
			if (uu < src.u_min) uu += period;
		}
		std::vector<double> vv = v;
		for (double& x : vv) {
			if (src.closed_curve) {
				x = std::fmod(x, 1.0);
				if (x < 0.0) x += 1.0;
			} else {
				x = std::clamp(x, 0.0, 1.0);
			}
		}
		Mat m = src.section(uu, vv);
		if (m.cols() != 3 || m.rows() != static_cast<int>(vv.size())) throw std::invalid_argument("TriangulateLoft: section() must return (n, 3) points, one per v");
		return m;
	};

	// Thin shells (LoftSource::mirror_u): the station across the wall from u.
	const bool has_mirror_u = !std::isnan(src.mirror_u);
	const double u_period = src.u_max - src.u_min;
	auto mirror_u = [&](double u) {
		double m = src.mirror_u - u;
		if (src.periodic_u) {
			m = src.u_min + std::fmod(m - src.u_min, u_period);
			if (m < src.u_min) m += u_period;
			if (m >= src.u_min + u_period - 1e-12) m -= u_period;
		}
		return m;
	};

	// Along-curve refinement of a parameter set against the curves at the
	// given stations; v is the parameter. A closed curve's wrap-around
	// interval is represented by appending v_0 + 1 to the list being refined
	// and removing it again afterwards.
	auto refine_v_at = [&](std::vector<double> start, const std::vector<double>& stations, bool* capped, int* rounds_out) {
		std::vector<double> work = start;
		if (src.closed_curve) work.push_back(start.front() + 1.0);
		int rounds = RefineChordal(work, tol, opt.max_v + (src.closed_curve ? 1 : 0),
			[&](const std::vector<double>& v) {
				std::vector<Mat> out;
				out.reserve(stations.size());
				for (double u : stations) out.push_back(section_at(u, v));
				return out;
			}, capped);
		if (rounds_out) *rounds_out = rounds;
		if (src.closed_curve) {
			// Map everything back into [0, 1) and drop the duplicate closure.
			std::vector<double> back;
			for (double v : work) {
				double x = v >= 1.0 ? v - 1.0 : v;
				if (x < 1.0 - 1e-12) back.push_back(x);
			}
			return SortedUnique(back);
		}
		return work;
	};
	const std::vector<double> vs0 = vs; // the seeds every ring starts from
	auto refine_v = [&](bool* capped) {
		int rounds = 0;
		vs = refine_v_at(vs, us, capped, &rounds);
		return rounds;
	};
	auto snap_station = [&](double u) {
		auto it = std::lower_bound(us.begin(), us.end(), u - 1e-9);
		if (it != us.end() && std::fabs(*it - u) <= 1e-9) return *it;
		return u;
	};
	auto mirror_stations = [&]() {
		if (!has_mirror_u) return;
		std::vector<double> all = us;
		for (double u : us) {
			double m = mirror_u(u);
			if (m >= src.u_min - 1e-12 && m <= src.u_max + 1e-12) all.push_back(std::clamp(m, src.u_min, src.u_max));
		}
		us = SortedUnique(all, 1e-9);
		if (src.periodic_u) us.erase(std::remove_if(us.begin(), us.end(), [&](double u) { return u >= src.u_min + u_period - 1e-12; }), us.end());
	};

	// Each station's own ring parameters: refined against that station's
	// curve alone, so a small ring (a fuselage tail, a blade tip) is not
	// sampled as densely as the largest one -- which is what made rings of
	// hundreds of points microns apart converge on an apex or a knife edge
	// as needle triangles. Thin shells: the two stations of a wall pair are
	// refined together (from the canonical one of the pair, so both get the
	// identical set), and a ring's set is closed under mirror_v, so the two
	// sheets' vertex rows face each other across the wall.
	std::unordered_map<double, std::vector<double>> ring_vs_cache;
	auto ring_vs_at = [&](double u) -> const std::vector<double>& {
		auto it = ring_vs_cache.find(u);
		if (it != ring_vs_cache.end()) return it->second;
		std::vector<double> stations{u};
		if (has_mirror_u) {
			double m = snap_station(mirror_u(u));
			if (std::fabs(m - u) > 1e-12) {
				double key = std::min(u, m);
				stations = {key, snap_station(mirror_u(key))};
			}
		}
		bool capped = false;
		std::vector<double> r = refine_v_at(vs0, stations, &capped, nullptr);
		if (src.mirror_v) {
			// Close the set under the mirror. The refined set is only roughly
			// symmetric, so a mirrored parameter usually lands a fraction of a
			// gap from an existing one; a new point there would be a near-twin
			// (a sliver triangle). The existing point is moved onto the mirror
			// position instead when it is within half its gap and not a seed,
			// so the two sides match exactly; a new point is added only where
			// the other side has nothing nearby.
			std::vector<double> work = r;
			std::vector<char> pinned(work.size(), 0); // seeds and already-moved points stay
			for (size_t k = 0; k < work.size(); ++k) {
				for (double s : vs0) if (std::fabs(work[k] - s) <= 1e-12) pinned[k] = 1;
			}
			std::vector<double> added;
			const size_t n0 = work.size();
			for (size_t k = 0; k < n0; ++k) {
				double m = src.mirror_v(u, r[k]);
				if (!std::isfinite(m)) continue;
				if (src.closed_curve) {
					m = std::fmod(m, 1.0);
					if (m < 0.0) m += 1.0;
					if (m >= 1.0 - 1e-12) m -= 1.0;
				} else {
					m = std::clamp(m, 0.0, 1.0);
				}
				// Nearest existing point (of the original set) and its local gap.
				auto it = std::lower_bound(r.begin(), r.end(), m);
				size_t j = static_cast<size_t>(it - r.begin());
				if (j == r.size() || (j > 0 && m - r[j - 1] < r[j] - m)) --j;
				double gap = 1.0;
				if (j > 0) gap = std::min(gap, r[j] - r[j - 1]);
				if (j + 1 < r.size()) gap = std::min(gap, r[j + 1] - r[j]);
				if (std::fabs(m - r[j]) <= 1e-9) continue;          // already matched
				if (std::fabs(m - r[j]) <= 0.5 * gap) {
					if (!pinned[j]) { work[j] = m; pinned[j] = 1; }     // snap onto the twin
					continue;
				}
				bool crowded = false;
				for (double a : added) if (std::fabs(a - m) <= 0.25 * gap) { crowded = true; break; }
				if (!crowded) added.push_back(m);
			}
			work.insert(work.end(), added.begin(), added.end());
			r = SortedUnique(work, 1e-9);
		}
		// Parameters the caller requires on an end ring (LoftSource::v_extra_*):
		// after the closure, and not mirrored -- they are the other side of a
		// chain break's shared stretch, which must match exactly, not a
		// thin-shell twin.
		const std::vector<double>* extra = nullptr;
		if (!src.periodic_u) {
			if (!src.v_extra_start.empty() && std::fabs(u - src.u_min) <= 1e-12) extra = &src.v_extra_start;
			else if (!src.v_extra_end.empty() && std::fabs(u - src.u_max) <= 1e-12) extra = &src.v_extra_end;
		}
		if (extra) {
			std::vector<double> work = r;
			for (double v : *extra) {
				if (!std::isfinite(v)) continue;
				if (src.closed_curve) {
					v = std::fmod(v, 1.0);
					if (v < 0.0) v += 1.0;
					if (v >= 1.0 - 1e-12) v = 0.0;
				} else {
					v = std::clamp(v, 0.0, 1.0);
				}
				work.push_back(v);
			}
			r = SortedUnique(work, 1e-9);
		}
		return ring_vs_cache.emplace(u, std::move(r)).first->second;
	};

	// Station refinement: every v is a curve along u; u is the parameter.
	auto refine_u = [&](bool* capped) {
		std::vector<double> work = us;
		if (src.periodic_u) work.push_back(us.front() + (src.u_max - src.u_min));
		int rounds = RefineChordal(work, tol, opt.max_u + (src.periodic_u ? 1 : 0),
			[&](const std::vector<double>& u) {
				// One (n_u, 3) matrix per v.
				int nv = static_cast<int>(vs.size());
				std::vector<Mat> out(nv, Mat(static_cast<int>(u.size()), 3));
				for (size_t i = 0; i < u.size(); ++i) {
					Mat s = section_at(u[i], vs);
					for (int j = 0; j < nv; ++j) {
						out[j](static_cast<int>(i), 0) = s(j, 0);
						out[j](static_cast<int>(i), 1) = s(j, 1);
						out[j](static_cast<int>(i), 2) = s(j, 2);
					}
				}
				return out;
			}, capped);
		if (src.periodic_u) {
			double period = src.u_max - src.u_min;
			std::vector<double> back;
			for (double u : work) {
				double x = u >= src.u_min + period ? u - period : u;
				if (x < src.u_min + period - 1e-12) back.push_back(x);
			}
			us = SortedUnique(back);
		} else {
			us = work;
		}
		return rounds;
	};

	LoftStats st;
	for (int pass = 0; pass < 4; ++pass) {
		size_t nv0 = vs.size(), nu0 = us.size();
		bool vc = false, uc = false;
		st.v_rounds += refine_v(&vc);
		st.u_rounds += refine_u(&uc);
		st.v_capped = st.v_capped || vc;
		st.u_capped = st.u_capped || uc;
		if (vs.size() == nv0 && us.size() == nu0) break;
	}

	// ---- aspect refinement of the stations ----
	// A strip much longer than the point spacing of its rings (each sampled
	// on its own) is split in two -- repeatedly, a geometric progression of
	// stations into an apex or a knife edge -- never below a few tolerances
	// of length. An apex ring has no spacing of its own; the other ring's
	// counts.
	auto aspect_refine = [&]() {
		if (opt.max_aspect <= 0.0) return;
		const double period = u_period;
		for (int round = 0; round < 64; ++round) {
			const int nu = static_cast<int>(us.size());
			if (nu >= opt.max_u) { st.u_capped = true; break; }
			std::vector<Mat> at(nu);
			std::vector<double> spacing(nu, 0.0);
			for (int i = 0; i < nu; ++i) {
				at[i] = section_at(us[i], vs);
				Mat own = section_at(us[i], ring_vs_at(us[i]));
				int n = own.rows();
				double sum = 0.0;
				int cnt = 0;
				for (int j = 0; j + 1 < n; ++j) { sum += Distance(RowVec(own, j), RowVec(own, j + 1)); ++cnt; }
				if (src.closed_curve && n > 1) { sum += Distance(RowVec(own, n - 1), RowVec(own, 0)); ++cnt; }
				spacing[i] = cnt ? sum / cnt : 0.0;
			}
			std::vector<double> add;
			const int n_pairs = src.periodic_u ? nu : nu - 1;
			for (int i = 0; i < n_pairs; ++i) {
				int k = (i + 1) % nu;
				double s;
				if (spacing[i] <= opt.apex_tolerance) s = spacing[k];
				else if (spacing[k] <= opt.apex_tolerance) s = spacing[i];
				else s = std::min(spacing[i], spacing[k]);
				if (s <= opt.apex_tolerance) continue;
				double d = 0.0;
				for (int j = 0; j < at[i].rows(); ++j) d = std::max(d, Distance(RowVec(at[i], j), RowVec(at[k], j)));
				if (d > opt.max_aspect * s && d > 4.0 * tol) {
					double u1 = k == 0 ? us[0] + period : us[k];
					double mid = 0.5 * (us[i] + u1);
					if (mid >= src.u_min + period) mid -= period;
					add.push_back(mid);
				}
			}
			if (add.empty()) break;
			int room = opt.max_u - nu;
			if (room < static_cast<int>(add.size())) { add.resize(std::max(room, 0)); st.u_capped = true; }
			us.insert(us.end(), add.begin(), add.end());
			us = SortedUnique(us);
			++st.u_rounds;
			if (add.empty()) break;
		}
	};
	aspect_refine();
	if (has_mirror_u) {
		// Both sides of the wall get each other's stations; the aspect rule
		// then sees the finer set, and its additions are mirrored in turn.
		// The ring sets are recomputed from the symmetric station set.
		mirror_stations();
		ring_vs_cache.clear();
		aspect_refine();
		mirror_stations();
		ring_vs_cache.clear();
	}
	st.n_stations = static_cast<int>(us.size());

	// ---- the point grid, with shared vertices ----
	// Every ring is sampled at its own parameters (ring_vs_at); the strips
	// between rings of different counts are zig-zagged by parameter.
	TriMesh mesh;
	const int nu = static_cast<int>(us.size());
	std::vector<std::vector<int>> rings(nu);
	std::vector<std::vector<double>> ring_v(nu);
	std::vector<bool> is_apex(nu, false);
	const double apex2 = opt.apex_tolerance * opt.apex_tolerance;
	// Exact positions of the previous ring's (and the first ring's) nodes,
	// so a point that coincides with one is shared rather than duplicated.
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
	std::unordered_map<PosKey, int, PosHash> prev_pos, first_pos;
	int max_ring = 0;
	for (int i = 0; i < nu; ++i) {
		ring_v[i] = ring_vs_at(us[i]);
		const int nv = static_cast<int>(ring_v[i].size());
		max_ring = std::max(max_ring, nv);
		Mat s = section_at(us[i], ring_v[i]);
		Vec3 first = RowVec(s, 0);
		bool apex = true;
		for (int j = 1; j < nv && apex; ++j) apex = DistanceSq(RowVec(s, j), first) <= apex2;
		is_apex[i] = apex;
		std::vector<int>& ring = rings[i];
		ring.resize(nv);
		std::unordered_map<PosKey, int, PosHash> this_pos;
		if (apex) {
			int idx = static_cast<int>(mesh.vertices.size());
			mesh.vertices.push_back(first);
			std::fill(ring.begin(), ring.end(), idx);
			this_pos[{first.x, first.y, first.z}] = idx;
			prev_pos = std::move(this_pos);
			if (i == 0) first_pos = prev_pos;
			continue;
		}
		// The previous station (and, on a periodic chain, the first), so a
		// point that coincides with its counterpart there is shared rather
		// than duplicated -- two stations sitting on the same curve (a rim of
		// zero wall thickness) then yield no sliver triangles at all.
		const bool use_first = src.periodic_u && i == nu - 1;
		for (int j = 0; j < nv; ++j) {
			Vec3 p = RowVec(s, j);
			// Alias a point that coincides with ANY earlier point of this ring
			// rather than duplicate it: a collapsed edge (a closed trailing
			// edge) repeats its predecessor, and a section squeezed flat (a
			// fuselage end with zero width but some height -- a knife edge)
			// retraces itself, so each point on the way back coincides with
			// one on the way out. Sharing those makes the strip on both sides
			// of the flat section meet along common edges, which keeps the
			// mesh a closed manifold; left as separate vertices the two sides
			// would be joined by nothing and the mesh would be open along the
			// edge (the distance field's sign is then wrong near it).
			{
				int alias = -1;
				for (int k = j - 1; k >= 0 && alias < 0; --k) {
					if (DistanceSq(p, mesh.vertices[ring[k]]) <= apex2) alias = ring[k];
				}
				if (alias >= 0) {
					ring[j] = alias;
					continue;
				}
			}
			PosKey key{p.x, p.y, p.z};
			auto pit = prev_pos.find(key);
			if (pit != prev_pos.end()) { ring[j] = pit->second; this_pos[key] = ring[j]; continue; }
			if (use_first) {
				auto fit = first_pos.find(key);
				if (fit != first_pos.end()) { ring[j] = fit->second; this_pos[key] = ring[j]; continue; }
			}
			ring[j] = static_cast<int>(mesh.vertices.size());
			mesh.vertices.push_back(p);
			this_pos[key] = ring[j];
		}
		// An open curve whose two ends coincide is closed after all.
		if (DistanceSq(mesh.vertices[ring[nv - 1]], mesh.vertices[ring[0]]) <= apex2 && ring[nv - 1] != ring[0]) {
			if (ring[nv - 1] == static_cast<int>(mesh.vertices.size()) - 1) mesh.vertices.pop_back();
			ring[nv - 1] = ring[0];
		}
		prev_pos = std::move(this_pos);
		if (i == 0) first_pos = prev_pos;
	}
	st.n_ring_points = max_ring;
	if (stats) *stats = st;

	// ---- strips ----
	for (int i = 0; i + 1 < nu; ++i) AppendZigZagStrip(mesh, rings[i], ring_v[i], rings[i + 1], ring_v[i + 1], src.closed_curve);
	if (src.periodic_u) {
		AppendZigZagStrip(mesh, rings[nu - 1], ring_v[nu - 1], rings[0], ring_v[0], src.closed_curve);
	} else {
		// ---- caps ----
		// The start cap runs against the first ring's direction and the end
		// cap with the last ring's, so every edge is shared by two triangles
		// traversed oppositely (a consistent manifold).
		if (src.cap_start && !is_apex[0]) {
			std::vector<int> rev(rings[0].rbegin(), rings[0].rend());
			AppendPolygonCap(mesh, rev);
		}
		if (src.cap_end && !is_apex[nu - 1]) AppendPolygonCap(mesh, rings[nu - 1]);
	}
	if (ends) {
		ends->start = rings[0];
		ends->v_start = ring_v[0];
		ends->end = rings[nu - 1];
		ends->v_end = ring_v[nu - 1];
	}
	mesh.RemoveDegenerateTriangles();
	if (opt.orient_outward) mesh.FlipIfInsideOut();
	return mesh;
}

// ---------------------------------------------------------------------------
// Chains
// ---------------------------------------------------------------------------

TriMesh TriangulateLoftChain(std::vector<LoftSource> segments, const std::vector<LoftBreak>& breaks, const LoftOptions& opt, std::vector<LoftStats>* stats) {
	const int n = static_cast<int>(segments.size());
	if (n == 0) throw std::invalid_argument("TriangulateLoftChain: no segments");
	if (static_cast<int>(breaks.size()) != n - 1) throw std::invalid_argument("TriangulateLoftChain: breaks.size() must be segments.size() - 1");
	if (stats) stats->assign(n, LoftStats{});
	if (n == 1) return TriangulateLoft(segments[0], opt, stats ? &(*stats)[0] : nullptr);
	for (int i = 0; i < n; ++i) {
		if (segments[i].periodic_u) throw std::invalid_argument("TriangulateLoftChain: a segment may not be periodic in u");
		if (i > 0) segments[i].cap_start = false;
		if (i + 1 < n) segments[i].cap_end = false;
	}
	for (const LoftBreak& br : breaks) {
		if (!br.a_to_b || !br.b_to_a) throw std::invalid_argument("TriangulateLoftChain: a break lacks its parameter maps");
	}
	LoftOptions seg_opt = opt;
	seg_opt.orient_outward = false; // one winding for all, oriented at the end

	auto in_range = [](double v, double lo, double hi) { return v >= lo - 1e-9 && v <= hi + 1e-9; };
	// Index of the ring point at parameter v, or -1.
	auto find_v = [](const std::vector<double>& vs, double v) {
		int best = -1;
		double best_d = 1e-8;
		for (size_t i = 0; i < vs.size(); ++i) {
			double d = std::fabs(vs[i] - v);
			if (d <= best_d) { best_d = d; best = static_cast<int>(i); }
		}
		return best;
	};

	// Pass 1: each segment's natural end rings, to tell each break's two
	// rings what the other has on the shared stretch.
	std::vector<LoftEndRings> natural(n);
	for (int i = 0; i < n; ++i) TriangulateLoft(segments[i], seg_opt, nullptr, &natural[i]);
	for (int k = 0; k + 1 < n; ++k) {
		const LoftBreak& br = breaks[k];
		for (double v : natural[k].v_end) if (in_range(v, br.va0, br.va1)) segments[k + 1].v_extra_start.push_back(br.a_to_b(v));
		for (double v : natural[k + 1].v_start) if (in_range(v, br.vb0, br.vb1)) segments[k].v_extra_end.push_back(br.b_to_a(v));
	}

	// Pass 2: the meshes, with the end rings complete.
	std::vector<TriMesh> meshes(n);
	std::vector<LoftEndRings> ends(n);
	for (int i = 0; i < n; ++i) meshes[i] = TriangulateLoft(segments[i], seg_opt, stats ? &(*stats)[i] : nullptr, &ends[i]);

	// Assemble: append each segment, its first ring's shared points taking
	// the previous segment's last-ring vertices, then cap the pocket.
	TriMesh out = meshes[0];
	std::vector<int> prev_ring = ends[0].end;
	std::vector<double> prev_v = ends[0].v_end;
	for (int k = 0; k + 1 < n; ++k) {
		const LoftBreak& br = breaks[k];
		const TriMesh& m = meshes[k + 1];
		const std::vector<int>& bring = ends[k + 1].start;
		const std::vector<double>& bv = ends[k + 1].v_start;
		const int base = static_cast<int>(out.vertices.size());
		std::vector<int> remap(m.vertices.size());
		for (size_t j = 0; j < remap.size(); ++j) remap[j] = base + static_cast<int>(j);
		for (size_t j = 0; j < bring.size(); ++j) {
			if (!in_range(bv[j], br.vb0, br.vb1)) continue;
			int i = find_v(prev_v, br.b_to_a(bv[j]));
			if (i < 0) throw std::runtime_error("TriangulateLoftChain: the previous ring has no counterpart of a shared point at v = " + std::to_string(bv[j]));
			remap[bring[j]] = prev_ring[i];
		}
		out.vertices.insert(out.vertices.end(), m.vertices.begin(), m.vertices.end());
		for (const auto& t : m.triangles) out.triangles.push_back({remap[t[0]], remap[t[1]], remap[t[2]]});

		// The pocket: the end cap of the previous segment (its last ring in
		// ring order) with the start cap of the next (its first ring
		// reversed), the shared stretch cancelling between them -- so the
		// previous ring's other stretch from va1 round to va0, then the next
		// ring's other stretch backwards from vb0 round to vb1.
		const int na = static_cast<int>(prev_ring.size()), nb = static_cast<int>(bring.size());
		int ia0 = find_v(prev_v, br.va0), ia1 = find_v(prev_v, br.va1);
		int ib0 = find_v(bv, br.vb0), ib1 = find_v(bv, br.vb1);
		if (ia0 < 0 || ia1 < 0 || ib0 < 0 || ib1 < 0) throw std::runtime_error("TriangulateLoftChain: a break's shared stretch does not end on a ring point");
		std::vector<int> poly;
		for (int i = ia1;; i = (i + 1) % na) {
			poly.push_back(prev_ring[i]);
			if (i == ia0) break;
		}
		for (int j = (ib0 - 1 + nb) % nb; j != ib1; j = (j - 1 + nb) % nb) poly.push_back(remap[bring[j]]);
		AppendPolygonCap(out, poly);

		prev_ring = ends[k + 1].end;
		for (int& idx : prev_ring) idx = remap[idx];
		prev_v = ends[k + 1].v_end;
	}

	// Drop the vertices the welding orphaned.
	std::vector<int> used(out.vertices.size(), -1);
	for (const auto& t : out.triangles) for (int idx : t) used[idx] = 1;
	std::vector<Vec3> kept;
	kept.reserve(out.vertices.size());
	for (size_t i = 0; i < used.size(); ++i) {
		if (used[i] < 0) continue;
		used[i] = static_cast<int>(kept.size());
		kept.push_back(out.vertices[i]);
	}
	for (auto& t : out.triangles) t = {used[t[0]], used[t[1]], used[t[2]]};
	out.vertices = std::move(kept);
	out.RemoveDegenerateTriangles();
	if (opt.orient_outward) out.FlipIfInsideOut();
	return out;
}

// ---------------------------------------------------------------------------
// Primitives
// ---------------------------------------------------------------------------

TriMesh BoxMesh(double lx, double ly, double lz) {
	TriMesh m;
	double hx = 0.5 * lx, hy = 0.5 * ly, hz = 0.5 * lz;
	for (int i = 0; i < 8; ++i) {
		m.vertices.push_back({(i & 1) ? hx : -hx, (i & 2) ? hy : -hy, (i & 4) ? hz : -hz});
	}
	// Faces as quads (outward, counter-clockwise from outside).
	const int quads[6][4] = {
		{0, 2, 3, 1}, // -z
		{4, 5, 7, 6}, // +z
		{0, 1, 5, 4}, // -y
		{2, 6, 7, 3}, // +y
		{0, 4, 6, 2}, // -x
		{1, 3, 7, 5}, // +x
	};
	for (const auto& q : quads) {
		m.triangles.push_back({q[0], q[1], q[2]});
		m.triangles.push_back({q[0], q[2], q[3]});
	}
	m.FlipIfInsideOut();
	return m;
}

LoftSource EllipsoidLoft(double a, double b, double c) {
	LoftSource src;
	src.section = [a, b, c](double u, const std::vector<double>& v) {
		double theta = -0.5 * kPi + kPi * u;
		double ct = std::cos(theta), stt = std::sin(theta);
		Mat out(static_cast<int>(v.size()), 3);
		for (size_t j = 0; j < v.size(); ++j) {
			double phi = 2.0 * kPi * v[j];
			out(static_cast<int>(j), 0) = a * ct * std::cos(phi);
			out(static_cast<int>(j), 1) = b * ct * std::sin(phi);
			out(static_cast<int>(j), 2) = c * stt;
		}
		return out;
	};
	src.closed_curve = true;
	src.u_seeds = {0.0, 0.5, 1.0};
	src.v_seeds = {0.0, 0.25, 0.5, 0.75};
	return src;
}

LoftSource CylinderLoft(double radius, double length) {
	LoftSource src;
	const double hl = 0.5 * length;
	src.u_min = 0.0;
	src.u_max = 3.0;
	src.section = [radius, length, hl](double u, const std::vector<double>& v) {
		double x, r;
		if (u <= 1.0) { x = -hl; r = radius * u; }
		else if (u <= 2.0) { x = -hl + (u - 1.0) * length; r = radius; }
		else { x = hl; r = radius * (3.0 - u); }
		Mat out(static_cast<int>(v.size()), 3);
		for (size_t j = 0; j < v.size(); ++j) {
			double phi = 2.0 * kPi * v[j];
			out(static_cast<int>(j), 0) = x;
			out(static_cast<int>(j), 1) = r * std::cos(phi);
			out(static_cast<int>(j), 2) = r * std::sin(phi);
		}
		return out;
	};
	src.closed_curve = true;
	src.u_seeds = {0.0, 1.0, 2.0, 3.0};
	src.v_seeds = {0.0, 0.25, 0.5, 0.75};
	return src;
}

LoftSource BoxLoft(double lx, double ly, double lz) {
	// A square ring in (y, z) scaled by the end-cap fraction, the ring's
	// four sides over v in quarters: (+y) -> (+z) -> (-y) -> (-z) edges.
	LoftSource src;
	const double hx = 0.5 * lx, hy = 0.5 * ly, hz = 0.5 * lz;
	src.u_min = 0.0;
	src.u_max = 3.0;
	src.section = [hx, hy, hz, lx](double u, const std::vector<double>& v) {
		double x, s;
		if (u <= 1.0) { x = -hx; s = u; }
		else if (u <= 2.0) { x = -hx + (u - 1.0) * lx; s = 1.0; }
		else { x = hx; s = 3.0 - u; }
		Mat out(static_cast<int>(v.size()), 3);
		for (size_t j = 0; j < v.size(); ++j) {
			double t = std::fmod(v[j], 1.0);
			if (t < 0.0) t += 1.0;
			double y, z;
			double f = 4.0 * t;
			if (f < 1.0) { y = hy; z = -hz + 2.0 * hz * f; }
			else if (f < 2.0) { y = hy - 2.0 * hy * (f - 1.0); z = hz; }
			else if (f < 3.0) { y = -hy; z = hz - 2.0 * hz * (f - 2.0); }
			else { y = -hy + 2.0 * hy * (f - 3.0); z = -hz; }
			out(static_cast<int>(j), 0) = x;
			out(static_cast<int>(j), 1) = s * y;
			out(static_cast<int>(j), 2) = s * z;
		}
		return out;
	};
	src.closed_curve = true;
	src.u_seeds = {0.0, 1.0, 2.0, 3.0};
	src.v_seeds = {0.0, 0.25, 0.5, 0.75};
	return src;
}

TriMesh EllipsoidMesh(double a, double b, double c, double tolerance) {
	LoftSource src = EllipsoidLoft(a, b, c);
	LoftOptions opt;
	opt.tolerance = tolerance;
	opt.min_v = 12;
	opt.min_u = 5;
	return TriangulateLoft(src, opt);
}

TriMesh CylinderMesh(double radius, double length, double tolerance) {
	LoftSource src = CylinderLoft(radius, length);
	LoftOptions opt;
	opt.tolerance = tolerance;
	opt.min_v = 12;
	opt.min_u = 4;
	return TriangulateLoft(src, opt);
}

} // namespace aerotelic
