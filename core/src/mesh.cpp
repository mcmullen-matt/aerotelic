#include "aerotelic/mesh.hpp"

#include <algorithm>
#include <cmath>
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
		// Split along the shorter diagonal: a0-b1 or a1-b0.
		if (DistanceSq(pa0, pb1) <= DistanceSq(pa1, pb0)) {
			mesh.triangles.push_back({a0, a1, b1});
			mesh.triangles.push_back({a0, b1, b0});
		} else {
			mesh.triangles.push_back({a0, a1, b0});
			mesh.triangles.push_back({a1, b1, b0});
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
	Vec3 normal{0, 0, 0};
	for (int i = 0; i < n; ++i) {
		const Vec3& p = mesh.vertices[ring[i]];
		const Vec3& q = mesh.vertices[ring[(i + 1) % n]];
		normal.x += (p.y - q.y) * (p.z + q.z);
		normal.y += (p.z - q.z) * (p.x + q.x);
		normal.z += (p.x - q.x) * (p.y + q.y);
	}
	if (LengthSq(normal) <= 0.0) return false;
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
	if (std::fabs(area2) <= 0.0) return false;
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

TriMesh TriangulateLoft(const LoftSource& src, const LoftOptions& opt, LoftStats* stats) {
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

	// Along-curve refinement: every station is a curve; v is the parameter.
	// A closed curve's wrap-around interval is represented by appending
	// v_0 + 1 to the list being refined and removing it again afterwards.
	auto refine_v = [&](bool* capped) {
		std::vector<double> work = vs;
		if (src.closed_curve) work.push_back(vs.front() + 1.0);
		int rounds = RefineChordal(work, tol, opt.max_v + (src.closed_curve ? 1 : 0),
			[&](const std::vector<double>& v) {
				std::vector<Mat> out;
				out.reserve(us.size());
				for (double u : us) out.push_back(section_at(u, v));
				return out;
			}, capped);
		if (src.closed_curve) {
			// Map everything back into [0, 1) and drop the duplicate closure.
			std::vector<double> back;
			for (double v : work) {
				double x = v >= 1.0 ? v - 1.0 : v;
				if (x < 1.0 - 1e-12) back.push_back(x);
			}
			vs = SortedUnique(back);
		} else {
			vs = work;
		}
		return rounds;
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
	st.n_stations = static_cast<int>(us.size());
	st.n_ring_points = static_cast<int>(vs.size());
	if (stats) *stats = st;

	// ---- the point grid, with shared vertices ----
	TriMesh mesh;
	const int nu = static_cast<int>(us.size());
	const int nv = static_cast<int>(vs.size());
	std::vector<std::vector<int>> rings(nu);
	std::vector<bool> is_apex(nu, false);
	const double apex2 = opt.apex_tolerance * opt.apex_tolerance;
	for (int i = 0; i < nu; ++i) {
		Mat s = section_at(us[i], vs);
		Vec3 first = RowVec(s, 0);
		bool apex = true;
		for (int j = 1; j < nv && apex; ++j) apex = DistanceSq(RowVec(s, j), first) <= apex2;
		is_apex[i] = apex;
		std::vector<int>& ring = rings[i];
		ring.resize(nv);
		if (apex) {
			int idx = static_cast<int>(mesh.vertices.size());
			mesh.vertices.push_back(first);
			std::fill(ring.begin(), ring.end(), idx);
			continue;
		}
		// The previous station (and, on a periodic chain, the first), so a
		// point that coincides with its counterpart there is shared rather
		// than duplicated -- two stations sitting on the same curve (a rim of
		// zero wall thickness) then yield no sliver triangles at all.
		const std::vector<int>* prev = i > 0 ? &rings[i - 1] : nullptr;
		const std::vector<int>* first_ring = (src.periodic_u && i == nu - 1) ? &rings[0] : nullptr;
		for (int j = 0; j < nv; ++j) {
			Vec3 p = RowVec(s, j);
			// Alias a point that coincides with its predecessor (a collapsed
			// edge, e.g. a closed trailing edge) rather than duplicate it.
			if (j > 0 && DistanceSq(p, mesh.vertices[ring[j - 1]]) <= apex2) {
				ring[j] = ring[j - 1];
				continue;
			}
			if (prev && DistanceSq(p, mesh.vertices[(*prev)[j]]) <= apex2) {
				ring[j] = (*prev)[j];
				continue;
			}
			if (first_ring && DistanceSq(p, mesh.vertices[(*first_ring)[j]]) <= apex2) {
				ring[j] = (*first_ring)[j];
				continue;
			}
			ring[j] = static_cast<int>(mesh.vertices.size());
			mesh.vertices.push_back(p);
		}
		// An open curve whose two ends coincide is closed after all.
		if (DistanceSq(mesh.vertices[ring[nv - 1]], mesh.vertices[ring[0]]) <= apex2 && ring[nv - 1] != ring[0]) {
			if (ring[nv - 1] == static_cast<int>(mesh.vertices.size()) - 1) mesh.vertices.pop_back();
			ring[nv - 1] = ring[0];
		}
	}

	// ---- strips ----
	for (int i = 0; i + 1 < nu; ++i) AppendRuledStrip(mesh, rings[i], rings[i + 1], true);
	if (src.periodic_u) {
		AppendRuledStrip(mesh, rings[nu - 1], rings[0], true);
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
	mesh.RemoveDegenerateTriangles();
	mesh.FlipIfInsideOut();
	return mesh;
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

TriMesh EllipsoidMesh(double a, double b, double c, double tolerance) {
	LoftSource src;
	src.section = [&](double u, const std::vector<double>& v) {
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
	LoftOptions opt;
	opt.tolerance = tolerance;
	opt.min_v = 12;
	opt.min_u = 5;
	return TriangulateLoft(src, opt);
}

TriMesh CylinderMesh(double radius, double length, double tolerance) {
	LoftSource src;
	const double hl = 0.5 * length;
	src.u_min = 0.0;
	src.u_max = 3.0;
	src.section = [&](double u, const std::vector<double>& v) {
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
	LoftOptions opt;
	opt.tolerance = tolerance;
	opt.min_v = 12;
	opt.min_u = 4;
	return TriangulateLoft(src, opt);
}

} // namespace aerotelic
