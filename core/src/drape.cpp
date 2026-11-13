#include "aerotelic/drape.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <unordered_map>

#include "aerotelic/sdf.hpp"

namespace aerotelic {

namespace {

constexpr double kPi = 3.14159265358979323846;

// In-plane orthonormal basis of a triangle.
void Basis(const Vec3& n, const Vec3& a, const Vec3& b, Vec3& e1, Vec3& e2) {
	e1 = Normalized(b - a);
	e2 = Cross(n, e1);
}

// Walks a geodesic approximately: from p on the surface along the unit
// tangent direction d for a length L, in small steps each projected back
// onto the surface (the closest point) with the direction carried into
// the new tangent plane -- the step-and-project integrator, whose error
// is of the order of the step squared times the curvature, nothing at the
// pitch used here. (An exact edge-crossing walk was tried first and
// tripped over rays through vertices.) On return p, t, d are the end
// point, its triangle and the direction there. False when the walk leaves
// the surface at a free edge: the projection then stops advancing.
struct Walker {
	const TriMesh& m;
	const MeshSdf& sdf;
	std::vector<Vec3> normals;

	Walker(const TriMesh& mesh, const MeshSdf& s) : m(mesh), sdf(s) {
		normals.resize(m.triangles.size());
		for (size_t t = 0; t < m.triangles.size(); ++t) normals[t] = m.TriangleNormal(static_cast<int>(t));
	}

	bool Walk(Vec3& p, int& t, Vec3& d, double L) const {
		const int steps = 8;
		const double h = L / steps;
		for (int k = 0; k < steps; ++k) {
			const Vec3& n = normals[t];
			d = Normalized(d - n * Dot(d, n));
			if (LengthSq(d) == 0.0) return false;
			const Vec3 q = p + d * h;
			const MeshSdf::Closest c = sdf.ClosestPoint(q);
			// Off a free edge: the projection lands back on the boundary
			// and the step does not advance.
			if (c.distance > 0.5 * h || Distance(c.point, p) < 0.25 * h) return false;
			p = c.point;
			t = c.triangle;
		}
		const Vec3& n = normals[t];
		d = Normalized(d - n * Dot(d, n));
		return LengthSq(d) > 0.0;
	}
};

struct NetNode {
	Vec3 p;
	int tri = -1;
	Vec3 dir;   // a generator node's onward direction
	bool ok = false;
};

struct Hash3 {
	double cell = 1.0;
	std::unordered_multimap<uint64_t, int> map;
	static uint64_t Key(int64_t x, int64_t y, int64_t z) {
		return (static_cast<uint64_t>(x & 0x1FFFFF) << 42) | (static_cast<uint64_t>(y & 0x1FFFFF) << 21) | static_cast<uint64_t>(z & 0x1FFFFF);
	}
	void Cell(const Vec3& p, int64_t& x, int64_t& y, int64_t& z) const {
		x = static_cast<int64_t>(std::floor(p.x / cell));
		y = static_cast<int64_t>(std::floor(p.y / cell));
		z = static_cast<int64_t>(std::floor(p.z / cell));
	}
	void Add(const Vec3& p, int id) {
		int64_t x, y, z;
		Cell(p, x, y, z);
		map.emplace(Key(x, y, z), id);
	}
	template <typename F> void Near(const Vec3& p, int reach, F&& f) const {
		int64_t x, y, z;
		Cell(p, x, y, z);
		for (int64_t i = x - reach; i <= x + reach; ++i)
			for (int64_t j = y - reach; j <= y + reach; ++j)
				for (int64_t k = z - reach; k <= z + reach; ++k) {
					auto range = map.equal_range(Key(i, j, k));
					for (auto it = range.first; it != range.second; ++it) f(it->second);
				}
	}
};

} // namespace

DrapeResult DrapeNet(const TriMesh& surface, const DrapeOptions& options) {
	if (surface.IsEmpty()) throw std::invalid_argument("DrapeNet: an empty surface");
	DrapeResult out;
	const size_t nt = surface.triangles.size();
	out.warp.assign(nt, Vec3{});
	out.weft.assign(nt, Vec3{});
	out.shear.assign(nt, 0.0);
	out.covered.assign(nt, 0);
	const Box3 bounds = surface.Bounds();
	const double L = options.cell > 0.0 ? options.cell : 0.01 * bounds.Diagonal();
	out.cell = L;
	if (!(L > 0.0)) throw std::invalid_argument("DrapeNet: the surface has no extent");

	const MeshSdf sdf(surface);
	const Walker walker(surface, sdf);

	// The seed and its frame.
	const MeshSdf::Closest c0 = sdf.ClosestPoint(options.seed);
	const Vec3 n0 = walker.normals[c0.triangle];
	Vec3 w0 = options.warp - n0 * Dot(options.warp, n0);
	if (Length(w0) < 1e-9 * std::max(Length(options.warp), 1e-300)) {
		out.notes.push_back("the warp direction is normal to the surface at the seed; no net was laid");
		return out;
	}
	w0 = Normalized(w0);
	const Vec3 f0 = Normalized(Cross(n0, w0));

	// Nodes by (i, j).
	std::map<std::pair<int, int>, NetNode> nodes;
	Hash3 hash;
	hash.cell = L;
	std::vector<std::pair<int, int>> node_index;   // id -> (i, j), for the overlap check
	auto put = [&](int i, int j, NetNode nd) {
		nd.ok = true;
		nodes[{i, j}] = nd;
		hash.Add(nd.p, static_cast<int>(node_index.size()));
		node_index.push_back({i, j});
	};
	auto get = [&](int i, int j) -> const NetNode* {
		auto it = nodes.find({i, j});
		return it == nodes.end() || !it->second.ok ? nullptr : &it->second;
	};
	// A node of the net may not land within 0.6 L of a node that is not
	// its neighbour: the net has wrapped round the body onto itself.
	auto overlaps = [&](const Vec3& p, int i, int j) {
		bool hit = false;
		hash.Near(p, 1, [&](int id) {
			if (hit) return;
			const auto ij = node_index[id];
			if (std::abs(ij.first - i) + std::abs(ij.second - j) <= 2) return;
			if (Distance(nodes[ij].p, p) < 0.6 * L) hit = true;
		});
		return hit;
	};

	{
		NetNode seed;
		seed.p = c0.point;
		seed.tri = c0.triangle;
		put(0, 0, seed);
	}
	// The generators: geodesics along +-warp (j = 0) and +-weft (i = 0).
	auto grow_generator = [&](int di, int dj, const Vec3& start_dir, int n) {
		const int pi = (n - 1) * di, pj = (n - 1) * dj;
		const NetNode* prev = get(pi, pj);
		if (prev == nullptr) return false;
		Vec3 p = prev->p, d = n == 1 ? start_dir : prev->dir;
		int t = prev->tri;
		if (!walker.Walk(p, t, d, L)) return false;
		if (overlaps(p, n * di, n * dj)) return false;
		NetNode nd;
		nd.p = p; nd.tri = t; nd.dir = d;
		put(n * di, n * dj, nd);
		return true;
	};
	// An interior node: on the surface, L from A = (i - si, j) and from
	// B = (i, j - sj), starting from the parallelogram over O = (i - si, j - sj).
	auto place = [&](int i, int j) {
		const int si = i > 0 ? 1 : -1, sj = j > 0 ? 1 : -1;
		const NetNode* A = get(i - si, j);
		const NetNode* B = get(i, j - sj);
		const NetNode* O = get(i - si, j - sj);
		if (!A || !B || !O) return false;
		Vec3 R = A->p + B->p - O->p;
		MeshSdf::Closest c = sdf.ClosestPoint(R);
		if (c.distance > 0.75 * L) return false;   // off the surface: a free edge
		R = c.point;
		int tri = c.triangle;
		const double L2 = L * L;
		bool converged = false;
		for (int iter = 0; iter < 60; ++iter) {
			const Vec3 ra = R - A->p, rb = R - B->p;
			const double r1 = LengthSq(ra) - L2, r2 = LengthSq(rb) - L2;
			if (std::fabs(r1) < 1e-10 * L2 && std::fabs(r2) < 1e-10 * L2) { converged = true; break; }
			const Vec3& n = walker.normals[tri];
			const auto& tv = surface.triangles[tri];
			Vec3 e1, e2;
			Basis(n, surface.vertices[tv[0]], surface.vertices[tv[1]], e1, e2);
			const double j11 = 2.0 * Dot(ra, e1), j12 = 2.0 * Dot(ra, e2);
			const double j21 = 2.0 * Dot(rb, e1), j22 = 2.0 * Dot(rb, e2);
			const double det = j11 * j22 - j12 * j21;
			if (std::fabs(det) < 1e-12 * L2 * L2) return false;   // A, B and R in a line
			double da = -(r1 * j22 - r2 * j12) / det;
			double db = -(j11 * r2 - j21 * r1) / det;
			const double step = std::sqrt(da * da + db * db);
			if (step > 0.5 * L) { da *= 0.5 * L / step; db *= 0.5 * L / step; }
			R += e1 * da + e2 * db;
			c = sdf.ClosestPoint(R);
			if (c.distance > 0.5 * L) return false;
			R = c.point;
			tri = c.triangle;
		}
		if (!converged) {
			const double r1 = std::fabs(LengthSq(R - A->p) - L2), r2 = std::fabs(LengthSq(R - B->p) - L2);
			if (r1 > 1e-4 * L2 || r2 > 1e-4 * L2) return false;
		}
		// The cell must not fold: R lies on the far side of the diagonal A-B
		// from O, and the warp and weft at R still make a sensible angle.
		const Vec3 nO = Cross(A->p - O->p, B->p - O->p);
		const Vec3 nR = Cross(B->p - R, A->p - R);
		if (Dot(nO, nR) <= 0.0) return false;
		const double cosang = Dot(Normalized(R - B->p), Normalized(R - A->p));
		if (std::fabs(cosang) > std::cos(5.0 * kPi / 180.0)) return false;   // sheared past 85 degrees
		if (overlaps(R, i, j)) return false;
		NetNode nd;
		nd.p = R; nd.tri = tri;
		put(i, j, nd);
		return true;
	};

	for (int n = 1; n <= options.max_steps; ++n) {
		int added = 0;
		added += grow_generator(1, 0, w0, n) ? 1 : 0;
		added += grow_generator(-1, 0, -w0, n) ? 1 : 0;
		added += grow_generator(0, 1, f0, n) ? 1 : 0;
		added += grow_generator(0, -1, -f0, n) ? 1 : 0;
		// The ring's sides: (+-n, j) for |j| < n, then (i, +-n) for |i| <= n.
		for (int j = 1; j < n; ++j) {
			for (int sj : {1, -1}) {
				for (int si : {1, -1}) added += place(si * n, sj * j) ? 1 : 0;
			}
		}
		for (int i = 1; i <= n; ++i) {
			for (int si : {1, -1}) {
				for (int sj : {1, -1}) added += place(si * i, sj * n) ? 1 : 0;
			}
		}
		if (added == 0) break;
	}

	// The cells: four present corners; the warp and weft across them and
	// the shear between.
	struct Cell { Vec3 centre, warp, weft, normal; double shear; };
	std::vector<Cell> cells;
	int imin = 0, imax = 0, jmin = 0, jmax = 0;
	for (const auto& kv : nodes) {
		if (!kv.second.ok) continue;
		imin = std::min(imin, kv.first.first); imax = std::max(imax, kv.first.first);
		jmin = std::min(jmin, kv.first.second); jmax = std::max(jmax, kv.first.second);
		out.net.push_back({kv.first.first, kv.first.second, kv.second.p});
	}
	out.nodes = static_cast<int>(out.net.size());
	for (int i = imin; i < imax; ++i) {
		for (int j = jmin; j < jmax; ++j) {
			const NetNode* p00 = get(i, j);
			const NetNode* p10 = get(i + 1, j);
			const NetNode* p01 = get(i, j + 1);
			const NetNode* p11 = get(i + 1, j + 1);
			if (!p00 || !p10 || !p01 || !p11) continue;
			Cell c;
			c.centre = (p00->p + p10->p + p01->p + p11->p) * 0.25;
			c.warp = Normalized((p10->p - p00->p) + (p11->p - p01->p));
			c.weft = Normalized((p01->p - p00->p) + (p11->p - p10->p));
			c.normal = Normalized(Cross(c.warp, c.weft));
			const double cosang = std::clamp(Dot(c.warp, c.weft), -1.0, 1.0);
			// The magnitude: which way the fibres closed depends on the
			// quadrant of the net, and the cloth locks either way.
			c.shear = std::fabs(90.0 - std::acos(cosang) * 180.0 / kPi);
			cells.push_back(c);
		}
	}
	out.cells = static_cast<int>(cells.size());
	if (cells.empty()) {
		out.notes.push_back("the net could not grow from the seed (no complete cell)");
		return out;
	}
	Hash3 cell_hash;
	cell_hash.cell = L;
	for (size_t k = 0; k < cells.size(); ++k) cell_hash.Add(cells[k].centre, static_cast<int>(k));
	double covered_area = 0.0, total_area = 0.0;
	for (size_t t = 0; t < nt; ++t) {
		const auto& tri = surface.triangles[t];
		const Vec3 centroid = (surface.vertices[tri[0]] + surface.vertices[tri[1]] + surface.vertices[tri[2]]) / 3.0;
		const Vec3 n = walker.normals[t];
		const double area = 0.5 * Length(Cross(surface.vertices[tri[1]] - surface.vertices[tri[0]], surface.vertices[tri[2]] - surface.vertices[tri[0]]));
		total_area += area;
		int best = -1;
		double best_d = 1.5 * L;
		cell_hash.Near(centroid, 2, [&](int k) {
			const Cell& c = cells[k];
			if (Dot(c.normal, n) < 0.3) return;   // the other skin of a thin body
			const double d = Distance(c.centre, centroid);
			if (d < best_d) { best_d = d; best = k; }
		});
		if (best < 0) continue;
		const Cell& c = cells[best];
		Vec3 w = Normalized(c.warp - n * Dot(c.warp, n));
		Vec3 f = Normalized(c.weft - n * Dot(c.weft, n));
		if (LengthSq(w) == 0.0 || LengthSq(f) == 0.0) continue;
		out.warp[t] = w;
		out.weft[t] = f;
		out.shear[t] = c.shear;
		out.covered[t] = 1;
		out.max_shear = std::max(out.max_shear, std::fabs(c.shear));
		covered_area += area;
	}
	out.covered_fraction = total_area > 0.0 ? covered_area / total_area : 0.0;
	return out;
}

std::vector<double> WarpDeviation(const TriMesh& surface, const DrapeResult& drape, const std::vector<Vec3>& reference) {
	std::vector<double> out(surface.triangles.size(), 0.0);
	for (size_t t = 0; t < surface.triangles.size() && t < reference.size() && t < drape.warp.size(); ++t) {
		if (!drape.covered[t]) continue;
		const Vec3 n = surface.TriangleNormal(static_cast<int>(t));
		const Vec3 r = Normalized(reference[t] - n * Dot(reference[t], n));
		if (LengthSq(r) == 0.0) continue;
		const Vec3& w = drape.warp[t];
		double a = std::atan2(Dot(Cross(r, w), n), Dot(r, w)) * 180.0 / kPi;
		if (a > 90.0) a -= 180.0;
		if (a <= -90.0) a += 180.0;
		out[t] = a;
	}
	return out;
}

} // namespace aerotelic
