#include "aerotelic/shell.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace aerotelic {

namespace {

double TetVolume(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
	return Dot(b - a, Cross(c - a, d - a)) / 6.0;
}

// Angle-weighted vertex normals of a closed, outward-oriented mesh.
std::vector<Vec3> VertexNormals(const TriMesh& m) {
	std::vector<Vec3> normals(m.vertices.size(), Vec3{});
	for (const auto& t : m.triangles) {
		const Vec3& a = m.vertices[t[0]];
		const Vec3& b = m.vertices[t[1]];
		const Vec3& c = m.vertices[t[2]];
		Vec3 n = Normalized(Cross(b - a, c - a));
		const Vec3* p[3] = {&a, &b, &c};
		for (int k = 0; k < 3; ++k) {
			Vec3 e1 = Normalized(*p[(k + 1) % 3] - *p[k]);
			Vec3 e2 = Normalized(*p[(k + 2) % 3] - *p[k]);
			double angle = std::acos(std::clamp(Dot(e1, e2), -1.0, 1.0));
			normals[t[k]] += n * angle;
		}
	}
	for (auto& n : normals) n = Normalized(n);
	return normals;
}

// The walk from one vertex: how far along -normal to go, and why it stopped.
struct Walk {
	double distance = 0.0;
	double depth = 0.0;     // field value at the end (negative inside)
	bool collapsed = false; // stopped on the mid-surface before full depth
	bool bad = false;       // could not enter the body at all
};

Walk WalkInward(const Vec3& v, const Vec3& dir, const MeshSdf& sdf, const ShellOptions& opt) {
	const double t = opt.thickness;
	const double step = std::max(opt.step_fraction, 0.01) * t;
	const double target = -t;
	const double tol = 1e-5 * t;
	Walk w;
	auto field = [&](double s) { return sdf.Distance(v + dir * s); };

	// Bisection for the field reaching the target between lo (above) and
	// hi (at or below).
	auto to_target = [&](double lo, double hi) {
		for (int it = 0; it < 60 && hi - lo > tol; ++it) {
			double mid = 0.5 * (lo + hi);
			if (field(mid) <= target) hi = mid; else lo = mid;
		}
		w.distance = hi;
		w.depth = field(hi);
		w.collapsed = false;
	};

	double s_prev = 0.0;
	double best_s = 0.0, best_b = std::min(field(0.0), 0.0);
	for (int k = 1; k <= 400; ++k) {
		double s = k * step;
		double b = field(s);
		if (b <= target) {
			to_target(s_prev, s);
			return w;
		}
		if (b >= 0.0 && k == 1) {
			// The first step left the body: the normal does not point in.
			w.bad = true;
			w.distance = 0.0;
			w.depth = best_b;
			return w;
		}
		// Already on (or walking along) the mid-surface: the surface is far
		// nearer than the distance walked. A vertex on a sharp trailing
		// edge is the case -- its normal bisects the wedge, and the field
		// keeps deepening along the wedge's middle, so the minimum test
		// below would never fire and the vertex would walk forward along
		// the camber line instead of staying put. A right-angle edge walks
		// its bisector at 0.71 of the distance and is left alone.
		if (-b < 0.5 * s) {
			w.distance = best_s;
			w.depth = best_b;
			w.collapsed = true;
			return w;
		}
		if (b > best_b + 1e-9 * t) {
			// Past the minimum: the mid-surface lies in [s_prev - step, s].
			// Golden-section search for the most negative field.
			double lo = std::max(0.0, s_prev - step), hi = s;
			const double g = 0.6180339887498949;
			double x1 = hi - g * (hi - lo), x2 = lo + g * (hi - lo);
			double f1 = field(x1), f2 = field(x2);
			for (int it = 0; it < 60 && hi - lo > tol; ++it) {
				if (f1 < f2) { hi = x2; x2 = x1; f2 = f1; x1 = hi - g * (hi - lo); f1 = field(x1); }
				else { lo = x1; x1 = x2; f1 = f2; x2 = lo + g * (hi - lo); f2 = field(x2); }
			}
			double xs = f1 < f2 ? x1 : x2, fs = std::min(f1, f2);
			if (fs <= target) {
				// The field dipped below the target between two samples (the
				// body is just over two skins thick here): the offset surface
				// was reached after all.
				to_target(std::max(0.0, s_prev - step), xs);
				return w;
			}
			w.distance = xs;
			w.depth = fs;
			w.collapsed = true;
			return w;
		}
		if (b < best_b) { best_b = b; best_s = s; }
		s_prev = s;
	}
	// Ran out of steps (should not happen: 400 steps is 40 thicknesses).
	w.distance = best_s;
	w.depth = best_b;
	w.collapsed = true;
	return w;
}

} // namespace

namespace {

// Closes every open loop of `m` with triangles. The loops this is for lie
// in the mid-surface of a thin region: the two sides' mixed triangles end
// on the same welded nodes but with different diagonals, so dropping the
// collapsed sheet leaves a chain of narrow lenses open along the pinch.
// Each lens is capped in its own plane (zero volume, so the enclosed
// volume is unchanged). The open edges are traced as an Euler circuit per
// component (every node has as many open edges going out as coming in,
// the sheet being orientable) and split into simple cycles at the nodes
// where chains touch.
void SealOpenLoops(TriMesh& m) {
	std::unordered_map<uint64_t, int> edge_count;
	auto key_of = [](int a, int b) { return (static_cast<uint64_t>(std::min(a, b)) << 32) | static_cast<uint64_t>(std::max(a, b)); };
	for (const auto& t : m.triangles)
		for (int k = 0; k < 3; ++k) ++edge_count[key_of(t[k], t[(k + 1) % 3])];
	// Directed open edges, reversed (so a cap traverses them against the
	// triangle that owns them).
	std::unordered_map<int, std::vector<int>> out; // node -> next nodes
	int n_open = 0;
	for (const auto& t : m.triangles)
		for (int k = 0; k < 3; ++k) {
			int a = t[k], b = t[(k + 1) % 3];
			if (edge_count[key_of(a, b)] != 1) continue;
			out[b].push_back(a);
			++n_open;
		}
	if (n_open == 0) return;
	std::unordered_map<int, size_t> next_out;
	int used = 0;
	while (used < n_open) {
		// A start node with unused outgoing edges.
		int start = -1;
		for (auto& [node, nexts] : out) {
			if (next_out[node] < nexts.size()) { start = node; break; }
		}
		if (start < 0) break;
		// Hierholzer's circuit.
		std::vector<int> stack = {start}, circuit;
		while (!stack.empty()) {
			int v = stack.back();
			auto& nexts = out[v];
			size_t& pos = next_out[v];
			if (pos < nexts.size()) {
				stack.push_back(nexts[pos++]);
				++used;
			} else {
				circuit.push_back(v);
				stack.pop_back();
			}
		}
		std::reverse(circuit.begin(), circuit.end()); // start ... start
		// Split into simple cycles.
		std::vector<int> path;
		std::unordered_map<int, int> where;
		for (size_t i = 0; i + 1 < circuit.size(); ++i) {
			int v = circuit[i];
			auto it = where.find(v);
			if (it != where.end()) {
				std::vector<int> cycle(path.begin() + it->second, path.end());
				for (int u : cycle) where.erase(u);
				path.resize(it->second);
				if (cycle.size() >= 3) AppendPolygonCap(m, cycle);
			}
			where[v] = static_cast<int>(path.size());
			path.push_back(v);
		}
		if (path.size() >= 3) AppendPolygonCap(m, path);
	}
	m.RemoveDegenerateTriangles();
}

} // namespace

TriMesh ShellResult::InnerSurface(bool drop_collapsed) const {
	TriMesh m;
	m.vertices = nodes;
	for (size_t i = 0; i < inner_triangles.size(); ++i) {
		const auto& t = inner_triangles[i];
		if (t[0] < 0) continue;
		if (drop_collapsed && inner_collapsed[i]) continue;
		m.triangles.push_back(t);
	}
	if (drop_collapsed && !m.triangles.empty()) SealOpenLoops(m);
	// Compact to the nodes used.
	std::vector<int> remap(m.vertices.size(), -1);
	std::vector<Vec3> kept;
	for (auto& t : m.triangles)
		for (int& idx : t) {
			if (remap[idx] < 0) { remap[idx] = static_cast<int>(kept.size()); kept.push_back(m.vertices[idx]); }
			idx = remap[idx];
		}
	m.vertices = std::move(kept);
	return m;
}

ShellResult BuildOffsetShell(const TriMesh& surface, const MeshSdf& sdf, const ShellOptions& opt) {
	if (surface.IsEmpty()) throw std::invalid_argument("BuildOffsetShell: empty surface");
	if (!(opt.thickness > 0.0)) throw std::invalid_argument("BuildOffsetShell: the thickness must be positive");
	const double t = opt.thickness;
	const int nv = static_cast<int>(surface.vertices.size());

	ShellResult r;
	r.nodes = surface.vertices;
	r.n_surface_nodes = nv;
	r.offset_node.assign(nv, -1);
	r.walk_distance.assign(nv, 0.0);
	r.depth.assign(nv, 0.0);
	r.collapsed.assign(nv, false);

	// ---- the walks ----
	std::vector<Vec3> normals = VertexNormals(surface);
	std::vector<Vec3> dirs(nv);
	std::vector<bool> collapsed_walk(nv, false);
	for (int i = 0; i < nv; ++i) {
		dirs[i] = -normals[i];
		Walk w = WalkInward(surface.vertices[i], dirs[i], sdf, opt);
		r.walk_distance[i] = w.distance;
		collapsed_walk[i] = w.collapsed;
		if (w.bad) ++r.n_bad_normals;
	}
	r.walk_unlimited = r.walk_distance;

	// ---- the Lipschitz bound along edges ----
	{
		std::vector<std::pair<int, int>> edges;
		edges.reserve(surface.triangles.size() * 3);
		for (const auto& tri : surface.triangles)
			for (int k = 0; k < 3; ++k) edges.push_back({tri[k], tri[(k + 1) % 3]});
		std::vector<bool> limited(nv, false);
		for (int iter = 0; iter < 200; ++iter) {
			bool changed = false;
			for (const auto& [a, b] : edges) {
				double len = Distance(surface.vertices[a], surface.vertices[b]);
				double cap = r.walk_distance[b] + opt.lipschitz * len;
				if (r.walk_distance[a] > cap + 1e-12 * t) { r.walk_distance[a] = cap; limited[a] = true; changed = true; }
				cap = r.walk_distance[a] + opt.lipschitz * len;
				if (r.walk_distance[b] > cap + 1e-12 * t) { r.walk_distance[b] = cap; limited[b] = true; changed = true; }
			}
			if (!changed) break;
		}
		for (int i = 0; i < nv; ++i) if (limited[i]) ++r.n_lipschitz_limited;
	}

	// ---- the offset nodes (welded), the prisms, the inner surface ----
	// Built in rounds: a prism whose top triangle faces against its bottom
	// has twisted (a vertex displaced sideways by more than its neighbours'
	// spacing -- a crease vertex walking its bisector beside a sliver), and
	// its vertices' walks are halved for the next round.
	for (int round = 0; round < 20; ++round) {
	r.nodes.assign(surface.vertices.begin(), surface.vertices.end());
	r.offset_node.assign(nv, -1);
	r.depth.assign(nv, 0.0);
	r.collapsed.assign(nv, false);
	r.n_collapsed_vertices = 0;
	r.tets.clear();
	r.tet_triangle.clear();
	r.inner_triangles.clear();
	r.inner_collapsed.clear();
	r.n_degenerate_tets = 0;
	r.n_negative_tets = 0;
	r.n_cancelled_pairs = 0;
	r.skin_volume = 0.0;
	{
		const double weld = opt.weld_fraction * t;
		const double weld2 = weld * weld;
		struct CellHash {
			size_t operator()(const std::array<int64_t, 3>& k) const { return static_cast<size_t>(k[0] * 73856093LL ^ k[1] * 19349663LL ^ k[2] * 83492791LL); }
		};
		std::unordered_map<std::array<int64_t, 3>, std::vector<int>, CellHash> grid;
		const double cell = std::max(weld, 1e-12);
		auto key_of = [&](const Vec3& p) {
			return std::array<int64_t, 3>{static_cast<int64_t>(std::floor(p.x / cell)), static_cast<int64_t>(std::floor(p.y / cell)), static_cast<int64_t>(std::floor(p.z / cell))};
		};
		for (int i = 0; i < nv; ++i) {
			double d = r.walk_distance[i];
			Vec3 p = surface.vertices[i] + dirs[i] * d;
			if (d <= weld) {
				// Went nowhere: the offset node is the vertex itself.
				r.offset_node[i] = i;
				r.depth[i] = sdf.Distance(surface.vertices[i]);
				r.collapsed[i] = true;
				continue;
			}
			auto k = key_of(p);
			int found = -1;
			for (int dx = -1; dx <= 1 && found < 0; ++dx)
				for (int dy = -1; dy <= 1 && found < 0; ++dy)
					for (int dz = -1; dz <= 1 && found < 0; ++dz) {
						auto it = grid.find({k[0] + dx, k[1] + dy, k[2] + dz});
						if (it == grid.end()) continue;
						for (int j : it->second) if (DistanceSq(r.nodes[j], p) <= weld2) { found = j; break; }
					}
			if (found < 0) {
				found = static_cast<int>(r.nodes.size());
				r.nodes.push_back(p);
				grid[k].push_back(found);
			}
			r.offset_node[i] = found;
			r.depth[i] = sdf.Distance(r.nodes[found]);
			r.collapsed[i] = collapsed_walk[i] || (-r.depth[i] < opt.collapse_fraction * t);
		}
		for (int i = 0; i < nv; ++i) if (r.collapsed[i]) ++r.n_collapsed_vertices;
	}

	// ---- prisms into tetrahedra, inner surface ----
	std::vector<int> folded_prism;
	r.min_tet_volume = std::numeric_limits<double>::infinity();
	r.inner_triangles.reserve(surface.triangles.size());
	r.inner_collapsed.reserve(surface.triangles.size());
	for (size_t ti = 0; ti < surface.triangles.size(); ++ti) {
		const auto& tri = surface.triangles[ti];
		// Prism nodes: bottom 0,1,2 (surface), top 3,4,5 (offset).
		int p[6] = {tri[0], tri[1], tri[2], r.offset_node[tri[0]], r.offset_node[tri[1]], r.offset_node[tri[2]]};

		// Inner triangle (the offset copy, same winding).
		{
			std::array<int, 3> it = {p[3], p[4], p[5]};
			bool degenerate = it[0] == it[1] || it[1] == it[2] || it[0] == it[2];
			r.inner_triangles.push_back(degenerate ? std::array<int, 3>{-1, -1, -1} : it);
			r.inner_collapsed.push_back(r.collapsed[tri[0]] && r.collapsed[tri[1]] && r.collapsed[tri[2]]);
		}

		// Rotate so the smallest-index node of the bottom is first (the
		// top follows); the rule then needs only the bottom's order, since
		// a node's offset carries its index ordering through the bottom's.
		// (Dompierre, Labbe, Vallet, Camarero: "How to subdivide pyramids,
		// prisms and hexahedra into tetrahedra", with the smallest global
		// index at node 0.)
		int rot = 0;
		int smallest = std::min({p[0], p[1], p[2], p[3], p[4], p[5]});
		for (int k = 0; k < 3; ++k) if (p[k] == smallest || p[k + 3] == smallest) rot = k;
		int q[6];
		for (int k = 0; k < 3; ++k) { q[k] = p[(rot + k) % 3]; q[k + 3] = p[3 + (rot + k) % 3]; }
		bool top_first = q[3] == smallest && q[0] != smallest;
		if (top_first) {
			// The smallest index is on the top: swap bottom and top (and
			// reverse one to keep the prism's handedness).
			std::swap(q[0], q[3]);
			std::swap(q[1], q[5]);
			std::swap(q[2], q[4]);
		}
		// The quad face not touching node 0 is (1,2,5,4): diagonal from its
		// smallest index.
		int m = std::min({q[1], q[2], q[4], q[5]});
		std::array<std::array<int, 4>, 3> cand;
		if (m == q[1] || m == q[5]) {
			cand = {{{q[0], q[1], q[2], q[5]}, {q[0], q[1], q[5], q[4]}, {q[0], q[4], q[5], q[3]}}};
		} else {
			cand = {{{q[0], q[1], q[2], q[4]}, {q[0], q[4], q[2], q[5]}, {q[0], q[4], q[5], q[3]}}};
		}
		// With the bottom wound outward and the top inside, every tet of a
		// valid prism comes out with a NEGATIVE signed volume from these
		// formulas (in both the plain and the swapped labelling); a folded
		// prism -- offset nodes crossed over sideways -- gives mixed signs.
		double e2 = std::max({DistanceSq(surface.vertices[tri[0]], surface.vertices[tri[1]]), DistanceSq(surface.vertices[tri[1]], surface.vertices[tri[2]]),
			DistanceSq(surface.vertices[tri[2]], surface.vertices[tri[0]])});
		const double vol_eps = 1e-12 * e2 * t;
		bool folded = false;
		for (auto tet : cand) {
			if (tet[0] == tet[1] || tet[0] == tet[2] || tet[0] == tet[3] || tet[1] == tet[2] || tet[1] == tet[3] || tet[2] == tet[3]) {
				++r.n_degenerate_tets;
				continue;
			}
			double vol = TetVolume(r.nodes[tet[0]], r.nodes[tet[1]], r.nodes[tet[2]], r.nodes[tet[3]]);
			if (std::fabs(vol) <= vol_eps) { ++r.n_degenerate_tets; continue; }
			if (vol > 0.0) folded = true;
			if (vol < 0.0) { std::swap(tet[2], tet[3]); vol = -vol; }
			r.tets.push_back(tet);
			r.tet_triangle.push_back(static_cast<int>(ti));
			r.min_tet_volume = std::min(r.min_tet_volume, vol);
			r.skin_volume += vol;
		}
		if (folded) folded_prism.push_back(static_cast<int>(ti));
	}
	if (r.tets.empty()) r.min_tet_volume = 0.0;

	// Where the two sides of a thin region collapsed onto the same
	// mid-surface nodes (a symmetric section does this exactly), the
	// inner copy holds each sheet triangle twice with opposite winding.
	// Such a pair bounds no volume: drop both. What remains is sealed
	// along the pinch by the nodes the two sides share.
	{
		std::unordered_map<uint64_t, int> seen; // sorted triple -> triangle index
		auto key_of = [](std::array<int, 3> t) {
			std::sort(t.begin(), t.end());
			return (static_cast<uint64_t>(t[0]) << 42) ^ (static_cast<uint64_t>(t[1]) << 21) ^ static_cast<uint64_t>(t[2]);
		};
		for (size_t i = 0; i < r.inner_triangles.size(); ++i) {
			auto& tri = r.inner_triangles[i];
			if (tri[0] < 0) continue;
			uint64_t key = key_of(tri);
			auto it = seen.find(key);
			if (it == seen.end()) { seen.emplace(key, static_cast<int>(i)); continue; }
			auto& other = r.inner_triangles[it->second];
			if (other[0] < 0) { it->second = static_cast<int>(i); continue; }
			// Same node set: opposite winding cancels; same winding is a
			// genuine double cover (left alone, reported).
			Vec3 n1 = Cross(r.nodes[tri[1]] - r.nodes[tri[0]], r.nodes[tri[2]] - r.nodes[tri[0]]);
			Vec3 n2 = Cross(r.nodes[other[1]] - r.nodes[other[0]], r.nodes[other[2]] - r.nodes[other[0]]);
			bool opposite = Dot(n1, n2) <= 0.0;
			if (!opposite) {
				// Degenerate (zero-area) triangles have no winding to compare:
				// treat them as cancelling too.
				opposite = LengthSq(n1) == 0.0 || LengthSq(n2) == 0.0;
			}
			if (opposite) {
				tri = {-1, -1, -1};
				other = {-1, -1, -1};
				++r.n_cancelled_pairs;
				seen.erase(it);
			}
		}
	}

	// Folded prisms: shorten the walks of their vertices and go again. A
	// prism folds where the offset nodes of an edge cross over -- normals
	// converging under a leading edge whose radius is near the skin
	// thickness, a crease vertex walking its bisector beside a sliver --
	// so each side edge is scaled back to keep a fifth of its length in
	// the offset layer; an edge that does not explain the fold is halved.
	// What is left after the rounds is reported as negative.
	r.n_negative_tets = 0;
	for (int ti : folded_prism)
		for (size_t k = 0; k < r.tet_triangle.size(); ++k) if (r.tet_triangle[k] == ti) ++r.n_negative_tets;
	r.n_relaxation_rounds = round;
	if (folded_prism.empty() || round == 19) break;
	std::vector<double> scale(nv, 1.0);
	for (int ti : folded_prism) {
		const auto& tri = surface.triangles[ti];
		for (int k = 0; k < 3; ++k) {
			int a = tri[k], b = tri[(k + 1) % 3];
			Vec3 e = surface.vertices[b] - surface.vertices[a];
			double e2 = LengthSq(e);
			if (e2 <= 0.0) continue;
			// Offset edge = e + (w_b d_b - w_a d_a); keep its projection on e
			// at least 0.2 |e|^2.
			Vec3 delta = dirs[b] * r.walk_distance[b] - dirs[a] * r.walk_distance[a];
			double shrink_along = -Dot(delta, e);
			double lambda = shrink_along > 0.0 ? 0.8 * e2 / shrink_along : 1.0;
			if (lambda >= 1.0) lambda = 0.5;
			lambda = std::clamp(lambda, 0.05, 0.9);
			scale[a] = std::min(scale[a], lambda);
			scale[b] = std::min(scale[b], lambda);
		}
	}
	for (int i = 0; i < nv; ++i) r.walk_distance[i] *= scale[i];
	} // rounds
	return r;
}

bool WriteShellVtk(const ShellResult& shell, const std::string& path) {
	FILE* f = std::fopen(path.c_str(), "w");
	if (!f) return false;
	std::fprintf(f, "# vtk DataFile Version 3.0\nAerotelic skin tetrahedra\nASCII\nDATASET UNSTRUCTURED_GRID\n");
	std::fprintf(f, "POINTS %zu double\n", shell.nodes.size());
	for (const auto& p : shell.nodes) std::fprintf(f, "%.9g %.9g %.9g\n", p.x, p.y, p.z);
	std::fprintf(f, "CELLS %zu %zu\n", shell.tets.size(), shell.tets.size() * 5);
	for (const auto& t : shell.tets) std::fprintf(f, "4 %d %d %d %d\n", t[0], t[1], t[2], t[3]);
	std::fprintf(f, "CELL_TYPES %zu\n", shell.tets.size());
	for (size_t i = 0; i < shell.tets.size(); ++i) std::fprintf(f, "10\n");
	std::fprintf(f, "CELL_DATA %zu\nSCALARS material int 1\nLOOKUP_TABLE default\n", shell.tets.size());
	for (size_t i = 0; i < shell.tets.size(); ++i) std::fprintf(f, "1\n");
	std::fclose(f);
	return true;
}

bool WriteStlAscii(const TriMesh& mesh, const std::string& path, const std::string& solid_name) {
	FILE* f = std::fopen(path.c_str(), "w");
	if (!f) return false;
	std::fprintf(f, "solid %s\n", solid_name.c_str());
	for (const auto& t : mesh.triangles) {
		const Vec3& a = mesh.vertices[t[0]];
		const Vec3& b = mesh.vertices[t[1]];
		const Vec3& c = mesh.vertices[t[2]];
		Vec3 n = Normalized(Cross(b - a, c - a));
		std::fprintf(f, "  facet normal %.9g %.9g %.9g\n    outer loop\n      vertex %.9g %.9g %.9g\n      vertex %.9g %.9g %.9g\n      vertex %.9g %.9g %.9g\n    endloop\n  endfacet\n",
			n.x, n.y, n.z, a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z);
	}
	std::fprintf(f, "endsolid %s\n", solid_name.c_str());
	std::fclose(f);
	return true;
}

} // namespace aerotelic
