#include "aerotelic/mesh_cut.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

namespace aerotelic {

namespace {

// Cuts by one plane in place: `tris` and `parents` are rebuilt, `verts`
// grows by the crossing points.
void CutByPlane(std::vector<Vec3>& verts, std::vector<std::array<int, 3>>& tris, std::vector<int>& parents,
	const Plane& p, double tol, MeshCutStats* stats) {
	const size_t nv = verts.size();
	std::vector<double> dist(nv);
	for (size_t i = 0; i < nv; ++i) {
		double d = p.Distance(verts[i]);
		if (std::abs(d) <= tol) {
			if (d != 0.0) {
				verts[i] = verts[i] - p.normal * d;
				if (stats) ++stats->snapped_vertices;
			}
			d = 0.0;
		}
		dist[i] = d;
	}
	// The crossing point of an edge, once per edge, interpolated from the
	// lower index so both triangles agree to the bit.
	std::map<std::pair<int, int>, int> crossing;
	auto cross = [&](int a, int b) {
		const std::pair<int, int> key{std::min(a, b), std::max(a, b)};
		auto it = crossing.find(key);
		if (it != crossing.end()) return it->second;
		const double da = dist[key.first], db = dist[key.second];
		const double t = da / (da - db);
		verts.push_back(Lerp(verts[key.first], verts[key.second], t));
		dist.push_back(0.0);
		const int idx = static_cast<int>(verts.size()) - 1;
		crossing.emplace(key, idx);
		if (stats) ++stats->new_vertices;
		return idx;
	};
	auto sign = [&](int v) { return dist[v] > 0.0 ? 1 : (dist[v] < 0.0 ? -1 : 0); };

	std::vector<std::array<int, 3>> out;
	std::vector<int> out_parent;
	out.reserve(tris.size() + tris.size() / 4);
	out_parent.reserve(out.capacity());
	for (size_t t = 0; t < tris.size(); ++t) {
		std::array<int, 3> tri = tris[t];
		const int s[3] = {sign(tri[0]), sign(tri[1]), sign(tri[2])};
		bool pos = false, neg = false;
		int zeros = 0;
		for (int k = 0; k < 3; ++k) {
			pos = pos || s[k] > 0;
			neg = neg || s[k] < 0;
			zeros += s[k] == 0 ? 1 : 0;
		}
		if (!(pos && neg)) {
			out.push_back(tri);
			out_parent.push_back(parents[t]);
			continue;
		}
		if (stats) ++stats->split_triangles;
		if (zeros == 1) {
			// Rotate so the on-plane vertex is first (the cyclic order, so
			// the winding, is kept); the opposite edge is split once.
			int r = 0;
			while (sign(tri[r]) != 0) ++r;
			const std::array<int, 3> rot{tri[r], tri[(r + 1) % 3], tri[(r + 2) % 3]};
			const int pm = cross(rot[1], rot[2]);
			out.push_back({rot[0], rot[1], pm});
			out_parent.push_back(parents[t]);
			out.push_back({rot[0], pm, rot[2]});
			out_parent.push_back(parents[t]);
			continue;
		}
		// No vertex on the plane: one vertex alone on its side. Rotate it
		// first; the two edges from it are split, the far side is a quad
		// cut into two.
		int lone = 0;
		for (int k = 0; k < 3; ++k) {
			if (s[k] != s[(k + 1) % 3] && s[k] != s[(k + 2) % 3]) lone = k;
		}
		const std::array<int, 3> rot{tri[lone], tri[(lone + 1) % 3], tri[(lone + 2) % 3]};
		const int p01 = cross(rot[0], rot[1]);
		const int p02 = cross(rot[0], rot[2]);
		out.push_back({rot[0], p01, p02});
		out_parent.push_back(parents[t]);
		out.push_back({p01, rot[1], rot[2]});
		out_parent.push_back(parents[t]);
		out.push_back({p01, rot[2], p02});
		out_parent.push_back(parents[t]);
	}
	tris.swap(out);
	parents.swap(out_parent);
}

} // namespace

TriMesh CutMeshByPlanes(const TriMesh& mesh, const std::vector<Plane>& planes, double tol,
	std::vector<int>* parent, MeshCutStats* stats) {
	if (stats) *stats = MeshCutStats();
	TriMesh out = mesh;
	std::vector<int> parents(mesh.triangles.size());
	for (size_t i = 0; i < parents.size(); ++i) parents[i] = static_cast<int>(i);
	for (const Plane& p : planes) {
		const double n = Length(p.normal);
		if (!(n > 0.0)) continue;
		Plane unit{p.normal / n, p.offset / n};
		CutByPlane(out.vertices, out.triangles, parents, unit, std::max(tol, 0.0), stats);
	}
	if (parent) *parent = std::move(parents);
	return out;
}

std::vector<std::vector<int>> SectionLoopsOnPlane(const TriMesh& cut, const Plane& p, double tol) {
	std::vector<std::vector<int>> loops;
	const size_t nv = cut.vertices.size();
	std::vector<double> dist(nv);
	std::vector<char> on(nv, 0);
	for (size_t i = 0; i < nv; ++i) {
		dist[i] = p.Distance(cut.vertices[i]);
		on[i] = std::abs(dist[i]) <= tol ? 1 : 0;
	}
	// Per on-plane edge: whether a triangle on the positive side (or in the
	// plane) uses it, and the negative-side triangle's traversal of it with
	// its outward normal.
	struct EdgeInfo {
		bool positive = false;
		bool negative = false;
		int a = -1, b = -1;
		Vec3 normal;
	};
	std::map<std::pair<int, int>, EdgeInfo> edges;
	for (size_t t = 0; t < cut.triangles.size(); ++t) {
		const auto& tri = cut.triangles[t];
		int on_count = on[tri[0]] + on[tri[1]] + on[tri[2]];
		if (on_count < 2) continue;
		for (int e = 0; e < 3; ++e) {
			const int a = tri[e], b = tri[(e + 1) % 3], c = tri[(e + 2) % 3];
			if (!on[a] || !on[b]) continue;
			EdgeInfo& info = edges[{std::min(a, b), std::max(a, b)}];
			if (dist[c] < -tol) {
				info.negative = true;
				info.a = a;
				info.b = b;
				info.normal = cut.TriangleNormal(static_cast<int>(t));
			} else {
				info.positive = true;
			}
		}
	}
	// Orient: the material lies to the LEFT of the edge (counter-clockwise
	// about the normal), and the negative-side triangle's outward normal
	// points away from the material.
	std::map<int, int> next;
	for (const auto& entry : edges) {
		const EdgeInfo& info = entry.second;
		if (!info.positive || !info.negative) continue;
		const Vec3 left = Cross(p.normal, cut.vertices[info.b] - cut.vertices[info.a]);
		if (Dot(left, info.normal) < 0.0) next[info.a] = info.b;
		else next[info.b] = info.a;
	}
	std::map<int, bool> used;
	for (const auto& start : next) {
		if (used[start.first]) continue;
		std::vector<int> loop;
		int v = start.first;
		while (!used[v]) {
			used[v] = true;
			loop.push_back(v);
			auto it = next.find(v);
			if (it == next.end()) break;
			v = it->second;
		}
		if (loop.size() >= 3) loops.push_back(std::move(loop));
	}
	return loops;
}

double LoopArea(const TriMesh& mesh, const std::vector<int>& loop, const Plane& p) {
	Vec3 u, v;
	PlaneBasis(p.normal, &u, &v);
	double area = 0.0;
	for (size_t i = 0; i < loop.size(); ++i) {
		const Vec3 a = mesh.vertices[loop[i]], b = mesh.vertices[loop[(i + 1) % loop.size()]];
		area += Dot(a, u) * Dot(b, v) - Dot(b, u) * Dot(a, v);
	}
	return 0.5 * area;
}

} // namespace aerotelic
