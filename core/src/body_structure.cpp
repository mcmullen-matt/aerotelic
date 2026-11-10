#include "aerotelic/body_structure.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>

namespace aerotelic {

bool SameStructureGeometry(const BodyStructure& a, const BodyStructure& b) {
	if (a.skin_thickness != b.skin_thickness) return false;
	return a.exclusions == b.exclusions;
}

std::string BodyStructureProblem(const BodyStructure& s) {
	if (s.skin_thickness < 0.0 || !std::isfinite(s.skin_thickness)) return "the skin thickness cannot be negative";
	if (s.asserted_mass && (!(*s.asserted_mass >= 0.0) || !std::isfinite(*s.asserted_mass))) return "the asserted mass cannot be negative";
	return std::string();
}

// ---- plane geometry -------------------------------------------------------

void PlaneBasis(const Vec3& n, Vec3* u, Vec3* v) {
	// Cross with the axis the normal leans on least, so the basis is well
	// conditioned whatever the normal.
	const double ax = std::fabs(n.x), ay = std::fabs(n.y), az = std::fabs(n.z);
	Vec3 a = ax <= ay && ax <= az ? Vec3{1.0, 0.0, 0.0} : (ay <= az ? Vec3{0.0, 1.0, 0.0} : Vec3{0.0, 0.0, 1.0});
	*u = Normalized(Cross(n, a));
	*v = Cross(n, *u);   // u x v = u x (n x u) = n (u.u) - u (u.n) = n
}

double PlaneSection::Area() const {
	double a = 0.0;
	for (double s : areas) a += s;
	return a;
}

double PlaneSection::Perimeter() const {
	double l = 0.0;
	for (const auto& loop : loops) {
		for (size_t i = 0; i < loop.size(); ++i) l += Distance(loop[i], loop[(i + 1) % loop.size()]);
	}
	return l;
}

PlaneSection SectionByPlane(const TriMesh& mesh, const Plane& p) {
	PlaneSection out;
	const size_t nv = mesh.vertices.size();
	std::vector<double> dist(nv);
	for (size_t i = 0; i < nv; ++i) dist[i] = p.Distance(mesh.vertices[i]);
	// A vertex exactly on the plane is taken as positive (simulation of
	// simplicity), so no crossing point ever coincides with a vertex.
	auto positive = [&](int i) { return dist[i] >= 0.0; };

	// The crossing point of an edge, computed once per edge and shared. A
	// crossing that lands exactly on a vertex (the vertex is on the plane)
	// is keyed by that vertex instead, so every edge into it agrees.
	using EdgeKey = std::pair<int, int>;
	std::map<EdgeKey, int> point_of_edge;
	std::map<int, int> point_of_vertex;
	std::vector<Vec3> points;
	auto point_index = [&](int a, int b) {
		const EdgeKey key{std::min(a, b), std::max(a, b)};
		const int lo = key.first, hi = key.second;
		for (int v : {lo, hi}) {
			if (dist[v] != 0.0) continue;
			auto it = point_of_vertex.find(v);
			if (it != point_of_vertex.end()) return it->second;
			points.push_back(mesh.vertices[v]);
			const int idx = static_cast<int>(points.size()) - 1;
			point_of_vertex.emplace(v, idx);
			return idx;
		}
		auto it = point_of_edge.find(key);
		if (it != point_of_edge.end()) return it->second;
		// Always interpolate from the lower index so both triangles agree
		// to the bit.
		const double t = dist[lo] / (dist[lo] - dist[hi]);
		points.push_back(Lerp(mesh.vertices[lo], mesh.vertices[hi], t));
		const int idx = static_cast<int>(points.size()) - 1;
		point_of_edge.emplace(key, idx);
		return idx;
	};

	// One directed segment per crossing triangle, from the crossing edge it
	// leaves to the one it enters, oriented along n x m (m the triangle's
	// outward normal): counter-clockwise about n with material on the
	// left, so an outer loop runs counter-clockwise and a hole clockwise.
	std::vector<std::array<int, 2>> segments;
	for (size_t t = 0; t < mesh.triangles.size(); ++t) {
		const auto& tri = mesh.triangles[t];
		const bool s0 = positive(tri[0]), s1 = positive(tri[1]), s2 = positive(tri[2]);
		if (s0 == s1 && s1 == s2) continue;
		int pts[2];
		int n = 0;
		for (int e = 0; e < 3; ++e) {
			const int a = tri[e], b = tri[(e + 1) % 3];
			if (positive(a) != positive(b)) pts[n++] = point_index(a, b);
		}
		// Both crossings on one on-plane vertex: the triangle only touches
		// the plane there and contributes no length.
		if (n != 2 || pts[0] == pts[1]) continue;
		const Vec3 dir = Cross(p.normal, mesh.TriangleNormal(static_cast<int>(t)));
		const Vec3 ab = points[pts[1]] - points[pts[0]];
		if (Dot(ab, dir) >= 0.0) segments.push_back({pts[0], pts[1]});
		else segments.push_back({pts[1], pts[0]});
	}

	// Chain: every point starts exactly one segment on a closed manifold.
	std::vector<int> next_of_point(points.size(), -1);
	for (size_t i = 0; i < segments.size(); ++i) next_of_point[segments[i][0]] = static_cast<int>(i);
	std::vector<bool> used(segments.size(), false);
	Vec3 u, v;
	PlaneBasis(p.normal, &u, &v);
	for (size_t start = 0; start < segments.size(); ++start) {
		if (used[start]) continue;
		std::vector<Vec3> loop;
		int seg = static_cast<int>(start);
		while (seg >= 0 && !used[seg]) {
			used[seg] = true;
			loop.push_back(points[segments[seg][0]]);
			seg = next_of_point[segments[seg][1]];
		}
		if (loop.size() < 3) continue;
		double area = 0.0;
		for (size_t i = 0; i < loop.size(); ++i) {
			const Vec3 a = loop[i], b = loop[(i + 1) % loop.size()];
			area += Dot(a, u) * Dot(b, v) - Dot(b, u) * Dot(a, v);
		}
		area *= 0.5;
		out.loops.push_back(std::move(loop));
		out.areas.push_back(area);
		out.outer.push_back(area > 0.0);
	}
	return out;
}

} // namespace aerotelic
