#include "aerotelic/body_structure.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>

namespace aerotelic {

const char* PlaneAxisName(PlaneAxis a) {
	switch (a) {
		case PlaneAxis::X: return "x";
		case PlaneAxis::Y: return "y";
		case PlaneAxis::Z: return "z";
		case PlaneAxis::Custom: return "custom";
	}
	return "y";
}

PlaneAxis ParsePlaneAxis(const std::string& s) {
	std::string t;
	for (char c : s) t.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
	if (t == "x") return PlaneAxis::X;
	if (t == "y") return PlaneAxis::Y;
	if (t == "z") return PlaneAxis::Z;
	if (t == "custom") return PlaneAxis::Custom;
	throw std::invalid_argument("unknown plane axis '" + s + "' (x, y, z or custom)");
}

namespace {

bool SameNetworkGeometry(const PlaneNetwork& a, const PlaneNetwork& b) {
	const Vec3 na = NetworkNormal(a), nb = NetworkNormal(b);
	return na.x == nb.x && na.y == nb.y && na.z == nb.z && a.start == b.start && a.spacing == b.spacing
		&& a.count == b.count && a.thickness == b.thickness && a.height == b.height;
}

} // namespace

bool SameStructureGeometry(const BodyStructure& a, const BodyStructure& b) {
	if (a.skin_thickness != b.skin_thickness) return false;
	if (a.networks.size() != b.networks.size()) return false;
	for (size_t i = 0; i < a.networks.size(); ++i) {
		if (!SameNetworkGeometry(a.networks[i], b.networks[i])) return false;
	}
	return a.exclusions == b.exclusions;
}

std::string PlaneNetworkProblem(const PlaneNetwork& n) {
	if (n.count < 1) return "it has no planes (the count must be at least 1)";
	if (!(n.thickness > 0.0) || !std::isfinite(n.thickness)) return "its thickness must be greater than zero";
	if (n.count > 1) {
		if (!(n.spacing > 0.0) || !std::isfinite(n.spacing)) return "its spacing must be greater than zero when it has more than one plane";
		if (n.spacing <= n.thickness) return "its planes are thicker than their spacing, so they would overlap";
	}
	if (n.height < 0.0 || !std::isfinite(n.height)) return "its height cannot be negative (0 spans the interior)";
	if (!std::isfinite(n.start)) return "its start is not a number";
	if (n.axis == PlaneAxis::Custom && !(Length(n.normal) > 0.0)) return "its custom normal is the zero vector";
	return std::string();
}

std::string BodyStructureProblem(const BodyStructure& s) {
	if (s.skin_thickness < 0.0 || !std::isfinite(s.skin_thickness)) return "the skin thickness cannot be negative";
	if (s.asserted_mass && (!(*s.asserted_mass >= 0.0) || !std::isfinite(*s.asserted_mass))) return "the asserted mass cannot be negative";
	if (!s.networks.empty() && !(s.skin_thickness > 0.0)) {
		return "a plane network needs a skin thickness: a solid body has no interior to fill";
	}
	for (const PlaneNetwork& n : s.networks) {
		const std::string why = PlaneNetworkProblem(n);
		if (!why.empty()) return "network '" + n.name + "': " + why;
	}
	return std::string();
}

// ---- plane geometry -------------------------------------------------------

Vec3 NetworkNormal(const PlaneNetwork& n) {
	switch (n.axis) {
		case PlaneAxis::X: return {1.0, 0.0, 0.0};
		case PlaneAxis::Y: return {0.0, 1.0, 0.0};
		case PlaneAxis::Z: return {0.0, 0.0, 1.0};
		case PlaneAxis::Custom: break;
	}
	const Vec3 u = Normalized(n.normal);
	return Length(u) > 0.0 ? u : Vec3{0.0, 1.0, 0.0};
}

double PlaneOffset(const PlaneNetwork& n, int k) {
	return n.start + k * n.spacing;
}

Plane NetworkPlane(const PlaneNetwork& n, int k) {
	Plane p;
	p.normal = NetworkNormal(n);
	p.offset = PlaneOffset(n, k);
	return p;
}

Plane PlacedPlane(const Plane& local, const std::array<double, 9>& R, const Vec3& T, double scale) {
	const Vec3 n = local.normal;
	Plane out;
	out.normal = Vec3{R[0] * n.x + R[1] * n.y + R[2] * n.z,
	                  R[3] * n.x + R[4] * n.y + R[5] * n.z,
	                  R[6] * n.x + R[7] * n.y + R[8] * n.z};
	// A rotation keeps the length; renormalise anyway against a scaled R.
	out.normal = Normalized(out.normal);
	out.offset = scale * local.offset + Dot(out.normal, T);
	return out;
}

Plane MirrorPlaneY(const Plane& p) {
	Plane out = p;
	out.normal.y = -out.normal.y;
	return out;
}

double PlaneNetworkField(const PlaneNetwork& n, const Vec3& p) {
	const double s = Dot(NetworkNormal(n), p);
	int k = 0;
	if (n.count > 1 && n.spacing > 0.0) {
		k = static_cast<int>(std::lround((s - n.start) / n.spacing));
		k = std::max(0, std::min(n.count - 1, k));
	}
	return std::fabs(s - PlaneOffset(n, k)) - 0.5 * n.thickness;
}

void PlaneBasis(const Vec3& n, Vec3* u, Vec3* v) {
	// Cross with the axis the normal leans on least, so the basis is well
	// conditioned whatever the normal.
	const double ax = std::fabs(n.x), ay = std::fabs(n.y), az = std::fabs(n.z);
	Vec3 a = ax <= ay && ax <= az ? Vec3{1.0, 0.0, 0.0} : (ay <= az ? Vec3{0.0, 1.0, 0.0} : Vec3{0.0, 0.0, 1.0});
	*u = Normalized(Cross(n, a));
	*v = Cross(n, *u);   // u x v = u x (n x u) = n (u.u) - u (u.n) = n
}

TriMesh SlabMesh(const Plane& p, double thickness, const Vec3& centre, double half_extent) {
	Vec3 u, v;
	PlaneBasis(p.normal, &u, &v);
	TriMesh slab = BoxMesh(2.0 * half_extent, 2.0 * half_extent, thickness);
	// Columns u, v, n: x' = u x + v y + n z, a rotation (det +1), so the
	// winding stays outward.
	const std::array<double, 9> R{u.x, v.x, p.normal.x,
	                              u.y, v.y, p.normal.y,
	                              u.z, v.z, p.normal.z};
	const Vec3 on_plane = centre - p.normal * p.Distance(centre);
	slab.Transform(R, on_plane);
	return slab;
}

bool PlaneBoxRectangle(const Plane& p, const Box3& box, std::array<Vec3, 4>* corners) {
	if (box.IsEmpty()) return false;
	bool above = false, below = false;
	Vec3 u, v;
	PlaneBasis(p.normal, &u, &v);
	const Vec3 c0 = box.Center() - p.normal * p.Distance(box.Center());
	double umin = 1e300, umax = -1e300, vmin = 1e300, vmax = -1e300;
	for (int i = 0; i < 8; ++i) {
		const Vec3 corner{(i & 1) ? box.max.x : box.min.x, (i & 2) ? box.max.y : box.min.y, (i & 4) ? box.max.z : box.min.z};
		const double d = p.Distance(corner);
		above = above || d > 0.0;
		below = below || d < 0.0;
		const Vec3 q = corner - p.normal * d - c0;
		const double qu = Dot(q, u), qv = Dot(q, v);
		umin = std::min(umin, qu); umax = std::max(umax, qu);
		vmin = std::min(vmin, qv); vmax = std::max(vmax, qv);
	}
	if (!(above && below)) return false;
	if (corners) {
		(*corners)[0] = c0 + u * umin + v * vmin;
		(*corners)[1] = c0 + u * umax + v * vmin;
		(*corners)[2] = c0 + u * umax + v * vmax;
		(*corners)[3] = c0 + u * umin + v * vmax;
	}
	return true;
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
