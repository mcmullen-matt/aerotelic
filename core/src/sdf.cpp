#include "aerotelic/sdf.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace aerotelic {

namespace {

constexpr int kLeafSize = 4;

uint64_t EdgeKey(int a, int b) {
	uint32_t lo = static_cast<uint32_t>(std::min(a, b)), hi = static_cast<uint32_t>(std::max(a, b));
	return (static_cast<uint64_t>(lo) << 32) | hi;
}

} // namespace

MeshSdf::MeshSdf(const TriMesh& mesh) : verts_(mesh.vertices), tris_(mesh.triangles) {
	if (tris_.empty()) throw std::invalid_argument("MeshSdf: empty mesh");
	for (const auto& v : verts_) bounds_.Add(v);
	ComputePseudoNormals();

	std::vector<Vec3> centroids(tris_.size());
	order_.resize(tris_.size());
	for (size_t t = 0; t < tris_.size(); ++t) {
		order_[t] = static_cast<int>(t);
		centroids[t] = (verts_[tris_[t][0]] + verts_[tris_[t][1]] + verts_[tris_[t][2]]) / 3.0;
	}
	nodes_.reserve(2 * tris_.size() / kLeafSize + 2);
	Build(centroids, 0, static_cast<int>(tris_.size()));
}

int MeshSdf::Build(std::vector<Vec3>& centroids, int begin, int end) {
	Node node;
	for (int i = begin; i < end; ++i) {
		const auto& t = tris_[order_[i]];
		node.box.Add(verts_[t[0]]);
		node.box.Add(verts_[t[1]]);
		node.box.Add(verts_[t[2]]);
	}
	int index = static_cast<int>(nodes_.size());
	nodes_.push_back(node);
	if (end - begin <= kLeafSize) {
		nodes_[index].first = begin;
		nodes_[index].count = end - begin;
		return index;
	}
	// Split at the median centroid along the longest axis of the centroid
	// bounds (not the triangle bounds: a few big triangles would otherwise
	// dominate the choice).
	Box3 cb;
	for (int i = begin; i < end; ++i) cb.Add(centroids[order_[i]]);
	Vec3 ext = cb.Extent();
	int axis = 0;
	if (ext.y > ext[axis]) axis = 1;
	if (ext.z > ext[axis]) axis = 2;
	int mid = (begin + end) / 2;
	std::nth_element(order_.begin() + begin, order_.begin() + mid, order_.begin() + end,
		[&](int a, int b) { return centroids[a][axis] < centroids[b][axis]; });
	int left = Build(centroids, begin, mid);
	int right = Build(centroids, mid, end);
	nodes_[index].left = left;
	nodes_[index].right = right;
	return index;
}

void MeshSdf::ComputePseudoNormals() {
	const size_t nt = tris_.size();
	face_normals_.assign(nt, Vec3{});
	vertex_normals_.assign(verts_.size(), Vec3{});
	edge_normals_.assign(3 * nt, Vec3{});
	std::unordered_map<uint64_t, Vec3> edge_sum;
	edge_sum.reserve(3 * nt);
	for (size_t t = 0; t < nt; ++t) {
		const auto& tri = tris_[t];
		const Vec3& a = verts_[tri[0]];
		const Vec3& b = verts_[tri[1]];
		const Vec3& c = verts_[tri[2]];
		Vec3 n = Cross(b - a, c - a);
		double len = Length(n);
		if (len > 0.0) n = n / len;
		face_normals_[t] = n;
		// Angle-weighted contribution to each vertex.
		const Vec3* p[3] = {&a, &b, &c};
		for (int k = 0; k < 3; ++k) {
			Vec3 e1 = Normalized(*p[(k + 1) % 3] - *p[k]);
			Vec3 e2 = Normalized(*p[(k + 2) % 3] - *p[k]);
			double angle = std::acos(std::clamp(Dot(e1, e2), -1.0, 1.0));
			vertex_normals_[tri[k]] += n * angle;
			edge_sum[EdgeKey(tri[k], tri[(k + 1) % 3])] += n;
		}
	}
	for (auto& v : vertex_normals_) v = Normalized(v);
	for (size_t t = 0; t < nt; ++t) {
		for (int k = 0; k < 3; ++k) {
			edge_normals_[3 * t + k] = Normalized(edge_sum[EdgeKey(tris_[t][k], tris_[t][(k + 1) % 3])]);
		}
	}
}

Vec3 MeshSdf::ClosestOnTriangle(int t, const Vec3& p, int& feature) const {
	// Ericson, Real-Time Collision Detection, 5.1.5, with the Voronoi
	// region recorded as the feature.
	const auto& tri = tris_[t];
	const Vec3& a = verts_[tri[0]];
	const Vec3& b = verts_[tri[1]];
	const Vec3& c = verts_[tri[2]];
	Vec3 ab = b - a, ac = c - a, ap = p - a;
	double d1 = Dot(ab, ap), d2 = Dot(ac, ap);
	if (d1 <= 0.0 && d2 <= 0.0) { feature = 0; return a; }
	Vec3 bp = p - b;
	double d3 = Dot(ab, bp), d4 = Dot(ac, bp);
	if (d3 >= 0.0 && d4 <= d3) { feature = 1; return b; }
	double vc = d1 * d4 - d3 * d2;
	if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
		feature = 3; // edge ab
		double v = d1 / (d1 - d3);
		return a + ab * v;
	}
	Vec3 cp = p - c;
	double d5 = Dot(ab, cp), d6 = Dot(ac, cp);
	if (d6 >= 0.0 && d5 <= d6) { feature = 2; return c; }
	double vb = d5 * d2 - d1 * d6;
	if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
		feature = 5; // edge ca
		double w = d2 / (d2 - d6);
		return a + ac * w;
	}
	double va = d3 * d6 - d5 * d4;
	if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
		feature = 4; // edge bc
		double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		return b + (c - b) * w;
	}
	feature = 6;
	double denom = 1.0 / (va + vb + vc);
	double v = vb * denom, w = vc * denom;
	return a + ab * v + ac * w;
}

Vec3 MeshSdf::PseudoNormal(int t, int feature) const {
	if (feature < 3) return vertex_normals_[tris_[t][feature]];
	if (feature < 6) return edge_normals_[3 * t + (feature - 3)];
	return face_normals_[t];
}

MeshSdf::Closest MeshSdf::ClosestPoint(const Vec3& p) const {
	Closest best;
	double best2 = std::numeric_limits<double>::infinity();
	// Explicit stack of (node, squared box distance), nearest child pushed
	// last so it is visited first.
	struct Item { int node; double d2; };
	Item stack[128];
	int sp = 0;
	stack[sp++] = {0, nodes_[0].box.DistanceSq(p)};
	while (sp > 0) {
		Item it = stack[--sp];
		if (it.d2 >= best2) continue;
		const Node& node = nodes_[it.node];
		if (node.IsLeaf()) {
			for (int i = node.first; i < node.first + node.count; ++i) {
				int t = order_[i];
				int feature;
				Vec3 q = ClosestOnTriangle(t, p, feature);
				double d2 = DistanceSq(p, q);
				if (d2 < best2) {
					best2 = d2;
					best.point = q;
					best.triangle = t;
				}
			}
			continue;
		}
		double dl = nodes_[node.left].box.DistanceSq(p);
		double dr = nodes_[node.right].box.DistanceSq(p);
		if (dl < dr) {
			if (dr < best2) stack[sp++] = {node.right, dr};
			if (dl < best2) stack[sp++] = {node.left, dl};
		} else {
			if (dl < best2) stack[sp++] = {node.left, dl};
			if (dr < best2) stack[sp++] = {node.right, dr};
		}
	}
	best.distance = std::sqrt(best2);
	return best;
}

double MeshSdf::Distance(const Vec3& p) const {
	Closest c = ClosestPoint(p);
	if (c.triangle < 0) return c.distance;
	int feature;
	Vec3 q = ClosestOnTriangle(c.triangle, p, feature);
	Vec3 n = PseudoNormal(c.triangle, feature);
	double side = Dot(p - q, n);
	return side < 0.0 ? -c.distance : c.distance;
}

MeshSdf::Closest MeshSdf::ClosestPointBruteForce(const Vec3& p) const {
	Closest best;
	double best2 = std::numeric_limits<double>::infinity();
	for (size_t t = 0; t < tris_.size(); ++t) {
		int feature;
		Vec3 q = ClosestOnTriangle(static_cast<int>(t), p, feature);
		double d2 = DistanceSq(p, q);
		if (d2 < best2) {
			best2 = d2;
			best.point = q;
			best.triangle = static_cast<int>(t);
		}
	}
	best.distance = std::sqrt(best2);
	return best;
}

} // namespace aerotelic
