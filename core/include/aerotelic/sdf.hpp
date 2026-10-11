#pragma once

// Signed distance to a closed triangle mesh, accelerated by a bounding
// volume hierarchy.
//
// The distance from a point to a mesh is the minimum over its triangles
// of the point-triangle distance. Evaluating every triangle for every
// query is O(triangles) per query -- hopeless for the millions of queries
// the lattice generator makes against a mesh of tens of thousands of
// triangles. The BVH is a binary tree of axis-aligned boxes over the
// triangles; a query descends it nearest-box-first and never enters a box
// farther away than the best distance found so far, so a query touches
// O(log triangles) boxes and a handful of triangles.
//
// The sign comes from the angle-weighted pseudo-normal of whatever
// feature (face, edge or vertex) the closest point lies on (Baerentzen &
// Aanaes, "Signed distance computation using the angle weighted
// pseudonormal", 2005): for a closed, consistently oriented mesh the
// pseudo-normal of a vertex or edge is guaranteed to classify a point
// correctly, where a plain face normal would be ambiguous at a crease.
// Negative is inside.

#include <cstddef>
#include <vector>

#include "aerotelic/mesh.hpp"
#include "aerotelic/vec3.hpp"

namespace aerotelic {

class MeshSdf {
public:
	struct Closest {
		double distance = 0.0;  // unsigned
		Vec3 point;             // on the mesh
		int triangle = -1;
	};

	// Copies the mesh. The sign is only meaningful for a closed, outward-
	// oriented mesh (TriMesh::IsClosedManifold(), TriMesh::FlipIfInsideOut()).
	// Throws std::invalid_argument on an empty mesh.
	explicit MeshSdf(const TriMesh& mesh);

	// Closest point via the BVH.
	Closest ClosestPoint(const Vec3& p) const;
	// Signed distance, negative inside.
	double Distance(const Vec3& p) const;
	bool Inside(const Vec3& p) const { return Distance(p) < 0.0; }

	// Every triangle, no BVH: the reference the BVH is checked against.
	Closest ClosestPointBruteForce(const Vec3& p) const;

	const Box3& bounds() const { return bounds_; }
	size_t n_triangles() const { return tris_.size(); }
	size_t n_nodes() const { return nodes_.size(); }

private:
	struct Node {
		Box3 box;
		int left = -1, right = -1; // children (inner node)
		int first = 0, count = 0;  // triangle range (leaf)
		bool IsLeaf() const { return count > 0; }
	};

	std::vector<Vec3> verts_;
	std::vector<std::array<int, 3>> tris_;
	std::vector<Vec3> face_normals_;
	std::vector<Vec3> vertex_normals_;      // angle-weighted
	std::vector<Vec3> edge_normals_;        // 3 per triangle: edge k = (v[k], v[k+1])
	std::vector<Node> nodes_;
	std::vector<int> order_;                // triangle indices, leaf ranges are contiguous here
	Box3 bounds_;

	int Build(std::vector<Vec3>& centroids, int begin, int end);
	void ComputePseudoNormals();
	// Closest point on triangle t to p, with which feature it lies on:
	// 0-2 vertex k, 3-5 edge k-3, 6 the face.
	Vec3 ClosestOnTriangle(int t, const Vec3& p, int& feature) const;
	Vec3 PseudoNormal(int t, int feature) const;
};

} // namespace aerotelic
