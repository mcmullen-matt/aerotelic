#pragma once

// An explicit solid skin: the body's surface triangulation offset inward
// by a thickness, as a layer of prisms (each split into tetrahedra) that
// is conformal to the surface mesh -- the shell a finite-element model
// gives its own material, with the inner surface left as the boundary of
// the interior (lattice) domain. Nothing here is sampled on a grid: every
// offset node is placed on the body's signed-distance field, so a skin of
// any thickness is exact and costs no voxels (the sampled skin of
// lattice.hpp needs the voxel finer than half the thickness, which an
// airframe-sized domain cannot afford).
//
// Placing an offset node. From each surface vertex the generator walks
// inward along the vertex's (angle-weighted) normal, evaluating the body's
// signed distance b at each step, and stops at the first of:
//   * b reaches -thickness: the node is on the true offset surface;
//   * b stops decreasing: the walk has reached the mid-surface between
//     this face and the opposite one (the body is thinner than two skins
//     here); the node stays on the mid-surface and the vertex is marked
//     COLLAPSED -- the skin fills the whole thickness, which is what a
//     trailing edge should be;
//   * b would become non-negative (a bad normal at a crease): the walk
//     ends where it is.
// The walk only ever moves to points with a more negative field, so a
// node can never land in the opposite skin nor outside the body, whatever
// the normal does at a grazing trailing edge: the field, not a ray hit,
// decides. The result is then polished onto the iso-surface by bisection.
//
// Keeping the layer valid. The walk distances are then limited so they
// cannot change faster than `lipschitz` times the edge length between
// neighbouring vertices (a long triangle with one vertex at full depth and
// another on a thin mid-surface would otherwise fold its prism); offset
// nodes that coincide are welded; each prism is split into three
// tetrahedra by the lowest-index rule (Dompierre et al.), so neighbouring
// prisms split their shared quad face the same way and the tetrahedra are
// conformal; a prism with a collapsed edge degenerates into fewer
// tetrahedra; every tetrahedron's volume is checked.
//
// The inner surface. The offset nodes with the surface's own connectivity
// form a closed surface (the offset copy of the surface), whose enclosed
// volume is the interior domain. Where the body is thinner than two skins
// the copy's two sides both lie on the mid-surface as a zero-volume double
// sheet; those triangles are flagged `collapsed`. Where the two sides'
// nodes coincide there (a symmetric section, whose mirrored vertices meet
// at the same mid-surface points), the coincident opposite triangles are
// cancelled and the remaining surface is sealed along the pinch by the
// shared nodes. Where they do not coincide (an asymmetric section), the
// double sheet stays, flagged, and a tetrahedral mesher of the interior
// should drop it and treat the body as solid skin there -- the two sides'
// triangulations of the sheet then do not share nodes, which is the one
// place the skin layer is not node-conformal across itself.

#include <array>
#include <string>
#include <vector>

#include "aerotelic/mesh.hpp"
#include "aerotelic/sdf.hpp"
#include "aerotelic/vec3.hpp"

namespace aerotelic {

struct ShellOptions {
	// Skin thickness, model units.
	double thickness = 0.02;
	// Bound on how fast the walk distance may change along a surface edge
	// (fraction of the edge length): the slope of the offset layer's inner
	// face. A steep inner face is a tilted prism, not a folded one (folds
	// come from sideways displacement, caught by the twist relaxation), so
	// the bound is loose; it only stops a lone deep walk beside shallow
	// neighbours.
	double lipschitz = 2.0;
	// Walk step as a fraction of the thickness.
	double step_fraction = 0.1;
	// A vertex whose walk ends shallower than this fraction of the
	// thickness is collapsed (on the mid-surface).
	double collapse_fraction = 0.98;
	// Offset nodes closer than this fraction of the thickness are welded
	// (the two sides of a thin region meeting on its mid-surface).
	double weld_fraction = 1e-2;
};

struct ShellResult {
	// All nodes: the surface vertices first (indices equal to the input
	// mesh's), then the offset nodes.
	std::vector<Vec3> nodes;
	int n_surface_nodes = 0;
	// Per surface vertex: its offset node (an index into `nodes`; equal to
	// the vertex itself when the walk went nowhere), the walk distance, the
	// field value at the node (negative inside), and whether it collapsed
	// onto the mid-surface.
	std::vector<int> offset_node;
	std::vector<double> walk_distance;
	std::vector<double> walk_unlimited; // the walk before the edge-slope bound and relaxation
	std::vector<double> depth;
	std::vector<bool> collapsed;
	// The skin as tetrahedra (indices into `nodes`), positively oriented.
	std::vector<std::array<int, 4>> tets;
	// Per tetrahedron: the surface triangle it came from.
	std::vector<int> tet_triangle;
	// The inner surface: one triangle per surface triangle (the offset
	// copy, same winding, so it faces outward as the boundary of the
	// interior domain), indices into `nodes`; -1s mark a triangle that
	// degenerated (a repeated node) and was dropped.
	std::vector<std::array<int, 3>> inner_triangles;
	std::vector<bool> inner_collapsed;

	// Diagnostics.
	int n_collapsed_vertices = 0;
	int n_bad_normals = 0;        // walks that could not enter the body
	int n_lipschitz_limited = 0;  // vertices whose distance the bound cut
	int n_degenerate_tets = 0;    // dropped (repeated node / no volume)
	int n_negative_tets = 0;      // inverted and left in (reported, not hidden)
	int n_cancelled_pairs = 0;    // coincident opposite inner triangles dropped (collapsed double sheet)
	int n_relaxation_rounds = 0;  // rounds of halving the walks of twisted prisms' vertices
	double min_tet_volume = 0.0;
	double skin_volume = 0.0;     // sum of the tetrahedra's volumes

	// The inner surface as a mesh (collapsed triangles included unless
	// `drop_collapsed`; dropping them opens the surface along the pinch).
	TriMesh InnerSurface(bool drop_collapsed = false) const;
};

// Builds the skin of a closed, outward-oriented surface mesh over its
// signed-distance field (MeshSdf of the same mesh). Throws
// std::invalid_argument on an empty mesh or a non-positive thickness.
ShellResult BuildOffsetShell(const TriMesh& surface, const MeshSdf& sdf, const ShellOptions& options);

// Writes the skin tetrahedra as a legacy ASCII VTK unstructured grid
// (cell data "material" = 1 for the skin), for ParaView and the like.
// Returns false if the file could not be written.
bool WriteShellVtk(const ShellResult& shell, const std::string& path);

// Writes a triangle mesh as ASCII STL under the given solid name.
bool WriteStlAscii(const TriMesh& mesh, const std::string& path, const std::string& solid_name = "aerotelic");

} // namespace aerotelic
