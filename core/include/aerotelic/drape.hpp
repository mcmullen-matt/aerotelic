#pragma once

// Kinematic DRAPE of a ply over a surface (2026-11-12): where the fibres
// of a cloth actually lie once it is laid on a doubly curved body, and how
// much it has to shear to get there.
//
// The cloth is a PIN-JOINTED NET: two families of inextensible fibres
// (warp and weft) crossing at pivots a fixed distance L apart. From a seed
// point and the warp direction there, the two generator lines are laid as
// geodesics of the surface, L at a step; every other node is the point of
// the surface at distance L from its two already-placed neighbours
// (Newton's iteration in the tangent plane, re-projected onto the mesh).
// The angle between warp and weft at a node is then whatever the surface
// forces: 90 degrees on a developable surface (a cylinder, a cone, a
// nearly-developable wing skin), less wherever there is Gaussian curvature
// -- the SHEAR ANGLE, which grows as the product of the two arc lengths
// from the generators times the curvature (the Chebyshev net equation,
// d^2 theta / du dv = -K sin theta), and past a cloth's LOCKING ANGLE
// means wrinkles, not drape. Mack & Taylor, "The fitting of woven cloth
// to surfaces", J. Textile Institute 47 (1956) T477; Van Der Weeen,
// "Algorithms for draping fabrics on doubly-curved surfaces", Int. J.
// Numer. Meth. Eng. 31 (1991) 1415; Hancock & Potter, Composites Part A
// 37 (2006) 413 for its validation against hand lay-up.
//
// The net stops at a free edge of the surface (an open mesh), where it
// would wrap round onto itself (a closed body), where a node cannot be
// placed (the cell would fold), and at `max_steps` from the seed. Every
// triangle of the surface takes the warp and weft directions and the
// shear of the nearest cell, projected into its plane; triangles no cell
// reaches are not covered and keep their reference direction with no
// shear.
//
// What the analysis does with it (structural_fea.py): a WOVEN ply's
// stiffness axes are rotated to the bisector of the actual warp and weft
// (the deviation from the reference direction, WarpDeviation) and its
// thickness grows by 1 / cos(shear) (areal weight is conserved as the
// cloth shears); a UNIDIRECTIONAL tape cannot shear in its plane, so its
// fibres follow the warp and the net's shear is reported as the gap or
// overlap the strips would need, with no change to the ply. The
// non-orthogonality of a sheared cloth's stiffness is NOT modelled (a
// documented limit: Hofstee & van Keulen, Composite Structures 54, 2001).

#include <string>
#include <vector>

#include "aerotelic/mesh.hpp"
#include "aerotelic/vec3.hpp"

namespace aerotelic {

struct DrapeOptions {
	// Where the first pivot is put (projected onto the surface) and the
	// warp direction there (projected into the surface's tangent plane).
	Vec3 seed;
	Vec3 warp{1.0, 0.0, 0.0};
	// The net's pitch L. 0: a hundredth of the surface's bounding diagonal.
	double cell = 0.0;
	// The furthest node from the seed along either family.
	int max_steps = 400;
};

struct DrapeResult {
	// Per triangle of the surface: unit warp and weft directions in the
	// triangle's plane (zero when not covered), the shear angle in degrees
	// (how far the angle between them has moved from 90, as a magnitude),
	// and whether a cell reached it.
	std::vector<Vec3> warp, weft;
	std::vector<double> shear;
	std::vector<char> covered;
	// The net: its pitch, nodes and cells, and the nodes themselves (i
	// along the warp, j along the weft, from the seed at 0, 0).
	struct Node {
		int i = 0, j = 0;
		Vec3 p;
	};
	double cell = 0.0;
	int nodes = 0, cells = 0;
	std::vector<Node> net;
	double max_shear = 0.0;        // degrees, over the covered triangles
	double covered_fraction = 0.0; // of the surface's area
	std::vector<std::string> notes;
};

// Drapes the net over `surface` (any triangle mesh; closed or open,
// consistently oriented). Throws std::invalid_argument on an empty mesh.
DrapeResult DrapeNet(const TriMesh& surface, const DrapeOptions& options);

// The angle, degrees, from a reference direction to the draped warp in
// every triangle (counter-clockwise about the triangle's normal, wrapped
// to (-90, 90] since a fibre has no arrow); 0 where not covered.
// `reference` is one vector per triangle (need not be in its plane).
std::vector<double> WarpDeviation(const TriMesh& surface, const DrapeResult& drape, const std::vector<Vec3>& reference);

} // namespace aerotelic
