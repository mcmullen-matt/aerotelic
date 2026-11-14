#pragma once

// The surface pressure of a panel solution carried onto another surface
// mesh (stage 4 of the flight-loads plan, 2026-11-14): the structural
// shells are a body's own triangles, thousands of them, while the panel
// method knows the pressure at a few hundred panel centres on a coarser
// structured grid of the same surface. Each face takes the pressure of
// the panels nearest its centroid ON ITS OWN SIDE of the surface -- a
// thin wing's upper and lower skins are a few per cent of the chord
// apart, closer than the panels are to one another, so distance alone
// would hand an upper panel's suction to a lower-surface face near the
// trailing edge; the panel's outward normal against the face's decides
// the side. A two-sided sheet (a cowl's mean surface, both sides wetted)
// gives its upper pressure to faces that look its way and its lower
// pressure to the rest.
//
// The value is the inverse-distance mean of the nearest acceptable panel
// and its grid neighbours (the panels one row and one column away in the
// same network, on the same side), so the field is continuous across the
// panel grid rather than a staircase; nothing is interpolated across
// networks. A face farther than `max_distance` from every acceptable
// panel (a body no panel describes) gets nothing and is counted.

#include <vector>

#include "aerotelic/vec3.hpp"

namespace aerotelic::aero {

// One panel of a solution, as placed in the world.
struct PressureSample {
	Vec3 centre, normal;     // the control point and the outward (upper-side) unit normal
	double area = 0.0;
	double cp_upper = 0.0;   // on the normal's side
	double cp_lower = 0.0;   // on the other side (meaningful for a two-sided sheet only)
	bool two_sided = false;  // a sheet with both sides wetted
	int network = 0;         // the grid it belongs to
	int row = 0, col = 0;    // its place in that grid (neighbours are +/-1)
};

struct MapOptions {
	// A panel is on a face's side when its normal makes at least this
	// cosine with the face's (0: the same hemisphere).
	double normal_min = 0.0;
	// Faces farther than this from every acceptable panel centre get no
	// pressure (0: a tenth of the samples' extent).
	double max_distance = 0.0;
	// Grid neighbours taken either way (1: the 3 x 3 block).
	int neighbourhood = 1;
};

struct MapStats {
	int faces = 0, mapped = 0, unmapped = 0;
	double max_distance = 0.0;   // the farthest mapped face from its panel
	double mean_distance = 0.0;
};

// The pressure coefficient per face (centres with outward unit normals),
// 0 where none applies; `mapped`, when given, says which faces got one.
std::vector<double> MapPressures(const std::vector<PressureSample>& samples, const std::vector<Vec3>& face_centres, const std::vector<Vec3>& face_normals,
	const MapOptions& options = {}, MapStats* stats = nullptr, std::vector<unsigned char>* mapped = nullptr);

} // namespace aerotelic::aero
