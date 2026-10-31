#pragma once

// Post-processing of a panel solution (panel_method.hpp): the loads strip
// by strip along a lifting network's span, and the elliptic distribution
// with the same total to plot against them.

#include <vector>

#include "aerotelic/panel_method.hpp"

namespace aerotelic::panel {

// One strip of a network: the panels of one column (or one row), the
// sections a wing is paneled in. Forces are the pressure integrated over
// the strip's panels exactly as the solver sums them for a closed
// network -- -Cp n dA, the panel's pressure on its outward normal over
// its area -- divided by the dynamic pressure, so an area, NOT by the
// reference area.
struct StripLoad {
	double s = 0.0;               // the strip centre along the span axis (area-weighted)
	double s_min = 0.0, s_max = 0.0; // its extent along the span axis, from its corners
	// The spacing of its two stations (each the mean of its corners along
	// the axis): the span the strip's load is per unit of. Narrower than
	// s_max - s_min where a station is tilted by dihedral.
	double width = 0.0;
	double area = 0.0;            // the strip's panel area
	// The local chord: the extent of the strip's corners along the chord
	// axis at each of its two stations, averaged (so a swept strip is not
	// over-measured). The section lift coefficient is force · lift / (q
	// width chord).
	double chord = 0.0;
	Vec3 force;                   // ∫ over the strip, / q
};

struct StripOptions {
	Vec3 span_axis{0.0, 1.0, 0.0}; // unit; the strips are ordered along it
	Vec3 chord_axis{1.0, 0.0, 0.0}; // unit; the chord is measured along it
	// A wing is paneled with the span along the columns (a strip is a
	// column of panels round the section, WingNetworks); a network built
	// the other way has a strip per row.
	bool strips_are_columns = true;
};

// The strips of `res` (which the solver computed from `net`), in increasing
// s. Panels of zero area (a collapsed tip row) contribute nothing.
std::vector<StripLoad> StripLoads(const Network& net, const NetworkResult& res, const StripOptions& options);

// One network's panels as a source of load for the total distribution of
// a configuration, reflected in y = 0 for the image of a network solved
// with the symmetry plane.
struct LoadSource {
	const Network* net = nullptr;
	const NetworkResult* res = nullptr;
	bool reflect_y = false;
};

// The load of every source summed into bins along `axis` (edges: n + 1
// increasing values, n forces returned, / q as the strips are). Each
// panel's force is spread over the bins its corners' extent along the axis
// overlaps, in proportion to the overlap, so a body paneled coarsely
// across the axis (a fuselage's ring of twelve) still gives a smooth and
// conservative distribution; a panel of no extent goes to the bin its
// centre falls in. Load outside the edges is dropped.
std::vector<Vec3> BinnedLoads(const std::vector<LoadSource>& sources, const Vec3& axis, const std::vector<double>& edges);

// A network cut by the plane Dot(P, axis) = s: the section's closed
// polyline (one segment per panel the plane crosses, its ends on the
// panel's two edges that run along the axis), the pressure on each
// segment interpolated along the axis between the panel's centre and its
// neighbour's, and the force per unit length along the axis
//   dF/ds / q = ∮ -Cp (t × axis) dl       (t the unit tangent of the cut)
// which is the force the pressure puts on the surface per unit of axis
// length: for a wing cut normal to its span, the lift and drag per unit
// span exactly, however the surface is swept or tapered (sweep and taper
// change only the component along the axis). No strip width enters, so
// a cut at a pointed tip, where the chord vanishes, gives zero.
struct SectionCut {
	bool found = false;         // the plane crosses the network
	Vec3 force_per_span;        // ∮ -Cp (t × axis) dl, / q: a length
	double length = 0.0;        // the cut's arc length
	std::vector<Vec3> points;   // the segment midpoints, in panel order
	std::vector<double> cp;     // the pressure on each segment
};

// `span_along_columns`: the axis runs along the network's columns (a wing
// from WingNetworks), else along its rows. A plane exactly on a station
// is taken once (half-open intervals), the far end of the network
// included.
SectionCut CutSection(const Network& net, const NetworkResult& res, const Vec3& axis, double s, bool span_along_columns = true);

// The load per unit span of an elliptic distribution over [s0 - half_span,
// s0 + half_span] whose integral is `total`:
//   l(s) = 4 total / (π b) sqrt(1 - ((s - s0) / half_span)²),  b = 2 half_span,
// zero outside. `total` and the result carry whatever units the caller's
// loads do.
double EllipticLoad(double total, double s0, double half_span, double s);

} // namespace aerotelic::panel
