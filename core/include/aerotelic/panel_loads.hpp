#pragma once

// Post-processing of a panel solution (panel_method.hpp): the loads strip
// by strip along a lifting network's span, and the elliptic distribution
// with the same total to plot against them.

#include <vector>

#include "aerotelic/panel_method.hpp"

namespace aerotelic::panel {

// One strip of a network: the panels of one column (or one row), the
// sections a wing is paneled in. Forces are the pressure integrated over
// the strip's panels exactly as the solver sums them for the network --
// -Cp n dA on a one-sided (closed) network, (Cp_lower - Cp_upper) n dA on
// a sheet -- divided by the dynamic pressure, so an area, NOT by the
// reference area.
struct StripLoad {
	double s = 0.0;               // the strip centre along the span axis (area-weighted)
	double s_min = 0.0, s_max = 0.0; // its extent along the span axis, from its corners
	double width = 0.0;           // s_max - s_min
	double area = 0.0;            // the strip's panel area
	Vec3 force;                   // ∫ over the strip, / q
};

struct StripOptions {
	Vec3 span_axis{0.0, 1.0, 0.0}; // unit; the strips are ordered along it
	// A thick wing is paneled with the span along the columns (a strip is a
	// column of panels round the section); the mean camber surface the
	// other way (a strip is a row). See wing_panels.cpp.
	bool strips_are_columns = true;
	// A sheet with both sides wetted (the mean camber surface): the load is
	// the difference of the two pressures.
	bool two_sided = false;
};

// The strips of `res` (which the solver computed from `net`), in increasing
// s. Panels of zero area (a collapsed tip row) contribute nothing.
std::vector<StripLoad> StripLoads(const Network& net, const NetworkResult& res, const StripOptions& options);

// The load per unit span of an elliptic distribution over [s0 - half_span,
// s0 + half_span] whose integral is `total`:
//   l(s) = 4 total / (π b) sqrt(1 - ((s - s0) / half_span)²),  b = 2 half_span,
// zero outside. `total` and the result carry whatever units the caller's
// loads do.
double EllipticLoad(double total, double s0, double half_span, double s);

} // namespace aerotelic::panel
