#pragma once

// Parting an airfoil section into a flap and a main element, worked out in
// the 2-D section plane. No booleans anywhere: each body's outline is
// assembled from pieces of the airfoil's own curve and circular arcs, and
// the loft (mesh.hpp) triangulates those outlines directly.
//
// The cut, at one station (chord-normalised airfoil coordinates, x aft,
// y up, the hinge at x = x_hinge):
//
//                         U'  T_u                   U'  : upper lip (main element skin ends)
//          . - - - - - - - +--o------------------   T_u : upper tangency (flap skin starts)
//        /   main element  (  |   nose  )  flap  \
//       (                 (   +         )         )     + : hinge centre
//        \                 (  |   arc  )         /
//          ` - - - - - - - +--o------------------
//                         L'  T_l
//
//   Flap nose: the circle about the hinge centre tangent to the upper and
//   lower skin (at T_u, T_l). Its radius and the centre's y are solved for;
//   the centre's x is the user's hinge x. The flap is that arc plus the
//   skin aft of the tangency points, closed by the trailing edge.
//
//   Main element: ends on the concentric circle of radius nose + gap (the
//   cavity). That circle is tangent to nothing; it crosses each skin twice,
//   and the main element's skin stops at the FORWARD crossings U', L', the
//   cavity arc between them (bowing forward about the hinge) closing it.
//   Between the lip and the tangency point the gap is open to the air.
//
//   Since nose and cavity are concentric and differ by the gap everywhere,
//   the nose clears the cavity at any deflection. What limits deflection is
//   the flap SKIN: the point a cavity radius from the hinge (the aft
//   crossing) sweeps along the cavity circle and lands on the lip corner --
//   trailing edge down, the lower skin onto L'; up, the upper onto U'. Those
//   two angles are reported, as are the lip corners' included angles (the
//   skin meets the cavity arc at an acute wedge, a knife edge like the
//   trailing edge).

#include <string>
#include <vector>

#include "aerotelic/airfoil.hpp"
#include "aerotelic/mat.hpp"

namespace aerotelic {

struct FlapCut {
	double x_hinge = 0.0, y_hinge = 0.0; // the hinge centre
	double gap = 0.0;                    // r_cavity - r_nose
	double r_nose = 0.0, r_cavity = 0.0;

	// Airfoil curve parameters (lower TE -> LE -> upper TE).
	double t_lower_tangent = 0.0, t_upper_tangent = 0.0; // the flap skin starts here
	double t_lower_lip = 0.0, t_upper_lip = 0.0;         // the main element skin ends here
	// The aft crossings of the cavity circle with the flap skin; NaN when
	// the skin never gets a cavity radius from the hinge (a short flap), in
	// which case the matching deflection limit is infinite.
	double t_lower_aft = 0.0, t_upper_aft = 0.0;

	// Angles about the hinge, radians, atan2 convention (0 aft, pi forward),
	// of the points above. Upper-surface points lie in (0, pi), lower in
	// (-pi, 0).
	double a_lower_tangent = 0.0, a_upper_tangent = 0.0;
	double a_lower_lip = 0.0, a_upper_lip = 0.0;
	double a_lower_aft = 0.0, a_upper_aft = 0.0;

	// Deflection to first contact, degrees: trailing edge down (the lower
	// skin's aft crossing reaches L') and up (the upper's reaches U').
	double deflection_down_limit_deg = 0.0, deflection_up_limit_deg = 0.0;
	// The included angle of the main element's lip corners, degrees.
	double lower_lip_angle_deg = 0.0, upper_lip_angle_deg = 0.0;
};

// Solves the cut for `airfoil` at hinge chord fraction `x_hinge` with the
// chord-normalised clearance `gap`. Throws std::invalid_argument, with the
// reason, when the hinge is off the section, the section has no thickness
// there, the tangent circle cannot be found, or the cavity circle reaches
// the leading edge without crossing the skin (the gap is too wide for the
// section).
FlapCut SolveFlapCut(const Airfoil& airfoil, double x_hinge, double gap);

// A body's section outline, as a piecewise curve over v in [0, 1]: pieces
// of the airfoil's curve (by its own parameter t) and circular arcs about
// the hinge, each monotone in x. The loft samples it as a ring; the piece
// boundaries are the ring's seeds. Fixed v ranges per piece at every
// station (not proportional to length) so the pieces correspond across
// the span and a boundary is a boundary on every ring.
//
//   Full:        the plain airfoil, v = t (lower TE -> LE -> upper TE), the
//                trailing edge closing it (open curve).
//   Flap:        [0, .35] lower skin TE -> T_l; [.35, .65] nose arc T_l ->
//                forward extreme -> T_u; [.65, 1] upper skin T_u -> TE; the
//                trailing edge closing it (open curve).
//   MainElement: [0, .25] lower skin L' -> LE; [.25, .5] upper skin LE ->
//                U'; [.5, 1] cavity arc U' -> forward extreme -> L' (closed:
//                v = 1 is v = 0).
class SectionLoop {
public:
	struct Piece {
		enum class Kind { Skin, Arc };
		Kind kind = Kind::Skin;
		double v0 = 0.0, v1 = 1.0;
		double t0 = 0.0, t1 = 1.0; // Skin: airfoil parameter at v0 and at v1
		double a0 = 0.0, a1 = 0.0; // Arc: angle at v0 and at v1 (no x extreme strictly between them)
	};

	static SectionLoop Full(const Airfoil& airfoil);
	static SectionLoop Flap(const Airfoil& airfoil, const FlapCut& cut);
	static SectionLoop MainElement(const Airfoil& airfoil, const FlapCut& cut);

	// True when v = 1 coincides with v = 0 (LoftSource::closed_curve).
	bool closed() const { return closed_; }
	// The piece boundaries strictly inside (0, 1) (LoftSource::v_seeds).
	const std::vector<double>& seeds() const { return seeds_; }
	const std::vector<Piece>& pieces() const { return pieces_; }

	std::vector<double> Point(double v) const; // (x, y)
	Mat Points(const std::vector<double>& v) const; // (n, 2)

	// The parameter of the point across the body at the same x: a vertical
	// line through the point crosses the outline an even number of times,
	// and consecutive crossings (by y) bound the body's material, so the
	// twin is the crossing paired with this one. On a flap or full section
	// that is the other skin (the thin trailing-edge wedge); on the main
	// element, in the lip, the skin's twin is the cavity arc (the thin lip
	// wedge). A point at an x extreme (leading edge, the arcs' forward
	// extreme, a closed trailing edge) is its own twin. Exact to ~1e-12 in
	// parameter, and an involution to the same. See LoftSource::mirror_v.
	double Mirror(double v) const;

	// The airfoil parameter of a point on a skin piece, and back. NaN when v
	// is on an arc / t is on no skin piece of this loop.
	double AirfoilParameter(double v) const;
	double ParameterAtAirfoil(double t) const;

private:
	SectionLoop() = default;
	std::vector<double> PointOnPiece(const Piece& p, double v) const;
	double Wrap(double v) const;

	const Airfoil* airfoil_ = nullptr;
	double cx_ = 0.0, cy_ = 0.0, r_ = 0.0; // the arcs' circle
	bool closed_ = false;
	std::vector<Piece> pieces_;
	std::vector<double> seeds_;
};

} // namespace aerotelic
