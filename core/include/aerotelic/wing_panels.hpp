#pragma once

// Panel networks (panel_method.hpp) from a WingModel: the wing's blended
// sections placed by its planform, paneled as the report's check cases
// are -- rows round the section from the trailing edge over the leading
// edge and back, columns along the span -- with a wake network shed from
// the trailing edge.

#include <optional>
#include <string>
#include <vector>

#include "aerotelic/panel_method.hpp"
#include "aerotelic/wing_model.hpp"

namespace aerotelic::panel {

struct WingPaneling {
	int chordwise = 12;        // panels per surface (upper, lower), cosine-spaced in the airfoil's parameter
	int spanwise = 10;         // panels over the semi-span, cosine-spaced in eta
	double eta_min = 0.0, eta_max = 1.0;
	double wake_length = 0.0;  // 0: twenty root chords
	Vec3 wake_direction{1.0, 0.0, 0.0};
	// Thick surface (one closed network per semi-span, the interior-
	// potential formulation with the sources prescribed, figure 8) or the
	// mean camber surface (a doublet/analysis sheet with the velocity
	// condition of the thin-wing cases).
	bool thick = true;
	// A tip (or root) station whose chord is under this fraction of the root
	// chord is collapsed to the midpoint of its chord: one apex unknown and
	// a column of triangular panels, the solver's collapsed-edge case. Paneled
	// as a ring instead, the sliver's two dozen corner unknowns sit a
	// millimetre apart and no condition tells them apart (the P-38 tip, 0.6%
	// of the root chord, gave Cp of -260 and a negative tip load at 16 x 16).
	// Zero disables the collapse; a pointed tip is then pulled in by
	// tip_margin instead -- which leaves the tube open at a finite chord,
	// where the ending doublet sheet acts as a bound vortex, so prefer the
	// collapse.
	double collapse_tip_fraction = 0.02;
	// With the collapse off, pointed tips get their last station this
	// fraction short of the tip.
	double tip_margin = 0.01;
	// The root station (eta_min = 0) is laid in the plane y = 0 exactly: a
	// section is placed normal to the dihedral curve, so a dihedral slope at
	// the root tilts the root section and its corners stick ± thickness ·
	// sin(dihedral) through the plane (0.017 on the P-38 wing). Left there,
	// the root edge is not on the symmetry plane -- it gets no symmetry
	// condition and the network's image crosses it, which blew the root
	// column's pressures up to Cp -30 and more.
	bool pin_root_to_plane = true;
};

// The body network (semi-span, y >= 0) and its wake; solve them with
// Options::symmetry_y = 0 and the free stream the wing flies in. The
// body's boundary condition and prescribed sources are set for
// `free_stream`.
std::vector<Network> WingNetworks(const WingModel& wing, const WingPaneling& paneling, const Vec3& free_stream);

// ---- non-lifting bodies: every model's lofts as closed networks ----

struct BodyPaneling {
	int along = 16;  // panels along the loft (its u), on top of the loft's own seed stations
	int around = 12; // panels round each section (its v), on top of the seeds
	// Source/analysis (type 1, the Neumann condition, no edge conditions
	// at all: the report's sphere and cylinder) or doublet/analysis with the
	// interior potential held at zero and the sources prescribed.
	bool sources = true;
	// With Options::symmetry_y set, a body that straddles the plane y = value
	// (a fuselage on the centreline) must not be paneled whole, or its image
	// doubles it: set this to the same value and only the y >= value side is
	// built, each ring cut where it crosses the plane, so the image supplies
	// the other half. A body wholly to one side is left alone. Throws if a
	// ring crosses the plane other than twice.
	std::optional<double> half_plane_y;
};

// One corner-point network from a loft: its stations are the loft's seed
// stations plus `along` uniform ones, its ring parameters the loft's seeds
// plus `around` uniform ones; a closed section's ring and a periodic
// chain's rows close on a seam, and a station collapsed to a point gives
// triangular panels. Panels face outward (the network is flipped if its
// quads enclose a negative volume). `free_stream` is a unit vector; it
// only matters for the prescribed sources of the doublet formulation.
Network LoftNetwork(const NamedLoft& loft, const std::string& name, const BodyPaneling& paneling, const Vec3& free_stream);

// All of a model's lofts (ISolidModel::BuildLofts) as networks named
// "<name>/<body>" (or `name` alone for a single body). Empty for a model
// without lofts.
std::vector<Network> BodyNetworks(const ISolidModel& model, const std::string& name, const BodyPaneling& paneling, const Vec3& free_stream);

} // namespace aerotelic::panel
