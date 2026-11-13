#pragma once

// Panel networks (panel_method.hpp) from a WingModel: the wing's blended
// sections placed by its planform, paneled as the report's check cases
// are -- rows round the section from the trailing edge over the leading
// edge and back, columns along the span -- with a wake network shed from
// the trailing edge. A BLUNT section (a .dat airfoil with an open
// trailing edge, a hand-edited one) is closed for the aero copy by the
// XFOIL-style blend of section_aero.hpp (CloseTrailingEdge: the gap taken
// out over the last 30% of the chord, quadratically), so the wake abuts
// one edge; until 2026-11-13 the wake's head sat on the base's midpoint,
// off the surface, and the solve threw.

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
	// THE WAKE (2026-11-13). A loft with a trailing-edge tag
	// (NamedLoft::trailing_edge) sheds a doublet wake from it, along
	// wake_direction (a unit vector; the free stream) for wake_length (0:
	// twenty times the body's extent); the body is then a doublet network
	// under the interior-potential formulation with prescribed sources,
	// whatever `sources` says, since only doublet networks can abut a
	// wake. false panels every body as a closed non-lifting one, as before.
	bool wake = true;
	Vec3 wake_direction{1.0, 0.0, 0.0};
	double wake_length = 0.0;
	// VSeam: how much of the ring, in its parameter v, the blunt-edge blend
	// spans at each end (the aero copy's closure of an open section).
	double seam_blend = 0.2;
	// A BURIED trailing edge sheds no wake (2026-11-13): a cowl whose rim
	// meets the boom behind it, a fin lofted into a fuselage. With the
	// other bodies of the configuration given (their placed triangle
	// meshes, not owned), the tagged edge is probed `buried_probe`
	// downstream along wake_direction (0: 2% of the body's extent) and the
	// wake is dropped when most of the probes lie inside another body or
	// within `buried_tolerance` of one (0: 5% of the extent); the body is
	// then paneled as a closed non-lifting one and `notes`, when given,
	// says so. A free wake run through a body behind it is nonsense for
	// both (the P-38's engine nacelle ahead of its boom lost the whole
	// configuration its lift).
	std::vector<const TriMesh*> other_bodies;
	double buried_probe = 0.0;
	double buried_tolerance = 0.0;
	std::vector<std::string>* notes = nullptr;
};

// Whether a wake shed from `trailing_edge` along `direction` would run
// into one of `bodies`: the edge's points stepped `probe` downstream lie
// inside a body along a quarter of the edge (the wake would pierce it),
// or within `tolerance` of a surface along `fraction` of it (the wake
// would sit on it) (BodyPaneling::other_bodies).
bool TrailingEdgeBuried(const std::vector<Vec3>& trailing_edge, const Vec3& direction, double probe, double tolerance,
	const std::vector<const TriMesh*>& bodies, double fraction = 0.5);

// One corner-point network from a loft: its stations are the loft's seed
// stations plus `along` uniform ones, its ring parameters the loft's seeds
// plus `around` uniform ones; a closed section's ring and a periodic
// chain's rows close on a seam, and a station collapsed to a point gives
// triangular panels. Panels face outward (the network is flipped if its
// quads enclose a negative volume). `free_stream` is a unit vector; it
// only matters for the prescribed sources of the doublet formulation.
// The loft's trailing-edge tag is IGNORED here: a closed non-lifting
// body, as before 2026-11-13. LoftNetworks honours it.
Network LoftNetwork(const NamedLoft& loft, const std::string& name, const BodyPaneling& paneling, const Vec3& free_stream);

// The networks of a loft WITH ITS WAKE when it carries a trailing-edge
// tag and paneling.wake is on (2026-11-13): VSeam -- the body as one
// doublet network with prescribed sources, open at its ends and the seam
// closed by the blend, the wake abutting both seam edges (what
// WingNetworks does with rows); UStation -- the wall's mean surface as one
// doublet SHEET under the velocity condition, facing outward, the ring
// wake abutting its last row (the thin annular wing); the wake network
// named "<name> wake" last. Without a tag, or with the wake off, the one
// closed network of LoftNetwork. A VSeam loft cut at the symmetry plane
// (half_plane_y) gets no wake: its ring is an arc, not a section.
std::vector<Network> LoftNetworks(const NamedLoft& loft, const std::string& name, const BodyPaneling& paneling, const Vec3& free_stream);

// All of a model's lofts (ISolidModel::BuildLofts) as networks named
// "<name>/<body>" (or `name` alone for a single body), through
// LoftNetworks -- with their wakes where tagged. Empty for a model
// without lofts.
std::vector<Network> BodyNetworks(const ISolidModel& model, const std::string& name, const BodyPaneling& paneling, const Vec3& free_stream);

} // namespace aerotelic::panel
