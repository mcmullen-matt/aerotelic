#pragma once

// The INTERNAL STRUCTURE a body's parameterisation plans on the body's
// own grid (2026-11-09 for wings, 2026-11-10 for fuselages and nacelles):
// ribs or frames whose rims are rings of the body mesh, and spar webs and
// stringers as ready meshes with their feet on the body's vertices. The
// application meshes the discs (tools/rib_gmsh.py) and shows, weighs and
// analyses the layers. Every element is a thin shell with its own
// thickness and material; a material without a density means the body's
// own.
//
// Bodies lofted as RINGS ALONG AN AXIS -- a fuselage, a nacelle -- share
// one description (FrameStructureSpec): frames in the section planes at
// stations along the length, of a given height in from the skin (a ring
// band; 0 keeps the whole bulkhead), and stringers running the length at a
// given position round the section. A wing's is in wing_model.hpp.

#include <array>
#include <string>
#include <vector>

#include "aerotelic/materials.hpp"
#include "aerotelic/mesh.hpp"

namespace aerotelic {

// One rib (frame, bulkhead) of a body, as the rib mesher takes it: the
// closed loop of BODY-MESH vertex ids round the section at its station,
// the loops of any holes in it (a nacelle's inner skin), the chords across
// it where spars pass (the two feet's vertex ids and the number of nodes
// across, the same the spar's own web carries there, so the two share
// their nodes), and the height kept in from the rims (0: the whole disc).
struct RibPlan {
	// The station; negative on the port half of a symmetric wing, 0 for
	// the centre rib in the symmetry plane; a fuselage's x fraction.
	double eta = 0.0;
	std::string name;
	std::vector<int> rim;
	std::vector<std::vector<int>> holes;
	std::vector<std::array<int, 3>> chords;
	double height = 0.0;
};

// A spar web or a stringer: a ready open mesh in the body's frame, its
// feet on the body mesh's own vertices.
struct ShellPlan {
	std::string name;
	TriMesh mesh;
	double thickness = 0.0;
	Material material;
};

// The structure of one body ("Main", a flap's name) the build planned on
// that body's mesh.
struct BodyStructurePlan {
	std::string body;
	std::vector<RibPlan> ribs;
	// What the ribs' layer is called under the body: "Ribs" (a wing),
	// "Frames" (a fuselage, a nacelle).
	std::string rib_layer = "Ribs";
	double rib_thickness = 0.0;
	Material rib_material;
	std::vector<ShellPlan> spars, stringers;
	// The body mesh's median edge: the rib discs' target size.
	double rib_size = 0.0;
	std::vector<std::string> notes;
	bool IsEmpty() const { return ribs.empty() && spars.empty() && stringers.empty(); }
};

// ---------------------------------------------------------------------
// Frames and stringers of a body lofted along its length (a fuselage, a
// nacelle): the stations are fractions of the length (0 at the nose or
// lip, 1 at the tail or trailing edge); a position round the section is
// the loft's own along-curve parameter in [0, 1) -- for a fuselage 0 is
// the top centre, 0.25 the starboard shoulder, 0.5 the bottom centre,
// 0.75 the port shoulder; for a nacelle the lip curve's own parameter,
// 0 where the guide curve starts.
// ---------------------------------------------------------------------

struct FrameRules {
	int between = 8;              // evenly spaced along the length, the ends not counted
	std::vector<double> extra;    // more stations, fractions of the length
	double height = 0.1;          // the band kept in from the skin, project units; 0: the whole bulkhead
	double thickness = 0.002;
	Material material;
};

struct FrameStringerSpec {
	std::string name = "Longeron";
	double around = 0.0;          // position round the section, [0, 1)
	double x_start = 0.0, x_end = 1.0;   // fractions of the length
	double height = 0.03;         // in from the skin, project units
	double thickness = 0.001;
	Material material;
};

struct FrameStructureSpec {
	FrameRules frames;
	// A fresh spec IS the default structure: eight frames and four
	// longerons at the quarter positions between top, sides and bottom.
	std::vector<FrameStringerSpec> stringers = DefaultStringers();
	static std::vector<FrameStringerSpec> DefaultStringers();
	static FrameStructureSpec Default();
	bool IsEmpty() const;
	// The frame stations the rules give, strictly inside (0, 1), sorted,
	// duplicates merged.
	std::vector<double> FrameStations() const;
	// Throws std::invalid_argument naming the element and the reason.
	void Validate() const;
};

// How a body's rings map onto its frames and stringers, for
// PlanFrameStructure: per frame station the ring of the outer skin there
// and (an annular body) the ring of the inner skin or -1; and the rings a
// stringer runs along, in order along the length, each with its station.
struct FrameRingSet {
	std::vector<double> stations;
	std::vector<int> outer, inner;
	std::vector<std::pair<double, int>> skin_rings;
};

// Plans the frames (rims from the outer rings, holes from the inner, the
// rules' height) and the stringers (strips from the skin rings' vertices
// at each stringer's position, standing in normal to the skin in the
// section plane -- whose normal is `plane_normal` -- towards the ring's
// centroid) on `body` from its `rings`. A ring collapsed to an apex, or
// one without the stringer's parameter, is skipped with a note.
BodyStructurePlan PlanFrameStructure(const std::string& body_name, const TriMesh& body, const LoftRings& rings,
	const FrameStructureSpec& spec, const FrameRingSet& set, const Vec3& plane_normal, const std::vector<std::string>& notes);

// Helpers shared with the wing's planner.
// The ring's vertex ids with repeats dropped (an apex ring, a ring closed
// on its first vertex), empty when fewer than three remain.
std::vector<int> RimOfRing(const std::vector<int>& ring);
// The ring's index of the point at along-curve parameter c, or -1.
int RingIndexAt(const std::vector<double>& ring_v, double c);
// The median edge length of a mesh.
double MedianEdge(const TriMesh& m);
// The strip between two polylines of equal length with `across` points
// between each pair (the ends included), as triangles: a spar's web, a
// stringer's blade. The across points are a + (b - a) * (j * step), the
// same arithmetic the rib mesher uses for a chord's nodes, so the two
// coincide to the bit.
TriMesh StripMesh(const std::vector<Vec3>& lower, const std::vector<Vec3>& upper, int across);

} // namespace aerotelic
