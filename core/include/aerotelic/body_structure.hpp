#pragma once

// What a solid BODY is made of and how it is built inside -- the
// attributes a parameterisation's bodies carry one by one (a wing's
// "Main" and each flap, a propeller's spinner and blades, a boolean
// result's "Body k"), since 2026-11-08. Before that one MassSpec per
// parameterisation gave every body the same material and skin, and a
// lattice was a scene entity of its own that targeted whole entities.
//
// A BodyStructure says:
//   - the material the body (or its skin) is made of, and whether the
//     body is SOLID (skin_thickness 0: a uniform solid of the material) or
//     a SKIN of that thickness enclosing an interior;
//   - what fills the interior of a skinned body: an optional lattice
//     INFILL (an OpenLattice spec, lattice.hpp, in a material of its own)
//     and any number of PLANE NETWORKS -- families of parallel cutting
//     planes whose intersection with the interior gives ribs, spars and
//     frames, each network with its own thickness and material, spanning
//     wall to wall or reaching only a given height in from the surface
//     (a stringer, a spar cap);
//   - reserved volumes (scene names) the interior structure must avoid;
//   - or an asserted total mass that stands in for all of the above in
//     the mass report.
//
// Units, as elsewhere: materials are SI (materials.hpp); skin thickness,
// a network's thickness and height are PHYSICAL build figures in the
// project's length unit, as typed, and are not scaled with the model
// scale (a 1:4 replica is still printed with a 2 mm rib); a network's
// start and spacing are GEOMETRY in the parameterisation's own full-scale
// frame -- plane k is normal . x = start + k * spacing there -- and are
// placed, scaled and mirrored with the body like a chord or a station.
//
// The structure's generated geometry (the skin shell, the infill, the
// ribs) and its finite-element model are built elsewhere from these
// figures; this header is the data and the plane geometry both share.

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "aerotelic/lattice.hpp"
#include "aerotelic/materials.hpp"
#include "aerotelic/mesh.hpp"
#include "aerotelic/vec3.hpp"

namespace aerotelic {

// The axis a network's planes are normal to: a coordinate axis, or any
// unit vector of the user's.
enum class PlaneAxis { X, Y, Z, Custom };
const char* PlaneAxisName(PlaneAxis a);
PlaneAxis ParsePlaneAxis(const std::string& s);

struct PlaneNetwork {
	std::string name = "Ribs";
	PlaneAxis axis = PlaneAxis::Y;
	// The unit normal for PlaneAxis::Custom; ignored (and kept as the
	// axis) otherwise. NetworkNormal() gives the one in force.
	Vec3 normal{0.0, 1.0, 0.0};
	// Plane k (k = 0 .. count-1): normal . x = start + k * spacing, in the
	// parameterisation's full-scale frame, project length unit.
	double start = 0.0;
	double spacing = 0.1;
	int count = 1;
	// The rib's thickness, and how far in from the body surface it reaches:
	// 0 spans the interior wall to wall; positive stops that far in (a
	// stringer or cap along the skin). Project length unit, not scaled.
	double thickness = 0.002;
	double height = 0.0;
	Material material;   // SI
};

// The lattice filling a skinned body's interior, in its own material.
// The spec's own skin_thickness is ignored: the body's rules.
struct InfillSpec {
	LatticeSpec lattice;
	Material material;   // SI
};

struct BodyStructure {
	// The solid's material, or the skin's. A material without a density
	// means nothing has been assigned (the body is left out of the mass
	// report, and the analysis refuses it by name).
	Material material;
	// 0: a uniform solid of `material`. Positive: a skin of this thickness
	// (project length unit, not scaled) enclosing an interior that the
	// infill and the networks fill.
	double skin_thickness = 0.0;
	std::optional<InfillSpec> infill;
	std::vector<PlaneNetwork> networks;
	// Scene names of the volumes the interior structure leaves alone (a
	// payload bay). Default none.
	std::vector<std::string> exclusions;
	// The user's total mass for the body, project mass unit: when set it
	// replaces every computed figure in the mass report (spread over the
	// body's surface as a thin shell), as the old Asserted mode did.
	std::optional<double> asserted_mass;

	bool IsSet() const { return material.HasDensity() || asserted_mass.has_value(); }
	bool IsSkin() const { return material.HasDensity() && skin_thickness > 0.0; }
	bool HasInterior() const { return skin_thickness > 0.0 && (infill.has_value() || !networks.empty()); }
};

// Every figure of the two specs equal (name and type included).
bool SameLatticeSpec(const LatticeSpec& a, const LatticeSpec& b);
// True when regenerating the structure's geometry would give the same
// result: skin thickness, infill lattice, every network's planes,
// thickness and height, the exclusions. Materials and the asserted mass
// do not count (they change the weights, not the shapes).
bool SameStructureGeometry(const BodyStructure& a, const BodyStructure& b);

// Why the network cannot be built, or empty: no planes, a non-positive
// spacing or thickness with more than one plane, planes thicker than
// their spacing (they would overlap), a negative height, a zero custom
// normal.
std::string PlaneNetworkProblem(const PlaneNetwork& n);
// Why the structure cannot be used, or empty: a negative skin thickness
// or asserted mass, an interior (infill or networks) on a solid body, or a
// network's problem prefixed with its name.
std::string BodyStructureProblem(const BodyStructure& s);

// ---- plane geometry -------------------------------------------------------

// normal . x = offset; `normal` unit.
struct Plane {
	Vec3 normal{0.0, 0.0, 1.0};
	double offset = 0.0;
	double Distance(const Vec3& p) const { return Dot(normal, p) - offset; }
};

// The network's unit normal: the axis's, or the custom one normalised.
Vec3 NetworkNormal(const PlaneNetwork& n);
// start + k * spacing.
double PlaneOffset(const PlaneNetwork& n, int k);
// Plane k in the parameterisation's frame.
Plane NetworkPlane(const PlaneNetwork& n, int k);
// The plane carried by a placement: a point x of the frame sits at
// scale * R x + T in the world (R row-major, a rotation), so the plane's
// normal becomes R n and its offset scale * d + (R n) . T.
Plane PlacedPlane(const Plane& local, const std::array<double, 9>& R, const Vec3& T, double scale);
// The plane's reflection in y = 0 (the mirrored half of a mirrored entity).
Plane MirrorPlaneY(const Plane& p);
// Signed distance from p to the nearest plane's slab: |n.p - d_k| - t/2
// with k the nearest plane (clamped to the network), negative inside the
// rib material. The implicit form of the network, for tests and read-outs.
double PlaneNetworkField(const PlaneNetwork& n, const Vec3& p);

// An orthonormal in-plane basis (u, v) with u x v = n.
void PlaneBasis(const Vec3& n, Vec3* u, Vec3* v);
// The slab of `thickness` about the plane: a box 2 * half_extent across in
// the plane, centred on `centre` projected onto the plane, closed and
// outward. The second operand of a rib boolean.
TriMesh SlabMesh(const Plane& p, double thickness, const Vec3& centre, double half_extent);
// The rectangle of the plane that covers the box's shadow on it (the
// preview of a cutting plane): false when the plane misses the box.
bool PlaneBoxRectangle(const Plane& p, const Box3& box, std::array<Vec3, 4>* corners);

// The section of a closed surface by a plane: its loops, each a closed
// polyline in the plane, counter-clockwise seen from the plane's normal
// side when it bounds material (outer) and clockwise around a hole; the
// crossing points are shared exactly between neighbouring triangles (keyed
// by the edge they lie on), so the loops close without welding. A vertex
// exactly on the plane counts as on the positive side -- the plane is in
// effect shifted a hair towards its negative side, so a plane lying on a
// face returns that face's outline and a plane through an edge passes
// cleanly through its vertices.
struct PlaneSection {
	std::vector<std::vector<Vec3>> loops;
	std::vector<bool> outer;      // per loop: bounds material (signed area > 0)
	std::vector<double> areas;    // per loop, signed: positive outer, negative hole
	// The material's area in the plane: the signed areas summed.
	double Area() const;
	// Every loop's length summed (the wetted perimeter of the section).
	double Perimeter() const;
};
PlaneSection SectionByPlane(const TriMesh& closed, const Plane& p);

} // namespace aerotelic
