#pragma once

// What a solid BODY is made of -- the attributes a parameterisation's
// bodies carry one by one (a wing's "Main" and each flap, a propeller's
// spinner and blades, a boolean result's "Body k"), since 2026-11-08.
// Before that one MassSpec per parameterisation gave every body the same
// material and skin.
//
// The application's structural elements are SHELLS: the skin, and the
// ribs, spars, frames and stringers the PARAMETERISATION defines and
// builds with the body (a wing's spars and ribs in wing_model.hpp,
// WingStructureSpec). A BodyStructure says:
//   - the MATERIAL the body (or its skin) is made of;
//   - whether the body is SOLID (skin_thickness 0: a uniform solid of the
//     material) or a SKIN of that thickness enclosing an interior that the
//     parameterisation's structure fills;
//   - EXCLUSIONS: reserved volumes (scene names) the interior structure
//     must avoid (a payload bay);
//   - or an ASSERTED total mass that stands in for all of the above in
//     the mass report.
// (Plane NETWORKS -- families of parallel cutting planes set per body,
// 2026-11-08 -- were dropped on 2026-11-10 in favour of structure defined
// in each parameterisation's own terms; a lattice infill went two days
// before. A file carrying `networks:` loads without them and says so.)
//
// Units, as elsewhere: materials are SI (materials.hpp); the skin
// thickness is a PHYSICAL build figure in the project's length unit, as
// typed, and is not scaled with the model scale (a 1:4 replica is still
// printed with a 2 mm skin).
//
// The plane geometry below (a plane, its in-plane basis, the section of
// a closed surface by it) stays: the mass report and the mesh cutter use
// it.

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "aerotelic/materials.hpp"
#include "aerotelic/mesh.hpp"
#include "aerotelic/vec3.hpp"

namespace aerotelic {

struct BodyStructure {
	// The solid's material, or the skin's. A material without a density
	// means nothing has been assigned (the body is left out of the mass
	// report, and the analysis refuses it by name).
	Material material;
	// 0: a uniform solid of `material`. Positive: a skin of this thickness
	// (project length unit, not scaled) enclosing an interior that the
	// parameterisation's structure fills.
	double skin_thickness = 0.0;
	// Scene names of the volumes the interior structure leaves alone (a
	// payload bay). Default none.
	std::vector<std::string> exclusions;
	// The user's total mass for the body, project mass unit: when set it
	// replaces every computed figure in the mass report (spread over the
	// body's surface as a thin shell), as the old Asserted mode did.
	std::optional<double> asserted_mass;

	bool IsSet() const { return material.HasDensity() || asserted_mass.has_value(); }
	bool IsSkin() const { return material.HasDensity() && skin_thickness > 0.0; }
};

// True when regenerating the structure's geometry would give the same
// result: the skin thickness and the exclusions. Materials and the
// asserted mass do not count (they change the weights, not the shapes).
bool SameStructureGeometry(const BodyStructure& a, const BodyStructure& b);

// Why the structure cannot be used, or empty: a negative skin thickness
// or asserted mass.
std::string BodyStructureProblem(const BodyStructure& s);

// ---- plane geometry -------------------------------------------------------

// normal . x = offset; `normal` unit.
struct Plane {
	Vec3 normal{0.0, 0.0, 1.0};
	double offset = 0.0;
	double Distance(const Vec3& p) const { return Dot(normal, p) - offset; }
};

// An orthonormal in-plane basis (u, v) with u x v = n.
void PlaneBasis(const Vec3& n, Vec3* u, Vec3* v);

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
