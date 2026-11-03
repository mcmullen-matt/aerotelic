#pragma once

// Mass, centre of mass and inertia of triangle meshes, and of assemblies
// of them (Analysis -> Mass Properties...). OCCT-free: it integrates the
// same placed meshes STL export and the lattice generator consume, so a
// boolean result or a lattice -- which have no B-rep solid -- weigh the
// same way a wing does.
//
// Three models of where the mass sits, chosen per parameterisation in the
// Scene tree's Material Properties dialog (MassSpec):
//   - Material, solid: the body is a uniform solid; mass = density * the
//     volume the closed surface encloses. Zero for a surface with no
//     thickness, which is why the others exist.
//   - Material, skin: the surface is a skin of the given thickness t in
//     the material; its volume is area * t and mass = density * area * t,
//     spread uniformly over the surface as a thin shell. The model for a
//     moulded or printed skin whose thickness the geometry does not carry,
//     and it holds for an open surface too.
//   - Asserted: the user states the total mass and it is spread uniformly
//     over the surface, a thin shell.
// Either way the result is one MassProperties: mass, centre, and the
// inertia tensor about that centre in the world axes. An assembly is the
// parallel-axis sum of its parts about the combined centre.

#include <array>
#include <string>
#include <vector>

#include "aerotelic/materials.hpp"
#include "aerotelic/mesh.hpp"

namespace aerotelic {

enum class MassMode {
	Unset,     // nothing assigned: the analysis leaves the entity out, and says so
	Asserted,  // the user's total mass, spread uniformly over the surface
	Material,  // made of a material: its density times the enclosed volume
};

// Per parameterisation, not per body: a propeller's blades and spinner
// share one setting (an asserted mass is shared between them by area).
// The asserted mass is in the PROJECT's mass unit, as typed; the material
// is SI, as every material is (materials.hpp) -- a preset is a property of
// the stuff, not of the units the project happens to be drawn in.
struct MassSpec {
	MassMode mode = MassMode::Unset;
	double mass = 0.0;   // Asserted: the total, in the project's mass unit
	Material material;   // Material: density (and the structural properties), SI
	// Material: the skin thickness, in the PROJECT's length unit (as typed,
	// like every geometric figure). 0 means a solid: the density weighs the
	// enclosed volume. Positive means a skin: every body of the
	// parameterisation is a shell of this thickness, its material volume
	// area * thickness and its mass density * area * thickness.
	double skin_thickness = 0.0;

	bool IsSkin() const { return mode == MassMode::Material && skin_thickness > 0.0; }
};

const char* MassModeName(MassMode m);
MassMode ParseMassMode(const std::string& s);

// Row-major symmetric 3x3 in the engineer's convention: the diagonal
// holds the moments Ixx, Iyy, Izz; the off-diagonals hold MINUS the
// products of inertia (-Ixy, -Ixz, -Iyz), so the tensor acts on an
// angular velocity directly.
using InertiaTensor = std::array<double, 9>;

struct MassProperties {
	double mass = 0.0;
	Vec3 centre;              // centre of mass
	InertiaTensor inertia{};  // about `centre`, in the world axes
	// What the mass was spread over, for the report: the enclosed volume
	// (signed-volume of the mesh; meaningful only when it is closed) and
	// the surface area.
	double volume = 0.0;
	double area = 0.0;
};

// `mesh` as a uniform solid of `density`. The surface must be closed and
// outward-facing for the volume integrals to mean anything (an inside-out
// mesh is taken as outward; an open one gives a figure that depends on
// where the origin is -- check TriMesh::IsClosedManifold first and say so).
MassProperties SolidMassProperties(const TriMesh& mesh, double density);

// `mesh` as a uniform thin shell carrying `mass` in total. Open or closed
// alike: only the triangles themselves are integrated.
MassProperties ShellMassProperties(const TriMesh& mesh, double mass);

// `mesh` as a skin of `thickness` in a material of `density` (mass per
// unit volume, in the mesh's units): mass = density * area * thickness,
// spread as ShellMassProperties spreads it; `volume` is the material's,
// area * thickness, not the enclosed volume. Open or closed alike.
MassProperties SkinMassProperties(const TriMesh& mesh, double density, double thickness);

// The inertia of `p` about an arbitrary point: the parallel-axis theorem,
// I_point = I_centre + m (|d|^2 1 - d d^T) with d from the point to the
// centre of mass.
InertiaTensor InertiaAbout(const MassProperties& p, const Vec3& point);

// The parts as one body: total mass, the mass-weighted centre, and every
// part's inertia carried to that centre by the parallel-axis theorem and
// summed. Volume and area are summed too.
MassProperties CombineMassProperties(const std::vector<MassProperties>& parts);

// Principal moments (ascending) and the unit principal axes they belong to
// (rows of a right-handed frame): the eigen-decomposition of the symmetric
// tensor, by Jacobi rotations.
void PrincipalInertia(const InertiaTensor& inertia, std::array<double, 3>& moments, std::array<Vec3, 3>& axes);

} // namespace aerotelic
