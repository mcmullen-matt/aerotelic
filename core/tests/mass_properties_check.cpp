// Checks mass_properties.hpp against closed forms: the solid and shell
// integrals on primitives whose mass, centre and inertia are textbook
// values, invariance under translation and rotation, the parallel-axis
// combination against a single body of the combined shape, and the
// principal decomposition. The mesh integrals are exact for a polyhedron,
// so a box matches to rounding; a sphere or cylinder matches its analytic
// value to the tessellation's own error, which is what the tolerance
// there allows for.

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include "aerotelic/feature_tree.hpp"
#include "aerotelic/mass_properties.hpp"
#include "aerotelic/mesh.hpp"
#include "aerotelic/primitive_model.hpp"

using namespace aerotelic;

namespace {

constexpr double kPi = 3.14159265358979323846;

int g_pass = 0, g_fail = 0;

void Check(bool cond, const std::string& msg) {
	if (cond) { g_pass++; return; }
	g_fail++;
	std::cout << "FAIL: " << msg << "\n";
}

void CheckClose(double a, double b, double tol, const std::string& msg) {
	Check(std::abs(a - b) <= tol, msg + " (" + std::to_string(a) + " vs " + std::to_string(b) + ")");
}

void CheckRel(double a, double b, double rel, const std::string& msg) {
	CheckClose(a, b, rel * std::max(std::abs(b), 1e-300), msg);
}

// Diagonal moments by index, products by their signed tensor entries.
double Ixx(const InertiaTensor& I) { return I[0]; }
double Iyy(const InertiaTensor& I) { return I[4]; }
double Izz(const InertiaTensor& I) { return I[8]; }
double MaxOffDiagonal(const InertiaTensor& I) {
	return std::max({std::abs(I[1]), std::abs(I[2]), std::abs(I[3]), std::abs(I[5]), std::abs(I[6]), std::abs(I[7])});
}

void CheckSolidBox() {
	const double lx = 2.0, ly = 3.0, lz = 5.0, rho = 7.0;
	const MassProperties p = SolidMassProperties(BoxMesh(lx, ly, lz), rho);
	const double m = rho * lx * ly * lz;
	CheckClose(p.mass, m, 1e-9, "solid box: mass = density * volume");
	CheckClose(p.volume, lx * ly * lz, 1e-9, "solid box: volume");
	CheckClose(p.area, 2.0 * (lx * ly + ly * lz + lz * lx), 1e-9, "solid box: area");
	CheckClose(Length(p.centre), 0.0, 1e-9, "solid box: centre at the origin");
	CheckClose(Ixx(p.inertia), m * (ly * ly + lz * lz) / 12.0, 1e-9, "solid box: Ixx = m (ly^2 + lz^2) / 12");
	CheckClose(Iyy(p.inertia), m * (lx * lx + lz * lz) / 12.0, 1e-9, "solid box: Iyy");
	CheckClose(Izz(p.inertia), m * (lx * lx + ly * ly) / 12.0, 1e-9, "solid box: Izz");
	CheckClose(MaxOffDiagonal(p.inertia), 0.0, 1e-9, "solid box: no products of inertia in its own axes");

	// Wound the other way it is the same body.
	TriMesh inside_out = BoxMesh(lx, ly, lz);
	inside_out.FlipWinding();
	const MassProperties q = SolidMassProperties(inside_out, rho);
	CheckClose(q.mass, m, 1e-9, "solid box: an inside-out mesh weighs the same");
	CheckClose(Ixx(q.inertia), Ixx(p.inertia), 1e-9, "solid box: ...with the same inertia");
}

void CheckShellBox() {
	// Six uniform plates: each a thin rectangle about its own centre, then
	// carried to the box centre by the parallel-axis theorem -- a different
	// derivation from the triangle integrals it is checked against.
	const double lx = 2.0, ly = 3.0, lz = 5.0, M = 11.0;
	const double area = 2.0 * (lx * ly + ly * lz + lz * lx);
	const MassProperties p = ShellMassProperties(BoxMesh(lx, ly, lz), M);
	CheckClose(p.mass, M, 1e-9, "shell box: the asserted mass comes back");
	CheckClose(p.area, area, 1e-9, "shell box: area");
	CheckClose(Length(p.centre), 0.0, 1e-9, "shell box: centre at the origin");

	auto plate = [&](double a, double b, double offset, double& about_normal, double& about_a, double& about_b) {
		// A plate a x b with its normal along the third axis, a distance
		// `offset` from the centre along that normal; two of them.
		const double mf = M * (a * b) / area;
		about_normal += 2.0 * mf * (a * a + b * b) / 12.0;
		about_a += 2.0 * (mf * b * b / 12.0 + mf * offset * offset);  // about the in-plane axis along a
		about_b += 2.0 * (mf * a * a / 12.0 + mf * offset * offset);
	};
	double ixx = 0.0, iyy = 0.0, izz = 0.0;
	plate(ly, lz, lx / 2.0, ixx, iyy, izz);  // the two x = const faces: a along y, b along z
	plate(lx, lz, ly / 2.0, iyy, ixx, izz);  // y = const faces: a along x, b along z
	plate(lx, ly, lz / 2.0, izz, ixx, iyy);  // z = const faces: a along x, b along y
	CheckClose(Ixx(p.inertia), ixx, 1e-9, "shell box: Ixx from six plates");
	CheckClose(Iyy(p.inertia), iyy, 1e-9, "shell box: Iyy from six plates");
	CheckClose(Izz(p.inertia), izz, 1e-9, "shell box: Izz from six plates");
	CheckClose(MaxOffDiagonal(p.inertia), 0.0, 1e-9, "shell box: no products of inertia");
}

void CheckSingleTriangle() {
	// A lamina: the polar moment of a triangle about its centroid is
	// m (a^2 + b^2 + c^2) / 36 with a, b, c the side lengths. 3-4-5 here.
	TriMesh t;
	t.vertices = {Vec3{0, 0, 0}, Vec3{3, 0, 0}, Vec3{0, 4, 0}};
	t.triangles = {{0, 1, 2}};
	const MassProperties p = ShellMassProperties(t, 9.0);
	CheckClose(p.area, 6.0, 1e-12, "triangle: area");
	CheckClose(p.centre.x, 1.0, 1e-12, "triangle: centroid x");
	CheckClose(p.centre.y, 4.0 / 3.0, 1e-12, "triangle: centroid y");
	CheckClose(Izz(p.inertia), 9.0 * (9.0 + 16.0 + 25.0) / 36.0, 1e-12, "triangle: polar moment m(a^2+b^2+c^2)/36");
	CheckClose(Ixx(p.inertia) + Iyy(p.inertia), Izz(p.inertia), 1e-12, "triangle: perpendicular-axis theorem for a lamina");
}

void CheckSphereAndCylinder() {
	// Tessellated, so the analytic values are matched to the tessellation's
	// own error -- tolerance 1e-3 on a unit radius is well under 1%.
	const double r = 1.0, rho = 3.0;
	const TriMesh sphere = EllipsoidMesh(r, r, r, 1e-3);
	Check(sphere.IsClosedManifold(), "sphere mesh is closed");
	const MassProperties s = SolidMassProperties(sphere, rho);
	const double ms = rho * 4.0 / 3.0 * kPi * r * r * r;
	CheckRel(s.mass, ms, 1e-2, "solid sphere: mass");
	CheckRel(Ixx(s.inertia), 0.4 * ms * r * r, 1e-2, "solid sphere: I = 2/5 m r^2");
	// Isotropic to the tessellation's own anisotropy (a lat/long-style
	// triangulation is not quite the same seen along every axis).
	CheckRel(Izz(s.inertia), Ixx(s.inertia), 1e-3, "solid sphere: isotropic");

	const MassProperties sh = ShellMassProperties(sphere, 5.0);
	CheckRel(Ixx(sh.inertia), 2.0 / 3.0 * 5.0 * r * r, 1e-2, "spherical shell: I = 2/3 m r^2");

	// Cylinder, axis along X (as BuildSolid makes it).
	const double rc = 0.5, L = 4.0;
	const MassProperties c = SolidMassProperties(CylinderMesh(rc, L, 1e-4), rho);
	const double mc = rho * kPi * rc * rc * L;
	CheckRel(c.mass, mc, 1e-2, "solid cylinder: mass");
	CheckRel(Ixx(c.inertia), 0.5 * mc * rc * rc, 1e-2, "solid cylinder: about its axis, m r^2 / 2");
	CheckRel(Iyy(c.inertia), mc * (3.0 * rc * rc + L * L) / 12.0, 1e-2, "solid cylinder: across, m (3 r^2 + L^2) / 12");

	// The model's exact readouts agree with what the mesh integrated.
	PrimitiveModel prim;
	prim.shape = PrimitiveShape::Cylinder;
	prim.cyl_radius = rc;
	prim.cyl_length = L;
	CheckRel(c.volume, prim.Volume(), 1e-2, "solid cylinder: mesh volume vs the model's exact volume");
	CheckRel(c.area, prim.SurfaceArea(), 1e-2, "solid cylinder: mesh area vs the model's exact area");
}

void CheckInvariances() {
	const double lx = 2.0, ly = 3.0, lz = 5.0, rho = 1.5;
	const MassProperties home = SolidMassProperties(BoxMesh(lx, ly, lz), rho);

	TriMesh moved = BoxMesh(lx, ly, lz);
	moved.Translate(Vec3{10.0, -3.0, 7.0});
	const MassProperties away = SolidMassProperties(moved, rho);
	CheckClose(away.mass, home.mass, 1e-9, "translation: mass unchanged");
	CheckClose(away.centre.x, 10.0, 1e-9, "translation: centre follows (x)");
	CheckClose(away.centre.y, -3.0, 1e-9, "translation: centre follows (y)");
	CheckClose(away.centre.z, 7.0, 1e-9, "translation: centre follows (z)");
	for (int i = 0; i < 9; ++i) {
		CheckClose(away.inertia[i], home.inertia[i], 1e-7, "translation: inertia about the centre unchanged, entry " + std::to_string(i));
	}
	// ...and about the origin it grows by the parallel-axis term.
	const InertiaTensor about_origin = InertiaAbout(away, Vec3{0, 0, 0});
	const double d2 = 100.0 + 9.0 + 49.0;
	CheckClose(Ixx(about_origin), Ixx(home.inertia) + home.mass * (d2 - 100.0), 1e-7, "parallel axis: Ixx about the origin");
	CheckClose(about_origin[1], -home.mass * (10.0 * -3.0), 1e-7, "parallel axis: the -Ixy product about the origin");

	// Rotated, the tensor changes but its principal moments do not, and the
	// principal axes are the box's own axes carried round.
	TriMesh turned = BoxMesh(lx, ly, lz);
	const Mat3 R = RotationMatrix({1.0, 2.0, -0.5}, 37.0);
	turned.Transform(R, Vec3{0, 0, 0});
	const MassProperties spun = SolidMassProperties(turned, rho);
	std::array<double, 3> moments, home_moments;
	std::array<Vec3, 3> axes, home_axes;
	PrincipalInertia(spun.inertia, moments, axes);
	PrincipalInertia(home.inertia, home_moments, home_axes);
	Check(MaxOffDiagonal(spun.inertia) > 1e-3, "rotation: the tensor really did pick up products of inertia");
	for (int n = 0; n < 3; ++n) {
		CheckClose(moments[n], home_moments[n], 1e-7, "rotation: principal moment " + std::to_string(n) + " unchanged");
		// The matching principal axis is a box axis turned by R (either sign).
		double best = 0.0;
		for (int k = 0; k < 3; ++k) {
			const Vec3 e = Vec3{R[k], R[3 + k], R[6 + k]};  // column k of R: the image of axis k
			best = std::max(best, std::abs(Dot(axes[n], e)));
		}
		CheckClose(best, 1.0, 1e-7, "rotation: principal axis " + std::to_string(n) + " is a turned box axis");
	}
	Check(moments[0] <= moments[1] && moments[1] <= moments[2], "principal moments come back ascending");
	CheckClose(Dot(Cross(axes[0], axes[1]), axes[2]), 1.0, 1e-9, "principal axes are a right-handed orthonormal frame");
}

void CheckCombine() {
	// Two unit cubes side by side are a 2 x 1 x 1 box: the parallel-axis
	// sum of the parts about their joint centre must equal the whole.
	const double rho = 2.0;
	TriMesh left = BoxMesh(1, 1, 1), right = BoxMesh(1, 1, 1);
	left.Translate(Vec3{-0.5, 0, 0});
	right.Translate(Vec3{0.5, 0, 0});
	const MassProperties whole = SolidMassProperties(BoxMesh(2, 1, 1), rho);
	const MassProperties sum = CombineMassProperties({SolidMassProperties(left, rho), SolidMassProperties(right, rho)});
	CheckClose(sum.mass, whole.mass, 1e-9, "combine: two cubes weigh the box");
	CheckClose(Length(sum.centre - whole.centre), 0.0, 1e-9, "combine: joint centre is the box's");
	for (int i = 0; i < 9; ++i) {
		CheckClose(sum.inertia[i], whole.inertia[i], 1e-9, "combine: inertia of the two cubes is the box's, entry " + std::to_string(i));
	}
	CheckClose(sum.volume, 2.0, 1e-9, "combine: volumes add");

	// Unequal masses: the centre is mass-weighted, and lies between.
	const MassProperties heavy = SolidMassProperties(left, 3.0 * rho);
	const MassProperties light = SolidMassProperties(right, rho);
	const MassProperties uneven = CombineMassProperties({heavy, light});
	CheckClose(uneven.centre.x, (3.0 * -0.5 + 1.0 * 0.5) / 4.0, 1e-9, "combine: mass-weighted centre");

	// Nothing to combine is nothing.
	const MassProperties none = CombineMassProperties({});
	CheckClose(none.mass, 0.0, 0.0, "combine: empty is zero mass");
}

void CheckModeNames() {
	Check(ParseMassMode("asserted") == MassMode::Asserted && ParseMassMode("Density") == MassMode::Density
		&& ParseMassMode("") == MassMode::Unset, "mass mode names parse, case-insensitively");
	Check(std::string(MassModeName(MassMode::Asserted)) == "asserted", "mass mode name");
	bool threw = false;
	try { ParseMassMode("heavy"); } catch (const std::invalid_argument&) { threw = true; }
	Check(threw, "an unknown mass mode is refused");
}

} // namespace

int main() {
	try {
		CheckSolidBox();
		CheckShellBox();
		CheckSingleTriangle();
		CheckSphereAndCylinder();
		CheckInvariances();
		CheckCombine();
		CheckModeNames();
	} catch (const std::exception& e) {
		std::cout << "UNEXPECTED EXCEPTION: " << e.what() << "\n";
		return 1;
	}
	std::cout << "\nRESULT: " << g_pass << " passed, " << g_fail << " failed\n";
	return g_fail == 0 ? 0 : 1;
}
