// Checks body_structure.hpp: the validation, what counts as the same
// geometry, and the plane geometry the mass report and the mesh cutter
// share (basis, section loops) on a box. (The plane networks' arithmetic
// that was checked here went with the networks on 2026-11-10.)

#include <cmath>
#include <cstdio>
#include <string>

#include "aerotelic/body_structure.hpp"

using namespace aerotelic;

namespace {

int g_failures = 0;

void Check(bool ok, const std::string& what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what.c_str());
	if (!ok) ++g_failures;
}

bool Near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }
bool Near(const Vec3& a, const Vec3& b, double tol) { return Distance(a, b) <= tol; }

} // namespace

int main() {
	// ---- validation ----
	{
		BodyStructure s;
		Check(!s.IsSet() && BodyStructureProblem(s).empty(), "an empty structure is unset and has no problem");
		s.material.density = 2700.0;
		Check(s.IsSet() && !s.IsSkin() && BodyStructureProblem(s).empty(), "a material alone is a solid");
		s.skin_thickness = 0.002;
		Check(s.IsSkin() && BodyStructureProblem(s).empty(), "a material with a skin thickness is a skin");
		s.skin_thickness = -0.1;
		Check(!BodyStructureProblem(s).empty(), "a negative skin thickness is refused");
		s.skin_thickness = 0.0;
		s.asserted_mass = -2.0;
		Check(!BodyStructureProblem(s).empty(), "a negative asserted mass is refused");
		s.asserted_mass = 3.0;
		Check(s.IsSet() && BodyStructureProblem(s).empty(), "an asserted mass sets the structure");
	}

	// ---- what counts as the same geometry ----
	{
		BodyStructure a;
		a.material.density = 2700.0;
		a.skin_thickness = 0.002;
		BodyStructure b = a;
		Check(SameStructureGeometry(a, b), "a copy has the same geometry");
		b.material.density = 160.0;
		b.asserted_mass = 1.0;
		Check(SameStructureGeometry(a, b), "materials and an asserted mass do not change the geometry");
		b = a; b.skin_thickness = 0.003;
		Check(!SameStructureGeometry(a, b), "the skin thickness does");
		b = a; b.exclusions.push_back("Bay");
		Check(!SameStructureGeometry(a, b), "the exclusions do");
	}

	// ---- the basis and the section, on a box ----
	{
		Vec3 u, v;
		const Vec3 n = Normalized(Vec3{1.0, 2.0, -0.5});
		PlaneBasis(n, &u, &v);
		Check(Near(Length(u), 1.0, 1e-15) && Near(Length(v), 1.0, 1e-15) && Near(Dot(u, v), 0.0, 1e-15) && Near(Cross(u, v), n, 1e-15),
			"plane basis: orthonormal with u x v = n");

		// A 2 x 4 x 6 box cut by z = 1: one loop, area 8, counter-clockwise
		// about +z (outer).
		const TriMesh b246 = BoxMesh(2.0, 4.0, 6.0);
		PlaneSection s = SectionByPlane(b246, Plane{Vec3{0, 0, 1}, 1.0});
		Check(s.loops.size() == 1 && s.outer.size() == 1 && s.outer[0], "section: a box cut by a plane gives one outer loop");
		Check(Near(s.Area(), 8.0, 1e-12), "section: the loop's area is the box's cross-section");
		Check(Near(s.Perimeter(), 12.0, 1e-12), "section: the loop's perimeter");
		bool on_plane = true;
		for (const Vec3& q : s.loops.empty() ? std::vector<Vec3>{} : s.loops[0]) on_plane = on_plane && Near(q.z, 1.0, 1e-15);
		Check(on_plane, "section: every loop point lies on the plane");
		Check(s.areas.size() == 1 && s.areas[0] > 0.0, "section: counter-clockwise about the normal (material to the left)");
		PlaneSection flipped = SectionByPlane(b246, Plane{Vec3{0, 0, -1}, -1.0});
		Check(flipped.loops.size() == 1 && flipped.areas[0] > 0.0 && Near(flipped.Area(), 8.0, 1e-12), "section: the same plane from the other side is still an outer loop");
		const TriMesh b222 = BoxMesh(2.0, 2.0, 2.0);
		PlaneSection oblique = SectionByPlane(b222, Plane{Normalized(Vec3{1, 1, 0}), 0.0});
		Check(oblique.loops.size() == 1 && Near(oblique.Area(), 4.0 * std::sqrt(2.0), 1e-12), "section: an oblique cut through a cube is a 2 sqrt 2 x 2 rectangle");
		PlaneSection face = SectionByPlane(b246, Plane{Vec3{0, 0, 1}, 3.0});
		Check(face.loops.size() == 1 && Near(face.Area(), 8.0, 1e-12), "section: a plane on a face gives that face's outline (the plane a hair inside)");
		Check(SectionByPlane(b246, Plane{Vec3{0, 0, 1}, 5.0}).loops.empty(), "section: a plane missing the body has no loops");
		TriMesh two = b222;
		TriMesh other = BoxMesh(1.0, 1.0, 1.0);
		other.Translate(Vec3{5.0, 0.0, 0.0});
		two.Append(other);
		PlaneSection pair = SectionByPlane(two, Plane{Vec3{0, 0, 1}, 0.0});
		Check(pair.loops.size() == 2 && Near(pair.Area(), 5.0, 1e-12), "section: two bodies give two loops whose areas sum");
	}

	if (g_failures == 0) std::printf("RESULT: 0 failure(s)\n");
	else std::printf("RESULT: %d failure(s)\n", g_failures);
	return g_failures == 0 ? 0 : 1;
}
