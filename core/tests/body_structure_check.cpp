// Checks body_structure.hpp: the plane networks' arithmetic (offsets,
// placement, mirroring, the implicit field), the validation, what counts
// as the same geometry, and the plane geometry shared by the preview and
// the generators (slab, rectangle, section loops) on a box.

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

constexpr double kPi = 3.14159265358979323846;

} // namespace

int main() {
	// ---- the network's planes ----
	{
		PlaneNetwork n;
		n.axis = PlaneAxis::Y;
		n.start = 0.05;
		n.spacing = 0.1;
		n.count = 4;
		Check(Near(PlaneOffset(n, 0), 0.05, 1e-15) && Near(PlaneOffset(n, 3), 0.35, 1e-15), "plane offsets: start + k spacing");
		Check(Near(NetworkPlane(n, 2).normal, Vec3{0, 1, 0}, 0.0) && Near(NetworkPlane(n, 2).offset, 0.25, 1e-15), "network plane: y normal, third plane at 0.25");
		PlaneNetwork c = n;
		c.axis = PlaneAxis::Custom;
		c.normal = Vec3{3.0, 4.0, 0.0};
		Check(Near(NetworkNormal(c), Vec3{0.6, 0.8, 0.0}, 1e-15), "custom normal is normalised");
		Check(Near(NetworkNormal(n), Vec3{0, 1, 0}, 0.0), "an axis ignores the custom vector");

		// Placed by a quarter turn about z, moved by (1, 2, 3) and scaled by 2:
		// the local plane y = 0.25 (normal +y) becomes normal -x at ... the
		// point (0, 0.25, 0) goes to R (0, 0.25, 0) * 2 + T = (-0.5, 0, 0) + T.
		const std::array<double, 9> Rz{0, -1, 0, 1, 0, 0, 0, 0, 1};   // rotation +90 deg about z
		const Plane placed = PlacedPlane(NetworkPlane(n, 2), Rz, Vec3{1, 2, 3}, 2.0);
		Check(Near(placed.normal, Vec3{-1, 0, 0}, 1e-15), "placed plane: the normal turns with the frame");
		Check(Near(placed.Distance(Vec3{0.5, 7.0, -2.0}), 0.0, 1e-12), "placed plane: passes through the placed, scaled point");
		const Plane m = MirrorPlaneY(Plane{Vec3{0.6, 0.8, 0.0}, 2.0});
		Check(Near(m.normal, Vec3{0.6, -0.8, 0.0}, 0.0) && m.offset == 2.0, "mirror in y: the normal's y flips, the offset stays");

		// The implicit field: negative inside a slab, positive between
		// planes, clamped beyond the last plane.
		n.thickness = 0.02;
		Check(PlaneNetworkField(n, Vec3{0, 0.25, 0}) < 0.0 && Near(PlaneNetworkField(n, Vec3{0, 0.25, 0}), -0.01, 1e-15), "field: a point on the third plane is -t/2");
		Check(Near(PlaneNetworkField(n, Vec3{0, 0.30, 0}), 0.04, 1e-15), "field: half way between planes is spacing/2 - t/2");
		Check(Near(PlaneNetworkField(n, Vec3{0, 1.35, 0}), 0.99, 1e-12), "field: beyond the last plane the last plane is the nearest");
		Check(Near(PlaneNetworkField(n, Vec3{0, -1.0, 0}), 1.04, 1e-12), "field: before the first plane the first is the nearest");
	}

	// ---- validation ----
	{
		PlaneNetwork n;
		Check(PlaneNetworkProblem(n).empty(), "a default network is fine");
		n.count = 0;
		Check(!PlaneNetworkProblem(n).empty(), "no planes is refused");
		n.count = 3; n.spacing = 0.001; n.thickness = 0.002;
		Check(!PlaneNetworkProblem(n).empty(), "planes thicker than their spacing are refused");
		n.spacing = 0.1; n.height = -1.0;
		Check(!PlaneNetworkProblem(n).empty(), "a negative height is refused");
		n.height = 0.0; n.axis = PlaneAxis::Custom; n.normal = Vec3{0, 0, 0};
		Check(!PlaneNetworkProblem(n).empty(), "a zero custom normal is refused");
		n.normal = Vec3{1, 1, 0};
		Check(PlaneNetworkProblem(n).empty(), "a custom normal of any length is fine");

		BodyStructure s;
		Check(!s.IsSet() && BodyStructureProblem(s).empty(), "an empty structure is unset and has no problem");
		s.material.density = 2700.0;
		Check(s.IsSet() && !s.IsSkin() && BodyStructureProblem(s).empty(), "a material alone is a solid");
		s.networks.push_back(n);
		Check(!BodyStructureProblem(s).empty(), "a network on a solid body is refused (no interior)");
		s.skin_thickness = 0.002;
		Check(s.IsSkin() && s.HasInterior() && BodyStructureProblem(s).empty(), "a skinned body may carry a network");
		s.networks[0].count = 0;
		Check(BodyStructureProblem(s).find("network '" + n.name + "'") == 0, "a network's problem names the network");
		s.networks.clear();
		s.skin_thickness = -0.1;
		Check(!BodyStructureProblem(s).empty(), "a negative skin thickness is refused");
		s.skin_thickness = 0.0;
		s.asserted_mass = -2.0;
		Check(!BodyStructureProblem(s).empty(), "a negative asserted mass is refused");
	}

	// ---- what counts as the same geometry ----
	{
		BodyStructure a;
		a.material.density = 2700.0;
		a.skin_thickness = 0.002;
		a.networks.push_back(PlaneNetwork{});
		BodyStructure b = a;
		Check(SameStructureGeometry(a, b), "a copy has the same geometry");
		b.material.density = 160.0;
		b.networks[0].material.density = 1240.0;
		b.asserted_mass = 1.0;
		Check(SameStructureGeometry(a, b), "materials and an asserted mass do not change the geometry");
		b = a; b.skin_thickness = 0.003;
		Check(!SameStructureGeometry(a, b), "the skin thickness does");
		b = a; b.networks[0].spacing += 0.01;
		Check(!SameStructureGeometry(a, b), "a network's spacing does");
		b = a; b.networks[0].height = 0.01;
		Check(!SameStructureGeometry(a, b), "a network's height does");
		b = a; b.exclusions.push_back("Bay");
		Check(!SameStructureGeometry(a, b), "the exclusions do");
	}

	// ---- the slab, the rectangle and the section, on a box ----
	{
		Vec3 u, v;
		const Vec3 n = Normalized(Vec3{1.0, 2.0, -0.5});
		PlaneBasis(n, &u, &v);
		Check(Near(Length(u), 1.0, 1e-15) && Near(Length(v), 1.0, 1e-15) && Near(Dot(u, v), 0.0, 1e-15) && Near(Cross(u, v), n, 1e-15),
			"plane basis: orthonormal with u x v = n");

		const Plane p{n, 0.3};
		const TriMesh slab = SlabMesh(p, 0.01, Vec3{1, 1, 1}, 0.5);
		Check(slab.IsClosedManifold() && slab.SignedVolume() > 0.0, "slab: closed and outward");
		Check(Near(slab.SignedVolume(), 1.0 * 1.0 * 0.01, 1e-12), "slab: volume = (2 half_extent)^2 thickness");
		double worst = 0.0;
		for (const Vec3& x : slab.vertices) worst = std::max(worst, std::fabs(std::fabs(p.Distance(x)) - 0.005));
		Check(worst < 1e-12, "slab: every vertex is t/2 off the plane");

		Box3 box;
		box.Add(Vec3{-1, -1, -1});
		box.Add(Vec3{1, 1, 1});
		std::array<Vec3, 4> corners;
		Check(PlaneBoxRectangle(Plane{Vec3{0, 0, 1}, 0.5}, box, &corners), "rectangle: a plane through the box is drawn");
		double area = 0.0;
		for (int i = 0; i < 4; ++i) {
			const Vec3 a = corners[i], b = corners[(i + 1) % 4];
			area += a.x * b.y - b.x * a.y;
		}
		Check(Near(std::fabs(area) * 0.5, 4.0, 1e-12) && Near(corners[0].z, 0.5, 1e-15), "rectangle: covers the box's shadow (area 4) on the plane");
		Check(!PlaneBoxRectangle(Plane{Vec3{0, 0, 1}, 2.0}, box, &corners), "rectangle: a plane missing the box is not drawn");

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
		// Seen from +z the loop runs counter-clockwise: the z component of
		// (a x b) summed is positive.
		Check(s.areas.size() == 1 && s.areas[0] > 0.0, "section: counter-clockwise about the normal (material to the left)");
		// Cut from the other side the loop runs the other way.
		PlaneSection flipped = SectionByPlane(b246, Plane{Vec3{0, 0, -1}, -1.0});
		Check(flipped.loops.size() == 1 && flipped.areas[0] > 0.0 && Near(flipped.Area(), 8.0, 1e-12), "section: the same plane from the other side is still an outer loop");
		// An oblique cut through the centre of a 2 x 2 x 2 box: a rectangle
		// 2 sqrt(2) by 2.
		const TriMesh b222 = BoxMesh(2.0, 2.0, 2.0);
		PlaneSection oblique = SectionByPlane(b222, Plane{Normalized(Vec3{1, 1, 0}), 0.0});
		Check(oblique.loops.size() == 1 && Near(oblique.Area(), 4.0 * std::sqrt(2.0), 1e-12), "section: an oblique cut through a cube is a 2 sqrt 2 x 2 rectangle");
		// A plane lying on the top face (z = 3) is, by the on-plane-is-positive
		// rule, the plane a hair inside the body: the face's own outline.
		PlaneSection face = SectionByPlane(b246, Plane{Vec3{0, 0, 1}, 3.0});
		Check(face.loops.size() == 1 && Near(face.Area(), 8.0, 1e-12), "section: a plane on a face gives that face's outline (the plane a hair inside)");
		Check(SectionByPlane(b246, Plane{Vec3{0, 0, 1}, 5.0}).loops.empty(), "section: a plane missing the body has no loops");
		// Two boxes: two loops, areas summed.
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
