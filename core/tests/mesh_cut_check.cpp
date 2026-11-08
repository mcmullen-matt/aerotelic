// Checks of mesh_cut.hpp: a box cut by planes that split triangles, that
// run through a row of vertices, and that run through vertices on the
// diagonal; closed in, closed out, the volume unchanged; one loop per
// plane with the analytic area; crossing planes share vertices.

#include <cmath>
#include <iostream>
#include <set>
#include <string>

#include "aerotelic/mesh_cut.hpp"

using namespace aerotelic;

namespace {

int g_pass = 0, g_fail = 0;

void Check(bool cond, const std::string& msg) {
	if (cond) ++g_pass;
	else { ++g_fail; std::cerr << "FAIL: " << msg << "\n"; }
}

void CheckClose(double a, double b, double tol, const std::string& msg) {
	Check(std::abs(a - b) <= tol, msg + " (" + std::to_string(a) + " vs " + std::to_string(b) + ")");
}

Plane Unit(Vec3 n, double d) {
	const double l = Length(n);
	return Plane{n / l, d / l};
}

} // namespace

int main() {
	// A 2 x 2 x 2 box: twelve triangles, vertices at +-1 (BoxMesh centres
	// on the origin; checked below rather than assumed).
	TriMesh box = BoxMesh(2.0, 2.0, 2.0);
	const Box3 b = box.Bounds();
	const Vec3 c = b.Center();
	box.Translate(c * -1.0);
	Check(box.IsClosedManifold() && box.SignedVolume() > 0.0, "the box is closed and outward");
	const double volume = box.SignedVolume();
	CheckClose(volume, 8.0, 1e-12, "the box's volume is 8");

	// One plane splitting triangles (x = 0.3), one through a row of
	// vertices (z = 1 is a face: skipped, use y = 0 through no vertex),
	// one through the diagonal vertices (x + y = 0).
	const std::vector<Plane> planes{Unit({1, 0, 0}, 0.3), Unit({0, 1, 0}, 0.0), Unit({1, 1, 0}, 0.0)};
	MeshCutStats stats;
	std::vector<int> parent;
	const TriMesh cut = CutMeshByPlanes(box, planes, 1e-9, &parent, &stats);
	Check(cut.IsClosedManifold(), "the cut box is a closed manifold");
	CheckClose(cut.SignedVolume(), volume, 1e-9, "the cut box's volume is unchanged");
	Check(parent.size() == cut.triangles.size(), "a parent per output triangle");
	bool parents_ok = true;
	for (int pidx : parent) parents_ok = parents_ok && pidx >= 0 && pidx < static_cast<int>(box.triangles.size());
	Check(parents_ok, "every parent is an input triangle");
	Check(stats.split_triangles > 0 && stats.new_vertices > 0, "triangles were split and crossing points added");
	Check(cut.triangles.size() > box.triangles.size(), "the cut has more triangles than the box");

	// The loops: one per plane, the square section's area (2 x 2 = 4 for
	// the axis planes; the diagonal's section is 2 sqrt 2 by 2).
	{
		const auto loops = SectionLoopsOnPlane(cut, planes[0], 1e-9);
		Check(loops.size() == 1, "x = 0.3: one loop (" + std::to_string(loops.size()) + ")");
		if (loops.size() == 1) {
			CheckClose(LoopArea(cut, loops[0], planes[0]), 4.0, 1e-9, "x = 0.3: the loop's area is the section's, counter-clockwise");
			// Every consecutive pair is an edge of the cut mesh.
			std::set<std::pair<int, int>> edges;
			for (const auto& t : cut.triangles) for (int e = 0; e < 3; ++e) edges.insert({std::min(t[e], t[(e + 1) % 3]), std::max(t[e], t[(e + 1) % 3])});
			bool all_edges = true;
			for (size_t i = 0; i < loops[0].size(); ++i) {
				const int a = loops[0][i], bb = loops[0][(i + 1) % loops[0].size()];
				all_edges = all_edges && edges.count({std::min(a, bb), std::max(a, bb)}) > 0;
			}
			Check(all_edges, "x = 0.3: the loop runs along mesh edges");
			// The loop's vertices sit on the plane.
			bool on_plane = true;
			for (int v : loops[0]) on_plane = on_plane && std::abs(planes[0].Distance(cut.vertices[v])) <= 1e-9;
			Check(on_plane, "x = 0.3: the loop's vertices lie on the plane");
		}
	}
	{
		const auto loops = SectionLoopsOnPlane(cut, planes[1], 1e-9);
		Check(loops.size() == 1, "y = 0: one loop");
		if (loops.size() == 1) CheckClose(LoopArea(cut, loops[0], planes[1]), 4.0, 1e-9, "y = 0: the loop's area is 4");
	}
	{
		const auto loops = SectionLoopsOnPlane(cut, planes[2], 1e-9);
		Check(loops.size() == 1, "x + y = 0: one loop through the diagonal vertices");
		if (loops.size() == 1) CheckClose(LoopArea(cut, loops[0], planes[2]), 2.0 * std::sqrt(2.0) * 2.0, 1e-9, "x + y = 0: the loop's area is 4 sqrt 2");
	}
	// Crossing planes share vertices: the line x = 0.3, y = 0 meets the
	// surface at z = +-1, and both loops pass through those two vertices.
	{
		const auto lx = SectionLoopsOnPlane(cut, planes[0], 1e-9);
		const auto ly = SectionLoopsOnPlane(cut, planes[1], 1e-9);
		std::set<int> sx, sy;
		for (const auto& l : lx) sx.insert(l.begin(), l.end());
		for (const auto& l : ly) sy.insert(l.begin(), l.end());
		int shared = 0;
		for (int v : sx) shared += sy.count(v) ? 1 : 0;
		Check(shared == 2, "the x and y loops share the two vertices on their line (" + std::to_string(shared) + ")");
	}
	// A plane on a face: nothing is split (every vertex of the face is
	// snapped, none crosses), the volume stands, and the plane's loop is
	// the face's outline (on-plane counts as positive, as SectionByPlane).
	{
		MeshCutStats s2;
		const TriMesh cut2 = CutMeshByPlanes(box, {Unit({0, 0, 1}, 1.0)}, 1e-9, nullptr, &s2);
		Check(s2.split_triangles == 0 && cut2.triangles.size() == box.triangles.size(), "a plane on a face splits nothing");
		CheckClose(cut2.SignedVolume(), volume, 1e-12, "a plane on a face leaves the volume");
	}
	// A plane missing the box: no loops, nothing split.
	{
		MeshCutStats s3;
		const TriMesh cut3 = CutMeshByPlanes(box, {Unit({1, 0, 0}, 5.0)}, 1e-9, nullptr, &s3);
		Check(s3.split_triangles == 0 && SectionLoopsOnPlane(cut3, Unit({1, 0, 0}, 5.0), 1e-9).empty(), "a plane past the box cuts nothing and has no loop");
	}
	// Snapping: a plane within the tolerance of a row of vertices moves them
	// onto it rather than slicing a sliver off.
	{
		MeshCutStats s4;
		const TriMesh cut4 = CutMeshByPlanes(box, {Unit({1, 0, 0}, 1.0 - 1e-7)}, 1e-6, nullptr, &s4);
		Check(s4.snapped_vertices == 4 && s4.split_triangles == 0, "a plane a hair inside a face snaps its four vertices and splits nothing");
		CheckClose(cut4.SignedVolume(), volume, 1e-5, "the snap moves the volume by no more than the tolerance");
	}

	std::cout << "RESULT: " << g_pass << " passed, " << g_fail << " failed\n";
	return g_fail == 0 ? 0 : 1;
}
