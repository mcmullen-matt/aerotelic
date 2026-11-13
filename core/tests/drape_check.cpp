// Checks drape.hpp, the pin-jointed net: a plane and a cylinder are
// developable, so the net lies on them without shear and the warp keeps
// its direction; a sphere has Gaussian curvature 1 / R^2, so the shear
// angle grows from zero on the two generators as K u v (the Chebyshev net
// equation to first order) along the diagonal; the net stops at the free
// edge of an open patch and does not run onto the far skin of a thin body.

#include <cmath>
#include <cstdio>
#include <string>

#include "aerotelic/drape.hpp"
#include "aerotelic/mesh.hpp"

using namespace aerotelic;

namespace {

int g_failures = 0;

void Check(bool ok, const std::string& what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what.c_str());
	if (!ok) ++g_failures;
}

std::string Fmt(double v) {
	char buf[64];
	std::snprintf(buf, sizeof buf, "%.4g", v);
	return buf;
}

constexpr double kPi = 3.14159265358979323846;

Vec3 Centroid(const TriMesh& m, size_t t) {
	const auto& tri = m.triangles[t];
	return (m.vertices[tri[0]] + m.vertices[tri[1]] + m.vertices[tri[2]]) / 3.0;
}

// A flat square patch in the z = 0 plane, open.
TriMesh Patch(double side, int n) {
	TriMesh m;
	for (int j = 0; j <= n; ++j)
		for (int i = 0; i <= n; ++i) m.vertices.push_back({-side / 2 + side * i / n, -side / 2 + side * j / n, 0.0});
	for (int j = 0; j < n; ++j)
		for (int i = 0; i < n; ++i) {
			const int a = j * (n + 1) + i, b = a + 1, c = a + n + 2, d = a + n + 1;
			m.triangles.push_back({a, b, c});
			m.triangles.push_back({a, c, d});
		}
	return m;
}

} // namespace

int main() {
	// ---- a plane ----
	{
		const TriMesh patch = Patch(2.0, 20);
		DrapeOptions o;
		o.seed = {0.0, 0.0, 0.0};
		o.warp = {1.0, 1.0, 0.0};   // 45 degrees to the grid, to exercise the walk across edges
		o.cell = 0.1;
		const DrapeResult d = DrapeNet(patch, o);
		int covered = 0;
		double max_shear = 0.0, max_dev = 0.0;
		const Vec3 w = Normalized(Vec3{1.0, 1.0, 0.0});
		for (size_t t = 0; t < patch.triangles.size(); ++t) {
			if (!d.covered[t]) continue;
			++covered;
			max_shear = std::max(max_shear, std::fabs(d.shear[t]));
			max_dev = std::max(max_dev, std::acos(std::min(1.0, std::fabs(Dot(d.warp[t], w)))) * 180.0 / kPi);
		}
		Check(d.cells > 100 && covered > static_cast<int>(0.5 * patch.triangles.size()), "plane: the net grows over the patch (" + std::to_string(d.nodes) + " nodes, " + std::to_string(d.cells) + " cells, " + std::to_string(covered) + " of " + std::to_string(patch.triangles.size()) + " triangles)");
		Check(max_shear < 1e-6, "plane: no shear (max " + Fmt(max_shear) + " deg)");
		Check(max_dev < 1e-4, "plane: the warp keeps its direction (max deviation " + Fmt(max_dev) + " deg)");
		// The net stops at the free edge: no node beyond the patch.
		double furthest = 0.0;
		for (const auto& nd : d.net) furthest = std::max({furthest, std::fabs(nd.p.x), std::fabs(nd.p.y)});
		Check(furthest <= 1.0 + 1e-9, "plane: no node leaves the patch (furthest " + Fmt(furthest) + ")");
		// Pivots are L apart along both families.
		double worst = 0.0;
		for (const auto& a : d.net) for (const auto& b : d.net) {
			if ((a.i == b.i && b.j == a.j + 1) || (a.j == b.j && b.i == a.i + 1)) worst = std::max(worst, std::fabs(Distance(a.p, b.p) - o.cell));
		}
		Check(worst < 1e-7, "plane: neighbouring pivots are the pitch apart (worst " + Fmt(worst) + ")");
		const std::vector<Vec3> ref(patch.triangles.size(), Vec3{1.0, 0.0, 0.0});
		const std::vector<double> dev = WarpDeviation(patch, d, ref);
		double dmin = 1e9, dmax = -1e9;
		for (size_t t = 0; t < dev.size(); ++t) if (d.covered[t]) { dmin = std::min(dmin, dev[t]); dmax = std::max(dmax, dev[t]); }
		Check(std::fabs(dmin - 45.0) < 1e-4 && std::fabs(dmax - 45.0) < 1e-4, "plane: the warp deviates 45 degrees from the x reference (" + Fmt(dmin) + " to " + Fmt(dmax) + ")");
	}

	// ---- a cylinder, axis x ----
	{
		const double R = 1.0, Lc = 6.0;
		const TriMesh cyl = CylinderMesh(R, Lc, 96, 60, 4);
		DrapeOptions o;
		o.seed = {0.0, 0.0, R};   // on the side, at mid-length
		o.warp = {1.0, 0.0, 0.0};  // along the axis
		o.cell = 0.1;
		o.max_steps = 60;
		const DrapeResult d = DrapeNet(cyl, o);
		int side = 0, covered = 0;
		double max_shear = 0.0, max_dev = 0.0;
		for (size_t t = 0; t < cyl.triangles.size(); ++t) {
			const Vec3 c = Centroid(cyl, t);
			const Vec3 n = cyl.TriangleNormal(static_cast<int>(t));
			if (std::fabs(n.x) > 0.5 || std::fabs(c.x) > 0.4 * Lc) continue;   // the side, away from the caps
			++side;
			if (!d.covered[t]) continue;
			++covered;
			max_shear = std::max(max_shear, std::fabs(d.shear[t]));
			max_dev = std::max(max_dev, std::acos(std::min(1.0, std::fabs(d.warp[t].x))) * 180.0 / kPi);
		}
		Check(covered > 0.9 * side, "cylinder: the net wraps the side (" + std::to_string(covered) + " of " + std::to_string(side) + " side triangles; " + std::to_string(d.nodes) + " nodes)");
		// The mesh is 96 facets round (3.75 degrees each): the step-and-
		// project walk kinks a little at every facet edge.
		Check(max_shear < 2.0, "cylinder: a developable surface takes the net without shear, within the faceting (max " + Fmt(max_shear) + " deg)");
		Check(max_dev < 2.0, "cylinder: the warp stays along the axis (max deviation " + Fmt(max_dev) + " deg)");
		Check(d.covered_fraction > 0.5, "cylinder: most of the surface is covered (" + Fmt(100.0 * d.covered_fraction) + "%)");
		// Round the circumference the net meets itself and stops: fewer
		// than a full turn plus a cell of weft nodes either way.
		int jmax = 0;
		for (const auto& nd : d.net) jmax = std::max(jmax, std::abs(nd.j));
		Check(jmax * o.cell < kPi * R + 2 * o.cell, "cylinder: the weft stops where the net would overlap itself (|j| <= " + std::to_string(jmax) + ", half the circumference is " + Fmt(kPi * R / o.cell) + " cells)");
	}

	// ---- a sphere ----
	{
		const double R = 1.0;
		const TriMesh sph = EllipsoidMesh(R, R, R, 2e-4);
		DrapeOptions o;
		o.seed = {0.0, 0.0, R};
		o.warp = {1.0, 0.0, 0.0};
		o.cell = 0.05;
		o.max_steps = 24;   // 1.2 R along each generator
		const DrapeResult d = DrapeNet(sph, o);
		Check(d.nodes > 1000, "sphere: the net grows from the pole (" + std::to_string(d.nodes) + " nodes, " + std::to_string(d.cells) + " cells)");
		// Shear on the generators is zero, and on the diagonal it grows
		// as (s / R)^2 to first order: at i = j = n the arcs are n L.
		auto node = [&](int i, int j) -> const DrapeResult::Node* {
			for (const auto& nd : d.net) if (nd.i == i && nd.j == j) return &nd;
			return nullptr;
		};
		auto shear_at = [&](int i, int j) {
			// The angle between warp and weft at the node from its four neighbours.
			const auto* c = node(i, j);
			const auto* a = node(i + 1, j);
			const auto* b = node(i, j + 1);
			const auto* am = node(i - 1, j);
			const auto* bm = node(i, j - 1);
			if (!c || !a || !b || !am || !bm) return 1e9;
			const Vec3 w = Normalized(a->p - am->p), f = Normalized(b->p - bm->p);
			return std::fabs(90.0 - std::acos(std::clamp(Dot(w, f), -1.0, 1.0)) * 180.0 / kPi);
		};
		const double g0 = std::fabs(shear_at(6, 0)), g0b = std::fabs(shear_at(0, -6));
		Check(g0 < 0.5 && g0b < 0.5, "sphere: no shear on the generators (" + Fmt(g0) + ", " + Fmt(g0b) + " deg)");
		bool monotonic = true, symmetric = true, first_order = true;
		std::string trace, quadrants;
		for (int n = 2; n <= 8; n += 2) {
			const double s = n * o.cell;
			const double predicted = (s / R) * (s / R) * 180.0 / kPi;
			const double g = shear_at(n, n), gq = shear_at(-n, n), gr = shear_at(-n, -n);
			trace += Fmt(g) + "/" + Fmt(predicted) + " ";
			quadrants += Fmt(g) + "," + Fmt(gq) + "," + Fmt(gr) + " ";
			// The loft's triangulation of the sphere is not the same in every
			// quadrant, so the four agree to the faceting, not to the bit.
			if (g < 1e8 && gq < 1e8 && gr < 1e8) {
				symmetric = symmetric && std::fabs(g - gq) < 0.15 * std::fabs(g) + 0.3 && std::fabs(g - gr) < 0.15 * std::fabs(g) + 0.3;
			} else {
				symmetric = false;
			}
			if (n > 2) monotonic = monotonic && g > shear_at(n - 2, n - 2);
			// Within 25% of K u v for arcs up to 0.4 R (second order terms grow after).
			if (s <= 0.4 * R + 1e-9) first_order = first_order && std::fabs(g - predicted) < 0.25 * predicted + 0.05;
		}
		Check(first_order, "sphere: the diagonal shear is K u v to first order (got/predicted deg: " + trace + ")");
		Check(monotonic, "sphere: the shear grows along the diagonal");
		Check(symmetric, "sphere: the four quadrants shear alike (" + quadrants + ")");
		Check(d.max_shear > 5.0, "sphere: a cloth draped 1.2 R from the pole shears appreciably (max " + Fmt(d.max_shear) + " deg)");
		Check(shear_at(6, 6) > 1.0, "sphere: the fibres have moved from 90 degrees at 6, 6 (" + Fmt(shear_at(6, 6)) + " deg)");
	}

	// ---- a thin box: the net on the top face does not colour the bottom face ----
	{
		const TriMesh box = BoxMesh(2.0, 2.0, 0.05, 20, 20, 1);
		DrapeOptions o;
		o.seed = {0.0, 0.0, 0.025};
		o.warp = {1.0, 0.0, 0.0};
		o.cell = 0.1;
		o.max_steps = 8;   // stay on the top face
		const DrapeResult d = DrapeNet(box, o);
		int top = 0, top_covered = 0, bottom_covered = 0;
		for (size_t t = 0; t < box.triangles.size(); ++t) {
			const Vec3 n = box.TriangleNormal(static_cast<int>(t));
			const Vec3 c = Centroid(box, t);
			if (n.z > 0.5 && std::fabs(c.x) < 0.7 && std::fabs(c.y) < 0.7) { ++top; top_covered += d.covered[t] ? 1 : 0; }
			if (n.z < -0.5) bottom_covered += d.covered[t] ? 1 : 0;
		}
		Check(top_covered == top && top > 0, "thin box: the top face under the net is covered (" + std::to_string(top_covered) + " of " + std::to_string(top) + ")");
		Check(bottom_covered == 0, "thin box: the bottom face, 50 mm below, is not (" + std::to_string(bottom_covered) + " triangles)");
	}

	// ---- degenerate input ----
	{
		const TriMesh patch = Patch(1.0, 4);
		DrapeOptions o;
		o.seed = {0.0, 0.0, 0.0};
		o.warp = {0.0, 0.0, 1.0};   // normal to the patch
		const DrapeResult d = DrapeNet(patch, o);
		Check(d.cells == 0 && !d.notes.empty(), "a warp normal to the surface lays no net and says so: " + (d.notes.empty() ? std::string() : d.notes[0]));
		bool threw = false;
		try { DrapeNet(TriMesh{}, o); } catch (const std::invalid_argument&) { threw = true; }
		Check(threw, "an empty surface is refused");
	}

	std::printf("RESULT: %d failure(s)\n", g_failures);
	return g_failures == 0 ? 0 : 1;
}
