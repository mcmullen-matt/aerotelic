// Checks the lattice generator (lattice.hpp): the volume-fraction
// calibration of every TPMS and strut type, marching tetrahedra on an
// analytic field, and whole-body generation inside a box and a sphere
// with a reserved volume.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "aerotelic/lattice.hpp"
#include "aerotelic/mesh.hpp"
#include "aerotelic/sdf.hpp"

using namespace aerotelic;

namespace {

int g_failures = 0;

void Check(bool ok, const std::string& what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what.c_str());
	if (!ok) ++g_failures;
}

constexpr double kPi = 3.14159265358979323846;

} // namespace

int Run();

int main() {
	setvbuf(stdout, nullptr, _IONBF, 0);
	try {
		return Run();
	} catch (const std::exception& e) {
		std::printf("FAIL: exception: %s\nRESULT: 1 failure(s)\n", e.what());
		return 1;
	}
}

int Run() {
	// ---- calibration: every type reaches its target fraction ----
	{
		for (const std::string& type : TpmsTypeNames()) {
			for (TpmsPhase phase : {TpmsPhase::Network, TpmsPhase::Sheet}) {
				LatticeSpec spec;
				spec.family = LatticeFamily::Tpms;
				spec.type = type;
				spec.phase = phase;
				spec.volume_fraction = 0.3;
				double level = CalibrateLevel(spec, nullptr, 32);
				double got = CellSolidFraction(spec, level, 32);
				char buf[160];
				std::snprintf(buf, sizeof buf, "TPMS %s %s: t = %.4f gives fraction %.3f (target 0.3)", type.c_str(), TpmsPhaseName(phase), level, got);
				Check(std::fabs(got - 0.3) < 0.02, buf);
			}
		}
		for (LatticeFamily fam : {LatticeFamily::Strut, LatticeFamily::Auxetic}) {
			for (const std::string& type : LatticeTypeNames(fam)) {
				LatticeSpec spec;
				spec.family = fam;
				spec.type = type;
				spec.volume_fraction = 0.2;
				double level = CalibrateLevel(spec, nullptr, 32);
				double got = CellSolidFraction(spec, level, 32);
				char buf[160];
				std::snprintf(buf, sizeof buf, "strut %s: R = %.4f gives fraction %.3f (target 0.2)", type.c_str(), level, got);
				Check(std::fabs(got - 0.2) < 0.02 && level > 0.0 && level < 0.5, buf);
			}
		}
		// Fraction is monotonic in the level for a gyroid network.
		LatticeSpec g;
		Check(CellSolidFraction(g, -0.5, 24) < CellSolidFraction(g, 0.0, 24) && CellSolidFraction(g, 0.0, 24) < CellSolidFraction(g, 0.5, 24),
			"gyroid network: fraction increases with t");
		Check(std::fabs(CellSolidFraction(g, 0.0, 32) - 0.5) < 0.01, "gyroid network: t = 0 is half the cell");
	}

	// ---- marching tetrahedra on an analytic sphere ----
	{
		LatticeGrid grid;
		grid.h = 0.05;
		grid.nx = grid.ny = grid.nz = 48;
		grid.origin = {-1.2, -1.2, -1.2};
		std::vector<float> values(grid.Nodes());
		for (int k = 0; k <= grid.nz; ++k)
			for (int j = 0; j <= grid.ny; ++j)
				for (int i = 0; i <= grid.nx; ++i) {
					Vec3 p = grid.origin + Vec3{i * grid.h, j * grid.h, k * grid.h};
					values[(static_cast<size_t>(k) * (grid.ny + 1) + j) * (grid.nx + 1) + i] = static_cast<float>(Length(p) - 1.0);
				}
		TriMesh m = MarchingTetrahedra(grid, values, 4);
		Check(m.IsClosedManifold(), "marching tetrahedra: sphere mesh is a closed manifold");
		double v = m.SignedVolume();
		char buf[160];
		std::snprintf(buf, sizeof buf, "marching tetrahedra: sphere volume %.4f within 1%% of %.4f and outward", v, 4.0 / 3.0 * kPi);
		Check(std::fabs(v - 4.0 / 3.0 * kPi) / (4.0 / 3.0 * kPi) < 0.01, buf);
		double worst = 0.0;
		for (const auto& p : m.vertices) worst = std::max(worst, std::fabs(Length(p) - 1.0));
		Check(worst < 2e-3, "marching tetrahedra: every vertex within 2e-3 of the sphere");
	}

	// ---- a gyroid in a box ----
	{
		TriMesh box = BoxMesh(2.0, 2.0, 2.0);
		MeshSdf box_sdf(box);
		LatticeSpec spec;
		spec.type = "Gyroid";
		spec.cell_size = 0.5;
		spec.volume_fraction = 0.3;
		spec.resolution = 16;
		LatticeDomain dom;
		dom.body = [&](const Vec3& p) { return box_sdf.Distance(p); };
		dom.bounds = box.Bounds();
		auto t0 = std::chrono::steady_clock::now();
		LatticeResult r = GenerateLattice(spec, dom);
		double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
		std::printf("gyroid in box: grid %dx%dx%d, %zu body evaluations of %zu nodes, %zu triangles, %.2f s\n", r.grid.nx, r.grid.ny, r.grid.nz,
			r.nodes_evaluated, r.grid.Nodes(), r.mesh.triangles.size(), dt);
		Check(!r.mesh.IsEmpty(), "gyroid in box: produced a mesh");
		Check(r.mesh.IsClosedManifold(), "gyroid in box: closed manifold");
		Check(r.mesh.SignedVolume() > 0.0, "gyroid in box: outward");
		char buf[160];
		std::snprintf(buf, sizeof buf, "gyroid in box: achieved fraction %.3f within 0.05 of 0.3", r.achieved_fraction);
		Check(std::fabs(r.achieved_fraction - 0.3) < 0.05, buf);
		// The polyhedral surface cuts the corners of the curved struts, so
		// the mesh volume runs a few percent under the sampled fraction at
		// 16 voxels per cell.
		double vol = r.mesh.SignedVolume();
		std::snprintf(buf, sizeof buf, "gyroid in box: mesh volume %.3f is about 0.3 of the box's 8 (within 0.2 of 2.4)", vol);
		Check(std::fabs(vol - 2.4) < 0.2, buf);
		Box3 mb = r.mesh.Bounds();
		Check(mb.min.x >= -1.0 - 1e-6 && mb.max.x <= 1.0 + 1e-6 && mb.min.z >= -1.0 - 1e-6 && mb.max.z <= 1.0 + 1e-6, "gyroid in box: stays inside the box");
		// Only the blocks straddling the box surface need the body; here
		// those are most of a small box, so just check that some were spared.
		Check(r.nodes_evaluated < r.grid.Nodes(), "gyroid in box: block skipping spared some body evaluations");
	}

	// ---- BCC struts in a sphere, minus a reserved sphere, with a skin ----
	{
		TriMesh sph = EllipsoidMesh(1.0, 1.0, 1.0, 2e-4);
		TriMesh hole = EllipsoidMesh(0.4, 0.4, 0.4, 2e-4);
		MeshSdf sph_sdf(sph), hole_sdf(hole);
		LatticeSpec spec;
		spec.family = LatticeFamily::Strut;
		spec.type = "BCC";
		spec.cell_size = 0.4;
		spec.volume_fraction = 0.25;
		spec.resolution = 12;
		spec.skin_thickness = 0.05;
		LatticeDomain dom;
		dom.body = [&](const Vec3& p) { return sph_sdf.Distance(p); };
		dom.exclusion = [&](const Vec3& p) { return hole_sdf.Distance(p); };
		dom.bounds = sph.Bounds();
		LatticeResult r = GenerateLattice(spec, dom);
		std::printf("BCC in sphere: %zu lattice triangles, %zu skin triangles\n", r.mesh.triangles.size(), r.skin.triangles.size());
		Check(r.mesh.IsClosedManifold(), "BCC in sphere: lattice is a closed manifold");
		Check(!r.skin.IsEmpty() && r.skin.IsClosedManifold() && r.skin.SignedVolume() > 0.0, "BCC in sphere: the skin is a closed, outward body of its own");
		double worst_out = 0.0, worst_in = 0.0, worst_lat_out = 0.0;
		for (const auto& p : r.skin.vertices) {
			worst_out = std::max(worst_out, Length(p) - 1.0);
			worst_in = std::max(worst_in, 0.4 - Length(p));
		}
		for (const auto& p : r.mesh.vertices) {
			worst_lat_out = std::max(worst_lat_out, Length(p) - (1.0 - spec.skin_thickness));
			worst_in = std::max(worst_in, 0.4 - Length(p));
		}
		Check(worst_out < r.grid.h, "BCC in sphere: no skin vertex outside the sphere");
		Check(worst_lat_out < r.grid.h, "BCC in sphere: the lattice stays inside the skin's inner offset");
		Check(worst_in < r.grid.h, "BCC in sphere: no vertex inside the reserved volume");
		// The skin body: points just inside the sphere are solid; points
		// deeper than the skin are not part of it.
		MeshSdf sk(r.skin);
		int solid = 0, deep_solid = 0, n = 0;
		for (int i = 0; i < 200; ++i) {
			double a = 2 * kPi * i / 200.0, b = kPi * (i % 17) / 17.0 - kPi / 2;
			Vec3 dir{std::cos(b) * std::cos(a), std::cos(b) * std::sin(a), std::sin(b)};
			++n;
			if (sk.Distance(dir * 0.975) < 0.0) ++solid;
			if (sk.Distance(dir * 0.85) < 0.0) ++deep_solid;
		}
		char buf[160];
		std::snprintf(buf, sizeof buf, "BCC in sphere: %d of %d points 25 mm under the surface are inside the skin body", solid, n);
		Check(solid >= n - 2, buf);
		std::snprintf(buf, sizeof buf, "BCC in sphere: %d of %d points 150 mm under the surface are inside the skin body (expected none)", deep_solid, n);
		Check(deep_solid == 0, buf);
		// The skin's volume is the two shells': 0.05 inside the sphere,
		// 4/3 pi (1 - 0.95^3) = 0.597, plus 0.05 around the reserved volume
		// (a payload bay keeps a solid wall too), 4/3 pi (0.45^3 - 0.4^3) =
		// 0.114; the inscribed polyhedra come out a little under.
		std::snprintf(buf, sizeof buf, "BCC in sphere: skin volume %.3f within 10%% of the two 0.05-thick shells' 0.711", r.skin.SignedVolume());
		Check(std::fabs(r.skin.SignedVolume() - 0.711) < 0.071, buf);
	}

	// ---- iso-surface phase with a wall thickness ----
	{
		TriMesh box = BoxMesh(1.0, 1.0, 1.0);
		MeshSdf box_sdf(box);
		LatticeSpec spec;
		spec.type = "Primitive";
		spec.phase = TpmsPhase::IsoSurface;
		spec.cell_size = 0.5;
		spec.wall_thickness = 0.04;
		spec.resolution = 20;
		LatticeDomain dom;
		dom.body = [&](const Vec3& p) { return box_sdf.Distance(p); };
		dom.bounds = box.Bounds();
		LatticeResult r = GenerateLattice(spec, dom);
		Check(r.mesh.IsClosedManifold() && r.mesh.SignedVolume() > 0.0, "iso-surface sheet in box: closed, outward");
		std::printf("iso-surface sheet: achieved fraction %.3f\n", r.achieved_fraction);
		Check(r.achieved_fraction > 0.05 && r.achieved_fraction < 0.5, "iso-surface sheet: a thin sheet, not solid or empty");
	}

	// ---- honeycomb cross-sections: the geometry the cells claim ----
	{
		LatticeSpec sq;
		sq.family = LatticeFamily::Honeycomb;
		sq.type = "Square";
		sq.cell_size = 1.0;
		HoneycombCell c = HoneycombCellFor(sq);
		Check(std::fabs(c.Inradius() - 0.5) < 1e-12 && std::fabs(c.TileArea() - 1.0) < 1e-12, "square honeycomb: a unit cell, half of it from wall to centre");
		Check(c.Distance(0.0, 0.37) < 1e-12 && std::fabs(c.Distance(0.5, 0.5) - 0.5) < 1e-12 && std::fabs(c.Distance(0.25, 0.5) - 0.25) < 1e-12,
			"square honeycomb: the walls are the lines x, y = integer");

		LatticeSpec rect = sq;
		rect.type = "Rectangle";
		rect.cell_aspect = 2.5;
		c = HoneycombCellFor(rect);
		Check(std::fabs(c.Inradius() - 0.5) < 1e-12 && std::fabs(c.TileArea() - 2.5) < 1e-12, "rectangular honeycomb: the aspect stretches the second side only");

		LatticeSpec hex = sq;
		hex.type = "Hexagon";
		c = HoneycombCellFor(hex);
		Check(std::fabs(c.Inradius() - 0.5) < 1e-12 && std::fabs(c.Distance(0.0, 0.0) - 0.5) < 1e-12,
			"hexagonal honeycomb: the cell size is the size across the flats");
		Check(c.Distance(0.5, 0.0) < 1e-12 && c.Distance(0.25, 0.25 * std::sqrt(3.0)) < 1e-12,
			"hexagonal honeycomb: a wall halfway to each of the six neighbours");

		LatticeSpec tri = sq;
		tri.type = "Triangle";
		c = HoneycombCellFor(tri);
		Check(std::fabs(c.Inradius() - 1.0 / (2 * std::sqrt(3.0))) < 1e-12, "triangular honeycomb: the incircle of a unit triangle");
		Check(c.Distance(0.0, 0.0) < 1e-12 && std::fabs(c.Distance(0.5, std::sqrt(3.0) / 6.0) - 1.0 / (2 * std::sqrt(3.0))) < 1e-12,
			"triangular honeycomb: the walls meet at the lattice points and the incentre is furthest from them");
	}

	// ---- honeycomb solid fraction against its closed form ----
	{
		// The wall distance is exact, so the void is each cell shrunk inward
		// by half the wall. A cell with an incircle (a square, a regular
		// hexagon, any triangle) shrinks to a similar cell, so the void is
		// (1 - t / 2r)^2 of the cell -- an independent check of the wall
		// distance, the sampled tile and the fraction together.
		struct Case { const char* type; double aspect; };
		const Case shrinks_similarly[] = {{"Square", 1.0}, {"Hexagon", 1.0}, {"Triangle", 1.0}, {"Triangle", 1.7}};
		for (const Case& k : shrinks_similarly) {
			LatticeSpec spec;
			spec.family = LatticeFamily::Honeycomb;
			spec.type = k.type;
			spec.cell_aspect = k.aspect;
			spec.cell_size = 1.0;
			spec.wall_sizing = WallSizing::Thickness;
			spec.wall_thickness = 0.12;
			const double r = HoneycombCellFor(spec).Inradius();
			const double void_side = 1.0 - spec.wall_thickness / (2 * r);
			const double want = 1.0 - void_side * void_side;
			const double got = CellSolidFraction(spec, 0.5 * spec.wall_thickness, 200);
			char buf[200];
			std::snprintf(buf, sizeof buf, "honeycomb %s (aspect %.2g): fraction %.4f matches the shrunken cell's %.4f", k.type, k.aspect, got, want);
			Check(std::fabs(got - want) < 0.01, buf);
		}
		// A rectangle does not shrink similarly, but its void is still a
		// rectangle: (A - t)(B - t) / AB.
		{
			LatticeSpec spec;
			spec.family = LatticeFamily::Honeycomb;
			spec.type = "Rectangle";
			spec.cell_size = 1.0;
			spec.cell_aspect = 2.5;
			spec.wall_sizing = WallSizing::Thickness;
			spec.wall_thickness = 0.1;
			const double want = 1.0 - (1.0 - 0.1 / 1.0) * (1.0 - 0.1 / 2.5);
			const double got = CellSolidFraction(spec, 0.05, 200);
			char buf[200];
			std::snprintf(buf, sizeof buf, "honeycomb Rectangle: fraction %.4f matches (A - t)(B - t) / AB = %.4f", got, want);
			Check(std::fabs(got - want) < 0.01, buf);
		}
		// A stretched hexagon has no closed form, so count a patch many
		// cells across instead: the one-tile sampling must cover the tiling
		// exactly once, neither double counting nor missing a sliver.
		{
			LatticeSpec spec;
			spec.family = LatticeFamily::Honeycomb;
			spec.type = "Hexagon";
			spec.cell_size = 1.0;
			spec.cell_aspect = 1.6;
			spec.wall_sizing = WallSizing::Thickness;
			spec.wall_thickness = 0.15;
			const HoneycombCell cell = HoneycombCellFor(spec);
			auto patch = [&](double w) {
				const int n = 1200;
				size_t inside = 0;
				for (int i = 0; i < n; ++i)
					for (int j = 0; j < n; ++j) {
						const double u = -0.5 * w + w * (i + 0.5) / n, v = -0.5 * w + w * (j + 0.5) / n;
						if (cell.Distance(u, v) <= 0.5 * spec.wall_thickness) ++inside;
					}
				return static_cast<double>(inside) / (static_cast<double>(n) * n);
			};
			// A square window cuts the cells at its border, which biases it by
			// about its perimeter over its area; what says the tile is the
			// right fundamental domain is that the bias shrinks with the
			// window rather than settling on an offset.
			const double tile = CellSolidFraction(spec, 0.5 * spec.wall_thickness, 200);
			const double small = patch(7.0), large = patch(28.0);
			char buf[220];
			std::snprintf(buf, sizeof buf, "stretched hexagon: one tile gives %.4f; a 7x7 patch %.4f, a 28x28 patch %.4f -- converging on the tile",
				tile, small, large);
			Check(std::fabs(tile - large) < 0.5 * std::fabs(tile - small) && std::fabs(tile - large) < 0.004, buf);
		}
	}

	// ---- honeycomb calibration and wall sizing ----
	{
		for (const std::string& type : HoneycombTypeNames()) {
			for (double aspect : {1.0, 2.0}) {
				LatticeSpec spec;
				spec.family = LatticeFamily::Honeycomb;
				spec.type = type;
				spec.cell_size = 1.0;
				spec.cell_aspect = aspect;
				spec.volume_fraction = 0.18;
				const double level = CalibrateLevel(spec, nullptr, 96);
				const double got = CellSolidFraction(spec, level, 96);
				char buf[200];
				std::snprintf(buf, sizeof buf, "honeycomb %s (aspect %.2g): a %.4f wall gives fraction %.3f (target 0.18)", type.c_str(), aspect, 2 * level, got);
				Check(std::fabs(got - 0.18) < 0.02 && level > 0.0 && level < HoneycombCellFor(spec).Inradius(), buf);
				Check(std::fabs(HoneycombWallThickness(spec, 96) - 2 * level) < 1e-12, "honeycomb " + type + ": the reported wall is twice the level");
			}
		}
		// Sizing by thickness takes the number as given, whatever the
		// volume fraction says.
		LatticeSpec spec;
		spec.family = LatticeFamily::Honeycomb;
		spec.type = "Hexagon";
		spec.cell_size = 1.0;
		spec.volume_fraction = 0.9; // ignored
		spec.wall_sizing = WallSizing::Thickness;
		spec.wall_thickness = 0.08;
		Check(std::fabs(HoneycombWallThickness(spec) - 0.08) < 1e-12 && std::fabs(LatticeField(spec).level() - 0.04) < 1e-12,
			"honeycomb: sizing by thickness uses the wall as given");
		// A wall as thick as the cell would come out solid, and says so.
		spec.wall_thickness = 1.5;
		bool threw = false;
		try { LatticeField solid(spec); } catch (const std::invalid_argument&) { threw = true; }
		Check(threw, "honeycomb: a wall thicker than the cell is refused rather than filled in");
	}

	// ---- the honeycomb field is prismatic, periodic and a true distance ----
	{
		LatticeSpec spec;
		spec.family = LatticeFamily::Honeycomb;
		spec.type = "Hexagon";
		spec.cell_size = 1.0;
		spec.wall_sizing = WallSizing::Thickness;
		spec.wall_thickness = 0.1;
		const LatticeField f(spec);
		const HoneycombCell c = HoneycombCellFor(spec);
		Check(std::fabs(f({0.3, 0.2, 0.0}) - f({0.3, 0.2, 13.7})) < 1e-12, "honeycomb: the field does not vary along the prism axis");
		Check(std::fabs(f({0.3, 0.2, 0.0}) - f({0.3 + c.a[0], 0.2 + c.a[1], 0.0})) < 1e-12 &&
			std::fabs(f({0.3, 0.2, 0.0}) - f({0.3 + c.b[0], 0.2 + c.b[1], 0.0})) < 1e-12,
			"honeycomb: the field is periodic on the tiling's lattice vectors");
		// The prism axis chooses which two axes the tiling lies in.
		LatticeSpec along_x = spec;
		along_x.axis = PrismAxis::X;
		const LatticeField fx(along_x);
		Check(std::fabs(fx({9.1, 0.3, 0.2}) - f({0.3, 0.2, 0.0})) < 1e-12, "honeycomb: the prism axis chooses the plane the cells tile");
		// A true distance: 1-Lipschitz, and zero exactly on the wall centre.
		double worst = 0.0;
		uint32_t rng = 7u;
		auto next = [&rng]() { rng = rng * 1664525u + 1013904223u; return (rng >> 8) * (1.0 / 16777216.0); };
		for (int i = 0; i < 5000; ++i) {
			const Vec3 p{next() * 4 - 2, next() * 4 - 2, 0.0};
			const Vec3 q{next() * 4 - 2, next() * 4 - 2, 0.0};
			worst = std::max(worst, (std::fabs(f(p) - f(q)) - Distance(p, q)) / std::max(Distance(p, q), 1e-9));
		}
		char buf[160];
		std::snprintf(buf, sizeof buf, "honeycomb: the field is 1-Lipschitz, so it is a distance (worst excess %.2e)", worst);
		Check(worst < 1e-9, buf);
		Check(std::fabs(f({0.5, 0.0, 0.0}) + 0.05) < 1e-12, "honeycomb: half a wall inside the wall's centre-line");
	}

	// ---- a hexagonal honeycomb in a box ----
	{
		TriMesh box = BoxMesh(2.0, 2.0, 2.0);
		MeshSdf box_sdf(box);
		LatticeSpec spec;
		spec.family = LatticeFamily::Honeycomb;
		spec.type = "Hexagon";
		spec.cell_size = 0.5;
		spec.volume_fraction = 0.25;
		spec.resolution = 24;
		LatticeDomain dom;
		dom.body = [&](const Vec3& p) { return box_sdf.Distance(p); };
		dom.bounds = box.Bounds();
		LatticeResult r = GenerateLattice(spec, dom);
		std::printf("hexagonal honeycomb in box: grid %dx%dx%d, %zu triangles, wall %.4f\n", r.grid.nx, r.grid.ny, r.grid.nz,
			r.mesh.triangles.size(), HoneycombWallThickness(spec));
		Check(!r.mesh.IsEmpty() && r.mesh.IsClosedManifold() && r.mesh.SignedVolume() > 0.0, "hexagonal honeycomb in box: a closed, outward mesh");
		char buf[200];
		std::snprintf(buf, sizeof buf, "hexagonal honeycomb in box: achieved fraction %.3f within 0.05 of 0.25", r.achieved_fraction);
		Check(std::fabs(r.achieved_fraction - 0.25) < 0.05, buf);
		// The walls are only three voxels across, which costs the sampled
		// fraction above a few percent (a slab of t straddles floor(t / h)
		// nodes, biased low). The mesh itself does not pay it: the field is
		// an exact distance, so the crossing marching tetrahedra interpolates
		// along each edge is the wall's true face. Hence the volume, not the
		// sample count, is what pins the wall thickness down.
		// The achieved fraction runs under the 0.25 asked for because four
		// cells do not fit the box a whole number of times: the boundary
		// cuts the tiling, which costs or gains a share of order 1 / cells
		// across (measured here: 0.227 at four cells, 0.251 at eight, 0.240
		// at sixteen -- settling on 0.25 slowly). That is the box's doing,
		// not the cell's: the calibration itself is exact to the closed
		// forms checked above. So what the mesh owes is the field's real
		// share of THIS box, which dense sampling of the cross-section gives
		// outright -- the cell being prismatic, its area share in z is its
		// volume share.
		// Fine enough that the probe's own quantization (a band of one pixel
		// along every wall face) stays well under the mesh tolerance below.
		const LatticeField field(spec);
		const int n = 1600;
		size_t in = 0;
		for (int i = 0; i < n; ++i)
			for (int j = 0; j < n; ++j) {
				const double x = -1.0 + 2.0 * (i + 0.5) / n, y = -1.0 + 2.0 * (j + 0.5) / n;
				if (field({x, y, 0.0}) <= 0.0) ++in;
			}
		const double exact = static_cast<double>(in) / (static_cast<double>(n) * n);
		const double volume = r.mesh.SignedVolume();
		std::snprintf(buf, sizeof buf, "hexagonal honeycomb in box: mesh volume %.4f is %.4f of the box, the field's own %.4f within 3%%",
			volume, volume / 8.0, exact);
		Check(std::fabs(volume / 8.0 - exact) < 0.03 * exact, buf);
		Box3 mb = r.mesh.Bounds();
		Check(mb.min.x >= -1.0 - 1e-6 && mb.max.x <= 1.0 + 1e-6 && mb.min.z >= -1.0 - 1e-6 && mb.max.z <= 1.0 + 1e-6,
			"hexagonal honeycomb in box: stays inside the box");
		Check(mb.min.z < -1.0 + r.grid.h && mb.max.z > 1.0 - r.grid.h, "hexagonal honeycomb in box: the prisms run the full height");
		// The same cell about each axis fills the same share of a cube.
		double fraction[3] = {0, 0, 0};
		for (int ax = 0; ax < 3; ++ax) {
			LatticeSpec turned = spec;
			turned.axis = static_cast<PrismAxis>(ax);
			fraction[ax] = GenerateLattice(turned, dom).achieved_fraction;
		}
		std::snprintf(buf, sizeof buf, "hexagonal honeycomb in a cube: the same fraction about x, y and z (%.3f, %.3f, %.3f)", fraction[0], fraction[1], fraction[2]);
		Check(std::fabs(fraction[0] - fraction[2]) < 0.01 && std::fabs(fraction[1] - fraction[2]) < 0.01, buf);
	}

	// ---- a wall thinner than the sampling is refused, not meshed ----
	{
		LatticeSpec spec;
		spec.family = LatticeFamily::Honeycomb;
		spec.type = "Square";
		spec.cell_size = 1.0;
		spec.wall_sizing = WallSizing::Thickness;
		spec.wall_thickness = 0.02;
		spec.resolution = 8; // a voxel of 0.125: the wall is a sixth of one
		Box3 b;
		b.Add({0, 0, 0});
		b.Add({1, 1, 1});
		Check(!LatticeResolutionWarning(spec, PlanLatticeGrid(spec, b)).empty(), "honeycomb: a wall thinner than a voxel is reported");
		LatticeDomain dom;
		dom.body = [](const Vec3&) { return -1.0; };
		dom.bounds = b;
		bool threw = false;
		try { GenerateLattice(spec, dom); } catch (const std::runtime_error&) { threw = true; }
		Check(threw, "honeycomb: generating such a wall throws rather than returning a ragged mesh");
		spec.resolution = 150; // a voxel of 1/150: three voxels across
		Check(LatticeResolutionWarning(spec, PlanLatticeGrid(spec, b)).empty(), "honeycomb: resolving the wall clears the complaint");
		// Only a thickness set apart from the cell can fall through: the
		// other families scale with it.
		LatticeSpec gyroid;
		gyroid.resolution = 2;
		Check(LatticeResolutionWarning(gyroid, PlanLatticeGrid(gyroid, b)).empty(), "a TPMS network has no independent wall to miss");
	}

	// ---- grid planning and the sample cap ----
	{
		LatticeSpec spec;
		spec.cell_size = 0.1;
		spec.resolution = 10;
		Box3 b;
		b.Add({0, 0, 0});
		b.Add({1, 1, 1});
		LatticeGrid g = PlanLatticeGrid(spec, b);
		Check(g.h == 0.01 && g.nx >= 100 && g.nx <= 106, "plan: 10 voxels per 0.1 cell over a unit box");
		LatticeDomain dom;
		dom.body = [](const Vec3&) { return -1.0; };
		dom.bounds = b;
		bool threw = false;
		try { GenerateLattice(spec, dom, nullptr, 0, 1000); } catch (const std::runtime_error&) { threw = true; }
		Check(threw, "generate: refuses a grid over the sample limit");
	}

	std::printf("RESULT: %d failure(s)\n", g_failures);
	return g_failures == 0 ? 0 : 1;
}
