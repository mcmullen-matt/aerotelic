// Checks the lattice generator (lattice.hpp): the volume-fraction
// calibration of every TPMS and strut type, marching tetrahedra on an
// analytic field, and whole-body generation inside a box and a sphere
// with a reserved volume.

#include <chrono>
#include <cmath>
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
