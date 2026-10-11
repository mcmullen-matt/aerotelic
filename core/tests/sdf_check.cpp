// Checks the BVH signed-distance field (sdf.hpp): against the analytic
// distance to a sphere, against brute force over every triangle, and
// against OCCT's solid classifier for inside/outside on a wing.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>

#include <BRepClass3d_SolidClassifier.hxx>
#include <gp_Pnt.hxx>

#include "aerotelic/airfoil.hpp"
#include "aerotelic/mesh.hpp"
#include "aerotelic/sdf.hpp"
#include "aerotelic/wing_model.hpp"

using namespace aerotelic;

namespace {

int g_failures = 0;

void Check(bool ok, const std::string& what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what.c_str());
	if (!ok) ++g_failures;
}

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
	std::mt19937 rng(11);

	// ---- sphere: analytic distance ----
	{
		const double tol = 1e-4;
		TriMesh sph = EllipsoidMesh(1.0, 1.0, 1.0, tol);
		MeshSdf sdf(sph);
		std::printf("sphere mesh: %zu triangles, %zu BVH nodes\n", sdf.n_triangles(), sdf.n_nodes());
		std::uniform_real_distribution<double> U(-2.0, 2.0);
		double worst = 0.0;
		int sign_errors = 0;
		for (int i = 0; i < 2000; ++i) {
			Vec3 p{U(rng), U(rng), U(rng)};
			double exact = Length(p) - 1.0;
			double d = sdf.Distance(p);
			worst = std::max(worst, std::fabs(d - exact));
			if (std::fabs(exact) > 1e-3 && (d < 0.0) != (exact < 0.0)) ++sign_errors;
		}
		char buf[160];
		std::snprintf(buf, sizeof buf, "sphere: signed distance within 2e-3 of |p| - 1 everywhere (worst %.3g)", worst);
		Check(worst <= 2e-3, buf);
		Check(sign_errors == 0, "sphere: sign correct for every point farther than 1e-3 from the surface");
		Check(sdf.Distance({0, 0, 0}) < -0.99, "sphere: centre is about -1");
		Check(std::fabs(sdf.Distance({3, 0, 0}) - 2.0) < 2e-3, "sphere: (3,0,0) is about +2");
	}

	// ---- wing: BVH == brute force, inside/outside vs OCCT ----
	{
		PlanformSpec ps;
		ps.sweep_angle = 25.0;
		ps.twist_curve = LinearCurveSpec(3.0, -2.0);
		WingModel wing(ps, "Wing");
		wing.AddSection(Airfoil::Naca4("4415"), 0.0);
		wing.AddSection(Airfoil::Naca4("0010"), 1.0);
		TriMesh mesh = wing.BuildMeshes(5e-4)[0].mesh;
		Check(mesh.IsClosedManifold(), "wing mesh: closed manifold");
		auto t0 = std::chrono::steady_clock::now();
		MeshSdf sdf(mesh);
		double t_build = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
		std::printf("wing mesh: %zu triangles, BVH built in %.3f s\n", sdf.n_triangles(), t_build);

		Box3 b = mesh.Bounds().Expanded(0.5);
		std::uniform_real_distribution<double> Ux(b.min.x, b.max.x), Uy(b.min.y, b.max.y), Uz(b.min.z, b.max.z);
		double worst = 0.0;
		for (int i = 0; i < 300; ++i) {
			Vec3 p{Ux(rng), Uy(rng), Uz(rng)};
			double a = sdf.ClosestPoint(p).distance, c = sdf.ClosestPointBruteForce(p).distance;
			worst = std::max(worst, std::fabs(a - c));
		}
		Check(worst <= 1e-12, "wing: BVH closest distance equals brute force (300 points)");

		// The sign against an independent oracle on the same mesh: ray parity
		// (a ray from the point crosses a closed surface an odd number of
		// times exactly when the point is inside).
		auto ray_inside = [&](const Vec3& p) {
			const Vec3 dir{0.3713906763541037, 0.5570860145311556, 0.7428, }; // an irrational-ish direction, no edge hits
			int crossings = 0;
			for (const auto& t : mesh.triangles) {
				const Vec3& a = mesh.vertices[t[0]];
				const Vec3& bb = mesh.vertices[t[1]];
				const Vec3& c = mesh.vertices[t[2]];
				Vec3 e1 = bb - a, e2 = c - a, h = Cross(dir, e2);
				double det = Dot(e1, h);
				if (std::fabs(det) < 1e-14) continue;
				double f = 1.0 / det;
				Vec3 s = p - a;
				double u = f * Dot(s, h);
				if (u < 0.0 || u > 1.0) continue;
				Vec3 q = Cross(s, e1);
				double v = f * Dot(dir, q);
				if (v < 0.0 || u + v > 1.0) continue;
				double tt = f * Dot(e2, q);
				if (tt > 0.0) ++crossings;
			}
			return (crossings % 2) == 1;
		};
		int sign_disagree = 0, inside_count = 0;
		for (int i = 0; i < 300; ++i) {
			Vec3 p{Ux(rng), Uy(rng), 0.5 * (Uz(rng) + 0.5 * (b.min.z + b.max.z))};
			bool in = sdf.Distance(p) < 0.0;
			if (in) ++inside_count;
			if (in != ray_inside(p)) ++sign_disagree;
		}
		char buf[200];
		std::snprintf(buf, sizeof buf, "wing: pseudo-normal sign agrees with ray parity on all 300 points (%d inside, %d disagree)", inside_count, sign_disagree);
		Check(sign_disagree == 0 && inside_count > 10, buf);

		// Inside/outside agreement with OCCT's solid, well away from the
		// surface (OCCT's loft is a slightly different surface -- see
		// mesh_check; it was seen to differ by up to 1.4 cm on this wing --
		// so points within 2.5 cm of the mesh are not compared).
		TopoDS_Shape solid = wing.BuildSolid();
		int disagree = 0, tested = 0;
		for (int i = 0; i < 400; ++i) {
			Vec3 p{Ux(rng), Uy(rng), 0.5 * (Uz(rng) + 0.5 * (b.min.z + b.max.z))};
			double d = sdf.Distance(p);
			if (std::fabs(d) < 2.5e-2) continue;
			BRepClass3d_SolidClassifier cls(solid, gp_Pnt(p.x, p.y, p.z), 1e-7);
			bool occt_in = cls.State() == TopAbs_IN;
			++tested;
			if (occt_in != (d < 0.0)) {
				++disagree;
				std::printf("  disagreement at (%.4f, %.4f, %.4f): mesh sdf %.4f, OCCT state %d, ray parity %d\n", p.x, p.y, p.z, d, int(cls.State()), int(ray_inside(p)));
			}
		}
		std::snprintf(buf, sizeof buf, "wing: inside/outside agrees with OCCT on %d of %d points more than 2.5 cm from the surface", tested - disagree, tested);
		Check(tested > 100 && disagree == 0, buf);

		// Throughput.
		t0 = std::chrono::steady_clock::now();
		const int n = 200000;
		double acc = 0.0;
		for (int i = 0; i < n; ++i) acc += sdf.Distance({Ux(rng), Uy(rng), Uz(rng)});
		double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
		std::printf("wing: %d signed-distance queries in %.3f s (%.2f us each; checksum %.3f)\n", n, dt, 1e6 * dt / n, acc);
		Check(dt / n < 20e-6, "wing: under 20 us per query single-threaded");
	}

	std::printf("RESULT: %d failure(s)\n", g_failures);
	return g_failures == 0 ? 0 : 1;
}
