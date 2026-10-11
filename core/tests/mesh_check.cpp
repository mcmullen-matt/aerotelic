// Checks the OCCT-free loft triangulation (mesh.hpp) on every body model:
// the meshes are closed, consistently oriented manifolds, enclose the same
// volume as the OCCT solid the model lofts, and lie within the tolerance
// of the true surface at random (station, along-curve) samples.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>

#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>

#include "aerotelic/fuselage_model.hpp"
#include "aerotelic/lofted_nacelle_model.hpp"
#include "aerotelic/mesh.hpp"
#include "aerotelic/nacelle_model.hpp"
#include "aerotelic/primitive_model.hpp"
#include "aerotelic/propeller_model.hpp"
#include "aerotelic/sdf.hpp"
#include "aerotelic/wing_model.hpp"

using namespace aerotelic;

namespace {

int g_failures = 0;

void Check(bool ok, const std::string& what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what.c_str());
	if (!ok) ++g_failures;
}

// OCCT's own tessellation of the solid, as a welded closed mesh. (GProp's
// VolumeProperties is NOT used as the reference: on the ruled wing loft it
// reports 2.52 where both this tessellation and the integrated section
// areas give 2.95-3.05 -- its Gauss integration over the loft's faces is
// off, not the meshes.)
TriMesh OcctTessellation(const TopoDS_Shape& shape, double deflection) {
	BRepMesh_IncrementalMesh mesher(shape, deflection);
	TriMesh occt;
	for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
		TopoDS_Face face = TopoDS::Face(ex.Current());
		TopLoc_Location loc;
		Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
		if (tri.IsNull()) continue;
		int base = static_cast<int>(occt.vertices.size());
		for (int i = 1; i <= tri->NbNodes(); ++i) {
			gp_Pnt p = tri->Node(i).Transformed(loc.Transformation());
			occt.vertices.push_back({p.X(), p.Y(), p.Z()});
		}
		bool rev = face.Orientation() == TopAbs_REVERSED;
		for (int i = 1; i <= tri->NbTriangles(); ++i) {
			int a, b, c;
			tri->Triangle(i).Get(a, b, c);
			if (rev) std::swap(b, c);
			occt.triangles.push_back({base + a - 1, base + b - 1, base + c - 1});
		}
	}
	occt.WeldVertices(1e-7);
	occt.FlipIfInsideOut();
	return occt;
}

double OcctVolume(const TopoDS_Shape& shape) { return OcctTessellation(shape, 2e-4).SignedVolume(); }

// The volume of a wing straight from its definition: section areas (the
// user's airfoils blended linearly in airfoil coordinates between
// sections, as AirfoilAt does) integrated along the span -- independent of
// both OCCT and the mesh builder.
double IntegratedWingVolume(const WingModel& wing, double s_min = 0.0, double s_max = 1.0, int n = 4000) {
	std::vector<double> v(600);
	for (size_t j = 0; j < v.size(); ++j) v[j] = double(j) / (v.size() - 1);
	const auto& secs = wing.sections();
	auto blended = [&](double s) {
		size_t hi = 0;
		while (hi < secs.size() && secs[hi].s < s) ++hi;
		if (hi == 0) return secs[0].airfoil->nurbs_curve().Evaluate(v);
		if (hi >= secs.size()) return secs.back().airfoil->nurbs_curve().Evaluate(v);
		const WingSection& lo = secs[hi - 1];
		const WingSection& up = secs[hi];
		double w = (s - lo.s) / (up.s - lo.s);
		Mat a = lo.airfoil->nurbs_curve().Evaluate(v), b = up.airfoil->nurbs_curve().Evaluate(v);
		Mat pts(a.rows(), 2);
		for (int r = 0; r < a.rows(); ++r) {
			pts(r, 0) = (1 - w) * a(r, 0) + w * b(r, 0);
			pts(r, 1) = (1 - w) * a(r, 1) + w * b(r, 1);
		}
		return pts;
	};
	double total = 0.0, prev_area = 0.0;
	Vec3 prev_origin;
	for (int i = 0; i <= n; ++i) {
		double s = s_min + (s_max - s_min) * i / n;
		Mat xyz = wing.planform().AirfoilCoordsToXyz(s, blended(s));
		Vec3 nrm{0, 0, 0}, centroid{0, 0, 0};
		for (int r = 0; r < xyz.rows(); ++r) {
			int q = (r + 1) % xyz.rows();
			Vec3 p{xyz(r, 0), xyz(r, 1), xyz(r, 2)}, pq{xyz(q, 0), xyz(q, 1), xyz(q, 2)};
			nrm.x += (p.y - pq.y) * (p.z + pq.z);
			nrm.y += (p.z - pq.z) * (p.x + pq.x);
			nrm.z += (p.x - pq.x) * (p.y + pq.y);
			centroid += p;
		}
		centroid = centroid / xyz.rows();
		double area = 0.5 * Length(nrm);
		// The spanwise step is the section origin's displacement along the
		// section's normal (sweep and twist move it within the plane).
		if (i > 0) total += 0.5 * (area + prev_area) * std::fabs(Dot(centroid - prev_origin, Normalized(nrm)));
		prev_area = area;
		prev_origin = centroid;
	}
	return total;
}

std::vector<TopoDS_Shape> TopLevel(const TopoDS_Shape& shape) {
	std::vector<TopoDS_Shape> out;
	if (shape.ShapeType() == TopAbs_COMPOUND) {
		for (TopoDS_Iterator it(shape); it.More(); it.Next()) out.push_back(it.Value());
	} else {
		out.push_back(shape);
	}
	return out;
}

double Seconds(const std::chrono::steady_clock::time_point& t0) {
	return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

// Common checks for one body, against a reference volume.
void CheckBody(const std::string& label, const TriMesh& mesh, double ref_volume, double volume_rel_tol, const char* ref_label = "OCCT") {
	Check(!mesh.IsEmpty(), label + ": has triangles (" + std::to_string(mesh.triangles.size()) + ")");
	Check(mesh.IsClosedManifold(), label + ": closed manifold");
	double v = mesh.SignedVolume();
	Check(v > 0.0, label + ": outward-oriented (signed volume > 0)");
	double rel = std::fabs(v - ref_volume) / std::max(ref_volume, 1e-12);
	char buf[200];
	std::snprintf(buf, sizeof buf, "%s: volume %.6g vs %s %.6g (rel %.3g <= %.3g)", label.c_str(), v, ref_label, ref_volume, rel, volume_rel_tol);
	Check(rel <= volume_rel_tol, buf);
}
void CheckBody(const std::string& label, const TriMesh& mesh, const TopoDS_Shape& solid, double volume_rel_tol) {
	CheckBody(label, mesh, OcctVolume(solid), volume_rel_tol);
}

// Distance from random true surface points to the mesh.
void CheckSurfaceError(const std::string& label, const TriMesh& mesh, const LoftSource& src, double tol, int n = 300) {
	MeshSdf sdf(mesh);
	std::mt19937 rng(7);
	std::uniform_real_distribution<double> U(0.0, 1.0);
	double worst = 0.0;
	for (int i = 0; i < n; ++i) {
		double u = src.u_min + (src.u_max - src.u_min) * U(rng);
		double v = U(rng);
		Mat p = src.section(u, {v});
		worst = std::max(worst, sdf.ClosestPoint({p(0, 0), p(0, 1), p(0, 2)}).distance);
	}
	char buf[160];
	std::snprintf(buf, sizeof buf, "%s: random surface points within %.2g of the mesh (worst %.3g)", label.c_str(), 3.0 * tol, worst);
	Check(worst <= 3.0 * tol, buf);
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
	const double tol = 1e-3;

	// ---- engine basics on an analytic cylinder ----
	{
		TriMesh cyl = CylinderMesh(1.0, 2.0, tol);
		Check(cyl.IsClosedManifold(), "cylinder: closed manifold");
		double v = cyl.SignedVolume();
		Check(std::fabs(v - 3.14159265358979 * 2.0) / (3.14159265358979 * 2.0) < 5e-3, "cylinder: volume within 0.5% of pi r^2 l");
		TriMesh box = BoxMesh(1.0, 2.0, 3.0);
		Check(box.IsClosedManifold() && std::fabs(box.SignedVolume() - 6.0) < 1e-12, "box: closed, volume 6");
		TriMesh sph = EllipsoidMesh(1.0, 1.0, 1.0, tol);
		Check(sph.IsClosedManifold(), "sphere: closed manifold");
		Check(std::fabs(sph.SignedVolume() - 4.18879020478639) / 4.18879020478639 < 5e-3, "sphere: volume within 0.5% of 4/3 pi r^3");
		TriMesh ell = EllipsoidMesh(2.0, 1.0, 0.5, tol);
		Check(ell.IsClosedManifold() && std::fabs(ell.SignedVolume() - 4.18879020478639) / 4.18879020478639 < 5e-3, "ellipsoid: closed, volume within 0.5%");
	}

	// ---- a knife-edge end: a section squeezed flat (zero width, some height) ----
	{
		// A body whose u = 0 section is a vertical line segment traced out
		// and back (every point on the way back coincides with one on the
		// way out), opening into a circle by u = 1: what a fuselage nose or
		// tail with zero half-width but z_top != z_bot produces. The flat
		// section can carry no cap; the mesh is closed only if the strip on
		// each side of it shares the edge.
		LoftSource src;
		src.section = [](double u, const std::vector<double>& v) {
			Mat out(static_cast<int>(v.size()), 3);
			for (size_t j = 0; j < v.size(); ++j) {
				double phi = 2.0 * 3.14159265358979323846 * v[j];
				out(static_cast<int>(j), 0) = 2.0 * u;
				out(static_cast<int>(j), 1) = u * std::sin(phi);
				out(static_cast<int>(j), 2) = std::cos(phi);
			}
			return out;
		};
		src.closed_curve = true;
		src.v_seeds = {0.0, 0.25, 0.5, 0.75};
		LoftOptions opt;
		opt.tolerance = tol;
		opt.min_v = 16;
		opt.apex_tolerance = 1e-9;
		TriMesh knife = TriangulateLoft(src, opt);
		Check(!knife.IsEmpty(), "knife-edge loft: has triangles");
		Check(knife.IsClosedManifold(), "knife-edge loft: closed manifold (the flat end's two sides share their edge)");
		Check(knife.SignedVolume() > 0.0, "knife-edge loft: outward");
		// The field's sign holds beyond the knife edge, where the pseudo-normal
		// of an open slit would have been one-sided.
		MeshSdf sdf(knife);
		bool signs_ok = true;
		for (double dy : {-1.5, -0.5, 0.5, 1.5})
			for (double dz : {-0.4, 0.0, 0.4})
				signs_ok = signs_ok && sdf.Distance({-0.3, dy, dz}) > 0.0;
		Check(signs_ok, "knife-edge loft: points beyond the knife edge are outside");
		Check(sdf.Distance({1.0, 0.0, 0.0}) < 0.0, "knife-edge loft: the axis is inside");
	}

	// ---- wing ----
	{
		PlanformSpec ps;
		WingModel wing(ps, "Wing");
		wing.AddSection(Airfoil::Naca4("2412"), 0.0);
		wing.AddSection(Airfoil::Naca4("0010"), 1.0);
		auto t0 = std::chrono::steady_clock::now();
		std::vector<NamedMesh> meshes = wing.BuildMeshes(tol);
		double t_mesh = Seconds(t0);
		t0 = std::chrono::steady_clock::now();
		TopoDS_Shape solid = wing.BuildSolid();
		double t_occt = Seconds(t0);
		std::printf("wing: mesh %.3f s, OCCT loft %.3f s\n", t_mesh, t_occt);
		Check(meshes.size() == 1 && meshes[0].name == "Main", "wing: one body named Main");
		// The reference is the wing's own definition; OCCT's ruled loft pairs
		// the section curves up by its own (arc-length) reparametrisation and
		// comes out a few percent larger, so it is only a loose cross-check.
		CheckBody("wing", meshes[0].mesh, IntegratedWingVolume(wing), 5e-3, "integrated sections");
		CheckBody("wing", meshes[0].mesh, solid, 5e-2);
		LoftSource src;
		src.closed_curve = false;
		src.section = [&](double s, const std::vector<double>& v) {
			// The mesh builder's own section map, reproduced from the public API.
			const auto& secs = wing.sections();
			std::shared_ptr<Airfoil> af = s <= 0.5 ? secs[0].airfoil : secs[1].airfoil;
			// At the ends the section is exactly the user's airfoil; sample there.
			double ss = s <= 0.5 ? 0.0 : 1.0;
			return wing.planform().AirfoilCoordsToXyz(ss, af->nurbs_curve().Evaluate(v));
		};
		CheckSurfaceError("wing (end sections)", meshes[0].mesh, src, tol);

		// A tapered, swept, twisted and dihedral wing (non-linear spanwise laws).
		PlanformSpec ps2;
		ps2.type = "standard";
		ps2.sweep_angle = 30.0;
		ps2.twist_curve = LinearCurveSpec(2.0, -3.0);
		ps2.dihedral_curve.degree = 2;
		ps2.dihedral_curve.poles = Mat::FromRows({{0.0, 0.0}, {0.5, 0.05}, {1.0, 0.25}});
		WingModel wing2(ps2, "Wing2");
		wing2.AddSection(Airfoil::Naca4("4415"), 0.0);
		wing2.AddSection(Airfoil::Naca4("2412"), 0.5);
		wing2.AddSection(Airfoil::Naca4("0008"), 1.0);
		std::vector<NamedMesh> m2 = wing2.BuildMeshes(tol);
		CheckBody("curved wing", m2[0].mesh, IntegratedWingVolume(wing2), 1e-2, "integrated sections");
		CheckBody("curved wing", m2[0].mesh, wing2.BuildSolid(), 5e-2);
	}

	// ---- fuselage ----
	{
		FuselageSpec fs;
		FuselageModel fus(fs);
		auto t0 = std::chrono::steady_clock::now();
		std::vector<NamedMesh> meshes = fus.BuildMeshes(tol);
		double t_mesh = Seconds(t0);
		t0 = std::chrono::steady_clock::now();
		TopoDS_Shape solid = fus.BuildSolid();
		std::printf("fuselage: mesh %.3f s, OCCT loft %.3f s\n", t_mesh, Seconds(t0));
		// OCCT lofts smoothly through the stations and opens the apexes to
		// 1 mm, so the volumes differ slightly by construction.
		CheckBody("fuselage", meshes[0].mesh, solid, 1.5e-2);
		LoftSource src;
		src.section = [&](double x, const std::vector<double>& v) {
			auto p = fus.SectionParams(x);
			Mat out(static_cast<int>(v.size()), 3);
			for (size_t j = 0; j < v.size(); ++j) {
				auto yz = ConicCrossSectionPoint(p[0], p[1], p[2], p[3], p[4], v[j]);
				out(static_cast<int>(j), 0) = x * fus.length;
				out(static_cast<int>(j), 1) = yz[0];
				out(static_cast<int>(j), 2) = yz[1];
			}
			return out;
		};
		CheckSurfaceError("fuselage", meshes[0].mesh, src, tol);
	}

	// ---- nacelle (tensor-product skins) ----
	{
		NacelleSpec ns;
		NacelleModel nac(ns);
		auto t0 = std::chrono::steady_clock::now();
		std::vector<NamedMesh> meshes = nac.BuildMeshes(tol);
		double t_mesh = Seconds(t0);
		t0 = std::chrono::steady_clock::now();
		TopoDS_Shape solid = nac.BuildSolid();
		std::printf("nacelle: mesh %.3f s, OCCT %.3f s\n", t_mesh, Seconds(t0));
		CheckBody("nacelle", meshes[0].mesh, solid, 5e-3);
		NacelleSpec ns2;
		ns2.chevron_count = 6;
		NacelleModel nac2(ns2);
		Check(nac2.BuildMeshes(tol).empty(), "nacelle with chevrons: no mesh builder (falls back to OCCT)");
	}

	// ---- lofted nacelle ----
	{
		LoftedNacelleSpec ls;
		LoftedNacelleModel lof(ls);
		auto t0 = std::chrono::steady_clock::now();
		std::vector<NamedMesh> meshes = lof.BuildMeshes(tol);
		double t_mesh = Seconds(t0);
		t0 = std::chrono::steady_clock::now();
		TopoDS_Shape solid = lof.BuildSolid();
		std::printf("lofted nacelle: mesh %.3f s, OCCT %.3f s\n", t_mesh, Seconds(t0));
		CheckBody("lofted nacelle", meshes[0].mesh, solid, 1.5e-2);

		// Two guide rings and a non-zero wall at both ends (real rims).
		LoftedNacelleSpec ls2;
		ls2.outer_curve.poles = Mat::FromRows({{0.00, 0.05}, {0.04, 0.22}, {0.55, 0.20}, {1.00, 0.04}});
		ls2.inner_curve.poles = Mat::FromRows({{0.00, -0.05}, {0.04, -0.14}, {0.55, -0.10}, {1.00, -0.04}});
		GuideSectionSpec gs;
		gs.station = 1.0;
		gs.curve = DefaultSectionCurveSpec(0.9);
		ls2.sections.push_back(gs);
		LoftedNacelleModel lof2(ls2);
		std::vector<NamedMesh> m2 = lof2.BuildMeshes(tol);
		CheckBody("lofted nacelle with rims", m2[0].mesh, lof2.BuildSolid(), 1.5e-2);
	}

	// ---- propeller ----
	{
		PropellerSpec sp;
		sp.blade_count = 3;
		PropellerModel prop(sp);
		auto t0 = std::chrono::steady_clock::now();
		// A finer tolerance: 1 mm on a 10 cm blade chord cuts visible corners.
		std::vector<NamedMesh> meshes = prop.BuildMeshes(2e-4);
		double t_mesh = Seconds(t0);
		t0 = std::chrono::steady_clock::now();
		TopoDS_Shape solid = prop.BuildSolid();
		std::printf("propeller: mesh %.3f s, OCCT %.3f s\n", t_mesh, Seconds(t0));
		std::vector<TopoDS_Shape> parts = TopLevel(solid);
		Check(meshes.size() == 4 && parts.size() == 4, "propeller: spinner + 3 blades in both builders");
		Check(meshes[0].name == "Spinner" && meshes[1].name == "Blade 1" && meshes[3].name == "Blade 3", "propeller: body names");
		CheckBody("propeller Spinner", meshes[0].mesh, parts[0], 2e-2);
		// Blades: against the blade WingModel's own definition over the
		// trimmed span (OCCT's loft differs as for the wing above).
		double blade_ref = IntegratedWingVolume(prop.blade(), prop.hub_radius, 1.0);
		for (size_t i = 1; i < meshes.size(); ++i) {
			CheckBody("propeller " + meshes[i].name, meshes[i].mesh, blade_ref, 1.5e-2, "integrated sections");
		}
		CheckBody("propeller Blade 1", meshes[1].mesh, parts[1], 5e-2);
		// Blades land on top of OCCT's placed blades (same turn and placement).
		Box3 bm = meshes[1].mesh.Bounds();
		GProp_GProps props;
		BRepGProp::VolumeProperties(parts[1], props);
		gp_Pnt c = props.CentreOfMass();
		Check(c.X() > bm.min.x - 1e-3 && c.X() < bm.max.x + 1e-3 && c.Y() > bm.min.y - 1e-3 && c.Y() < bm.max.y + 1e-3 &&
			c.Z() > bm.min.z - 1e-3 && c.Z() < bm.max.z + 1e-3, "propeller: OCCT blade 1 centroid inside mesh blade 1 bounds");
	}

	// ---- primitives ----
	{
		PrimitiveModel sphere;
		sphere.shape = PrimitiveShape::Sphere;
		sphere.radius = 1.5;
		sphere.stretch = {1.0, 0.5, 2.0};
		CheckBody("ellipsoid primitive", sphere.BuildMeshes(tol)[0].mesh, sphere.BuildSolid(), 5e-3);
		PrimitiveModel cyl;
		cyl.shape = PrimitiveShape::Cylinder;
		cyl.cyl_radius = 0.7;
		cyl.cyl_length = 3.0;
		CheckBody("cylinder primitive", cyl.BuildMeshes(tol)[0].mesh, cyl.BuildSolid(), 5e-3);
		PrimitiveModel box;
		box.shape = PrimitiveShape::Box;
		box.size = {1.0, 2.0, 0.5};
		CheckBody("box primitive", box.BuildMeshes(tol)[0].mesh, box.BuildSolid(), 1e-9);
	}

	std::printf("RESULT: %d failure(s)\n", g_failures);
	return g_failures == 0 ? 0 : 1;
}
