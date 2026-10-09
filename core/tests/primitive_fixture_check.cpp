// Cross-checks the C++ PrimitiveModel port's pure-math side (volume,
// surface_area, extents) against core/primitive_model.py (see
// tools/generate_primitive_fixtures.py). build_solid() is OCCT-only on
// both sides, so instead of a Python cross-check it's smoke-tested here
// directly: build the solid, measure its own bounding box, and check that
// against Extents() -- an internal consistency property, not a port of
// anything Python computes.

#include <fstream>
#include <iostream>
#include <string>

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>

#include "aerotelic/primitive_model.hpp"
#include "token_reader.hpp"

using namespace aerotelic;

namespace {

int g_pass = 0, g_fail = 0;

void Check(bool cond, const std::string& msg) {
	if (cond) { g_pass++; return; }
	g_fail++;
	std::cout << "FAIL: " << msg << "\n";
}

void CheckClose(double a, double b, double tol, const std::string& msg) {
	Check(std::abs(a - b) <= tol, msg + " (" + std::to_string(a) + " vs " + std::to_string(b) + ")");
}

void RunCase(TokenReader& r, const std::string& name) {
	PrimitiveModel p;
	double volume_ref = 0, area_ref = 0;
	std::array<double, 3> extents_ref{};

	for (;;) {
		std::string tok = r.NextTok();
		if (tok == "END") break;
		else if (tok == "SHAPE") p.shape = ParseShape(r.NextTok());
		else if (tok == "RADIUS") p.radius = r.NextDouble();
		else if (tok == "STRETCH") { auto v = r.NextDoubles(3); p.stretch = {v[0], v[1], v[2]}; }
		else if (tok == "SIZE") { auto v = r.NextDoubles(3); p.size = {v[0], v[1], v[2]}; }
		else if (tok == "CYL_RADIUS") p.cyl_radius = r.NextDouble();
		else if (tok == "CYL_LENGTH") p.cyl_length = r.NextDouble();
		else if (tok == "VOLUME") volume_ref = r.NextDouble();
		else if (tok == "SURFACE_AREA") area_ref = r.NextDouble();
		else if (tok == "EXTENTS") { auto v = r.NextDoubles(3); extents_ref = {v[0], v[1], v[2]}; }
		else throw std::runtime_error("unknown field '" + tok + "' in case " + name);
	}

	CheckClose(p.Volume(), volume_ref, 1e-9, name + " volume");
	CheckClose(p.SurfaceArea(), area_ref, 1e-6, name + " surface_area"); // ellipsoid is a 64x128-pt quadrature, not exact
	auto ext = p.Extents();
	for (int i = 0; i < 3; ++i) CheckClose(ext[i], extents_ref[i], 1e-9, name + " extents[" + std::to_string(i) + "]");

	// build_solid() smoke test: the built shape's own bounding box should
	// match Extents() -- both sides of this check live entirely in C++/OCCT,
	// so it's a self-consistency check, not a Python cross-check.
	TopoDS_Shape solid = p.BuildSolid();
	Bnd_Box box;
	// Add() computes a loose, pole-based bound for a non-analytic surface --
	// which is what a GTransformed sphere becomes -- rather than the true
	// geometric extent; AddOptimal() is the tight version. (True sphere, box
	// and cylinder are exact analytic/polygonal shapes either way gives the
	// same answer; only the ellipsoid case actually exercises the difference.)
	BRepBndLib::AddOptimal(solid, box);
	double xmin, ymin, zmin, xmax, ymax, zmax;
	box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
	double tol = p.shape == PrimitiveShape::Sphere ? 1e-3 : 1e-6; // sphere tessellation has bbox slack
	CheckClose(xmax - xmin, ext[0], tol * std::max(1.0, ext[0]), name + " build_solid bbox X");
	CheckClose(ymax - ymin, ext[1], tol * std::max(1.0, ext[1]), name + " build_solid bbox Y");
	CheckClose(zmax - zmin, ext[2], tol * std::max(1.0, ext[2]), name + " build_solid bbox Z");
	CheckClose(xmin + xmax, 0.0, 1e-6, name + " build_solid centred on origin (X)");
	CheckClose(ymin + ymax, 0.0, 1e-6, name + " build_solid centred on origin (Y)");
	CheckClose(zmin + zmax, 0.0, 1e-6, name + " build_solid centred on origin (Z)");
}

void CheckValidateRejectsNonPositive() {
	PrimitiveModel p;
	p.shape = PrimitiveShape::Sphere;
	p.radius = 0.0;
	bool threw = false;
	try { p.Validate(); } catch (const std::invalid_argument&) { threw = true; }
	Check(threw, "Validate rejects zero sphere radius");

	PrimitiveModel b;
	b.shape = PrimitiveShape::Box;
	b.size = {1.0, -1.0, 1.0};
	threw = false;
	try { b.Validate(); } catch (const std::invalid_argument&) { threw = true; }
	Check(threw, "Validate rejects negative box size");
}

} // namespace

int main() {
	try {
		std::ifstream in(AEROTELIC_TESTS_DIR "/primitive_fixtures.txt");
		if (!in) throw std::runtime_error("cannot open primitive_fixtures.txt");
		TokenReader r(in);
		while (r.HasNext()) {
			r.Expect("CASE");
			std::string name = r.NextTok();
			RunCase(r, name);
		}
		CheckValidateRejectsNonPositive();
	} catch (const std::exception& e) {
		std::cout << "FIXTURE PARSE/RUN ERROR: " << e.what() << "\n";
		return 1;
	}

	std::cout << "\nRESULT: " << g_pass << " passed, " << g_fail << " failed\n";
	return g_fail == 0 ? 0 : 1;
}
