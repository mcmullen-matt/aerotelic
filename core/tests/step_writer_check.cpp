// Drives aerotelic_step_writer the way the application does: a body is
// written to a .brep with the geometry toolkits alone, the writer runs as
// a separate process, and a STEP file comes back.

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <string>

#include <BRepTools.hxx>
#include <TopoDS_Shape.hxx>

#include "aerotelic/primitive_model.hpp"

#ifndef AEROTELIC_STEP_WRITER
#error "AEROTELIC_STEP_WRITER (path of aerotelic_step_writer.exe) must be defined"
#endif

namespace {

int g_failures = 0;

void Check(bool ok, const char* what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what);
	if (!ok) ++g_failures;
}

std::string FirstLine(const std::string& path) {
	std::ifstream in(path);
	std::string line;
	std::getline(in, line);
	return line;
}

bool Contains(const std::string& path, const std::string& needle) {
	std::ifstream in(path);
	std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
	return text.find(needle) != std::string::npos;
}

} // namespace

int main() {
	setvbuf(stdout, nullptr, _IONBF, 0);
	const std::string dir = std::string(AEROTELIC_TESTS_DIR);
	const std::string brep = dir + "/step_writer_check_tmp.brep";
	const std::string step = dir + "/step_writer_check_tmp.step";
	std::remove(step.c_str());

	aerotelic::PrimitiveModel sphere;
	sphere.shape = aerotelic::PrimitiveShape::Sphere;
	sphere.radius = 1.5;
	const TopoDS_Shape shape = sphere.BuildSolid();
	Check(BRepTools::Write(shape, brep.c_str()), "body written to .brep with the geometry toolkits");

	const std::string cmd = "\"\"" AEROTELIC_STEP_WRITER "\" \"" + step + "\" --named \"Sphere 1\" \"" + brep + "\"\"";
	const int rc = std::system(cmd.c_str());
	Check(rc == 0, "aerotelic_step_writer exited 0");
	const std::string head = FirstLine(step);
	Check(head.rfind("ISO-10303-21", 0) == 0, "a STEP file (ISO-10303-21) was written");

	std::remove(step.c_str());
	const int rc_plain = std::system(("\"\"" AEROTELIC_STEP_WRITER "\" \"" + step + "\" --plain \"Sphere 1\" \"" + brep + "\" \"Sphere 2\" \"" + brep + "\"\"").c_str());
	Check(rc_plain == 0 && FirstLine(step).rfind("ISO-10303-21", 0) == 0, "--plain with two bodies writes a STEP file");

	const int rc_bad = std::system(("\"\"" AEROTELIC_STEP_WRITER "\" \"" + step + "\" --named \"X\" \"" + dir + "/does_not_exist.brep\"\"").c_str());
	Check(rc_bad != 0, "a missing .brep is reported with a non-zero exit");

	// --unit: the file DECLARES the length unit (Geometry > Units). Feet
	// is a conversion-based unit named FOOT in STEP; millimetres and metres
	// are the SI unit with and without the MILLI prefix. Without --unit
	// OCCT's default, millimetres, applies as it always did.
	auto write_with_unit = [&](const std::string& unit) {
		std::remove(step.c_str());
		const std::string flag = unit.empty() ? std::string() : " --unit " + unit;
		return std::system(("\"\"" AEROTELIC_STEP_WRITER "\" \"" + step + "\"" + flag + " --named \"Sphere 1\" \"" + brep + "\"\"").c_str());
	};
	Check(write_with_unit("FT") == 0 && Contains(step, "FOOT"), "--unit FT writes a STEP file declaring FOOT");
	Check(write_with_unit("MM") == 0 && Contains(step, ".MILLI."), "--unit MM declares millimetres");
	Check(write_with_unit("M") == 0 && Contains(step, ".METRE.") && !Contains(step, ".MILLI."), "--unit M declares metres, unprefixed");
	Check(write_with_unit("") == 0 && Contains(step, ".MILLI."), "no --unit is OCCT's default, millimetres, as before");
	Check(write_with_unit("FURLONG") != 0, "an unknown unit is refused with a non-zero exit");

	std::remove(brep.c_str());
	std::remove(step.c_str());
	std::printf("RESULT: %d failure(s)\n", g_failures);
	return g_failures == 0 ? 0 : 1;
}
