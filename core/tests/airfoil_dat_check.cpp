// Checks ParseSeligDat / AirfoilFromDatText against a NACA 2412 written
// in Selig order, first as-is and then translated, rotated and scaled, to
// confirm the normalisation brings both to the same unit-chord section.

#include <cmath>
#include <cstdio>
#include <sstream>
#include <string>

#include "aerotelic/airfoil.hpp"
#include "aerotelic/airfoil_dat.hpp"

using namespace aerotelic;

namespace {

int g_failures = 0;

void Check(bool ok, const char* what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what);
	if (!ok) ++g_failures;
}

// A Selig-order (TE -> upper -> LE -> lower -> TE) NACA 2412 contour,
// optionally transformed. ContourPoints() runs lower TE -> LE -> upper TE,
// so it is emitted backwards.
std::string MakeDat(const char* name, double tx, double ty, double angle_deg, double scale, bool lednicer_header) {
	std::unique_ptr<Airfoil> ref = Airfoil::Naca4("2412", 41);
	const Mat c = ref->ContourPoints(81);
	std::ostringstream out;
	out << name << "\n";
	if (lednicer_header) out << "  41.  41.\n";
	const double a = angle_deg * 3.14159265358979323846 / 180.0;
	for (int i = c.rows() - 1; i >= 0; --i) {
		const double x = c(i, 0) * scale, y = c(i, 1) * scale;
		const double xr = std::cos(a) * x - std::sin(a) * y + tx;
		const double yr = std::sin(a) * x + std::cos(a) * y + ty;
		char line[96];
		std::snprintf(line, sizeof line, "  %.6f  %.6f\n", xr, yr);
		out << line;
	}
	return out.str();
}

double MaxDiff(const Airfoil& a, const Airfoil& b) {
	const Mat pa = a.ContourPoints(60), pb = b.ContourPoints(60);
	double worst = 0.0;
	for (int i = 0; i < pa.rows(); ++i) {
		worst = std::fmax(worst, std::fabs(pa(i, 0) - pb(i, 0)));
		worst = std::fmax(worst, std::fabs(pa(i, 1) - pb(i, 1)));
	}
	return worst;
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
	const std::string plain = MakeDat("NACA 2412 test", 0.0, 0.0, 0.0, 1.0, false);
	const int expected_rows = Airfoil::Naca4("2412", 41)->ContourPoints(81).rows();
	DatContents dat = ParseSeligDat(plain);
	Check(dat.name == "NACA 2412 test", "name is the first non-coordinate line");
	Check(dat.points.rows() == expected_rows && dat.points.cols() == 2, "every pair read");
	const int last = dat.points.rows() - 1;
	Check(std::fabs(dat.points(0, 0) - 1.0) < 1e-6 && std::fabs(dat.points(last, 0) - 1.0) < 1e-6, "both trailing-edge ends at x = 1");
	// Lower TE first after the reversal: the first point's y is below the last's.
	Check(dat.points(0, 1) <= dat.points(last, 1) + 1e-9, "order is lower TE -> LE -> upper TE");
	double min_x = 1.0;
	for (int i = 0; i < dat.points.rows(); ++i) min_x = std::fmin(min_x, dat.points(i, 0));
	Check(std::fabs(min_x) < 1e-6, "leading edge at the origin");

	std::unique_ptr<Airfoil> a = AirfoilFromDatText(plain, "fallback");
	Check(a->name == "NACA 2412 test", "airfoil takes the file's name");
	const std::pair<double, double> mt = a->MaxThickness();
	Check(std::fabs(mt.second - 0.12) < 0.006, "max thickness about 12%");

	// Translated and scaled by 1.2, with a Lednicer count header:
	// normalisation brings it back to the same section. (The first
	// coordinate lines must stay at x <= 1.5 or the reference algorithm
	// drops them as count headers; and a rotation moves the minimum-x
	// point off the true leading edge, which it rejects after re-checking
	// the same index each pass -- the database's files are never rotated,
	// so that is as in WingDesigner.)
	const std::string moved = MakeDat("moved", -0.3, 0.2, 0.0, 1.2, true);
	std::unique_ptr<Airfoil> b = AirfoilFromDatText(moved, "fallback");
	Check(MaxDiff(*a, *b) < 2e-3, "translated/scaled file normalises to the same contour");
	bool rotated_threw = false;
	try { ParseSeligDat(MakeDat("turned", 0.0, 0.0, 17.0, 1.0, false)); } catch (const std::invalid_argument&) { rotated_threw = true; }
	Check(rotated_threw, "a strongly rotated contour is rejected as unrotatable, as in the Python");

	// A file without a name line takes the fallback.
	std::string nameless = plain.substr(plain.find('\n') + 1);
	std::unique_ptr<Airfoil> c = AirfoilFromDatText(nameless, "stem");
	Check(c->name == "stem", "nameless file takes the fallback name");
	Check(FirstDatNameLine(plain) == "NACA 2412 test" && FirstDatNameLine(nameless).empty(), "FirstDatNameLine");

	bool threw = false;
	try { ParseSeligDat("just a name\n0 0\n1 1\n"); } catch (const std::invalid_argument&) { threw = true; }
	Check(threw, "fewer than four pairs is rejected");

	std::printf("RESULT: %d failure(s)\n", g_failures);
	return g_failures == 0 ? 0 : 1;
}
