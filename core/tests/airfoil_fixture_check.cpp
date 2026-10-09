// Cross-checks the C++ Airfoil port against reference values dumped from
// core/airfoil.py (see tools/generate_airfoil_fixtures.py).

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "aerotelic/airfoil.hpp"
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

void CheckMatClose(const Mat& a, const Mat& b, double tol, const std::string& msg) {
	if (a.rows() != b.rows() || a.cols() != b.cols()) {
		Check(false, msg + " shape mismatch");
		return;
	}
	for (int i = 0; i < a.rows(); ++i)
		for (int j = 0; j < a.cols(); ++j)
			CheckClose(a(i, j), b(i, j), tol, msg + "[" + std::to_string(i) + "][" + std::to_string(j) + "]");
}

struct CaseData {
	std::string name;
	double t_le_ref;
	NurbsSpec curve_spec;
	std::vector<double> t_values, x_values;
	Mat upper_ref, lower_ref;
	std::vector<double> camber_ref, thickness_ref;
	double maxt_x, maxt_v, maxc_x, maxc_v;
	Mat contour_ref;
};

CaseData ParseCase(TokenReader& r, const std::string& name) {
	CaseData c;
	c.name = name;
	int degree = -1, periodic = 0, n_poles = -1, dim = -1;
	int n_t = -1, n_x = -1, n_contour = -1;

	for (;;) {
		std::string tok = r.NextTok();
		if (tok == "END") break;
		else if (tok == "TLE") c.t_le_ref = r.NextDouble();
		else if (tok == "DEGREE") degree = r.NextInt();
		else if (tok == "PERIODIC") periodic = r.NextInt();
		else if (tok == "NPOLES") n_poles = r.NextInt();
		else if (tok == "DIM") dim = r.NextInt();
		else if (tok == "POLES") c.curve_spec.poles = r.NextMat(n_poles, dim);
		else if (tok == "WEIGHTS") c.curve_spec.weights = r.NextDoubles(n_poles);
		else if (tok == "KNOTS") {
			int n_knots = periodic ? n_poles : (n_poles - degree - 1);
			auto k = r.NextDoubles(n_knots);
			if (periodic) c.curve_spec.knots = k; else c.curve_spec.interior_knots = k;
		}
		else if (tok == "NT") n_t = r.NextInt();
		else if (tok == "TVALUES") c.t_values = r.NextDoubles(n_t);
		else if (tok == "UPPER") c.upper_ref = r.NextMat(n_t, 2);
		else if (tok == "LOWER") c.lower_ref = r.NextMat(n_t, 2);
		else if (tok == "NX") n_x = r.NextInt();
		else if (tok == "XVALUES") c.x_values = r.NextDoubles(n_x);
		else if (tok == "CAMBER") c.camber_ref = r.NextDoubles(n_x);
		else if (tok == "THICKNESS") c.thickness_ref = r.NextDoubles(n_x);
		else if (tok == "MAXT") { c.maxt_x = r.NextDouble(); c.maxt_v = r.NextDouble(); }
		else if (tok == "MAXC") { c.maxc_x = r.NextDouble(); c.maxc_v = r.NextDouble(); }
		else if (tok == "NCONTOUR") n_contour = r.NextInt();
		else if (tok == "CONTOUR") c.contour_ref = r.NextMat(n_contour, 2);
		else throw std::runtime_error("unknown field '" + tok + "' in case " + name);
	}
	c.curve_spec.degree = degree;
	c.curve_spec.periodic = periodic != 0;
	return c;
}

void CheckAgainstCase(const Airfoil& af, const CaseData& c) {
	Mat upper = af.UpperXY(c.t_values);
	Mat lower = af.LowerXY(c.t_values);
	CheckMatClose(upper, c.upper_ref, 1e-8, c.name + " upper_xy");
	CheckMatClose(lower, c.lower_ref, 1e-8, c.name + " lower_xy");

	auto camber = af.Camber(c.x_values);
	auto thickness = af.Thickness(c.x_values);
	for (size_t i = 0; i < c.x_values.size(); ++i) {
		CheckClose(camber[i], c.camber_ref[i], 1e-7, c.name + " camber[" + std::to_string(i) + "]");
		CheckClose(thickness[i], c.thickness_ref[i], 1e-7, c.name + " thickness[" + std::to_string(i) + "]");
	}

	auto [mt_x, mt_v] = af.MaxThickness();
	auto [mc_x, mc_v] = af.MaxCamber();
	CheckClose(mt_x, c.maxt_x, 1e-9, c.name + " max_thickness x");
	CheckClose(mt_v, c.maxt_v, 1e-7, c.name + " max_thickness v");
	CheckClose(mc_x, c.maxc_x, 1e-9, c.name + " max_camber x");
	CheckClose(mc_v, c.maxc_v, 1e-7, c.name + " max_camber v");

	Mat contour = af.ContourPoints(200);
	CheckMatClose(contour, c.contour_ref, 1e-8, c.name + " contour_points");
}

Mat BuildHandmadePts() {
	const int n = 41;
	Mat pts(2 * n - 1, 2);
	std::vector<double> xc(n), thick(n), camb(n);
	for (int i = 0; i < n; ++i) {
		double th = 3.14159265358979323846 * i / (n - 1);
		xc[i] = 0.5 * (1.0 - std::cos(th));
		thick[i] = 0.08 * std::sin(th);
		camb[i] = 0.03 * std::sin(th) * (1.0 - xc[i]);
	}
	for (int i = 0; i < n; ++i) {
		int row = n - 1 - i; // lower, reversed
		pts(row, 0) = xc[i];
		pts(row, 1) = camb[i] - thick[i];
	}
	for (int i = 1; i < n; ++i) {
		int row = n - 1 + i; // upper, skipping index 0
		pts(row, 0) = xc[i];
		pts(row, 1) = camb[i] + thick[i];
	}
	return pts;
}

} // namespace

int main() {
	try {
		std::ifstream in(AEROTELIC_TESTS_DIR "/airfoil_fixtures.txt");
		if (!in) throw std::runtime_error("cannot open airfoil_fixtures.txt");
		TokenReader r(in);

		std::vector<CaseData> cases;
		while (r.HasNext()) {
			r.Expect("CASE");
			std::string name = r.NextTok();
			cases.push_back(ParseCase(r, name));
		}

		for (const auto& c : cases) {
			if (c.name.rfind("naca_", 0) == 0) {
				std::string code = c.name.substr(5);
				auto af = Airfoil::Naca4(code);
				CheckAgainstCase(*af, c);
			} else if (c.name == "handmade") {
				auto af = Airfoil::FromPoints(BuildHandmadePts(), 3, "handmade");
				CheckAgainstCase(*af, c);
			} else if (c.name == "handmade_roundtrip") {
				auto af = Airfoil::FromCurve(c.curve_spec, "handmade_roundtrip");
				CheckAgainstCase(*af, c);
			} else {
				throw std::runtime_error("unrecognized case name " + c.name);
			}
		}
	} catch (const std::exception& e) {
		std::cout << "FIXTURE PARSE/RUN ERROR: " << e.what() << "\n";
		return 1;
	}

	std::cout << "\nRESULT: " << g_pass << " passed, " << g_fail << " failed\n";
	return g_fail == 0 ? 0 : 1;
}
