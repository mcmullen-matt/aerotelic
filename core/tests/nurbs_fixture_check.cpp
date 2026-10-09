// Cross-checks the C++ NurbsCurve port against reference values dumped
// directly from core/nurbs.py (see tools/generate_nurbs_fixtures.py) --
// not an independently re-derived "brute force" reference, deliberately:
// the point is to catch divergence from the actual Python behavior this
// is replacing, not to re-prove the NURBS math from scratch.
//
// Also runs a few self-contained checks (Boehm knot insertion shape
// preservation, basis_matrix partition-of-unity) that don't need a
// Python reference because they're internal consistency properties.

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <stdexcept>

#include "aerotelic/nurbs.hpp"
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

void RunCurveCase(const std::string& name, TokenReader& r) {
	int degree = -1, periodic = 0, clamped_seam = 0, n_poles = -1, dim = -1;
	Mat poles;
	std::vector<double> weights, knots, samples;
	Mat eval_ref, deriv_ref;
	int n_samples = -1;

	for (;;) {
		std::string tok = r.NextTok();
		if (tok == "END") break;
		if (tok == "DEGREE") degree = r.NextInt();
		else if (tok == "PERIODIC") periodic = r.NextInt();
		else if (tok == "CLAMPED_SEAM") clamped_seam = r.NextInt();
		else if (tok == "NPOLES") n_poles = r.NextInt();
		else if (tok == "DIM") dim = r.NextInt();
		else if (tok == "POLES") poles = r.NextMat(n_poles, dim);
		else if (tok == "WEIGHTS") weights = r.NextDoubles(n_poles);
		else if (tok == "KNOTS") {
			int n_knots = periodic ? n_poles : (n_poles - degree - 1);
			knots = r.NextDoubles(n_knots);
		} else if (tok == "NSAMPLES") n_samples = r.NextInt();
		else if (tok == "SAMPLES") samples = r.NextDoubles(n_samples);
		else if (tok == "EVAL") eval_ref = r.NextMat(n_samples, dim);
		else if (tok == "DERIV") deriv_ref = r.NextMat(n_samples, dim);
		else throw std::runtime_error("unknown field '" + tok + "' in case " + name);
	}

	NurbsSpec spec;
	spec.degree = degree;
	spec.poles = poles;
	spec.weights = weights;
	spec.periodic = periodic != 0;
	spec.clamped_seam = clamped_seam != 0;
	if (periodic) spec.knots = knots; else spec.interior_knots = knots;

	auto curve = NurbsCurve::Create(spec);

	for (int k = 0; k < n_samples; ++k) {
		std::vector<double> c = curve->Evaluate(samples[k]);
		for (int d = 0; d < dim; ++d) {
			CheckClose(c[d], eval_ref(k, d), 1e-9, name + " evaluate[" + std::to_string(k) + "][" + std::to_string(d) + "]");
		}
		std::vector<double> dc = curve->Derivative(samples[k]);
		for (int d = 0; d < dim; ++d) {
			CheckClose(dc[d], deriv_ref(k, d), 1e-7, name + " derivative[" + std::to_string(k) + "][" + std::to_string(d) + "]");
		}
	}
}

void RunFitCurveCase(const std::string& name, TokenReader& r) {
	int degree = -1, n_pts = -1, dim = -1;
	Mat pts;
	std::vector<double> params_ref, interior_ref;
	Mat poles_ref;

	for (;;) {
		std::string tok = r.NextTok();
		if (tok == "END") break;
		if (tok == "DEGREE") degree = r.NextInt();
		else if (tok == "NPTS") n_pts = r.NextInt();
		else if (tok == "DIM") dim = r.NextInt();
		else if (tok == "PTS") pts = r.NextMat(n_pts, dim);
		else if (tok == "PARAMS") params_ref = r.NextDoubles(n_pts);
		else if (tok == "POLES") poles_ref = r.NextMat(n_pts, dim);
		else if (tok == "INTERIOR_KNOTS") {
			int n_int = n_pts - degree - 1;
			interior_ref = r.NextDoubles(n_int);
		} else throw std::runtime_error("unknown field '" + tok + "' in case " + name);
	}

	FitCurve fit(pts, degree);
	auto params = fit.collocation_parameters();
	Check(static_cast<int>(params.size()) == static_cast<int>(params_ref.size()), name + " param count");
	for (size_t i = 0; i < params_ref.size(); ++i) CheckClose(params[i], params_ref[i], 1e-9, name + " param[" + std::to_string(i) + "]");

	Mat poles = fit.poles();
	for (int i = 0; i < n_pts; ++i)
		for (int d = 0; d < dim; ++d)
			CheckClose(poles(i, d), poles_ref(i, d), 1e-6, name + " pole[" + std::to_string(i) + "][" + std::to_string(d) + "]");

	auto interior = fit.interior_knots();
	for (size_t i = 0; i < interior_ref.size(); ++i) CheckClose(interior[i], interior_ref[i], 1e-9, name + " interior_knot[" + std::to_string(i) + "]");

	// The defining property: the fitted curve interpolates the data exactly.
	for (int i = 0; i < n_pts; ++i) {
		std::vector<double> c = fit.Evaluate(params[i]);
		for (int d = 0; d < dim; ++d) CheckClose(c[d], pts(i, d), 1e-8, name + " interpolates pt[" + std::to_string(i) + "][" + std::to_string(d) + "]");
	}
}

void RunDihedralCase(const std::string& name, TokenReader& r) {
	int n_poles = -1, dim = -1;
	Mat poles;
	std::vector<double> interior_ref;

	for (;;) {
		std::string tok = r.NextTok();
		if (tok == "END") break;
		if (tok == "NPOLES") n_poles = r.NextInt();
		else if (tok == "DIM") dim = r.NextInt();
		else if (tok == "POLES") poles = r.NextMat(n_poles, dim);
		else if (tok == "INTERIOR_KNOTS") interior_ref = r.NextDoubles(n_poles - 3 - 1);
		else throw std::runtime_error("unknown field '" + tok + "' in case " + name);
	}

	NurbsSpec spec;
	spec.degree = 3;
	spec.poles = poles;
	DihedralCurve d(spec);
	auto interior = d.interior_knots();
	for (size_t i = 0; i < interior_ref.size(); ++i) CheckClose(interior[i], interior_ref[i], 1e-9, name + " interior_knot[" + std::to_string(i) + "]");
}

void RunFixtureFile(const std::string& path) {
	std::ifstream in(path);
	if (!in) throw std::runtime_error("cannot open " + path);
	TokenReader r(in);
	while (r.HasNext()) {
		r.Expect("CASE");
		std::string name = r.NextTok();
		if (name.rfind("fitcurve_", 0) == 0) RunFitCurveCase(name, r);
		else if (name.rfind("dihedral_", 0) == 0) RunDihedralCase(name, r);
		else RunCurveCase(name, r);
	}
}

// ---------------------------------------------------------------------
// Self-contained checks (no Python reference needed).
// ---------------------------------------------------------------------

void CheckInsertKnotPreservesShape() {
	{
		NurbsSpec spec;
		spec.degree = 3;
		spec.poles = Mat::FromRows({{0,0},{1,2},{2,-1},{3,1.5},{4,0},{5,2},{6,-0.5},{7,1}});
		spec.weights = std::vector<double>{1,1.2,0.9,1.1,1.3,0.8,1.0,1.1};
		ClampedNurbsCurve c(spec);
		std::vector<double> s;
		for (int i = 0; i <= 400; ++i) s.push_back(i / 400.0);
		Mat before = c.Evaluate(s);
		for (double ub : {0.37, 0.37, 0.05, 0.9, 0.37}) c.InsertKnot(ub);
		Check(c.n_poles() == 8 + 5, "clamped insert_knot pole count");
		Mat after = c.Evaluate(s);
		double maxdiff = 0.0;
		for (int i = 0; i < before.rows(); ++i)
			for (int j = 0; j < before.cols(); ++j)
				maxdiff = std::max(maxdiff, std::abs(before(i, j) - after(i, j)));
		Check(maxdiff < 1e-9, "clamped insert_knot shape preserved (maxdiff=" + std::to_string(maxdiff) + ")");
		bool threw = false;
		try { c.InsertKnot(0.37); } catch (const std::invalid_argument&) { threw = true; }
		Check(threw, "clamped insert_knot rejects multiplicity > degree");
	}
	{
		NurbsSpec spec;
		spec.degree = 3;
		spec.periodic = true;
		spec.poles = Mat::FromRows({{1,0},{0.7,0.7},{0,1},{-0.7,0.7},{-1,0},{-0.7,-0.7}});
		spec.weights = std::vector<double>{1,1.1,0.9,1.2,0.8,1.0};
		PeriodicNurbsCurve c(spec);
		std::vector<double> s;
		for (int i = 0; i <= 400; ++i) s.push_back(i / 400.0);
		Mat before = c.Evaluate(s);
		for (double ub : {0.0, 0.02, 0.98, 0.0}) c.InsertKnot(ub);
		Mat after = c.Evaluate(s);
		double maxdiff = 0.0;
		for (int i = 0; i < before.rows(); ++i)
			for (int j = 0; j < before.cols(); ++j)
				maxdiff = std::max(maxdiff, std::abs(before(i, j) - after(i, j)));
		Check(maxdiff < 1e-9, "periodic insert_knot shape preserved (maxdiff=" + std::to_string(maxdiff) + ")");
		Check(c.knots()[0] == 0.0, "periodic insert_knot keeps seam at 0");
		Check(c.n_poles() == static_cast<int>(c.knots().size()) && c.n_poles() == 6 + 4, "periodic insert_knot pole/knot count");
	}
}

void CheckBasisMatrixPartitionOfUnity() {
	NurbsSpec spec;
	spec.degree = 2;
	spec.poles = Mat::FromRows({{0,0},{1,2},{2,-1},{3,1},{4,0},{5,1.5},{6,-1}});
	spec.weights = std::vector<double>{1,1.1,0.9,1.2,0.8,1.0,1.1};
	ClampedNurbsCurve c(spec);
	std::vector<double> s;
	for (int i = 0; i <= 96; ++i) s.push_back(i / 96.0);
	Mat N = c.BasisMatrix(s, false);
	Mat R = c.BasisMatrix(s, true);
	Mat evald = c.Evaluate(s);
	for (int k = 0; k < N.rows(); ++k) {
		double sumN = 0.0, sumR = 0.0;
		for (int i = 0; i < N.cols(); ++i) { sumN += N(k, i); sumR += R(k, i); }
		CheckClose(sumN, 1.0, 1e-9, "basis_matrix partition of unity (N)");
		CheckClose(sumR, 1.0, 1e-9, "basis_matrix partition of unity (R)");
		for (int d = 0; d < evald.cols(); ++d) {
			double recon = 0.0;
			for (int i = 0; i < R.cols(); ++i) recon += R(k, i) * c.poles()(i, d);
			CheckClose(recon, evald(k, d), 1e-9, "basis_matrix reproduces evaluate()");
		}
	}
}

} // namespace

int main() {
	try {
		RunFixtureFile(AEROTELIC_TESTS_DIR "/nurbs_fixtures.txt");
	} catch (const std::exception& e) {
		std::cout << "FIXTURE PARSE ERROR: " << e.what() << "\n";
		return 1;
	}
	CheckInsertKnotPreservesShape();
	CheckBasisMatrixPartitionOfUnity();

	std::cout << "\nRESULT: " << g_pass << " passed, " << g_fail << " failed\n";
	return g_fail == 0 ? 0 : 1;
}
