// Checks the in-house FFT (fft.hpp) against a direct DFT, the spectral
// helpers (fourier_periodic.hpp) against closed forms, and the periodic
// cubic spline against a smooth periodic function.

#include <cmath>
#include <complex>
#include <cstdio>
#include <vector>

#include "aerotelic/fft.hpp"
#include "aerotelic/fourier_periodic.hpp"
#include "aerotelic/periodic_spline.hpp"

using namespace aerotelic;
using cd = std::complex<double>;

namespace {

int g_failures = 0;
constexpr double kPi = 3.14159265358979323846;

void Check(bool ok, const char* what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what);
	if (!ok) ++g_failures;
}

double Rand(unsigned& s) { s = s * 1664525u + 1013904223u; return (s >> 8) / 16777216.0 - 0.5; }

} // namespace

int main() {
	setvbuf(stdout, nullptr, _IONBF, 0);

	// ---- FFT vs direct DFT, and the inverse ------------------------------------
	for (int n : {1, 2, 8, 64, 512}) {
		unsigned seed = 7u + n;
		std::vector<cd> x(n), X;
		for (auto& v : x) v = cd(Rand(seed), Rand(seed));
		X = x;
		fft::Forward(X);
		double err = 0.0;
		for (int k = 0; k < n; ++k) {
			cd s = 0.0;
			for (int j = 0; j < n; ++j) s += x[j] * std::polar(1.0, -2.0 * kPi * j * k / n);
			err = std::fmax(err, std::abs(s - X[k]));
		}
		char what[64];
		std::snprintf(what, sizeof what, "forward FFT matches the DFT for N = %d (err %.1e)", n, err);
		Check(err < 1e-11 * n, what);
		fft::Inverse(X);
		double rt = 0.0;
		for (int j = 0; j < n; ++j) rt = std::fmax(rt, std::abs(X[j] - x[j]));
		std::snprintf(what, sizeof what, "inverse(forward(x)) == x for N = %d (err %.1e)", n, rt);
		Check(rt < 1e-12 * n, what);
	}
	Check(!fft::IsSupportedSize(12) && fft::IsSupportedSize(1024), "supported sizes are powers of two");

	// ---- conjugate and derivative ------------------------------------------------
	{
		const int n = 128;
		std::vector<double> f(n), g_ref(n), d_ref(n);
		for (int j = 0; j < n; ++j) {
			const double phi = 2.0 * kPi * j / n;
			// f = Re(1/zeta + 0.3/zeta^3) on |zeta| = 1 -> cos phi + 0.3 cos 3phi
			f[j] = std::cos(phi) + 0.3 * std::cos(3.0 * phi) + 2.0;
			g_ref[j] = -std::sin(phi) - 0.3 * std::sin(3.0 * phi);
			d_ref[j] = -std::sin(phi) - 0.9 * std::sin(3.0 * phi);
		}
		std::vector<double> g = ConjugateExterior(f), d = DerivativePeriodic(f);
		double eg = 0.0, ed = 0.0;
		for (int j = 0; j < n; ++j) { eg = std::fmax(eg, std::fabs(g[j] - g_ref[j])); ed = std::fmax(ed, std::fabs(d[j] - d_ref[j])); }
		Check(eg < 1e-12, "exterior conjugate of Re(1/zeta + 0.3/zeta^3) is Im(...) (sign convention)");
		Check(ed < 1e-11, "spectral derivative");
	}

	// ---- periodic cubic spline ---------------------------------------------------
	{
		std::vector<double> x, y;
		unsigned seed = 3u;
		double t = -3.0;
		while (t < -3.0 + 2.0 * kPi - 0.035) { x.push_back(t); y.push_back(std::sin(t) + 0.5 * std::cos(2.0 * t)); t += 0.03 + 0.02 * (Rand(seed) + 0.5); }
		PeriodicCubicSpline s(x, y, 2.0 * kPi);
		double ev = 0.0, ed = 0.0;
		for (int k = 0; k < 1000; ++k) {
			const double u = -10.0 + 20.0 * k / 999.0;
			ev = std::fmax(ev, std::fabs(s.Evaluate(u) - (std::sin(u) + 0.5 * std::cos(2.0 * u))));
			ed = std::fmax(ed, std::fabs(s.Derivative(u) - (std::cos(u) - std::sin(2.0 * u))));
		}
		std::printf("spline errors: value %.2e derivative %.2e\n", ev, ed);
		Check(ev < 2e-6, "periodic spline reproduces sin t + 0.5 cos 2t across the seam");
		Check(ed < 2e-4, "periodic spline derivative");
	}

	std::printf("RESULT: %d failure(s)\n", g_failures);
	return g_failures == 0 ? 0 : 1;
}
