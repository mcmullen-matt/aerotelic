// Validates the Theodorsen / Ives conformal map (conformal_airfoil.hpp)
// against two airfoils with closed-form potential flow:
//   - a Joukowski airfoil (cusped trailing edge), whose exact surface
//     speed follows from inverting z = zeta + a^2/zeta;
//   - a van de Vooren airfoil (finite trailing-edge angle; Katz & Plotkin,
//     Low-Speed Aerodynamics, section 6.6), compared by interpolation in x.
// Plus the integrated lift against Kutta-Joukowski.

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <vector>

#include "aerotelic/conformal_airfoil.hpp"

using namespace aerotelic;
using cd = std::complex<double>;

namespace {

int g_failures = 0;
constexpr double kPi = 3.14159265358979323846;

void Check(bool ok, const char* what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what);
	if (!ok) ++g_failures;
}

// Pressure integration of a counter-clockwise closed polygon, as the section
// layer does it, to compare with the Kutta-Joukowski lift.
double IntegratedCl(const ConformalMap& m, const ConformalFlow& f) {
	const int n = static_cast<int>(m.z.size());
	double cx = 0.0, cy = 0.0;
	for (int i = 0; i < n; ++i) {
		const int j = (i + 1) % n;
		const double dx = m.z[j].real() - m.z[i].real(), dy = m.z[j].imag() - m.z[i].imag();
		const double c = 0.5 * (f.cp[i] + f.cp[j]);
		cx += -c * dy;
		cy += c * dx;
	}
	cx /= m.chord;
	cy /= m.chord;
	return cy * std::cos(f.alpha) - cx * std::sin(f.alpha);
}

} // namespace

int main() {
	setvbuf(stdout, nullptr, _IONBF, 0);

	// ---- Joukowski ----------------------------------------------------------------
	{
		const double a = 1.0;
		const cd mu(-0.1, 0.08);
		const double R = std::abs(a - mu);
		const int M = 400;
		const double beta_te = std::arg(a - mu); // the cusp, zeta = a
		Mat contour(M + 1, 2);
		for (int k = 0; k <= M; ++k) {
			const double beta = beta_te + 2.0 * kPi - 2.0 * kPi * k / M; // clockwise: lower TE -> LE -> upper TE
			const cd zeta = mu + R * std::polar(1.0, beta);
			const cd z = zeta + a * a / zeta;
			contour(k, 0) = z.real();
			contour(k, 1) = z.imag();
		}
		ConformalMapOptions opt;
		const ConformalMap m = MapAirfoil(contour, opt);
		std::printf("Joukowski: ok=%d iterations=%d residual=%.1e max|eps'|=%.3f tau=%.2f deg n=%.4f (%s)\n",
			m.ok, m.iterations, m.residual, m.max_eps_prime, m.te_angle * 180 / kPi, m.n_kt, m.error.c_str());
		Check(m.ok, "Joukowski airfoil maps");
		Check(m.te_angle < 0.02, "Joukowski trailing edge recognised as a cusp");
		if (m.ok) {
			const double alpha_deg = 4.0;
			const ConformalFlow f = SolveFlow(m, alpha_deg);
			const double alpha = alpha_deg * kPi / 180.0;
			const double gamma_over_u = -4.0 * kPi * R * std::sin(beta_te - alpha);
			double worst = 0.0, worst_x = 0.0, worst_away = 0.0;
			const int N = static_cast<int>(m.z.size());
			for (int j = 0; j < N; ++j) {
				const cd z = m.z[j];
				// invert z = zeta + a^2/zeta: the root outside the circle
				const cd s = std::sqrt(z * z - 4.0 * a * a);
				cd z1 = 0.5 * (z + s), z2 = 0.5 * (z - s);
				const cd zeta = (std::abs(z1 - mu) > std::abs(z2 - mu)) ? z1 : z2;
				const double beta = std::arg(zeta - mu);
				const double wc = std::abs(2.0 * std::sin(beta - alpha) + gamma_over_u / (2.0 * kPi * R));
				const double q = wc / std::abs(1.0 - a * a / (zeta * zeta));
				const double cp_exact = 1.0 - q * q;
				const double e = std::fabs(cp_exact - f.cp[j]);
				if (e > worst) { worst = e; worst_x = (z.real() - m.x_min) / m.chord; }
				if (std::abs(z - m.z_te) > 0.02 * m.chord) worst_away = std::fmax(worst_away, e);
			}
			std::printf("Joukowski Cp: worst |dCp| = %.2e at x/c = %.3f; away from the TE %.2e; Cl kutta %.5f integrated %.5f alpha0 %.3f deg\n",
				worst, worst_x, worst_away, f.cl_kutta, IntegratedCl(m, f), f.alpha0 * 180 / kPi);
			Check(worst_away < 2e-3, "Joukowski Cp matches the exact solution to 2e-3 away from the trailing edge");
			Check(worst < 2e-2, "Joukowski Cp near the cusp within 2e-2");
			const double cl_exact = 8.0 * kPi * (R / m.chord) * std::sin(alpha - beta_te);
			Check(std::fabs(f.cl_kutta - cl_exact) < 2e-3, "Kutta-Joukowski lift matches the exact circle's");
			Check(std::fabs(IntegratedCl(m, f) - f.cl_kutta) < 3e-3, "integrated pressure lift matches Kutta-Joukowski");
		}
	}

	// ---- van de Vooren ---------------------------------------------------------------
	{
		const double tau_deg = 10.0, eps = 0.05, ell = 1.0;
		const double k = 2.0 - tau_deg / 180.0;
		const double a = 2.0 * ell * std::pow(1.0 + eps, k - 1.0) / std::pow(2.0, k);
		auto zmap = [&](cd zeta) { return std::pow(zeta - a, k) / std::pow(zeta - a * eps, k - 1.0) + ell; };
		auto dzmap = [&](cd zeta) {
			return std::pow(zeta - a, k - 1.0) * (k * (zeta - a * eps) - (k - 1.0) * (zeta - a)) / std::pow(zeta - a * eps, k);
		};
		const int M = 600;
		Mat contour(M + 1, 2);
		for (int i = 0; i <= M; ++i) {
			const double beta = 2.0 * kPi - 2.0 * kPi * i / M;
			const cd z = zmap(a * std::polar(1.0, beta));
			contour(i, 0) = z.real();
			contour(i, 1) = z.imag();
		}
		const ConformalMap m = MapAirfoil(contour);
		std::printf("van de Vooren: ok=%d iterations=%d residual=%.1e max|eps'|=%.3f tau=%.2f deg (target %.1f) n=%.4f (%s)\n",
			m.ok, m.iterations, m.residual, m.max_eps_prime, m.te_angle * 180 / kPi, tau_deg, m.n_kt, m.error.c_str());
		Check(m.ok, "van de Vooren airfoil maps");
		Check(std::fabs(m.te_angle * 180 / kPi - tau_deg) < 1.5, "trailing-edge angle measured within 1.5 deg");
		if (m.ok) {
			const double alpha_deg = 5.0;
			const ConformalFlow f = SolveFlow(m, alpha_deg);
			const double alpha = alpha_deg * kPi / 180.0;
			// Exact Cp on each surface, densely in beta, as (x, cp) tables.
			const double gamma_over_u = -4.0 * kPi * a * std::sin(0.0 - alpha);
			std::vector<double> xu, cu, xl, cl;
			const int D = 20000;
			for (int i = 1; i < D; ++i) {
				const double beta = kPi * i / D; // upper: beta in (0, pi)
				for (int side = 0; side < 2; ++side) {
					const double b = side == 0 ? beta : 2.0 * kPi - beta;
					const cd zeta = a * std::polar(1.0, b);
					const cd z = zmap(zeta);
					const double wc = std::abs(2.0 * std::sin(b - alpha) + gamma_over_u / (2.0 * kPi * a));
					const double q = wc / std::abs(dzmap(zeta));
					(side == 0 ? xu : xl).push_back(z.real());
					(side == 0 ? cu : cl).push_back(1.0 - q * q);
				}
			}
			auto interp = [](const std::vector<double>& xs, const std::vector<double>& ys, double x) {
				// xs runs from the TE (x = ell) to the LE (x = -ell): decreasing
				for (size_t i = 1; i < xs.size(); ++i) {
					if ((xs[i - 1] >= x && x >= xs[i]) || (xs[i - 1] <= x && x <= xs[i])) {
						const double t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
						return ys[i - 1] + t * (ys[i] - ys[i - 1]);
					}
				}
				return std::nan("");
			};
			double worst = 0.0, worst_x = 0.0;
			int compared = 0;
			for (size_t j = 0; j < m.z.size(); ++j) {
				const cd z = m.z[j];
				if (std::fabs(z.imag()) < 2e-3 || std::abs(z - m.z_te) < 0.02 * m.chord) continue;
				const double ref = z.imag() > 0 ? interp(xu, cu, z.real()) : interp(xl, cl, z.real());
				if (std::isnan(ref)) continue;
				++compared;
				const double e = std::fabs(ref - f.cp[j]);
				if (e > worst) { worst = e; worst_x = (z.real() - m.x_min) / m.chord; }
			}
			std::printf("van de Vooren Cp: %d points compared, worst |dCp| = %.2e at x/c = %.3f; Cl kutta %.5f exact %.5f integrated %.5f\n",
				compared, worst, worst_x, f.cl_kutta, 8.0 * kPi * (a / m.chord) * std::sin(alpha), IntegratedCl(m, f));
			Check(compared > 300, "most grid points compared");
			Check(worst < 3e-3, "van de Vooren Cp matches the exact solution to 3e-3 (2% of chord from the TE excluded)");
			Check(std::fabs(f.cl_kutta - 8.0 * kPi * (a / m.chord) * std::sin(alpha)) < 3e-3, "van de Vooren lift matches 8 pi (a/c) sin alpha");
			Check(std::fabs(IntegratedCl(m, f) - f.cl_kutta) < 3e-3, "integrated lift matches Kutta-Joukowski (finite TE angle)");
			const ConformalFlow f0 = SolveFlow(m, 0.0);
			Check(std::fabs(f0.cl_kutta) < 1e-6 && std::fabs(IntegratedCl(m, f0)) < 1e-4, "symmetric section has no lift at zero incidence");
		}
	}

	// ---- an open NACA 0012 closed by the aft blend (the section layer's closure) ----
	{
		const int n = 61;
		Mat pts(2 * n - 1, 2);
		for (int i = 0; i < n; ++i) {
			const double u = kPi * i / (n - 1);
			const double x = 0.5 * (1.0 - std::cos(u));
			const double yt = 0.6 * (0.2969 * std::sqrt(x) - 0.1260 * x - 0.3516 * x * x + 0.2843 * x * x * x - 0.1015 * x * x * x * x);
			pts(n - 1 - i, 0) = x; pts(n - 1 - i, 1) = -yt;
			if (i > 0) { pts(n - 1 + i, 0) = x; pts(n - 1 + i, 1) = yt; }
		}
		const double gap = pts(2 * n - 2, 1) - pts(0, 1), x0 = 0.7;
		for (int i = 0; i < pts.rows(); ++i) {
			const double x = pts(i, 0);
			if (x <= x0) continue;
			const double f = (x - x0) / (1.0 - x0);
			pts(i, 1) += (i <= n - 1 ? 0.5 : -0.5) * gap * f * f;
		}
		pts(0, 1) = pts(2 * n - 2, 1) = 0.0;
		const ConformalMap m = MapAirfoil(pts);
		std::printf("blended NACA 0012: ok=%d it=%d residual=%.1e max|eps'|=%.3f tau=%.2f deg phi_te=%.4f psi0=%.4f (%s)\n",
			m.ok, m.iterations, m.residual, m.max_eps_prime, m.te_angle * 180 / kPi, m.phi_te, m.psi0, m.error.c_str());
		if (!m.eps.empty()) {
			double emin = m.eps[0], emax = m.eps[0];
			for (double e : m.eps) { emin = std::fmin(emin, e); emax = std::fmax(emax, e); }
			std::printf("   eps range %.4f .. %.4f, theta0 %.4f thetaN %.4f\n", emin, emax, m.theta.front(), m.theta.back());
		}
		Check(m.ok, "blend-closed NACA 0012 maps");
	}

	std::printf("RESULT: %d failure(s)\n", g_failures);
	return g_failures == 0 ? 0 : 1;
}
