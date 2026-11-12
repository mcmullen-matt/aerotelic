#include "aerotelic/conformal_airfoil.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "aerotelic/fft.hpp"
#include "aerotelic/fourier_periodic.hpp"
#include "aerotelic/periodic_spline.hpp"

namespace aerotelic {

namespace {

using cd = std::complex<double>;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

double Reduce(double angle) { // into [0, 2 pi)
	angle -= kTwoPi * std::floor(angle / kTwoPi);
	return angle;
}

// Walks the contour from index `from` in steps of `step` until the arc
// length from the start point exceeds `length`; returns that vertex.
cd PointAtArcLength(const std::vector<cd>& v, int from, int step, double length) {
	double s = 0.0;
	int i = from;
	const int n = static_cast<int>(v.size());
	for (int k = 0; k < n - 1; ++k) {
		const int j = (i + step + n) % n;
		s += std::abs(v[j] - v[i]);
		i = j;
		if (s >= length) break;
	}
	return v[i];
}

// Direction in which a surface leaves the trailing edge: a least-squares
// quadratic y = s x + k x^2 (local coordinates with the trailing edge at the
// origin) through the vertices within `length` of it along the contour, so
// that the curvature term absorbs a cusp's or a curved surface's bend and
// `s` is the tangent slope. Falls back to the chord to the farthest such
// vertex when there are too few points.
cd TrailingEdgeTangent(const std::vector<cd>& v, int step, double length) {
	const int n = static_cast<int>(v.size());
	const cd z_te = v[0];
	std::vector<cd> pts;
	double s_acc = 0.0;
	int i = 0;
	for (int k = 0; k < n - 1; ++k) {
		const int j = (i + step + n) % n;
		s_acc += std::abs(v[j] - v[i]);
		i = j;
		pts.push_back(v[j] - z_te);
		if (s_acc >= length) break;
	}
	if (pts.empty()) return cd(-1.0, 0.0);
	// Local frame: x along the chord from the TE to the farthest collected
	// point, y across it.
	const cd far = pts.back();
	const cd ex = far / std::abs(far);
	double sxx = 0.0, sxxx = 0.0, sxxxx = 0.0, sxy = 0.0, sxxy = 0.0;
	for (const cd& p : pts) {
		const double x = p.real() * ex.real() + p.imag() * ex.imag();
		const double y = -p.real() * ex.imag() + p.imag() * ex.real();
		sxx += x * x; sxxx += x * x * x; sxxxx += x * x * x * x; sxy += x * y; sxxy += x * x * y;
	}
	const double det = sxx * sxxxx - sxxx * sxxx;
	double slope = 0.0;
	if (pts.size() >= 3 && std::abs(det) > 1e-30) slope = (sxy * sxxxx - sxxy * sxxx) / det;
	const cd d_local(1.0, slope);
	const cd d = ex * d_local; // rotate back
	return d / std::abs(d);
}

// Radius of the circle through three points (infinite for collinear).
double CircumRadius(cd p, cd q, cd r) {
	const double a = std::abs(q - r), b = std::abs(p - r), c = std::abs(p - q);
	const double s = 0.5 * (a + b + c);
	const double area2 = s * (s - a) * (s - b) * (s - c);
	if (area2 <= 0.0) return 1e300;
	return a * b * c / (4.0 * std::sqrt(area2));
}

// Normalised Fourier coefficients of a real grid function, k = 0..N-1 in FFT
// order (negative frequencies in the upper half).
std::vector<cd> Coefficients(const std::vector<double>& f) {
	const int n = static_cast<int>(f.size());
	std::vector<cd> a(n);
	for (int j = 0; j < n; ++j) a[j] = cd(f[j], 0.0);
	fft::Forward(a);
	for (auto& v : a) v /= static_cast<double>(n);
	return a;
}

// Trigonometric interpolation of the series at phi (and its derivative).
void EvalSeries(const std::vector<cd>& F, double phi, double& value, double& derivative) {
	const int n = static_cast<int>(F.size());
	cd v = F[0], d = 0.0;
	for (int k = 1; k < n / 2; ++k) {
		const cd e = std::polar(1.0, k * phi);
		const cd term = F[k] * e + F[n - k] * std::conj(e);
		v += term;
		d += cd(0.0, static_cast<double>(k)) * (F[k] * e - F[n - k] * std::conj(e));
	}
	value = v.real();
	derivative = d.real();
}

struct PointData {
	cd z;
	double dz_dzeta_abs;
};

PointData PointAt(const ConformalMap& m, double phi_rel, double psi, double eps, double dpsi, double deps) {
	const double theta = phi_rel + eps;
	const cd zp_over_a = std::exp(cd(psi, theta));          // zeta' / a
	const cd w = (zp_over_a - 1.0) / (zp_over_a + 1.0);     // Karman-Trefftz variable
	const double n = m.n_kt;
	const cd wn = std::pow(w, n);
	const cd z = (m.z_te - m.z_nose * wn) / (1.0 - wn);
	// dz/dzeta' = 4 n^2 a^2 w^(n-1) / ((1 - w^n)^2 (zeta' + a)^2)
	const cd zeta_p = m.a * zp_over_a;
	const cd dz_dzp = 4.0 * n * n * m.a * m.a * std::pow(w, n - 1.0) / ((1.0 - wn) * (1.0 - wn) * (zeta_p + m.a) * (zeta_p + m.a));
	const double dzp_dzeta = std::exp(psi - m.psi0) * std::sqrt((1.0 + deps) * (1.0 + deps) + dpsi * dpsi);
	return {z, std::abs(dz_dzp) * dzp_dzeta};
}

} // namespace

ConformalMap MapAirfoil(const Mat& contour, const ConformalMapOptions& options) {
	ConformalMap m;
	auto fail = [&](const std::string& why) { m.ok = false; m.error = why; return m; };

	if (contour.cols() != 2 || contour.rows() < 8) return fail("conformal map: need an (M, 2) contour with M >= 8");
	if (!fft::IsSupportedSize(options.n_points) || options.n_points < 32) return fail("conformal map: n_points must be a power of two >= 32");

	// ---- vertices, closed trailing edge -------------------------------------
	const int M = contour.rows();
	std::vector<cd> v;
	v.reserve(M);
	for (int i = 0; i < M; ++i) v.emplace_back(contour(i, 0), contour(i, 1));
	double xmin = v[0].real(), xmax = v[0].real();
	for (const cd& p : v) { xmin = std::min(xmin, p.real()); xmax = std::max(xmax, p.real()); }
	m.chord = xmax - xmin;
	m.x_min = xmin;
	if (!(m.chord > 0.0)) return fail("conformal map: degenerate chord");
	if (std::abs(v.front() - v.back()) > 1e-6 * m.chord) return fail("conformal map: the trailing edge must be closed (first and last points coincide)");
	const cd z_te = 0.5 * (v.front() + v.back());
	v.pop_back();
	v.front() = z_te;
	const int K = static_cast<int>(v.size());
	m.z_te = z_te;

	// ---- trailing-edge angle, Karman-Trefftz exponent -------------------------
	const cd d_l = TrailingEdgeTangent(v, +1, options.te_tangent_fraction * m.chord);
	const cd d_u = TrailingEdgeTangent(v, -1, options.te_tangent_fraction * m.chord);
	double cos_tau = d_l.real() * d_u.real() + d_l.imag() * d_u.imag();
	cos_tau = std::clamp(cos_tau, -1.0, 1.0);
	m.te_angle = std::acos(cos_tau);
	m.n_kt = 2.0 - m.te_angle / kPi;

	// ---- nose singularity -----------------------------------------------------
	int i_le = 0;
	double far = 0.0;
	for (int i = 0; i < K; ++i) {
		const double d = std::abs(v[i] - z_te);
		if (d > far) { far = d; i_le = i; }
	}
	const cd z_le = v[i_le];
	const cd p_a = PointAtArcLength(v, i_le, +1, 0.01 * m.chord);
	const cd p_b = PointAtArcLength(v, i_le, -1, 0.01 * m.chord);
	double r_le = CircumRadius(p_a, z_le, p_b);
	r_le = std::clamp(r_le, 1e-4 * m.chord, 0.2 * m.chord);
	m.le_radius = r_le;
	const cd u_chord = (z_te - z_le) / std::abs(z_te - z_le);
	m.z_nose = z_le + options.nose_fraction * r_le * u_chord;
	m.a = (z_te - m.z_nose) / (2.0 * m.n_kt);

	// ---- the near-circle: (theta_k, psi_k) for every vertex but the TE ---------
	// r = (z - z_te) / (z - z_nose); its argument runs continuously from about
	// -pi + tau/2 at the lower trailing edge up to pi - tau/2 at the upper one
	// (the contour winds once round z_nose and turns through -tau at z_te), so
	// w = r^(1/n) stays in the right half-plane and zeta' = a (1 + w) / (1 - w)
	// traverses the near-circle clockwise from theta = 0 to theta = -2 pi.
	std::vector<double> theta(K), psi(K);
	theta[0] = 0.0;
	psi[0] = 0.0;
	double chi_prev = 0.0, theta_prev = 0.0;
	for (int k = 1; k < K; ++k) {
		const cd r = (v[k] - z_te) / (v[k] - m.z_nose);
		double chi = std::arg(r);
		if (k == 1) {
			if (chi > 0.0) chi -= kTwoPi; // expected near -pi + tau/2
		} else {
			while (chi - chi_prev > kPi) chi -= kTwoPi;
			while (chi - chi_prev < -kPi) chi += kTwoPi;
		}
		chi_prev = chi;
		const cd w = std::polar(std::pow(std::abs(r), 1.0 / m.n_kt), chi / m.n_kt);
		const cd zp_over_a = (1.0 + w) / (1.0 - w);
		psi[k] = std::log(std::abs(zp_over_a));
		double th = std::arg(zp_over_a);
		if (k == 1) {
			if (th > kPi / 2) th -= kTwoPi; // expected slightly below 0
		} else {
			while (th - theta_prev > kPi) th -= kTwoPi;
			while (th - theta_prev < -kPi) th += kTwoPi;
		}
		theta_prev = th;
		theta[k] = th;
		if (!std::isfinite(psi[k]) || !std::isfinite(th)) return fail("conformal map: the Karman-Trefftz pre-map produced a non-finite point");
	}
	for (int k = 1; k < K; ++k) {
		if (!(theta[k] < theta[k - 1])) return fail("conformal map: the near-circle is not star-shaped about its centre (angle not monotonic at vertex " + std::to_string(k) + ")");
	}
	if (theta[K - 1] <= -kTwoPi) return fail("conformal map: the near-circle winds more than once");

	// Samples for the periodic spline psi(theta), increasing theta, TE last.
	std::vector<double> sx, sy;
	sx.reserve(K);
	sy.reserve(K);
	for (int k = K - 1; k >= 1; --k) { sx.push_back(theta[k]); sy.push_back(psi[k]); }
	sx.push_back(0.0);
	sy.push_back(0.0);
	PeriodicCubicSpline spline(sx, sy, kTwoPi);

	// ---- Theodorsen-Garrick iteration ------------------------------------------
	const int N = options.n_points;
	m.phi.resize(N);
	for (int j = 0; j < N; ++j) m.phi[j] = kTwoPi * j / N;
	std::vector<double> eps(N, 0.0), psi_g(N), eps_new;
	double residual = 0.0;
	int it = 0;
	for (; it < options.max_iterations; ++it) {
		for (int j = 0; j < N; ++j) psi_g[j] = spline.Evaluate(m.phi[j] + eps[j]);
		eps_new = ConjugateExterior(psi_g);
		residual = 0.0;
		for (int j = 0; j < N; ++j) residual = std::max(residual, std::abs(eps_new[j] - eps[j]));
		if (!std::isfinite(residual) || residual > 10.0) return fail("conformal map: Theodorsen's iteration diverged (the section is too far from its near-circle; try a different nose singularity)");
		for (int j = 0; j < N; ++j) eps[j] += options.relaxation * (eps_new[j] - eps[j]);
		if (residual < options.tolerance) { ++it; break; }
	}
	m.iterations = it;
	m.residual = residual;
	for (int j = 0; j < N; ++j) psi_g[j] = spline.Evaluate(m.phi[j] + eps[j]);
	double mean = 0.0;
	for (double p : psi_g) mean += p;
	mean /= N;
	m.psi0 = mean;
	m.psi = psi_g;
	m.eps = eps;
	m.theta.resize(N);
	for (int j = 0; j < N; ++j) m.theta[j] = m.phi[j] + eps[j];
	m.dpsi = DerivativePeriodic(psi_g);
	m.deps = DerivativePeriodic(eps);
	std::vector<double> psi_centred(N);
	for (int j = 0; j < N; ++j) psi_centred[j] = psi_g[j] - mean;
	m.psi_hat = Coefficients(psi_centred);
	m.eps_hat = Coefficients(eps);
	m.max_eps_prime = 0.0;
	m.max_psi_prime = 0.0;
	for (int j = 0; j < N; ++j) {
		m.max_eps_prime = std::max(m.max_eps_prime, std::abs(m.deps[j]));
		m.max_psi_prime = std::max(m.max_psi_prime, std::abs(m.dpsi[j]));
	}
	if (residual >= options.tolerance * 100.0 && it >= options.max_iterations) {
		return fail("conformal map: Theodorsen's iteration did not converge in " + std::to_string(options.max_iterations) + " sweeps (residual " + std::to_string(residual) + ")");
	}

	// ---- grid points and metric -------------------------------------------------
	m.z.resize(N);
	m.dz_dzeta_abs.resize(N);
	for (int j = 0; j < N; ++j) {
		const PointData pd = PointAt(m, m.phi[j], psi_g[j], eps[j], m.dpsi[j], m.deps[j]);
		m.z[j] = pd.z;
		m.dz_dzeta_abs[j] = pd.dz_dzeta_abs;
	}

	// ---- trailing edge image: theta(phi) = phi + eps(phi) = 0 mod 2 pi -----------
	// theta is monotonic in phi (1 + eps' > 0), so bracket the crossing on the
	// grid and refine with Newton on the trigonometric interpolant.
	{
		double phi_te = 0.0;
		bool found = false;
		for (int j = 0; j < N && !found; ++j) {
			const int jn = (j + 1) % N;
			double t0 = m.theta[j], t1 = m.theta[jn] + (jn == 0 ? kTwoPi : 0.0);
			// want k*2pi in [t0, t1)
			const double k = std::ceil(t0 / kTwoPi);
			const double target = k * kTwoPi;
			if (target >= t0 && target <= t1) { // inclusive: theta_0 can sit exactly on 0 (a symmetric section)
				phi_te = m.phi[j] + (target - t0) / (t1 - t0) * (kTwoPi / N);
				for (int n_it = 0; n_it < 50; ++n_it) {
					double e, de;
					EvalSeries(m.eps_hat, phi_te, e, de);
					const double g = phi_te + e - target;
					const double step = g / (1.0 + de);
					phi_te -= step;
					if (std::abs(step) < 1e-14) break;
				}
				found = true;
			}
		}
		if (!found) {
			double tmin = m.theta[0], tmax = m.theta[0], emin = eps[0], emax = eps[0];
			for (int j = 0; j < N; ++j) { tmin = std::min(tmin, m.theta[j]); tmax = std::max(tmax, m.theta[j]); emin = std::min(emin, eps[j]); emax = std::max(emax, eps[j]); }
			return fail("conformal map: could not locate the trailing edge's image (theta " + std::to_string(tmin) + ".." + std::to_string(tmax)
				+ ", eps " + std::to_string(emin) + ".." + std::to_string(emax) + ", theta0 " + std::to_string(m.theta[0]) + ", thetaN " + std::to_string(m.theta[N - 1]) + ")");
		}
		m.phi_te = Reduce(phi_te);
	}

	m.ok = true;
	return m;
}

ConformalFlow SolveFlow(const ConformalMap& m, double alpha_deg) {
	ConformalFlow f;
	if (!m.ok) return f;
	const int N = static_cast<int>(m.phi.size());
	f.alpha = alpha_deg * kPi / 180.0;
	const double R = m.radius();
	const double vartheta_te = m.absolute_angle(m.phi_te);
	// Circle flow W = U (zeta e^-ia + R^2 e^ia / zeta) + (i Gamma / 2 pi) ln zeta;
	// on the circle |dW/dzeta| = |2 U sin(vartheta - alpha) + Gamma / (2 pi R)|,
	// and the Kutta condition (zero at the TE image) fixes Gamma.
	f.gamma = -4.0 * kPi * R * std::sin(vartheta_te - f.alpha);
	f.alpha0 = vartheta_te - kTwoPi * std::floor((vartheta_te + kPi) / kTwoPi); // into (-pi, pi]
	f.cl_kutta = 8.0 * kPi * (R / m.chord) * std::sin(f.alpha - vartheta_te);
	f.cl_alpha = 8.0 * kPi * (R / m.chord) * std::cos(f.alpha - vartheta_te);
	f.phi_stagnation = Reduce(kPi + 2.0 * f.alpha - vartheta_te - std::arg(m.a));
	f.q.resize(N);
	f.cp.resize(N);
	for (int j = 0; j < N; ++j) {
		const double vt = m.absolute_angle(m.phi[j]);
		const double wc = std::abs(2.0 * std::sin(vt - f.alpha) - 2.0 * std::sin(vartheta_te - f.alpha));
		f.q[j] = wc / m.dz_dzeta_abs[j];
		f.cp[j] = 1.0 - f.q[j] * f.q[j];
	}
	return f;
}

cd MapPointAt(const ConformalMap& m, double phi_rel) {
	double psi, dpsi, eps, deps;
	EvalSeries(m.psi_hat, phi_rel, psi, dpsi);
	EvalSeries(m.eps_hat, phi_rel, eps, deps);
	return PointAt(m, phi_rel, psi + m.psi0, eps, dpsi, deps).z;
}

double SpeedAt(const ConformalMap& m, const ConformalFlow& f, double phi_rel) {
	double psi, dpsi, eps, deps;
	EvalSeries(m.psi_hat, phi_rel, psi, dpsi);
	EvalSeries(m.eps_hat, phi_rel, eps, deps);
	const PointData pd = PointAt(m, phi_rel, psi + m.psi0, eps, dpsi, deps);
	const double vt = m.absolute_angle(phi_rel);
	const double vartheta_te = m.absolute_angle(m.phi_te);
	const double wc = std::abs(2.0 * std::sin(vt - f.alpha) - 2.0 * std::sin(vartheta_te - f.alpha));
	return pd.dz_dzeta_abs > 0.0 ? wc / pd.dz_dzeta_abs : 0.0;
}

} // namespace aerotelic
