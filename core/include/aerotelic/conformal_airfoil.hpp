#pragma once

// Incompressible potential flow about a single airfoil by conformal mapping:
// Theodorsen's method (Theodorsen 1931, NACA TR 411; Theodorsen & Garrick
// 1933, NACA TR 452) in the form David Ives gave it in "A Modern Look at
// Conformal Mapping, Including Multiply Connected Regions", AIAA Journal
// 14(8):1006-1011, 1976:
//
//   1. A Karman-Trefftz transformation with exponent n = 2 - tau/pi (tau the
//      trailing-edge included angle) opens the trailing edge into a smooth
//      point, taking the airfoil to a NEAR-circle. Its two singular points
//      are the trailing edge and a point inside the nose.
//   2. Theodorsen's map zeta' = zeta exp(sum_k c_k zeta^-k) takes the
//      near-circle zeta' = |a| exp(psi + i theta) to the exact circle
//      zeta = |a| exp(psi0 + i phi). On the circle psi - psi0 and
//      epsilon = theta - phi are harmonic conjugates; the iteration
//        epsilon^(m+1)(phi) = K[ psi(phi + epsilon^(m)(phi)) ]
//      (Garrick's conjugate operator K evaluated with the FFT, psi(theta)
//      interpolated by a periodic cubic spline) converges when the
//      near-circle is close enough to a circle -- Warschawski's condition,
//      which the Karman-Trefftz pre-map is there to secure.
//   3. The circle flow at incidence with the circulation fixed by the Kutta
//      condition at the trailing edge's image gives the surface speed
//      q = |w_circle| / (|dz/dzeta'| |dzeta'/dzeta|), with
//      |dzeta'/dzeta| = exp(psi - psi0) sqrt((1 + eps')^2 + psi'^2).
//
// Single body only. Mach 0.

#include <complex>
#include <string>
#include <vector>

#include "aerotelic/mat.hpp"

namespace aerotelic {

struct ConformalMapOptions {
	int n_points = 512;            // circle grid, power of two
	int max_iterations = 200;
	double tolerance = 1e-10;      // max change of epsilon between sweeps
	double relaxation = 1.0;       // under-relaxation of the epsilon update
	double nose_fraction = 0.5;    // nose singularity at this fraction of the LE radius inside the LE
	double te_tangent_fraction = 0.015; // chord fraction over which the TE tangents are measured
};

struct ConformalMap {
	bool ok = false;
	std::string error;
	int iterations = 0;
	double residual = 0.0;      // last max |delta epsilon|
	double max_eps_prime = 0.0; // max |d epsilon / d phi|, the convergence margin (must stay < 1)
	double max_psi_prime = 0.0;

	// Karman-Trefftz pre-map.
	std::complex<double> z_te, z_nose, a; // a = (z_te - z_nose) / (2 n)
	double n_kt = 2.0;
	double te_angle = 0.0;      // tau, radians
	double chord = 1.0;
	double x_min = 0.0;         // leading-edge x, for x/c
	double le_radius = 0.0;

	// Theodorsen's map on the circle grid phi_j = 2 pi j / N; theta and psi
	// are relative to the TE direction (zeta'/a = exp(psi + i theta)).
	double psi0 = 0.0;          // circle radius R = |a| exp(psi0)
	std::vector<double> phi, theta, psi, eps, dpsi, deps;
	std::vector<std::complex<double>> z;   // grid points in the airfoil plane
	std::vector<double> dz_dzeta_abs;      // |dz/dzeta| at the grid points
	double phi_te = 0.0;        // circle angle (relative) of the trailing edge's image

	// Fourier coefficients of psi - psi0 and of epsilon on the circle,
	// normalised (f(phi) = sum_k F_k exp(i k phi), k = -N/2+1 .. N/2-1), for
	// evaluating the map between grid points (MapPointAt / SpeedAt).
	std::vector<std::complex<double>> psi_hat, eps_hat;

	double radius() const { return std::abs(a) * std::exp(psi0); }
	// Absolute circle angle of a relative phi.
	double absolute_angle(double phi_rel) const { return phi_rel + std::arg(a); }
};

// `contour`: (M, 2) points lower TE -> LE -> upper TE with a CLOSED trailing
// edge (first and last rows coincide, or are made to by averaging). The
// contour must be simple and wind clockwise in that order (an airfoil with
// the chord along +x does).
ConformalMap MapAirfoil(const Mat& contour, const ConformalMapOptions& options = {});

struct ConformalFlow {
	double alpha = 0.0;         // radians
	double gamma = 0.0;         // circulation / (U |a| e^psi0) ... stored as Gamma / U
	double cl_kutta = 0.0;      // 8 pi (R / c) sin(alpha - alpha0): the exact Kutta-Joukowski lift
	double alpha0 = 0.0;        // zero-lift angle, radians (the TE image's absolute angle)
	double cl_alpha = 0.0;      // per radian
	double phi_stagnation = 0.0;// relative circle angle of the front stagnation point
	std::vector<double> q;      // |V| / U at the grid points
	std::vector<double> cp;     // 1 - q^2
};

ConformalFlow SolveFlow(const ConformalMap& map, double alpha_deg);

// Flow quantities at an arbitrary relative circle angle, by trigonometric
// interpolation of the map's series (exact for the spline-smoothed contour):
// the point z and the speed q/U. Used to place the stagnation point and the
// trailing edge exactly on the surface distributions.
std::complex<double> MapPointAt(const ConformalMap& map, double phi_rel);
double SpeedAt(const ConformalMap& map, const ConformalFlow& flow, double phi_rel);

} // namespace aerotelic
