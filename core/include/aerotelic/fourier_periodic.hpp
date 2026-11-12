#pragma once

// Spectral operations on a real 2 pi-periodic function sampled at the N
// uniform angles phi_j = 2 pi j / N (N a power of two, see fft.hpp). These
// are the two operations Theodorsen's method needs (conformal_airfoil.hpp):
// the harmonic conjugate and the derivative, both exact for trigonometric
// polynomials of degree < N/2. The Nyquist coefficient (k = N/2) carries no
// phase information and is dropped by both, the usual convention.

#include <vector>

namespace aerotelic {

// Conjugate of f for a function analytic OUTSIDE the unit circle: with
//   f(phi) = a_0 + sum_k (a_k cos k phi + b_k sin k phi)
// returns
//   g(phi) = sum_k (b_k cos k phi - a_k sin k phi),
// so that f + i g is the boundary value of sum_k c_k zeta^{-k} (plus the
// constant a_0). Example: f = cos phi (Re of 1/zeta) gives g = -sin phi. This
// is Garrick's conjugate-function relation between Theodorsen's psi and
// epsilon, evaluated with the FFT as in Ives (1976).
std::vector<double> ConjugateExterior(const std::vector<double>& f);

// df/dphi.
std::vector<double> DerivativePeriodic(const std::vector<double>& f);

} // namespace aerotelic
