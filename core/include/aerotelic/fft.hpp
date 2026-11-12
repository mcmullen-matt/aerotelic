#pragma once

// The discrete Fourier transform used by the conformal-mapping aerodynamics
// (conformal_airfoil.hpp, through fourier_periodic.hpp).
//
// DELIBERATELY SEGREGATED: this header is the only FFT interface the rest of
// core sees, and src/fft.cpp is the only place a transform is computed. The
// implementation is a plain iterative radix-2 Cooley-Tukey (power-of-two
// sizes only), written in-house because FFTW's GPL licence is incompatible
// with the application's distribution terms. To swap in an optimised library
// (Intel MKL's FFTW3 wrappers, pocketfft, KissFFT, ...) replace the bodies of
// the three functions below and nothing else: callers depend on exactly this
// contract and on no other property of the implementation.
//
// Contract (matches FFTW's FFTW_FORWARD / FFTW_BACKWARD sign convention):
//   Forward:  X_k = sum_j x_j exp(-2 pi i j k / N),  unnormalised.
//   Inverse:  x_j = (1/N) sum_k X_k exp(+2 pi i j k / N),  i.e. Inverse(Forward(x)) == x.
// Both operate in place. Sizes must satisfy IsSupportedSize(); callers
// (fourier_periodic.hpp) check it and throw std::invalid_argument otherwise,
// so an implementation that supports arbitrary sizes only needs to widen
// IsSupportedSize().

#include <complex>
#include <vector>

namespace aerotelic::fft {

bool IsSupportedSize(int n);

void Forward(std::vector<std::complex<double>>& data);
void Inverse(std::vector<std::complex<double>>& data);

} // namespace aerotelic::fft
