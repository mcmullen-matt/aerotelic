#include "aerotelic/fourier_periodic.hpp"

#include <complex>
#include <stdexcept>

#include "aerotelic/fft.hpp"

namespace aerotelic {

namespace {

using cd = std::complex<double>;

std::vector<cd> Spectrum(const std::vector<double>& f) {
	const int n = static_cast<int>(f.size());
	if (!fft::IsSupportedSize(n)) throw std::invalid_argument("fourier_periodic: sample count must be a power of two");
	std::vector<cd> a(n);
	for (int j = 0; j < n; ++j) a[j] = cd(f[j], 0.0);
	fft::Forward(a);
	return a;
}

std::vector<double> RealPart(std::vector<cd>& a) {
	fft::Inverse(a);
	std::vector<double> out(a.size());
	for (size_t j = 0; j < a.size(); ++j) out[j] = a[j].real();
	return out;
}

} // namespace

std::vector<double> ConjugateExterior(const std::vector<double>& f) {
	std::vector<cd> a = Spectrum(f);
	const int n = static_cast<int>(a.size());
	// Forward FFT with exp(-i j k 2pi/N): X_k for k = 1..N/2-1 holds
	// (a_k - i b_k) N/2, X_{N-k} its conjugate. Multiplying the positive
	// frequencies by +i and the negative by -i turns a_k cos + b_k sin into
	// b_k cos - a_k sin (see the header).
	a[0] = cd(0.0, 0.0);
	for (int k = 1; k < n / 2; ++k) {
		a[k] *= cd(0.0, 1.0);
		a[n - k] *= cd(0.0, -1.0);
	}
	if (n >= 2) a[n / 2] = cd(0.0, 0.0);
	return RealPart(a);
}

std::vector<double> DerivativePeriodic(const std::vector<double>& f) {
	std::vector<cd> a = Spectrum(f);
	const int n = static_cast<int>(a.size());
	a[0] = cd(0.0, 0.0);
	for (int k = 1; k < n / 2; ++k) {
		a[k] *= cd(0.0, static_cast<double>(k));
		a[n - k] *= cd(0.0, -static_cast<double>(k));
	}
	if (n >= 2) a[n / 2] = cd(0.0, 0.0);
	return RealPart(a);
}

} // namespace aerotelic
