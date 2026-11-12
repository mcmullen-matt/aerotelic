// In-house radix-2 FFT. See fft.hpp for the contract and for why this is not
// a library; everything below is replaceable wholesale.

#include "aerotelic/fft.hpp"

#include <cmath>
#include <stdexcept>

namespace aerotelic::fft {

namespace {

constexpr double kPi = 3.14159265358979323846;

void Transform(std::vector<std::complex<double>>& a, bool inverse) {
	const int n = static_cast<int>(a.size());
	if (!IsSupportedSize(n)) throw std::invalid_argument("fft: size must be a power of two");
	if (n == 1) return;

	// Bit-reversal permutation.
	for (int i = 1, j = 0; i < n; ++i) {
		int bit = n >> 1;
		for (; j & bit; bit >>= 1) j ^= bit;
		j ^= bit;
		if (i < j) std::swap(a[i], a[j]);
	}

	// Butterflies, with the twiddles of each stage computed directly from
	// cos/sin rather than by repeated multiplication (which would compound
	// rounding over long stages).
	const double sign = inverse ? 1.0 : -1.0;
	std::vector<std::complex<double>> tw(n / 2);
	for (int len = 2; len <= n; len <<= 1) {
		const int half = len / 2;
		for (int k = 0; k < half; ++k) {
			const double ang = sign * 2.0 * kPi * k / len;
			tw[k] = std::complex<double>(std::cos(ang), std::sin(ang));
		}
		for (int i = 0; i < n; i += len) {
			for (int k = 0; k < half; ++k) {
				const std::complex<double> u = a[i + k];
				const std::complex<double> v = a[i + k + half] * tw[k];
				a[i + k] = u + v;
				a[i + k + half] = u - v;
			}
		}
	}
	if (inverse) {
		const double inv = 1.0 / n;
		for (auto& v : a) v *= inv;
	}
}

} // namespace

bool IsSupportedSize(int n) { return n >= 1 && (n & (n - 1)) == 0; }

void Forward(std::vector<std::complex<double>>& data) { Transform(data, false); }
void Inverse(std::vector<std::complex<double>>& data) { Transform(data, true); }

} // namespace aerotelic::fft
