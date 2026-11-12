#pragma once

// Dense LU factorisation with partial pivoting, row-major, for the small
// dense systems of the two-dimensional section methods (the vortex-panel
// streamfunction system, the coupled boundary-layer Newton system). Kept
// header-only and dependency-free on purpose; the 3-D panel method has its
// own file-local copy, which this does not try to share.

#include <cmath>
#include <stdexcept>
#include <vector>

namespace aerotelic {

class DenseLU {
public:
	DenseLU() = default;

	// Factors the n x n matrix given row-major (a[i*n + j]).
	DenseLU(std::vector<double> a, int n) : n_(n), lu_(std::move(a)), piv_(n) {
		if (static_cast<int>(lu_.size()) != n * n) throw std::invalid_argument("DenseLU: size mismatch");
		double max_pivot = 0.0, min_pivot = 1e300;
		for (int k = 0; k < n; ++k) {
			int p = k;
			double best = std::fabs(lu_[k * n + k]);
			for (int i = k + 1; i < n; ++i) {
				const double v = std::fabs(lu_[i * n + k]);
				if (v > best) { best = v; p = i; }
			}
			piv_[k] = p;
			if (p != k) for (int j = 0; j < n; ++j) std::swap(lu_[k * n + j], lu_[p * n + j]);
			const double pivot = lu_[k * n + k];
			if (pivot == 0.0) { singular_ = true; return; }
			max_pivot = std::fmax(max_pivot, std::fabs(pivot));
			min_pivot = std::fmin(min_pivot, std::fabs(pivot));
			for (int i = k + 1; i < n; ++i) {
				const double f = lu_[i * n + k] / pivot;
				if (f == 0.0) continue;
				lu_[i * n + k] = f;
				double* ri = &lu_[i * n];
				const double* rk = &lu_[k * n];
				for (int j = k + 1; j < n; ++j) ri[j] -= f * rk[j];
			}
		}
		pivot_ratio_ = max_pivot > 0.0 ? min_pivot / max_pivot : 0.0;
	}

	bool ok() const { return n_ > 0 && !singular_; }
	int size() const { return n_; }
	double pivot_ratio() const { return pivot_ratio_; }

	// Solves in place.
	void Solve(std::vector<double>& b) const {
		if (!ok()) throw std::runtime_error("DenseLU: singular matrix");
		const int n = n_;
		for (int k = 0; k < n; ++k) if (piv_[k] != k) std::swap(b[k], b[piv_[k]]);
		for (int i = 0; i < n; ++i) {
			double s = b[i];
			const double* ri = &lu_[i * n];
			for (int j = 0; j < i; ++j) s -= ri[j] * b[j];
			b[i] = s;
		}
		for (int i = n - 1; i >= 0; --i) {
			double s = b[i];
			const double* ri = &lu_[i * n];
			for (int j = i + 1; j < n; ++j) s -= ri[j] * b[j];
			b[i] = s / ri[i];
		}
	}

	std::vector<double> Solve(const std::vector<double>& b) const {
		std::vector<double> x = b;
		Solve(x);
		return x;
	}

private:
	int n_ = 0;
	std::vector<double> lu_;
	std::vector<int> piv_;
	bool singular_ = false;
	double pivot_ratio_ = 0.0;
};

} // namespace aerotelic
