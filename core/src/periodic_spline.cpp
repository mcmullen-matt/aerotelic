#include "aerotelic/periodic_spline.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aerotelic {

namespace {

// Solves the cyclic tridiagonal system with diagonals (a: sub, b: main,
// c: super) and corner entries alpha (bottom-left) and beta (top-right),
// Numerical Recipes' cyclic() via Sherman-Morrison.
std::vector<double> SolveCyclicTridiagonal(std::vector<double> a, std::vector<double> b, std::vector<double> c,
	double alpha, double beta, const std::vector<double>& r) {
	const int n = static_cast<int>(b.size());
	if (n < 3) throw std::invalid_argument("cyclic tridiagonal: need at least 3 unknowns");
	auto tridiag = [&](const std::vector<double>& bb, const std::vector<double>& rr) {
		std::vector<double> u(n), gam(n);
		double bet = bb[0];
		if (bet == 0.0) throw std::runtime_error("cyclic tridiagonal: zero pivot");
		u[0] = rr[0] / bet;
		for (int j = 1; j < n; ++j) {
			gam[j] = c[j - 1] / bet;
			bet = bb[j] - a[j] * gam[j];
			if (bet == 0.0) throw std::runtime_error("cyclic tridiagonal: zero pivot");
			u[j] = (rr[j] - a[j] * u[j - 1]) / bet;
		}
		for (int j = n - 2; j >= 0; --j) u[j] -= gam[j + 1] * u[j + 1];
		return u;
	};
	const double gamma = -b[0];
	std::vector<double> bb(b);
	bb[0] = b[0] - gamma;
	bb[n - 1] = b[n - 1] - alpha * beta / gamma;
	std::vector<double> x = tridiag(bb, r);
	std::vector<double> u(n, 0.0);
	u[0] = gamma;
	u[n - 1] = alpha;
	std::vector<double> z = tridiag(bb, u);
	const double fact = (x[0] + beta * x[n - 1] / gamma) / (1.0 + z[0] + beta * z[n - 1] / gamma);
	for (int i = 0; i < n; ++i) x[i] -= fact * z[i];
	return x;
}

} // namespace

PeriodicCubicSpline::PeriodicCubicSpline(const std::vector<double>& x, const std::vector<double>& y, double period)
	: x_(x), y_(y), period_(period) {
	const int n = static_cast<int>(x.size());
	if (n < 3 || y.size() != x.size()) throw std::invalid_argument("periodic spline: need >= 3 samples");
	if (!(period > 0.0) || x.back() - x.front() >= period) throw std::invalid_argument("periodic spline: samples must span less than one period");
	for (int i = 1; i < n; ++i) if (!(x[i] > x[i - 1])) throw std::invalid_argument("periodic spline: x must be strictly increasing");

	// Interval widths, the last one wrapping round to x_0 + period.
	std::vector<double> h(n);
	for (int i = 0; i < n - 1; ++i) h[i] = x[i + 1] - x[i];
	h[n - 1] = x[0] + period - x[n - 1];

	// Second-derivative continuity at each node i (indices mod n):
	//   h_{i-1} M_{i-1} + 2 (h_{i-1} + h_i) M_i + h_i M_{i+1}
	//     = 6 [ (y_{i+1} - y_i) / h_i - (y_i - y_{i-1}) / h_{i-1} ].
	std::vector<double> a(n), b(n), c(n), r(n);
	for (int i = 0; i < n; ++i) {
		const int im = (i + n - 1) % n, ip = (i + 1) % n;
		a[i] = h[im];
		b[i] = 2.0 * (h[im] + h[i]);
		c[i] = h[i];
		r[i] = 6.0 * ((y[ip] - y[i]) / h[i] - (y[i] - y[im]) / h[im]);
	}
	// Corner entries: row 0 couples to M_{n-1} through a[0] = h_{n-1} (the
	// top-right entry, beta) and row n-1 couples to M_0 through c[n-1] =
	// h_{n-1} (bottom-left, alpha).
	m_ = SolveCyclicTridiagonal(a, b, c, /*alpha=*/h[n - 1], /*beta=*/h[n - 1], r);
}

void PeriodicCubicSpline::Locate(double x, int& i, double& t, double& h) const {
	const int n = size();
	double u = x - x_[0];
	u -= period_ * std::floor(u / period_);
	u += x_[0];
	// u in [x_0, x_0 + period)
	auto it = std::upper_bound(x_.begin(), x_.end(), u);
	i = static_cast<int>(it - x_.begin()) - 1;
	if (i < 0) i = 0;
	if (i >= n) i = n - 1;
	t = u - x_[i];
	h = (i < n - 1) ? (x_[i + 1] - x_[i]) : (x_[0] + period_ - x_[n - 1]);
}

double PeriodicCubicSpline::Evaluate(double x) const {
	int i; double t, h;
	Locate(x, i, t, h);
	const int ip = (i + 1) % size();
	const double A = (h - t) / h, B = t / h;
	return A * y_[i] + B * y_[ip] + ((A * A * A - A) * m_[i] + (B * B * B - B) * m_[ip]) * h * h / 6.0;
}

double PeriodicCubicSpline::Derivative(double x) const {
	int i; double t, h;
	Locate(x, i, t, h);
	const int ip = (i + 1) % size();
	const double A = (h - t) / h, B = t / h;
	return (y_[ip] - y_[i]) / h - (3.0 * A * A - 1.0) / 6.0 * h * m_[i] + (3.0 * B * B - 1.0) / 6.0 * h * m_[ip];
}

} // namespace aerotelic
