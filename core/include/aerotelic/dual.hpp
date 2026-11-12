#pragma once

// Forward-mode automatic differentiation: a value with N partial
// derivatives carried through arithmetic. The integral boundary-layer
// closures and residuals (ibl_closures.hpp) are written once as templates
// over the number type and evaluated with Dual to obtain the exact
// Jacobian blocks the coupled Newton solver needs -- in place of the
// hand-chained derivative code of XFOIL / mfoil, which is the largest and
// most error-prone part of those programs.

#include <array>
#include <cmath>

namespace aerotelic {

template <int N>
struct Dual {
	double v = 0.0;
	std::array<double, N> d{};

	Dual() { d.fill(0.0); }
	Dual(double x) : v(x) { d.fill(0.0); } // NOLINT: implicit from double on purpose
	static Dual Var(double x, int slot) { Dual r(x); r.d[slot] = 1.0; return r; }

	Dual operator-() const { Dual r; r.v = -v; for (int i = 0; i < N; ++i) r.d[i] = -d[i]; return r; }
	Dual& operator+=(const Dual& o) { v += o.v; for (int i = 0; i < N; ++i) d[i] += o.d[i]; return *this; }
	Dual& operator-=(const Dual& o) { v -= o.v; for (int i = 0; i < N; ++i) d[i] -= o.d[i]; return *this; }
	Dual& operator*=(const Dual& o) { for (int i = 0; i < N; ++i) d[i] = d[i] * o.v + v * o.d[i]; v *= o.v; return *this; }
	Dual& operator/=(const Dual& o) { const double inv = 1.0 / o.v; for (int i = 0; i < N; ++i) d[i] = (d[i] * o.v - v * o.d[i]) * inv * inv; v *= inv; return *this; }
	Dual& operator*=(double s) { v *= s; for (int i = 0; i < N; ++i) d[i] *= s; return *this; }
	Dual& operator/=(double s) { return (*this) *= (1.0 / s); }
};

template <int N> Dual<N> operator+(Dual<N> a, const Dual<N>& b) { return a += b; }
template <int N> Dual<N> operator-(Dual<N> a, const Dual<N>& b) { return a -= b; }
template <int N> Dual<N> operator*(Dual<N> a, const Dual<N>& b) { return a *= b; }
template <int N> Dual<N> operator/(Dual<N> a, const Dual<N>& b) { return a /= b; }
template <int N> Dual<N> operator+(Dual<N> a, double b) { a.v += b; return a; }
template <int N> Dual<N> operator-(Dual<N> a, double b) { a.v -= b; return a; }
template <int N> Dual<N> operator*(Dual<N> a, double b) { return a *= b; }
template <int N> Dual<N> operator/(Dual<N> a, double b) { return a /= b; }
template <int N> Dual<N> operator+(double a, Dual<N> b) { b.v += a; return b; }
template <int N> Dual<N> operator-(double a, const Dual<N>& b) { return (-b) + a; }
template <int N> Dual<N> operator*(double a, Dual<N> b) { return b *= a; }
template <int N> Dual<N> operator/(double a, const Dual<N>& b) { return Dual<N>(a) / b; }

template <int N> bool operator<(const Dual<N>& a, const Dual<N>& b) { return a.v < b.v; }
template <int N> bool operator>(const Dual<N>& a, const Dual<N>& b) { return a.v > b.v; }
template <int N> bool operator<=(const Dual<N>& a, const Dual<N>& b) { return a.v <= b.v; }
template <int N> bool operator>=(const Dual<N>& a, const Dual<N>& b) { return a.v >= b.v; }
template <int N> bool operator<(const Dual<N>& a, double b) { return a.v < b; }
template <int N> bool operator>(const Dual<N>& a, double b) { return a.v > b; }
template <int N> bool operator<=(const Dual<N>& a, double b) { return a.v <= b; }
template <int N> bool operator>=(const Dual<N>& a, double b) { return a.v >= b; }
template <int N> bool operator<(double a, const Dual<N>& b) { return a < b.v; }
template <int N> bool operator>(double a, const Dual<N>& b) { return a > b.v; }

template <int N> Dual<N> Chain(const Dual<N>& x, double fv, double dfdx) {
	Dual<N> r; r.v = fv; for (int i = 0; i < N; ++i) r.d[i] = dfdx * x.d[i]; return r;
}
template <int N> Dual<N> log(const Dual<N>& x) { return Chain(x, std::log(x.v), 1.0 / x.v); }
template <int N> Dual<N> exp(const Dual<N>& x) { const double e = std::exp(x.v); return Chain(x, e, e); }
template <int N> Dual<N> sqrt(const Dual<N>& x) { const double s = std::sqrt(x.v); return Chain(x, s, 0.5 / s); }
template <int N> Dual<N> tanh(const Dual<N>& x) { const double t = std::tanh(x.v); return Chain(x, t, 1.0 - t * t); }
template <int N> Dual<N> pow(const Dual<N>& x, double p) { return Chain(x, std::pow(x.v, p), p * std::pow(x.v, p - 1.0)); }
template <int N> Dual<N> pow(const Dual<N>& x, const Dual<N>& p) { // x^p = exp(p ln x)
	return exp(p * log(x));
}
template <int N> Dual<N> fabs(const Dual<N>& x) { return x.v < 0.0 ? -x : x; }
template <int N> double Value(const Dual<N>& x) { return x.v; }
inline double Value(double x) { return x; }

} // namespace aerotelic
