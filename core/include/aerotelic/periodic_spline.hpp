#pragma once

// A periodic cubic spline through irregularly spaced samples of a periodic
// function: the interpolant Theodorsen's iteration needs for psi(theta), the
// near-circle's log-radius as a function of its angle (Ives 1976 uses a
// periodic spline for exactly this step). Natural C2 periodic end
// conditions; the cyclic tridiagonal system is solved by Sherman-Morrison.

#include <vector>

namespace aerotelic {

class PeriodicCubicSpline {
public:
	// x strictly increasing with x.back() - x.front() < period; y the values.
	// The function repeats with the given period.
	PeriodicCubicSpline(const std::vector<double>& x, const std::vector<double>& y, double period);

	double Evaluate(double x) const;
	double Derivative(double x) const;

	int size() const { return static_cast<int>(x_.size()); }

private:
	std::vector<double> x_, y_, m_; // m_ = second derivatives at the nodes
	double period_ = 0.0;

	// Interval index i (x_i <= x < x_{i+1}, the last wrapping to x_0 + period)
	// and the local offset t = x - x_i after reduction into one period.
	void Locate(double x, int& i, double& t, double& h) const;
};

} // namespace aerotelic
