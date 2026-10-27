#include "aerotelic/panel_loads.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace aerotelic::panel {

std::vector<StripLoad> StripLoads(const Network& net, const NetworkResult& res, const StripOptions& options) {
	const int rows = res.panel_rows, cols = res.panel_cols;
	if (rows <= 0 || cols <= 0 || res.panels.size() != static_cast<size_t>(rows) * cols)
		throw std::invalid_argument("StripLoads: the network result carries no panel grid");
	if (net.rows != rows + 1 || net.cols != cols + 1)
		throw std::invalid_argument("StripLoads: the network and its result do not match");

	const Vec3 axis = Normalized(options.span_axis);
	const int n_strips = options.strips_are_columns ? cols : rows;
	const int n_along = options.strips_are_columns ? rows : cols;
	std::vector<StripLoad> strips(static_cast<size_t>(n_strips));
	for (int k = 0; k < n_strips; ++k) {
		StripLoad& st = strips[static_cast<size_t>(k)];
		st.s_min = std::numeric_limits<double>::infinity();
		st.s_max = -st.s_min;
		double s_weighted = 0.0;
		for (int m = 0; m < n_along; ++m) {
			const int i = options.strips_are_columns ? m : k;
			const int j = options.strips_are_columns ? k : m;
			const PanelResult& p = res.panels[static_cast<size_t>(i) * cols + j];
			const double cp_net = options.two_sided ? (p.cp_lower - p.cp_upper) : -p.cp_upper;
			st.force += p.normal * (cp_net * p.area);
			st.area += p.area;
			s_weighted += Dot(p.centre, axis) * p.area;
			// The strip's extent from the panel's four corners.
			const Vec3 c[4] = {net.Corner(i + 1, j + 1), net.Corner(i + 1, j + 2), net.Corner(i + 2, j + 2), net.Corner(i + 2, j + 1)};
			for (const Vec3& q : c) {
				const double s = Dot(q, axis);
				st.s_min = std::min(st.s_min, s);
				st.s_max = std::max(st.s_max, s);
			}
		}
		st.width = st.s_max - st.s_min;
		st.s = st.area > 0.0 ? s_weighted / st.area : 0.5 * (st.s_min + st.s_max);
	}
	std::sort(strips.begin(), strips.end(), [](const StripLoad& a, const StripLoad& b) { return a.s < b.s; });
	return strips;
}

double EllipticLoad(double total, double s0, double half_span, double s) {
	if (!(half_span > 0.0)) return 0.0;
	const double x = (s - s0) / half_span;
	if (x <= -1.0 || x >= 1.0) return 0.0;
	constexpr double kPi = 3.14159265358979323846;
	return 4.0 * total / (kPi * 2.0 * half_span) * std::sqrt(1.0 - x * x);
}

} // namespace aerotelic::panel
