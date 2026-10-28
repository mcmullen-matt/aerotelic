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
	const Vec3 chord_axis = Normalized(options.chord_axis);
	const int n_strips = options.strips_are_columns ? cols : rows;
	const int n_along = options.strips_are_columns ? rows : cols;
	// The chord at a station: the extent of its corners along the chord axis.
	auto station_chord = [&](int station) {
		double lo = std::numeric_limits<double>::infinity(), hi = -lo;
		const int n = options.strips_are_columns ? net.rows : net.cols;
		for (int m = 1; m <= n; ++m) {
			const Vec3 q = options.strips_are_columns ? net.Corner(m, station) : net.Corner(station, m);
			const double x = Dot(q, chord_axis);
			lo = std::min(lo, x);
			hi = std::max(hi, x);
		}
		return hi - lo;
	};
	// A station's place along the span axis: the mean of its corners. The
	// strip width is the spacing of its two stations' means, NOT the extent
	// of its corners: a section tilted by dihedral (placed normal to the
	// dihedral curve) spreads its corners along the span by ± thickness ·
	// sin(dihedral), which would inflate a narrow strip's width by a fifth
	// and read its load per span that much low (the P-38 root strip,
	// 2026-10-27).
	auto station_s = [&](int station) {
		double sum = 0.0;
		const int n = options.strips_are_columns ? net.rows : net.cols;
		for (int m = 1; m <= n; ++m) {
			const Vec3 q = options.strips_are_columns ? net.Corner(m, station) : net.Corner(station, m);
			sum += Dot(q, axis);
		}
		return sum / n;
	};
	std::vector<StripLoad> strips(static_cast<size_t>(n_strips));
	for (int k = 0; k < n_strips; ++k) {
		StripLoad& st = strips[static_cast<size_t>(k)];
		st.s_min = std::numeric_limits<double>::infinity();
		st.s_max = -st.s_min;
		st.chord = 0.5 * (station_chord(k + 1) + station_chord(k + 2));
		double s_weighted = 0.0;
		for (int m = 0; m < n_along; ++m) {
			const int i = options.strips_are_columns ? m : k;
			const int j = options.strips_are_columns ? k : m;
			const PanelResult& p = res.panels[static_cast<size_t>(i) * cols + j];
			st.force += p.normal * (-p.cp_upper * p.area);
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
		st.width = std::fabs(station_s(k + 2) - station_s(k + 1));
		st.s = st.area > 0.0 ? s_weighted / st.area : 0.5 * (st.s_min + st.s_max);
	}
	std::sort(strips.begin(), strips.end(), [](const StripLoad& a, const StripLoad& b) { return a.s < b.s; });
	return strips;
}

std::vector<Vec3> BinnedLoads(const std::vector<LoadSource>& sources, const Vec3& axis, const std::vector<double>& edges) {
	if (edges.size() < 2) throw std::invalid_argument("BinnedLoads: need at least two bin edges");
	for (size_t b = 1; b < edges.size(); ++b)
		if (!(edges[b] > edges[b - 1])) throw std::invalid_argument("BinnedLoads: the bin edges must increase");
	const Vec3 a = Normalized(axis);
	std::vector<Vec3> out(edges.size() - 1);
	for (const LoadSource& src : sources) {
		if (src.net == nullptr || src.res == nullptr) throw std::invalid_argument("BinnedLoads: a source without a network or a result");
		const Network& net = *src.net;
		const NetworkResult& res = *src.res;
		const int rows = res.panel_rows, cols = res.panel_cols;
		if (rows <= 0 || cols <= 0 || res.panels.size() != static_cast<size_t>(rows) * cols)
			throw std::invalid_argument("BinnedLoads: the network result carries no panel grid");
		if (net.rows != rows + 1 || net.cols != cols + 1)
			throw std::invalid_argument("BinnedLoads: the network and its result do not match");
		for (int i = 0; i < rows; ++i) {
			for (int j = 0; j < cols; ++j) {
				const PanelResult& p = res.panels[static_cast<size_t>(i) * cols + j];
				Vec3 n = p.normal;
				if (src.reflect_y) n.y = -n.y;
				const Vec3 f = n * (-p.cp_upper * p.area);
				double lo = std::numeric_limits<double>::infinity(), hi = -lo;
				const Vec3 c[4] = {net.Corner(i + 1, j + 1), net.Corner(i + 1, j + 2), net.Corner(i + 2, j + 2), net.Corner(i + 2, j + 1)};
				for (Vec3 q : c) {
					if (src.reflect_y) q.y = -q.y;
					const double s = Dot(q, a);
					lo = std::min(lo, s);
					hi = std::max(hi, s);
				}
				const double extent = hi - lo;
				if (extent <= 1e-12 * std::max(1.0, std::fabs(hi))) {
					const double mid = 0.5 * (lo + hi);
					for (size_t b = 0; b + 1 < edges.size(); ++b) {
						if (mid >= edges[b] && (mid < edges[b + 1] || (b + 2 == edges.size() && mid <= edges[b + 1]))) { out[b] += f; break; }
					}
					continue;
				}
				for (size_t b = 0; b + 1 < edges.size(); ++b) {
					const double overlap = std::min(hi, edges[b + 1]) - std::max(lo, edges[b]);
					if (overlap > 0.0) out[b] += f * (overlap / extent);
				}
			}
		}
	}
	return out;
}

SectionCut CutSection(const Network& net, const NetworkResult& res, const Vec3& axis_in, double s, bool span_along_columns) {
	const int rows = res.panel_rows, cols = res.panel_cols;
	if (rows <= 0 || cols <= 0 || res.panels.size() != static_cast<size_t>(rows) * cols)
		throw std::invalid_argument("CutSection: the network result carries no panel grid");
	if (net.rows != rows + 1 || net.cols != cols + 1)
		throw std::invalid_argument("CutSection: the network and its result do not match");
	const Vec3 axis = Normalized(axis_in);
	// (a, b): a along the section, b along the axis; corner(a, b) and panel(a, b).
	const int n_along = span_along_columns ? rows : cols;   // panels round the section
	const int n_span = span_along_columns ? cols : rows;    // panels along the axis
	auto corner = [&](int a, int b) { return span_along_columns ? net.Corner(a + 1, b + 1) : net.Corner(b + 1, a + 1); };
	auto panel = [&](int a, int b) -> const PanelResult& { return span_along_columns ? res.panels[static_cast<size_t>(a) * cols + b] : res.panels[static_cast<size_t>(b) * cols + a]; };
	double s_lo = std::numeric_limits<double>::infinity(), s_hi = -s_lo;
	for (const Vec3& c : net.corners) { const double v = Dot(c, axis); s_lo = std::min(s_lo, v); s_hi = std::max(s_hi, v); }
	SectionCut cut;
	if (!(s_hi > s_lo)) return cut;
	const double eps = 1e-9 * (s_hi - s_lo);
	if (s < s_lo - eps || s > s_hi + eps) return cut;
	// The point where the edge P0 -> P1 crosses the plane: inside the
	// half-open interval [min, max), or at the network's far end.
	auto on_edge = [&](const Vec3& p0, const Vec3& p1, Vec3& out) {
		const double sa = Dot(p0, axis), sb = Dot(p1, axis);
		const double lo = std::min(sa, sb), hi = std::max(sa, sb);
		const bool at_end = std::fabs(s - s_hi) <= eps && std::fabs(hi - s_hi) <= eps;
		if (hi - lo <= eps) {
			if (std::fabs(s - sa) > eps) return false;
			out = (p0 + p1) * 0.5;
			return true;
		}
		if (!((s >= lo - eps && s < hi - eps) || at_end)) return false;
		const double t = std::clamp((s - sa) / (sb - sa), 0.0, 1.0);
		out = p0 + (p1 - p0) * t;
		return true;
	};
	for (int a = 0; a < n_along; ++a) {
		for (int b = 0; b < n_span; ++b) {
			Vec3 pa, pb;
			if (!on_edge(corner(a, b), corner(a, b + 1), pa)) continue;
			if (!on_edge(corner(a + 1, b), corner(a + 1, b + 1), pb)) continue;
			const PanelResult& p = panel(a, b);
			// The pressure along the axis: between this panel's centre and
			// the neighbour's on the side the plane lies, held constant
			// past the end panels.
			double cp = p.cp_upper;
			const double sc = Dot(p.centre, axis);
			// The neighbour on the plane's side of this centre -- whichever
			// way the columns run along the axis (a network reflected in
			// y = 0 runs the other way).
			for (int bn : {b + 1, b - 1}) {
				if (bn < 0 || bn >= n_span) continue;
				const PanelResult& q = panel(a, bn);
				const double sn = Dot(q.centre, axis);
				if ((sn - sc) * (s - sc) <= 0.0 || std::fabs(sn - sc) <= eps) continue;
				cp += (q.cp_upper - p.cp_upper) * std::clamp((s - sc) / (sn - sc), 0.0, 1.0);
				break;
			}
			const Vec3 d = pb - pa;
			const double len = Length(d);
			cut.found = true;
			cut.points.push_back((pa + pb) * 0.5);
			cut.cp.push_back(cp);
			if (len <= 0.0) continue;
			Vec3 n2 = Cross(d * (1.0 / len), axis);
			if (Dot(n2, p.normal) < 0.0) n2 = n2 * -1.0;
			cut.force_per_span += n2 * (-cp * len);
			cut.length += len;
		}
	}
	return cut;
}

double EllipticLoad(double total, double s0, double half_span, double s) {
	if (!(half_span > 0.0)) return 0.0;
	const double x = (s - s0) / half_span;
	if (x <= -1.0 || x >= 1.0) return 0.0;
	constexpr double kPi = 3.14159265358979323846;
	return 4.0 * total / (kPi * 2.0 * half_span) * std::sqrt(1.0 - x * x);
}

} // namespace aerotelic::panel
