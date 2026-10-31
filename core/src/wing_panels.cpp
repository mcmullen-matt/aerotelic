#include "aerotelic/wing_panels.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aerotelic::panel {

std::vector<Network> WingNetworks(const WingModel& wing, const WingPaneling& pan, const Vec3& free_stream) {
	if (wing.sections().empty()) throw std::invalid_argument("WingNetworks: the wing has no sections");
	if (pan.chordwise < 2 || pan.spanwise < 1) throw std::invalid_argument("WingNetworks: need at least 2 chordwise and 1 spanwise panels");
	const Planform& pf = wing.planform();

	// Span stations, cosine in eta. A station whose chord has (all but)
	// vanished is collapsed to a point -- one apex unknown, triangular
	// panels -- rather than paneled as a ring of corners crowded into a
	// sliver, which no condition can tell apart; failing that (the fraction
	// set to zero) it is pulled in by the margin.
	std::vector<double> etas = CosineSpacing(pan.spanwise);
	for (double& e : etas) e = pan.eta_min + (pan.eta_max - pan.eta_min) * e;
	const double root_chord = std::max(pf.RootChord(), 1e-12);
	auto pointed = [&](double eta) { return pf.ChordAt(eta) <= std::max(1e-9, pan.collapse_tip_fraction) * root_chord; };
	const bool collapse_tip = pan.collapse_tip_fraction > 0.0 && pointed(etas.back());
	const bool collapse_root = pan.collapse_tip_fraction > 0.0 && pointed(etas.front());
	if (!collapse_tip && pointed(etas.back())) etas.back() -= pan.tip_margin * (pan.eta_max - pan.eta_min);
	if (!collapse_root && pointed(etas.front())) etas.front() += pan.tip_margin * (pan.eta_max - pan.eta_min);
	const int n_s = static_cast<int>(etas.size());
	// Every corner of a collapsed station on its chord's midpoint.
	auto collapse = [&](Mat& xyz, int j) {
		if (!((j == n_s - 1 && collapse_tip) || (j == 0 && collapse_root))) return;
		Mat c = pf.AirfoilCoordsToXyz(etas[j], Mat::FromRows({{0.5, 0.0}}));
		for (int i = 0; i < xyz.rows(); ++i) { xyz(i, 0) = c(0, 0); xyz(i, 1) = c(0, 1); xyz(i, 2) = c(0, 2); }
	};

	// Chordwise parameters on the airfoil curve (lower TE -> LE -> upper TE,
	// t in [0, 1]), cosine-spaced on each surface about the leading edge.
	std::vector<double> cs = CosineSpacing(pan.chordwise);

	Network body;
	body.name = wing.name;
	body.type = NetworkType::DoubletAnalysis;
	std::vector<Vec3> te;
	if (pan.thick) {
		// Rows: TE upper -> LE -> TE lower (2 n_c + 1 points), columns: span.
		body.rows = 2 * pan.chordwise + 1;
		body.cols = n_s;
		body.corners.resize(static_cast<size_t>(body.rows) * body.cols);
		for (int j = 0; j < n_s; ++j) {
			std::shared_ptr<Airfoil> af = wing.AirfoilAt(etas[j]);
			const double t_le = af->LeadingEdgeParameter();
			std::vector<double> ts;
			for (int k = pan.chordwise; k >= 0; --k) ts.push_back(t_le + (1.0 - t_le) * cs[k]); // upper: TE -> LE
			for (int k = 1; k <= pan.chordwise; ++k) ts.push_back(t_le - t_le * cs[k]);          // lower: LE -> TE
			Mat pts2 = af->nurbs_curve().Evaluate(ts);
			Mat xyz = pf.AirfoilCoordsToXyz(etas[j], pts2);
			collapse(xyz, j);
			const bool pin = pan.pin_root_to_plane && j == 0 && etas[0] == 0.0;
			for (int i = 0; i < body.rows; ++i) body.corners[static_cast<size_t>(i) * body.cols + j] = {xyz(i, 0), pin ? 0.0 : xyz(i, 1), xyz(i, 2)};
			Vec3 te_u = body.corners[static_cast<size_t>(0) * body.cols + j], te_l = body.corners[static_cast<size_t>(body.rows - 1) * body.cols + j];
			te.push_back((te_u + te_l) * 0.5);
		}
		body.bc = BoundaryCondition::InteriorPotentialZero();
		body.prescribed_source = [free_stream](const Vec3&, const Vec3& n) { return -Dot(free_stream, n); };
	} else {
		// The mean surface: rows along the span, columns chordwise LE -> TE
		// (ζ up); the velocity condition of the thin-wing cases.
		body.rows = n_s;
		body.cols = pan.chordwise + 1;
		body.corners.resize(static_cast<size_t>(body.rows) * body.cols);
		for (int i = 0; i < n_s; ++i) {
			std::shared_ptr<Airfoil> af = wing.AirfoilAt(etas[i]);
			Mat up = af->UpperXY(cs), lo = af->LowerXY(cs);
			Mat mean(body.cols, 2);
			for (int k = 0; k < body.cols; ++k) { mean(k, 0) = 0.5 * (up(k, 0) + lo(k, 0)); mean(k, 1) = 0.5 * (up(k, 1) + lo(k, 1)); }
			Mat xyz = pf.AirfoilCoordsToXyz(etas[i], mean);
			collapse(xyz, i);
			const bool pin = pan.pin_root_to_plane && i == 0 && etas[0] == 0.0;
			for (int k = 0; k < body.cols; ++k) body.corners[static_cast<size_t>(i) * body.cols + k] = {xyz(k, 0), pin ? 0.0 : xyz(k, 1), xyz(k, 2)};
			te.push_back(body.corners[static_cast<size_t>(i) * body.cols + body.cols - 1]);
		}
		body.bc = BoundaryCondition::Impermeable();
	}
	// Outward (thick) / upward (thin) normals: ζ = (∂P/∂J) x (∂P/∂I).
	{
		int I = std::max(1, body.rows / 2), J = std::max(1, body.cols / 2);
		Vec3 P1 = body.Corner(I, J), P2 = body.Corner(I, J + 1), P3 = body.Corner(I + 1, J + 1), P4 = body.Corner(I + 1, J);
		Vec3 zeta = Cross(P1 + P4 - P2 - P3, P1 + P2 - P3 - P4);
		Vec3 centre = (P1 + P2 + P3 + P4) * 0.25;
		Vec3 want;
		if (pan.thick) {
			// Away from the section's chord line.
			Mat c = pf.AirfoilCoordsToXyz(etas[std::min(J, n_s - 1)], Mat::FromRows({{0.5, 0.0}}));
			want = centre - Vec3{c(0, 0), c(0, 1), c(0, 2)};
		} else {
			want = {0.0, 0.0, 1.0};
		}
		if (Dot(zeta, want) < 0.0) FlipNormals(body);
	}
	double length = pan.wake_length > 0.0 ? pan.wake_length : 20.0 * pf.RootChord();
	Network wake = WakeNetwork(wing.name + " wake", te, pan.wake_direction, length);
	return {body, wake};
}

// ---------------------------------------------------------------------------
// Bodies from lofts
// ---------------------------------------------------------------------------

namespace {

// n + 1 uniform values on [lo, hi] merged with the seeds inside it, sorted, no duplicates.
std::vector<double> MergedValues(double lo, double hi, int n, const std::vector<double>& seeds) {
	std::vector<double> v;
	for (int i = 0; i <= n; ++i) v.push_back(lo + (hi - lo) * i / n);
	for (double s : seeds) if (s >= lo - 1e-12 && s <= hi + 1e-12) v.push_back(std::clamp(s, lo, hi));
	std::sort(v.begin(), v.end());
	std::vector<double> out;
	const double eps = 1e-9 * std::max(hi - lo, 1e-300);
	for (double x : v) if (out.empty() || x - out.back() > eps) out.push_back(x);
	return out;
}

double RowSpread(const std::vector<Vec3>& row) {
	Vec3 c{0, 0, 0};
	for (const Vec3& p : row) c = c + p;
	c = c * (1.0 / static_cast<double>(row.size()));
	double r = 0.0;
	for (const Vec3& p : row) r = std::max(r, Distance(p, c));
	return r;
}

Vec3 RowCentroid(const std::vector<Vec3>& row) {
	// Of the closed polygon's vertices (the last one repeats the first).
	Vec3 c{0, 0, 0};
	size_t n = row.size() > 1 ? row.size() - 1 : row.size();
	for (size_t j = 0; j < n; ++j) c = c + row[j];
	return c * (1.0 / static_cast<double>(n));
}

// The area the ring encloses (Newell's vector, halved): zero for a ring
// that has collapsed to a line -- a fuselage whose half-width vanishes at
// the nose or the tail while its height does not.
double RingArea(const std::vector<Vec3>& row) {
	const Vec3 c = RowCentroid(row);
	Vec3 a{0, 0, 0};
	for (size_t j = 0; j + 1 < row.size(); ++j) a += Cross(row[j] - c, row[j + 1] - c);
	if (row.size() > 1 && Distance(row.front(), row.back()) > 0.0) a += Cross(row.back() - c, row.front() - c);
	return 0.5 * Length(a);
}

} // namespace

Network LoftNetwork(const NamedLoft& loft, const std::string& name, const BodyPaneling& pan, const Vec3& free_stream) {
	const LoftSource& src = loft.source;
	if (!src.section) throw std::invalid_argument("LoftNetwork: the loft '" + name + "' has no section function");
	// Rows along u (the loft's stations), columns round v.
	std::vector<double> us = MergedValues(src.u_min, src.u_max, std::max(pan.along, 1), src.u_seeds);
	const int around = std::max(pan.around, 3);
	std::vector<double> vs = MergedValues(0.0, 1.0, around, src.v_seeds);
	auto sample = [&](const std::vector<double>& vv) {
		std::vector<std::vector<Vec3>> out;
		for (double u : us) {
			Mat pts = src.section(u, vv);
			std::vector<Vec3> row;
			for (int j = 0; j < pts.rows(); ++j) row.push_back({pts(j, 0), pts(j, 1), pts(j, 2)});
			out.push_back(std::move(row));
		}
		return out;
	};
	auto size_of = [](const std::vector<std::vector<Vec3>>& rr) {
		Vec3 lo = rr.front().front(), hi = lo;
		for (const auto& r : rr) for (const Vec3& p : r) {
			lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
			hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
		}
		return std::max(Distance(lo, hi), 1e-300);
	};
	std::vector<std::vector<Vec3>> rows = sample(vs);
	double scale = size_of(rows);
	bool half = false; // cut at the symmetry plane: the ring runs from one crossing to the other

	// A body straddling the symmetry plane: keep the y >= plane side only.
	if (pan.half_plane_y) {
		const double y0 = *pan.half_plane_y, tol = 1e-6 * scale;
		double ymin = 1e300, ymax = -1e300;
		size_t widest = 0;
		double spread = -1.0;
		for (size_t i = 0; i < rows.size(); ++i) {
			for (const Vec3& p : rows[i]) { ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y); }
			double s = RowSpread(rows[i]);
			if (s > spread) { spread = s; widest = i; }
		}
		if (ymin < y0 - tol && ymax > y0 + tol) {
			// The crossings of the widest ring, by a fine scan and bisection.
			const int n_scan = 1024;
			std::vector<double> scan(n_scan + 1);
			for (int k = 0; k <= n_scan; ++k) scan[k] = static_cast<double>(k) / n_scan;
			Mat probe = src.section(us[widest], scan);
			std::vector<double> roots;
			for (int k = 0; k < n_scan; ++k) {
				double ya = probe(k, 1) - y0, yb = probe(k + 1, 1) - y0;
				if (ya == 0.0) { roots.push_back(scan[k]); continue; }
				if ((ya < 0.0) == (yb < 0.0)) continue;
				double lo = scan[k], hi = scan[k + 1];
				for (int it = 0; it < 60 && hi - lo > 1e-14; ++it) {
					double mid = 0.5 * (lo + hi);
					double ym = src.section(us[widest], {mid})(0, 1) - y0;
					if ((ym < 0.0) == (ya < 0.0)) lo = mid; else hi = mid;
				}
				roots.push_back(0.5 * (lo + hi));
			}
			if (probe(n_scan, 1) - y0 == 0.0 && (roots.empty() || roots.back() < 1.0 - 1e-12)) roots.push_back(1.0);
			// A ring closing at v = 1 = v = 0 crossing exactly there counts once.
			if (src.closed_curve && roots.size() >= 2 && roots.front() < 1e-12 && roots.back() > 1.0 - 1e-12) roots.pop_back();
			if (roots.size() != 2) throw std::invalid_argument("LoftNetwork: the body '" + name + "' crosses the symmetry plane " + std::to_string(roots.size()) + " times round a section; it cannot be cut in half there");
			const double r1 = roots[0], r2 = roots[1];
			std::vector<double> half_vs;
			double y_mid = src.section(us[widest], {0.5 * (r1 + r2)})(0, 1) - y0;
			if (y_mid > 0.0) {
				half_vs = MergedValues(r1, r2, std::max(2, static_cast<int>(std::lround(around * (r2 - r1)))), src.v_seeds);
			} else {
				// The positive arc wraps through v = 1 = v = 0: [r2, 1] then [0, r1].
				std::vector<double> a = MergedValues(r2, 1.0, std::max(1, static_cast<int>(std::lround(around * (1.0 - r2)))), src.v_seeds);
				std::vector<double> b = MergedValues(0.0, r1, std::max(1, static_cast<int>(std::lround(around * r1))), src.v_seeds);
				half_vs = a;
				// On a closed ring v = 1 and v = 0 are one point; an open curve's
				// straight closing segment stays as a column pair.
				for (size_t k = src.closed_curve ? 1 : 0; k < b.size(); ++k) half_vs.push_back(b[k]);
			}
			vs = half_vs;
			rows = sample(vs);
			half = true;
			// Pin the crossings onto the plane: the bisection left them a hair off.
			for (auto& r : rows) { r.front().y = y0; r.back().y = y0; }
		}
	}
	const double coincide = 1e-9 * scale;
	// An open section closes on the straight segment from its last point
	// back to its first (an airfoil's trailing edge): one more column, unless
	// the two ends already meet. A half ring's ends lie on the plane instead.
	if (!src.closed_curve && !half) {
		bool apart = false;
		for (const auto& r : rows) apart = apart || Distance(r.front(), r.back()) > coincide;
		if (apart) for (auto& r : rows) r.push_back(r.front());
	}
	// Rows that coincide with their predecessor (a rim of zero thickness, a
	// seed on top of a uniform station) would make panels of no area.
	{
		std::vector<std::vector<Vec3>> kept;
		for (auto& r : rows) {
			if (!kept.empty()) {
				double d = 0.0;
				for (size_t j = 0; j < r.size(); ++j) d = std::max(d, Distance(r[j], kept.back()[j]));
				if (d <= coincide) continue;
			}
			kept.push_back(std::move(r));
		}
		rows = std::move(kept);
	}
	// Caps: an open end that is not already an apex gets a row collapsed to
	// its centroid, a fan of triangular panels (from the midpoint of the
	// chord on the plane for a half ring, so the fan fills the half disc).
	// An end ring that has collapsed to a line (a fuselage whose half-width
	// vanishes at the tail while its height does not) encloses nothing and
	// needs no cap: a fan on it would be a ring of panels of no area, whose
	// frames are NaN and take the whole solution with them (the P-38 boom,
	// 2026-10-27).
	if (!src.periodic_u) {
		auto apex_of = [half](const std::vector<Vec3>& r) { return half ? (r.front() + r.back()) * 0.5 : RowCentroid(r); };
		auto open = [&](const std::vector<Vec3>& r) { return RowSpread(r) > coincide && RingArea(r) > 1e-9 * scale * scale; };
		if (src.cap_start && open(rows.front())) rows.insert(rows.begin(), std::vector<Vec3>(rows.front().size(), apex_of(rows.front())));
		if (src.cap_end && open(rows.back())) rows.push_back(std::vector<Vec3>(rows.back().size(), apex_of(rows.back())));
	}
	if (rows.size() < 2) throw std::invalid_argument("LoftNetwork: the loft '" + name + "' collapses to one station");

	Network net;
	net.name = name;
	net.type = pan.sources ? NetworkType::SourceAnalysis : NetworkType::DoubletAnalysis;
	net.rows = static_cast<int>(rows.size());
	net.cols = static_cast<int>(rows.front().size());
	net.corners.resize(static_cast<size_t>(net.rows) * net.cols);
	for (int i = 0; i < net.rows; ++i)
		for (int j = 0; j < net.cols; ++j) net.corners[static_cast<size_t>(i) * net.cols + j] = rows[i][j];
	// Outward: the quads as triangles enclose a positive volume (a half body
	// is closed by its reflection first, so the volume is of the whole).
	{
		TriMesh m;
		m.vertices = net.corners;
		if (half) {
			const double y0 = *pan.half_plane_y;
			for (const Vec3& p : net.corners) m.vertices.push_back({p.x, 2.0 * y0 - p.y, p.z});
		}
		for (int i = 0; i + 1 < net.rows; ++i) {
			for (int j = 0; j + 1 < net.cols; ++j) {
				int a = i * net.cols + j, b = i * net.cols + j + 1, c = (i + 1) * net.cols + j + 1, d = (i + 1) * net.cols + j;
				// Panel order P1 (I,J) -> P2 (I,J+1) -> P3 (I+1,J+1) -> P4 (I+1,J) is CCW
				// seen from ζ = (∂P/∂J) x (∂P/∂I) = (P2 - P1) x (P4 - P1), so the
				// triangles (a, b, d) and (b, c, d) face ζ.
				m.triangles.push_back({a, b, d});
				m.triangles.push_back({b, c, d});
				if (half) {
					// The reflection reverses the winding.
					const int o = net.rows * net.cols;
					m.triangles.push_back({a + o, d + o, b + o});
					m.triangles.push_back({b + o, d + o, c + o});
				}
			}
		}
		m.RemoveDegenerateTriangles();
		if (m.SignedVolume() < 0.0) FlipNormals(net);
	}
	net.curved = !loft.flat;
	if (pan.sources) {
		net.bc = BoundaryCondition::Impermeable();
	} else {
		net.bc = BoundaryCondition::InteriorPotentialZero();
		net.prescribed_source = [free_stream](const Vec3&, const Vec3& n) { return -Dot(free_stream, n); };
	}
	return net;
}

std::vector<Network> BodyNetworks(const ISolidModel& model, const std::string& name, const BodyPaneling& pan, const Vec3& free_stream) {
	std::vector<NamedLoft> lofts = model.BuildLofts();
	std::vector<Network> out;
	for (const NamedLoft& l : lofts) {
		std::string n = lofts.size() == 1 ? name : name + "/" + l.name;
		out.push_back(LoftNetwork(l, n, pan, free_stream));
	}
	return out;
}

} // namespace aerotelic::panel
