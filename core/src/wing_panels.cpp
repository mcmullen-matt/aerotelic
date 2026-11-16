#include "aerotelic/wing_panels.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "aerotelic/sdf.hpp"
#include "aerotelic/section_aero.hpp"
#include "aerotelic/section_cut.hpp"

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
	// A deflected flap's end stations join the span stations, so the kink of
	// the section begins and ends where the flap does.
	bool any_deflected = false;
	for (const FlapSpec& f : wing.flaps) {
		if (f.deflection_deg == 0.0) continue;
		any_deflected = true;
		for (double e : {f.eta_root, f.eta_tip}) {
			if (e <= pan.eta_min || e >= pan.eta_max) continue;
			bool have = false;
			for (double x : etas) have = have || std::fabs(x - e) < 1e-6;
			if (!have) etas.push_back(e);
		}
	}
	if (any_deflected) std::sort(etas.begin(), etas.end());
	// The section at a station kinked by every flap that spans it: the
	// points aft of the hinge turned about the hinge centre of the flap's
	// own section cut (so the aero copy and the flap body agree on the
	// axis), clamped to the section's limits like the body.
	auto kink = [&](Mat& pts2, double eta, const Airfoil& af) {
		if (!any_deflected) return;
		for (const FlapSpec& f : wing.flaps) {
			if (f.deflection_deg == 0.0 || eta < f.eta_root - 1e-9 || eta > f.eta_tip + 1e-9) continue;
			const double chord = pf.ChordAt(eta);
			if (chord <= 0.0) continue;
			FlapCut cut;
			try { cut = SolveFlapCut(af, wing.HingeFraction(f, eta), f.gap / chord); } catch (const std::invalid_argument&) { continue; }
			double d = f.deflection_deg;
			if (cut.deflection_down_limit_deg > 0.0 && d > cut.deflection_down_limit_deg) d = cut.deflection_down_limit_deg;
			if (cut.deflection_up_limit_deg > 0.0 && d < -cut.deflection_up_limit_deg) d = -cut.deflection_up_limit_deg;
			pts2 = DeflectAboutHinge(pts2, cut.x_hinge, cut.y_hinge, d, std::max(pan.hinge_blend, 0.0));
		}
	};
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
			// A blunt section: the aero copy closes it by the XFOIL-style
			// blend (the gap taken out over the last 30% of the chord), so
			// the two trailing-edge rows coincide and the wake abuts them.
			{
				double gap = 0.0;
				Mat closed = CloseTrailingEdge(pts2, 0.7, &gap);
				if (gap > 1e-9) pts2 = closed;
			}
			kink(pts2, etas[j], *af);
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
			kink(mean, etas[i], *af);
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

// n + 1 uniform values on [lo, hi] merged with the seeds inside it, sorted,
// no duplicates: mesh.hpp's GridParameters, the same construction the
// triangulation uses for a structured grid (LoftOptions::grid_u / grid_v),
// so a body's panels and its triangles sample the loft at the same places.
std::vector<double> MergedValues(double lo, double hi, int n, const std::vector<double>& seeds) {
	return GridParameters(lo, hi, n, seeds);
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

// The loft sampled onto its grid: one row of corner points per station,
// the station of each row (NaN for a cap), whether the body was cut at
// the symmetry plane, and -- with the trailing-edge tag honoured -- the
// row that is the trailing-edge ring (UStation) or whether the open
// section's ends were blended together (VSeam).
struct SampledLoft {
	std::vector<std::vector<Vec3>> rows;
	std::vector<double> row_u;
	std::vector<bool> row_cap;
	bool half = false;
	double scale = 1.0;
	int te_row = -1;
	bool seam_closed = false;
};

// Whether the rows, as corner quads, enclose a negative volume (a half
// body is closed by its reflection first, so the volume is of the whole):
// the network must then be flipped to face outward.
bool NeedsFlip(const std::vector<std::vector<Vec3>>& rows, bool half, double y0) {
	const int nr = static_cast<int>(rows.size()), nc = static_cast<int>(rows.front().size());
	TriMesh m;
	for (const auto& r : rows) for (const Vec3& p : r) m.vertices.push_back(p);
	if (half) for (const auto& r : rows) for (const Vec3& p : r) m.vertices.push_back({p.x, 2.0 * y0 - p.y, p.z});
	for (int i = 0; i + 1 < nr; ++i) {
		for (int j = 0; j + 1 < nc; ++j) {
			int a = i * nc + j, b = i * nc + j + 1, c = (i + 1) * nc + j + 1, d = (i + 1) * nc + j;
			// Panel order P1 (I,J) -> P2 (I,J+1) -> P3 (I+1,J+1) -> P4 (I+1,J) is CCW
			// seen from ζ = (∂P/∂J) x (∂P/∂I) = (P2 - P1) x (P4 - P1), so the
			// triangles (a, b, d) and (b, c, d) face ζ.
			m.triangles.push_back({a, b, d});
			m.triangles.push_back({b, c, d});
			if (half) {
				// The reflection reverses the winding.
				const int o = nr * nc;
				m.triangles.push_back({a + o, d + o, b + o});
				m.triangles.push_back({b + o, d + o, c + o});
			}
		}
	}
	m.RemoveDegenerateTriangles();
	return m.SignedVolume() < 0.0;
}

// A network from rows of corner points: source/analysis (impermeable), a
// doublet/analysis closed body under the interior-potential condition
// with the sources prescribed, or a doublet/analysis SHEET under the
// velocity condition (the thin-wing cases: one-sided impermeability, no
// sources); flipped when asked so it faces outward.
enum class NetKind { Sources, Doublet, Sheet };
Network MakeNetwork(const std::vector<std::vector<Vec3>>& rows, const std::string& name, bool curved, NetKind kind, const Vec3& free_stream, bool flip) {
	Network net;
	net.name = name;
	net.type = kind == NetKind::Sources ? NetworkType::SourceAnalysis : NetworkType::DoubletAnalysis;
	net.rows = static_cast<int>(rows.size());
	net.cols = static_cast<int>(rows.front().size());
	net.corners.resize(static_cast<size_t>(net.rows) * net.cols);
	for (int i = 0; i < net.rows; ++i)
		for (int j = 0; j < net.cols; ++j) net.corners[static_cast<size_t>(i) * net.cols + j] = rows[i][j];
	if (flip) FlipNormals(net);
	net.curved = curved;
	if (kind == NetKind::Doublet) {
		net.bc = BoundaryCondition::InteriorPotentialZero();
		net.prescribed_source = [free_stream](const Vec3&, const Vec3& n) { return -Dot(free_stream, n); };
	} else {
		net.bc = BoundaryCondition::Impermeable();
	}
	return net;
}

// Whether a sheet's middle panel faces away from the sheet's centroid:
// the orientation an annular sheet's "upper" side wants (the outside).
bool SheetFacesInward(const std::vector<std::vector<Vec3>>& rows) {
	Vec3 centroid{0.0, 0.0, 0.0};
	int count = 0;
	for (const auto& r : rows) for (const Vec3& p : r) { centroid += p; ++count; }
	centroid = centroid * (1.0 / std::max(count, 1));
	const int I = std::max(0, static_cast<int>(rows.size()) / 2 - 1), J = std::max(0, static_cast<int>(rows.front().size()) / 2 - 1);
	if (I + 1 >= static_cast<int>(rows.size()) || J + 1 >= static_cast<int>(rows.front().size())) return false;
	const Vec3 P1 = rows[I][J], P2 = rows[I][J + 1], P3 = rows[I + 1][J + 1], P4 = rows[I + 1][J];
	const Vec3 zeta = Cross(P1 + P4 - P2 - P3, P1 + P2 - P3 - P4);
	return Dot(zeta, (P1 + P2 + P3 + P4) * 0.25 - centroid) < 0.0;
}

SampledLoft SampleLoft(const NamedLoft& loft, const std::string& name, const BodyPaneling& pan, bool with_tag) {
	const LoftSource& src = loft.source;
	if (!src.section) throw std::invalid_argument("LoftNetwork: the loft '" + name + "' has no section function");
	const LoftTrailingEdge& tag = loft.trailing_edge;
	const bool u_station = with_tag && tag.kind == LoftTrailingEdge::Kind::UStation;
	const bool v_seam = with_tag && tag.kind == LoftTrailingEdge::Kind::VSeam;
	SampledLoft out;
	// Rows along u (the loft's stations), columns round v. A thin annular
	// wall (UStation) is paneled as its MEAN SURFACE from the first station
	// to the trailing-edge station: each row the mean of the station and
	// its twin across the wall (LoftSource::mirror_u), the thick closed
	// body being ill-conditioned under the interior-potential formulation
	// (a cowl's wall is thin against its panels: Cp of -900, 2026-11-13).
	const bool mean_sheet = u_station && std::isfinite(src.mirror_u);
	std::vector<double> useeds;
	for (double s : src.u_seeds) if (!mean_sheet || (s >= src.u_min && s <= tag.u)) useeds.push_back(s);
	if (mean_sheet) useeds.push_back(tag.u);
	std::vector<double> us = MergedValues(src.u_min, mean_sheet ? tag.u : src.u_max, std::max(pan.along, 1), useeds);
	const int around = std::max(pan.around, 3);
	std::vector<double> vs = MergedValues(0.0, 1.0, around, src.v_seeds);
	auto sample = [&](const std::vector<double>& vv) {
		std::vector<std::vector<Vec3>> rr;
		for (double u : us) {
			Mat pts = src.section(u, vv);
			if (mean_sheet) {
				Mat twin = src.section(src.mirror_u - u, vv);
				for (int j = 0; j < pts.rows() && j < twin.rows(); ++j) for (int c = 0; c < 3; ++c) pts(j, c) = 0.5 * (pts(j, c) + twin(j, c));
			}
			std::vector<Vec3> row;
			for (int j = 0; j < pts.rows(); ++j) row.push_back({pts(j, 0), pts(j, 1), pts(j, 2)});
			rr.push_back(std::move(row));
		}
		return rr;
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
	// A tagged seam (the aero copy of a lifting body) is closed by the
	// blend instead: the gap taken out over the last `seam_blend` of the
	// ring's parameter at each end, quadratically, so the surface slopes
	// are kept and the two ends meet at the gap's midpoint -- never a base
	// column, which would leave the wake nothing to abut.
	if (!src.closed_curve && !half) {
		bool apart = false;
		for (const auto& r : rows) apart = apart || Distance(r.front(), r.back()) > coincide;
		if (v_seam) {
			const double vb = std::clamp(pan.seam_blend, 1e-6, 0.49);
			for (auto& r : rows) {
				const Vec3 g = r.back() - r.front();
				if (apart) {
					for (size_t j = 0; j < r.size() && j < vs.size(); ++j) {
						const double v = vs[j];
						if (v < vb) { const double f = (vb - v) / vb; r[j] += g * (0.5 * f * f); }
						else if (v > 1.0 - vb) { const double f = (v - (1.0 - vb)) / vb; r[j] -= g * (0.5 * f * f); }
					}
				}
				const Vec3 mid = (r.front() + r.back()) * 0.5;
				r.front() = mid;
				r.back() = mid;
			}
			out.seam_closed = true;
		} else if (apart) {
			for (auto& r : rows) r.push_back(r.front());
		}
	}
	// Rows that coincide with their predecessor (a rim of zero thickness, a
	// seed on top of a uniform station, the collapsed trailing-edge rim)
	// would make panels of no area.
	std::vector<double> row_u = us;
	{
		std::vector<std::vector<Vec3>> kept;
		std::vector<double> kept_u;
		for (size_t i = 0; i < rows.size(); ++i) {
			if (!kept.empty()) {
				double d = 0.0;
				for (size_t j = 0; j < rows[i].size(); ++j) d = std::max(d, Distance(rows[i][j], kept.back()[j]));
				if (d <= coincide) continue;
			}
			kept.push_back(std::move(rows[i]));
			kept_u.push_back(row_u[i]);
		}
		rows = std::move(kept);
		row_u = std::move(kept_u);
	}
	std::vector<bool> row_cap(rows.size(), false);
	// Caps: an open end that is not already an apex gets a row collapsed to
	// its centroid, a fan of triangular panels (from the midpoint of the
	// chord on the plane for a half ring, so the fan fills the half disc).
	// An end ring that has collapsed to a line (a fuselage whose half-width
	// vanishes at the tail while its height does not) encloses nothing and
	// needs no cap: a fan on it would be a ring of panels of no area, whose
	// frames are NaN and take the whole solution with them (the P-38 boom,
	// 2026-10-27).
	// A lifting aero copy is left OPEN at its ends, as WingNetworks leaves a
	// wing's tube: capped, the interior-potential formulation on a sharp-
	// edged section goes singular (a capped airfoil slab with its wake,
	// 2026-11-13), and a mean-surface sheet has no inside to cap.
	if (!src.periodic_u && !v_seam && !mean_sheet) {
		auto apex_of = [half](const std::vector<Vec3>& r) { return half ? (r.front() + r.back()) * 0.5 : RowCentroid(r); };
		auto open = [&](const std::vector<Vec3>& r) { return RowSpread(r) > coincide && RingArea(r) > 1e-9 * scale * scale; };
		const double nan = std::numeric_limits<double>::quiet_NaN();
		if (src.cap_start && open(rows.front())) {
			rows.insert(rows.begin(), std::vector<Vec3>(rows.front().size(), apex_of(rows.front())));
			row_u.insert(row_u.begin(), nan);
			row_cap.insert(row_cap.begin(), true);
		}
		if (src.cap_end && open(rows.back())) {
			rows.push_back(std::vector<Vec3>(rows.back().size(), apex_of(rows.back())));
			row_u.push_back(nan);
			row_cap.push_back(true);
		}
	}
	if (rows.size() < 2) throw std::invalid_argument("LoftNetwork: the loft '" + name + "' collapses to one station");
	if (mean_sheet) out.te_row = static_cast<int>(rows.size()) - 1;
	out.rows = std::move(rows);
	out.row_u = std::move(row_u);
	out.row_cap = std::move(row_cap);
	out.half = half;
	out.scale = scale;
	return out;
}

} // namespace

Network LoftNetwork(const NamedLoft& loft, const std::string& name, const BodyPaneling& pan, const Vec3& free_stream) {
	SampledLoft s = SampleLoft(loft, name, pan, false);
	const bool flip = NeedsFlip(s.rows, s.half, pan.half_plane_y.value_or(0.0));
	return MakeNetwork(s.rows, name, !loft.flat, pan.sources ? NetKind::Sources : NetKind::Doublet, free_stream, flip);
}

std::vector<Network> LoftNetworks(const NamedLoft& loft, const std::string& name, const BodyPaneling& pan, const Vec3& free_stream) {
	const LoftTrailingEdge& tag = loft.trailing_edge;
	if (!pan.wake || !tag.IsSet()) return {LoftNetwork(loft, name, pan, free_stream)};
	const bool v_seam = tag.kind == LoftTrailingEdge::Kind::VSeam;
	if (!v_seam && !std::isfinite(loft.source.mirror_u)) return {LoftNetwork(loft, name, pan, free_stream)};
	SampledLoft s = SampleLoft(loft, name, pan, true);
	if ((v_seam && (s.half || !s.seam_closed)) || (!v_seam && s.te_row < 0)) {
		// No edge to shed from (a half ring, a closed curve tagged as a seam):
		// the closed body, as before.
		return {LoftNetwork(loft, name, pan, free_stream)};
	}
	const double length = pan.wake_length > 0.0 ? pan.wake_length : 20.0 * s.scale;
	std::vector<Network> out;
	std::vector<Vec3> te;
	// The trailing edge, for the buried test before any network is made.
	if (!pan.other_bodies.empty()) {
		std::vector<Vec3> edge;
		if (v_seam) { for (size_t i = 0; i < s.rows.size(); ++i) if (!s.row_cap[i]) edge.push_back(s.rows[i].front()); }
		else edge = s.rows.back();
		const double probe = pan.buried_probe > 0.0 ? pan.buried_probe : 0.02 * s.scale;
		const double tolerance = pan.buried_tolerance > 0.0 ? pan.buried_tolerance : 0.05 * s.scale;
		if (TrailingEdgeBuried(edge, pan.wake_direction, probe, tolerance, pan.other_bodies)) {
			if (pan.notes) pan.notes->push_back(name + ": its trailing edge runs into another body (a wake shed there would pierce it), so it is paneled as a closed non-lifting body");
			return {LoftNetwork(loft, name, pan, free_stream)};
		}
	}
	if (v_seam) {
		// One thick network, open at its ends, under the interior-potential
		// formulation (what WingNetworks makes of a wing); the trailing edge
		// is the seam point of every station.
		const bool flip = NeedsFlip(s.rows, s.half, pan.half_plane_y.value_or(0.0));
		out.push_back(MakeNetwork(s.rows, name, !loft.flat, NetKind::Doublet, free_stream, flip));
		for (size_t i = 0; i < s.rows.size(); ++i) if (!s.row_cap[i]) te.push_back(s.rows[i].front());
	} else {
		// The wall's mean surface as one sheet under the velocity condition,
		// facing outward, its last row the trailing edge.
		out.push_back(MakeNetwork(s.rows, name, !loft.flat, NetKind::Sheet, free_stream, SheetFacesInward(s.rows)));
		te = s.rows.back();
	}
	if (te.size() < 2) return {LoftNetwork(loft, name, pan, free_stream)};
	out.push_back(WakeNetwork(name + " wake", te, pan.wake_direction, length));
	return out;
}

std::vector<std::vector<Vec3>> LoftTrailingEdgePolylines(const NamedLoft& loft, int samples) {
	std::vector<std::vector<Vec3>> out;
	const LoftTrailingEdge& tag = loft.trailing_edge;
	const LoftSource& src = loft.source;
	if (!tag.IsSet() || !src.section) return out;
	if (tag.kind == LoftTrailingEdge::Kind::UStation) {
		std::vector<double> vs = GridParameters(0.0, 1.0, std::max(samples, 8), src.v_seeds);
		Mat p = src.section(tag.u, vs);
		std::vector<Vec3> ring;
		for (int i = 0; i < p.rows(); ++i) ring.push_back({p(i, 0), p(i, 1), p(i, 2)});
		if (!ring.empty() && Distance(ring.front(), ring.back()) > 0.0) ring.push_back(ring.front());
		out.push_back(std::move(ring));
		return out;
	}
	std::vector<double> us = GridParameters(src.u_min, src.u_max, std::max(samples, 8), src.u_seeds);
	std::vector<Vec3> line;
	for (double u : us) {
		Mat p = src.section(u, {0.0, 1.0});
		if (p.rows() < 2) continue;
		line.push_back({0.5 * (p(0, 0) + p(1, 0)), 0.5 * (p(0, 1) + p(1, 1)), 0.5 * (p(0, 2) + p(1, 2))});
	}
	out.push_back(std::move(line));
	return out;
}

bool TrailingEdgeBuried(const std::vector<Vec3>& trailing_edge, const Vec3& direction, double probe, double tolerance,
	const std::vector<const TriMesh*>& bodies, double fraction) {
	if (trailing_edge.empty() || bodies.empty()) return false;
	const Vec3 step = Normalized(direction) * probe;
	std::vector<std::unique_ptr<MeshSdf>> fields;
	for (const TriMesh* m : bodies) if (m != nullptr && !m->triangles.empty()) fields.push_back(std::make_unique<MeshSdf>(*m));
	if (fields.empty()) return false;
	int near = 0, inside = 0;
	for (const Vec3& p : trailing_edge) {
		const Vec3 q = p + step;
		double d = 1e300;
		for (const auto& f : fields) d = std::min(d, f->Distance(q));
		if (d < tolerance) ++near;
		if (d < 0.0) ++inside;
	}
	// A wake that would run INSIDE a body along a quarter of its edge
	// pierces it; one that merely skims a body along most of its edge sits
	// on it.
	const double n = static_cast<double>(trailing_edge.size());
	return inside >= 0.25 * n || near >= fraction * n;
}

std::vector<Network> BodyNetworks(const ISolidModel& model, const std::string& name, const BodyPaneling& pan, const Vec3& free_stream) {
	std::vector<NamedLoft> lofts = model.BuildLofts();
	std::vector<Network> out;
	for (const NamedLoft& l : lofts) {
		std::string n = lofts.size() == 1 ? name : name + "/" + l.name;
		for (Network& net : LoftNetworks(l, n, pan, free_stream)) out.push_back(std::move(net));
	}
	return out;
}

} // namespace aerotelic::panel
