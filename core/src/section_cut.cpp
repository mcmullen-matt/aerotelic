#include "aerotelic/section_cut.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>

namespace aerotelic {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

struct Pt {
	double x, y;
};

Pt Eval(const NurbsCurve& c, double t) {
	auto p = c.Evaluate(t);
	return {p[0], p[1]};
}

Pt Deriv(const NurbsCurve& c, double t) {
	auto p = c.Derivative(t);
	return {p[0], p[1]};
}

// Second derivative by central difference of the first (the curve exposes
// only the first); h well above the roundoff floor of the hodograph and
// well below any feature of a fitted airfoil.
Pt Deriv2(const NurbsCurve& c, double t) {
	const double h = 1e-6;
	double a = std::max(0.0, t - h), b = std::min(1.0, t + h);
	Pt da = Deriv(c, a), db = Deriv(c, b);
	return {(db.x - da.x) / (b - a), (db.y - da.y) / (b - a)};
}

// The parameter in [ta, tb] at which x crosses `x`, by bisection; x is
// monotone over one surface of a sane airfoil, which is all this asks.
double TAtX(const NurbsCurve& c, double ta, double tb, double x) {
	double fa = Eval(c, ta).x - x, fb = Eval(c, tb).x - x;
	if (fa * fb > 0.0) throw std::invalid_argument("hinge x is not crossed by the surface between the given parameters");
	double lo = ta, hi = tb;
	for (int it = 0; it < 80 && hi - lo > 1e-14; ++it) {
		double mid = 0.5 * (lo + hi);
		double fm = Eval(c, mid).x - x;
		if (fa * fm <= 0.0) { hi = mid; fb = fm; } else { lo = mid; fa = fm; }
	}
	return 0.5 * (lo + hi);
}

// Gaussian elimination with partial pivoting on a 4x4 system; false when
// singular.
bool Solve4(double A[4][4], double b[4], double x[4]) {
	int idx[4] = {0, 1, 2, 3};
	for (int k = 0; k < 4; ++k) {
		int piv = k;
		for (int i = k + 1; i < 4; ++i) if (std::fabs(A[idx[i]][k]) > std::fabs(A[idx[piv]][k])) piv = i;
		std::swap(idx[k], idx[piv]);
		double d = A[idx[k]][k];
		if (std::fabs(d) < 1e-300) return false;
		for (int i = k + 1; i < 4; ++i) {
			double f = A[idx[i]][k] / d;
			if (f == 0.0) continue;
			for (int j = k; j < 4; ++j) A[idx[i]][j] -= f * A[idx[k]][j];
			b[idx[i]] -= f * b[idx[k]];
		}
	}
	for (int k = 3; k >= 0; --k) {
		double s = b[idx[k]];
		for (int j = k + 1; j < 4; ++j) s -= A[idx[k]][j] * x[j];
		x[k] = s / A[idx[k]][k];
	}
	return true;
}

// Marches from t_from toward t_to looking for the first sign change of
// f(t) = |C(t) - c|^2 - r^2 (negative at t_from), then bisects. NaN when f
// stays negative all the way.
double FirstCrossing(const NurbsCurve& c, double t_from, double t_to, double cx, double cy, double r2) {
	auto f = [&](double t) {
		Pt p = Eval(c, t);
		return (p.x - cx) * (p.x - cx) + (p.y - cy) * (p.y - cy) - r2;
	};
	const int n = 256;
	double prev_t = t_from, prev_f = f(t_from);
	// Already on the circle (a zero gap: the cavity is the nose, and the
	// crossing is the tangency point itself).
	if (prev_f >= -1e-14) return t_from;
	for (int i = 1; i <= n; ++i) {
		double t = t_from + (t_to - t_from) * i / n;
		double ft = f(t);
		if (prev_f < 0.0 && ft >= 0.0) {
			double lo = prev_t, hi = t, flo = prev_f;
			for (int it = 0; it < 80 && std::fabs(hi - lo) > 1e-14; ++it) {
				double mid = 0.5 * (lo + hi), fm = f(mid);
				if (flo * fm <= 0.0) hi = mid; else { lo = mid; flo = fm; }
			}
			return 0.5 * (lo + hi);
		}
		prev_t = t;
		prev_f = ft;
	}
	return kNaN;
}

double AngleAbout(const Pt& p, double cx, double cy) { return std::atan2(p.y - cy, p.x - cx); }

double Deg(double rad) { return rad * 180.0 / kPi; }

double AngleBetween(Pt a, Pt b) {
	double na = std::hypot(a.x, a.y), nb = std::hypot(b.x, b.y);
	if (na < 1e-300 || nb < 1e-300) return 0.0;
	double cs = std::clamp((a.x * b.x + a.y * b.y) / (na * nb), -1.0, 1.0);
	return std::acos(cs);
}

std::string Fmt(const char* fmt, double a, double b = 0.0) {
	char buf[200];
	std::snprintf(buf, sizeof buf, fmt, a, b);
	return buf;
}

} // namespace

// ---------------------------------------------------------------------------
// SolveFlapCut
// ---------------------------------------------------------------------------

FlapCut SolveFlapCut(const Airfoil& airfoil, double x_hinge, double gap) {
	const NurbsCurve& c = airfoil.nurbs_curve();
	const double t_le = airfoil.LeadingEdgeParameter();
	if (!(gap >= 0.0) || !std::isfinite(gap)) throw std::invalid_argument("the flap gap must be a finite, non-negative length");
	const Pt le = Eval(c, t_le);
	const double x_te = std::max(Eval(c, 0.0).x, Eval(c, 1.0).x);
	if (!(x_hinge > le.x && x_hinge < x_te)) throw std::invalid_argument(Fmt("hinge x = %.4f is off the section (x runs %.4f to ", x_hinge, le.x) + Fmt("%.4f)", x_te));

	// ---- initial guess: the vertical chord at the hinge ----
	double tu = TAtX(c, t_le, 1.0, x_hinge), tl = TAtX(c, 0.0, t_le, x_hinge);
	Pt pu = Eval(c, tu), pl = Eval(c, tl);
	double yc = 0.5 * (pu.y + pl.y), R = 0.5 * (pu.y - pl.y);
	if (!(R > 1e-9)) throw std::invalid_argument(Fmt("the section has no thickness at the hinge (x = %.4f)", x_hinge));

	// ---- Newton for the tangent circle ----
	// Unknowns X = (y_c, R, t_u, t_l); equations: the tangency point is a
	// radius from the centre, and the radius is normal to the curve there,
	// on each surface.
	double X[4] = {yc, R, tu, tl};
	auto residual = [&](const double* Xv, double* F) {
		Pt Pu = Eval(c, Xv[2]), Du = Deriv(c, Xv[2]);
		Pt Pl = Eval(c, Xv[3]), Dl = Deriv(c, Xv[3]);
		double ux = Pu.x - x_hinge, uy = Pu.y - Xv[0];
		double lx = Pl.x - x_hinge, ly = Pl.y - Xv[0];
		F[0] = ux * ux + uy * uy - Xv[1] * Xv[1];
		F[1] = ux * Du.x + uy * Du.y;
		F[2] = lx * lx + ly * ly - Xv[1] * Xv[1];
		F[3] = lx * Dl.x + ly * Dl.y;
		return std::sqrt(F[0] * F[0] + F[1] * F[1] + F[2] * F[2] + F[3] * F[3]);
	};
	double F[4];
	double fn = residual(X, F);
	bool converged = fn < 1e-14;
	for (int it = 0; it < 60 && !converged; ++it) {
		Pt Pu = Eval(c, X[2]), Du = Deriv(c, X[2]), DDu = Deriv2(c, X[2]);
		Pt Pl = Eval(c, X[3]), Dl = Deriv(c, X[3]), DDl = Deriv2(c, X[3]);
		double ux = Pu.x - x_hinge, uy = Pu.y - X[0];
		double lx = Pl.x - x_hinge, ly = Pl.y - X[0];
		double J[4][4] = {
			{-2.0 * uy, -2.0 * X[1], 2.0 * (ux * Du.x + uy * Du.y), 0.0},
			{-Du.y, 0.0, Du.x * Du.x + Du.y * Du.y + ux * DDu.x + uy * DDu.y, 0.0},
			{-2.0 * ly, -2.0 * X[1], 0.0, 2.0 * (lx * Dl.x + ly * Dl.y)},
			{-Dl.y, 0.0, 0.0, Dl.x * Dl.x + Dl.y * Dl.y + lx * DDl.x + ly * DDl.y},
		};
		double rhs[4] = {-F[0], -F[1], -F[2], -F[3]}, dx[4];
		if (!Solve4(J, rhs, dx)) break;
		// Backtracking: the step must reduce the residual, and the tangency
		// parameters must stay on their own surfaces.
		double lambda = 1.0, Xn[4], Fn[4], fnn = fn;
		bool better = false;
		for (int k = 0; k < 30; ++k) {
			for (int i = 0; i < 4; ++i) Xn[i] = X[i] + lambda * dx[i];
			Xn[2] = std::clamp(Xn[2], t_le, 1.0);
			Xn[3] = std::clamp(Xn[3], 0.0, t_le);
			fnn = residual(Xn, Fn);
			if (fnn < fn) { better = true; break; }
			lambda *= 0.5;
		}
		if (!better) break;
		double step = 0.0;
		for (int i = 0; i < 4; ++i) step = std::max(step, std::fabs(Xn[i] - X[i]));
		for (int i = 0; i < 4; ++i) { X[i] = Xn[i]; F[i] = Fn[i]; }
		fn = fnn;
		converged = fn < 1e-14 || step < 1e-14;
	}
	if (!(fn < 1e-9)) throw std::invalid_argument(Fmt("the tangent nose circle at x = %.4f did not converge (residual %.3g)", x_hinge, fn));
	yc = X[0]; R = X[1]; tu = X[2]; tl = X[3];
	if (!(R > 0.0) || !(tu > t_le && tu < 1.0) || !(tl > 0.0 && tl < t_le)) {
		throw std::invalid_argument(Fmt("no tangent nose circle inside the section at x = %.4f", x_hinge));
	}
	pu = Eval(c, tu);
	pl = Eval(c, tl);

	FlapCut cut;
	cut.x_hinge = x_hinge;
	cut.y_hinge = yc;
	cut.gap = gap;
	cut.r_nose = R;
	cut.r_cavity = R + gap;
	cut.t_upper_tangent = tu;
	cut.t_lower_tangent = tl;
	cut.a_upper_tangent = AngleAbout(pu, x_hinge, yc);
	cut.a_lower_tangent = AngleAbout(pl, x_hinge, yc);
	if (!(cut.a_upper_tangent > 0.0 && cut.a_upper_tangent < kPi) || !(cut.a_lower_tangent < 0.0 && cut.a_lower_tangent > -kPi)) {
		throw std::invalid_argument(Fmt("the nose circle's tangency points at x = %.4f are not on opposite sides of the hinge", x_hinge));
	}

	// ---- the cavity circle's crossings of the skin ----
	const double r2 = cut.r_cavity * cut.r_cavity;
	cut.t_upper_lip = FirstCrossing(c, tu, t_le, x_hinge, yc, r2);
	cut.t_lower_lip = FirstCrossing(c, tl, t_le, x_hinge, yc, r2);
	if (std::isnan(cut.t_upper_lip) || std::isnan(cut.t_lower_lip)) {
		throw std::invalid_argument(Fmt("the cavity (radius %.4f about x = %.4f) reaches the leading edge: the gap is too wide for this section", cut.r_cavity, x_hinge));
	}
	cut.t_upper_aft = FirstCrossing(c, tu, 1.0, x_hinge, yc, r2);
	cut.t_lower_aft = FirstCrossing(c, tl, 0.0, x_hinge, yc, r2);
	cut.a_upper_lip = AngleAbout(Eval(c, cut.t_upper_lip), x_hinge, yc);
	cut.a_lower_lip = AngleAbout(Eval(c, cut.t_lower_lip), x_hinge, yc);
	cut.a_upper_aft = std::isnan(cut.t_upper_aft) ? kNaN : AngleAbout(Eval(c, cut.t_upper_aft), x_hinge, yc);
	cut.a_lower_aft = std::isnan(cut.t_lower_aft) ? kNaN : AngleAbout(Eval(c, cut.t_lower_aft), x_hinge, yc);

	// ---- deflection limits ----
	// Trailing edge down turns the flap clockwise (angles decrease): the
	// lower aft crossing, at angle a_lower_aft (near -pi/2 from below, i.e.
	// greater than a_lower_lip), sweeps down onto L'. Up: the upper aft
	// crossing (angle below a_upper_lip) sweeps up onto U'.
	const double inf = std::numeric_limits<double>::infinity();
	cut.deflection_down_limit_deg = std::isnan(cut.a_lower_aft) ? inf : Deg(cut.a_lower_aft - cut.a_lower_lip);
	cut.deflection_up_limit_deg = std::isnan(cut.a_upper_aft) ? inf : Deg(cut.a_upper_lip - cut.a_upper_aft);

	// ---- lip corners ----
	// Upper: the skin runs forward as -C'(t) (t increases aft on the upper
	// surface); the cavity arc leaves U' with increasing angle. Lower: the
	// skin runs forward as +C'(t); the arc leaves L' with decreasing angle.
	{
		Pt d = Deriv(c, cut.t_upper_lip);
		Pt skin{-d.x, -d.y}, arc{-std::sin(cut.a_upper_lip), std::cos(cut.a_upper_lip)};
		cut.upper_lip_angle_deg = Deg(AngleBetween(skin, arc));
		d = Deriv(c, cut.t_lower_lip);
		Pt skin_l{d.x, d.y}, arc_l{std::sin(cut.a_lower_lip), -std::cos(cut.a_lower_lip)};
		cut.lower_lip_angle_deg = Deg(AngleBetween(skin_l, arc_l));
	}
	return cut;
}

// ---------------------------------------------------------------------------
// SectionLoop
// ---------------------------------------------------------------------------

SectionLoop SectionLoop::Full(const Airfoil& airfoil) {
	SectionLoop loop;
	loop.airfoil_ = &airfoil;
	loop.closed_ = false;
	double t_le = airfoil.LeadingEdgeParameter();
	Piece lower, upper;
	lower.kind = Piece::Kind::Skin; lower.v0 = 0.0; lower.v1 = t_le; lower.t0 = 0.0; lower.t1 = t_le;
	upper.kind = Piece::Kind::Skin; upper.v0 = t_le; upper.v1 = 1.0; upper.t0 = t_le; upper.t1 = 1.0;
	loop.pieces_ = {lower, upper};
	// No seeds: the leading edge's parameter varies along the span, and a
	// seed must be the same on every ring.
	return loop;
}

SectionLoop SectionLoop::Flap(const Airfoil& airfoil, const FlapCut& cut) {
	SectionLoop loop;
	loop.airfoil_ = &airfoil;
	loop.closed_ = false;
	loop.cx_ = cut.x_hinge; loop.cy_ = cut.y_hinge; loop.r_ = cut.r_nose;
	Piece lower, arc_lo, arc_up, upper;
	lower.kind = Piece::Kind::Skin; lower.v0 = 0.0; lower.v1 = 0.35; lower.t0 = 0.0; lower.t1 = cut.t_lower_tangent;
	// The nose arc from T_l forward through the extreme (angle pi) to T_u:
	// angle decreasing from a_lower_tangent + 2 pi (in (pi, 2 pi)) to
	// a_upper_tangent (in (0, pi)).
	arc_lo.kind = Piece::Kind::Arc; arc_lo.v0 = 0.35; arc_lo.v1 = 0.5; arc_lo.a0 = cut.a_lower_tangent + 2.0 * kPi; arc_lo.a1 = kPi;
	arc_up.kind = Piece::Kind::Arc; arc_up.v0 = 0.5; arc_up.v1 = 0.65; arc_up.a0 = kPi; arc_up.a1 = cut.a_upper_tangent;
	upper.kind = Piece::Kind::Skin; upper.v0 = 0.65; upper.v1 = 1.0; upper.t0 = cut.t_upper_tangent; upper.t1 = 1.0;
	loop.pieces_ = {lower, arc_lo, arc_up, upper};
	loop.seeds_ = {0.35, 0.5, 0.65};
	return loop;
}

SectionLoop SectionLoop::MainElement(const Airfoil& airfoil, const FlapCut& cut) {
	SectionLoop loop;
	loop.airfoil_ = &airfoil;
	loop.closed_ = true;
	loop.cx_ = cut.x_hinge; loop.cy_ = cut.y_hinge; loop.r_ = cut.r_cavity;
	double t_le = airfoil.LeadingEdgeParameter();
	Piece lower, upper, arc_up, arc_lo;
	lower.kind = Piece::Kind::Skin; lower.v0 = 0.0; lower.v1 = 0.25; lower.t0 = cut.t_lower_lip; lower.t1 = t_le;
	upper.kind = Piece::Kind::Skin; upper.v0 = 0.25; upper.v1 = 0.5; upper.t0 = t_le; upper.t1 = cut.t_upper_lip;
	// The cavity arc from U' forward through the extreme to L': angle
	// increasing from a_upper_lip (in (0, pi)) to a_lower_lip + 2 pi.
	arc_up.kind = Piece::Kind::Arc; arc_up.v0 = 0.5; arc_up.v1 = 0.75; arc_up.a0 = cut.a_upper_lip; arc_up.a1 = kPi;
	arc_lo.kind = Piece::Kind::Arc; arc_lo.v0 = 0.75; arc_lo.v1 = 1.0; arc_lo.a0 = kPi; arc_lo.a1 = cut.a_lower_lip + 2.0 * kPi;
	loop.pieces_ = {lower, upper, arc_up, arc_lo};
	loop.seeds_ = {0.25, 0.5, 0.75};
	return loop;
}

double SectionLoop::Wrap(double v) const {
	if (closed_) {
		v = std::fmod(v, 1.0);
		if (v < 0.0) v += 1.0;
		return v;
	}
	return std::clamp(v, 0.0, 1.0);
}

std::vector<double> SectionLoop::PointOnPiece(const Piece& p, double v) const {
	double f = p.v1 > p.v0 ? (v - p.v0) / (p.v1 - p.v0) : 0.0;
	f = std::clamp(f, 0.0, 1.0);
	if (p.kind == Piece::Kind::Skin) {
		double t = p.t0 + (p.t1 - p.t0) * f;
		return airfoil_->nurbs_curve().Evaluate(t);
	}
	double a = p.a0 + (p.a1 - p.a0) * f;
	return {cx_ + r_ * std::cos(a), cy_ + r_ * std::sin(a)};
}

std::vector<double> SectionLoop::Point(double v) const {
	v = Wrap(v);
	for (const Piece& p : pieces_) {
		if (v >= p.v0 && v <= p.v1) return PointOnPiece(p, v);
	}
	return PointOnPiece(pieces_.back(), v);
}

Mat SectionLoop::Points(const std::vector<double>& v) const {
	Mat out(static_cast<int>(v.size()), 2);
	for (size_t i = 0; i < v.size(); ++i) out.set_row(static_cast<int>(i), Point(v[i]));
	return out;
}

double SectionLoop::Mirror(double v) const {
	v = Wrap(v);
	std::vector<double> self = Point(v);
	const double x = self[0];
	struct Root { double y, v; };
	std::vector<Root> roots;
	// The point itself is a crossing (found directly: a fitted skin can
	// overshoot its end's x by a hair near the trailing edge, where the
	// end-range test below would miss it).
	roots.push_back({self[1], v});
	const Piece* own = nullptr;
	for (const Piece& p : pieces_) if (v >= p.v0 && v <= p.v1) { own = &p; break; }
	for (const Piece& p : pieces_) {
		if (&p == own) continue;
		std::vector<double> P0 = PointOnPiece(p, p.v0), P1 = PointOnPiece(p, p.v1);
		double fa = P0[0] - x, fb = P1[0] - x;
		const double eps = 1e-13;
		double vr;
		if (std::fabs(fa) <= eps || std::fabs(fb) <= eps) {
			// The query point is (or is within roundoff of) an end of this
			// piece -- the usual case for a point on a piece boundary, which
			// both its pieces report. Bisecting would read the end's residual
			// with a roundoff sign and walk off to the far end.
			vr = std::fabs(fa) <= std::fabs(fb) ? p.v0 : p.v1;
		} else if (fa * fb > 0.0) {
			continue; // x outside the piece's range
		} else {
			// x is monotone along the piece: bisect for x(v) = x.
			double va = p.v0, vb = p.v1;
			for (int it = 0; it < 80 && std::fabs(vb - va) > 1e-15; ++it) {
				double vm = 0.5 * (va + vb);
				double fm = PointOnPiece(p, vm)[0] - x;
				if (fa * fm <= 0.0) vb = vm; else { va = vm; fa = fm; }
			}
			vr = 0.5 * (va + vb);
		}
		roots.push_back({PointOnPiece(p, vr)[1], vr});
	}
	std::sort(roots.begin(), roots.end(), [](const Root& a, const Root& b) { return a.y < b.y; });
	// A shared piece boundary is found from both its pieces: one point.
	std::vector<Root> uniq;
	for (const Root& r : roots) {
		if (!uniq.empty() && std::fabs(uniq.back().y - r.y) <= 1e-10) continue;
		uniq.push_back(r);
	}
	if (uniq.size() < 2 || uniq.size() % 2 != 0) return v; // an x extreme: its own twin
	size_t k = 0;
	for (size_t i = 1; i < uniq.size(); ++i) if (std::fabs(uniq[i].y - self[1]) < std::fabs(uniq[k].y - self[1])) k = i;
	double m = uniq[k ^ 1].v;
	// The closing point of a closed loop is v = 0, not v = 1.
	if (closed_ && m >= 1.0 - 1e-12) m = 0.0;
	return m;
}

// Both maps accept a parameter a hair (1e-9) outside a skin piece and clamp
// it: the loft's parameter sets are merged at that tolerance, so a piece
// boundary can arrive as its neighbour within it.
double SectionLoop::AirfoilParameter(double v) const {
	if (!closed_) v = std::clamp(v, 0.0, 1.0);
	else if (v < 0.0 && v > -1e-9) v = 0.0;
	else v = Wrap(v);
	for (const Piece& p : pieces_) {
		if (p.kind != Piece::Kind::Skin || v < p.v0 - 1e-9 || v > p.v1 + 1e-9) continue;
		double f = p.v1 > p.v0 ? (v - p.v0) / (p.v1 - p.v0) : 0.0;
		return p.t0 + (p.t1 - p.t0) * std::clamp(f, 0.0, 1.0);
	}
	return kNaN;
}

double SectionLoop::ParameterAtAirfoil(double t) const {
	for (const Piece& p : pieces_) {
		if (p.kind != Piece::Kind::Skin) continue;
		double lo = std::min(p.t0, p.t1), hi = std::max(p.t0, p.t1);
		if (t < lo - 1e-9 || t > hi + 1e-9) continue;
		double f = hi > lo ? (t - p.t0) / (p.t1 - p.t0) : 0.0;
		return p.v0 + (p.v1 - p.v0) * std::clamp(f, 0.0, 1.0);
	}
	return kNaN;
}

} // namespace aerotelic
