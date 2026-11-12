#include "aerotelic/section_aero.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "aerotelic/airfoil.hpp"
#include "aerotelic/conformal_airfoil.hpp"

namespace aerotelic {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
const double kNaN = std::numeric_limits<double>::quiet_NaN();

double Reduce(double angle) {
	angle -= kTwoPi * std::floor(angle / kTwoPi);
	return angle;
}

// Linear interpolation of y(x) on a surface whose x need not be monotonic
// near the stagnation point: the first crossing of `x` walking from the
// stagnation point.
double InterpolateAlong(const std::vector<double>& xs, const std::vector<double>& ys, double x) {
	for (size_t i = 1; i < xs.size(); ++i) {
		const double x0 = xs[i - 1], x1 = xs[i];
		if ((x0 <= x && x <= x1) || (x1 <= x && x <= x0)) {
			if (x1 == x0) return ys[i];
			return ys[i - 1] + (x - x0) / (x1 - x0) * (ys[i] - ys[i - 1]);
		}
	}
	return kNaN;
}

// ---------------------------------------------------------------------------
// Pressure integration over the closed grid polygon (counter-clockwise):
// dF = -Cp n ds with n ds = (dy, -dx), moment nose-up positive.
// ---------------------------------------------------------------------------
void IntegratePressure(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& cp,
	double chord, double alpha_rad, double x_ref, double y_ref,
	double& cl, double& cd, double& cm) {
	const int n = static_cast<int>(x.size());
	double cx = 0.0, cy = 0.0, m = 0.0;
	for (int i = 0; i < n; ++i) {
		const int j = (i + 1) % n;
		const double dx = x[j] - x[i], dy = y[j] - y[i];
		const double c = 0.5 * (cp[i] + cp[j]);
		const double xm = 0.5 * (x[i] + x[j]), ym = 0.5 * (y[i] + y[j]);
		cx += -c * dy;
		cy += c * dx;
		// counter-clockwise torque of (dFx, dFy) = (-c dy, c dx) about the reference
		m += (xm - x_ref) * (c * dx) - (ym - y_ref) * (-c * dy);
	}
	cx /= chord;
	cy /= chord;
	m /= chord * chord;
	cl = cy * std::cos(alpha_rad) - cx * std::sin(alpha_rad);
	cd = cx * std::cos(alpha_rad) + cy * std::sin(alpha_rad);
	cm = -m; // nose-up positive
}

// ---------------------------------------------------------------------------
// Theodorsen / Ives conformal mapping
// ---------------------------------------------------------------------------
class ConformalMappingMethod : public SectionAeroMethod {
public:
	std::string Name() const override { return "Conformal map (Theodorsen/Ives)"; }
	std::string Description() const override {
		return "Exact incompressible potential flow by Theodorsen's airfoil-to-circle map, computed as in Ives (1976): "
			"Karman-Trefftz pre-map, Garrick conjugate by FFT. Kutta condition at the trailing edge.";
	}

	SectionAeroResult Solve(const SectionAeroRequest& request) const override {
		SectionAeroResult r;
		r.method = Name();
		r.alpha_deg = request.alpha_deg;
		r.reynolds = request.reynolds;
		const auto t0 = std::chrono::steady_clock::now();

		r.contour = CloseTrailingEdge(request.contour, request.te_blend_start, &r.te_gap_closed);
		ConformalMapOptions opt;
		opt.n_points = request.n_points;
		const ConformalMap map = MapAirfoil(r.contour, opt);
		r.iterations = map.iterations;
		r.residual = map.residual;
		r.convergence_margin = map.max_eps_prime;
		r.te_angle_deg = map.te_angle * 180.0 / kPi;
		if (!map.ok) { r.error = map.error; return r; }
		if (map.max_eps_prime > 0.9) r.warnings.push_back("conformal map close to its convergence limit (max |d eps/d phi| = " + std::to_string(map.max_eps_prime) + ")");

		const ConformalFlow flow = SolveFlow(map, request.alpha_deg);
		const int N = static_cast<int>(map.phi.size());
		const double chord = map.chord, x_min = map.x_min;
		const double alpha = flow.alpha;

		// Integrated coefficients over the grid polygon (increasing phi is
		// counter-clockwise round the section).
		std::vector<double> gx(N), gy(N);
		for (int j = 0; j < N; ++j) { gx[j] = map.z[j].real(); gy[j] = map.z[j].imag(); }
		double cl, cd, cm_le, cm_c4;
		IntegratePressure(gx, gy, flow.cp, chord, alpha, x_min, 0.0, cl, cd, cm_le);
		IntegratePressure(gx, gy, flow.cp, chord, alpha, x_min + 0.25 * chord, 0.0, cl, cd, cm_c4);
		r.cl = cl;
		r.cd_pressure = cd;
		r.cm_le = cm_le;
		r.cm_c4 = cm_c4;
		r.cl_exact = flow.cl_kutta;
		r.alpha0_deg = flow.alpha0 * 180.0 / kPi;
		r.cl_alpha = flow.cl_alpha;
		r.cp_min = *std::min_element(flow.cp.begin(), flow.cp.end());

		// Surfaces from the stagnation point. Increasing phi from the
		// stagnation point reaches the trailing edge along the LOWER surface
		// (theta decreases lower TE -> LE -> upper TE), decreasing phi along
		// the upper one.
		const double phi_s = flow.phi_stagnation;
		const double span_lower = Reduce(map.phi_te - phi_s);
		struct Pt { double d, x, y, q; };
		std::vector<Pt> lower, upper;
		for (int j = 0; j < N; ++j) {
			const double d_lo = Reduce(map.phi[j] - phi_s);
			const double d_up = Reduce(phi_s - map.phi[j]);
			if (d_lo > 1e-12 && d_lo < span_lower) lower.push_back({d_lo, map.z[j].real(), map.z[j].imag(), flow.q[j]});
			else if (d_up > 1e-12 && d_up < kTwoPi - span_lower) upper.push_back({d_up, map.z[j].real(), map.z[j].imag(), flow.q[j]});
		}
		auto by_d = [](const Pt& a, const Pt& b) { return a.d < b.d; };
		std::sort(lower.begin(), lower.end(), by_d);
		std::sort(upper.begin(), upper.end(), by_d);

		const std::complex<double> z_s = MapPointAt(map, phi_s);
		auto build = [&](const std::vector<Pt>& pts, SectionSurface& s) {
			s.x.clear(); s.y.clear(); s.s.clear(); s.q.clear(); s.cp.clear();
			auto push = [&](double x, double y, double q) {
				double ds = 0.0;
				if (!s.x.empty()) ds = std::hypot(x - s.x.back() * chord - x_min, y - s.y.back() * chord) / chord;
				s.x.push_back((x - x_min) / chord);
				s.y.push_back(y / chord);
				s.s.push_back(s.s.empty() ? 0.0 : s.s.back() + ds);
				s.q.push_back(q);
				s.cp.push_back(1.0 - q * q);
			};
			push(z_s.real(), z_s.imag(), 0.0);
			for (const Pt& p : pts) push(p.x, p.y, p.q);
			// Trailing edge: a finite included angle is a stagnation point of
			// the inviscid flow; a cusp has a finite speed, extrapolated.
			double q_te = 0.0;
			if (map.te_angle < 1e-3 && pts.size() >= 2) {
				const Pt& a = pts[pts.size() - 2];
				const Pt& b = pts.back();
				const double da = std::hypot(b.x - a.x, b.y - a.y), db = std::hypot(map.z_te.real() - b.x, map.z_te.imag() - b.y);
				q_te = std::max(0.0, b.q + (b.q - a.q) * (da > 0.0 ? db / da : 0.0));
			}
			push(map.z_te.real(), map.z_te.imag(), q_te);
		};
		build(lower, r.lower);
		build(upper, r.upper);

		r.solve_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
		r.ok = true;
		return r;
	}
};

const std::vector<std::unique_ptr<SectionAeroMethod>>& Registry() {
	static std::vector<std::unique_ptr<SectionAeroMethod>> methods = [] {
		std::vector<std::unique_ptr<SectionAeroMethod>> v;
		v.push_back(std::make_unique<ConformalMappingMethod>());
		v.push_back(MakeVortexPanelInviscidMethod());
		v.push_back(MakeVortexPanelIblMethod());
		return v;
	}();
	return methods;
}

} // namespace

// ---------------------------------------------------------------------------

SectionAeroRequest SectionRequestFor(const Airfoil& airfoil, int n_per_surface) {
	SectionAeroRequest req;
	req.contour = airfoil.ContourPoints(2 * n_per_surface);
	return req;
}

const std::vector<const SectionAeroMethod*>& SectionAeroMethods() {
	static std::vector<const SectionAeroMethod*> view = [] {
		std::vector<const SectionAeroMethod*> v;
		for (const auto& m : Registry()) v.push_back(m.get());
		return v;
	}();
	return view;
}

const SectionAeroMethod* FindSectionAeroMethod(const std::string& name) {
	for (const SectionAeroMethod* m : SectionAeroMethods()) if (m->Name() == name) return m;
	return nullptr;
}

Mat CloseTrailingEdge(const Mat& contour, double blend_start, double* gap_out) {
	const int n = contour.rows();
	if (n < 2 || contour.cols() != 2) throw std::invalid_argument("CloseTrailingEdge: need an (M, 2) contour");
	double xmin = contour(0, 0), xmax = contour(0, 0);
	for (int i = 0; i < n; ++i) { xmin = std::min(xmin, contour(i, 0)); xmax = std::max(xmax, contour(i, 0)); }
	const double chord = xmax - xmin;
	// The gap vector, upper end minus lower end (normally vertical; a
	// hand-moved trailing-edge pole can also offset it along the chord).
	const double gap_x = contour(n - 1, 0) - contour(0, 0), gap_y = contour(n - 1, 1) - contour(0, 1);
	const double gap = std::hypot(gap_x, gap_y);
	if (gap_out) *gap_out = gap / (chord > 0.0 ? chord : 1.0);
	Mat out = contour;
	if (!(chord > 0.0) || gap < 1e-12 * chord) {
		// Already closed (up to rounding): make the ends identical.
		const double xm = 0.5 * (contour(0, 0) + contour(n - 1, 0)), ym = 0.5 * (contour(0, 1) + contour(n - 1, 1));
		out(0, 0) = xm; out(0, 1) = ym; out(n - 1, 0) = xm; out(n - 1, 1) = ym;
		return out;
	}
	const double x0 = xmin + std::clamp(blend_start, 0.0, 0.999) * chord;
	// The contour runs lower TE -> LE -> upper TE: the minimum-x index splits it.
	int i_le = 0;
	for (int i = 1; i < n; ++i) if (contour(i, 0) < contour(i_le, 0)) i_le = i;
	for (int i = 0; i < n; ++i) {
		const double x = contour(i, 0);
		if (x <= x0) continue;
		const double f = (x - x0) / (xmax - x0);
		const double w = (i <= i_le) ? 0.5 * f * f : -0.5 * f * f; // lower up, upper down
		out(i, 0) += w * gap_x;
		out(i, 1) += w * gap_y;
	}
	const double xm = 0.5 * (out(0, 0) + out(n - 1, 0)), ym = 0.5 * (out(0, 1) + out(n - 1, 1));
	out(0, 0) = xm; out(0, 1) = ym; out(n - 1, 0) = xm; out(n - 1, 1) = ym;
	return out;
}

// ---------------------------------------------------------------------------
// Viscous cues on an inviscid surface distribution
// ---------------------------------------------------------------------------
void AddBoundaryLayerCues(SectionSurface& sf, double reynolds, double alpha_deg, double transition_re_x) {
	const int n = static_cast<int>(sf.s.size());
	sf.transition = {};
	sf.separation = {};
	sf.stratford.assign(n, 0.0);
	sf.stratford_range_end_x = -1.0;
	sf.cd_friction = 0.0;
	sf.cf_integral_x_equiv = 0.0;
	if (n < 4) { sf.transition.note = "too few points"; sf.separation.note = "too few points"; return; }

	// Suction peak.
	int ip = 0;
	for (int i = 1; i < n; ++i) if (sf.q[i] > sf.q[ip]) ip = i;
	sf.peak_index = ip;
	const double q_max = sf.q[ip];

	// ---- transition cue: the earlier of Re_x >= Re_tr and the first adverse
	// gradient (the suction peak). Re_x = Re_c (V_e / U) (s / c).
	{
		int i_re = -1;
		for (int i = 1; i < n; ++i) {
			if (reynolds * sf.q[i] * sf.s[i] >= transition_re_x) { i_re = i; break; }
		}
		int i_tr = ip;
		std::string note = "first adverse pressure gradient (suction peak)";
		if (i_re >= 0 && i_re < ip) { i_tr = i_re; note = "Re_x reached " + std::to_string(transition_re_x); }
		if (ip == n - 1 && i_re < 0) {
			sf.transition.note = "no adverse gradient and Re_x below the transition value: laminar to the trailing edge";
		} else {
			sf.transition.found = true;
			sf.transition.x = sf.x[i_tr];
			sf.transition.y = sf.y[i_tr];
			sf.transition.s = sf.s[i_tr];
			sf.transition.note = note;
		}
	}

	// ---- skin-friction drag: turbulent flat plate from the stagnation point,
	// Cf = 0.0592 Re_s^-0.2 at the local edge speed, projected on the wind.
	{
		const double alpha = alpha_deg * kPi / 180.0;
		const double ex = std::cos(alpha), ey = std::sin(alpha);
		double cd = 0.0;
		for (int i = 1; i < n; ++i) {
			const double ds = sf.s[i] - sf.s[i - 1];
			if (ds <= 0.0) continue;
			const double sm = 0.5 * (sf.s[i] + sf.s[i - 1]);
			const double qm = 0.5 * (sf.q[i] + sf.q[i - 1]);
			const double re_s = reynolds * qm * sm;
			if (re_s <= 0.0) continue;
			const double cf = 0.0592 * std::pow(re_s, -0.2);
			const double tx = (sf.x[i] - sf.x[i - 1]) / ds, ty = (sf.y[i] - sf.y[i - 1]) / ds;
			cd += cf * qm * qm * (tx * ex + ty * ey) * ds;
		}
		sf.cd_friction = cd;
	}

	// ---- Stratford (1959): C̄p = 1 - (V/V_max)^2 from the peak, x from the
	// equivalent flat-plate origin, x_EL = ∫ (V/V_max)^3 ds over the
	// favourable run; S = C̄p (x dC̄p/dx)^1/2 (1e-6 Re_x)^-0.1 against 0.39
	// (d2p/dx2 >= 0) or 0.35 (< 0), valid while C̄p <= 4/7.
	{
		double x_el = 0.0;
		for (int i = 1; i <= ip; ++i) {
			const double ds = sf.s[i] - sf.s[i - 1];
			const double r0 = sf.q[i - 1] / q_max, r1 = sf.q[i] / q_max;
			x_el += 0.5 * (r0 * r0 * r0 + r1 * r1 * r1) * ds;
		}
		sf.cf_integral_x_equiv = x_el;
		const int m = n - ip;
		if (m < 3 || !(q_max > 0.0)) { sf.separation.note = "no pressure recovery"; return; }
		std::vector<double> xx(m), cpb(m);
		for (int k = 0; k < m; ++k) {
			xx[k] = x_el + (sf.s[ip + k] - sf.s[ip]);
			const double rq = sf.q[ip + k] / q_max;
			cpb[k] = 1.0 - rq * rq;
		}
		auto d1 = [&](int k) {
			const int a = std::max(0, k - 1), b = std::min(m - 1, k + 1);
			return (cpb[b] - cpb[a]) / (xx[b] - xx[a]);
		};
		double s_prev = 0.0;
		for (int k = 1; k < m; ++k) {
			const double cp_k = cpb[k];
			if (cp_k > 4.0 / 7.0) {
				sf.stratford_range_end_x = sf.x[ip + k];
				sf.separation.note = sf.separation.found ? sf.separation.note
					: "no separation before C̄p exceeded 4/7 at x/c = " + std::to_string(sf.x[ip + k]) + " (criterion no longer valid)";
				break;
			}
			const double dcp = d1(k);
			const double re_x = reynolds * q_max * xx[k];
			double S = 0.0;
			if (dcp > 0.0 && re_x > 0.0 && cp_k > 0.0) S = cp_k * std::sqrt(xx[k] * dcp) * std::pow(1e-6 * re_x, -0.1);
			sf.stratford[ip + k] = S;
			const double d2 = (d1(std::min(m - 1, k + 1)) - d1(std::max(0, k - 1))) / (xx[std::min(m - 1, k + 1)] - xx[std::max(0, k - 1)]);
			const double limit = d2 >= 0.0 ? 0.39 : 0.35;
			if (!sf.separation.found && S >= limit) {
				// interpolate the crossing between k-1 and k
				const double t = (S - s_prev) > 0.0 ? (limit - s_prev) / (S - s_prev) : 1.0;
				const int a = ip + k - 1, b = ip + k;
				sf.separation.found = true;
				sf.separation.x = sf.x[a] + t * (sf.x[b] - sf.x[a]);
				sf.separation.y = sf.y[a] + t * (sf.y[b] - sf.y[a]);
				sf.separation.s = sf.s[a] + t * (sf.s[b] - sf.s[a]);
				sf.separation.note = "Stratford S reached " + std::to_string(limit) + " (turbulent from stagnation)";
			}
			s_prev = S;
		}
		if (!sf.separation.found && sf.separation.note.empty()) sf.separation.note = "Stratford's criterion not reached before the trailing edge";
	}
}

SectionAeroResult SolveSection(const SectionAeroMethod& method, const SectionAeroRequest& request) {
	SectionAeroResult r = method.Solve(request);
	if (!r.ok || method.Viscous()) return r; // a boundary-layer method carries its own cues
	AddBoundaryLayerCues(r.upper, request.reynolds, request.alpha_deg, request.transition_re_x);
	AddBoundaryLayerCues(r.lower, request.reynolds, request.alpha_deg, request.transition_re_x);
	r.cd_friction = r.upper.cd_friction + r.lower.cd_friction;
	return r;
}

// ---------------------------------------------------------------------------

const char* SectionAeroScalarName(SectionAeroScalar s) {
	switch (s) {
	case SectionAeroScalar::Cl: return "Cl";
	case SectionAeroScalar::CmC4: return "Cm c/4";
	case SectionAeroScalar::CdFriction: return "Cd friction";
	case SectionAeroScalar::CpUpperAtX: return "Cp upper at x/c";
	case SectionAeroScalar::CpLowerAtX: return "Cp lower at x/c";
	}
	return "?";
}

double SectionAeroScalarValue(const SectionAeroResult& r, SectionAeroScalar scalar, double probe_x) {
	switch (scalar) {
	case SectionAeroScalar::Cl: return r.cl;
	case SectionAeroScalar::CmC4: return r.cm_c4;
	case SectionAeroScalar::CdFriction: return r.cd_friction;
	case SectionAeroScalar::CpUpperAtX: return InterpolateAlong(r.upper.x, r.upper.cp, probe_x);
	case SectionAeroScalar::CpLowerAtX: return InterpolateAlong(r.lower.x, r.lower.cp, probe_x);
	}
	return kNaN;
}

PoleSensitivity ComputePoleSensitivity(const NurbsSpec& curve, const SectionAeroMethod& method,
	const SectionAeroRequest& request, SectionAeroScalar scalar, double probe_x, double step) {
	PoleSensitivity out;
	const int n_per_surface = std::max(50, request.contour.rows() / 2);
	auto evaluate = [&](const NurbsSpec& spec, double& value, std::string& err) {
		try {
			std::unique_ptr<Airfoil> af = Airfoil::FromCurve(spec, "sensitivity");
			SectionAeroRequest req = request;
			req.contour = af->ContourPoints(2 * n_per_surface);
			const SectionAeroResult r = SolveSection(method, req);
			if (!r.ok) { err = r.error; return false; }
			value = SectionAeroScalarValue(r, scalar, probe_x);
			return std::isfinite(value);
		} catch (const std::exception& e) {
			err = e.what();
			return false;
		}
	};
	std::string err;
	if (!evaluate(curve, out.base, err)) { out.error = err.empty() ? "scalar not available at the probe" : err; return out; }
	const int n = curve.poles.rows();
	out.d_dx.assign(n, kNaN);
	out.d_dy.assign(n, kNaN);
	int failed = 0;
	for (int i = 0; i < n; ++i) {
		for (int c = 0; c < 2; ++c) {
			NurbsSpec plus = curve, minus = curve;
			plus.poles(i, c) += step;
			minus.poles(i, c) -= step;
			double vp, vm;
			// A pole whose perturbation the method cannot analyse (typically a
			// trailing-edge pole moved so the section no longer closes cleanly)
			// is reported as NaN rather than failing the whole table.
			if (!evaluate(plus, vp, err) || !evaluate(minus, vm, err)) { ++failed; out.error = "pole " + std::to_string(i) + ": " + err; continue; }
			(c == 0 ? out.d_dx : out.d_dy)[i] = (vp - vm) / (2.0 * step);
		}
	}
	out.ok = failed < 2 * n;
	if (out.ok && failed == 0) out.error.clear();
	return out;
}

} // namespace aerotelic
