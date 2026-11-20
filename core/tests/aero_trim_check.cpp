// Stage 4 of the flight-loads plan (aero_trim.hpp, aero_loads.hpp): the
// rigid trim recovers the state of a linear model exactly; the pressure
// mapping carries a panel field onto a finer surface mesh on the right side
// of thin and two-sided surfaces and converges with the panel count.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "aerotelic/aero_loads.hpp"
#include "aerotelic/aero_trim.hpp"

using namespace aerotelic;
using namespace aerotelic::aero;

namespace {

int g_pass = 0, g_fail = 0;
constexpr double kPi = 3.14159265358979323846;

void Check(bool ok, const std::string& what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what.c_str());
	if (ok) ++g_pass; else ++g_fail;
}

std::string F(const char* fmt, double a, double b = 0.0, double c = 0.0) {
	char buf[240];
	std::snprintf(buf, sizeof buf, fmt, a, b, c);
	return buf;
}

// A linear aircraft: CL = a0 + a alpha + b delta, Cm = m0 + c alpha + d delta.
struct Linear {
	double a0, a, b, m0, c, d;
	TrimPoint At(double alpha, double delta) const { return {alpha, delta, a0 + a * alpha + b * delta, m0 + c * alpha + d * delta}; }
};

void TrimChecks() {
	const Linear L{0.1, 0.09, 0.03, 0.02, -0.015, -0.04};
	const double target = 0.6;
	const TrimPoint base = L.At(2.0, 0.0);
	const TrimResult r = SolveTrim(base, L.At(3.0, 0.0), L.At(2.0, 1.0), target);
	Check(r.ok, "trim: the 2 x 2 system solves");
	const TrimPoint at = L.At(r.alpha_deg, r.delta_deg);
	Check(std::fabs(at.CL - target) < 1e-12 && std::fabs(at.Cm) < 1e-12, F("trim: the trimmed state has CL = target and Cm = 0 (CL %.6f, Cm %.2e)", at.CL, at.Cm));
	// By hand: a da + b dd = 0.6 - 0.28 = 0.32; c da + d dd = -(0.02 - 0.03) = 0.01.
	const double det = L.a * L.d - L.b * L.c;
	const double da = (0.32 * L.d - L.b * 0.01) / det, dd = (L.a * 0.01 - 0.32 * L.c) / det;
	Check(std::fabs(r.alpha_deg - (2.0 + da)) < 1e-9 && std::fabs(r.delta_deg - dd) < 1e-9, F("trim: alpha %.4f, delta %.4f deg as by hand", r.alpha_deg, r.delta_deg));
	Check(std::fabs(r.dCL_dalpha - L.a) < 1e-12 && std::fabs(r.dCm_ddelta - L.d) < 1e-12 && std::fabs(r.determinant - det) < 1e-12, "trim: the derivatives and determinant are the model's");

	// Singular: the control does exactly what the angle of attack does.
	const Linear S{0.1, 0.09, 0.045, 0.02, -0.015, -0.0075};
	const TrimResult rs = SolveTrim(S.At(0, 0), S.At(1, 0), S.At(0, 1), 0.5);
	Check(!rs.ok && !rs.note.empty(), "trim: a control acting like the angle of attack is refused as singular");
	Check(!SolveTrim(base, base, L.At(2.0, 1.0), target).ok, "trim: a zero step is refused");

	// Alpha alone.
	const TrimResult ra = SolveTrimAlpha(base, L.At(3.0, 0.0), target);
	Check(ra.ok && std::fabs(L.At(ra.alpha_deg, 0.0).CL - target) < 1e-12 && ra.delta_deg == 0.0, F("trim (alpha alone): CL = target at alpha %.4f deg, the control untouched", ra.alpha_deg));

	// The required lift coefficient and the dynamic pressure.
	const double q = DynamicPressure(1.225, 20.0);
	Check(std::fabs(q - 245.0) < 1e-9, F("q = rho V^2 / 2 = %.1f Pa", q));
	Check(std::fabs(RequiredLiftCoefficient(2.0, 50.0, q, 0.5) - 100.0 / 122.5) < 1e-12, "CL required = n W / (q S)");
	Check(RequiredLiftCoefficient(1.0, 50.0, 0.0, 0.5) == 0.0, "CL required is 0 at q = 0 rather than infinite");
}

// A NONLINEAR aircraft: the lift curve bends over (the term in alpha^2) and
// the control's moment depends on the angle of attack. That is what makes a
// single Newton step wrong, and it is why the trim iterates -- my_drone's
// Case 1 was stored from one step at C_L -0.60 against the 1.24 it needed.
struct Nonlinear {
	double a0 = 0.1, a = 0.09, b = 0.03, k = 0.0008;
	double m0 = 0.02, c = -0.015, d = -0.04, e = 0.0005;
	TrimPoint At(double alpha, double delta) const {
		return {alpha, delta,
			a0 + a * alpha + b * delta - k * alpha * alpha,
			m0 + c * alpha + d * delta + e * alpha * delta};
	}
};

// Drives a TrimDriver to Done or Failed against a model, counting the solves
// it asked for -- exactly the loop the application runs asynchronously.
template <class Model>
int DriveTrim(TrimDriver& D, const Model& M, int solve_budget = 80) {
	int solves = 0;
	while (solves < solve_budget) {
		const TrimDriver::Ask ask = D.Next();
		if (ask == TrimDriver::Ask::Done || ask == TrimDriver::Ask::Failed) break;
		if (ask == TrimDriver::Ask::Derivatives) {
			const TrimPoint bs = D.BaseState(), as = D.StepAlphaState(), ds = D.StepDeltaState();
			const TrimPoint base = M.At(bs.alpha_deg, bs.delta_deg);
			if (!D.BaseAlreadySolved()) ++solves;
			const TrimPoint alpha_step = M.At(as.alpha_deg, as.delta_deg);
			++solves;
			TrimPoint delta_step = base;
			if (D.settings().control) {
				delta_step = M.At(ds.alpha_deg, ds.delta_deg);
				++solves;
			}
			D.FeedDerivatives(base, alpha_step, delta_step);
		} else {
			const TrimPoint ns = D.NextState();
			const TrimPoint at = M.At(ns.alpha_deg, ns.delta_deg);
			++solves;
			D.FeedState(at);
		}
	}
	return solves;
}

void IterationChecks() {
	const Nonlinear N;
	const double target = 1.24;   // my_drone Case 1's required lift coefficient

	// The tolerance arithmetic.
	TrimTolerance tol;
	Check(std::fabs(LiftTolerance(target, tol) - 0.005 * target) < 1e-12, F("tolerance: 0.5%% of C_L = %.4f", LiftTolerance(target, tol)));
	Check(std::fabs(LiftTolerance(0.0, tol) - tol.CL_floor) < 1e-12, "tolerance: the floor holds when the condition needs no lift");
	{
		const TrimPoint at{5.0, 0.0, target + LiftTolerance(target, tol), 0.0};
		Check(std::fabs(ResidualNorm(at, target, true, tol) - 1.0) < 1e-9, "residual: exactly one tolerance out reads a norm of 1");
		Check(Converged(at, target, true, tol), "residual: one tolerance out counts as trimmed");
		const TrimPoint off{5.0, 0.0, target, 2.0 * tol.Cm};
		Check(!Converged(off, target, true, tol) && std::fabs(ResidualNorm(off, target, true, tol) - 2.0) < 1e-9, "residual: the moment is in the norm too (2 tolerances out)");
		Check(Converged(off, target, false, tol), "residual: with no control the moment is not driven, so that state is trimmed on lift alone");
	}

	// ONE STEP IS NOT A TRIM: the single Newton step the trim used to take.
	const TrimPoint base = N.At(2.0, 0.0);
	const TrimResult one = SolveTrim(base, N.At(3.0, 0.0), N.At(2.0, 1.0), target);
	Check(one.ok, "one step: the 2 x 2 system solves");
	const TrimPoint landed = N.At(one.alpha_deg, one.delta_deg);
	const double one_err = LiftErrorFraction(landed, target);
	std::printf("one Newton step from alpha 2: alpha %.3f, delta %.3f -> C_L %.4f against %.4f (%.1f%% off), C_m %.4f, residual norm %.1f\n",
		one.alpha_deg, one.delta_deg, landed.CL, target, 100.0 * one_err, landed.Cm, ResidualNorm(landed, target, true, tol));
	Check(one_err < -0.04 && one_err > -0.25, F("one step: the state it lands on is %.1f%% under the lift it needs -- not a trim", 100.0 * one_err));
	Check(!Converged(landed, target, true, tol), "one step: that state does not pass the convergence test");

	// THE ITERATION reaches it.
	{
		TrimDriver::Settings s;
		s.CL_target = target;
		s.control = true;
		TrimDriver D(s, 2.0, 0.0);
		const int solves = DriveTrim(D, N);
		const TrimPoint& bestp = D.Best();
		std::printf("iterated trim: %d solves, %d corrections, %d Jacobians -> alpha %.3f, delta %.3f, C_L %.5f, C_m %.6f, norm %.3f\n",
			solves, D.Iterations(), D.Jacobians(), bestp.alpha_deg, bestp.delta_deg, bestp.CL, bestp.Cm, D.BestNorm());
		Check(D.Next() == TrimDriver::Ask::Done, "iterated trim: the driver finishes");
		Check(D.Trimmed(), F("iterated trim: converged (best residual norm %.3f)", D.BestNorm()));
		Check(std::fabs(bestp.CL - target) <= LiftTolerance(target, tol) && std::fabs(bestp.Cm) <= tol.Cm,
			F("iterated trim: C_L %.5f within tolerance of the target and C_m %.6f driven to zero", bestp.CL, bestp.Cm));
		Check(std::fabs(LiftErrorFraction(bestp, target)) < 0.006, F("iterated trim: the lift is %.3f%% off, against %.1f%% from the single step", 100.0 * LiftErrorFraction(bestp, target), 100.0 * one_err));
		Check(solves <= 20 && D.Iterations() <= 8, F("iterated trim: %g solves in %g corrections", static_cast<double>(solves), static_cast<double>(D.Iterations())));
	}

	// The same without the trust region (the pure chord iteration on one
	// Jacobian measurement plus re-measurements when it stalls).
	{
		TrimDriver::Settings s;
		s.CL_target = target;
		s.control = true;
		s.max_step_deg = 0.0;
		TrimDriver D(s, 2.0, 0.0);
		const int solves = DriveTrim(D, N);
		std::printf("iterated trim, no trust region: %d solves, %d corrections, %d Jacobians, norm %.3f\n", solves, D.Iterations(), D.Jacobians(), D.BestNorm());
		Check(D.Trimmed(), F("iterated trim without the trust region: converged too (norm %.3f)", D.BestNorm()));
	}

	// The trust region holds a single correction inside its band.
	{
		TrimDriver::Settings s;
		s.CL_target = target;
		s.control = true;
		s.max_step_deg = 4.0;
		s.jacobian_move_deg = 0.0;      // no re-measurement: the clamp alone
		s.stall_ratio = 0.0;
		TrimDriver D(s, 2.0, 0.0);
		D.FeedDerivatives(N.At(2.0, 0.0), N.At(3.0, 0.0), N.At(2.0, 1.0));
		Check(D.Next() == TrimDriver::Ask::State && std::fabs(D.NextState().alpha_deg - 2.0) <= 4.0 + 1e-9 && D.LastStepClamped(),
			F("trust region: the first correction moves alpha to %.3f, not past 4 degrees, and says it was clamped", D.NextState().alpha_deg));
		const int solves = DriveTrim(D, N);
		Check(D.Trimmed(), F("trust region: clamped corrections still converge (norm %.3f, %g more solves)", D.BestNorm(), static_cast<double>(solves)));
	}

	// OUT OF BUDGET is not a trim: one correction only, which is what the
	// application did before 2026-11-19. Done, but not Trimmed -- the caller
	// must refuse to store it.
	{
		TrimDriver::Settings s;
		s.CL_target = target;
		s.control = true;
		s.max_step_deg = 0.0;
		s.jacobian_move_deg = 0.0;
		s.stall_ratio = 0.0;
		s.max_iterations = 1;
		TrimDriver D(s, 2.0, 0.0);
		DriveTrim(D, N);
		Check(D.Next() == TrimDriver::Ask::Done && !D.Trimmed(), "budget: one correction finishes the run but does NOT count as trimmed");
		Check(std::fabs(D.BestLiftErrorFraction() - one_err) < 1e-9, F("budget: it reports the single step's %.1f%% lift error for the caller to refuse", 100.0 * D.BestLiftErrorFraction()));
		Check(!D.Note().empty(), "budget: it says it did not converge");
	}

	// A condition the configuration cannot reach is refused by the band.
	{
		TrimDriver::Settings s;
		s.CL_target = 5.0;
		s.control = true;
		TrimDriver D(s, 2.0, 0.0);
		DriveTrim(D, N);
		Check(D.Next() == TrimDriver::Ask::Failed && D.Note().find("potential flow") != std::string::npos,
			"band: a lift coefficient of 5 asks for an angle outside the band and is refused");
	}

	// The angle of attack alone: lift driven, moment left alone.
	{
		TrimDriver::Settings s;
		s.CL_target = 0.9;
		s.control = false;
		TrimDriver D(s, 0.0, 0.0);
		const int solves = DriveTrim(D, N);
		Check(D.Trimmed() && std::fabs(D.Best().CL - 0.9) <= LiftTolerance(0.9, tol) && D.Best().delta_deg == 0.0,
			F("alpha alone: C_L %.5f at alpha %.3f, the control untouched", D.Best().CL, D.Best().alpha_deg));
		Check(solves <= 12, F("alpha alone: %g solves", static_cast<double>(solves)));
	}

	// A solve the caller rejects (a broken panel solution) abandons the run.
	{
		TrimDriver::Settings s;
		s.CL_target = target;
		s.control = true;
		TrimDriver D(s, 2.0, 0.0);
		D.Fail("the pressures and the wake circulation disagree");
		Check(D.Next() == TrimDriver::Ask::Failed && !D.Note().empty(), "a rejected solve abandons the run with the caller's reason");
	}
}

// A sphere of radius 1: panel centres on a lat-long grid carrying the
// potential-flow pressure 1 - 9/4 sin^2(theta) from the x axis; the faces
// are a finer triangulation's centroids.
void SphereSamples(int rows, int cols, std::vector<PressureSample>& out) {
	out.clear();
	for (int i = 0; i < rows; ++i) {
		const double th = kPi * (i + 0.5) / rows; // from +x
		for (int j = 0; j < cols; ++j) {
			const double ph = 2.0 * kPi * (j + 0.5) / cols;
			PressureSample s;
			s.normal = {std::cos(th), std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph)};
			s.centre = s.normal;
			s.area = (kPi / rows) * (2.0 * kPi / cols) * std::sin(th);
			s.cp_upper = 1.0 - 2.25 * std::sin(th) * std::sin(th);
			s.network = 0;
			s.row = i;
			s.col = j;
			out.push_back(s);
		}
	}
}

double SphereRms(int rows, int cols, int face_rows, int face_cols, MapStats* stats = nullptr) {
	std::vector<PressureSample> samples;
	SphereSamples(rows, cols, samples);
	std::vector<Vec3> centres, normals;
	std::vector<double> exact;
	for (int i = 0; i < face_rows; ++i) {
		const double th = kPi * (i + 0.5) / face_rows;
		for (int j = 0; j < face_cols; ++j) {
			const double ph = 2.0 * kPi * (j + 0.5) / face_cols;
			const Vec3 n{std::cos(th), std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph)};
			centres.push_back(n);
			normals.push_back(n);
			exact.push_back(1.0 - 2.25 * std::sin(th) * std::sin(th));
		}
	}
	const std::vector<double> cp = MapPressures(samples, centres, normals, {}, stats);
	double s2 = 0.0;
	for (size_t k = 0; k < cp.size(); ++k) s2 += (cp[k] - exact[k]) * (cp[k] - exact[k]);
	return std::sqrt(s2 / cp.size());
}

void MappingChecks() {
	MapStats st;
	const double rms_coarse = SphereRms(8, 16, 40, 80, &st);
	Check(st.faces == 3200 && st.mapped == 3200 && st.unmapped == 0, F("mapping: every one of %g sphere faces got a pressure", st.faces));
	// 22-degree panels on a field that varies as sin^2: a tenth of the
	// span of Cp, mostly the inverse-distance blend across the block.
	Check(rms_coarse < 0.12, F("mapping: 8 x 16 panels onto 40 x 80 faces, rms error %.4f (Cp spans -1.25..1)", rms_coarse));
	const double rms_fine = SphereRms(24, 48, 40, 80);
	Check(rms_fine < 0.4 * rms_coarse, F("mapping: three times the panels, the error falls %.4f -> %.4f", rms_coarse, rms_fine));
	// The farthest face from its panel is about half a panel's diagonal
	// (a 22 x 22 degree panel on a unit sphere is 0.39 across).
	Check(st.max_distance < 0.3 && st.mean_distance < 0.15, F("mapping: faces found panels within %.3f (mean %.3f) of a unit sphere", st.max_distance, st.mean_distance));

	// A thin slab: upper panels at z = +h facing up with Cp -1, lower at
	// z = -h facing down with Cp +0.5, h a tenth of the panel pitch. A
	// lower face offset sideways by a third of a pitch is nearer an upper
	// panel in distance; its side rule must still give it the lower value.
	{
		std::vector<PressureSample> s;
		const double h = 0.01, pitch = 0.1;
		for (int i = 0; i < 10; ++i) {
			for (int j = 0; j < 10; ++j) {
				PressureSample u;
				u.centre = {(i + 0.5) * pitch, (j + 0.5) * pitch, h};
				u.normal = {0, 0, 1};
				u.area = pitch * pitch;
				u.cp_upper = -1.0;
				u.network = 0; u.row = i; u.col = j;
				s.push_back(u);
				PressureSample l = u;
				l.centre.z = -h;
				l.normal = {0, 0, -1};
				l.cp_upper = 0.5;
				l.network = 1;
				s.push_back(l);
			}
		}
		std::vector<Vec3> c = {{0.5 + pitch / 3.0, 0.5, -h}, {0.5 + pitch / 3.0, 0.5, h}, {0.52, 0.47, -h}};
		std::vector<Vec3> n = {{0, 0, -1}, {0, 0, 1}, {0, 0, -1}};
		MapStats ts;
		const std::vector<double> cp = MapPressures(s, c, n, {}, &ts);
		Check(ts.mapped == 3 && std::fabs(cp[0] - 0.5) < 1e-12 && std::fabs(cp[1] + 1.0) < 1e-12 && std::fabs(cp[2] - 0.5) < 1e-12,
			F("mapping: a thin slab's lower faces take the lower panels' pressure (%.3f, %.3f, %.3f)", cp[0], cp[1], cp[2]));
		// A face far from everything gets nothing.
		const std::vector<double> far = MapPressures(s, {{5.0, 5.0, 0.0}}, {{0, 0, 1}}, {}, &ts);
		Check(ts.unmapped == 1 && far[0] == 0.0, "mapping: a face far from every panel is left unloaded and counted");
	}

	// A two-sided sheet at z = 0 facing +z: faces facing up take the upper
	// pressure, faces facing down the lower one.
	{
		std::vector<PressureSample> s;
		for (int i = 0; i < 5; ++i) {
			for (int j = 0; j < 5; ++j) {
				PressureSample p;
				p.centre = {(i + 0.5) * 0.2, (j + 0.5) * 0.2, 0.0};
				p.normal = {0, 0, 1};
				p.area = 0.04;
				p.cp_upper = -1.0;
				p.cp_lower = 0.3;
				p.two_sided = true;
				p.network = 0; p.row = i; p.col = j;
				s.push_back(p);
			}
		}
		const std::vector<double> cp = MapPressures(s, {{0.5, 0.5, 0.001}, {0.5, 0.5, -0.001}}, {{0, 0, 1}, {0, 0, -1}});
		Check(std::fabs(cp[0] + 1.0) < 1e-12 && std::fabs(cp[1] - 0.3) < 1e-12, F("mapping: a two-sided sheet gives %.2f upward and %.2f downward", cp[0], cp[1]));
	}

	// A linear field is reproduced within the blend of neighbours: Cp = x
	// on a flat grid, faces between panel centres read between them.
	{
		std::vector<PressureSample> s;
		for (int i = 0; i < 20; ++i) {
			for (int j = 0; j < 4; ++j) {
				PressureSample p;
				p.centre = {(i + 0.5) * 0.05, (j + 0.5) * 0.05, 0.0};
				p.normal = {0, 0, 1};
				p.area = 0.0025;
				p.cp_upper = p.centre.x;
				p.row = i; p.col = j;
				s.push_back(p);
			}
		}
		std::vector<Vec3> c, n;
		std::vector<double> want;
		for (int k = 0; k < 50; ++k) {
			c.push_back({0.2 + 0.6 * k / 49.0, 0.1, 0.0});
			n.push_back({0, 0, 1});
			want.push_back(c.back().x);
		}
		const std::vector<double> cp = MapPressures(s, c, n);
		double worst = 0.0;
		for (size_t k = 0; k < cp.size(); ++k) worst = std::max(worst, std::fabs(cp[k] - want[k]));
		Check(worst < 0.012, F("mapping: a linear field over a 0.05 pitch is reproduced within %.4f", worst));
	}
}

} // namespace

int main() {
	TrimChecks();
	IterationChecks();
	MappingChecks();
	std::printf("RESULT: %d passed, %d failed\n", g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}
