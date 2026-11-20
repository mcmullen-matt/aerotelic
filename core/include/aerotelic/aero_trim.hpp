#pragma once

// Rigid trim of a configuration on a linear aerodynamic model (stage 4 of
// the flight-loads plan, 2026-11-14): the angle of attack and one control
// deflection that give the lift a flight condition needs and no pitching
// moment about the mass centre.
//
// The panel method is linear in the free stream's direction only to first
// order (the wake sheds along the stream and a flap kinks the section), so
// the trim is a Newton iteration on finite-difference derivatives: the base
// solution, one with the angle of attack stepped, one with the control
// stepped -- three linear solves -- give the 2 x 2 system
//
//   [ dCL/da  dCL/dd ] [ da ]   [ CL_target - CL0 ]
//   [ dCm/da  dCm/dd ] [ dd ] = [       0 - Cm0   ]
//
// whose solution is the next state. With no control, the one equation
// CL = CL_target in the angle of attack alone, the moment reported
// untrimmed.
//
// ONE SUCH STEP IS NOT A TRIM (2026-11-19). The step lands where the LINEAR
// model is trimmed; the configuration there has a different CL and Cm, and
// over a step of fifteen degrees -- which is what a low-speed, high load
// factor corner of the V-n diagram asks for -- the difference is not small.
// my_drone's Case 1 was stored from a single step at C_L -0.60 against the
// 1.24 it needed, and a thickness optimisation was run on it. So the caller
// ITERATES: StepFrom corrects the state it is given with a Jacobian it keeps
// (the chord method: no new derivatives per iteration, one solve each), and
// ResidualNorm says when to stop. The Jacobian is re-taken when the state
// has wandered far from where it was measured or when the residual stops
// falling; `max_step_deg` keeps a single correction inside the band where
// the derivatives mean anything. The iterate to believe is the one with the
// smallest residual, not the last.
//
// Cm here is the pitching moment coefficient ABOUT THE MASS CENTRE (set
// panel::Options::moment_reference to it), positive nose up in the usual
// convention or whatever the caller's -- the trim only drives it to zero.

#include <string>

namespace aerotelic::aero {

// One solved operating point.
struct TrimPoint {
	double alpha_deg = 0.0;
	double delta_deg = 0.0;
	double CL = 0.0;
	double Cm = 0.0;
};

struct TrimResult {
	bool ok = false;
	double alpha_deg = 0.0;
	double delta_deg = 0.0;
	// The derivatives the step used, per degree.
	double dCL_dalpha = 0.0, dCL_ddelta = 0.0, dCm_dalpha = 0.0, dCm_ddelta = 0.0;
	// The determinant of the 2 x 2 (per degree squared); near zero the
	// control and the angle of attack do the same thing and no trim exists.
	double determinant = 0.0;
	std::string note;   // why not, when !ok
};

// The 2 x 2 trim: `alpha_step` differs from `base` in alpha only,
// `delta_step` in delta only. Fails on a zero step or a singular system.
// ONE step from `base` -- see the file header on why that is not a trim;
// JacobianFrom + StepFrom are the iterating form, and this is them called
// once.
TrimResult SolveTrim(const TrimPoint& base, const TrimPoint& alpha_step, const TrimPoint& delta_step, double CL_target);

// Angle of attack alone: CL = CL_target by dCL/dalpha; delta stays the
// base's and Cm is not driven.
TrimResult SolveTrimAlpha(const TrimPoint& base, const TrimPoint& alpha_step, double CL_target);

// ---- the iterating form ----

// The linearisation of the configuration about a state, per degree. With no
// control (`delta_step` null) only the alpha column is measured and the
// moment is not driven.
struct TrimJacobian {
	bool ok = false;
	bool control = false;
	double dCL_dalpha = 0.0, dCL_ddelta = 0.0, dCm_dalpha = 0.0, dCm_ddelta = 0.0;
	double determinant = 0.0;
	// Where it was measured, so the caller can tell when the state has
	// wandered too far from it.
	double alpha_deg = 0.0, delta_deg = 0.0;
	std::string note;
};
TrimJacobian JacobianFrom(const TrimPoint& base, const TrimPoint& alpha_step, const TrimPoint* delta_step);

// How far a state is from the flight condition, in units of the tolerance:
// the larger of the lift and moment residuals so scaled. Converged when it
// is not above 1. The lift tolerance is a fraction of the required
// coefficient, with a floor for conditions that need almost no lift; the
// moment's is absolute (and ignored with no control, which cannot trim it).
struct TrimTolerance {
	double CL_fraction = 0.005;
	double CL_floor = 0.002;
	double Cm = 0.002;
};
// The absolute lift tolerance the fraction and the floor come to.
double LiftTolerance(double CL_target, const TrimTolerance& tol);
double ResidualNorm(const TrimPoint& at, double CL_target, bool control, const TrimTolerance& tol);
inline bool Converged(const TrimPoint& at, double CL_target, bool control, const TrimTolerance& tol) {
	return ResidualNorm(at, CL_target, control, tol) <= 1.0;
}

// One correction of `at` by the kept Jacobian: the state the linear model
// says is trimmed. `max_step_deg` (not positive: unlimited) scales the
// correction down so that neither component exceeds it, keeping the Newton
// direction, and reports that it did.
struct TrimStep {
	bool ok = false;
	double alpha_deg = 0.0, delta_deg = 0.0;
	double d_alpha = 0.0, d_delta = 0.0;
	bool clamped = false;
	std::string note;
};
TrimStep StepFrom(const TrimPoint& at, const TrimJacobian& J, double CL_target, double max_step_deg = 0.0);

// How far a state's lift is from the condition, as a fraction of what the
// condition needs (the figure to put in front of the user: "48% under the
// lift it needs"). Zero target: the absolute miss.
double LiftErrorFraction(const TrimPoint& at, double CL_target);

// The iteration itself, as a plain object the caller DRIVES rather than a
// loop with a callback: Next() says what wants solving, Feed...() hands back
// what came out. A caller that meshes on one thread and solves on another
// drives it exactly as the tests drive it synchronously.
class TrimDriver {
public:
	struct Settings {
		double CL_target = 0.0;
		bool control = false;            // a 2 x 2 trim; else the angle of attack alone
		TrimTolerance tolerance;
		double step = 1.0;               // the derivative step, degrees
		double max_step_deg = 8.0;       // trust region on one correction (0: unlimited)
		double jacobian_move_deg = 5.0;  // re-measure once the state has moved this far from it
		double stall_ratio = 1.5;        // ... or when the residual falls by less than this factor
		int max_iterations = 8;          // corrections solved after the first state
		int max_jacobians = 4;           // times the derivatives may be measured
		double alpha_limit = 25.0, delta_limit = 45.0;  // outside this no potential flow describes it
	};
	enum class Ask {
		Derivatives,  // solve BaseState() (unless BaseAlreadySolved()), StepAlphaState() and, with a control, StepDeltaState(); then FeedDerivatives
		State,        // solve NextState(); then FeedState
		Done,         // Trimmed() says whether it converged; Best() is the iterate to use
		Failed        // Note() says why
	};

	TrimDriver(const Settings& settings, double alpha0, double delta0);

	Ask Next() const { return ask_; }
	const Settings& settings() const { return settings_; }
	// The states Ask::Derivatives wants (CL/Cm unset).
	TrimPoint BaseState() const { return {alpha_, delta_, 0.0, 0.0}; }
	TrimPoint StepAlphaState() const { return {alpha_ + settings_.step, delta_, 0.0, 0.0}; }
	TrimPoint StepDeltaState() const { return {alpha_, delta_ + settings_.step, 0.0, 0.0}; }
	// True when the base of a re-measured Jacobian is the state just solved,
	// so only the stepped states need solving.
	bool BaseAlreadySolved() const { return !first_derivatives_; }
	// The state Ask::State wants solved.
	TrimPoint NextState() const { return {alpha_, delta_, 0.0, 0.0}; }

	// `delta_step` is ignored without a control.
	void FeedDerivatives(const TrimPoint& base, const TrimPoint& alpha_step, const TrimPoint& delta_step);
	void FeedState(const TrimPoint& at);
	// Abandons the run with a reason (a solve that failed or was refused).
	void Fail(const std::string& why);

	// The iterate with the smallest residual -- the one to believe, which is
	// not always the last (the chord iteration can step past it).
	const TrimPoint& Best() const { return best_; }
	double BestNorm() const { return best_norm_; }
	bool HasBest() const { return has_best_; }
	// Converged: the best iterate is inside the tolerance.
	bool Trimmed() const { return has_best_ && best_norm_ <= 1.0; }
	double BestLiftErrorFraction() const { return has_best_ ? LiftErrorFraction(best_, settings_.CL_target) : 0.0; }
	int Iterations() const { return iterations_; }
	int Jacobians() const { return jacobians_; }
	bool LastStepClamped() const { return last_clamped_; }
	bool LastRefreshStalled() const { return last_stalled_; }
	const TrimJacobian& Jacobian() const { return jacobian_; }
	const std::string& Note() const { return note_; }

private:
	void Account(const TrimPoint& at);   // best tracking
	// Convergence, a re-measurement or the next correction. `allow_refresh`
	// is false straight after a measurement: the Jacobian there is fresh, and
	// the stall test at its own base point would always fire.
	void PlanAfter(const TrimPoint& at, bool allow_refresh);

	Settings settings_;
	Ask ask_ = Ask::Derivatives;
	double alpha_ = 0.0, delta_ = 0.0;
	TrimJacobian jacobian_;
	TrimPoint best_;
	double best_norm_ = 0.0, prev_norm_ = 0.0;
	bool has_best_ = false, first_derivatives_ = true;
	bool last_clamped_ = false, last_stalled_ = false;
	int iterations_ = 0, jacobians_ = 0;
	std::string note_;
};

// q = rho V^2 / 2 (SI in, Pa out).
inline double DynamicPressure(double density, double speed) { return 0.5 * density * speed * speed; }

// The lift coefficient a flight condition needs: n W / (q S), everything
// SI (weight in newtons, S in m^2). Zero when q S is not positive.
double RequiredLiftCoefficient(double load_factor, double weight, double dynamic_pressure, double reference_area);

} // namespace aerotelic::aero
