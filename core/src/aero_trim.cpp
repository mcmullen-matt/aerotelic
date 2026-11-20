#include "aerotelic/aero_trim.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace aerotelic::aero {

TrimJacobian JacobianFrom(const TrimPoint& base, const TrimPoint& alpha_step, const TrimPoint* delta_step) {
	TrimJacobian J;
	J.control = delta_step != nullptr;
	J.alpha_deg = base.alpha_deg;
	J.delta_deg = base.delta_deg;
	const double da = alpha_step.alpha_deg - base.alpha_deg;
	if (std::fabs(da) < 1e-12) {
		J.note = J.control ? "a derivative step is zero" : "the angle of attack step is zero";
		return J;
	}
	J.dCL_dalpha = (alpha_step.CL - base.CL) / da;
	J.dCm_dalpha = (alpha_step.Cm - base.Cm) / da;
	if (!J.control) {
		if (!(std::fabs(J.dCL_dalpha) > 1e-12)) {
			J.note = "the lift does not change with the angle of attack: no trim";
			return J;
		}
		J.determinant = J.dCL_dalpha;
		J.ok = true;
		return J;
	}
	const double dd = delta_step->delta_deg - base.delta_deg;
	if (std::fabs(dd) < 1e-12) {
		J.note = "a derivative step is zero";
		return J;
	}
	J.dCL_ddelta = (delta_step->CL - base.CL) / dd;
	J.dCm_ddelta = (delta_step->Cm - base.Cm) / dd;
	J.determinant = J.dCL_dalpha * J.dCm_ddelta - J.dCL_ddelta * J.dCm_dalpha;
	const double scale = std::fabs(J.dCL_dalpha * J.dCm_ddelta) + std::fabs(J.dCL_ddelta * J.dCm_dalpha);
	if (!(std::fabs(J.determinant) > 1e-9 * (scale > 0.0 ? scale : 1.0))) {
		J.note = "the control and the angle of attack act alike (singular trim system): no trim";
		return J;
	}
	J.ok = true;
	return J;
}

double LiftTolerance(double CL_target, const TrimTolerance& tol) {
	return std::max(std::fabs(tol.CL_fraction * CL_target), std::fabs(tol.CL_floor));
}

double ResidualNorm(const TrimPoint& at, double CL_target, bool control, const TrimTolerance& tol) {
	const double tL = LiftTolerance(CL_target, tol);
	double n = tL > 0.0 ? std::fabs(CL_target - at.CL) / tL : 0.0;
	if (control && tol.Cm > 0.0) n = std::max(n, std::fabs(at.Cm) / tol.Cm);
	return n;
}

TrimStep StepFrom(const TrimPoint& at, const TrimJacobian& J, double CL_target, double max_step_deg) {
	TrimStep s;
	s.alpha_deg = at.alpha_deg;
	s.delta_deg = at.delta_deg;
	if (!J.ok) {
		s.note = J.note.empty() ? "the trim system is not usable" : J.note;
		return s;
	}
	const double rL = CL_target - at.CL;
	if (J.control) {
		const double rM = -at.Cm;
		s.d_alpha = (rL * J.dCm_ddelta - J.dCL_ddelta * rM) / J.determinant;
		s.d_delta = (J.dCL_dalpha * rM - rL * J.dCm_dalpha) / J.determinant;
	} else {
		s.d_alpha = rL / J.dCL_dalpha;
		s.d_delta = 0.0;
	}
	if (!std::isfinite(s.d_alpha) || !std::isfinite(s.d_delta)) {
		s.note = "the trim step is not finite";
		return s;
	}
	if (max_step_deg > 0.0) {
		const double worst = std::max(std::fabs(s.d_alpha), std::fabs(s.d_delta));
		if (worst > max_step_deg) {
			const double f = max_step_deg / worst;
			s.d_alpha *= f;
			s.d_delta *= f;
			s.clamped = true;
		}
	}
	s.alpha_deg = at.alpha_deg + s.d_alpha;
	s.delta_deg = at.delta_deg + s.d_delta;
	s.ok = true;
	return s;
}

namespace {

// The one-step forms below report the Jacobian they used in the result.
TrimResult FromStep(const TrimJacobian& J, const TrimStep& s) {
	TrimResult r;
	r.dCL_dalpha = J.dCL_dalpha;
	r.dCL_ddelta = J.dCL_ddelta;
	r.dCm_dalpha = J.dCm_dalpha;
	r.dCm_ddelta = J.dCm_ddelta;
	r.determinant = J.determinant;
	if (!J.ok) {
		r.note = J.note;
		return r;
	}
	if (!s.ok) {
		r.note = s.note;
		return r;
	}
	r.alpha_deg = s.alpha_deg;
	r.delta_deg = s.delta_deg;
	r.ok = true;
	return r;
}

} // namespace

TrimResult SolveTrim(const TrimPoint& base, const TrimPoint& alpha_step, const TrimPoint& delta_step, double CL_target) {
	const TrimJacobian J = JacobianFrom(base, alpha_step, &delta_step);
	return FromStep(J, StepFrom(base, J, CL_target));
}

TrimResult SolveTrimAlpha(const TrimPoint& base, const TrimPoint& alpha_step, double CL_target) {
	const TrimJacobian J = JacobianFrom(base, alpha_step, nullptr);
	TrimResult r = FromStep(J, StepFrom(base, J, CL_target));
	if (r.ok) r.delta_deg = base.delta_deg;
	return r;
}

double LiftErrorFraction(const TrimPoint& at, double CL_target) {
	return std::fabs(CL_target) > 1e-12 ? (at.CL - CL_target) / CL_target : at.CL - CL_target;
}

// ---- the driver ----

TrimDriver::TrimDriver(const Settings& settings, double alpha0, double delta0)
	: settings_(settings), alpha_(alpha0), delta_(delta0) {
	if (settings_.step == 0.0) settings_.step = 1.0;
}

void TrimDriver::Fail(const std::string& why) {
	note_ = why;
	ask_ = Ask::Failed;
}

void TrimDriver::Account(const TrimPoint& at) {
	const double n = ResidualNorm(at, settings_.CL_target, settings_.control, settings_.tolerance);
	if (!has_best_ || n < best_norm_) {
		best_ = at;
		best_norm_ = n;
		has_best_ = true;
	}
}

void TrimDriver::PlanAfter(const TrimPoint& at, bool allow_refresh) {
	const double n = ResidualNorm(at, settings_.CL_target, settings_.control, settings_.tolerance);
	if (n <= 1.0) {
		ask_ = Ask::Done;
		return;
	}
	if (iterations_ >= settings_.max_iterations) {
		// Out of budget: not a failure of the method, an answer that did not
		// converge. The caller decides what to do with Best().
		note_ = "the trim did not converge in " + std::to_string(settings_.max_iterations) + " corrections";
		ask_ = Ask::Done;
		return;
	}
	// Re-measure the derivatives when the state has wandered from where they
	// were taken, or when the residual has stopped falling (the chord
	// iteration on a stale Jacobian).
	const bool moved = allow_refresh && settings_.jacobian_move_deg > 0.0
		&& (std::fabs(at.alpha_deg - jacobian_.alpha_deg) > settings_.jacobian_move_deg
			|| std::fabs(at.delta_deg - jacobian_.delta_deg) > settings_.jacobian_move_deg);
	const bool stalled = allow_refresh && settings_.stall_ratio > 1.0 && iterations_ > 0 && !last_clamped_
		&& n > prev_norm_ / settings_.stall_ratio;
	prev_norm_ = n;
	alpha_ = at.alpha_deg;
	delta_ = at.delta_deg;
	if ((moved || stalled) && jacobians_ < settings_.max_jacobians) {
		last_stalled_ = stalled;
		ask_ = Ask::Derivatives;
		return;
	}
	const TrimStep s = StepFrom(at, jacobian_, settings_.CL_target, settings_.max_step_deg);
	if (!s.ok) {
		Fail(s.note);
		return;
	}
	if (std::fabs(s.alpha_deg) > settings_.alpha_limit || (settings_.control && std::fabs(s.delta_deg) > settings_.delta_limit)) {
		char buf[260];
		std::snprintf(buf, sizeof buf,
			"the correction asks for alpha = %.2f deg, delta = %.2f deg, outside anything a potential flow describes (|alpha| < %.0f, |delta| < %.0f). The condition is too slow or too heavy for these bodies, or the control too small",
			s.alpha_deg, s.delta_deg, settings_.alpha_limit, settings_.delta_limit);
		Fail(buf);
		return;
	}
	last_clamped_ = s.clamped;
	alpha_ = s.alpha_deg;
	delta_ = s.delta_deg;
	ask_ = Ask::State;
}

void TrimDriver::FeedDerivatives(const TrimPoint& base, const TrimPoint& alpha_step, const TrimPoint& delta_step) {
	const TrimJacobian J = JacobianFrom(base, alpha_step, settings_.control ? &delta_step : nullptr);
	++jacobians_;
	if (!J.ok) {
		Fail(J.note);
		return;
	}
	jacobian_ = J;
	// The base state is itself a solved iterate; on a re-measurement it was
	// already accounted by FeedState.
	if (first_derivatives_) Account(base);
	first_derivatives_ = false;
	PlanAfter(base, false);
}

void TrimDriver::FeedState(const TrimPoint& at) {
	++iterations_;
	Account(at);
	PlanAfter(at, true);
}

double RequiredLiftCoefficient(double load_factor, double weight, double dynamic_pressure, double reference_area) {
	const double qs = dynamic_pressure * reference_area;
	return qs > 0.0 ? load_factor * weight / qs : 0.0;
}

} // namespace aerotelic::aero
