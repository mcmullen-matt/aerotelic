#include "aerotelic/aero_trim.hpp"

#include <cmath>

namespace aerotelic::aero {

TrimResult SolveTrim(const TrimPoint& base, const TrimPoint& alpha_step, const TrimPoint& delta_step, double CL_target) {
	TrimResult r;
	const double da = alpha_step.alpha_deg - base.alpha_deg;
	const double dd = delta_step.delta_deg - base.delta_deg;
	if (std::fabs(da) < 1e-12 || std::fabs(dd) < 1e-12) {
		r.note = "a derivative step is zero";
		return r;
	}
	r.dCL_dalpha = (alpha_step.CL - base.CL) / da;
	r.dCm_dalpha = (alpha_step.Cm - base.Cm) / da;
	r.dCL_ddelta = (delta_step.CL - base.CL) / dd;
	r.dCm_ddelta = (delta_step.Cm - base.Cm) / dd;
	r.determinant = r.dCL_dalpha * r.dCm_ddelta - r.dCL_ddelta * r.dCm_dalpha;
	const double scale = std::fabs(r.dCL_dalpha * r.dCm_ddelta) + std::fabs(r.dCL_ddelta * r.dCm_dalpha);
	if (!(std::fabs(r.determinant) > 1e-9 * (scale > 0.0 ? scale : 1.0))) {
		r.note = "the control and the angle of attack act alike (singular trim system): no trim";
		return r;
	}
	const double rL = CL_target - base.CL;
	const double rM = -base.Cm;
	const double step_a = (rL * r.dCm_ddelta - r.dCL_ddelta * rM) / r.determinant;
	const double step_d = (r.dCL_dalpha * rM - rL * r.dCm_dalpha) / r.determinant;
	if (!std::isfinite(step_a) || !std::isfinite(step_d)) {
		r.note = "the trim step is not finite";
		return r;
	}
	r.alpha_deg = base.alpha_deg + step_a;
	r.delta_deg = base.delta_deg + step_d;
	r.ok = true;
	return r;
}

TrimResult SolveTrimAlpha(const TrimPoint& base, const TrimPoint& alpha_step, double CL_target) {
	TrimResult r;
	const double da = alpha_step.alpha_deg - base.alpha_deg;
	if (std::fabs(da) < 1e-12) {
		r.note = "the angle of attack step is zero";
		return r;
	}
	r.dCL_dalpha = (alpha_step.CL - base.CL) / da;
	r.dCm_dalpha = (alpha_step.Cm - base.Cm) / da;
	if (!(std::fabs(r.dCL_dalpha) > 1e-12)) {
		r.note = "the lift does not change with the angle of attack: no trim";
		return r;
	}
	r.determinant = r.dCL_dalpha;
	r.alpha_deg = base.alpha_deg + (CL_target - base.CL) / r.dCL_dalpha;
	r.delta_deg = base.delta_deg;
	r.ok = std::isfinite(r.alpha_deg);
	if (!r.ok) r.note = "the trim step is not finite";
	return r;
}

double RequiredLiftCoefficient(double load_factor, double weight, double dynamic_pressure, double reference_area) {
	const double qs = dynamic_pressure * reference_area;
	return qs > 0.0 ? load_factor * weight / qs : 0.0;
}

} // namespace aerotelic::aero
