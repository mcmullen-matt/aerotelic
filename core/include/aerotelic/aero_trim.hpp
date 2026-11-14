#pragma once

// Rigid trim of a configuration on a linear aerodynamic model (stage 4 of
// the flight-loads plan, 2026-11-14): the angle of attack and one control
// deflection that give the lift a flight condition needs and no pitching
// moment about the mass centre.
//
// The panel method is linear in the free stream's direction only to first
// order (the wake sheds along the stream and a flap kinks the section), so
// the trim is a Newton step on finite-difference derivatives: the base
// solution, one with the angle of attack stepped, one with the control
// stepped -- three linear solves -- give the 2 x 2 system
//
//   [ dCL/da  dCL/dd ] [ da ]   [ CL_target - CL0 ]
//   [ dCm/da  dCm/dd ] [ dd ] = [       0 - Cm0   ]
//
// whose solution is the trimmed state. A fourth solve at that state is the
// check (the caller's: it reports the residual). With no control, the one
// equation CL = CL_target in the angle of attack alone, the moment reported
// untrimmed.
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
TrimResult SolveTrim(const TrimPoint& base, const TrimPoint& alpha_step, const TrimPoint& delta_step, double CL_target);

// Angle of attack alone: CL = CL_target by dCL/dalpha; delta stays the
// base's and Cm is not driven.
TrimResult SolveTrimAlpha(const TrimPoint& base, const TrimPoint& alpha_step, double CL_target);

// q = rho V^2 / 2 (SI in, Pa out).
inline double DynamicPressure(double density, double speed) { return 0.5 * density * speed * speed; }

// The lift coefficient a flight condition needs: n W / (q S), everything
// SI (weight in newtons, S in m^2). Zero when q S is not positive.
double RequiredLiftCoefficient(double load_factor, double weight, double dynamic_pressure, double reference_area);

} // namespace aerotelic::aero
