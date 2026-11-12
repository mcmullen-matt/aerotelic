#pragma once

// Two-dimensional aerodynamics of one airfoil section, per unit span: the
// common request/result shape every analysis method fills, the registry of
// methods, and the viscous cues layered on any method's inviscid pressure
// distribution. The dialog runs several methods on the same section and
// juxtaposes their results, so a method is only the inviscid solve; the
// trailing-edge closure, the transition and separation cues and the
// skin-friction drag are shared here and applied identically to all.
//
// Conventions: chord along +x, leading edge at the minimum x; angle of attack
// in degrees; Mach 0; Reynolds number on the chord. Surfaces are reported
// from the front stagnation point to the trailing edge.

#include <memory>
#include <string>
#include <vector>

#include "aerotelic/mat.hpp"
#include "aerotelic/nurbs.hpp"

namespace aerotelic {

class Airfoil;

struct SectionAeroRequest {
	// (M, 2) lower TE -> LE -> upper TE; an open trailing edge is closed
	// before analysis (see CloseTrailingEdge).
	Mat contour;
	double alpha_deg = 0.0;
	double reynolds = 1.0e6;
	int n_points = 512;             // method resolution (power of two for the conformal map)
	int panel_nodes = 200;          // airfoil nodes of the panel methods (cosine resampling of the contour)
	double te_blend_start = 0.7;    // x/c from which the trailing-edge gap is blended out
	double transition_re_x = 3.5e6; // Re_x at which a laminar run is taken to transition (inviscid methods' cue)
	// Boundary-layer method (mfoil):
	double ncrit = 9.0;             // e^N critical amplification factor
	double xft_upper = 1.0;         // forced transition x/c per surface (1 = natural only)
	double xft_lower = 1.0;
	double h_sep_lam = 3.83;        // laminar separation cue: H_k where the laminar c_f closure vanishes
	double h_sep_turb = 2.8;        // turbulent incipient detachment cue (Kline 2.7 / Castillo-George 2.76)
};

// The request for an Airfoil: its curve sampled with cosine spacing,
// `n_per_surface` points on each surface.
SectionAeroRequest SectionRequestFor(const Airfoil& airfoil, int n_per_surface = 300);

struct SurfaceCue {
	bool found = false;
	double x = 0.0, y = 0.0;    // location, chord units (x/c)
	double s = 0.0;             // arc length from the stagnation point / c
	std::string note;           // which rule fired, or why not
};

struct SectionSurface {
	// From the front stagnation point (s = 0, cp = 1) to the trailing edge.
	std::vector<double> x, y, s;    // x/c, y/c, s/c
	std::vector<double> q;          // |V| / U
	std::vector<double> cp;         // 1 - q^2
	std::vector<double> stratford;  // Stratford's parameter S(x) (0 ahead of the suction peak)
	int peak_index = 0;             // suction peak (max q)
	// Cues. Inviscid methods: transition = the earlier of Re_x >= transition_re_x
	// and the first adverse gradient; separation = Stratford (1959), fully
	// turbulent from stagnation. The boundary-layer method: transition = the
	// e^N location (or forced), separation = the shape-factor thresholds,
	// detachment = where the solved turbulent c_f falls to zero.
	SurfaceCue transition;
	SurfaceCue separation;
	SurfaceCue detachment;
	double stratford_range_end_x = -1.0; // x/c where Cpbar first exceeded 4/7 (criterion no longer valid), -1 if never
	double cd_friction = 0.0;       // this surface's share of the skin-friction drag
	double cf_integral_x_equiv = 0.0; // Stratford's equivalent flat-plate length x_EL / c at the peak
	// Boundary-layer distributions (the boundary-layer method only; empty otherwise).
	std::vector<double> theta, delta_star, hk, cf, amp; // amp: n~ (laminar) or sqrt(c_tau) (turbulent)
	std::vector<char> turb;
};

struct SectionAeroResult {
	std::string method;
	bool ok = false;
	std::string error;
	std::vector<std::string> warnings;

	double alpha_deg = 0.0, reynolds = 0.0;
	bool converged = true;          // false when an iterative method stopped at its limit
	// Integrated from the pressure distribution over the closed contour. For
	// inviscid methods cd_pressure is the d'Alembert residual; for the
	// boundary-layer method it is the pressure drag cd - cd_friction, with
	// cd_total the Squire-Young total.
	double cl = 0.0, cd_pressure = 0.0, cm_le = 0.0, cm_c4 = 0.0;
	double cd_total = 0.0;
	// Method-specific exact values where the method has them (else NaN).
	double cl_exact = 0.0;          // e.g. Kutta-Joukowski from the circulation
	double alpha0_deg = 0.0;        // zero-lift angle
	double cl_alpha = 0.0;          // per radian
	double cp_min = 0.0;
	double cd_friction = 0.0;       // sum of the two surfaces' flat-plate estimates

	SectionSurface upper, lower;

	// Geometry actually analysed.
	Mat contour;                    // after the trailing-edge closure
	double te_gap_closed = 0.0;     // the gap removed, chord units
	double te_angle_deg = 0.0;

	// Diagnostics.
	int iterations = 0;
	double residual = 0.0;
	double convergence_margin = 0.0; // conformal map: max |d eps / d phi|
	double solve_ms = 0.0;
};

class SectionAeroMethod {
public:
	virtual ~SectionAeroMethod() = default;
	virtual std::string Name() const = 0;
	virtual std::string Description() const = 0;
	// False with a reason when the method needs something absent (an
	// external program, a Python environment, ...).
	virtual bool Available(std::string* why = nullptr) const { (void)why; return true; }
	// Sub-millisecond: safe to run on every pole drag, in sweeps and in
	// finite-difference sensitivities. Slow methods run on request.
	virtual bool Fast() const { return true; }
	// Solves the boundary layer itself (its cues are its own; the shared
	// inviscid cues are not layered on it).
	virtual bool Viscous() const { return false; }
	// The inviscid solve; the shared post-processing (cues, friction drag)
	// is added by SolveSection.
	virtual SectionAeroResult Solve(const SectionAeroRequest& request) const = 0;
};

// All registered methods, in display order. The first is the conformal map
// (Theodorsen / Ives), then the vortex-panel methods (section_aero_panel.cpp).
const std::vector<const SectionAeroMethod*>& SectionAeroMethods();

// The registry's constructors for the methods that live in their own files.
std::unique_ptr<SectionAeroMethod> MakeVortexPanelInviscidMethod();
std::unique_ptr<SectionAeroMethod> MakeVortexPanelIblMethod();
const SectionAeroMethod* FindSectionAeroMethod(const std::string& name);

// Runs the method and layers the shared post-processing on its result.
SectionAeroResult SolveSection(const SectionAeroMethod& method, const SectionAeroRequest& request);

// XFOIL-style trailing-edge closure: the gap between the two trailing-edge
// points is removed by moving the upper surface down and the lower surface
// up, each by half the gap times ((x - x0) / (1 - x0))^2 for x > x0, so the
// shift is C1 at x0 = blend_start and complete at the trailing edge. The
// closed contour's first and last rows coincide. Returns the gap removed.
Mat CloseTrailingEdge(const Mat& contour, double blend_start, double* gap_out = nullptr);

// The shared viscous post-processing on one surface (from stagnation): the
// transition cue, Stratford's turbulent separation criterion and the
// flat-plate skin-friction drag. Exposed for the tests; SolveSection calls
// it for both surfaces.
void AddBoundaryLayerCues(SectionSurface& surface, double reynolds, double alpha_deg, double transition_re_x);

// ---- control-point sensitivity ---------------------------------------------

enum class SectionAeroScalar { Cl, CmC4, CdFriction, CpUpperAtX, CpLowerAtX };

const char* SectionAeroScalarName(SectionAeroScalar s);
double SectionAeroScalarValue(const SectionAeroResult& result, SectionAeroScalar scalar, double probe_x);

struct PoleSensitivity {
	bool ok = false;
	std::string error;
	double base = 0.0;                  // the scalar at the unperturbed curve
	std::vector<double> d_dx, d_dy;     // per pole, per unit (chord) displacement
};

// Central finite differences of `scalar` with respect to each pole of the
// section curve (an Airfoil curve spec, lower TE -> LE -> upper TE), step in
// chord units. The request's contour is replaced by each perturbed curve's
// sampling; its other fields are kept.
PoleSensitivity ComputePoleSensitivity(const NurbsSpec& curve, const SectionAeroMethod& method,
	const SectionAeroRequest& request, SectionAeroScalar scalar, double probe_x, double step = 1e-3);

} // namespace aerotelic
