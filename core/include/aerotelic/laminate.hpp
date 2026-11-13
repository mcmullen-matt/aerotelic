#pragma once

// Classical lamination theory for the composite LAMINATES of
// materials.hpp (2026-11-12): the stack a Material's plies describe, its
// A, B, D stiffness, the effective constants the materials table shows,
// the stacking notation a user types, and the Tsai-Wu criterion the
// margins are reported against.
//
// Conventions. The laminate's x axis is its REFERENCE DIRECTION (the
// fibre direction of a 0 degree ply): on a body's skin the loft's own
// along-the-loft direction (the span of a wing, the length of a
// fuselage), on a planar member its longest in-plane extent -- see
// structural_fea.py. A ply's angle is measured from x towards y,
// counter-clockwise about the outward normal z (the right-hand rule).
// Plies are listed from the OUTER face (the one the normal leaves, z =
// +h/2, where a laminator lays the first ply on a female tool) inward; a
// symmetric laminate lists its outer half. Through-thickness positions z
// run from +h/2 at the outer face to -h/2.
//
// Theory as in Jones, Mechanics of Composite Materials, 2nd ed. (1999),
// ch. 4, and Nettles, Basic Mechanics of Laminated Composite Plates, NASA
// RP-1351 (1994), whose worked examples laminate_check.cpp reproduces.
// Strength: Tsai & Wu, "A General Theory of Strength for Anisotropic
// Materials", J. Composite Materials 5 (1971) 58-80, in plane stress with
// the interaction term F12 = -1/2 sqrt(F11 F22) (Tsai & Hahn's choice).

#include <array>
#include <string>
#include <vector>

#include "aerotelic/materials.hpp"

namespace aerotelic {

// A symmetric 3x3 in Voigt order (x, y, xy), row-major.
using Mat3 = std::array<double, 9>;

// The stack as laid, outer face first: the listed plies, then (symmetric)
// their mirror image, the centre ply once when symmetric_odd. Thicknesses
// resolved (Ply::Thickness).
std::vector<Ply> ExpandedStack(const Material& m);

// The total thickness and the mass per unit area of the stack, SI.
double StackThickness(const std::vector<Ply>& stack);
double StackArealMass(const std::vector<Ply>& stack);

// The reduced stiffness Q of a lamina in its own axes (Pa), Voigt order
// (1, 2, 12): Q11, Q12, Q22, Q66 non-zero.
Mat3 ReducedStiffness(const Lamina& l);
// Q rotated to the laminate axes for a ply at `angle_deg` (Q-bar).
Mat3 RotatedStiffness(const Mat3& q, double angle_deg);

struct LaminateStiffness {
	Mat3 a{}, b{}, d{};   // N/m, N, N m
	double thickness = 0.0;
	double areal_mass = 0.0;   // kg/m^2
	// Every ply's bounds through the thickness, outer face first: z_top >
	// z_bottom, z_top of the first ply = +thickness / 2.
	std::vector<double> z_top, z_bottom;
};
LaminateStiffness ComputeStiffness(const std::vector<Ply>& stack);

// The effective constants of the stack, as a material: the in-plane
// moduli with the curvatures FREE (the compliance block of the full
// [A B; B D] inverse -- A^-1 for a symmetric laminate) and the flexural
// moduli from the bending block, 12 / (h^3 d11). `symmetric` is B = 0
// (to a tolerance), `balanced` A16 = A26 = 0.
struct LaminateConstants {
	double ex = 0.0, ey = 0.0, gxy = 0.0, nuxy = 0.0, nuyx = 0.0;   // in-plane
	double ex_flexural = 0.0, ey_flexural = 0.0;
	double density = 0.0;      // areal mass / thickness
	double thickness = 0.0;
	bool symmetric = false, balanced = false;
	// True when the two shear-extension couplings A16, A26 vanish AND B = 0
	// AND D16 = D26 = 0: the laminate behaves as an orthotropic plate.
	bool orthotropic = false;
};
LaminateConstants EngineeringConstants(const LaminateStiffness& s);

// Midplane strains and curvatures (x, y, xy) under the resultants (N, M)
// -- the full inverse of [A B; B D] -- and the stresses of every ply in
// its OWN axes (1, 2, 12) at a through-thickness position.
struct LaminateStrainState {
	std::array<double, 3> strain{};      // midplane
	std::array<double, 3> curvature{};
};
LaminateStrainState Deformation(const LaminateStiffness& s, const std::array<double, 3>& n, const std::array<double, 3>& m);
// The stress (1, 2, 12) in a ply of angle `angle_deg` and stiffness `q`
// (its own axes) at height z under the state.
std::array<double, 3> PlyStress(const Mat3& q, double angle_deg, const LaminateStrainState& st, double z);

// Tsai-Wu plane-stress reserve factor of a ply stress (1, 2, 12), Pa: the
// factor the stress can be scaled by before the criterion reaches 1
// (above 1 is safe; a margin is this minus one). Infinity when the lamina
// has no strengths; 0 is never returned for a non-zero stress.
double TsaiWuReserveFactor(const Lamina& l, const std::array<double, 3>& stress);

// The first-ply-failure strength of the stack under uniaxial tension
// along x, Pa (the smallest N_x / h at which any ply's Tsai-Wu reserve
// factor, taken at its two faces, reaches 1), or 0 when a ply has no
// strengths.
double FirstPlyFailureTension(const std::vector<Ply>& stack);

// Fills a laminate Material's effective figures from its plies: density,
// youngs_modulus (ex), poisson_ratio (nuxy), allowable_stress (the
// first-ply-failure tension). A material that is not a laminate is left
// alone. Returns the constants.
LaminateConstants RefreshLaminate(Material& m);

// ---- stacking notation -----------------------------------------------------
// "[0/±45/90]s", "[0/90]2s", "[45/-45/0_2/90~]s", "0/90": angles separated
// by '/', optional brackets, ± (or +-) for a pair, an angle's _k (or xk)
// repeat, the group's trailing k repeat and s for symmetric; a trailing ~
// on the LAST angle of a symmetric stack marks it the single centre ply
// (the overbar of the printed notation). Spaces are ignored. Returns the
// angles as listed (the half stack when symmetric) with the flags, or
// false with a reason.
bool ParseStackingNotation(const std::string& text, std::vector<double>* angles, bool* symmetric, bool* symmetric_odd, std::string* error);
// The notation of a material's plies: ± for a +a/-a pair, _k for a run,
// ]s for symmetric, ~ for the odd centre ply.
std::string StackingNotation(const Material& m);

// A laminate material from a notation and ONE lamina (the common case):
// name, category "Laminate", the plies at the lamina's ply thickness,
// refreshed. Throws std::invalid_argument on bad notation.
Material MakeLaminate(const std::string& name, const std::string& notation, const Material& lamina_row, const std::string& source);

// Why `m`'s stack cannot be analysed -- no plies, a ply without a lamina
// name, without stiffness or thickness -- or empty.
std::string LaminateProblem(const Material& m);

} // namespace aerotelic
