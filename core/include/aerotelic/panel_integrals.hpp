#pragma once

// The panel influence integrals of Johnson, "A General Panel Method for the
// Analysis and Design of Arbitrary Configurations in Incompressible Flows",
// NASA CR-3079 (1980), appendix D: the potential and velocity induced at a
// field point by a curved panel carrying a linear source or a quadratic
// doublet distribution, in closed form.
//
// A panel is a plane quadrilateral Σ in its own (ξ, η, ζ) frame (appendix A),
// bulged into ζ = a ξ² + b η². The singularity on it is
//     σ(ξ, η) = σ0 + σξ ξ + ση η                                  (D.8)
//     μ(ξ, η) = μ0 + μξ ξ + μη η + ½ μξξ ξ² + μξη ξη + ½ μηη η²    (D.9)
// and the induced potential and velocity are linear in those coefficients:
// Influence holds the coefficient rows.
//
// Near field (D.2, D.3): everything reduces to the integrals
//     H(M, N, K) = ∬ (ξ - x)^(M-1) (η - y)^(N-1) / ρ^K dξ dη,  ρ² = (ξ-x)² + (η-y)² + h²
// over Σ, evaluated from the perimeter line integrals F(M, N, K) by the
// recursions of section D.3 (procedures 1-5), with H(1, 1, 1) and F(1, 1, 1)
// as the only transcendental seeds (an arctangent sum and a logarithm).
// Near the panel plane the regular part H* (D.51) is recursed instead and
// the jump terms 2π ν(M, N, K) sign(h) are added where they survive.
//
// Far field (D.4): the multipole expansion to the quadrupole in the panel
// moments C(M, N) = ∬ ξ^(M-1) η^(N-1) dξ dη, used when the panel's extent is
// under a fifth of the distance to the field point.

#include <array>

#include "aerotelic/vec3.hpp"

namespace aerotelic::panel {

constexpr int kMaxQ = 6; // MXQ of (D.36): the largest M, N needed (doublet velocity, curved)
constexpr int kMaxK = 7; // MXK of (D.36)

struct LocalPanel {
	// Corners of Σ, counter-clockwise in (ξ, η) (so each side's outer normal
	// is the tangent turned clockwise, as figure D.3 has it). A triangular
	// panel repeats a corner.
	std::array<double, 4> xi{}, eta{};
	double a = 0.0, b = 0.0; // ζ = a ξ² + b η²

	// Moments C(M, N), 1-based, for M + N - 1 <= kMaxQ + 3 (the far field
	// reaches C(M + 3, N)). Filled by ComputeMoments().
	double C[kMaxQ + 5][kMaxQ + 5] = {};
	void ComputeMoments();

	// Largest |Q| over the corners, Q = (ξ, η, ζ): the ε of (D.72) is this
	// over the distance to the field point.
	double MaxExtent() const;
	double Area() const;
};

struct Influence {
	// Potential per unit coefficient: source (σ0, σξ, ση), doublet
	// (μ0, μξ, μη, μξξ, μξη, μηη).
	double phi_source[3] = {};
	double phi_doublet[6] = {};
	// Velocity in the panel frame, [component][coefficient].
	double vel_source[3][3] = {};
	double vel_doublet[3][6] = {};
};

struct IntegralOptions {
	double delta_h = 0.01;  // δh: |h| < δh dH is "near the plane"
	double delta_g = 0.01;  // δg: g < δg dF is "near the perimeter"
	double delta_0 = 1e-8;  // δ0: |h| < δ0 dH is "on the plane" (average values)
	int nhk = 16, nfk = 16; // reverse-recursion head room (D.49, D.68)
	double far_ratio = 0.2; // use the far field when extent / distance < this (D.74: 1/5)
	bool allow_far_field = true;
};

// The influence of the panel at the field point q, given in the panel's own
// frame. `used_far_field` reports which expansion was taken.
Influence ComputeInfluence(const LocalPanel& panel, const Vec3& q, const IntegralOptions& opt = {}, bool* used_far_field = nullptr);
Influence NearFieldInfluence(const LocalPanel& panel, const Vec3& q, const IntegralOptions& opt = {});
Influence FarFieldInfluence(const LocalPanel& panel, const Vec3& q);

// Reference values by brute-force quadrature of (D.1)-(D.4) over the
// curved panel, for tests: slow, and only meaningful off the panel.
Influence QuadratureInfluence(const LocalPanel& panel, const Vec3& q, int n = 400);

} // namespace aerotelic::panel
