// Checks the closed-form panel influence integrals (panel_integrals.hpp)
// against brute-force quadrature of the same curved-panel integrals: flat
// and curved panels, field points over the panel, beside it, close to the
// plane and far away; the far-field expansion against the near field; and
// the jumps across the panel (potential ±μ/2, normal velocity ±σ/2,
// tangential velocity ±∇μ/2) that the H* bookkeeping must reproduce.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "aerotelic/panel_integrals.hpp"

using namespace aerotelic;
using namespace aerotelic::panel;

namespace {

int g_pass = 0, g_fail = 0;

void Check(bool ok, const std::string& what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what.c_str());
	if (ok) ++g_pass; else ++g_fail;
}

std::string F(const std::string& fmt, double a, double b = 0.0, double c = 0.0) {
	char buf[240];
	std::snprintf(buf, sizeof buf, fmt.c_str(), a, b, c);
	return buf;
}

// Largest relative difference over the rows, each relative to the larger
// magnitude in that row (coefficient rows differ in scale).
double RowError(const double* u, const double* v, int n) {
	double worst = 0.0, scale = 1e-12;
	for (int i = 0; i < n; ++i) scale = std::max(scale, std::max(std::fabs(u[i]), std::fabs(v[i])));
	for (int i = 0; i < n; ++i) worst = std::max(worst, std::fabs(u[i] - v[i]) / scale);
	return worst;
}

double InfluenceError(const Influence& a, const Influence& b, bool velocities = true) {
	double e = std::max(RowError(a.phi_source, b.phi_source, 3), RowError(a.phi_doublet, b.phi_doublet, 6));
	if (velocities) {
		for (int k = 0; k < 3; ++k) {
			e = std::max(e, RowError(a.vel_source[k], b.vel_source[k], 3));
			e = std::max(e, RowError(a.vel_doublet[k], b.vel_doublet[k], 6));
		}
	}
	return e;
}

LocalPanel MakePanel(double a, double b, bool skew) {
	LocalPanel p;
	if (skew) {
		p.xi = {-0.9, 0.8, 1.1, -0.6};
		p.eta = {-0.5, -0.7, 0.6, 0.9};
	} else {
		p.xi = {-1.0, 1.0, 1.0, -1.0};
		p.eta = {-1.0, -1.0, 1.0, 1.0};
	}
	p.a = a; p.b = b;
	p.ComputeMoments();
	return p;
}

} // namespace

int main() {
	setvbuf(stdout, nullptr, _IONBF, 0);
	IntegralOptions opt;
	opt.allow_far_field = false;

	// ---- moments ----
	{
		LocalPanel sq = MakePanel(0.0, 0.0, false);
		Check(std::fabs(sq.C[1][1] - 4.0) < 1e-12 && std::fabs(sq.C[3][1] - 4.0 / 3.0) < 1e-12 && std::fabs(sq.C[2][2]) < 1e-12 && std::fabs(sq.C[3][3] - 4.0 / 9.0) < 1e-12,
			"moments: C(1,1) = 4, C(3,1) = 4/3, C(2,2) = 0, C(3,3) = 4/9 on the 2x2 square");
		Check(std::fabs(sq.Area() - 4.0) < 1e-12, "area of the square");
	}

	// ---- near field against quadrature ----
	struct Case { const char* name; LocalPanel panel; Vec3 q; double tol; };
	std::vector<Case> cases = {
		{"flat square, point above the centre", MakePanel(0, 0, false), {0.0, 0.0, 1.0}, 2e-4},
		{"flat square, point above a corner region", MakePanel(0, 0, false), {0.7, -0.4, 0.6}, 2e-4},
		{"flat square, point beside the panel", MakePanel(0, 0, false), {2.5, 0.3, 0.4}, 2e-4},
		{"flat square, point beside, nearly in the plane", MakePanel(0, 0, false), {1.6, 0.2, 0.004}, 2e-4},
		{"flat skew quad, point above", MakePanel(0, 0, true), {0.2, 0.1, 0.8}, 2e-4},
		{"flat skew quad, point beside and below", MakePanel(0, 0, true), {-1.8, 0.9, -0.5}, 2e-4},
		{"curved square (a = 0.04, b = -0.03), point above", MakePanel(0.04, -0.03, false), {0.1, -0.2, 1.2}, 3e-3},
		{"curved square, point beside", MakePanel(0.04, -0.03, false), {2.2, 0.4, 0.3}, 3e-3},
		{"curved skew quad, point above", MakePanel(0.03, 0.05, true), {-0.3, 0.2, 0.9}, 3e-3},
		{"curved skew quad, point far", MakePanel(0.03, 0.05, true), {3.0, -2.0, 2.5}, 3e-3},
	};
	for (const Case& c : cases) {
		Influence near = NearFieldInfluence(c.panel, c.q, opt);
		Influence ref = QuadratureInfluence(c.panel, c.q, 600);
		double e = InfluenceError(near, ref);
		Check(e < c.tol, F(std::string("near field vs quadrature: ") + c.name + " (%.2e)", e));
	}

	// ---- far field against the near field ----
	{
		LocalPanel p = MakePanel(0.03, -0.02, true);
		for (Vec3 q : {Vec3{12.0, 3.0, 5.0}, Vec3{-20.0, 15.0, -2.0}, Vec3{0.5, 0.2, 25.0}, Vec3{60.0, -10.0, 8.0}}) {
			Influence near = NearFieldInfluence(p, q, opt);
			Influence far = FarFieldInfluence(p, q);
			double eps = p.MaxExtent() / Length(q);
			// Sources keep the quadrupole (error O(ε³)); doublets stop at the
			// dipole-of-dipole term (D.92 note), so O(ε²).
			double es = std::max(RowError(near.phi_source, far.phi_source, 3), std::max(RowError(near.vel_source[0], far.vel_source[0], 3), std::max(RowError(near.vel_source[1], far.vel_source[1], 3), RowError(near.vel_source[2], far.vel_source[2], 3))));
			double ed = std::max(RowError(near.phi_doublet, far.phi_doublet, 6), std::max(RowError(near.vel_doublet[0], far.vel_doublet[0], 6), std::max(RowError(near.vel_doublet[1], far.vel_doublet[1], 6), RowError(near.vel_doublet[2], far.vel_doublet[2], 6))));
			// (The near field is itself only O(δ²) on a curved panel, D.6, so
			// the floor below is that, not roundoff.)
			Check(es < 30.0 * eps * eps * eps + 5e-4, F("far field vs near field, source, extent/distance %.3f (%.2e)", eps, es));
			Check(ed < 4.0 * eps * eps + 5e-4, F("far field vs near field, doublet, extent/distance %.3f (%.2e)", eps, ed));
		}
	}

	// ---- the plane: jumps and averages (section D.5) ----
	{
		LocalPanel p = MakePanel(0.0, 0.0, false);
		const Vec3 inside{0.3, -0.2, 0.0};
		Influence avg = NearFieldInfluence(p, inside, opt);
		Influence up = NearFieldInfluence(p, {0.3, -0.2, 1e-6}, opt);
		Influence dn = NearFieldInfluence(p, {0.3, -0.2, -1e-6}, opt);
		// Doublet potential: +μ/2 above, -μ/2 below, 0 on the plane (μ evaluated at the point).
		double mu_at[6] = {1.0, 0.3, -0.2, 0.5 * 0.09, -0.06, 0.5 * 0.04};
		bool ok_phi = true;
		for (int c = 0; c < 6; ++c) {
			ok_phi = ok_phi && std::fabs(up.phi_doublet[c] - dn.phi_doublet[c] - mu_at[c]) < 1e-4;
			ok_phi = ok_phi && std::fabs(avg.phi_doublet[c] - 0.5 * (up.phi_doublet[c] + dn.phi_doublet[c])) < 1e-4;
		}
		Check(ok_phi, "doublet potential jumps by μ across the panel; the on-plane value is the average");
		// Source normal velocity: +σ/2 above, -σ/2 below.
		double sig_at[3] = {1.0, 0.3, -0.2};
		bool ok_vn = true;
		for (int c = 0; c < 3; ++c) {
			ok_vn = ok_vn && std::fabs(up.vel_source[2][c] - dn.vel_source[2][c] - sig_at[c]) < 1e-4;
			ok_vn = ok_vn && std::fabs(avg.vel_source[2][c] - 0.5 * (up.vel_source[2][c] + dn.vel_source[2][c])) < 1e-4;
		}
		Check(ok_vn, "source normal velocity jumps by σ across the panel; the on-plane value is the average");
		// Doublet tangential velocity: +∇μ/2 above, -∇μ/2 below.
		double dmu_dx[6] = {0.0, 1.0, 0.0, 0.3, -0.2, 0.0}, dmu_dy[6] = {0.0, 0.0, 1.0, 0.0, 0.3, -0.2};
		bool ok_vt = true;
		for (int c = 0; c < 6; ++c) {
			ok_vt = ok_vt && std::fabs(up.vel_doublet[0][c] - dn.vel_doublet[0][c] - dmu_dx[c]) < 1e-3;
			ok_vt = ok_vt && std::fabs(up.vel_doublet[1][c] - dn.vel_doublet[1][c] - dmu_dy[c]) < 1e-3;
		}
		Check(ok_vt, "doublet tangential velocity jumps by ∇μ across the panel");
		// Continuity of the regular parts: the on-plane evaluation (H*) is the
		// limit of the off-plane one (procedure 1 at |h| just above δh dH).
		Influence off = NearFieldInfluence(p, {0.3, -0.2, 0.02}, opt);
		Influence offm = NearFieldInfluence(p, {0.3, -0.2, -0.02}, opt);
		// The doublet potential is odd in h, so the two average to the
		// on-plane value; the source potential has the kink |h| σ/2 at the
		// panel, so their mean sits σ h / 2 above it.
		Influence mean;
		for (int c = 0; c < 6; ++c) mean.phi_doublet[c] = 0.5 * (off.phi_doublet[c] + offm.phi_doublet[c]);
		for (int c = 0; c < 3; ++c) mean.phi_source[c] = 0.5 * (off.phi_source[c] + offm.phi_source[c]) - 0.5 * sig_at[c] * 0.02;
		Check(RowError(mean.phi_doublet, avg.phi_doublet, 6) < 2e-2 && RowError(mean.phi_source, avg.phi_source, 3) < 2e-3,
			F("on-plane values continue the off-plane ones (procedure 3 joins procedure 1; %.2e, %.2e)", RowError(mean.phi_doublet, avg.phi_doublet, 6), RowError(mean.phi_source, avg.phi_source, 3)));
		// Source potential and doublet potential by quadrature at a point just
		// above the plane inside Σ (quadrature converges there for the potential).
		Influence ref = QuadratureInfluence(p, {0.3, -0.2, 0.05}, 800);
		Influence near = NearFieldInfluence(p, {0.3, -0.2, 0.05}, opt);
		Check(RowError(near.phi_source, ref.phi_source, 3) < 2e-3 && RowError(near.phi_doublet, ref.phi_doublet, 6) < 2e-3,
			F("potentials just above the panel vs quadrature (%.2e, %.2e)", RowError(near.phi_source, ref.phi_source, 3), RowError(near.phi_doublet, ref.phi_doublet, 6)));
	}

	// ---- a triangular (collapsed-corner) panel ----
	{
		LocalPanel t;
		t.xi = {0.0, 1.0, 0.0, 0.0};
		t.eta = {0.0, 0.0, 1.0, 1.0};
		t.ComputeMoments();
		Check(std::fabs(t.C[1][1] - 0.5) < 1e-12, "triangle: area 1/2 from the moments");
		Influence near = NearFieldInfluence(t, {0.2, 0.3, 0.7}, opt);
		Influence ref = QuadratureInfluence(t, {0.2, 0.3, 0.7}, 600);
		double e = InfluenceError(near, ref);
		Check(e < 5e-4, F("triangle: near field vs quadrature (%.2e)", e));
	}

	std::printf("\nRESULT: %d passed, %d failed\n", g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}
