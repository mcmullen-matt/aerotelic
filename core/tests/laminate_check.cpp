// Checks laminate.hpp against the worked examples of Nettles, "Basic
// Mechanics of Laminated Composite Plates", NASA RP-1351 (1994) -- the
// AS4/3501-6 lamina of its Example 1 (E1 20.01 Msi, E2 1.301 Msi, G12
// 1.001 Msi, nu12 0.3, ply 0.005 in), its reduced and rotated stiffnesses,
// the A and D of the symmetric [0/45]s (Examples 1 and 3) and the B, D and
// Ex of the unsymmetric [0/45]T (Example 2) -- in the report's own units
// (psi, in, lb), since the theory is unit-free; then the properties a
// quasi-isotropic stack must have, the Tsai-Wu criterion's closed forms,
// the stacking notation both ways, the library file's lamina and laminate
// rows, and the presets.

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "aerotelic/laminate.hpp"
#include "aerotelic/materials.hpp"

using namespace aerotelic;

namespace {

int g_failures = 0;

void Check(bool ok, const std::string& what) {
	std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what.c_str());
	if (!ok) ++g_failures;
}

bool Rel(double a, double b, double rel) { return std::fabs(a - b) <= rel * std::max(std::fabs(b), 1e-300); }
bool Abs(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

std::string Fmt(double v) {
	char buf[64];
	std::snprintf(buf, sizeof buf, "%.6g", v);
	return buf;
}

// Nettles' lamina, psi and inches.
Lamina NettlesLamina() {
	Lamina l;
	l.form = "UD";
	l.ply_thickness = 0.005;
	l.e1 = 20.01e6;
	l.e2 = 1.301e6;
	l.nu12 = 0.3;
	l.g12 = 1.001e6;
	return l;
}

Material Stack(const std::vector<double>& angles, bool symmetric, const Lamina& l, double density = 1.0) {
	Material m;
	m.name = "test";
	m.symmetric = symmetric;
	for (double a : angles) {
		Ply p;
		p.angle = a;
		p.lamina = "nettles";
		p.density = density;
		p.props = l;
		m.plies.push_back(p);
	}
	return m;
}

double A(const Mat3& m, int i, int j) { return m[i * 3 + j]; }

} // namespace

int main() {
	const Lamina lam = NettlesLamina();

	// ---- Q and Q-bar (RP-1351 eqs E1.1, E1.3) ----
	// Nettles rounds nu21 = nu12 E2 / E1 = 0.0195 to 0.02 before forming Q,
	// which moves his figures by 1.5e-4; the tolerance allows for it.
	{
		const Mat3 q = ReducedStiffness(lam);
		Check(Rel(A(q, 0, 0), 20130785.0, 3e-4) && Rel(A(q, 1, 1), 1308853.0, 3e-4) && Rel(A(q, 0, 1), 392656.0, 3e-4) && Rel(A(q, 2, 2), 1001000.0, 1e-9),
			"Q of the AS4/3501-6 lamina: " + Fmt(A(q, 0, 0)) + ", " + Fmt(A(q, 1, 1)) + ", " + Fmt(A(q, 0, 1)) + ", " + Fmt(A(q, 2, 2)) + " psi (RP-1351 E1.1)");
		const Mat3 r = RotatedStiffness(q, 45.0);
		Check(Rel(A(r, 0, 0), 6557237.0, 3e-4) && Rel(A(r, 0, 1), 4555238.0, 3e-4) && Rel(A(r, 1, 1), 6557237.0, 3e-4)
			&& Rel(A(r, 0, 2), 4705483.0, 3e-4) && Rel(A(r, 1, 2), 4705483.0, 3e-4) && Rel(A(r, 2, 2), 5163582.0, 3e-4),
			"Q-bar at 45 degrees (RP-1351 E1.3): " + Fmt(A(r, 0, 0)) + ", " + Fmt(A(r, 0, 1)) + ", " + Fmt(A(r, 0, 2)) + ", " + Fmt(A(r, 2, 2)));
		const Mat3 r0 = RotatedStiffness(q, 0.0);
		const Mat3 r90 = RotatedStiffness(q, 90.0);
		Check(Rel(A(r0, 0, 0), A(q, 0, 0), 1e-12) && Rel(A(r90, 0, 0), A(q, 1, 1), 1e-9) && Rel(A(r90, 1, 1), A(q, 0, 0), 1e-9) && std::fabs(A(r90, 0, 2)) < 1e-6 * A(q, 0, 0),
			"Q-bar at 0 is Q, at 90 swaps 1 and 2 with no coupling");
		const Mat3 rm = RotatedStiffness(q, -45.0);
		Check(Rel(A(rm, 0, 0), A(r, 0, 0), 1e-12) && Rel(A(rm, 0, 2), -A(r, 0, 2), 1e-12), "Q-bar at -45 has the same normal terms and the opposite couplings");
	}

	// ---- [0/45]s, h = 0.02 in (RP-1351 Examples 1 and 3) ----
	{
		Material m = Stack({0.0, 45.0}, true, lam);
		const std::vector<Ply> stack = ExpandedStack(m);
		Check(stack.size() == 4 && stack[0].angle == 0.0 && stack[1].angle == 45.0 && stack[2].angle == 45.0 && stack[3].angle == 0.0, "[0/45]s expands to 0/45/45/0");
		Check(Rel(StackThickness(stack), 0.02, 1e-12), "...0.02 in thick");
		const LaminateStiffness s = ComputeStiffness(stack);
		Check(Rel(A(s.a, 0, 0), 266880.0, 1e-3) && Rel(A(s.a, 0, 1), 49479.0, 1e-3) && Rel(A(s.a, 1, 1), 78661.0, 1e-3)
			&& Rel(A(s.a, 0, 2), 47055.0, 1e-3) && Rel(A(s.a, 1, 2), 47055.0, 1e-3) && Rel(A(s.a, 2, 2), 61646.0, 1e-3),
			"A of [0/45]s (RP-1351 E1.4): A11 " + Fmt(A(s.a, 0, 0)) + ", A12 " + Fmt(A(s.a, 0, 1)) + ", A22 " + Fmt(A(s.a, 1, 1)) + ", A16 " + Fmt(A(s.a, 0, 2)) + ", A66 " + Fmt(A(s.a, 2, 2)) + " lb/in");
		double bmax = 0.0;
		for (double v : s.b) bmax = std::max(bmax, std::fabs(v));
		Check(bmax == 0.0, "B of a symmetric stack is exactly zero");
		Check(Rel(A(s.d, 0, 0), 12.3, 5e-3) && Rel(A(s.d, 0, 1), 0.609, 5e-3) && Rel(A(s.d, 1, 1), 1.31, 5e-3)
			&& Rel(A(s.d, 0, 2), 0.392, 5e-3) && Rel(A(s.d, 2, 2), 1.014, 5e-3),
			"D of [0/45]s (RP-1351 Example 3): D11 " + Fmt(A(s.d, 0, 0)) + ", D12 " + Fmt(A(s.d, 0, 1)) + ", D22 " + Fmt(A(s.d, 1, 1)) + ", D16 " + Fmt(A(s.d, 0, 2)) + ", D66 " + Fmt(A(s.d, 2, 2)) + " in-lb");
		const LaminateConstants c = EngineeringConstants(s);
		Check(Rel(c.ex, 11333000.0, 1e-3), "Ex of [0/45]s is 11,333,000 psi (RP-1351 E1.5): " + Fmt(c.ex));
		Check(c.symmetric && !c.balanced && !c.orthotropic, "[0/45]s is symmetric, not balanced (A16 = A26 != 0), not orthotropic");
		// Example 3: Nx = 500 lb/in gives eps_x0 0.00221, eps_y0 -0.0007, gamma_xy0 -0.00115.
		const LaminateStrainState st = Deformation(s, {500.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
		Check(Rel(st.strain[0], 0.00221, 5e-3) && Abs(st.strain[1], -0.0007, 1e-5) && Rel(st.strain[2], -0.00115, 1e-2) && std::fabs(st.curvature[0]) < 1e-12,
			"Nx = 500 lb/in on [0/45]s: eps_x " + Fmt(st.strain[0]) + ", eps_y " + Fmt(st.strain[1]) + ", gamma_xy " + Fmt(st.strain[2]) + ", no curvature (RP-1351 Example 3)");
		// The ply stresses: the 0 ply carries the fibre stress, and the two
		// 45 plies the same stress as each other (symmetric, no curvature).
		const Mat3 q = ReducedStiffness(lam);
		const auto s0 = PlyStress(q, 0.0, st, 0.5 * (s.z_top[0] + s.z_bottom[0]));
		const auto s1 = PlyStress(q, 45.0, st, 0.5 * (s.z_top[1] + s.z_bottom[1]));
		const auto s2 = PlyStress(q, 45.0, st, 0.5 * (s.z_top[2] + s.z_bottom[2]));
		Check(s0[0] > s1[0] && Rel(s1[0], s2[0], 1e-9) && Rel(s1[2], s2[2], 1e-9), "the 0 ply carries more fibre stress than the 45s, which carry the same as each other");
		// Force balance: the ply stresses in the laminate axes integrate to Nx.
		double nx = 0.0;
		for (size_t i = 0; i < stack.size(); ++i) {
			const Mat3 qb = RotatedStiffness(q, stack[i].angle);
			const double t = s.z_top[i] - s.z_bottom[i];
			nx += (A(qb, 0, 0) * st.strain[0] + A(qb, 0, 1) * st.strain[1] + A(qb, 0, 2) * st.strain[2]) * t;
		}
		Check(Rel(nx, 500.0, 1e-9), "the ply stresses integrate back to Nx (" + Fmt(nx) + " lb/in)");
	}

	// ---- [0/45]T, h = 0.01 in (RP-1351 Example 2) ----
	{
		Material m = Stack({0.0, 45.0}, false, lam);
		const std::vector<Ply> stack = ExpandedStack(m);
		Check(stack.size() == 2, "[0/45]T has two plies");
		const LaminateStiffness s = ComputeStiffness(stack);
		Check(Rel(A(s.b, 0, 0), 170.0, 5e-3) && Rel(A(s.b, 0, 1), -52.0, 1e-2) && Rel(A(s.b, 1, 1), -66.0, 1e-2)
			&& Rel(A(s.b, 0, 2), -59.0, 1e-2) && Rel(A(s.b, 1, 2), -59.0, 1e-2) && Rel(A(s.b, 2, 2), -52.0, 1e-2),
			"B of [0/45]T with the 0 ply at the outer face (RP-1351 Example 2): B11 " + Fmt(A(s.b, 0, 0)) + ", B12 " + Fmt(A(s.b, 0, 1)) + ", B22 " + Fmt(A(s.b, 1, 1)) + ", B16 " + Fmt(A(s.b, 0, 2)) + ", B66 " + Fmt(A(s.b, 2, 2)) + " lb");
		Check(Rel(A(s.d, 0, 0), 1.112, 5e-3) && Rel(A(s.d, 0, 1), 0.206, 5e-3) && Rel(A(s.d, 1, 1), 0.328, 5e-3)
			&& Rel(A(s.d, 0, 2), 0.196, 5e-3) && Rel(A(s.d, 2, 2), 0.257, 5e-3),
			"D of [0/45]T: D11 " + Fmt(A(s.d, 0, 0)) + ", D12 " + Fmt(A(s.d, 0, 1)) + ", D22 " + Fmt(A(s.d, 1, 1)) + ", D16 " + Fmt(A(s.d, 0, 2)) + ", D66 " + Fmt(A(s.d, 2, 2)) + " in-lb");
		const LaminateConstants c = EngineeringConstants(s);
		// Nettles carries B and D rounded to two or three figures into the
		// inverse, so his Ex is half a percent off the exact value.
		Check(Rel(c.ex, 5839000.0, 1e-2), "Ex of [0/45]T with the curvatures free is 5,839,000 psi (RP-1351 Example 2): " + Fmt(c.ex));
		Check(!c.symmetric, "[0/45]T is not symmetric");
		const LaminateStrainState st = Deformation(s, {100.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
		Check(std::fabs(st.curvature[0]) > 0.0, "an unsymmetric stack curves under in-plane tension (kappa_x " + Fmt(st.curvature[0]) + " 1/in per 100 lb/in)");
		Check(s.z_top[0] == 0.005 && Rel(s.z_bottom[0], 0.0, 1e-12) && Rel(s.z_bottom[1], -0.005, 1e-12), "the first listed ply lies at the outer face, z from +h/2 down");
	}

	// ---- a quasi-isotropic stack ----
	{
		Material m = Stack({0.0, 45.0, -45.0, 90.0}, true, lam, 0.057);
		const std::vector<Ply> stack = ExpandedStack(m);
		const LaminateStiffness s = ComputeStiffness(stack);
		const LaminateConstants c = EngineeringConstants(s);
		Check(stack.size() == 8 && Rel(c.thickness, 0.04, 1e-12), "[0/±45/90]s is eight plies, 0.04 in");
		Check(Rel(c.ex, c.ey, 1e-9), "quasi-isotropic: Ex = Ey (" + Fmt(c.ex) + " psi)");
		Check(Rel(c.gxy, c.ex / (2.0 * (1.0 + c.nuxy)), 1e-9), "quasi-isotropic: Gxy = Ex / 2(1 + nu) (" + Fmt(c.gxy) + ", nu " + Fmt(c.nuxy) + ")");
		// Symmetric and balanced, but its +45 and -45 plies sit at different
		// heights, so it bends and twists together (D16, D26 != 0): not an
		// orthotropic plate. A cross-ply is.
		Check(c.symmetric && c.balanced && !c.orthotropic, "...symmetric and balanced, but bend-twist coupled (D16 != 0), so not orthotropic");
		Check(EngineeringConstants(ComputeStiffness(ExpandedStack(Stack({0.0, 90.0}, true, lam)))).orthotropic, "[0/90]s is an orthotropic plate");
		Check(c.ex_flexural != c.ex && c.ex_flexural > 0.0, "the flexural modulus differs from the in-plane one (" + Fmt(c.ex_flexural) + " vs " + Fmt(c.ex) + ")");
		Check(Rel(c.density, 0.057, 1e-12), "the density is the plies' (all the same here)");
		// A [0/90]s cross-ply: balanced, not quasi-isotropic.
		Material cp = Stack({0.0, 90.0}, true, lam);
		const LaminateConstants cc = EngineeringConstants(ComputeStiffness(ExpandedStack(cp)));
		Check(Rel(cc.ex, cc.ey, 1e-9) && cc.gxy < 0.5 * cc.ex / (2.0 * (1.0 + cc.nuxy)) && cc.balanced && cc.symmetric, "[0/90]s: Ex = Ey but a low shear modulus; balanced and symmetric");
		// RefreshLaminate fills the four figures.
		RefreshLaminate(m);
		Check(Rel(m.density, 0.057, 1e-12) && Rel(m.youngs_modulus, c.ex, 1e-12) && Rel(m.poisson_ratio, c.nuxy, 1e-12) && m.allowable_stress == 0.0,
			"RefreshLaminate: density, Ex and nu_xy filled; no allowable without strengths");
		// An odd centre ply.
		Material odd = Stack({0.0, 90.0, 45.0}, true, lam);
		odd.symmetric_odd = true;
		const std::vector<Ply> os = ExpandedStack(odd);
		Check(os.size() == 5 && os[2].angle == 45.0 && os[3].angle == 90.0 && os[4].angle == 0.0, "[0/90/45~]s expands to 0/90/45/90/0");
		Check(EngineeringConstants(ComputeStiffness(os)).symmetric, "...and is symmetric");
	}

	// ---- Tsai-Wu ----
	{
		Lamina l = lam;
		l.t1 = 300e3; l.c1 = 200e3; l.t2 = 8e3; l.c2 = 30e3; l.s12 = 12e3;
		Check(l.HasStrengths(), "a lamina with all five strengths has strengths");
		Check(Rel(TsaiWuReserveFactor(l, {100e3, 0.0, 0.0}), 3.0, 1e-12), "Tsai-Wu under fibre tension alone: RF = T1 / sigma1 = 3");
		Check(Rel(TsaiWuReserveFactor(l, {-100e3, 0.0, 0.0}), 2.0, 1e-12), "...under fibre compression: RF = C1 / |sigma1| = 2");
		Check(Rel(TsaiWuReserveFactor(l, {0.0, 4e3, 0.0}), 2.0, 1e-12) && Rel(TsaiWuReserveFactor(l, {0.0, -10e3, 0.0}), 3.0, 1e-12), "...transverse tension T2 / sigma2, compression C2 / |sigma2|");
		Check(Rel(TsaiWuReserveFactor(l, {0.0, 0.0, 6e3}), 2.0, 1e-12) && Rel(TsaiWuReserveFactor(l, {0.0, 0.0, -6e3}), 2.0, 1e-12), "...shear: S12 / |tau12| either way");
		const double rf = TsaiWuReserveFactor(l, {100e3, 4e3, 3e3});
		Check(rf < 2.0 && rf > 1.0, "a combined stress has a lower reserve than any component alone (" + Fmt(rf) + ")");
		Check(std::isinf(TsaiWuReserveFactor(lam, {100e3, 0.0, 0.0})), "no strengths: an infinite reserve (no margin reported)");
		Check(std::isinf(TsaiWuReserveFactor(l, {0.0, 0.0, 0.0})), "no stress: an infinite reserve");
		// First-ply failure of a unidirectional stack in tension is T1.
		Material ud = Stack({0.0, 0.0, 0.0, 0.0}, false, l);
		Check(Rel(FirstPlyFailureTension(ExpandedStack(ud)), 300e3, 1e-9), "first-ply failure of [0]_4 under tension along the fibres is T1");
		Material ud90 = Stack({90.0, 90.0}, false, l);
		Check(Rel(FirstPlyFailureTension(ExpandedStack(ud90)), 8e3, 1e-9), "...of [90]_2 is T2");
		Material qi = Stack({0.0, 45.0, -45.0, 90.0}, true, l);
		const double fpf = FirstPlyFailureTension(ExpandedStack(qi));
		Check(fpf > 8e3 && fpf < 300e3, "a quasi-isotropic stack fails first between T2 and T1 (" + Fmt(fpf) + " psi: the 90 ply's transverse tension)");
		RefreshLaminate(qi);
		Check(Rel(qi.allowable_stress, fpf, 1e-12), "RefreshLaminate puts the first-ply-failure strength in the allowable");
		Material nos = Stack({0.0}, false, lam);
		Check(FirstPlyFailureTension(ExpandedStack(nos)) == 0.0, "a stack with a lamina lacking strengths has no first-ply-failure figure");
	}

	// ---- stacking notation ----
	{
		std::vector<double> a;
		bool sym = false, odd = false;
		std::string err;
		Check(ParseStackingNotation("[0/\xC2\xB1""45/90]s", &a, &sym, &odd, &err) && a == std::vector<double>{0.0, 45.0, -45.0, 90.0} && sym && !odd, "[0/±45/90]s parses: " + err);
		Check(ParseStackingNotation("[0/+-45/90]s", &a, &sym, &odd, &err) && a == std::vector<double>{0.0, 45.0, -45.0, 90.0} && sym, "+- stands for the plus-minus sign");
		Check(ParseStackingNotation("[0/90]2s", &a, &sym, &odd, &err) && a == std::vector<double>{0.0, 90.0, 0.0, 90.0} && sym, "[0/90]2s repeats the group");
		Check(ParseStackingNotation("[45/-45/0_2/90~]s", &a, &sym, &odd, &err) && a == std::vector<double>{45.0, -45.0, 0.0, 0.0, 90.0} && sym && odd, "_2 repeats a ply, ~ marks the centre ply");
		Check(ParseStackingNotation("0/90", &a, &sym, &odd, &err) && a == std::vector<double>{0.0, 90.0} && !sym, "bare angles without brackets");
		Check(ParseStackingNotation(" [ 0 / 30.5 ] ", &a, &sym, &odd, &err) && a.size() == 2 && a[1] == 30.5, "spaces and a fractional angle");
		Check(!ParseStackingNotation("[0/90", &a, &sym, &odd, &err) && !err.empty(), "an unclosed bracket is refused: " + err);
		Check(!ParseStackingNotation("[0/x]", &a, &sym, &odd, &err), "a non-number is refused: " + err);
		Check(!ParseStackingNotation("[0/90~]", &a, &sym, &odd, &err), "a centre mark without s is refused: " + err);
		Check(!ParseStackingNotation("[0~/90]s", &a, &sym, &odd, &err), "a centre mark not on the last ply is refused: " + err);
		Check(!ParseStackingNotation("", &a, &sym, &odd, &err), "an empty notation is refused");
		Check(!ParseStackingNotation("[0/200]", &a, &sym, &odd, &err), "an angle beyond 180 is refused");
		Material m = Stack({0.0, 45.0, -45.0, 90.0}, true, lam);
		Check(StackingNotation(m) == "[0/\xC2\xB1""45/90]s", "the notation of 0/45/-45/90 symmetric: " + StackingNotation(m));
		Material m2 = Stack({0.0, 0.0, 90.0}, false, lam);
		Check(StackingNotation(m2) == "[0_2/90]", "a run is written _2: " + StackingNotation(m2));
		Material m3 = Stack({45.0, -45.0, 0.0, 0.0, 90.0}, true, lam);
		m3.symmetric_odd = true;
		Check(StackingNotation(m3) == "[\xC2\xB1""45/0_2/90~]s", "the odd centre ply is written ~: " + StackingNotation(m3));
		Material m4 = Stack({-30.0, 30.0}, false, lam);
		Check(StackingNotation(m4) == "[\xE2\x88\x93""30]", "a -a/+a pair is written with the minus-plus sign: " + StackingNotation(m4));
		// Round trip through the notation.
		for (const Material* mm : {&m, &m2, &m3, &m4}) {
			std::vector<double> back;
			bool s2 = false, o2 = false;
			std::vector<double> want;
			for (const Ply& p : mm->plies) want.push_back(p.angle);
			Check(ParseStackingNotation(StackingNotation(*mm), &back, &s2, &o2, &err) && back == want && s2 == mm->symmetric && o2 == mm->symmetric_odd, "notation round trip: " + StackingNotation(*mm));
		}
	}

	// ---- the library file and the presets ----
	{
		Material lamina_row;
		lamina_row.name = "Test tape";
		lamina_row.category = "Lamina";
		lamina_row.density = 1570.0;
		lamina_row.source = "a test";
		Lamina l;
		l.ply_thickness = 1.25e-4; l.e1 = 140e9; l.e2 = 10e9; l.nu12 = 0.3; l.g12 = 5e9;
		l.t1 = 2000e6; l.c1 = 1200e6; l.t2 = 50e6; l.c2 = 200e6; l.s12 = 80e6;
		lamina_row.lamina = l;
		Check(lamina_row.IsLamina() && !lamina_row.IsLaminate() && !lamina_row.IsStructural() && lamina_row.HasDensity(), "a lamina row is a lamina: weighs, is not assignable");
		Check(MaterialProblem(lamina_row).empty(), "a complete lamina has no problem");
		Lamina bad = l;
		bad.c1 = 0.0;
		Check(!LaminaProblem(bad).empty(), "a lamina with four of five strengths is refused: " + LaminaProblem(bad));
		bad = l;
		bad.nu12 = 4.0;
		Check(!LaminaProblem(bad).empty(), "an inadmissible nu12 is refused: " + LaminaProblem(bad));
		const Material lam8 = MakeLaminate("Test [0/±45/90]s", "[0/+-45/90]s", lamina_row, "a test");
		Check(lam8.IsLaminate() && lam8.plies.size() == 4 && lam8.symmetric && lam8.category == "Laminate" && lam8.plies[1].lamina == "Test tape" && lam8.plies[1].props.e1 == 140e9,
			"MakeLaminate builds the plies from the notation and copies the lamina");
		Check(Rel(StackThickness(ExpandedStack(lam8)), 1e-3, 1e-12) && Rel(lam8.density, 1570.0, 1e-12) && lam8.youngs_modulus > 40e9 && lam8.youngs_modulus < 70e9 && lam8.allowable_stress > 0.0,
			"...1 mm thick, the tape's density, a quasi-isotropic modulus (" + Fmt(lam8.youngs_modulus) + " Pa) and a first-ply-failure allowable (" + Fmt(lam8.allowable_stress) + " Pa)");
		Check(lam8.IsStructural() && MaterialProblem(lam8).empty(), "a laminate is structural and has no problem");
		bool threw = false;
		try { MakeLaminate("x", "[0/", lamina_row, ""); } catch (const std::invalid_argument&) { threw = true; }
		Check(threw, "MakeLaminate refuses bad notation");
		// The file: a lamina and a laminate naming it, then the laminate
		// resolved on reading; the plies' figures recomputed.
		Material edited = lam8;
		edited.plies[0].thickness = 2e-4;   // a thicker outer ply
		RefreshLaminate(edited);
		Check(edited.youngs_modulus != lam8.youngs_modulus, "a thicker 0 ply changes the laminate's modulus");
		const std::string text = WriteMaterialsYaml({lamina_row, edited});
		Check(text.find("lamina:") != std::string::npos && text.find("plies:") != std::string::npos && text.find("stacking:") != std::string::npos, "the library file writes the lamina figures and the plies");
		const std::vector<Material> back = ReadMaterialsYaml(text);
		Check(back.size() == 2 && back[0].IsLamina() && SameMaterial(back[0], lamina_row), "the lamina round-trips");
		Check(back.size() == 2 && back[1].IsLaminate() && SameMaterial(back[1], edited) && back[1].plies[0].thickness == 2e-4 && back[1].plies[0].props.e1 == 140e9,
			"the laminate round-trips: plies resolved to the file's lamina, the edited ply thickness kept");
		Check(back.size() == 2 && Rel(back[1].youngs_modulus, edited.youngs_modulus, 1e-9) && Rel(back[1].density, edited.density, 1e-9), "...and its effective figures recomputed the same");
		Check(!SameMaterial(back[1], lam8), "the edited laminate is not the original");
		// A laminate alone, its lamina among the extra laminae (the presets).
		const std::string alone = WriteMaterialsYaml({lam8});
		threw = false;
		try { ReadMaterialsYaml(alone); } catch (const std::runtime_error&) { threw = true; }
		Check(threw, "a laminate naming a lamina the file does not have is refused");
		const std::vector<Material> extra = {lamina_row};
		const std::vector<Material> resolved = ReadMaterialsYaml(alone, &extra);
		Check(resolved.size() == 1 && SameMaterial(resolved[0], lam8), "...and resolves against the extra laminae");
		// The presets carry the MIL-HDBK-17-2F laminae and stacks of them.
		const Material* as4 = FindMaterialPreset("AS4/3502 carbon tape (MIL-HDBK-17-2F 4.2.8)");
		Check(as4 != nullptr && as4->IsLamina() && as4->density == 1570.0 && as4->lamina->e1 == 133.1e9 && as4->lamina->t1 == 1413e6 && as4->lamina->ply_thickness == 1.397e-4 && as4->category == "Lamina" && as4->source.find("MIL-HDBK-17-2F") != std::string::npos,
			"the AS4/3502 tape preset carries the handbook's figures and cites it");
		const Material* cloth = FindMaterialPreset("T300/934 carbon plain weave (MIL-HDBK-17-2F 4.2.6)");
		Check(cloth != nullptr && cloth->IsLamina() && cloth->lamina->IsWoven() && cloth->lamina->locking_angle == 35.0, "the T300/934 plain weave is a woven lamina with a locking angle");
		const Material* qi = FindMaterialPreset("CFRP AS4/3502 tape [0/\xC2\xB1""45/90]s");
		Check(qi != nullptr && qi->IsLaminate() && qi->symmetric && qi->plies.size() == 4 && qi->plies[0].props.e1 == 133.1e9 && qi->plies[0].density == 1570.0, "the quasi-isotropic AS4/3502 preset resolved its plies to the tape");
		Check(qi != nullptr && Rel(qi->density, 1570.0, 1e-12) && Rel(StackThickness(ExpandedStack(*qi)), 8 * 1.397e-4, 1e-12) && qi->youngs_modulus > 45e9 && qi->youngs_modulus < 60e9 && qi->allowable_stress > 100e6,
			"...1.12 mm, the tape's density, Ex " + Fmt(qi ? qi->youngs_modulus : 0.0) + " Pa, first-ply-failure allowable " + Fmt(qi ? qi->allowable_stress : 0.0) + " Pa");
		Check(qi != nullptr && IsMaterialPreset(*qi), "a copy of a laminate preset is still the preset");
		int laminae = 0, laminates = 0;
		for (const Material& m : MaterialPresets()) { laminae += m.IsLamina() ? 1 : 0; laminates += m.IsLaminate() ? 1 : 0; }
		Check(laminae >= 4 && laminates >= 4, "at least four laminae and four laminates among the presets");
		bool all_ok = true;
		for (const Material& m : MaterialPresets()) if (m.IsLamina() || m.IsLaminate()) all_ok = all_ok && MaterialProblem(m).empty() && !m.source.empty();
		Check(all_ok, "every lamina and laminate preset is complete and sourced");
	}

	std::printf("RESULT: %d failure(s)\n", g_failures);
	return g_failures == 0 ? 0 : 1;
}
