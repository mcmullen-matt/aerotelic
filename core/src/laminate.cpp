#include "aerotelic/laminate.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace aerotelic {

namespace {

constexpr double kPi = 3.14159265358979323846;

double& At(Mat3& m, int i, int j) { return m[i * 3 + j]; }
double At(const Mat3& m, int i, int j) { return m[i * 3 + j]; }

// The 6x6 [A B; B D] inverted by Gauss-Jordan with partial pivoting.
// Returns false when singular.
bool Invert6(const double in[6][6], double out[6][6]) {
	double a[6][12];
	for (int i = 0; i < 6; ++i) {
		for (int j = 0; j < 6; ++j) { a[i][j] = in[i][j]; a[i][j + 6] = i == j ? 1.0 : 0.0; }
	}
	for (int c = 0; c < 6; ++c) {
		int p = c;
		for (int r = c + 1; r < 6; ++r) if (std::fabs(a[r][c]) > std::fabs(a[p][c])) p = r;
		if (std::fabs(a[p][c]) < 1e-300) return false;
		if (p != c) for (int j = 0; j < 12; ++j) std::swap(a[p][j], a[c][j]);
		const double inv = 1.0 / a[c][c];
		for (int j = 0; j < 12; ++j) a[c][j] *= inv;
		for (int r = 0; r < 6; ++r) {
			if (r == c) continue;
			const double f = a[r][c];
			if (f == 0.0) continue;
			for (int j = 0; j < 12; ++j) a[r][j] -= f * a[c][j];
		}
	}
	for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) out[i][j] = a[i][j + 6];
	return true;
}

void FullCompliance(const LaminateStiffness& s, double inv[6][6], bool* ok) {
	double abd[6][6];
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			abd[i][j] = At(s.a, i, j);
			abd[i][j + 3] = At(s.b, i, j);
			abd[i + 3][j] = At(s.b, i, j);
			abd[i + 3][j + 3] = At(s.d, i, j);
		}
	}
	*ok = Invert6(abd, inv);
}

double Norm(const Mat3& m) {
	double n = 0.0;
	for (double v : m) n = std::max(n, std::fabs(v));
	return n;
}

std::string Trim(const std::string& s) {
	size_t a = s.find_first_not_of(" \t\r\n");
	if (a == std::string::npos) return std::string();
	size_t b = s.find_last_not_of(" \t\r\n");
	return s.substr(a, b - a + 1);
}

std::string FormatAngle(double a) {
	std::ostringstream os;
	if (std::fabs(a - std::round(a)) < 1e-9) os << static_cast<long long>(std::llround(a));
	else os << a;
	return os.str();
}

} // namespace

// ---------------------------------------------------------------------------
// The stack
// ---------------------------------------------------------------------------

std::vector<Ply> ExpandedStack(const Material& m) {
	std::vector<Ply> stack;
	for (const Ply& p : m.plies) {
		Ply q = p;
		q.thickness = p.Thickness();
		stack.push_back(q);
	}
	if (m.symmetric && !m.plies.empty()) {
		const size_t n = m.plies.size();
		const size_t first_mirrored = m.symmetric_odd ? n - 1 : n;   // the centre ply is not repeated
		for (size_t i = first_mirrored; i-- > 0;) {
			Ply q = m.plies[i];
			q.thickness = m.plies[i].Thickness();
			stack.push_back(q);
		}
	}
	return stack;
}

double StackThickness(const std::vector<Ply>& stack) {
	double h = 0.0;
	for (const Ply& p : stack) h += p.Thickness();
	return h;
}

double StackArealMass(const std::vector<Ply>& stack) {
	double m = 0.0;
	for (const Ply& p : stack) m += p.density * p.Thickness();
	return m;
}

// ---------------------------------------------------------------------------
// Stiffness
// ---------------------------------------------------------------------------

Mat3 ReducedStiffness(const Lamina& l) {
	Mat3 q{};
	const double nu21 = l.e1 > 0.0 ? l.nu12 * l.e2 / l.e1 : 0.0;
	const double den = 1.0 - l.nu12 * nu21;
	if (!(den > 0.0)) return q;
	At(q, 0, 0) = l.e1 / den;
	At(q, 1, 1) = l.e2 / den;
	At(q, 0, 1) = At(q, 1, 0) = l.nu12 * l.e2 / den;
	At(q, 2, 2) = l.g12;
	return q;
}

Mat3 RotatedStiffness(const Mat3& q, double angle_deg) {
	// Jones eq. 2.84 (Q-bar for a ply at theta from the laminate x axis).
	const double t = angle_deg * kPi / 180.0;
	const double m = std::cos(t), n = std::sin(t);
	const double m2 = m * m, n2 = n * n, m4 = m2 * m2, n4 = n2 * n2, m3 = m2 * m, n3 = n2 * n;
	const double q11 = At(q, 0, 0), q12 = At(q, 0, 1), q22 = At(q, 1, 1), q66 = At(q, 2, 2);
	Mat3 r{};
	At(r, 0, 0) = q11 * m4 + 2.0 * (q12 + 2.0 * q66) * m2 * n2 + q22 * n4;
	At(r, 0, 1) = (q11 + q22 - 4.0 * q66) * m2 * n2 + q12 * (m4 + n4);
	At(r, 1, 1) = q11 * n4 + 2.0 * (q12 + 2.0 * q66) * m2 * n2 + q22 * m4;
	At(r, 0, 2) = (q11 - q12 - 2.0 * q66) * m3 * n + (q12 - q22 + 2.0 * q66) * m * n3;
	At(r, 1, 2) = (q11 - q12 - 2.0 * q66) * m * n3 + (q12 - q22 + 2.0 * q66) * m3 * n;
	At(r, 2, 2) = (q11 + q22 - 2.0 * q12 - 2.0 * q66) * m2 * n2 + q66 * (m4 + n4);
	At(r, 1, 0) = At(r, 0, 1);
	At(r, 2, 0) = At(r, 0, 2);
	At(r, 2, 1) = At(r, 1, 2);
	return r;
}

LaminateStiffness ComputeStiffness(const std::vector<Ply>& stack) {
	LaminateStiffness s;
	s.thickness = StackThickness(stack);
	s.areal_mass = StackArealMass(stack);
	double z = 0.5 * s.thickness;   // the outer face
	for (const Ply& p : stack) {
		const double t = p.Thickness();
		const double z_top = z, z_bot = z - t;
		s.z_top.push_back(z_top);
		s.z_bottom.push_back(z_bot);
		const Mat3 qb = RotatedStiffness(ReducedStiffness(p.props), p.angle);
		const double d1 = z_top - z_bot;
		const double d2 = 0.5 * (z_top * z_top - z_bot * z_bot);
		const double d3 = (z_top * z_top * z_top - z_bot * z_bot * z_bot) / 3.0;
		for (int k = 0; k < 9; ++k) {
			s.a[k] += qb[k] * d1;
			s.b[k] += qb[k] * d2;
			s.d[k] += qb[k] * d3;
		}
		z = z_bot;
	}
	// Round-off in B of a symmetric stack: anything below a billionth of
	// A h is zero.
	const double tiny = 1e-12 * Norm(s.a) * std::max(s.thickness, 1e-300);
	for (double& v : s.b) if (std::fabs(v) < tiny) v = 0.0;
	return s;
}

LaminateConstants EngineeringConstants(const LaminateStiffness& s) {
	LaminateConstants c;
	c.thickness = s.thickness;
	c.density = s.thickness > 0.0 ? s.areal_mass / s.thickness : 0.0;
	const double h = s.thickness;
	if (!(h > 0.0)) return c;
	const double a_scale = Norm(s.a);
	c.symmetric = Norm(s.b) <= 1e-9 * a_scale * h;
	c.balanced = std::fabs(At(s.a, 0, 2)) <= 1e-9 * a_scale && std::fabs(At(s.a, 1, 2)) <= 1e-9 * a_scale;
	const double d_scale = Norm(s.d);
	c.orthotropic = c.symmetric && c.balanced && std::fabs(At(s.d, 0, 2)) <= 1e-9 * d_scale && std::fabs(At(s.d, 1, 2)) <= 1e-9 * d_scale;
	double inv[6][6];
	bool ok = false;
	FullCompliance(s, inv, &ok);
	if (!ok) return c;
	// in-plane, curvatures free: eps = a' N; moduli per unit thickness
	const double a11 = inv[0][0], a22 = inv[1][1], a66 = inv[2][2], a12 = inv[0][1];
	if (a11 > 0.0) { c.ex = 1.0 / (h * a11); c.nuxy = -a12 / a11; }
	if (a22 > 0.0) { c.ey = 1.0 / (h * a22); c.nuyx = -a12 / a22; }
	if (a66 > 0.0) c.gxy = 1.0 / (h * a66);
	const double d11 = inv[3][3], d22 = inv[4][4];
	const double h3 = h * h * h;
	if (d11 > 0.0) c.ex_flexural = 12.0 / (h3 * d11);
	if (d22 > 0.0) c.ey_flexural = 12.0 / (h3 * d22);
	return c;
}

LaminateStrainState Deformation(const LaminateStiffness& s, const std::array<double, 3>& n, const std::array<double, 3>& m) {
	LaminateStrainState st;
	double inv[6][6];
	bool ok = false;
	FullCompliance(s, inv, &ok);
	if (!ok) return st;
	const double rhs[6] = {n[0], n[1], n[2], m[0], m[1], m[2]};
	for (int i = 0; i < 6; ++i) {
		double v = 0.0;
		for (int j = 0; j < 6; ++j) v += inv[i][j] * rhs[j];
		if (i < 3) st.strain[i] = v; else st.curvature[i - 3] = v;
	}
	return st;
}

std::array<double, 3> PlyStress(const Mat3& q, double angle_deg, const LaminateStrainState& st, double z) {
	// Strain at z in the laminate axes, stress there from Q-bar, rotated
	// into the ply's axes (Jones eq. 2.69 for stresses).
	const Mat3 qb = RotatedStiffness(q, angle_deg);
	double eps[3];
	for (int i = 0; i < 3; ++i) eps[i] = st.strain[i] + z * st.curvature[i];
	double sx[3] = {0.0, 0.0, 0.0};
	for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) sx[i] += At(qb, i, j) * eps[j];
	const double t = angle_deg * kPi / 180.0;
	const double m = std::cos(t), n = std::sin(t);
	std::array<double, 3> s;
	s[0] = m * m * sx[0] + n * n * sx[1] + 2.0 * m * n * sx[2];
	s[1] = n * n * sx[0] + m * m * sx[1] - 2.0 * m * n * sx[2];
	s[2] = -m * n * sx[0] + m * n * sx[1] + (m * m - n * n) * sx[2];
	return s;
}

double TsaiWuReserveFactor(const Lamina& l, const std::array<double, 3>& stress) {
	if (!l.HasStrengths()) return std::numeric_limits<double>::infinity();
	const double f1 = 1.0 / l.t1 - 1.0 / l.c1, f2 = 1.0 / l.t2 - 1.0 / l.c2;
	const double f11 = 1.0 / (l.t1 * l.c1), f22 = 1.0 / (l.t2 * l.c2), f66 = 1.0 / (l.s12 * l.s12);
	const double f12 = -0.5 * std::sqrt(f11 * f22);
	const double s1 = stress[0], s2 = stress[1], s6 = stress[2];
	// F(lambda s) = a lambda^2 + b lambda = 1
	const double a = f11 * s1 * s1 + f22 * s2 * s2 + f66 * s6 * s6 + 2.0 * f12 * s1 * s2;
	const double b = f1 * s1 + f2 * s2;
	if (a <= 0.0) {
		if (b <= 0.0) return std::numeric_limits<double>::infinity();
		return 1.0 / b;
	}
	return (-b + std::sqrt(b * b + 4.0 * a)) / (2.0 * a);
}

double FirstPlyFailureTension(const std::vector<Ply>& stack) {
	if (stack.empty()) return 0.0;
	for (const Ply& p : stack) if (!p.props.HasStrengths()) return 0.0;
	const LaminateStiffness s = ComputeStiffness(stack);
	if (!(s.thickness > 0.0)) return 0.0;
	// A unit resultant N_x = 1 N/m; every ply's reserve factor at its two
	// faces; the stress at failure is RF_min / h.
	const LaminateStrainState st = Deformation(s, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
	double rf_min = std::numeric_limits<double>::infinity();
	for (size_t i = 0; i < stack.size(); ++i) {
		const Mat3 q = ReducedStiffness(stack[i].props);
		for (double z : {s.z_top[i], s.z_bottom[i]}) {
			const double rf = TsaiWuReserveFactor(stack[i].props, PlyStress(q, stack[i].angle, st, z));
			rf_min = std::min(rf_min, rf);
		}
	}
	if (!std::isfinite(rf_min)) return 0.0;
	return rf_min / s.thickness;
}

LaminateConstants RefreshLaminate(Material& m) {
	LaminateConstants c;
	if (!m.IsLaminate()) return c;
	const std::vector<Ply> stack = ExpandedStack(m);
	const LaminateStiffness s = ComputeStiffness(stack);
	c = EngineeringConstants(s);
	m.density = c.density;
	m.youngs_modulus = c.ex;
	m.poisson_ratio = std::isfinite(c.nuxy) ? std::clamp(c.nuxy, 0.0, 0.499) : 0.0;
	m.allowable_stress = FirstPlyFailureTension(stack);
	if (m.category.empty()) m.category = "Laminate";
	return c;
}

// ---------------------------------------------------------------------------
// Notation
// ---------------------------------------------------------------------------

bool ParseStackingNotation(const std::string& text_in, std::vector<double>* angles, bool* symmetric, bool* symmetric_odd, std::string* error) {
	auto fail = [&](const std::string& why) { if (error) *error = why; return false; };
	std::string text;
	for (char ch : text_in) if (!std::isspace(static_cast<unsigned char>(ch))) text.push_back(ch);
	// "+-" as typed stands for the plus-minus sign; the sign itself may
	// arrive as UTF-8 (0xC2 0xB1).
	for (size_t p; (p = text.find("+-")) != std::string::npos;) text.replace(p, 2, "\xC2\xB1");
	for (size_t p; (p = text.find("-+")) != std::string::npos;) text.replace(p, 2, "\xE2\x88\x93");
	if (text.empty()) return fail("no stacking given");
	// The group: [ ... ] with a trailing repeat and/or s, or bare angles.
	std::string body = text, tail;
	const size_t open = text.find_first_of("[(");
	if (open != std::string::npos) {
		const size_t close = text.find_first_of("])", open + 1);
		if (close == std::string::npos) return fail("the bracket is not closed");
		if (open != 0) return fail("text before the opening bracket");
		body = text.substr(open + 1, close - open - 1);
		tail = text.substr(close + 1);
	}
	int group_repeat = 1;
	bool sym = false;
	{
		size_t i = 0;
		while (i < tail.size() && std::isdigit(static_cast<unsigned char>(tail[i]))) ++i;
		if (i > 0) group_repeat = std::stoi(tail.substr(0, i));
		if (group_repeat < 1 || group_repeat > 64) return fail("the group repeat must be 1 to 64");
		std::string rest = tail.substr(i);
		if (rest == "s" || rest == "S") sym = true;
		else if (!rest.empty()) return fail("after the bracket only a repeat and 's' are understood, not '" + rest + "'");
	}
	if (body.empty()) return fail("no angles between the brackets");
	std::vector<double> out;
	bool odd = false;
	std::vector<std::string> items;
	{
		std::string cur;
		for (char ch : body) {
			if (ch == '/' || ch == ',') { items.push_back(cur); cur.clear(); }
			else cur.push_back(ch);
		}
		items.push_back(cur);
	}
	for (size_t k = 0; k < items.size(); ++k) {
		std::string item = items[k];
		if (item.empty()) return fail("an empty entry between separators");
		bool plus_minus = false, minus_plus = false;
		if (item.rfind("\xC2\xB1", 0) == 0) { plus_minus = true; item = item.substr(2); }
		else if (item.rfind("\xE2\x88\x93", 0) == 0) { minus_plus = true; item = item.substr(3); }
		bool centre = false;
		if (!item.empty() && item.back() == '~') { centre = true; item.pop_back(); }
		int repeat = 1;
		{
			const size_t u = item.find_first_of("_x");
			if (u != std::string::npos) {
				const std::string r = item.substr(u + 1);
				if (r.empty() || r.find_first_not_of("0123456789") != std::string::npos) return fail("the repeat after '" + item.substr(0, u) + "' must be a whole number");
				repeat = std::stoi(r);
				if (repeat < 1 || repeat > 64) return fail("a ply repeat must be 1 to 64");
				item = item.substr(0, u);
			}
		}
		if (item.empty()) return fail("an entry without an angle");
		char* end = nullptr;
		const double a = std::strtod(item.c_str(), &end);
		if (end == item.c_str() || *end != '\0' || !std::isfinite(a)) return fail("'" + item + "' is not an angle");
		if (a < -180.0 || a > 180.0) return fail("angles are given between -180 and 180 degrees");
		if (centre) {
			if (!sym) return fail("the centre-ply mark ~ needs a symmetric stack ([...]s)");
			if (k + 1 != items.size()) return fail("the centre-ply mark ~ goes on the last angle only");
			if (plus_minus || minus_plus || repeat != 1) return fail("the centre ply is one ply: no pair, no repeat");
			odd = true;
		}
		for (int r = 0; r < repeat; ++r) {
			if (plus_minus) { out.push_back(a); out.push_back(-a); }
			else if (minus_plus) { out.push_back(-a); out.push_back(a); }
			else out.push_back(a);
		}
	}
	if (group_repeat > 1) {
		if (odd) return fail("a group repeat cannot carry a centre-ply mark");
		std::vector<double> once = out;
		for (int r = 1; r < group_repeat; ++r) out.insert(out.end(), once.begin(), once.end());
	}
	if (out.size() > 512) return fail("more than 512 plies");
	if (angles) *angles = out;
	if (symmetric) *symmetric = sym;
	if (symmetric_odd) *symmetric_odd = odd;
	if (error) error->clear();
	return true;
}

std::string StackingNotation(const Material& m) {
	if (m.plies.empty()) return std::string();
	std::string s = "[";
	const size_t n = m.plies.size();
	size_t i = 0;
	bool first = true;
	while (i < n) {
		const double a = m.plies[i].angle;
		const bool last_is_centre = m.symmetric && m.symmetric_odd && i + 1 == n;
		if (!first) s += "/";
		first = false;
		// a +a/-a pair
		if (!last_is_centre && a != 0.0 && i + 1 < n && m.plies[i + 1].angle == -a && !(m.symmetric && m.symmetric_odd && i + 2 == n)) {
			s += (a > 0.0 ? "\xC2\xB1" : "\xE2\x88\x93") + FormatAngle(std::fabs(a));
			i += 2;
			continue;
		}
		// a run of equal angles
		size_t j = i + 1;
		while (j < n && m.plies[j].angle == a && !(m.symmetric && m.symmetric_odd && j + 1 == n)) ++j;
		s += FormatAngle(a);
		if (j - i > 1) s += "_" + std::to_string(j - i);
		if (last_is_centre) s += "~";
		i = j;
	}
	s += "]";
	if (m.symmetric) s += "s";
	return s;
}

Material MakeLaminate(const std::string& name, const std::string& notation, const Material& lamina_row, const std::string& source) {
	if (!lamina_row.lamina) throw std::invalid_argument("'" + lamina_row.name + "' is not a lamina");
	std::vector<double> angles;
	bool sym = false, odd = false;
	std::string err;
	if (!ParseStackingNotation(notation, &angles, &sym, &odd, &err)) throw std::invalid_argument("stacking '" + notation + "': " + err);
	Material m;
	m.name = name;
	m.category = "Laminate";
	m.source = source;
	m.symmetric = sym;
	m.symmetric_odd = odd;
	for (double a : angles) {
		Ply p;
		p.angle = a;
		p.thickness = 0.0;
		p.lamina = lamina_row.name;
		p.density = lamina_row.density;
		p.props = *lamina_row.lamina;
		m.plies.push_back(p);
	}
	RefreshLaminate(m);
	return m;
}

std::string LaminateProblem(const Material& m) {
	if (m.plies.empty()) return "the laminate has no plies";
	for (size_t i = 0; i < m.plies.size(); ++i) {
		const Ply& p = m.plies[i];
		const std::string who = "ply " + std::to_string(i + 1);
		if (p.lamina.empty()) return who + " names no lamina";
		if (!std::isfinite(p.angle)) return who + ": the angle is not a number";
		if (!(p.density > 0.0)) return who + " (" + p.lamina + "): the lamina has no density";
		const std::string lp = LaminaProblem(p.props);
		if (!lp.empty()) return who + " (" + p.lamina + "): " + lp;
		if (!(p.Thickness() > 0.0)) return who + " (" + p.lamina + "): no thickness";
	}
	if (m.symmetric_odd && !m.symmetric) return "an odd centre ply needs a symmetric stack";
	return std::string();
}

} // namespace aerotelic
