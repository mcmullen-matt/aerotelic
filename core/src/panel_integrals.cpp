#include "aerotelic/panel_integrals.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace aerotelic::panel {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kFourPi = 4.0 * kPi;

// ---------------------------------------------------------------------------
// Sides of Σ (figure D.3)
// ---------------------------------------------------------------------------

struct Side {
	double nu_xi, nu_eta; // unit outer normal ν
	double abar;          // signed distance from (x, y) to the side's line along ν
	double l1, l2;        // tangent coordinates of the ends from the foot of that perpendicular
	double g2, g;         // g² = ā² + h²
	bool valid;
};

// Point-in-polygon by crossing number, for the (possibly non-convex) Σ.
bool InsideSigma(const LocalPanel& p, double x, double y) {
	bool inside = false;
	for (int i = 0, j = 3; i < 4; j = i++) {
		double xi = p.xi[i], yi = p.eta[i], xj = p.xi[j], yj = p.eta[j];
		if ((yi > y) != (yj > y)) {
			double xc = xj + (y - yj) * (xi - xj) / (yi - yj);
			if (x < xc) inside = !inside;
		}
	}
	return inside;
}

// Minimum distance in the plane from (x, y) to the perimeter of Σ.
double PerimeterDistance(const LocalPanel& p, double x, double y) {
	double best = 1e300;
	for (int i = 0; i < 4; ++i) {
		int j = (i + 1) % 4;
		double ax = p.xi[i], ay = p.eta[i], bx = p.xi[j], by = p.eta[j];
		double dx = bx - ax, dy = by - ay;
		double len2 = dx * dx + dy * dy;
		double t = len2 > 0.0 ? std::clamp(((x - ax) * dx + (y - ay) * dy) / len2, 0.0, 1.0) : 0.0;
		double px = ax + t * dx - x, py = ay + t * dy - y;
		best = std::min(best, std::sqrt(px * px + py * py));
	}
	return best;
}

std::array<Side, 4> MakeSides(const LocalPanel& p, double x, double y, double h) {
	std::array<Side, 4> sides{};
	for (int i = 0; i < 4; ++i) {
		int j = (i + 1) % 4;
		Side& s = sides[i];
		double tx = p.xi[j] - p.xi[i], ty = p.eta[j] - p.eta[i];
		double len = std::sqrt(tx * tx + ty * ty);
		if (len < 1e-14) { s.valid = false; continue; }
		tx /= len; ty /= len;
		// t = (-ν_η, ν_ξ)  <=>  ν = (t_η, -t_ξ): the outer normal of a CCW polygon.
		s.nu_xi = ty; s.nu_eta = -tx;
		double rx = p.xi[i] - x, ry = p.eta[i] - y;
		s.abar = rx * s.nu_xi + ry * s.nu_eta;
		s.l1 = rx * tx + ry * ty;
		s.l2 = s.l1 + len;
		s.g2 = s.abar * s.abar + h * h;
		s.g = std::sqrt(s.g2);
		s.valid = true;
	}
	return sides;
}

// ---------------------------------------------------------------------------
// F integrals of one side (D.40), procedures 4 and 5
// ---------------------------------------------------------------------------

struct SideIntegrals {
	// F(1, 1, K) for odd K = 1, 3, ..., indexed (K + 1) / 2 - 1.
	std::vector<double> f11;
	// F(M, N, 1), 1-based, M + N - 1 <= mxq.
	double fmn1[kMaxQ + 2][kMaxQ + 2] = {};
	// F(1, N, K), N 1-based, K odd >= 3 indexed (K + 1) / 2 - 1 (index 0 = K = 1 mirrors fmn1).
	double f1nk[kMaxQ + 2][(kMaxK + 1) / 2 + 1] = {};
};

// E(M, N, K) = [(ξ - x)^(M-1) (η - y)^(N-1) / ρ^K] between the side's ends (D.58).
double EFunc(const Side& s, int M, int N, int K) {
	auto at = [&](double l) {
		double dx = s.abar * s.nu_xi - l * s.nu_eta;
		double dy = s.abar * s.nu_eta + l * s.nu_xi;
		double rho = std::sqrt(l * l + s.g2);
		return std::pow(dx, M - 1) * std::pow(dy, N - 1) * std::pow(rho, -K);
	};
	return at(s.l2) - at(s.l1);
}

// F(1, 1, 1) by (D.60), three forms against cancellation.
double F111(const Side& s) {
	double g2 = std::max(s.g2, 1e-300);
	double r1 = std::sqrt(s.l1 * s.l1 + g2), r2 = std::sqrt(s.l2 * s.l2 + g2);
	if (s.l1 >= 0.0 && s.l2 >= 0.0) return std::log((r2 + s.l2) / (r1 + s.l1));
	if (s.l1 < 0.0 && s.l2 < 0.0) return std::log((r1 - s.l1) / (r2 - s.l2));
	return std::log((r1 - s.l1) * (r2 + s.l2) / g2);
}

SideIntegrals ComputeSideIntegrals(const Side& s, double h, int mxq, int mxk, int mxfk, double dF, const IntegralOptions& opt) {
	SideIntegrals out;
	const int nk = (mxfk + 1) / 2; // F(1,1,K) for K = 1..mxfk odd
	out.f11.assign(nk + opt.nfk / 2 + 2, 0.0);
	const bool near_perimeter = s.g < opt.delta_g * dF;
	out.f11[0] = F111(s);
	if (!near_perimeter) {
		// Procedure 4, step 2 (D.61).
		for (int K = 3; K <= mxfk; K += 2) {
			int k = (K + 1) / 2 - 1;
			out.f11[k] = ((K - 3) * out.f11[k - 1] - s.nu_eta * EFunc(s, 2, 1, K - 2) + s.nu_xi * EFunc(s, 1, 2, K - 2)) / (s.g2 * (K - 2));
		}
	} else {
		// Procedure 5: the reverse recursion (D.68)-(D.69) from a vanishing
		// high-K seed, which is accurate because F(1,1,K) decays like g^(1-K)
		// only when g is small and the point is off the segment... the seed
		// error dies out through the (K-3)/(g²(K-2)) factors.
		const int ktop = mxfk + opt.nfk; // odd
		std::vector<double> f(ktop / 2 + 2, 0.0);
		f[(ktop + 1) / 2 - 1] = 0.0;
		for (int K = ktop; K >= 5; K -= 2) {
			int k = (K + 1) / 2 - 1;
			f[k - 1] = (s.g2 * (K - 2) * f[k] + s.nu_eta * EFunc(s, 2, 1, K - 2) - s.nu_xi * EFunc(s, 1, 2, K - 2)) / (K - 3);
		}
		for (int K = 3; K <= mxfk; K += 2) out.f11[(K + 1) / 2 - 1] = f[(K + 1) / 2 - 1];
	}

	// Step 3: F(M, N, 1) by (D.62)-(D.65).
	double (&F)[kMaxQ + 2][kMaxQ + 2] = out.fmn1;
	F[1][1] = out.f11[0];
	const double h2 = h * h;
	if (std::fabs(s.nu_eta) <= std::fabs(s.nu_xi)) {
		for (int N = 2; N <= mxq; ++N) {
			double prev2 = N >= 3 ? F[1][N - 2] : 0.0;
			F[1][N] = ((2 * N - 3) * s.abar * s.nu_eta * F[1][N - 1] - (N - 2) * (s.abar * s.abar + s.nu_xi * s.nu_xi * h2) * prev2 + s.nu_xi * EFunc(s, 1, N - 1, -1)) / (N - 1);
		}
		for (int M = 2; M <= mxq; ++M)
			for (int N = 1; N <= mxq - M + 1; ++N) F[M][N] = (-s.nu_eta * F[M - 1][N + 1] + s.abar * F[M - 1][N]) / s.nu_xi;
	} else {
		for (int M = 2; M <= mxq; ++M) {
			double prev2 = M >= 3 ? F[M - 2][1] : 0.0;
			F[M][1] = ((2 * M - 3) * s.abar * s.nu_xi * F[M - 1][1] - (M - 2) * (s.abar * s.abar + s.nu_eta * s.nu_eta * h2) * prev2 - s.nu_eta * EFunc(s, M - 1, 1, -1)) / (M - 1);
		}
		for (int N = 2; N <= mxq; ++N)
			for (int M = 1; M <= mxq - N + 1; ++M) F[M][N] = (-s.nu_xi * F[M + 1][N - 1] + s.abar * F[M][N - 1]) / s.nu_eta;
	}

	// Steps 4, 5: F(1, N, K), K = 3 .. mxk - 2 (D.66), (D.67).
	for (int N = 1; N <= mxq; ++N) out.f1nk[N][0] = F[1][N];
	for (int K = 3; K <= mxk - 2; K += 2) {
		int k = (K + 1) / 2 - 1;
		out.f1nk[1][k] = out.f11[k];
		if (mxq >= 2) out.f1nk[2][k] = s.nu_eta * s.abar * out.f11[k] - s.nu_xi / (K - 2) * EFunc(s, 1, 1, K - 2);
		for (int N = 3; N <= mxq; ++N) {
			out.f1nk[N][k] = 2.0 * s.abar * s.nu_eta * out.f1nk[N - 1][k] - (s.abar * s.abar + s.nu_xi * s.nu_xi * h2) * out.f1nk[N - 2][k] + s.nu_xi * s.nu_xi * out.f1nk[N - 2][k - 1];
		}
	}
	return out;
}

// ---------------------------------------------------------------------------
// H integrals (D.35), procedures 1-3
// ---------------------------------------------------------------------------

struct HTable {
	// H(M, N, K), 1-based M, N, K odd indexed (K + 1) / 2 - 1.
	double H[kMaxQ + 2][kMaxQ + 2][(kMaxK + 1) / 2] = {};
	bool star = false; // the regular part H* (D.51): the point is on/near the plane inside Σ
	double h = 0.0;
	int sign_h = 0;    // sign of h, 0 when on the plane (average values)
	double at(int M, int N, int K) const { return H[M][N][(K + 1) / 2 - 1]; }
};

// ν(M, N, K) of (D.52).
double NuMNK(int M, int N, int K) {
	if (M % 2 == 0 || N % 2 == 0) return 0.0;
	auto dfact = [](int m) { // 1 for m = 1, else 1·1·3·5···(m-2)
		double r = 1.0;
		for (int i = 1; i <= m - 2; i += 2) r *= i;
		return r;
	};
	double num = dfact(M) * dfact(N);
	double den = 1.0;
	for (int i = 1; i <= (M + N) / 2; ++i) den *= (K - 2 * i);
	return num / den;
}

HTable ComputeH(const LocalPanel& p, double x, double y, double h, int mxq, int mxk, const IntegralOptions& opt) {
	HTable T;
	T.h = h;
	const double dH = PerimeterDistance(p, x, y);
	const bool inside = InsideSigma(p, x, y);
	const bool near_plane = std::fabs(h) < opt.delta_h * dH;
	T.star = near_plane && inside;
	T.sign_h = std::fabs(h) < opt.delta_0 * dH ? 0 : (h > 0.0 ? 1 : -1);
	const int mxfk = near_plane ? opt.nhk + mxk - 2 : mxk - 2;
	const double dF = std::sqrt(dH * dH + h * h);
	const double h2 = h * h;

	auto sides = MakeSides(p, x, y, h);
	std::array<SideIntegrals, 4> SI;
	for (int i = 0; i < 4; ++i) if (sides[i].valid) SI[i] = ComputeSideIntegrals(sides[i], h, mxq, mxk, mxfk, dF, opt);

	auto sum_a_f11 = [&](int K) { // Σ ā F(1,1,K)
		double s = 0.0;
		for (int i = 0; i < 4; ++i) if (sides[i].valid) s += sides[i].abar * SI[i].f11[(K + 1) / 2 - 1];
		return s;
	};
	// H(1, 1, K) for K = 1 .. mxk, plus the high-K ladder of procedure 2.
	std::vector<double> h11((opt.nhk + mxk + 1) / 2 + 1, 0.0);
	auto h11at = [&](int K) -> double& { return h11[(K + 1) / 2 - 1]; };
	if (!near_plane) {
		// Procedure 1, steps 1-2: (D.41), (D.42).
		double atan_sum = 0.0;
		for (const Side& s : sides) {
			if (!s.valid) continue;
			double c1 = s.g2 + std::fabs(h) * std::sqrt(s.l1 * s.l1 + s.g2);
			double c2 = s.g2 + std::fabs(h) * std::sqrt(s.l2 * s.l2 + s.g2);
			atan_sum += std::atan2(s.abar * (s.l2 * c1 - s.l1 * c2), c1 * c2 + s.abar * s.abar * s.l1 * s.l2);
		}
		h11at(1) = -std::fabs(h) * atan_sum + sum_a_f11(1);
		for (int K = 3; K <= mxk; K += 2) h11at(K) = ((K - 4) * h11at(K - 2) + sum_a_f11(K - 2)) / ((K - 2) * h2);
	} else {
		// Procedure 2 (and 3, which yields H*): (D.49), (D.50).
		const int ktop = opt.nhk + mxk;
		h11at(ktop) = 0.0;
		for (int K = ktop; K >= 3; K -= 2) h11at(K - 2) = (h2 * (K - 2) * h11at(K) - sum_a_f11(K - 2)) / (K - 4);
	}
	for (int K = 1; K <= mxk; K += 2) T.H[1][1][(K + 1) / 2 - 1] = h11at(K);

	auto sumF = [&](auto&& per_side) { // Σ over valid sides
		double s = 0.0;
		for (int i = 0; i < 4; ++i) if (sides[i].valid) s += per_side(sides[i], SI[i]);
		return s;
	};
	// Step 3 (D.43): H(2, N, 1).
	for (int N = 1; N <= mxq - 1; ++N) {
		T.H[2][N][0] = (h2 * sumF([&](const Side& s, const SideIntegrals& f) { return s.nu_xi * f.fmn1[1][N]; }) + sumF([&](const Side& s, const SideIntegrals& f) { return s.abar * f.fmn1[2][N]; })) / (N + 1);
	}
	// Step 4 (D.44): H(1, N, 1).
	for (int N = 2; N <= mxq; ++N) {
		double prev = N >= 3 ? T.H[1][N - 2][0] : 0.0;
		T.H[1][N][0] = (-h2 * (N - 2) * prev + h2 * sumF([&](const Side& s, const SideIntegrals& f) { return s.nu_eta * f.fmn1[1][N - 1]; }) + sumF([&](const Side& s, const SideIntegrals& f) { return s.abar * f.fmn1[1][N]; })) / N;
	}
	// Step 5 (D.45): H(M, N, 1), M >= 3.
	for (int M = 3; M <= mxq; ++M) {
		for (int N = 1; N <= mxq - M + 1; ++N) {
			T.H[M][N][0] = (-h2 * (M - 2) * T.H[M - 2][N][0] + h2 * sumF([&](const Side& s, const SideIntegrals& f) { return s.nu_xi * f.fmn1[M - 1][N]; }) + sumF([&](const Side& s, const SideIntegrals& f) { return s.abar * f.fmn1[M][N]; })) / (M + N - 1);
		}
	}
	for (int K = 3; K <= mxk; K += 2) {
		const int k = (K + 1) / 2 - 1;
		// Step 6 (D.46): H(1, N, K).
		for (int N = 2; N <= mxq; ++N) {
			double prev = N >= 3 ? T.H[1][N - 2][k - 1] : 0.0;
			T.H[1][N][k] = ((N - 2) * prev - sumF([&](const Side& s, const SideIntegrals& f) { return s.nu_eta * f.f1nk[N - 1][k - 1]; })) / (K - 2);
		}
		// Step 7 (D.47): H(2, N, K).
		for (int N = 1; N <= mxq - 1; ++N) {
			T.H[2][N][k] = -sumF([&](const Side& s, const SideIntegrals& f) { return s.nu_xi * f.f1nk[N][k - 1]; }) / (K - 2);
		}
		// Step 8 (D.48): H(M, N, K), M >= 3.
		for (int M = 3; M <= mxq; ++M) {
			for (int N = 1; N <= mxq - M + 1; ++N) {
				T.H[M][N][k] = -T.H[M - 2][N + 2][k] - h2 * T.H[M - 2][N][k] + T.H[M - 2][N][k - 1];
			}
		}
	}
	return T;
}

// Accumulates coef · h^j · H(M, N, K). With H* in hand (near the plane,
// inside Σ) the jump 2π ν sign(h)^j |h|^(j+M+N-K) is added where its
// exponent is non-negative; the negative-exponent pieces cancel between the
// terms of every formula of section D.2 (only ν T h/|h| survives, D.3).
struct Accumulator {
	const HTable& T;
	double sum = 0.0;
	void add(double coef, int j, int M, int N, int K) {
		if (coef == 0.0) return;
		double hj = 1.0;
		for (int i = 0; i < j; ++i) hj *= T.h;
		sum += coef * hj * T.at(M, N, K);
		if (T.star) {
			double nu = NuMNK(M, N, K);
			if (nu == 0.0) return;
			int e = j + M + N - K;
			if (e < 0) return;
			double sgn = (j % 2 == 0) ? 1.0 : static_cast<double>(T.sign_h);
			double mag = e == 0 ? 1.0 : std::pow(std::fabs(T.h), e);
			if (e == 0 && T.sign_h == 0 && j % 2 == 1) sgn = 0.0;
			sum += coef * 2.0 * kPi * nu * sgn * mag;
		}
	}
};

} // namespace

// ---------------------------------------------------------------------------
// LocalPanel
// ---------------------------------------------------------------------------

void LocalPanel::ComputeMoments() {
	// C(M, N) = ∬ ξ^(M-1) η^(N-1) dξ dη = ∮ (ξ^M / M) η^(N-1) dη (Green), each
	// side by Gauss-Legendre, exact for these polynomial degrees.
	static const double gx[6] = {-0.9324695142031521, -0.6612093864662645, -0.2386191860831969, 0.2386191860831969, 0.6612093864662645, 0.9324695142031521};
	static const double gw[6] = {0.1713244923791704, 0.3607615730481386, 0.4679139345726910, 0.4679139345726910, 0.3607615730481386, 0.1713244923791704};
	for (auto& row : C) for (double& v : row) v = 0.0;
	const int top = kMaxQ + 3;
	for (int i = 0; i < 4; ++i) {
		int j = (i + 1) % 4;
		double x0 = xi[i], y0 = eta[i], x1 = xi[j], y1 = eta[j];
		double deta = y1 - y0;
		if (deta == 0.0) continue;
		for (int q = 0; q < 6; ++q) {
			double t = 0.5 * (gx[q] + 1.0);
			double xq = x0 + t * (x1 - x0), yq = y0 + t * (y1 - y0);
			double w = 0.5 * gw[q] * deta;
			for (int M = 1; M <= top; ++M) {
				double xm = std::pow(xq, M) / M;
				for (int N = 1; N <= top - M + 1; ++N) C[M][N] += w * xm * std::pow(yq, N - 1);
			}
		}
	}
}

double LocalPanel::MaxExtent() const {
	double m = 0.0;
	for (int i = 0; i < 4; ++i) {
		double z = a * xi[i] * xi[i] + b * eta[i] * eta[i];
		m = std::max(m, std::sqrt(xi[i] * xi[i] + eta[i] * eta[i] + z * z));
	}
	return m;
}

double LocalPanel::Area() const {
	double s = 0.0;
	for (int i = 0; i < 4; ++i) {
		int j = (i + 1) % 4;
		s += xi[i] * eta[j] - xi[j] * eta[i];
	}
	return 0.5 * s;
}

// ---------------------------------------------------------------------------
// Near field (D.2)
// ---------------------------------------------------------------------------

Influence NearFieldInfluence(const LocalPanel& p, const Vec3& q, const IntegralOptions& opt) {
	const double x = q.x, y = q.y, z = q.z;
	// (x0, y0): the point of Σ nearest (x, y); z0 the surface height there (D.15).
	double x0 = x, y0 = y;
	if (!InsideSigma(p, x, y)) {
		double best = 1e300;
		for (int i = 0; i < 4; ++i) {
			int j = (i + 1) % 4;
			double ax = p.xi[i], ay = p.eta[i], dx = p.xi[j] - ax, dy = p.eta[j] - ay;
			double len2 = dx * dx + dy * dy;
			double t = len2 > 0.0 ? std::clamp(((x - ax) * dx + (y - ay) * dy) / len2, 0.0, 1.0) : 0.0;
			double px = ax + t * dx, py = ay + t * dy;
			double d2 = (px - x) * (px - x) + (py - y) * (py - y);
			if (d2 < best) { best = d2; x0 = px; y0 = py; }
		}
	}
	const double z0 = p.a * x0 * x0 + p.b * y0 * y0;
	const double h = z - z0;
	const double a = p.a, b = p.b, c = a * x * x + b * y * y - z0; // (D.24)
	const bool curved = a != 0.0 || b != 0.0;
	const int mxq = curved ? 6 : 4, mxk = curved ? 7 : 5;
	HTable T = ComputeH(p, x, y, h, mxq, mxk, opt);

	// ---- source potential I(M, N) (D.23) ----
	auto Is = [&](int M, int N) {
		Accumulator A{T};
		A.add(1.0, 0, M, N, 1);
		if (curved) {
			A.add(a, 1, M + 2, N, 3); A.add(2.0 * a * x, 1, M + 1, N, 3);
			A.add(b, 1, M, N + 2, 3); A.add(2.0 * b * y, 1, M, N + 1, 3);
			A.add(c, 1, M, N, 3);
		}
		return -A.sum / kFourPi;
	};
	// ---- source velocity J(M, N) (page 119) ----
	auto Jsx = [&](int M, int N) {
		Accumulator A{T};
		A.add(1.0, 0, M + 1, N, 3);
		if (curved) {
			A.add(3.0 * a, 1, M + 3, N, 5); A.add(6.0 * a * x, 1, M + 2, N, 5);
			A.add(3.0 * b, 1, M + 1, N + 2, 5); A.add(6.0 * b * y, 1, M + 1, N + 1, 5);
			A.add(3.0 * c, 1, M + 1, N, 5);
		}
		return -A.sum / kFourPi;
	};
	auto Jsy = [&](int M, int N) {
		Accumulator A{T};
		A.add(1.0, 0, M, N + 1, 3);
		if (curved) {
			A.add(3.0 * a, 1, M + 2, N + 1, 5); A.add(6.0 * a * x, 1, M + 1, N + 1, 5);
			A.add(3.0 * b, 1, M, N + 3, 5); A.add(6.0 * b * y, 1, M, N + 2, 5);
			A.add(3.0 * c, 1, M, N + 1, 5);
		}
		return -A.sum / kFourPi;
	};
	auto Jsz = [&](int M, int N) {
		Accumulator A{T};
		A.add(-1.0, 1, M, N, 3);
		if (curved) {
			A.add(a, 0, M + 2, N, 3); A.add(-3.0 * a, 2, M + 2, N, 5); A.add(2.0 * a * x, 0, M + 1, N, 3); A.add(-6.0 * a * x, 2, M + 1, N, 5);
			A.add(b, 0, M, N + 2, 3); A.add(-3.0 * b, 2, M, N + 2, 5); A.add(2.0 * b * y, 0, M, N + 1, 3); A.add(-6.0 * b * y, 2, M, N + 1, 5);
			A.add(c, 0, M, N, 3); A.add(-3.0 * c, 2, M, N, 5);
		}
		return -A.sum / kFourPi;
	};
	// ---- doublet potential I(M, N) (D.32) ----
	auto Id = [&](int M, int N) {
		Accumulator A{T};
		A.add(1.0, 1, M, N, 3);
		if (curved) {
			A.add(a, 0, M + 2, N, 3); A.add(3.0 * a, 2, M + 2, N, 5); A.add(6.0 * a * x, 2, M + 1, N, 5);
			A.add(b, 0, M, N + 2, 3); A.add(3.0 * b, 2, M, N + 2, 5); A.add(6.0 * b * y, 2, M, N + 1, 5);
			A.add(-c, 0, M, N, 3); A.add(3.0 * c, 2, M, N, 5);
		}
		return A.sum / kFourPi;
	};
	// ---- doublet velocity J(M, N) (page 121) ----
	auto Jdx = [&](int M, int N) {
		Accumulator A{T};
		A.add(3.0, 1, M + 1, N, 5);
		if (curved) {
			A.add(3.0 * a, 0, M + 3, N, 5); A.add(-2.0 * a, 0, M + 1, N, 3); A.add(15.0 * a, 2, M + 3, N, 7); A.add(-2.0 * a * x, 0, M, N, 3); A.add(30.0 * a * x, 2, M + 2, N, 7);
			A.add(3.0 * b, 0, M + 1, N + 2, 5); A.add(15.0 * b, 2, M + 1, N + 2, 7); A.add(30.0 * b * y, 2, M + 1, N + 1, 7);
			A.add(-3.0 * c, 0, M + 1, N, 5); A.add(15.0 * c, 2, M + 1, N, 7);
		}
		return A.sum / kFourPi;
	};
	auto Jdy = [&](int M, int N) {
		Accumulator A{T};
		A.add(3.0, 1, M, N + 1, 5);
		if (curved) {
			A.add(3.0 * a, 0, M + 2, N + 1, 5); A.add(15.0 * a, 2, M + 2, N + 1, 7); A.add(30.0 * a * x, 2, M + 1, N + 1, 7);
			A.add(3.0 * b, 0, M, N + 3, 5); A.add(-2.0 * b, 0, M, N + 1, 3); A.add(15.0 * b, 2, M, N + 3, 7); A.add(-2.0 * b * y, 0, M, N, 3); A.add(30.0 * b * y, 2, M, N + 2, 7);
			A.add(-3.0 * c, 0, M, N + 1, 5); A.add(15.0 * c, 2, M, N + 1, 7);
		}
		return A.sum / kFourPi;
	};
	auto Jdz = [&](int M, int N) {
		Accumulator A{T};
		A.add(1.0, 0, M, N, 3); A.add(-3.0, 2, M, N, 5);
		if (curved) {
			A.add(3.0 * a, 1, M + 2, N, 5); A.add(-15.0 * a, 3, M + 2, N, 7); A.add(12.0 * a * x, 1, M + 1, N, 5); A.add(-30.0 * a * x, 3, M + 1, N, 7);
			A.add(3.0 * b, 1, M, N + 2, 5); A.add(-15.0 * b, 3, M, N + 2, 7); A.add(12.0 * b * y, 1, M, N + 1, 5); A.add(-30.0 * b * y, 3, M, N + 1, 7);
			A.add(9.0 * c, 1, M, N, 5); A.add(-15.0 * c, 3, M, N, 7);
		}
		return A.sum / kFourPi;
	};

	Influence out;
	// Source: φ = σ(x,y) I(1,1) + σξ I(2,1) + ση I(1,2), σ(x,y) = σ0 + σξ x + ση y (D.21, D.22).
	{
		double I11 = Is(1, 1), I21 = Is(2, 1), I12 = Is(1, 2);
		out.phi_source[0] = I11;
		out.phi_source[1] = x * I11 + I21;
		out.phi_source[2] = y * I11 + I12;
		double J11[3] = {Jsx(1, 1), Jsy(1, 1), Jsz(1, 1)};
		double J21[3] = {Jsx(2, 1), Jsy(2, 1), Jsz(2, 1)};
		double J12[3] = {Jsx(1, 2), Jsy(1, 2), Jsz(1, 2)};
		for (int k = 0; k < 3; ++k) {
			out.vel_source[k][0] = J11[k];
			out.vel_source[k][1] = x * J11[k] + J21[k];
			out.vel_source[k][2] = y * J11[k] + J12[k];
		}
	}
	// Doublet (D.30), (D.31), (D.33).
	{
		double I11 = Id(1, 1), I21 = Id(2, 1), I12 = Id(1, 2), I31 = Id(3, 1), I22 = Id(2, 2), I13 = Id(1, 3);
		auto fill = [&](double* row, double a11, double a21, double a12, double a31, double a22, double a13) {
			row[0] = a11;
			row[1] = x * a11 + a21;
			row[2] = y * a11 + a12;
			row[3] = 0.5 * x * x * a11 + x * a21 + 0.5 * a31;
			row[4] = x * y * a11 + y * a21 + x * a12 + a22;
			row[5] = 0.5 * y * y * a11 + y * a12 + 0.5 * a13;
		};
		fill(out.phi_doublet, I11, I21, I12, I31, I22, I13);
		double (*J[3])(int, int) = {nullptr, nullptr, nullptr};
		(void)J;
		double jx11 = Jdx(1, 1), jx21 = Jdx(2, 1), jx12 = Jdx(1, 2), jx31 = Jdx(3, 1), jx22 = Jdx(2, 2), jx13 = Jdx(1, 3);
		double jy11 = Jdy(1, 1), jy21 = Jdy(2, 1), jy12 = Jdy(1, 2), jy31 = Jdy(3, 1), jy22 = Jdy(2, 2), jy13 = Jdy(1, 3);
		double jz11 = Jdz(1, 1), jz21 = Jdz(2, 1), jz12 = Jdz(1, 2), jz31 = Jdz(3, 1), jz22 = Jdz(2, 2), jz13 = Jdz(1, 3);
		fill(out.vel_doublet[0], jx11, jx21, jx12, jx31, jx22, jx13);
		fill(out.vel_doublet[1], jy11, jy21, jy12, jy31, jy22, jy13);
		fill(out.vel_doublet[2], jz11, jz21, jz12, jz31, jz22, jz13);
	}
	return out;
}

// ---------------------------------------------------------------------------
// Far field (D.4)
// ---------------------------------------------------------------------------

Influence FarFieldInfluence(const LocalPanel& p, const Vec3& q) {
	const double P = Length(q);
	const Vec3 ph = q / P;
	const double a = p.a, b = p.b;
	auto C = [&](int M, int N) { return p.C[M][N]; };
	Influence out;
	// Source (D.76)-(D.85).
	auto source = [&](int M, int N, double& phi, Vec3& vel) {
		double E1 = C(M, N);
		Vec3 E2{C(M + 1, N), C(M, N + 1), a * C(M + 2, N) + b * C(M, N + 2)};
		double E3 = C(M + 2, N) + C(M, N + 2);
		double E4[3][3] = {
			{C(M + 2, N), C(M + 1, N + 1), a * C(M + 3, N) + b * C(M + 1, N + 2)},
			{C(M + 1, N + 1), C(M, N + 2), a * C(M + 2, N + 1) + b * C(M, N + 3)},
			{a * C(M + 3, N) + b * C(M + 1, N + 2), a * C(M + 2, N + 1) + b * C(M, N + 3), 0.0},
		};
		Vec3 E4p{0, 0, 0};
		for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) E4p[i] += E4[i][j] * ph[j];
		double pE4p = Dot(ph, E4p);
		phi = -(E1 / P + Dot(E2, ph) / (P * P) + (-0.5 * E3 + 1.5 * pE4p) / (P * P * P)) / kFourPi;
		vel = (ph * (-E1) / (P * P) + (E2 - ph * (3.0 * Dot(E2, ph))) / (P * P * P) + (ph * (1.5 * E3) - ph * (7.5 * pE4p) + E4p * 3.0) / (P * P * P * P)) * (-1.0 / kFourPi);
	};
	// Doublet (D.86)-(D.92).
	auto doublet = [&](int M, int N, double& phi, Vec3& vel) {
		Vec3 E2{-2.0 * a * C(M + 1, N), -2.0 * b * C(M, N + 1), C(M, N)};
		double E3 = -a * C(M + 2, N) - b * C(M, N + 2);
		double E4[3][3] = {
			{-2.0 * a * C(M + 2, N), -(a + b) * C(M + 1, N + 1), 0.5 * C(M + 1, N)},
			{-(a + b) * C(M + 1, N + 1), -2.0 * b * C(M, N + 2), 0.5 * C(M, N + 1)},
			{0.5 * C(M + 1, N), 0.5 * C(M, N + 1), a * C(M + 2, N) + b * C(M, N + 2)},
		};
		Vec3 E4p{0, 0, 0};
		for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) E4p[i] += E4[i][j] * ph[j];
		double pE4p = Dot(ph, E4p);
		phi = (Dot(E2, ph) / (P * P) + (-E3 + 3.0 * pE4p) / (P * P * P)) / kFourPi;
		vel = ((E2 - ph * (3.0 * Dot(E2, ph))) / (P * P * P) + (ph * (3.0 * E3) - ph * (15.0 * pE4p) + E4p * 6.0) / (P * P * P * P)) / kFourPi;
	};
	{
		const int MN[3][2] = {{1, 1}, {2, 1}, {1, 2}};
		for (int c = 0; c < 3; ++c) {
			double phi; Vec3 v;
			source(MN[c][0], MN[c][1], phi, v);
			out.phi_source[c] = phi;
			for (int k = 0; k < 3; ++k) out.vel_source[k][c] = v[k];
		}
	}
	{
		// φ = μ0 I(1,1) + μξ I(2,1) + μη I(1,2) + ½ μξξ I(3,1) + μξη I(2,2) + ½ μηη I(1,3) (D.86)
		const int MN[6][2] = {{1, 1}, {2, 1}, {1, 2}, {3, 1}, {2, 2}, {1, 3}};
		const double scale[6] = {1.0, 1.0, 1.0, 0.5, 1.0, 0.5};
		for (int c = 0; c < 6; ++c) {
			double phi; Vec3 v;
			doublet(MN[c][0], MN[c][1], phi, v);
			out.phi_doublet[c] = scale[c] * phi;
			for (int k = 0; k < 3; ++k) out.vel_doublet[k][c] = scale[c] * v[k];
		}
	}
	return out;
}

Influence ComputeInfluence(const LocalPanel& p, const Vec3& q, const IntegralOptions& opt, bool* used_far_field) {
	const double P = Length(q);
	const bool far = opt.allow_far_field && P > 0.0 && p.MaxExtent() / P < opt.far_ratio;
	if (used_far_field) *used_far_field = far;
	return far ? FarFieldInfluence(p, q) : NearFieldInfluence(p, q, opt);
}

// ---------------------------------------------------------------------------
// Reference quadrature
// ---------------------------------------------------------------------------

Influence QuadratureInfluence(const LocalPanel& p, const Vec3& q, int n) {
	// Bilinear map of the unit square onto Σ, n x n midpoint cells; the
	// surface ζ = a ξ² + b η² and its normal n dS = (-2aξ, -2bη, 1) dξ dη
	// (D.11, D.12) enter exactly, so this is the true curved-panel integral.
	Influence out;
	const Vec3 P = q;
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < n; ++j) {
			double u = (i + 0.5) / n, v = (j + 0.5) / n;
			// Bilinear interpolation over corners 0..3 (0->1->2->3 CCW).
			double xi = (1 - u) * (1 - v) * p.xi[0] + u * (1 - v) * p.xi[1] + u * v * p.xi[2] + (1 - u) * v * p.xi[3];
			double eta = (1 - u) * (1 - v) * p.eta[0] + u * (1 - v) * p.eta[1] + u * v * p.eta[2] + (1 - u) * v * p.eta[3];
			double dxu = -(1 - v) * p.xi[0] + (1 - v) * p.xi[1] + v * p.xi[2] - v * p.xi[3];
			double dxv = -(1 - u) * p.xi[0] - u * p.xi[1] + u * p.xi[2] + (1 - u) * p.xi[3];
			double dyu = -(1 - v) * p.eta[0] + (1 - v) * p.eta[1] + v * p.eta[2] - v * p.eta[3];
			double dyv = -(1 - u) * p.eta[0] - u * p.eta[1] + u * p.eta[2] + (1 - u) * p.eta[3];
			double jac = std::fabs(dxu * dyv - dxv * dyu) / (n * n);
			double zeta = p.a * xi * xi + p.b * eta * eta;
			Vec3 Q{xi, eta, zeta};
			Vec3 R = Q - P;
			double r = Length(R);
			Vec3 nds{-2.0 * p.a * xi, -2.0 * p.b * eta, 1.0}; // n dS / dξdη
			double sig[3] = {1.0, xi, eta};
			double mu[6] = {1.0, xi, eta, 0.5 * xi * xi, xi * eta, 0.5 * eta * eta};
			double secant = std::sqrt(1.0 + 4.0 * p.a * p.a * xi * xi + 4.0 * p.b * p.b * eta * eta);
			// Source: φ = σ (-1/4πR) dS, v = ∇_P φ = σ (-R)/(4πR³) dS (R = Q - P; D.106).
			for (int c = 0; c < 3; ++c) {
				out.phi_source[c] += sig[c] * (-1.0 / (kFourPi * r)) * secant * jac;
				Vec3 g = R * (-1.0 / (kFourPi * r * r * r));
				for (int k = 0; k < 3; ++k) out.vel_source[k][c] += sig[c] * g[k] * secant * jac;
			}
			// Doublet: φ = μ (-R·n)/(4πR³) dS; v = ∇_P: μ [ n/(4πR³) - 3 (R·n) R/(4πR⁵) ] dS.
			double Rn = Dot(R, nds);
			for (int c = 0; c < 6; ++c) {
				out.phi_doublet[c] += mu[c] * (-Rn / (kFourPi * r * r * r)) * jac;
				Vec3 g = nds / (kFourPi * r * r * r) - R * (3.0 * Rn / (kFourPi * r * r * r * r * r));
				for (int k = 0; k < 3; ++k) out.vel_doublet[k][c] += mu[c] * g[k] * jac;
			}
		}
	}
	return out;
}

} // namespace aerotelic::panel
