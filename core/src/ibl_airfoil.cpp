#include "aerotelic/ibl_airfoil.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>

#include "aerotelic/dense_lu.hpp"
#include "aerotelic/dual.hpp"
#include "aerotelic/ibl_closures.hpp"
#include "aerotelic/vortex_panel.hpp"

namespace aerotelic {

namespace {

using namespace vortexpanel;
using namespace ibl;

constexpr double kPi = 3.14159265358979323846;
constexpr int ND = 12;
using D = Dual<ND>;
using SD = State<D>;
using S4 = State<double>;

SD Seed(const S4& U, int base) {
	SD r;
	for (int k = 0; k < 4; ++k) r[k] = D::Var(U[k], base + k);
	return r;
}
SD Const(const S4& U) {
	SD r;
	for (int k = 0; k < 4; ++k) r[k] = D(U[k]);
	return r;
}
S4 ValueOf(const SD& U) { return {U[0].v, U[1].v, U[2].v, U[3].v}; }

double Norm(const std::vector<double>& v) {
	double s = 0.0;
	for (double x : v) s += x * x;
	return std::sqrt(s);
}

// Solves the small dense system A x = b (n <= 4), A row-major.
bool SolveSmall(int n, std::vector<double> A, std::vector<double> b, std::vector<double>& x) {
	for (int k = 0; k < n; ++k) {
		int p = k;
		for (int i = k + 1; i < n; ++i) if (std::fabs(A[i * n + k]) > std::fabs(A[p * n + k])) p = i;
		if (std::fabs(A[p * n + k]) < 1e-300) return false;
		if (p != k) { for (int j = 0; j < n; ++j) std::swap(A[k * n + j], A[p * n + j]); std::swap(b[k], b[p]); }
		for (int i = k + 1; i < n; ++i) {
			const double f = A[i * n + k] / A[k * n + k];
			for (int j = k; j < n; ++j) A[i * n + j] -= f * A[k * n + j];
			b[i] -= f * b[k];
		}
	}
	x.assign(n, 0.0);
	for (int i = n - 1; i >= 0; --i) {
		double s = b[i];
		for (int j = i + 1; j < n; ++j) s -= A[i * n + j] * x[j];
		x[i] = s / A[i * n + i];
	}
	return true;
}

// Trailing-edge information (TE_info): bisector, gap, thickness slope.
struct TeInfo { Vec2 t; double hTE = 0.0, dtdx = 0.0; };
TeInfo TrailingEdgeInfo(const std::vector<Vec2>& X) {
	const int N = static_cast<int>(X.size());
	Vec2 t1 = X[0] - X[1]; t1 = t1 * (1.0 / t1.norm());
	Vec2 t2 = X[N - 1] - X[N - 2]; t2 = t2 * (1.0 / t2.norm());
	Vec2 t = (t1 + t2) * 0.5; t = t * (1.0 / std::max(t.norm(), 1e-300));
	const Vec2 s = X[N - 1] - X[0];
	TeInfo info;
	info.t = t;
	info.hTE = -s.x * t.y + s.y * t.x;
	info.dtdx = t1.x * t2.y - t2.x * t1.y;
	return info;
}

class Solver {
public:
	Solver(const Mat& contour, double alpha_deg, const IblOptions& options) : opt(options), alpha_deg(alpha_deg) {
		alpha = alpha_deg * kPi / 180.0;
		cst.Vinf = 1.0;
		cst.rho0 = 1.0;
		cst.ncrit = opt.ncrit;
		InviscidOptions io;
		io.wake_length = opt.wake_length;
		io.n_wake = opt.n_wake;
		io.alpha_wake_deg = alpha_deg;
		inv = SolveInviscid(contour, io);
		g = &inv.geom;
		N = g->n_airfoil();
		Nw = g->n_wake();
		Nsys = N + Nw;
		cst.mu0 = cst.rho0 * cst.Vinf * g->chord / opt.reynolds;
		// s along airfoil then wake (continuation of the upper surface)
		s_all.resize(Nsys);
		for (int i = 0; i < N; ++i) s_all[i] = g->s[i];
		for (int i = 0; i < Nw; ++i) s_all[N + i] = g->s[N - 1] + g->sw[i];
		BuildWakeReference();
		gamma = inv.GammaAt(alpha);
		FindStagnation();
		IdentifySurfaces();
		te = TrailingEdgeInfo(g->nodes);
		SetWakeGap();
		BuildUeSigma();
		RebuildUeM();
	}

	IblOptions opt;
	double alpha_deg, alpha;
	IblConstants cst;
	InviscidSolution inv;
	const Paneling* g = nullptr;
	int N = 0, Nw = 0, Nsys = 0;
	std::vector<double> s_all;
	std::vector<Vec2> wake_t;                 // unit tangents at the wake nodes
	std::vector<double> uewiref0, uewiref90;  // inviscid wake ue reference solutions
	std::vector<double> gamma;
	std::vector<int> sgnue;
	int Istag[2] = {0, 1};
	double sstag = 0.0, sstag_ue[2] = {0.0, 0.0};
	Vec2 xstag;
	std::vector<double> xi;
	std::vector<std::vector<int>> Is;
	TeInfo te;
	std::vector<double> wgap;
	std::vector<double> ue_sigma; // Nsys x (Nsys-2)
	std::vector<double> ue_m;     // Nsys x Nsys
	std::vector<S4> U;
	std::vector<char> turb;
	double Xt_xi[2] = {-1.0, -1.0}, Xt_x[2] = {-1.0, -1.0};
	bool forced[2] = {false, false};
	bool converged = false;
	int iterations = 0;
	double rnorm = 0.0;
	std::vector<std::string> warnings;
	// global residual pieces
	std::vector<double> R, R_U, R_x;

	// ---- geometry-level setup -------------------------------------------------------
	void BuildWakeReference() {
		wake_t.resize(Nw);
		uewiref0.resize(Nw);
		uewiref90.resize(Nw);
		const std::vector<double> gam_a = inv.GammaAt(alpha);
		for (int i = 0; i < Nw; ++i) {
			Vec2 v = InviscidVelocityAt(inv, gam_a, g->wake[i], alpha);
			const double n = v.norm();
			wake_t[i] = n > 1e-12 ? v * (1.0 / n) : Vec2(1.0, 0.0);
			const Vec2 v0 = InviscidVelocityAt(inv, inv.gamma0, g->wake[i], 0.0);
			const Vec2 v90 = InviscidVelocityAt(inv, inv.gamma90, g->wake[i], 0.5 * kPi);
			uewiref0[i] = v0.dot(wake_t[i]);
			uewiref90[i] = v90.dot(wake_t[i]);
		}
	}

	void FindStagnation() {
		// The sign change of gamma nearest the leading edge (a finite-angle
		// trailing edge has gamma ~ 0 at its own nodes, which must not count).
		const int j = StagnationInterval(*g, gamma) + 1;
		if (j < 1 || j >= N - 1) throw std::runtime_error("no stagnation point in the inviscid solution");
		Istag[0] = j - 1;
		Istag[1] = j;
		const double G0 = gamma[j - 1], G1 = gamma[j], S0 = g->s[j - 1], S1 = g->s[j];
		const double den = G1 - G0;
		const double w1 = G1 / den, w2 = -G0 / den;
		sstag = w1 * S0 + w2 * S1;
		xstag = g->nodes[j - 1] * w1 + g->nodes[j] * w2;
		sgnue.assign(N, 1);
		for (int i = 0; i < j; ++i) sgnue[i] = -1;
		ComputeXi();
	}

	void ComputeXi() {
		xi.resize(Nsys);
		for (int i = 0; i < N; ++i) xi[i] = std::fabs(g->s[i] - sstag);
		for (int i = 0; i < Nw; ++i) xi[N + i] = s_all[N + i] - sstag;
	}

	void IdentifySurfaces() {
		Is.assign(3, {});
		for (int i = Istag[0]; i >= 0; --i) Is[0].push_back(i);
		for (int i = Istag[1]; i < N; ++i) Is[1].push_back(i);
		for (int i = N; i < Nsys; ++i) Is[2].push_back(i);
	}

	void SetWakeGap() {
		const double flen = 2.5;
		const double dtdx = std::clamp(te.dtdx, -3.0 / flen, 3.0 / flen);
		const double Lw = flen * te.hTE;
		wgap.assign(Nw, 0.0);
		if (!(Lw > 0.0)) return;
		for (int i = 0; i < Nw; ++i) {
			const double xib = (xi[N + i] - xi[N]) / Lw;
			if (xib <= 1.0) wgap[i] = te.hTE * (1.0 + (2.0 + flen * dtdx) * xib) * (1.0 - xib) * (1.0 - xib);
		}
	}

	// B, Bp, Cgam, Csig -> ue_sigma (calc_ue_m)
	void BuildUeSigma() {
		const int Ns = Nsys - 2; // source columns: N-1 airfoil panels + Nw-1 wake nodes
		const int n1 = N + 1;
		// B: (N+1) x Ns, the airfoil-node streamfunction of unit sources.
		std::vector<double> B(static_cast<size_t>(n1) * Ns, 0.0);
		auto wake3 = [&](int j, Vec2& a, Vec2& m, Vec2& b) {
			a = g->wake[j];
			b = g->wake[j + 1];
			m = (a + b) * 0.5;
			if (j == Nw - 2) b = b * 2.0 - m; // the last half panel duplicated past the end
		};
		for (int i = 0; i < N - 1; ++i) { // node N-1 duplicates node 0 and its row is the extrapolation: zero source row
			const Vec2 xi_p = g->nodes[i];
			for (int j = 0; j + 1 < N; ++j) B[i * Ns + j] = PsiConstSource(FrameOf(g->nodes[j], g->nodes[j + 1], xi_p));
			for (int j = 0; j + 1 < Nw; ++j) {
				Vec2 a, m, b;
				wake3(j, a, m, b);
				const Coef2 l = PsiLinearSource(FrameOf(a, m, xi_p));
				if (j > 0) { B[i * Ns + N - 1 + j] += 0.5 * l.c1 + l.c2; B[i * Ns + N - 1 + j - 1] += 0.5 * l.c1; }
				else B[i * Ns + N - 1 + j] += l.c2;
				const Coef2 r = PsiLinearSource(FrameOf(m, b, xi_p));
				B[i * Ns + N - 1 + j] += r.c1 + 0.5 * r.c2;
				if (j < Nw - 2) B[i * Ns + N - 1 + j + 1] += 0.5 * r.c2;
				else B[i * Ns + N - 1 + j] += 0.5 * r.c2;
			}
		}
		// Bp = -A^-1 B, rows 0..N-1
		std::vector<double> Bp(static_cast<size_t>(N) * Ns, 0.0);
		std::vector<double> col(n1);
		for (int c = 0; c < Ns; ++c) {
			for (int r = 0; r < n1; ++r) col[r] = -B[r * Ns + c];
			inv.lu.Solve(col);
			for (int r = 0; r < N; ++r) Bp[r * Ns + c] = col[r];
		}
		// Cgam: d(wake tangential velocity)/d(gamma), Nw x N
		std::vector<double> Cgam(static_cast<size_t>(Nw) * N, 0.0);
		for (int i = 0; i < Nw; ++i) {
			for (int j = 0; j + 1 < N; ++j) {
				const VelCoef2 v = VelLinearVortex(FrameOf(g->nodes[j], g->nodes[j + 1], g->wake[i]));
				Cgam[i * N + j] += v.v1.dot(wake_t[i]);
				Cgam[i * N + j + 1] += v.v2.dot(wake_t[i]);
			}
		}
		// Csig: d(wake tangential velocity)/d(sources), Nw x Ns (build_Csig_bulk)
		std::vector<double> Csig(static_cast<size_t>(Nw) * Ns, 0.0);
		for (int i = 0; i < Nw; ++i) {
			const Vec2 xp = g->wake[i];
			const Vec2 ti = wake_t[i];
			const int jstart = (i == 0) ? 1 : 0, jend = N - 1 - ((i == 0) ? 1 : 0);
			for (int j = jstart; j < jend; ++j) {
				const VelCoef2 v = VelLinearSource(FrameOf(g->nodes[j], g->nodes[j + 1], xp));
				Csig[i * Ns + j] += (v.v1 + v.v2).dot(ti);
			}
			for (int j = 0; j < Nw; ++j) {
				const int I1 = std::max(j - 1, 0), I2 = j, I3 = std::min(j + 1, Nw - 1);
				Vec2 lo0 = (g->wake[I1] + g->wake[I2]) * 0.5, lo1 = g->wake[I2];
				Vec2 hi0 = g->wake[I2], hi1 = (g->wake[I2] + g->wake[I3]) * 0.5;
				if (j == Nw - 1) hi1 = hi0 * 2.0 - lo0;
				const double d1 = (lo1 - lo0).norm(), d2 = (hi1 - hi0).norm();
				if (i == j) {
					if (j == 0) {
						const double dl = (g->nodes[1] - g->nodes[0]).norm(), du = (g->nodes[N - 1] - g->nodes[N - 2]).norm();
						Csig[i * Ns + 0] += (0.5 / kPi) * (std::log(dl / d2) + 1.0);
						Csig[i * Ns + N - 2] += (0.5 / kPi) * (std::log(du / d2) + 1.0);
						Csig[i * Ns + N - 1] += -0.5 / kPi;
					} else if (j == Nw - 1) {
						// handled through j-1
					} else {
						const double aa = (0.25 / kPi) * std::log(d1 / d2);
						Csig[i * Ns + N - 1 + j - 1] += aa + 0.5 / kPi;
						Csig[i * Ns + N - 1 + j] += aa - 0.5 / kPi;
					}
				} else {
					if (j == 0) {
						const VelCoef2 v = VelLinearSource(FrameOf(hi0, hi1, xp));
						const double a = v.v1.dot(ti), b = v.v2.dot(ti);
						Csig[i * Ns + N - 1] += b;
						Csig[i * Ns + 0] += a;
						Csig[i * Ns + N - 2] += a;
					} else if (j == Nw - 1) {
						const VelCoef2 v = VelLinearSource(FrameOf(lo0, hi1, xp));
						Csig[i * Ns + N + Nw - 3] += (v.v1 + v.v2).dot(ti);
					} else {
						const VelCoef2 v1 = VelLinearSource(FrameOf(lo0, lo1, xp));
						const VelCoef2 v2 = VelLinearSource(FrameOf(hi0, hi1, xp));
						const double a1 = v1.v1.dot(ti), b1 = v1.v2.dot(ti), a2 = v2.v1.dot(ti), b2 = v2.v2.dot(ti);
						Csig[i * Ns + N - 1 + j - 1] += a1 + 0.5 * b1;
						Csig[i * Ns + N - 1 + j] += 0.5 * a2 + b2;
					}
				}
			}
		}
		// Dw = Cgam Bp + Csig, first wake row = last airfoil row of Bp
		ue_sigma.assign(static_cast<size_t>(Nsys) * Ns, 0.0);
		for (int r = 0; r < N; ++r) for (int c = 0; c < Ns; ++c) ue_sigma[r * Ns + c] = Bp[r * Ns + c];
		for (int i = 0; i < Nw; ++i) {
			for (int c = 0; c < Ns; ++c) {
				double v = Csig[i * Ns + c];
				for (int j = 0; j < N; ++j) v += Cgam[i * N + j] * Bp[j * Ns + c];
				ue_sigma[(N + i) * Ns + c] = v;
			}
		}
		for (int c = 0; c < Ns; ++c) ue_sigma[N * Ns + c] = Bp[(N - 1) * Ns + c];
	}

	// ue_m = diag(sgue) ue_sigma sigma_m (rebuild_ue_m)
	void RebuildUeM() {
		const int Ns = Nsys - 2;
		ue_m.assign(static_cast<size_t>(Nsys) * Nsys, 0.0);
		for (int r = 0; r < Nsys; ++r) {
			const double sg = r < N ? sgnue[r] : 1.0;
			double* row = &ue_m[static_cast<size_t>(r) * Nsys];
			const double* us = &ue_sigma[static_cast<size_t>(r) * Ns];
			for (int c = 0; c + 1 < N; ++c) {
				const double ds = g->s[c + 1] - g->s[c];
				row[c] += sg * us[c] * (-sgnue[c] / ds);
				row[c + 1] += sg * us[c] * (sgnue[c + 1] / ds);
			}
			for (int c = 0; c + 1 < Nw; ++c) {
				const double ds = s_all[N + c + 1] - s_all[N + c];
				row[N + c] += sg * us[N - 1 + c] * (-1.0 / ds);
				row[N + c + 1] += sg * us[N - 1 + c] * (1.0 / ds);
			}
		}
	}

	std::vector<double> UeInv() const {
		std::vector<double> ue(Nsys);
		const double ca = std::cos(alpha), sa = std::sin(alpha);
		for (int i = 0; i < N; ++i) ue[i] = sgnue[i] * (inv.gamma0[i] * ca + inv.gamma90[i] * sa);
		for (int i = 0; i < Nw; ++i) ue[N + i] = uewiref0[i] * ca + uewiref90[i] * sa;
		ue[N] = ue[N - 1];
		return ue;
	}

	// ---- residual pieces ------------------------------------------------------------
	IblFlags FlagsFor(int si, bool is_turb, bool simi = false) const {
		IblFlags f;
		f.wake = (si == 2);
		f.turb = f.wake || is_turb;
		f.simi = simi;
		return f;
	}

	double XcOf(int node) const { return (g->nodes[node].x - g->x_min) / g->chord; }

	// The xi of the forced-transition station on a surface, or -1 when none;
	// the interval [a, b] (surface order) that contains it is forced.
	double ForcedXi(int si, int a, int b) const {
		if (si > 1) return -1.0;
		const double xft = (si == 1) ? opt.xft_upper : opt.xft_lower;
		if (xft >= 1.0) return -1.0;
		const double xa = XcOf(a), xb = XcOf(b);
		if (!((xa <= xft && xft <= xb) || (xb <= xft && xft <= xa))) return -1.0;
		const double t = (xb == xa) ? 0.0 : (xft - xa) / (xb - xa);
		return xi[a] + t * (xi[b] - xi[a]);
	}

	// Transition interval residual with the transition location solved
	// inside the interval and its derivatives through the implicit function
	// theorem. Seeds: U1 0-3, U2 4-7, x1 8, x2 9 (xt in slot 10 internally).
	// A forced transition (forced_xi >= 0) fixes the location instead: the
	// laminar sub-interval then carries whatever amplification it reaches.
	std::array<D, 3> TransitionResidual(int si, double x1v, double x2v, const S4& U1v, const S4& U2v, double aux, double& xt_out, double forced_xi = -1.0) const {
		IblFlags fl = FlagsFor(si, false), ft = FlagsFor(si, true);
		const double dxv = x2v - x1v;
		if (forced_xi >= 0.0) {
			const double xt = std::clamp(forced_xi, x1v + 1e-3 * dxv, x2v - 1e-3 * dxv);
			xt_out = xt;
			const SD U1 = Seed(U1v, 0), U2 = Seed(U2v, 4);
			const D x1 = D::Var(x1v, 8), x2 = D::Var(x2v, 9);
			// The forced station is a fixed fraction of the interval (its x/c
			// is geometric), so it moves with the interval's end coordinates.
			const double tfrac = (xt - x1v) / dxv;
			const D xt_d = x1 + tfrac * (x2 - x1);
			const D w2 = (xt_d - x1) / (x2 - x1);
			const D w1 = 1.0 - w2;
			SD Ut;
			for (int k = 0; k < 4; ++k) Ut[k] = w1 * U1[k] + w2 * U2[k];
			// laminar side: the amplification integrated to xt
			SD Utl = Ut;
			Utl[2] = U1[2];
			Utl[2] = U1[2] + 0.5 * (xt_d - x1) * (Damp(U1, fl, cst) + Damp(Utl, fl, cst));
			SD Utt = Ut;
			Utt[2] = Cttr(Ut, ft, cst);
			const D auxd(aux);
			const std::array<D, 3> Rl = ResidualStation(fl, cst, x1, xt_d, U1, Utl, auxd, auxd);
			const std::array<D, 3> Rt = ResidualStation(ft, cst, xt_d, x2, Utt, U2, auxd, auxd);
			// The laminar amplification residual is satisfied by construction;
			// only the turbulent lag residual carries the third row.
			return {Rl[0] + Rt[0], Rl[1] + Rt[1], Rt[2]};
		}
		double xt = x1v + 0.5 * dxv;
		// Newton on doubles for xt
		for (int it = 0; it < 20; ++it) {
			const double w2 = (xt - x1v) / dxv, w1 = 1.0 - w2;
			S4 Ut{w1 * U1v[0] + w2 * U2v[0], w1 * U1v[1] + w2 * U2v[1], cst.ncrit, w1 * U1v[3] + w2 * U2v[3]};
			const double damp1 = Damp(U1v, fl, cst), dampt = Damp(Ut, fl, cst);
			const double Rxt = cst.ncrit - U1v[2] - 0.5 * (xt - x1v) * (damp1 + dampt);
			// d dampt / d xt through Ut (finite difference on the interpolation)
			const double h = 1e-6 * dxv;
			const double w2h = (xt + h - x1v) / dxv, w1h = 1.0 - w2h;
			S4 Uth{w1h * U1v[0] + w2h * U2v[0], w1h * U1v[1] + w2h * U2v[1], cst.ncrit, w1h * U1v[3] + w2h * U2v[3]};
			const double dampt_xt = (Damp(Uth, fl, cst) - dampt) / h;
			const double Rxt_xt = -0.5 * (damp1 + dampt) - 0.5 * (xt - x1v) * dampt_xt;
			double dxt = -Rxt / Rxt_xt;
			const double dmax = 0.2 * dxv * (1.1 - static_cast<double>(it) / 20.0);
			if (std::fabs(dxt) > dmax) dxt *= dmax / std::fabs(dxt);
			if (std::fabs(Rxt) < 1e-10) break;
			xt += dxt;
		}
		xt_out = xt;
		// Duals: the transition residual with xt seeded in slot 10 gives dxt/d(everything).
		const SD U1 = Seed(U1v, 0), U2 = Seed(U2v, 4);
		const D x1 = D::Var(x1v, 8), x2 = D::Var(x2v, 9);
		D xt_d = D::Var(xt, 10);
		auto interp = [&](const D& xt_in) {
			const D w2 = (xt_in - x1) / (x2 - x1);
			const D w1 = 1.0 - w2;
			SD Ut;
			for (int k = 0; k < 4; ++k) Ut[k] = w1 * U1[k] + w2 * U2[k];
			Ut[2] = D(cst.ncrit);
			return Ut;
		};
		{
			const SD Ut = interp(xt_d);
			const D Rxt = cst.ncrit - U1[2] - 0.5 * (xt_d - x1) * (Damp(U1, fl, cst) + Damp(Ut, fl, cst));
			D xt_total(xt);
			const double dR_dxt = Rxt.d[10];
			for (int k = 0; k < 10; ++k) xt_total.d[k] = -Rxt.d[k] / dR_dxt;
			xt_d = xt_total;
		}
		const SD Ut = interp(xt_d);
		SD Utl = Ut;
		Utl[2] = D(cst.ncrit);
		SD Utt = Ut;
		Utt[2] = Cttr(Ut, ft, cst);
		const D auxd(aux);
		const std::array<D, 3> Rl = ResidualStation(fl, cst, x1, xt_d, U1, Utl, auxd, auxd);
		const std::array<D, 3> Rt = ResidualStation(ft, cst, xt_d, x2, Utt, U2, auxd, auxd);
		return {Rl[0] + Rt[0], Rl[1] + Rt[1], Rl[2] + Rt[2]};
	}

	// ---- initialisation by marching (init_boundary_layer) ------------------------------
	void InitBoundaryLayer() {
		const double Hmaxl = 3.8, Hmaxt = 2.5;
		const std::vector<double> ueinv = UeInv();
		U.assign(Nsys, S4{0, 0, 0, 0});
		turb.assign(Nsys, 0);
		for (int si = 0; si < 3; ++si) {
			const std::vector<int>& I = Is[si];
			const int n = static_cast<int>(I.size());
			std::vector<double> xs(n), ue(n), aux(n, 0.0);
			double uemax = 0.0;
			for (int k = 0; k < n; ++k) { xs[k] = xi[I[k]]; ue[k] = ueinv[I[k]]; uemax = std::max(uemax, std::fabs(ue[k])); }
			for (int k = 0; k < n; ++k) ue[k] = std::max(ue[k], 1e-8 * uemax);
			if (si == 2) for (int k = 0; k < n; ++k) aux[k] = wgap[k];
			std::vector<S4> Us(n, S4{0, 0, 0, 0});
			bool param_turb = (si == 2);
			int i0 = 0;
			if (si < 2) {
				double K; bool hitstag;
				if (FirstNodeAtStagnation(si)) { K = ue[1] / xs[1]; hitstag = true; } else { K = ue[0] / xs[0]; hitstag = false; }
				const double nu = cst.mu0 / cst.rho0;
				const double th = std::sqrt(0.45 * nu / (6.0 * K)), ds = 2.2 * th;
				const double xst = 1e-6;
				S4 Ust{th, ds, 0.0, K * xst};
				for (int it = 0; it < 20; ++it) {
					const SD Ud = Seed(Ust, 0);
					const std::array<D, 3> Rd = ResidualStation(FlagsFor(si, false, true), cst, D(xst), D(xst), Ud, Ud, D(0.0), D(0.0));
					std::vector<double> Rv{Rd[0].v, Rd[1].v, Rd[2].v};
					if (Norm(Rv) < 1e-10) break;
					std::vector<double> A(9), b(3), dU;
					for (int r = 0; r < 3; ++r) { for (int c = 0; c < 3; ++c) A[r * 3 + c] = Rd[r].d[c]; b[r] = -Rv[r]; }
					if (!SolveSmall(3, A, b, dU)) break;
					const double dm = std::max(std::fabs(dU[0] / Ust[0]), std::fabs(dU[1] / Ust[1]));
					const double om = dm < 0.2 ? 1.0 : 0.2 / dm;
					for (int k = 0; k < 3; ++k) Ust[k] += om * dU[k];
				}
				if (hitstag) { Us[0] = Ust; Us[0][3] = ue[0]; i0 = 1; }
				Us[i0] = Ust;
				Us[i0][3] = ue[i0];
			} else {
				// the wake start from the two trailing-edge states
				const S4& Ul = U[Is[0].back()];
				const S4& Uu = U[Is[1].back()];
				const bool tl = turb[Is[0].back()] != 0, tu = turb[Is[1].back()] != 0;
				const std::array<double, 3> Rw = ResidualWakeStart<double>(cst, Ul, Uu, S4{0, 0, 0, 0}, te.hTE, tl, tu);
				Us[0] = S4{-Rw[0], -Rw[1], -Rw[2], ue[0]};
				param_turb = true;
				turb[I[0]] = 1;
			}
			bool tran = false;
			int i = i0 + 1;
			double ct_guess = 0.0;
			while (i < n) {
				Us[i] = Us[i - 1];
				Us[i][3] = ue[i];
				if (tran) { ct_guess = Cttr(Us[i], FlagsFor(si, true), cst); Us[i][2] = ct_guess; }
				turb[I[i]] = (tran || param_turb) ? 1 : 0;
				bool direct = true;
				const int nNewton = 30, iNswitch = 12;
				double Hktgt = 0.0;
				int it = 0;
				for (it = 0; it < nNewton; ++it) {
					std::array<D, 3> Rd;
					if (tran) {
						double xt;
						Rd = TransitionResidual(si, xs[i - 1], xs[i], Us[i - 1], Us[i], aux[i - 1], xt, forced[si] ? ForcedXi(si, I[i - 1], I[i]) : -1.0);
						xt_last = xt;
					} else {
						Rd = ResidualStation(FlagsFor(si, param_turb), cst, D(xs[i - 1]), D(xs[i]), Seed(Us[i - 1], 0), Seed(Us[i], 4), D(aux[i - 1]), D(aux[i]));
					}
					std::vector<double> Rv{Rd[0].v, Rd[1].v, Rd[2].v};
					if (Norm(Rv) < 1e-10) break;
					std::vector<double> dU(4, 0.0);
					if (direct) {
						std::vector<double> A(9), b(3), x;
						for (int r = 0; r < 3; ++r) { for (int c = 0; c < 3; ++c) A[r * 3 + c] = Rd[r].d[4 + c]; b[r] = -Rv[r]; }
						if (!SolveSmall(3, A, b, x)) break;
						dU = {x[0], x[1], x[2], 0.0};
					} else {
						const D hk = Hk(Seed(Us[i], 4), FlagsFor(si, param_turb));
						std::vector<double> A(16), b(4), x;
						for (int r = 0; r < 3; ++r) { for (int c = 0; c < 4; ++c) A[r * 4 + c] = Rd[r].d[4 + c]; b[r] = -Rv[r]; }
						for (int c = 0; c < 4; ++c) A[3 * 4 + c] = hk.d[4 + c];
						b[3] = Hktgt - hk.v;
						if (!SolveSmall(4, A, b, x)) break;
						dU = x;
					}
					double dm = std::max(std::fabs(dU[0] / Us[i - 1][0]), std::fabs(dU[1] / Us[i - 1][1]));
					if (!direct) dm = std::max(dm, std::fabs(dU[3] / Us[i - 1][3]));
					if (param_turb) dm = std::max(dm, std::fabs(dU[2] / Us[i - 1][2]));
					else if (direct) dm = std::max(dm, std::fabs(dU[2] / 10.0));
					const double om = dm > 0.3 ? 0.3 / dm : 1.0;
					S4 Ui = Us[i];
					for (int k = 0; k < 4; ++k) Ui[k] += om * dU[k];
					if (param_turb) Ui[2] = std::max(std::min(Ui[2], 0.3), 1e-7);
					const double Hmax = param_turb ? Hmaxt : Hmaxl;
					const double hk_i = Ui[1] / Ui[0];
					if (direct && (hk_i > Hmax || it > iNswitch)) {
						direct = false;
						const double hk1 = Us[i - 1][1] / Us[i - 1][0];
						const double Hkr = (xs[i] - xs[i - 1]) / Us[i - 1][0];
						if (si == 2) {
							double H2 = hk1;
							for (int k = 0; k < 6; ++k) H2 -= (H2 + 0.03 * Hkr * std::pow(H2 - 1.0, 3) - hk1) / (1.0 + 0.09 * Hkr * (H2 - 1.0) * (H2 - 1.0));
							Hktgt = std::max(H2, 1.01);
						} else if (param_turb) Hktgt = hk1 - 0.15 * Hkr;
						else Hktgt = hk1 + 0.03 * Hkr;
						if (si != 2) Hktgt = std::max(Hktgt, Hmax);
						if (it > iNswitch) { Us[i] = Us[i - 1]; Us[i][3] = ue[i]; }
					} else {
						Us[i] = Ui;
					}
				}
				if (it >= nNewton - 1) {
					warnings.push_back("boundary-layer initialisation did not converge on surface " + std::to_string(si) + " node " + std::to_string(i));
					Us[i] = Us[i - 1];
					Us[i][3] = ue[i];
					if (si < 2) {
						const double f = std::sqrt(xs[i] / xs[i - 1]);
						Us[i][0] = Us[i - 1][0] * f;
						Us[i][1] = Us[i - 1][1] * f;
					} else {
						const double rlen = (xs[i] - xs[i - 1]) / (10.0 * Us[i - 1][1]);
						Us[i][1] = (Us[i - 1][1] + Us[i - 1][0] * rlen) / (1.0 + rlen);
					}
				}
				// transition check (natural or forced)
				if (!param_turb && !tran) {
					if (Us[i][2] > cst.ncrit) tran = true;
					else if (si < 2 && ForcedXi(si, I[i - 1], I[i]) >= 0.0) { tran = true; forced[si] = true; }
					if (tran) continue; // redo this station with the transition residual
				}
				if (tran) {
					StoreTransition(si, I[i - 1], I[i], xt_last);
					param_turb = true;
					tran = false;
				}
				++i;
			}
			for (int k = 0; k < n; ++k) U[I[k]] = Us[k];
		}
	}

	double xt_last = 0.0;

	void StoreTransition(int si, int i0, int i1, double xt) {
		if (si > 1) return;
		const double xi0 = xi[i0], xi1 = xi[i1];
		const double x0 = g->nodes[i0].x, x1 = g->nodes[i1].x;
		Xt_xi[si] = xt;
		Xt_x[si] = x0 + (xt - xi0) / (xi1 - xi0) * (x1 - x0);
	}

	// ---- the global system (build_glob_sys) ------------------------------------------
	void BuildGlobalSystem() {
		const int nR = 3 * Nsys, nU = 4 * Nsys;
		R.assign(nR, 0.0);
		R_U.assign(static_cast<size_t>(nR) * nU, 0.0);
		R_x.assign(static_cast<size_t>(nR) * Nsys, 0.0);
		auto addU = [&](int row, int node, int k, double v) { R_U[static_cast<size_t>(row) * nU + 4 * node + k] += v; };
		auto addX = [&](int row, int node, double v) { R_x[static_cast<size_t>(row) * Nsys + node] += v; };
		// Scatter a residual dual with seeds (nodeA 0-3, nodeB 4-7, xA 8, xB 9).
		auto scatter2 = [&](int row, const D& r, int nodeA, int nodeB) {
			R[row] += r.v;
			for (int k = 0; k < 4; ++k) { addU(row, nodeA, k, r.d[k]); addU(row, nodeB, k, r.d[4 + k]); }
			addX(row, nodeA, r.d[8]);
			addX(row, nodeB, r.d[9]);
		};
		for (int si = 0; si < 3; ++si) {
			const std::vector<int>& I = Is[si];
			const int n = static_cast<int>(I.size());
			const int i0 = FirstNodeAtStagnation(si) ? 1 : 0;
			if (si < 2) {
				const int a = I[i0], b = I[i0 + 1];
				const SD Ua = Seed(U[a], 0), Ub = Seed(U[b], 4);
				const D xa = D::Var(xi[a], 8), xb = D::Var(xi[b], 9);
				const SD Ust = StagnationState(Ua, Ub, xa, xb);
				const double xst = 1e-6;
				const std::array<D, 3> R1 = ResidualStation(FlagsFor(si, false, true), cst, D(xst), D(xst), Ust, Ust, D(0.0), D(0.0));
				for (int k = 0; k < 3; ++k) scatter2(3 * a + k, R1[k], a, b);
				if (i0 == 1) {
					const int f = I[0];
					for (int k = 0; k < 3; ++k) {
						const int row = 3 * f + k;
						R[row] += U[f][k] - Ust[k].v;
						addU(row, f, k, 1.0);
						for (int c = 0; c < 4; ++c) { addU(row, a, c, -Ust[k].d[c]); addU(row, b, c, -Ust[k].d[4 + c]); }
						addX(row, a, -Ust[k].d[8]);
						addX(row, b, -Ust[k].d[9]);
					}
				}
			} else {
				const int il = Is[0].back(), iu = Is[1].back(), iw = Is[2][0];
				const SD Ul = Seed(U[il], 0), Uu = Seed(U[iu], 4), Uw = Seed(U[iw], 8);
				const std::array<D, 3> R1 = ResidualWakeStart(cst, Ul, Uu, Uw, te.hTE, turb[il] != 0, turb[iu] != 0);
				for (int k = 0; k < 3; ++k) {
					const int row = 3 * iw + k;
					R[row] += R1[k].v;
					for (int c = 0; c < 4; ++c) { addU(row, il, c, R1[k].d[c]); addU(row, iu, c, R1[k].d[4 + c]); addU(row, iw, c, R1[k].d[8 + c]); }
				}
			}
			for (int i = i0 + 1; i < n; ++i) {
				const int a = I[i - 1], b = I[i];
				const bool tran = (turb[a] != 0) != (turb[b] != 0);
				std::array<D, 3> Ri;
				if (tran) {
					double xt;
					const double fxi = ForcedXi(si, a, b);
					Ri = TransitionResidual(si, xi[a], xi[b], U[a], U[b], si == 2 ? wgap[i - 1] : 0.0, xt, fxi);
					if (si < 2) forced[si] = fxi >= 0.0;
					StoreTransition(si, a, b, xt);
				} else {
					const double auxa = si == 2 ? wgap[i - 1] : 0.0, auxb = si == 2 ? wgap[i] : 0.0;
					Ri = ResidualStation(FlagsFor(si, turb[a] != 0), cst, D::Var(xi[a], 8), D::Var(xi[b], 9), Seed(U[a], 0), Seed(U[b], 4), D(auxa), D(auxb));
				}
				for (int k = 0; k < 3; ++k) scatter2(3 * b + k, Ri[k], a, b);
			}
		}
		// The stagnation point moves with ue at the two nodes about it:
		// R_ue += R_x x_st st_ue, x_st = -sgnue on the airfoil, -1 in the wake.
		for (int row = 0; row < nR; ++row) {
			double rst = 0.0;
			const double* rx = &R_x[static_cast<size_t>(row) * Nsys];
			for (int j = 0; j < N; ++j) rst += rx[j] * (-sgnue[j]);
			for (int j = N; j < Nsys; ++j) rst += rx[j] * (-1.0);
			R_U[static_cast<size_t>(row) * nU + 4 * Istag[0] + 3] += rst * sstag_ue[0];
			R_U[static_cast<size_t>(row) * nU + 4 * Istag[1] + 3] += rst * sstag_ue[1];
		}
	}

	// Solves the global system for the update (solve_glob).
	bool SolveGlobal(std::vector<double>& dU) {
		const int nU = 4 * Nsys, nR = 3 * Nsys;
		std::vector<double> ue(Nsys), ds(Nsys);
		double uemax = 0.0;
		for (int i = 0; i < Nsys; ++i) { ue[i] = U[i][3]; ds[i] = U[i][1]; uemax = std::max(uemax, std::fabs(ue[i])); }
		for (int i = 0; i < Nsys; ++i) ue[i] = std::max(ue[i], 1e-10 * uemax);
		const std::vector<double> ueinv = UeInv();
		std::vector<double> A(static_cast<size_t>(nU) * nU, 0.0), b(nU, 0.0);
		for (int r = 0; r < nR; ++r) {
			std::copy(R_U.begin() + static_cast<size_t>(r) * nU, R_U.begin() + static_cast<size_t>(r + 1) * nU, A.begin() + static_cast<size_t>(r) * nU);
			b[r] = R[r];
		}
		for (int k = 0; k < Nsys; ++k) {
			const int row = nR + k;
			double* ar = &A[static_cast<size_t>(row) * nU];
			double s = 0.0;
			const double* um = &ue_m[static_cast<size_t>(k) * Nsys];
			for (int j = 0; j < Nsys; ++j) {
				s += um[j] * ds[j] * ue[j];
				ar[4 * j + 3] -= um[j] * ds[j];
				ar[4 * j + 1] = -um[j] * ue[j];
			}
			ar[4 * k + 3] += 1.0;
			b[row] = ue[k] - (ueinv[k] + s);
		}
		DenseLU lu(std::move(A), nU);
		if (!lu.ok()) return false;
		for (double& v : b) v = -v;
		lu.Solve(b);
		dU = std::move(b);
		return true;
	}

	// update_state: under-relaxed update with the paper's limits.
	void UpdateState(const std::vector<double>& dU) {
		double ctmax = 0.0;
		for (int i = 0; i < Nsys; ++i) if (turb[i]) ctmax = std::max(ctmax, U[i][2]);
		double omega = 1.0;
		const char* why = "none";
		int why_node = -1;
		auto limit = [&](double om, const char* reason, int node) { if (om < omega) { omega = om; why = reason; why_node = node; } };
		for (int k = 0; k < 2; ++k) {
			double fmin = 0.0; int imin = -1;
			for (int i = 0; i < Nsys; ++i) { const double f = dU[4 * i + k] / U[i][k]; if (f < fmin) { fmin = f; imin = i; } }
			if (fmin < -0.5) limit(std::fabs(0.5 / fmin), k == 0 ? "theta decrease" : "delta* decrease", imin);
		}
		for (int i = 0; i < Nsys; ++i) {
			const double u = U[i][2], du = dU[4 * i + 2];
			if (!turb[i] && u < 0.2) continue;
			if (turb[i] && u < 0.1 * ctmax) continue;
			if (u == 0.0 || du == 0.0) continue;
			if (u + du < 0.0) limit(0.8 * std::fabs(u / du), "negative amp/ctau", i);
		}
		double dumax = 0.0; int imax = -1;
		for (int i = 0; i < Nsys; ++i) if (!turb[i] && std::fabs(dU[4 * i + 2]) > dumax) { dumax = std::fabs(dU[4 * i + 2]); imax = i; }
		if (dumax > 0.0) limit(2.0 / dumax, "amp change", imax);
		dumax = 0.0; imax = -1;
		for (int i = 0; i < Nsys; ++i) if (turb[i] && std::fabs(dU[4 * i + 2]) > dumax) { dumax = std::fabs(dU[4 * i + 2]); imax = i; }
		if (dumax > 0.0) limit(0.05 / dumax, "ctau change", imax);
		double fmax = 0.0; imax = -1;
		for (int i = 0; i < Nsys; ++i) if (std::fabs(dU[4 * i + 3]) / cst.Vinf > fmax) { fmax = std::fabs(dU[4 * i + 3]) / cst.Vinf; imax = i; }
		if (fmax > 0.0) limit(0.2 / fmax, "ue change", imax);
		if (opt.verbose && omega < 1.0) std::printf("    limited by %s at node %d (omega %.4f; state th %.3e ds %.3e sa %.3e ue %.3e, dU %.2e %.2e %.2e %.2e)\n", why, why_node, omega,
			why_node >= 0 ? U[why_node][0] : 0.0, why_node >= 0 ? U[why_node][1] : 0.0, why_node >= 0 ? U[why_node][2] : 0.0, why_node >= 0 ? U[why_node][3] : 0.0,
			why_node >= 0 ? dU[4 * why_node] : 0.0, why_node >= 0 ? dU[4 * why_node + 1] : 0.0, why_node >= 0 ? dU[4 * why_node + 2] : 0.0, why_node >= 0 ? dU[4 * why_node + 3] : 0.0);
		for (int i = 0; i < Nsys; ++i) for (int k = 0; k < 4; ++k) U[i][k] += omega * dU[4 * i + k];
		// Hk floors and negative ctau
		for (int si = 0; si < 3; ++si) {
			const double Hkmin = (si == 2) ? 1.00005 : 1.02;
			for (int j : Is[si]) {
				const double hk = U[j][1] / U[j][0];
				if (hk < Hkmin) U[j][1] += 2.0 * (Hkmin - hk) * U[j][0];
			}
		}
		for (int i = 0; i < Nsys; ++i) if (turb[i] && U[i][2] < 0.0) U[i][2] = 0.1 * ctmax;
		last_omega = omega;
	}
	double last_omega = 1.0;

	// Whether a surface's first node sits at the stagnation point itself (a
	// symmetric section at zero incidence has a node exactly there): its state
	// is then the extrapolated stagnation state rather than a station.
	bool FirstNodeAtStagnation(int si) const {
		if (si > 1 || Is[si].size() < 3) return false;
		return xi[Is[si][0]] == 0.0; // pinned by MoveStagnation / FindStagnation
	}
	static constexpr double kStagTol = 1e-2;

	// stagpoint_move: moves the stagnation point with the viscous ue. A node
	// whose ue is within a small tolerance of zero is the stagnation point
	// itself; the panel is not changed for it (which would otherwise flip
	// the surfaces every iteration), and it is given zero weight.
	bool MoveStagnation() {
		int I0 = Istag[0], I1 = Istag[1];
		bool newpanel = true;
		const double tol = kStagTol * cst.Vinf;
		if (U[I1][3] < -tol) {
			int j = I1;
			for (; j < N; ++j) if (U[j][3] > -tol) break;
			if (j >= N - 1 && U[j][3] <= 0.0) return false;
			for (int k = I1; k < j; ++k) U[k][3] *= -1.0;
			I0 = j - 1; I1 = j;
		} else if (U[I0][3] < -tol) {
			int j = I0;
			for (; j >= 0; --j) if (U[j][3] > -tol) break;
			if (j <= 0 && U[j][3] <= 0.0) return false;
			for (int k = j + 1; k <= I0; ++k) U[k][3] *= -1.0;
			I0 = j; I1 = j + 1;
		} else newpanel = false;
		const double ue0 = std::max(U[I0][3], 0.0), ue1 = std::max(U[I1][3], 0.0), S0 = g->s[I0], S1 = g->s[I1];
		if (ue0 + ue1 <= 0.0) return false;
		if (ue1 < tol * std::max(ue0, 1e-300) || ue0 < tol * std::max(ue1, 1e-300)) {
			// Pinned to the node: the stagnation point is the node itself and
			// does not move with that node's (near-zero) edge velocity.
			const bool at1 = ue1 < tol * ue0;
			sstag = at1 ? S1 : S0;
			xstag = at1 ? g->nodes[I1] : g->nodes[I0];
			sstag_ue[0] = sstag_ue[1] = 0.0;
		} else {
			const double den = ue0 + ue1, w1 = ue1 / den, w2 = ue0 / den;
			sstag = w1 * S0 + w2 * S1;
			xstag = g->nodes[I0] * w1 + g->nodes[I1] * w2;
			sstag_ue[0] = ue1 * (S1 - S0) / (den * den);
			sstag_ue[1] = -ue0 * (S1 - S0) / (den * den);
		}
		ComputeXi();
		if (newpanel) {
			Istag[0] = I0; Istag[1] = I1;
			for (int i = 0; i <= I0; ++i) sgnue[i] = -1;
			for (int i = I0 + 1; i < N; ++i) sgnue[i] = 1;
			IdentifySurfaces();
			RebuildUeM();
		}
		return true;
	}

	// march_amplification: the last laminar node of a surface.
	int MarchAmplification(int si, std::vector<double>& sa_out) {
		const std::vector<int>& I = Is[si];
		const int n = static_cast<int>(I.size());
		sa_out.resize(n);
		for (int k = 0; k < n; ++k) sa_out[k] = U[I[k]][2];
		sa_out[0] = 0.0;
		const IblFlags fl = FlagsFor(si, false);
		int i = 1;
		for (; i < n; ++i) {
			if (ForcedXi(si, I[i - 1], I[i]) >= 0.0) break; // forced transition holds in this interval
			S4 U1 = U[I[i - 1]];
			U1[2] = sa_out[i - 1];
			S4 U2 = U[I[i]];
			if (turb[I[i]]) U2[2] = U1[2] * 1.01;
			const double dx = xi[I[i]] - xi[I[i - 1]];
			for (int it = 0; it < 20; ++it) {
				const double damp1 = Damp(U1, fl, cst);
				SD U2d = Const(U2);
				U2d[2] = D::Var(U2[2], 6);
				const D damp2 = Damp(U2d, fl, cst);
				const double damp = 0.5 * (damp1 + damp2.v);
				const double Ramp = U2[2] - U1[2] - damp * dx;
				if (std::fabs(Ramp) < 1e-12) break;
				const double Ramp_U = 1.0 - 0.5 * damp2.d[6] * dx;
				double du = -Ramp / Ramp_U;
				const double dmax = 0.5 * (1.01 - static_cast<double>(it) / 20.0);
				if (std::fabs(du) > dmax) du *= dmax / std::fabs(du);
				U2[2] += du;
			}
			if (U2[2] > cst.ncrit) break;
			sa_out[i] = U2[2];
		}
		return i - 1;
	}

	void UpdateTransition() {
		for (int si = 0; si < 2; ++si) {
			const std::vector<int>& I = Is[si];
			const int n = static_cast<int>(I.size());
			int ilam0 = n - 1;
			for (int k = 0; k < n; ++k) if (turb[I[k]]) { ilam0 = k - 1; break; }
			std::vector<double> sa_new;
			const int ilam = MarchAmplification(si, sa_new);
			if (ilam == ilam0) continue;
			if (ilam < ilam0) {
				// transition moved upstream: the nodes between become turbulent
				const double sa0 = Cttr(U[I[ilam + 1]], FlagsFor(si, true), cst);
				const double sa1 = (ilam0 < n - 1) ? U[I[ilam0 + 1]][2] : sa0;
				const double dx = xi[I[std::min(ilam0 + 1, n - 1)]] - xi[I[ilam + 1]];
				for (int i = ilam + 1; i <= ilam0; ++i) {
					double f = (dx == 0.0 || i == ilam + 1) ? 0.0 : (xi[I[i]] - xi[I[ilam + 1]]) / dx;
					if (ilam + 1 == ilam0) f = 1.0;
					U[I[i]][2] = sa0 + f * (sa1 - sa0);
					if (U[I[i]][2] <= 0.0) U[I[i]][2] = sa0;
					turb[I[i]] = 1;
				}
				for (int i = 0; i <= ilam; ++i) U[I[i]][2] = sa_new[i];
			} else {
				// transition moved downstream: the marched amplification fills the new laminar nodes
				for (int i = ilam0 < 0 ? 0 : ilam0; i <= ilam; ++i) turb[I[i]] = 0;
				for (int i = 0; i <= ilam; ++i) U[I[i]][2] = sa_new[i];
			}
		}
	}

	// ---- the coupled Newton loop (solve_coupled) ------------------------------------------
	void SolveCoupled() {
		converged = false;
		for (int it = 0; it < opt.max_newton; ++it) {
			BuildGlobalSystem();
			rnorm = Norm(R);
			iterations = it;
			if (opt.verbose) {
				int rmax = 0;
				for (int r = 1; r < 3 * Nsys; ++r) if (std::fabs(R[r]) > std::fabs(R[rmax])) rmax = r;
				int lam[2] = {-1, -1};
				for (int si = 0; si < 2; ++si) for (int k = 0; k < static_cast<int>(Is[si].size()); ++k) if (!turb[Is[si][k]]) lam[si] = k;
				std::printf("  [a=%.1f N=%d xft=%.2f] Newton %d: |R| = %.3e omega %.3f stag %d-%d s %.6f ue(%.2e, %.2e) | max |R| %.2e at node %d eq %d (x/c %.3f turb %d) | last laminar lower %d upper %d (xt %.4f %.4f)\n",
					alpha_deg, N, opt.xft_upper, it, rnorm, last_omega, Istag[0], Istag[1], sstag, U[Istag[0]][3], U[Istag[1]][3],
					std::fabs(R[rmax]), rmax / 3, rmax % 3, XcOf(std::min(rmax / 3, N - 1)), static_cast<int>(turb[rmax / 3]), lam[0], lam[1], Xt_x[0], Xt_x[1]);
			}
			if (!std::isfinite(rnorm)) { warnings.push_back("residual became non-finite"); return; }
			if (rnorm < opt.rtol) { converged = true; return; }
			std::vector<double> dU;
			if (!SolveGlobal(dU)) { warnings.push_back("singular global Jacobian"); return; }
			UpdateState(dU);
			if (!MoveStagnation()) { warnings.push_back("stagnation point lost"); return; }
			UpdateTransition();
		}
		BuildGlobalSystem();
		rnorm = Norm(R);
		iterations = opt.max_newton;
	}

	// Re-solve at a new incidence keeping the boundary-layer state (continuation).
	void SetAlpha(double new_alpha_deg) {
		alpha_deg = new_alpha_deg;
		alpha = alpha_deg * kPi / 180.0;
		gamma = inv.GammaAt(alpha);
		// the wake was traced at the original incidence; keep it (the paper's option)
		const std::vector<double> ueinv = UeInv();
		for (int i = 0; i < Nsys; ++i) U[i][3] = std::max(ueinv[i], 1e-8);
	}

	// ---- post-processing ----------------------------------------------------------------------
	void Fill(IblResult& r) const {
		r.alpha_deg = alpha_deg;
		r.converged = converged;
		r.iterations = iterations;
		r.residual_norm = rnorm;
		r.warnings = warnings;
		r.n_airfoil = N;
		r.n_wake = Nw;
		r.chord = g->chord;
		r.x_min = g->x_min;
		r.s_stag = sstag;
		r.x_stag = xstag.x;
		r.y_stag = xstag.y;
		r.lower = Is[0];
		r.upper = Is[1];
		r.wake = Is[2];
		r.nodes.resize(Nsys);
		std::vector<double> cp(N);
		for (int i = 0; i < Nsys; ++i) {
			IblNode& nd = r.nodes[i];
			const Vec2 p = i < N ? g->nodes[i] : g->wake[i - N];
			nd.x = p.x; nd.y = p.y;
			nd.xi = xi[i];
			nd.theta = U[i][0]; nd.delta_star = U[i][1]; nd.sa = U[i][2]; nd.ue = U[i][3];
			nd.turb = turb[i] != 0;
			const int si = (i >= N) ? 2 : (i <= Istag[0] ? 0 : 1);
			const IblFlags f = FlagsFor(si, nd.turb);
			nd.hk = U[i][1] / U[i][0];
			nd.cf = Cf(U[i], f, cst) * nd.ue * nd.ue / (cst.Vinf * cst.Vinf);
			nd.ret = Ret(U[i], cst);
			nd.cp = 1.0 - (nd.ue / cst.Vinf) * (nd.ue / cst.Vinf);
			if (i < N) cp[i] = nd.cp;
		}
		const PanelLoads le = IntegrateLoads(*g, cp, alpha, Vec2(g->x_min, 0.0));
		const PanelLoads c4 = IntegrateLoads(*g, cp, alpha, Vec2(g->x_min + 0.25 * g->chord, 0.0));
		r.cl = le.cl;
		r.cm_le = le.cm;
		r.cm_c4 = c4.cm;
		r.cdpi = le.cd;
		r.cl_inviscid = IntegrateLoads(*g, inv.CpAt(alpha), alpha, Vec2(g->x_min, 0.0)).cl;
		// Squire-Young from the last wake node
		{
			const S4& Uw = U[Nsys - 1];
			const double Hw = Uw[1] / Uw[0];
			r.cd = 2.0 * Uw[0] * std::pow(Uw[3] / cst.Vinf, 0.5 * (5.0 + Hw)) / g->chord;
		}
		// skin friction drag by trapezoidal integration from the stagnation point
		{
			double Df = 0.0;
			const double ca = std::cos(alpha), sa = std::sin(alpha);
			for (int si = 0; si < 2; ++si) {
				double cf1 = 0.0, ue1 = 0.0;
				Vec2 x1 = xstag;
				for (int j : Is[si]) {
					const IblFlags f = FlagsFor(si, turb[j] != 0);
					const double cf2 = Cf(U[j], f, cst), ue2 = U[j][3];
					const Vec2 x2 = g->nodes[j];
					const double dx = (x2.x - x1.x) * ca + (x2.y - x1.y) * sa;
					Df += 0.25 * (cf1 * ue1 * ue1 + cf2 * ue2 * ue2) * dx;
					cf1 = cf2; ue1 = ue2; x1 = x2;
				}
			}
			r.cdf = Df / (0.5 * cst.Vinf * cst.Vinf * g->chord);
		}
		r.cdp = r.cd - r.cdf;
		r.xt_lower = Xt_x[0] >= 0.0 ? (Xt_x[0] - g->x_min) / g->chord : -1.0;
		r.xt_upper = Xt_x[1] >= 0.0 ? (Xt_x[1] - g->x_min) / g->chord : -1.0;
		if (!turb[Is[0].back()]) r.xt_lower = -1.0;
		if (!turb[Is[1].back()]) r.xt_upper = -1.0;
		r.xt_lower_forced = forced[0];
		r.xt_upper_forced = forced[1];
	}
};

} // namespace

double IblJacobianCheck(const Mat& closed_contour, double alpha_deg, const IblOptions& options, int stride, bool verbose) {
	Solver S(closed_contour, alpha_deg, options);
	S.InitBoundaryLayer();
	if (!S.MoveStagnation()) return -1.0;
	const int Nsys = S.Nsys, nU = 4 * Nsys, nR = 3 * Nsys;
	// Full residual (boundary layer + ue rows) and Jacobian at the state.
	auto full_residual = [&](std::vector<double>& out) {
		S.BuildGlobalSystem();
		out.assign(nR + Nsys, 0.0);
		for (int r = 0; r < nR; ++r) out[r] = S.R[r];
		const std::vector<double> ueinv = S.UeInv();
		for (int k = 0; k < Nsys; ++k) {
			double s = 0.0;
			for (int j = 0; j < Nsys; ++j) s += S.ue_m[static_cast<size_t>(k) * Nsys + j] * S.U[j][1] * S.U[j][3];
			out[nR + k] = S.U[k][3] - (ueinv[k] + s);
		}
	};
	std::vector<double> R0;
	full_residual(R0);
	// the assembled Jacobian: boundary-layer rows from R_U, ue rows as in SolveGlobal
	std::vector<double> J(static_cast<size_t>(nR + Nsys) * nU, 0.0);
	for (int r = 0; r < nR; ++r) for (int c = 0; c < nU; ++c) J[static_cast<size_t>(r) * nU + c] = S.R_U[static_cast<size_t>(r) * nU + c];
	for (int k = 0; k < Nsys; ++k) {
		double* row = &J[static_cast<size_t>(nR + k) * nU];
		for (int j = 0; j < Nsys; ++j) {
			const double um = S.ue_m[static_cast<size_t>(k) * Nsys + j];
			row[4 * j + 3] -= um * S.U[j][1];
			row[4 * j + 1] -= um * S.U[j][3];
		}
		row[4 * k + 3] += 1.0;
	}
	double worst = 0.0;
	int worst_col = -1, worst_row = -1;
	const std::vector<S4> U0 = S.U;
	const int Istag0 = S.Istag[0], Istag1 = S.Istag[1];
	for (int c = 0; c < nU; c += stride) {
		const int node = c / 4, k = c % 4;
		const double h = 1e-5 * std::fabs(U0[node][k]) + 1e-12;
		std::vector<double> Rp, Rm;
		S.U = U0; S.U[node][k] += h;
		// The stagnation point depends on ue at its two nodes; the assembled
		// Jacobian carries that through R_x, so the difference must too.
		if (k == 3 && (node == Istag0 || node == Istag1)) S.MoveStagnation();
		full_residual(Rp);
		S.U = U0; S.U[node][k] -= h;
		if (k == 3 && (node == Istag0 || node == Istag1)) S.MoveStagnation();
		full_residual(Rm);
		S.U = U0;
		if (k == 3 && (node == Istag0 || node == Istag1)) S.MoveStagnation();
		for (int r = 0; r < nR + Nsys; ++r) {
			const double fd = (Rp[r] - Rm[r]) / (2.0 * h);
			const double an = J[static_cast<size_t>(r) * nU + c];
			// absolute floor: the residual's roundoff over the step, scaled by its size
			const double floor_abs = 1e-9 * (std::fabs(R0[r]) + 1.0) / h;
			const double err = std::fabs(fd - an) / (std::fabs(fd) + std::fabs(an) + floor_abs);
			if (err > worst) { worst = err; worst_col = c; worst_row = r; }
		}
	}
	if (verbose) {
		std::printf("Jacobian check: worst rel. error %.3e at row %d col %d (Nsys %d, Istag %d %d)\n", worst, worst_row, worst_col, Nsys, Istag0, Istag1);
		if (worst_col >= 0) {
			const int node = worst_col / 4, k = worst_col % 4;
			std::printf("  worst column: node %d state %d value %.6e; row %d is %s\n", node, k, U0[node][k], worst_row,
				worst_row < nR ? "a boundary-layer row" : "an edge-velocity row");
			for (double rel : {1e-4, 1e-5, 1e-6, 1e-7}) {
				const double h = rel * std::fabs(U0[node][k]) + 1e-14;
				std::vector<double> Rp, Rm;
				S.U = U0; S.U[node][k] += h; full_residual(Rp);
				S.U = U0; S.U[node][k] -= h; full_residual(Rm);
				S.U = U0;
				std::printf("    h rel %.0e: fd %.6e (R+ %.6e R- %.6e) analytic %.6e\n", rel, (Rp[worst_row] - Rm[worst_row]) / (2.0 * h), Rp[worst_row], Rm[worst_row], J[static_cast<size_t>(worst_row) * nU + worst_col]);
			}
			if (worst_row >= nR) {
				const int kk = worst_row - nR;
				std::printf("    ue_m[%d][%d] = %.6e, ue_%d = %.6e, ds_%d = %.6e\n", kk, node, S.ue_m[static_cast<size_t>(kk) * Nsys + node], node, U0[node][3], node, U0[node][1]);
			} else {
				const int rn = worst_row / 3;
				std::printf("    row of node %d (turb %d, xi %.6f, x/c %.4f), R0 %.3e; analytic nonzeros:", rn, S.turb[rn], S.xi[rn], S.XcOf(rn < S.N ? rn : S.N - 1), R0[worst_row]);
				for (int c2 = 0; c2 < nU; ++c2) { const double v = J[static_cast<size_t>(worst_row) * nU + c2]; if (v != 0.0) std::printf(" [n%d s%d %.2e]", c2 / 4, c2 % 4, v); }
				std::printf("\n");
			}
		}
		// Separate the two row families.
		double worst_bl = 0.0, worst_ue = 0.0;
		for (int c = 0; c < nU; c += stride) {
			const int node = c / 4, k = c % 4;
			const double h = 1e-6 * std::fabs(U0[node][k]) + 1e-12;
			std::vector<double> Rp, Rm;
			S.U = U0; S.U[node][k] += h; if (k == 3 && (node == Istag0 || node == Istag1)) S.MoveStagnation(); full_residual(Rp);
			S.U = U0; S.U[node][k] -= h; if (k == 3 && (node == Istag0 || node == Istag1)) S.MoveStagnation(); full_residual(Rm);
			S.U = U0; if (k == 3 && (node == Istag0 || node == Istag1)) S.MoveStagnation();
			for (int r = 0; r < nR + Nsys; ++r) {
				const double fd = (Rp[r] - Rm[r]) / (2.0 * h);
				const double an = J[static_cast<size_t>(r) * nU + c];
				const double err = std::fabs(fd - an) / (std::fabs(fd) + std::fabs(an) + 1e-4);
				(r < nR ? worst_bl : worst_ue) = std::fmax(r < nR ? worst_bl : worst_ue, err);
			}
		}
		std::printf("  boundary-layer rows worst %.3e, edge-velocity rows worst %.3e\n", worst_bl, worst_ue);
		// Where the coupling is large: the biggest entries of ue_sigma and ue_m,
		// and the terms of the first edge-velocity residual.
		const int Ns = Nsys - 2;
		double max_us = 0.0; int us_r = 0, us_c = 0;
		for (int r = 0; r < Nsys; ++r) for (int c = 0; c < Ns; ++c) { const double v = std::fabs(S.ue_sigma[static_cast<size_t>(r) * Ns + c]); if (v > max_us) { max_us = v; us_r = r; us_c = c; } }
		double max_um = 0.0; int um_r = 0, um_c = 0;
		for (int r = 0; r < Nsys; ++r) for (int c = 0; c < Nsys; ++c) { const double v = std::fabs(S.ue_m[static_cast<size_t>(r) * Nsys + c]); if (v > max_um) { max_um = v; um_r = r; um_c = c; } }
		std::printf("  max |ue_sigma| %.3e at (%d, %d); max |ue_m| %.3e at (%d, %d); N %d Nw %d\n", max_us, us_r, us_c, max_um, um_r, um_c, S.N, S.Nw);
		const std::vector<double> ueinv = S.UeInv();
		for (int k : {0, S.N - 1, S.N, S.Nsys - 1}) {
			double s = 0.0, big = 0.0; int bj = 0;
			for (int j = 0; j < Nsys; ++j) { const double t = S.ue_m[static_cast<size_t>(k) * Nsys + j] * S.U[j][1] * S.U[j][3]; s += t; if (std::fabs(t) > big) { big = std::fabs(t); bj = j; } }
			std::printf("  node %d: ue %.4f ueinv %.4f  D m = %.4f (largest term %.4f from node %d, ue_m %.3e m %.3e)\n", k, S.U[k][3], ueinv[k], s, big, bj, S.ue_m[static_cast<size_t>(k) * Nsys + bj], S.U[bj][1] * S.U[bj][3]);
		}
	}
	return worst;
}

IblResult SolveIblAirfoil(const Mat& closed_contour, double alpha_deg, const IblOptions& options) {
	IblResult result;
	const auto t0 = std::chrono::steady_clock::now();
	try {
		Solver S(closed_contour, alpha_deg, options);
		S.InitBoundaryLayer();
		if (!S.MoveStagnation()) {
			result.ok = false;
			result.error = "stagnation point lost after initialisation";
			return result;
		}
		S.SolveCoupled();
		if (!S.converged && options.continuation && std::fabs(alpha_deg) > 0.5) {
			// Angle-of-attack continuation: converge at a lower incidence and
			// walk up in steps, keeping the boundary-layer state.
			const int steps = std::max(2, static_cast<int>(std::ceil(std::fabs(alpha_deg) / 2.5)));
			bool ok_chain = true;
			Solver C(closed_contour, 0.0, options);
			C.InitBoundaryLayer();
			if (!C.MoveStagnation()) ok_chain = false;
			else C.SolveCoupled();
			if (ok_chain && C.converged) {
				for (int k = 1; k <= steps && ok_chain; ++k) {
					C.SetAlpha(alpha_deg * k / steps);
					C.SolveCoupled();
					if (!C.converged) ok_chain = false;
				}
			} else ok_chain = false;
			if (ok_chain) {
				C.Fill(result);
				result.warnings.push_back("converged by angle-of-attack continuation from 0 deg");
				result.ok = true;
				result.solve_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
				return result;
			}
			S.warnings.push_back("angle-of-attack continuation also failed; reporting the direct attempt");
		}
		S.Fill(result);
		result.ok = true;
	} catch (const std::exception& e) {
		result.ok = false;
		result.error = e.what();
	}
	result.solve_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
	return result;
}

} // namespace aerotelic
