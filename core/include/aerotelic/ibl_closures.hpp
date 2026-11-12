#pragma once

// The integral boundary-layer closures and station residuals of XFOIL as
// documented and re-implemented by Fidkowski (mfoil; appendix B and
// section IV of the paper), incompressible (M_e = 0: H_k = H, H** = 0,
// F_c = 1). Written as templates over the number type so that the same code
// evaluated with Dual<> (dual.hpp) yields the exact Jacobians.
//
// State at a node: U = [theta, delta*, sa, ue], sa = amplification factor
// n~ on laminar nodes or sqrt(c_tau) on turbulent ones. The closures take
// the flags of the station (wake / turbulent / similarity) in IblFlags.
//
// Every formula below is the one in mfoil's get_* / residual_* functions
// (MIT licence), including its floors and clips, which the paper's text
// omits.

#include <array>
#include <cmath>

#include "aerotelic/dual.hpp"

namespace aerotelic {
namespace ibl {

struct IblConstants {
	double Vinf = 1.0;
	double rho0 = 1.0;
	double mu0 = 1.0e-6;   // rho Vinf c / Re
	double ncrit = 9.0;
	double Klag = 5.6;     // shear lag constant
	double Dlr = 0.9;      // wall/wake dissipation length ratio
	double Cuq = 1.0;      // scales the uq term of the lag equation
	double CtauC = 1.8;    // c_tau initialisation constant
	double CtauE = 3.3;    // c_tau initialisation exponent
	double GA = 6.7, GB = 0.75, GC = 18.0; // G-beta locus
};

struct IblFlags {
	bool wake = false;
	bool turb = false;
	bool simi = false;
};

template <class T> using State = std::array<T, 4>;

// ---- helpers for the floors (keep a floored value with zero derivative) ----
template <class T> T Floor(const T& x, double lo) { return x < lo ? T(lo) : x; }
template <class T> T Ceil(const T& x, double hi) { return x > hi ? T(hi) : x; }

// ---- basic quantities ---------------------------------------------------------

template <class T> T H(const State<T>& U) { return U[1] / U[0]; }
template <class T> T Hk(const State<T>& U, const IblFlags&) { return U[1] / U[0]; }
template <class T> T Ret(const State<T>& U, const IblConstants& c) { return c.rho0 * U[0] * U[3] / c.mu0; }

template <class T> T HkFloored(const State<T>& U, const IblFlags& f) {
	T hk = Hk(U, f);
	return Floor(hk, f.wake ? 1.00005 : 1.05);
}

// Kinetic energy shape parameter H* (get_Hs).
template <class T> T Hs(const State<T>& U, const IblFlags& f, const IblConstants& c) {
	using std::log;
	const T hk = HkFloored(U, f);
	if (f.turb) {
		const double Hsmin = 1.5, dHsinf = 0.015;
		const T ret = Ret(U, c);
		T Ho = T(4.0);
		if (ret > 400.0) Ho = 3.0 + 400.0 / ret;
		T Reb = ret;
		if (ret < 200.0) Reb = T(200.0);
		if (hk < Ho) {
			const T Hr = (Ho - hk) / (Ho - 1.0);
			const T aa = (2.0 - Hsmin - 4.0 / Reb) * Hr * Hr;
			return Hsmin + 4.0 / Reb + aa * 1.5 / (hk + 0.5);
		}
		const T lrb = log(Reb);
		const T aa = hk - Ho + 4.0 / lrb;
		const T bb = 0.007 * lrb / (aa * aa) + dHsinf / hk;
		return Hsmin + 4.0 / Reb + (hk - Ho) * (hk - Ho) * bb;
	}
	const T a = hk - 4.35;
	if (hk < 4.35) {
		const T num = 0.0111 * a * a - 0.0278 * a * a * a;
		return num / (hk + 1.0) + 1.528 - 0.0002 * (a * hk) * (a * hk);
	}
	return 0.015 * a * a / hk + 1.528;
}

// Boundary-layer thickness delta (get_de), limited to 12 theta.
template <class T> T De(const State<T>& U, const IblFlags& f) {
	const T hk = Hk(U, f);
	const T aa = 3.15 + 1.72 / (hk - 1.0);
	T de = U[0] * aa + U[1];
	if (de > 12.0 * U[0]) de = 12.0 * U[0];
	return de;
}

// Skin friction coefficient (get_cf); zero in the wake.
template <class T> T Cf(const State<T>& U, const IblFlags& f, const IblConstants& c) {
	using std::log; using std::exp; using std::tanh; using std::pow;
	if (f.wake) return T(0.0);
	const T hk = Hk(U, f);
	const T ret = Ret(U, c);
	if (f.turb) {
		T aa = -1.33 * hk;
		if (aa < -17.0) aa = -20.0 + 3.0 * exp((aa + 17.0) / 3.0);
		T bb = log(ret);
		if (bb < 3.0) bb = T(3.0);
		bb = bb / std::log(10.0);
		const T cc = -1.74 - 0.31 * hk;
		const T dd = tanh(4.0 - hk / 0.875);
		const T cf0 = 0.3 * exp(aa) * pow(bb, cc);
		return cf0 + 1.1e-4 * (dd - 1.0);
	}
	T num;
	if (hk < 5.5) {
		const T t = 5.5 - hk;
		num = 0.0727 * t * t * t / (hk + 1.0) - 0.07;
	} else {
		const T t = 1.0 - 1.0 / (hk - 4.5);
		num = 0.015 * t * t - 0.07;
	}
	return num / ret;
}

// Normalised wall slip velocity (get_Us).
template <class T> T Us(const State<T>& U, const IblFlags& f, const IblConstants& c) {
	const T hs = Hs(U, f, c);
	const T hk = HkFloored(U, f);
	const T h = H(U);
	const double bi = 1.0 / c.GB;
	T us = 0.5 * hs * (1.0 - bi * (hk - 1.0) / h);
	if (!f.wake && us > 0.95) us = T(0.98);
	if (!f.wake && us > 0.99995) us = T(0.99995);
	return us;
}

// Dissipation coefficient parts (get_cDi_*): all return 2 c_D / H* ("cDi").
template <class T> T CDiLam(const State<T>& U, const IblFlags& f, const IblConstants& c) {
	using std::pow;
	const T hk = Hk(U, f);
	const T ret = Ret(U, c);
	T num;
	if (hk < 4.0) num = 0.00205 * pow(4.0 - hk, 5.5) + 0.207;
	else { const T h1 = hk - 4.0; num = -0.0016 * h1 * h1 / (1.0 + 0.02 * h1 * h1) + 0.207; }
	return num / ret;
}
template <class T> T CDiTurbWall(const State<T>& U, const IblFlags& f, const IblConstants& c) {
	using std::log; using std::tanh;
	if (f.wake) return T(0.0);
	const T cf = Cf(U, f, c), hk = Hk(U, f), hs = Hs(U, f, c), us = Us(U, f, c), ret = Ret(U, c);
	const T lr = log(ret);
	const T Hmin = 1.0 + 2.1 / lr;
	const T aa = tanh((hk - 1.0) / (Hmin - 1.0));
	const T fac = 0.5 + 0.5 * aa;
	return 0.5 * cf * us * (2.0 / hs) * fac;
}
template <class T> T CDiLamWake(const State<T>& U, const IblFlags& fin, const IblConstants& c) {
	IblFlags f = fin; f.turb = false;
	const T hk = Hk(U, f), hs = Hs(U, f, c), ret = Ret(U, c);
	const T num = 2.0 * 1.1 * (1.0 - 1.0 / hk) * (1.0 - 1.0 / hk) * (1.0 / hk);
	return num / (hs * ret);
}
template <class T> T CDiOuter(const State<T>& U, const IblFlags& f, const IblConstants& c) {
	if (!f.turb) return T(0.0);
	const T hs = Hs(U, f, c), us = Us(U, f, c);
	const T ct = U[2] * U[2];
	return ct * (0.995 - us) * 2.0 / hs;
}
template <class T> T CDiLamStress(const State<T>& U, const IblFlags& f, const IblConstants& c) {
	const T hs = Hs(U, f, c), us = Us(U, f, c), ret = Ret(U, c);
	const T num = 0.15 * (0.995 - us) * (0.995 - us) * 2.0;
	return num / (hs * ret);
}
template <class T> T CDi(const State<T>& U, const IblFlags& f, const IblConstants& c) {
	if (!f.turb) return CDiLam(U, f, c);
	T cdi(0.0), cdil;
	if (!f.wake) { cdi += CDiTurbWall(U, f, c); cdil = CDiLam(U, f, c); }
	else cdil = CDiLamWake(U, f, c);
	cdi += CDiOuter(U, f, c);
	cdi += CDiLamStress(U, f, c);
	if (cdil > cdi) cdi = cdil;
	if (f.wake) cdi = 2.0 * cdi;
	return cdi;
}

// Amplification rate dn~/dxi (get_damp), with mfoil's extra term near n_crit.
template <class T> T Damp(const State<T>& U, const IblFlags& f, const IblConstants& c) {
	using std::log; using std::exp; using std::tanh; using std::pow;
	const T hk = Floor(Hk(U, f), 1.05);
	const T ret = Ret(U, c);
	const T th = U[0];
	const T Hmi = 1.0 / (hk - 1.0);
	const T aa = 2.492 * pow(Hmi, 0.43);
	const T bb = tanh(14.0 * Hmi - 9.24);
	const T lrc = aa + 0.7 * (bb + 1.0);
	const T lr = log(ret) / std::log(10.0);
	const double dl = 0.1;
	T damp(0.0);
	if (lr >= lrc - dl) {
		const T rn = (lr - (lrc - dl)) / (2.0 * dl);
		T rf;
		if (rn >= 1.0) rf = T(1.0);
		else rf = 3.0 * rn * rn - 2.0 * rn * rn * rn;
		const T ar = 3.87 * Hmi - 2.52;
		const T ex = exp(-ar * ar);
		const T da = 0.028 * (hk - 1.0) - 0.0345 * ex;
		const T af = -0.05 + 2.7 * Hmi - 5.5 * Hmi * Hmi + 3.0 * Hmi * Hmi * Hmi + 0.1 * exp(-20.0 * Hmi);
		damp = rf * af * da / th;
	}
	const T nx = 5.0 * (U[2] - c.ncrit);
	const T eex = 1.0 + tanh(nx);
	damp += eex * 0.001 / th;
	return damp;
}

// Equilibrium sqrt(c_tau) (get_cteq).
template <class T> T Cteq(const State<T>& U, const IblFlags& f, const IblConstants& c) {
	using std::sqrt;
	const double CC = 0.5 / (c.GA * c.GA * c.GB);
	const T hk = Hk(U, f), hs = Hs(U, f, c), h = H(U), ret = Ret(U, c), us = Us(U, f, c);
	T hk_use = hk, hkc;
	if (f.wake) {
		hk_use = Floor(hk, 1.00005);
		hkc = hk_use - 1.0;
	} else {
		const T hkl = Floor(hk, 1.05);
		hkc = hkl - 1.0 - c.GC / ret;
		if (hkc < 0.01) hkc = T(0.01);
	}
	const T num = CC * hs * (hk_use - 1.0) * hkc * hkc;
	const T den = (1.0 - us) * h * hk_use * hk_use;
	return sqrt(num / den);
}

// sqrt(c_tau) just after transition (get_cttr): on the airfoil.
template <class T> T Cttr(const State<T>& U, const IblFlags& fin, const IblConstants& c) {
	using std::exp;
	IblFlags f = fin; f.wake = false;
	const T cteq = Cteq(U, f, c);
	const T hk = Floor(Hk(U, f), 1.05);
	const T cc = c.CtauC * exp(-c.CtauE / (hk - 1.0));
	return cc * cteq;
}

// Equilibrium (1/ue) due/dxi (get_uq).
template <class T> T Uq(const T& ds, const T& cf, const T& hk_in, const T& ret, const IblFlags& f, const IblConstants& c) {
	double A = c.GA, C = c.GC;
	if (f.wake) { A *= c.Dlr; C = 0.0; }
	T hk = hk_in;
	if (f.wake && hk < 1.00005) hk = T(1.00005);
	if (!f.wake && hk < 1.05) hk = T(1.05);
	T hkc = hk - 1.0 - C / ret;
	if (hkc < 0.01) hkc = T(0.01);
	const T ratio = hkc / (A * hk);
	const T ut = 0.5 * cf - ratio * ratio;
	return ut / (c.GB * ds);
}

// Upwinding factor between two states (get_upw), 0.5 trapezoidal .. 1 backward Euler.
template <class T> T Upw(const State<T>& U1, const State<T>& U2, const IblFlags& f) {
	using std::log; using std::exp;
	const T hk1 = Hk(U1, f), hk2 = Hk(U2, f);
	const double C = f.wake ? 1.0 : 5.0;
	const T Huc = C / (hk2 * hk2);
	const T aa = (hk2 - 1.0) / (hk1 - 1.0);
	const T la = log(aa < 0.0 ? -aa : aa);
	T Hls = la * la;
	if (Hls > 15.0) Hls = T(15.0);
	return 1.0 - 0.5 * exp(-Hls * Huc);
}

template <class T> T Upwind(const T& upw, const T& f1, const T& f2) { return (1.0 - upw) * f1 + upw * f2; }

// ---- the station residual (residual_station): momentum, shape, amp/lag ----
// x1, x2: xi of the two nodes; aux: wake gap at each (zero on the airfoil).
template <class T> std::array<T, 3> ResidualStation(const IblFlags& f, const IblConstants& c,
	const T& x1, const T& x2, State<T> U1, State<T> U2, const T& aux1, const T& aux2) {
	using std::log;
	U1[1] -= aux1;
	U2[1] -= aux2;
	State<T> Um{0.5 * (U1[0] + U2[0]), 0.5 * (U1[1] + U2[1]), 0.5 * (U1[2] + U2[2]), 0.5 * (U1[3] + U2[3])};
	T thlog = log(U2[0] / U1[0]);
	T uelog = log(U2[3] / U1[3]);
	T xlog = log(x2 / x1);
	T dx = x2 - x1;
	const T upw = Upw(U1, U2, f);
	const T h = 0.5 * (H(U1) + H(U2));
	const T hs1 = Hs(U1, f, c), hs2 = Hs(U2, f, c);
	T hslog = log(hs2 / hs1);
	if (f.simi) {
		thlog = T(0.0);
		hslog = T(0.0);
		uelog = T(1.0);
		xlog = T(1.0);
		dx = 0.5 * (x1 + x2);
	}
	const T hw = 0.5 * (aux1 / U1[0] + aux2 / U2[0]);

	T Rlag;
	if (f.turb) {
		const T salog = log(U2[2] / U1[2]);
		const T de = Upwind(T(0.5), De(U1, f), De(U2, f));
		const T us = Upwind(T(0.5), Us(U1, f, c), Us(U2, f, c));
		const T hk = Upwind(upw, Hk(U1, f), Hk(U2, f));
		const T ret = Upwind(T(0.5), Ret(U1, c), Ret(U2, c));
		const T cf = Upwind(upw, Cf(U1, f, c), Cf(U2, f, c));
		const T dsa = 0.5 * (U1[1] + U2[1]);
		const T uq = Uq(dsa, cf, hk, ret, f, c);
		const T cteq = Upwind(upw, Cteq(U1, f, c), Cteq(U2, f, c));
		const T saa = Upwind(upw, U1[2], U2[2]);
		const T Clag = c.Klag / c.GB / (1.0 + us);
		const double ald = f.wake ? c.Dlr : 1.0;
		Rlag = Clag * (cteq - ald * saa) * dx - 2.0 * de * salog + 2.0 * de * (uq * dx - uelog) * c.Cuq;
	} else if (f.simi) {
		Rlag = U1[2] + U2[2];
	} else {
		const T damp = Upwind(T(0.5), Damp(U1, f, c), Damp(U2, f, c));
		Rlag = U2[2] - U1[2] - damp * dx;
	}

	// momentum: cf xi / theta averaged with the midpoint evaluation
	const T cfxt1 = Cf(U1, f, c) * x1 / U1[0];
	const T cfxt2 = Cf(U2, f, c) * x2 / U2[0];
	const T cfxtm = Cf(Um, f, c) * (0.5 * (x1 + x2)) / Um[0];
	const T cfxt = 0.25 * cfxt1 + 0.5 * cfxtm + 0.25 * cfxt2;
	const T Rmom = thlog + (2.0 + h + hw) * uelog - 0.5 * xlog * cfxt;

	// shape parameter: dissipation and friction upwinded
	const T cDixt = Upwind(upw, CDi(U1, f, c) * x1 / U1[0], CDi(U2, f, c) * x2 / U2[0]);
	const T cfxtu = Upwind(upw, cfxt1, cfxt2);
	const T Rshape = hslog + (1.0 - h - hw) * uelog + xlog * (0.5 * cfxtu - cDixt);

	return {Rmom, Rshape, Rlag};
}

// The state extrapolated to the stagnation point from the first two nodes
// (stagnation_state): theta, delta*, sa linearly to xi = 0, and ue = K xst
// with K from the quadratic fit ue = K xi through the two nodes.
template <class T> State<T> StagnationState(const State<T>& U1, const State<T>& U2, const T& x1, const T& x2, double xst = 1e-6) {
	const T dx = x2 - x1;
	const T w1 = x2 / dx, w2 = -x1 / dx;
	const T rx = x2 / x1;
	const T wk1 = rx / dx, wk2 = -1.0 / (rx * dx);
	const T K = wk1 * U1[3] + wk2 * U2[3];
	State<T> Ust;
	for (int k = 0; k < 3; ++k) Ust[k] = U1[k] * w1 + U2[k] * w2;
	Ust[3] = K * xst;
	return Ust;
}

// The first wake node's residual (wake_sys): theta and delta* are the sums
// of the two trailing-edge states (plus the gap), sqrt(c_tau) their
// theta-weighted mean, laminar trailing edges taking the transition value.
template <class T> std::array<T, 3> ResidualWakeStart(const IblConstants& c, const State<T>& Ul, const State<T>& Uu, const State<T>& Uw,
	double hTE, bool turb_l, bool turb_u) {
	IblFlags f; f.turb = true;
	const T ctl = turb_l ? Ul[2] : Cttr(Ul, f, c);
	const T ctu = turb_u ? Uu[2] : Cttr(Uu, f, c);
	const T thsum = Ul[0] + Uu[0];
	const T ctw = (ctl * Ul[0] + ctu * Uu[0]) / thsum;
	return {Uw[0] - (Ul[0] + Uu[0]), Uw[1] - (Ul[1] + Uu[1] + hTE), Uw[2] - ctw};
}

} // namespace ibl
} // namespace aerotelic
