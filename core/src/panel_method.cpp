#include "aerotelic/panel_method.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>

namespace aerotelic::panel {

namespace {

constexpr double kPi = 3.14159265358979323846;

// ---------------------------------------------------------------------------
// Dense linear algebra
// ---------------------------------------------------------------------------

struct Dense {
	int n = 0, m = 0;
	std::vector<double> a;
	Dense() = default;
	Dense(int rows, int cols) : n(rows), m(cols), a(static_cast<size_t>(rows) * cols, 0.0) {}
	double& operator()(int i, int j) { return a[static_cast<size_t>(i) * m + j]; }
	double operator()(int i, int j) const { return a[static_cast<size_t>(i) * m + j]; }
};

// Solves A x = b in place by LU with partial pivoting (appendix E, in core).
// Returns the smallest pivot over the largest, a cheap conditioning hint.
double SolveInPlace(Dense& A, std::vector<double>& b) {
	const int n = A.n;
	if (A.m != n || static_cast<int>(b.size()) != n) throw std::invalid_argument("panel method: the system is not square");
	double pmin = 1e300, pmax = 0.0;
	for (int k = 0; k < n; ++k) {
		int piv = k;
		double best = std::fabs(A(k, k));
		for (int i = k + 1; i < n; ++i) if (std::fabs(A(i, k)) > best) { best = std::fabs(A(i, k)); piv = i; }
		if (best < 1e-300) throw std::runtime_error("panel method: singular influence matrix at row " + std::to_string(k));
		pmin = std::min(pmin, best);
		pmax = std::max(pmax, best);
		if (piv != k) {
			for (int j = 0; j < n; ++j) std::swap(A(k, j), A(piv, j));
			std::swap(b[k], b[piv]);
		}
		const double d = A(k, k);
		for (int i = k + 1; i < n; ++i) {
			double f = A(i, k) / d;
			if (f == 0.0) continue;
			A(i, k) = f;
			double* ri = &A(i, 0);
			const double* rk = &A(k, 0);
			for (int j = k + 1; j < n; ++j) ri[j] -= f * rk[j];
			b[i] -= f * b[k];
		}
	}
	for (int i = n - 1; i >= 0; --i) {
		double s = b[i];
		for (int j = i + 1; j < n; ++j) s -= A(i, j) * b[j];
		b[i] = s / A(i, i);
	}
	return pmax > 0.0 ? pmin / pmax : 0.0;
}

// Small symmetric solve (normal equations of the fits), Gaussian elimination.
std::vector<double> SolveSmall(std::vector<double> A, std::vector<double> b, int n) {
	for (int k = 0; k < n; ++k) {
		int piv = k;
		for (int i = k + 1; i < n; ++i) if (std::fabs(A[i * n + k]) > std::fabs(A[piv * n + k])) piv = i;
		if (piv != k) { for (int j = 0; j < n; ++j) std::swap(A[k * n + j], A[piv * n + j]); std::swap(b[k], b[piv]); }
		double d = A[k * n + k];
		if (std::fabs(d) < 1e-300) throw std::runtime_error("panel method: a singular fit");
		for (int i = k + 1; i < n; ++i) {
			double f = A[i * n + k] / d;
			if (f == 0.0) continue;
			for (int j = k; j < n; ++j) A[i * n + j] -= f * A[k * n + j];
			b[i] -= f * b[k];
		}
	}
	std::vector<double> x(n);
	for (int i = n - 1; i >= 0; --i) {
		double s = b[i];
		for (int j = i + 1; j < n; ++j) s -= A[i * n + j] * x[j];
		x[i] = s / A[i * n + i];
	}
	return x;
}

// ---------------------------------------------------------------------------
// Panel geometry (appendix A)
// ---------------------------------------------------------------------------

struct PanelGeom {
	Vec3 origin;      // R0 (A.18), on the curved surface; the centre control point
	Vec3 ex, ey, ez;  // ξ̂, η̂, ζ̂
	LocalPanel local; // Σ and the curvature
	double delta = 0.0;
	Vec3 corners[4];
	Vec3 average;     // P0 (A.2)
	double diameter = 1.0;

	Vec3 ToLocal(const Vec3& p) const { Vec3 d = p - origin; return {Dot(d, ex), Dot(d, ey), Dot(d, ez)}; }
	Vec3 ToGlobalDir(const Vec3& v) const { return ex * v.x + ey * v.y + ez * v.z; }
};

struct Frame { Vec3 ex, ey, ez; };

Frame FlatFrame(const Vec3 P[4]) {
	Vec3 u = P[0] + P[3] - P[1] - P[2];
	Vec3 w = P[0] + P[1] - P[2] - P[3];
	Vec3 ez = Cross(u, w);
	if (LengthSq(ez) < 1e-30 * (LengthSq(u) * LengthSq(w) + 1e-300)) {
		// A collapsed panel: use the diagonals.
		u = P[2] - P[0];
		w = P[3] - P[1];
		ez = Cross(u, w);
	}
	Frame f;
	f.ex = Normalized(u);
	f.ez = Normalized(ez);
	f.ey = Cross(f.ez, f.ex);
	return f;
}

// Weighted least-squares quadratic ζ(ξ, η) over the points (A.9), (A.10);
// returns (ζ0, ζξ, ζη, ζξξ, ζξη, ζηη). Fewer than six distinct points: a
// plane (quadratic terms zero).
std::array<double, 6> FitQuadratic(const std::vector<Vec3>& local, const std::vector<double>& w, double scale) {
	const int n = static_cast<int>(local.size());
	int ncoef = n >= 6 ? 6 : (n >= 3 ? 3 : 1);
	std::vector<double> A(ncoef * ncoef, 0.0), b(ncoef, 0.0);
	for (int k = 0; k < n; ++k) {
		double xi = local[k].x / scale, eta = local[k].y / scale, z = local[k].z / scale;
		double V[6] = {1.0, xi, eta, 0.5 * xi * xi, xi * eta, 0.5 * eta * eta};
		for (int i = 0; i < ncoef; ++i) {
			b[i] += w[k] * V[i] * z;
			for (int j = 0; j < ncoef; ++j) A[i * ncoef + j] += w[k] * V[i] * V[j];
		}
	}
	for (int i = 0; i < ncoef; ++i) A[i * ncoef + i] += 1e-14 * (A[i * ncoef + i] + 1.0);
	std::vector<double> c = SolveSmall(A, b, ncoef);
	std::array<double, 6> out{};
	for (int i = 0; i < ncoef; ++i) out[i] = c[i];
	// Undo the scaling: ζ = scale · f(ξ/scale, η/scale).
	out[0] *= scale;                 // ζ0
	// ζξ, ζη unchanged; quadratic coefficients / scale
	out[3] /= scale; out[4] /= scale; out[5] /= scale;
	return out;
}

PanelGeom BuildPanel(const Network& net, int i, int j, bool curved, double own_weight) {
	PanelGeom g;
	const int I = i + 1, J = j + 1;
	g.corners[0] = net.Corner(I, J);
	g.corners[1] = net.Corner(I, J + 1);
	g.corners[2] = net.Corner(I + 1, J + 1);
	g.corners[3] = net.Corner(I + 1, J);
	g.average = (g.corners[0] + g.corners[1] + g.corners[2] + g.corners[3]) * 0.25;
	Frame f = FlatFrame(g.corners);
	g.ex = f.ex; g.ey = f.ey; g.ez = f.ez;
	// Centroid of the projected quadrilateral (A.3), (A.4).
	double xi[5], eta[5];
	for (int k = 0; k < 4; ++k) {
		Vec3 d = g.corners[k] - g.average;
		xi[k] = Dot(d, g.ex); eta[k] = Dot(d, g.ey);
	}
	xi[4] = xi[0]; eta[4] = eta[0];
	double D = 0.0, cx = 0.0, cy = 0.0;
	for (int k = 0; k < 4; ++k) {
		double Dk = 0.5 * (xi[k] * eta[k + 1] - eta[k] * xi[k + 1]);
		D += Dk;
		cx += Dk * (xi[k] + xi[k + 1]);
		cy += Dk * (eta[k] + eta[k + 1]);
	}
	Vec3 centroid = g.average;
	if (std::fabs(D) > 1e-300) centroid = g.average + g.ex * (cx / (3.0 * D)) + g.ey * (cy / (3.0 * D));
	g.origin = centroid;
	double diam = 0.0;
	for (int k = 0; k < 4; ++k) diam = std::max(diam, Distance(g.corners[k], centroid));
	g.diameter = std::max(2.0 * diam, 1e-12);

	if (curved && net.curved && !net.IsWake()) {
		// The fit set Π: the corners (heavy) and the grid points around them.
		std::vector<Vec3> pts;
		std::vector<double> w;
		for (int di = -1; di <= 2; ++di) {
			for (int dj = -1; dj <= 2; ++dj) {
				int II = I + di, JJ = J + dj;
				if (II < 1 || II > net.rows || JJ < 1 || JJ > net.cols) continue;
				bool corner = (di == 0 || di == 1) && (dj == 0 || dj == 1);
				Vec3 p = net.Corner(II, JJ);
				bool dup = false;
				for (size_t q = 0; q < pts.size(); ++q) if (DistanceSq(pts[q], p) < 1e-24 * g.diameter * g.diameter) { dup = true; if (corner) w[q] = own_weight; break; }
				if (dup) continue;
				pts.push_back(p);
				w.push_back(corner ? own_weight : 1.0);
			}
		}
		Vec3 ex = g.ex, ey = g.ey, ez = g.ez, O = centroid;
		std::array<double, 6> c{};
		bool converged = false;
		for (int it = 0; it < 20; ++it) {
			std::vector<Vec3> local(pts.size());
			for (size_t k = 0; k < pts.size(); ++k) { Vec3 d = pts[k] - O; local[k] = {Dot(d, ex), Dot(d, ey), Dot(d, ez)}; }
			c = FitQuadratic(local, w, g.diameter);
			// Rotate about ζ to kill the ξη term (A.11).
			double psi = 0.5 * std::atan2(2.0 * c[4], c[3] - c[5]);
			if (std::fabs(c[4]) > 1e-12 / g.diameter) {
				Vec3 nex = ex * std::cos(psi) + ey * std::sin(psi);
				Vec3 ney = ex * (-std::sin(psi)) + ey * std::cos(psi);
				ex = nex; ey = ney;
				for (size_t k = 0; k < pts.size(); ++k) { Vec3 d = pts[k] - O; local[k] = {Dot(d, ex), Dot(d, ey), Dot(d, ez)}; }
				c = FitQuadratic(local, w, g.diameter);
			}
			double d = c[1], e = c[2];
			if (std::fabs(d) < 1e-10 && std::fabs(e) < 1e-10) { converged = true; break; }
			// Rotate to remove the linear terms (A.15), (A.16).
			double alpha = std::asin(d / std::sqrt(1.0 + d * d));
			double beta = std::asin(e / std::sqrt(1.0 + d * d + e * e));
			double ca = std::cos(alpha), sa = std::sin(alpha), cb = std::cos(beta), sb = std::sin(beta);
			Vec3 nex = ex * ca + ez * sa;
			Vec3 ney = ex * (-sb * sa) + ey * cb + ez * (sb * ca);
			Vec3 nez = ex * (-cb * sa) + ey * (-sb) + ez * (cb * ca);
			ex = nex; ey = ney; ez = nez;
		}
		if (converged) {
			g.ex = ex; g.ey = ey; g.ez = ez;
			g.origin = O + ez * c[0]; // (A.18)
			g.local.a = 0.5 * c[3];
			g.local.b = 0.5 * c[5];
		}
	}
	for (int k = 0; k < 4; ++k) {
		Vec3 q = g.ToLocal(g.corners[k]);
		g.local.xi[k] = q.x;
		g.local.eta[k] = q.y;
	}
	// Σ must be counter-clockwise in (ξ, η) for the outer normals of the
	// sides; the frame of (A.1) makes it so, but a collapsed panel may not.
	if (g.local.Area() < 0.0) {
		std::swap(g.local.xi[1], g.local.xi[3]);
		std::swap(g.local.eta[1], g.local.eta[3]);
		std::swap(g.corners[1], g.corners[3]);
	}
	g.local.ComputeMoments();
	double dmax = 0.0;
	for (int k = 0; k < 4; ++k) dmax = std::max(dmax, std::sqrt(g.local.a * g.local.a * g.local.xi[k] * g.local.xi[k] + g.local.b * g.local.b * g.local.eta[k] * g.local.eta[k]));
	g.delta = 0.5 * dmax;
	return g;
}

// ---------------------------------------------------------------------------
// Networks: parameters, splines, control points
// ---------------------------------------------------------------------------

enum class PointKind { Centre, EdgeMid, Corner, ColumnHead, Apex };

struct ParamPoint {
	Vec3 pos;
	int li = 0, lj = 0; // lattice indices (type 2); column (type 8)
	PointKind kind = PointKind::Centre;
	int global = -1;    // index into the unknown vector (shared by merged points)
	int owner_i = 0, owner_j = 0; // a panel the point belongs to
};

struct PanelSpline {
	int ncoef = 0;             // 3 (source) or 6 (doublet)
	std::vector<int> params;   // global parameter indices
	std::vector<double> B;     // ncoef x params.size(), row-major (B.3)
};

struct NetworkData {
	const Network* net = nullptr;
	int index = 0;
	int prows = 0, pcols = 0;
	std::vector<PanelGeom> geoms;        // prows * pcols
	std::vector<PanelSpline> splines;    // doublet splines (or source splines for type 1)
	std::vector<PanelSpline> prescribed; // type-1 splines of the prescribed source, if any
	std::vector<double> prescribed_values; // per panel
	std::vector<ParamPoint> params;      // this network's parameter points (merged duplicates share `global`)
	int lattice_rows = 0, lattice_cols = 0; // type 2: (M + 1) x (N + 1)
	std::vector<int> lattice_to_param;   // lattice (li, lj) -> index into params (-1 none)
	std::array<EdgeKind, 4> edge_kind{};
	struct Abutment { int network = -1, edge = -1; };
	std::array<std::vector<Abutment>, 4> abut; // partners per edge
	std::array<bool, 4> takes_mu{};      // on an abutment: this edge imposes μ continuity (else gradient continuity)
	std::array<bool, 4> collapsed{};

	const PanelGeom& geom(int i, int j) const { return geoms[static_cast<size_t>(i) * pcols + j]; }
	const PanelSpline& spline(int i, int j) const { return splines[static_cast<size_t>(i) * pcols + j]; }
};

struct ControlPoint {
	Vec3 pos, normal;
	int network = 0, pi = 0, pj = 0; // owning panel
	PointKind kind = PointKind::Centre;
	int param = -1;                  // the parameter this point answers for (type 2 edges, wake heads)
	int li = 0, lj = 0;
};

// The basis V(ξ, η) and its gradient for ncoef coefficients.
void Basis(int ncoef, double xi, double eta, double* V, double* Vx, double* Vy) {
	double full[6] = {1.0, xi, eta, 0.5 * xi * xi, xi * eta, 0.5 * eta * eta};
	double fx[6] = {0.0, 1.0, 0.0, xi, eta, 0.0};
	double fy[6] = {0.0, 0.0, 1.0, 0.0, xi, eta};
	for (int i = 0; i < ncoef; ++i) { V[i] = full[i]; if (Vx) Vx[i] = fx[i]; if (Vy) Vy[i] = fy[i]; }
}

// Weighted least-squares spline matrix (B.6)-(B.8) for one panel.
PanelSpline FitSpline(int ncoef, const PanelGeom& g, const std::vector<std::pair<Vec3, int>>& points, const std::vector<double>& w, double own_weight) {
	PanelSpline s;
	s.ncoef = ncoef;
	const int n = static_cast<int>(points.size());
	// Distinct global parameters.
	for (const auto& [p, gi] : points) if (std::find(s.params.begin(), s.params.end(), gi) == s.params.end()) s.params.push_back(gi);
	const int np = static_cast<int>(s.params.size());
	int nc = ncoef;
	if (np < nc) nc = np >= 3 ? 3 : 1; // too few parameters for the full basis
	const double scale = g.diameter;
	std::vector<double> B1(nc * nc, 0.0), B2(nc * np, 0.0);
	for (int k = 0; k < n; ++k) {
		Vec3 q = g.ToLocal(points[k].first);
		double V[6];
		Basis(nc, q.x / scale, q.y / scale, V, nullptr, nullptr);
		int col = static_cast<int>(std::find(s.params.begin(), s.params.end(), points[k].second) - s.params.begin());
		for (int i = 0; i < nc; ++i) {
			B2[i * np + col] += w[k] * V[i];
			for (int jj = 0; jj < nc; ++jj) B1[i * nc + jj] += w[k] * V[i] * V[jj];
		}
	}
	(void)own_weight;
	for (int i = 0; i < nc; ++i) B1[i * nc + i] += 1e-14 * (B1[i * nc + i] + 1.0);
	s.B.assign(static_cast<size_t>(ncoef) * np, 0.0);
	for (int col = 0; col < np; ++col) {
		std::vector<double> rhs(nc);
		for (int i = 0; i < nc; ++i) rhs[i] = B2[i * np + col];
		std::vector<double> c = SolveSmall(B1, rhs, nc);
		// Undo the coordinate scaling: coefficient of ξ^p η^q divides by scale^(p+q).
		const double powers[6] = {1.0, scale, scale, scale * scale, scale * scale, scale * scale};
		for (int i = 0; i < nc; ++i) s.B[static_cast<size_t>(i) * np + col] = c[i] / powers[i];
	}
	return s;
}

// The wake spline (type 8): constant along the column (the stream), and
// across the columns the quadratic through the heads of columns j-1, j,
// j+1. With w the in-plane column direction, the across coordinate is
// c = ξ w_η - η w_ξ and μ = c0 + c1 c + ½ c2 c².
PanelSpline FitWakeSpline(const PanelGeom& g, const std::vector<std::pair<Vec3, int>>& heads) {
	PanelSpline s;
	s.ncoef = 6;
	const int np = static_cast<int>(heads.size());
	for (const auto& [p, gi] : heads) s.params.push_back(gi);
	s.B.assign(static_cast<size_t>(6) * np, 0.0);
	Vec3 w3 = g.ToLocal(g.corners[3]) - g.ToLocal(g.corners[0]); // P1 -> P4: along the column
	double wn = std::hypot(w3.x, w3.y);
	double wx = wn > 0.0 ? w3.x / wn : 0.0, wy = wn > 0.0 ? w3.y / wn : 1.0;
	int nc = np >= 3 ? 3 : (np == 2 ? 2 : 1);
	const double scale = g.diameter;
	std::vector<double> A(nc * nc, 0.0), rhs(nc);
	std::vector<double> cs(np);
	for (int k = 0; k < np; ++k) {
		Vec3 q = g.ToLocal(heads[k].first);
		cs[k] = (q.x * wy - q.y * wx) / scale;
	}
	for (int k = 0; k < np; ++k) {
		double V[3] = {1.0, cs[k], 0.5 * cs[k] * cs[k]};
		for (int i = 0; i < nc; ++i) for (int jj = 0; jj < nc; ++jj) A[i * nc + jj] += V[i] * V[jj];
	}
	for (int i = 0; i < nc; ++i) A[i * nc + i] += 1e-14 * (A[i * nc + i] + 1.0);
	for (int col = 0; col < np; ++col) {
		double V[3] = {1.0, cs[col], 0.5 * cs[col] * cs[col]};
		for (int i = 0; i < nc; ++i) rhs[i] = V[i];
		std::vector<double> c = SolveSmall(A, rhs, nc);
		double c1 = nc >= 2 ? c[1] / scale : 0.0, c2 = nc >= 3 ? c[2] / (scale * scale) : 0.0;
		s.B[static_cast<size_t>(0) * np + col] = c[0];
		s.B[static_cast<size_t>(1) * np + col] = c1 * wy;        // μξ
		s.B[static_cast<size_t>(2) * np + col] = -c1 * wx;       // μη
		s.B[static_cast<size_t>(3) * np + col] = c2 * wy * wy;   // μξξ
		s.B[static_cast<size_t>(4) * np + col] = -c2 * wy * wx;  // μξη
		s.B[static_cast<size_t>(5) * np + col] = c2 * wx * wx;   // μηη
	}
	return s;
}

// μ (or σ) and its in-plane gradient at a point, as rows over the panel's parameters.
void SplineRows(const PanelGeom& g, const PanelSpline& s, const Vec3& p, std::vector<double>* value, std::vector<double>* gx, std::vector<double>* gy) {
	Vec3 q = g.ToLocal(p);
	double V[6], Vx[6], Vy[6];
	Basis(s.ncoef, q.x, q.y, V, Vx, Vy);
	const int np = static_cast<int>(s.params.size());
	if (value) value->assign(np, 0.0);
	if (gx) gx->assign(np, 0.0);
	if (gy) gy->assign(np, 0.0);
	for (int c = 0; c < s.ncoef; ++c) {
		for (int k = 0; k < np; ++k) {
			double b = s.B[static_cast<size_t>(c) * np + k];
			if (value) (*value)[k] += V[c] * b;
			if (gx) (*gx)[k] += Vx[c] * b;
			if (gy) (*gy)[k] += Vy[c] * b;
		}
	}
}

// ---------------------------------------------------------------------------
// The solver state
// ---------------------------------------------------------------------------

} // namespace

struct Solution::Internal {
	std::vector<Network> networks; // copies
	std::vector<NetworkData> data;
	Options options;
	int n_unknowns = 0;
};

namespace {

Vec3 Reflect(const Vec3& p, double y0) { return {p.x, 2.0 * y0 - p.y, p.z}; }

// Accumulates the influence of every panel at a point into coefficient rows
// over the unknowns (and the known part from prescribed sources).
struct InfluenceRows {
	std::vector<double> phi;      // n_unknowns
	std::vector<double> vel[3];   // n_unknowns each
	double phi_known = 0.0;
	Vec3 vel_known{0, 0, 0};
};

void AccumulateInfluence(const Solution::Internal& S, const Vec3& P, InfluenceRows& rows, bool want_velocity) {
	const int n = S.n_unknowns;
	rows.phi.assign(n, 0.0);
	for (auto& v : rows.vel) v.assign(n, 0.0);
	rows.phi_known = 0.0;
	rows.vel_known = {0, 0, 0};
	const bool mirrored = S.options.symmetry_y.has_value();
	for (const NetworkData& nd : S.data) {
		for (int i = 0; i < nd.prows; ++i) {
			for (int j = 0; j < nd.pcols; ++j) {
				const PanelGeom& g = nd.geom(i, j);
				for (int side = 0; side < (mirrored ? 2 : 1); ++side) {
					Vec3 Pq = side == 0 ? P : Reflect(P, *S.options.symmetry_y);
					Vec3 q = g.ToLocal(Pq);
					Influence inf = ComputeInfluence(g.local, q, S.options.integrals);
					// Global velocity per coefficient, reflected back for the image.
					auto to_global = [&](double vx, double vy, double vz) {
						Vec3 v = g.ToGlobalDir({vx, vy, vz});
						if (side == 1) v.y = -v.y;
						return v;
					};
					const PanelSpline& s = nd.spline(i, j);
					const int np = static_cast<int>(s.params.size());
					if (nd.net->IsDoublet()) {
						for (int c = 0; c < 6; ++c) {
							Vec3 vg = want_velocity ? to_global(inf.vel_doublet[0][c], inf.vel_doublet[1][c], inf.vel_doublet[2][c]) : Vec3{0, 0, 0};
							for (int k = 0; k < np; ++k) {
								double b = s.B[static_cast<size_t>(c) * np + k];
								if (b == 0.0) continue;
								int gi = s.params[k];
								rows.phi[gi] += inf.phi_doublet[c] * b;
								if (want_velocity) { rows.vel[0][gi] += vg.x * b; rows.vel[1][gi] += vg.y * b; rows.vel[2][gi] += vg.z * b; }
							}
						}
					} else {
						for (int c = 0; c < 3; ++c) {
							Vec3 vg = want_velocity ? to_global(inf.vel_source[0][c], inf.vel_source[1][c], inf.vel_source[2][c]) : Vec3{0, 0, 0};
							for (int k = 0; k < np; ++k) {
								double b = s.B[static_cast<size_t>(c) * np + k];
								if (b == 0.0) continue;
								int gi = s.params[k];
								rows.phi[gi] += inf.phi_source[c] * b;
								if (want_velocity) { rows.vel[0][gi] += vg.x * b; rows.vel[1][gi] += vg.y * b; rows.vel[2][gi] += vg.z * b; }
							}
						}
					}
					if (!nd.prescribed.empty()) {
						// Known source: coefficients = B_src · σ_values.
						const PanelSpline& ps = nd.prescribed[static_cast<size_t>(i) * nd.pcols + j];
						const int npp = static_cast<int>(ps.params.size());
						double coef[3] = {0, 0, 0};
						for (int c = 0; c < 3; ++c) for (int k = 0; k < npp; ++k) coef[c] += ps.B[static_cast<size_t>(c) * npp + k] * nd.prescribed_values[ps.params[k]];
						for (int c = 0; c < 3; ++c) {
							rows.phi_known += inf.phi_source[c] * coef[c];
							if (want_velocity) rows.vel_known += to_global(inf.vel_source[0][c], inf.vel_source[1][c], inf.vel_source[2][c]) * coef[c];
						}
					}
				}
			}
		}
	}
}

// Edge helpers: the grid points along an edge, in order.
std::vector<Vec3> EdgePoints(const Network& net, int edge) {
	std::vector<Vec3> pts;
	switch (edge) {
		case 0: for (int J = 1; J <= net.cols; ++J) pts.push_back(net.Corner(1, J)); break;
		case 1: for (int I = 1; I <= net.rows; ++I) pts.push_back(net.Corner(I, net.cols)); break;
		case 2: for (int J = 1; J <= net.cols; ++J) pts.push_back(net.Corner(net.rows, J)); break;
		default: for (int I = 1; I <= net.rows; ++I) pts.push_back(net.Corner(I, 1)); break;
	}
	return pts;
}

double DistanceToPolyline(const std::vector<Vec3>& poly, const Vec3& p, int* segment = nullptr) {
	double best = 1e300;
	for (size_t k = 0; k + 1 < poly.size(); ++k) {
		Vec3 a = poly[k], d = poly[k + 1] - a;
		double len2 = LengthSq(d);
		double t = len2 > 0.0 ? std::clamp(Dot(p - a, d) / len2, 0.0, 1.0) : 0.0;
		double dist = Distance(a + d * t, p);
		if (dist < best) { best = dist; if (segment) *segment = static_cast<int>(k); }
	}
	if (poly.size() == 1) best = Distance(poly[0], p);
	return best;
}

// The panel of `nd` adjacent to segment k of its edge.
std::pair<int, int> EdgePanel(const NetworkData& nd, int edge, int segment) {
	switch (edge) {
		case 0: return {0, segment};
		case 1: return {segment, nd.pcols - 1};
		case 2: return {nd.prows - 1, segment};
		default: return {segment, 0};
	}
}

// Adds coef × (row over the panel's parameters) into a global row.
void AddRow(std::vector<double>& global, const PanelSpline& s, const std::vector<double>& local, double coef) {
	for (size_t k = 0; k < s.params.size(); ++k) global[s.params[k]] += coef * local[k];
}

// In-plane outward normal of an edge at a point, from the owning panel.
Vec3 EdgeOutwardNormal(const NetworkData& nd, int edge, int pi, int pj, const Vec3& p) {
	const PanelGeom& g = nd.geom(pi, pj);
	std::vector<Vec3> pts = EdgePoints(*nd.net, edge);
	int seg = 0;
	DistanceToPolyline(pts, p, &seg);
	Vec3 t = Normalized(pts[seg + 1] - pts[seg]);
	Vec3 n = Normalized(Cross(g.ez, t));
	// Point away from the panel's centre.
	if (Dot(n, p - g.origin) < 0.0) n = -n;
	return n;
}

} // namespace

// ---------------------------------------------------------------------------
// Solve
// ---------------------------------------------------------------------------

Solution Solve(const std::vector<Network>& networks_in, const Vec3& free_stream, const Options& options) {
	using clock = std::chrono::steady_clock;
	auto S = std::make_shared<Solution::Internal>();
	S->networks = networks_in;
	S->options = options;
	const double tol = options.abutment_tolerance;
	const int nn = static_cast<int>(S->networks.size());
	if (nn == 0) throw std::invalid_argument("panel method: no networks");
	S->data.resize(nn);

	// ---- geometry ----
	for (int k = 0; k < nn; ++k) {
		const Network& net = S->networks[k];
		NetworkData& nd = S->data[k];
		nd.net = &net;
		nd.index = k;
		if (net.rows < 2 || net.cols < 2 || static_cast<int>(net.corners.size()) != net.rows * net.cols) {
			throw std::invalid_argument("panel method: network '" + net.name + "' needs an M x N grid of corners with M, N >= 2");
		}
		nd.prows = net.rows - 1;
		nd.pcols = net.cols - 1;
		nd.geoms.reserve(static_cast<size_t>(nd.prows) * nd.pcols);
		for (int i = 0; i < nd.prows; ++i) for (int j = 0; j < nd.pcols; ++j) nd.geoms.push_back(BuildPanel(net, i, j, options.curved_panels, options.own_weight));
	}

	// ---- singularity parameters (figure B.1) ----
	int n_unknowns = 0;
	for (NetworkData& nd : S->data) {
		const Network& net = *nd.net;
		const int M = net.rows, N = net.cols;
		auto grid_scale = [&]() {
			Box3 b;
			for (const Vec3& c : net.corners) b.Add(c);
			return std::max(b.Diagonal(), 1e-12);
		}();
		const double same = 1e-9 * grid_scale;
		switch (net.type) {
			case NetworkType::SourceAnalysis:
				for (int i = 0; i < nd.prows; ++i) for (int j = 0; j < nd.pcols; ++j) {
					ParamPoint p;
					p.pos = nd.geom(i, j).average; p.li = i; p.lj = j; p.kind = PointKind::Centre; p.owner_i = i; p.owner_j = j;
					p.global = n_unknowns++;
					nd.params.push_back(p);
				}
				break;
			case NetworkType::DoubletAnalysis: {
				nd.lattice_rows = M + 1; nd.lattice_cols = N + 1;
				nd.lattice_to_param.assign(static_cast<size_t>(nd.lattice_rows) * nd.lattice_cols, -1);
				for (int li = 0; li <= M; ++li) {
					for (int lj = 0; lj <= N; ++lj) {
						ParamPoint p;
						p.li = li; p.lj = lj;
						bool erow = li == 0 || li == M, ecol = lj == 0 || lj == N;
						int gi = li == 0 ? 1 : (li == M ? M : 0), gj = lj == 0 ? 1 : (lj == N ? N : 0);
						if (erow && ecol) { p.kind = PointKind::Corner; p.pos = net.Corner(gi, gj); }
						else if (erow) { p.kind = PointKind::EdgeMid; p.pos = (net.Corner(gi, lj) + net.Corner(gi, lj + 1)) * 0.5; }
						else if (ecol) { p.kind = PointKind::EdgeMid; p.pos = (net.Corner(li, gj) + net.Corner(li + 1, gj)) * 0.5; }
						else { p.kind = PointKind::Centre; p.pos = nd.geom(li - 1, lj - 1).average; }
						p.owner_i = std::clamp(li - 1, 0, nd.prows - 1);
						p.owner_j = std::clamp(lj - 1, 0, nd.pcols - 1);
						nd.lattice_to_param[static_cast<size_t>(li) * nd.lattice_cols + lj] = static_cast<int>(nd.params.size());
						nd.params.push_back(p);
					}
				}
				// A collapsed edge (every grid point of it coincident, an apex):
				// its lattice points are one parameter.
				for (int edge = 0; edge < 4; ++edge) {
					std::vector<Vec3> pts = EdgePoints(net, edge);
					bool collapsed = true;
					for (size_t q = 1; q < pts.size(); ++q) collapsed = collapsed && Distance(pts[q], pts[0]) < same;
					nd.collapsed[edge] = collapsed;
				}
				std::vector<int> merged_to(nd.params.size(), -1);
				for (int edge = 0; edge < 4; ++edge) {
					if (!nd.collapsed[edge]) continue;
					int first = -1;
					for (int li = 0; li <= M; ++li) {
						for (int lj = 0; lj <= N; ++lj) {
							bool on = (edge == 0 && li == 0) || (edge == 2 && li == M) || (edge == 1 && lj == N) || (edge == 3 && lj == 0);
							if (!on) continue;
							int idx = nd.lattice_to_param[static_cast<size_t>(li) * nd.lattice_cols + lj];
							if (first < 0) { first = idx; nd.params[idx].kind = PointKind::Apex; }
							else merged_to[idx] = first;
						}
					}
				}
				for (size_t q = 0; q < nd.params.size(); ++q) {
					if (merged_to[q] >= 0) continue;
					nd.params[q].global = n_unknowns++;
				}
				for (size_t q = 0; q < nd.params.size(); ++q) if (merged_to[q] >= 0) nd.params[q].global = nd.params[merged_to[q]].global;
				break;
			}
			case NetworkType::DoubletWake:
				for (int j = 0; j < nd.pcols; ++j) {
					ParamPoint p;
					p.pos = (net.Corner(1, j + 1) + net.Corner(1, j + 2)) * 0.5; p.li = 0; p.lj = j; p.kind = PointKind::ColumnHead; p.owner_i = 0; p.owner_j = j;
					p.global = n_unknowns++;
					nd.params.push_back(p);
				}
				break;
			case NetworkType::DoubletWakeConstant: {
				ParamPoint p;
				p.pos = (net.Corner(1, 1) + net.Corner(1, 2)) * 0.5; p.kind = PointKind::ColumnHead; p.owner_i = 0; p.owner_j = 0;
				p.global = n_unknowns++;
				nd.params.push_back(p);
				break;
			}
		}
	}
	S->n_unknowns = n_unknowns;

	// ---- splines (B.2) ----
	for (NetworkData& nd : S->data) {
		const Network& net = *nd.net;
		const int M = net.rows, N = net.cols;
		nd.splines.resize(static_cast<size_t>(nd.prows) * nd.pcols);
		for (int i = 0; i < nd.prows; ++i) {
			for (int j = 0; j < nd.pcols; ++j) {
				const PanelGeom& g = nd.geom(i, j);
				std::vector<std::pair<Vec3, int>> pts;
				std::vector<double> w;
				switch (net.type) {
					case NetworkType::SourceAnalysis:
						for (const ParamPoint& p : nd.params) {
							int di = p.li - i, dj = p.lj - j;
							if (std::abs(di) > 1 || std::abs(dj) > 1) continue;
							pts.push_back({p.pos, p.global});
							w.push_back(di == 0 && dj == 0 ? options.own_weight : 1.0);
						}
						nd.splines[static_cast<size_t>(i) * nd.pcols + j] = FitSpline(3, g, pts, w, options.own_weight);
						break;
					case NetworkType::DoubletAnalysis:
						for (const ParamPoint& p : nd.params) {
							int di = p.li - (i + 1), dj = p.lj - (j + 1);
							if (std::abs(di) > 1 || std::abs(dj) > 1) continue;
							bool belongs_i = p.li == i + 1 || (p.li == 0 && i == 0) || (p.li == M && i == nd.prows - 1);
							bool belongs_j = p.lj == j + 1 || (p.lj == 0 && j == 0) || (p.lj == N && j == nd.pcols - 1);
							pts.push_back({p.pos, p.global});
							w.push_back(belongs_i && belongs_j ? options.own_weight : 1.0);
						}
						nd.splines[static_cast<size_t>(i) * nd.pcols + j] = FitSpline(6, g, pts, w, options.own_weight);
						break;
					case NetworkType::DoubletWake:
						for (int jj = j - 1; jj <= j + 1; ++jj) {
							if (jj < 0 || jj >= nd.pcols) continue;
							pts.push_back({nd.params[jj].pos, nd.params[jj].global});
						}
						nd.splines[static_cast<size_t>(i) * nd.pcols + j] = FitWakeSpline(g, pts);
						break;
					case NetworkType::DoubletWakeConstant: {
						PanelSpline s;
						s.ncoef = 6;
						s.params = {nd.params[0].global};
						s.B.assign(6, 0.0);
						s.B[0] = 1.0;
						nd.splines[static_cast<size_t>(i) * nd.pcols + j] = s;
						break;
					}
				}
			}
		}
		// A prescribed source distribution: type-1 spline through its values.
		if (net.prescribed_source) {
			nd.prescribed.resize(nd.splines.size());
			nd.prescribed_values.resize(nd.splines.size());
			for (int i = 0; i < nd.prows; ++i) for (int j = 0; j < nd.pcols; ++j) {
				const PanelGeom& g = nd.geom(i, j);
				nd.prescribed_values[static_cast<size_t>(i) * nd.pcols + j] = net.prescribed_source(g.origin, g.ez);
			}
			for (int i = 0; i < nd.prows; ++i) for (int j = 0; j < nd.pcols; ++j) {
				const PanelGeom& g = nd.geom(i, j);
				std::vector<std::pair<Vec3, int>> pts;
				std::vector<double> w;
				for (int ii = i - 1; ii <= i + 1; ++ii) for (int jj = j - 1; jj <= j + 1; ++jj) {
					if (ii < 0 || ii >= nd.prows || jj < 0 || jj >= nd.pcols) continue;
					pts.push_back({nd.geom(ii, jj).average, ii * nd.pcols + jj});
					w.push_back(ii == i && jj == j ? options.own_weight : 1.0);
				}
				nd.prescribed[static_cast<size_t>(i) * nd.pcols + j] = FitSpline(3, g, pts, w, options.own_weight);
			}
		}
	}

	// ---- edge kinds and abutments (C.3) ----
	for (NetworkData& nd : S->data) {
		if (!nd.net->IsDoublet()) continue;
		for (int edge = 0; edge < 4; ++edge) {
			EdgeKind kind = nd.net->edges[edge];
			std::vector<Vec3> pts = EdgePoints(*nd.net, edge);
			if (kind == EdgeKind::Auto || kind == EdgeKind::Abut) {
				// Symmetry plane?
				if (kind == EdgeKind::Auto && options.symmetry_y) {
					bool on = true;
					for (const Vec3& p : pts) on = on && std::fabs(p.y - *options.symmetry_y) < tol;
					if (on) { nd.edge_kind[edge] = EdgeKind::Symmetry; continue; }
				}
				// Abutting doublet-network edges: every point of this edge within tol of theirs.
				for (NetworkData& other : S->data) {
					if (!other.net->IsDoublet()) continue;
					for (int oe = 0; oe < 4; ++oe) {
						if (&other == &nd && oe == edge) continue;
						if (other.collapsed[oe] || nd.collapsed[edge]) continue;
						std::vector<Vec3> opts = EdgePoints(*other.net, oe);
						bool all = true;
						for (const Vec3& p : pts) all = all && DistanceToPolyline(opts, p) < tol;
						bool all_back = true;
						for (const Vec3& p : opts) all_back = all_back && DistanceToPolyline(pts, p) < tol;
						if (all || all_back) nd.abut[edge].push_back({other.index, oe});
					}
				}
				if (!nd.abut[edge].empty()) nd.edge_kind[edge] = EdgeKind::Abut;
				else if (kind == EdgeKind::Abut) throw std::invalid_argument("panel method: edge " + std::to_string(edge) + " of network '" + nd.net->name + "' abuts nothing");
				else nd.edge_kind[edge] = EdgeKind::Free;
			} else {
				nd.edge_kind[edge] = kind;
			}
		}
	}
	// Which side of an abutment imposes μ continuity: a wake always (at its
	// heads); otherwise the lower network index, and on a seam of one
	// network the lower edge number.
	for (NetworkData& nd : S->data) {
		for (int edge = 0; edge < 4; ++edge) {
			nd.takes_mu[edge] = false;
			if (nd.edge_kind[edge] != EdgeKind::Abut) continue;
			if (nd.net->IsWake()) { nd.takes_mu[edge] = true; continue; }
			bool mu = true;
			for (const auto& ab : nd.abut[edge]) {
				const NetworkData& other = S->data[ab.network];
				if (other.net->IsWake()) { mu = false; break; }
				if (other.index < nd.index || (other.index == nd.index && ab.edge < edge)) mu = false;
			}
			nd.takes_mu[edge] = mu;
		}
	}

	// ---- control points (figure C.1) ----
	std::vector<ControlPoint> cps;
	for (NetworkData& nd : S->data) {
		const Network& net = *nd.net;
		const int M = net.rows, N = net.cols;
		const bool potential_network = net.IsDoublet() && !net.IsWake() && net.bc.kind == BoundaryCondition::Kind::Potential;
		auto displaced_edge_point = [&](const ParamPoint& p) {
			// (C.1), (C.2): a hair inward along the owning panel's edge normals.
			const PanelGeom& g = nd.geom(p.owner_i, p.owner_j);
			Vec3 d{0, 0, 0};
			if (potential_network && options.potential_edge_inset > 0.0) {
				// Under the potential condition the displaced point is only a
				// collocation point for that condition (the points of a wake-
				// shedding edge take the gradient condition at the undisplaced
				// point, and nothing registers an abutment by it), so it goes
				// well inside the owning panel, towards its centre: a hair from
				// the edge the condition loses precision -- the near-field
				// potential of the neighbouring panel is evaluated at an in-plane
				// point almost on its edge -- and the corner and edge-midpoint
				// strengths come out wrong, which the centre strengths hardly
				// feel but the spline GRADIENT does (seen 2026-10-27 on the P-38
				// wing's root and tip columns: μ smooth, |∇μ| ten times its
				// neighbours', Cp of -30 at the leading edge). A fraction of the
				// way to the centre, not of the diameter: on a narrow panel the
				// latter overshoots the centre and lands by the opposite edge.
				d = (g.origin - p.pos) * options.potential_edge_inset;
			} else if (p.kind == PointKind::Corner || p.kind == PointKind::Apex) {
				// Towards the panel centre, by ε of the diameter.
				d = Normalized(g.origin - p.pos) * (options.edge_epsilon * g.diameter);
			} else {
				for (int edge = 0; edge < 4; ++edge) {
					bool on = (edge == 0 && p.li == 0) || (edge == 2 && p.li == M) || (edge == 1 && p.lj == N) || (edge == 3 && p.lj == 0);
					if (!on) continue;
					Vec3 n = EdgeOutwardNormal(nd, edge, p.owner_i, p.owner_j, p.pos);
					d -= n * (options.edge_epsilon * g.diameter);
				}
			}
			// On the owning panel's curved surface, so that its own influence
			// is the average value there (the displaced point of the report
			// is off the surface, which its edge-velocity algorithm tolerated;
			// a potential condition evaluated there would see a one-sided jump).
			Vec3 q = g.ToLocal(p.pos + d);
			q.z = g.local.a * q.x * q.x + g.local.b * q.y * q.y;
			return g.origin + g.ToGlobalDir(q);
		};
		std::set<int> seen;
		for (size_t q = 0; q < nd.params.size(); ++q) {
			const ParamPoint& p = nd.params[q];
			if (seen.count(p.global)) continue; // merged (apex) points: one control point
			seen.insert(p.global);
			ControlPoint c;
			c.network = nd.index; c.pi = p.owner_i; c.pj = p.owner_j; c.kind = p.kind; c.param = p.global; c.li = p.li; c.lj = p.lj;
			const PanelGeom& g = nd.geom(p.owner_i, p.owner_j);
			c.normal = g.ez;
			if (p.kind == PointKind::Centre) c.pos = g.origin;
			else c.pos = displaced_edge_point(p);
			cps.push_back(c);
		}
	}
	if (static_cast<int>(cps.size()) != n_unknowns) throw std::runtime_error("panel method: control points and parameters differ in number");

	// ---- the equations ----
	auto t0 = clock::now();
	Dense A(n_unknowns, n_unknowns);
	std::vector<double> rhs(n_unknowns, 0.0);
	InfluenceRows rows;
	for (int r = 0; r < n_unknowns; ++r) {
		const ControlPoint& c = cps[r];
		NetworkData& nd = S->data[c.network];
		const Network& net = *nd.net;
		const PanelGeom& g = nd.geom(c.pi, c.pj);
		const PanelSpline& own = nd.spline(c.pi, c.pj);
		std::vector<double> local_value, local_gx, local_gy;

		auto centre_condition = [&]() {
			const BoundaryCondition& bc = net.bc;
			if (bc.kind == BoundaryCondition::Kind::Velocity) {
				AccumulateInfluence(*S, c.pos, rows, true);
				for (int k = 0; k < n_unknowns; ++k) A(r, k) = bc.c_average * (c.normal.x * rows.vel[0][k] + c.normal.y * rows.vel[1][k] + c.normal.z * rows.vel[2][k]);
				rhs[r] = bc.beta - bc.c_average * (Dot(c.normal, free_stream) + Dot(c.normal, rows.vel_known));
				// n · V_D: σ at the point for a source network; a prescribed source's σ is known.
				if (net.type == NetworkType::SourceAnalysis) {
					SplineRows(g, own, c.pos, &local_value, nullptr, nullptr);
					std::vector<double> tmp(n_unknowns, 0.0);
					AddRow(tmp, own, local_value, bc.c_difference);
					for (int k = 0; k < n_unknowns; ++k) A(r, k) += tmp[k];
				}
				if (!nd.prescribed.empty()) {
					const PanelSpline& ps = nd.prescribed[static_cast<size_t>(c.pi) * nd.pcols + c.pj];
					SplineRows(g, ps, c.pos, &local_value, nullptr, nullptr);
					double sigma = 0.0;
					for (size_t k = 0; k < ps.params.size(); ++k) sigma += local_value[k] * nd.prescribed_values[ps.params[k]];
					rhs[r] -= bc.c_difference * sigma;
				}
			} else {
				AccumulateInfluence(*S, c.pos, rows, false);
				for (int k = 0; k < n_unknowns; ++k) A(r, k) = bc.c_average * rows.phi[k];
				rhs[r] = bc.beta - bc.c_average * rows.phi_known;
				if (net.IsDoublet()) {
					SplineRows(g, own, c.pos, &local_value, nullptr, nullptr);
					std::vector<double> tmp(n_unknowns, 0.0);
					AddRow(tmp, own, local_value, bc.c_difference);
					for (int k = 0; k < n_unknowns; ++k) A(r, k) += tmp[k];
				}
			}
		};

		// The doublet value and normal-gradient rows of a partner network at the point.
		auto partner_rows = [&](const NetworkData& other, int oedge, const Vec3& p, std::vector<double>& value, Vec3& normal, std::vector<double>* grad_along, const Vec3& dir) {
			std::vector<Vec3> opts = EdgePoints(*other.net, oedge);
			int seg = 0;
			DistanceToPolyline(opts, p, &seg);
			auto [oi, oj] = EdgePanel(other, oedge, seg);
			const PanelGeom& og = other.geom(oi, oj);
			const PanelSpline& os = other.spline(oi, oj);
			std::vector<double> v, gx, gy;
			SplineRows(og, os, p, &v, &gx, &gy);
			value.assign(n_unknowns, 0.0);
			AddRow(value, os, v, 1.0);
			normal = og.ez;
			if (grad_along) {
				grad_along->assign(n_unknowns, 0.0);
				double dx = Dot(dir, og.ex), dy = Dot(dir, og.ey);
				AddRow(*grad_along, os, gx, dx);
				AddRow(*grad_along, os, gy, dy);
			}
		};

		if (c.kind == PointKind::Centre) {
			centre_condition();
			continue;
		}
		if (c.kind == PointKind::ColumnHead) {
			// μ_wake(p) = Σ s_i μ_i(p) over the doublet edges abutting the wake's head edge.
			if (nd.edge_kind[0] != EdgeKind::Abut) throw std::invalid_argument("panel method: wake '" + net.name + "' has no network along its head edge");
			SplineRows(g, own, c.pos, &local_value, nullptr, nullptr);
			std::vector<double> row(n_unknowns, 0.0);
			AddRow(row, own, local_value, 1.0);
			for (const auto& ab : nd.abut[0]) {
				const NetworkData& other = S->data[ab.network];
				std::vector<Vec3> opts = EdgePoints(*other.net, ab.edge);
				if (DistanceToPolyline(opts, c.pos) > 1e-3 * g.diameter + tol) continue; // this head is not on that edge
				std::vector<double> value; Vec3 on;
				partner_rows(other, ab.edge, c.pos, value, on, nullptr, Vec3{});
				double s = Dot(on, g.ez) >= 0.0 ? 1.0 : -1.0;
				for (int k = 0; k < n_unknowns; ++k) row[k] -= s * value[k];
			}
			for (int k = 0; k < n_unknowns; ++k) A(r, k) = row[k];
			rhs[r] = 0.0;
			continue;
		}
		// A type-2 edge point.
		const int M = net.rows, N = net.cols;
		std::vector<int> on_edges;
		if (c.li == 0) on_edges.push_back(0);
		if (c.lj == N) on_edges.push_back(1);
		if (c.li == M) on_edges.push_back(2);
		if (c.lj == 0) on_edges.push_back(3);
		auto abuts_wake = [&](int e) {
			if (nd.edge_kind[e] != EdgeKind::Abut) return false;
			for (const auto& ab : nd.abut[e]) if (S->data[ab.network].net->IsWake()) return true;
			return false;
		};
		int edge = on_edges[0];
		// Priority: free, symmetry, μ continuity, gradient continuity.
		auto rank = [&](int e) {
			switch (nd.edge_kind[e]) {
				case EdgeKind::Free: return 0;
				case EdgeKind::Symmetry: return 1;
				default: return nd.takes_mu[e] ? 2 : 3;
			}
		};
		const bool potential = net.bc.kind == BoundaryCondition::Kind::Potential;
		if (potential) {
			// With potential conditions at the centres the edges take them
			// too (C.3, last paragraph) -- except where the edge sheds a wake.
			// There the doublet gradient must match the wake's (the Kutta
			// condition): tying the wake to the edge values alone would admit
			// the non-lifting flow, whose potential is continuous round the
			// trailing edge, so the wake would carry nothing.
			// Nor on a symmetry plane or an abutment: there the edge strengths
			// are the ONLY unknowns the potential condition has to pin by a
			// point a fraction of a panel from the centre's own point, which
			// is ill-conditioned -- the potential is smooth, the two
			// conditions nearly the same, and the discretisation error of the
			// near-field influence gets amplified into a root column whose μ
			// oscillates (the P-38 wing at 12 x 16, Cp -800 at the root). The
			// zero normal gradient on the plane (μ = φ_outer is symmetric) and
			// the continuity across an abutment are exact and involve the
			// splines alone, so they take over. A free edge keeps the
			// potential condition: there is nothing exact to put in its place
			// (μ = 0 is wrong on a thick body, whose μ is the outer potential).
			// An apex (a pointed tip: the whole edge one unknown) lies on the
			// wake-shedding edges too, but a gradient condition at a point
			// where the surface pinches to nothing is meaningless; it takes
			// the potential condition, and the wake column it heads carries
			// the apex's own jump, zero, as a pointed tip should.
			int wake_edge = -1;
			if (c.kind != PointKind::Apex) for (int e : on_edges) if (abuts_wake(e)) { wake_edge = e; break; }
			if (wake_edge >= 0) {
				edge = wake_edge;
			} else if (c.kind == PointKind::Apex) {
				centre_condition();
				continue;
			} else {
				int best = -1;
				for (int e : on_edges) {
					if (!options.potential_edge_conditions) break;
					if (nd.edge_kind[e] != EdgeKind::Symmetry && nd.edge_kind[e] != EdgeKind::Abut) continue;
					if (best < 0 || rank(e) < rank(best)) best = e;
				}
				if (best < 0) { centre_condition(); continue; }
				edge = best;
			}
		} else {
			for (int e : on_edges) if (rank(e) < rank(edge)) edge = e;
		}
		const Vec3 p = nd.params[nd.lattice_to_param[static_cast<size_t>(c.li) * nd.lattice_cols + c.lj]].pos; // the undisplaced point
		SplineRows(g, own, p, &local_value, &local_gx, &local_gy);
		std::vector<double> row(n_unknowns, 0.0);
		switch (nd.edge_kind[edge]) {
			case EdgeKind::Free:
				AddRow(row, own, local_value, 1.0);
				break;
			case EdgeKind::Symmetry: {
				Vec3 n = EdgeOutwardNormal(nd, edge, c.pi, c.pj, p);
				AddRow(row, own, local_gx, Dot(n, g.ex));
				AddRow(row, own, local_gy, Dot(n, g.ey));
				break;
			}
			default: {
				// Which partner edge passes through this point; a wake first
				// (a closed trailing edge also meets its own other row there).
				const NetworkData::Abutment* partner = nullptr;
				for (int pass = 0; pass < 2 && !partner; ++pass) {
					for (const auto& ab : nd.abut[edge]) {
						const bool wake = S->data[ab.network].net->IsWake();
						if ((pass == 0) != wake) continue;
						std::vector<Vec3> opts = EdgePoints(*S->data[ab.network].net, ab.edge);
						if (DistanceToPolyline(opts, p) < 1e-3 * g.diameter + tol) { partner = &ab; break; }
					}
				}
				if (!partner) {
					// Past the partner's end: a free edge there -- μ = 0 under a
					// velocity condition, the potential condition otherwise.
					if (potential) { centre_condition(); goto next_point; }
					AddRow(row, own, local_value, 1.0);
					break;
				}
				const NetworkData& other = S->data[partner->network];
				Vec3 dir = EdgeOutwardNormal(nd, edge, c.pi, c.pj, p);
				std::vector<double> value, grad; Vec3 on;
				partner_rows(other, partner->edge, p, value, on, &grad, dir);
				double s = Dot(on, g.ez) >= 0.0 ? 1.0 : -1.0;
				if (nd.takes_mu[edge]) {
					AddRow(row, own, local_value, 1.0);
					for (int k = 0; k < n_unknowns; ++k) row[k] -= s * value[k];
				} else {
					AddRow(row, own, local_gx, Dot(dir, g.ex));
					AddRow(row, own, local_gy, Dot(dir, g.ey));
					for (int k = 0; k < n_unknowns; ++k) row[k] -= s * grad[k];
				}
				break;
			}
		}
		for (int k = 0; k < n_unknowns; ++k) A(r, k) = row[k];
		rhs[r] = 0.0;
	next_point:;
	}
	Solution sol;
	sol.seconds_influence = std::chrono::duration<double>(clock::now() - t0).count();

	// ---- solve (appendix E) ----
	t0 = clock::now();
	sol.pivot_ratio = SolveInPlace(A, rhs);
	sol.parameters = rhs;
	sol.seconds_solve = std::chrono::duration<double>(clock::now() - t0).count();
	sol.internal = S;
	sol.free_stream = free_stream;
	sol.unknowns = n_unknowns;

	// ---- aerodynamic quantities (appendix F) ----
	const double vinf2 = std::max(LengthSq(free_stream), 1e-300);
	const bool mirrored = options.symmetry_y.has_value();
	for (NetworkData& nd : S->data) {
		const Network& net = *nd.net;
		NetworkResult nr;
		nr.name = net.name;
		nr.type = net.type;
		nr.panel_rows = nd.prows;
		nr.panel_cols = nd.pcols;
		const bool one_sided = net.type == NetworkType::SourceAnalysis || net.bc.kind == BoundaryCondition::Kind::Potential;
		// The lower side is the interior with zero perturbation potential
		// (InteriorPotentialZero: φ_lower = 0) -- the identity below holds
		// for that side only.
		const bool dirichlet_interior = net.IsDoublet() && !net.IsWake()
			&& net.bc.kind == BoundaryCondition::Kind::Potential && net.bc.beta == 0.0
			&& std::fabs(net.bc.c_average - 1.0) < 1e-12 && std::fabs(net.bc.c_difference + 0.5) < 1e-12;
		for (int i = 0; i < nd.prows; ++i) {
			for (int j = 0; j < nd.pcols; ++j) {
				const PanelGeom& g = nd.geom(i, j);
				PanelResult pr;
				pr.centre = g.origin;
				pr.normal = g.ez;
				pr.area = g.local.Area();
				pr.delta = g.delta;
				sol.max_delta = std::max(sol.max_delta, g.delta);
				const PanelSpline& s = nd.spline(i, j);
				std::vector<double> v, gx, gy;
				SplineRows(g, s, g.origin, &v, &gx, &gy);
				double value = 0.0, dgx = 0.0, dgy = 0.0;
				for (size_t k = 0; k < s.params.size(); ++k) {
					value += v[k] * sol.parameters[s.params[k]];
					dgx += gx[k] * sol.parameters[s.params[k]];
					dgy += gy[k] * sol.parameters[s.params[k]];
				}
				if (net.IsDoublet()) { pr.mu = value; pr.grad_mu = g.ex * dgx + g.ey * dgy; }
				else pr.sigma = value;
				if (!nd.prescribed.empty()) {
					const PanelSpline& ps = nd.prescribed[static_cast<size_t>(i) * nd.pcols + j];
					std::vector<double> pv;
					SplineRows(g, ps, g.origin, &pv, nullptr, nullptr);
					for (size_t k = 0; k < ps.params.size(); ++k) pr.sigma += pv[k] * nd.prescribed_values[ps.params[k]];
				}
				Vec3 vd = g.ez * pr.sigma + pr.grad_mu; // (C.10)
				AccumulateInfluence(*S, g.origin, rows, true);
				Vec3 va = free_stream + rows.vel_known;
				for (int k = 0; k < n_unknowns; ++k) va += Vec3{rows.vel[0][k], rows.vel[1][k], rows.vel[2][k]} * sol.parameters[k];
				pr.v_average = va;
				pr.v_upper = va + vd * 0.5;
				pr.v_lower = va - vd * 0.5;
				if (dirichlet_interior && Length(pr.v_lower - free_stream) > options.interior_velocity_tolerance * std::sqrt(vinf2)) {
					// The sum has gone wrong here (the interior side should
					// carry the free stream exactly): the identity instead.
					pr.v_lower = free_stream;
					pr.v_upper = free_stream + vd;
					pr.v_average = (pr.v_upper + pr.v_lower) * 0.5;
				}
				pr.cp_upper = 1.0 - LengthSq(pr.v_upper) / vinf2;
				pr.cp_lower = 1.0 - LengthSq(pr.v_lower) / vinf2;
				// Loads: pressure times the outward normal; both sides of a sheet.
				double cp_net = one_sided ? -pr.cp_upper : (pr.cp_lower - pr.cp_upper);
				Vec3 f = g.ez * (cp_net * pr.area / options.reference_area);
				nr.force_coefficient += f;
				nr.moment_coefficient += Cross(g.origin - options.moment_reference, f);
				nr.panels.push_back(pr);
			}
		}
		if (mirrored) {
			nr.force_coefficient = {2.0 * nr.force_coefficient.x, 0.0, 2.0 * nr.force_coefficient.z};
			nr.moment_coefficient = {0.0, 2.0 * nr.moment_coefficient.y, 0.0};
		}
		sol.force_coefficient += nr.force_coefficient;
		sol.moment_coefficient += nr.moment_coefficient;
		sol.networks.push_back(std::move(nr));
	}
	// Lift from the wakes' head doublet strength: C_L = (2 / V∞ S_R) ∫ μ dy.
	{
		double integral = 0.0;
		bool any = false;
		for (const NetworkData& nd : S->data) {
			if (!nd.net->IsWake()) continue;
			any = true;
			for (int j = 0; j < nd.pcols; ++j) {
				double mu = nd.net->type == NetworkType::DoubletWake ? sol.parameters[nd.params[j].global] : sol.parameters[nd.params[0].global];
				double dy = std::fabs(nd.net->Corner(1, j + 2).y - nd.net->Corner(1, j + 1).y);
				integral += mu * dy;
			}
		}
		if (any) {
			if (mirrored) integral *= 2.0;
			sol.lift_from_wake = 2.0 * integral / (std::sqrt(vinf2) * options.reference_area);
		}
	}
	return sol;
}

// ---------------------------------------------------------------------------
// Field evaluation
// ---------------------------------------------------------------------------

Vec3 VelocityAt(const Solution& sol, const Vec3& point) {
	InfluenceRows rows;
	AccumulateInfluence(*sol.internal, point, rows, true);
	Vec3 v = sol.free_stream + rows.vel_known;
	for (int k = 0; k < sol.unknowns; ++k) v += Vec3{rows.vel[0][k], rows.vel[1][k], rows.vel[2][k]} * sol.parameters[k];
	return v;
}

double PotentialAt(const Solution& sol, const Vec3& point) {
	InfluenceRows rows;
	AccumulateInfluence(*sol.internal, point, rows, false);
	double phi = rows.phi_known;
	for (int k = 0; k < sol.unknowns; ++k) phi += rows.phi[k] * sol.parameters[k];
	return phi;
}

// ---------------------------------------------------------------------------
// Builders
// ---------------------------------------------------------------------------

Network GridNetwork(NetworkType type, const std::string& name, int rows, int cols, const std::function<Vec3(double, double)>& surface, const std::vector<double>* u_values, const std::vector<double>* v_values) {
	Network net;
	net.type = type;
	net.name = name;
	net.rows = rows;
	net.cols = cols;
	net.corners.resize(static_cast<size_t>(rows) * cols);
	for (int i = 0; i < rows; ++i) {
		double u = u_values ? (*u_values)[i] : static_cast<double>(i) / (rows - 1);
		for (int j = 0; j < cols; ++j) {
			double v = v_values ? (*v_values)[j] : static_cast<double>(j) / (cols - 1);
			net.corners[static_cast<size_t>(i) * cols + j] = surface(u, v);
		}
	}
	return net;
}

Network WakeNetwork(const std::string& name, const std::vector<Vec3>& trailing_edge, const Vec3& direction, double length, NetworkType type) {
	Network net;
	net.type = type;
	net.name = name;
	net.rows = 2;
	net.cols = static_cast<int>(trailing_edge.size());
	net.corners.resize(static_cast<size_t>(2) * net.cols);
	Vec3 d = Normalized(direction) * length;
	for (int j = 0; j < net.cols; ++j) {
		net.corners[j] = trailing_edge[j];
		net.corners[static_cast<size_t>(net.cols) + j] = trailing_edge[j] + d;
	}
	// The wake's upper side is direction x (edge tangent): behind a wing
	// whose trailing edge runs root to tip in +y and sheds in +x, that is
	// +z, the wing's own upper side, so μ_wake = μ_upper - μ_lower comes out
	// positive for positive lift.
	if (net.cols >= 2) {
		int J = net.cols / 2;
		Vec3 P1 = net.Corner(1, J), P2 = net.Corner(1, J + 1), P3 = net.Corner(2, J + 1), P4 = net.Corner(2, J);
		Vec3 zeta = Cross(P1 + P4 - P2 - P3, P1 + P2 - P3 - P4);
		Vec3 want = Cross(direction, trailing_edge.back() - trailing_edge.front());
		if (Dot(zeta, want) < 0.0) FlipNormals(net);
	}
	return net;
}

void FlipNormals(Network& net) {
	// Reversing the column order reverses ζ = (∂P/∂J) x (∂P/∂I).
	std::vector<Vec3> c = net.corners;
	for (int i = 0; i < net.rows; ++i)
		for (int j = 0; j < net.cols; ++j) net.corners[static_cast<size_t>(i) * net.cols + j] = c[static_cast<size_t>(i) * net.cols + (net.cols - 1 - j)];
	std::swap(net.edges[1], net.edges[3]);
}

std::vector<double> CosineSpacing(int n) {
	std::vector<double> out(n + 1);
	for (int i = 0; i <= n; ++i) out[i] = 0.5 * (1.0 - std::cos(kPi * i / n));
	return out;
}

} // namespace aerotelic::panel
