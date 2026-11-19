#include "aerotelic/tri_panel.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <unordered_map>

#include "aerotelic/mesh_cut.hpp"
#include "aerotelic/parallel.hpp"
#include "aerotelic/sdf.hpp"

namespace aerotelic::tripanel {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kFourPi = 4.0 * kPi;

// ---------------------------------------------------------------------------
// Panel geometry and the closed-form influence of a flat triangle
// ---------------------------------------------------------------------------

struct PanelGeom {
	Vec3 v[3];
	Vec3 c, n;
	double area = 0.0, radius = 0.0;
	Vec3 inward[3];   // n x e_hat per edge: into the triangle
	double len[3] = {0.0, 0.0, 0.0};
	bool degenerate = false;
};

PanelGeom MakeGeom(const Vec3& a, const Vec3& b, const Vec3& c, double tiny_area) {
	PanelGeom g;
	g.v[0] = a; g.v[1] = b; g.v[2] = c;
	g.c = (a + b + c) * (1.0 / 3.0);
	const Vec3 cr = Cross(b - a, c - a);
	const double l = Length(cr);
	g.area = 0.5 * l;
	if (g.area <= tiny_area) { g.degenerate = true; g.n = {0.0, 0.0, 1.0}; return g; }
	g.n = cr * (1.0 / l);
	g.radius = std::sqrt(g.area);
	for (int k = 0; k < 3; ++k) {
		const Vec3 e = g.v[(k + 1) % 3] - g.v[k];
		g.len[k] = Length(e);
		g.inward[k] = g.len[k] > 0.0 ? Cross(g.n, e * (1.0 / g.len[k])) : Vec3{0.0, 0.0, 0.0};
	}
	return g;
}

PanelGeom Mirrored(const PanelGeom& g, double y0) {
	auto m = [y0](Vec3 p) { p.y = 2.0 * y0 - p.y; return p; };
	// The reflection reverses the winding; swapping two vertices puts the
	// normal back outward.
	return MakeGeom(m(g.v[0]), m(g.v[2]), m(g.v[1]), 0.0);
}

// The signed solid angle of the triangle at P (positive on the +n side:
// integral of n . (P - Q) / r^3 over the triangle), and the integral of
// 1 / r over it. Van Oosterom & Strackee, IEEE Trans. Biomed. Eng. 30
// (1983) 125 for the solid angle; Hess & Smith (1967) for the edge terms.
struct Influence { double omega = 0.0, inv_r = 0.0; };

Influence Exact(const PanelGeom& g, const Vec3& P) {
	Influence out;
	const Vec3 R[3] = {g.v[0] - P, g.v[1] - P, g.v[2] - P};
	const double r[3] = {Length(R[0]), Length(R[1]), Length(R[2])};
	const double triple = Dot(R[0], Cross(R[1], R[2]));
	const double denom = r[0] * r[1] * r[2] + Dot(R[0], R[1]) * r[2] + Dot(R[1], R[2]) * r[0] + Dot(R[2], R[0]) * r[1];
	out.omega = 2.0 * std::atan2(-triple, denom);
	const double h = Dot(P - g.c, g.n);
	const Vec3 pp = P - g.n * h;
	double sum = 0.0;
	for (int k = 0; k < 3; ++k) {
		const double r1 = r[k], r2 = r[(k + 1) % 3], d = g.len[k];
		if (d <= 0.0) continue;
		const double den = r1 + r2 - d;
		if (den <= 1e-13 * d) continue; // P on the edge: rho vanishes with it
		const double rho = Dot(pp - g.v[k], g.inward[k]);
		sum += rho * std::log((r1 + r2 + d) / den);
	}
	out.inv_r = sum - std::fabs(h) * std::fabs(out.omega);
	return out;
}

Influence Far(const PanelGeom& g, const Vec3& P) {
	Influence out;
	const Vec3 d = P - g.c;
	const double r2 = LengthSq(d), r = std::sqrt(r2);
	out.inv_r = g.area / r;
	out.omega = g.area * Dot(g.n, d) / (r2 * r);
	return out;
}

inline Influence InfluenceAt(const PanelGeom& g, const Vec3& P, double far_radii) {
	if (g.degenerate) return {};
	const double r2 = DistanceSq(P, g.c);
	const double lim = far_radii * g.radius;
	return r2 > lim * lim ? Far(g, P) : Exact(g, P);
}

// ---------------------------------------------------------------------------
// Dense solvers
// ---------------------------------------------------------------------------

// LU with partial pivoting in place, row-major; x returned in b. Returns
// the smallest pivot over the largest.
double SolveLu(std::vector<double>& A, std::vector<double>& b, int n, int threads) {
	double pmin = std::numeric_limits<double>::infinity(), pmax = 0.0;
	std::vector<int> perm(n);
	for (int k = 0; k < n; ++k) {
		int p = k;
		double best = std::fabs(A[static_cast<size_t>(k) * n + k]);
		for (int i = k + 1; i < n; ++i) {
			const double v = std::fabs(A[static_cast<size_t>(i) * n + k]);
			if (v > best) { best = v; p = i; }
		}
		if (best <= 0.0) throw std::runtime_error("tri_panel: the influence matrix is singular");
		pmin = std::min(pmin, best);
		pmax = std::max(pmax, best);
		if (p != k) {
			for (int j = 0; j < n; ++j) std::swap(A[static_cast<size_t>(k) * n + j], A[static_cast<size_t>(p) * n + j]);
			std::swap(b[k], b[p]);
		}
		const double* rowk = &A[static_cast<size_t>(k) * n];
		const double pivot = rowk[k];
		const size_t rows_below = static_cast<size_t>(n - k - 1);
		ParallelFor(rows_below, [&](size_t ii) {
			const int i = k + 1 + static_cast<int>(ii);
			double* rowi = &A[static_cast<size_t>(i) * n];
			const double f = rowi[k] / pivot;
			if (f == 0.0) return;
			rowi[k] = f;
			for (int j = k + 1; j < n; ++j) rowi[j] -= f * rowk[j];
		}, threads, 32);
	}
	// Forward: L has unit diagonal, multipliers stored below it.
	for (int i = 1; i < n; ++i) {
		const double* rowi = &A[static_cast<size_t>(i) * n];
		double s = b[i];
		for (int j = 0; j < i; ++j) s -= rowi[j] * b[j];
		b[i] = s;
	}
	for (int i = n - 1; i >= 0; --i) {
		const double* rowi = &A[static_cast<size_t>(i) * n];
		double s = b[i];
		for (int j = i + 1; j < n; ++j) s -= rowi[j] * b[j];
		b[i] = s / rowi[i];
	}
	return pmax > 0.0 ? pmin / pmax : 0.0;
}

void MatVec(const std::vector<double>& A, const std::vector<double>& x, std::vector<double>& y, int n, int threads) {
	ParallelFor(static_cast<size_t>(n), [&](size_t i) {
		const double* row = &A[i * n];
		double s = 0.0;
		for (int j = 0; j < n; ++j) s += row[j] * x[j];
		y[i] = s;
	}, threads, 16);
}

// Restarted GMRES on the Jacobi-scaled system A D^-1 y = b, x = D^-1 y.
// Returns the relative residual reached; iterations in `iters`.
double SolveGmres(const std::vector<double>& A, const std::vector<double>& b, std::vector<double>& x, int n, int threads, double tol, int m, int max_iter, int& iters) {
	std::vector<double> dinv(n);
	for (int i = 0; i < n; ++i) {
		const double d = A[static_cast<size_t>(i) * n + i];
		if (std::fabs(d) < 1e-300) throw std::runtime_error("tri_panel: a zero diagonal in the influence matrix");
		dinv[i] = 1.0 / d;
	}
	auto apply = [&](const std::vector<double>& y, std::vector<double>& out) {
		std::vector<double> t(n);
		for (int i = 0; i < n; ++i) t[i] = dinv[i] * y[i];
		MatVec(A, t, out, n, threads);
	};
	auto norm = [](const std::vector<double>& v) { double s = 0.0; for (double a : v) s += a * a; return std::sqrt(s); };
	const double bnorm = std::max(norm(b), 1e-300);
	std::vector<double> y(n, 0.0);  // the scaled unknown
	for (int i = 0; i < n; ++i) y[i] = A[static_cast<size_t>(i) * n + i] * x[i];
	std::vector<double> r(n), Ay(n);
	iters = 0;
	double rel = 1.0;
	while (iters < max_iter) {
		apply(y, Ay);
		for (int i = 0; i < n; ++i) r[i] = b[i] - Ay[i];
		double beta = norm(r);
		rel = beta / bnorm;
		if (rel < tol) break;
		std::vector<std::vector<double>> V;
		V.push_back(r);
		for (double& a : V[0]) a /= beta;
		std::vector<double> H(static_cast<size_t>(m + 1) * m, 0.0); // H[i * m + j]
		std::vector<double> cs(m), sn(m), g(m + 1, 0.0);
		g[0] = beta;
		int k = 0;
		for (; k < m && iters < max_iter; ++k, ++iters) {
			std::vector<double> w(n);
			apply(V[k], w);
			for (int i = 0; i <= k; ++i) {
				double h = 0.0;
				for (int q = 0; q < n; ++q) h += w[q] * V[i][q];
				H[static_cast<size_t>(i) * m + k] = h;
				for (int q = 0; q < n; ++q) w[q] -= h * V[i][q];
			}
			const double hn = norm(w);
			H[static_cast<size_t>(k + 1) * m + k] = hn;
			if (hn > 1e-300) { for (double& a : w) a /= hn; }
			V.push_back(w);
			// Apply the previous rotations to the new column, then a new one.
			for (int i = 0; i < k; ++i) {
				const double a = H[static_cast<size_t>(i) * m + k], bb = H[static_cast<size_t>(i + 1) * m + k];
				H[static_cast<size_t>(i) * m + k] = cs[i] * a + sn[i] * bb;
				H[static_cast<size_t>(i + 1) * m + k] = -sn[i] * a + cs[i] * bb;
			}
			const double a = H[static_cast<size_t>(k) * m + k], bb = H[static_cast<size_t>(k + 1) * m + k];
			const double rr = std::hypot(a, bb);
			cs[k] = rr > 0.0 ? a / rr : 1.0;
			sn[k] = rr > 0.0 ? bb / rr : 0.0;
			H[static_cast<size_t>(k) * m + k] = rr;
			H[static_cast<size_t>(k + 1) * m + k] = 0.0;
			const double gk = g[k];
			g[k] = cs[k] * gk;
			g[k + 1] = -sn[k] * gk;
			rel = std::fabs(g[k + 1]) / bnorm;
			if (rel < tol) { ++k; ++iters; break; }
		}
		// Solve the triangular system for the k coefficients and update y.
		std::vector<double> coef(k, 0.0);
		for (int i = k - 1; i >= 0; --i) {
			double s = g[i];
			for (int j = i + 1; j < k; ++j) s -= H[static_cast<size_t>(i) * m + j] * coef[j];
			const double d = H[static_cast<size_t>(i) * m + i];
			coef[i] = std::fabs(d) > 1e-300 ? s / d : 0.0;
		}
		for (int j = 0; j < k; ++j) for (int q = 0; q < n; ++q) y[q] += coef[j] * V[j][q];
		if (rel < tol) break;
	}
	apply(y, Ay);
	for (int i = 0; i < n; ++i) r[i] = b[i] - Ay[i];
	rel = norm(r) / bnorm;
	for (int i = 0; i < n; ++i) x[i] = dinv[i] * y[i];
	return rel;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

uint64_t EdgeKey(int a, int b) {
	if (a > b) std::swap(a, b);
	return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) | static_cast<uint32_t>(b);
}

double DistanceToPolyline(const Vec3& p, const std::vector<Vec3>& line) {
	double best = std::numeric_limits<double>::infinity();
	if (line.size() == 1) return Distance(p, line[0]);
	for (size_t i = 0; i + 1 < line.size(); ++i) {
		const Vec3 a = line[i], b = line[i + 1], e = b - a;
		const double l2 = LengthSq(e);
		double t = l2 > 0.0 ? Dot(p - a, e) / l2 : 0.0;
		t = std::clamp(t, 0.0, 1.0);
		best = std::min(best, Distance(p, a + e * t));
	}
	return best;
}

struct Prepared {
	const Body* body = nullptr;
	BodyResult result;
	std::vector<PanelGeom> geoms;
	int first = 0; // the index of its first panel among the unknowns
};

} // namespace

// ---------------------------------------------------------------------------
// Edges and statistics
// ---------------------------------------------------------------------------

std::vector<std::array<int, 2>> EdgesOnPolylines(const TriMesh& mesh, const std::vector<std::vector<Vec3>>& polylines, double tol) {
	std::vector<std::array<int, 2>> out;
	if (polylines.empty()) return out;
	// Vertices on any polyline, once.
	std::vector<char> on(mesh.vertices.size(), 0);
	for (size_t i = 0; i < mesh.vertices.size(); ++i) {
		for (const auto& line : polylines) {
			if (line.empty()) continue;
			if (DistanceToPolyline(mesh.vertices[i], line) <= tol) { on[i] = 1; break; }
		}
	}
	std::unordered_map<uint64_t, int> seen;
	for (const auto& t : mesh.triangles) {
		for (int k = 0; k < 3; ++k) {
			const int a = t[k], b = t[(k + 1) % 3];
			if (!on[a] || !on[b]) continue;
			const uint64_t key = EdgeKey(a, b);
			if (seen.count(key)) continue;
			seen[key] = 1;
			const Vec3 mid = (mesh.vertices[a] + mesh.vertices[b]) * 0.5;
			bool mid_on = false;
			for (const auto& line : polylines) if (!line.empty() && DistanceToPolyline(mid, line) <= tol) { mid_on = true; break; }
			if (!mid_on) continue;
			out.push_back({std::min(a, b), std::max(a, b)});
		}
	}
	return out;
}

MeshStats ComputeMeshStats(const TriMesh& mesh) {
	MeshStats s;
	s.triangles = static_cast<int>(mesh.triangles.size());
	s.vertices = static_cast<int>(mesh.vertices.size());
	if (mesh.triangles.empty()) return s;
	std::vector<double> edges;
	edges.reserve(mesh.triangles.size() * 3);
	s.min_edge = std::numeric_limits<double>::infinity();
	for (const auto& t : mesh.triangles) {
		const Vec3& a = mesh.vertices[t[0]]; const Vec3& b = mesh.vertices[t[1]]; const Vec3& c = mesh.vertices[t[2]];
		const double l0 = Distance(a, b), l1 = Distance(b, c), l2 = Distance(c, a);
		edges.push_back(l0); edges.push_back(l1); edges.push_back(l2);
		const double longest = std::max({l0, l1, l2});
		const double area = 0.5 * Length(Cross(b - a, c - a));
		s.area += area;
		const double aspect = area > 0.0 ? longest * longest / (2.0 * area) : 1e300;
		s.worst_aspect = std::max(s.worst_aspect, aspect);
		if (aspect > 20.0) ++s.slivers;
		s.min_edge = std::min({s.min_edge, l0, l1, l2});
		s.max_edge = std::max(s.max_edge, longest);
	}
	std::nth_element(edges.begin(), edges.begin() + edges.size() / 2, edges.end());
	s.median_edge = edges[edges.size() / 2];
	s.volume = mesh.SignedVolume();
	s.closed = mesh.IsClosedManifold();
	s.outward = s.volume > 0.0;
	return s;
}

int TrianglesNear(const TriMesh& a, const TriMesh& b, double distance) {
	if (a.triangles.empty() || b.triangles.empty()) return 0;
	MeshSdf sdf(b);
	int n = 0;
	for (const auto& t : a.triangles) {
		const Vec3 c = (a.vertices[t[0]] + a.vertices[t[1]] + a.vertices[t[2]]) * (1.0 / 3.0);
		if (std::fabs(sdf.Distance(c)) < distance) ++n;
	}
	return n;
}

// ---------------------------------------------------------------------------
// The solve
// ---------------------------------------------------------------------------

Solution Solve(const std::vector<Body>& bodies, const Vec3& free_stream, const Options& options) {
	using Clock = std::chrono::steady_clock;
	if (bodies.empty()) throw std::invalid_argument("tri_panel: no body to panel");
	const double vinf = Length(free_stream);
	if (!(vinf > 0.0)) throw std::invalid_argument("tri_panel: the free stream has no magnitude");
	const Vec3 d = free_stream * (1.0 / vinf);

	Solution sol;
	sol.free_stream = free_stream;

	// The configuration's extent.
	Box3 box;
	for (const Body& b : bodies) for (const Vec3& v : b.mesh.vertices) box.Add(v);
	const double diag = std::max(box.Diagonal(), 1e-300);
	const double tiny = 1e-9 * diag;
	const double tiny_area = 1e-20 * diag * diag;

	// ---- prepare every body: validate, cut at the plane, panel geometry ----
	std::vector<Prepared> prep(bodies.size());
	int n_panels = 0;
	for (size_t bi = 0; bi < bodies.size(); ++bi) {
		const Body& b = bodies[bi];
		Prepared& p = prep[bi];
		p.body = &b;
		p.result.name = b.name;
		if (b.mesh.triangles.empty()) throw std::invalid_argument("tri_panel: the body '" + b.name + "' has no triangles");
		if (!b.mesh.IsClosedManifold()) throw std::invalid_argument("tri_panel: the body '" + b.name + "' is not a closed surface; the method needs closed bodies");
		if (!(b.mesh.SignedVolume() > 0.0)) throw std::invalid_argument("tri_panel: the body '" + b.name + "' faces inward (negative volume)");
		TriMesh mesh = b.mesh;
		std::vector<int> parent(mesh.triangles.size());
		for (size_t i = 0; i < parent.size(); ++i) parent[i] = static_cast<int>(i);
		if (options.symmetry_y) {
			const double y0 = *options.symmetry_y;
			// Vertices a whisker off the plane are put ON it first. A body
			// nudged off the plane by a millionth of the configuration (the
			// retry of a refused fuse; a fin placed at y = 1e-7 by hand)
			// otherwise leaves, once cut, slivers a millionth wide facing
			// their own image, and the gradient fit across them runs to 1e5
			// (my_drone's fin: Cp -5e9, 2026-11-16).
			const double snap = std::max(tiny, 1e-5 * diag);
			int snapped = 0;
			for (Vec3& v : mesh.vertices) if (v.y != y0 && std::fabs(v.y - y0) <= snap) { v.y = y0; ++snapped; }
			if (snapped > 0) p.result.notes.push_back(b.name + ": " + std::to_string(snapped) + " vertex(es) within " + std::to_string(snap) + " of the symmetry plane put on it");
			double ymin = 1e300, ymax = -1e300;
			for (const Vec3& v : mesh.vertices) { ymin = std::min(ymin, v.y); ymax = std::max(ymax, v.y); }
			if (ymin < y0 - tiny && ymax > y0 + tiny) {
				// Straddling the plane: cut, keep the y >= plane side.
				Plane pl;
				pl.normal = {0.0, 1.0, 0.0};
				pl.offset = y0;
				std::vector<int> cut_parent;
				TriMesh cut = CutMeshByPlanes(mesh, {pl}, tiny, &cut_parent);
				TriMesh half;
				half.vertices = cut.vertices;
				std::vector<int> half_parent;
				for (size_t t = 0; t < cut.triangles.size(); ++t) {
					const auto& tr = cut.triangles[t];
					const double cy = (cut.vertices[tr[0]].y + cut.vertices[tr[1]].y + cut.vertices[tr[2]].y) / 3.0 - y0;
					if (cy < -tiny) continue;
					half.triangles.push_back(tr);
					half_parent.push_back(parent[cut_parent[t]]);
				}
				mesh = std::move(half);
				parent = std::move(half_parent);
				p.result.cut_at_plane = true;
			} else if (ymax <= y0 + tiny) {
				p.result.notes.push_back(b.name + ": lies wholly on the y < plane side; its image is what the plane shows");
			}
			// A triangle lying IN the plane (a half wing's root cap, the wall a
			// cut leaves) coincides with its own image, facing the other way:
			// it is not a wetted surface and would sit on its image's control
			// point. Left out, whether the body was cut or not.
			{
				TriMesh kept;
				kept.vertices = mesh.vertices;
				std::vector<int> kept_parent;
				int dropped = 0;
				for (size_t t = 0; t < mesh.triangles.size(); ++t) {
					const auto& tr = mesh.triangles[t];
					bool in_plane = true;
					for (int k = 0; k < 3 && in_plane; ++k) in_plane = std::fabs(mesh.vertices[tr[k]].y - y0) <= tiny;
					if (in_plane) { ++dropped; continue; }
					kept.triangles.push_back(tr);
					kept_parent.push_back(parent[t]);
				}
				if (dropped > 0) {
					mesh = std::move(kept);
					parent = std::move(kept_parent);
					p.result.dropped_in_plane = dropped;
					p.result.notes.push_back(b.name + ": " + std::to_string(dropped) + " triangle(s) lying in the symmetry plane left out (the image closes the body there)");
				}
			}
			if (mesh.triangles.empty()) throw std::invalid_argument("tri_panel: the body '" + b.name + "' has nothing on the y >= plane side");
		}
		p.geoms.reserve(mesh.triangles.size());
		int degenerate = 0;
		for (const auto& t : mesh.triangles) {
			p.geoms.push_back(MakeGeom(mesh.vertices[t[0]], mesh.vertices[t[1]], mesh.vertices[t[2]], tiny_area));
			if (p.geoms.back().degenerate) ++degenerate;
		}
		if (degenerate > 0) p.result.notes.push_back(b.name + ": " + std::to_string(degenerate) + " triangle(s) of no area carry no panel");
		p.result.mesh = std::move(mesh);
		p.result.parent = std::move(parent);
		p.first = n_panels;
		n_panels += static_cast<int>(p.geoms.size());
	}
	if (n_panels > options.max_panels) {
		throw std::invalid_argument("tri_panel: " + std::to_string(n_panels) + " panels is more than the solver takes (" + std::to_string(options.max_panels)
			+ "): coarsen the bodies' grids on their Mesh tabs (stations and points round a section, refinement off), or raise the limit");
	}
	sol.unknowns = n_panels;

	// ---- the wakes: trailing-edge edges with a triangle on each side ----
	const double wake_length = options.wake_length > 0.0 ? options.wake_length : 20.0 * diag;
	std::vector<std::unique_ptr<MeshSdf>> sdfs(bodies.size());
	auto sdf_of = [&](size_t bi) -> const MeshSdf& {
		if (!sdfs[bi]) sdfs[bi] = std::make_unique<MeshSdf>(bodies[bi].mesh);
		return *sdfs[bi];
	};
	const double probe = options.buried_probe > 0.0 ? options.buried_probe : 0.02 * diag;
	const double buried_tol = options.buried_tolerance > 0.0 ? options.buried_tolerance : 0.05 * diag;
	for (size_t bi = 0; bi < bodies.size(); ++bi) {
		Prepared& p = prep[bi];
		const Body& b = bodies[bi];
		if (b.trailing_edges.empty()) continue;
		const TriMesh& mesh = p.result.mesh;
		const double tol = b.trailing_edge_tolerance > 0.0 ? b.trailing_edge_tolerance : 1e-6 * diag;
		std::vector<std::array<int, 2>> edges = EdgesOnPolylines(mesh, b.trailing_edges, tol);
		if (edges.empty()) { p.result.notes.push_back(b.name + ": no mesh edge lies on its trailing edge, so it sheds no wake"); continue; }
		// Edge -> triangles.
		std::unordered_map<uint64_t, std::vector<int>> adj;
		for (size_t t = 0; t < mesh.triangles.size(); ++t) {
			const auto& tr = mesh.triangles[t];
			for (int k = 0; k < 3; ++k) adj[EdgeKey(tr[k], tr[(k + 1) % 3])].push_back(static_cast<int>(t));
		}
		int non_manifold = 0, parallel = 0;
		for (const auto& e : edges) {
			auto it = adj.find(EdgeKey(e[0], e[1]));
			if (it == adj.end()) continue;
			const std::vector<int>& tris = it->second;
			if (tris.size() == 1) { ++p.result.rim_edges; continue; }
			if (tris.size() != 2) { ++non_manifold; continue; }
			const Vec3 a = mesh.vertices[e[0]], bb = mesh.vertices[e[1]];
			// An edge lying in y = 0 with the stream in that plane and no plane
			// option (a centreline fin, 2026-11-16: my_drone's fin wake ran
			// along the tailplane's root a fraction of a millimetre from its
			// panels and left the tailplane 30% asymmetric): no wake, as the
			// rim rule gives with the plane on. See BodyResult::centreline_edges.
			if (!options.symmetry_y && std::fabs(d.y) < 1e-9 && std::fabs(a.y) <= 1e-5 * diag && std::fabs(bb.y) <= 1e-5 * diag) { ++p.result.centreline_edges; continue; }
			const Vec3 dl = bb - a;
			Vec3 nw = Cross(d, dl);
			const double nwl = Length(nw);
			if (nwl <= 1e-12 * Length(dl)) { ++parallel; continue; }
			nw = nw * (1.0 / nwl);
			if (options.buried_rule && bodies.size() > 1) {
				const Vec3 mid = (a + bb) * 0.5 + d * probe;
				bool buried = false;
				for (size_t bj = 0; bj < bodies.size() && !buried; ++bj) {
					if (bj == bi) continue;
					if (sdf_of(bj).Distance(mid) < buried_tol) buried = true;
				}
				if (buried) { ++p.result.buried_edges; continue; }
			}
			const int t0 = tris[0], t1 = tris[1];
			const double d0 = Dot(p.geoms[t0].n, nw), d1 = Dot(p.geoms[t1].n, nw);
			WakeStrip w;
			w.body = static_cast<int>(bi);
			w.upper = p.first + (d0 >= d1 ? t0 : t1);
			w.lower = p.first + (d0 >= d1 ? t1 : t0);
			// Corners ordered so the strip's normal is nw: (b, a, a + L d, b + L d).
			w.corners = {bb, a, a + d * wake_length, bb + d * wake_length};
			sol.wakes.push_back(w);
			++p.result.wake_edges;
		}
		if (non_manifold > 0) p.result.notes.push_back(b.name + ": " + std::to_string(non_manifold) + " trailing-edge edge(s) with more than two triangles shed no wake");
		if (parallel > 0) p.result.notes.push_back(b.name + ": " + std::to_string(parallel) + " trailing-edge edge(s) run along the stream and shed no wake");
		if (p.result.buried_edges > 0) p.result.notes.push_back(b.name + ": " + std::to_string(p.result.buried_edges) + " trailing-edge edge(s) run into another body downstream and shed no wake");
		if (p.result.rim_edges > 0) p.result.notes.push_back(b.name + ": " + std::to_string(p.result.rim_edges) + " trailing-edge edge(s) lie on the symmetry plane's rim (the image is the other side) and shed no wake");
		if (p.result.centreline_edges > 0) p.result.notes.push_back(b.name + ": " + std::to_string(p.result.centreline_edges) + " trailing-edge edge(s) lie in the plane y = 0 with the free stream in it (a centreline fin) and shed no wake: a sheet there would run along the centreline geometry within a fraction of a panel of its panels; by symmetry its strength is zero (a sideslip sheds it)");
	}

	// ---- the influence matrix ----
	const auto t_influence0 = Clock::now();
	std::vector<const PanelGeom*> geom_of(n_panels);
	std::vector<double> sigma(n_panels, 0.0);
	for (const Prepared& p : prep) {
		for (size_t t = 0; t < p.geoms.size(); ++t) {
			geom_of[p.first + t] = &p.geoms[t];
			sigma[p.first + t] = p.geoms[t].degenerate ? 0.0 : -Dot(free_stream, p.geoms[t].n);
		}
	}
	const bool mirrored = options.symmetry_y.has_value();
	const double y0 = options.symmetry_y.value_or(0.0);
	std::vector<PanelGeom> image_geoms;
	std::vector<double> image_sigma;
	if (mirrored) {
		image_geoms.resize(n_panels);
		image_sigma.resize(n_panels);
		for (int j = 0; j < n_panels; ++j) {
			image_geoms[j] = geom_of[j]->degenerate ? *geom_of[j] : Mirrored(*geom_of[j], y0);
			image_sigma[j] = geom_of[j]->degenerate ? 0.0 : -Dot(free_stream, image_geoms[j].n);
		}
	}
	// Wake strips as two triangles each (and their images).
	struct WakeTri { PanelGeom g; int upper, lower; };
	std::vector<WakeTri> wake_tris;
	for (const WakeStrip& w : sol.wakes) {
		const Vec3& c0 = w.corners[0]; const Vec3& c1 = w.corners[1]; const Vec3& c2 = w.corners[2]; const Vec3& c3 = w.corners[3];
		const PanelGeom g0 = MakeGeom(c0, c1, c2, 0.0), g1 = MakeGeom(c0, c2, c3, 0.0);
		wake_tris.push_back({g0, w.upper, w.lower});
		wake_tris.push_back({g1, w.upper, w.lower});
		if (mirrored) {
			wake_tris.push_back({Mirrored(g0, y0), w.upper, w.lower});
			wake_tris.push_back({Mirrored(g1, y0), w.upper, w.lower});
		}
	}

	const size_t nn = static_cast<size_t>(n_panels);
	std::vector<double> A;
	try {
		A.assign(nn * nn, 0.0);
	} catch (const std::bad_alloc&) {
		throw std::runtime_error("tri_panel: out of memory for " + std::to_string(n_panels) + " x " + std::to_string(n_panels) + " influence coefficients: coarsen the grids");
	}
	std::vector<double> rhs(nn, 0.0);
	const double far = options.far_field_radii;
	ParallelFor(nn, [&](size_t i) {
		double* row = &A[i * nn];
		const PanelGeom& gi = *geom_of[i];
		if (gi.degenerate) { row[i] = 1.0; rhs[i] = 0.0; return; }
		const Vec3 P = gi.c;
		double r = 0.0;
		for (size_t j = 0; j < nn; ++j) {
			const PanelGeom& gj = *geom_of[j];
			if (gj.degenerate) continue;
			if (j == i) {
				row[j] += -0.5;
				const Influence self = Exact(gj, P);
				r -= -self.inv_r / kFourPi * sigma[j];
			} else {
				const Influence f = InfluenceAt(gj, P, far);
				row[j] += f.omega / kFourPi;
				r -= -f.inv_r / kFourPi * sigma[j];
			}
			if (mirrored) {
				const Influence f = InfluenceAt(image_geoms[j], P, far);
				row[j] += f.omega / kFourPi;
				r -= -f.inv_r / kFourPi * image_sigma[j];
			}
		}
		for (const WakeTri& w : wake_tris) {
			const Influence f = Exact(w.g, P);
			const double c = f.omega / kFourPi;
			row[w.upper] += c;
			row[w.lower] -= c;
		}
		rhs[i] = r;
	}, options.threads, 8);
	sol.seconds_influence = std::chrono::duration<double>(Clock::now() - t_influence0).count();

	// ---- solve ----
	const auto t_solve0 = Clock::now();
	std::vector<double> mu(nn, 0.0);
	if (n_panels <= options.direct_max_panels) {
		mu = rhs;
		sol.pivot_ratio = SolveLu(A, mu, n_panels, options.threads);
		sol.direct = true;
	} else {
		sol.direct = false;
		sol.residual = SolveGmres(A, rhs, mu, n_panels, options.threads, options.gmres_tolerance, options.gmres_restart, options.gmres_max_iterations, sol.iterations);
		if (!(sol.residual < 1e-4)) throw std::runtime_error("tri_panel: GMRES did not converge (relative residual " + std::to_string(sol.residual) + " after " + std::to_string(sol.iterations) + " iterations)");
	}
	A.clear();
	A.shrink_to_fit();
	sol.seconds_solve = std::chrono::duration<double>(Clock::now() - t_solve0).count();

	// ---- surface velocity, pressure, forces ----
	const double cos_max = std::cos(options.gradient_max_angle_deg * kPi / 180.0);
	const Vec3 ref = options.moment_reference;
	const double inv_s = 1.0 / std::max(options.reference_area, 1e-300);
	for (Prepared& p : prep) {
		const TriMesh& mesh = p.result.mesh;
		std::vector<std::vector<int>> vert_tris(mesh.vertices.size());
		for (size_t t = 0; t < mesh.triangles.size(); ++t) for (int k = 0; k < 3; ++k) vert_tris[mesh.triangles[t][k]].push_back(static_cast<int>(t));
		p.result.panels.resize(p.geoms.size());
		Vec3 F{0.0, 0.0, 0.0}, M{0.0, 0.0, 0.0};
		for (size_t t = 0; t < p.geoms.size(); ++t) {
			const PanelGeom& g = p.geoms[t];
			PanelResult& r = p.result.panels[t];
			r.centre = g.c;
			r.normal = g.n;
			r.area = g.degenerate ? 0.0 : g.area;
			r.sigma = sigma[p.first + t];
			r.mu = mu[p.first + t];
			if (g.degenerate) { r.cp = 0.0; r.velocity = free_stream; continue; }
			// Neighbours across edges and vertices, facing the same way.
			std::vector<int> nb;
			for (int k = 0; k < 3; ++k) for (int o : vert_tris[mesh.triangles[t][k]]) if (o != static_cast<int>(t) && std::find(nb.begin(), nb.end(), o) == nb.end()) nb.push_back(o);
			Vec3 t1 = Cross(g.n, Vec3{1.0, 0.0, 0.0});
			if (LengthSq(t1) < 1e-12) t1 = Cross(g.n, Vec3{0.0, 1.0, 0.0});
			t1 = Normalized(t1);
			const Vec3 t2 = Cross(g.n, t1);
			double a11 = 0.0, a12 = 0.0, a22 = 0.0, b1 = 0.0, b2 = 0.0;
			int used = 0;
			for (int o : nb) {
				const PanelGeom& go = p.geoms[o];
				if (go.degenerate || Dot(go.n, g.n) < cos_max) continue;
				const Vec3 dd = go.c - g.c;
				const double x = Dot(dd, t1), y = Dot(dd, t2);
				const double l2 = x * x + y * y;
				if (l2 <= 0.0) continue;
				const double w = 1.0 / std::sqrt(l2);
				const double dm = mu[p.first + o] - r.mu;
				a11 += w * x * x; a12 += w * x * y; a22 += w * y * y;
				b1 += w * x * dm; b2 += w * y * dm;
				++used;
			}
			// The 2 x 2 normal matrix is symmetric positive semi-definite; its
			// eigenvalues say how well the neighbours resolve the gradient. When
			// they lie nearly in a line (on a leading edge most neighbours face
			// away and are excluded above, leaving two or three along the span
			// -- my_drone's boom at its cowling, 2026-11-18: a determinant of
			// 1e-56 and a gradient of 1e28), only the component along their line
			// is known and only it is taken; the old relative determinant test
			// never fired.
			const double tr = a11 + a22, det = a11 * a22 - a12 * a12;
			const double disc = std::sqrt(std::max(0.0, 0.25 * tr * tr - det));
			const double l1 = 0.5 * tr + disc, l2 = 0.5 * tr - disc;
			Vec3 grad{0.0, 0.0, 0.0};
			if (used >= 2 && l1 > 0.0 && l2 > 1e-6 * l1) {
				const double gx = (b1 * a22 - b2 * a12) / det, gy = (a11 * b2 - a12 * b1) / det;
				grad = t1 * gx + t2 * gy;
			} else if (used >= 1 && l1 > 0.0) {
				double ex, ey;   // the eigenvector of l1
				if (std::fabs(a12) > 1e-300 * l1) { ex = l1 - a22; ey = a12; }
				else if (a11 >= a22) { ex = 1.0; ey = 0.0; }
				else { ex = 0.0; ey = 1.0; }
				const double el = std::hypot(ex, ey);
				ex /= el; ey /= el;
				const double along = (ex * b1 + ey * b2) / l1;
				grad = (t1 * ex + t2 * ey) * along;
				++p.result.rank1_gradients;
			}
			r.grad_mu = grad;
			r.velocity = free_stream - g.n * Dot(free_stream, g.n) + grad;
			// A surface speed beyond any potential flow about a closed body is
			// a fit gone wrong, not a flow: capped, counted and reported, so a
			// handful of panels cannot carry the force sum away.
			if (options.max_speed_ratio > 0.0) {
				const double vmax = options.max_speed_ratio * vinf;
				const double v2 = LengthSq(r.velocity);
				if (v2 > vmax * vmax) {
					r.velocity = r.velocity * (vmax / std::sqrt(v2));
					if (p.result.capped_panels == 0) p.result.first_capped_at = g.c;
					++p.result.capped_panels;
				}
			}
			r.cp = 1.0 - LengthSq(r.velocity) / (vinf * vinf);
			const Vec3 f = g.n * (-r.cp * g.area);
			F += f;
			M += Cross(g.c - ref, f);
			if (mirrored) {
				const Vec3 fi{f.x, -f.y, f.z};
				const Vec3 ci{g.c.x, 2.0 * y0 - g.c.y, g.c.z};
				F += fi;
				M += Cross(ci - ref, fi);
			}
		}
		p.result.force_coefficient = F * inv_s;
		p.result.moment_coefficient = M * inv_s;
		sol.force_coefficient += p.result.force_coefficient;
		sol.moment_coefficient += p.result.moment_coefficient;
		if (p.result.rank1_gradients > 0) {
			p.result.notes.push_back(p.result.name + ": " + std::to_string(p.result.rank1_gradients)
				+ " panel(s) have their neighbours in a line (a leading edge, a seam) and take only the gradient along it");
		}
		if (p.result.capped_panels > 0) {
			const Vec3& c = p.result.first_capped_at;
			p.result.notes.push_back(p.result.name + ": WARNING " + std::to_string(p.result.capped_panels) + " panel(s) had a surface speed beyond "
				+ std::to_string(options.max_speed_ratio) + " times the free stream, capped there (an ill-conditioned gradient fit, first at ("
				+ std::to_string(c.x) + ", " + std::to_string(c.y) + ", " + std::to_string(c.z) + ")): the pressures on and around them are not to be trusted");
		}
		for (const std::string& n : p.result.notes) sol.notes.push_back(n);
	}
	for (WakeStrip& w : sol.wakes) w.mu = mu[w.upper] - mu[w.lower];
	if (!sol.wakes.empty()) {
		Vec3 lhat;
		if (options.lift_direction) lhat = Normalized(*options.lift_direction);
		else {
			lhat = Vec3{0.0, 0.0, 1.0} - d * d.z;
			lhat = LengthSq(lhat) > 1e-24 ? Normalized(lhat) : Vec3{0.0, 0.0, 1.0};
		}
		double lift = 0.0;
		for (const WakeStrip& w : sol.wakes) {
			const Vec3 dl = w.corners[0] - w.corners[1]; // b - a: the strip's normal is d x dl
			lift += w.mu * Dot(Cross(d, dl), lhat);
			if (mirrored) {
				const Vec3 dli{-dl.x, dl.y, -dl.z}; // the image strip, oriented with its mirrored normal
				lift += w.mu * Dot(Cross(d, dli), lhat);
			}
		}
		sol.lift_from_wake = 2.0 / (vinf * std::max(options.reference_area, 1e-300)) * lift;
	}
	for (Prepared& p : prep) sol.bodies.push_back(std::move(p.result));
	return sol;
}

// ---------------------------------------------------------------------------
// Post-processing
// ---------------------------------------------------------------------------

SectionCut CutSection(const BodyResult& body, const Vec3& axis, double s, bool mirror_y, const std::vector<char>* mask) {
	SectionCut out;
	const TriMesh& mesh = body.mesh;
	auto pos = [&](int i) { Vec3 p = mesh.vertices[i]; if (mirror_y) p.y = -p.y; return p; };
	for (size_t t = 0; t < mesh.triangles.size(); ++t) {
		const PanelResult& r = body.panels[t];
		if (!(r.area > 0.0)) continue;
		if (mask != nullptr && t < mask->size() && !(*mask)[t]) continue;
		const auto& tr = mesh.triangles[t];
		Vec3 v[3] = {pos(tr[0]), pos(tr[1]), pos(tr[2])};
		double f[3];
		for (int k = 0; k < 3; ++k) f[k] = Dot(v[k], axis) - s;
		// A vertex ON the plane counts as on its positive side: an edge lying
		// in the plane is then found once, by the triangle below it, not by
		// both (a cut on a ring of the mesh -- the root of a wing built tip to
		// tip, a station of its grid -- doubled the load there, 2026-11-16),
		// and a triangle lying in the plane (a tip cap) is not a section.
		std::vector<Vec3> pts;
		for (int k = 0; k < 3; ++k) {
			const int k2 = (k + 1) % 3;
			if ((f[k] < 0.0) != (f[k2] < 0.0)) {
				const double w = f[k] / (f[k] - f[k2]);
				pts.push_back(v[k] + (v[k2] - v[k]) * w);
			}
		}
		if (pts.size() != 2) continue;
		Vec3 n = r.normal;
		if (mirror_y) n.y = -n.y;
		Vec3 tvec = pts[1] - pts[0];
		const double l = Length(tvec);
		if (l <= 0.0) continue;
		tvec = tvec * (1.0 / l);
		if (Dot(Cross(tvec, axis), n) < 0.0) tvec = -tvec;
		out.force_per_span += Cross(tvec, axis) * (-r.cp * l);
		out.length += l;
		out.points.push_back((pts[0] + pts[1]) * 0.5);
		out.cp.push_back(r.cp);
		out.found = true;
	}
	return out;
}

std::vector<Vec3> BinnedLoads(const Solution& sol, const Vec3& axis, const std::vector<double>& edges, bool mirror_y) {
	std::vector<Vec3> bins(edges.size() > 1 ? edges.size() - 1 : 0);
	if (bins.empty()) return bins;
	auto spread = [&](const Vec3& f, double lo, double hi) {
		if (hi <= lo) {
			auto it = std::upper_bound(edges.begin(), edges.end(), lo);
			const int b = static_cast<int>(it - edges.begin()) - 1;
			if (b >= 0 && b < static_cast<int>(bins.size())) bins[b] += f;
			return;
		}
		for (size_t b = 0; b < bins.size(); ++b) {
			const double o = std::min(hi, edges[b + 1]) - std::max(lo, edges[b]);
			if (o > 0.0) bins[b] += f * (o / (hi - lo));
		}
	};
	for (const BodyResult& body : sol.bodies) {
		for (size_t t = 0; t < body.panels.size(); ++t) {
			const PanelResult& r = body.panels[t];
			if (!(r.area > 0.0)) continue;
			const auto& tr = body.mesh.triangles[t];
			for (int side = 0; side < (mirror_y ? 2 : 1); ++side) {
				double lo = 1e300, hi = -1e300;
				for (int k = 0; k < 3; ++k) {
					Vec3 v = body.mesh.vertices[tr[k]];
					if (side == 1) v.y = -v.y;
					const double a = Dot(v, axis);
					lo = std::min(lo, a); hi = std::max(hi, a);
				}
				Vec3 n = r.normal;
				if (side == 1) n.y = -n.y;
				spread(n * (-r.cp * r.area), lo, hi);
			}
		}
	}
	return bins;
}

} // namespace aerotelic::tripanel
