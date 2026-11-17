// span_load_optimizer: makes a configuration's lift distribution along the
// span as near an ellipse as its wings' chord and twist allow.
//
//   span_load_optimizer <project.yaml> [--wing <name>] [--tail <name>]
//                       [--alpha deg] [--budget seconds] [--out result.yaml]
//
// THE PIPELINE is the Panel Method dialog's: every entity's aero copy at the
// display tolerance (0.15% of its own diagonal), the bodies fused by the mesh
// boolean (a refused exact contact retried nudged in x and z), the triangle
// panel method with the symmetry plane on, and the "All bodies" view's 40
// bins of lift per unit span along y.
//
// THE VARIABLES (ten): the main wing's chord multipliers at eta 0, 0.5 and 1
// (its leading and trailing edges become a three-pole quadratic Bezier pair
// anchored on the baseline quarter-chord line, so the wing does not move) and
// its twist poles at the same stations; the tailplane's chord multipliers at
// root and tip and its twist at root and tip (its planform is converted to
// the same anchored nurbs form). Span, dihedral, sections, flaps, structure
// and placement are untouched. Bounds: chord 0.6 to 1.6 times the baseline,
// twist -8 to +8 degrees.
//
// THE OBJECTIVE: the relative RMS distance of the binned lift from the
// ellipse with the same integral (plus a small penalty for a variable pushed
// past its bound). It is a noisy black box -- a parameter change moves
// triangles and seams -- so the optimiser is Nelder & Mead's downhill
// simplex (Computer Journal 7, 1965): reflection 1, expansion 2, contraction
// 0.5, shrink 0.5, stopped when the simplex's spread of objective values is
// below 1e-5 after 50 iterations, or at the wall-clock budget. About 1.7 s
// an evaluation on my_drone; its 646 evaluations converged in 166 s and took
// the measure from 0.156 to 0.045 (2026-11-16).
//
// THE OUTPUT: the project with the optimised wings, written as a NEW file
// (--out; default <project>_elliptic.yaml beside the input), the tables of
// the baseline, the optimised and the elliptic loading on stdout. The input
// is never modified.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>

#include "aerotelic/mesh_boolean.hpp"
#include "aerotelic/project_yaml.hpp"
#include "aerotelic/tri_panel.hpp"
#include "aerotelic/wing_model.hpp"
#include "aerotelic/fuselage_model.hpp"
#include "aerotelic/nacelle_model.hpp"
#include "aerotelic/lofted_nacelle_model.hpp"
#include "aerotelic/primitive_model.hpp"

using namespace aerotelic;
using namespace aerotelic::tripanel;
using Clock = std::chrono::steady_clock;

namespace {
struct Placed {
	std::array<double, 9> R{1, 0, 0, 0, 1, 0, 0, 0, 1};
	Vec3 t{0, 0, 0};
	Vec3 Apply(const Vec3& p) const {
		return {R[0] * p.x + R[1] * p.y + R[2] * p.z + t.x, R[3] * p.x + R[4] * p.y + R[5] * p.z + t.y, R[6] * p.x + R[7] * p.y + R[8] * p.z + t.z};
	}
};
Placed FromPlacement(const aerotelic::Placement& p) {
	Placed out;
	out.t = {p.translation[0], p.translation[1], p.translation[2]};
	Vec3 k{p.rotation_axis[0], p.rotation_axis[1], p.rotation_axis[2]};
	const double len = Length(k);
	if (len < 1e-12 || std::fabs(p.rotation_deg) < 1e-12) return out;
	k = k * (1.0 / len);
	const double th = p.rotation_deg * 3.14159265358979323846 / 180.0, c = std::cos(th), s = std::sin(th);
	out.R = {c + (1 - c) * k.x * k.x, (1 - c) * k.x * k.y - s * k.z, (1 - c) * k.x * k.z + s * k.y,
		(1 - c) * k.y * k.x + s * k.z, c + (1 - c) * k.y * k.y, (1 - c) * k.y * k.z - s * k.x,
		(1 - c) * k.z * k.x - s * k.y, (1 - c) * k.z * k.y + s * k.x, c + (1 - c) * k.z * k.z};
	return out;
}
std::unique_ptr<ISolidModel> ModelOf(const project_yaml::ProjectEntity& e) {
	switch (e.type) {
		case project_yaml::EntityType::Wing: return std::make_unique<WingModel>(std::get<WingModelSpec>(e.model));
		case project_yaml::EntityType::Fuselage: return std::make_unique<FuselageModel>(std::get<FuselageSpec>(e.model));
		case project_yaml::EntityType::Nacelle: return std::make_unique<NacelleModel>(std::get<NacelleSpec>(e.model));
		case project_yaml::EntityType::LoftedNacelle: return std::make_unique<LoftedNacelleModel>(std::get<LoftedNacelleSpec>(e.model));
		case project_yaml::EntityType::Primitive: {
			const project_yaml::PrimitiveSpec& ps = std::get<project_yaml::PrimitiveSpec>(e.model);
			auto pm = std::make_unique<PrimitiveModel>();
			pm->shape = ps.shape; pm->name = ps.name; pm->radius = ps.radius; pm->stretch = ps.stretch; pm->size = ps.size; pm->cyl_radius = ps.cyl_radius; pm->cyl_length = ps.cyl_length;
			return pm;
		}
		default: return nullptr;
	}
}
double DiagOf(const TriMesh& m) {
	Vec3 lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
	for (const Vec3& v : m.vertices) { lo.x = std::min(lo.x, v.x); lo.y = std::min(lo.y, v.y); lo.z = std::min(lo.z, v.z); hi.x = std::max(hi.x, v.x); hi.y = std::max(hi.y, v.y); hi.z = std::max(hi.z, v.z); }
	return Distance(lo, hi);
}
double ToleranceOf(ISolidModel& model) {
	std::vector<NamedMesh> first = model.BuildMeshes(0.004);
	double diag = 0.0;
	for (const NamedMesh& nm : first) diag = std::max(diag, DiagOf(nm.mesh));
	return std::max(diag * 0.0015, 1e-4);
}
TriMesh FuseOrNudge(const TriMesh& a, const TriMesh& b, double diag) {
	try {
		return MeshBoolean(a, b, MeshBooleanOp::Fuse);
	} catch (const MeshBooleanError&) {
		const double eps = 1e-6 * diag;
		TriMesh nudged = b;
		nudged.Translate({eps * 0.7071067812, 0.0, eps * 0.7071067812});
		return MeshBoolean(a, nudged, MeshBooleanOp::Fuse);
	}
}

// A wing's planform anchors: the quarter-chord x and the chord at eta (the
// baseline, twist zero), so new chords keep the quarter-chord line.
struct Anchors { std::vector<double> eta, xqc, chord; };
Anchors AnchorsOf(const WingModel& w, const std::vector<double>& etas) {
	Anchors a;
	for (double eta : etas) {
		Mat q = w.planform().AirfoilCoordsToXyz(eta, Mat::FromRows({{0.25, 0.0}}));
		a.eta.push_back(eta);
		a.xqc.push_back(q(0, 0));
		a.chord.push_back(w.planform().ChordAt(eta));
	}
	return a;
}
NurbsSpec CurveOf(const std::vector<double>& etas, const std::vector<double>& values) {
	NurbsSpec s;
	const int n = (int)etas.size();
	s.degree = n - 1; // 2 poles: linear; 3 poles: a quadratic Bezier
	std::vector<std::vector<double>> rows;
	for (int i = 0; i < n; ++i) rows.push_back({etas[i], values[i]});
	s.poles = Mat::FromRows(rows);
	s.weights = std::vector<double>(n, 1.0);
	s.interior_knots = std::vector<double>{};
	return s;
}
// Writes the chord multipliers and twists (degrees) into a wing spec as a
// nurbs planform anchored on the baseline quarter-chord line.
void ApplyDesign(WingModelSpec& spec, const Anchors& a, const std::vector<double>& mult, const std::vector<double>& twist_deg) {
	PlanformSpec& p = spec.planform;
	const double semi = p.semi_span, rot = p.rotation_fraction;
	NurbsSpec dihedral = p.dihedral_curve;
	std::vector<double> xle, xte;
	for (size_t i = 0; i < a.eta.size(); ++i) {
		const double c = a.chord[i] * mult[i];
		xle.push_back(a.xqc[i] - 0.25 * c);
		xte.push_back(a.xqc[i] + 0.75 * c);
	}
	p.type = "nurbs";
	p.semi_span = semi;
	p.rotation_fraction = rot;
	p.dihedral_curve = dihedral;
	p.le_curve = CurveOf(a.eta, xle);
	p.te_curve = CurveOf(a.eta, xte);
	p.twist_curve = CurveOf(a.eta, twist_deg);
}

struct Eval {
	double J = 1e3, lift = 0.0, cl = 0.0;
	std::vector<double> bins;
	int triangles = 0;
	std::string error;
};
} // namespace

int main(int argc, char** argv) {
	setvbuf(stdout, nullptr, _IONBF, 0);
	const auto t_start = Clock::now();
	auto elapsed = [&]() { return std::chrono::duration<double>(Clock::now() - t_start).count(); };
	try {
		std::string path, out_path, wing_name = "Wing", tail_name = "HT";
		double budget_s = 570.0, alpha_deg = 4.0;
		for (int i = 1; i < argc; ++i) {
			const std::string a = argv[i];
			if (a == "--out" && i + 1 < argc) out_path = argv[++i];
			else if (a == "--budget" && i + 1 < argc) budget_s = std::atof(argv[++i]);
			else if (a == "--alpha" && i + 1 < argc) alpha_deg = std::atof(argv[++i]);
			else if (a == "--wing" && i + 1 < argc) wing_name = argv[++i];
			else if (a == "--tail" && i + 1 < argc) tail_name = argv[++i];
			else path = a;
		}
		if (path.empty()) { std::printf("usage: span_load_optimizer <project.yaml> [--wing <name>] [--tail <name>] [--alpha deg] [--budget seconds] [--out result.yaml]\n"); return 2; }
		if (out_path.empty()) { const size_t dot = path.rfind('.'); out_path = (dot == std::string::npos ? path : path.substr(0, dot)) + "_elliptic.yaml"; }
		project_yaml::ProjectFile pf = project_yaml::LoadProjectYaml(path);
		const double alpha = alpha_deg * 3.14159265358979323846 / 180.0;
		const Vec3 vinf{std::cos(alpha), 0.0, std::sin(alpha)};
		const Vec3 lift_dir{-std::sin(alpha), 0.0, std::cos(alpha)};

		// The fixed bodies fused once; the two wings by index.
		int i_wing = -1, i_ht = -1;
		TriMesh fixed;
		bool have_fixed = false;
		double all_diag = 0.0;
		{
			TriMesh all;
			for (const project_yaml::ProjectEntity& e : pf.entities) {
				std::unique_ptr<ISolidModel> m = ModelOf(e);
				if (!m) continue;
				const Placed P = FromPlacement(e.placement);
				for (const NamedMesh& nm : m->BuildMeshes(0.004)) for (const Vec3& v : nm.mesh.vertices) all.vertices.push_back(P.Apply(v));
			}
			all_diag = DiagOf(all);
		}
		for (size_t i = 0; i < pf.entities.size(); ++i) {
			const project_yaml::ProjectEntity& e = pf.entities[i];
			if (e.name == wing_name) { i_wing = (int)i; continue; }
			if (e.name == tail_name) { i_ht = (int)i; continue; }
			std::unique_ptr<ISolidModel> m = ModelOf(e);
			if (!m) continue;
			const Placed P = FromPlacement(e.placement);
			const double tol = ToleranceOf(*m);
			TriMesh mesh;
			if (e.type == project_yaml::EntityType::Wing) mesh = static_cast<WingModel*>(m.get())->BuildAeroMesh(tol).mesh;
			else mesh = m->BuildMeshes(tol)[0].mesh;
			for (Vec3& v : mesh.vertices) v = P.Apply(v);
			if (!have_fixed) { fixed = mesh; have_fixed = true; }
			else fixed = FuseOrNudge(fixed, mesh, all_diag);
			std::printf("fixed body %s: %zu triangles (fused so far %zu)\n", e.name.c_str(), mesh.triangles.size(), fixed.triangles.size());
		}
		if (i_wing < 0 || i_ht < 0) throw std::runtime_error("the wing '" + wing_name + "' or the tailplane '" + tail_name + "' is not in the project (--wing, --tail)");
		const WingModelSpec base_wing = std::get<WingModelSpec>(pf.entities[i_wing].model);
		const WingModelSpec base_ht = std::get<WingModelSpec>(pf.entities[i_ht].model);
		const Placed P_wing = FromPlacement(pf.entities[i_wing].placement), P_ht = FromPlacement(pf.entities[i_ht].placement);
		const Anchors a_wing = AnchorsOf(WingModel(base_wing), {0.0, 0.5, 1.0});
		const Anchors a_ht = AnchorsOf(WingModel(base_ht), {0.0, 1.0});
		double tol_wing, tol_ht, ref_area, ref_len;
		{
			WingModel w(base_wing), h(base_ht);
			tol_wing = ToleranceOf(w); tol_ht = ToleranceOf(h);
			ref_area = 2.0 * w.planform().PlanformArea() + 2.0 * h.planform().PlanformArea();
			ref_len = w.planform().ChordAt(0.0);
		}
		std::printf("anchors Wing: xqc %.4f %.4f %.4f chord %.4f %.4f %.4f; HT: xqc %.4f %.4f chord %.4f %.4f; tol %.4f / %.4f\n",
			a_wing.xqc[0], a_wing.xqc[1], a_wing.xqc[2], a_wing.chord[0], a_wing.chord[1], a_wing.chord[2], a_ht.xqc[0], a_ht.xqc[1], a_ht.chord[0], a_ht.chord[1], tol_wing, tol_ht);

		// Design vector: Wing chord multipliers at eta 0, 0.5, 1; Wing twist at
		// 0, 0.5, 1 (deg); HT chord multipliers at 0, 1; HT twist at 0, 1.
		const int NV = 10;
		const double lo_b[NV] = {0.6, 0.6, 0.6, -8, -8, -8, 0.6, 0.6, -8, -8};
		const double hi_b[NV] = {1.6, 1.6, 1.6, 8, 8, 8, 1.6, 1.6, 8, 8};
		const int NB = 40;
		const double y_lo = -1.5, y_hi = 1.5;
		int n_eval = 0;
		auto evaluate = [&](const std::vector<double>& xin) {
			Eval ev;
			std::vector<double> x = xin;
			double viol = 0.0;
			for (int i = 0; i < NV; ++i) { const double c = std::clamp(x[i], lo_b[i], hi_b[i]); viol += std::fabs(c - x[i]); x[i] = c; }
			++n_eval;
			try {
				WingModelSpec ws = base_wing, hs = base_ht;
				ApplyDesign(ws, a_wing, {x[0], x[1], x[2]}, {x[3], x[4], x[5]});
				ApplyDesign(hs, a_ht, {x[6], x[7]}, {x[8], x[9]});
				WingModel w(ws), h(hs);
				std::vector<std::vector<Vec3>> te_w, te_h;
				NamedMesh aw = w.BuildAeroMesh(tol_wing, &te_w), ah = h.BuildAeroMesh(tol_ht, &te_h);
				for (Vec3& v : aw.mesh.vertices) v = P_wing.Apply(v);
				for (Vec3& v : ah.mesh.vertices) v = P_ht.Apply(v);
				for (auto& l : te_w) for (Vec3& v : l) v = P_wing.Apply(v);
				for (auto& l : te_h) for (Vec3& v : l) v = P_ht.Apply(v);
				TriMesh fused = FuseOrNudge(fixed, aw.mesh, all_diag);
				fused = FuseOrNudge(fused, ah.mesh, all_diag);
				Body b;
				b.name = "BoolOp 1";
				b.mesh = fused;
				for (auto& l : te_w) b.trailing_edges.push_back(l);
				for (auto& l : te_h) b.trailing_edges.push_back(l);
				b.trailing_edge_tolerance = 1e-6 * all_diag;
				Options opt;
				opt.symmetry_y = 0.0;
				opt.reference_area = ref_area;
				opt.wake_length = 20.0 * ref_len;
				Solution s = Solve({b}, vinf, opt);
				std::vector<double> edges;
				for (int k = 0; k <= NB; ++k) edges.push_back(y_lo + (y_hi - y_lo) * k / NB);
				std::vector<Vec3> bins = BinnedLoads(s, {0.0, 1.0, 0.0}, edges, true);
				const double dy = (y_hi - y_lo) / NB;
				double L = 0.0;
				for (int k = 0; k < NB; ++k) { ev.bins.push_back(Dot(bins[k], lift_dir) / dy); L += ev.bins.back() * dy; }
				ev.lift = L;
				ev.cl = L / ref_area;
				ev.triangles = (int)fused.triangles.size();
				// Relative RMS distance to the ellipse with the same integral.
				const double bspan = y_hi - y_lo;
				double sq = 0.0;
				for (int k = 0; k < NB; ++k) {
					const double yc = y_lo + (k + 0.5) * dy, xx = 2.0 * yc / bspan;
					const double ell = 4.0 * L / (3.14159265358979323846 * bspan) * std::sqrt(std::max(0.0, 1.0 - xx * xx));
					sq += (ev.bins[k] - ell) * (ev.bins[k] - ell);
				}
				ev.J = L > 1e-6 ? std::sqrt(sq / NB) / (L / bspan) + 0.5 * viol : 1e3;
			} catch (const std::exception& ex) {
				ev.error = ex.what();
				ev.J = 1e3;
			}
			return ev;
		};

		std::vector<double> x0 = {1, 1, 1, 0, 0, 0, 1, 1, 0, 0};
		const Eval base = evaluate(x0);
		if (!base.error.empty()) throw std::runtime_error("baseline failed: " + base.error);
		std::printf("baseline: J %.4f (relative RMS from the ellipse), L/q %.4f, C_L %.4f, %d triangles, %.1f s per evaluation\n", base.J, base.lift, base.cl, base.triangles, elapsed());

		// Nelder-Mead.
		const double step[NV] = {0.15, 0.15, 0.15, 2.0, 2.0, 2.0, 0.15, 0.15, 2.0, 2.0};
		std::vector<std::vector<double>> X(NV + 1, x0);
		std::vector<double> F(NV + 1);
		for (int i = 1; i <= NV; ++i) X[i][i - 1] += step[i - 1];
		for (int i = 0; i <= NV; ++i) F[i] = evaluate(X[i]).J;
		double best_logged = base.J;
		int iter = 0;
		auto order = [&]() {
			std::vector<int> idx(NV + 1);
			for (int i = 0; i <= NV; ++i) idx[i] = i;
			std::sort(idx.begin(), idx.end(), [&](int a, int b) { return F[a] < F[b]; });
			std::vector<std::vector<double>> X2; std::vector<double> F2;
			for (int i : idx) { X2.push_back(X[i]); F2.push_back(F[i]); }
			X = X2; F = F2;
		};
		order();
		while (elapsed() < budget_s) {
			++iter;
			std::vector<double> cen(NV, 0.0);
			for (int i = 0; i < NV; ++i) for (int j = 0; j < NV; ++j) cen[j] += X[i][j] / NV;
			auto lin = [&](const std::vector<double>& a, const std::vector<double>& b, double t) { std::vector<double> r(NV); for (int j = 0; j < NV; ++j) r[j] = a[j] + t * (b[j] - a[j]); return r; };
			std::vector<double> xr = lin(cen, X[NV], -1.0);
			double fr = evaluate(xr).J;
			if (fr < F[0]) {
				std::vector<double> xe = lin(cen, X[NV], -2.0);
				double fe = evaluate(xe).J;
				if (fe < fr) { X[NV] = xe; F[NV] = fe; } else { X[NV] = xr; F[NV] = fr; }
			} else if (fr < F[NV - 1]) {
				X[NV] = xr; F[NV] = fr;
			} else {
				std::vector<double> xc = fr < F[NV] ? lin(cen, X[NV], -0.5) : lin(cen, X[NV], 0.5);
				double fc = evaluate(xc).J;
				if (fc < std::min(fr, F[NV])) { X[NV] = xc; F[NV] = fc; }
				else {
					for (int i = 1; i <= NV; ++i) { X[i] = lin(X[0], X[i], 0.5); F[i] = evaluate(X[i]).J; }
				}
			}
			order();
			if (F[0] < best_logged - 1e-4) {
				best_logged = F[0];
				std::printf("  iter %d, eval %d, %.0f s: J %.4f  wing c x(%.3f %.3f %.3f) tw(%.2f %.2f %.2f)  ht c x(%.3f %.3f) tw(%.2f %.2f)\n", iter, n_eval, elapsed(), F[0],
					X[0][0], X[0][1], X[0][2], X[0][3], X[0][4], X[0][5], X[0][6], X[0][7], X[0][8], X[0][9]);
			}
			double spread = 0.0;
			for (int i = 1; i <= NV; ++i) spread = std::max(spread, std::fabs(F[i] - F[0]));
			if (spread < 1e-5 && iter > 50) { std::printf("converged (simplex spread %.1e)\n", spread); break; }
		}
		const Eval best = evaluate(X[0]);
		std::printf("\nRESULT after %d iterations, %d evaluations, %.0f s: J %.4f -> %.4f; L/q %.4f -> %.4f; C_L %.4f -> %.4f\n", iter, n_eval, elapsed(), base.J, best.J, base.lift, best.lift, base.cl, best.cl);
		std::printf("Wing chord multipliers (root, mid pole, tip): %.4f %.4f %.4f -> chords %.4f %.4f %.4f m; twist (root, mid pole, tip) %.3f %.3f %.3f deg\n",
			X[0][0], X[0][1], X[0][2], a_wing.chord[0] * X[0][0], a_wing.chord[1] * X[0][1], a_wing.chord[2] * X[0][2], X[0][3], X[0][4], X[0][5]);
		std::printf("HT chord multipliers (root, tip): %.4f %.4f -> chords %.4f %.4f m; twist (root, tip) %.3f %.3f deg\n", X[0][6], X[0][7], a_ht.chord[0] * X[0][6], a_ht.chord[1] * X[0][7], X[0][8], X[0][9]);
		std::printf("\n   2y/b     baseline   optimised    ellipse(opt)\n");
		const double dy = (y_hi - y_lo) / NB, bspan = y_hi - y_lo;
		for (int k = 0; k < NB; ++k) {
			const double yc = y_lo + (k + 0.5) * dy, xx = 2.0 * yc / bspan;
			const double ell = 4.0 * best.lift / (3.14159265358979323846 * bspan) * std::sqrt(std::max(0.0, 1.0 - xx * xx));
			std::printf("  %6.3f   %9.5f   %9.5f   %9.5f\n", xx, base.bins[k], best.bins[k], ell);
		}
		// The project with the optimised wings, as a new file.
		{
			WingModelSpec ws = base_wing, hs = base_ht;
			ApplyDesign(ws, a_wing, {X[0][0], X[0][1], X[0][2]}, {X[0][3], X[0][4], X[0][5]});
			ApplyDesign(hs, a_ht, {X[0][6], X[0][7]}, {X[0][8], X[0][9]});
			pf.entities[i_wing].model = ws;
			pf.entities[i_ht].model = hs;
			project_yaml::SaveProjectYaml(pf, out_path);
			std::printf("\nwritten %s\n", out_path.c_str());
		}
	} catch (const std::exception& e) {
		std::printf("EXCEPTION: %s\n", e.what());
		return 1;
	}
	return 0;
}
