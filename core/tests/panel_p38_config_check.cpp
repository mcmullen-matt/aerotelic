// The whole P-38 of ue/Projects/p38.yaml -- the centre fuselage on the
// plane (cut in half), the wing, a boom and a vertical tail (both
// mirrored, so imaged) -- paneled and placed as the High Order Panel
// dialog does it, solved with the image at 4 degrees: every body alone and
// all together must give finite pressures and a lifting wing. Found
// 2026-10-27 as a NaN solution in the dialog's smoke run of this project
// after the wing paneling changes (see panel_p38_check.cpp).

#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>

#include "aerotelic/panel_loads.hpp"
#include "aerotelic/project_yaml.hpp"
#include "aerotelic/wing_panels.hpp"

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

// A placement as the entity actor applies it: a rotation about an axis
// through the origin, then a translation (AAerotelicEntityActor::ApplyPlacement).
struct Placed {
	std::array<double, 9> R{1, 0, 0, 0, 1, 0, 0, 0, 1};
	Vec3 t{0, 0, 0};
	Vec3 Apply(const Vec3& p) const {
		return {R[0] * p.x + R[1] * p.y + R[2] * p.z + t.x, R[3] * p.x + R[4] * p.y + R[5] * p.z + t.y, R[6] * p.x + R[7] * p.y + R[8] * p.z + t.z};
	}
	Vec3 InverseDir(const Vec3& d) const { // R^T d
		return {R[0] * d.x + R[3] * d.y + R[6] * d.z, R[1] * d.x + R[4] * d.y + R[7] * d.z, R[2] * d.x + R[5] * d.y + R[8] * d.z};
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
	// Rodrigues: R = I c + s [k]x + (1 - c) k k^T
	out.R = {
		c + (1 - c) * k.x * k.x, (1 - c) * k.x * k.y - s * k.z, (1 - c) * k.x * k.z + s * k.y,
		(1 - c) * k.y * k.x + s * k.z, c + (1 - c) * k.y * k.y, (1 - c) * k.y * k.z - s * k.x,
		(1 - c) * k.z * k.x - s * k.y, (1 - c) * k.z * k.y + s * k.x, c + (1 - c) * k.z * k.z,
	};
	return out;
}

// A model from a project entity's spec, as the document subsystem builds it
// (null for a propeller, which is not paneled).
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

// The entity's bodies as placed triangle meshes (and the mirror's), for
// the buried-trailing-edge test of the others.
std::vector<TriMesh> PlacedMeshes(const project_yaml::ProjectEntity& e) {
	std::vector<TriMesh> out;
	std::unique_ptr<ISolidModel> model = ModelOf(e);
	if (!model) return out;
	const Placed P = FromPlacement(e.placement);
	for (NamedMesh& nm : model->BuildMeshes(0.01)) {
		for (Vec3& v : nm.mesh.vertices) v = P.Apply(v);
		if (e.mirror) {
			TriMesh m = nm.mesh;
			m.Mirror(Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0});
			out.push_back(std::move(m));
		}
		out.push_back(std::move(nm.mesh));
	}
	return out;
}

} // namespace
#include "aerotelic/sdf.hpp"
#include <cstdlib>
namespace {

std::vector<Network> EntityNetworks(const project_yaml::ProjectEntity& e, const Vec3& stream, bool symmetry, const std::vector<const TriMesh*>& others, std::vector<std::string>* notes) {
	const Placed P = FromPlacement(e.placement);
	std::vector<Network> own;
	if (e.type == project_yaml::EntityType::Wing) {
		WingModel w(std::get<WingModelSpec>(e.model));
		const Vec3 stream_model = P.InverseDir(stream);
		WingPaneling pan;
		pan.chordwise = 12;
		pan.spanwise = 10;
		pan.wake_length = 20.0 * w.planform().RootChord();
		pan.wake_direction = stream_model;
		own = WingNetworks(w, pan, stream_model);
		for (Network& n : own) {
			n.name = e.name + (n.name == w.name ? "" : " wake");
			for (Vec3& c : n.corners) c = P.Apply(c);
			if (n.prescribed_source) n.prescribed_source = [stream](const Vec3&, const Vec3& nrm) { return -Dot(stream, nrm); };
		}
	} else {
		// As the document subsystem builds a model from its spec.
		std::unique_ptr<ISolidModel> model;
		switch (e.type) {
			case project_yaml::EntityType::Fuselage: model = std::make_unique<FuselageModel>(std::get<FuselageSpec>(e.model)); break;
			case project_yaml::EntityType::Nacelle: model = std::make_unique<NacelleModel>(std::get<NacelleSpec>(e.model)); break;
			case project_yaml::EntityType::LoftedNacelle: model = std::make_unique<LoftedNacelleModel>(std::get<LoftedNacelleSpec>(e.model)); break;
			case project_yaml::EntityType::Primitive: {
				const project_yaml::PrimitiveSpec& ps = std::get<project_yaml::PrimitiveSpec>(e.model);
				auto pm = std::make_unique<PrimitiveModel>();
				pm->shape = ps.shape; pm->name = ps.name; pm->radius = ps.radius; pm->stretch = ps.stretch; pm->size = ps.size; pm->cyl_radius = ps.cyl_radius; pm->cyl_length = ps.cyl_length;
				model = std::move(pm);
				break;
			}
			default: return own; // a propeller: not paneled by the dialog either
		}
		std::vector<NamedLoft> lofts = model->BuildLofts();
		BodyPaneling bp;
		bp.along = 16;
		bp.around = 12;
		bp.wake_direction = stream;
		bp.other_bodies = others;
		bp.notes = notes;
		if (std::getenv("AEROTELIC_P38_NO_LOFT_WAKES")) bp.wake = false;
		if (symmetry && !e.mirror) bp.half_plane_y = 0.0;
		// Diagnostic: where a tagged loft's trailing-edge ring sits against the
		// other bodies (the buried test's inputs).
		for (const NamedLoft& l : lofts) {
			if (!l.trailing_edge.IsSet() || l.trailing_edge.kind != LoftTrailingEdge::Kind::UStation || !std::isfinite(l.source.mirror_u)) continue;
			std::vector<double> vs;
			for (int k = 0; k <= 16; ++k) vs.push_back(k / 16.0);
			Mat a = l.source.section(l.trailing_edge.u, vs), b = l.source.section(l.source.mirror_u - l.trailing_edge.u, vs);
			double closest = 1e300;
			int inside = 0;
			Vec3 first;
			for (int k = 0; k < a.rows(); ++k) {
				Vec3 p{0.5 * (a(k, 0) + b(k, 0)), 0.5 * (a(k, 1) + b(k, 1)), 0.5 * (a(k, 2) + b(k, 2))};
				p = P.Apply(p);
				if (k == 0) first = p;
				double d = 1e300;
				for (const TriMesh* m : others) { if (m == nullptr || m->triangles.empty()) continue; MeshSdf f(*m); d = std::min(d, f.Distance(p + stream * 0.02)); }
				closest = std::min(closest, d);
				if (d < 0.0) ++inside;
			}
			std::printf("  %s: trailing-edge ring at x %.3f (first point %.3f, %.3f, %.3f), closest other body %.4f, %d of %d probes inside one\n", e.name.c_str(), first.x, first.x, first.y, first.z, closest, inside, (int)a.rows());
		}
		for (const NamedLoft& l : lofts) {
			NamedLoft placed = l;
			std::function<Mat(double, const std::vector<double>&)> section = l.source.section;
			placed.source.section = [section, P](double u, const std::vector<double>& v) {
				Mat pts = section(u, v);
				for (int i = 0; i < pts.rows(); ++i) {
					Vec3 q = P.Apply({pts(i, 0), pts(i, 1), pts(i, 2)});
					pts(i, 0) = q.x; pts(i, 1) = q.y; pts(i, 2) = q.z;
				}
				return pts;
			};
			for (Network& n : LoftNetworks(placed, lofts.size() == 1 ? e.name : e.name + "/" + l.name, bp, stream)) own.push_back(std::move(n));
		}
	}
	return own;
}

bool Finite(const Solution& s) {
	for (const NetworkResult& nr : s.networks) for (const PanelResult& p : nr.panels) if (!std::isfinite(p.cp_upper) || !std::isfinite(p.mu)) return false;
	return std::isfinite(s.force_coefficient.x) && std::isfinite(s.force_coefficient.z);
}

} // namespace

int main(int argc, char** argv) {
	setvbuf(stdout, nullptr, _IONBF, 0);
	try {
		std::vector<std::string> candidates;
		if (argc > 1) candidates.push_back(argv[1]);
		for (const char* rel : {"ue/Projects/p38.yaml", "../ue/Projects/p38.yaml", "../../ue/Projects/p38.yaml", "../../../ue/Projects/p38.yaml"}) candidates.push_back(rel);
		std::string path;
		for (const std::string& c : candidates) if (std::ifstream(c).good()) { path = c; break; }
		if (path.empty()) throw std::runtime_error("ue/Projects/p38.yaml not found from the working directory (pass its path as the first argument)");
		project_yaml::ProjectFile pf = project_yaml::LoadProjectYaml(path);
		Check(pf.entities.size() == 8, F("the P-38 project has %.0f entities (8 expected)", static_cast<double>(pf.entities.size())));

		const double alpha = 4.0 * 3.14159265358979323846 / 180.0;
		const Vec3 vinf{std::cos(alpha), 0.0, std::sin(alpha)};
		const Vec3 lift{-std::sin(alpha), 0.0, std::cos(alpha)};
		Options opt;
		opt.symmetry_y = 0.0;
		opt.reference_area = 30.75;

		// Every entity's placed meshes, for the buried-trailing-edge test:
		// the engine nacelle's rim meets the boom behind it and sheds no wake.
		std::vector<std::vector<TriMesh>> meshes;
		for (const project_yaml::ProjectEntity& e : pf.entities) meshes.push_back(PlacedMeshes(e));
		std::vector<std::string> notes;
		// Each entity alone.
		std::vector<Network> all;
		for (size_t ei = 0; ei < pf.entities.size(); ++ei) {
			const project_yaml::ProjectEntity& e = pf.entities[ei];
			std::vector<const TriMesh*> others;
			for (size_t k = 0; k < meshes.size(); ++k) if (k != ei) for (const TriMesh& m : meshes[k]) others.push_back(&m);
			std::vector<Network> nets = EntityNetworks(e, vinf, true, others, &notes);
			if (nets.empty()) { std::printf("%s: not paneled\n", e.name.c_str()); continue; }
			int corners = 0;
			for (const Network& n : nets) corners += n.rows * n.cols;
			Solution s = Solve(nets, vinf, opt);
			const bool finite = Finite(s);
			double cpmin = 1.0, cpmax = -1e9;
			if (finite) for (const NetworkResult& nr : s.networks) { if (nr.type != NetworkType::DoubletAnalysis && nr.type != NetworkType::SourceAnalysis) continue; for (const PanelResult& p : nr.panels) { cpmin = std::min(cpmin, p.cp_upper); cpmax = std::max(cpmax, p.cp_upper); } }
			std::printf("%s alone: %zu networks, %d corners, %d unknowns, pivot %.2e, C_L %.4f, Cp %.3f .. %.3f\n", e.name.c_str(), nets.size(), corners, s.unknowns, s.pivot_ratio, Dot(s.force_coefficient, lift), cpmin, cpmax);
			Check(finite, e.name + ": alone, the solution is finite");
			if (finite) Check(cpmin > -10.0 && cpmax < 1.05, e.name + F(": alone, Cp within -10 .. 1.05 (%.3f .. %.3f)", cpmin, cpmax));
			for (Network& n : nets) all.push_back(std::move(n));
		}
		for (const std::string& n : notes) std::printf("note: %s\n", n.c_str());
		// All together.
		Solution s = Solve(all, vinf, opt);
		std::printf("all together: %d unknowns, pivot %.2e, C_L %.4f, wake %.4f, %.2f s\n", s.unknowns, s.pivot_ratio, Dot(s.force_coefficient, lift), s.lift_from_wake.value_or(0.0), s.seconds_influence + s.seconds_solve);
		Check(Finite(s), "all together: the solution is finite");
		if (Finite(s)) {
			Check(Dot(s.force_coefficient, lift) > 0.2 && Dot(s.force_coefficient, lift) < 0.7, F("all together: C_L %.4f plausible", Dot(s.force_coefficient, lift)));
			for (const NetworkResult& nr : s.networks) {
				if (nr.type != NetworkType::DoubletAnalysis && nr.type != NetworkType::SourceAnalysis) continue;
				double cpmin = 1.0;
				for (const PanelResult& p : nr.panels) cpmin = std::min(cpmin, p.cp_upper);
				std::printf("   %s: Cp min %.3f, force (%.4f, %.4f, %.4f)\n", nr.name.c_str(), cpmin, nr.force_coefficient.x, nr.force_coefficient.y, nr.force_coefficient.z);
			}
		}
	} catch (const std::exception& e) {
		Check(false, std::string("exception: ") + e.what());
	}
	std::printf("RESULT: %d passed, %d failed\n", g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}
