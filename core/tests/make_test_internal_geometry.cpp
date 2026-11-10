// Writes ue/Projects/test_internal_geometry.yaml (2026-11-10): one plain
// trapezoidal wing with a forward and an aft spar and the default ribs,
// a 2 mm aluminium skin on its body so the structure generates on load --
// the simplest project that should show spars in the viewport.
//
//   make_test_internal_geometry <path.yaml>

#include <cmath>
#include <cstdio>
#include <string>

#include "aerotelic/airfoil.hpp"
#include "aerotelic/materials.hpp"
#include "aerotelic/project_yaml.hpp"
#include "aerotelic/wing_model.hpp"

using namespace aerotelic;
using namespace aerotelic::project_yaml;

int main(int argc, char** argv) {
	if (argc < 2) { std::printf("usage: make_test_internal_geometry <path.yaml>\n"); return 2; }

	PlanformSpec ps;
	ps.type = "standard";
	ps.semi_span = 5.0;
	ps.root_chord = 1.5;
	ps.tip_chord = 0.8;
	ps.sweep_angle = 5.0;
	ps.sweep_fraction = 0.25;
	ps.rotation_fraction = 0.25;
	WingModel wing(ps, "Wing");
	wing.AddSection(Airfoil::Naca4("2412"), 0.0);
	wing.AddSection(Airfoil::Naca4("2412"), 1.0);

	WingStructureSpec s;
	s.spars.clear();
	WingSparSpec forward;
	forward.name = "Forward spar";
	forward.x_root = forward.x_tip = 0.25;
	forward.thickness = 0.003;
	WingSparSpec aft;
	aft.name = "Aft spar";
	aft.x_root = aft.x_tip = 0.65;
	aft.thickness = 0.003;
	s.spars = {forward, aft};
	s.ribs.between = 6;
	s.ribs.thickness = 0.002;
	wing.structure = s;

	ProjectEntity e;
	e.name = "Wing";
	e.type = EntityType::Wing;
	e.mirror = false;
	e.model = wing.Serialize();
	ProjectBody body;
	body.name = "Main";
	if (const Material* al = FindMaterialPreset("Aluminium 2024-T3 sheet")) body.structure.material = *al;
	body.structure.skin_thickness = 0.002;
	e.bodies.push_back(body);

	ProjectFile pf;
	pf.entities.push_back(e);
	SaveProjectYaml(pf, argv[1]);
	std::printf("wrote %s\n", argv[1]);

	// The same wing SYMMETRIC with 5 degrees of dihedral (2026-11-10): one
	// body tip to tip through the derived section in y = 0, the spars
	// uninterrupted, a centre rib; written beside the first file as
	// test_symmetric_wing.yaml.
	{
		std::string path = argv[1];
		const size_t slash = path.find_last_of("/\\");
		path = (slash == std::string::npos ? std::string() : path.substr(0, slash + 1)) + "test_symmetric_wing.yaml";
		PlanformSpec sps = ps;
		sps.dihedral_curve = LinearCurveSpec(0.0, std::tan(5.0 * 3.14159265358979323846 / 180.0));
		WingModel sym(sps, "Wing");
		sym.AddSection(Airfoil::Naca4("2412"), 0.0);
		sym.AddSection(Airfoil::Naca4("2412"), 1.0);
		sym.structure = s;
		sym.symmetric = true;
		ProjectEntity se = e;
		se.model = sym.Serialize();
		ProjectFile spf;
		spf.entities.push_back(se);
		SaveProjectYaml(spf, path);
		std::printf("wrote %s\n", path.c_str());
		std::vector<NamedMesh> sb = sym.BuildMeshes(2e-3);
		const WingBodyStructurePlan& sp = sym.LastStructurePlans()[0];
		std::printf("symmetric: %zu body(ies), closed %d, plan: %zu ribs, %zu spar webs\n", sb.size(), sb[0].mesh.IsClosedManifold() ? 1 : 0, sp.ribs.size(), sp.spars.size());
		for (const std::string& n : sp.notes) std::printf("  note: %s\n", n.c_str());
	}

	// Read it back and build it, as the application will.
	ProjectFile back = LoadProjectYaml(argv[1]);
	const WingModelSpec& spec = std::get<WingModelSpec>(back.entities[0].model);
	WingModel again(spec);
	std::vector<NamedMesh> bodies = again.BuildMeshes(2e-3);
	std::printf("reloaded: %zu spars (%s, %s), %d ribs between; body '%s' skin %.4g of %s; built %zu body(ies), plan: %zu ribs, %zu spar webs\n",
		spec.structure.spars.size(), spec.structure.spars[0].name.c_str(), spec.structure.spars[1].name.c_str(), spec.structure.ribs.between,
		back.entities[0].bodies[0].name.c_str(), back.entities[0].bodies[0].structure.skin_thickness, back.entities[0].bodies[0].structure.material.name.c_str(),
		bodies.size(), again.LastStructurePlans()[0].ribs.size(), again.LastStructurePlans()[0].spars.size());
	for (const std::string& n : again.LastStructurePlans()[0].notes) std::printf("  note: %s\n", n.c_str());
	return 0;
}
