// Writes ue/Projects/test_internal_geometry.yaml (2026-11-10): one plain
// trapezoidal wing with a forward and an aft spar and the default ribs,
// a 2 mm aluminium skin on its body so the structure generates on load --
// the simplest project that should show spars in the viewport.
//
//   make_test_internal_geometry <path.yaml>

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
