// Checks the YAML project-file layer two ways:
//  1. Self-contained round-trips (build a model, serialize, write+read the
//     YAML text, reconstruct, confirm it rebuilds to the same geometry) --
//     covers all six entity types, including the two (lofted_nacelle,
//     primitive) none of the repo's sample projects happen to use.
//  2. Loading a REAL Python-saved project file end to end
//     (aerotelic/projects/p38.yaml), to confirm actual file compatibility,
//     not just that this port's own round-trip is self-consistent.

#include <fstream>
#include <iostream>
#include <string>

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include "aerotelic/project_yaml.hpp"

using namespace aerotelic;
using namespace aerotelic::project_yaml;

namespace {

int g_pass = 0, g_fail = 0;

void Check(bool cond, const std::string& msg) {
	if (cond) { g_pass++; return; }
	g_fail++;
	std::cout << "FAIL: " << msg << "\n";
}

void CheckClose(double a, double b, double tol, const std::string& msg) {
	Check(std::abs(a - b) <= tol, msg + " (" + std::to_string(a) + " vs " + std::to_string(b) + ")");
}

double Volume(const TopoDS_Shape& s) {
	GProp_GProps props;
	BRepGProp::VolumeProperties(s, props);
	return props.Mass();
}

// Round-trips `entity` through YAML text (not just the in-memory Node
// tree) so a real parse/emit cycle is exercised, not just the struct
// mapping.
ProjectEntity RoundTripThroughText(const ProjectEntity& entity) {
	ProjectFile pf;
	pf.entities.push_back(entity);
	// Scratch file next to the fixtures (the tests directory is always
	// writable in a checkout; the build directory's location is not fixed).
	const std::string path = AEROTELIC_TESTS_DIR "/project_yaml_check_tmp.yaml";
	SaveProjectYaml(pf, path);
	ProjectFile reloaded = LoadProjectYaml(path);
	if (reloaded.entities.size() != 1) throw std::runtime_error("round trip did not produce exactly one entity");
	return reloaded.entities[0];
}

void CheckWingRoundTrip() {
	PlanformSpec pspec;
	pspec.type = "double_trap";
	pspec.semi_span = 5.0;
	pspec.kink_fraction = 0.35;
	WingModelSpec wspec;
	wspec.name = "TestWing";
	wspec.planform = pspec;
	wspec.sections = {{0.0, Airfoil::Naca4("2412")->Serialize(), "NACA 2412"}, {1.0, Airfoil::Naca4("0012")->Serialize(), "NACA 0012"}};

	WingModel original(wspec);

	ProjectEntity e;
	e.name = "Wing1";
	e.type = EntityType::Wing;
	e.placement.translation = {1.0, 2.0, 3.0};
	e.model = original.Serialize(); // the materialized model, not the raw input spec -- see CheckFuselageRoundTrip

	ProjectEntity back = RoundTripThroughText(e);
	Check(back.type == EntityType::Wing, "wing round trip: type");
	CheckClose(back.placement.translation[1], 2.0, 1e-9, "wing round trip: placement");
	auto& ws = std::get<WingModelSpec>(back.model);
	Check(ws.sections.size() == 2, "wing round trip: section count");

	WingModel rebuilt(ws);
	CheckClose(Volume(original.BuildSolid(30, true, std::nullopt)), Volume(rebuilt.BuildSolid(30, true, std::nullopt)), 1e-6, "wing round trip: volume");
}

void CheckFuselageRoundTrip() {
	// Serializing an already-constructed model (not the raw input spec) --
	// a fresh FuselageSpec deliberately leaves interior_knots unset to mean
	// "the constructor computes it," which a real Save() never serializes
	// directly; only a *materialized* model's own Serialize() output ever
	// reaches a project file, same as every real Save() does.
	FuselageModel original(FuselageSpec{});
	ProjectEntity e;
	e.name = "Fuse1";
	e.type = EntityType::Fuselage;
	e.model = original.Serialize();

	ProjectEntity back = RoundTripThroughText(e);
	auto& fs = std::get<FuselageSpec>(back.model);
	Check(!fs.legacy, "fuselage round trip: not mistaken for a legacy file (pole_links was written)");

	FuselageModel rebuilt(fs);
	CheckClose(Volume(original.BuildSolid(30, 40, std::nullopt)), Volume(rebuilt.BuildSolid(30, 40, std::nullopt)), 1e-6, "fuselage round trip: volume");
}

void CheckNacelleRoundTrip() {
	NacelleSpec nspec;
	nspec.chevron_count = 4;
	nspec.chevron_depth = 0.2;
	NacelleModel original(nspec);
	ProjectEntity e;
	e.name = "Nacelle1";
	e.type = EntityType::Nacelle;
	e.model = original.Serialize();

	ProjectEntity back = RoundTripThroughText(e);
	auto& ns = std::get<NacelleSpec>(back.model);
	NacelleModel rebuilt(ns);
	CheckClose(Volume(original.BuildSolid()), Volume(rebuilt.BuildSolid()), 1e-6, "nacelle round trip: volume");
}

void CheckLoftedNacelleRoundTrip() {
	LoftedNacelleModel original(LoftedNacelleSpec{}); // defaults (single guide section)
	ProjectEntity e;
	e.name = "LoftedNacelle1";
	e.type = EntityType::LoftedNacelle;
	e.model = original.Serialize();

	ProjectEntity back = RoundTripThroughText(e);
	auto& ls = std::get<LoftedNacelleSpec>(back.model);
	LoftedNacelleModel rebuilt(ls);
	CheckClose(Volume(original.BuildSolid(30, 80, std::nullopt)), Volume(rebuilt.BuildSolid(30, 80, std::nullopt)), 1e-6, "lofted_nacelle round trip: volume");
}

void CheckPropellerRoundTrip() {
	PropellerSpec pspec;
	pspec.radius = 1.2;
	PropellerModel original(pspec);
	ProjectEntity e;
	e.name = "Prop1";
	e.type = EntityType::Propeller;
	e.model = original.Serialize();

	ProjectEntity back = RoundTripThroughText(e);
	auto& ps = std::get<PropellerSpec>(back.model);
	Check(ps.blade.has_value(), "propeller round trip: blade spec present");
	PropellerModel rebuilt(ps);
	CheckClose(Volume(original.BuildSolid(20, std::nullopt, false)), Volume(rebuilt.BuildSolid(20, std::nullopt, false)), 1e-6, "propeller round trip: volume");
}

void CheckPrimitiveRoundTrip() {
	PrimitiveSpec pspec;
	pspec.shape = PrimitiveShape::Cylinder;
	pspec.cyl_radius = 0.5;
	pspec.cyl_length = 2.5;
	ProjectEntity e;
	e.name = "Prim1";
	e.type = EntityType::Primitive;
	e.model = pspec;

	ProjectEntity back = RoundTripThroughText(e);
	auto& ps = std::get<PrimitiveSpec>(back.model);
	Check(ps.shape == PrimitiveShape::Cylinder, "primitive round trip: shape");
	CheckClose(ps.cyl_radius, 0.5, 1e-9, "primitive round trip: cyl_radius");

	PrimitiveModel original;
	original.shape = pspec.shape;
	original.cyl_radius = pspec.cyl_radius;
	original.cyl_length = pspec.cyl_length;
	PrimitiveModel rebuilt;
	rebuilt.shape = ps.shape;
	rebuilt.cyl_radius = ps.cyl_radius;
	rebuilt.cyl_length = ps.cyl_length;
	CheckClose(Volume(original.BuildSolid()), Volume(rebuilt.BuildSolid()), 1e-6, "primitive round trip: volume");
}

// A lattice under `lattices:` (OpenLattice parameters, targets, reserved
// volumes) round-trips, and its references are validated like a boolean's.
void CheckLatticeRoundTrip() {
	ProjectFile pf;
	ProjectEntity wing;
	wing.name = "Wing";
	wing.type = EntityType::Wing;
	wing.mirror = true;
	wing.model = WingModelSpec{};
	ProjectEntity bay;
	bay.name = "Bay";
	bay.type = EntityType::Primitive;
	bay.model = PrimitiveSpec{};
	pf.entities.push_back(wing);
	pf.entities.push_back(bay);

	ProjectLattice l;
	l.name = "Lattice 1";
	l.spec.family = LatticeFamily::Strut;
	l.spec.type = "Octet";
	l.spec.cell_size = 0.25;
	l.spec.volume_fraction = 0.22;
	l.spec.resolution = 10;
	l.spec.skin_thickness = 0.01;
	l.spec.origin = Vec3{0.1, 0.2, 0.3};
	l.targets = {{"Wing", false}, {"Wing", true}};
	l.exclusions = {{"Bay", false}};
	l.color = std::array<double, 3>{0.1, 0.2, 0.3};
	pf.lattices.push_back(l);

	const std::string path = AEROTELIC_TESTS_DIR "/project_yaml_check_tmp.yaml";
	SaveProjectYaml(pf, path);
	ProjectFile back = LoadProjectYaml(path);
	Check(back.lattices.size() == 1, "lattice round trip: one lattice");
	if (back.lattices.empty()) return;
	const ProjectLattice& b = back.lattices[0];
	Check(b.name == "Lattice 1" && b.spec.family == LatticeFamily::Strut && b.spec.type == "Octet", "lattice round trip: name, family, type");
	CheckClose(b.spec.cell_size, 0.25, 1e-12, "lattice round trip: cell size");
	CheckClose(b.spec.volume_fraction, 0.22, 1e-12, "lattice round trip: volume fraction");
	Check(b.spec.resolution == 10, "lattice round trip: resolution");
	CheckClose(b.spec.skin_thickness, 0.01, 1e-12, "lattice round trip: skin");
	CheckClose(b.spec.origin.z, 0.3, 1e-12, "lattice round trip: origin");
	Check(b.targets.size() == 2 && b.targets[1].mirror && b.targets[1].name == "Wing", "lattice round trip: targets (with the mirror)");
	Check(b.exclusions.size() == 1 && b.exclusions[0].name == "Bay", "lattice round trip: exclusions");
	Check(b.color && std::abs((*b.color)[2] - 0.3) < 1e-12, "lattice round trip: colour");

	// A TPMS lattice keeps its phase; a bad reference is refused.
	pf.lattices[0].spec.family = LatticeFamily::Tpms;
	pf.lattices[0].spec.type = "Gyroid";
	pf.lattices[0].spec.phase = TpmsPhase::Sheet;
	pf.lattices[0].targets = {{"Nope", false}};
	SaveProjectYaml(pf, path);
	bool threw = false;
	try { LoadProjectYaml(path); } catch (const std::invalid_argument&) { threw = true; }
	Check(threw, "lattice round trip: a target that does not exist is refused");
	pf.lattices[0].targets = {{"Wing", false}};
	SaveProjectYaml(pf, path);
	Check(LoadProjectYaml(path).lattices[0].spec.phase == TpmsPhase::Sheet, "lattice round trip: TPMS phase");
}

void CheckRealPythonProjectFile() {
	// The Python app's repository (sibling of this one) ships the sample project.
	const std::string path = "C:\\Users\\Matt\\devel_1\\WingDesigner\\projects\\p38.yaml";
	ProjectFile project;
	try {
		project = LoadProjectYaml(path);
	} catch (const std::exception& e) {
		Check(false, std::string("loading the real p38.yaml project failed: ") + e.what());
		return;
	}
	Check(project.entities.size() == 7, "p38.yaml: entity count (fuselage/wing x3, fuselage, wing, nacelle, propeller)");

	int n_fuselage = 0, n_wing = 0, n_nacelle = 0, n_propeller = 0;
	for (auto& e : project.entities) {
		switch (e.type) {
			case EntityType::Fuselage: n_fuselage++; break;
			case EntityType::Wing: n_wing++; break;
			case EntityType::Nacelle: n_nacelle++; break;
			case EntityType::Propeller: n_propeller++; break;
			default: break;
		}
	}
	Check(n_fuselage == 2 && n_wing == 3 && n_nacelle == 1 && n_propeller == 1, "p38.yaml: entity type mix matches the file's own `type:` lines");

	// Actually construct every entity's model (not just parse the YAML)
	// and build a real solid for a representative few, to confirm the
	// loaded specs are genuinely usable, not just well-formed YAML.
	int built = 0;
	for (auto& e : project.entities) {
		try {
			if (auto* ws = std::get_if<WingModelSpec>(&e.model)) {
				WingModel wm(*ws);
				if (built < 2) { Volume(wm.BuildSolid(20, true, std::nullopt)); built++; }
			} else if (auto* fs = std::get_if<FuselageSpec>(&e.model)) {
				FuselageModel fm(*fs);
				if (built < 2) { Volume(fm.BuildSolid(20, 30, std::nullopt)); built++; }
			} else if (auto* ns = std::get_if<NacelleSpec>(&e.model)) {
				NacelleModel nm(*ns);
				Volume(nm.BuildSolid());
			} else if (auto* ps = std::get_if<PropellerSpec>(&e.model)) {
				PropellerModel pm(*ps);
				Volume(pm.BuildSolid(20, std::nullopt, false));
			}
		} catch (const std::exception& ex) {
			Check(false, "p38.yaml entity '" + e.name + "' (" + EntityTypeName(e.type) + ") failed to build: " + ex.what());
		}
	}
	Check(true, "p38.yaml: every entity constructed and the sampled ones built real solids without throwing");
}

} // namespace

int main() {
	try {
		CheckWingRoundTrip();
		CheckFuselageRoundTrip();
		CheckNacelleRoundTrip();
		CheckLoftedNacelleRoundTrip();
		CheckPropellerRoundTrip();
		CheckPrimitiveRoundTrip();
		CheckLatticeRoundTrip();
		CheckRealPythonProjectFile();
	} catch (const std::exception& e) {
		std::cout << "UNEXPECTED EXCEPTION: " << e.what() << "\n";
		return 1;
	}
	std::cout << "\nRESULT: " << g_pass << " passed, " << g_fail << " failed\n";
	return g_fail == 0 ? 0 : 1;
}
