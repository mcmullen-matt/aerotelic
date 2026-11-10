// Checks the YAML project-file layer two ways:
//  1. Self-contained round-trips (build a model, serialize, write+read the
//     YAML text, reconstruct, confirm it rebuilds to the same geometry) --
//     covers all six entity types, including the two (lofted_nacelle,
//     primitive) none of the repo's sample projects happen to use.
//  2. Loading a REAL Python-saved project file end to end
//     (aerotelic/projects/p38.yaml), to confirm actual file compatibility,
//     not just that this port's own round-trip is self-consistent.

#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
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
	Check(ps.spinner, "propeller round trip: the spinner flag survives (on)");
	PropellerModel rebuilt(ps);
	CheckClose(Volume(original.BuildSolid(20, std::nullopt, false)), Volume(rebuilt.BuildSolid(20, std::nullopt, false)), 1e-6, "propeller round trip: volume");

	// Off must survive too, and must not take the profile with it.
	PropellerSpec no_spinner = pspec;
	no_spinner.spinner = false;
	ProjectEntity e2;
	e2.name = "Prop2";
	e2.type = EntityType::Propeller;
	e2.model = PropellerModel(no_spinner).Serialize();
	auto& ps2 = std::get<PropellerSpec>(RoundTripThroughText(e2).model);
	Check(!ps2.spinner, "propeller round trip: the spinner flag survives (off)");
	CheckClose(ps2.spinner_length, no_spinner.spinner_length, 1e-12, "propeller round trip: the spinner length is kept while off");

	// A file written before the spinner became optional has no `spinner`
	// key at all, and must still come back with one -- otherwise opening
	// an old project would quietly delete a body.
	YAML::Node legacy = Encode(PropellerSpec());
	legacy.remove("spinner");
	Check(!legacy["spinner"], "legacy propeller: the test's own node really has no spinner key");
	Check(DecodePropellerSpec(legacy).spinner, "legacy propeller: a file with no spinner key still builds one");

	// The spin direction survives, and a file written before it existed
	// comes back positive -- the hand it was drawn with.
	PropellerSpec negative = pspec;
	negative.spin_direction = SpinDirection::Negative;
	ProjectEntity e3;
	e3.name = "Prop3";
	e3.type = EntityType::Propeller;
	e3.model = PropellerModel(negative).Serialize();
	auto& ps3 = std::get<PropellerSpec>(RoundTripThroughText(e3).model);
	Check(ps3.spin_direction == SpinDirection::Negative, "propeller round trip: the spin direction survives (negative)");
	Check(std::get<PropellerSpec>(back.model).spin_direction == SpinDirection::Positive,
		"propeller round trip: and positive");
	YAML::Node no_spin = Encode(PropellerSpec());
	no_spin.remove("spin_direction");
	Check(!no_spin["spin_direction"], "legacy propeller: the test's own node really has no spin_direction key");
	Check(DecodePropellerSpec(no_spin).spin_direction == SpinDirection::Positive,
		"legacy propeller: a file with no spin_direction key turns positive, as it did when it was written");
	// A direction that is not one of the two is a bad file, not a default.
	YAML::Node bad = Encode(PropellerSpec());
	bad["spin_direction"] = "sideways";
	bool spin_threw = false;
	try { DecodePropellerSpec(bad); } catch (const std::invalid_argument&) { spin_threw = true; }
	Check(spin_threw, "propeller: an unknown spin direction in a file is refused");
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

// A placement written by the Transform dialog: an axis as typed (not
// normalised, and kept even at zero angle) and, when the Euler tab
// authored the rotation, the triple beside the axis+angle it composes to.
void CheckPlacementRoundTrip() {
	ProjectEntity e;
	e.name = "Prim1";
	e.type = EntityType::Primitive;
	e.model = PrimitiveSpec{};
	e.placement.translation = {1.0, 2.0, 3.0};
	e.placement.rotation_axis = {0.0, 2.0, 1.0};  // as typed
	e.placement.rotation_deg = 0.0;               // ...and no angle yet

	ProjectEntity back = RoundTripThroughText(e);
	CheckClose(back.placement.rotation_axis[1], 2.0, 1e-12, "placement round trip: the axis is stored as typed");
	CheckClose(back.placement.rotation_axis[2], 1.0, 1e-12, "placement round trip: ...all of it");
	CheckClose(back.placement.rotation_deg, 0.0, 1e-12, "placement round trip: a zero angle does not erase the axis");
	Check(!back.placement.euler.used, "placement round trip: no Euler block without an Euler rotation");

	// With the Euler tab in use: the triple comes back, and the axis+angle
	// beside it is the one it composes to.
	e.placement.euler.used = true;
	e.placement.euler.intrinsic = true;
	e.placement.euler.order = EulerOrder::ZYX;
	e.placement.euler.angles_deg = {15.0, -30.0, 45.0};
	SyncPlacementRotation(e.placement);

	ProjectEntity with_euler = RoundTripThroughText(e);
	const Placement& p = with_euler.placement;
	Check(p.euler.used && p.euler.intrinsic && p.euler.order == EulerOrder::ZYX,
		"placement round trip: the Euler order and frame survive");
	CheckClose(p.euler.angles_deg[0], 15.0, 1e-12, "placement round trip: euler angle 0");
	CheckClose(p.euler.angles_deg[1], -30.0, 1e-12, "placement round trip: euler angle 1");
	CheckClose(p.euler.angles_deg[2], 45.0, 1e-12, "placement round trip: euler angle 2");
	CheckClose(p.rotation_deg, e.placement.rotation_deg, 1e-9,
		"placement round trip: the derived angle agrees with the triple");
	for (int i = 0; i < 3; ++i) {
		CheckClose(p.rotation_axis[i], e.placement.rotation_axis[i], 1e-9,
			"placement round trip: the derived axis agrees with the triple");
	}
}

// `mass:` on an entity -- asserted or by density -- round-trips, and an
// entity without one writes no key at all.
void CheckMassSpecRoundTrip() {
	ProjectEntity e;
	e.name = "Prim1";
	e.type = EntityType::Primitive;
	e.model = PrimitiveSpec{};

	ProjectEntity none = RoundTripThroughText(e);
	Check(none.mass.mode == MassMode::Unset, "mass round trip: no setting stays unset");

	e.mass.mode = MassMode::Asserted;
	e.mass.mass = 12.5;
	ProjectEntity asserted = RoundTripThroughText(e);
	Check(asserted.mass.mode == MassMode::Asserted, "mass round trip: asserted mode survives");
	CheckClose(asserted.mass.mass, 12.5, 1e-12, "mass round trip: the asserted mass");

	e.mass.mode = MassMode::Material;
	// The MIL-HDBK-5J Ti-6Al-4V sheet (the ASM-typical "Titanium Ti-6Al-4V"
	// is no longer a preset: see materials.cpp).
	e.mass.material = *FindMaterialPreset("Titanium Ti-6Al-4V annealed sheet");
	ProjectEntity by_material = RoundTripThroughText(e);
	Check(by_material.mass.mode == MassMode::Material, "mass round trip: material mode survives");
	Check(by_material.mass.material.name == "Titanium Ti-6Al-4V annealed sheet", "mass round trip: the material's name");
	CheckClose(by_material.mass.material.density, 4429.0, 1e-12, "mass round trip: the density");
	CheckClose(by_material.mass.material.youngs_modulus, 110.3e9, 1e-3, "mass round trip: Young's modulus");
	CheckClose(by_material.mass.material.poisson_ratio, 0.31, 1e-12, "mass round trip: Poisson's ratio");
	CheckClose(by_material.mass.material.allowable_stress, 869.0e6, 1e-3, "mass round trip: the allowable");
	Check(IsMaterialPreset(by_material.mass.material), "mass round trip: it is still the preset");
	CheckClose(by_material.mass.skin_thickness, 0.0, 0.0, "mass round trip: a solid writes no skin thickness and reads back solid");

	// A skin of the material: the thickness travels beside it, as typed.
	e.mass.skin_thickness = 0.0025;
	ProjectEntity skin = RoundTripThroughText(e);
	Check(skin.mass.mode == MassMode::Material && skin.mass.IsSkin(), "mass round trip: a skin stays a skin");
	CheckClose(skin.mass.skin_thickness, 0.0025, 1e-15, "mass round trip: the skin thickness");
	Check(skin.mass.material.name == "Titanium Ti-6Al-4V annealed sheet", "mass round trip: the skin's material");
	e.mass.skin_thickness = 0.0;

	// The forms the file is meant to read as, the pre-material one included.
	MassSpec from_text = DecodeMassSpec(YAML::Load("{asserted: 3.5}"));
	Check(from_text.mode == MassMode::Asserted && from_text.mass == 3.5, "mass: `{asserted: 3.5}` decodes");
	from_text = DecodeMassSpec(YAML::Load("{material: {name: Custom, density: 1200, youngs_modulus: 3.0e9, poisson_ratio: 0.35, allowable_stress: 5.0e7}}"));
	Check(from_text.mode == MassMode::Material && from_text.material.density == 1200.0 && from_text.material.youngs_modulus == 3.0e9
		&& from_text.material.name == "Custom", "mass: a custom material decodes");
	from_text = DecodeMassSpec(YAML::Load("{material: {name: Custom, density: 1200}, skin_thickness: 0.003}"));
	Check(from_text.mode == MassMode::Material && from_text.skin_thickness == 0.003 && from_text.IsSkin(), "mass: `skin_thickness` beside the material decodes");
	from_text = DecodeMassSpec(YAML::Load("{density: 1200}"));
	Check(from_text.mode == MassMode::Material && from_text.material.density == 1200.0 && from_text.material.name == "Custom"
		&& from_text.material.youngs_modulus == 0.0, "mass: the old `{density: 1200}` reads as a custom material with only a density");
}

// `bodies:` on an entity: each solid body's structure -- material, skin,
// exclusions, asserted mass -- round-trips with only the keys that were
// set, a body with nothing set is not written, a boolean result's bodies
// carry theirs too, a structure that cannot be used is refused with the
// body named, and a legacy `networks:` key is dropped and reported.
void CheckBodiesRoundTrip() {
	ProjectEntity e;
	e.name = "Wing1";
	e.type = EntityType::Primitive;
	e.model = PrimitiveSpec{};

	// A body with nothing set writes no entry at all.
	ProjectBody blank;
	blank.name = "Main";
	e.bodies.push_back(blank);
	Check(RoundTripThroughText(e).bodies.empty(), "bodies: a body with nothing set is not written");
	e.bodies.clear();

	ProjectBody main;
	main.name = "Main";
	main.structure.material = *FindMaterialPreset("Aluminium 6061-T6 sheet");
	e.bodies.push_back(main);

	ProjectBody flap;
	flap.name = "Flap 1";
	flap.color = std::array<double, 3>{0.2, 0.4, 0.6};
	flap.structure.material = *FindMaterialPreset("Balsa, medium (160 kg/m^3)");
	flap.structure.skin_thickness = 0.0015;
	flap.structure.exclusions = {"Bay"};
	flap.structure.asserted_mass = 0.2;
	e.bodies.push_back(flap);

	ProjectEntity back = RoundTripThroughText(e);
	Check(back.bodies.size() == 2 && back.bodies[0].name == "Main" && back.bodies[1].name == "Flap 1", "bodies: both bodies come back by name");
	if (back.bodies.size() != 2) return;
	const BodyStructure& m = back.bodies[0].structure;
	Check(m.IsSet() && !m.IsSkin() && m.material.name == "Aluminium 6061-T6 sheet" && m.material.density == 2713.0, "bodies: a solid body keeps its material");
	Check(!back.bodies[0].color && !back.bodies[0].hidden, "bodies: no colour or hidden flag was invented");
	const ProjectBody& f = back.bodies[1];
	Check(f.color && std::abs((*f.color)[1] - 0.4) < 1e-12, "bodies: the flap's colour");
	const BodyStructure& s = f.structure;
	Check(s.IsSkin() && s.material.name == "Balsa, medium (160 kg/m^3)" && s.material.density == 160.0, "bodies: the flap's skin material");
	CheckClose(s.skin_thickness, 0.0015, 1e-15, "bodies: the skin thickness");
	Check(s.exclusions.size() == 1 && s.exclusions[0] == "Bay", "bodies: the exclusions by scene name");
	Check(s.asserted_mass && *s.asserted_mass == 0.2, "bodies: the asserted mass");
	Check(SameStructureGeometry(s, flap.structure), "bodies: the geometry is the same after the round trip");

	// A boolean result's bodies carry structure too.
	{
		ProjectFile pf;
		ProjectEntity a = e, b = e;
		a.name = "A"; a.bodies.clear();
		b.name = "B"; b.bodies.clear();
		pf.entities = {a, b};
		ProjectBoolean fuse;
		fuse.name = "Fused";
		fuse.operands = {{"A", false}, {"B", false}};
		ProjectBody body1;
		body1.name = "Body 1";
		body1.hidden = true;
		body1.structure.material = *FindMaterialPreset("Steel 4130, normalised");
		body1.structure.skin_thickness = 0.001;
		fuse.bodies.push_back(body1);
		pf.booleans.push_back(fuse);
		const std::string path = AEROTELIC_TESTS_DIR "/project_yaml_check_tmp.yaml";
		SaveProjectYaml(pf, path);
		ProjectFile reloaded = LoadProjectYaml(path);
		Check(reloaded.booleans.size() == 1 && reloaded.booleans[0].bodies.size() == 1, "bodies: a boolean's body is written and read");
		if (reloaded.booleans.size() == 1 && reloaded.booleans[0].bodies.size() == 1) {
			const ProjectBody& rb = reloaded.booleans[0].bodies[0];
			Check(rb.name == "Body 1" && rb.hidden && rb.structure.material.name == "Steel 4130, normalised" && rb.structure.skin_thickness == 0.001,
				"bodies: the boolean body's hidden flag, material and skin");
		}
	}

	// A structure that cannot be used is refused, naming the body; a legacy
	// plane network is dropped and flagged, not an error.
	{
		bool threw = false;
		std::string what;
		try {
			DecodeProjectBody(YAML::Load("{name: Flap, material: {name: X, density: 100}, skin_thickness: -0.001}"));
		} catch (const std::invalid_argument& ex) { threw = true; what = ex.what(); }
		Check(threw && what.find("Flap") != std::string::npos && what.find("negative") != std::string::npos, "bodies: a negative skin thickness is refused with the body named");
		bool had_infill = false, had_networks = false;
		const ProjectBody old = DecodeProjectBody(YAML::Load("{name: Main, material: {name: X, density: 100}, skin_thickness: 0.001, networks: [{axis: x, start: 0.3, count: 1, thickness: 0.004}]}"), &had_infill, &had_networks);
		Check(old.structure.IsSkin() && !had_infill && had_networks, "bodies: a legacy networks: key loads the skin and is flagged");
	}
}

// `mesh:` on an entity -- the density of its grid (the Mesh tab) --
// round-trips with only the keys that were set, and an entity left
// automatic writes no key at all.
void CheckMeshSpecRoundTrip() {
	ProjectEntity e;
	e.name = "Box1";
	e.type = EntityType::Primitive;
	e.model = PrimitiveSpec{};
	ProjectEntity none = RoundTripThroughText(e);
	Check(none.mesh.IsDefault(), "mesh round trip: nothing set stays automatic");
	Check(EncodeMeshSpec(MeshSpec{}).size() == 0, "mesh: automatic encodes to an empty map (and is not written)");

	e.mesh.stations = 20;
	e.mesh.around = 32;
	e.mesh.refine = false;
	e.mesh.tolerance = 0.002;
	e.mesh.max_aspect = -1.0;
	e.mesh.cells = {2, 20, 2};
	e.mesh.cap_rings = 3;
	e.mesh.chordwise = 16;
	e.mesh.spanwise = 12;
	ProjectEntity back = RoundTripThroughText(e);
	Check(back.mesh.stations == 20 && back.mesh.around == 32 && !back.mesh.refine && back.mesh.cells == std::array<int, 3>{2, 20, 2}
		&& back.mesh.cap_rings == 3 && back.mesh.chordwise == 16 && back.mesh.spanwise == 12, "mesh round trip: every count survives");
	CheckClose(back.mesh.tolerance, 0.002, 1e-15, "mesh round trip: the tolerance");
	CheckClose(back.mesh.max_aspect, -1.0, 0.0, "mesh round trip: the aspect switch");

	MeshSpec partial = DecodeMeshSpec(YAML::Load("{stations: 5, cells: [1, 2, 3]}"));
	Check(partial.stations == 5 && partial.cells == std::array<int, 3>{1, 2, 3} && partial.refine && partial.around == 0 && partial.tolerance == 0.0,
		"mesh: a partial `mesh:` decodes with the rest automatic");
	Check(DecodeMeshSpec(YAML::Node()).IsDefault(), "mesh: no key at all is automatic");
	e.mesh = MeshSpec{};
}

// `units:` is always written -- the defaults too -- and read back; a file
// without it is metres and kilograms; a unit the application does not
// offer is refused.
void CheckUnitsRoundTrip() {
	const std::string path = AEROTELIC_TESTS_DIR "/project_yaml_check_tmp.yaml";
	ProjectEntity e;
	e.name = "Prim1";
	e.type = EntityType::Primitive;
	e.model = PrimitiveSpec{};

	ProjectFile pf;
	pf.entities.push_back(e);
	SaveProjectYaml(pf, path);
	{
		std::ifstream in(path);
		std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
		Check(text.find("units:") != std::string::npos && text.find("length: m") != std::string::npos
			&& text.find("mass: kg") != std::string::npos && text.find("time: s") != std::string::npos
			&& text.find("stress: Pa") != std::string::npos,
			"units: the defaults are written out, time and stress included");
		Check(text.find("units:") < text.find("entities:"), "units: comes first in the file");
	}
	ProjectFile back = LoadProjectYaml(path);
	Check(back.units == UnitSystem{}, "units: the defaults read back");

	pf.units.length = LengthUnit::Foot;
	pf.units.mass = MassUnit::PoundMass;
	pf.units.stress = StressUnit::Psi;
	SaveProjectYaml(pf, path);
	back = LoadProjectYaml(path);
	Check(back.units.length == LengthUnit::Foot && back.units.mass == MassUnit::PoundMass && back.units.stress == StressUnit::Psi,
		"units: feet, pounds and psi round-trip");

	// A file from before the key existed.
	{
		std::ofstream out(path);
		out << "entities:\n  - name: P\n    type: primitive\n    model: {shape: box}\n";
	}
	back = LoadProjectYaml(path);
	Check(back.units == UnitSystem{}, "units: a file without the key is metres and kilograms");

	// One claiming a unit the application does not offer.
	{
		std::ofstream out(path);
		out << "units: {length: inch, mass: kg, time: s}\nentities: []\n";
	}
	bool threw = false;
	try { LoadProjectYaml(path); } catch (const std::invalid_argument&) { threw = true; }
	Check(threw, "units: an unoffered length unit is refused on load");
	{
		std::ofstream out(path);
		out << "units: {length: m, mass: kg, time: min}\nentities: []\n";
	}
	threw = false;
	try { LoadProjectYaml(path); } catch (const std::invalid_argument&) { threw = true; }
	Check(threw, "units: a time unit other than seconds is refused on load");
}

// `scale:` (Model > Scale) is written only when the vehicle is not
// built full size, read back, absent means 1, and anything but a positive
// number is refused both ways. The entities beneath it are the full-scale
// aircraft whatever it says.
void CheckScaleRoundTrip() {
	const std::string path = AEROTELIC_TESTS_DIR "/project_yaml_check_tmp.yaml";
	ProjectEntity e;
	e.name = "Wing";
	e.type = EntityType::Wing;
	e.model = WingModelSpec{};
	e.placement.translation = {4.0, 0.0, 1.0};

	ProjectFile pf;
	pf.entities.push_back(e);
	SaveProjectYaml(pf, path);
	{
		std::ifstream in(path);
		std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
		Check(text.find("scale:") == std::string::npos, "scale: not written at full size, so a 1:1 file reads as it always did");
	}
	ProjectFile back = LoadProjectYaml(path);
	Check(back.scale == 1.0, "scale: absent reads as full size");

	pf.scale = 0.25;
	SaveProjectYaml(pf, path);
	{
		std::ifstream in(path);
		std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
		Check(text.find("scale: 0.25") != std::string::npos, "scale: 0.25 is written");
		Check(text.find("units:") < text.find("scale:") && text.find("scale:") < text.find("entities:"), "scale: sits between units and entities");
	}
	back = LoadProjectYaml(path);
	Check(back.scale == 0.25, "scale: 0.25 reads back");
	Check(back.entities.size() == 1 && std::get<WingModelSpec>(back.entities[0].model).planform.semi_span == kDefaultSemiSpan,
		"scale: the parameterisation beneath it is still the full-scale one");
	Check(back.entities[0].placement.translation[0] == 4.0, "scale: the placement beneath it is still the full-scale one");

	for (const char* bad : {"0", "-2", ".nan"}) {
		std::ofstream out(path);
		out << "units: {length: m, mass: kg, time: s}\nscale: " << bad << "\nentities: []\n";
		out.close();
		bool threw = false;
		try { LoadProjectYaml(path); } catch (const std::invalid_argument&) { threw = true; }
		Check(threw, std::string("scale: '") + bad + "' is refused on load");
	}
	pf.scale = 0.0;
	bool threw = false;
	try { SaveProjectYaml(pf, path); } catch (const std::invalid_argument&) { threw = true; }
	Check(threw, "scale: zero is refused on save");
}

// A file from before 2026-11-08 with a `lattices:` section: lattices are
// no longer supported at all, so the entries are skipped by name (the
// rest of the file loads) and never written back. A body whose entry
// carries the `infill:` key of the same era loads with the infill dropped
// and is reported by "<entity> / <body>" in ignored_infill.
void CheckUnsupportedLattices() {
	const std::string path = AEROTELIC_TESTS_DIR "/project_yaml_check_tmp.yaml";
	{
		// Two real entities as the writer puts them down, then the old
		// section appended as a file of the time would have had it.
		ProjectFile old;
		ProjectEntity w;
		w.name = "Wing";
		w.type = EntityType::Wing;
		w.mirror = true;
		w.model = WingModelSpec{};
		ProjectEntity bay;
		bay.name = "Bay";
		bay.type = EntityType::Primitive;
		bay.model = PrimitiveSpec{};
		old.entities.push_back(w);
		old.entities.push_back(bay);
		SaveProjectYaml(old, path);
		std::ofstream out(path, std::ios::app);
		out << "\nlattices:\n"
		       "  - name: Lattice 1\n"
		       "    lattice: {family: strut, type: Octet, cell_size: 0.25, volume_fraction: 0.22, resolution: 10}\n"
		       "    targets: [Wing, {mirror: Wing}]\n"
		       "    exclusions: [Bay]\n"
		       "  - name: Lattice 2\n"
		       "    lattice: {family: tpms, type: Gyroid, cell_size: 0.5}\n"
		       "    targets: [Nope]\n";
	}
	ProjectFile back;
	bool threw = false;
	try { back = LoadProjectYaml(path); } catch (const std::exception& e) { threw = true; std::cerr << e.what() << "\n"; }
	Check(!threw, "old lattices: a file with a lattices: section still loads");
	Check(back.entities.size() == 2, "old lattices: the entities load");
	Check(back.unsupported_lattices.size() == 2 && back.unsupported_lattices[0] == "Lattice 1" && back.unsupported_lattices[1] == "Lattice 2",
		"old lattices: the lattice entries are named as unsupported, however they refer");
	SaveProjectYaml(back, path);
	{
		std::ifstream in(path);
		std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
		Check(text.find("lattices") == std::string::npos, "old lattices: nothing is written back under lattices:");
	}
	Check(LoadProjectYaml(path).unsupported_lattices.empty(), "old lattices: the re-saved file has none");
	// The entity type itself is refused under entities:.
	{
		std::ofstream out(path);
		out << "entities:\n  - name: L\n    type: lattice\n    model: {}\n";
	}
	threw = false;
	try { LoadProjectYaml(path); } catch (const std::invalid_argument&) { threw = true; }
	Check(threw, "old lattices: an entity of type lattice is refused");

	// A body with an `infill:` key, as a file of 2026-11-08 wrote one: the
	// infill is dropped, the body's skin and network load, the file is
	// otherwise intact, and the body is reported by its owner and name.
	{
		std::ofstream out(path);
		out << "units: {length: m, mass: kg, time: s, stress: Pa}\n"
		       "entities:\n"
		       "  - name: Wing\n"
		       "    type: primitive\n"
		       "    model: {shape: sphere, name: Wing, radius: 1.0}\n"
		       "    bodies:\n"
		       "      - name: Main\n"
		       "        material: {name: Balsa, density: 160.0, youngs_modulus: 3.0e9, poisson_ratio: 0.3, allowable_stress: 1.0e7}\n"
		       "        skin_thickness: 0.002\n"
		       "        infill:\n"
		       "          material: {name: PLA, density: 1240.0, youngs_modulus: 3.5e9, poisson_ratio: 0.36, allowable_stress: 5.0e7}\n"
		       "          lattice: {family: honeycomb, type: Hexagon, cell_size: 0.02, cell_aspect: 1.75, axis: y, wall_sizing: thickness, wall_thickness: 0.03}\n"
		       "        networks:\n"
		       "          - {name: Ribs, axis: y, start: 0.05, spacing: 0.1, count: 3, thickness: 0.002, material: {name: Ply, density: 680.0, youngs_modulus: 9.0e9, poisson_ratio: 0.3, allowable_stress: 3.0e7}}\n"
		       "      - name: Other\n"
		       "        material: {name: Balsa, density: 160.0, youngs_modulus: 3.0e9, poisson_ratio: 0.3, allowable_stress: 1.0e7}\n";
	}
	ProjectFile inf;
	threw = false;
	try { inf = LoadProjectYaml(path); } catch (const std::exception& e) { threw = true; std::cerr << e.what() << "\n"; }
	Check(!threw, "old infill: a body with an infill: key still loads");
	Check(inf.entities.size() == 1 && inf.entities[0].bodies.size() == 2, "old infill: the entity and both bodies load");
	if (inf.entities.size() == 1 && inf.entities[0].bodies.size() == 2) {
		const BodyStructure& s = inf.entities[0].bodies[0].structure;
		Check(s.IsSkin() && s.material.name == "Balsa", "old infill: the body's skin material survives");
		CheckClose(s.skin_thickness, 0.002, 1e-15, "old infill: the skin thickness survives");
		Check(!inf.entities[0].bodies[1].structure.IsSkin(), "old infill: the other body is a plain solid");
	}
	Check(inf.ignored_infill.size() == 1 && inf.ignored_infill[0] == "Wing / Main",
		"old infill: the body whose infill was dropped is reported as '<entity> / <body>'");
	Check(inf.ignored_networks.size() == 1 && inf.ignored_networks[0] == "Wing / Main",
		"old networks: the body whose plane network was dropped is reported as '<entity> / <body>'");
	Check(inf.unsupported_lattices.empty(), "old infill: an infill: key is not a lattices: entry");
	SaveProjectYaml(inf, path);
	{
		std::ifstream in(path);
		std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
		Check(text.find("infill") == std::string::npos && text.find("networks") == std::string::npos, "old infill: nothing is written back under infill: or networks:");
	}
	Check(LoadProjectYaml(path).ignored_infill.empty() && LoadProjectYaml(path).ignored_networks.empty(), "old infill: the re-saved file has none to report");
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

	// Every body's own mesh is a closed, outward manifold -- what a signed
	// distance field (sdf.hpp) needs for its sign. The booms end
	// in knife edges (zero width, some height), the tailplane's tip chord
	// closes to zero: both must come out closed.
	for (auto& e : project.entities) {
		std::vector<NamedMesh> meshes;
		try {
			if (auto* ws = std::get_if<WingModelSpec>(&e.model)) meshes = WingModel(*ws).BuildMeshes(2e-3);
			else if (auto* fs = std::get_if<FuselageSpec>(&e.model)) meshes = FuselageModel(*fs).BuildMeshes(2e-3);
			else if (auto* ns = std::get_if<NacelleSpec>(&e.model)) meshes = NacelleModel(*ns).BuildMeshes(2e-3);
			else if (auto* ps = std::get_if<PropellerSpec>(&e.model)) meshes = PropellerModel(*ps).BuildMeshes(2e-3);
		} catch (const std::exception& ex) {
			Check(false, "p38.yaml entity '" + e.name + "' failed to mesh: " + ex.what());
			continue;
		}
		Check(!meshes.empty(), "p38.yaml entity '" + e.name + "' has a mesh builder");
		for (const NamedMesh& m : meshes) {
			Check(m.mesh.IsClosedManifold() && m.mesh.SignedVolume() > 0.0, "p38.yaml '" + e.name + "/" + m.name + "': closed, outward mesh");
		}
	}

	// A planform whose trailing edge crosses ahead of the leading edge at
	// the tip (the tailplane as it was saved before 2026-10-11) is refused
	// with the span range named, by both build paths.
	for (auto& e : project.entities) {
		auto* ws = std::get_if<WingModelSpec>(&e.model);
		if (!ws || e.name != "HT") continue;
		WingModelSpec bad = *ws;
		Mat te = bad.planform.te_curve->poles;
		te(te.rows() - 1, 1) = te(te.rows() - 1, 1) - 0.01; // tip trailing edge 10 mm ahead of the leading edge
		bad.planform.te_curve->poles = te;
		WingModel wm(bad);
		Check(wm.NegativeChordRange().has_value(), "negative tip chord: NegativeChordRange reports it");
		std::string msg;
		try {
			wm.BuildMeshes(2e-3);
			Check(false, "negative tip chord: BuildMeshes should refuse");
		} catch (const std::invalid_argument& ex) {
			msg = ex.what();
			Check(msg.find("negative chord") != std::string::npos && msg.find("100.00%") != std::string::npos,
				"negative tip chord: BuildMeshes refuses, naming the span range (" + msg + ")");
		}
		try {
			wm.BuildSolid();
			Check(false, "negative tip chord: BuildSolid should refuse");
		} catch (const std::invalid_argument&) {
			Check(true, "negative tip chord: BuildSolid refuses too");
		}
		// The saved file itself (tip chord exactly zero: a pointed tip) builds.
		Check(!WingModel(*ws).NegativeChordRange().has_value(), "p38 HT as saved: no negative chord (tip chord is zero)");
	}
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
		CheckPlacementRoundTrip();
		CheckMassSpecRoundTrip();
		CheckBodiesRoundTrip();
		CheckMeshSpecRoundTrip();
		CheckUnitsRoundTrip();
		CheckScaleRoundTrip();
		CheckUnsupportedLattices();
		CheckRealPythonProjectFile();
	} catch (const std::exception& e) {
		std::cout << "UNEXPECTED EXCEPTION: " << e.what() << "\n";
		return 1;
	}
	std::cout << "\nRESULT: " << g_pass << " passed, " << g_fail << " failed\n";
	return g_fail == 0 ? 0 : 1;
}
