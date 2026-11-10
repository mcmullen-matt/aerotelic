#pragma once

// YAML (de)serialization for Aerotelic project files, matching the Python
// app's exact file shape (see ui/main_window.py's _write_project/
// _load_project_path) so existing .yaml projects open directly and files
// this port writes stay readable by the Python app.
//
// This is the text-encoding boundary every *Spec struct in core/ was
// deliberately kept clear of (see nurbs.hpp's NurbsSpec and every other
// model's file header) -- yaml-cpp types are expected to appear in this
// module's API, unlike the rest of aerotelic_cpp_core.
//
// Not ported (same scope cuts as the structs being encoded): reference
// images, pole_links (beyond the bare presence of an empty 'pole_links'
// key on a saved Fuselage, needed so the Python app doesn't mistake a
// file this app wrote for a pre-2026 legacy one -- see EncodeFuselage).
// The 'booleans' top-level key is the list of boolean operations as the
// user authored them (main_window.py _booleans_for_save / _load_boolean):
// name, operation, operands by name, display state. The feature-graph
// nodes behind an operation are an implementation detail and are rebuilt
// from this on load.

#include <optional>
#include <string>
#include <variant>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "aerotelic/body_structure.hpp"
#include "aerotelic/feature_tree.hpp"
#include "aerotelic/fuselage_model.hpp"
#include "aerotelic/mass_properties.hpp"
#include "aerotelic/units.hpp"
#include "aerotelic/lofted_nacelle_model.hpp"
#include "aerotelic/nacelle_model.hpp"
#include "aerotelic/primitive_model.hpp"
#include "aerotelic/propeller_model.hpp"
#include "aerotelic/wing_model.hpp"

namespace aerotelic::project_yaml {

// ---- building blocks (reusable on their own) ----

YAML::Node Encode(const NurbsSpec& spec);
NurbsSpec DecodeNurbsSpec(const YAML::Node& node);

YAML::Node Encode(const PlanformSpec& spec);
PlanformSpec DecodePlanformSpec(const YAML::Node& node);

YAML::Node Encode(const WingModelSpec& spec);
WingModelSpec DecodeWingModelSpec(const YAML::Node& node);
// The wing's `structure:` key (spars, ribs, stringers); a missing or
// non-map node decodes to WingStructureSpec::Default().
YAML::Node EncodeWingStructure(const WingStructureSpec& s);
WingStructureSpec DecodeWingStructure(const YAML::Node& node);

YAML::Node Encode(const NacelleSpec& spec);
NacelleSpec DecodeNacelleSpec(const YAML::Node& node);

YAML::Node Encode(const LoftedNacelleSpec& spec);
LoftedNacelleSpec DecodeLoftedNacelleSpec(const YAML::Node& node);

YAML::Node Encode(const PropellerSpec& spec);
PropellerSpec DecodePropellerSpec(const YAML::Node& node);

YAML::Node Encode(const FuselageSpec& spec);
FuselageSpec DecodeFuselageSpec(const YAML::Node& node, bool* out_legacy = nullptr);

// Mirrors PrimitiveModel's public fields directly (it has no separate
// *Spec struct of its own).
struct PrimitiveSpec {
	PrimitiveShape shape = PrimitiveShape::Sphere;
	std::string name = "Primitive";
	double radius = 1.0;
	std::array<double, 3> stretch = {1.0, 1.0, 1.0};
	std::array<double, 3> size = {2.0, 2.0, 2.0};
	double cyl_radius = 1.0;
	double cyl_length = 2.0;
};
YAML::Node Encode(const PrimitiveSpec& spec);
PrimitiveSpec DecodePrimitiveSpec(const YAML::Node& node);

YAML::Node Encode(const Placement& p);
Placement DecodePlacement(const YAML::Node& node);

// ---- project-level ----

// Boolean is the kind of a boolean operation's result: a scene item like
// the others (listed, coloured, hidden, saved), but it is never an entry
// under `entities:` -- it lives under `booleans:` (ProjectBoolean).
// (Until 2026-11-08 a Lattice kind existed too, a lattice infill generated
// in other bodies and saved under `lattices:`; lattice infill was removed
// altogether in favour of shell structure only, and the old entries are
// skipped on load, see ProjectFile::unsupported_lattices.)
enum class EntityType { Wing, Fuselage, Nacelle, LoftedNacelle, Propeller, Primitive, Boolean };
const char* EntityTypeName(EntityType t);
EntityType ParseEntityType(const std::string& s);

using EntityModelSpec = std::variant<WingModelSpec, FuselageSpec, NacelleSpec, LoftedNacelleSpec, PropellerSpec, PrimitiveSpec>;

// One solid body of an entity or a boolean result (`bodies:`), by its role
// name: its display overrides (colour, hidden) and what it is made of and
// built with -- its skin, plane networks, asserted mass
// (body_structure.hpp). Written only when something is set.
struct ProjectBody {
	std::string name;
	std::optional<std::array<double, 3>> color;
	bool hidden = false;
	BodyStructure structure;

	bool IsDefault() const { return !color && !hidden && !structure.IsSet() && structure.skin_thickness <= 0.0 && structure.exclusions.empty(); }
};
using ProjectBooleanBody = ProjectBody;

struct ProjectEntity {
	std::string name;
	EntityType type;
	bool mirror = false;
	Placement placement{{0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, 0.0};
	EntityModelSpec model;
	// Display state, written only when it differs from how the entity was
	// made (the Python app's rule: defaults and inherited values never
	// reach the file). An unset colour means "use the per-type default".
	// RGB in 0..1, no alpha -- the Qt app offered none.
	std::optional<std::array<double, 3>> color;
	bool hidden = false;
	// How the entity weighs (Analysis -> Mass Properties): written only
	// when set, as `mass: {asserted: <total>}` or `mass: {density: <rho>}`.
	MassSpec mass;
	// The density of its grid (the design panel's Mesh tab, mesh.hpp
	// MeshSpec): written only when anything is set, as `mesh: {...}` with
	// the non-default keys alone.
	MeshSpec mesh;
	// Its bodies' display overrides and structure (`bodies:`), by role
	// name; a body with nothing set is not listed. Since 2026-11-08 the
	// material, the skin and the internal structure live here, per body;
	// `mass:` above is the form before that (one setting for every body),
	// still read, no longer written once the bodies carry theirs.
	std::vector<ProjectBody> bodies;
};

// `mesh:` on an entity: stations, around, refine, tolerance, max_aspect,
// cells [nx, ny, nz], cap_rings, chordwise, spanwise -- each only when it
// differs from automatic.
YAML::Node EncodeMeshSpec(const MeshSpec& m);
MeshSpec DecodeMeshSpec(const YAML::Node& node);

// One operand of a boolean operation, by scene name: an entity, the
// mirrored half of a mirrored entity (`{mirror: <entity name>}` in the
// file), or an earlier boolean result.
struct BooleanOperand {
	std::string name;
	bool mirror = false;
};

// A material as the file writes it: the name and the four figures (SI),
// in flow style, so the file stands on its own when a preset changes.
YAML::Node EncodeMaterial(const Material& m);
Material DecodeMaterial(const YAML::Node& node);
// The structure's keys -- material, skin_thickness, exclusions,
// asserted_mass -- the non-default ones only, written into `into` (a
// body's own mapping). Decode throws std::invalid_argument when the
// structure cannot be used (BodyStructureProblem). Two legacy keys are
// ignored, not errors, so a file of their time still opens: `infill:` (a
// lattice infill, removed 2026-11-08) and `networks:` (plane networks,
// removed 2026-11-10); `had_infill` / `had_networks`, when given, are set
// to true if one was met so the caller can report it
// (ProjectFile::ignored_infill / ignored_networks).
void EncodeBodyStructure(const BodyStructure& s, YAML::Node& into);
BodyStructure DecodeBodyStructure(const YAML::Node& node, bool* had_infill = nullptr, bool* had_networks = nullptr);
YAML::Node Encode(const ProjectBody& b);
ProjectBody DecodeProjectBody(const YAML::Node& node, bool* had_infill = nullptr, bool* had_networks = nullptr);

struct ProjectBoolean {
	std::string name;
	// "fuse" | "cut" | "common"; cut is first minus the rest.
	std::string operation = "fuse";
	// In build order: folded left to right ((A op B) op C).
	std::vector<BooleanOperand> operands;
	std::optional<double> fillet_radius;
	bool hidden = false;
	std::optional<std::array<double, 3>> color;
	std::vector<ProjectBody> bodies;
	MassSpec mass;
};

// `mass:` as every scene item wrote it before the bodies carried their
// structure (entity, boolean result); still read as the legacy form.
YAML::Node EncodeMassSpec(const MassSpec& m);
MassSpec DecodeMassSpec(const YAML::Node& node);

YAML::Node Encode(const ProjectBoolean& b);
// `ignored_infill` / `ignored_networks`, when given, collect "<boolean> /
// <body>" for every body whose entry carried a legacy `infill:` or
// `networks:` key (see ProjectFile).
ProjectBoolean DecodeProjectBoolean(const YAML::Node& node, std::vector<std::string>* ignored_infill = nullptr, std::vector<std::string>* ignored_networks = nullptr);

struct ProjectFile {
	// What every length and mass in the file means (units.hpp). Always
	// written; absent on load means metres and kilograms.
	UnitSystem units;
	// The model scale (Model > Scale): the size the vehicle is BUILT at
	// as a fraction of the size it is DRAWN at. The parameterisations and
	// placements below are always the full-scale aircraft; a replica is this
	// times smaller, every solid scaled about the origin and every placement
	// with it (feature_tree.hpp Place), with the materials' skin thicknesses
	// and the structures' build figures as typed. Written when it is not 1;
	// absent on load means 1 (full size). Must be a positive number.
	double scale = 1.0;
	std::vector<ProjectEntity> entities;
	// In build order; every operand names something listed before it.
	std::vector<ProjectBoolean> booleans;
	// The names of the `lattices:` entries of a file from before 2026-11-08,
	// NOT decoded: lattices are no longer supported at all (the structure
	// is shells only -- skins and plane networks), so the loader skips them
	// like any dependency that cannot be satisfied and the application
	// reports them by name. Never written.
	std::vector<std::string> unsupported_lattices;
	// The bodies -- "<entity or boolean> / <body>" -- whose entry carried
	// an `infill:` key (a lattice infill of the interior, which a file from
	// 2026-11-08 could hold): the key is dropped, the rest of the body's
	// structure loads, and the application reports the loss by name. Never
	// written.
	std::vector<std::string> ignored_infill;
	// Likewise the bodies whose entry carried a `networks:` key (plane
	// networks, a file from 2026-11-08 or 09): dropped on load, reported by
	// name; the structure is defined in the parameterisation now.
	std::vector<std::string> ignored_networks;
};

ProjectFile LoadProjectYaml(const std::string& path);
void SaveProjectYaml(const ProjectFile& project, const std::string& path);

} // namespace aerotelic::project_yaml
