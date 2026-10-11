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

#include "aerotelic/feature_tree.hpp"
#include "aerotelic/fuselage_model.hpp"
#include "aerotelic/lattice.hpp"
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
// Lattice likewise: a lattice infill generated in other bodies, saved
// under `lattices:` (ProjectLattice) and regenerated on load.
enum class EntityType { Wing, Fuselage, Nacelle, LoftedNacelle, Propeller, Primitive, Boolean, Lattice };
const char* EntityTypeName(EntityType t);
EntityType ParseEntityType(const std::string& s);

using EntityModelSpec = std::variant<WingModelSpec, FuselageSpec, NacelleSpec, LoftedNacelleSpec, PropellerSpec, PrimitiveSpec>;

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
};

// One operand of a boolean operation, by scene name: an entity, the
// mirrored half of a mirrored entity (`{mirror: <entity name>}` in the
// file), or an earlier boolean result.
struct BooleanOperand {
	std::string name;
	bool mirror = false;
};

// Per-solid display overrides of a boolean result (`bodies:`).
struct ProjectBooleanBody {
	std::string name;
	std::optional<std::array<double, 3>> color;
	bool hidden = false;
};

struct ProjectBoolean {
	std::string name;
	// "fuse" | "cut" | "common"; cut is first minus the rest.
	std::string operation = "fuse";
	// In build order: folded left to right ((A op B) op C).
	std::vector<BooleanOperand> operands;
	std::optional<double> fillet_radius;
	bool hidden = false;
	std::optional<std::array<double, 3>> color;
	std::vector<ProjectBooleanBody> bodies;
};

YAML::Node Encode(const ProjectBoolean& b);
ProjectBoolean DecodeProjectBoolean(const YAML::Node& node);

// A lattice infill (Lattice -> Generate Lattice...): the OpenLattice
// parameters (lattice.hpp), the bodies it fills and the reserved volumes
// it leaves alone, all by scene name like a boolean's operands.
struct ProjectLattice {
	std::string name;
	LatticeSpec spec;
	std::vector<BooleanOperand> targets;
	std::vector<BooleanOperand> exclusions;
	bool hidden = false;
	std::optional<std::array<double, 3>> color;
};

YAML::Node Encode(const LatticeSpec& spec);
LatticeSpec DecodeLatticeSpec(const YAML::Node& node);
YAML::Node Encode(const ProjectLattice& l);
ProjectLattice DecodeProjectLattice(const YAML::Node& node);

struct ProjectFile {
	std::vector<ProjectEntity> entities;
	// In build order; every operand names something listed before it.
	std::vector<ProjectBoolean> booleans;
	// After the booleans; targets name entities or boolean results.
	std::vector<ProjectLattice> lattices;
};

ProjectFile LoadProjectYaml(const std::string& path);
void SaveProjectYaml(const ProjectFile& project, const std::string& path);

} // namespace aerotelic::project_yaml
