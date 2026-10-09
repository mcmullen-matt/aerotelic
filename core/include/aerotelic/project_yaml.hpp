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
// The 'booleans' top-level key (feature-tree booleans) is preserved
// verbatim across a load-then-save round trip but not decoded into any
// typed structure -- there's no UI to create or edit one yet.

#include <optional>
#include <string>
#include <variant>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "aerotelic/feature_tree.hpp"
#include "aerotelic/fuselage_model.hpp"
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

enum class EntityType { Wing, Fuselage, Nacelle, LoftedNacelle, Propeller, Primitive };
const char* EntityTypeName(EntityType t);
EntityType ParseEntityType(const std::string& s);

using EntityModelSpec = std::variant<WingModelSpec, FuselageSpec, NacelleSpec, LoftedNacelleSpec, PropellerSpec, PrimitiveSpec>;

struct ProjectEntity {
	std::string name;
	EntityType type;
	bool mirror = false;
	Placement placement{{0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, 0.0};
	EntityModelSpec model;
};

struct ProjectFile {
	std::vector<ProjectEntity> entities;
	// Preserved verbatim across load-then-save; not decoded (no UI to
	// create/edit booleans yet).
	std::optional<YAML::Node> booleans_raw;
};

ProjectFile LoadProjectYaml(const std::string& path);
void SaveProjectYaml(const ProjectFile& project, const std::string& path);

} // namespace aerotelic::project_yaml
