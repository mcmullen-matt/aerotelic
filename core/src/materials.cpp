#include "aerotelic/materials.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace aerotelic {

namespace {

// The presets are core/data/materials_presets.yaml -- a human-readable
// file whose every row cites a source that may be redistributed publicly
// and commercially: MIL-HDBK-5J (US DoD, public domain) for the metals,
// the USDA Wood Handbook and ANC-18 (US Government works) for the woods,
// and no one's figures for the two representative laminates. CMake embeds
// the file's bytes here at configure time (editing the file re-runs
// configure), so the library needs no file beside it at run time.
//
// Materials whose sources' terms restrict redistribution -- the ASM
// Handbook figures the first seven metal rows carried, and every
// manufacturer data sheet (the resins, the filaments, the PMI foam, AK
// Steel's 17-4PH) -- are NOT presets any more. They are kept in
// core/data/materials_restricted.yaml, which is not embedded, and a user
// who holds them keeps them in their own library file (Model > Material
// Properties, Custom tab), where they are read like any material of
// theirs. The split was done on 2026-11-07; the preset names in project
// files saved before it still resolve, since an entity carries a copy of
// its material.
const unsigned char kPresetsYaml[] = {
#include "materials_presets_yaml.inc"
};

} // namespace

const std::vector<Material>& MaterialPresets() {
	static const std::vector<Material> presets =
		ReadMaterialsYaml(std::string(reinterpret_cast<const char*>(kPresetsYaml), sizeof(kPresetsYaml)));
	return presets;
}

const Material* FindMaterialPreset(const std::string& name) {
	for (const Material& m : MaterialPresets()) {
		if (m.name == name) return &m;
	}
	return nullptr;
}

bool SameMaterial(const Material& a, const Material& b) {
	auto same = [](double x, double y) { return std::abs(x - y) <= 1e-9 * std::max(std::abs(x), std::abs(y)) + 1e-300; };
	return a.name == b.name && same(a.density, b.density) && same(a.youngs_modulus, b.youngs_modulus)
		&& same(a.poisson_ratio, b.poisson_ratio) && same(a.allowable_stress, b.allowable_stress);
}

bool IsMaterialPreset(const Material& m) {
	const Material* p = FindMaterialPreset(m.name);
	return p != nullptr && SameMaterial(m, *p);
}

std::string MaterialProblem(const Material& m) {
	auto blank = [](const std::string& s) { return s.find_first_not_of(" \t\r\n") == std::string::npos; };
	if (blank(m.name)) return "the material has no name";
	if (!std::isfinite(m.density) || m.density <= 0.0) return "the density must be positive";
	if (!std::isfinite(m.youngs_modulus) || m.youngs_modulus <= 0.0) return "Young's modulus must be positive";
	if (!std::isfinite(m.poisson_ratio) || m.poisson_ratio < 0.0 || m.poisson_ratio >= 0.5) return "Poisson's ratio must lie in [0, 0.5)";
	if (!std::isfinite(m.allowable_stress) || m.allowable_stress < 0.0) return "the allowable stress cannot be negative (0 reports no margin)";
	return std::string();
}

std::string WriteMaterialsYaml(const std::vector<Material>& materials) {
	YAML::Emitter out;
	// 12 significant digits: 0.33 reads as 0.33, not 0.33000000000000002,
	// and no material figure carries more. The figures go through the
	// emitter directly: a double assigned to a YAML::Node is stringified
	// at full precision on assignment, and the emitter's precision would
	// never see it.
	out.SetDoublePrecision(12);
	out << YAML::BeginMap << YAML::Key << "materials" << YAML::Value << YAML::BeginSeq;
	for (const Material& m : materials) {
		out << YAML::BeginMap;
		out << YAML::Key << "name" << YAML::Value << m.name;
		out << YAML::Key << "category" << YAML::Value << m.category;
		out << YAML::Key << "density" << YAML::Value << m.density;
		out << YAML::Key << "youngs_modulus" << YAML::Value << m.youngs_modulus;
		out << YAML::Key << "poisson_ratio" << YAML::Value << m.poisson_ratio;
		out << YAML::Key << "allowable_stress" << YAML::Value << m.allowable_stress;
		out << YAML::Key << "source" << YAML::Value << m.source;
		out << YAML::EndMap;
	}
	out << YAML::EndSeq << YAML::EndMap;
	return std::string("# Aerotelic material library (Model > Material Properties). SI: kg/m^3, Pa.\n") + out.c_str() + "\n";
}

std::vector<Material> ReadMaterialsYaml(const std::string& text) {
	std::vector<Material> out;
	if (text.find_first_not_of(" \t\r\n") == std::string::npos) return out;
	YAML::Node root;
	try {
		root = YAML::Load(text);
	} catch (const YAML::Exception& e) {
		throw std::runtime_error(std::string("material library: ") + e.what());
	}
	if (!root.IsMap()) throw std::runtime_error("material library: the file is not a mapping with a 'materials' list");
	const YAML::Node list = root["materials"];
	if (!list) return out;
	if (!list.IsSequence()) throw std::runtime_error("material library: 'materials' is not a list");
	try {
		for (const YAML::Node& n : list) {
			if (!n.IsMap() || !n["name"]) throw std::runtime_error("material library: a material has no name");
			Material m;
			m.name = n["name"].as<std::string>();
			if (n["category"]) m.category = n["category"].as<std::string>();
			if (n["density"]) m.density = n["density"].as<double>();
			if (n["youngs_modulus"]) m.youngs_modulus = n["youngs_modulus"].as<double>();
			if (n["poisson_ratio"]) m.poisson_ratio = n["poisson_ratio"].as<double>();
			if (n["allowable_stress"]) m.allowable_stress = n["allowable_stress"].as<double>();
			if (n["source"]) m.source = n["source"].as<std::string>();
			out.push_back(m);
		}
	} catch (const YAML::Exception& e) {
		throw std::runtime_error(std::string("material library: ") + e.what());
	}
	return out;
}

} // namespace aerotelic
