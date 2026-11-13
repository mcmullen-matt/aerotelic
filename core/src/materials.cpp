#include "aerotelic/materials.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "aerotelic/laminate.hpp"

namespace aerotelic {

namespace {

// The presets are core/data/materials_presets.yaml -- a human-readable
// file whose every row cites a source that may be redistributed publicly
// and commercially: MIL-HDBK-5J (US DoD, public domain) for the metals,
// MIL-HDBK-17-2F (US DoD, Distribution Statement A) for the composite
// laminae, the USDA Wood Handbook and ANC-18 (US Government works) for the
// woods, and no one's figures for the two representative isotropic
// laminates. CMake embeds the file's bytes here at configure time (editing
// the file re-runs configure), so the library needs no file beside it at
// run time.
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

bool Same(double x, double y) { return std::abs(x - y) <= 1e-9 * std::max(std::abs(x), std::abs(y)) + 1e-300; }

bool Blank(const std::string& s) { return s.find_first_not_of(" \t\r\n") == std::string::npos; }

double Num(const YAML::Node& n, const char* key, double fallback = 0.0) {
	return n[key] ? n[key].as<double>() : fallback;
}

Lamina ReadLamina(const YAML::Node& n) {
	Lamina l;
	if (n["form"]) l.form = n["form"].as<std::string>();
	l.ply_thickness = Num(n, "ply_thickness");
	l.e1 = Num(n, "e1"); l.e2 = Num(n, "e2"); l.nu12 = Num(n, "nu12");
	l.g12 = Num(n, "g12"); l.g13 = Num(n, "g13"); l.g23 = Num(n, "g23");
	l.t1 = Num(n, "t1"); l.c1 = Num(n, "c1"); l.t2 = Num(n, "t2"); l.c2 = Num(n, "c2");
	l.s12 = Num(n, "s12"); l.s13 = Num(n, "s13"); l.s23 = Num(n, "s23");
	l.locking_angle = Num(n, "locking_angle");
	return l;
}

void WriteLamina(YAML::Emitter& out, const Lamina& l) {
	out << YAML::BeginMap;
	out << YAML::Key << "form" << YAML::Value << l.form;
	out << YAML::Key << "ply_thickness" << YAML::Value << l.ply_thickness;
	out << YAML::Key << "e1" << YAML::Value << l.e1;
	out << YAML::Key << "e2" << YAML::Value << l.e2;
	out << YAML::Key << "nu12" << YAML::Value << l.nu12;
	out << YAML::Key << "g12" << YAML::Value << l.g12;
	if (l.g13 > 0.0) out << YAML::Key << "g13" << YAML::Value << l.g13;
	if (l.g23 > 0.0) out << YAML::Key << "g23" << YAML::Value << l.g23;
	out << YAML::Key << "t1" << YAML::Value << l.t1;
	out << YAML::Key << "c1" << YAML::Value << l.c1;
	out << YAML::Key << "t2" << YAML::Value << l.t2;
	out << YAML::Key << "c2" << YAML::Value << l.c2;
	out << YAML::Key << "s12" << YAML::Value << l.s12;
	if (l.s13 > 0.0) out << YAML::Key << "s13" << YAML::Value << l.s13;
	if (l.s23 > 0.0) out << YAML::Key << "s23" << YAML::Value << l.s23;
	if (l.locking_angle > 0.0) out << YAML::Key << "locking_angle" << YAML::Value << l.locking_angle;
	out << YAML::EndMap;
}

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

bool SameLamina(const Lamina& a, const Lamina& b) {
	return a.form == b.form && Same(a.ply_thickness, b.ply_thickness) && Same(a.e1, b.e1) && Same(a.e2, b.e2) && Same(a.nu12, b.nu12)
		&& Same(a.g12, b.g12) && Same(a.g13, b.g13) && Same(a.g23, b.g23) && Same(a.t1, b.t1) && Same(a.c1, b.c1) && Same(a.t2, b.t2)
		&& Same(a.c2, b.c2) && Same(a.s12, b.s12) && Same(a.s13, b.s13) && Same(a.s23, b.s23) && Same(a.locking_angle, b.locking_angle);
}

bool SameStack(const Material& a, const Material& b) {
	if (a.plies.size() != b.plies.size() || a.symmetric != b.symmetric || a.symmetric_odd != b.symmetric_odd) return false;
	for (size_t i = 0; i < a.plies.size(); ++i) {
		const Ply& p = a.plies[i];
		const Ply& q = b.plies[i];
		if (!Same(p.angle, q.angle) || !Same(p.Thickness(), q.Thickness()) || p.lamina != q.lamina || !Same(p.density, q.density) || !SameLamina(p.props, q.props)) return false;
	}
	return true;
}

bool SameMaterial(const Material& a, const Material& b) {
	if (a.name != b.name) return false;
	if (a.IsLamina() != b.IsLamina() || a.IsLaminate() != b.IsLaminate()) return false;
	if (a.IsLamina()) return Same(a.density, b.density) && SameLamina(*a.lamina, *b.lamina);
	if (a.IsLaminate()) return SameStack(a, b);
	return Same(a.density, b.density) && Same(a.youngs_modulus, b.youngs_modulus)
		&& Same(a.poisson_ratio, b.poisson_ratio) && Same(a.allowable_stress, b.allowable_stress);
}

bool IsMaterialPreset(const Material& m) {
	const Material* p = FindMaterialPreset(m.name);
	return p != nullptr && SameMaterial(m, *p);
}

std::string LaminaProblem(const Lamina& l) {
	if (l.form != "UD" && l.form != "Woven") return "the form must be UD or Woven";
	if (!std::isfinite(l.ply_thickness) || l.ply_thickness <= 0.0) return "the ply thickness must be positive";
	if (!std::isfinite(l.e1) || l.e1 <= 0.0) return "E1 must be positive";
	if (!std::isfinite(l.e2) || l.e2 <= 0.0) return "E2 must be positive";
	if (!std::isfinite(l.g12) || l.g12 <= 0.0) return "G12 must be positive";
	if (!std::isfinite(l.nu12) || l.nu12 < 0.0) return "nu12 cannot be negative";
	// Thermodynamic admissibility of an orthotropic lamina: nu12 nu21 < 1.
	if (!(1.0 - l.nu12 * l.nu12 * l.e2 / l.e1 > 0.0)) return "nu12 is too large for E2 / E1 (nu12^2 E2 / E1 must be below 1)";
	for (double v : {l.g13, l.g23, l.t1, l.c1, l.t2, l.c2, l.s12, l.s13, l.s23, l.locking_angle}) {
		if (!std::isfinite(v) || v < 0.0) return "a strength, a transverse shear modulus or the locking angle is negative";
	}
	const bool any = l.t1 > 0.0 || l.c1 > 0.0 || l.t2 > 0.0 || l.c2 > 0.0 || l.s12 > 0.0;
	if (any && !l.HasStrengths()) return "give all five of T1, C1, T2, C2 and S12, or none (no margin is then reported)";
	return std::string();
}

std::string MaterialProblem(const Material& m) {
	if (Blank(m.name)) return "the material has no name";
	if (!std::isfinite(m.density) || m.density <= 0.0) return "the density must be positive";
	if (m.IsLamina()) return LaminaProblem(*m.lamina);
	if (m.IsLaminate()) return LaminateProblem(m);
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
		if (m.IsLamina()) {
			out << YAML::Key << "lamina" << YAML::Value;
			WriteLamina(out, *m.lamina);
		} else {
			// A laminate's four figures are its effective constants, written
			// for the reader's sake and recomputed from the plies on reading.
			out << YAML::Key << "youngs_modulus" << YAML::Value << m.youngs_modulus;
			out << YAML::Key << "poisson_ratio" << YAML::Value << m.poisson_ratio;
			out << YAML::Key << "allowable_stress" << YAML::Value << m.allowable_stress;
		}
		if (m.IsLaminate()) {
			out << YAML::Key << "stacking" << YAML::Value << StackingNotation(m);
			out << YAML::Key << "symmetric" << YAML::Value << m.symmetric;
			if (m.symmetric_odd) out << YAML::Key << "symmetric_odd" << YAML::Value << true;
			out << YAML::Key << "plies" << YAML::Value << YAML::BeginSeq;
			for (const Ply& p : m.plies) {
				out << YAML::Flow << YAML::BeginMap;
				out << YAML::Key << "angle" << YAML::Value << p.angle;
				if (p.thickness > 0.0) out << YAML::Key << "thickness" << YAML::Value << p.thickness;
				out << YAML::Key << "lamina" << YAML::Value << p.lamina;
				out << YAML::EndMap;
			}
			out << YAML::EndSeq;
		}
		out << YAML::Key << "source" << YAML::Value << m.source;
		out << YAML::EndMap;
	}
	out << YAML::EndSeq << YAML::EndMap;
	return std::string("# Aerotelic material library (Model > Material Properties). SI: kg/m^3, Pa, m.\n") + out.c_str() + "\n";
}

std::vector<Material> ReadMaterialsYaml(const std::string& text, const std::vector<Material>* extra_laminae) {
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
	// The plies name their laminae; resolved once every row is read, so a
	// laminate may precede its lamina in the file.
	struct PendingPly { size_t material; size_t ply; std::string lamina; };
	std::vector<PendingPly> pending;
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
			if (const YAML::Node l = n["lamina"]; l && l.IsMap()) {
				m.lamina = ReadLamina(l);
				m.youngs_modulus = 0.0;
				m.poisson_ratio = 0.0;
				m.allowable_stress = 0.0;
				if (m.category.empty()) m.category = "Lamina";
			}
			if (const YAML::Node plies = n["plies"]; plies && plies.IsSequence()) {
				if (n["symmetric"]) m.symmetric = n["symmetric"].as<bool>();
				if (n["symmetric_odd"]) m.symmetric_odd = n["symmetric_odd"].as<bool>();
				for (const YAML::Node& pn : plies) {
					if (!pn.IsMap() || !pn["lamina"]) throw std::runtime_error("material library: a ply of '" + m.name + "' names no lamina");
					Ply p;
					p.angle = Num(pn, "angle");
					p.thickness = Num(pn, "thickness");
					p.lamina = pn["lamina"].as<std::string>();
					// A ply may carry the lamina's figures itself (a project
					// file's copy); a library file names it and is resolved.
					if (const YAML::Node lp = pn["props"]; lp && lp.IsMap()) {
						p.props = ReadLamina(lp);
						p.density = Num(pn, "density");
					} else {
						pending.push_back({out.size(), m.plies.size(), p.lamina});
					}
					m.plies.push_back(p);
				}
				if (m.plies.empty()) throw std::runtime_error("material library: '" + m.name + "' has an empty plies list");
				m.lamina.reset();
				if (m.category.empty()) m.category = "Laminate";
			}
			out.push_back(m);
		}
	} catch (const YAML::Exception& e) {
		throw std::runtime_error(std::string("material library: ") + e.what());
	}
	auto find_lamina = [&](const std::string& name) -> const Material* {
		for (const Material& m : out) if (m.IsLamina() && m.name == name) return &m;
		if (extra_laminae) for (const Material& m : *extra_laminae) if (m.IsLamina() && m.name == name) return &m;
		return nullptr;
	};
	for (const PendingPly& pp : pending) {
		const Material* l = find_lamina(pp.lamina);
		if (l == nullptr) throw std::runtime_error("material library: '" + out[pp.material].name + "' uses the lamina '" + pp.lamina + "', which neither the file nor the presets have");
		Ply& p = out[pp.material].plies[pp.ply];
		p.props = *l->lamina;
		p.density = l->density;
	}
	for (Material& m : out) if (m.IsLaminate()) RefreshLaminate(m);
	return out;
}

} // namespace aerotelic
