#include "aerotelic/materials.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace aerotelic {

namespace {

const char* const kMetal = "Metal";
const char* const kComposite = "Composite";
const char* const kWood = "Wood";
const char* const kFoam = "Foam";
const char* const kResin = "SLA resin";
const char* const kFdm = "FDM thermoplastic";

// The conventions every data-sheet preset follows, so each row's source
// can stay short.
const char* const kResinNote = "post-cured ASTM D638 tensile modulus and strength; allowable = tensile yield where the sheet gives one, else strength at break; nu 0.35 (literature, cured acrylates)";
const char* const kAnycubicNote = "data sheet gives ranges, no post-cure, no tensile modulus (store page adds a Young's modulus range): LOWER bound of E and tensile strength, UPPER bound of density; nu 0.35";
const char* const kFdmNote = "as printed, flat (XY/XZ) orientation, dry; allowable = tensile yield where given, else tensile strength; a printed part is weaker across its layers";

} // namespace

const std::vector<Material>& MaterialPresets() {
	// name, density kg/m^3, E Pa, nu, allowable Pa, category, source
	static const std::vector<Material> presets = {
		// Handbook metals: typical room-temperature figures (MMPDS-01 /
		// ASM Handbook vol. 2); the allowable is the tensile yield.
		{"Aluminium 2024-T3", 2780.0, 73.1e9, 0.33, 345.0e6, kMetal,
			"MMPDS-01 / ASM Handbook vol. 2, typical room-temperature values, T3 sheet; allowable = tensile yield"},
		{"Aluminium 7075-T6", 2810.0, 71.7e9, 0.33, 503.0e6, kMetal,
			"MMPDS-01 / ASM Handbook vol. 2, typical room-temperature values, T6 sheet; allowable = tensile yield"},
		{"Aluminium 6061-T6", 2700.0, 68.9e9, 0.33, 276.0e6, kMetal,
			"MMPDS-01 / ASM Handbook vol. 2, typical room-temperature values, T6; allowable = tensile yield"},
		{"Titanium Ti-6Al-4V", 4430.0, 113.8e9, 0.342, 880.0e6, kMetal,
			"MMPDS-01 / ASM Handbook vol. 2, annealed, typical; allowable = tensile yield"},
		{"Steel 4130", 7850.0, 205.0e9, 0.29, 435.0e6, kMetal,
			"ASM Handbook vol. 1, normalised at 870 C, typical; allowable = tensile yield"},
		{"Steel 17-4PH H900", 7800.0, 196.0e9, 0.27, 1170.0e6, kMetal,
			"ASM Handbook vol. 1 / AK Steel 17-4 PH data sheet, condition H900, typical; allowable = tensile yield"},
		{"Magnesium AZ31B", 1770.0, 45.0e9, 0.35, 200.0e6, kMetal,
			"ASM Handbook vol. 2, AZ31B-H24 sheet, typical; allowable = tensile yield"},

		// Laminates: representative quasi-isotropic figures, not any one
		// manufacturer's.
		{"CFRP quasi-isotropic", 1600.0, 55.0e9, 0.30, 500.0e6, kComposite,
			"Representative quasi-isotropic carbon/epoxy laminate (standard-modulus fibre, ~60% fibre volume, [0/±45/90]s), in-plane; allowable = a representative laminate tensile strength"},
		{"GFRP quasi-isotropic", 1900.0, 20.0e9, 0.28, 250.0e6, kComposite,
			"Representative quasi-isotropic E-glass/epoxy laminate (~55% fibre volume), in-plane; allowable = a representative laminate tensile strength"},

		// Woods, along the grain, at 12% moisture: the USDA Wood Handbook's
		// clear-wood figures (Table 5-3 domestic, 5-5 imported), density from
		// its specific gravity (oven-dry weight, volume at 12%: SG x 1120),
		// Poisson's ratio nu_LR from its Table 5-1 where the species is
		// listed. The allowable is the compression strength parallel to the
		// grain -- the lower of the two and what a spar cap buckles at. Balsa
		// comes in grades the hobby trade sells by weight (contest-light,
		// medium, hard); its modulus and strength go roughly as its density
		// (Borrega & Gibson 2015), so the lighter and heavier grades are the
		// Handbook's 180 kg/m^3 figures scaled by density. The two plywoods
		// are the solid wood's figures at two-thirds, the cross ply of a
		// three-ply sheet adding little along the face grain (ANC-18's
		// approach); their nu is an in-plane plywood figure, approximate.
		{"Spruce (along grain)", 450.0, 10.0e9, 0.37, 40.0e6, kWood,
			"USDA Wood Handbook (FPL-GTR-190) Table 5-3, Sitka spruce at 12% moisture, parallel to grain; nu_LR 0.372 (Table 5-1); allowable ~ compression strength parallel to grain"},
		{"Balsa, light (100 kg/m^3)", 100.0, 1.9e9, 0.23, 8.3e6, kWood,
			"USDA Wood Handbook Table 5-5 balsa (SG 0.16, 12% moisture: E 3.4 GPa, compression parallel 14.9 MPa) scaled to a 6 lb/ft^3 contest-light grade in proportion to density after Borrega & Gibson (2015); nu_LR 0.229 (Table 5-1); allowable = compression parallel to grain"},
		{"Balsa, medium (160 kg/m^3)", 160.0, 3.0e9, 0.23, 13.0e6, kWood,
			"USDA Wood Handbook Table 5-5 balsa (SG 0.16, 12% moisture: E 3.4 GPa, compression parallel 14.9 MPa) scaled to a 10 lb/ft^3 medium grade in proportion to density after Borrega & Gibson (2015); nu_LR 0.229 (Table 5-1); allowable = compression parallel to grain"},
		{"Balsa, hard (240 kg/m^3)", 240.0, 4.5e9, 0.23, 20.0e6, kWood,
			"USDA Wood Handbook Table 5-5 balsa (SG 0.16, 12% moisture: E 3.4 GPa, compression parallel 14.9 MPa) scaled to a 15 lb/ft^3 hard grade in proportion to density after Borrega & Gibson (2015); nu_LR 0.229 (Table 5-1); allowable = compression parallel to grain"},
		{"Basswood, American (along grain)", 415.0, 10.1e9, 0.36, 32.6e6, kWood,
			"USDA Wood Handbook Table 5-3, American basswood at 12% moisture (SG 0.37): E 10.1 GPa, MOR 60 MPa; nu_LR 0.364 (Table 5-1); allowable = compression parallel to grain 32.6 MPa"},
		{"Pine, Eastern white (along grain)", 390.0, 8.5e9, 0.33, 33.1e6, kWood,
			"USDA Wood Handbook Table 5-3, Eastern white pine at 12% moisture (SG 0.35): E 8.5 GPa, MOR 59 MPa; nu_LR 0.33 as the western white pine of Table 5-1; allowable = compression parallel to grain 33.1 MPa"},
		{"Yellow-poplar (along grain)", 470.0, 10.9e9, 0.32, 38.2e6, kWood,
			"USDA Wood Handbook Table 5-3, yellow-poplar at 12% moisture (SG 0.42): E 10.9 GPa, MOR 70 MPa; nu_LR 0.318 (Table 5-1); allowable = compression parallel to grain 38.2 MPa"},
		{"Douglas-fir, Coast (along grain)", 540.0, 13.4e9, 0.29, 49.9e6, kWood,
			"USDA Wood Handbook Table 5-3, Douglas-fir (Coast) at 12% moisture (SG 0.48): E 13.4 GPa, MOR 85 MPa; nu_LR 0.292 (Table 5-1); allowable = compression parallel to grain 49.9 MPa"},
		{"Birch plywood, aircraft 3-ply (face grain)", 690.0, 9.3e9, 0.20, 37.5e6, kWood,
			"Yellow birch of USDA Wood Handbook Table 5-3 (12% moisture, SG 0.62: E 13.9 GPa, compression parallel 56.3 MPa) at two-thirds for a three-ply sheet of equal veneers along the face grain (ANC-18 Design of Wood Aircraft Structures); nu 0.20 in-plane, approximate; allowable = two-thirds of the compression parallel to grain"},
		{"Poplar plywood, lite-ply 3-ply (face grain)", 470.0, 7.3e9, 0.20, 25.5e6, kWood,
			"Yellow-poplar of USDA Wood Handbook Table 5-3 (12% moisture, SG 0.42: E 10.9 GPa, compression parallel 38.2 MPa) at two-thirds for a three-ply sheet of equal veneers along the face grain (ANC-18); nu 0.20 in-plane, approximate; allowable = two-thirds of the compression parallel to grain. Hobby lite-ply is often a lighter poplar: weigh a sheet"},
		{"Foam, PMI 71 kg/m^3", 71.0, 0.105e9, 0.30, 1.9e6, kFoam,
			"Evonik ROHACELL 71 IG-F data sheet (nominal 75 kg/m^3 grade): E 105 MPa; allowable between the compressive (1.7 MPa) and tensile (2.8 MPa) strengths"},

		// Photopolymer thermosets of resin (SLA / LCD / DLP) printers: the
		// manufacturers' technical data sheets, POST-CURED figures (ASTM D638
		// tensile modulus and ultimate strength; the allowable is the yield
		// where the sheet gives one, else the strength at break), densities
		// from Formlabs' post-cured density table or the sheet. Poisson's
		// ratio is not on the sheets: 0.35 is the measured figure for cured
		// acrylate resins (Formlabs Clear 0.35, mSLA resins 0.34), 0.30 for
		// the glass-filled one, 0.38 for the soft polypropylene-like one.
		{"SLA resin, standard (Formlabs Grey V4)", 1140.0, 2.8e9, 0.35, 65.0e6, kResin,
			std::string("Formlabs Grey Resin V4 TDS; density from Formlabs' post-cured density table; ") + kResinNote},
		{"SLA resin, clear (Formlabs Clear V4.1)", 1170.0, 2.37e9, 0.35, 53.0e6, kResin,
			std::string("Formlabs Clear Resin V4.1 TDS; density from Formlabs' post-cured density table; ") + kResinNote},
		{"SLA resin, tough ABS-like (Formlabs Tough 2000 V2)", 1090.0, 1.8e9, 0.35, 40.4e6, kResin,
			std::string("Formlabs Tough 2000 Resin V2 TDS; density from Formlabs' post-cured density table; ") + kResinNote},
		{"SLA resin, tough (Siraya Tech Blu)", 1150.0, 1.8e9, 0.35, 51.0e6, kResin,
			std::string("Siraya Tech Blu TDS; ") + kResinNote},
		{"SLA resin, durable PP-like (Formlabs Durable)", 1130.0, 1.26e9, 0.38, 31.8e6, kResin,
			std::string("Formlabs Durable Resin TDS; density from Formlabs' post-cured density table; nu 0.38 for the soft polypropylene-like resin; ") + kResinNote},
		{"SLA resin, high-temp (Formlabs High Temp V2)", 1190.0, 2.8e9, 0.35, 58.3e6, kResin,
			std::string("Formlabs High Temp Resin V2 TDS; density from Formlabs' post-cured density table; ") + kResinNote},
		{"SLA resin, glass-filled (Formlabs Rigid 10K)", 1700.0, 11.0e9, 0.30, 88.0e6, kResin,
			std::string("Formlabs Rigid 10K Resin TDS; nu 0.30 for the glass-filled resin; ") + kResinNote},

		// Anycubic's LCD / DLP resins (2026-11-06). Their data sheets give
		// RANGES, not values, name no post-cure, and list no tensile
		// modulus (their "bending strength" and "bending modulus" rows are
		// labelled the wrong way round); the store pages add a Young's
		// modulus range. Taken conservatively: the LOWER bound of the
		// Young's modulus and of the tensile strength (ASTM D638), the UPPER
		// bound of the density. ABS-Like V2 has no Young's modulus anywhere,
		// so its lower flexural modulus stands in. Poisson's ratio 0.35 as
		// for the other acrylates, 0.38 for the ductile Tough 2.0 and 0.42
		// for Tough Ultra, which at 70-80% elongation and Shore D 71-75 is
		// nearly an elastomer -- treat its linear results with care.
		{"SLA resin, standard (Anycubic Standard)", 1200.0, 1.6e9, 0.35, 40.0e6, kResin,
			std::string("Anycubic Standard Resin TDS and store page; ") + kAnycubicNote},
		{"SLA resin, standard (Anycubic Standard V2)", 1250.0, 1.2e9, 0.35, 35.0e6, kResin,
			std::string("Anycubic Standard Resin V2 TDS and store page; ") + kAnycubicNote},
		{"SLA resin, standard (Anycubic 16K Standard)", 1100.0, 1.8e9, 0.35, 40.0e6, kResin,
			std::string("Anycubic 16K Standard Resin TDS and store page; ") + kAnycubicNote},
		{"SLA resin, ABS-like (Anycubic ABS-Like 2.0)", 1090.0, 1.8e9, 0.35, 34.0e6, kResin,
			std::string("Anycubic ABS-Like Resin 2.0 TDS and store page; ") + kAnycubicNote},
		{"SLA resin, ABS-like (Anycubic ABS-Like V2)", 1200.0, 1.0e9, 0.35, 35.0e6, kResin,
			std::string("Anycubic ABS-Like Resin V2 TDS; no Young's modulus anywhere, so the lower flexural modulus stands in; ") + kAnycubicNote},
		{"SLA resin, ABS-like (Anycubic ABS-Like 3.0)", 1090.0, 1.3e9, 0.35, 38.0e6, kResin,
			std::string("Anycubic ABS-Like Resin 3.0 TDS and store page; ") + kAnycubicNote},
		{"SLA resin, ABS-like (Anycubic ABS-Like Pro 2)", 1250.0, 1.6e9, 0.35, 35.0e6, kResin,
			std::string("Anycubic ABS-Like Resin Pro 2 TDS and store page; ") + kAnycubicNote},
		{"SLA resin, water-wash ABS-like (Anycubic Water-Wash ABS-Like 3.0)", 1100.0, 1.8e9, 0.35, 36.0e6, kResin,
			std::string("Anycubic Water-Wash ABS-Like Resin 3.0 TDS and store page; ") + kAnycubicNote},
		{"SLA resin, water-wash (Anycubic Water-Wash 2.0)", 1090.0, 1.1e9, 0.35, 38.0e6, kResin,
			std::string("Anycubic Water-Wash Resin 2.0 TDS and store page; ") + kAnycubicNote},
		{"SLA resin, tough (Anycubic Tough 2.0)", 1150.0, 0.9e9, 0.38, 30.0e6, kResin,
			std::string("Anycubic Tough Resin 2.0 TDS and store page; nu 0.38 for the ductile resin; ") + kAnycubicNote},
		{"SLA resin, soft tough (Anycubic Tough Ultra)", 1250.0, 0.35e9, 0.42, 20.0e6, kResin,
			std::string("Anycubic Tough Resin Ultra TDS and store page; nu 0.42 -- at 70-80% elongation and Shore D 71-75 nearly an elastomer, so treat linear results with care; ") + kAnycubicNote},
		{"SLA resin, clear (Anycubic High Clear)", 1120.0, 0.7e9, 0.35, 28.0e6, kResin,
			std::string("Anycubic High Clear Resin TDS and store page; ") + kAnycubicNote},
		{"SLA resin, fast (Anycubic High Speed 2.0)", 1200.0, 1.3e9, 0.35, 37.0e6, kResin,
			std::string("Anycubic High Speed Resin 2.0 TDS and store page; ") + kAnycubicNote},
		{"SLA resin, plant-based (Anycubic Plant-Based 2.0)", 1100.0, 1.3e9, 0.35, 35.0e6, kResin,
			std::string("Anycubic Plant-Based Resin 2.0 TDS and store page; ") + kAnycubicNote},
		{"SLA resin, DLP (Anycubic DLP Craftsman)", 1200.0, 1.3e9, 0.35, 36.0e6, kResin,
			std::string("Anycubic DLP Craftsman Resin TDS and store page; ") + kAnycubicNote},
		{"SLA resin, texture (Anycubic 14K Texture)", 1110.0, 1.3e9, 0.35, 35.0e6, kResin,
			std::string("Anycubic 14K Texture Resin TDS and store page; ") + kAnycubicNote},
		{"SLA resin, bio (Anycubic Bio)", 1150.0, 1.2e9, 0.35, 35.0e6, kResin,
			std::string("Anycubic Bio Resin TDS and store page; ") + kAnycubicNote},

		// Thermoplastics of fused-deposition (FDM / FFF) printers: the
		// filament makers' technical data sheets, AS PRINTED, in the flat
		// (XY / XZ) orientation, dry. A printed part is weaker across its
		// layers (Z strength is typically 60-85% of XY on these sheets) and
		// its stiffness depends on infill and walls: these are the figures
		// for a solid part loaded along the layers, an upper bound for the
		// isotropic analysis. The allowable is the tensile yield where the
		// sheet gives one, else the tensile strength. Poisson's ratios are
		// the measured ones for printed parts (PLA 0.36, ABS 0.37, PETG 0.38,
		// nylon 0.39, PC 0.37, PEEK 0.40, PEI 0.36, 0.35 for the fibre-filled).
		{"FDM PLA (Prusament)", 1240.0, 2.3e9, 0.36, 51.0e6, kFdm,
			std::string("Prusament PLA TDS; nu 0.36 (literature, printed PLA); ") + kFdmNote},
		{"FDM PETG (Prusament)", 1270.0, 1.5e9, 0.38, 47.0e6, kFdm,
			std::string("Prusament PETG TDS; nu 0.38 (literature, printed PETG); ") + kFdmNote},
		{"FDM ABS (Bambu Lab)", 1050.0, 2.2e9, 0.37, 33.0e6, kFdm,
			std::string("Bambu Lab ABS TDS; nu 0.37 (literature, printed ABS); ") + kFdmNote},
		{"FDM ASA (Prusament)", 1070.0, 1.6e9, 0.37, 42.0e6, kFdm,
			std::string("Prusament ASA TDS; nu 0.37 (as ABS); ") + kFdmNote},
		{"FDM polycarbonate (Prusament PC Blend)", 1220.0, 1.9e9, 0.37, 63.0e6, kFdm,
			std::string("Prusament PC Blend TDS; nu 0.37 (literature, printed PC); ") + kFdmNote},
		{"FDM nylon PA6/66 (Polymaker CoPA)", 1120.0, 2.22e9, 0.39, 66.2e6, kFdm,
			std::string("Polymaker PolyMide CoPA TDS; nu 0.39 (literature, printed nylon); ") + kFdmNote},
		{"FDM nylon-CF (Bambu Lab PAHT-CF)", 1060.0, 3.86e9, 0.35, 92.0e6, kFdm,
			std::string("Bambu Lab PAHT-CF TDS; nu 0.35 (fibre-filled); ") + kFdmNote},
		{"FDM nylon-CF (Markforged Onyx)", 1200.0, 2.4e9, 0.35, 37.0e6, kFdm,
			std::string("Markforged composites data sheet, Onyx; nu 0.35 (fibre-filled); ") + kFdmNote},
		{"FDM PEI (Stratasys ULTEM 9085)", 1340.0, 2.52e9, 0.36, 69.2e6, kFdm,
			std::string("Stratasys ULTEM 9085 resin data sheet, XZ orientation column; nu 0.36 (literature, printed PEI); ") + kFdmNote},
		{"FDM PEEK (3DXTech ThermaX)", 1300.0, 3.72e9, 0.40, 100.0e6, kFdm,
			std::string("3DXTech ThermaX PEEK TDS; nu 0.40 (literature, PEEK); ") + kFdmNote},
	};
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
	YAML::Node root;
	YAML::Node list(YAML::NodeType::Sequence);
	for (const Material& m : materials) {
		YAML::Node n;
		n["name"] = m.name;
		n["category"] = m.category;
		n["density"] = m.density;
		n["youngs_modulus"] = m.youngs_modulus;
		n["poisson_ratio"] = m.poisson_ratio;
		n["allowable_stress"] = m.allowable_stress;
		n["source"] = m.source;
		list.push_back(n);
	}
	root["materials"] = list;
	YAML::Emitter out;
	// 12 significant digits: 0.3 reads as 0.3, not 0.29999999999999999, and
	// no material figure carries more.
	out.SetDoublePrecision(12);
	out << root;
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
