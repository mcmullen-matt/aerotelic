#include "aerotelic/materials.hpp"

#include <algorithm>
#include <cmath>

namespace aerotelic {

const std::vector<Material>& MaterialPresets() {
	// name, density kg/m^3, E Pa, nu, yield Pa
	static const std::vector<Material> presets = {
		{"Aluminium 2024-T3", 2780.0, 73.1e9, 0.33, 345.0e6},
		{"Aluminium 7075-T6", 2810.0, 71.7e9, 0.33, 503.0e6},
		{"Aluminium 6061-T6", 2700.0, 68.9e9, 0.33, 276.0e6},
		{"Titanium Ti-6Al-4V", 4430.0, 113.8e9, 0.342, 880.0e6},
		{"Steel 4130", 7850.0, 205.0e9, 0.29, 435.0e6},
		{"Steel 17-4PH H900", 7800.0, 196.0e9, 0.27, 1170.0e6},
		{"Magnesium AZ31B", 1770.0, 45.0e9, 0.35, 200.0e6},
		{"CFRP quasi-isotropic", 1600.0, 55.0e9, 0.30, 500.0e6},
		{"GFRP quasi-isotropic", 1900.0, 20.0e9, 0.28, 250.0e6},
		{"Spruce (along grain)", 450.0, 10.0e9, 0.37, 40.0e6},
		{"Foam, PMI 71 kg/m^3", 71.0, 0.105e9, 0.30, 1.9e6},

		// Photopolymer thermosets of resin (SLA / LCD / DLP) printers: the
		// manufacturers' technical data sheets, POST-CURED figures (ASTM D638
		// tensile modulus and ultimate strength; the allowable is the yield
		// where the sheet gives one, else the strength at break), densities
		// from Formlabs' post-cured density table or the sheet. Poisson's
		// ratio is not on the sheets: 0.35 is the measured figure for cured
		// acrylate resins (Formlabs Clear 0.35, mSLA resins 0.34), 0.30 for
		// the glass-filled one, 0.38 for the soft polypropylene-like one.
		{"SLA resin, standard (Formlabs Grey V4)", 1140.0, 2.8e9, 0.35, 65.0e6},
		{"SLA resin, clear (Formlabs Clear V4.1)", 1170.0, 2.37e9, 0.35, 53.0e6},
		{"SLA resin, tough ABS-like (Formlabs Tough 2000 V2)", 1090.0, 1.8e9, 0.35, 40.4e6},
		{"SLA resin, tough (Siraya Tech Blu)", 1150.0, 1.8e9, 0.35, 51.0e6},
		{"SLA resin, durable PP-like (Formlabs Durable)", 1130.0, 1.26e9, 0.38, 31.8e6},
		{"SLA resin, high-temp (Formlabs High Temp V2)", 1190.0, 2.8e9, 0.35, 58.3e6},
		{"SLA resin, glass-filled (Formlabs Rigid 10K)", 1700.0, 11.0e9, 0.30, 88.0e6},

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
		{"FDM PLA (Prusament)", 1240.0, 2.3e9, 0.36, 51.0e6},
		{"FDM PETG (Prusament)", 1270.0, 1.5e9, 0.38, 47.0e6},
		{"FDM ABS (Bambu Lab)", 1050.0, 2.2e9, 0.37, 33.0e6},
		{"FDM ASA (Prusament)", 1070.0, 1.6e9, 0.37, 42.0e6},
		{"FDM polycarbonate (Prusament PC Blend)", 1220.0, 1.9e9, 0.37, 63.0e6},
		{"FDM nylon PA6/66 (Polymaker CoPA)", 1120.0, 2.22e9, 0.39, 66.2e6},
		{"FDM nylon-CF (Bambu Lab PAHT-CF)", 1060.0, 3.86e9, 0.35, 92.0e6},
		{"FDM nylon-CF (Markforged Onyx)", 1200.0, 2.4e9, 0.35, 37.0e6},
		{"FDM PEI (Stratasys ULTEM 9085)", 1340.0, 2.52e9, 0.36, 69.2e6},
		{"FDM PEEK (3DXTech ThermaX)", 1300.0, 3.72e9, 0.40, 100.0e6},
	};
	return presets;
}

const Material* FindMaterialPreset(const std::string& name) {
	for (const Material& m : MaterialPresets()) {
		if (m.name == name) return &m;
	}
	return nullptr;
}

bool IsMaterialPreset(const Material& m) {
	const Material* p = FindMaterialPreset(m.name);
	if (p == nullptr) return false;
	auto same = [](double a, double b) { return std::abs(a - b) <= 1e-9 * std::max(std::abs(a), std::abs(b)) + 1e-300; };
	return same(m.density, p->density) && same(m.youngs_modulus, p->youngs_modulus)
		&& same(m.poisson_ratio, p->poisson_ratio) && same(m.allowable_stress, p->allowable_stress);
}

} // namespace aerotelic
