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
