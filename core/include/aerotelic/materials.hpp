#pragma once

// Structural materials: what a parameterisation is made of (the Scene
// tree's Material Properties dialog), and the handful of presets it
// offers. A material is a property of the stuff, not of anyone's units,
// so every number here is SI -- kg/m^3, Pa -- whatever the project is
// drawn in; the dialog converts for display and the analysis bridges
// hand them on as they are. Picking a preset copies its numbers in and
// keeps its name; editing a number makes the material "Custom".

#include <string>
#include <vector>

namespace aerotelic {

struct Material {
	std::string name = "Custom";
	double density = 0.0;           // kg/m^3
	double youngs_modulus = 0.0;    // Pa
	double poisson_ratio = 0.3;
	// The stress a margin is reported against (yield, or an allowable of
	// the designer's choosing); 0 means no margin is reported.
	double allowable_stress = 0.0;  // Pa

	// Enough to weigh with, and enough to analyse with.
	bool HasDensity() const { return density > 0.0; }
	bool IsStructural() const { return density > 0.0 && youngs_modulus > 0.0 && poisson_ratio >= 0.0 && poisson_ratio < 0.5; }
};

// The presets, in the order the dialog lists them. Typical handbook
// values (room temperature; the allowable is the yield strength, a
// representative quasi-isotropic figure for the laminate), then the
// 3D-printing polymers: the photopolymer resins of SLA / LCD printers
// (post-cured data-sheet figures) and the thermoplastics of FDM printers
// (as printed, along the layers -- a printed part is weaker across them;
// see materials.cpp for the sources and caveats).
const std::vector<Material>& MaterialPresets();
// The preset of that name, or null. The name written to a project file
// is the preset's, so a renamed preset would orphan old files: do not.
const Material* FindMaterialPreset(const std::string& name);
// True when `m` is a preset by name AND by numbers -- an edited preset
// is no longer the preset.
bool IsMaterialPreset(const Material& m);

} // namespace aerotelic
