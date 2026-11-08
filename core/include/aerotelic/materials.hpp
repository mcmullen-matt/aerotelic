#pragma once

// Structural materials: what a parameterisation is made of (the Scene
// tree's Material Properties dialog picks one), the presets the
// application ships with, and the user's own library (Model > Material
// Properties). A material is a property of the stuff, not of anyone's
// units, so every number here is SI -- kg/m^3, Pa -- whatever the project
// is drawn in; the dialogs convert for display and the analysis bridges
// hand them on as they are.

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
	// The family it is listed under in the materials table: "Metal",
	// "Composite", "Wood", "Foam", "SLA resin", "FDM thermoplastic", or
	// whatever the user types for one of their own.
	std::string category;
	// Where the figures came from and the convention they follow -- the
	// handbook or data sheet, the condition, what the allowable is -- shown
	// in the table's Source column and cited in the Help tab. Empty for a
	// material typed in without one.
	std::string source;

	// Enough to weigh with, and enough to analyse with.
	bool HasDensity() const { return density > 0.0; }
	bool IsStructural() const { return density > 0.0 && youngs_modulus > 0.0 && poisson_ratio >= 0.0 && poisson_ratio < 0.5; }
};

// The presets, in the order the dialog lists them: the rows of
// core/data/materials_presets.yaml (embedded at build time), every one
// from a source that may be redistributed -- MIL-HDBK-5J design values for
// the metals (the allowable is the handbook's design-minimum yield), the
// USDA Wood Handbook and ANC-18 for the woods (the allowable is the
// compression strength along the grain), representative figures for the
// two laminates. Each row's `source` says which table and convention.
// Materials from sources whose terms restrict redistribution (ASM
// Handbook figures, manufacturers' data sheets) are not presets: see
// core/data/materials_restricted.yaml and materials.cpp.
const std::vector<Material>& MaterialPresets();
// The preset of that name, or null. The name written to a project file
// is the preset's, so a renamed preset would orphan old files: do not.
const Material* FindMaterialPreset(const std::string& name);
// True when `m` is a preset by name AND by numbers -- an edited preset
// is no longer the preset. The category and source do not count.
bool IsMaterialPreset(const Material& m);
// True when the two carry the same name and the same four figures.
bool SameMaterial(const Material& a, const Material& b);

// Why `m` cannot be a material of the library -- an empty name, no
// density, no modulus, a Poisson's ratio outside [0, 0.5), a negative
// allowable -- or empty when it can.
std::string MaterialProblem(const Material& m);

// The user's material library file (Model > Material Properties, Custom
// tab): `materials:` a list of {name, category, density, youngs_modulus,
// poisson_ratio, allowable_stress, source}, SI. Write gives the whole
// file's text; Read parses it, an empty text being an empty library, and
// throws std::runtime_error on a malformed one (a material without a
// name, a figure that is not a number).
std::string WriteMaterialsYaml(const std::vector<Material>& materials);
std::vector<Material> ReadMaterialsYaml(const std::string& text);

} // namespace aerotelic
