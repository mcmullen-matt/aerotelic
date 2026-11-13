#pragma once

// Structural materials: what a parameterisation is made of (the Scene
// tree's Material Properties dialog picks one), the presets the
// application ships with, and the user's own library (Model > Material
// Properties). A material is a property of the stuff, not of anyone's
// units, so every number here is SI -- kg/m^3, Pa -- whatever the project
// is drawn in; the dialogs convert for display and the analysis bridges
// hand them on as they are.

#include <optional>
#include <string>
#include <vector>

namespace aerotelic {

// An orthotropic LAMINA (2026-11-12): the material of ONE PLY of a
// composite laminate -- a unidirectional tape or a woven cloth -- in the
// ply's own axes: 1 along the fibre (a cloth's warp), 2 across it in the
// plane, 3 through the thickness. SI. A lamina is not assigned to a shell
// by itself; a LAMINATE (Material::plies) stacks plies of laminae.
struct Lamina {
	// "UD" -- a unidirectional tape, which cannot shear in its plane when
	// draped -- or "Woven" -- a cloth, which shears at its crossovers up to
	// its locking angle (drape.hpp).
	std::string form = "UD";
	double ply_thickness = 0.0;   // m, the nominal cured ply thickness
	double e1 = 0.0, e2 = 0.0;    // Pa
	double nu12 = 0.0;            // the major Poisson's ratio (-eps2 / eps1 under sigma1)
	double g12 = 0.0;             // Pa, in-plane shear
	double g13 = 0.0, g23 = 0.0;  // Pa, transverse shear (0: g13 = g12, g23 = g12 / 2)
	// Strengths, Pa, as Tsai-Wu wants them: tension and compression along
	// the fibre (t1, c1) and across (t2, c2), in-plane shear s12, and the
	// transverse shears s13, s23 (0: s12). All of t1, c1, t2, c2 and s12
	// positive for a margin to be reported; 0 means unknown.
	double t1 = 0.0, c1 = 0.0, t2 = 0.0, c2 = 0.0, s12 = 0.0, s13 = 0.0, s23 = 0.0;
	// A cloth's shear LOCKING ANGLE, degrees: the in-plane shear beyond
	// which it wrinkles rather than drapes (plain weaves 30-40, satins
	// 50-60). 0: none known (a tape, or not given).
	double locking_angle = 0.0;

	bool HasStrengths() const { return t1 > 0.0 && c1 > 0.0 && t2 > 0.0 && c2 > 0.0 && s12 > 0.0; }
	bool IsWoven() const { return form == "Woven"; }
	double G13() const { return g13 > 0.0 ? g13 : g12; }
	double G23() const { return g23 > 0.0 ? g23 : 0.5 * g12; }
	double S13() const { return s13 > 0.0 ? s13 : s12; }
	double S23() const { return s23 > 0.0 ? s23 : s12; }
};

// One ply of a laminate: its angle from the laminate's reference
// direction (degrees, counter-clockwise seen from the outer face -- about
// the outward normal), its thickness and the lamina it is cut from. The
// lamina's figures are COPIED into the ply, so an entity's material and a
// project file stand on their own, as every material does.
struct Ply {
	double angle = 0.0;
	double thickness = 0.0;  // m; 0 means the lamina's nominal ply thickness
	std::string lamina;      // the lamina material's name
	double density = 0.0;    // the lamina's, kg/m^3
	Lamina props;            // the lamina's figures

	double Thickness() const { return thickness > 0.0 ? thickness : props.ply_thickness; }
};

struct Material {
	std::string name = "Custom";
	double density = 0.0;           // kg/m^3
	double youngs_modulus = 0.0;    // Pa
	double poisson_ratio = 0.3;
	// The stress a margin is reported against (yield, or an allowable of
	// the designer's choosing); 0 means no margin is reported.
	double allowable_stress = 0.0;  // Pa
	// The family it is listed under in the materials table: "Metal",
	// "Composite", "Wood", "Foam", "SLA resin", "FDM thermoplastic",
	// "Lamina", "Laminate", or whatever the user types for one of their own.
	std::string category;
	// Where the figures came from and the convention they follow -- the
	// handbook or data sheet, the condition, what the allowable is -- shown
	// in the table's Source column and cited in the Help tab. Empty for a
	// material typed in without one.
	std::string source;

	// ---- composites (2026-11-12) ----
	// Set: this row is a LAMINA, the orthotropic material of one ply
	// (above). Its density is `density`; youngs_modulus / poisson_ratio /
	// allowable_stress are not used (0), and it cannot be assigned to a
	// shell on its own -- IsStructural() is false.
	std::optional<Lamina> lamina;
	// Non-empty: this row is a LAMINATE, a stack of plies. `symmetric` says
	// the plies listed are the OUTER HALF of the stack, from the outer face
	// to the midplane, and the inner half mirrors them (`symmetric_odd`:
	// the last listed ply is the single centre ply, not mirrored). The
	// isotropic figures then hold the stack's EFFECTIVE in-plane constants
	// from classical lamination theory (laminate.hpp, RefreshLaminate):
	// density the thickness-weighted mean, youngs_modulus the modulus along
	// the reference direction with the curvatures free, poisson_ratio
	// nu_xy, allowable_stress the first-ply-failure tensile strength along
	// the reference direction by Tsai-Wu (0 when a lamina lacks strengths)
	// -- so every consumer that weighs or sorts by them keeps working; the
	// analysis itself gets the plies (structural_fea.py) and reports a
	// Tsai-Wu reserve factor rather than a von Mises margin.
	std::vector<Ply> plies;
	bool symmetric = false;
	bool symmetric_odd = false;

	bool IsLamina() const { return lamina.has_value() && plies.empty(); }
	bool IsLaminate() const { return !plies.empty(); }
	// Enough to weigh with, and enough to analyse with.
	bool HasDensity() const { return density > 0.0; }
	bool IsStructural() const { return !IsLamina() && density > 0.0 && youngs_modulus > 0.0 && poisson_ratio >= 0.0 && poisson_ratio < 0.5; }
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
// True when the two carry the same name and the same four figures -- and,
// for a lamina, the same lamina figures; for a laminate, the same stack
// (angles, thicknesses, lamina names and figures, symmetry).
bool SameMaterial(const Material& a, const Material& b);
bool SameLamina(const Lamina& a, const Lamina& b);
bool SameStack(const Material& a, const Material& b);

// Why `m` cannot be a material of the library -- an empty name, no
// density, no modulus, a Poisson's ratio outside [0, 0.5), a negative
// allowable; for a lamina, a missing modulus or ply thickness; for a
// laminate, a ply without a lamina or a thickness -- or empty when it can.
std::string MaterialProblem(const Material& m);
std::string LaminaProblem(const Lamina& l);

// The user's material library file (Model > Material Properties, Custom
// tab): `materials:` a list of {name, category, density, youngs_modulus,
// poisson_ratio, allowable_stress, source}, SI; a lamina row adds
// `lamina: {form, ply_thickness, e1, e2, nu12, g12, g13, g23, t1, c1, t2,
// c2, s12, s13, s23, locking_angle}`; a laminate row adds `plies: [{angle,
// thickness, lamina}]` (the lamina by NAME, resolved against the laminae
// of the same file and the presets on reading, and the stack's effective
// figures recomputed), `symmetric`, `symmetric_odd`. Write gives the whole
// file's text; Read parses it, an empty text being an empty library, and
// throws std::runtime_error on a malformed one (a material without a
// name, a figure that is not a number, a ply naming a lamina the file and
// the presets do not have). `extra_laminae`, when given, are looked up
// after the file's own (the presets, for a user's file).
std::string WriteMaterialsYaml(const std::vector<Material>& materials);
std::vector<Material> ReadMaterialsYaml(const std::string& text, const std::vector<Material>* extra_laminae = nullptr);

} // namespace aerotelic
