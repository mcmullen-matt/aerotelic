#pragma once

// The project's units (Geometry -> Units). Every number the user types
// into a parameterisation is unit-agnostic to the geometry: a wing with a
// span of 12 is the same shape in feet or metres. What the units decide
// is what those numbers MEAN when they leave the geometry -- for a mass
// report, a STEP file, or an analysis code that computes forces -- and
// the rule is that nothing is converted on the way in or when the setting
// changes (every typed number keeps its value and means the new unit from
// then on), while everything handed to an analysis code is converted to
// SI first. Time is seconds, always; a rotational speed given in rev/min
// is its own unit with its own conversion where it is consumed.
//
// Length and mass are chosen separately (mm + kg is how CAD works, ft +
// lbm how US aero does), and every compound unit -- density, speed,
// pressure, inertia -- is derived from the two rather than chosen on its
// own, so the system can never be inconsistent. Saved with the project as
// `units: {length: m, mass: kg, time: s}`; a file without the key is
// metres and kilograms, which is what every project before the setting
// existed was taken to be.

#include <string>

namespace aerotelic {

enum class LengthUnit { Metre, Millimetre, Foot };
enum class MassUnit { Kilogram, PoundMass };

const char* LengthUnitName(LengthUnit u);   // "m", "mm", "ft"
const char* MassUnitName(MassUnit u);       // "kg", "lbm"
// Case-insensitive, with the everyday spellings ("meters", "feet",
// "pounds", ...). Throws std::invalid_argument for anything else.
LengthUnit ParseLengthUnit(const std::string& s);
MassUnit ParseMassUnit(const std::string& s);
// Seconds is the only time unit; this accepts its spellings and throws for
// any other, so a file that claims minutes is refused rather than misread.
void CheckTimeUnit(const std::string& s);

struct UnitSystem {
	LengthUnit length = LengthUnit::Metre;
	MassUnit mass = MassUnit::Kilogram;

	// Multiply a number in project units by these to get SI. Exact by
	// definition: 1 ft = 0.3048 m, 1 lbm = 0.45359237 kg.
	double LengthToSI() const;
	double MassToSI() const;
	double AreaToSI() const { const double l = LengthToSI(); return l * l; }
	double VolumeToSI() const { const double l = LengthToSI(); return l * l * l; }
	double SpeedToSI() const { return LengthToSI(); }
	double DensityToSI() const { return MassToSI() / VolumeToSI(); }
	// Pressure is mass / (length s^2) and dynamic viscosity mass / (length s):
	// both scale as mass / length.
	double PressureToSI() const { return MassToSI() / LengthToSI(); }
	double ViscosityToSI() const { return MassToSI() / LengthToSI(); }
	double InertiaToSI() const { return MassToSI() * AreaToSI(); }

	const char* LengthName() const { return LengthUnitName(length); }
	const char* MassName() const { return MassUnitName(mass); }
	std::string AreaName() const { return std::string(LengthName()) + "^2"; }
	std::string VolumeName() const { return std::string(LengthName()) + "^3"; }
	std::string SpeedName() const { return std::string(LengthName()) + "/s"; }
	std::string DensityName() const { return std::string(MassName()) + "/" + LengthName() + "^3"; }
	std::string InertiaName() const { return std::string(MassName()) + " " + LengthName() + "^2"; }
	// "m, kg, s": the status bar and every report header.
	std::string Describe() const { return std::string(LengthName()) + ", " + MassName() + ", s"; }
	// What OCCT's STEP writer wants in write.step.unit.
	const char* StepUnitName() const;

	bool operator==(const UnitSystem& o) const { return length == o.length && mass == o.mass; }
	bool operator!=(const UnitSystem& o) const { return !(*this == o); }
};

} // namespace aerotelic
