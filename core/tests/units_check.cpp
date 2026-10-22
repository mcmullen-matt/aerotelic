// Checks units.hpp: the factors are the defined constants, every compound
// factor follows from the two base ones, the names parse both ways, and
// the only time unit is the second.

#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

#include "aerotelic/units.hpp"

using namespace aerotelic;

namespace {

int g_pass = 0, g_fail = 0;

void Check(bool cond, const std::string& msg) {
	if (cond) { g_pass++; return; }
	g_fail++;
	std::cout << "FAIL: " << msg << "\n";
}

void CheckClose(double a, double b, double tol, const std::string& msg) {
	Check(std::abs(a - b) <= tol, msg + " (" + std::to_string(a) + " vs " + std::to_string(b) + ")");
}

bool Throws(const std::function<void()>& f) {
	try { f(); } catch (const std::invalid_argument&) { return true; }
	return false;
}

} // namespace

int main() {
	try {
		UnitSystem si;
		Check(si.length == LengthUnit::Metre && si.mass == MassUnit::Kilogram, "the default is metres and kilograms");
		CheckClose(si.LengthToSI(), 1.0, 0.0, "m -> m is 1");
		CheckClose(si.DensityToSI(), 1.0, 0.0, "kg/m^3 -> SI is 1");
		Check(si.Describe() == "m, kg, s", "SI describes as m, kg, s");
		Check(std::string(si.StepUnitName()) == "M", "SI writes STEP in M");

		UnitSystem imperial;
		imperial.length = LengthUnit::Foot;
		imperial.mass = MassUnit::PoundMass;
		CheckClose(imperial.LengthToSI(), 0.3048, 0.0, "a foot is 0.3048 m exactly");
		CheckClose(imperial.MassToSI(), 0.45359237, 0.0, "a pound-mass is 0.45359237 kg exactly");
		CheckClose(imperial.AreaToSI(), 0.3048 * 0.3048, 1e-15, "ft^2");
		CheckClose(imperial.VolumeToSI(), 0.3048 * 0.3048 * 0.3048, 1e-15, "ft^3");
		CheckClose(imperial.SpeedToSI(), 0.3048, 0.0, "ft/s -> m/s");
		CheckClose(imperial.DensityToSI(), 16.018463373960138, 1e-12, "lbm/ft^3 -> kg/m^3 is 16.018");
		CheckClose(imperial.PressureToSI(), 0.45359237 / 0.3048, 1e-15, "pressure scales as mass/length");
		CheckClose(imperial.ViscosityToSI(), 0.45359237 / 0.3048, 1e-15, "viscosity scales as mass/length");
		CheckClose(imperial.InertiaToSI(), 0.45359237 * 0.3048 * 0.3048, 1e-15, "lbm ft^2 -> kg m^2");
		Check(imperial.DensityName() == "lbm/ft^3" && imperial.InertiaName() == "lbm ft^2" && imperial.SpeedName() == "ft/s",
			"compound names follow the base units");
		Check(imperial.Describe() == "ft, lbm, s", "imperial describes as ft, lbm, s");
		Check(std::string(imperial.StepUnitName()) == "FT", "feet write STEP in FT");

		// Air at sea level, the number the flow dialogs will show in each system.
		CheckClose(1.225 / imperial.DensityToSI(), 0.076474, 1e-6, "1.225 kg/m^3 is 0.0765 lbm/ft^3");
		CheckClose(340.0 / imperial.SpeedToSI(), 1115.5, 0.1, "340 m/s is 1115.5 ft/s");

		UnitSystem cad;
		cad.length = LengthUnit::Millimetre;
		CheckClose(cad.LengthToSI(), 0.001, 0.0, "a millimetre");
		CheckClose(cad.DensityToSI(), 1.0e9, 1e-3, "kg/mm^3 -> kg/m^3 is 1e9");
		Check(std::string(cad.StepUnitName()) == "MM", "millimetres write STEP in MM");
		Check(cad != si && imperial != si && si == UnitSystem{}, "comparison");

		// Names both ways, with the everyday spellings.
		Check(ParseLengthUnit("m") == LengthUnit::Metre && ParseLengthUnit("Meters") == LengthUnit::Metre
			&& ParseLengthUnit("metres") == LengthUnit::Metre, "metre spellings");
		Check(ParseLengthUnit("mm") == LengthUnit::Millimetre && ParseLengthUnit("millimeters") == LengthUnit::Millimetre, "millimetre spellings");
		Check(ParseLengthUnit("ft") == LengthUnit::Foot && ParseLengthUnit("feet") == LengthUnit::Foot && ParseLengthUnit("FOOT") == LengthUnit::Foot, "foot spellings");
		Check(ParseMassUnit("kg") == MassUnit::Kilogram && ParseMassUnit("kilograms") == MassUnit::Kilogram, "kilogram spellings");
		Check(ParseMassUnit("lbm") == MassUnit::PoundMass && ParseMassUnit("pounds") == MassUnit::PoundMass && ParseMassUnit("lb") == MassUnit::PoundMass, "pound spellings");
		for (LengthUnit u : {LengthUnit::Metre, LengthUnit::Millimetre, LengthUnit::Foot}) {
			Check(ParseLengthUnit(LengthUnitName(u)) == u, std::string("length name round-trips: ") + LengthUnitName(u));
		}
		for (MassUnit u : {MassUnit::Kilogram, MassUnit::PoundMass}) {
			Check(ParseMassUnit(MassUnitName(u)) == u, std::string("mass name round-trips: ") + MassUnitName(u));
		}
		Check(Throws([] { ParseLengthUnit("inch"); }), "inches are refused (not offered)");
		Check(Throws([] { ParseMassUnit("slug"); }), "slugs are refused (the user types lbm; SI is what the codes get)");
		CheckTimeUnit("s");
		CheckTimeUnit("seconds");
		Check(Throws([] { CheckTimeUnit("min"); }), "a time unit other than seconds is refused");
	} catch (const std::exception& e) {
		std::cout << "UNEXPECTED EXCEPTION: " << e.what() << "\n";
		return 1;
	}
	std::cout << "\nRESULT: " << g_pass << " passed, " << g_fail << " failed\n";
	return g_fail == 0 ? 0 : 1;
}
