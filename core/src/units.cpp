#include "aerotelic/units.hpp"

#include <cctype>
#include <stdexcept>

namespace aerotelic {

namespace {

std::string Lower(const std::string& s) {
	std::string l;
	for (char c : s) l += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	return l;
}

} // namespace

const char* LengthUnitName(LengthUnit u) {
	switch (u) {
		case LengthUnit::Metre: return "m";
		case LengthUnit::Millimetre: return "mm";
		case LengthUnit::Foot: return "ft";
	}
	return "m";
}

const char* MassUnitName(MassUnit u) {
	switch (u) {
		case MassUnit::Kilogram: return "kg";
		case MassUnit::PoundMass: return "lbm";
	}
	return "kg";
}

LengthUnit ParseLengthUnit(const std::string& s) {
	const std::string l = Lower(s);
	if (l == "m" || l == "metre" || l == "metres" || l == "meter" || l == "meters") return LengthUnit::Metre;
	if (l == "mm" || l == "millimetre" || l == "millimetres" || l == "millimeter" || l == "millimeters") return LengthUnit::Millimetre;
	if (l == "ft" || l == "foot" || l == "feet") return LengthUnit::Foot;
	throw std::invalid_argument("unknown length unit '" + s + "' (expected m, mm or ft)");
}

MassUnit ParseMassUnit(const std::string& s) {
	const std::string l = Lower(s);
	if (l == "kg" || l == "kilogram" || l == "kilograms") return MassUnit::Kilogram;
	if (l == "lbm" || l == "lb" || l == "pound" || l == "pounds" || l == "pound-mass") return MassUnit::PoundMass;
	throw std::invalid_argument("unknown mass unit '" + s + "' (expected kg or lbm)");
}

void CheckTimeUnit(const std::string& s) {
	const std::string l = Lower(s);
	if (l == "s" || l == "sec" || l == "second" || l == "seconds") return;
	throw std::invalid_argument("unsupported time unit '" + s + "': time is always in seconds");
}

double UnitSystem::LengthToSI() const {
	switch (length) {
		case LengthUnit::Metre: return 1.0;
		case LengthUnit::Millimetre: return 0.001;
		case LengthUnit::Foot: return 0.3048;
	}
	return 1.0;
}

double UnitSystem::MassToSI() const {
	switch (mass) {
		case MassUnit::Kilogram: return 1.0;
		case MassUnit::PoundMass: return 0.45359237;
	}
	return 1.0;
}

const char* UnitSystem::StepUnitName() const {
	switch (length) {
		case LengthUnit::Metre: return "M";
		case LengthUnit::Millimetre: return "MM";
		case LengthUnit::Foot: return "FT";
	}
	return "M";
}

} // namespace aerotelic
