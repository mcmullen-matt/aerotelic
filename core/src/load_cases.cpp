#include "aerotelic/load_cases.hpp"

#include <cmath>
#include <set>

namespace aerotelic {

double VnDiagramSpec::StallSpeed(double cl, double n, double density, double weight, double area) {
	const double denom = density * cl * area;
	if (!(weight > 0.0) || !(std::fabs(denom) > 0.0)) return 0.0;
	const double v2 = 2.0 * n * weight / denom;
	return v2 > 0.0 ? std::sqrt(v2) : 0.0;
}

std::string LoadCasesProblem(const LoadCasesSpec& spec) {
	const VnDiagramSpec& vn = spec.vn;
	if (!(vn.cl_max > vn.cl_min)) return "the V-n diagram's CL_max must exceed CL_min";
	if (!(vn.n_max > vn.n_min)) return "the V-n diagram's n_max must exceed n_min";
	if (!(vn.v_ne > 0.0)) return "the V-n diagram's never-exceed speed must be positive";
	if (!(vn.density > 0.0)) return "the V-n diagram's air density must be positive";
	std::set<std::string> names;
	for (const AeroLoadCaseSpec& c : spec.aero) {
		if (c.name.empty()) return "an aero load case has no name";
		if (!names.insert(c.name).second) return "two aero load cases are named '" + c.name + "'";
		if (!(c.speed > 0.0)) return "aero load case '" + c.name + "': the speed must be positive";
		if (!(c.density > 0.0)) return "aero load case '" + c.name + "': the air density must be positive";
		if (!std::isfinite(c.load_factor)) return "aero load case '" + c.name + "': the load factor is not a number";
	}
	names.clear();
	for (const StaticLoadCaseSpec& c : spec.structural) {
		if (c.name.empty()) return "a structural load case has no name";
		if (!names.insert(c.name).second) return "two structural load cases are named '" + c.name + "'";
		if (!std::isfinite(c.load_factor)) return "structural load case '" + c.name + "': the load factor is not a number";
		for (const LoadCaseSupportSpec& s : c.supports) {
			if (s.axis != 'x' && s.axis != 'y' && s.axis != 'z') return "structural load case '" + c.name + "': a support's axis must be x, y or z";
			if (s.at != "value" && s.at != "min" && s.at != "max") return "structural load case '" + c.name + "': a support's 'at' must be value, min or max";
			if (s.tolerance < 0.0) return "structural load case '" + c.name + "': a support's tolerance must not be negative";
		}
	}
	return std::string();
}

} // namespace aerotelic
