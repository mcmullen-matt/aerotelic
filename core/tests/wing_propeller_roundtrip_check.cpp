// Self-contained checks for the WingModelSpec/PropellerSpec round-trip
// gap closed this pass: WingModel's sections previously could only be
// added via AddSection() and were dropped by any spec-based
// reconstruction; PropellerModel::Serialize() previously didn't carry the
// blade's edits at all. No Python fixture needed -- this is purely
// "does serialize-then-reconstruct reproduce the same model," an internal
// consistency property.

#include <cmath>
#include <iostream>
#include <string>

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include "aerotelic/propeller_model.hpp"
#include "aerotelic/wing_model.hpp"

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

double Volume(const TopoDS_Shape& s) {
	GProp_GProps props;
	BRepGProp::VolumeProperties(s, props);
	return props.Mass();
}

void CheckWingModelSectionsRoundTrip() {
	PlanformSpec pspec;
	pspec.type = "standard";
	pspec.semi_span = 6.0;
	pspec.root_chord = 2.0;
	pspec.tip_chord = 0.8;

	WingModel wm(pspec, "TestWing");
	wm.AddSection(Airfoil::Naca4("2412"), 0.0);
	wm.AddSection(Airfoil::Naca4("4412"), 0.5);
	wm.AddSection(Airfoil::Naca4("0012"), 1.0);

	WingModelSpec spec = wm.Serialize();
	Check(spec.name == "TestWing", "WingModel::Serialize captures the name");
	Check(spec.sections.size() == 3, "WingModel::Serialize captures every section");
	Check(spec.planform.type == "standard", "WingModel::Serialize captures the planform type");
	CheckClose(spec.planform.semi_span, 6.0, 1e-9, "WingModel::Serialize captures planform params");

	WingModel reloaded(spec);
	Check(reloaded.sections().size() == 3, "reconstructed WingModel has every section");
	for (size_t i = 0; i < reloaded.sections().size(); ++i) {
		CheckClose(reloaded.sections()[i].s, wm.sections()[i].s, 1e-12, "reconstructed section station[" + std::to_string(i) + "]");
	}

	TopoDS_Shape original_solid = wm.BuildSolid(40, true, std::nullopt);
	TopoDS_Shape reloaded_solid = reloaded.BuildSolid(40, true, std::nullopt);
	CheckClose(Volume(original_solid), Volume(reloaded_solid), 1e-6, "reconstructed WingModel rebuilds to the same volume");
}

void CheckPropellerBladeRoundTrip() {
	PropellerSpec pspec;
	pspec.radius = 1.0;
	PropellerModel pm(pspec);

	// Edit the live blade beyond what the default shape has, so a
	// round-trip that silently reverted to MakeDefaultBlade would be
	// caught: add a fourth section the default doesn't have.
	pm.blade().AddSection(Airfoil::Naca4("4415"), 0.7);
	Check(pm.blade().sections().size() == 4, "edited blade has the added section before serializing");

	PropellerSpec serialized = pm.Serialize();
	Check(serialized.blade.has_value(), "PropellerModel::Serialize carries the blade spec");
	Check(serialized.blade->sections.size() == 4, "PropellerModel::Serialize captures the blade's edited sections");

	PropellerModel reloaded(serialized);
	Check(reloaded.blade().sections().size() == 4, "reconstructed PropellerModel's blade keeps the added section");

	double original_volume = Volume(pm.BuildSolid(20, std::nullopt, false));
	double reloaded_volume = Volume(reloaded.BuildSolid(20, std::nullopt, false));
	CheckClose(original_volume, reloaded_volume, 1e-6, "reconstructed PropellerModel rebuilds to the same volume");
}

void CheckDefaultBladeStillWorksWithNoSpec() {
	PropellerSpec pspec; // no .blade set
	PropellerModel pm(pspec);
	Check(pm.blade().sections().size() == 3, "default-blade propeller still gets the 3 default NACA sections");
	Check(!pm.Serialize().blade->sections.empty(), "even the default blade now serializes its sections (used to be dropped)");
}

} // namespace

int main() {
	try {
		CheckWingModelSectionsRoundTrip();
		CheckPropellerBladeRoundTrip();
		CheckDefaultBladeStillWorksWithNoSpec();
	} catch (const std::exception& e) {
		std::cout << "UNEXPECTED EXCEPTION: " << e.what() << "\n";
		return 1;
	}
	std::cout << "\nRESULT: " << g_pass << " passed, " << g_fail << " failed\n";
	return g_fail == 0 ? 0 : 1;
}
