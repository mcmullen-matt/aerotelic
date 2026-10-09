// Cross-checks the C++ PropellerModel port against reference values
// dumped from core/propeller_model.py (see
// tools/generate_propeller_fixtures.py). Uses the default blade shape
// throughout, matching this port's scope cut (propeller_model.hpp).

#include <fstream>
#include <iostream>
#include <string>

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include "aerotelic/propeller_model.hpp"
#include "token_reader.hpp"

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

PropellerSpec SpecFor(const std::string& name) {
	PropellerSpec s;
	if (name == "default" || name == "rescaled") {
		// defaults as-is
	} else if (name == "three_blade") {
		s.blade_count = 3;
		s.radius = 1.1;
		s.blade_x = 0.3;
		s.hub_radius = 0.15;
	} else {
		throw std::runtime_error("unknown case " + name);
	}
	return s;
}

void RunCase(TokenReader& r, const std::string& name) {
	double spinner_radius_ref, volume_ref;
	int n_x = -1, n_eta = -1;
	std::vector<double> x_values, spinner_radius_at_ref, eta_values, chord_over_r_ref, blade_angle_ref;
	std::vector<double> metrics_ref;
	int metrics_blade_count_ref = 0;

	for (;;) {
		std::string tok = r.NextTok();
		if (tok == "END") break;
		else if (tok == "SPINNER_RADIUS") spinner_radius_ref = r.NextDouble();
		else if (tok == "NX") n_x = r.NextInt();
		else if (tok == "XVALUES") x_values = r.NextDoubles(n_x);
		else if (tok == "SPINNER_RADIUS_AT") spinner_radius_at_ref = r.NextDoubles(n_x);
		else if (tok == "NETA") n_eta = r.NextInt();
		else if (tok == "ETAVALUES") eta_values = r.NextDoubles(n_eta);
		else if (tok == "CHORD_OVER_R") chord_over_r_ref = r.NextDoubles(n_eta);
		else if (tok == "BLADE_ANGLE") blade_angle_ref = r.NextDoubles(n_eta);
		else if (tok == "METRICS") {
			metrics_ref.push_back(r.NextDouble());             // diameter
			metrics_blade_count_ref = r.NextInt();              // blade_count
			for (int i = 0; i < 6; ++i) metrics_ref.push_back(r.NextDouble());
		}
		else if (tok == "VOLUME") volume_ref = r.NextDouble();
		else throw std::runtime_error("unknown field '" + tok + "' in case " + name);
	}

	PropellerModel pm(SpecFor(name));
	if (name == "rescaled") pm.SetRadius(1.25);

	CheckClose(pm.SpinnerRadius(), spinner_radius_ref, 1e-6, name + " spinner_radius");

	for (int i = 0; i < n_x; ++i) {
		CheckClose(pm.SpinnerRadiusAt(x_values[i]), spinner_radius_at_ref[i], 1e-6, name + " spinner_radius_at[" + std::to_string(i) + "]");
	}

	auto chords = pm.ChordOverR(eta_values);
	for (int i = 0; i < n_eta; ++i) {
		CheckClose(chords[i], chord_over_r_ref[i], 1e-6, name + " chord_over_r[" + std::to_string(i) + "]");
		CheckClose(pm.BladeAngle(eta_values[i]), blade_angle_ref[i], 1e-6, name + " blade_angle[" + std::to_string(i) + "]");
	}

	PropellerMetrics m = pm.Metrics();
	CheckClose(m.diameter, metrics_ref[0], 1e-9, name + " metrics.diameter");
	Check(m.blade_count == metrics_blade_count_ref, name + " metrics.blade_count");
	CheckClose(m.solidity, metrics_ref[1], 1e-6, name + " metrics.solidity");
	CheckClose(m.activity_factor, metrics_ref[2], 1e-3, name + " metrics.activity_factor");
	CheckClose(m.beta_75, metrics_ref[3], 1e-6, name + " metrics.beta_75");
	CheckClose(m.pitch_ratio, metrics_ref[4], 1e-6, name + " metrics.pitch_ratio");
	CheckClose(m.spinner_diameter, metrics_ref[5], 1e-6, name + " metrics.spinner_diameter");
	CheckClose(m.spinner_length, metrics_ref[6], 1e-6, name + " metrics.spinner_length");

	TopoDS_Shape shape = pm.BuildSolid();
	GProp_GProps props;
	BRepGProp::VolumeProperties(shape, props);
	CheckClose(props.Mass(), volume_ref, 1e-3, name + " build_solid volume");
}

} // namespace

int main() {
	try {
		std::ifstream in(AEROTELIC_TESTS_DIR "/propeller_fixtures.txt");
		if (!in) throw std::runtime_error("cannot open propeller_fixtures.txt");
		TokenReader r(in);
		while (r.HasNext()) {
			r.Expect("CASE");
			std::string name = r.NextTok();
			RunCase(r, name);
		}
	} catch (const std::exception& e) {
		std::cout << "FIXTURE PARSE/RUN ERROR: " << e.what() << "\n";
		return 1;
	}
	std::cout << "\nRESULT: " << g_pass << " passed, " << g_fail << " failed\n";
	return g_fail == 0 ? 0 : 1;
}
