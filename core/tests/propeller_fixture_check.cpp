// Cross-checks the C++ PropellerModel port against reference values
// dumped from core/propeller_model.py (see
// tools/generate_propeller_fixtures.py). Uses the default blade shape
// throughout, matching this port's scope cut (propeller_model.hpp).

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>

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

// The spinner is optional (PropellerSpec::spinner). On by default; off
// drops exactly one body and relaxes the checks that only exist to keep
// the blade roots buried in it.
void CheckOptionalSpinner() {
	PropellerSpec spec;
	Check(spec.spinner, "spinner: on by default");

	PropellerModel with(spec);
	std::vector<NamedMesh> m_with = with.BuildMeshes(0.01);
	Check(!m_with.empty() && m_with[0].name == "Spinner", "spinner on: the first body is the Spinner");
	Check(static_cast<int>(m_with.size()) == spec.blade_count + 1, "spinner on: one body per blade plus the spinner");

	spec.spinner = false;
	PropellerModel without(spec);
	std::vector<NamedMesh> m_without = without.BuildMeshes(0.01);
	Check(static_cast<int>(m_without.size()) == spec.blade_count, "spinner off: one body per blade and no more");
	bool any_spinner = false;
	for (const NamedMesh& nm : m_without) {
		if (nm.name == "Spinner") any_spinner = true;
	}
	Check(!any_spinner, "spinner off: no body called Spinner");
	Check(!m_without.empty() && m_without[0].name == "Blade 1", "spinner off: the body list starts at Blade 1");

	// The blades themselves must be untouched by the flag.
	Check(m_with.size() == m_without.size() + 1, "spinner off: exactly one body fewer");
	for (size_t i = 0; i < m_without.size(); ++i) {
		Check(m_without[i].name == m_with[i + 1].name, "spinner off: blade " + std::to_string(i + 1) + " keeps its name");
		CheckClose(m_without[i].mesh.SignedVolume(), m_with[i + 1].mesh.SignedVolume(), 1e-12,
			"spinner off: blade " + std::to_string(i + 1) + " is unchanged");
	}

	// A blade root outside the spinner is an error WITH a spinner (the
	// root would be left bare) and perfectly fine without one.
	PropellerSpec bare = spec;
	bare.spinner = true;
	bare.hub_radius = 0.95;
	bool threw = false;
	try { PropellerModel(bare).Validate(); } catch (const std::exception&) { threw = true; }
	Check(threw, "spinner on: a blade root outside the spinner is refused");
	bare.spinner = false;
	threw = false;
	try { PropellerModel(bare).Validate(); } catch (const std::exception&) { threw = true; }
	Check(!threw, "spinner off: the same blade root is allowed -- the roots are meant to be bare");

	// The flag survives a round trip, and the profile with it.
	PropellerSpec back = without.Serialize();
	Check(!back.spinner, "spinner off: the flag round-trips through Serialize");
	CheckClose(back.spinner_length, spec.spinner_length, 1e-12, "spinner off: the length is kept");
	Check(back.spinner_curve.poles.rows() == spec.spinner_curve.poles.rows(),
		"spinner off: the profile is kept, so switching it back on restores the shape");
	PropellerSpec on_again = back;
	on_again.spinner = true;
	std::vector<NamedMesh> m_again = PropellerModel(on_again).BuildMeshes(0.01);
	Check(!m_again.empty() && m_again[0].name == "Spinner", "switching it back on rebuilds the Spinner");
	CheckClose(m_again[0].mesh.SignedVolume(), m_with[0].mesh.SignedVolume(), 1e-12,
		"switching it back on gives the same spinner");

	// An unfused solid is the bodies as they stand; it must lose the
	// spinner too, not just the mesh path.
	TopoDS_Shape solid = without.BuildSolid();
	int n_solids = 0;
	for (TopExp_Explorer e(solid, TopAbs_SOLID); e.More(); e.Next()) ++n_solids;
	Check(n_solids == spec.blade_count, "spinner off: BuildSolid returns the blades alone");

	std::cout << "optional spinner: on -> " << m_with.size() << " bodies (" << m_with[0].name
	          << " + " << spec.blade_count << " blades), off -> " << m_without.size()
	          << " bodies (" << m_without[0].name << " ...), blades identical either way\n";
}

} // namespace

int main() {
	try {
		CheckOptionalSpinner();
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
