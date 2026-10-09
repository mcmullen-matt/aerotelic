// Cross-checks the C++ LoftedNacelleModel port against reference values
// dumped from core/lofted_nacelle_model.py (see
// tools/generate_lofted_nacelle_fixtures.py).

#include <fstream>
#include <iostream>
#include <string>

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include "aerotelic/lofted_nacelle_model.hpp"
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

LoftedNacelleSpec SpecFor(const std::string& name) {
	LoftedNacelleSpec s;
	if (name == "default") {
		// defaults as-is
	} else if (name == "multi_guide") {
		s.length = 3.5;
		s.centreline_curve.degree = 2;
		s.centreline_curve.poles = Mat::FromRows({{0.0, 0.0}, {0.5, 0.05}, {1.0, 0.0}});
		s.sections.clear();
		GuideSectionSpec g0, g1, g2;
		g0.station = 0.0; g0.curve = DefaultSectionCurveSpec(1.0);
		g1.station = 0.5; g1.curve = DefaultSectionCurveSpec(1.3);
		g2.station = 1.0; g2.curve = DefaultSectionCurveSpec(0.8);
		s.sections = {g0, g1, g2};
		s.outer_curve.degree = 3;
		s.outer_curve.poles = Mat::FromRows({{0.0, 0.0}, {0.05, 0.2}, {0.6, 0.18}, {1.0, 0.0}});
		s.inner_curve.degree = 3;
		s.inner_curve.poles = Mat::FromRows({{0.0, 0.0}, {0.05, -0.12}, {0.6, -0.1}, {1.0, 0.0}});
	} else if (name == "chevrons") {
		s.length = 2.8;
		s.sections.clear();
		GuideSectionSpec g0;
		g0.station = 0.0; g0.curve = DefaultSectionCurveSpec(1.0);
		s.sections = {g0};
		s.chevron_count = 6;
		s.chevron_depth = 0.2;
	} else {
		throw std::runtime_error("unknown case " + name);
	}
	return s;
}

void RunCase(TokenReader& r, const std::string& name) {
	double te_perim_ref, tip_angle_ref, volume_ref;
	std::vector<double> metrics_ref, x_values, mean_radius_ref, stations_ref;
	int n_x = -1, n_stations_ref = -1;

	for (;;) {
		std::string tok = r.NextTok();
		if (tok == "END") break;
		else if (tok == "TE_PERIMETER") te_perim_ref = r.NextDouble();
		else if (tok == "CHEVRON_TIP_ANGLE") tip_angle_ref = r.NextDouble();
		else if (tok == "METRICS") metrics_ref = r.NextDoubles(7);
		else if (tok == "NX") n_x = r.NextInt();
		else if (tok == "XVALUES") x_values = r.NextDoubles(n_x);
		else if (tok == "MEAN_RADIUS") mean_radius_ref = r.NextDoubles(n_x);
		else if (tok == "NSTATIONS") n_stations_ref = r.NextInt();
		else if (tok == "STATIONS") stations_ref = r.NextDoubles(n_stations_ref);
		else if (tok == "VOLUME") volume_ref = r.NextDouble();
		else throw std::runtime_error("unknown field '" + tok + "' in case " + name);
	}

	LoftedNacelleModel nm(SpecFor(name));
	CheckClose(nm.TePerimeter(), te_perim_ref, 1e-6, name + " te_perimeter");
	CheckClose(nm.ChevronTipAngle(), tip_angle_ref, 1e-6, name + " chevron_tip_angle");

	LoftedNacelleMetrics m = nm.Metrics();
	CheckClose(m.length, metrics_ref[0], 1e-9, name + " metrics.length");
	CheckClose(m.lip_diameter, metrics_ref[1], 1e-6, name + " metrics.lip_diameter");
	CheckClose(m.max_diameter, metrics_ref[2], 1e-6, name + " metrics.max_diameter");
	CheckClose(m.te_diameter, metrics_ref[3], 1e-6, name + " metrics.te_diameter");
	CheckClose(m.max_thickness, metrics_ref[4], 1e-6, name + " metrics.max_thickness");
	Check(m.n_sections == static_cast<int>(metrics_ref[5]), name + " metrics.n_sections");
	CheckClose(m.chevron_depth, metrics_ref[6], 1e-6, name + " metrics.chevron_depth");

	for (int i = 0; i < n_x; ++i) {
		CheckClose(nm.MeanRadius(x_values[i]), mean_radius_ref[i], 1e-6, name + " mean_radius[" + std::to_string(i) + "]");
	}

	std::vector<double> stations = nm.StationFracs(30);
	Check(static_cast<int>(stations.size()) == n_stations_ref,
		name + " station count (" + std::to_string(stations.size()) + " vs " + std::to_string(n_stations_ref) + ")");
	for (size_t i = 0; i < std::min(stations.size(), stations_ref.size()); ++i) {
		CheckClose(stations[i], stations_ref[i], 1e-6, name + " station[" + std::to_string(i) + "]");
	}

	TopoDS_Shape shape = nm.BuildSolid();
	GProp_GProps props;
	BRepGProp::VolumeProperties(shape, props);
	CheckClose(props.Mass(), volume_ref, 1e-3, name + " build_solid volume");
}

} // namespace

int main() {
	try {
		std::ifstream in(AEROTELIC_TESTS_DIR "/lofted_nacelle_fixtures.txt");
		if (!in) throw std::runtime_error("cannot open lofted_nacelle_fixtures.txt");
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
