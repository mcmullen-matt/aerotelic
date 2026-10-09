// Cross-checks the C++ NacelleModel port against reference values dumped
// from core/nacelle_model.py (see tools/generate_nacelle_fixtures.py).

#include <fstream>
#include <iostream>
#include <string>

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include "aerotelic/nacelle_model.hpp"
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

NacelleSpec SpecFor(const std::string& name) {
	NacelleSpec s;
	if (name == "default") {
		// defaults as-is
	} else if (name == "no_chevrons_custom") {
		s.length = 3.2;
		s.upper_curve.degree = 3;
		s.upper_curve.poles = Mat::FromRows({{0.0, 1.0}, {0.01, 1.2}, {0.3, 1.35}, {0.75, 1.2}, {1.0, 0.9}});
		s.lower_curve.degree = 3;
		s.lower_curve.poles = Mat::FromRows({{0.0, 1.0}, {0.01, 0.85}, {0.25, 0.82}, {0.75, 0.88}, {1.0, 0.9}});
		s.chevron_count = 0;
	} else if (name == "chevrons") {
		s.length = 2.5;
		s.upper_curve.degree = 3;
		s.upper_curve.poles = Mat::FromRows({{0.0, 1.0}, {0.01, 1.15}, {0.3, 1.25}, {0.75, 1.15}, {1.0, 0.95}});
		s.lower_curve.degree = 3;
		s.lower_curve.poles = Mat::FromRows({{0.0, 1.0}, {0.01, 0.9}, {0.25, 0.88}, {0.75, 0.9}, {1.0, 0.95}});
		s.chevron_count = 6;
		s.chevron_depth = 0.15;
	} else if (name == "chevron_angle_legacy") {
		s.length = 2.0;
		s.chevron_count = 8;
		s.chevron_angle = 40.0;
	} else {
		throw std::runtime_error("unknown case " + name);
	}
	return s;
}

void RunCase(TokenReader& r, const std::string& name) {
	double lip_radius_ref, te_radius_ref, ring_perim_ref, te_perim_ref, tooth_width_ref, tip_angle_ref, volume_ref;
	std::vector<double> metrics_ref;

	for (;;) {
		std::string tok = r.NextTok();
		if (tok == "END") break;
		else if (tok == "LIP_RADIUS") lip_radius_ref = r.NextDouble();
		else if (tok == "TE_RADIUS") te_radius_ref = r.NextDouble();
		else if (tok == "RING_PERIMETER") ring_perim_ref = r.NextDouble();
		else if (tok == "TE_PERIMETER") te_perim_ref = r.NextDouble();
		else if (tok == "TOOTH_WIDTH") tooth_width_ref = r.NextDouble();
		else if (tok == "CHEVRON_TIP_ANGLE") tip_angle_ref = r.NextDouble();
		else if (tok == "METRICS") metrics_ref = r.NextDoubles(5);
		else if (tok == "VOLUME") volume_ref = r.NextDouble();
		else throw std::runtime_error("unknown field '" + tok + "' in case " + name);
	}

	NacelleModel nm(SpecFor(name));
	CheckClose(nm.LipRadius(), lip_radius_ref, 1e-9, name + " lip_radius");
	CheckClose(nm.TeRadius(), te_radius_ref, 1e-9, name + " te_radius");
	CheckClose(nm.RingPerimeter(), ring_perim_ref, 1e-6, name + " ring_perimeter");
	CheckClose(nm.TePerimeter(), te_perim_ref, 1e-6, name + " te_perimeter");
	CheckClose(nm.ToothWidth(), tooth_width_ref, 1e-6, name + " tooth_width");
	CheckClose(nm.ChevronTipAngle(), tip_angle_ref, 1e-6, name + " chevron_tip_angle");

	NacelleMetrics m = nm.Metrics();
	CheckClose(m.length, metrics_ref[0], 1e-9, name + " metrics.length");
	CheckClose(m.lip_diameter, metrics_ref[1], 1e-6, name + " metrics.lip_diameter");
	CheckClose(m.max_diameter, metrics_ref[2], 1e-6, name + " metrics.max_diameter");
	CheckClose(m.te_diameter, metrics_ref[3], 1e-6, name + " metrics.te_diameter");
	CheckClose(m.chevron_depth, metrics_ref[4], 1e-6, name + " metrics.chevron_depth");

	TopoDS_Shape shape = nm.BuildSolid();
	GProp_GProps props;
	BRepGProp::VolumeProperties(shape, props);
	CheckClose(props.Mass(), volume_ref, 1e-3, name + " build_solid volume");
}

} // namespace

int main() {
	try {
		std::ifstream in(AEROTELIC_TESTS_DIR "/nacelle_fixtures.txt");
		if (!in) throw std::runtime_error("cannot open nacelle_fixtures.txt");
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
