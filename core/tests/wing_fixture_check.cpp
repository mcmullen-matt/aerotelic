// Cross-checks the C++ Planform/WingModel port against reference values
// dumped from core/wing_model.py (see tools/generate_wing_fixtures.py).
// control_surfaces/reference_images/pole_links are out of scope (see
// wing_model.hpp's file header) so nothing here exercises them.

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include "aerotelic/airfoil.hpp"
#include "aerotelic/wing_model.hpp"
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

PlanformSpec SpecFor(const std::string& name) {
	PlanformSpec s;
	if (name == "standard" || name == "trap_linear") {
		s.type = "standard";
		s.semi_span = 8.0; s.root_chord = 2.5; s.tip_chord = 1.0;
		s.sweep_angle = 15.0; s.sweep_fraction = 0.25; s.rotation_fraction = 0.25;
	} else if (name == "taper") {
		s.type = "taper";
		s.semi_span = 6.0; s.root_chord = 2.0; s.tip_chord = 0.8;
		s.sweep_angle = 10.0; s.sweep_fraction = 0.0; s.rotation_fraction = 0.3;
	} else if (name == "double_trap" || name == "double_trap_kink") {
		s.type = "double_trap";
		s.semi_span = 9.0; s.root_chord = 3.0; s.tip_chord = 1.2;
		s.kink_fraction = 0.4; s.leading_edge_sweep_angle = 25.0;
		s.trailing_edge_sweep_angle = 5.0; s.rotation_fraction = 0.25;
	} else if (name == "nurbs" || name == "nurbs_curved_tip") {
		s.type = "nurbs";
		s.semi_span = 7.0; s.rotation_fraction = 0.25;
		NurbsSpec le; le.degree = 3; le.poles = Mat::FromRows({{0.0, 0.0}, {0.5, 0.1}, {0.8, 0.3}, {1.0, 0.6}});
		NurbsSpec te; te.degree = 1; te.poles = Mat::FromRows({{0.0, 2.0}, {1.0, 1.0}});
		s.le_curve = le;
		s.te_curve = te;
	} else {
		throw std::runtime_error("unknown planform case " + name);
	}
	return s;
}

void RunPlanformCase(TokenReader& r, const std::string& name) {
	double semi_span_ref, root_chord_ref, tip_chord_ref, area_ref, ar_ref;
	int n_s = -1, n_outline = -1;
	std::vector<double> s_values;
	Mat xyz_ref, outline_ref;

	for (;;) {
		std::string tok = r.NextTok();
		if (tok == "END") break;
		else if (tok == "SEMI_SPAN") semi_span_ref = r.NextDouble();
		else if (tok == "ROOT_CHORD") root_chord_ref = r.NextDouble();
		else if (tok == "TIP_CHORD") tip_chord_ref = r.NextDouble();
		else if (tok == "PLANFORM_AREA") area_ref = r.NextDouble();
		else if (tok == "ASPECT_RATIO") ar_ref = r.NextDouble();
		else if (tok == "NS") n_s = r.NextInt();
		else if (tok == "SVALUES") s_values = r.NextDoubles(n_s);
		else if (tok == "XYZ") xyz_ref = r.NextMat(n_s * 4, 3);
		else if (tok == "NOUTLINE") n_outline = r.NextInt();
		else if (tok == "OUTLINE") outline_ref = r.NextMat(n_outline, 3);
		else throw std::runtime_error("unknown field '" + tok + "' in planform case " + name);
	}

	auto pf = MakePlanform(SpecFor(name));
	CheckClose(pf->SemiSpan(), semi_span_ref, 1e-9, name + " semi_span");
	CheckClose(pf->RootChord(), root_chord_ref, 1e-7, name + " root_chord");
	CheckClose(pf->TipChord(), tip_chord_ref, 1e-7, name + " tip_chord");
	CheckClose(pf->PlanformArea(), area_ref, 1e-5, name + " planform_area");
	CheckClose(pf->AspectRatio(), ar_ref, 1e-5, name + " aspect_ratio");

	Mat coords = Mat::FromRows({{0.0, 0.0}, {1.0, 0.0}, {0.25, 0.05}, {0.75, -0.03}});
	for (int k = 0; k < n_s; ++k) {
		Mat xyz = pf->AirfoilCoordsToXyz(s_values[k], coords);
		for (int i = 0; i < 4; ++i)
			for (int c = 0; c < 3; ++c)
				CheckClose(xyz(i, c), xyz_ref(k * 4 + i, c), 1e-6, name + " xyz[" + std::to_string(k) + "][" + std::to_string(i) + "][" + std::to_string(c) + "]");
	}

	Mat outline = pf->PlanformOutline(50);
	for (int i = 0; i < outline.rows(); ++i)
		for (int c = 0; c < 3; ++c)
			CheckClose(outline(i, c), outline_ref(i, c), 1e-6, name + " outline[" + std::to_string(i) + "][" + std::to_string(c) + "]");
}

void RunLoftCase(TokenReader& r, const std::string& name) {
	int n_stations_ref = -1;
	std::vector<double> stations_ref;
	double volume_ref = 0.0;
	for (;;) {
		std::string tok = r.NextTok();
		if (tok == "END") break;
		else if (tok == "NSTATIONS") n_stations_ref = r.NextInt();
		else if (tok == "STATIONS") stations_ref = r.NextDoubles(n_stations_ref);
		else if (tok == "VOLUME") volume_ref = r.NextDouble();
		else throw std::runtime_error("unknown field '" + tok + "' in loft case " + name);
	}

	WingModel wm(SpecFor(name), "t");
	struct Case { std::string name; std::vector<std::pair<double, std::string>> sections; int n_span; };
	std::vector<Case> cases = {
		{"trap_linear", {{0.0, "2412"}, {1.0, "0012"}}, 40},
		{"double_trap_kink", {{0.0, "4412"}, {0.4, "2412"}, {1.0, "0012"}}, 40},
		{"nurbs_curved_tip", {{0.0, "4412"}, {1.0, "0012"}}, 60},
	};
	auto it = std::find_if(cases.begin(), cases.end(), [&](const Case& c) { return c.name == name; });
	if (it == cases.end()) throw std::runtime_error("unrecognized loft case " + name);
	for (auto& [s, code] : it->sections) wm.AddSection(Airfoil::Naca4(code), s);
	int n_span = it->n_span;

	std::vector<double> stations = wm.LoftStations(n_span);
	Check(static_cast<int>(stations.size()) == n_stations_ref, name + " station count (" + std::to_string(stations.size()) + " vs " + std::to_string(n_stations_ref) + ")");
	for (size_t i = 0; i < std::min(stations.size(), stations_ref.size()); ++i) {
		CheckClose(stations[i], stations_ref[i], 1e-6, name + " station[" + std::to_string(i) + "]");
	}

	TopoDS_Shape shape = wm.BuildSolid(n_span, true, std::nullopt);
	GProp_GProps props;
	BRepGProp::VolumeProperties(shape, props);
	CheckClose(props.Mass(), volume_ref, 1e-4, name + " build_solid volume");
}

} // namespace

int main() {
	try {
		std::ifstream in(AEROTELIC_TESTS_DIR "/wing_fixtures.txt");
		if (!in) throw std::runtime_error("cannot open wing_fixtures.txt");
		TokenReader r(in);
		while (r.HasNext()) {
			std::string kind = r.NextTok();
			std::string name = r.NextTok();
			if (kind == "PLANFORM_CASE") RunPlanformCase(r, name);
			else if (kind == "LOFT_CASE") RunLoftCase(r, name);
			else throw std::runtime_error("unknown case kind " + kind);
		}
	} catch (const std::exception& e) {
		std::cout << "FIXTURE PARSE/RUN ERROR: " << e.what() << "\n";
		return 1;
	}
	std::cout << "\nRESULT: " << g_pass << " passed, " << g_fail << " failed\n";
	return g_fail == 0 ? 0 : 1;
}
