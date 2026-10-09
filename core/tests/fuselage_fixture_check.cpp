// Cross-checks the C++ FuselageModel port against reference values dumped
// from core/fuselage_model.py (see tools/generate_fuselage_fixtures.py).

#include <fstream>
#include <iostream>
#include <string>

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include "aerotelic/fuselage_model.hpp"
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

FuselageSpec SpecFor(const std::string& name) {
	FuselageSpec s;
	if (name == "default") {
		// defaults as-is
	} else if (name == "custom") {
		s.length = 12.0;
		s.z_upper_curve.degree = 2;
		s.z_upper_curve.poles = Mat::FromRows({{0.0, 0.3}, {0.1, 1.2}, {0.7, 1.1}, {1.0, 0.2}});
		s.z_lower_curve.degree = 2;
		s.z_lower_curve.poles = Mat::FromRows({{0.0, -0.1}, {0.1, -0.9}, {0.7, -0.8}, {1.0, -0.05}});
		s.y_half_curve.degree = 2;
		s.y_half_curve.poles = Mat::FromRows({{0.0, 0.15}, {0.15, 1.0}, {0.7, 0.95}, {1.0, 0.1}});
		s.rho_upper_curve.degree = 1;
		s.rho_upper_curve.poles = Mat::FromRows({{0.0, 0.3}, {1.0, 0.5}});
		s.rho_lower_curve.degree = 1;
		s.rho_lower_curve.poles = Mat::FromRows({{0.0, 0.45}, {1.0, 0.3}});
		s.legacy = false;
	} else {
		throw std::runtime_error("unknown case " + name);
	}
	return s;
}

void RunCase(TokenReader& r, const std::string& name) {
	int n_x = -1;
	std::vector<double> x_values;
	Mat profile_z_ref, section_params_ref;
	std::vector<double> qs_scalars_ref;
	int n_contour = -1;
	Mat qs_contour_ref;
	std::vector<double> metrics_ref;
	int n_stations_ref = -1;
	std::vector<double> stations_ref;
	double volume_ref = 0.0;

	for (;;) {
		std::string tok = r.NextTok();
		if (tok == "END") break;
		else if (tok == "NX") n_x = r.NextInt();
		else if (tok == "XVALUES") x_values = r.NextDoubles(n_x);
		else if (tok == "PROFILE_Z") profile_z_ref = r.NextMat(n_x, 2);
		else if (tok == "SECTION_PARAMS") section_params_ref = r.NextMat(n_x, 5);
		else if (tok == "QS_SCALARS") qs_scalars_ref = r.NextDoubles(8);
		else if (tok == "QS_NCONTOUR") n_contour = r.NextInt();
		else if (tok == "QS_CONTOUR_3D") qs_contour_ref = r.NextMat(n_contour, 3);
		else if (tok == "METRICS") metrics_ref = r.NextDoubles(4);
		else if (tok == "NSTATIONS") n_stations_ref = r.NextInt();
		else if (tok == "STATIONS") stations_ref = r.NextDoubles(n_stations_ref);
		else if (tok == "VOLUME") volume_ref = r.NextDouble();
		else throw std::runtime_error("unknown field '" + tok + "' in case " + name);
	}

	FuselageModel fm(SpecFor(name));

	for (int i = 0; i < n_x; ++i) {
		auto [z_top, z_bot] = fm.ProfileZ(x_values[i]);
		CheckClose(z_top, profile_z_ref(i, 0), 1e-7, name + " profile_z.top[" + std::to_string(i) + "]");
		CheckClose(z_bot, profile_z_ref(i, 1), 1e-7, name + " profile_z.bot[" + std::to_string(i) + "]");

		auto p = fm.SectionParams(x_values[i]);
		for (int c = 0; c < 5; ++c) {
			CheckClose(p[c], section_params_ref(i, c), 1e-7, name + " section_params[" + std::to_string(i) + "][" + std::to_string(c) + "]");
		}
	}

	FuselageSection sec = fm.QuerySection(0.4, 80);
	CheckClose(sec.x_pos, qs_scalars_ref[0], 1e-7, name + " query_section.x_pos");
	CheckClose(sec.z_top, qs_scalars_ref[1], 1e-7, name + " query_section.z_top");
	CheckClose(sec.z_bot, qs_scalars_ref[2], 1e-7, name + " query_section.z_bot");
	CheckClose(sec.y_half, qs_scalars_ref[3], 1e-7, name + " query_section.y_half");
	CheckClose(sec.rho_upper, qs_scalars_ref[4], 1e-7, name + " query_section.rho_upper");
	CheckClose(sec.rho_lower, qs_scalars_ref[5], 1e-7, name + " query_section.rho_lower");
	CheckClose(sec.height, qs_scalars_ref[6], 1e-7, name + " query_section.height");
	CheckClose(sec.width, qs_scalars_ref[7], 1e-7, name + " query_section.width");
	Check(sec.contour_3d.rows() == n_contour, name + " contour_3d row count");
	for (int i = 0; i < n_contour; ++i) {
		for (int c = 0; c < 3; ++c) {
			CheckClose(sec.contour_3d(i, c), qs_contour_ref(i, c), 1e-6, name + " contour_3d[" + std::to_string(i) + "][" + std::to_string(c) + "]");
		}
	}

	FuselageMetrics m = fm.Metrics();
	CheckClose(m.length, metrics_ref[0], 1e-9, name + " metrics.length");
	CheckClose(m.max_width, metrics_ref[1], 1e-6, name + " metrics.max_width");
	CheckClose(m.max_height, metrics_ref[2], 1e-6, name + " metrics.max_height");
	CheckClose(m.fineness_ratio, metrics_ref[3], 1e-6, name + " metrics.fineness_ratio");

	std::vector<double> stations = fm.StationFracs(30);
	Check(static_cast<int>(stations.size()) == n_stations_ref,
		name + " station count (" + std::to_string(stations.size()) + " vs " + std::to_string(n_stations_ref) + ")");
	for (size_t i = 0; i < std::min(stations.size(), stations_ref.size()); ++i) {
		CheckClose(stations[i], stations_ref[i], 1e-6, name + " station[" + std::to_string(i) + "]");
	}

	TopoDS_Shape shape = fm.BuildSolid(30, 40, std::nullopt);
	GProp_GProps props;
	BRepGProp::VolumeProperties(shape, props);
	CheckClose(props.Mass(), volume_ref, 1e-3, name + " build_solid volume");
}

} // namespace

int main() {
	try {
		std::ifstream in(AEROTELIC_TESTS_DIR "/fuselage_fixtures.txt");
		if (!in) throw std::runtime_error("cannot open fuselage_fixtures.txt");
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
