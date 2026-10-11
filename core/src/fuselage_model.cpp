#include "aerotelic/fuselage_model.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Writer.hxx>
#include <gp_Pnt.hxx>

namespace aerotelic {

namespace {

std::vector<double> Linspace(double a, double b, int n) {
	std::vector<double> out(n);
	for (int i = 0; i < n; ++i) out[i] = a + (b - a) * i / (n - 1);
	return out;
}

void WithNoseStation(NurbsSpec& spec, bool legacy) {
	spec.poles(0, 0) = 0.0;
	if (legacy) spec.poles(0, 1) = 0.0;
}

} // namespace

Mat ConicCrossSection2D(double z_top, double z_bot, double y_half, double rho_upper, double rho_lower, int n_pts) {
	double z_shoulder = 0.5 * (z_top + z_bot);
	int n = std::max(4, n_pts / 4);
	std::vector<double> t = Linspace(0.0, 1.0, n);

	auto rqb = [&](double rho, double p0y, double p0z, double p1y, double p1z, double p2y, double p2z) {
		double rho_c = std::clamp(rho, 0.005, 0.995);
		double w = rho_c / (1.0 - rho_c);
		Mat out(n, 2);
		for (int i = 0; i < n; ++i) {
			double ti = t[i];
			double denom = (1 - ti) * (1 - ti) + 2.0 * ti * (1 - ti) * w + ti * ti;
			out(i, 0) = ((1 - ti) * (1 - ti) * p0y + 2 * ti * (1 - ti) * w * p1y + ti * ti * p2y) / denom;
			out(i, 1) = ((1 - ti) * (1 - ti) * p0z + 2 * ti * (1 - ti) * w * p1z + ti * ti * p2z) / denom;
		}
		return out;
	};

	// Right half: top-centre -> shoulder -> bottom-centre.
	Mat ur = rqb(rho_upper, 0.0, z_top, y_half, z_top, y_half, z_shoulder);
	Mat lr = rqb(rho_lower, y_half, z_shoulder, y_half, z_bot, 0.0, z_bot);
	int n_ur = ur.rows(), n_lr = lr.rows();
	Mat right(n_ur + n_lr - 1, 2); // lr[1:] -- avoid duplicate at the shoulder
	for (int i = 0; i < n_ur; ++i) right.set_row(i, ur.row(i));
	for (int i = 1; i < n_lr; ++i) right.set_row(n_ur + i - 1, lr.row(i));

	// Left half: mirror of right, reversed.
	int n_right = right.rows();
	Mat left(n_right, 2);
	for (int i = 0; i < n_right; ++i) {
		left(i, 0) = -right(n_right - 1 - i, 0);
		left(i, 1) = right(n_right - 1 - i, 1);
	}

	// Full closed loop: right + left interior (drop duplicate top/bottom centres).
	int n_left_interior = n_right - 2;
	Mat result(n_right + n_left_interior, 2);
	for (int i = 0; i < n_right; ++i) result.set_row(i, right.row(i));
	for (int i = 0; i < n_left_interior; ++i) result.set_row(n_right + i, left.row(i + 1));
	return result;
}

std::array<double, 2> ConicCrossSectionPoint(double z_top, double z_bot, double y_half, double rho_upper, double rho_lower, double v) {
	double z_shoulder = 0.5 * (z_top + z_bot);
	v = v - std::floor(v);
	int q = std::min(3, static_cast<int>(v * 4.0));
	double t = v * 4.0 - q;
	double rho = (q == 0 || q == 3) ? rho_upper : rho_lower;
	double rho_c = std::clamp(rho, 0.005, 0.995);
	double w = rho_c / (1.0 - rho_c);
	double p0y, p0z, p1y, p1z, p2y, p2z;
	switch (q) {
		case 0: p0y = 0.0; p0z = z_top; p1y = y_half; p1z = z_top; p2y = y_half; p2z = z_shoulder; break;
		case 1: p0y = y_half; p0z = z_shoulder; p1y = y_half; p1z = z_bot; p2y = 0.0; p2z = z_bot; break;
		case 2: p0y = 0.0; p0z = z_bot; p1y = -y_half; p1z = z_bot; p2y = -y_half; p2z = z_shoulder; break;
		default: p0y = -y_half; p0z = z_shoulder; p1y = -y_half; p1z = z_top; p2y = 0.0; p2z = z_top; break;
	}
	double denom = (1 - t) * (1 - t) + 2.0 * t * (1 - t) * w + t * t;
	return {((1 - t) * (1 - t) * p0y + 2 * t * (1 - t) * w * p1y + t * t * p2y) / denom,
		((1 - t) * (1 - t) * p0z + 2 * t * (1 - t) * w * p1z + t * t * p2z) / denom};
}

Mat ConicCrossSection3D(double x_pos, double z_top, double z_bot, double y_half, double rho_upper, double rho_lower, int n_pts) {
	Mat yz = ConicCrossSection2D(z_top, z_bot, y_half, rho_upper, rho_lower, n_pts);
	Mat out(yz.rows(), 3);
	for (int i = 0; i < yz.rows(); ++i) {
		out(i, 0) = x_pos;
		out(i, 1) = yz(i, 0);
		out(i, 2) = yz(i, 1);
	}
	return out;
}

FuselageSpec::FuselageSpec() {
	z_upper_curve.degree = 2;
	z_upper_curve.poles = Mat::FromRows({{0.00, 0.00}, {0.08, 1.50}, {0.75, 1.50}, {1.00, 0.15}});
	z_lower_curve.degree = 2;
	z_lower_curve.poles = Mat::FromRows({{0.00, 0.00}, {0.08, -1.30}, {0.75, -1.30}, {1.00, -0.10}});
	y_half_curve.degree = 2;
	y_half_curve.poles = Mat::FromRows({{0.00, 0.00}, {0.12, 1.50}, {0.75, 1.50}, {1.00, 0.05}});
	rho_upper_curve.degree = 1;
	rho_upper_curve.poles = Mat::FromRows({{0.00, kFuselageRhoEllipse}, {1.00, kFuselageRhoEllipse}});
	rho_lower_curve.degree = 1;
	rho_lower_curve.poles = Mat::FromRows({{0.00, kFuselageRhoEllipse}, {1.00, kFuselageRhoEllipse}});
}

FuselageModel::FuselageModel(const FuselageSpec& spec) {
	length = spec.length;
	name = spec.name;
	reference_images = spec.reference_images;

	NurbsSpec zu = spec.z_upper_curve, zl = spec.z_lower_curve, yh = spec.y_half_curve;
	WithNoseStation(zu, spec.legacy);
	WithNoseStation(zl, spec.legacy);
	WithNoseStation(yh, spec.legacy);
	z_upper_curve_ = NurbsCurve::Create(zu);
	z_lower_curve_ = NurbsCurve::Create(zl);
	if (z_upper_curve_->periodic() || z_lower_curve_->periodic()) {
		throw std::invalid_argument("outline curves must be open (clamped) curves");
	}
	y_half_curve_ = NurbsCurve::Create(yh);
	rho_upper_curve_ = NurbsCurve::Create(spec.rho_upper_curve);
	rho_lower_curve_ = NurbsCurve::Create(spec.rho_lower_curve);
}

FuselageSpec FuselageModel::Serialize() const {
	FuselageSpec s;
	s.length = length;
	s.name = name;
	s.z_upper_curve = z_upper_curve_->Serialize();
	s.z_lower_curve = z_lower_curve_->Serialize();
	s.y_half_curve = y_half_curve_->Serialize();
	s.rho_upper_curve = rho_upper_curve_->Serialize();
	s.rho_lower_curve = rho_lower_curve_->Serialize();
	s.reference_images = reference_images;
	return s;
}

void FuselageModel::SetLength(double new_length) {
	double old = length;
	if (!(new_length > 0.0)) throw std::invalid_argument("length must be positive");
	length = new_length;
	if (!(old > 0.0) || new_length == old) return;
	double k = new_length / old;
	auto scale_col1 = [&](NurbsCurve& c) {
		Mat poles = c.poles();
		for (int i = 0; i < poles.rows(); ++i) poles(i, 1) *= k;
		c.SetPolesAndWeights(poles, c.weights());
	};
	scale_col1(*z_upper_curve_);
	scale_col1(*z_lower_curve_);
	scale_col1(*y_half_curve_);
}

std::pair<double, double> FuselageModel::ProfileZ(double x_frac) const {
	return {z_upper_curve_->ZAtEta(x_frac)[1], z_lower_curve_->ZAtEta(x_frac)[1]};
}

std::array<double, 5> FuselageModel::SectionParams(double x_frac) const {
	x_frac = std::clamp(x_frac, 0.0, 1.0);
	auto [z_top, z_bot] = ProfileZ(x_frac);
	double y_half = std::max(std::abs(y_half_curve_->ZAtEta(x_frac)[1]), 1e-6);
	double rho_u = std::clamp(rho_upper_curve_->ZAtEta(x_frac)[1], 0.005, 0.995);
	double rho_l = std::clamp(rho_lower_curve_->ZAtEta(x_frac)[1], 0.005, 0.995);
	return {z_top, z_bot, y_half, rho_u, rho_l};
}

FuselageSection FuselageModel::QuerySection(double x_frac, int n_pts) const {
	x_frac = std::clamp(x_frac, 0.0, 1.0);
	double x_pos = x_frac * length;
	auto [z_top, z_bot, y_half, rho_u, rho_l] = SectionParams(x_frac);
	FuselageSection sec;
	sec.x_frac = x_frac;
	sec.x_pos = x_pos;
	sec.z_top = z_top;
	sec.z_bot = z_bot;
	sec.y_half = y_half;
	sec.rho_upper = rho_u;
	sec.rho_lower = rho_l;
	sec.height = z_top - z_bot;
	sec.width = 2.0 * y_half;
	sec.contour_2d = ConicCrossSection2D(z_top, z_bot, y_half, rho_u, rho_l, n_pts);
	sec.contour_3d = ConicCrossSection3D(x_pos, z_top, z_bot, y_half, rho_u, rho_l, n_pts);
	return sec;
}

FuselageMetrics FuselageModel::Metrics() const {
	std::vector<double> xs = Linspace(0.0, 1.0, 50);
	double max_w = 0.0, max_h = 0.0;
	for (double x : xs) {
		auto p = SectionParams(x);
		max_w = std::max(max_w, 2.0 * p[2]);
		max_h = std::max(max_h, p[0] - p[1]);
	}
	FuselageMetrics m;
	m.length = length;
	m.max_width = max_w;
	m.max_height = max_h;
	m.fineness_ratio = length / std::max(max_w, 1e-6);
	return m;
}

std::vector<double> FuselageModel::StationFracs(int n_stations, std::optional<double> tol_opt, int max_stations) const {
	double tol = tol_opt.value_or(kStationTol);
	std::vector<double> fracs = Linspace(0.0, 1.0, std::max(n_stations, 2));
	std::map<double, std::array<double, 3>> cache;
	auto dims = [&](double x) -> const std::array<double, 3>& {
		auto it = cache.find(x);
		if (it != cache.end()) return it->second;
		auto p = SectionParams(x);
		return cache.emplace(x, std::array<double, 3>{p[0], p[1], p[2]}).first->second;
	};

	while (static_cast<int>(fracs.size()) < max_stations) {
		std::vector<std::pair<double, double>> bad;
		for (size_t i = 0; i + 1 < fracs.size(); ++i) {
			double a = fracs[i], b = fracs[i + 1];
			if (b - a <= kMinStationGap) continue;
			double mid = 0.5 * (a + b);
			const auto& da = dims(a);
			const auto& db = dims(b);
			const auto& dm = dims(mid);
			double err = 0.0;
			for (int k = 0; k < 3; ++k) err = std::max(err, std::abs(dm[k] - 0.5 * (da[k] + db[k])));
			if (err > tol) bad.push_back({err, mid});
		}
		if (bad.empty()) break;
		std::sort(bad.begin(), bad.end(), [](const auto& x, const auto& y) { return x.first > y.first; });
		int n_add = std::min(static_cast<int>(bad.size()), max_stations - static_cast<int>(fracs.size()));
		for (int i = 0; i < n_add; ++i) fracs.push_back(bad[i].second);
		std::sort(fracs.begin(), fracs.end());
	}
	return fracs;
}

TopoDS_Shape FuselageModel::BuildSolid(int n_stations, int n_pts, std::optional<double> station_tol) const {
	BRepOffsetAPI_ThruSections loft(true, false);
	for (double x_frac : StationFracs(n_stations, station_tol)) {
		double x_pos = x_frac * length;
		auto [z_top, z_bot, y_half, rho_u, rho_l] = SectionParams(x_frac);

		// Never hand the loft a degenerate section: open an apex up to a
		// minimum-size polygon about its centre instead.
		double half_h = std::max(0.5 * std::abs(z_top - z_bot), kMinSectionHalf);
		double z_mid = 0.5 * (z_top + z_bot);
		z_top = z_mid + half_h;
		z_bot = z_mid - half_h;
		y_half = std::max(y_half, kMinSectionHalf);

		Mat pts = ConicCrossSection3D(x_pos, z_top, z_bot, y_half, rho_u, rho_l, n_pts);
		BRepBuilderAPI_MakePolygon poly;
		for (int i = 0; i < pts.rows(); ++i) poly.Add(gp_Pnt(pts(i, 0), pts(i, 1), pts(i, 2)));
		poly.Close();
		if (poly.IsDone()) loft.AddWire(poly.Wire());
	}
	loft.Build();
	if (!loft.IsDone()) throw std::runtime_error("Fuselage lofting failed");
	return loft.Shape();
}

std::vector<NamedMesh> FuselageModel::BuildMeshes(double tolerance) const {
	LoftSource src;
	src.u_seeds = StationFracs(30, tolerance);
	src.v_seeds = {0.0, 0.25, 0.5, 0.75};
	src.closed_curve = true;
	src.section = [&](double x_frac, const std::vector<double>& v) {
		auto [z_top, z_bot, y_half, rho_u, rho_l] = SectionParams(x_frac);
		// SectionParams() floors y_half at 1e-6 so the loft never sees a zero
		// width; the mesh wants the true apex instead.
		if (std::abs(y_half_curve_->ZAtEta(x_frac)[1]) < 1e-9) y_half = 0.0;
		double x = x_frac * length;
		Mat out(static_cast<int>(v.size()), 3);
		for (size_t j = 0; j < v.size(); ++j) {
			auto yz = ConicCrossSectionPoint(z_top, z_bot, y_half, rho_u, rho_l, v[j]);
			out(static_cast<int>(j), 0) = x;
			out(static_cast<int>(j), 1) = yz[0];
			out(static_cast<int>(j), 2) = yz[1];
		}
		return out;
	};
	LoftOptions opt;
	opt.tolerance = tolerance;
	opt.min_v = 16;
	opt.apex_tolerance = 1e-5;
	return {{"Main", TriangulateLoft(src, opt)}};
}

// FuselageModel::ExportStep() is defined in step_export.cpp, deliberately
// its own translation unit -- see that file's header comment for why.

} // namespace aerotelic
