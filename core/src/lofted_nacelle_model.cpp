#include "aerotelic/lofted_nacelle_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepLib.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <gp_Pnt.hxx>

#include "aerotelic/nacelle_model.hpp"

namespace aerotelic {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kBlendSamples = 512;
constexpr int kRefineRingPts = 60;

double BinomCoeff(int n, int k) {
	if (k < 0 || k > n) return 0.0;
	double result = 1.0;
	for (int i = 0; i < k; ++i) result *= static_cast<double>(n - i) / (i + 1);
	return result;
}

std::vector<double> Linspace(double a, double b, int n) {
	std::vector<double> out(n);
	for (int i = 0; i < n; ++i) out[i] = a + (b - a) * i / (n - 1);
	return out;
}

// np.linspace(a, b, n, endpoint=False): n points, step (b-a)/n, excludes b.
std::vector<double> LinspaceExcl(double a, double b, int n) {
	std::vector<double> out(n);
	for (int i = 0; i < n; ++i) out[i] = a + (b - a) * i / n;
	return out;
}

double LinInterp(double x, const std::vector<double>& xs, const std::vector<double>& ys) {
	if (x <= xs.front()) return ys.front();
	if (x >= xs.back()) return ys.back();
	auto it = std::upper_bound(xs.begin(), xs.end(), x);
	size_t i = static_cast<size_t>(it - xs.begin()) - 1;
	double t = (x - xs[i]) / (xs[i + 1] - xs[i]);
	return ys[i] + t * (ys[i + 1] - ys[i]);
}

std::vector<double> Unwrap(std::vector<double> a) {
	for (size_t i = 1; i < a.size(); ++i) {
		while (a[i] - a[i - 1] > kPi) a[i] -= 2.0 * kPi;
		while (a[i] - a[i - 1] < -kPi) a[i] += 2.0 * kPi;
	}
	return a;
}

double FModPositive(double x, double m) {
	double r = std::fmod(x, m);
	return r < 0.0 ? r + m : r;
}

// Python's _merge_station(): puts x into sorted fracs, taking over a
// neighbour within min_gap of it rather than adding a near-duplicate.
void MergeStation(std::vector<double>& fracs, double x, double min_gap) {
	int best = 0;
	double best_d = std::abs(fracs[0] - x);
	for (size_t i = 1; i < fracs.size(); ++i) {
		double d = std::abs(fracs[i] - x);
		if (d < best_d) {
			best_d = d;
			best = static_cast<int>(i);
		}
	}
	if (best_d < min_gap) {
		fracs[best] = x;
	} else {
		fracs.push_back(x);
		std::sort(fracs.begin(), fracs.end());
	}
}

void PinXEnds(NurbsSpec& spec) {
	int n = spec.poles.rows();
	spec.poles(0, 0) = 0.0;
	spec.poles(n - 1, 0) = 1.0;
}

} // namespace

NurbsSpec DefaultSectionCurveSpec(double radius) { return DefaultFrontCurveSpec(radius); }

LoftedNacelleSpec::LoftedNacelleSpec() {
	centreline_curve.degree = 2;
	centreline_curve.poles = Mat::FromRows({{0.0, 0.0}, {0.5, 0.0}, {1.0, 0.0}});
	outer_curve.degree = 3;
	outer_curve.poles = Mat::FromRows({{0.00, 0.00}, {0.04, 0.22}, {0.55, 0.20}, {1.00, 0.00}});
	inner_curve.degree = 3;
	inner_curve.poles = Mat::FromRows({{0.00, 0.00}, {0.04, -0.14}, {0.55, -0.10}, {1.00, 0.00}});
	GuideSectionSpec gs;
	gs.station = 0.0;
	gs.curve = DefaultSectionCurveSpec(1.10);
	sections.push_back(gs);
}

LoftedNacelleModel::LoftedNacelleModel(const LoftedNacelleSpec& spec) {
	length = spec.length;
	name = spec.name;
	reference_images = spec.reference_images;

	NurbsSpec cl = spec.centreline_curve, oc = spec.outer_curve, ic = spec.inner_curve;
	PinXEnds(cl);
	PinXEnds(oc);
	PinXEnds(ic);
	centreline_curve_ = NurbsCurve::Create(cl);
	outer_curve_ = NurbsCurve::Create(oc);
	inner_curve_ = NurbsCurve::Create(ic);
	if (centreline_curve_->periodic()) throw std::invalid_argument("the centreline curve must be an open (clamped) curve");
	if (outer_curve_->periodic()) throw std::invalid_argument("the outer curve must be an open (clamped) curve");
	if (inner_curve_->periodic()) throw std::invalid_argument("the inner curve must be an open (clamped) curve");

	if (spec.sections.empty()) throw std::invalid_argument("a lofted nacelle needs at least one guide curve");
	for (const auto& gs : spec.sections) {
		auto curve = NurbsCurve::Create(gs.curve);
		if (!curve->periodic()) throw std::invalid_argument("a guide curve must be a closed (periodic) curve");
		GuideSection section;
		section.station = std::clamp(gs.station, 0.0, 1.0);
		section.curve.reset(static_cast<PeriodicNurbsCurve*>(curve.release()));
		sections_.push_back(std::move(section));
	}
	SortSections();

	chevron_count = spec.chevron_count;
	chevron_depth = spec.chevron_depth;
}

void LoftedNacelleModel::SortSections() {
	std::sort(sections_.begin(), sections_.end(), [](const GuideSection& a, const GuideSection& b) { return a.station < b.station; });
}

GuideSection* LoftedNacelleModel::FindSectionByCurve(const PeriodicNurbsCurve* curve) {
	for (auto& s : sections_) {
		if (s.curve.get() == curve) return &s;
	}
	return nullptr;
}

LoftedNacelleSpec LoftedNacelleModel::Serialize() const {
	LoftedNacelleSpec s;
	s.length = length;
	s.name = name;
	s.centreline_curve = centreline_curve_->Serialize();
	s.outer_curve = outer_curve_->Serialize();
	s.inner_curve = inner_curve_->Serialize();
	s.sections.clear();
	for (auto& sec : sections_) {
		GuideSectionSpec gs;
		gs.station = sec.station;
		gs.curve = sec.curve->Serialize();
		s.sections.push_back(gs);
	}
	s.reference_images = reference_images;
	s.chevron_count = chevron_count;
	s.chevron_depth = chevron_depth;
	return s;
}

void LoftedNacelleModel::SetLength(double new_length) {
	double old = length;
	if (!(new_length > 0.0)) throw std::invalid_argument("length must be positive");
	length = new_length;
	if (!(old > 0.0) || new_length == old) return;
	double k = new_length / old;

	auto scale_cols = [&](NurbsCurve& c, std::vector<int> cols) {
		Mat poles = c.poles();
		for (int i = 0; i < poles.rows(); ++i)
			for (int col : cols) poles(i, col) *= k;
		c.SetPolesAndWeights(poles, c.weights());
	};
	scale_cols(*centreline_curve_, {1});
	scale_cols(*outer_curve_, {1});
	scale_cols(*inner_curve_, {1});
	for (auto& sec : sections_) scale_cols(*sec.curve, {0, 1});
	chevron_depth *= k;
}

void LoftedNacelleModel::AddSection(double station, std::optional<NurbsSpec> curve_spec) {
	if (static_cast<int>(sections_.size()) >= kLoftedNacelleMaxSections) {
		throw std::invalid_argument("no more than " + std::to_string(kLoftedNacelleMaxSections) + " guide curves");
	}
	station = std::clamp(station, 0.0, 1.0);
	NurbsSpec spec;
	if (curve_spec) {
		spec = *curve_spec;
	} else {
		const GuideSection* nearest = &sections_[0];
		for (auto& sec : sections_) {
			if (std::abs(sec.station - station) < std::abs(nearest->station - station)) nearest = &sec;
		}
		spec = nearest->curve->Serialize();
	}
	auto curve = NurbsCurve::Create(spec);
	GuideSection section;
	section.station = station;
	section.curve.reset(static_cast<PeriodicNurbsCurve*>(curve.release()));
	sections_.push_back(std::move(section));
	SortSections();
}

void LoftedNacelleModel::RemoveSection(int index) {
	if (sections_.size() <= 1) throw std::invalid_argument("the body needs at least one guide curve");
	sections_.erase(sections_.begin() + index);
}

std::vector<double> LoftedNacelleModel::SectionStations() const {
	std::vector<double> out;
	for (auto& s : sections_) out.push_back(s.station);
	return out;
}

std::vector<double> LoftedNacelleModel::YAtMany(const NurbsCurve& curve, const std::vector<double>& xs) const {
	Mat f = curve.Sample(200);
	std::vector<double> fx(f.rows()), fy(f.rows());
	for (int i = 0; i < f.rows(); ++i) {
		fx[i] = f(i, 0);
		fy[i] = f(i, 1);
	}
	std::vector<double> out(xs.size());
	for (size_t i = 0; i < xs.size(); ++i) out[i] = LinInterp(xs[i], fx, fy);
	return out;
}

double LoftedNacelleModel::YAt(const NurbsCurve& curve, double x) const { return YAtMany(curve, {x})[0]; }

double LoftedNacelleModel::CentrelineZ(double x_frac) const { return centreline_curve_->ZAtEta(std::clamp(x_frac, 0.0, 1.0))[1]; }

std::pair<double, double> LoftedNacelleModel::Skins(double x_frac) const {
	double x = std::clamp(x_frac, 0.0, 1.0);
	return {outer_curve_->ZAtEta(x)[1], inner_curve_->ZAtEta(x)[1]};
}

double LoftedNacelleModel::WallThickness(double x_frac) const {
	auto [o, i] = Skins(x_frac);
	return o - i;
}

std::vector<double> LoftedNacelleModel::Bernstein(int n_ctrl, double t) {
	int degree = n_ctrl - 1;
	std::vector<double> basis(n_ctrl);
	for (int k = 0; k < n_ctrl; ++k) basis[k] = BinomCoeff(degree, k) * std::pow(t, k) * std::pow(1.0 - t, degree - k);
	return basis;
}

LoftedNacelleModel::GuideTable LoftedNacelleModel::BuildGuideTable(int n_pts) const {
	int n = static_cast<int>(sections_.size());
	std::vector<int> order(n);
	for (int i = 0; i < n; ++i) order[i] = i;
	std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
		return std::clamp(sections_[a].station, 0.0, 1.0) < std::clamp(sections_[b].station, 0.0, 1.0);
	});

	GuideTable tab;
	for (int idx : order) tab.stations.push_back(std::clamp(sections_[idx].station, 0.0, 1.0));

	std::vector<double> params = LinspaceExcl(0.0, 1.0, n_pts);
	for (int idx : order) tab.rings.push_back(sections_[idx].curve->Evaluate(params));

	if (tab.stations.size() > 1) {
		tab.t = Linspace(0.0, 1.0, kBlendSamples);
		tab.x.resize(kBlendSamples);
		for (int ti = 0; ti < kBlendSamples; ++ti) {
			std::vector<double> basis = Bernstein(static_cast<int>(tab.stations.size()), tab.t[ti]);
			double sum = 0.0;
			for (size_t i = 0; i < basis.size(); ++i) sum += basis[i] * tab.stations[i];
			tab.x[ti] = sum;
		}
	} else {
		tab.t = {0.0};
		tab.x = {0.0};
	}
	return tab;
}

std::vector<double> LoftedNacelleModel::BlendWeights(double x_frac, const GuideTable& tab) {
	const auto& st = tab.stations;
	std::vector<double> w(st.size(), 0.0);
	if (st.size() == 1 || x_frac <= st[0]) { w[0] = 1.0; return w; }
	if (x_frac >= st.back()) { w.back() = 1.0; return w; }
	double t = LinInterp(x_frac, tab.x, tab.t);
	return Bernstein(static_cast<int>(st.size()), t);
}

Mat LoftedNacelleModel::Blend(double x_frac, const GuideTable& tab) {
	const auto& st = tab.stations;
	if (st.size() == 1 || x_frac <= st[0]) return tab.rings[0];
	if (x_frac >= st.back()) return tab.rings.back();
	double t = LinInterp(x_frac, tab.x, tab.t);
	std::vector<double> basis = Bernstein(static_cast<int>(st.size()), t);
	int n_pts = tab.rings[0].rows();
	Mat result(n_pts, 2);
	for (int p = 0; p < n_pts; ++p) {
		double rx = 0.0, ry = 0.0;
		for (size_t i = 0; i < basis.size(); ++i) {
			rx += basis[i] * tab.rings[i](p, 0);
			ry += basis[i] * tab.rings[i](p, 1);
		}
		result(p, 0) = rx;
		result(p, 1) = ry;
	}
	return result;
}

Mat LoftedNacelleModel::Offset(const Mat& ring, double distance) {
	int n = ring.rows();
	double min_r = std::numeric_limits<double>::infinity();
	for (int i = 0; i < n; ++i) min_r = std::min(min_r, std::hypot(ring(i, 0), ring(i, 1)));
	if (!(min_r > 0.0)) {
		throw std::invalid_argument("the guide curve passes through the centreline, so there is no radius to measure the skins from");
	}
	Mat out(n, 2);
	for (int i = 0; i < n; ++i) {
		double r = std::hypot(ring(i, 0), ring(i, 1));
		out(i, 0) = ring(i, 0) + distance * ring(i, 0) / r;
		out(i, 1) = ring(i, 1) + distance * ring(i, 1) / r;
	}
	return out;
}

std::pair<Mat, Mat> LoftedNacelleModel::RingsYZImpl(double x_frac, const GuideTable& table) const {
	Mat p = Blend(x_frac, table);
	auto [outer, inner] = Skins(x_frac);
	double base_z = CentrelineZ(x_frac);
	Mat outer_ring = Offset(p, outer), inner_ring = Offset(p, inner);
	for (int i = 0; i < outer_ring.rows(); ++i) outer_ring(i, 1) += base_z;
	for (int i = 0; i < inner_ring.rows(); ++i) inner_ring(i, 1) += base_z;
	return {outer_ring, inner_ring};
}

Mat LoftedNacelleModel::Closed(const Mat& ring) {
	int n = ring.rows();
	Mat out(n + 1, ring.cols());
	for (int i = 0; i < n; ++i) out.set_row(i, ring.row(i));
	out.set_row(n, ring.row(0));
	return out;
}

std::pair<Mat, Mat> LoftedNacelleModel::RingsYZ(double x_frac, int n_pts) const {
	GuideTable table = BuildGuideTable(n_pts);
	auto [outer, inner] = RingsYZImpl(x_frac, table);
	return {Closed(outer), Closed(inner)};
}

Mat LoftedNacelleModel::MeanYZ(double x_frac, int n_pts) const {
	GuideTable table = BuildGuideTable(n_pts);
	Mat p = Blend(x_frac, table);
	double cz = CentrelineZ(x_frac);
	for (int i = 0; i < p.rows(); ++i) p(i, 1) += cz;
	return Closed(p);
}

double LoftedNacelleModel::MeanRadius(double x_frac, int n_pts) const {
	GuideTable table = BuildGuideTable(n_pts);
	Mat p = Blend(x_frac, table);
	double sum_r = 0.0;
	for (int i = 0; i < p.rows(); ++i) sum_r += std::hypot(p(i, 0), p(i, 1));
	return sum_r / p.rows();
}

Mat LoftedNacelleModel::OutlineZs(int n) const {
	std::vector<double> xs = Linspace(0.0, 1.0, n);
	std::vector<double> cz = YAtMany(*centreline_curve_, xs);
	std::vector<double> outs = YAtMany(*outer_curve_, xs);
	GuideTable table = BuildGuideTable(120);
	Mat out(n, 5);
	for (int i = 0; i < n; ++i) {
		Mat ring = Blend(xs[i], table);
		Mat offs = Offset(ring, outs[i]);
		double skin_max = -std::numeric_limits<double>::infinity(), skin_min = std::numeric_limits<double>::infinity();
		for (int p = 0; p < offs.rows(); ++p) {
			double skin = cz[i] + offs(p, 1);
			skin_max = std::max(skin_max, skin);
			skin_min = std::min(skin_min, skin);
		}
		double ring_z_max = -std::numeric_limits<double>::infinity(), ring_z_min = std::numeric_limits<double>::infinity();
		for (int p = 0; p < ring.rows(); ++p) {
			ring_z_max = std::max(ring_z_max, ring(p, 1));
			ring_z_min = std::min(ring_z_min, ring(p, 1));
		}
		out(i, 0) = xs[i];
		out(i, 1) = skin_max;
		out(i, 2) = skin_min;
		out(i, 3) = cz[i] + ring_z_max;
		out(i, 4) = cz[i] + ring_z_min;
	}
	return out;
}

double LoftedNacelleModel::TePerimeter(int n) const {
	Mat yz = MeanYZ(1.0, n);
	double total = 0.0;
	for (int i = 0; i + 1 < yz.rows(); ++i) {
		double dy = yz(i + 1, 0) - yz(i, 0), dz = yz(i + 1, 1) - yz(i, 1);
		total += std::sqrt(dy * dy + dz * dz);
	}
	return total;
}

double LoftedNacelleModel::ChevronTipAngle() const {
	if (chevron_count <= 0 || !(chevron_depth > 0.0)) return 0.0;
	return 2.0 * (180.0 / kPi) * std::atan(0.5 * (TePerimeter() / chevron_count) / chevron_depth);
}

LoftedNacelleMetrics LoftedNacelleModel::Metrics() const {
	std::vector<double> xs = Linspace(0.0, 1.0, 60);
	std::vector<double> outs = YAtMany(*outer_curve_, xs);
	std::vector<double> ins = YAtMany(*inner_curve_, xs);
	GuideTable table = BuildGuideTable(120);
	std::vector<double> diameters(xs.size());
	double max_thickness = -std::numeric_limits<double>::infinity();
	for (size_t i = 0; i < xs.size(); ++i) {
		Mat ring = Blend(xs[i], table);
		Mat offs = Offset(ring, outs[i]);
		double sum_r = 0.0;
		for (int p = 0; p < offs.rows(); ++p) sum_r += std::hypot(offs(p, 0), offs(p, 1));
		diameters[i] = 2.0 * sum_r / offs.rows();
		max_thickness = std::max(max_thickness, outs[i] - ins[i]);
	}
	LoftedNacelleMetrics m;
	m.length = length;
	m.lip_diameter = diameters.front();
	m.max_diameter = *std::max_element(diameters.begin(), diameters.end());
	m.te_diameter = diameters.back();
	m.max_thickness = max_thickness;
	m.n_sections = static_cast<int>(sections_.size());
	m.chevron_depth = chevron_count > 0 ? chevron_depth : 0.0;
	return m;
}

void LoftedNacelleModel::Validate() const {
	std::vector<double> xs = Linspace(0.0, 1.0, 201);
	std::vector<double> outs = YAtMany(*outer_curve_, xs);
	std::vector<double> ins = YAtMany(*inner_curve_, xs);
	for (double v : outs) {
		if (v < 0.0) throw std::invalid_argument("the outer skin falls inside the mean line -- move the guide curve in instead of the outer curve");
	}
	for (double v : ins) {
		if (v > 0.0) throw std::invalid_argument("the inner skin stands outside the mean line -- move the guide curve out instead of the inner curve");
	}
	double max_gap = -std::numeric_limits<double>::infinity();
	for (size_t i = 0; i < xs.size(); ++i) max_gap = std::max(max_gap, outs[i] - ins[i]);
	if (!(max_gap > 0.0)) throw std::invalid_argument("the cowl wall has no thickness: the outer and inner curves are on the mean line everywhere");

	for (size_t i = 0; i < sections_.size(); ++i) {
		if (!(sections_[i].station >= 0.0 && sections_[i].station <= 1.0)) {
			throw std::invalid_argument("guide curve " + std::to_string(i + 1) + " is outside the body");
		}
	}

	GuideTable table = BuildGuideTable(200);
	for (size_t i = 0; i < xs.size(); i += 10) {
		Mat p = Blend(xs[i], table);
		double inner_v = ins[i];
		double min_val = std::numeric_limits<double>::infinity();
		for (int row = 0; row < p.rows(); ++row) min_val = std::min(min_val, std::hypot(p(row, 0), p(row, 1)) + inner_v);
		if (!(min_val > 0.0)) {
			throw std::invalid_argument("the inner skin crosses the centreline -- the wall is thicker than the ring it sits on is wide");
		}
	}
	if (chevron_count > 0 && !(chevron_depth > 0.0)) throw std::invalid_argument("chevrons need a depth greater than zero");
}

std::vector<double> LoftedNacelleModel::StationFracs(int n_stations, std::optional<double> tol_opt, int max_stations) const {
	double tol = tol_opt.value_or(kStationTol);
	std::vector<double> fracs = Linspace(0.0, 1.0, std::max(n_stations, 2));
	for (auto& s : sections_) MergeStation(fracs, std::clamp(s.station, 0.0, 1.0), kMinStationGap);

	GuideTable table = BuildGuideTable(kRefineRingPts);
	std::map<double, std::vector<double>> cache;
	auto dims = [&](double x) -> const std::vector<double>& {
		auto it = cache.find(x);
		if (it != cache.end()) return it->second;
		auto [outer, inner] = Skins(x);
		std::vector<double> v = {CentrelineZ(x), outer, inner};
		Mat blended = Blend(x, table);
		for (int i = 0; i < blended.rows(); ++i) {
			v.push_back(blended(i, 0));
			v.push_back(blended(i, 1));
		}
		return cache.emplace(x, std::move(v)).first->second;
	};

	while (static_cast<int>(fracs.size()) < max_stations) {
		std::vector<std::pair<double, double>> bad; // (err, mid)
		for (size_t i = 0; i + 1 < fracs.size(); ++i) {
			double a = fracs[i], b = fracs[i + 1];
			double mid = 0.5 * (a + b);
			double err = 0.0;
			if (b - a > kMinStationGap) {
				const auto& da = dims(a);
				const auto& db = dims(b);
				const auto& dm = dims(mid);
				for (size_t k = 0; k < dm.size(); ++k) err = std::max(err, std::abs(dm[k] - 0.5 * (da[k] + db[k])));
			}
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

TopoDS_Wire LoftedNacelleModel::SectionWire(double x_frac, const Mat& ring) const {
	BRepBuilderAPI_MakePolygon poly;
	double x = x_frac * length;
	for (int i = 0; i < ring.rows(); ++i) poly.Add(gp_Pnt(x, ring(i, 0), ring(i, 1)));
	poly.Close();
	if (!poly.IsDone()) throw std::runtime_error("could not make a section wire");
	return poly.Wire();
}

TopoDS_Shape LoftedNacelleModel::Shell(const std::vector<TopoDS_Wire>& wires, bool ruled) const {
	BRepOffsetAPI_ThruSections loft(false, ruled);
	for (auto& w : wires) loft.AddWire(w);
	loft.Build();
	if (!loft.IsDone()) throw std::runtime_error("a nacelle skin did not loft");
	return loft.Shape();
}

TopoDS_Shape LoftedNacelleModel::CowlSolid(int n_stations, int n_pts, std::optional<double> station_tol) const {
	GuideTable table = BuildGuideTable(n_pts);
	std::vector<double> fracs = StationFracs(n_stations, station_tol);
	std::vector<TopoDS_Wire> outers, inners;
	for (double x : fracs) {
		auto [o, i] = RingsYZImpl(x, table);
		outers.push_back(SectionWire(x, o));
		inners.push_back(SectionWire(x, i));
	}
	double tol = 1e-6 * std::max(length, 1.0);
	BRepBuilderAPI_Sewing sewing(tol);
	sewing.Add(Shell(outers));
	sewing.Add(Shell(inners));
	int n_f = static_cast<int>(fracs.size());
	int ends[2] = {0, n_f - 1};
	for (int end : ends) {
		if (std::abs(WallThickness(fracs[end])) > tol) sewing.Add(Shell({outers[end], inners[end]}, true));
	}
	sewing.Perform();
	TopoDS_Shape sewn = sewing.SewedShape();

	TopExp_Explorer exp(sewn, TopAbs_SHELL);
	if (!exp.More()) throw std::runtime_error("the nacelle skins did not sew into a shell");
	BRepBuilderAPI_MakeSolid maker(TopoDS::Shell(exp.Current()));
	if (!maker.IsDone()) throw std::runtime_error("the nacelle shell did not close into a solid");
	TopoDS_Solid solid = maker.Solid();
	BRepLib::OrientClosedSolid(solid);
	return solid;
}

LoftedNacelleModel::TeArcTable LoftedNacelleModel::ComputeTeArcTable(int n) const {
	GuideTable table = BuildGuideTable(n);
	Mat rel = Closed(Blend(1.0, table));
	std::vector<double> arc(rel.rows());
	arc[0] = 0.0;
	for (int i = 1; i < rel.rows(); ++i) {
		double dy = rel(i, 0) - rel(i - 1, 0), dz = rel(i, 1) - rel(i - 1, 1);
		arc[i] = arc[i - 1] + std::sqrt(dy * dy + dz * dz);
	}
	return {arc, rel, arc.back()};
}

std::pair<double, double> LoftedNacelleModel::CutScales(double depth, int n_pts) const {
	double L = length;
	GuideTable table = BuildGuideTable(n_pts);
	Mat blend_te = Blend(1.0, table);
	int np = blend_te.rows();
	std::vector<double> r_te(np);
	for (int i = 0; i < np; ++i) r_te[i] = std::hypot(blend_te(i, 0), blend_te(i, 1));
	double cz_te = CentrelineZ(1.0);

	double s_in = std::numeric_limits<double>::infinity(), s_out = 0.0;
	std::vector<double> xs = Linspace(std::max(0.0, 1.0 - depth / L - 0.02), 1.0, 24);
	for (double x : xs) {
		Mat p = Blend(x, table);
		auto [outer, inner] = Skins(x);
		double drift_z = CentrelineZ(x) - cz_te;
		Mat off_in = Offset(p, inner), off_out = Offset(p, outer);
		for (int i = 0; i < np; ++i) {
			s_in = std::min(s_in, std::hypot(off_in(i, 0), off_in(i, 1) + drift_z) / r_te[i]);
			s_out = std::max(s_out, std::hypot(off_out(i, 0), off_out(i, 1) + drift_z) / r_te[i]);
		}
	}
	return {(1.0 - kCutMargin) * s_in, (1.0 + kCutMargin) * s_out};
}

std::vector<TopoDS_Shape> LoftedNacelleModel::ChevronCutters(double depth) const {
	double L = length;
	int n = chevron_count;
	TeArcTable table = ComputeTeArcTable();
	const auto& arc = table.arc;
	const Mat& rel = table.rel;
	double perim = table.perimeter;

	std::vector<double> angs(rel.rows());
	for (int i = 0; i < rel.rows(); ++i) angs[i] = std::atan2(rel(i, 1), rel(i, 0));
	angs = Unwrap(angs);
	bool all_pos = true, all_neg = true;
	for (size_t i = 1; i < angs.size(); ++i) {
		double d = angs[i] - angs[i - 1];
		if (!(d > 0.0)) all_pos = false;
		if (!(d < 0.0)) all_neg = false;
	}
	if (!(all_pos || all_neg)) {
		throw std::invalid_argument("chevrons need a trailing-edge section that every ray from the centreline crosses once (no dents that fold back)");
	}

	int best_idx = 0;
	double best_val = -std::numeric_limits<double>::infinity();
	for (int i = 0; i < rel.rows(); ++i) {
		double val = rel(i, 1) / std::hypot(rel(i, 0), rel(i, 1));
		if (val > best_val) {
			best_val = val;
			best_idx = i;
		}
	}
	double a_top = arc[best_idx];

	auto [r_in, r_out] = CutScales(depth);
	double x_end = L * (1.0 + kCutOverhang);
	double cz_te = CentrelineZ(1.0);

	double width = perim / n;
	double overlap = 0.05 * width;
	int m = kCutSections;

	std::vector<double> rel_x(rel.rows()), rel_y(rel.rows());
	for (int i = 0; i < rel.rows(); ++i) {
		rel_x[i] = rel(i, 0);
		rel_y[i] = rel(i, 1);
	}

	auto quad = [&](double a, double a_mid) -> TopoDS_Wire {
		double s = FModPositive(a, perim);
		double px = LinInterp(s, arc, rel_x);
		double py = LinInterp(s, arc, rel_y);
		double x_cut = L - depth * (1.0 - std::abs(a - a_mid) / (0.5 * width));
		BRepBuilderAPI_MakePolygon poly;
		double corners[4][2] = {{x_cut, r_in}, {x_end, r_in}, {x_end, r_out}, {x_cut, r_out}};
		for (auto& c : corners) poly.Add(gp_Pnt(c[0], px * c[1], cz_te + py * c[1]));
		poly.Close();
		return poly.Wire();
	};

	std::vector<TopoDS_Shape> cutters;
	for (int k = 0; k < n; ++k) {
		double a0 = a_top + k * width;
		double a_mid = a0 + 0.5 * width;
		double ranges[2][2] = {{a0 - overlap, a_mid}, {a_mid, a0 + width + overlap}};
		for (auto& range : ranges) {
			BRepOffsetAPI_ThruSections loft(true, false, 1e-6);
			for (int i = 0; i <= m; ++i) {
				double a = range[0] + (range[1] - range[0]) * i / m;
				loft.AddWire(quad(a, a_mid));
			}
			loft.Build();
			if (!loft.IsDone()) throw std::runtime_error("could not build the cutter for chevron " + std::to_string(k + 1));
			cutters.push_back(loft.Shape());
		}
	}
	return cutters;
}

std::vector<NamedMesh> LoftedNacelleModel::BuildMeshes(double tolerance) const {
	Validate();
	if (chevron_count > 0) return {};

	// One periodic chain in u over [0, 4]: outer skin lip -> TE (x = u),
	// TE rim (outer -> inner at x = 1), inner skin TE -> lip (x = 3 - u),
	// lip rim (inner -> outer at x = 0). The rims are exact ruled annuli,
	// so refinement adds nothing there; where a wall has no thickness the
	// two rings coincide and the builder shares their vertices.
	GuideTable table = BuildGuideTable(kRefineRingPts);
	std::vector<double> fracs = StationFracs(30, tolerance);
	LoftSource src;
	src.u_min = 0.0;
	src.u_max = 4.0;
	src.periodic_u = true;
	src.closed_curve = true;
	src.u_seeds = {0.0, 1.0, 2.0, 3.0};
	for (double x : fracs) {
		src.u_seeds.push_back(x);
		src.u_seeds.push_back(3.0 - x);
	}
	// Station u on the outer skin and 3 - u on the inner are the two sides
	// of the wall at the same x (and the rims map onto themselves), so the
	// wall is sampled with matched vertex rows.
	src.mirror_u = 3.0;
	const double L = length;
	src.section = [&](double u, const std::vector<double>& v) {
		double x_frac, w_inner; // w_inner: 0 on the outer skin, 1 on the inner
		if (u <= 1.0) { x_frac = u; w_inner = 0.0; }
		else if (u <= 2.0) { x_frac = 1.0; w_inner = u - 1.0; }
		else if (u <= 3.0) { x_frac = 3.0 - u; w_inner = 1.0; }
		else { x_frac = 0.0; w_inner = 4.0 - u; }
		std::vector<double> weights = BlendWeights(x_frac, table);
		auto [outer, inner] = Skins(x_frac);
		double distance = (1.0 - w_inner) * outer + w_inner * inner;
		double cz = CentrelineZ(x_frac);
		Mat out(static_cast<int>(v.size()), 3);
		for (size_t j = 0; j < v.size(); ++j) {
			double py = 0.0, pz = 0.0;
			for (size_t i = 0; i < weights.size(); ++i) {
				if (weights[i] == 0.0) continue;
				std::vector<double> p = sections_[i].curve->Evaluate(v[j]);
				py += weights[i] * p[0];
				pz += weights[i] * p[1];
			}
			double r = std::hypot(py, pz);
			if (!(r > 0.0)) throw std::invalid_argument("the guide curve passes through the centreline, so there is no radius to measure the skins from");
			out(static_cast<int>(j), 0) = x_frac * L;
			out(static_cast<int>(j), 1) = py + distance * py / r;
			out(static_cast<int>(j), 2) = pz + distance * pz / r + cz;
		}
		return out;
	};
	LoftOptions opt;
	opt.tolerance = tolerance;
	opt.min_v = 24;
	opt.min_u = 8;
	opt.apex_tolerance = 1e-5;
	return {{"Main", TriangulateLoft(src, opt)}};
}

TopoDS_Shape LoftedNacelleModel::BuildSolid(int n_stations, int n_pts, std::optional<double> station_tol) const {
	Validate();
	TopoDS_Shape solid = CowlSolid(n_stations, n_pts, station_tol);

	if (chevron_count > 0) {
		double depth = chevron_depth;
		if (depth >= 0.9 * length) {
			throw std::invalid_argument("chevrons " + std::to_string(depth) + " deep do not fit a " + std::to_string(length) + " long nacelle");
		}
		TopTools_ListOfShape args, tools;
		args.Append(solid);
		for (auto& c : ChevronCutters(depth)) tools.Append(c);
		BRepAlgoAPI_Cut cut;
		cut.SetArguments(args);
		cut.SetTools(tools);
		cut.SetRunParallel(true);
		cut.Build();
		if (!cut.IsDone()) throw std::runtime_error("cutting the chevrons failed");
		solid = cut.Shape();
	}
	return solid;
}

} // namespace aerotelic
