#include "aerotelic/nacelle_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepLib.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array2OfReal.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt.hxx>

namespace aerotelic {

namespace {

constexpr double kPi = 3.14159265358979323846;

double DegToRad(double d) { return d * kPi / 180.0; }

double FModPositive(double x, double m) {
	double r = std::fmod(x, m);
	return r < 0.0 ? r + m : r;
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

// Pins both skins' end poles at x/L 0 and 1, and the lower skin's end
// poles onto the upper's -- the lip and trailing edge are single points
// shared by the two curves. Operates on the raw pole arrays before either
// NurbsCurve is constructed (Python's _pin_ends()).
void PinEnds(NurbsSpec& upper, NurbsSpec& lower) {
	int nu = upper.poles.rows(), nl = lower.poles.rows();
	upper.poles(0, 0) = 0.0;
	lower.poles(0, 0) = 0.0;
	upper.poles(nu - 1, 0) = 1.0;
	lower.poles(nl - 1, 0) = 1.0;
	lower.poles(0, 1) = upper.poles(0, 1);
	lower.poles(nl - 1, 1) = upper.poles(nu - 1, 1);
}

} // namespace

NurbsSpec DefaultFrontCurveSpec(double radius, int n_poles) {
	std::vector<std::vector<double>> pts(n_poles, std::vector<double>(2));
	for (int i = 0; i < n_poles; ++i) {
		double ang = kPi / 2.0 + 2.0 * kPi * (i + 1) / n_poles;
		pts[i] = {std::cos(ang), std::sin(ang)};
	}
	NurbsSpec unit_spec;
	unit_spec.degree = 3;
	unit_spec.periodic = true;
	unit_spec.poles = Mat::FromRows(pts);
	PeriodicNurbsCurve c(unit_spec);

	Mat samples = c.Sample(721); // [:-1] drops the periodic closure duplicate
	double sum_r = 0.0;
	for (int i = 0; i < samples.rows() - 1; ++i) sum_r += std::hypot(samples(i, 0), samples(i, 1));
	double mean_r = sum_r / (samples.rows() - 1);
	double scale = radius / mean_r;

	NurbsSpec out;
	out.degree = 3;
	out.periodic = true;
	Mat scaled(n_poles, 2);
	for (int i = 0; i < n_poles; ++i) {
		scaled(i, 0) = pts[i][0] * scale;
		scaled(i, 1) = pts[i][1] * scale;
	}
	out.poles = scaled;
	return out;
}

NacelleSpec::NacelleSpec() {
	upper_curve.degree = 3;
	upper_curve.poles = Mat::FromRows({{0.00, 1.00}, {0.002, 1.16}, {0.25, 1.30}, {0.70, 1.24}, {1.00, 0.96}});
	lower_curve.degree = 3;
	lower_curve.poles = Mat::FromRows({{0.00, 1.00}, {0.002, 0.88}, {0.20, 0.87}, {0.70, 0.91}, {1.00, 0.96}});
	front_curve = DefaultFrontCurveSpec(1.0);
}

NacelleModel::NacelleModel(const NacelleSpec& spec) {
	length = spec.length;
	name = spec.name;
	reference_images = spec.reference_images;

	NurbsSpec upper = spec.upper_curve, lower = spec.lower_curve;
	PinEnds(upper, lower);
	upper_curve_ = NurbsCurve::Create(upper);
	lower_curve_ = NurbsCurve::Create(lower);
	if (upper_curve_->periodic() || lower_curve_->periodic()) {
		throw std::invalid_argument("the side-view skins must be open (clamped) curves");
	}

	auto fc = NurbsCurve::Create(spec.front_curve);
	if (!fc->periodic()) throw std::invalid_argument("the front-view lip curve must be a closed (periodic) curve");
	front_curve_.reset(static_cast<PeriodicNurbsCurve*>(fc.release()));

	chevron_count = spec.chevron_count;
	if (spec.chevron_depth) {
		chevron_depth = *spec.chevron_depth;
	} else if (spec.chevron_angle) {
		chevron_depth = DepthFromAngle(*spec.chevron_angle);
	} else {
		chevron_depth = kNacelleDefaultChevronDepth;
	}
}

void NacelleModel::SetFrontCurve(const NurbsSpec& spec) {
	auto fc = NurbsCurve::Create(spec);
	if (!fc->periodic()) throw std::invalid_argument("the front-view lip curve must be a closed (periodic) curve");
	front_curve_.reset(static_cast<PeriodicNurbsCurve*>(fc.release()));
}

NacelleSpec NacelleModel::Serialize() const {
	NacelleSpec s;
	s.length = length;
	s.name = name;
	s.upper_curve = upper_curve_->Serialize();
	s.lower_curve = lower_curve_->Serialize();
	s.front_curve = front_curve_->Serialize();
	s.chevron_count = chevron_count;
	s.chevron_depth = chevron_depth;
	s.reference_images = reference_images;
	return s;
}

void NacelleModel::SetLength(double new_length) {
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
	scale_cols(*upper_curve_, {1});
	scale_cols(*lower_curve_, {1});
	scale_cols(*front_curve_, {0, 1});
	chevron_depth *= k;
}

double NacelleModel::LipRadius() const { return upper_curve_->poles()(0, 1); }

double NacelleModel::TeRadius() const {
	Mat p = upper_curve_->poles();
	return p(p.rows() - 1, 1);
}

Mat NacelleModel::FrontYZ(int n) const {
	Mat hv = front_curve_->Sample(n);
	Mat out(hv.rows(), 2);
	for (int i = 0; i < hv.rows(); ++i) {
		out(i, 0) = -hv(i, 0);
		out(i, 1) = hv(i, 1);
	}
	return out;
}

double NacelleModel::RingPerimeter(double scale) const {
	Mat yz = FrontYZ(2001);
	double total = 0.0;
	for (int i = 0; i + 1 < yz.rows(); ++i) {
		double dy = (yz(i + 1, 0) - yz(i, 0)) * scale, dz = (yz(i + 1, 1) - yz(i, 1)) * scale;
		total += std::sqrt(dy * dy + dz * dz);
	}
	return total;
}

double NacelleModel::TePerimeter() const { return RingPerimeter(TeRadius() / LipRadius()); }

double NacelleModel::ToothWidth() const { return chevron_count <= 0 ? 0.0 : TePerimeter() / chevron_count; }

double NacelleModel::ChevronTipAngle() const {
	double width = ToothWidth(), depth = chevron_depth;
	if (!(width > 0.0) || !(depth > 0.0)) return 0.0;
	return 2.0 * (180.0 / kPi) * std::atan(0.5 * width / depth);
}

double NacelleModel::DepthFromAngle(double angle_deg) const {
	double width = ToothWidth();
	double half = DegToRad(std::clamp(angle_deg, 1e-6, 180.0 - 1e-6)) / 2.0;
	if (!(width > 0.0)) return kNacelleDefaultChevronDepth;
	return 0.5 * width / std::tan(half);
}

NacelleMetrics NacelleModel::Metrics() const {
	Mat yz = FrontYZ();
	int n = yz.rows() - 1;
	double sum_r = 0.0;
	for (int i = 0; i < n; ++i) sum_r += std::hypot(yz(i, 0), yz(i, 1));
	double mean_r = sum_r / n;
	double k = mean_r / LipRadius();
	Mat y_out_pts = upper_curve_->Sample(400);
	double max_abs = 0.0;
	for (int i = 0; i < y_out_pts.rows(); ++i) max_abs = std::max(max_abs, std::abs(y_out_pts(i, 1)));
	NacelleMetrics m;
	m.length = length;
	m.lip_diameter = 2.0 * mean_r;
	m.max_diameter = 2.0 * max_abs * std::abs(k);
	m.te_diameter = 2.0 * TeRadius() * std::abs(k);
	m.chevron_depth = chevron_count > 0 ? chevron_depth : 0.0;
	return m;
}

void NacelleModel::Validate() const {
	if (!(LipRadius() > 0.0)) throw std::invalid_argument("the lip must be above the nacelle axis (y/L > 0)");

	auto check_skin = [](const char* label, const NurbsCurve& c) {
		Mat pts = c.Sample(400);
		double min_y = pts(0, 1);
		for (int i = 1; i < pts.rows(); ++i) min_y = std::min(min_y, pts(i, 1));
		if (!(min_y > 0.0)) throw std::invalid_argument(std::string("the ") + label + " skin crosses the nacelle axis");
	};
	check_skin("outer", *upper_curve_);
	check_skin("inner", *lower_curve_);

	bool any_pos = false, any_neg = false;
	for (int i = 0; i < 97; ++i) {
		double x = 0.02 + (0.98 - 0.02) * i / 96.0;
		double yu = upper_curve_->ZAtEta(x)[1];
		double yl = lower_curve_->ZAtEta(x)[1];
		double gap = yu - yl;
		if (gap > 0.0) any_pos = true;
		if (gap < 0.0) any_neg = true;
	}
	if (any_pos && any_neg) throw std::invalid_argument("the outer and inner skins cross each other");

	Mat yz = FrontYZ();
	int n = yz.rows() - 1;
	double min_r = std::numeric_limits<double>::infinity(), max_r = 0.0;
	for (int i = 0; i < n; ++i) {
		double r = std::hypot(yz(i, 0), yz(i, 1));
		min_r = std::min(min_r, r);
		max_r = std::max(max_r, r);
	}
	if (min_r <= 1e-9 * std::max(max_r, 1.0)) throw std::invalid_argument("the lip curve passes through the nacelle axis");

	if (chevron_count > 0 && !(chevron_depth > 0.0)) throw std::invalid_argument("chevrons need a depth greater than zero");
}

Handle(Geom_BSplineSurface) NacelleModel::SkinSurface(const NurbsCurve& side) const {
	double L = length, y_lip = LipRadius();
	const PeriodicNurbsCurve& fc = *front_curve_;
	int pv = fc.degree();
	// OCC's periodic B-spline numbers its poles from a different starting
	// pole than ours: degree - seam multiplicity + 1 places round (measured
	// for degrees 1-4 at every seam multiplicity; same curve).
	int shift = pv - fc.seam_multiplicity() + 1;

	Mat fpoles_raw = fc.poles();
	std::vector<double> fw_raw = fc.weights();
	int nv = fpoles_raw.rows();
	Mat fpoles(nv, 2);
	std::vector<double> fw(nv);
	for (int j = 0; j < nv; ++j) {
		int src = ((j - shift) % nv + nv) % nv; // np.roll(arr, shift): out[i] = arr[(i-shift) mod n]
		fpoles(j, 0) = fpoles_raw(src, 0);
		fpoles(j, 1) = fpoles_raw(src, 1);
		fw[j] = fw_raw[src];
	}

	Mat sp = side.poles();
	std::vector<double> sw = side.weights();
	int nu = sp.rows();

	TColgp_Array2OfPnt poles(1, nu, 1, nv);
	TColStd_Array2OfReal weights(1, nu, 1, nv);
	for (int i = 0; i < nu; ++i) {
		double x = sp(i, 0) * L;
		double k = sp(i, 1) / y_lip;
		for (int j = 0; j < nv; ++j) {
			double fY = -fpoles(j, 0), fZ = fpoles(j, 1);
			poles.SetValue(i + 1, j + 1, gp_Pnt(x, fY * k, fZ * k));
			weights.SetValue(i + 1, j + 1, sw[i] * fw[j]);
		}
	}

	auto knots_of = [](const NurbsCurve& curve) {
		auto [vals, mults] = curve.GetUniqueKnots();
		TColStd_Array1OfReal ka(1, static_cast<int>(vals.size()));
		TColStd_Array1OfInteger ma(1, static_cast<int>(vals.size()));
		for (size_t m = 0; m < vals.size(); ++m) {
			ka.SetValue(static_cast<int>(m) + 1, vals[m]);
			ma.SetValue(static_cast<int>(m) + 1, mults[m]);
		}
		return std::make_pair(ka, ma);
	};
	auto [uk, um] = knots_of(side);
	auto [vk, vm] = knots_of(fc);
	return new Geom_BSplineSurface(poles, weights, uk, vk, um, vm, side.degree(), pv, false, true);
}

TopoDS_Shape NacelleModel::CowlSolid() const {
	double tol = 1e-6 * std::max(length, 1.0);
	BRepBuilderAPI_Sewing sewing(tol);
	for (const NurbsCurve* curve : {static_cast<const NurbsCurve*>(upper_curve_.get()), static_cast<const NurbsCurve*>(lower_curve_.get())}) {
		Handle(Geom_BSplineSurface) surf = SkinSurface(*curve);
		BRepBuilderAPI_MakeFace face(surf, tol);
		if (!face.IsDone()) throw std::runtime_error("could not make a face from a nacelle skin");
		sewing.Add(face.Face());
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

NacelleModel::ArcTable NacelleModel::ComputeArcTable(int n) const {
	std::vector<double> t(n);
	for (int i = 0; i < n; ++i) t[i] = static_cast<double>(i) / (n - 1);
	Mat hv = front_curve_->Evaluate(t);
	Mat yz(n, 2);
	for (int i = 0; i < n; ++i) {
		yz(i, 0) = -hv(i, 0);
		yz(i, 1) = hv(i, 1);
	}
	std::vector<double> arc(n);
	arc[0] = 0.0;
	for (int i = 1; i < n; ++i) {
		double dy = yz(i, 0) - yz(i - 1, 0), dz = yz(i, 1) - yz(i - 1, 1);
		arc[i] = arc[i - 1] + std::sqrt(dy * dy + dz * dz);
	}
	return {t, arc, yz};
}

std::vector<TopoDS_Shape> NacelleModel::ChevronCutters(double depth) const {
	double L = length;
	int n = chevron_count;
	ArcTable table = ComputeArcTable();
	const auto& t_tab = table.t;
	const auto& arc = table.arc;
	const Mat& yz = table.yz;
	double perim = arc.back();

	std::vector<double> angs(yz.rows());
	for (int i = 0; i < yz.rows(); ++i) angs[i] = std::atan2(yz(i, 1), yz(i, 0));
	angs = Unwrap(angs);
	bool all_pos = true, all_neg = true;
	for (size_t i = 1; i < angs.size(); ++i) {
		double d = angs[i] - angs[i - 1];
		if (!(d > 0.0)) all_pos = false;
		if (!(d < 0.0)) all_neg = false;
	}
	if (!(all_pos || all_neg)) {
		throw std::invalid_argument("chevrons need a lip curve that every ray from the axis crosses once (no dents that fold back)");
	}

	int best_idx = 0;
	double best_val = -std::numeric_limits<double>::infinity();
	for (int i = 0; i < yz.rows(); ++i) {
		double val = yz(i, 1) / std::hypot(yz(i, 0), yz(i, 1));
		if (val > best_val) {
			best_val = val;
			best_idx = i;
		}
	}
	double a_top = arc[best_idx];

	double x0 = std::max(0.0, 1.0 - depth / L - 0.02);
	std::vector<double> ys;
	for (const NurbsCurve* c : {static_cast<const NurbsCurve*>(upper_curve_.get()), static_cast<const NurbsCurve*>(lower_curve_.get())}) {
		Mat pts = c->Sample(400);
		for (int i = 0; i < pts.rows(); ++i) {
			if (pts(i, 0) >= x0) ys.push_back(pts(i, 1));
		}
	}
	ys.push_back(TeRadius());
	double y_lip = LipRadius();
	double y_min = *std::min_element(ys.begin(), ys.end());
	double y_max = *std::max_element(ys.begin(), ys.end());
	double r_in = 0.5 * y_min / y_lip, r_out = 1.5 * y_max / y_lip;
	double x_end = L * (1.0 + kCutOverhang);

	double width = perim / n;
	double overlap = 0.05 * width;
	int m = kCutSections;

	auto quad = [&](double a, double a_mid) -> TopoDS_Wire {
		double a_mod = FModPositive(a, perim);
		double t = LinInterp(a_mod, arc, t_tab);
		std::vector<double> hv = front_curve_->Evaluate(t);
		double px = -hv[0], py = hv[1];
		double x_cut = L - depth * (1.0 - std::abs(a - a_mid) / (0.5 * width));
		BRepBuilderAPI_MakePolygon poly;
		double corners[4][2] = {{x_cut, r_in}, {x_end, r_in}, {x_end, r_out}, {x_cut, r_out}};
		for (auto& pr : corners) poly.Add(gp_Pnt(pr[0], px * pr[1], py * pr[1]));
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

std::vector<NamedMesh> NacelleModel::BuildMeshes(double tolerance) const {
	std::vector<NamedLoft> lofts = BuildLofts();
	if (lofts.empty()) return {};
	LoftOptions opt;
	opt.tolerance = tolerance;
	opt.min_v = 24;
	opt.min_u = 8;
	opt.apex_tolerance = 1e-5;
	return {{"Main", TriangulateLoft(lofts[0].source, opt)}};
}

std::vector<NamedLoft> NacelleModel::BuildLofts() const {
	Validate();
	if (chevron_count > 0) return {};

	// u in [0, 2], periodic: the outer skin at x fraction u, then the inner
	// skin back from the trailing edge at x fraction 2 - u. PinEnds() made
	// both skins share the lip and TE points, so the chain closes. The
	// stations are by x fraction rather than by each curve's own parameter
	// so that a station u on the outer skin and its twin 2 - u on the inner
	// skin sit at the same x: the wall between them is then sampled with
	// matched vertex rows (LoftSource::mirror_u), without which the thin
	// wedge at the trailing edge cannot be given a conforming tetrahedral
	// mesh.
	const double L = length, y_lip = LipRadius();
	LoftSource src;
	src.u_min = 0.0;
	src.u_max = 2.0;
	src.periodic_u = true;
	src.closed_curve = true;
	src.u_seeds = {0.0, 1.0};
	src.mirror_u = 2.0;
	// The point of a side curve at a given x fraction, by bisection on the
	// curve's x (monotone: the poles' x are), exact to ~1e-13 in parameter.
	auto side_at_x = [](const NurbsCurve& side, double x_frac) {
		double lo = 0.0, hi = 1.0;
		for (int it = 0; it < 60 && hi - lo > 1e-13; ++it) {
			double mid = 0.5 * (lo + hi);
			if (side.Evaluate(mid)[0] < x_frac) lo = mid; else hi = mid;
		}
		return side.Evaluate(0.5 * (lo + hi));
	};
	src.section = [this, L, y_lip, side_at_x](double u, const std::vector<double>& v) {
		const NurbsCurve& side = u <= 1.0 ? *upper_curve_ : *lower_curve_;
		double x_frac = std::clamp(u <= 1.0 ? u : 2.0 - u, 0.0, 1.0);
		std::vector<double> sp = side_at_x(side, x_frac);
		double x = x_frac * L;
		double k = sp[1] / y_lip;
		Mat hv = front_curve_->Evaluate(v);
		Mat out(hv.rows(), 3);
		for (int j = 0; j < hv.rows(); ++j) {
			out(j, 0) = x;
			out(j, 1) = -hv(j, 0) * k;
			out(j, 2) = hv(j, 1) * k;
		}
		return out;
	};
	NamedLoft loft;
	loft.name = "Main";
	loft.source = std::move(src);
	return {loft};
}

TopoDS_Shape NacelleModel::BuildSolid() const {
	Validate();
	TopoDS_Shape solid = CowlSolid();

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
