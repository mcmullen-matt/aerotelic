#include "aerotelic/wing_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Writer.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <gp_Pnt.hxx>

namespace aerotelic {

namespace {

constexpr double kPi = 3.14159265358979323846;

double DegToRad(double d) { return d * kPi / 180.0; }

double RoundKey(double x) { return std::round(x * 1e12) / 1e12; }

std::array<double, 3> Cross3(const std::array<double, 3>& a, const std::array<double, 3>& b) {
	return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

// np.interp: clamps outside [xs.front(), xs.back()], assumes xs ascending
// (the caller's responsibility, same contract np.interp has).
double LinInterp(double x, const std::vector<double>& xs, const std::vector<double>& ys) {
	if (x <= xs.front()) return ys.front();
	if (x >= xs.back()) return ys.back();
	auto it = std::upper_bound(xs.begin(), xs.end(), x);
	size_t i = static_cast<size_t>(it - xs.begin()) - 1;
	double t = (x - xs[i]) / (xs[i + 1] - xs[i]);
	return ys[i] + t * (ys[i + 1] - ys[i]);
}

// Shared final step of TrapPlanform/DoubleTrapPlanform/NURBSPlanform's
// airfoil_coords_to_xyz (Python's airfoil_to_cross_section): rotate the
// chord-scaled airfoil-space point by the local twist, then place it along
// x_prime/z_prime at rotation_xyz.
Mat AirfoilToCrossSection(const Mat& airfoil_coords, double twist_angle_deg, double chord, double rotation_fraction,
	const std::array<double, 3>& rotation_xyz, const std::array<double, 3>& x_prime, const std::array<double, 3>& z_prime) {
	double twist_rad = DegToRad(twist_angle_deg);
	double ct = std::cos(twist_rad), st = std::sin(twist_rad);
	int n = airfoil_coords.rows();
	Mat out(n, 3);
	for (int i = 0; i < n; ++i) {
		double nx = chord * (airfoil_coords(i, 0) - rotation_fraction);
		double ny = chord * airfoil_coords(i, 1);
		double rx = ct * nx + st * ny;
		double ry = -st * nx + ct * ny;
		for (int c = 0; c < 3; ++c) out(i, c) = rx * x_prime[c] + ry * z_prime[c] + rotation_xyz[c];
	}
	return out;
}

} // namespace

NurbsSpec LinearCurveSpec(double y0, double y1) {
	NurbsSpec s;
	s.degree = 1;
	s.poles = Mat::FromRows({{0.0, y0}, {1.0, y1}});
	return s;
}

PlanformSpec::PlanformSpec() {
	twist_curve = LinearCurveSpec(kDefaultRootTwistDeg, kDefaultTipTwistDeg);
	dihedral_curve = LinearCurveSpec(kDefaultRootDihedral, kDefaultTipDihedral);
}

std::unique_ptr<Planform> MakePlanform(const PlanformSpec& spec) {
	if (spec.type == "standard") return std::make_unique<TrapPlanform>(spec);
	if (spec.type == "taper") return std::make_unique<TrapTaperPlanform>(spec);
	if (spec.type == "double_trap") return std::make_unique<DoubleTrapPlanform>(spec);
	if (spec.type == "nurbs") return std::make_unique<NURBSPlanform>(spec);
	throw std::invalid_argument("Unknown planform type '" + spec.type + "'");
}

// ===========================================================================
// TrapPlanform
// ===========================================================================

TrapPlanform::TrapPlanform(const PlanformSpec& spec) {
	semi_span_ = spec.semi_span;
	root_chord_ = spec.root_chord;
	tip_chord_ = spec.tip_chord;
	sweep_angle_deg_ = spec.sweep_angle;
	sweep_fraction_ = spec.sweep_fraction;
	rotation_fraction_ = spec.rotation_fraction;
	twist_curve_ = NurbsCurve::Create(spec.twist_curve);
	dihedral_curve_ = std::make_unique<DihedralCurve>(spec.dihedral_curve);
}

Mat TrapPlanform::AirfoilCoordsToXyz(double s, const Mat& airfoil_coords) const {
	std::vector<double> eta_yz = dihedral_curve_->Evaluate(s);
	double eta = eta_yz[0];
	double root_sweep_point = root_chord_ * sweep_fraction_;
	double tip_sweep_point = root_sweep_point + eta * semi_span_ * std::tan(DegToRad(sweep_angle_deg_));
	double chord = root_chord_ * (1.0 - eta) + tip_chord_ * eta;
	double le = tip_sweep_point - sweep_fraction_ * chord;
	std::array<double, 3> rotation_xyz = {le + rotation_fraction_ * chord, eta * semi_span_, eta_yz[1] * semi_span_};

	std::vector<double> deta_yz_ds = dihedral_curve_->Derivative(s);
	double norm = std::sqrt(deta_yz_ds[0] * deta_yz_ds[0] + deta_yz_ds[1] * deta_yz_ds[1]);
	std::array<double, 3> y_prime = {0.0, deta_yz_ds[0] / norm, deta_yz_ds[1] / norm};
	std::array<double, 3> x_prime = {1.0, 0.0, 0.0};
	std::array<double, 3> z_prime = Cross3(x_prime, y_prime);

	double twist_deg = twist_curve_->Evaluate(s)[1];
	return AirfoilToCrossSection(airfoil_coords, twist_deg, chord, rotation_fraction_, rotation_xyz, x_prime, z_prime);
}

Mat TrapPlanform::PlanformOutline(int n) const {
	n = std::max(n, 2);
	Mat out(n, 3);
	for (int i = 0; i < n; ++i) {
		double eta = static_cast<double>(i) / (n - 1);
		double sweep_point = root_chord_ * sweep_fraction_ + eta * semi_span_ * std::tan(DegToRad(sweep_angle_deg_));
		double chord = root_chord_ * (1.0 - eta) + tip_chord_ * eta;
		double le = sweep_point - sweep_fraction_ * chord;
		out(i, 0) = eta;
		out(i, 1) = le;
		out(i, 2) = le + chord;
	}
	return out;
}

PlanformSpec TrapPlanform::Serialize() const {
	PlanformSpec s;
	s.type = "standard";
	s.semi_span = semi_span_;
	s.root_chord = root_chord_;
	s.tip_chord = tip_chord_;
	s.sweep_angle = sweep_angle_deg_;
	s.sweep_fraction = sweep_fraction_;
	s.rotation_fraction = rotation_fraction_;
	s.twist_curve = twist_curve_->Serialize();
	s.dihedral_curve = dihedral_curve_->Serialize();
	return s;
}

PlanformSpec TrapTaperPlanform::Serialize() const {
	PlanformSpec s = TrapPlanform::Serialize();
	s.type = "taper";
	return s;
}

// ===========================================================================
// DoubleTrapPlanform
// ===========================================================================

DoubleTrapPlanform::DoubleTrapPlanform(const PlanformSpec& spec) {
	semi_span_ = spec.semi_span;
	root_chord_ = spec.root_chord;
	tip_chord_ = spec.tip_chord;
	kink_fraction_ = spec.kink_fraction;
	rotation_fraction_ = spec.rotation_fraction;
	leading_edge_sweep_angle_deg_ = spec.leading_edge_sweep_angle;
	trailing_edge_sweep_angle_deg_ = spec.trailing_edge_sweep_angle;
	twist_curve_ = NurbsCurve::Create(spec.twist_curve);
	dihedral_curve_ = std::make_unique<DihedralCurve>(spec.dihedral_curve);
}

Mat DoubleTrapPlanform::AirfoilCoordsToXyz(double s, const Mat& airfoil_coords) const {
	double tip_le = semi_span_ * std::tan(DegToRad(leading_edge_sweep_angle_deg_));
	double kink_le = kink_fraction_ * tip_le;
	double kink_te = root_chord_ + kink_fraction_ * semi_span_ * std::tan(DegToRad(trailing_edge_sweep_angle_deg_));
	double kink_chord = kink_te - kink_le;

	std::vector<double> eta_yz = dihedral_curve_->Evaluate(s);
	double eta = eta_yz[0];
	double le = eta * tip_le;
	double chord;
	if (eta < kink_fraction_) {
		double t = eta / kink_fraction_;
		chord = root_chord_ * (1.0 - t) + kink_chord * t;
	} else {
		double t = (eta - kink_fraction_) / (1.0 - kink_fraction_);
		chord = kink_chord * (1.0 - t) + tip_chord_ * t;
	}

	std::array<double, 3> rotation_xyz = {le + rotation_fraction_ * chord, eta * semi_span_, eta_yz[1] * semi_span_};
	std::vector<double> deta_yz_ds = dihedral_curve_->Derivative(s);
	double norm = std::sqrt(deta_yz_ds[0] * deta_yz_ds[0] + deta_yz_ds[1] * deta_yz_ds[1]);
	std::array<double, 3> y_prime = {0.0, deta_yz_ds[0] / norm, deta_yz_ds[1] / norm};
	std::array<double, 3> x_prime = {1.0, 0.0, 0.0};
	std::array<double, 3> z_prime = Cross3(x_prime, y_prime);

	double twist_deg = twist_curve_->Evaluate(s)[1];
	return AirfoilToCrossSection(airfoil_coords, twist_deg, chord, rotation_fraction_, rotation_xyz, x_prime, z_prime);
}

Mat DoubleTrapPlanform::PlanformOutline(int n) const {
	double k = std::clamp(kink_fraction_, 0.0, 1.0);
	n = std::max(n, 2);
	std::vector<double> eta(n);
	for (int i = 0; i < n; ++i) eta[i] = static_cast<double>(i) / (n - 1);
	eta.push_back(k);
	std::sort(eta.begin(), eta.end());
	eta.erase(std::unique(eta.begin(), eta.end()), eta.end());

	double tip_le = semi_span_ * std::tan(DegToRad(leading_edge_sweep_angle_deg_));
	double kink_le = k * tip_le;
	double kink_te = root_chord_ + k * semi_span_ * std::tan(DegToRad(trailing_edge_sweep_angle_deg_));
	double kink_chord = kink_te - kink_le;

	Mat out(static_cast<int>(eta.size()), 3);
	for (size_t i = 0; i < eta.size(); ++i) {
		double e = eta[i];
		double le = e * tip_le;
		double chord;
		if (e < k) {
			double t = (k > 0.0) ? e / k : 0.0;
			chord = (k > 0.0) ? root_chord_ * (1.0 - t) + kink_chord * t : root_chord_;
		} else {
			double t = (k < 1.0) ? (e - k) / (1.0 - k) : 0.0;
			chord = (k < 1.0) ? kink_chord * (1.0 - t) + tip_chord_ * t : tip_chord_;
		}
		out(static_cast<int>(i), 0) = e;
		out(static_cast<int>(i), 1) = le;
		out(static_cast<int>(i), 2) = le + chord;
	}
	return out;
}

double DoubleTrapPlanform::PlanformArea() const {
	double tip_le = semi_span_ * std::tan(DegToRad(leading_edge_sweep_angle_deg_));
	double kink_le = kink_fraction_ * tip_le;
	double kink_te = root_chord_ + kink_fraction_ * semi_span_ * std::tan(DegToRad(trailing_edge_sweep_angle_deg_));
	double kink_chord = kink_te - kink_le;
	double inboard_area = Span() * kink_fraction_ * 0.5 * (root_chord_ + kink_chord);
	double outboard_area = Span() * (1.0 - kink_fraction_) * 0.5 * (tip_chord_ + kink_chord);
	return inboard_area + outboard_area;
}

PlanformSpec DoubleTrapPlanform::Serialize() const {
	PlanformSpec s;
	s.type = "double_trap";
	s.semi_span = semi_span_;
	s.root_chord = root_chord_;
	s.tip_chord = tip_chord_;
	s.kink_fraction = kink_fraction_;
	s.rotation_fraction = rotation_fraction_;
	s.leading_edge_sweep_angle = leading_edge_sweep_angle_deg_;
	s.trailing_edge_sweep_angle = trailing_edge_sweep_angle_deg_;
	s.twist_curve = twist_curve_->Serialize();
	s.dihedral_curve = dihedral_curve_->Serialize();
	return s;
}

// ===========================================================================
// NURBSPlanform
// ===========================================================================

NURBSPlanform::NURBSPlanform(const PlanformSpec& spec) {
	semi_span_ = spec.semi_span;
	rotation_fraction_ = spec.rotation_fraction;

	double root_sweep_point = kDefaultRootChord * kDefaultSweepFraction;
	double tip_sweep_point = root_sweep_point + kDefaultSemiSpan * std::tan(DegToRad(kDefaultSweepAngleDeg));
	double default_tip_le = tip_sweep_point - kDefaultTipChord * kDefaultSweepFraction;

	le_curve_ = spec.le_curve ? NurbsCurve::Create(*spec.le_curve) : NurbsCurve::Create(LinearCurveSpec(0.0, default_tip_le));
	te_curve_ = spec.te_curve ? NurbsCurve::Create(*spec.te_curve)
		: NurbsCurve::Create(LinearCurveSpec(kDefaultRootChord, default_tip_le + kDefaultTipChord));
	twist_curve_ = NurbsCurve::Create(spec.twist_curve);
	dihedral_curve_ = std::make_unique<DihedralCurve>(spec.dihedral_curve);
}

double NURBSPlanform::ValueAtEta(const NurbsCurve& curve, double eta) {
	if (eta <= 0.0) return curve.poles()(0, 1);
	int n = curve.n_poles();
	if (eta >= 1.0) return curve.poles()(n - 1, 1);

	auto s = curve.Inverse(0, eta, eta);
	if (!s || !(*s >= 0.0 && *s <= 1.0)) {
		Mat pts = curve.Sample(200);
		std::vector<double> x(pts.rows()), sgrid(pts.rows());
		for (int i = 0; i < pts.rows(); ++i) {
			x[i] = pts(i, 0);
			sgrid[i] = static_cast<double>(i) / (pts.rows() - 1);
		}
		double s0 = LinInterp(eta, x, sgrid);
		s = curve.Inverse(0, eta, s0);
		if (!s || !(*s >= 0.0 && *s <= 1.0)) {
			std::vector<double> y(pts.rows());
			for (int i = 0; i < pts.rows(); ++i) y[i] = pts(i, 1);
			return LinInterp(eta, x, y);
		}
	}
	return curve.Evaluate(*s)[1];
}

Mat NURBSPlanform::AirfoilCoordsToXyz(double s, const Mat& airfoil_coords) const {
	if (airfoil_coords.cols() != 2) throw std::invalid_argument("Airfoil coords should contain 2D points");
	std::vector<double> eta_yz = dihedral_curve_->Evaluate(s);
	double eta = eta_yz[0];
	double x_le_v = XLe(eta);
	double chord = XTe(eta) - x_le_v;
	double y = eta * semi_span_, z = eta_yz[1] * semi_span_;
	std::array<double, 3> rotation_xyz = {x_le_v + rotation_fraction_ * chord, y, z};

	std::vector<double> deta_yz_ds = dihedral_curve_->Derivative(s);
	double norm = std::sqrt(deta_yz_ds[0] * deta_yz_ds[0] + deta_yz_ds[1] * deta_yz_ds[1]);
	std::array<double, 3> y_prime = {0.0, deta_yz_ds[0] / norm, deta_yz_ds[1] / norm};
	std::array<double, 3> x_prime = {1.0, 0.0, 0.0};
	std::array<double, 3> z_prime = Cross3(x_prime, y_prime);

	double twist_deg = ValueAtEta(*twist_curve_, eta);
	return AirfoilToCrossSection(airfoil_coords, twist_deg, chord, rotation_fraction_, rotation_xyz, x_prime, z_prime);
}

Mat NURBSPlanform::PlanformOutline(int n) const {
	n = std::max(n, 2);
	std::vector<double> eta(n);
	for (int i = 0; i < n; ++i) eta[i] = static_cast<double>(i) / (n - 1);
	auto at = [&](const NurbsCurve& curve) {
		Mat pts = curve.Sample(n * 2);
		std::vector<double> x(pts.rows()), y(pts.rows());
		for (int i = 0; i < pts.rows(); ++i) {
			x[i] = pts(i, 0);
			y[i] = pts(i, 1);
		}
		std::vector<double> out(n);
		for (int i = 0; i < n; ++i) out[i] = LinInterp(eta[i], x, y);
		return out;
	};
	std::vector<double> le_vals = at(*le_curve_), te_vals = at(*te_curve_);
	Mat out(n, 3);
	for (int i = 0; i < n; ++i) {
		out(i, 0) = eta[i];
		out(i, 1) = le_vals[i];
		out(i, 2) = te_vals[i];
	}
	return out;
}

void NURBSPlanform::SetSemiSpan(double new_value) {
	double old = semi_span_;
	semi_span_ = new_value;
	if (!(old > 0.0) || new_value == old) return;
	double k = new_value / old;
	for (NurbsCurve* c : {le_curve_.get(), te_curve_.get()}) {
		Mat poles = c->poles();
		for (int i = 0; i < poles.rows(); ++i) poles(i, 1) *= k;
		c->SetPolesAndWeights(poles, c->weights());
	}
}

double NURBSPlanform::TipChord() const {
	Mat le = le_curve_->poles(), te = te_curve_->poles();
	return te(te.rows() - 1, 1) - le(le.rows() - 1, 1);
}

double NURBSPlanform::RootChord() const {
	Mat le = le_curve_->poles(), te = te_curve_->poles();
	return te(0, 1) - le(0, 1);
}

double NURBSPlanform::PlanformArea() const {
	const int N = 201;
	std::vector<double> etas(N);
	for (int i = 0; i < N; ++i) etas[i] = static_cast<double>(i) / (N - 1);
	Mat le_pts = le_curve_->Sample(N), te_pts = te_curve_->Sample(N);
	std::vector<double> lex(N), ley(N), tex(N), tey(N);
	for (int i = 0; i < N; ++i) {
		lex[i] = le_pts(i, 0);
		ley[i] = le_pts(i, 1);
		tex[i] = te_pts(i, 0);
		tey[i] = te_pts(i, 1);
	}
	double area = 0.0;
	double prev_chord = LinInterp(etas[0], tex, tey) - LinInterp(etas[0], lex, ley);
	for (int i = 1; i < N; ++i) {
		double chord = LinInterp(etas[i], tex, tey) - LinInterp(etas[i], lex, ley);
		area += 0.5 * (prev_chord + chord) * (etas[i] - etas[i - 1]) * semi_span_;
		prev_chord = chord;
	}
	return 2.0 * area;
}

PlanformSpec NURBSPlanform::Serialize() const {
	PlanformSpec s;
	s.type = "nurbs";
	s.semi_span = semi_span_;
	s.rotation_fraction = rotation_fraction_;
	s.le_curve = le_curve_->Serialize();
	s.te_curve = te_curve_->Serialize();
	s.twist_curve = twist_curve_->Serialize();
	s.dihedral_curve = dihedral_curve_->Serialize();
	return s;
}

// ===========================================================================
// WingModel
// ===========================================================================

WingModel::WingModel(const PlanformSpec& spec, std::string name_) : name(std::move(name_)) {
	owned_planform_ = MakePlanform(spec);
	planform_ = owned_planform_.get();
}

WingModel::WingModel(Planform* external_planform, std::string name_) : name(std::move(name_)) {
	planform_ = external_planform;
}

WingModel::WingModel(const WingModelSpec& spec) : name(spec.name) {
	owned_planform_ = MakePlanform(spec.planform);
	planform_ = owned_planform_.get();
	for (const auto& sec : spec.sections) {
		AddSection(Airfoil::FromCurve(sec.airfoil_curve, sec.airfoil_name), sec.s);
	}
}

WingModelSpec WingModel::Serialize() const {
	WingModelSpec spec;
	spec.name = name;
	spec.planform = planform_->Serialize();
	for (const auto& sec : user_sections_) {
		WingSectionSpec ss;
		ss.s = sec.s;
		ss.airfoil_curve = sec.airfoil->Serialize();
		ss.airfoil_name = sec.airfoil->name;
		spec.sections.push_back(std::move(ss));
	}
	return spec;
}

void WingModel::AddSection(std::shared_ptr<Airfoil> airfoil, double s) {
	user_sections_.push_back({std::move(airfoil), s});
	std::sort(user_sections_.begin(), user_sections_.end(), [](const WingSection& a, const WingSection& b) { return a.s < b.s; });
}

void WingModel::RemoveSection(int index) {
	user_sections_.erase(user_sections_.begin() + index);
}

void WingModel::SetSectionS(int index, double new_s) {
	user_sections_[index].s = new_s;
	std::sort(user_sections_.begin(), user_sections_.end(), [](const WingSection& a, const WingSection& b) { return a.s < b.s; });
}

void WingModel::SetSectionAirfoil(int index, std::shared_ptr<Airfoil> airfoil) {
	user_sections_[index].airfoil = std::move(airfoil);
}

bool WingModel::SpanwiseIsLinear() const {
	auto linear = [](const NurbsCurve& c) { return c.degree() == 1 && c.n_poles() == 2; };
	if (auto* nurbs_pf = dynamic_cast<const NURBSPlanform*>(planform_)) {
		if (!(linear(nurbs_pf->le_curve()) && linear(nurbs_pf->te_curve()))) return false;
	}
	return linear(planform_->TwistCurveRef()) && linear(planform_->DihedralCurveRef());
}

std::optional<std::array<std::array<double, 2>, 3>> WingModel::SpineFrame(double s) const {
	const NurbsCurve& curve = planform_->DihedralCurveRef();
	double semi = planform_->SemiSpan();
	std::vector<double> e = curve.Evaluate(s);
	std::vector<double> d = curve.Derivative(s);
	double nx = d[0], ny = d[1];
	double norm = std::sqrt(nx * nx + ny * ny);
	if (norm < 1e-300) return std::nullopt;
	nx /= norm;
	ny /= norm;
	return std::array<std::array<double, 2>, 3>{
		std::array<double, 2>{e[0] * semi, e[1] * semi},
		std::array<double, 2>{nx, ny},
		std::array<double, 2>{-ny, nx},
	};
}

double WingModel::PlaneMeetDistance(double s1, double s2) const {
	auto f1 = SpineFrame(s1), f2 = SpineFrame(s2);
	if (!f1 || !f2) return std::numeric_limits<double>::infinity();
	const auto& o1 = (*f1)[0];
	const auto& z1 = (*f1)[2];
	const auto& o2 = (*f2)[0];
	const auto& n2 = (*f2)[1];
	double denom = n2[0] * z1[0] + n2[1] * z1[1];
	if (std::abs(denom) < 1e-12) return std::numeric_limits<double>::infinity();
	double dot = n2[0] * (o2[0] - o1[0]) + n2[1] * (o2[1] - o1[1]);
	return dot / denom;
}

double WingModel::SectionReach(double s, std::map<double, double>& cache) const {
	double key = RoundKey(s);
	auto it = cache.find(key);
	if (it != cache.end()) return it->second;

	double rf = planform_->RotationFraction();
	double ax_max = 0.0, ay_max = 0.0;
	for (const auto& sec : user_sections_) {
		Mat p = sec.airfoil->nurbs_curve().poles();
		for (int i = 0; i < p.rows(); ++i) {
			ax_max = std::max(ax_max, std::abs(p(i, 0) - rf));
			ay_max = std::max(ay_max, std::abs(p(i, 1)));
		}
	}
	Mat probe = Mat::FromRows({{0.0, 0.0}, {1.0, 0.0}});
	Mat le_te = planform_->AirfoilCoordsToXyz(s, probe);
	double dx = le_te(1, 0) - le_te(0, 0), dy = le_te(1, 1) - le_te(0, 1), dz = le_te(1, 2) - le_te(0, 2);
	double chord = std::sqrt(dx * dx + dy * dy + dz * dz);
	double tw = DegToRad(planform_->TwistCurveRef().Evaluate(s)[1]);
	double val = chord * (std::abs(std::sin(tw)) * ax_max + std::abs(std::cos(tw)) * ay_max);
	cache[key] = val;
	return val;
}

double WingModel::FoldMargin(double s1, double s2, std::map<double, double>& reach_cache) const {
	double meet = PlaneMeetDistance(s1, s2);
	if (!std::isfinite(meet)) return std::numeric_limits<double>::infinity();
	double reach = std::max(SectionReach(s1, reach_cache), SectionReach(s2, reach_cache));
	return std::abs(meet) - kFoldSafety * reach;
}

std::vector<std::pair<double, bool>> WingModel::CreaseBrackets(std::map<double, double>& reach_cache, double max_half_width) const {
	const NurbsCurve& curve = planform_->DihedralCurveRef();
	std::vector<std::pair<double, bool>> out;
	auto [unique_knots, mults] = curve.GetUniqueKnots();
	for (size_t idx = 0; idx < unique_knots.size(); ++idx) {
		double knot = unique_knots[idx];
		int mult = mults[idx];
		if (knot <= 0.0 || knot >= 1.0) continue;
		if (mult < curve.degree()) continue;

		double h = 1e-3;
		for (int iter = 0; iter < 12; ++iter) {
			double lo = std::max(0.0, knot - h), hi = std::min(1.0, knot + h);
			if (hi - lo < 1e-9) break;
			double meet = std::abs(PlaneMeetDistance(lo, hi));
			double need = kFoldSafety * std::max(SectionReach(lo, reach_cache), SectionReach(hi, reach_cache));
			if (!std::isfinite(meet)) break;
			if (meet >= need) break;
			if (meet < 1e-12) h *= 4.0;
			else h *= std::min(4.0, 1.05 * need / meet);
			if (h > max_half_width) {
				h = max_half_width;
				break;
			}
		}
		out.push_back({std::max(0.0, knot - h), true});
		out.push_back({std::min(1.0, knot + h), true});
	}
	return out;
}

Mat WingModel::ProbeCoords() const {
	double rf = planform_->RotationFraction();
	double t = 0.0;
	for (const auto& sec : user_sections_) {
		Mat p = sec.airfoil->nurbs_curve().poles();
		for (int i = 0; i < p.rows(); ++i) t = std::max(t, std::abs(p(i, 1)));
	}
	return Mat::FromRows({{0.0, 0.0}, {1.0, 0.0}, {rf, t}, {rf, -t}});
}

Mat WingModel::PlacedProbe(double s, const Mat& probe_pts, std::map<double, Mat>& cache) const {
	double key = RoundKey(s);
	auto it = cache.find(key);
	if (it != cache.end()) return it->second;
	Mat placed = planform_->AirfoilCoordsToXyz(s, probe_pts);
	cache[key] = placed;
	return placed;
}

std::pair<double, double> WingModel::StripError(double a, double b, const Mat& probe_pts, std::map<double, Mat>& cache, int n_probe) const {
	Mat pa = PlacedProbe(a, probe_pts, cache);
	Mat pb = PlacedProbe(b, probe_pts, cache);
	double worst = 0.0, at = 0.5 * (a + b);
	for (int k = 1; k <= n_probe; ++k) {
		double w = static_cast<double>(k) / (n_probe + 1);
		double s = a + w * (b - a);
		Mat ps = PlacedProbe(s, probe_pts, cache);
		double dev = 0.0;
		for (int i = 0; i < ps.rows(); ++i) {
			double dx = ps(i, 0) - ((1.0 - w) * pa(i, 0) + w * pb(i, 0));
			double dy = ps(i, 1) - ((1.0 - w) * pa(i, 1) + w * pb(i, 1));
			double dz = ps(i, 2) - ((1.0 - w) * pa(i, 2) + w * pb(i, 2));
			dev = std::max(dev, std::sqrt(dx * dx + dy * dy + dz * dz));
		}
		if (dev > worst) {
			worst = dev;
			at = s;
		}
	}
	return {worst, at};
}

std::vector<std::pair<double, bool>> WingModel::RefineStations(std::vector<std::pair<double, bool>> stations, double tol, int budget,
	double merge_tol, const Mat& probe_pts, std::map<double, Mat>& probe_cache) const {
	std::vector<std::pair<double, bool>> out = std::move(stations);
	budget = std::min(budget, kMaxStations);
	std::map<std::pair<double, double>, std::pair<double, double>> measured;
	while (static_cast<int>(out.size()) < budget) {
		std::vector<std::pair<double, double>> bad; // (err, at)
		for (size_t i = 0; i + 1 < out.size(); ++i) {
			double a = out[i].first, b = out[i + 1].first;
			if (b - a <= 2.0 * merge_tol) continue;
			auto key = std::make_pair(RoundKey(a), RoundKey(b));
			auto it = measured.find(key);
			if (it == measured.end()) it = measured.emplace(key, StripError(a, b, probe_pts, probe_cache)).first;
			double err = it->second.first, at = it->second.second;
			if (err > tol) bad.push_back({err, std::min(std::max(at, a + merge_tol), b - merge_tol)});
		}
		if (bad.empty()) break;
		std::sort(bad.begin(), bad.end(), [](const auto& x, const auto& y) { return x.first > y.first; });
		int n_add = std::min(static_cast<int>(bad.size()), budget - static_cast<int>(out.size()));
		for (int i = 0; i < n_add; ++i) out.push_back({bad[i].second, false});
		std::sort(out.begin(), out.end(), [](const auto& x, const auto& y) { return x.first < y.first; });
	}
	return out;
}

std::vector<double> WingModel::LoftStations(int n_span, std::optional<double> tol_opt) const {
	std::map<double, double> reach_cache;
	std::map<double, Mat> probe_cache;
	Mat probe_pts = ProbeCoords();
	double tol = tol_opt.value_or(station_tol);
	const double merge_tol = 1e-3;

	std::vector<std::pair<double, bool>> wanted = {{0.0, true}, {1.0, true}};
	for (const auto& sec : user_sections_) wanted.push_back({sec.s, true});
	if (planform_->IsDoubleTrap()) wanted.push_back({planform_->KinkFraction(), true});
	for (auto& b : CreaseBrackets(reach_cache)) wanted.push_back(b);

	std::sort(wanted.begin(), wanted.end(), [](const auto& x, const auto& y) {
		if (x.first != y.first) return x.first < y.first;
		return (!x.second) < (!y.second); // locked (True) sorts before unlocked, as in Python's key=(w[0], not w[1])
	});

	std::vector<std::pair<double, bool>> merged;
	for (auto& [s0, locked] : wanted) {
		double s = std::clamp(s0, 0.0, 1.0);
		if (merged.empty() || s - merged.back().first > merge_tol) {
			merged.push_back({s, locked});
		} else if (s == 1.0) {
			merged.back() = {1.0, merged.back().second || locked};
		} else if (locked && !merged.back().second) {
			merged.back() = {merged.back().first, true};
		}
	}

	if (!SpanwiseIsLinear()) {
		merged = RefineStations(merged, tol, std::max(2, n_span), merge_tol, probe_pts, probe_cache);
	}

	std::vector<std::pair<double, bool>> kept = {merged[0]};
	for (size_t i = 1; i < merged.size(); ++i) {
		auto cand = merged[i];
		if (FoldMargin(kept.back().first, cand.first, reach_cache) >= 0.0) {
			kept.push_back(cand);
			continue;
		}
		if (!cand.second) continue;
		if (!kept.back().second) {
			kept.back() = cand;
			continue;
		}
		throw std::runtime_error(
			"Wing cannot be lofted without self-intersection: the sections at s=" + std::to_string(kept.back().first) +
			" and s=" + std::to_string(cand.first) + " are closer than the local curvature allows (margin " +
			std::to_string(FoldMargin(kept.back().first, cand.first, reach_cache)) +
			" m). The section is too thick, or the spanwise curvature too tight, for a single ruled loft -- reduce "
			"thickness, relax the dihedral/twist curve, or split the surface into separate panels at the crease.");
	}
	std::vector<double> out;
	out.reserve(kept.size());
	for (auto& [s, locked] : kept) out.push_back(s);
	return out;
}

std::shared_ptr<Airfoil> WingModel::AirfoilAt(double eta) const {
	std::vector<WingSection> secs = user_sections_;
	std::sort(secs.begin(), secs.end(), [](const WingSection& a, const WingSection& b) { return a.s < b.s; });
	for (const auto& sec : secs) {
		if (std::abs(sec.s - eta) < 1e-9) return sec.airfoil;
	}
	if (eta <= secs.front().s) return secs.front().airfoil;
	if (eta >= secs.back().s) return secs.back().airfoil;

	const WingSection* lo = nullptr;
	const WingSection* hi = nullptr;
	for (const auto& sec : secs) {
		if (sec.s < eta && (!lo || sec.s > lo->s)) lo = &sec;
		if (sec.s > eta && (!hi || sec.s < hi->s)) hi = &sec;
	}
	if (lo->airfoil.get() == hi->airfoil.get()) return lo->airfoil;

	double w = (eta - lo->s) / (hi->s - lo->s);
	const int n = 200;
	Mat lo_pts = lo->airfoil->ContourPoints(n);
	Mat hi_pts = hi->airfoil->ContourPoints(n);
	Mat pts(lo_pts.rows(), 2);
	for (int i = 0; i < lo_pts.rows(); ++i) {
		pts(i, 0) = (1.0 - w) * lo_pts(i, 0) + w * hi_pts(i, 0);
		pts(i, 1) = (1.0 - w) * lo_pts(i, 1) + w * hi_pts(i, 1);
	}
	return Airfoil::FromPoints(pts, 3, lo->airfoil->name + " -> " + hi->airfoil->name);
}

Handle(Geom_BSplineCurve) WingModel::SectionCurve(const WingSection& section) const {
	const NurbsCurve& curve2d = section.airfoil->nurbs_curve();
	Mat poles3d = planform_->AirfoilCoordsToXyz(section.s, curve2d.poles());
	int n = poles3d.rows();
	TColgp_Array1OfPnt polesOCC(1, n);
	for (int j = 0; j < n; ++j) polesOCC.SetValue(j + 1, gp_Pnt(poles3d(j, 0), poles3d(j, 1), poles3d(j, 2)));

	auto [unique_knots, mults] = curve2d.GetUniqueKnots();
	int m = static_cast<int>(unique_knots.size());
	TColStd_Array1OfReal knotsOCC(1, m);
	TColStd_Array1OfInteger multsOCC(1, m);
	for (int j = 0; j < m; ++j) {
		knotsOCC.SetValue(j + 1, unique_knots[j]);
		multsOCC.SetValue(j + 1, mults[j]);
	}
	return new Geom_BSplineCurve(polesOCC, knotsOCC, multsOCC, curve2d.degree());
}

TopoDS_Wire WingModel::SectionWire(const WingSection& section) const {
	const NurbsCurve& curve2d = section.airfoil->nurbs_curve();
	Handle(Geom_BSplineCurve) curveOCC = SectionCurve(section);
	BRepBuilderAPI_MakeWire wireOCC(BRepBuilderAPI_MakeEdge(curveOCC, 0.0, 1.0).Edge());

	if (section.airfoil->te_line() != nullptr) {
		Mat two_pts = Mat::FromRows({curve2d.Evaluate(0.0), curve2d.Evaluate(1.0)});
		Mat te3d = planform_->AirfoilCoordsToXyz(section.s, two_pts);
		gp_Pnt lower(te3d(0, 0), te3d(0, 1), te3d(0, 2));
		gp_Pnt upper(te3d(1, 0), te3d(1, 1), te3d(1, 2));
		wireOCC.Add(BRepBuilderAPI_MakeEdge(upper, lower).Edge());
	}
	if (!wireOCC.IsDone()) throw std::runtime_error("Section wire construction failed at eta=" + std::to_string(section.s));
	return wireOCC.Wire();
}

TopoDS_Shape WingModel::BuildSolid(int n_span, bool ruled, std::optional<double> station_tol_override) const {
	if (user_sections_.empty()) throw std::invalid_argument("No airfoil sections provided to the wing loft");
	std::vector<double> stations = LoftStations(n_span, station_tol_override);

	BRepOffsetAPI_ThruSections loft(true, ruled, 1e-6);
	for (double eta : stations) {
		WingSection sec{AirfoilAt(eta), eta};
		loft.AddWire(SectionWire(sec));
	}
	loft.Build();
	if (!loft.IsDone()) throw std::runtime_error("Lofting failed (ThruSections through " + std::to_string(stations.size()) + " sections)");
	return loft.Shape();
}

// WingModel::ExportStep() is defined in step_export.cpp, deliberately its
// own translation unit -- see that file's header comment for why.

} // namespace aerotelic
