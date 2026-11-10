#include "aerotelic/wing_model.hpp"

#include <algorithm>
#include <array>
#include <functional>
#include <map>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Trsf.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Writer.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <gp_Pnt.hxx>

#include "aerotelic/section_cut.hpp"

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

double DoubleTrapPlanform::ChordAt(double eta) const {
	double tip_le = semi_span_ * std::tan(DegToRad(leading_edge_sweep_angle_deg_));
	double kink_le = kink_fraction_ * tip_le;
	double kink_te = root_chord_ + kink_fraction_ * semi_span_ * std::tan(DegToRad(trailing_edge_sweep_angle_deg_));
	double kink_chord = kink_te - kink_le;
	if (eta < kink_fraction_) {
		double t = eta / kink_fraction_;
		return root_chord_ * (1.0 - t) + kink_chord * t;
	}
	double t = (eta - kink_fraction_) / (1.0 - kink_fraction_);
	return kink_chord * (1.0 - t) + tip_chord_ * t;
}

Mat DoubleTrapPlanform::AirfoilCoordsToXyz(double s, const Mat& airfoil_coords) const {
	double tip_le = semi_span_ * std::tan(DegToRad(leading_edge_sweep_angle_deg_));

	std::vector<double> eta_yz = dihedral_curve_->Evaluate(s);
	double eta = eta_yz[0];
	double le = eta * tip_le;
	double chord = ChordAt(eta);

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
	reference_images = spec.reference_images;
	flaps = spec.flaps;
	structure = spec.structure;
	symmetric = spec.symmetric;
}

WingModelSpec WingModel::Serialize() const {
	WingModelSpec spec;
	spec.name = name;
	spec.planform = planform_->Serialize();
	spec.reference_images = reference_images;
	spec.flaps = flaps;
	spec.structure = structure;
	spec.symmetric = symmetric;
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

std::optional<std::pair<double, double>> WingModel::NegativeChordRange() const {
	// The chord is a smooth function of eta (linear for the trapezoids, a
	// NURBS difference otherwise), so a fine uniform grid plus the exact
	// ends and the user's stations finds any crossing that matters. A
	// zero chord is a pointed tip, not an error.
	constexpr int kSamples = 2000;
	constexpr double kEps = 1e-12;
	std::vector<double> etas;
	etas.reserve(kSamples + 1 + user_sections_.size());
	for (int i = 0; i <= kSamples; ++i) etas.push_back(static_cast<double>(i) / kSamples);
	for (const auto& sec : user_sections_) etas.push_back(std::clamp(sec.s, 0.0, 1.0));
	std::sort(etas.begin(), etas.end());
	std::optional<std::pair<double, double>> range;
	for (double eta : etas) {
		if (planform_->ChordAt(eta) < -kEps) {
			if (!range) range = {eta, eta};
			else range->second = eta;
		}
	}
	return range;
}

void WingModel::RequirePositiveChord() const {
	auto range = NegativeChordRange();
	if (!range) return;
	// No entity name here: the model's own `name` is the model kind ("Wing")
	// in saved files; the application prefixes the scene name.
	char buf[400];
	std::snprintf(buf, sizeof buf,
		"the planform has a negative chord from %.2f%% to %.2f%% of the semi-span: the trailing edge lies ahead of the leading edge "
		"there, so the section is turned inside out. Move the trailing-edge curve aft of the leading-edge curve (or raise the tip "
		"chord above zero) before this wing can be built.",
		100.0 * range->first, 100.0 * range->second);
	throw std::invalid_argument(buf);
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
	// The same airfoil at both ends (the same object, or -- as a saved
	// project reloads them -- two objects holding the same curve): the blend
	// IS that airfoil. Re-fitting it from sampled points would give the
	// interior stations a curve with a different parameterisation from the
	// end stations', which the mesh builder sees as a jump in the surface at
	// the user's sections and refines against in vain (a band of stations
	// bunched at the root and tip).
	if (lo->airfoil.get() == hi->airfoil.get()) return lo->airfoil;
	{
		const NurbsCurve& a = lo->airfoil->nurbs_curve();
		const NurbsCurve& b = hi->airfoil->nurbs_curve();
		bool same = a.degree() == b.degree() && a.n_poles() == b.n_poles();
		if (same) {
			Mat pa = a.poles(), pb = b.poles();
			for (int i = 0; same && i < pa.rows(); ++i)
				for (int j = 0; same && j < pa.cols(); ++j) same = std::abs(pa(i, j) - pb(i, j)) <= 1e-12;
			auto [ka, ma] = a.GetUniqueKnots();
			auto [kb, mb] = b.GetUniqueKnots();
			same = same && ka.size() == kb.size() && ma == mb;
			for (size_t i = 0; same && i < ka.size(); ++i) same = std::abs(ka[i] - kb[i]) <= 1e-12;
		}
		if (same) return lo->airfoil;
	}

	double w = (eta - lo->s) / (hi->s - lo->s);
	// The exact blend in B-spline form (common knots, blended poles): one
	// parameterisation for every station between the two sections, so the
	// loft between them is a true ruled surface and a chordwise parameter
	// means the same point along the whole span. See Airfoil::Blend.
	if (auto blended = Airfoil::Blend(*lo->airfoil, *hi->airfoil, w)) return std::shared_ptr<Airfoil>(std::move(blended));

	// Fallback for curves that cannot share knots (different degrees, a
	// periodic curve): a fit through blended sample points.
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
	RequirePositiveChord();
	std::vector<double> stations = LoftStations(n_span, station_tol_override);

	BRepOffsetAPI_ThruSections loft(true, ruled, 1e-6);
	for (double eta : stations) {
		WingSection sec{AirfoilAt(eta), eta};
		loft.AddWire(SectionWire(sec));
	}
	loft.Build();
	if (!loft.IsDone()) throw std::runtime_error("Lofting failed (ThruSections through " + std::to_string(stations.size()) + " sections)");
	if (!symmetric) return loft.Shape();
	// Both halves as one solid: the half fused with its reflection in
	// y = 0. With a root dihedral the two tilted root sections overlap
	// inside the fuse (the mesh takes the derived section instead, see the
	// header); without one they share the root face.
	const TopoDS_Shape half = loft.Shape();
	gp_Trsf mirror;
	mirror.SetMirror(gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 1.0, 0.0)));
	const TopoDS_Shape other = BRepBuilderAPI_Transform(half, mirror, true).Shape();
	BRepAlgoAPI_Fuse fuse(half, other);
	fuse.SetFuzzyValue(1e-6);
	fuse.Build();
	if (!fuse.IsDone()) throw std::runtime_error("the symmetric wing's two halves could not be fused");
	return fuse.Shape();
}

TriMesh WingModel::BuildMesh(double tolerance, double s_min, double s_max) const {
	if (user_sections_.empty()) throw std::invalid_argument("No airfoil sections provided to the wing loft");
	RequirePositiveChord();
	if (!(s_max > s_min)) throw std::invalid_argument("the wing mesh's span range is empty");
	s_min = std::clamp(s_min, 0.0, 1.0);
	s_max = std::clamp(s_max, 0.0, 1.0);

	// The same stations BuildSolid() lofts through (sections, chord-law
	// breakpoints, crease brackets, fold-safe, refined on the probe points
	// to `tolerance`); the mesh builder refines further on the full
	// section polygon.
	std::vector<double> stations = LoftStations(60, tolerance);
	std::vector<double> seeds = {s_min, s_max};
	for (double s : stations) {
		if (s > s_min && s < s_max) seeds.push_back(s);
	}

	// One blended airfoil per station the builder asks for (AirfoilAt's
	// re-fit between two user sections is the costly part).
	std::map<double, std::shared_ptr<Airfoil>> airfoils;
	LoftSource src;
	src.u_min = s_min;
	src.u_max = s_max;
	src.u_seeds = seeds;
	// An airfoil runs lower TE -> LE -> upper TE over t in [0, 1]; the two
	// ends are joined by the straight trailing-edge segment (or coincide).
	src.closed_curve = false;
	auto airfoil_at = [&](double s) -> const Airfoil& {
		double key = RoundKey(s);
		auto it = airfoils.find(key);
		if (it == airfoils.end()) it = airfoils.emplace(key, AirfoilAt(s)).first;
		return *it->second;
	};
	src.section = [&](double s, const std::vector<double>& v) {
		Mat pts2 = airfoil_at(s).nurbs_curve().Evaluate(v);
		return planform_->AirfoilCoordsToXyz(s, pts2);
	};
	// The upper and lower surfaces are sampled at matched chordwise
	// stations, so the two sheets' vertex rows face each other across the
	// section and the thin wedge at the trailing edge can be given a
	// conforming tetrahedral mesh (see LoftSource::mirror_v).
	src.mirror_v = [&](double s, double v) { return airfoil_at(s).MirrorParameter(v); };
	LoftOptions base;
	base.min_v = 24;
	return TriangulateLoft(src, mesh.Apply(base, tolerance));
}

std::vector<NamedMesh> WingModel::BuildMeshes(double tolerance) const {
	std::vector<WingBodyStructurePlan> plans;
	std::vector<NamedMesh> out = BuildBodies(tolerance, plans);
	last_structure_ = std::move(plans);
	return out;
}

// ---------------------------------------------------------------------
// Flaps
// ---------------------------------------------------------------------

double WingModel::PlanLineFraction(double eta_root, double x_root, double eta_tip, double x_tip, double eta) const {
	// Plan view (x, y) of a chord fraction at a station.
	auto plan = [&](double e, double frac) {
		Mat p = planform_->AirfoilCoordsToXyz(e, Mat::FromRows({{frac, 0.0}}));
		return std::array<double, 2>{p(0, 0), p(0, 1)};
	};
	const auto r = plan(eta_root, x_root), t = plan(eta_tip, x_tip);
	const auto a = plan(eta, 0.0), b = plan(eta, 1.0);
	// The station's chord line a + s (b - a) meets the line r + w (t - r).
	const double dx1 = b[0] - a[0], dy1 = b[1] - a[1], dx2 = t[0] - r[0], dy2 = t[1] - r[1];
	const double det = dx2 * dy1 - dx1 * dy2;
	if (std::fabs(det) <= 1e-12 * std::max(1.0, std::hypot(dx1, dy1) * std::hypot(dx2, dy2))) {
		// Parallel (a zero span, or a line along the chord): fall back to
		// interpolating the fractions.
		double w = eta_tip > eta_root ? (eta - eta_root) / (eta_tip - eta_root) : 0.0;
		return x_root + (x_tip - x_root) * std::clamp(w, 0.0, 1.0);
	}
	const double rx = r[0] - a[0], ry = r[1] - a[1];
	return (dx2 * ry - rx * dy2) / det;
}

double WingModel::HingeFraction(const FlapSpec& flap, double eta) const {
	return PlanLineFraction(flap.eta_root, flap.hinge_root, flap.eta_tip, flap.hinge_tip, eta);
}

std::pair<double, double> WingModel::PocketExtent(const FlapSpec& flap) const {
	const double semi = planform_->SemiSpan();
	const double d = semi > 0.0 ? flap.side_gap / semi : 0.0;
	return {std::max(0.0, flap.eta_root - d), std::min(1.0, flap.eta_tip + d)};
}

void WingModel::ValidateFlaps() const {
	std::vector<const FlapSpec*> sorted;
	for (const FlapSpec& f : flaps) sorted.push_back(&f);
	std::sort(sorted.begin(), sorted.end(), [](const FlapSpec* a, const FlapSpec* b) { return a->eta_root < b->eta_root; });
	for (size_t i = 0; i < sorted.size(); ++i) {
		const FlapSpec& f = *sorted[i];
		if (!(f.eta_root >= 0.0 && f.eta_tip <= 1.0 && f.eta_tip > f.eta_root)) {
			throw std::invalid_argument("flap '" + f.name + "': the span extent must run 0 <= root < tip <= 1");
		}
		if (!(f.hinge_root > 0.0 && f.hinge_root < 1.0 && f.hinge_tip > 0.0 && f.hinge_tip < 1.0)) {
			throw std::invalid_argument("flap '" + f.name + "': the hinge must lie strictly between the leading and trailing edges");
		}
		if (!(f.gap >= 0.0)) throw std::invalid_argument("flap '" + f.name + "': the gap must not be negative");
		if (!(f.side_gap >= 0.0)) throw std::invalid_argument("flap '" + f.name + "': the side gap must not be negative");
		if (i > 0 && PocketExtent(f).first <= PocketExtent(*sorted[i - 1]).second + 1e-9) {
			throw std::invalid_argument("flaps '" + sorted[i - 1]->name + "' and '" + f.name + "' overlap or share an edge station once their side gaps are allowed for; leave a stretch of plain wing between them");
		}
	}
}

// ---------------------------------------------------------------------
// Structure
// ---------------------------------------------------------------------

std::vector<WingSparSpec> WingStructureSpec::DefaultSpars() {
	WingSparSpec front;
	front.name = "Front spar";
	front.x_root = front.x_tip = 0.25;
	WingSparSpec rear;
	rear.name = "Rear spar";
	rear.x_root = rear.x_tip = 0.65;
	return {front, rear};
}

WingStructureSpec WingStructureSpec::Default() {
	return WingStructureSpec();
}

WingStructureSpec WingStructureSpec::DefaultFor(const std::vector<FlapSpec>& flaps) {
	WingStructureSpec s;
	// The rear spar sits at 0.65, or a tenth of the chord ahead of the most
	// forward flap hinge when that is nearer: a spar must stay ahead of the
	// pocket's lips (a rudder hinged at 0.66 would otherwise refuse the
	// default spar).
	double hinge = 1.0;
	for (const FlapSpec& f : flaps) hinge = std::min({hinge, f.hinge_root, f.hinge_tip});
	s.spars[1].x_root = s.spars[1].x_tip = std::clamp(std::min(0.65, hinge - 0.1), 0.3, 0.65);
	return s;
}

bool WingStructureSpec::IsEmpty() const {
	return spars.empty() && stringers.empty() && !ribs.at_sections && !ribs.at_flap_ends && ribs.between <= 0 && ribs.extra.empty();
}

std::vector<double> WingModel::RibStations() const {
	std::vector<double> out = structure.ribs.extra;
	if (structure.ribs.at_sections) {
		for (const WingSection& sec : user_sections_) out.push_back(sec.s);
	}
	if (structure.ribs.at_flap_ends) {
		for (const FlapSpec& f : flaps) {
			const auto [lo, hi] = PocketExtent(f);
			out.push_back(lo);
			out.push_back(hi);
		}
	}
	for (int i = 1; i <= structure.ribs.between; ++i) out.push_back(static_cast<double>(i) / (structure.ribs.between + 1));
	std::vector<double> kept;
	for (double s : out) if (s > 1e-6 && s < 1.0 - 1e-6) kept.push_back(s);
	std::sort(kept.begin(), kept.end());
	std::vector<double> merged;
	for (double s : kept) if (merged.empty() || s - merged.back() > 1e-6) merged.push_back(s);
	return merged;
}

double WingModel::FractionAlong(double eta_root, double x_root, double eta_tip, double x_tip, double eta) {
	if (!(eta_tip > eta_root)) return x_root;
	const double w = std::clamp((eta - eta_root) / (eta_tip - eta_root), 0.0, 1.0);
	return x_root + (x_tip - x_root) * w;
}

void WingModel::ValidateStructure() const {
	auto fraction_ok = [](double x) { return x > 0.0 && x < 1.0; };
	auto range_ok = [](double a, double b) { return a >= 0.0 && b <= 1.0 && b > a; };
	for (const WingSparSpec& sp : structure.spars) {
		if (!(fraction_ok(sp.x_root) && fraction_ok(sp.x_tip))) throw std::invalid_argument("spar '" + sp.name + "': the chord fractions must lie strictly between the leading and trailing edges");
		if (!range_ok(sp.eta_root, sp.eta_tip)) throw std::invalid_argument("spar '" + sp.name + "': the span extent must run 0 <= root < tip <= 1");
		if (!(sp.thickness >= 0.0)) throw std::invalid_argument("spar '" + sp.name + "': the thickness must not be negative");
	}
	for (const WingStringerSpec& st : structure.stringers) {
		if (!(fraction_ok(st.x_root) && fraction_ok(st.x_tip))) throw std::invalid_argument("stringer '" + st.name + "': the chord fractions must lie strictly between the leading and trailing edges");
		if (!range_ok(st.eta_root, st.eta_tip)) throw std::invalid_argument("stringer '" + st.name + "': the span extent must run 0 <= root < tip <= 1");
		if (!(st.thickness >= 0.0) || !(st.height >= 0.0)) throw std::invalid_argument("stringer '" + st.name + "': the thickness and the height must not be negative");
	}
	for (double s : structure.ribs.extra) {
		if (!(s >= 0.0 && s <= 1.0)) throw std::invalid_argument("rib station " + std::to_string(s) + " lies outside the span (0 to 1)");
	}
	if (!(structure.ribs.thickness >= 0.0)) throw std::invalid_argument("the ribs' thickness must not be negative");
	if (structure.ribs.between < 0 || structure.ribs.between > 400) throw std::invalid_argument("the number of ribs between the caps must be 0 to 400");
	// Two spars that both cover a station must keep their order there (one
	// ahead of the other): lines that cross or coincide make one web.
	for (size_t i = 0; i < structure.spars.size(); ++i) {
		for (size_t j = i + 1; j < structure.spars.size(); ++j) {
			const WingSparSpec& a = structure.spars[i];
			const WingSparSpec& b = structure.spars[j];
			const double lo = std::max(a.eta_root, b.eta_root), hi = std::min(a.eta_tip, b.eta_tip);
			if (hi <= lo) continue;
			int sign = 0;
			for (int k = 0; k <= 8; ++k) {
				const double e = lo + (hi - lo) * k / 8.0;
				const double d = FractionAlong(b.eta_root, b.x_root, b.eta_tip, b.x_tip, e) - FractionAlong(a.eta_root, a.x_root, a.eta_tip, a.x_tip, e);
				const int s = d > 1e-6 ? 1 : (d < -1e-6 ? -1 : 0);
				if (s == 0 || (sign != 0 && s != sign)) {
					throw std::invalid_argument("spars '" + a.name + "' and '" + b.name + "' cross or coincide near eta = " + std::to_string(e) + ": keep one ahead of the other");
				}
				sign = s;
			}
		}
	}
	// A spar or a stringer must end clear of a flap's pocket or run right
	// through it, and where it runs through it must stay ahead of the lips
	// (the main element has no skin aft of them there).
	for (const FlapSpec& f : flaps) {
		const auto [lo, hi] = PocketExtent(f);
		auto check_line = [&](const std::string& what, double er, double xr, double et, double xt) {
			if (et <= lo + 1e-9 || er >= hi - 1e-9) return;
			if (er > lo + 1e-9 || et < hi - 1e-9) {
				throw std::invalid_argument(what + " ends inside the pocket of flap '" + f.name + "' (eta " + std::to_string(lo) + " to " + std::to_string(hi) + "): end it outside the pocket or run it through");
			}
			for (double e : {lo, 0.5 * (lo + hi), hi}) {
				std::shared_ptr<Airfoil> af = AirfoilAt(e);
				const double chord = planform_->ChordAt(e);
				const FlapCut cut = SolveFlapCut(*af, HingeFraction(f, e), chord > 0.0 ? f.gap / chord : 0.0);
				const double x_lip = std::min(af->nurbs_curve().Evaluate(cut.t_lower_lip)[0], af->nurbs_curve().Evaluate(cut.t_upper_lip)[0]);
				const double x = FractionAlong(er, xr, et, xt, e);
				if (!(x < x_lip - 1e-6)) {
					char buf[300];
					std::snprintf(buf, sizeof buf, " runs into the pocket of flap '%s' (chord fraction %.3f at eta %.3f, the lip is at %.3f): move it forward of the flap", f.name.c_str(), x, e, x_lip);
					throw std::invalid_argument(what + buf);
				}
			}
		};
		for (const WingSparSpec& sp : structure.spars) check_line("spar '" + sp.name + "'", sp.eta_root, sp.x_root, sp.eta_tip, sp.x_tip);
		for (const WingStringerSpec& st : structure.stringers) check_line("stringer '" + st.name + "'", st.eta_root, st.x_root, st.eta_tip, st.x_tip);
	}
}

namespace {

// A piecewise-linear map between the loft's CANONICAL along-curve
// parameter and a section loop's own parameter at one station. The
// anchors -- the loop's corners, every spar's two feet, every stringer's
// foot -- sit at fixed canonical values on every ring whatever their own
// parameter does along the span, so each is one vertex line of the mesh;
// the loft refines between them.
struct ParamMap {
	std::vector<double> c, t;
	double Forward(double v) const { return LinInterp(v, c, t); }
	double Inverse(double tt) const { return LinInterp(tt, t, c); }
};

struct Anchor {
	enum class Kind { Fixed, SparLower, SparUpper, StringerFoot };
	Kind kind = Kind::Fixed;
	int index = -1;        // the spar's / stringer's index
	double fixed = 0.0;    // Kind::Fixed: the loop's own parameter, the same at every station
};

// A stretch of the main element's span lofted as one segment: a full
// section, or the section notched by one flap. Its parameterisation's
// anchors are the elements that run through the whole stretch (the
// stretches are cut at every element's ends, so an element either covers
// a stretch or misses it).
struct Piece {
	double a = 0.0, b = 1.0;
	int flap = -1;
	// A symmetric wing's root bay: the ruled stretch from the derived
	// section in y = 0 (the root section projected onto the symmetry plane)
	// to the section at b, blended linearly between them.
	bool bay = false;
	std::vector<Anchor> anchors;
	std::vector<double> canon;    // canonical value per anchor, ascending
	std::map<double, ParamMap> maps;
	// The chord fraction of the spar's / stringer's line at eta.
	int FindAnchor(Anchor::Kind kind, int index) const {
		for (size_t i = 0; i < anchors.size(); ++i) if (anchors[i].kind == kind && anchors[i].index == index) return static_cast<int>(i);
		return -1;
	}
};

// MedianEdge, RingIndexAt and StripMesh are structure_plan.hpp's, shared
// with the fuselage's and the nacelles' frame planner.

std::string RibName(int k) { return "Rib " + std::to_string(k); }

}  // namespace

std::vector<NamedMesh> WingModel::BuildBodies(double tolerance, std::vector<WingBodyStructurePlan>& plans) const {
	if (user_sections_.empty()) throw std::invalid_argument("No airfoil sections provided to the wing loft");
	RequirePositiveChord();
	ValidateFlaps();
	plans.clear();
	// A fault in the structure (a spar through a flap's pocket, spars that
	// cross) must never cost the wing its skin: the wing is built with its
	// ribs but without the spars and stringers, and every body's plan
	// carries the reason, which the application shows against the layers.
	std::string structure_problem;
	std::vector<WingSparSpec> spars = structure.spars;
	std::vector<WingStringerSpec> stringers = structure.stringers;
	try {
		ValidateStructure();
	} catch (const std::invalid_argument& e) {
		structure_problem = std::string("the structure was not built: ") + e.what();
		spars.clear();
		stringers.clear();
	}
	std::vector<FlapSpec> sorted = flaps;
	std::sort(sorted.begin(), sorted.end(), [](const FlapSpec& a, const FlapSpec& b) { return a.eta_root < b.eta_root; });
	const int n_flaps = static_cast<int>(sorted.size());
	const std::vector<double> rib_stations = RibStations();

	// The loft's own stations, less any within a hundredth of the span of a
	// rib station: a rib's ring beside a refinement station would make a
	// strip a few millimetres wide between rings a third of a metre apart
	// (a 24:1 jump in element size that a shell analysis reads as a stress
	// spike); the rib station stands in for it, and the loft refines
	// between the stations again on its own anyway.
	const std::vector<double> stations = LoftStations(60, tolerance);
	auto near_rib = [&](double s) {
		for (double r : rib_stations) if (std::fabs(s - r) > 1e-12 && std::fabs(s - r) < 0.01) return true;
		return false;
	};
	auto seeds_in = [&](double a, double b) {
		std::vector<double> out = {a, b};
		for (double s : stations) if (s > a && s < b && !near_rib(s)) out.push_back(s);
		for (double s : rib_stations) if (s > a && s < b) out.push_back(s);
		return out;
	};

	// One blended airfoil, and per flap one section cut, per station asked
	// for.
	std::map<double, std::shared_ptr<Airfoil>> airfoils;
	auto airfoil_at = [&](double s) -> const std::shared_ptr<Airfoil>& {
		double key = RoundKey(s);
		auto it = airfoils.find(key);
		if (it == airfoils.end()) it = airfoils.emplace(key, AirfoilAt(s)).first;
		return it->second;
	};
	struct StationCut {
		std::shared_ptr<Airfoil> airfoil;
		FlapCut cut;
		std::shared_ptr<SectionLoop> main_loop, flap_loop;
	};
	std::map<std::pair<int, double>, StationCut> cuts;
	auto cut_at = [&](int fi, double s) -> const StationCut& {
		auto key = std::make_pair(fi, RoundKey(s));
		auto it = cuts.find(key);
		if (it != cuts.end()) return it->second;
		const FlapSpec& f = sorted[fi];
		StationCut sc;
		sc.airfoil = airfoil_at(s);
		double chord = planform_->ChordAt(s);
		double gap = chord > 0.0 ? f.gap / chord : 0.0;
		try {
			sc.cut = SolveFlapCut(*sc.airfoil, HingeFraction(f, s), gap);
		} catch (const std::invalid_argument& e) {
			throw std::invalid_argument("flap '" + f.name + "' at eta = " + std::to_string(s) + ": " + e.what());
		}
		sc.main_loop = std::make_shared<SectionLoop>(SectionLoop::MainElement(*sc.airfoil, sc.cut));
		sc.flap_loop = std::make_shared<SectionLoop>(SectionLoop::Flap(*sc.airfoil, sc.cut));
		return cuts.emplace(key, std::move(sc)).first->second;
	};
	// Where each element really runs. Since the loft fills every ring with
	// points spread between the elements' feet, the feet must stay apart:
	// where the chord collapses (a pointed tip, the P-38 fin's near-point
	// root) the elements stop together at the last station where every
	// foot on a surface is at least eight tolerances from its neighbours
	// and from the trailing edge. Beyond that the webs would converge into
	// needles; a rib closes the bay instead.
	std::vector<double> spar_root(spars.size()), spar_tip(spars.size()), stringer_root(stringers.size()), stringer_tip(stringers.size());
	std::vector<std::string> clip_notes;
	for (size_t k = 0; k < spars.size(); ++k) { spar_root[k] = spars[k].eta_root; spar_tip[k] = spars[k].eta_tip; }
	for (size_t k = 0; k < stringers.size(); ++k) { stringer_root[k] = stringers[k].eta_root; stringer_tip[k] = stringers[k].eta_tip; }
	auto spar_x = [&](int k, double s) { return FractionAlong(spars[k].eta_root, spars[k].x_root, spars[k].eta_tip, spars[k].x_tip, s); };
	auto stringer_x = [&](int k, double s) { return FractionAlong(stringers[k].eta_root, stringers[k].x_root, stringers[k].eta_tip, stringers[k].x_tip, s); };
	{
		const double apart = 8.0 * std::max(tolerance, 1e-9);
		auto feet_apart = [&](double s) {
			const Airfoil& af = *airfoil_at(s);
			std::vector<std::array<double, 2>> lower = {{1.0, 0.0}}, upper = {{1.0, 0.0}};   // the trailing edge, in airfoil coordinates
			{
				std::vector<double> te0 = af.nurbs_curve().Evaluate(0.0), te1 = af.nurbs_curve().Evaluate(1.0);
				lower[0] = {te0[0], te0[1]};
				upper[0] = {te1[0], te1[1]};
			}
			auto foot = [&](double x, bool up) {
				std::vector<double> p = af.nurbs_curve().Evaluate(af.ParameterAtX(x, up));
				(up ? upper : lower).push_back({p[0], p[1]});
			};
			for (size_t k = 0; k < spars.size(); ++k) {
				if (s < spar_root[k] - 1e-9 || s > spar_tip[k] + 1e-9) continue;
				foot(spar_x(static_cast<int>(k), s), false);
				foot(spar_x(static_cast<int>(k), s), true);
			}
			for (size_t k = 0; k < stringers.size(); ++k) {
				if (s < stringer_root[k] - 1e-9 || s > stringer_tip[k] + 1e-9) continue;
				foot(stringer_x(static_cast<int>(k), s), stringers[k].upper);
			}
			for (std::vector<std::array<double, 2>>* side : {&lower, &upper}) {
				if (side->size() < 2) continue;
				Mat pts(static_cast<int>(side->size()), 2);
				for (size_t i = 0; i < side->size(); ++i) { pts(static_cast<int>(i), 0) = (*side)[i][0]; pts(static_cast<int>(i), 1) = (*side)[i][1]; }
				Mat xyz = planform_->AirfoilCoordsToXyz(s, pts);
				std::vector<std::pair<double, Vec3>> by_x;
				for (int i = 0; i < xyz.rows(); ++i) by_x.push_back({(*side)[i][0], Vec3{xyz(i, 0), xyz(i, 1), xyz(i, 2)}});
				std::sort(by_x.begin(), by_x.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
				for (size_t i = 1; i < by_x.size(); ++i) {
					if (Length(by_x[i].second - by_x[i - 1].second) < apart) return false;
				}
			}
			return true;
		};
		if (!spars.empty() || !stringers.empty()) {
			// The outermost and innermost stations where the feet are apart (the
			// stations between them are taken to be as well), on a fine grid;
			// an element's own end is kept where it is itself fine.
			constexpr int kScan = 1000;
			double s_out = -1.0, s_in = 2.0;
			for (int i = kScan; i >= 0; --i) {
				const double s = static_cast<double>(i) / kScan;
				if (feet_apart(s)) { s_out = s; break; }
			}
			for (int i = 0; i <= kScan; ++i) {
				const double s = static_cast<double>(i) / kScan;
				if (feet_apart(s)) { s_in = s; break; }
			}
			auto clip = [&](const std::string& what, double& root, double& tip) {
				if (s_out < 0.0) {
					clip_notes.push_back(what + " is left out: nowhere along the span are the structure's feet far enough apart for the mesh");
					tip = root;
					return;
				}
				if (tip > s_out + 1e-12 && !feet_apart(tip)) {
					clip_notes.push_back(what + " ends at eta " + std::to_string(s_out) + " rather than " + std::to_string(tip) + ": beyond it the feet of the structure come closer than the mesh can hold apart (a rib closes the bay)");
					tip = s_out;
				}
				if (root < s_in - 1e-12 && !feet_apart(root)) {
					clip_notes.push_back(what + " starts at eta " + std::to_string(s_in) + " rather than " + std::to_string(root) + ": inboard of it the feet of the structure come closer than the mesh can hold apart");
					root = s_in;
				}
				if (!(tip > root + 1e-9)) tip = root;
			};
			for (size_t k = 0; k < spars.size(); ++k) clip("spar '" + spars[k].name + "'", spar_root[k], spar_tip[k]);
			for (size_t k = 0; k < stringers.size(); ++k) clip("stringer '" + stringers[k].name + "'", stringer_root[k], stringer_tip[k]);
		}
	}

	LoftOptions base;
	base.min_v = 24;
	const LoftOptions opt = mesh.Apply(base, tolerance);

	// ---- the main element's pieces ----
	// The span cut at every pocket face and every spar's and stringer's
	// ends; a piece inside a pocket lofts the notched section.
	std::vector<double> bounds = {0.0, 1.0};
	for (const FlapSpec& f : sorted) {
		const auto [lo, hi] = PocketExtent(f);
		bounds.push_back(lo);
		bounds.push_back(hi);
	}
	for (size_t k = 0; k < spars.size(); ++k) { if (spar_tip[k] > spar_root[k]) { bounds.push_back(spar_root[k]); bounds.push_back(spar_tip[k]); } }
	for (size_t k = 0; k < stringers.size(); ++k) { if (stringer_tip[k] > stringer_root[k]) { bounds.push_back(stringer_root[k]); bounds.push_back(stringer_tip[k]); } }
	const double eps = 1e-9;
	const double semi = planform_->SemiSpan();
	// A symmetric wing's root bay (see the header): where the root
	// section straddles y = 0, the stretch from the symmetry plane to the
	// first station -- a loft station, a rib station or a piece boundary --
	// beyond which the whole section lies on the starboard side is lofted
	// as a ruled strip from the DERIVED section (the root section projected
	// onto y = 0) to that station's section. With no dihedral at the root
	// the root section lies in y = 0 already and there is no bay.
	double bay_end = 0.0;
	std::vector<std::string> sym_notes;
	if (symmetric) {
		// The first station on a fine grid whose root airfoil, placed, is
		// wholly at y >= 0 (the sections near the root are the root's
		// airfoil to within the blend; the planform's own placement decides).
		const Mat af0 = airfoil_at(0.0)->nurbs_curve().Sample(64);
		auto min_y = [&](double s) {
			Mat p = planform_->AirfoilCoordsToXyz(s, af0);
			double y = std::numeric_limits<double>::infinity();
			for (int i = 0; i < p.rows(); ++i) y = std::min(y, p(i, 1));
			return y;
		};
		double s_clear = 0.0;
		if (min_y(0.0) < -1e-12 * std::max(semi, 1.0)) {
			constexpr int kScan = 2000;
			s_clear = 1.0;
			for (int i = 1; i <= kScan; ++i) {
				const double s = static_cast<double>(i) / kScan;
				if (min_y(s) >= std::max(tolerance, 1e-12)) { s_clear = s; break; }
			}
		}
		if (s_clear > 0.0) {
			// The first piece boundary beyond the root, and the first seeded
			// station at or beyond the clearance; the bay ends at the nearer.
			double first_bound = 1.0;
			for (double b : bounds) if (b > eps && b < first_bound) first_bound = b;
			bay_end = first_bound;
			for (double s : seeds_in(0.0, 1.0)) {
				if (s > eps && s >= s_clear - eps && s < bay_end) bay_end = s;
			}
			if (bay_end < s_clear - eps) {
				sym_notes.push_back("the root bay ends at eta " + std::to_string(bay_end) + ", inboard of eta " + std::to_string(s_clear) + " where the section first clears the symmetry plane: the two halves overlap there");
			}
			for (double r : rib_stations) {
				if (r > eps && r < bay_end - eps) sym_notes.push_back("rib at eta " + std::to_string(r) + " lies inside the root bay (eta 0 to " + std::to_string(bay_end) + ") and is left out");
			}
			bounds.push_back(bay_end);
		}
	}
	std::sort(bounds.begin(), bounds.end());
	{
		std::vector<double> merged;
		for (double b : bounds) if (merged.empty() || b - merged.back() > 1e-9) merged.push_back(b);
		bounds = merged;
	}
	std::vector<Piece> pieces;
	for (size_t i = 0; i + 1 < bounds.size(); ++i) {
		Piece p;
		p.a = bounds[i];
		p.b = bounds[i + 1];
		if (p.b - p.a <= eps) continue;
		p.bay = bay_end > 0.0 && p.a <= eps;
		for (int fi = 0; fi < n_flaps; ++fi) {
			const auto [lo, hi] = PocketExtent(sorted[fi]);
			if (lo <= p.a + eps && p.b <= hi + eps) p.flap = fi;
		}
		p.anchors.push_back({Anchor::Kind::Fixed, -1, 0.0});
		p.anchors.push_back({Anchor::Kind::Fixed, -1, 1.0});
		if (p.flap >= 0) {
			p.anchors.push_back({Anchor::Kind::Fixed, -1, 0.25});
			p.anchors.push_back({Anchor::Kind::Fixed, -1, 0.5});
			p.anchors.push_back({Anchor::Kind::Fixed, -1, 0.75});
		}
		for (size_t k = 0; k < spars.size(); ++k) {
			if (spar_tip[k] > spar_root[k] && spar_root[k] <= p.a + eps && spar_tip[k] >= p.b - eps) {
				p.anchors.push_back({Anchor::Kind::SparLower, static_cast<int>(k), 0.0});
				p.anchors.push_back({Anchor::Kind::SparUpper, static_cast<int>(k), 0.0});
			}
		}
		for (size_t k = 0; k < stringers.size(); ++k) {
			if (stringer_tip[k] > stringer_root[k] && stringer_root[k] <= p.a + eps && stringer_tip[k] >= p.b - eps) {
				p.anchors.push_back({Anchor::Kind::StringerFoot, static_cast<int>(k), 0.0});
			}
		}
		pieces.push_back(std::move(p));
	}

	// An anchor's own parameter at a station: on the airfoil curve for a
	// full piece, on the notched loop for a pocket piece.
	auto own_params = [&](const Piece& p, double s) {
		std::vector<double> t(p.anchors.size());
		const Airfoil& af = *airfoil_at(s);
		const SectionLoop* loop = p.flap >= 0 ? cut_at(p.flap, s).main_loop.get() : nullptr;
		for (size_t i = 0; i < p.anchors.size(); ++i) {
			const Anchor& an = p.anchors[i];
			double v = 0.0;
			switch (an.kind) {
			case Anchor::Kind::Fixed: v = an.fixed; break;
			case Anchor::Kind::SparLower: v = af.ParameterAtX(spar_x(an.index, s), false); break;
			case Anchor::Kind::SparUpper: v = af.ParameterAtX(spar_x(an.index, s), true); break;
			case Anchor::Kind::StringerFoot: v = af.ParameterAtX(stringer_x(an.index, s), stringers[an.index].upper); break;
			}
			if (loop && an.kind != Anchor::Kind::Fixed) v = loop->ParameterAtAirfoil(v);
			if (!std::isfinite(v)) throw std::runtime_error("the structure's foot at eta = " + std::to_string(s) + " is off the section (a spar behind a flap's lip?)");
			t[i] = v;
		}
		return t;
	};
	// Canonical values: the own parameters at the piece's start, which
	// orders the anchors; the map at any station pairs the two lists.
	for (Piece& p : pieces) {
		std::vector<double> t0 = own_params(p, p.a);
		std::vector<size_t> order(p.anchors.size());
		for (size_t i = 0; i < order.size(); ++i) order[i] = i;
		std::sort(order.begin(), order.end(), [&](size_t x, size_t y) { return t0[x] < t0[y]; });
		std::vector<Anchor> sorted_anchors;
		p.canon.clear();
		for (size_t i : order) {
			if (!p.canon.empty() && t0[i] - p.canon.back() <= 1e-9) {
				throw std::invalid_argument("two structural elements meet the skin at the same point at eta = " + std::to_string(p.a) + ": move one of them");
			}
			sorted_anchors.push_back(p.anchors[i]);
			p.canon.push_back(t0[i]);
		}
		p.anchors = std::move(sorted_anchors);
	}
	auto map_at = [&](Piece& p, double s) -> const ParamMap& {
		const double key = RoundKey(s);
		auto it = p.maps.find(key);
		if (it != p.maps.end()) return it->second;
		ParamMap m;
		m.c = p.canon;
		m.t = own_params(p, s);
		for (size_t i = 1; i < m.t.size(); ++i) {
			if (!(m.t[i] > m.t[i - 1])) throw std::runtime_error("the structure's feet change order along the span at eta = " + std::to_string(s) + " (two elements cross on the skin)");
		}
		return p.maps.emplace(key, std::move(m)).first->second;
	};

	// ---- the main element's segments ----
	std::vector<LoftSource> segments;
	std::vector<LoftBreak> breaks;
	for (size_t pi = 0; pi < pieces.size(); ++pi) {
		Piece& p = pieces[pi];
		LoftSource src;
		src.u_min = p.a;
		src.u_max = p.b;
		// The root bay is one ruled strip: its two end rings only (the loft
		// adds stations of its own only where the strip is too long).
		src.u_seeds = p.bay ? std::vector<double>{p.a, p.b} : seeds_in(p.a, p.b);
		for (double c : p.canon) if (c > 1e-9 && c < 1.0 - 1e-9) src.v_seeds.push_back(c);
		src.closed_curve = p.flap >= 0;
		src.uniform_rings = true;
		Piece* pp = &p;
		if (p.flap < 0) {
			src.section = [&, pp](double s, const std::vector<double>& v) {
				const ParamMap& m = map_at(*pp, s);
				std::vector<double> t(v.size());
				for (size_t i = 0; i < v.size(); ++i) t[i] = m.Forward(v[i]);
				return planform_->AirfoilCoordsToXyz(s, airfoil_at(s)->nurbs_curve().Evaluate(t));
			};
			src.mirror_v = [&, pp](double s, double v) {
				const ParamMap& m = map_at(*pp, s);
				return m.Inverse(airfoil_at(s)->MirrorParameter(m.Forward(v)));
			};
		} else {
			const int fi = p.flap;
			src.section = [&, pp, fi](double s, const std::vector<double>& v) {
				const ParamMap& m = map_at(*pp, s);
				std::vector<double> t(v.size());
				for (size_t i = 0; i < v.size(); ++i) t[i] = m.Forward(v[i]);
				return planform_->AirfoilCoordsToXyz(s, cut_at(fi, s).main_loop->Points(t));
			};
			src.mirror_v = [&, pp, fi](double s, double v) {
				const ParamMap& m = map_at(*pp, s);
				return m.Inverse(cut_at(fi, s).main_loop->Mirror(m.Forward(v)));
			};
		}
		if (p.bay) {
			// The derived section at s = 0 (the root section with y dropped
			// to 0) blended linearly into the section at the bay's end: the
			// strip is ruled exactly, so the loft adds no station between,
			// and the ends are the two sections to the bit.
			std::function<Mat(double, const std::vector<double>&)> plain = src.section;
			const double b = p.b;
			src.section = [plain, b](double s, const std::vector<double>& v) {
				Mat a = plain(0.0, v);
				for (int i = 0; i < a.rows(); ++i) a(i, 1) = 0.0;
				const double w = b > 0.0 ? std::clamp(s / b, 0.0, 1.0) : 0.0;
				if (w <= 0.0) return a;
				Mat e = plain(b, v);
				if (w >= 1.0) return e;
				Mat out(a.rows(), 3);
				for (int i = 0; i < a.rows(); ++i) {
					for (int j = 0; j < 3; ++j) out(i, j) = (1.0 - w) * a(i, j) + w * e(i, j);
				}
				return out;
			};
			src.cap_start = false;
		} else if (symmetric && pi == 0) {
			// No bay (no dihedral at the root): the root ring lies in y = 0
			// and is the shared ring itself; no cap there either.
			src.cap_start = false;
		}
		segments.push_back(std::move(src));
		if (pi + 1 < pieces.size()) {
			Piece* pa = &pieces[pi];
			Piece* pb = &pieces[pi + 1];
			const double e = p.b;
			LoftBreak br;
			if (pa->flap < 0 && pb->flap < 0) {
				// Full to full: the whole ring is shared.
				br.va0 = 0.0; br.va1 = 1.0; br.vb0 = 0.0; br.vb1 = 1.0;
				br.a_to_b = [&, pa, pb, e](double v) { return map_at(*pb, e).Inverse(map_at(*pa, e).Forward(v)); };
				br.b_to_a = [&, pa, pb, e](double v) { return map_at(*pa, e).Inverse(map_at(*pb, e).Forward(v)); };
			} else if (pa->flap < 0) {
				// Full to notched: the skin between the lips, t in
				// [t_lower_lip, t_upper_lip] against the loop's v in [0, 0.5].
				const StationCut& sc = cut_at(pb->flap, e);
				std::shared_ptr<SectionLoop> loop = sc.main_loop;
				br.va0 = map_at(*pa, e).Inverse(sc.cut.t_lower_lip);
				br.va1 = map_at(*pa, e).Inverse(sc.cut.t_upper_lip);
				br.vb0 = map_at(*pb, e).Inverse(0.0);
				br.vb1 = map_at(*pb, e).Inverse(0.5);
				br.a_to_b = [&, pa, pb, e, loop](double v) { return map_at(*pb, e).Inverse(loop->ParameterAtAirfoil(map_at(*pa, e).Forward(v))); };
				br.b_to_a = [&, pa, pb, e, loop](double v) { return map_at(*pa, e).Inverse(loop->AirfoilParameter(map_at(*pb, e).Forward(v))); };
			} else if (pb->flap < 0) {
				const StationCut& sc = cut_at(pa->flap, e);
				std::shared_ptr<SectionLoop> loop = sc.main_loop;
				br.va0 = map_at(*pa, e).Inverse(0.0);
				br.va1 = map_at(*pa, e).Inverse(0.5);
				br.vb0 = map_at(*pb, e).Inverse(sc.cut.t_lower_lip);
				br.vb1 = map_at(*pb, e).Inverse(sc.cut.t_upper_lip);
				br.a_to_b = [&, pa, pb, e, loop](double v) { return map_at(*pb, e).Inverse(loop->AirfoilParameter(map_at(*pa, e).Forward(v))); };
				br.b_to_a = [&, pa, pb, e, loop](double v) { return map_at(*pa, e).Inverse(loop->ParameterAtAirfoil(map_at(*pb, e).Forward(v))); };
			} else {
				throw std::invalid_argument("two flap pockets meet at eta = " + std::to_string(e) + " with no plain wing between them");
			}
			breaks.push_back(std::move(br));
		}
	}

	std::vector<NamedMesh> out;
	std::vector<LoftRings> rings;
	out.push_back({"Main", TriangulateLoftChain(segments, breaks, opt, nullptr, &rings)});
	if (symmetric) {
		// The ring in the symmetry plane is at y = 0 to the bit, so the port
		// half (the reflection below) shares its vertices exactly.
		const double eps_y = 1e-9 * std::max(semi, 1.0);
		for (Vec3& v : out.back().mesh.vertices) if (std::fabs(v.y) <= eps_y) v.y = 0.0;
	}
	auto reflected = [](Vec3 v) { v.y = -v.y; return v; };

	// ---- the main element's structure, on its rings ----
	{
		const TriMesh& body = out.back().mesh;
		WingBodyStructurePlan plan;
		plan.body = "Main";
		plan.rib_thickness = structure.ribs.thickness;
		plan.rib_material = structure.ribs.material;
		plan.rib_size = MedianEdge(body);
		plan.notes = clip_notes;
		plan.notes.insert(plan.notes.end(), sym_notes.begin(), sym_notes.end());
		if (!structure_problem.empty()) plan.notes.insert(plan.notes.begin(), structure_problem);
		// The piece a rib station falls in: strictly inside one, or at a
		// boundary the notched one (its ring bounds the disc ahead of the
		// pocket; the full ring's aft stretch is the pocket face itself).
		auto piece_for = [&](double u) {
			int best = -1;
			for (size_t pi = 0; pi < pieces.size(); ++pi) {
				const Piece& p = pieces[pi];
				if (u < p.a - eps || u > p.b + eps) continue;
				if (best < 0 || p.flap >= 0) best = static_cast<int>(pi);
			}
			return best;
		};
		// Each spar's feet along the span: (u, lower, upper) per ring of
		// every piece it covers, the duplicate at a piece boundary dropped.
		struct Foot { double u; Vec3 lower, upper; };
		std::vector<std::vector<Foot>> spar_feet(spars.size());
		std::vector<std::vector<std::pair<double, Vec3>>> stringer_feet(stringers.size());
		std::vector<std::vector<std::pair<double, Vec3>>> stringer_in(stringers.size());
		for (size_t pi = 0; pi < pieces.size(); ++pi) {
			const Piece& p = pieces[pi];
			const LoftRings& R = rings[pi];
			for (size_t ri = 0; ri < R.u.size(); ++ri) {
				const double u = R.u[ri];
				const std::vector<int>& ring = R.rings[ri];
				const std::vector<double>& rv = R.ring_v[ri];
				for (size_t k = 0; k < spars.size(); ++k) {
					const int lo = p.FindAnchor(Anchor::Kind::SparLower, static_cast<int>(k));
					const int up = p.FindAnchor(Anchor::Kind::SparUpper, static_cast<int>(k));
					if (lo < 0 || up < 0) continue;
					const int jl = RingIndexAt(rv, p.canon[lo]), ju = RingIndexAt(rv, p.canon[up]);
					if (jl < 0 || ju < 0) continue;
					if (!spar_feet[k].empty() && std::fabs(spar_feet[k].back().u - u) <= 1e-9) continue;
					spar_feet[k].push_back({u, body.vertices[ring[jl]], body.vertices[ring[ju]]});
				}
				for (size_t k = 0; k < stringers.size(); ++k) {
					const int fa = p.FindAnchor(Anchor::Kind::StringerFoot, static_cast<int>(k));
					if (fa < 0) continue;
					const int j = RingIndexAt(rv, p.canon[fa]);
					if (j < 0) continue;
					if (!stringer_feet[k].empty() && std::fabs(stringer_feet[k].back().first - u) <= 1e-9) continue;
					// Inward along the section: normal to the skin in the section
					// plane, towards the section's centroid.
					const int nr = static_cast<int>(ring.size());
					const Vec3& pt = body.vertices[ring[j]];
					const Vec3& prev = body.vertices[ring[(j - 1 + nr) % nr]];
					const Vec3& next = body.vertices[ring[(j + 1) % nr]];
					Mat probe = planform_->AirfoilCoordsToXyz(u, Mat::FromRows({{0.0, 0.0}, {1.0, 0.0}, {0.5, 0.1}}));
					Vec3 p0{probe(0, 0), probe(0, 1), probe(0, 2)}, p1{probe(1, 0), probe(1, 1), probe(1, 2)}, p2{probe(2, 0), probe(2, 1), probe(2, 2)};
					Vec3 nrm = Cross(p1 - p0, p2 - p0);
					Vec3 tangent = next - prev;
					Vec3 inward = Cross(nrm, tangent);
					const double len = Length(inward);
					if (len <= 1e-300) continue;
					inward = inward * (1.0 / len);
					Vec3 centroid{0, 0, 0};
					for (int idx : ring) centroid = centroid + body.vertices[idx];
					centroid = centroid * (1.0 / nr);
					if (Dot(inward, centroid - pt) < 0.0) inward = inward * -1.0;
					stringer_feet[k].push_back({u, pt});
					stringer_in[k].push_back({u, pt + inward * stringers[k].height});
				}
			}
		}
		// Across each spar's web: enough nodes for the rib size at its
		// deepest station, the same count on every rib it crosses.
		std::vector<int> spar_across(spars.size(), 2);
		for (size_t k = 0; k < spars.size(); ++k) {
			double deepest = 0.0;
			for (const Foot& f : spar_feet[k]) deepest = std::max(deepest, Length(f.upper - f.lower));
			spar_across[k] = plan.rib_size > 0.0 ? std::max(2, static_cast<int>(std::lround(deepest / plan.rib_size)) + 1) : 2;
		}
		// The ribs: at the rib stations, and -- symmetric -- one in the
		// symmetry plane on the derived (or root) ring, the stations inside
		// the root bay left out (noted above).
		std::vector<double> main_rib_stations = rib_stations;
		if (symmetric) main_rib_stations.insert(main_rib_stations.begin(), 0.0);
		int rib_no = 0;
		for (double u : main_rib_stations) {
			if (bay_end > 0.0 && u > eps && u < bay_end - eps) continue;
			const int pi = piece_for(u);
			if (pi < 0) { plan.notes.push_back("rib at eta " + std::to_string(u) + ": no piece of the span holds it"); continue; }
			const Piece& p = pieces[pi];
			const LoftRings& R = rings[pi];
			const int ri = R.Find(u);
			if (ri < 0) { plan.notes.push_back("rib at eta " + std::to_string(u) + ": the loft has no ring there"); continue; }
			WingRibPlan rib;
			rib.eta = u;
			rib.name = symmetric && u <= eps ? std::string("Centre rib") : RibName(++rib_no);
			for (int idx : R.rings[ri]) if (rib.rim.empty() || rib.rim.back() != idx) rib.rim.push_back(idx);
			while (rib.rim.size() > 1 && rib.rim.back() == rib.rim.front()) rib.rim.pop_back();
			if (rib.rim.size() < 3) { plan.notes.push_back(rib.name + ": the section at eta " + std::to_string(u) + " has no area"); continue; }
			for (size_t k = 0; k < spars.size(); ++k) {
				const int lo = p.FindAnchor(Anchor::Kind::SparLower, static_cast<int>(k));
				const int up = p.FindAnchor(Anchor::Kind::SparUpper, static_cast<int>(k));
				if (lo < 0 || up < 0) continue;
				const int jl = RingIndexAt(R.ring_v[ri], p.canon[lo]), ju = RingIndexAt(R.ring_v[ri], p.canon[up]);
				if (jl < 0 || ju < 0) continue;
				rib.chords.push_back({R.rings[ri][jl], R.rings[ri][ju], spar_across[k]});
			}
			plan.ribs.push_back(std::move(rib));
		}
		// The spar webs and the stringers. On a symmetric wing an element
		// whose first station is in the symmetry plane is ONE strip from tip
		// to tip, its port stations the reflections of its starboard ones
		// (the same lerp arithmetic across, so it still shares the port
		// ribs' chord nodes); one that starts outboard is two strips, its
		// own and "<name> (port)".
		auto web = [&](const std::string& name, double thickness, const Material& material, std::vector<Vec3> a, std::vector<Vec3> b, int across, std::vector<WingShellPlan>& into) {
			WingShellPlan sh;
			sh.name = name;
			sh.thickness = thickness;
			sh.material = material;
			if (symmetric && !a.empty() && a.front().y == 0.0 && b.front().y == 0.0) {
				std::vector<Vec3> fa, fb;
				for (size_t i = a.size(); i-- > 1;) { fa.push_back(reflected(a[i])); fb.push_back(reflected(b[i])); }
				fa.insert(fa.end(), a.begin(), a.end());
				fb.insert(fb.end(), b.begin(), b.end());
				sh.mesh = StripMesh(fa, fb, across);
				into.push_back(std::move(sh));
				return;
			}
			sh.mesh = StripMesh(a, b, across);
			into.push_back(sh);
			if (symmetric) {
				sh.name = name + " (port)";
				sh.mesh.Mirror(Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0});
				into.push_back(std::move(sh));
			}
		};
		for (size_t k = 0; k < spars.size(); ++k) {
			if (spar_feet[k].size() < 2) { plan.notes.push_back("spar '" + spars[k].name + "': fewer than two stations in its span"); continue; }
			std::vector<Vec3> lower, upper;
			for (const Foot& f : spar_feet[k]) { lower.push_back(f.lower); upper.push_back(f.upper); }
			web(spars[k].name, spars[k].thickness, spars[k].material, std::move(lower), std::move(upper), spar_across[k], plan.spars);
		}
		for (size_t k = 0; k < stringers.size(); ++k) {
			if (stringer_feet[k].size() < 2) { plan.notes.push_back("stringer '" + stringers[k].name + "': fewer than two stations in its span"); continue; }
			std::vector<Vec3> feet, inner;
			for (const auto& f : stringer_feet[k]) feet.push_back(f.second);
			for (const auto& f : stringer_in[k]) inner.push_back(f.second);
			web(stringers[k].name, stringers[k].thickness, stringers[k].material, std::move(feet), std::move(inner), 2, plan.stringers);
		}
		if (symmetric) {
			// The port half: the starboard mesh reflected in y = 0, the ring
			// in the plane shared, the winding reversed to stay outward; the
			// ribs reflected with it (their rims and chords on the reflected
			// vertices), the centre rib shared.
			TriMesh& m = out.back().mesh;
			const int n0 = static_cast<int>(m.vertices.size());
			std::vector<int> port(n0);
			std::vector<Vec3> added;
			for (int i = 0; i < n0; ++i) {
				if (m.vertices[i].y == 0.0) { port[i] = i; continue; }
				port[i] = n0 + static_cast<int>(added.size());
				added.push_back(reflected(m.vertices[i]));
			}
			m.vertices.insert(m.vertices.end(), added.begin(), added.end());
			const size_t nt = m.triangles.size();
			for (size_t t = 0; t < nt; ++t) {
				const std::array<int, 3> tr = m.triangles[t];
				if (port[tr[0]] == tr[0] && port[tr[1]] == tr[1] && port[tr[2]] == tr[2]) continue;
				m.triangles.push_back({port[tr[0]], port[tr[2]], port[tr[1]]});
			}
			const size_t n_ribs = plan.ribs.size();
			for (size_t r = 0; r < n_ribs; ++r) {
				if (plan.ribs[r].eta <= eps) continue;
				WingRibPlan rib = plan.ribs[r];
				rib.eta = -rib.eta;
				rib.name += " (port)";
				for (int& idx : rib.rim) idx = port[idx];
				for (auto& ch : rib.chords) { ch[0] = port[ch[0]]; ch[1] = port[ch[1]]; }
				plan.ribs.push_back(std::move(rib));
			}
		}
		plans.push_back(std::move(plan));
	}

	// ---- the flaps ----
	// The skin aft of the tangency points and the nose arc
	// (SectionLoop::Flap), capped at both ends; ribs at the rib stations
	// inside each.
	for (int fi = 0; fi < n_flaps; ++fi) {
		const FlapSpec& f = sorted[fi];
		LoftSource src;
		src.u_min = f.eta_root;
		src.u_max = f.eta_tip;
		src.u_seeds = seeds_in(f.eta_root, f.eta_tip);
		src.closed_curve = false;
		src.v_seeds = {0.35, 0.5, 0.65};
		src.uniform_rings = true;
		src.section = [&, fi](double s, const std::vector<double>& v) { return planform_->AirfoilCoordsToXyz(s, cut_at(fi, s).flap_loop->Points(v)); };
		src.mirror_v = [&, fi](double s, double v) { return cut_at(fi, s).flap_loop->Mirror(v); };
		LoftRings fr;
		out.push_back({f.name, TriangulateLoft(src, opt, nullptr, nullptr, &fr)});
		WingBodyStructurePlan plan;
		plan.body = f.name;
		plan.rib_thickness = structure.ribs.thickness;
		plan.rib_material = structure.ribs.material;
		plan.rib_size = MedianEdge(out.back().mesh);
		if (!structure_problem.empty()) plan.notes.push_back(structure_problem);
		int rib_no = 0;
		for (double u : rib_stations) {
			if (u <= f.eta_root + eps || u >= f.eta_tip - eps) continue;
			const int ri = fr.Find(u);
			if (ri < 0) { plan.notes.push_back("rib at eta " + std::to_string(u) + ": the flap's loft has no ring there"); continue; }
			WingRibPlan rib;
			rib.eta = u;
			rib.name = RibName(++rib_no);
			for (int idx : fr.rings[ri]) if (rib.rim.empty() || rib.rim.back() != idx) rib.rim.push_back(idx);
			while (rib.rim.size() > 1 && rib.rim.back() == rib.rim.front()) rib.rim.pop_back();
			if (rib.rim.size() < 3) continue;
			plan.ribs.push_back(std::move(rib));
		}
		plans.push_back(plan);
		if (symmetric) {
			// The port flap: a body of its own, the reflection of this one
			// (the same vertex numbering, so the plan's rims hold).
			NamedMesh port{f.name + " (port)", out.back().mesh};
			port.mesh.Mirror(Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0});
			out.push_back(std::move(port));
			plan.body = f.name + " (port)";
			for (WingRibPlan& rib : plan.ribs) { rib.eta = -rib.eta; rib.name += " (port)"; }
			plans.push_back(std::move(plan));
		}
	}
	return out;
}

// WingModel::ExportStep() is defined in step_export.cpp, deliberately its
// own translation unit -- see that file's header comment for why.

} // namespace aerotelic
