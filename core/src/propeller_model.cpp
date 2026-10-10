#include "aerotelic/propeller_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <BRep_Builder.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS_Compound.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

namespace aerotelic {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kPitchAxis = 0.35;

double DegToRad(double d) { return d * kPi / 180.0; }

std::vector<double> Linspace(double a, double b, int n) {
	std::vector<double> out(n);
	for (int i = 0; i < n; ++i) out[i] = a + (b - a) * i / (n - 1);
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

double Trapezoid(const std::vector<double>& y, const std::vector<double>& x) {
	double total = 0.0;
	for (size_t i = 0; i + 1 < x.size(); ++i) total += 0.5 * (y[i] + y[i + 1]) * (x[i + 1] - x[i]);
	return total;
}

} // namespace

PropellerSpec::PropellerSpec() {
	spinner_curve.degree = 3;
	spinner_curve.poles = Mat::FromRows({{0.00, 0.000}, {0.00, 0.015}, {0.20, 0.100}, {0.50, 0.180}, {0.88, 0.180}, {1.00, 0.165}, {1.00, 0.000}});
}

std::unique_ptr<WingModel> MakeDefaultBlade(double radius) {
	// Chord over R against r/R, cubic: narrow shank, widest mid-blade, a
	// finite tip chord (a zero-chord section does not loft).
	static const std::vector<std::pair<double, double>> kChordPoles = {
		{0.00, 0.085}, {0.15, 0.095}, {0.40, 0.160}, {0.75, 0.145}, {0.95, 0.100}, {1.00, 0.040}};
	// Blade angle beta in degrees against r/R: close to a constant-pitch
	// helix, flattened inboard where the shank is buried in the spinner.
	static const std::vector<std::pair<double, double>> kTwistPoles = {
		{0.00, 60.0}, {0.15, 58.0}, {0.30, 33.0}, {0.55, 21.0}, {0.80, 15.5}, {1.00, 13.5}};

	std::vector<std::vector<double>> le_pts, te_pts, twist_pts;
	for (auto& [eta, c] : kChordPoles) {
		le_pts.push_back({eta, -kPitchAxis * c * radius});
		te_pts.push_back({eta, (1.0 - kPitchAxis) * c * radius});
	}
	for (auto& [eta, v] : kTwistPoles) twist_pts.push_back({eta, v});

	PlanformSpec pspec;
	pspec.type = "nurbs";
	pspec.semi_span = radius;
	pspec.rotation_fraction = kPitchAxis;
	NurbsSpec le;
	le.degree = 3;
	le.poles = Mat::FromRows(le_pts);
	NurbsSpec te;
	te.degree = 3;
	te.poles = Mat::FromRows(te_pts);
	pspec.le_curve = le;
	pspec.te_curve = te;
	pspec.dihedral_curve = LinearCurveSpec(0.0, 0.0);
	NurbsSpec twist;
	twist.degree = 3;
	twist.poles = Mat::FromRows(twist_pts);
	pspec.twist_curve = twist;

	auto wing = std::make_unique<WingModel>(pspec, "Blade");
	wing->AddSection(Airfoil::Naca4("4421"), 0.0);
	wing->AddSection(Airfoil::Naca4("4412"), 0.45);
	wing->AddSection(Airfoil::Naca4("4407"), 1.0);
	return wing;
}

PropellerModel::PropellerModel(const PropellerSpec& spec) {
	name = spec.name;
	radius = spec.radius;
	if (!(radius > 0.0)) throw std::invalid_argument("the propeller radius must be positive");
	blade_count = spec.blade_count;
	blade_x = spec.blade_x;
	hub_radius = spec.hub_radius;
	spinner_length = spec.spinner_length;
	reference_images = spec.reference_images;

	// Held on the axis at both ends, and at the two ends of the length.
	NurbsSpec spinner_spec = spec.spinner_curve;
	int n = spinner_spec.poles.rows();
	spinner_spec.poles(0, 0) = 0.0;
	spinner_spec.poles(0, 1) = 0.0;
	spinner_spec.poles(n - 1, 0) = 1.0;
	spinner_spec.poles(n - 1, 1) = 0.0;
	spinner_curve_ = NurbsCurve::Create(spinner_spec);
	if (spinner_curve_->periodic()) throw std::invalid_argument("the spinner profile must be an open (clamped) curve");

	blade_ = spec.blade ? std::make_unique<WingModel>(*spec.blade) : MakeDefaultBlade(radius);
	// The blade's semi-span is the radius by definition; bring it into
	// line if it disagrees (its edges keep their proportions). Applies
	// whether the blade came from a custom spec or the default -- Python
	// runs this same check unconditionally after either path.
	if (std::abs(blade_->planform().SemiSpan() - radius) > 1e-12 * radius) {
		blade_->planform().SetSemiSpan(radius);
	}
}

PropellerSpec PropellerModel::Serialize() const {
	PropellerSpec s;
	s.name = name;
	s.radius = radius;
	s.blade_count = blade_count;
	s.blade_x = blade_x;
	s.hub_radius = hub_radius;
	s.spinner_length = spinner_length;
	s.spinner_curve = spinner_curve_->Serialize();
	s.blade = blade_->Serialize();
	s.reference_images = reference_images;
	return s;
}

void PropellerModel::SetRadius(double new_radius) {
	if (!(new_radius > 0.0)) throw std::invalid_argument("the propeller radius must be positive");
	radius = new_radius;
	blade_->planform().SetSemiSpan(new_radius);
}

double PropellerModel::SpinnerRadius() const {
	Mat pts = spinner_curve_->Sample(400);
	double max_r = pts(0, 1);
	for (int i = 1; i < pts.rows(); ++i) max_r = std::max(max_r, pts(i, 1));
	return max_r;
}

double PropellerModel::SpinnerRadiusAt(double x_over_r) const {
	double l = spinner_length;
	if (!(l > 0.0)) return 0.0;
	Mat pts = spinner_curve_->Sample(800);
	std::vector<double> xs(pts.rows()), ys(pts.rows());
	double running_max = -std::numeric_limits<double>::infinity();
	for (int i = 0; i < pts.rows(); ++i) {
		running_max = std::max(running_max, pts(i, 0) * l);
		xs[i] = running_max;
		ys[i] = pts(i, 1);
	}
	if (!(xs.front() < x_over_r && x_over_r < xs.back())) return 0.0;
	return LinInterp(x_over_r, xs, ys);
}

std::vector<double> PropellerModel::ChordOverR(const std::vector<double>& eta) const {
	auto* p = dynamic_cast<NURBSPlanform*>(&blade_->planform());
	std::vector<double> out(eta.size());
	for (size_t i = 0; i < eta.size(); ++i) out[i] = p->Chord(eta[i]) / radius;
	return out;
}

double PropellerModel::BladeAngle(double eta) const {
	auto* p = dynamic_cast<NURBSPlanform*>(&blade_->planform());
	return NURBSPlanform::ValueAtEta(p->TwistCurveRef(), eta);
}

PropellerMetrics PropellerModel::Metrics() const {
	std::vector<double> x = Linspace(hub_radius, 1.0, 241);
	std::vector<double> c = ChordOverR(x);
	double solidity = blade_count * Trapezoid(c, x) / kPi;

	std::vector<double> xa = Linspace(0.15, 1.0, 241);
	std::vector<double> ca = ChordOverR(xa);
	std::vector<double> integrand(xa.size());
	for (size_t i = 0; i < xa.size(); ++i) integrand[i] = 0.5 * ca[i] * std::pow(xa[i], 3);
	double af = 1e5 / 16.0 * Trapezoid(integrand, xa);

	double beta = BladeAngle(0.75);
	PropellerMetrics m;
	m.diameter = Diameter();
	m.blade_count = blade_count;
	m.solidity = solidity;
	m.activity_factor = af;
	m.beta_75 = beta;
	m.pitch_ratio = kPi * 0.75 * std::tan(DegToRad(beta));
	m.spinner_diameter = 2.0 * SpinnerRadius() * radius;
	m.spinner_length = spinner_length * radius;
	return m;
}

void PropellerModel::Validate() const {
	if (!(blade_count >= 1 && blade_count <= kMaxBlades)) {
		throw std::invalid_argument("a propeller needs 1 to " + std::to_string(kMaxBlades) + " blades");
	}
	if (!(hub_radius > 0.0 && hub_radius < 1.0)) {
		throw std::invalid_argument("the blade root (hub) radius must lie between the axis and the tip");
	}
	if (!(spinner_length > 0.0)) throw std::invalid_argument("the spinner needs a length greater than zero");

	Mat pts = spinner_curve_->Sample(400);
	double min_r = std::numeric_limits<double>::infinity();
	for (int i = 1; i < pts.rows() - 1; ++i) min_r = std::min(min_r, pts(i, 1));
	if (!(min_r > 0.0)) throw std::invalid_argument("the spinner profile touches or crosses the axis between its ends");

	if (hub_radius >= SpinnerRadius()) {
		throw std::invalid_argument("the blade root (hub) radius is outside the spinner -- the blade root would be left bare");
	}
}

TopoDS_Shape PropellerModel::SpinnerSolid() const {
	double R = radius, l = spinner_length * radius;
	const NurbsCurve& c = *spinner_curve_;
	Mat poles = c.poles();
	std::vector<double> weights = c.weights();
	int n = poles.rows();
	TColgp_Array1OfPnt arr(1, n);
	TColStd_Array1OfReal w(1, n);
	for (int i = 0; i < n; ++i) {
		arr.SetValue(i + 1, gp_Pnt(poles(i, 0) * l, poles(i, 1) * R, 0.0));
		w.SetValue(i + 1, weights[i]);
	}
	auto [vals, mults] = c.GetUniqueKnots();
	int m = static_cast<int>(vals.size());
	TColStd_Array1OfReal ka(1, m);
	TColStd_Array1OfInteger ma(1, m);
	for (int j = 0; j < m; ++j) {
		ka.SetValue(j + 1, vals[j]);
		ma.SetValue(j + 1, mults[j]);
	}
	Handle(Geom_BSplineCurve) curve = new Geom_BSplineCurve(arr, w, ka, ma, c.degree());

	BRepBuilderAPI_MakeWire wire(BRepBuilderAPI_MakeEdge(curve).Edge(), BRepBuilderAPI_MakeEdge(gp_Pnt(l, 0.0, 0.0), gp_Pnt(0.0, 0.0, 0.0)).Edge());
	if (!wire.IsDone()) throw std::runtime_error("could not close the spinner profile along the axis");
	BRepBuilderAPI_MakeFace face(wire.Wire(), true);
	if (!face.IsDone()) throw std::runtime_error("could not make a face from the spinner profile");
	BRepPrimAPI_MakeRevol revol(face.Face(), gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), 2.0 * kPi);
	revol.Build();
	if (!revol.IsDone()) throw std::runtime_error("revolving the spinner profile failed");
	return revol.Shape();
}

TopoDS_Shape PropellerModel::BladeSolid(int n_span, std::optional<double> station_tol) const {
	double R = radius;
	TopoDS_Shape wing = blade_->BuildSolid(n_span, true, station_tol);

	double big = 4.0 * R;
	TopoDS_Shape box = BRepPrimAPI_MakeBox(gp_Pnt(-big, hub_radius * R, -big), gp_Pnt(big, 2.0 * R, big)).Shape();
	BRepAlgoAPI_Common common(wing, box);
	if (!common.IsDone()) throw std::runtime_error("trimming the blade at the hub radius failed");
	TopoDS_Shape blade_shape = common.Shape();

	gp_Trsf turn;
	turn.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)), -0.5 * kPi);
	return BRepBuilderAPI_Transform(blade_shape, turn, true).Shape();
}

TopoDS_Shape PropellerModel::BuildSolid(int n_span, std::optional<double> station_tol, bool fuse) const {
	Validate();
	double R = radius;
	TopoDS_Shape spinner = SpinnerSolid();
	TopoDS_Shape blade_shape = BladeSolid(n_span, station_tol);

	int n = blade_count;
	std::vector<TopoDS_Shape> blades;
	for (int k = 0; k < n; ++k) {
		gp_Trsf t;
		t.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), 2.0 * kPi * k / n);
		gp_Trsf shift;
		shift.SetTranslation(gp_Vec(blade_x * R, 0.0, 0.0));
		blades.push_back(BRepBuilderAPI_Transform(blade_shape, shift.Multiplied(t), true).Shape());
	}

	if (!fuse) {
		BRep_Builder builder;
		TopoDS_Compound compound;
		builder.MakeCompound(compound);
		builder.Add(compound, spinner);
		for (auto& b : blades) builder.Add(compound, b);
		return compound;
	}

	TopTools_ListOfShape args, tools;
	args.Append(spinner);
	for (auto& b : blades) tools.Append(b);
	BRepAlgoAPI_Fuse fuse_op;
	fuse_op.SetArguments(args);
	fuse_op.SetTools(tools);
	fuse_op.SetRunParallel(true);
	fuse_op.Build();
	if (!fuse_op.IsDone()) throw std::runtime_error("fusing the blades to the spinner failed");
	TopoDS_Shape shape = fuse_op.Shape();

	std::vector<TopoDS_Shape> solids;
	TopExp_Explorer exp(shape, TopAbs_SOLID);
	while (exp.More()) {
		solids.push_back(exp.Current());
		exp.Next();
	}
	if (solids.size() == 1) shape = solids[0];
	return shape;
}

} // namespace aerotelic
