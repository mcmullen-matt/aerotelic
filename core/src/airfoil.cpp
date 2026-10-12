#include "aerotelic/airfoil.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace aerotelic {

namespace {

constexpr double kPi = 3.14159265358979323846;

double Interp1(double x, const std::vector<double>& xs, const std::vector<double>& ys) {
	// np.interp semantics: clamps outside [xs.front(), xs.back()].
	if (x <= xs.front()) return ys.front();
	if (x >= xs.back()) return ys.back();
	auto it = std::upper_bound(xs.begin(), xs.end(), x);
	size_t i = static_cast<size_t>(it - xs.begin()) - 1;
	double t = (x - xs[i]) / (xs[i + 1] - xs[i]);
	return ys[i] + t * (ys[i + 1] - ys[i]);
}

std::vector<double> InterpCol(const std::vector<double>& xq, const Mat& xy, bool reverse) {
	int n = xy.rows();
	std::vector<double> xs(n), ys(n);
	for (int i = 0; i < n; ++i) {
		int src = reverse ? (n - 1 - i) : i;
		xs[i] = xy(src, 0);
		ys[i] = xy(src, 1);
	}
	std::vector<double> out(xq.size());
	for (size_t k = 0; k < xq.size(); ++k) out[k] = Interp1(xq[k], xs, ys);
	return out;
}

int ArgMinCol0(const Mat& m) {
	int best = 0;
	for (int i = 1; i < m.rows(); ++i) if (m(i, 0) < m(best, 0)) best = i;
	return best;
}

Mat ReverseRows(const Mat& m) {
	Mat out(m.rows(), m.cols());
	for (int i = 0; i < m.rows(); ++i) out.set_row(i, m.row(m.rows() - 1 - i));
	return out;
}

double Median(std::vector<double> v) {
	std::sort(v.begin(), v.end());
	size_t n = v.size();
	return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

// See airfoil.py's _wound_backwards docstring for why this is a median
// test rather than "any station negative".
bool WoundBackwards(const std::vector<double>& thickness) {
	return Median(thickness) < 0.0;
}

std::vector<double> Linspace(double a, double b, int n) {
	std::vector<double> out(n);
	for (int i = 0; i < n; ++i) out[i] = a + (b - a) * i / (n - 1);
	return out;
}

} // namespace

bool IsNaca4Code(const std::string& code_in) {
	// Python's str.strip(): leading/trailing whitespace only.
	std::string code = code_in;
	size_t b = code.find_first_not_of(" \t\n\r");
	size_t e = code.find_last_not_of(" \t\n\r");
	code = (b == std::string::npos) ? "" : code.substr(b, e - b + 1);

	if (code.size() != 4 || !std::all_of(code.begin(), code.end(), [](char c) { return std::isdigit(static_cast<unsigned char>(c)); })) {
		return false;
	}
	int m = code[0] - '0', p = code[1] - '0', t = std::stoi(code.substr(2));
	return t > 0 && ((m == 0) == (p == 0));
}

std::unique_ptr<Airfoil> Airfoil::Naca4(const std::string& code, int n) {
	if (!IsNaca4Code(code)) throw std::invalid_argument("not a NACA four-digit section: " + code);
	double m = (code[0] - '0') / 100.0, p = (code[1] - '0') / 10.0, t = std::stoi(code.substr(2)) / 100.0;

	std::vector<double> x(n), yt(n), yc(n);
	for (int i = 0; i < n; ++i) {
		double u = kPi * i / (n - 1);
		x[i] = 0.5 * (1.0 - std::cos(u));
	}
	for (int i = 0; i < n; ++i) {
		double xi = x[i];
		yt[i] = 5.0 * t * (0.2969 * std::sqrt(xi) - 0.1260 * xi - 0.3516 * xi * xi
			+ 0.2843 * xi * xi * xi - 0.1036 * xi * xi * xi * xi);
	}
	yt[n - 1] = 0.0;
	for (int i = 0; i < n; ++i) {
		double xi = x[i];
		if (m > 0.0 && p > 0.0) {
			yc[i] = (xi < p) ? m / (p * p) * (2.0 * p * xi - xi * xi)
				: m / ((1.0 - p) * (1.0 - p)) * ((1.0 - 2.0 * p) + 2.0 * p * xi - xi * xi);
		} else {
			yc[i] = 0.0;
		}
	}

	// lower = column_stack([x, yc-yt])[::-1]; upper = column_stack([x, yc+yt])[1:]
	Mat pts(2 * n - 1, 2);
	for (int i = 0; i < n; ++i) {
		int row = n - 1 - i; // lower, reversed
		pts(row, 0) = x[i];
		pts(row, 1) = yc[i] - yt[i];
	}
	for (int i = 1; i < n; ++i) {
		int row = n - 1 + i; // upper, skipping index 0
		pts(row, 0) = x[i];
		pts(row, 1) = yc[i] + yt[i];
	}
	auto af = FromPoints(pts, 3, "NACA " + code);
	return af;
}

std::unique_ptr<Airfoil> Airfoil::FromPoints(const Mat& pts, int degree, std::string name) {
	auto af = std::unique_ptr<Airfoil>(new Airfoil());
	af->name = std::move(name);
	af->pts_arr_ = pts;

	auto fit = std::make_unique<FitCurve>(af->pts_arr_, degree);
	int le_idx = ArgMinCol0(af->pts_arr_);
	af->t_le_ = fit->collocation_parameters()[le_idx];
	af->curve_ = std::move(fit);

	if (WoundBackwards(af->Thickness())) {
		af->pts_arr_ = ReverseRows(af->pts_arr_);
		auto fit2 = std::make_unique<FitCurve>(af->pts_arr_, degree);
		le_idx = ArgMinCol0(af->pts_arr_);
		af->t_le_ = fit2->collocation_parameters()[le_idx];
		af->curve_ = std::move(fit2);
		if (WoundBackwards(af->Thickness())) {
			throw std::invalid_argument("Negative thickness detected on this airfoil");
		}
	}
	af->BuildTeLine();
	return af;
}

std::unique_ptr<Airfoil> Airfoil::FromCurve(const NurbsSpec& curve_spec, std::string name) {
	auto af = std::unique_ptr<Airfoil>(new Airfoil());
	af->name = std::move(name);
	af->curve_ = NurbsCurve::Create(curve_spec);
	const int n_sample = 400;
	af->pts_arr_ = af->curve_->Sample(n_sample);
	int le_idx = ArgMinCol0(af->pts_arr_);
	af->t_le_ = static_cast<double>(le_idx) / (n_sample - 1);
	af->BuildTeLine();
	return af;
}

void Airfoil::BuildTeLine() {
	int n = pts_arr_.rows();
	if (VecNorm(VecSub(pts_arr_.row(0), pts_arr_.row(n - 1))) < 1e-6) {
		te_line_ = nullptr;
		return;
	}
	NurbsSpec spec;
	spec.degree = 1;
	spec.poles = Mat::FromRows({pts_arr_.row(0), pts_arr_.row(n - 1)});
	te_line_ = NurbsCurve::Create(spec);
}

NurbsSpec Airfoil::Serialize() const { return curve_->Serialize(); }

std::vector<double> Airfoil::UpperXY(double t) const {
	double t_scaled = (1.0 - t_le_) * std::clamp(t, 0.0, 1.0) + t_le_;
	return curve_->Evaluate(t_scaled);
}

Mat Airfoil::UpperXY(const std::vector<double>& t) const {
	std::vector<double> ts(t.size());
	for (size_t i = 0; i < t.size(); ++i) ts[i] = (1.0 - t_le_) * std::clamp(t[i], 0.0, 1.0) + t_le_;
	return curve_->Evaluate(ts);
}

std::vector<double> Airfoil::LowerXY(double t) const {
	double t_scaled = t_le_ - t_le_ * std::clamp(t, 0.0, 1.0);
	return curve_->Evaluate(t_scaled);
}

Mat Airfoil::LowerXY(const std::vector<double>& t) const {
	std::vector<double> ts(t.size());
	for (size_t i = 0; i < t.size(); ++i) ts[i] = t_le_ - t_le_ * std::clamp(t[i], 0.0, 1.0);
	return curve_->Evaluate(ts);
}

double Airfoil::MirrorParameter(double t) const {
	t = std::clamp(t, 0.0, 1.0);
	const double x = curve_->Evaluate(t)[0];
	// The other surface's parameter range, oriented so x increases with s.
	// Lower surface: t in [0, t_le], x falls from the TE to the LE. Upper:
	// t in [t_le, 1], x rises from the LE to the TE.
	const bool on_upper = t >= t_le_;
	double lo = on_upper ? 0.0 : t_le_, hi = on_upper ? t_le_ : 1.0;
	double x_lo = curve_->Evaluate(lo)[0], x_hi = curve_->Evaluate(hi)[0];
	if (x <= std::min(x_lo, x_hi)) return t_le_;
	if (x >= std::max(x_lo, x_hi)) return on_upper ? 0.0 : 1.0;
	// Bisection: f(s) = x(s) - x changes sign over [lo, hi].
	bool increasing = x_hi > x_lo;
	for (int it = 0; it < 60; ++it) {
		double mid = 0.5 * (lo + hi);
		double xm = curve_->Evaluate(mid)[0];
		if ((xm < x) == increasing) lo = mid; else hi = mid;
		if (hi - lo < 1e-13) break;
	}
	return 0.5 * (lo + hi);
}

std::pair<Mat, Mat> Airfoil::UniformSampleSurfaces(int n) const {
	std::vector<double> t = Linspace(0.0, 1.0, n);
	return {UpperXY(t), LowerXY(t)};
}

std::pair<Mat, Mat> Airfoil::CosineSampleSurfaces(int n) const {
	std::vector<double> t(n);
	for (int i = 0; i < n; ++i) t[i] = 0.5 * (std::cos(kPi * i / (n - 1)) + 1.0);
	return {UpperXY(t), LowerXY(t)};
}

Mat Airfoil::ContourPoints(int n) const {
	auto [upper, lower] = CosineSampleSurfaces(n / 2);
	Mat rev_upper = ReverseRows(upper);
	int n_lower = lower.rows(), n_rev = rev_upper.rows() - 1;
	Mat out(n_lower + n_rev, 2);
	for (int i = 0; i < n_lower; ++i) out.set_row(i, lower.row(i));
	for (int i = 0; i < n_rev; ++i) out.set_row(n_lower + i, rev_upper.row(i + 1));
	return out;
}

std::vector<double> Airfoil::Camber(const std::vector<double>& x) const {
	auto [upper, lower] = CosineSampleSurfaces(200);
	std::vector<double> up = InterpCol(x, upper, true);
	std::vector<double> lo = InterpCol(x, lower, true);
	std::vector<double> out(x.size());
	for (size_t i = 0; i < x.size(); ++i) out[i] = 0.5 * (up[i] + lo[i]);
	return out;
}

std::vector<double> Airfoil::Thickness(const std::vector<double>& x) const {
	auto [upper, lower] = CosineSampleSurfaces(200);
	std::vector<double> up = InterpCol(x, upper, true);
	std::vector<double> lo = InterpCol(x, lower, true);
	std::vector<double> out(x.size());
	for (size_t i = 0; i < x.size(); ++i) out[i] = up[i] - lo[i];
	return out;
}

std::vector<double> Airfoil::Thickness() const { return Thickness(Linspace(0.01, 0.99, 100)); }

std::pair<double, double> Airfoil::MaxThickness() const {
	auto [upper, lower] = UniformSampleSurfaces(200);
	std::vector<double> x = Linspace(0.0, 1.0, 200);
	std::vector<double> up = InterpCol(x, upper, false);
	std::vector<double> lo = InterpCol(x, lower, false);
	int best = 0;
	double best_v = up[0] - lo[0];
	for (size_t i = 1; i < x.size(); ++i) {
		double v = up[i] - lo[i];
		if (v > best_v) { best_v = v; best = static_cast<int>(i); }
	}
	return {x[best], best_v};
}

std::pair<double, double> Airfoil::MaxCamber() const {
	auto [upper, lower] = UniformSampleSurfaces(200);
	std::vector<double> x = Linspace(0.0, 1.0, 200);
	std::vector<double> up = InterpCol(x, upper, false);
	std::vector<double> lo = InterpCol(x, lower, false);
	int best = 0;
	double best_v = 0.5 * (up[0] + lo[0]);
	for (size_t i = 1; i < x.size(); ++i) {
		double v = 0.5 * (up[i] + lo[i]);
		if (v > best_v) { best_v = v; best = static_cast<int>(i); }
	}
	return {x[best], best_v};
}

} // namespace aerotelic
