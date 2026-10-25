#include "aerotelic/nurbs.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <limits>

namespace aerotelic {

namespace {

int FloorMod(int a, int n) {
	int r = a % n;
	return r < 0 ? r + n : r;
}

double FloorMod1(double u) {
	double r = std::fmod(u, 1.0);
	return r < 0.0 ? r + 1.0 : r;
}

// Groups `sorted` values that compare exactly equal (matches np.unique,
// which the Python module relies on for knot-multiplicity bookkeeping).
std::pair<std::vector<double>, std::vector<int>> UniqueWithCounts(std::vector<double> values) {
	std::sort(values.begin(), values.end());
	std::vector<double> uniq;
	std::vector<int> counts;
	for (double v : values) {
		if (!uniq.empty() && uniq.back() == v) {
			counts.back()++;
		} else {
			uniq.push_back(v);
			counts.push_back(1);
		}
	}
	return {uniq, counts};
}

double LinInterp(double x, const std::vector<double>& xs, const std::vector<double>& ys) {
	// np.interp semantics: clamps outside [xs.front(), xs.back()].
	if (x <= xs.front()) return ys.front();
	if (x >= xs.back()) return ys.back();
	auto it = std::upper_bound(xs.begin(), xs.end(), x);
	size_t i = static_cast<size_t>(it - xs.begin()) - 1;
	double t = (x - xs[i]) / (xs[i + 1] - xs[i]);
	return ys[i] + t * (ys[i + 1] - ys[i]);
}

} // namespace

// ===========================================================================
// NurbsCurve
// ===========================================================================

std::unique_ptr<NurbsCurve> NurbsCurve::Create(const NurbsSpec& spec) {
	if (spec.periodic) return std::make_unique<PeriodicNurbsCurve>(spec);
	return std::make_unique<ClampedNurbsCurve>(spec);
}

void NurbsCurve::InitShared(const NurbsSpec& spec) {
	if (spec.poles.rows() < 2) {
		throw std::invalid_argument("poles must be a 2-D array with at least two rows");
	}
	poles_ = spec.poles;
	int n = poles_.rows();
	degree_ = std::clamp(spec.degree, 1, max_degree());
	if (spec.weights) {
		if (static_cast<int>(spec.weights->size()) != n) {
			throw std::invalid_argument("weights must have one entry per pole");
		}
		weights_ = *spec.weights;
	} else {
		weights_.assign(n, 1.0);
	}
}

NurbsSpec NurbsCurve::Serialize() const {
	NurbsSpec out;
	out.degree = degree_;
	out.poles = poles_;
	out.weights = weights_;
	SerializeKnots(out);
	return out;
}

void NurbsCurve::UpdatePole(int index, const std::vector<double>& pole) {
	if (!(0 <= index && index < n_poles())) throw std::invalid_argument("The index is invalid");
	poles_.set_row(index, pole);
}

void NurbsCurve::SetPoleWeight(int index, double weight) {
	if (!(0 <= index && index < n_poles())) throw std::invalid_argument("The index is invalid");
	weights_[index] = weight;
}

void NurbsCurve::SetPolesAndWeights(const Mat& poles, const std::vector<double>& weights) {
	if (poles.rows() == 0 || static_cast<int>(weights.size()) != poles.rows()) {
		throw std::invalid_argument("The set of poles is invalid");
	}
	bool count_changed = poles.rows() != poles_.rows();
	poles_ = poles;
	weights_ = weights;
	degree_ = std::min(degree_, max_degree());
	if (count_changed) ResetKnots();
}

void NurbsCurve::SetDegree(int degree) {
	degree = std::clamp(degree, 1, max_degree());
	if (degree_ != degree) {
		degree_ = degree;
		AfterDegreeChange();
	}
}

void NurbsCurve::CheckMultiplicity(const std::vector<double>& values, const std::string& where) const {
	if (values.empty()) return;
	auto [uniq, counts] = UniqueWithCounts(values);
	int max_count = *std::max_element(counts.begin(), counts.end());
	if (max_count > degree_) {
		throw std::invalid_argument(where + ": a knot repeats " + std::to_string(max_count) +
			" times, but multiplicity may not exceed the degree (" + std::to_string(degree_) + ")");
	}
}

Mat NurbsCurve::Homogeneous() const {
	int n = poles_.rows(), d = poles_.cols();
	Mat out(n, d + 1);
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < d; ++j) out(i, j) = poles_(i, j) * weights_[i];
		out(i, d) = weights_[i];
	}
	return out;
}

NurbsCurve::SpanBasis NurbsCurve::SpansAndBasis(double u, const std::vector<double>& U, int p) {
	int n = static_cast<int>(U.size()) - p - 2; // index of the last real span
	int raw = static_cast<int>(std::upper_bound(U.begin(), U.end(), u) - U.begin()) - 1;
	int span = std::clamp(raw, p, n);

	std::vector<double> N(p + 1, 0.0), left(p + 1, 0.0), right(p + 1, 0.0);
	N[0] = 1.0;
	for (int j = 1; j <= p; ++j) {
		left[j] = u - U[span + 1 - j];
		right[j] = U[span + j] - u;
		double saved = 0.0;
		for (int r = 0; r < j; ++r) {
			double temp = N[r] / (right[r + 1] + left[j - r]);
			N[r] = saved + right[r + 1] * temp;
			saved = left[j - r] * temp;
		}
		N[j] = saved;
	}
	return {span, N};
}

std::vector<double> NurbsCurve::EvalSpline(double u, const std::vector<double>& E, int d, const Mat& ctrl) const {
	auto [span, N] = SpansAndBasis(u, E, d);
	std::vector<int> ext(d + 1);
	for (int j = 0; j <= d; ++j) ext[j] = span - d + j;
	std::vector<int> idx = PoleIndices(ext);

	std::vector<double> out(ctrl.cols(), 0.0);
	for (int j = 0; j <= d; ++j) {
		for (int c = 0; c < ctrl.cols(); ++c) out[c] += N[j] * ctrl(idx[j], c);
	}
	return out;
}

Mat NurbsCurve::BasisMatrix(const std::vector<double>& s, bool rational) const {
	int p = degree_;
	std::vector<double> E = BuildKnotVector();
	Mat B(static_cast<int>(s.size()), n_poles(), 0.0);
	for (size_t k = 0; k < s.size(); ++k) {
		double u = Wrap(s[k]);
		auto [span, N] = SpansAndBasis(u, E, p);
		std::vector<int> ext(p + 1);
		for (int j = 0; j <= p; ++j) ext[j] = span - p + j;
		std::vector<int> cols = PoleIndices(ext);
		for (int j = 0; j <= p; ++j) B(static_cast<int>(k), cols[j]) += N[j];
	}
	if (rational) {
		for (int k = 0; k < B.rows(); ++k) {
			double sum = 0.0;
			for (int i = 0; i < B.cols(); ++i) { B(k, i) *= weights_[i]; sum += B(k, i); }
			for (int i = 0; i < B.cols(); ++i) B(k, i) /= sum;
		}
	}
	return B;
}

std::vector<double> NurbsCurve::Evaluate(double s) const {
	double u = Wrap(s);
	std::vector<double> Cw = EvalSpline(u, BuildKnotVector(), degree_, Homogeneous());
	double w = Cw.back();
	std::vector<double> C(Cw.size() - 1);
	for (size_t i = 0; i < C.size(); ++i) C[i] = Cw[i] / w;
	return C;
}

Mat NurbsCurve::Evaluate(const std::vector<double>& s) const {
	Mat out(static_cast<int>(s.size()), poles_.cols());
	for (size_t k = 0; k < s.size(); ++k) out.set_row(static_cast<int>(k), Evaluate(s[k]));
	return out;
}

std::vector<double> NurbsCurve::Derivative(double s) const {
	double u = Wrap(s);
	int p = degree_;
	std::vector<double> E = BuildKnotVector();
	Mat Pw = Homogeneous();

	std::vector<double> Cw = EvalSpline(u, E, p, Pw);
	double w = Cw.back();
	std::vector<double> C(Cw.size() - 1);
	for (size_t i = 0; i < C.size(); ++i) C[i] = Cw[i] / w;

	Mat Qw = Hodograph(Pw, E);
	std::vector<double> Einner(E.begin() + 1, E.end() - 1);
	std::vector<double> dCw = EvalSpline(u, Einner, p - 1, Qw);

	std::vector<double> dC(dCw.size() - 1);
	for (size_t i = 0; i < dC.size(); ++i) dC[i] = (dCw[i] - dCw.back() * C[i]) / w;
	return dC;
}

Mat NurbsCurve::Derivative(const std::vector<double>& s) const {
	Mat out(static_cast<int>(s.size()), poles_.cols());
	for (size_t k = 0; k < s.size(); ++k) out.set_row(static_cast<int>(k), Derivative(s[k]));
	return out;
}

Mat NurbsCurve::Sample(int n) const {
	std::vector<double> s(n);
	for (int i = 0; i < n; ++i) s[i] = static_cast<double>(i) / (n - 1);
	return Evaluate(s);
}

std::vector<double> NurbsCurve::ZAtEta(double eta) const {
	Mat f = Sample(200);
	std::vector<double> x(f.rows());
	for (int i = 0; i < f.rows(); ++i) x[i] = f(i, 0);
	std::vector<double> out(f.cols());
	for (int c = 0; c < f.cols(); ++c) {
		std::vector<double> y(f.rows());
		for (int i = 0; i < f.rows(); ++i) y[i] = f(i, c);
		out[c] = LinInterp(eta, x, y);
	}
	return out;
}

std::optional<double> NurbsCurve::Inverse(int index, double value, double initial_s, double tol) const {
	double s = initial_s;
	for (int i = 0; i < 30; ++i) {
		double f = Evaluate(s)[index] - value;
		if (std::abs(f) < tol) return s;
		double df = Derivative(s)[index];
		if (std::abs(df) < 1e-300) return std::nullopt;
		s -= f / df;
	}
	return std::nullopt;
}

double NurbsCurve::BoehmAlpha(double u_bar, const std::function<double(int)>& knot, int i, int p) {
	double a = knot(i), b = knot(i + p);
	return b > a ? (u_bar - a) / (b - a) : 0.0;
}

// ===========================================================================
// ClampedNurbsCurve
// ===========================================================================

ClampedNurbsCurve::ClampedNurbsCurve(const NurbsSpec& spec) {
	InitShared(spec);
	LoadKnots(spec);
}

void ClampedNurbsCurve::LoadKnots(const NurbsSpec& spec) {
	if (spec.interior_knots) {
		interior_knots_ = *spec.interior_knots;
		int expected = n_poles() - degree_ - 1;
		if (static_cast<int>(interior_knots_.size()) != expected) {
			throw std::invalid_argument("expected " + std::to_string(expected) + " interior knots for " +
				std::to_string(n_poles()) + " poles of degree " + std::to_string(degree_) + ", got " +
				std::to_string(interior_knots_.size()));
		}
		for (double k : interior_knots_) {
			if (k <= 0.0 || k >= 1.0) throw std::invalid_argument("interior knots must lie strictly inside (0, 1)");
		}
		std::sort(interior_knots_.begin(), interior_knots_.end());
		CheckMultiplicity(interior_knots_, "interior_knots");
	} else {
		ResetKnots();
	}
}

void ClampedNurbsCurve::SerializeKnots(NurbsSpec& out) const {
	out.interior_knots = interior_knots_;
}

void ClampedNurbsCurve::RebuildInteriorKnots() {
	int n_int = std::max(0, max_degree() - degree_);
	interior_knots_.resize(n_int);
	for (int i = 0; i < n_int; ++i) interior_knots_[i] = static_cast<double>(i + 1) / (n_int + 1);
}

void ClampedNurbsCurve::ResetKnots() { RebuildInteriorKnots(); }
void ClampedNurbsCurve::AfterDegreeChange() { ResetKnots(); }

std::vector<double> ClampedNurbsCurve::BuildKnotVector() const {
	int p = degree_;
	std::vector<double> U;
	U.reserve(2 * (p + 1) + interior_knots_.size());
	U.insert(U.end(), p + 1, 0.0);
	U.insert(U.end(), interior_knots_.begin(), interior_knots_.end());
	U.insert(U.end(), p + 1, 1.0);
	return U;
}

std::pair<std::vector<double>, std::vector<int>> ClampedNurbsCurve::GetUniqueKnots() const {
	return UniqueWithCounts(BuildKnotVector());
}

std::vector<int> ClampedNurbsCurve::PoleIndices(const std::vector<int>& ext) const { return ext; }
double ClampedNurbsCurve::Wrap(double u) const { return u; }

Mat ClampedNurbsCurve::Hodograph(const Mat& Pw, const std::vector<double>& U) const {
	int p = degree_, n = Pw.rows(), d = Pw.cols();
	Mat Qw(n - 1, d);
	for (int i = 0; i < n - 1; ++i) {
		double span_w = U[p + 1 + i] - U[1 + i];
		for (int c = 0; c < d; ++c) {
			double v = p * (Pw(i + 1, c) - Pw(i, c)) / span_w;
			Qw(i, c) = std::isfinite(v) ? v : 0.0;
		}
	}
	return Qw;
}

void ClampedNurbsCurve::ChangeKnot(int index, double knot) {
	if (!(0 <= index && index < static_cast<int>(interior_knots_.size()))) {
		throw std::invalid_argument("The index is invalid");
	}
	std::vector<double> proposed = interior_knots_;
	proposed[index] = std::clamp(knot, 1e-4, 1.0 - 1e-4);
	std::sort(proposed.begin(), proposed.end());
	CheckMultiplicity(proposed, "change_knot");
	interior_knots_ = proposed;
}

void ClampedNurbsCurve::InsertKnot(double knot_value) {
	double u_bar = knot_value;
	if (!(0.0 < u_bar && u_bar < 1.0)) throw std::invalid_argument("Inserted knot value must lie strictly inside (0, 1)");
	std::vector<double> U = BuildKnotVector();
	int p = degree_;
	int k = static_cast<int>(std::upper_bound(U.begin(), U.end(), u_bar) - U.begin()) - 1;
	// np.isclose's default tolerance (atol=1e-8, rtol=1e-5); the knot multiplicity
	// check must use the same tolerance np.isclose(U, u_bar) does in the Python version.
	int s = 0;
	for (double v : U) {
		double atol = 1e-8, rtol = 1e-5;
		if (std::abs(v - u_bar) <= atol + rtol * std::abs(u_bar)) s++;
	}
	if (s >= p) {
		throw std::invalid_argument("knot " + std::to_string(u_bar) + " already has multiplicity " +
			std::to_string(s) + "; it may not exceed the degree (" + std::to_string(p) + ")");
	}

	Mat Pw = Homogeneous();
	int n = Pw.rows(), d = Pw.cols();
	Mat Qw(n + 1, d);
	for (int i = 0; i <= n; ++i) {
		if (i <= k - p) {
			Qw.set_row(i, Pw.row(i));
		} else if (i <= k - s) {
			double a = BoehmAlpha(u_bar, [&](int j) { return U[j]; }, i, p);
			std::vector<double> row(d);
			for (int c = 0; c < d; ++c) row[c] = a * Pw(i, c) + (1.0 - a) * Pw(i - 1, c);
			Qw.set_row(i, row);
		} else {
			Qw.set_row(i, Pw.row(i - 1));
		}
	}

	weights_.resize(n + 1);
	Mat new_poles(n + 1, d - 1);
	for (int i = 0; i <= n; ++i) {
		weights_[i] = Qw(i, d - 1);
		for (int c = 0; c < d - 1; ++c) new_poles(i, c) = Qw(i, c) / weights_[i];
	}
	poles_ = new_poles;
	interior_knots_.insert(interior_knots_.begin() + (k - p), u_bar);
	std::sort(interior_knots_.begin(), interior_knots_.end());
}

void ClampedNurbsCurve::DeleteKnot(int index) {
	if (!(0 <= index && index < static_cast<int>(interior_knots_.size()))) {
		throw std::invalid_argument("The index is invalid");
	}
	interior_knots_.erase(interior_knots_.begin() + index);
	poles_ = poles_.RemoveRow(index + 1);
	weights_.erase(weights_.begin() + index + 1);
}

void ClampedNurbsCurve::AddPole(int index, const std::vector<double>& pole, double weight) {
	if (!(0 < index && index < n_poles()) || static_cast<int>(pole.size()) != poles_.cols()) {
		throw std::invalid_argument("The index is invalid");
	}
	poles_ = poles_.InsertRow(index, pole);
	weights_.insert(weights_.begin() + index, weight);
	ResetKnots();
}

void ClampedNurbsCurve::DeletePole(int index) {
	if (!(0 < index && index < n_poles() - 1)) throw std::invalid_argument("The index is invalid");
	poles_ = poles_.RemoveRow(index);
	weights_.erase(weights_.begin() + index);
	degree_ = std::min(degree_, max_degree());
	ResetKnots();
}

// ===========================================================================
// PeriodicNurbsCurve
// ===========================================================================

PeriodicNurbsCurve::PeriodicNurbsCurve(const NurbsSpec& spec) {
	InitShared(spec);
	LoadKnots(spec);
}

void PeriodicNurbsCurve::LoadKnots(const NurbsSpec& spec) {
	int n = n_poles();
	clamped_seam_ = spec.clamped_seam;
	if (spec.knots) {
		if (static_cast<int>(spec.knots->size()) != n) {
			throw std::invalid_argument("a periodic curve needs one knot per pole: expected " +
				std::to_string(n) + ", got " + std::to_string(spec.knots->size()));
		}
		knots_ = *spec.knots;
		std::sort(knots_.begin(), knots_.end());
		ValidateKnots(knots_, "knots");
	} else {
		ResetKnots();
	}
}

void PeriodicNurbsCurve::ValidateKnots(const std::vector<double>& k, const std::string& where) const {
	if (k[0] != 0.0) throw std::invalid_argument(where + ": the first knot is the seam and must be 0");
	for (double v : k) {
		if (v < 0.0 || v >= 1.0) throw std::invalid_argument(where + ": knots must lie in [0, 1)");
	}
	CheckMultiplicity(k, where);
	if (clamped_seam_) {
		int zeros = 0;
		for (double v : k) if (v == 0.0) zeros++;
		if (zeros != degree_) {
			throw std::invalid_argument(where + ": a clamped seam needs exactly " + std::to_string(degree_) +
				" knots at 0, got " + std::to_string(zeros));
		}
	}
}

void PeriodicNurbsCurve::SerializeKnots(NurbsSpec& out) const {
	out.periodic = true;
	if (clamped_seam_) out.clamped_seam = true;
	out.knots = knots_;
}

int PeriodicNurbsCurve::seam_multiplicity() const {
	int c = 0;
	for (double v : knots_) if (v == 0.0) c++;
	return c;
}

void PeriodicNurbsCurve::ResetKnots() {
	int n = n_poles();
	if (clamped_seam_) {
		int p = degree_;
		knots_.assign(p, 0.0);
		// linspace(0, 1, n - p + 2)[1:-1]
		int m = n - p + 2;
		for (int i = 1; i < m - 1; ++i) knots_.push_back(static_cast<double>(i) / (m - 1));
	} else {
		knots_.resize(n);
		for (int i = 0; i < n; ++i) knots_[i] = static_cast<double>(i) / n;
	}
}

void PeriodicNurbsCurve::AfterDegreeChange() {
	if (clamped_seam_) {
		ResetKnots();
	} else {
		CheckMultiplicity(knots_, "set_degree");
	}
}

double PeriodicNurbsCurve::Kappa(int i) const {
	int n = n_poles();
	int r = FloorMod(i, n);
	int q = (i - r) / n;
	return knots_[r] + q;
}

std::vector<double> PeriodicNurbsCurve::BuildKnotVector() const {
	int p = degree_, n = n_poles();
	std::vector<double> E;
	E.reserve(n + 2 * p + 1);
	for (int i = -p; i <= n + p; ++i) E.push_back(Kappa(i));
	return E;
}

std::pair<std::vector<double>, std::vector<int>> PeriodicNurbsCurve::GetUniqueKnots() const {
	auto [vals, counts] = UniqueWithCounts(knots_);
	vals.push_back(1.0);
	counts.push_back(counts[0]);
	return {vals, counts};
}

std::vector<int> PeriodicNurbsCurve::PoleIndices(const std::vector<int>& ext) const {
	int n = n_poles(), p = degree_;
	std::vector<int> out(ext.size());
	for (size_t i = 0; i < ext.size(); ++i) out[i] = FloorMod(ext[i] - p, n);
	return out;
}

double PeriodicNurbsCurve::Wrap(double u) const { return FloorMod1(u); }

Mat PeriodicNurbsCurve::Hodograph(const Mat& Pw, const std::vector<double>& /*E*/) const {
	int p = degree_, n = Pw.rows(), d = Pw.cols();
	Mat Qw(n, d);
	for (int i = 0; i < n; ++i) {
		double span_w = Kappa(i + p + 1) - Kappa(i + 1);
		int next = (i + 1) % n;
		for (int c = 0; c < d; ++c) {
			double v = p * (Pw(next, c) - Pw(i, c)) / span_w;
			Qw(i, c) = std::isfinite(v) ? v : 0.0;
		}
	}
	return Qw;
}

void PeriodicNurbsCurve::ChangeKnot(int index, double knot) {
	int pinned = n_pinned_knots();
	if (!(pinned <= index && index < n_poles())) {
		throw std::invalid_argument("The index is invalid (the seam knots are fixed)");
	}
	std::vector<double> proposed = knots_;
	double lo = clamped_seam_ ? 1e-4 : 0.0;
	proposed[index] = std::clamp(knot, lo, 1.0 - 1e-4);
	std::sort(proposed.begin(), proposed.end());
	ValidateKnots(proposed, "change_knot");
	knots_ = proposed;
}

void PeriodicNurbsCurve::InsertKnot(double knot_value) {
	double u_bar = knot_value;
	if (!(0.0 <= u_bar && u_bar < 1.0)) throw std::invalid_argument("Inserted knot value must lie in [0, 1)");
	int p = degree_, n = n_poles();
	int k = static_cast<int>(std::upper_bound(knots_.begin(), knots_.end(), u_bar) - knots_.begin()) - 1;
	int s = 0;
	for (double v : knots_) {
		double atol = 1e-8, rtol = 1e-5;
		if (std::abs(v - u_bar) <= atol + rtol * std::abs(u_bar)) s++;
	}
	if (s >= p) {
		throw std::invalid_argument("knot " + std::to_string(u_bar) + " already has multiplicity " +
			std::to_string(s) + "; it may not exceed the degree (" + std::to_string(p) + ")");
	}

	Mat Pw = Homogeneous();
	int d = Pw.cols();
	Mat Qw(n + 1, d);
	int lo = k - p + 1, hi = k - s;
	for (int j = 0; j <= n; ++j) {
		int i;
		if (lo <= j && j <= hi) {
			i = j;
		} else if (lo <= j - (n + 1) && j - (n + 1) <= hi) {
			i = j - (n + 1);
		} else if (j <= k - p) {
			Qw.set_row(j, Pw.row(j));
			continue;
		} else {
			Qw.set_row(j, Pw.row(FloorMod(j - 1, n)));
			continue;
		}
		double a = BoehmAlpha(u_bar, [&](int idx) { return Kappa(idx); }, i, p);
		std::vector<double> row(d);
		int i_mod = FloorMod(i, n), im1_mod = FloorMod(i - 1, n);
		for (int c = 0; c < d; ++c) row[c] = a * Pw(i_mod, c) + (1.0 - a) * Pw(im1_mod, c);
		Qw.set_row(j, row);
	}

	weights_.resize(n + 1);
	Mat new_poles(n + 1, d - 1);
	for (int i = 0; i <= n; ++i) {
		weights_[i] = Qw(i, d - 1);
		for (int c = 0; c < d - 1; ++c) new_poles(i, c) = Qw(i, c) / weights_[i];
	}
	poles_ = new_poles;
	knots_.push_back(u_bar);
	std::sort(knots_.begin(), knots_.end());
}

void PeriodicNurbsCurve::DeleteKnot(int index) {
	if (!(n_pinned_knots() <= index && index < n_poles())) {
		throw std::invalid_argument("The index is invalid (the seam knots cannot be deleted)");
	}
	if (n_poles() - 1 < degree_ + 1) throw std::invalid_argument("cannot delete: the curve needs at least degree+1 poles");
	knots_.erase(knots_.begin() + index);
	poles_ = poles_.RemoveRow(index);
	weights_.erase(weights_.begin() + index);
}

void PeriodicNurbsCurve::AddPole(int index, const std::vector<double>& pole, double weight) {
	if (!(0 <= index && index <= n_poles()) || static_cast<int>(pole.size()) != poles_.cols()) {
		throw std::invalid_argument("The index is invalid");
	}
	poles_ = poles_.InsertRow(index, pole);
	weights_.insert(weights_.begin() + index, weight);
	ResetKnots();
}

void PeriodicNurbsCurve::DeletePole(int index) {
	if (!(0 <= index && index < n_poles())) throw std::invalid_argument("The index is invalid");
	if (n_poles() - 1 < degree_ + 1) throw std::invalid_argument("cannot delete: the curve needs at least degree+1 poles");
	poles_ = poles_.RemoveRow(index);
	weights_.erase(weights_.begin() + index);
	degree_ = std::min(degree_, max_degree());
	ResetKnots();
}

// ===========================================================================
// FitCurve
// ===========================================================================

FitCurve::FitCurve(const Mat& pts, int degree) {
	if (pts.rows() < degree + 1) {
		throw std::invalid_argument("Need at least " + std::to_string(degree + 1) + " points for degree " + std::to_string(degree) + ".");
	}
	degree_ = degree;
	params_ = ArcLengthParams(pts);
	MakeClampedKnots(params_);
	Mat ab = CollocationBanded(params_);
	poles_ = SolveBanded(degree_, degree_, ab, pts);
	weights_.assign(poles_.rows(), 1.0);
}

std::vector<double> FitCurve::ArcLengthParams(const Mat& pts) const {
	int n = pts.rows();
	std::vector<double> cumulative(n, 0.0);
	for (int i = 1; i < n; ++i) {
		cumulative[i] = cumulative[i - 1] + VecNorm(VecSub(pts.row(i), pts.row(i - 1)));
	}
	double total = cumulative.back();
	if (total < 1e-14) throw std::invalid_argument("Data points are all coincident.");
	for (double& v : cumulative) v /= total;
	return cumulative;
}

void FitCurve::MakeClampedKnots(const std::vector<double>& t) {
	int n = static_cast<int>(t.size()), p = degree_;
	interior_knots_.resize(n - p - 1);
	for (int j = 0; j < n - p - 1; ++j) {
		double sum = 0.0;
		for (int k = j + 1; k < j + p + 1; ++k) sum += t[k];
		interior_knots_[j] = sum / p;
	}
}

Mat FitCurve::CollocationBanded(const std::vector<double>& t) const {
	int n = static_cast<int>(t.size()), p = degree_;
	int ku = p, kl = p;
	Mat ab(kl + ku + 1, n, 0.0);
	std::vector<double> U = BuildKnotVector();
	for (int i = 0; i < n; ++i) {
		auto [span, N] = SpansAndBasis(t[i], U, p);
		for (int j = 0; j <= p; ++j) {
			int col = span - p + j;
			ab(ku + i - col, col) = N[j];
		}
	}
	return ab;
}

// ===========================================================================
// DihedralCurve
// ===========================================================================

void DihedralCurve::RebuildInteriorKnots() {
	int n_int = std::max(0, max_degree() - degree_);
	if (n_int == 0) {
		interior_knots_.clear();
		return;
	}
	int n = poles_.rows();
	std::vector<double> delta_s(n, 0.0);
	for (int i = 0; i < n - 1; ++i) {
		delta_s[i + 1] = delta_s[i] + VecNorm(VecSub(poles_.row(i + 1), poles_.row(i)));
	}
	double total = delta_s.back();
	std::vector<double> mid(delta_s.begin() + 1, delta_s.end() - 1);
	for (double& v : mid) v /= total;
	while (static_cast<int>(mid.size()) > n_int) {
		std::vector<double> next(mid.size() - 1);
		for (size_t i = 0; i < next.size(); ++i) next[i] = 0.5 * (mid[i] + mid[i + 1]);
		mid = next;
	}
	interior_knots_ = mid;
}

// ===========================================================================
// MakeCommonKnots
// ===========================================================================

bool MakeCommonKnots(ClampedNurbsCurve& a, ClampedNurbsCurve& b) {
	if (a.degree() != b.degree()) return false;
	// InsertKnot's own coincidence test (np.isclose's defaults): two knots it
	// cannot tell apart must not both end up in one knot vector.
	auto same = [](double u, double v) { return std::abs(u - v) <= 1e-8 + 1e-5 * std::abs(v); };

	// Work on copies; the arguments change only on success.
	ClampedNurbsCurve ca(a.Serialize()), cb(b.Serialize());
	try {
		// 1. Snap b's knots that coincide with a knot of a onto a's value.
		// ChangeKnot re-sorts, so restart the scan after each move.
		for (bool moved = true; moved;) {
			moved = false;
			std::vector<double> ka = ca.interior_knots(), kb = cb.interior_knots();
			for (size_t j = 0; j < kb.size() && !moved; ++j) {
				for (double v : ka) {
					if (kb[j] != v && same(kb[j], v)) {
						cb.ChangeKnot(static_cast<int>(j), v);
						moved = true;
						break;
					}
				}
			}
		}
		// 2. Insert into each curve the knots it lacks, up to the other's
		// multiplicity.
		auto insert_missing = [&](ClampedNurbsCurve& dst, const ClampedNurbsCurve& src) {
			auto [ks, ms] = UniqueWithCounts(src.interior_knots());
			auto [kd, md] = UniqueWithCounts(dst.interior_knots());
			for (size_t i = 0; i < ks.size(); ++i) {
				int have = 0;
				for (size_t j = 0; j < kd.size(); ++j) if (kd[j] == ks[i]) have = md[j];
				for (int r = have; r < ms[i]; ++r) dst.InsertKnot(ks[i]);
			}
		};
		insert_missing(ca, cb);
		insert_missing(cb, ca);
	} catch (const std::invalid_argument&) {
		return false; // a multiplicity would exceed the degree
	}
	if (ca.interior_knots() != cb.interior_knots() || ca.n_poles() != cb.n_poles()) return false;

	// Neither curve may have changed shape beyond the snap in step 1 (a
	// knot moved by at most ~1e-5, so a point by ~1e-5 of the curve's
	// length). Anything larger means a near-coincident triple of knots
	// defeated InsertKnot's multiplicity count; refuse rather than blend
	// a corrupted curve.
	const int n_check = 64;
	for (int k = 0; k <= n_check; ++k) {
		double u = static_cast<double>(k) / n_check;
		std::vector<double> pa = a.Evaluate(u), qa = ca.Evaluate(u), pb = b.Evaluate(u), qb = cb.Evaluate(u);
		if (VecNorm(VecSub(pa, qa)) > 1e-4 || VecNorm(VecSub(pb, qb)) > 1e-4) return false;
	}
	a = ca;
	b = cb;
	return true;
}

// ===========================================================================
// SolveBanded
// ===========================================================================

Mat SolveBanded(int kl, int ku, Mat ab, Mat rhs) {
	int n = ab.cols();
	int dim = rhs.cols();
	// Doolittle banded LU without pivoting (see nurbs.hpp for why that's
	// acceptable here), eliminating column by column in ab's storage.
	for (int j = 0; j < n; ++j) {
		double pivot = ab(ku, j);
		for (int i = j + 1; i <= std::min(j + kl, n - 1); ++i) {
			int row_in_ab = ku + i - j;
			double factor = ab(row_in_ab, j) / pivot;
			if (factor == 0.0) continue;
			for (int c = j; c <= std::min(j + ku, n - 1); ++c) {
				int this_row = ku + i - c;
				int pivot_row = ku + j - c;
				if (this_row >= 0 && this_row < ab.rows() && pivot_row >= 0 && pivot_row < ab.rows()) {
					ab(this_row, c) -= factor * ab(pivot_row, c);
				}
			}
			for (int c = 0; c < dim; ++c) rhs(i, c) -= factor * rhs(j, c);
		}
	}
	Mat x(n, dim);
	for (int i = n - 1; i >= 0; --i) {
		for (int c = 0; c < dim; ++c) {
			double sum = rhs(i, c);
			for (int jcol = i + 1; jcol <= std::min(i + ku, n - 1); ++jcol) {
				sum -= ab(ku + i - jcol, jcol) * x(jcol, c);
			}
			x(i, c) = sum / ab(ku, i);
		}
	}
	return x;
}

} // namespace aerotelic
