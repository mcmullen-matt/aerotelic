#pragma once

// Port of aerotelic/core/nurbs.py. Rational B-spline curves, evaluated
// directly from the NURBS equation (Piegl & Tiller, The NURBS Book:
// A2.1 FindSpan, A2.2 BasisFuns, A4.1 CurvePoint). See nurbs.py's module
// docstring for the full derivation; comments here only cover points
// where the C++ translation itself needs explanation.
//
// Deliberate differences from the Python module:
//   - double only (Python's dtype parameter is never exercised with
//     anything else in this codebase).
//   - scalar-oriented API (evaluate(double), evaluate(vector<double>))
//     instead of numpy vectorisation; the vectorised Python code becomes
//     explicit loops, which is the natural C++ shape and was never a
//     semantic requirement.
//   - load_pole_links() is not ported: it is pure ui/nurbs_editor.py
//     bookkeeping that no geometry code reads (see the Python docstring).

#include <vector>
#include <optional>
#include <memory>
#include <string>
#include <functional>

#include "aerotelic/mat.hpp"

namespace aerotelic {

// Mirrors the input/serialize dict. Which fields are meaningful depends
// on `periodic`; Create() dispatches on it exactly as NURBSCurve.__new__ does.
struct NurbsSpec {
	int degree = 0;
	Mat poles;
	std::optional<std::vector<double>> weights;
	bool periodic = false;
	std::optional<std::vector<double>> interior_knots;   // clamped only
	std::optional<std::vector<double>> knots;             // periodic only
	bool clamped_seam = false;
};

class NurbsCurve {
public:
	virtual ~NurbsCurve() = default;

	// Dispatches to ClampedNurbsCurve or PeriodicNurbsCurve on spec.periodic,
	// matching NURBSCurve.__new__.
	static std::unique_ptr<NurbsCurve> Create(const NurbsSpec& spec);

	NurbsSpec Serialize() const;

	// ---- properties ----
	int degree() const { return degree_; }
	int n_poles() const { return poles_.rows(); }
	int max_degree() const { return poles_.rows() - 1; }
	virtual bool periodic() const { return false; }
	Mat poles() const { return poles_; }
	std::vector<double> weights() const { return weights_; }
	Mat poles_and_weights() const { return poles_.WithColumn(weights_); }
	virtual std::vector<double> knots() const = 0;
	// Unique knots of build_knot_vector() with multiplicities. Both
	// subclasses already implement this identically in spirit (clamped:
	// the knot vector's own duplicates; periodic: one period plus its
	// closing value at 1.0) -- hoisted here so callers that only have a
	// NurbsCurve& (e.g. WingModel reading a Planform's dihedral curve
	// generically) can call it without knowing which subclass it is.
	virtual std::pair<std::vector<double>, std::vector<int>> GetUniqueKnots() const = 0;

	// ---- pole operations that leave the knots alone ----
	void UpdatePole(int index, const std::vector<double>& pole);
	void SetPoleWeight(int index, double weight);
	void SetPolesAndWeights(const Mat& poles, const std::vector<double>& weights);
	void SetDegree(int degree);

	// ---- evaluation ----
	std::vector<double> Evaluate(double s) const;
	Mat Evaluate(const std::vector<double>& s) const;
	std::vector<double> Derivative(double s) const;
	Mat Derivative(const std::vector<double>& s) const;
	Mat Sample(int n) const;
	std::vector<double> ZAtEta(double eta) const;
	// Newton's method for the parameter at which Evaluate(s)[index] == value.
	std::optional<double> Inverse(int index, double value, double initial_s, double tol = 1e-9) const;

	// (m, n_poles): column i is N_{i,p}(s), or the rational blend if `rational`.
	Mat BasisMatrix(const std::vector<double>& s, bool rational = false) const;

	// Public entry point for the protected ResetKnots() hook (reset_knots()
	// in Python) -- it's protected because it's also an internal hook a
	// subclass's own constructor/SetDegree() calls, but the curve-editor
	// UI's "Reset Knots" button needs to call it from outside the class.
	void ResetKnotsPublic() { ResetKnots(); }

protected:
	int degree_ = 0;
	Mat poles_;
	std::vector<double> weights_;

	void InitShared(const NurbsSpec& spec);

	// ---- hooks a subclass must provide (NURBSCurve's abstract methods) ----
	virtual void LoadKnots(const NurbsSpec& spec) = 0;
	virtual void SerializeKnots(NurbsSpec& out) const = 0;
	virtual void ResetKnots() = 0;
	virtual std::vector<double> BuildKnotVector() const = 0;
	virtual std::vector<int> PoleIndices(const std::vector<int>& ext) const = 0;
	virtual double Wrap(double u) const = 0;
	// Homogeneous poles of the degree-(p-1) derivative curve.
	virtual Mat Hodograph(const Mat& Pw, const std::vector<double>& E) const = 0;
	virtual void AfterDegreeChange() = 0;

	void CheckMultiplicity(const std::vector<double>& values, const std::string& where) const;

	Mat Homogeneous() const;

	struct SpanBasis { int span; std::vector<double> N; };
	// For parameter u on knot array U of degree p: the knot span and the
	// p+1 non-zero basis functions there (P&T A2.1 FindSpan + A2.2 BasisFuns,
	// fused -- same as nurbs.py's _spans_and_basis but for one u at a time).
	static SpanBasis SpansAndBasis(double u, const std::vector<double>& U, int p);

	// Non-rational B-spline evaluation of degree d at one u, using whichever
	// stored poles PoleIndices maps the window to.
	std::vector<double> EvalSpline(double u, const std::vector<double>& E, int d, const Mat& ctrl) const;

	static double BoehmAlpha(double u_bar, const std::function<double(int)>& knot, int i, int p);

public:
	// Build().evaluate fast path for a single std::vector<double> of poles-and-
	// weight row dimension; used by SpanBasis callers in nurbs.cpp.
	static std::vector<double> UniqueSortedValues(std::vector<double> values);
};

// ===========================================================================
// Clamped (open) curves
// ===========================================================================

class ClampedNurbsCurve : public NurbsCurve {
public:
	explicit ClampedNurbsCurve(const NurbsSpec& spec);

	std::vector<double> interior_knots() const { return interior_knots_; }
	std::vector<double> knots() const override { return interior_knots_; }

	// Unique knots of build_knot_vector() with multiplicities.
	std::pair<std::vector<double>, std::vector<int>> GetUniqueKnots() const override;

	void ChangeKnot(int index, double knot);
	// Boehm's algorithm (P&T A5.1): insert a knot, recomputing poles so the
	// curve's shape is unchanged. Adds one pole.
	void InsertKnot(double knot_value);
	// NOT shape-preserving (unlike InsertKnot): drops an interior knot and
	// the pole after it.
	void DeleteKnot(int index);
	void AddPole(int index, const std::vector<double>& pole, double weight = 1.0);
	void DeletePole(int index);

protected:
	std::vector<double> interior_knots_;

	void LoadKnots(const NurbsSpec& spec) override;
	void SerializeKnots(NurbsSpec& out) const override;
	void ResetKnots() override;
	std::vector<double> BuildKnotVector() const override;
	std::vector<int> PoleIndices(const std::vector<int>& ext) const override;
	double Wrap(double u) const override;
	Mat Hodograph(const Mat& Pw, const std::vector<double>& U) const override;
	void AfterDegreeChange() override;

	// Default: uniform spacing. DihedralCurve overrides this.
	virtual void RebuildInteriorKnots();

	// For FitCurve, which -- like Python's FitCurve.__init__ -- bypasses
	// the normal spec-driven constructor because poles are solved for,
	// not supplied upfront.
	ClampedNurbsCurve() = default;
};

// ===========================================================================
// Periodic (closed) curves
// ===========================================================================

class PeriodicNurbsCurve : public NurbsCurve {
public:
	explicit PeriodicNurbsCurve(const NurbsSpec& spec);

	bool periodic() const override { return true; }
	bool clamped_seam() const { return clamped_seam_; }
	int seam_multiplicity() const;
	// How many leading knots are pinned at the seam and cannot be edited:
	// all p of a clamped seam, otherwise just the seam itself.
	int n_pinned_knots() const { return clamped_seam_ ? degree_ : 1; }
	std::vector<double> knots() const override { return knots_; }

	// The periodic knot sequence at any integer index, kappa(i) = knots[i
	// mod n] + floor(i / n).
	double Kappa(int i) const;

	std::pair<std::vector<double>, std::vector<int>> GetUniqueKnots() const override;

	void ChangeKnot(int index, double knot);
	void InsertKnot(double knot_value);
	void DeleteKnot(int index);
	void AddPole(int index, const std::vector<double>& pole, double weight = 1.0);
	void DeletePole(int index);

protected:
	std::vector<double> knots_;
	bool clamped_seam_ = false;

	void LoadKnots(const NurbsSpec& spec) override;
	void ValidateKnots(const std::vector<double>& k, const std::string& where) const;
	void SerializeKnots(NurbsSpec& out) const override;
	void ResetKnots() override;
	std::vector<double> BuildKnotVector() const override;
	std::vector<int> PoleIndices(const std::vector<int>& ext) const override;
	double Wrap(double u) const override;
	Mat Hodograph(const Mat& Pw, const std::vector<double>& E) const override;
	void AfterDegreeChange() override;
};

// ===========================================================================
// Clamped specialisations
// ===========================================================================

// A clamped B-spline (unit weights) interpolating given points exactly.
class FitCurve : public ClampedNurbsCurve {
public:
	FitCurve(const Mat& pts, int degree);

	std::vector<double> collocation_parameters() const { return params_; }

	std::vector<double> ArcLengthParams(const Mat& pts) const;
	void MakeClampedKnots(const std::vector<double>& t);
	// A[i,j] = N_j(t_i) in LAPACK/scipy banded storage: ab[ku+i-j, j] = A[i,j].
	Mat CollocationBanded(const std::vector<double>& t) const;

private:
	std::vector<double> params_;
};

// Clamped curve whose default interior knots follow the pole polygon's
// cumulative chord length rather than uniform spacing.
class DihedralCurve : public ClampedNurbsCurve {
public:
	// ClampedNurbsCurve(spec)'s constructor body calls ResetKnots() while
	// this object's dynamic type is still ClampedNurbsCurve -- C++ virtual
	// dispatch during base-class construction cannot reach DihedralCurve's
	// override, unlike Python's __new__-based dispatch the original class
	// relies on. So when no interior_knots were given, the base constructor
	// just computed (and discarded) the wrong, uniformly-spaced default;
	// redo it here now that RebuildInteriorKnots() can dispatch correctly.
	explicit DihedralCurve(const NurbsSpec& spec) : ClampedNurbsCurve(spec) {
		if (!spec.interior_knots) RebuildInteriorKnots();
	}

protected:
	void RebuildInteriorKnots() override;
};

// Solves A x = rhs for banded A (bandwidth kl=ku) given in the same
// ab[ku+i-j, j] = A[i,j] storage FitCurve::CollocationBanded produces.
// Doolittle banded LU, no pivoting: adequate for the Schoenberg-Whitney
// collocation matrices this is used for (diagonally well-conditioned by
// construction), unlike a general banded solve. scipy's solve_banded calls
// LAPACK's gbsv, which does pivot -- this is a deliberate simplification,
// not a hidden gap: flag it if FitCurve is ever used on an adversarial
// knot placement.
Mat SolveBanded(int kl, int ku, Mat ab, Mat rhs);

} // namespace aerotelic
