#pragma once

// Port of aerotelic/core/airfoil.py. A 2-D airfoil cross-section
// interpolated as a B-spline with chord-length parameterisation.

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "aerotelic/mat.hpp"
#include "aerotelic/nurbs.hpp"

namespace aerotelic {

bool IsNaca4Code(const std::string& code);

class Airfoil {
public:
	// Fits a fresh curve through raw contour points (ordered
	// lower_TE -> LE -> upper_TE, x in [0, 1]). Flips winding once and
	// retries if the fit comes out upper-surface-first; throws
	// std::invalid_argument if it still reads as negative thickness after that.
	static std::unique_ptr<Airfoil> FromPoints(const Mat& pts, int degree = 3, std::string name = "Unnamed");

	// Reconstructs exactly from a previously-serialized curve -- e.g. one
	// hand-edited after the original fit -- with no re-fit, no flip-detection.
	static std::unique_ptr<Airfoil> FromCurve(const NurbsSpec& curve_spec, std::string name = "Unnamed");

	// A NACA four-digit section with a closed trailing edge, cosine-spaced,
	// ordered lower TE -> LE -> upper TE.
	static std::unique_ptr<Airfoil> Naca4(const std::string& code, int n = 61);

	std::string name;

	NurbsSpec Serialize() const;

	const NurbsCurve& nurbs_curve() const { return *curve_; }
	// nullptr when the trailing edge is already closed (pts[0] == pts[-1]).
	const NurbsCurve* te_line() const { return te_line_.get(); }

	// ---- geometry queries ----
	std::vector<double> UpperXY(double t) const;
	Mat UpperXY(const std::vector<double>& t) const;
	std::vector<double> LowerXY(double t) const;
	Mat LowerXY(const std::vector<double>& t) const;

	// Closed contour, lower TE -> LE -> upper TE.
	Mat ContourPoints(int n = 200) const;

	std::vector<double> Camber(const std::vector<double>& x) const;
	std::vector<double> Thickness(const std::vector<double>& x) const;
	std::vector<double> Thickness() const; // default x = linspace(0.01, 0.99, 100)

	std::pair<double, double> MaxThickness() const; // (x_location, value)
	std::pair<double, double> MaxCamber() const;     // (x_location, value)

private:
	Airfoil() = default;

	void BuildTeLine();
	// (upper, lower), each (n, 2), t uniform over [0, 1].
	std::pair<Mat, Mat> UniformSampleSurfaces(int n = 200) const;
	// (upper, lower), each (n, 2), t cosine-spaced -- x runs TE -> LE -> TE
	// so these come out with x *decreasing*; see thickness()'s note on why
	// that matters for interpolation.
	std::pair<Mat, Mat> CosineSampleSurfaces(int n = 200) const;

	std::unique_ptr<NurbsCurve> curve_;
	Mat pts_arr_;
	double t_le_ = 0.0;
	std::unique_ptr<NurbsCurve> te_line_;
};

} // namespace aerotelic
