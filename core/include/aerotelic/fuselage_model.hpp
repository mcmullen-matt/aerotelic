#pragma once

// Port of aerotelic/core/fuselage_model.py. Five NURBS guide curves on the
// symmetry plane; each cross-section is four rational-quadratic-Bezier
// (conic) arcs, one per quadrant, assembled into a closed polygon and
// lofted. reference_images and pole_links are not ported -- same call as
// every other body model (see wing_model.hpp's file header); pole_links'
// nose-follow behavior becomes a UI-widget-level concern when the curve
// editor is built, not a core-model one.
//
// Scope cuts: upper_lower_from_profile() (migrating the short-lived
// 2026-09-19/20 single-closed-curve project format) is not ported -- a
// one-time legacy-file converter, not geometry. Python's build_solid()
// does not check BRepOffsetAPI_ThruSections::IsDone() before returning;
// this port does and throws on failure, matching every sibling model's
// convention (a deliberate fix, not a literal mirror -- see the Python
// source's build_solid() for the gap being closed).

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "aerotelic/mat.hpp"
#include "aerotelic/nurbs.hpp"
#include "aerotelic/solid_model.hpp"

namespace aerotelic {

// Closed (N, 2) array of (y, z) for one full conic cross-section.
Mat ConicCrossSection2D(double z_top, double z_bot, double y_half, double rho_upper, double rho_lower, int n_pts = 40);
// (N, 3) array of (x, y, z) for a full cross-section at x_pos.
Mat ConicCrossSection3D(double x_pos, double z_top, double z_bot, double y_half, double rho_upper, double rho_lower, int n_pts = 40);

// The conic rho whose quadrant arc is an exact quarter-circle/ellipse
// (middle weight cos 45 degrees); the default for both rho curves.
constexpr double kFuselageRhoEllipse = 0.41421356237309515; // sqrt(2) - 1

struct FuselageSpec {
	// Fills the five curves with _DEFAULT_FUSELAGE's values.
	FuselageSpec();

	double length = 20.0;
	std::string name = "Fuselage";
	NurbsSpec z_upper_curve, z_lower_curve, y_half_curve, rho_upper_curve, rho_lower_curve;
	// True for a file saved before pole_links existed (Python's
	// 'pole_links' not in d): pins the nose pole's height to 0, not just
	// its station, on z_upper_curve/z_lower_curve/y_half_curve. The YAML
	// loader is what should set this -- see the file header.
	bool legacy = false;
};

struct FuselageSection {
	double x_frac, x_pos, z_top, z_bot, y_half, rho_upper, rho_lower, height, width;
	Mat contour_2d, contour_3d;
};

struct FuselageMetrics {
	double length, max_width, max_height, fineness_ratio;
};

class FuselageModel : public ISolidModel {
public:
	explicit FuselageModel(const FuselageSpec& spec);

	std::string name;
	double length;

	const NurbsCurve& z_upper_curve() const { return *z_upper_curve_; }
	const NurbsCurve& z_lower_curve() const { return *z_lower_curve_; }
	const NurbsCurve& y_half_curve() const { return *y_half_curve_; }
	const NurbsCurve& rho_upper_curve() const { return *rho_upper_curve_; }
	const NurbsCurve& rho_lower_curve() const { return *rho_lower_curve_; }
	// Mutable twins for the curve-editor UI (see wing_model.hpp's Planform
	// note on why these were added on top of the otherwise read-only port).
	NurbsCurve& z_upper_curve_mut() { return *z_upper_curve_; }
	NurbsCurve& z_lower_curve_mut() { return *z_lower_curve_; }
	NurbsCurve& y_half_curve_mut() { return *y_half_curve_; }
	NurbsCurve& rho_upper_curve_mut() { return *rho_upper_curve_; }
	NurbsCurve& rho_lower_curve_mut() { return *rho_lower_curve_; }

	FuselageSpec Serialize() const;
	// Keeps proportions: z_upper/z_lower/y_half scale with the length (rho
	// curves are pure shape parameters and are left alone).
	void SetLength(double new_length);

	std::pair<double, double> ProfileZ(double x_frac) const; // (z_top, z_bot)
	std::array<double, 5> SectionParams(double x_frac) const; // (z_top, z_bot, y_half, rho_upper, rho_lower)
	FuselageSection QuerySection(double x_frac, int n_pts = 80) const;
	FuselageMetrics Metrics() const;

	// Normalised x of the loft stations: n_stations uniform ones, refined
	// by bisecting the worst-fitting interval (on z_top/z_bot/y_half)
	// until within tol or max_stations is reached.
	std::vector<double> StationFracs(int n_stations = 30, std::optional<double> tol = std::nullopt, int max_stations = kMaxStations) const;

	TopoDS_Shape BuildSolid(int n_stations, int n_pts, std::optional<double> station_tol) const;
	TopoDS_Shape BuildSolid() const override { return BuildSolid(30, 40, std::nullopt); }

	void ExportStep(const std::string& path, int n_stations = 30, int n_pts = 40, std::optional<double> station_tol = std::nullopt) const;

	static constexpr double kStationTol = 2e-3;
	static constexpr int kMaxStations = 200;
	static constexpr double kMinStationGap = 5e-5;
	// An apex (pointed nose/tail) opens to this minimum half-height/width
	// rather than a degenerate vertex (lofting to a true vertex through a
	// dense polygon is minutes slower; see the Python docstring).
	static constexpr double kMinSectionHalf = 1e-3;

private:
	std::unique_ptr<NurbsCurve> z_upper_curve_, z_lower_curve_, y_half_curve_, rho_upper_curve_, rho_lower_curve_;
};

} // namespace aerotelic
