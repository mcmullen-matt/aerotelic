#pragma once

// Port of aerotelic/core/lofted_nacelle_model.py. See that module's
// docstring for the construction (mean-line + guide-ring Bernstein blend,
// skin offsets, lofted + sewn solid, chevron cutting). reference_images
// and pole_links are not ported -- same call as wing_model.hpp/
// nacelle_model.hpp on their equivalents.

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <TopoDS_Wire.hxx>

#include "aerotelic/mat.hpp"
#include "aerotelic/nurbs.hpp"
#include "aerotelic/solid_model.hpp"

namespace aerotelic {

constexpr double kLoftedNacelleDefaultLength = 4.0;
constexpr int kLoftedNacelleMaxChevrons = 72;
constexpr int kLoftedNacelleMaxSections = 24;

// A closed near-circle to start a guide curve from (same helper
// nacelle_model.py's default_front_curve is; shared there as
// DefaultFrontCurveSpec).
NurbsSpec DefaultSectionCurveSpec(double radius = 1.0);

struct GuideSectionSpec { double station = 0.0; NurbsSpec curve; };

struct LoftedNacelleSpec {
	// Fills centreline_curve/outer_curve/inner_curve/sections with
	// _DEFAULT_LOFTED_NACELLE's values.
	LoftedNacelleSpec();

	double length = kLoftedNacelleDefaultLength;
	std::string name = "Nacelle";
	NurbsSpec centreline_curve, outer_curve, inner_curve;
	std::vector<GuideSectionSpec> sections;
	int chevron_count = 0;
	double chevron_depth = 0.25;
};

struct GuideSection {
	double station;
	std::unique_ptr<PeriodicNurbsCurve> curve;
};

struct LoftedNacelleMetrics {
	double length, lip_diameter, max_diameter, te_diameter, max_thickness, chevron_depth;
	int n_sections;
};

class LoftedNacelleModel : public ISolidModel {
public:
	explicit LoftedNacelleModel(const LoftedNacelleSpec& spec);

	std::string name;
	double length;
	int chevron_count;
	double chevron_depth;

	const NurbsCurve& centreline_curve() const { return *centreline_curve_; }
	const NurbsCurve& outer_curve() const { return *outer_curve_; }
	const NurbsCurve& inner_curve() const { return *inner_curve_; }
	const std::vector<GuideSection>& sections() const { return sections_; }
	// Mutable twins for the curve-editor UI (see wing_model.hpp's Planform
	// note on why these were added on top of the otherwise read-only port).
	NurbsCurve& centreline_curve_mut() { return *centreline_curve_; }
	NurbsCurve& outer_curve_mut() { return *outer_curve_; }
	NurbsCurve& inner_curve_mut() { return *inner_curve_; }
	// Index into sections() by stable identity: SortSections() (triggered by
	// AddSection/RemoveSection and the Guide-curves tab's own station edits)
	// reorders the vector, so a UI tracking "the guide curve being edited"
	// must do so by this pointer, not by index -- same as the Python panel's
	// own note about tracking the GuideSection object rather than an index.
	GuideSection* FindSectionByCurve(const PeriodicNurbsCurve* curve);
	// Re-sorts sections_ by station (same ordering AddSection/RemoveSection
	// already maintain) after the Guide-curves tab edits a station directly.
	void ResortSections() { SortSections(); }

	LoftedNacelleSpec Serialize() const;
	void SetLength(double new_length);

	void AddSection(double station, std::optional<NurbsSpec> curve = std::nullopt);
	void RemoveSection(int index); // throws if only one section remains
	std::vector<double> SectionStations() const;

	double CentrelineZ(double x_frac) const;
	std::pair<double, double> Skins(double x_frac) const; // (outer, inner)
	double WallThickness(double x_frac) const;

	// (outer, inner) at x_frac as closed (n+1, 2) arrays of (Y, Z).
	std::pair<Mat, Mat> RingsYZ(double x_frac, int n_pts = 200) const;
	Mat MeanYZ(double x_frac, int n_pts = 200) const;
	double MeanRadius(double x_frac, int n_pts = 200) const;
	// (n, 5) of (x/L, outer top, outer bottom, mean top, mean bottom).
	Mat OutlineZs(int n = 200) const;

	double TePerimeter(int n = 2001) const;
	double ChevronTipAngle() const;
	LoftedNacelleMetrics Metrics() const;

	void Validate() const;

	std::vector<double> StationFracs(int n_stations = 30, std::optional<double> tol = std::nullopt, int max_stations = kMaxStations) const;

	TopoDS_Shape BuildSolid(int n_stations, int n_pts, std::optional<double> station_tol) const;
	TopoDS_Shape BuildSolid() const override { return BuildSolid(30, 80, std::nullopt); }

	static constexpr double kStationTol = 2e-3;
	static constexpr int kMaxStations = 200;
	static constexpr double kMinStationGap = 5e-5;
	static constexpr double kCutOverhang = 0.05;
	static constexpr double kCutMargin = 0.10;
	static constexpr int kCutSections = 8;

private:
	// Guide curves sampled alike so the blend is a weighted sum (Python's
	// _GuideTable): stations (sorted), rings (one (n_pts,2) Mat per guide),
	// and a dense (t, x) table inverting the Bernstein blend's x(t).
	struct GuideTable { std::vector<double> stations; std::vector<Mat> rings; std::vector<double> t, x; };
	GuideTable BuildGuideTable(int n_pts) const;
	static std::vector<double> Bernstein(int n_ctrl, double t);
	static Mat Blend(double x_frac, const GuideTable& tab);
	static Mat Offset(const Mat& ring, double distance);
	std::pair<Mat, Mat> RingsYZImpl(double x_frac, const GuideTable& table) const;
	static Mat Closed(const Mat& ring);

	double YAt(const NurbsCurve& curve, double x) const; // see Python's _y_at, single-x convenience
	std::vector<double> YAtMany(const NurbsCurve& curve, const std::vector<double>& xs) const;

	TopoDS_Wire SectionWire(double x_frac, const Mat& ring) const;
	TopoDS_Shape Shell(const std::vector<TopoDS_Wire>& wires, bool ruled = false) const;
	TopoDS_Shape CowlSolid(int n_stations, int n_pts, std::optional<double> station_tol) const;

	struct TeArcTable { std::vector<double> arc; Mat rel; double perimeter; };
	TeArcTable ComputeTeArcTable(int n = 2001) const;
	std::pair<double, double> CutScales(double depth, int n_pts = 240) const;
	std::vector<TopoDS_Shape> ChevronCutters(double depth) const;

	void SortSections();

	std::unique_ptr<NurbsCurve> centreline_curve_, outer_curve_, inner_curve_;
	std::vector<GuideSection> sections_;
};

} // namespace aerotelic
