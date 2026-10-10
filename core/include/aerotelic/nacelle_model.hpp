#pragma once

// Port of aerotelic/core/nacelle_model.py. See that module's docstring for
// the construction (tensor-product skins swept around a lip curve,
// chevron cutting). reference_images is not ported (UI-editor display
// data; see wing_model.hpp's file header for the same call on that file).

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Geom_BSplineSurface.hxx>

#include "aerotelic/mat.hpp"
#include "aerotelic/nurbs.hpp"
#include "aerotelic/reference_image.hpp"
#include "aerotelic/solid_model.hpp"

namespace aerotelic {

constexpr double kNacelleDefaultLength = 4.0;
constexpr double kNacelleDefaultChevronDepth = 0.25;
constexpr int kMaxChevrons = 72;

// A closed cubic very close to a circle of `radius` (see
// default_front_curve()'s Python docstring for why not an exact circle).
NurbsSpec DefaultFrontCurveSpec(double radius, int n_poles = 8);

struct NacelleSpec {
	// Fills upper_curve/lower_curve/front_curve/chevron_depth with
	// _DEFAULT_NACELLE's values (a radius-1 front curve, the fixed skin
	// poles) -- same reasoning as PlanformSpec's constructor.
	NacelleSpec();

	double length = kNacelleDefaultLength;
	std::string name = "Nacelle";
	NurbsSpec upper_curve, lower_curve, front_curve;
	int chevron_count = 0;
	// Exactly one of these should be set; neither set means the default
	// depth. Mirrors Python's 'chevron_depth' in d / 'chevron_angle' in d /
	// neither three-way branch -- a plain double can't distinguish
	// "not given" from "given as the default value".
	std::optional<double> chevron_depth;
	std::optional<double> chevron_angle; // legacy, converted on load
	ReferenceImageMap reference_images; // "side", "front"
};

struct NacelleMetrics {
	double length, lip_diameter, max_diameter, te_diameter, chevron_depth;
};

class NacelleModel : public ISolidModel {
public:
	explicit NacelleModel(const NacelleSpec& spec);

	std::string name;
	double length;
	int chevron_count;
	double chevron_depth;
	ReferenceImageMap reference_images;

	const NurbsCurve& upper_curve() const { return *upper_curve_; }
	const NurbsCurve& lower_curve() const { return *lower_curve_; }
	const PeriodicNurbsCurve& front_curve() const { return *front_curve_; }
	// Mutable twins for the curve-editor UI (see wing_model.hpp's Planform
	// note on why these were added on top of the otherwise read-only port).
	NurbsCurve& upper_curve_mut() { return *upper_curve_; }
	NurbsCurve& lower_curve_mut() { return *lower_curve_; }
	PeriodicNurbsCurve& front_curve_mut() { return *front_curve_; }
	// Replaces the front (lip) curve wholesale -- "Reset to Circle...".
	void SetFrontCurve(const NurbsSpec& spec);

	NacelleSpec Serialize() const;
	// Scales every curve's lengths (not the chevron angle equivalents) by
	// length's ratio, keeping proportions -- see Python's set_length().
	void SetLength(double new_length);

	double LipRadius() const;
	double TeRadius() const;
	Mat FrontYZ(int n = 721) const; // lip ring in vehicle (Y, Z), closed
	double RingPerimeter(double scale = 1.0) const;
	double TePerimeter() const;
	double ToothWidth() const;
	double ChevronTipAngle() const;
	NacelleMetrics Metrics() const;

	// Throws std::invalid_argument, in words a user can act on, for a
	// shape that cannot be built.
	void Validate() const;
	TopoDS_Shape BuildSolid() const override;

	static constexpr double kCutOverhang = 0.05;
	static constexpr int kCutSections = 8;

private:
	double DepthFromAngle(double angle_deg) const;
	Handle(Geom_BSplineSurface) SkinSurface(const NurbsCurve& side) const;
	TopoDS_Shape CowlSolid() const;
	struct ArcTable { std::vector<double> t, arc; Mat yz; };
	ArcTable ComputeArcTable(int n = 4001) const;
	std::vector<TopoDS_Shape> ChevronCutters(double depth) const;

	std::unique_ptr<NurbsCurve> upper_curve_, lower_curve_;
	std::unique_ptr<PeriodicNurbsCurve> front_curve_;
};

} // namespace aerotelic
