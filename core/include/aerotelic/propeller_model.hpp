#pragma once

// Port of aerotelic/core/propeller_model.py. See that module's docstring
// for the construction (blade as a turned NURBS-planform WingModel,
// spinner as a revolved profile). reference_images is not ported, same
// call as the other body models.
//
// PropellerSpec::blade is optional: absent means build the default blade
// (MakeDefaultBlade, parameterised only by radius); present carries a full
// WingModelSpec (planform + sections) through, which is what lets a
// hand-edited blade round-trip through Serialize()/construction instead of
// silently reverting to the default on save/reload.

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "aerotelic/airfoil.hpp"
#include "aerotelic/nurbs.hpp"
#include "aerotelic/reference_image.hpp"
#include "aerotelic/solid_model.hpp"
#include "aerotelic/wing_model.hpp"

namespace aerotelic {

constexpr double kPropellerDefaultRadius = 0.965;
constexpr int kPropellerMaxBlades = 12;

struct PropellerSpec {
	// Fills spinner_curve with _SPINNER_CURVE's default ogive profile.
	PropellerSpec();

	std::string name = "Propeller";
	double radius = kPropellerDefaultRadius;
	int blade_count = 2;
	double blade_x = 0.26;
	double hub_radius = 0.12;
	double spinner_length = 0.40;
	NurbsSpec spinner_curve;
	std::optional<WingModelSpec> blade; // absent => MakeDefaultBlade(radius)
	ReferenceImageMap reference_images; // "spinner"
};

struct PropellerMetrics {
	double diameter;
	int blade_count;
	double solidity;
	double activity_factor;
	double beta_75;
	double pitch_ratio;
	double spinner_diameter;
	double spinner_length;
};

// The default blade: a NURBS-planform WingModel whose semi-span is
// `radius`, with the three NACA sections _default_blade_dict() places.
std::unique_ptr<WingModel> MakeDefaultBlade(double radius);

class PropellerModel : public ISolidModel {
public:
	explicit PropellerModel(const PropellerSpec& spec);

	std::string name;
	double radius;
	int blade_count;
	double blade_x;
	double hub_radius;
	double spinner_length;
	ReferenceImageMap reference_images;

	const NurbsCurve& spinner_curve() const { return *spinner_curve_; }
	NurbsCurve& spinner_curve_mut() { return *spinner_curve_; }
	WingModel& blade() { return *blade_; }
	const WingModel& blade() const { return *blade_; }

	PropellerSpec Serialize() const;
	// Keeps every proportion: the blade's semi-span follows (and its edges
	// scale with it, via NURBSPlanform::SetSemiSpan).
	void SetRadius(double new_radius);

	double Diameter() const { return 2.0 * radius; }
	double SpinnerRadius() const;                        // over R
	double SpinnerRadiusAt(double x_over_r) const;        // over R, 0 off the spinner's ends
	std::vector<double> ChordOverR(const std::vector<double>& eta) const;
	double BladeAngle(double eta) const;                  // degrees from the plane of rotation
	PropellerMetrics Metrics() const;

	// Throws std::invalid_argument, in words a user can act on, for a
	// propeller that cannot be built.
	void Validate() const;

	// Nothing fuses by default (see the Python docstring on why): the
	// spinner and every blade come back as one compound of overlapping
	// solids unless fuse=true.
	TopoDS_Shape BuildSolid(int n_span, std::optional<double> station_tol, bool fuse) const;
	TopoDS_Shape BuildSolid() const override { return BuildSolid(40, std::nullopt, false); }

	static constexpr int kMaxBlades = kPropellerMaxBlades;

private:
	TopoDS_Shape SpinnerSolid() const;
	TopoDS_Shape BladeSolid(int n_span, std::optional<double> station_tol) const;

	std::unique_ptr<NurbsCurve> spinner_curve_;
	std::unique_ptr<WingModel> blade_;
};

} // namespace aerotelic
