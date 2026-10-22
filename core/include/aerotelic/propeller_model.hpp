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

// Which way the propeller turns, by the right-hand rule with the thumb
// along the shaft pointing from the propeller towards the spinner. The
// spinner's tip is the front of the hub and +x is aft, so that thumb
// points along -x and a positive spin is the NEGATIVE sense about the
// model's +x axis -- clockwise seen from behind, the usual right-hand
// tractor propeller.
//
// Positive is the hand the model had before this field existed, so it is
// the default and every project saved without it reads back unchanged.
// Negative is the mirror image: the blade is reflected in the plane of
// the axis and the span, which reverses its camber and twist so it still
// leads with the leading edge and still thrusts forward while turning
// the other way. Nothing else about the propeller changes -- the spinner
// is a body of revolution, and the blade's own twist curve and every
// derived metric are left alone (the hand lives in the geometry, not in
// the sign of beta).
enum class SpinDirection { Positive, Negative };

const char* SpinDirectionName(SpinDirection d);
SpinDirection ParseSpinDirection(const std::string& s);

struct PropellerSpec {
	// Fills spinner_curve with _SPINNER_CURVE's default ogive profile.
	PropellerSpec();

	std::string name = "Propeller";
	double radius = kPropellerDefaultRadius;
	int blade_count = 2;
	double blade_x = 0.26;
	double hub_radius = 0.12;
	// Whether the spinner is built at all. On by default, which is both
	// the usual propeller and what every project saved before this flag
	// existed meant. Off leaves the blades alone -- a bare hub, or a
	// rotor whose hub is modelled by other entities -- and the profile
	// below is kept untouched so it comes back when it is switched on.
	bool spinner = true;
	double spinner_length = 0.40;
	// See SpinDirection. Positive both is the common propeller and is what
	// every project saved before this field existed meant.
	SpinDirection spin_direction = SpinDirection::Positive;
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
	// See PropellerSpec::spinner. The profile and its length stay live
	// either way, so the Spinner editor still works with it switched off
	// and nothing is lost by switching it off and back on.
	bool spinner;
	double spinner_length;
	// See SpinDirection. Only the blades' own geometry depends on it, so
	// it can be switched at any time and switched back with nothing lost.
	SpinDirection spin_direction;
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

	// "Spinner" (the profile revolved as a chain of rings, only when
	// `spinner` is on) and "Blade k" (the blade's WingModel mesh from the
	// hub station outward -- the span range standing in for BuildSolid()'s
	// box trim, exact when the blade has no rake -- turned and placed like
	// BuildSolid()'s blades), without OCCT. Never fused, as BuildSolid()'s
	// default. With the spinner off the body list starts at "Blade 1";
	// bodies are matched by name everywhere they are persisted, so a
	// project that saved a "Spinner" body's colour simply stops finding it.
	std::vector<NamedMesh> BuildMeshes(double tolerance) const override;

	static constexpr int kMaxBlades = kPropellerMaxBlades;

private:
	TopoDS_Shape SpinnerSolid() const;
	TopoDS_Shape BladeSolid(int n_span, std::optional<double> station_tol) const;

	std::unique_ptr<NurbsCurve> spinner_curve_;
	std::unique_ptr<WingModel> blade_;
};

} // namespace aerotelic
