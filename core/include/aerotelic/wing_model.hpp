#pragma once

// Port of aerotelic/core/wing_model.py. See that module's docstring for the
// concepts (Airfoil / WingSection / Planform / WingModel).
//
// Scope cut, deliberately: control_surfaces (add_control_surface(),
// build_split(), control_surface_report()) are NOT ported here -- they need
// core/control_surface.py, which this pass excludes. build_solid() itself
// never depended on them (the Python docstring is explicit: "build_solid()
// is unchanged and still returns the one-piece wing"), so the loft is
// complete without them; only the split/report path is missing.
// reference_images and pole_links are likewise not ported -- same call as
// nurbs.py's load_pole_links: UI-editor display/bookkeeping that no
// geometry reads.

#include <array>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Geom_BSplineCurve.hxx>
#include <TopoDS_Wire.hxx>

#include "aerotelic/airfoil.hpp"
#include "aerotelic/mat.hpp"
#include "aerotelic/nurbs.hpp"
#include "aerotelic/reference_image.hpp"
#include "aerotelic/solid_model.hpp"

namespace aerotelic {

constexpr double kDefaultSemiSpan = 10.0;
constexpr double kDefaultRootChord = 3.1;
constexpr double kDefaultTipChord = 0.5;
constexpr double kDefaultSweepAngleDeg = 20.0;
constexpr double kDefaultSweepFraction = 0.0;
constexpr double kDefaultRotationFraction = 0.25;
constexpr double kDefaultRootTwistDeg = 0.0;
constexpr double kDefaultTipTwistDeg = 0.0;
constexpr double kDefaultRootDihedral = 0.0;
constexpr double kDefaultTipDihedral = 0.0;
constexpr double kDefaultKinkFraction = 0.3;
constexpr double kDefaultLeadingEdgeSweepAngleDeg = 20.0;
constexpr double kDefaultTrailingEdgeSweepAngleDeg = 0.0;

// How much clearance a ruled strip must keep between its sections and the
// line where their two planes meet; see WingModel's _fold_margin equivalent.
constexpr double kFoldSafety = 1.5;
constexpr double kExportStationTol = 2e-4; // metres

// ---------------------------------------------------------------------
// Planform
// ---------------------------------------------------------------------

// Common interface WingModel's loft logic needs from any planform type.
// Every concrete planform has a dihedral curve and a twist curve (not
// optional in practice, so not Optional here either).
struct PlanformSpec;

class Planform {
public:
	virtual ~Planform() = default;

	// Places 2-D airfoil-space points (chord-normalised, x in [0,1]) in 3-D
	// at curve parameter s: chord, sweep offset, twist and dihedral baked in.
	virtual Mat AirfoilCoordsToXyz(double s, const Mat& airfoil_coords) const = 0;
	// (eta, x_le, x_te) rows along the span, for drawing.
	virtual Mat PlanformOutline(int n = 200) const = 0;

	virtual double SemiSpan() const = 0;
	// The signed chord at span fraction eta: trailing edge minus leading
	// edge, exactly the scale AirfoilCoordsToXyz applies to the airfoil.
	// Zero collapses the section to a point (a pointed tip); negative turns
	// it inside out, which no loft can use -- WingModel refuses to build
	// such a planform (see RequirePositiveChord()).
	virtual double ChordAt(double eta) const = 0;
	// Plain assignment for Trap*/DoubleTrap; NURBSPlanform overrides this to
	// also rescale le_curve/te_curve proportionally (see its override).
	virtual void SetSemiSpan(double new_value) = 0;
	double Span() const { return 2.0 * SemiSpan(); }
	virtual double RotationFraction() const = 0;
	virtual double TipChord() const = 0;
	virtual double RootChord() const = 0;
	virtual double TaperRatio() const { return TipChord() / RootChord(); }
	virtual double PlanformArea() const = 0;
	double AspectRatio() const { return Span() * Span() / std::max(PlanformArea(), 1e-6); }

	virtual const NurbsCurve& DihedralCurveRef() const = 0;
	virtual const NurbsCurve& TwistCurveRef() const = 0;
	// Mutable twins of the above, added for the curve-editor UI: every other
	// accessor in this port was deliberately read-only (the backend was
	// validated purely for geometry construction), but an interactive editor
	// has to call NurbsCurve::UpdatePole() etc. on the live object the model
	// actually lofts from, not a copy.
	virtual NurbsCurve& DihedralCurveMut() = 0;
	virtual NurbsCurve& TwistCurveMut() = 0;

	// Only meaningful when IsDoubleTrap(): the loft needs the kink station
	// as a geometrically-required breakpoint (see _loft_stations).
	virtual bool IsDoubleTrap() const { return false; }
	virtual double KinkFraction() const { return 0.0; }

	virtual PlanformSpec Serialize() const = 0;
};

struct PlanformSpec {
	// Matches _DEFAULT_TRAP_PLANFORM: twist_curve/dihedral_curve default to
	// valid linear (degree 1, 2-pole) curves, not an empty NurbsSpec --
	// the same defaulting Python's {**_DEFAULT_TRAP_PLANFORM, **(input_dict
	// or {})} merge does before the Planform constructor ever runs.
	PlanformSpec();

	std::string type = "standard"; // "standard" | "taper" | "double_trap" | "nurbs"
	double semi_span = kDefaultSemiSpan;
	double root_chord = kDefaultRootChord;
	double tip_chord = kDefaultTipChord;
	double sweep_angle = kDefaultSweepAngleDeg;       // standard/taper
	double sweep_fraction = kDefaultSweepFraction;     // standard/taper
	double rotation_fraction = kDefaultRotationFraction;
	double kink_fraction = kDefaultKinkFraction;                                   // double_trap
	double leading_edge_sweep_angle = kDefaultLeadingEdgeSweepAngleDeg;             // double_trap
	double trailing_edge_sweep_angle = kDefaultTrailingEdgeSweepAngleDeg;           // double_trap
	NurbsSpec twist_curve;
	NurbsSpec dihedral_curve;
	std::optional<NurbsSpec> le_curve, te_curve; // nurbs
};

NurbsSpec LinearCurveSpec(double y0, double y1);

std::unique_ptr<Planform> MakePlanform(const PlanformSpec& spec);

class TrapPlanform : public Planform {
public:
	explicit TrapPlanform(const PlanformSpec& spec);

	Mat AirfoilCoordsToXyz(double s, const Mat& airfoil_coords) const override;
	Mat PlanformOutline(int n = 200) const override;
	double SemiSpan() const override { return semi_span_; }
	double ChordAt(double eta) const override { return root_chord_ * (1.0 - eta) + tip_chord_ * eta; }
	void SetSemiSpan(double new_value) override { semi_span_ = new_value; }
	double RotationFraction() const override { return rotation_fraction_; }
	double TipChord() const override { return tip_chord_; }
	double RootChord() const override { return root_chord_; }
	double PlanformArea() const override { return Span() * 0.5 * (root_chord_ + tip_chord_); }
	const NurbsCurve& DihedralCurveRef() const override { return *dihedral_curve_; }
	const NurbsCurve& TwistCurveRef() const override { return *twist_curve_; }
	NurbsCurve& DihedralCurveMut() override { return *dihedral_curve_; }
	NurbsCurve& TwistCurveMut() override { return *twist_curve_; }

	PlanformSpec Serialize() const override;

	double sweep_angle() const { return sweep_angle_deg_; }
	double sweep_fraction() const { return sweep_fraction_; }

	// Plain field writes, added for the Planform dialog -- this port's other
	// accessors were validated read-only for geometry construction (see
	// DihedralCurveMut()'s own note), but an interactive "Standard" tab has
	// to set these directly, same as Python's bare `planform.root_chord = v`.
	void SetRootChord(double v) { root_chord_ = v; }
	void SetTipChord(double v) { tip_chord_ = v; }
	void SetSweepAngle(double deg) { sweep_angle_deg_ = deg; }
	void SetSweepFraction(double f) { sweep_fraction_ = f; }

protected:
	double dtype_semi_span() const { return semi_span_; }

	double semi_span_, root_chord_, tip_chord_, sweep_angle_deg_, sweep_fraction_, rotation_fraction_;
	std::unique_ptr<NurbsCurve> twist_curve_;
	std::unique_ptr<DihedralCurve> dihedral_curve_;
};

// Reuses TrapPlanform's construction/AirfoilCoordsToXyz/PlanformOutline;
// only the chord setters' cross-coupling and serialize()'s 'type' differ.
class TrapTaperPlanform : public TrapPlanform {
public:
	explicit TrapTaperPlanform(const PlanformSpec& spec) : TrapPlanform(spec), taper_ratio_(tip_chord_ / root_chord_) {}
	PlanformSpec Serialize() const override;

	void SetTipChord(double v) { tip_chord_ = v; root_chord_ = tip_chord_ / taper_ratio_; }
	void SetRootChord(double v) { root_chord_ = v; tip_chord_ = root_chord_ * taper_ratio_; }
	double TaperRatio() const override { return taper_ratio_; }
	void SetTaperRatio(double v) { taper_ratio_ = v; tip_chord_ = root_chord_ * taper_ratio_; }

private:
	double taper_ratio_;
};

// Two trapezoidal panels joined at a kink station. NOT a TrapPlanform
// subclass in this port (unlike Python, which inherits it purely to reuse
// semi_span/span and airfoil_to_cross_section) -- that relationship doesn't
// fit cleanly in C++ since every other TrapPlanform member is irrelevant
// here; the shared final step is factored into a free function instead
// (see AirfoilToCrossSection in wing_model.cpp).
class DoubleTrapPlanform : public Planform {
public:
	explicit DoubleTrapPlanform(const PlanformSpec& spec);

	Mat AirfoilCoordsToXyz(double s, const Mat& airfoil_coords) const override;
	Mat PlanformOutline(int n = 200) const override;
	double SemiSpan() const override { return semi_span_; }
	double ChordAt(double eta) const override;
	void SetSemiSpan(double new_value) override { semi_span_ = new_value; }
	double RotationFraction() const override { return rotation_fraction_; }
	double TipChord() const override { return tip_chord_; }
	double RootChord() const override { return root_chord_; }
	double PlanformArea() const override;
	const NurbsCurve& DihedralCurveRef() const override { return *dihedral_curve_; }
	const NurbsCurve& TwistCurveRef() const override { return *twist_curve_; }
	NurbsCurve& DihedralCurveMut() override { return *dihedral_curve_; }
	NurbsCurve& TwistCurveMut() override { return *twist_curve_; }
	bool IsDoubleTrap() const override { return true; }
	double KinkFraction() const override { return kink_fraction_; }

	PlanformSpec Serialize() const override;

	// See TrapPlanform's identical note: plain field writes for the
	// Planform dialog's Cranked tab.
	void SetRootChord(double v) { root_chord_ = v; }
	void SetTipChord(double v) { tip_chord_ = v; }
	void SetKinkFraction(double v) { kink_fraction_ = v; }
	void SetLeadingEdgeSweepAngle(double deg) { leading_edge_sweep_angle_deg_ = deg; }
	void SetTrailingEdgeSweepAngle(double deg) { trailing_edge_sweep_angle_deg_ = deg; }
	double LeadingEdgeSweepAngle() const { return leading_edge_sweep_angle_deg_; }
	double TrailingEdgeSweepAngle() const { return trailing_edge_sweep_angle_deg_; }

private:
	double semi_span_, root_chord_, tip_chord_, kink_fraction_, rotation_fraction_;
	double leading_edge_sweep_angle_deg_, trailing_edge_sweep_angle_deg_;
	std::unique_ptr<NurbsCurve> twist_curve_;
	std::unique_ptr<DihedralCurve> dihedral_curve_;
};

// Leading/trailing edge as two independent clamped NURBS curves in (eta, x).
class NURBSPlanform : public Planform {
public:
	explicit NURBSPlanform(const PlanformSpec& spec);

	Mat AirfoilCoordsToXyz(double s, const Mat& airfoil_coords) const override;
	Mat PlanformOutline(int n = 200) const override;
	double SemiSpan() const override { return semi_span_; }
	double ChordAt(double eta) const override { return Chord(eta); }
	// Rescales le_curve/te_curve poles proportionally -- the semi-span is
	// NURBSPlanform's only dimensional input (see the Python docstring).
	void SetSemiSpan(double new_value) override;
	double RotationFraction() const override { return rotation_fraction_; }
	double TipChord() const override;
	double RootChord() const override;
	double PlanformArea() const override;
	const NurbsCurve& DihedralCurveRef() const override { return *dihedral_curve_; }
	const NurbsCurve& TwistCurveRef() const override { return *twist_curve_; }
	NurbsCurve& DihedralCurveMut() override { return *dihedral_curve_; }
	NurbsCurve& TwistCurveMut() override { return *twist_curve_; }

	PlanformSpec Serialize() const override;

	// Channel 1 of `curve` where channel 0 equals eta (inverts the curve's
	// own eta(s), exact at the clamped ends, Newton elsewhere).
	static double ValueAtEta(const NurbsCurve& curve, double eta);
	double XLe(double eta) const { return ValueAtEta(*le_curve_, eta); }
	double XTe(double eta) const { return ValueAtEta(*te_curve_, eta); }
	double Chord(double eta) const { return XTe(eta) - XLe(eta); }

	const NurbsCurve& le_curve() const { return *le_curve_; }
	const NurbsCurve& te_curve() const { return *te_curve_; }
	NurbsCurve& le_curve_mut() { return *le_curve_; }
	NurbsCurve& te_curve_mut() { return *te_curve_; }

private:
	double semi_span_, rotation_fraction_;
	std::unique_ptr<NurbsCurve> le_curve_, te_curve_;
	std::unique_ptr<NurbsCurve> twist_curve_;
	std::unique_ptr<DihedralCurve> dihedral_curve_;
};

// ---------------------------------------------------------------------
// WingSection / WingModel
// ---------------------------------------------------------------------

struct WingSection {
	std::shared_ptr<Airfoil> airfoil;
	double s = 0.0;
};

struct WingSectionSpec {
	double s = 0.0;
	NurbsSpec airfoil_curve;
	std::string airfoil_name = "Unnamed";
};

// A trailing-edge flap: a stretch of the span parted from the wing along a
// hinge line straight in plan view, worked out in each station's section
// plane (section_cut.hpp) -- no booleans. The flap's nose is the circle
// about the hinge tangent to both skins; the main element ends on the
// concentric circle `gap` larger. The main element stays ONE closed body
// with a pocket where the flap sits (mesh.hpp, TriangulateLoftChain); the
// flap is its own closed body. Flaps' pockets (their spans widened by the
// side gap) may not overlap or share an edge station: leave a stretch of
// plain wing between two flaps.
struct FlapSpec {
	std::string name = "Flap";
	// Spanwise extent, as fractions of the semi-span.
	double eta_root = 0.55, eta_tip = 0.95;
	// Hinge position as a chord fraction at each end of that extent; the
	// hinge line is straight in plan view between them (so on a tapered or
	// swept wing equal values do not mean a constant chord fraction).
	double hinge_root = 0.70, hinge_tip = 0.70;
	// Clearance between the flap nose and the main element's cavity, in the
	// model's own units (divided by the local chord on the way into the
	// section plane, so it is constant along the span).
	double gap = 0.004;
	// Spanwise clearance between each end cap of the flap and the main
	// element's pocket face, in the model's own units: the flap keeps
	// [eta_root, eta_tip]; the pocket is cut this much wider at each end
	// (clamped to the root and tip).
	double side_gap = 0.004;
};

// The full round-trip spec Save/Open needs: a PlanformSpec-only
// constructor (below) leaves sections to be added via AddSection(), which
// is fine for interactive construction but loses section edits on
// serialize -- this is what closes that gap (also used to carry
// PropellerModel's blade through its own Serialize()).
struct WingModelSpec {
	std::string name = "Wing";
	PlanformSpec planform;
	std::vector<WingSectionSpec> sections;
	ReferenceImageMap reference_images; // "planform", "dihedral"
	std::vector<FlapSpec> flaps;
};

class WingModel : public ISolidModel {
public:
	// Error-driven adaptive loft refinement (see wing_model.py's "Fold
	// avoidance" / "Error-driven refinement" sections): a ruled strip
	// stays within this many metres of the spanwise laws it approximates.
	static constexpr double kStationTol = 1e-3;
	static constexpr int kMaxStations = 300;

	// Builds (and owns) a Planform from `spec`, with no sections yet --
	// Python's "planform is a dict" constructor path, minus
	// control_surfaces/reference_images/pole_links (see the file header).
	explicit WingModel(const PlanformSpec& spec, std::string name = "Wing");
	// Uses an externally-owned, already-live Planform (e.g. one a UI editor
	// keeps mutating) without copying or owning it -- Python's "planform is
	// an already-constructed object" path.
	explicit WingModel(Planform* external_planform, std::string name = "Wing");
	// Builds the planform AND every section from `spec` -- the round-trip
	// path (see WingModelSpec).
	explicit WingModel(const WingModelSpec& spec);

	std::string name;
	double station_tol = kStationTol;
	// Tracing underlays per editor (see reference_image.hpp); saved, never
	// read by the geometry.
	ReferenceImageMap reference_images;
	// Trailing-edge flaps (see FlapSpec). With any, BuildMeshes() yields the
	// notched main element plus one body per flap; BuildSolid()/ExportStep()
	// still give the one-piece wing.
	std::vector<FlapSpec> flaps;

	// The hinge chord fraction of `flap` at span fraction eta (the straight
	// plan-view hinge line's crossing of that station's chord line), and the
	// section cut there. Throws std::invalid_argument (via SolveFlapCut)
	// where the cut cannot be made.
	double HingeFraction(const FlapSpec& flap, double eta) const;
	// Checks the flaps' extents and ordering; throws std::invalid_argument
	// naming the offending flap.
	void ValidateFlaps() const;
	// The span fractions of `flap`'s pocket in the main element: its extent
	// widened by the side gap at each end, clamped to [0, 1].
	std::pair<double, double> PocketExtent(const FlapSpec& flap) const;

	Planform& planform() { return *planform_; }
	const Planform& planform() const { return *planform_; }

	WingModelSpec Serialize() const;

	void AddSection(std::shared_ptr<Airfoil> airfoil, double s);
	void RemoveSection(int index);
	const std::vector<WingSection>& sections() const { return user_sections_; }
	// Edits for the Sections tab's table: re-sorts by s (mirroring
	// AddSection's own sort) since the table's row order follows it.
	void SetSectionS(int index, double new_s);
	void SetSectionAirfoil(int index, std::shared_ptr<Airfoil> airfoil);

	// The span fractions (eta, as fractions of the semi-span, first to last)
	// over which the planform's chord is negative -- the trailing edge
	// ahead of the leading edge, a section turned inside out -- or nothing
	// when the chord is positive (or exactly zero, a pointed tip) all along.
	// Sampled on a fine grid plus the user sections' stations.
	std::optional<std::pair<double, double>> NegativeChordRange() const;
	// Throws std::invalid_argument naming the wing and the span range when
	// NegativeChordRange() finds one. Every build path calls this first: the
	// geometry is refused until the planform is fixed, rather than lofted
	// through an inverted section (a folded surface whose distance field
	// reports inside and outside the wrong way round near it).
	void RequirePositiveChord() const;

	// Lofts a closed solid with BRepOffsetAPI_ThruSections through explicitly
	// placed airfoil sections. Throws std::invalid_argument with no sections
	// or a negative chord (RequirePositiveChord()), std::runtime_error if the
	// loft or a fold-free station set can't be found.
	TopoDS_Shape BuildSolid(int n_span, bool ruled, std::optional<double> station_tol_override) const;
	TopoDS_Shape BuildSolid() const override { return BuildSolid(40, true, std::nullopt); }

	void ExportStep(const std::string& filepath, int n_span = 200, double station_tol_override = kExportStationTol) const;

	// The loft as a closed triangle mesh (mesh.hpp), built from the section
	// curves without OCCT: ruled strips between the LoftStations() (refined
	// further wherever a strip or a section polygon strays more than
	// `tolerance` from the surface), capped at both ends. [s_min, s_max]
	// restricts the span -- PropellerModel trims its blades at the hub this
	// way instead of a boolean cut.
	TriMesh BuildMesh(double tolerance, double s_min = 0.0, double s_max = 1.0) const;
	// {"Main"} without flaps; with them, the main element (one closed body
	// with a pocket per flap) followed by the flaps, each by its name.
	std::vector<NamedMesh> BuildMeshes(double tolerance) const override;

	// Sorted curve parameters to loft through: the user's sections, root,
	// tip, chord-law breakpoints, crease brackets, and error-driven
	// refinement up to n_span stations. Exposed (not just build_solid()'s
	// internal detail) because it's the part most worth cross-checking --
	// see tests/wing_fixture_check.cpp.
	std::vector<double> LoftStations(int n_span, std::optional<double> tol = std::nullopt) const;

	// The airfoil at span fraction eta: a user section where there is one,
	// else the exact B-spline blend of the two sections around it
	// (Airfoil::Blend). Public for the panel-method network builder
	// (wing_panels.hpp).
	std::shared_ptr<Airfoil> AirfoilAt(double eta) const;

private:
	std::unique_ptr<Planform> owned_planform_;
	Planform* planform_; // = owned_planform_.get(), or external_planform

	std::vector<WingSection> user_sections_;

	bool SpanwiseIsLinear() const;
	std::optional<std::array<std::array<double, 2>, 3>> SpineFrame(double s) const; // (origin_yz, normal_yz, in_plane_yz)
	double PlaneMeetDistance(double s1, double s2) const;
	double SectionReach(double s, std::map<double, double>& cache) const;
	double FoldMargin(double s1, double s2, std::map<double, double>& reach_cache) const;
	std::vector<std::pair<double, bool>> CreaseBrackets(std::map<double, double>& reach_cache, double max_half_width = 0.25) const;

	Mat ProbeCoords() const;
	Mat PlacedProbe(double s, const Mat& probe_pts, std::map<double, Mat>& cache) const;
	std::pair<double, double> StripError(double a, double b, const Mat& probe_pts, std::map<double, Mat>& cache, int n_probe = 3) const;
	std::vector<std::pair<double, bool>> RefineStations(std::vector<std::pair<double, bool>> stations, double tol, int budget, double merge_tol,
		const Mat& probe_pts, std::map<double, Mat>& probe_cache) const;

	std::vector<NamedMesh> BuildFlappedMeshes(double tolerance) const;
	Handle(Geom_BSplineCurve) SectionCurve(const WingSection& section) const;
	TopoDS_Wire SectionWire(const WingSection& section) const;
};

} // namespace aerotelic
