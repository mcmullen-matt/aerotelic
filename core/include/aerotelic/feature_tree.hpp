#pragma once

// Port of aerotelic/core/feature_tree.py. The dependency graph that owns
// derived geometry: Feature nodes cache their result, mark_dirty()
// propagates forward, rebuild() recomputes only what is stale. See the
// Python module's docstring for the full rationale.
//
// Deliberate differences from Python:
//   - Python's "one resolver callable, different return shape depending
//     on caller" (ComponentSourceFeature wants a model, EntityShapeFeature
//     wants a (model, get_transform) pair) doesn't type-check in C++;
//     FeatureTreeResolver below splits it into the two calls it actually
//     ever makes.
//   - serialize()/_load() work against a typed FeatureSpec struct rather
//     than an untyped dict -- same shape, same field names, just typed.
//     Encoding that struct to/from a text format (YAML, same as the
//     Python project files) is left to whatever layer owns project I/O;
//     nothing else in core/ depends on FeatureTree, so that's a boundary
//     worth keeping explicit rather than baking a text format in here.

#include <array>
#include <exception>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <BRepAlgoAPI_BooleanOperation.hxx>
#include <TopoDS_Shape.hxx>

#include "aerotelic/solid_model.hpp"

namespace aerotelic {

// ---------------------------------------------------------------------
// OCC geometry helpers
// ---------------------------------------------------------------------

TopoDS_Shape ApplyTranslation(const TopoDS_Shape& shape, const std::array<double, 3>& translation);
TopoDS_Shape ApplyRotation(const TopoDS_Shape& shape, const std::array<double, 3>& axis, double angle_deg);
// `shape` rotated then translated. Skips the OCCT round-trip entirely when
// both are no-ops (see the Python docstring: BRepBuilderAPI_Transform
// rebuilds the whole shape whether or not the transform does anything).
TopoDS_Shape Place(const TopoDS_Shape& shape, const std::array<double, 3>& translation,
	const std::array<double, 3>& rotation_axis = {0, 0, 0}, double rotation_deg = 0.0);
TopoDS_Shape ApplyMirror(const TopoDS_Shape& shape, const std::array<double, 3>& point, const std::array<double, 3>& normal);

// Runs fuse/cut/common with history tracking on. Throws std::invalid_argument
// for an unknown op_type, std::runtime_error if the operation fails.
std::unique_ptr<BRepAlgoAPI_BooleanOperation> BooleanOperation(const std::string& op_type, const TopoDS_Shape& body1, const TopoDS_Shape& body2);

// Rounds the seam a BooleanOperation() just created. Returns (shape, applied);
// applied is false (and shape is the un-filleted input) when there were no
// section edges or the solver failed at this radius -- not an error, see the
// Python docstring on why a caller must check this rather than assume success.
std::pair<TopoDS_Shape, bool> FilletSeam(BRepAlgoAPI_BooleanOperation& bool_op, double fillet_radius);

// ---------------------------------------------------------------------
// Feature nodes
// ---------------------------------------------------------------------

enum class FeatureKind { ComponentSource, EntityShape, Placement, Mirror, Boolean, Fillet };
const char* FeatureKindName(FeatureKind k);

// (tx, ty, tz, ax, ay, az, angle_deg) -- an entity's current placement,
// supplied by whatever owns the scene (not this module's business).
struct EntityTransform { double tx, ty, tz, ax, ay, az, angle_deg; };

struct Placement { std::array<double, 3> translation; std::array<double, 3> rotation_axis; double rotation_deg; };

// One node in the dependency graph. Subclasses implement Compute(), called
// only when the node is dirty and only after every upstream input is rebuilt.
class Feature {
public:
	Feature(std::string name, FeatureKind kind) : name(std::move(name)), kind(kind) {}
	// Deliberately does NOT call Detach(): FeatureTree's map destroys
	// Features in arbitrary (hash) order on teardown, not dependency order,
	// so detaching here could dereference an upstream Feature* that the same
	// teardown has already destroyed. Python has no equivalent hazard --
	// _detach() there is only ever called explicitly (Add()'s rejection path,
	// Remove()'s cascade) -- so this mirrors that: every real detach happens
	// at an explicit call site, never implicitly during destruction.
	virtual ~Feature() = default;

	std::string name;
	FeatureKind kind;
	bool dirty = true;
	TopoDS_Shape result;
	std::exception_ptr error;
	// True for nodes generated as a step inside a larger authored operation
	// (e.g. an intermediate fuse folding several operands) rather than
	// created by the user directly.
	bool internal = false;

	std::vector<Feature*> inputs() const { return inputs_; }
	std::vector<Feature*> dependents() const { return dependents_; }

	void MarkDirty();
	// Recomputes only if dirty, bringing every upstream input up to date
	// first. Rethrows whatever Compute() throws, having recorded it in `error`.
	TopoDS_Shape Rebuild();

	// Only ComponentSourceFeature/EntityShapeFeature have a ref; mirrors
	// Python's getattr(f, 'ref', None) duck-typing.
	virtual std::optional<std::string> Ref() const { return std::nullopt; }

protected:
	void DependsOn(Feature* upstream);
	void DependsOn(Feature* a, Feature* b) { DependsOn(a); DependsOn(b); }
	virtual TopoDS_Shape Compute() = 0;

private:
	void Detach();
	std::vector<Feature*> inputs_;
	std::vector<Feature*> dependents_;
	friend class FeatureTree;
};

// Wraps a model (e.g. PrimitiveModel/WingModel); produces its canonical,
// unplaced shape via BuildSolid().
class ComponentSourceFeature : public Feature {
public:
	ComponentSourceFeature(std::string name, ISolidModel* model, std::string ref)
		: Feature(std::move(name), FeatureKind::ComponentSource), model(model), ref(std::move(ref)) {}
	ISolidModel* model;
	std::string ref;
	std::optional<std::string> Ref() const override { return ref; }
protected:
	TopoDS_Shape Compute() override { return model->BuildSolid(); }
};

// A scene entity's CURRENT placed shape: its model's canonical solid with
// whatever translation/rotation is presently set for it.
class EntityShapeFeature : public Feature {
public:
	EntityShapeFeature(std::string name, ISolidModel* model, std::function<EntityTransform()> get_transform, std::string ref)
		: Feature(std::move(name), FeatureKind::EntityShape), model(model), get_transform(std::move(get_transform)), ref(std::move(ref)) {}
	ISolidModel* model;
	std::function<EntityTransform()> get_transform;
	std::string ref;
	std::optional<std::string> Ref() const override { return ref; }

	// Hands this node a solid someone has *just* built from the same model,
	// to be used instead of building it again (consumed by the next Compute()
	// and discarded; see the Python docstring).
	void OfferSolid(const TopoDS_Shape& shape) { offered_ = shape; MarkDirty(); }
protected:
	TopoDS_Shape Compute() override;
private:
	std::optional<TopoDS_Shape> offered_;
};

// Bakes a rotation+translation into an upstream shape.
class PlacementFeature : public Feature {
public:
	PlacementFeature(std::string name, Feature* source, Placement placement)
		: Feature(std::move(name), FeatureKind::Placement), source(source), placement(std::move(placement)) {
		DependsOn(source);
	}
	Feature* source;
	Placement placement;
protected:
	TopoDS_Shape Compute() override { return Place(source->result, placement.translation, placement.rotation_axis, placement.rotation_deg); }
};

// Derives the port-side shape by reflecting an upstream (starboard) node
// about the Y=0 symmetry plane by default. Never authored independently.
class MirrorFeature : public Feature {
public:
	MirrorFeature(std::string name, Feature* source,
		std::array<double, 3> point = {0, 0, 0}, std::array<double, 3> normal = {0, 1, 0})
		: Feature(std::move(name), FeatureKind::Mirror), source(source), point(point), normal(normal) {
		DependsOn(source);
	}
	Feature* source;
	std::array<double, 3> point, normal;
protected:
	TopoDS_Shape Compute() override { return ApplyMirror(source->result, point, normal); }
};

// fuse/cut/common between two upstream shapes. Retains the operation object
// so a downstream FilletFeature can re-derive the seam edges fresh from
// THIS build rather than a stored reference.
class BooleanFeature : public Feature {
public:
	BooleanFeature(std::string name, Feature* body1, Feature* body2, std::string op_type = "fuse")
		: Feature(std::move(name), FeatureKind::Boolean), body1(body1), body2(body2), op_type(std::move(op_type)) {
		DependsOn(body1, body2);
	}
	Feature* body1;
	Feature* body2;
	std::string op_type;
	std::unique_ptr<BRepAlgoAPI_BooleanOperation> bool_op;
protected:
	TopoDS_Shape Compute() override {
		bool_op = BooleanOperation(op_type, body1->result, body2->result);
		return bool_op->Shape();
	}
};

// Rounds the seam its upstream BooleanFeature just created. `applied`
// reports whether the fillet actually took effect; see FilletSeam().
class FilletFeature : public Feature {
public:
	FilletFeature(std::string name, BooleanFeature* boolean, double fillet_radius)
		: Feature(std::move(name), FeatureKind::Fillet), boolean(boolean), fillet_radius(fillet_radius) {
		DependsOn(boolean);
	}
	BooleanFeature* boolean;
	double fillet_radius;
	bool applied = false;
protected:
	TopoDS_Shape Compute() override {
		auto [shape, ok] = FilletSeam(*boolean->bool_op, fillet_radius);
		applied = ok;
		return shape;
	}
};

// ---------------------------------------------------------------------
// Serialization (typed; see the file header for why it's not a dict/YAML)
// ---------------------------------------------------------------------

struct FeatureSpec {
	FeatureKind kind;
	std::string name;
	std::vector<std::string> inputs;
	bool internal = false;
	std::optional<std::string> ref;              // ComponentSource / EntityShape
	std::optional<Placement> placement;           // Placement
	std::optional<std::array<double, 3>> point, normal; // Mirror
	std::optional<std::string> op_type;           // Boolean
	std::optional<double> fillet_radius;          // Fillet
};

struct FeatureResult { std::string name, label, starboard; std::optional<std::string> port; };

struct FeatureTreeSpec {
	std::vector<FeatureSpec> features;
	std::vector<FeatureResult> results;
};

// ref -> model for component_source nodes; ref -> (model, get_transform)
// for entity_shape nodes. Required only when loading a tree that has them.
struct FeatureTreeResolver {
	std::function<ISolidModel*(const std::string&)> resolve_component;
	std::function<std::pair<ISolidModel*, std::function<EntityTransform()>>(const std::string&)> resolve_entity;
};

// ---------------------------------------------------------------------
// FeatureTree
// ---------------------------------------------------------------------

// Owns the feature nodes and the derived results built from them. Empty on
// construction unless loaded from a FeatureTreeSpec.
class FeatureTree {
public:
	FeatureTree() = default;
	explicit FeatureTree(const FeatureTreeSpec& spec, const FeatureTreeResolver& resolver) { Load(spec, resolver); }

	// Takes ownership. Throws std::invalid_argument if the name is taken
	// (detaching the rejected node from its inputs' dependent lists first,
	// mirroring the Python add()'s care about not leaving it half-wired).
	Feature* Add(std::unique_ptr<Feature> feature);
	Feature* Get(const std::string& name) const;
	bool Contains(const std::string& name) const { return features_.count(name) != 0; }
	size_t size() const { return features_.size(); }

	// Features in build order (index 0 built first).
	const std::vector<std::string>& order() const { return order_; }
	int IndexOf(const std::string& name) const;

	// (lowest, highest) positions `name` may legally occupy, bounded by its
	// direct inputs and dependents.
	std::pair<int, int> AllowedIndexRange(const std::string& name) const;
	// Throws std::invalid_argument if `index` would put the node before an
	// input or after a dependent.
	void Move(const std::string& name, int index);

	// Removes a feature and everything downstream of it (cascading), and
	// drops any result referencing a removed node.
	void Remove(const std::string& name);
	// Removes every source node carrying `ref` (and its dependents).
	void RemoveByRef(const std::string& ref);

	void MarkDirty(const std::string& name);
	// Marks every source node carrying `ref` dirty -- the hook for "the
	// model or placement behind this entity changed".
	void MarkDirtyByRef(const std::string& ref);

	// Nodes nothing else depends on -- the tree's outputs.
	std::vector<Feature*> Terminals() const;
	// Recomputes stale terminals (or `names`, if given) and whatever they
	// depend on. Returns {feature name: exception} for nodes that failed;
	// a failure is contained to its own branch, not the whole tree.
	std::map<std::string, std::exception_ptr> Rebuild(const std::vector<std::string>* names = nullptr);

	FeatureResult AddResult(const std::string& name, const std::string& label, Feature* starboard, Feature* port = nullptr);
	const std::vector<FeatureResult>& results() const { return results_; }

	FeatureTreeSpec Serialize() const;
	void Load(const FeatureTreeSpec& spec, const FeatureTreeResolver& resolver);

private:
	bool IsTopological(const std::vector<std::string>& order) const;

	std::unordered_map<std::string, std::unique_ptr<Feature>> features_;
	std::vector<std::string> order_;
	std::vector<FeatureResult> results_;
};

// Builds one boolean operation over `operands`, folded left to right
// ((A op B) op C), and returns its terminal (after an optional fillet).
// All-or-nothing: if any step fails, every node this added is removed again
// before the error propagates.
Feature* FoldBoolean(FeatureTree& tree, const std::string& name, const std::vector<Feature*>& operands,
	const std::string& op_type, std::optional<double> fillet_radius = std::nullopt, bool rebuild = true);

} // namespace aerotelic
