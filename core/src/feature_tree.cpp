#include "aerotelic/feature_tree.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <TopoDS.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

namespace aerotelic {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

// ---------------------------------------------------------------------
// OCC geometry helpers
// ---------------------------------------------------------------------

TopoDS_Shape ApplyTranslation(const TopoDS_Shape& shape, const std::array<double, 3>& translation) {
	gp_Trsf trsf;
	trsf.SetTranslation(gp_Vec(translation[0], translation[1], translation[2]));
	return BRepBuilderAPI_Transform(shape, trsf, true).Shape();
}

TopoDS_Shape ApplyRotation(const TopoDS_Shape& shape, const std::array<double, 3>& axis, double angle_deg) {
	gp_Ax1 rot_axis(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(gp_Vec(axis[0], axis[1], axis[2])));
	gp_Trsf trsf;
	trsf.SetRotation(rot_axis, angle_deg * kPi / 180.0);
	return BRepBuilderAPI_Transform(shape, trsf, true).Shape();
}

TopoDS_Shape Place(const TopoDS_Shape& shape_in, const std::array<double, 3>& translation,
	const std::array<double, 3>& rotation_axis, double rotation_deg) {
	TopoDS_Shape shape = shape_in;
	// A zero-length axis is no rotation, whatever the angle says: gp_Dir
	// throws on a null vector, and a project file (or a half-typed axis in
	// the Transform dialog) can hold one.
	const double axis_len_sq = rotation_axis[0] * rotation_axis[0] + rotation_axis[1] * rotation_axis[1]
		+ rotation_axis[2] * rotation_axis[2];
	if (rotation_deg != 0.0 && axis_len_sq > 0.0) shape = ApplyRotation(shape, rotation_axis, rotation_deg);
	double tx = translation[0], ty = translation[1], tz = translation[2];
	if (tx != 0.0 || ty != 0.0 || tz != 0.0) shape = ApplyTranslation(shape, {tx, ty, tz});
	return shape;
}

// ---------------------------------------------------------------------
// Euler rotations
// ---------------------------------------------------------------------

const char* EulerOrderName(EulerOrder o) {
	switch (o) {
		case EulerOrder::XYZ: return "xyz";
		case EulerOrder::XZY: return "xzy";
		case EulerOrder::YXZ: return "yxz";
		case EulerOrder::YZX: return "yzx";
		case EulerOrder::ZXY: return "zxy";
		case EulerOrder::ZYX: return "zyx";
	}
	return "zyx";
}

EulerOrder ParseEulerOrder(const std::string& s) {
	std::string l;
	for (char c : s) l += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	if (l == "xyz") return EulerOrder::XYZ;
	if (l == "xzy") return EulerOrder::XZY;
	if (l == "yxz") return EulerOrder::YXZ;
	if (l == "yzx") return EulerOrder::YZX;
	if (l == "zxy") return EulerOrder::ZXY;
	if (l == "zyx") return EulerOrder::ZYX;
	throw std::invalid_argument("unknown Euler order '" + s + "' (expected xyz, xzy, yxz, yzx, zxy or zyx)");
}

std::array<int, 3> EulerOrderAxes(EulerOrder o) {
	switch (o) {
		case EulerOrder::XYZ: return {0, 1, 2};
		case EulerOrder::XZY: return {0, 2, 1};
		case EulerOrder::YXZ: return {1, 0, 2};
		case EulerOrder::YZX: return {1, 2, 0};
		case EulerOrder::ZXY: return {2, 0, 1};
		case EulerOrder::ZYX: return {2, 1, 0};
	}
	return {2, 1, 0};
}

Mat3 RotationMatrix(const std::array<double, 3>& axis, double angle_deg) {
	const double len = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
	if (!(len > 0.0)) return {1, 0, 0, 0, 1, 0, 0, 0, 1};
	const double x = axis[0] / len, y = axis[1] / len, z = axis[2] / len;
	const double a = angle_deg * kPi / 180.0;
	const double c = std::cos(a), s = std::sin(a), t = 1.0 - c;
	return {
		c + t * x * x,     t * x * y - s * z, t * x * z + s * y,
		t * x * y + s * z, c + t * y * y,     t * y * z - s * x,
		t * x * z - s * y, t * y * z + s * x, c + t * z * z,
	};
}

Mat3 MatrixMultiply(const Mat3& a, const Mat3& b) {
	Mat3 m{};
	for (int r = 0; r < 3; ++r) {
		for (int c = 0; c < 3; ++c) {
			m[3 * r + c] = a[3 * r] * b[c] + a[3 * r + 1] * b[3 + c] + a[3 * r + 2] * b[6 + c];
		}
	}
	return m;
}

Mat3 EulerMatrix(const EulerRotation& e) {
	const std::array<int, 3> axes = EulerOrderAxes(e.order);
	Mat3 r[3];
	for (int n = 0; n < 3; ++n) {
		std::array<double, 3> unit{0.0, 0.0, 0.0};
		unit[axes[n]] = 1.0;
		r[n] = RotationMatrix(unit, e.angles_deg[n]);
	}
	// Intrinsic: each turn is about the axes the earlier turns already
	// moved, so a later rotation post-multiplies -- M = R0 R1 R2.
	// Extrinsic: every turn is about the fixed world axes, so a later one
	// pre-multiplies -- M = R2 R1 R0. (Which is why intrinsic XYZ and
	// extrinsic ZYX are the same rotation, read in opposite order.)
	if (e.intrinsic) return MatrixMultiply(MatrixMultiply(r[0], r[1]), r[2]);
	return MatrixMultiply(MatrixMultiply(r[2], r[1]), r[0]);
}

std::pair<std::array<double, 3>, double> AxisAngleFromMatrix(const Mat3& m) {
	// Shepperd: take the quaternion component the trace says is largest,
	// which keeps the division well-conditioned at every angle (the naive
	// acos((tr - 1) / 2) loses all axis precision near 0 and 180 degrees).
	const double trace = m[0] + m[4] + m[8];
	double qw, qx, qy, qz;
	if (trace > 0.0) {
		const double s = std::sqrt(trace + 1.0) * 2.0;
		qw = 0.25 * s; qx = (m[7] - m[5]) / s; qy = (m[2] - m[6]) / s; qz = (m[3] - m[1]) / s;
	} else if (m[0] > m[4] && m[0] > m[8]) {
		const double s = std::sqrt(1.0 + m[0] - m[4] - m[8]) * 2.0;
		qw = (m[7] - m[5]) / s; qx = 0.25 * s; qy = (m[1] + m[3]) / s; qz = (m[2] + m[6]) / s;
	} else if (m[4] > m[8]) {
		const double s = std::sqrt(1.0 + m[4] - m[0] - m[8]) * 2.0;
		qw = (m[2] - m[6]) / s; qx = (m[1] + m[3]) / s; qy = 0.25 * s; qz = (m[5] + m[7]) / s;
	} else {
		const double s = std::sqrt(1.0 + m[8] - m[0] - m[4]) * 2.0;
		qw = (m[3] - m[1]) / s; qx = (m[2] + m[6]) / s; qy = (m[5] + m[7]) / s; qz = 0.25 * s;
	}
	// q and -q are the same rotation; the one with qw >= 0 is the short way
	// round, so the angle lands in [0, 180] rather than [180, 360).
	if (qw < 0.0) { qw = -qw; qx = -qx; qy = -qy; qz = -qz; }
	const double vec_len = std::sqrt(qx * qx + qy * qy + qz * qz);
	if (!(vec_len > 1e-15)) {
		return std::make_pair(std::array<double, 3>{0.0, 0.0, 1.0}, 0.0);
	}
	const std::array<double, 3> axis{qx / vec_len, qy / vec_len, qz / vec_len};
	return std::make_pair(axis, 2.0 * std::atan2(vec_len, qw) * 180.0 / kPi);
}

std::pair<std::array<double, 3>, double> AxisAngleFromEuler(const EulerRotation& e) {
	return AxisAngleFromMatrix(EulerMatrix(e));
}

std::array<double, 3> EulerAnglesFromMatrix(const Mat3& m, EulerOrder order, bool intrinsic) {
	std::array<int, 3> axes = EulerOrderAxes(order);
	// An extrinsic sequence is the reversed intrinsic one, so solve the
	// intrinsic form of the reversed order and reverse the answer back.
	if (!intrinsic) std::swap(axes[0], axes[2]);
	const int i = axes[0], j = axes[1], k = axes[2];
	// +1 when (i, j, k) is a cyclic permutation of (0, 1, 2): the sign the
	// closed form below carries for M[i][k] = sin(middle).
	const double eps = ((j == (i + 1) % 3) ? 1.0 : -1.0);
	const double sin_mid = std::max(-1.0, std::min(1.0, eps * m[3 * i + k]));
	std::array<double, 3> out{};
	out[1] = std::asin(sin_mid) * 180.0 / kPi;
	if (std::abs(sin_mid) < 1.0 - 1e-12) {
		out[0] = std::atan2(-eps * m[3 * j + k], m[3 * k + k]) * 180.0 / kPi;
		out[2] = std::atan2(-eps * m[3 * i + j], m[3 * i + i]) * 180.0 / kPi;
	} else {
		// Gimbal lock: the first and third axes coincide, so only their
		// sum is determined. Report it all on the first.
		out[0] = std::atan2(sin_mid * m[3 * j + i], m[3 * j + j]) * 180.0 / kPi;
		out[2] = 0.0;
	}
	if (!intrinsic) std::swap(out[0], out[2]);
	return out;
}

void SyncPlacementRotation(Placement& p) {
	if (!p.euler.used) return;
	const auto [axis, angle_deg] = AxisAngleFromEuler(p.euler);
	p.rotation_axis = axis;
	p.rotation_deg = angle_deg;
}

TopoDS_Shape ApplyMirror(const TopoDS_Shape& shape, const std::array<double, 3>& point, const std::array<double, 3>& normal) {
	gp_Ax2 plane(gp_Pnt(point[0], point[1], point[2]), gp_Dir(normal[0], normal[1], normal[2]));
	gp_Trsf trsf;
	trsf.SetMirror(plane);
	return BRepBuilderAPI_Transform(shape, trsf, true).Shape();
}

std::unique_ptr<BRepAlgoAPI_BooleanOperation> BooleanOperation(const std::string& op_type, const TopoDS_Shape& body1, const TopoDS_Shape& body2) {
	std::unique_ptr<BRepAlgoAPI_BooleanOperation> op;
	if (op_type == "fuse") op = std::make_unique<BRepAlgoAPI_Fuse>(body1, body2);
	else if (op_type == "cut") op = std::make_unique<BRepAlgoAPI_Cut>(body1, body2);
	else if (op_type == "common") op = std::make_unique<BRepAlgoAPI_Common>(body1, body2);
	else throw std::invalid_argument("Unknown boolean operation '" + op_type + "'");
	op->SetToFillHistory(true);
	op->Build();
	if (!op->IsDone()) throw std::runtime_error("Boolean " + op_type + " failed");
	return op;
}

std::pair<TopoDS_Shape, bool> FilletSeam(BRepAlgoAPI_BooleanOperation& bool_op, double fillet_radius) {
	TopoDS_Shape shape = bool_op.Shape();
	const TopTools_ListOfShape& section_edges = bool_op.SectionEdges();
	if (section_edges.IsEmpty()) return {shape, false};

	BRepFilletAPI_MakeFillet mk_fillet(shape);
	for (TopTools_ListIteratorOfListOfShape it(section_edges); it.More(); it.Next()) {
		mk_fillet.Add(fillet_radius, TopoDS::Edge(it.Value()));
	}
	mk_fillet.Build();
	if (!mk_fillet.IsDone()) return {shape, false};
	return {mk_fillet.Shape(), true};
}

const char* FeatureKindName(FeatureKind k) {
	switch (k) {
		case FeatureKind::ComponentSource: return "component_source";
		case FeatureKind::EntityShape: return "entity_shape";
		case FeatureKind::Placement: return "placement";
		case FeatureKind::Mirror: return "mirror";
		case FeatureKind::Boolean: return "boolean";
		case FeatureKind::Fillet: return "fillet";
	}
	return "";
}

// ---------------------------------------------------------------------
// Feature
// ---------------------------------------------------------------------

void Feature::DependsOn(Feature* upstream) {
	inputs_.push_back(upstream);
	upstream->dependents_.push_back(this);
}

void Feature::Detach() {
	for (Feature* u : inputs_) {
		auto& deps = u->dependents_;
		deps.erase(std::remove(deps.begin(), deps.end(), this), deps.end());
	}
}

void Feature::MarkDirty() {
	if (dirty) return;
	dirty = true;
	for (Feature* dep : dependents_) dep->MarkDirty();
}

TopoDS_Shape Feature::Rebuild() {
	if (!dirty) return result;
	for (Feature* u : inputs_) u->Rebuild();
	try {
		result = Compute();
		error = nullptr;
	} catch (...) {
		error = std::current_exception();
		throw;
	}
	dirty = false;
	return result;
}

TopoDS_Shape EntityShapeFeature::Compute() {
	std::optional<TopoDS_Shape> shape = offered_;
	offered_.reset();
	TopoDS_Shape s = shape ? *shape : model->BuildSolid();
	EntityTransform t = get_transform();
	return Place(s, {t.tx, t.ty, t.tz}, {t.ax, t.ay, t.az}, t.angle_deg);
}

// ---------------------------------------------------------------------
// FeatureTree
// ---------------------------------------------------------------------

Feature* FeatureTree::Add(std::unique_ptr<Feature> feature) {
	if (features_.count(feature->name)) {
		feature->Detach();
		throw std::invalid_argument("Feature '" + feature->name + "' already exists in this tree");
	}
	Feature* ptr = feature.get();
	order_.push_back(feature->name);
	features_[feature->name] = std::move(feature);
	return ptr;
}

Feature* FeatureTree::Get(const std::string& name) const {
	auto it = features_.find(name);
	return it == features_.end() ? nullptr : it->second.get();
}

int FeatureTree::IndexOf(const std::string& name) const {
	auto it = std::find(order_.begin(), order_.end(), name);
	if (it == order_.end()) throw std::invalid_argument("No feature named '" + name + "'");
	return static_cast<int>(it - order_.begin());
}

bool FeatureTree::IsTopological(const std::vector<std::string>& order) const {
	std::unordered_set<std::string> seen;
	for (const auto& n : order) {
		Feature* f = Get(n);
		if (!f) return false;
		for (Feature* u : f->inputs()) {
			if (!seen.count(u->name)) return false;
		}
		seen.insert(n);
	}
	return seen.size() == features_.size();
}

std::pair<int, int> FeatureTree::AllowedIndexRange(const std::string& name) const {
	Feature* f = Get(name);
	int lo = -1;
	for (Feature* u : f->inputs()) lo = std::max(lo, IndexOf(u->name));
	lo += 1;
	int hi = static_cast<int>(order_.size());
	for (Feature* d : f->dependents()) hi = std::min(hi, IndexOf(d->name));
	return {lo, hi - 1};
}

void FeatureTree::Move(const std::string& name, int index) {
	if (!features_.count(name)) throw std::invalid_argument("No feature named '" + name + "'");

	std::vector<std::string> new_order;
	for (auto& n : order_) if (n != name) new_order.push_back(n);
	index = std::max(0, std::min(index, static_cast<int>(new_order.size())));
	new_order.insert(new_order.begin() + index, name);

	if (!IsTopological(new_order)) {
		auto [lo, hi] = AllowedIndexRange(name);
		throw std::invalid_argument("Cannot move '" + name + "' to position " + std::to_string(index) +
			": it must stay after its inputs and before anything built from it (allowed " +
			std::to_string(lo) + "-" + std::to_string(hi) + ")");
	}
	order_ = new_order;
}

void FeatureTree::Remove(const std::string& name) {
	Feature* feature = Get(name);
	if (!feature) return;

	std::unordered_map<std::string, Feature*> doomed;
	std::vector<Feature*> stack{feature};
	while (!stack.empty()) {
		Feature* f = stack.back();
		stack.pop_back();
		if (doomed.count(f->name)) continue;
		doomed[f->name] = f;
		for (Feature* d : f->dependents()) stack.push_back(d);
	}

	// Detach every doomed node from its (possibly also-doomed) inputs before
	// destroying ANY of them: Detach() dereferences upstream Feature*s, which
	// must still be alive when it runs. features_ (an unordered_map) destroys
	// entries in arbitrary order, not dependency order -- unlike Python,
	// where del self._features[n] doesn't actually free the object as long as
	// a sibling doomed node's _inputs still references it. So here, nothing
	// may be erased until every doomed node's linkage is already severed.
	for (auto& [n, f] : doomed) f->Detach();

	std::unordered_set<std::string> removed;
	for (auto& [n, f] : doomed) {
		auto it = features_.find(n);
		if (it == features_.end() || it->second.get() != f) continue; // never registered (or superseded)
		order_.erase(std::remove(order_.begin(), order_.end(), n), order_.end());
		removed.insert(n);
		features_.erase(it); // destroys f
	}

	results_.erase(std::remove_if(results_.begin(), results_.end(), [&](const FeatureResult& r) {
		return removed.count(r.starboard) != 0 || (r.port && removed.count(*r.port) != 0);
	}), results_.end());
}

void FeatureTree::RemoveByRef(const std::string& ref) {
	std::vector<std::string> names;
	for (auto& n : order_) {
		auto r = Get(n)->Ref();
		if (r && *r == ref) names.push_back(n);
	}
	for (auto& n : names) Remove(n);
}

void FeatureTree::MarkDirty(const std::string& name) {
	Feature* f = Get(name);
	if (f) f->MarkDirty();
}

void FeatureTree::MarkDirtyByRef(const std::string& ref) {
	for (auto& n : order_) {
		Feature* f = Get(n);
		auto r = f->Ref();
		if (r && *r == ref) f->MarkDirty();
	}
}

std::vector<Feature*> FeatureTree::Terminals() const {
	std::vector<Feature*> out;
	for (auto& n : order_) {
		Feature* f = Get(n);
		if (f->dependents().empty()) out.push_back(f);
	}
	return out;
}

std::map<std::string, std::exception_ptr> FeatureTree::Rebuild(const std::vector<std::string>* names) {
	std::vector<Feature*> targets;
	if (!names) {
		targets = Terminals();
	} else {
		for (auto& n : *names) {
			Feature* f = Get(n);
			if (f) targets.push_back(f);
		}
	}
	std::map<std::string, std::exception_ptr> errors;
	for (Feature* f : targets) {
		try {
			f->Rebuild();
		} catch (...) {
			errors[f->name] = std::current_exception();
		}
	}
	return errors;
}

FeatureResult FeatureTree::AddResult(const std::string& name, const std::string& label, Feature* starboard, Feature* port) {
	FeatureResult r{name, label, starboard->name, port ? std::optional<std::string>(port->name) : std::nullopt};
	results_.push_back(r);
	return r;
}

FeatureTreeSpec FeatureTree::Serialize() const {
	FeatureTreeSpec spec;
	for (auto& n : order_) {
		Feature* f = Get(n);
		FeatureSpec fs;
		fs.kind = f->kind;
		fs.name = f->name;
		for (Feature* u : f->inputs()) fs.inputs.push_back(u->name);
		fs.internal = f->internal;
		switch (f->kind) {
			case FeatureKind::ComponentSource:
				fs.ref = static_cast<ComponentSourceFeature*>(f)->ref;
				break;
			case FeatureKind::EntityShape:
				fs.ref = static_cast<EntityShapeFeature*>(f)->ref;
				break;
			case FeatureKind::Placement:
				fs.placement = static_cast<PlacementFeature*>(f)->placement;
				break;
			case FeatureKind::Mirror: {
				auto* m = static_cast<MirrorFeature*>(f);
				fs.point = m->point;
				fs.normal = m->normal;
				break;
			}
			case FeatureKind::Boolean:
				fs.op_type = static_cast<BooleanFeature*>(f)->op_type;
				break;
			case FeatureKind::Fillet:
				fs.fillet_radius = static_cast<FilletFeature*>(f)->fillet_radius;
				break;
		}
		spec.features.push_back(std::move(fs));
	}
	spec.results = results_;
	return spec;
}

void FeatureTree::Load(const FeatureTreeSpec& spec, const FeatureTreeResolver& resolver) {
	for (const auto& fs : spec.features) {
		std::vector<std::string> missing;
		for (auto& n : fs.inputs) if (!features_.count(n)) missing.push_back(n);
		if (!missing.empty()) {
			std::string joined;
			for (auto& m : missing) joined += m + " ";
			throw std::invalid_argument("Feature '" + fs.name + "' is listed before its input(s) " + joined +
				"-- the saved feature order is not a valid build sequence");
		}
		std::vector<Feature*> inputs;
		for (auto& n : fs.inputs) inputs.push_back(features_[n].get());

		std::unique_ptr<Feature> feature;
		switch (fs.kind) {
			case FeatureKind::ComponentSource: {
				if (!resolver.resolve_component) throw std::invalid_argument("A resolver is required to load component_source features");
				ISolidModel* model = resolver.resolve_component(*fs.ref);
				feature = std::make_unique<ComponentSourceFeature>(fs.name, model, *fs.ref);
				break;
			}
			case FeatureKind::EntityShape: {
				if (!resolver.resolve_entity) throw std::invalid_argument("A resolver is required to load entity_shape features");
				auto resolved = resolver.resolve_entity(*fs.ref);
				feature = std::make_unique<EntityShapeFeature>(fs.name, resolved.first, resolved.second, *fs.ref);
				break;
			}
			case FeatureKind::Placement:
				feature = std::make_unique<PlacementFeature>(fs.name, inputs[0], *fs.placement);
				break;
			case FeatureKind::Mirror:
				feature = std::make_unique<MirrorFeature>(fs.name, inputs[0],
					fs.point.value_or(std::array<double, 3>{0, 0, 0}),
					fs.normal.value_or(std::array<double, 3>{0, 1, 0}));
				break;
			case FeatureKind::Boolean:
				feature = std::make_unique<BooleanFeature>(fs.name, inputs[0], inputs[1], fs.op_type.value_or("fuse"));
				break;
			case FeatureKind::Fillet:
				feature = std::make_unique<FilletFeature>(fs.name, static_cast<BooleanFeature*>(inputs[0]), *fs.fillet_radius);
				break;
		}
		feature->internal = fs.internal;
		Add(std::move(feature));
	}
	results_ = spec.results;
}

// ---------------------------------------------------------------------
// FoldBoolean
// ---------------------------------------------------------------------

Feature* FoldBoolean(FeatureTree& tree, const std::string& name, const std::vector<Feature*>& operands,
	const std::string& op_type, std::optional<double> fillet_radius, bool rebuild) {
	if (operands.size() < 2) throw std::invalid_argument("A boolean operation needs at least two operands");

	std::vector<std::string> added;
	try {
		Feature* acc = operands[0];
		for (size_t i = 1; i < operands.size(); ++i) {
			bool last = (i == operands.size() - 1);
			std::string node_name = name + "_boolean" + (last ? "" : std::to_string(i));
			Feature* node = tree.Add(std::make_unique<BooleanFeature>(node_name, acc, operands[i], op_type));
			added.push_back(node->name);
			node->internal = !last;
			acc = node;
		}

		Feature* terminal = acc;
		if (fillet_radius) {
			terminal->internal = true;
			Feature* f = tree.Add(std::make_unique<FilletFeature>(name + "_fillet", static_cast<BooleanFeature*>(acc), *fillet_radius));
			added.push_back(f->name);
			terminal = f;
		}

		if (rebuild) terminal->Rebuild();
		return terminal;
	} catch (...) {
		for (auto& n : added) tree.Remove(n);
		throw;
	}
}

} // namespace aerotelic
