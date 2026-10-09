// Self-contained checks for the C++ FeatureTree port (core/feature_tree.py).
// No Python fixtures: this is pure graph-dirty-propagation logic plus OCCT
// boolean/mirror/fillet operations, validated against real mass properties
// (GProp) and internal consistency (volume-preserving mirror, additive
// volume for non-overlapping fuses, etc.) rather than a Python cross-check.

#include <iostream>
#include <string>

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include "aerotelic/feature_tree.hpp"
#include "aerotelic/primitive_model.hpp"

using namespace aerotelic;

namespace {

int g_pass = 0, g_fail = 0;

void Check(bool cond, const std::string& msg) {
	if (cond) { g_pass++; return; }
	g_fail++;
	std::cout << "FAIL: " << msg << "\n";
}

void CheckClose(double a, double b, double tol, const std::string& msg) {
	Check(std::abs(a - b) <= tol, msg + " (" + std::to_string(a) + " vs " + std::to_string(b) + ")");
}

double Volume(const TopoDS_Shape& s) {
	GProp_GProps props;
	BRepGProp::VolumeProperties(s, props);
	return props.Mass();
}

PrimitiveModel MakeBox(double lx, double ly, double lz) {
	PrimitiveModel p;
	p.shape = PrimitiveShape::Box;
	p.size = {lx, ly, lz};
	return p;
}

void CheckDirtyPropagationAndFuse() {
	PrimitiveModel box1 = MakeBox(2, 2, 2);  // volume 8
	PrimitiveModel box2 = MakeBox(1, 1, 1);  // volume 1
	FeatureTree tree;

	auto* src1 = tree.Add(std::make_unique<ComponentSourceFeature>("box1_src", &box1, "box1"));
	auto* src2 = tree.Add(std::make_unique<ComponentSourceFeature>("box2_src", &box2, "box2"));
	// Move box2 far away so fuse is non-overlapping -- a clean additive-volume check.
	auto* placed2 = tree.Add(std::make_unique<PlacementFeature>("box2_placed", src2,
		Placement{{10.0, 0.0, 0.0}, {0, 0, 1}, 0.0}));
	auto* fused = tree.Add(std::make_unique<BooleanFeature>("fused", src1, placed2, "fuse"));

	Check(src1->dirty && src2->dirty && placed2->dirty && fused->dirty, "all nodes start dirty");
	TopoDS_Shape result = fused->Rebuild();
	Check(!fused->dirty && !src1->dirty && !placed2->dirty, "rebuild clears dirty along the chain");
	CheckClose(Volume(result), 8.0 + 1.0, 1e-9, "non-overlapping fuse volume is additive");

	// Mark the root dirty; propagation should reach the terminal without a rebuild.
	src1->MarkDirty();
	Check(fused->dirty && placed2->dirty == false && src2->dirty == false,
		"mark_dirty propagates downstream only (not to unrelated siblings)");
	fused->Rebuild();
	Check(!fused->dirty, "rebuild after mark_dirty clears it again");
}

void CheckMirrorPreservesVolume() {
	PrimitiveModel box = MakeBox(2, 3, 4);
	FeatureTree tree;
	auto* src = tree.Add(std::make_unique<ComponentSourceFeature>("box_src", &box, "box"));
	auto* placed = tree.Add(std::make_unique<PlacementFeature>("box_placed", src,
		Placement{{0.0, 5.0, 0.0}, {0, 0, 1}, 0.0}));
	auto* mirrored = tree.Add(std::make_unique<MirrorFeature>("box_mirror", placed));

	TopoDS_Shape original = placed->Rebuild();
	TopoDS_Shape mirror = mirrored->Rebuild();
	CheckClose(Volume(original), Volume(mirror), 1e-9, "mirror preserves volume");

	GProp_GProps p1, p2;
	BRepGProp::VolumeProperties(original, p1);
	BRepGProp::VolumeProperties(mirror, p2);
	CheckClose(p1.CentreOfMass().Y(), 5.0, 1e-9, "original centroid Y is +5");
	CheckClose(p2.CentreOfMass().Y(), -5.0, 1e-9, "mirrored centroid Y is -5");
}

void CheckFilletAppliedFlag() {
	// Two boxes that genuinely overlap -> fillet has section edges to round.
	PrimitiveModel box1 = MakeBox(4, 4, 4);
	PrimitiveModel box2 = MakeBox(2, 6, 2);
	FeatureTree tree;
	auto* src1 = tree.Add(std::make_unique<ComponentSourceFeature>("b1", &box1, "b1"));
	auto* src2 = tree.Add(std::make_unique<ComponentSourceFeature>("b2", &box2, "b2"));
	auto* fused = tree.Add(std::make_unique<BooleanFeature>("fuse_overlap", src1, src2, "fuse"));
	fused->Rebuild();
	auto filleted_owned = std::make_unique<FilletFeature>("filleted", static_cast<BooleanFeature*>(fused), 0.3);
	FilletFeature* filleted = filleted_owned.get();
	tree.Add(std::move(filleted_owned));
	filleted->Rebuild();
	Check(filleted->applied, "fillet applies when the boolean actually created a seam");

	// Two boxes far apart -> fuse has no section edges -> fillet is a no-op.
	PrimitiveModel box3 = MakeBox(1, 1, 1);
	PrimitiveModel box4 = MakeBox(1, 1, 1);
	FeatureTree tree2;
	auto* s3 = tree2.Add(std::make_unique<ComponentSourceFeature>("b3", &box3, "b3"));
	auto* s4raw = tree2.Add(std::make_unique<ComponentSourceFeature>("b4", &box4, "b4"));
	auto* placed4 = tree2.Add(std::make_unique<PlacementFeature>("b4_placed", s4raw, Placement{{20, 0, 0}, {0, 0, 1}, 0.0}));
	auto* fused2 = tree2.Add(std::make_unique<BooleanFeature>("fuse_apart", s3, placed4, "fuse"));
	fused2->Rebuild();
	auto filleted2_owned = std::make_unique<FilletFeature>("filleted2", static_cast<BooleanFeature*>(fused2), 0.1);
	FilletFeature* filleted2 = filleted2_owned.get();
	tree2.Add(std::move(filleted2_owned));
	filleted2->Rebuild();
	Check(!filleted2->applied, "fillet is a documented no-op when there's no seam");
}

void CheckTreeBookkeeping() {
	PrimitiveModel box1 = MakeBox(1, 1, 1), box2 = MakeBox(1, 1, 1);
	FeatureTree tree;
	auto* s1 = tree.Add(std::make_unique<ComponentSourceFeature>("s1", &box1, "ref1"));
	auto* s2 = tree.Add(std::make_unique<ComponentSourceFeature>("s2", &box2, "ref2"));
	auto* placed = tree.Add(std::make_unique<PlacementFeature>("placed", s1, Placement{{1, 0, 0}, {0, 0, 1}, 0.0}));
	auto* fused = tree.Add(std::make_unique<BooleanFeature>("fused", placed, s2, "fuse"));

	Check(tree.size() == 4, "size() counts all added nodes");
	Check(tree.Contains("fused") && !tree.Contains("nope"), "Contains");
	Check(tree.order() == std::vector<std::string>{"s1", "s2", "placed", "fused"}, "order() is build order");

	auto [lo, hi] = tree.AllowedIndexRange("placed");
	Check(lo == 1 && hi == 2, "AllowedIndexRange bounded by inputs (s1 at 0) and dependents (fused at 3)");
	bool threw = false;
	try { tree.Move("placed", 0); } catch (const std::invalid_argument&) { threw = true; }
	Check(threw, "Move before an input is rejected");
	tree.Move("s2", 2); // legal: s2 has no inputs, fused (its only dependent) is after
	Check(tree.order()[2] == "s2", "legal Move takes effect");

	fused->Rebuild();
	Check(!fused->dirty, "sanity rebuild before cascading remove");
	tree.Remove("placed"); // should cascade to "fused" too
	Check(!tree.Contains("placed") && !tree.Contains("fused"), "Remove cascades downstream");
	Check(tree.Contains("s1") && tree.Contains("s2"), "Remove doesn't touch unrelated nodes");

	tree.MarkDirtyByRef("ref2"); // s2 still present
	Check(tree.Get("s2")->dirty, "MarkDirtyByRef reaches the matching ref");
	tree.RemoveByRef("ref1");
	Check(!tree.Contains("s1"), "RemoveByRef removes the matching ref's node");
}

void CheckFoldBooleanAndSerializeRoundTrip() {
	PrimitiveModel a = MakeBox(1, 1, 1), b = MakeBox(1, 1, 1), c = MakeBox(1, 1, 1);
	FeatureTree tree;
	auto* sa = tree.Add(std::make_unique<ComponentSourceFeature>("a", &a, "a"));
	auto* sb = tree.Add(std::make_unique<ComponentSourceFeature>("b", &b, "b"));
	auto* sc = tree.Add(std::make_unique<ComponentSourceFeature>("c", &c, "c"));
	auto* pb = tree.Add(std::make_unique<PlacementFeature>("pb", sb, Placement{{5, 0, 0}, {0, 0, 1}, 0.0}));
	auto* pc = tree.Add(std::make_unique<PlacementFeature>("pc", sc, Placement{{10, 0, 0}, {0, 0, 1}, 0.0}));

	Feature* terminal = FoldBoolean(tree, "combo", {sa, pb, pc}, "fuse");
	CheckClose(Volume(terminal->result), 3.0, 1e-9, "FoldBoolean over 3 non-overlapping boxes sums volumes");
	Check(tree.Get("combo_boolean1")->internal, "FoldBoolean marks intermediate nodes internal");
	Check(!terminal->internal, "FoldBoolean leaves the terminal non-internal");

	tree.AddResult("combo_result", "Combo", terminal);
	FeatureTreeSpec spec = tree.Serialize();
	Check(spec.features.size() == tree.size(), "Serialize captures every node");
	Check(spec.results.size() == 1 && spec.results[0].starboard == terminal->name, "Serialize captures results");

	FeatureTreeResolver resolver;
	resolver.resolve_component = [&](const std::string& ref) -> ISolidModel* {
		if (ref == "a") return &a;
		if (ref == "b") return &b;
		if (ref == "c") return &c;
		throw std::invalid_argument("unknown ref");
	};
	FeatureTree reloaded(spec, resolver);
	Check(reloaded.size() == tree.size(), "round-tripped tree has the same node count");
	Feature* reloaded_terminal = reloaded.Get(terminal->name);
	Check(reloaded_terminal != nullptr, "round-tripped terminal exists by name");
	reloaded_terminal->Rebuild();
	CheckClose(Volume(reloaded_terminal->result), 3.0, 1e-9, "round-tripped tree rebuilds to the same volume");
}

} // namespace

int main() {
	try {
		CheckDirtyPropagationAndFuse();
		CheckMirrorPreservesVolume();
		CheckFilletAppliedFlag();
		CheckTreeBookkeeping();
		CheckFoldBooleanAndSerializeRoundTrip();
	} catch (const std::exception& e) {
		std::cout << "UNEXPECTED EXCEPTION: " << e.what() << "\n";
		return 1;
	}
	std::cout << "\nRESULT: " << g_pass << " passed, " << g_fail << " failed\n";
	return g_fail == 0 ? 0 : 1;
}
