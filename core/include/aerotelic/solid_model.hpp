#pragma once

#include <vector>

#include <TopoDS_Shape.hxx>

#include "aerotelic/mesh.hpp"

namespace aerotelic {

// Common interface every entity model (PrimitiveModel, WingModel, ...)
// implements so FeatureTree's ComponentSourceFeature/EntityShapeFeature
// can hold one generically -- the C++ equivalent of core/feature_tree.py's
// duck-typed `model.build_solid()`.
class ISolidModel {
public:
	virtual ~ISolidModel() = default;
	virtual TopoDS_Shape BuildSolid() const = 0;

	// The model's bodies as closed triangle meshes built directly from its
	// section curves (mesh.hpp), within `tolerance` (model units) of the
	// surface, without OCCT -- what the viewport, STL export and the
	// lattice generator use. One entry per body, named as the viewport's
	// roles ("Main"; "Spinner", "Blade 1", ...). Empty means the model has
	// no mesh builder (or cannot use one in its current state, e.g. a
	// nacelle with chevrons, which needs a boolean cut) and the caller
	// must tessellate BuildSolid() instead.
	virtual std::vector<NamedMesh> BuildMeshes(double tolerance) const { (void)tolerance; return {}; }
};

} // namespace aerotelic
