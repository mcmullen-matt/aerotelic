#pragma once

#include <vector>

#include <TopoDS_Shape.hxx>

#include "aerotelic/mesh.hpp"
#include "aerotelic/structure_plan.hpp"

namespace aerotelic {

// One body of a model as the section function its mesh is lofted from
// (LoftSource), self-contained: the closure owns or points at the model,
// which must outlive it. What the panel method panels into a corner-point
// network (wing_panels.hpp), since a loft IS a structured grid.
struct NamedLoft {
	std::string name;
	LoftSource source;
	// The surface is made of planes meeting at edges (a box): panel it with
	// flat panels, since a curvature fit across a sharp edge is nonsense.
	bool flat = false;
};

// Common interface every entity model (PrimitiveModel, WingModel, ...)
// implements so FeatureTree's ComponentSourceFeature/EntityShapeFeature
// can hold one generically -- the C++ equivalent of core/feature_tree.py's
// duck-typed `model.build_solid()`.
class ISolidModel {
public:
	virtual ~ISolidModel() = default;
	virtual TopoDS_Shape BuildSolid() const = 0;

	// The density of this body's grid (mesh.hpp MeshSpec): set on the
	// entity's Mesh tab, saved with the entity, read by BuildMeshes (every
	// implementation goes through mesh.Apply over its own defaults) and by
	// the panel method for its network counts. Not part of the model's
	// parameterisation spec: the document copies it across clone / apply /
	// load / save.
	MeshSpec mesh;

	// The model's bodies as closed triangle meshes built directly from its
	// section curves (mesh.hpp), within `tolerance` (model units) of the
	// surface, without OCCT -- what the viewport, STL export and the
	// mass report use. One entry per body, named as the viewport's
	// roles ("Main"; "Spinner", "Blade 1", ...). Empty means the model has
	// no mesh builder (or cannot use one in its current state, e.g. a
	// nacelle with chevrons, which needs a boolean cut) and the caller
	// must tessellate BuildSolid() instead.
	virtual std::vector<NamedMesh> BuildMeshes(double tolerance) const { (void)tolerance; return {}; }

	// The same bodies as lofts (see NamedLoft), for the panel method. Empty
	// where BuildMeshes() is empty, and for models whose bodies are not one
	// loft each (a propeller's blades are turned and copied).
	virtual std::vector<NamedLoft> BuildLofts() const { return {}; }

	// The internal structure the last BuildMeshes() planned on its bodies
	// (structure_plan.hpp): one entry per body with ribs or frames, spars
	// and stringers to build. Empty for a model with no structure of its
	// own (a primitive, a boolean result).
	virtual const std::vector<BodyStructurePlan>& LastStructurePlans() const {
		static const std::vector<BodyStructurePlan> none;
		return none;
	}
};

} // namespace aerotelic
