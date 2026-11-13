#pragma once

#include <vector>

#include <TopoDS_Shape.hxx>

#include "aerotelic/mesh.hpp"
#include "aerotelic/structure_plan.hpp"

namespace aerotelic {

// Where a loft's TRAILING EDGE is, in its own parameters (2026-11-13): the
// edge the panel method sheds a wake from, so a lifting body other than a
// wing -- a cowl, a fin lofted as a body, a flap -- carries a Kutta
// condition. Set by the parameterisation that knows where its trailing
// edge is (the lesson of the structure work: put the feature in the
// parameterisation, not in a post-hoc search), never guessed from the
// triangles. Two kinds:
//   VSeam    an airfoil-sectioned loft (LoftSource::closed_curve false):
//            the trailing edge is the seam v = 0 / v = 1 at every station,
//            the two ends of each open section. A blunt section (the ends
//            apart) is closed for the aero copy by the XFOIL-style blend
//            (wing_panels.hpp), never by a base panel.
//   UStation a THIN ANNULAR WALL whose two skins are the stations u and
//            mirror_u - u (LoftSource::mirror_u: a cowl parametrised out
//            along the outer skin and back along the inner), meeting at
//            the trailing edge at station u. The aero copy is the wall's
//            MEAN SURFACE from the first station (the lip) to u, a sheet
//            under the velocity condition with a ring wake at u -- the
//            thin annular wing. (The thick closed cowl under the interior-
//            potential formulation is ill-conditioned: its wall is thin
//            against its panels, Cp of -900 on 2026-11-13.) The stations
//            from u_lo to u_hi are the rim, kept for the geometry's sake.
struct LoftTrailingEdge {
	enum class Kind { None, VSeam, UStation };
	Kind kind = Kind::None;
	double u = 0.0;
	double u_lo = 0.0, u_hi = 0.0;
	bool IsSet() const { return kind != Kind::None; }
	static LoftTrailingEdge Seam() { LoftTrailingEdge t; t.kind = Kind::VSeam; return t; }
	static LoftTrailingEdge Station(double u, double u_lo, double u_hi) { LoftTrailingEdge t; t.kind = Kind::UStation; t.u = u; t.u_lo = u_lo; t.u_hi = u_hi; return t; }
};

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
	// Where the wake leaves, if anywhere (see LoftTrailingEdge).
	LoftTrailingEdge trailing_edge;
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
