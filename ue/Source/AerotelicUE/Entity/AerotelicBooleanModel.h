#pragma once

#include "CoreMinimal.h"
#include <string>
#include <vector>

#include "aerotelic/mesh.hpp"
#include "aerotelic/solid_model.hpp"

class AAerotelicEntityActor;

// One operand of a boolean operation: a scene entity's placed shape, the
// mirrored half of a mirrored entity, or an earlier boolean result (also
// an entity actor, of kind Boolean). Held by pointer, so renames need no
// bookkeeping; saved by name.
struct FAerotelicBooleanOperand
{
	TWeakObjectPtr<AAerotelicEntityActor> Entity;
	bool bMirror = false;
};

// The model of a boolean result (Geometry -> Boolean Operation): the
// operands' placed MESHES folded left to right through the core's mesh
// boolean (mesh_boolean.hpp: triangle-triangle intersections on exact
// predicates, the cut triangles re-triangulated in their planes, pieces
// classified by the other body's distance field), (A op B) op C as
// feature_tree.py's fold_boolean built its chain. Cut is first minus the
// rest; fuse and common are associative. An operand with several bodies
// (a propeller) is fused into one first. Built like any other entity
// model, so the result is a normal scene item: body, colour, visibility,
// section, styles, STL export.
//
// A result OWNS NO STRUCTURE (2026-11-20, the owner's rule): every face
// of it is a piece of one operand body's surface and keeps THAT body's
// skin -- material and thickness -- fused to a different skin or not; the
// ribs, spars and frames inside are the operands' generated layers taken
// over (since 2026-11-10). To change what a result is made of, change the
// operand. Each body's NamedMesh carries the provenance (face_source /
// sources, exact: the mesh boolean cuts triangles, it never makes one out
// of two bodies), and the mass report, the structural model builder and
// the Scene tree read the operands' structures through it
// (AerotelicSkinSources.h). A cut's pocket wall is the cutter's surface
// and keeps the cutter's skin, by the same rule.
//
// No OpenCascade: a boolean result has no B-rep solid, so it is not
// exported to STEP (export the operands and combine them downstream).
class FAerotelicBooleanModel : public aerotelic::ISolidModel
{
public:
	// "fuse" | "cut" | "common".
	std::string Operation = "fuse";
	TArray<FAerotelicBooleanOperand> Operands;

	// The tolerance the document validates a new operation at (the export
	// tolerance).
	static constexpr double kValidationTolerance = 2e-3;

	// Throws std::runtime_error: a boolean result is a mesh, not a solid.
	virtual TopoDS_Shape BuildSolid() const override;

	// The result at the given tolerance (the operands are rebuilt at it),
	// as its solid bodies: one shell is "Main"; a result of several shells
	// that touch nothing of one another (a flap fused alongside its wing
	// stays a body of its own) is "Body 1" (the largest) onwards.
	// Throws std::runtime_error, naming the operands, when an operand
	// is gone or has no geometry, is not a closed outward surface, when the
	// mesh boolean refuses a degenerate contact (two triangles overlapping in
	// one plane, a vertex of one body exactly on a face of the other), or
	// when the result would be empty or does not close.
	virtual std::vector<aerotelic::NamedMesh> BuildMeshes(double Tolerance) const override;

	// True if this result depends on Entity, directly or through another
	// boolean result.
	bool DependsOn(const AAerotelicEntityActor* Entity) const;
};
