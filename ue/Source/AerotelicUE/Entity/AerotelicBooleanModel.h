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

// The model of a boolean result (Geometry -> Boolean Operation...): the
// operands' placed MESHES folded left to right through the core's mesh
// boolean (mesh_boolean.hpp: triangle-triangle intersections on exact
// predicates, the cut triangles re-triangulated in their planes, pieces
// classified by the other body's distance field), (A op B) op C as
// feature_tree.py's fold_boolean built its chain. Cut is first minus the
// rest; fuse and common are associative. An operand with several bodies
// (a propeller) is fused into one first. Built like any other entity
// model, so the result is a normal scene item: body, colour, visibility,
// section, styles, STL export, a lattice target, a tetrahedral mesh.
//
// No OpenCascade: a boolean result has no B-rep solid, so it is not
// exported to STEP (export the operands and combine them downstream).
class FAerotelicBooleanModel : public aerotelic::ISolidModel
{
public:
	// "fuse" | "cut" | "common".
	std::string Operation = "fuse";
	TArray<FAerotelicBooleanOperand> Operands;

	// The tolerance the document validates a new operation at (the lattice
	// and export tolerance).
	static constexpr double kValidationTolerance = 2e-3;

	// Throws std::runtime_error: a boolean result is a mesh, not a solid.
	virtual TopoDS_Shape BuildSolid() const override;

	// The result mesh at the given tolerance (the operands are rebuilt at
	// it). Throws std::runtime_error, naming the operands, when an operand
	// is gone or has no geometry, is not a closed outward surface, when the
	// mesh boolean refuses a degenerate contact (two triangles overlapping in
	// one plane, a vertex of one body exactly on a face of the other), or
	// when the result would be empty or does not close.
	virtual std::vector<aerotelic::NamedMesh> BuildMeshes(double Tolerance) const override;

	// True if this result depends on Entity, directly or through another
	// boolean result.
	bool DependsOn(const AAerotelicEntityActor* Entity) const;
};
