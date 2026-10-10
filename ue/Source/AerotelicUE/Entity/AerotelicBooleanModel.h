#pragma once

#include "CoreMinimal.h"
#include <string>

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
// operands' placed shapes folded left to right with the core's
// BooleanOperation ((A op B) op C), as feature_tree.py's fold_boolean
// builds its chain. Cut is first minus the rest; fuse and common are
// associative. Built like any other entity model, so the result is a
// normal scene item: bodies, colours, visibility, section, styles.
class FAerotelicBooleanModel : public aerotelic::ISolidModel
{
public:
	// "fuse" | "cut" | "common".
	std::string Operation = "fuse";
	TArray<FAerotelicBooleanOperand> Operands;

	// Throws std::runtime_error when an operand is gone or OCCT fails
	// ("Boolean cut failed"), like the Python. A boolean over lofted
	// bodies can take tens of seconds, so the result of one build is kept
	// for the next call and consumed by it (EntityShapeFeature::OfferSolid):
	// the document validates an operation by building it, and the actor's
	// first mesh build then takes that shape instead of computing it again.
	virtual TopoDS_Shape BuildSolid() const override;

	// True if this result depends on Entity, directly or through another
	// boolean result.
	bool DependsOn(const AAerotelicEntityActor* Entity) const;

private:
	mutable TopoDS_Shape Offered;
};
