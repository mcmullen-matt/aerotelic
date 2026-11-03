#pragma once

#include "CoreMinimal.h"

class AAerotelicEntityActor;

struct FAerotelicBooleanOperand;

// "Generate Lattice" on a body's row in the Scene tree (and Edit on a
// lattice row): the dialog that picks the lattice family and type (TPMS
// with its phase, strut-based or auxetic -- OpenLattice's library, see
// core lattice.hpp), the parameters that control its generation (unit
// cell size, volume fraction or wall thickness, voxels per cell), the
// bodies it fills -- the body picked (with its mirrored half) to start
// with, any others addable -- and the reserved volumes it leaves alone
// (every primitive by default -- WingDesigner's primitives exist to mark
// payload bays and the like). The skin thickness is each body's own
// (Material Properties). OK generates the lattice as a new scene item
// (or regenerates `Existing`) and reports the result through Status.
void ShowAerotelicLatticeDialog(AAerotelicEntityActor* Existing, TFunction<void(const FText&)> Status, const FAerotelicBooleanOperand* FillBody = nullptr);
