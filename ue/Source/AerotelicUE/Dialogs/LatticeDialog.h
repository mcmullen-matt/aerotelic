#pragma once

#include "CoreMinimal.h"

class AAerotelicEntityActor;

// Lattice -> Generate Lattice... (and Edit on a lattice row): the dialog
// that picks the lattice family and type (TPMS with its phase, strut-based
// or auxetic -- OpenLattice's library, see core lattice.hpp), the
// parameters that control its generation (unit cell size, volume fraction
// or wall thickness, voxels per cell, solid skin), the bodies it fills
// (every non-primitive body of the scene by default, mirrored halves and
// boolean results included) and the reserved volumes it leaves alone
// (every primitive by default -- WingDesigner's primitives exist to mark
// payload bays and the like). OK generates the lattice as a new scene
// item (or regenerates `Existing`) and reports the result through Status.
void ShowAerotelicLatticeDialog(AAerotelicEntityActor* Existing, TFunction<void(const FText&)> Status);
