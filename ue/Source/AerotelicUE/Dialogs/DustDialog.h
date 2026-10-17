#pragma once

#include "CoreMinimal.h"

// Analysis -> DUST (Vortex Particle Method)...: writes a complete DUST
// case for the chosen bodies. The same two-list body picker the Boolean,
// export and Potential Flow dialogs use answers "which geometry", and
// the rest of the dialog configures the parts of DUST this interface
// touches: the free stream, the time stepping, the wake's particles and
// the octree the fast multipole method needs.
//
// The dialog writes input files; it does not run DUST. Its Help tab
// gives the three commands that do (core/include/aerotelic/dust_case.hpp
// explains why it stops there). Modal, like the other analysis dialogs;
// Status receives the status-bar line.
void ShowAerotelicDustDialog(TFunction<void(const FText&)> Status);
