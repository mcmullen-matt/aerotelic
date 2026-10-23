#pragma once

#include "CoreMinimal.h"

// Analysis -> Structural (Kratos FEA)...: pick a lattice (its filled
// bodies' skin / core tetrahedra are the model: the skin made of each
// body's material, the core of the lattice's, from Material Properties
// in the Scene tree), choose the support plane and the loads -- an
// inertial load factor, a uniform surface pressure, a point force -- and
// solve on KratosMultiphysics (Entity/AerotelicStructuralFea.h). Modal;
// the solve runs in place; the peaks, the margins and the force balance
// appear in the dialog, the displacement and von Mises fields go to the
// VTK file named. Status receives the status-bar line.
void ShowAerotelicStructuralFeaDialog(TFunction<void(const FText&)> Status);
