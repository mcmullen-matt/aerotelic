#pragma once

#include "CoreMinimal.h"

// Analysis -> Structural (Kratos FEA)...: pick a lattice and a model of
// it -- its filled bodies' surfaces as shells of the skin thickness over
// tetrahedra carrying the lattice's homogenised stiffness (the default,
// sized for a desktop), or the skin / core tetrahedra at full resolution
// -- choose the support plane and the loads (an inertial load factor, a
// uniform surface pressure, a point force) and solve on KratosMultiphysics
// (Entity/AerotelicStructuralFea.h). The skin is made of each body's
// material, the lattice of the material set on it in the Scene tree's
// Material Properties: the stuff its struts are printed in for the
// homogenised model, an effective material for the full-resolution one.
// Modal; the solve runs in place; the homogenised properties, the peaks,
// the margins and the force balance appear in the dialog, the
// displacement and stress fields go to the VTK file named. Status
// receives the status-bar line.
void ShowAerotelicStructuralFeaDialog(TFunction<void(const FText&)> Status);
