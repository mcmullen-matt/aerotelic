#pragma once

#include "CoreMinimal.h"

// Analysis -> Structural (Kratos FEA): pick a lattice and a model of
// it -- its filled bodies' surfaces as shells of the skin thickness over
// tetrahedra carrying the lattice's homogenised stiffness (the default,
// sized for a desktop), or the skin / core tetrahedra at full resolution
// -- choose the support plane and the loads (an inertial load factor, a
// uniform surface pressure, a point force) and solve on KratosMultiphysics
// (Entity/AerotelicStructuralFea.h). The skin is made of each body's
// material, the lattice of the material set on it in the Scene tree's
// Material Properties: the stuff its struts are printed in for the
// homogenised model, an effective material for the full-resolution one.
// Modeless, tabbed. Set-up holds the choices and the results; Build
// Model meshes the bodies and derives what the solver will be handed
// (Entity/AerotelicFeaPreprocess.h) for the three pre-processor tabs --
// Mesh (the wetted faces by region, with their edges), Loads (the
// pressure, the point force and the inertial load as arrows), Support
// (a marker on every node the support fixes, and the plane) -- each an
// engine-rendered view (Widgets/SceneView.h) with the main view's
// orbit / pan / zoom; Solve builds the same model and runs it. The
// homogenised properties, the peaks, the margins and the force balance
// appear in the dialog, the displacement and stress fields go to the VTK
// file named; no post-processing view yet. Status receives the status-bar
// line. -AerotelicFeaSmoke builds the model at once, screenshots the
// three pre-processor tabs to Saved/Screenshots and quits.
void ShowAerotelicStructuralFeaDialog(TFunction<void(const FText&)> Status);
