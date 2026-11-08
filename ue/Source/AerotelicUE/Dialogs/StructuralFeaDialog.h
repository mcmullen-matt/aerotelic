#pragma once

#include "CoreMinimal.h"

// Analysis -> Structural (Kratos FEA): pick bodies, choose the support
// planes (or patches of them) and the loads (an inertial load factor, a
// uniform surface pressure, point forces) and solve on KratosMultiphysics
// (Entity/AerotelicStructuralFea.h). The model is SHELLS ONLY (2026-11-08:
// the tetrahedral solids, the homogenised lattice cores and the
// full-resolution skin / core mesh are gone): each body's own grid as
// shells of its skin thickness and material, and each of its plane
// networks as unloaded rib shells inside it, their rims on the skin's
// nodes (Entity/AerotelicFeaModelBuilder.h). A body with no skin
// thickness cannot be modelled and the Set-up tab says so.
// Modeless, tabbed. Set-up holds the choices and the results; Build
// Model meshes the ribs and derives what the solver will be handed
// (Entity/AerotelicFeaPreprocess.h) for the three pre-processor tabs --
// Mesh (the wetted faces by region, with their edges, the rib shells
// inside), Loads (the pressure, the point forces and the inertial load as
// arrows), Support (a marker on every node the supports fix, each plane
// as a frame with draggable patch handles) -- each an engine-rendered view
// (Widgets/SceneView.h) with the main view's orbit / pan / zoom; Solve
// builds the same model and runs it on a worker thread, with Cancel. The
// peaks, the margins, the force balance and the natural frequencies
// appear in the dialog, the displacement and stress fields go to the VTK
// file named; no post-processing view yet. Status receives the status-bar
// line. -AerotelicFeaSmoke builds the model at once, screenshots the
// three pre-processor tabs to Saved/Screenshots and quits.
void ShowAerotelicStructuralFeaDialog(TFunction<void(const FText&)> Status);
