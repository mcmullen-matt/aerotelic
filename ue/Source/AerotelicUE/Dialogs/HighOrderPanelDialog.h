#pragma once

#include "CoreMinimal.h"

// Analysis -> High Order Panel...: the incompressible panel method of
// Johnson, NASA CR-3079 (core panel_method.hpp), on the lofted bodies of
// the scene, chosen with the two-list picker the export dialogs use. A
// wing is its blended sections paneled round the surface (or as the mean
// camber surface) with a wake shed from the trailing edge; a fuselage,
// nacelle or primitive is a closed source network built from its loft.
// A symmetry plane at y = 0 is optional (on by default when something is
// mirrored or a wing root sits on the plane).
//
// Four tabs: the set-up (bodies, paneling, a list of angles of attack or
// of sideslip, references, the solution file), the bodies coloured by Cp
// in an engine-rendered view with a colour bar (SAerotelicSceneView and
// AAerotelicColoredMeshActor), the span loading of each wing against the
// elliptic distribution of the same lift, and the wind-axis force and
// moment coefficients against the swept angle. The solves run on a worker
// thread, one per angle. Modeless, like the design dialogs, so the world
// keeps rendering the view. Status receives the status-bar line.
//
// Independent of the Kratos route (Potential Flow (Kratos)...), which is
// left as it is.
void ShowAerotelicHighOrderPanelDialog(TFunction<void(const FText&)> Status);
