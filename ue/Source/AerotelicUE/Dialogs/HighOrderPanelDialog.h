#pragma once

#include "CoreMinimal.h"

// Analysis -> Panel Method: the incompressible panel method ON THE BODY
// TRIANGULATION (core tri_panel.hpp, through Entity/AerotelicAeroTrim.h)
// over the scene's bodies, chosen with the two-list picker the export
// dialogs use. Since 2026-11-15 the panels are the triangles the viewport
// draws: a wing as its sealed aero copy with a wake from its trailing
// edge, every other body as its own triangles, a boolean result as its
// operands' aero copies, overlapping bodies fused into one surface first.
// A symmetry plane at y = 0 is optional (on by default when something is
// mirrored, a wing is symmetric or a wing root sits on the plane). The
// file keeps its historical name; the higher-order method it was written
// for (Johnson, NASA CR-3079) stays in the core as the validation reference.
//
// Five tabs: the set-up (bodies, the mesh report with a switch that shows
// the mesh in the 3D view, a list of angles of attack or of sideslip,
// references, the solution file), the panels coloured by Cp with their
// edges in an engine-rendered view with a colour bar (SAerotelicSceneView
// and AAerotelicColoredMeshActor), the span loading of each wing against
// the elliptic distribution of the same lift, the wind-axis force and
// moment coefficients against the swept angle, and help. The solves run on
// a worker thread, one per angle. Modeless, like the design dialogs, so the
// world keeps rendering the view. Status receives the status-bar line.
//
// Independent of the Kratos route (Potential Flow (Kratos)), which is
// left as it is.
void ShowAerotelicHighOrderPanelDialog(TFunction<void(const FText&)> Status);
