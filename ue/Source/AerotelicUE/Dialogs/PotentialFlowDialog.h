#pragma once

#include "CoreMinimal.h"

// Analysis -> Potential Flow (Kratos)...: pick the geometry the flow is to
// be computed around with the same two-list picker the Boolean Operation
// dialog uses (every entity, the mirrored half of each mirrored one, every
// boolean result), set the free stream and the size of the air around it,
// name a solution file, and solve. Modal; the solve runs in place and the
// statistics appear in the dialog. Status receives the status-bar line.
//
// The grid and the solution are written as one legacy VTK unstructured
// grid: the tetrahedra of the air with the velocity potential, the
// velocity and the pressure coefficient at every node, and the bodies'
// own surface triangles alongside them (cell data `region` = 1) so the
// pressure distribution on the aircraft can be shown on its own.
void ShowAerotelicPotentialFlowDialog(TFunction<void(const FText&)> Status);
