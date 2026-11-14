#pragma once

#include "CoreMinimal.h"

// Model > Load Cases (2026-11-15): the tabbed dialog that defines what the
// structural analysis is run for, saved with the project (load_cases.hpp).
//
//   AERO: the V-n diagram -- CL_max, CL_min, V_ne, n_max, n_min, the air
//   density, the weight and area (the mass report of the chosen bodies and
//   the chosen wings' planform area, or typed). Clicking in the plot adds a
//   point; each point is a row of the table below with its name, speed,
//   density, load factor, the flap the trim turns and the solid bodies
//   involved. Solve runs the panel method's trim (AerotelicAeroTrim) and
//   stores the pressure field for the structural analysis.
//
//   STRUCTURAL: a table of cases (name, load factor) over the Kratos
//   dialog's old editors -- the bodies, the loads (inertial, pressure,
//   point forces) and the constraints (support planes and patches, or free
//   flight), with the same pictures on the full built model and the same
//   patch dragging (FeaCaseState.h).
//
// Modeless; Status receives the status-bar line.
void ShowAerotelicLoadCasesDialog(TFunction<void(const FText&)> Status);
