#pragma once

#include "CoreMinimal.h"

// File -> Export STL... (main_window.py _export_stl_dialog): choose bodies
// with the shared picker, set one mesh density (linear and angular
// deflection), see the mesh statistics, choose binary or ASCII, and write
// one multi-body file -- each body placed as it sits in the scene. Modal.
// Status receives the status-bar line.
void ShowAerotelicExportStlDialog(TFunction<void(const FText&)> Status);

// Lattice -> Export Skin Tetrahedra (VTK)...: the skin of every lattice in
// the scene (shell.hpp's prism tetrahedra, built by the last generation)
// as one legacy VTK unstructured grid, for a finite-element model. Asks
// for the file; reports through Status.
void ShowAerotelicExportSkinVtkDialog(TFunction<void(const FText&)> Status);

// File -> Export STEP... (_export_step_dialog): the same body picker, the
// loft station tolerance with "Rebuild bodies at this tolerance", and
// "Write named parts (STEP assembly)". The file is written by the
// separate aerotelic_step_writer process (Export/AerotelicExport.h).
void ShowAerotelicExportStepDialog(TFunction<void(const FText&)> Status);
