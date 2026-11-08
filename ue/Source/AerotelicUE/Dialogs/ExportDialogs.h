#pragma once

#include "CoreMinimal.h"

// File -> Export STL (main_window.py _export_stl_dialog): choose bodies
// with the shared picker, set one mesh density (linear and angular
// deflection), see the mesh statistics, choose binary or ASCII, and write
// one multi-body file -- each body placed as it sits in the scene. Modal.
// Status receives the status-bar line.
void ShowAerotelicExportStlDialog(TFunction<void(const FText&)> Status);

// (The tetrahedral VTK exports -- Export Skin Tetrahedra, Export
// Tetrahedral Mesh -- went with the tetrahedra on 2026-11-08: the
// application models shells only.)

// File -> Export STEP (_export_step_dialog): the same body picker, the
// loft station tolerance with "Rebuild bodies at this tolerance", and
// "Write named parts (STEP assembly)". The file is written by the
// separate aerotelic_step_writer process (Export/AerotelicExport.h).
void ShowAerotelicExportStepDialog(TFunction<void(const FText&)> Status);

// File -> Export 3D PDF: the same body picker and mesh density as
// Export STL, plus the document title and page size, and one page holding
// the bodies as an interactive 3D annotation -- a U3D model (ECMA-363)
// written by core/include/aerotelic/pdf3d.hpp, each body a named node in
// its scene colour. Modal, like the others.
void ShowAerotelicExport3DPdfDialog(TFunction<void(const FText&)> Status);
