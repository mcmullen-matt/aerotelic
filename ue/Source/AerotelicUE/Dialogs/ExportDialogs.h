#pragma once

#include "CoreMinimal.h"

// File -> Export STL... (main_window.py _export_stl_dialog): choose bodies
// with the shared picker, set one mesh density (linear and angular
// deflection), see the mesh statistics, choose binary or ASCII, and write
// one multi-body file -- each body placed as it sits in the scene. Modal.
// Status receives the status-bar line.
void ShowAerotelicExportStlDialog(TFunction<void(const FText&)> Status);

// Lattice -> Export Skin Tetrahedra (VTK)...: the skin tetrahedra (material
// 1) of every body filled by a lattice -- the tetrahedral mesh each lattice
// model builds on request over the body's surface and MeshLib's inward
// offset at the skin depth (tet.hpp TetrahedralizeNested: a constrained
// Delaunay mesh conforming to both exactly, so the skin's outer faces tile
// the body's own triangulation and its inner faces the offset) -- as one
// legacy VTK unstructured grid, for a finite-element model. Asks for the
// file; the status line reports the volumes (skin + core against the
// bodies') so the conformity can be checked; a message lists bodies left
// out or not conformed exactly.
void ShowAerotelicExportSkinVtkDialog(TFunction<void(const FText&)> Status);

// Lattice -> Export Tetrahedral Mesh (VTK)...: the same meshes with both
// the skin (material 1) and the core (material 2, inside the offset: the
// volume the lattice's material card applies to), as one legacy VTK
// unstructured grid. Skin + core tile each body exactly.
void ShowAerotelicExportTetVtkDialog(TFunction<void(const FText&)> Status);

// File -> Export STEP... (_export_step_dialog): the same body picker, the
// loft station tolerance with "Rebuild bodies at this tolerance", and
// "Write named parts (STEP assembly)". The file is written by the
// separate aerotelic_step_writer process (Export/AerotelicExport.h).
void ShowAerotelicExportStepDialog(TFunction<void(const FText&)> Status);
