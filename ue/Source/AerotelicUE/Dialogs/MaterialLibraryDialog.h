#pragma once

#include "CoreMinimal.h"

// Model > Material Properties: the materials a body can be made of, as
// three tabs.
//   * Materials -- one row per material, preset or the user's own, with
//     its category, density, Young's modulus, Poisson's ratio, allowable
//     stress (in the project's units) and the source of the figures; any
//     column header sorts the table, ascending then descending. A
//     double-click takes a row to the Custom tab as a starting point.
//   * Custom -- define your own: every figure typed in the project's
//     units (kept in SI), a name, a category and a source note; Add,
//     Update and Remove write the per-user library file at once
//     (AerotelicMaterialLibrary), so the material is offered in every
//     project from then on.
//   * Help -- how to use the two, the conventions the presets follow and
//     the citations for their figures.
// Assigning a material to an entity is the Scene tree's Material
// Properties on its row. Modal; `Status` gets a line for the status bar.
void ShowAerotelicMaterialLibraryDialog(TFunction<void(const FText&)> Status = nullptr);
