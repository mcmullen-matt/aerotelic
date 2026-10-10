#pragma once

#include "CoreMinimal.h"

// Geometry -> Airfoils...: the airfoil library panel (ui/airfoil_panel.py)
// as a modeless window, one instance: Download Selig DB / Re-download,
// Load .dat..., the search box and source combo (All / Selig / Custom),
// the result list, the 2-D preview and the statistics. Preview only --
// assigning an airfoil to a section is done from a wing's Sections tab,
// whose picker shares the same library.
void OpenAerotelicAirfoilLibrary();
