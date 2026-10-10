#pragma once

#include "CoreMinimal.h"
#include <memory>
#include "aerotelic/airfoil.hpp"

// Airfoil picker (section_panel.py AirfoilPickerDialog): the library
// browser over the sources In this project / NACA 4-digit / Selig database
// / Custom (.dat), with search, preview and statistics, Load .dat..., and
// the database download offered while it is missing. Double-click or OK
// accepts. Modal: blocks until the user accepts or cancels. Returns unset
// if cancelled. (InitialCode is kept for callers; the list opens on the
// common NACA codes.)
TOptional<std::shared_ptr<aerotelic::Airfoil>> PromptForAirfoil(
	const TArray<std::shared_ptr<aerotelic::Airfoil>>& ProjectAirfoils, const FString& InitialCode = TEXT("2412"));
