#pragma once

#include "CoreMinimal.h"
#include <memory>
#include "aerotelic/airfoil.hpp"

// Airfoil picker: every project airfoil already in the scene offered first
// (click to choose it outright, matching section_panel.py's
// AirfoilPickerDialog merging in project_airfoils()), plus a NACA 4-digit
// code entry with a live preview (Selig-database browsing and .dat loading
// are out of scope this pass -- see the approved plan). Modal: blocks until
// the user accepts or cancels. Returns unset if cancelled.
TOptional<std::shared_ptr<aerotelic::Airfoil>> PromptForAirfoil(
	const TArray<std::shared_ptr<aerotelic::Airfoil>>& ProjectAirfoils, const FString& InitialCode = TEXT("2412"));
