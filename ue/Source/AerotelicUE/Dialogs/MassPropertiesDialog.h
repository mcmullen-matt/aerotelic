#pragma once

#include "CoreMinimal.h"

// Analysis -> Mass Properties...: pick scene bodies with the same two-list
// picker the Boolean Operation, export and flow dialogs use, mesh them at
// a chosen tolerance, and report the mass, centre of mass and inertia of
// each and of the assembly (Entity/AerotelicMassProperties.h). Each
// entity weighs the way its own Mass Properties setting in the Scene tree
// says; one without a setting is listed as left out. Modal; the report
// can be copied or saved as text. Status receives the status-bar line.
void ShowAerotelicMassPropertiesDialog(TFunction<void(const FText&)> Status);
