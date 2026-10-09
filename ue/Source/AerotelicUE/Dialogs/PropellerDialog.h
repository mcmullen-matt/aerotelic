#pragma once

#include "CoreMinimal.h"

class AAerotelicEntityActor;
struct FAerotelicDesignContent;

// Builds the Propeller design panel for Entity (GetEntityType() ==
// EntityType::Propeller): ui/propeller_panel.py's Propeller tab, the blade's
// wing tabs (planform/sections/twist/dihedral in units of the tip radius)
// and the Spinner tab.
void BuildPropellerDesignContent(AAerotelicEntityActor* Entity, FAerotelicDesignContent& Out);
