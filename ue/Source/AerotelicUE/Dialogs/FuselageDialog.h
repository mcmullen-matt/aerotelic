#pragma once

#include "CoreMinimal.h"

class AAerotelicEntityActor;
struct FAerotelicDesignContent;

// Builds the Fuselage design panel for Entity (GetEntityType() ==
// EntityType::Fuselage). Port of ui/fuselage_panel.py's Profile/Width/
// rho-upper/rho-lower tabs -- the Sections tab (a read-only cross-section
// preview/table, no editable state) and the Help tab are left out of this
// pass's scope, same call as every other panel here.
void BuildFuselageDesignContent(AAerotelicEntityActor* Entity, FAerotelicDesignContent& Out);
