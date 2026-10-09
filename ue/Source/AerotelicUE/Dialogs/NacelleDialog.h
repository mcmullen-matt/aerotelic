#pragma once

#include "CoreMinimal.h"

class AAerotelicEntityActor;
struct FAerotelicDesignContent;

// Builds the Nacelle (revolved cowl) design panel for Entity
// (GetEntityType() == EntityType::Nacelle): ui/nacelle_panel.py's Side view /
// Front view / Chevrons tabs plus the shared Length field.
void BuildNacelleDesignContent(AAerotelicEntityActor* Entity, FAerotelicDesignContent& Out);
