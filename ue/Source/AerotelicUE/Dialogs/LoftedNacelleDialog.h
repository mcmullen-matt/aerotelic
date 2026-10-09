#pragma once

#include "CoreMinimal.h"

class AAerotelicEntityActor;
struct FAerotelicDesignContent;

// Builds the Lofted Nacelle design panel for Entity (GetEntityType() ==
// EntityType::LoftedNacelle): ui/lofted_nacelle_panel.py's Centreline /
// Guide curves / Skins / Chevrons tabs plus the shared Length field.
void BuildLoftedNacelleDesignContent(AAerotelicEntityActor* Entity, FAerotelicDesignContent& Out);
