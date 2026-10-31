#pragma once

#include "CoreMinimal.h"

class AAerotelicEntityActor;
struct FAerotelicDesignContent;
namespace aerotelic { class LoftedNacelleModel; }

// Builds the Lofted Nacelle design panel for Entity (GetEntityType() ==
// EntityType::LoftedNacelle) over Model, the panel's DRAFT of the entity's
// nacelle (see FAerotelicDesignDraft): ui/lofted_nacelle_panel.py's
// Centreline / Guide curves / Skins / Chevrons tabs plus the shared Length
// field. Every edit calls OnChanged; nothing reaches the entity until
// Apply.
void BuildLoftedNacelleDesignContent(AAerotelicEntityActor* Entity, aerotelic::LoftedNacelleModel* Model, FSimpleDelegate OnChanged, FAerotelicDesignContent& Out);
