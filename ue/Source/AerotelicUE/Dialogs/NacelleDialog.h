#pragma once

#include "CoreMinimal.h"

class AAerotelicEntityActor;
struct FAerotelicDesignContent;
namespace aerotelic { class NacelleModel; }

// Builds the Nacelle (revolved cowl) design panel for Entity
// (GetEntityType() == EntityType::Nacelle) over Model, the panel's DRAFT of
// the entity's nacelle (see FAerotelicDesignDraft): ui/nacelle_panel.py's
// Side view / Front view / Chevrons tabs plus the shared Length field.
// Every edit calls OnChanged; nothing reaches the entity until Apply.
void BuildNacelleDesignContent(AAerotelicEntityActor* Entity, aerotelic::NacelleModel* Model, FSimpleDelegate OnChanged, FAerotelicDesignContent& Out);
