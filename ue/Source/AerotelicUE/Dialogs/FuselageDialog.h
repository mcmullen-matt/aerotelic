#pragma once

#include "CoreMinimal.h"

class AAerotelicEntityActor;
struct FAerotelicDesignContent;
namespace aerotelic { class FuselageModel; }

// Builds the Fuselage design panel for Entity (GetEntityType() ==
// EntityType::Fuselage) over Model, the panel's DRAFT of the entity's
// fuselage (see FAerotelicDesignDraft). Port of ui/fuselage_panel.py's
// Profile/Width/rho-upper/rho-lower tabs plus the Sections preview and the
// Help tab. Every edit calls OnChanged; nothing reaches the entity until
// Apply.
void BuildFuselageDesignContent(AAerotelicEntityActor* Entity, aerotelic::FuselageModel* Model, FSimpleDelegate OnChanged, FAerotelicDesignContent& Out);
