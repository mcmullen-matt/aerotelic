#pragma once

#include "CoreMinimal.h"

class AAerotelicEntityActor;
struct FAerotelicDesignContent;
namespace aerotelic { class PropellerModel; }

// Builds the Propeller design panel for Entity (GetEntityType() ==
// EntityType::Propeller) over Model, the panel's DRAFT of the entity's
// propeller (see FAerotelicDesignDraft): ui/propeller_panel.py's Propeller
// tab, the blade's wing tabs (planform/sections/twist/dihedral in units of
// the tip radius) and the Spinner tab. Every edit calls OnChanged; nothing
// reaches the entity until Apply.
void BuildPropellerDesignContent(AAerotelicEntityActor* Entity, aerotelic::PropellerModel* Model, FSimpleDelegate OnChanged, FAerotelicDesignContent& Out);
