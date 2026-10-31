#pragma once

#include "CoreMinimal.h"

class AAerotelicEntityActor;
struct FAerotelicDesignContent;
namespace aerotelic { class WingModel; }

// Builds the Wing design panel for Entity (GetEntityType() ==
// EntityType::Wing) over Model, the panel's DRAFT of the entity's wing
// (see FAerotelicDesignDraft): the Planform / Sections / Twist / Dihedral
// tabs of ui/planform_panel.py et al., shown in the design dock. Every
// edit calls OnChanged; nothing reaches the entity until Apply.
void BuildWingDesignContent(AAerotelicEntityActor* Entity, aerotelic::WingModel* Model, FSimpleDelegate OnChanged, FAerotelicDesignContent& Out);
