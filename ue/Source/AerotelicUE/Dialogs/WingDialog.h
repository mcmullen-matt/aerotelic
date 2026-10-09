#pragma once

#include "CoreMinimal.h"

class AAerotelicEntityActor;
struct FAerotelicDesignContent;

// Builds the Wing design panel for Entity, whose model must be a WingModel
// (GetEntityType() == EntityType::Wing): the Planform / Sections / Twist /
// Dihedral tabs of ui/planform_panel.py et al., shown in the design dock.
void BuildWingDesignContent(AAerotelicEntityActor* Entity, FAerotelicDesignContent& Out);
