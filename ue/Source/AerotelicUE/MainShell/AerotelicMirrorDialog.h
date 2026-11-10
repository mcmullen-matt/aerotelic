#pragma once

#include "CoreMinimal.h"

class AAerotelicEntityActor;

// The scene tree's Mirror dialog (2026-11-10, beside Transform): a
// modeless window per entity with its mirror plane -- whether the
// reflected copy is on, the plane's normal and its offset along the
// normal, the entities[].mirror / mirror_plane fields the project file
// stores. The copy is derived: it follows the entity, its placement and
// its generated structure, and is a body of its own to the booleans, the
// exports, the mass report and the analyses ("<entity> (mirror)").
// Re-choosing Mirror for an entity brings its window forward.
void OpenAerotelicMirrorDialog(AAerotelicEntityActor* Entity);
