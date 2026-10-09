#pragma once

#include "CoreMinimal.h"

class AAerotelicEntityActor;

// The scene tree's Transform dialog (main_window.py's _show_transform_dialog):
// a modeless window per entity with its placement -- translation, rotation
// axis and angle, the same entities[].placement fields the project file
// stores. Re-choosing Transform for an entity brings its window forward.
void OpenAerotelicTransformDialog(AAerotelicEntityActor* Entity);
