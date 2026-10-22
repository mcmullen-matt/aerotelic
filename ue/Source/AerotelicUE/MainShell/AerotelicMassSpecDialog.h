#pragma once

#include "CoreMinimal.h"

class AAerotelicEntityActor;

// The scene tree's "Mass Properties..." on an entity: how this
// parameterisation weighs. Either the user asserts the total mass (spread
// uniformly over the surface when weighed -- the model for a skin whose
// thickness the geometry does not carry, where a density would give zero)
// or gives a density for the enclosed volume. Modal; writes the entity's
// MassSpec on OK, which the project file keeps. Analysis -> Mass
// Properties... then weighs the bodies.
void OpenAerotelicMassSpecDialog(AAerotelicEntityActor* Entity);
