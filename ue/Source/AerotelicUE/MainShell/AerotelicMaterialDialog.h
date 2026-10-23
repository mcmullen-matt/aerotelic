#pragma once

#include "CoreMinimal.h"

class AAerotelicEntityActor;

// The scene tree's "Material Properties..." on an entity: what this
// parameterisation is made of. Either a MATERIAL -- picked from the
// presets (materials.hpp) or typed: density, Young's modulus, Poisson's
// ratio and an allowable stress, shown in the project's units and kept in
// SI, since a material is a property of the stuff and not of the units --
// or an ASSERTED total mass, spread uniformly over the surface when
// weighed (the model for a skin whose thickness the geometry does not
// carry, where a density would give zero). Modal; writes the entity's
// MassSpec on OK, which the project file keeps. Analysis > Mass
// Properties weighs by it; Analysis > Structural analyses with it.
void OpenAerotelicMaterialDialog(AAerotelicEntityActor* Entity);
