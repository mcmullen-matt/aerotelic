#pragma once

#include "CoreMinimal.h"

class AAerotelicEntityActor;

// The scene tree's "Material Properties" on an entity: what this
// parameterisation is made of. Either a MATERIAL -- picked from the
// presets (materials.hpp) or typed: density, Young's modulus, Poisson's
// ratio and an allowable stress, shown in the project's units and kept in
// SI, since a material is a property of the stuff and not of the units --
// as a solid, or as a SKIN of a given thickness (project length units, as
// typed) on every body the parameterisation produces, whose volume is
// then area times thickness; or an ASSERTED total mass, spread uniformly
// over the surface when weighed. Modal; writes the entity's
// MassSpec on OK, which the project file keeps. Analysis > Mass
// Properties weighs by it; Analysis > Structural analyses with it.
void OpenAerotelicMaterialDialog(AAerotelicEntityActor* Entity);
