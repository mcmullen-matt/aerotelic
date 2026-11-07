#pragma once

#include "CoreMinimal.h"

class AAerotelicEntityActor;

// The scene tree's "Material Properties" on an entity: what this
// parameterisation is made of, picked from ONE list -- not set, an
// asserted total mass, or a material of the library (the presets and the
// user's own, Model > Material Properties; the entity takes a copy, name
// and figures, which the project file keeps) -- with the skin thickness
// (project length units, as typed; 0 = a solid, else every body the
// parameterisation produces is a skin that thick) or, for an asserted
// mass, the mass. Below, read-only: the material's card (density, modulus,
// Poisson's ratio, allowable, source, in the project's units) and the
// entity's geometry (enclosed volume, surface area, closed or open, the
// material volume and what it would weigh). Nothing is typed here but the
// thickness or the mass: a material's figures are edited in the library.
// Modal; writes the entity's MassSpec on OK. Analysis > Mass Properties
// weighs by it; Analysis > Structural analyses with it.
void OpenAerotelicMaterialDialog(AAerotelicEntityActor* Entity);
