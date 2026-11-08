#pragma once

#include "CoreMinimal.h"

class AAerotelicEntityActor;

// The Scene tree's "Structure..." on a body: what that ONE solid body is
// made of and built with (aerotelic::BodyStructure), in four tabs --
//   Skin:   the material (picked from the library, Model > Material
//           Properties) and the skin thickness (0 = a solid body);
//   Infill: an optional lattice filling the interior, with its own material;
//   Planes: the networks of cutting planes -- ribs, spars, frames -- each a
//           set of parallel planes normal to x, y, z or a custom axis, from a
//           start position at a spacing, so many of them, a thickness, a
//           height in from the surface (0 = wall to wall) and a material;
//   Mass:   an asserted total that replaces every computed figure, and an
//           estimate of what the structure weighs;
// over the body's own geometry (enclosed volume, surface area). MODELESS,
// one window per body: while the Planes tab is edited the 3D view shows the
// network's planes as translucent rectangles over the body, live. Apply
// writes the structure to the body (the lattice over it regenerates when
// its geometry changed); Revert re-reads the body's. A mirrored half shares
// the body's structure.
void OpenAerotelicStructureDialog(AAerotelicEntityActor* Entity, const FString& BodyRole);
