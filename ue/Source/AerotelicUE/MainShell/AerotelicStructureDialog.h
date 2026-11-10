#pragma once

#include "CoreMinimal.h"

class AAerotelicEntityActor;

// The Scene tree's "Structure..." on a body: what that ONE solid body is
// made of (aerotelic::BodyStructure), in two tabs --
//   Skin:   the material (picked from the library, Model > Material
//           Properties) and the skin thickness (0 = a solid body);
//   Mass:   an asserted total that replaces every computed figure, and an
//           estimate of what the body weighs, its generated layers included;
// over the body's own geometry (enclosed volume, surface area). MODELESS,
// one window per body. Apply writes the structure to the body
// (UAerotelicDocumentSubsystem::ApplyBodyStructure), and a body with a skin
// then builds its parameterisation's structure -- a wing's spars, ribs and
// stringers from the wing dialog's Structure tab -- at once; Revert re-reads
// the body's. A mirrored half shares the body's structure.
// (A Planes tab of per-body cutting-plane networks lived here from
// 2026-11-08 to 2026-11-10, with a translucent plane preview in the 3D
// view; both went when the structure moved into the parameterisations. An
// Infill tab for a lattice filling the interior went on 2026-11-08.)
void OpenAerotelicStructureDialog(AAerotelicEntityActor* Entity, const FString& BodyRole);
