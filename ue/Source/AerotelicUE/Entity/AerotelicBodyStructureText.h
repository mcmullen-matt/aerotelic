#pragma once

#include "CoreMinimal.h"

#include "aerotelic/body_structure.hpp"
#include "aerotelic/units.hpp"

class AAerotelicEntityActor;

// A body's structure in one line, in the project's units, for the Scene
// tree's tooltips and the dialogs' captions: "not set", "asserted 0.2 kg",
// "Aluminium 6061-T6 sheet, solid", "balsa skin 1.5 mm; gyroid 30% infill
// (PLA); Ribs: 9 planes normal y, 2 mm (birch ply); Spar: 1 plane, 3 mm,
// 10 mm deep (spruce)".
FString DescribeAerotelicBodyStructure(const aerotelic::BodyStructure& S, const aerotelic::UnitSystem& Units);

// One line per body of the entity: "<body>: <structure>".
FString DescribeAerotelicEntityStructures(const AAerotelicEntityActor* Entity, const aerotelic::UnitSystem& Units);
