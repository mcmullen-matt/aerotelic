#pragma once

#include "CoreMinimal.h"
#include "Widgets/SWidget.h"

#include "aerotelic/project_yaml.hpp"

namespace aerotelic { class ISolidModel; }

// The "Mesh" tab every design panel ends with (before Help): the density of
// the entity's grid -- ISolidModel::mesh, a MeshSpec (mesh.hpp) -- which
// is the ONE grid the viewport, the structural analysis, the flow solvers'
// wetted surface and the panel method all use. The rows depend on the
// kind of entity: a lofted body (wing, fuselage, nacelle, propeller, the
// sphere primitive) has panels along and round, the refinement switch,
// its tolerance and the strip aspect; a wing (and a propeller's blades)
// also the panel method's chordwise and spanwise counts; a box its cells
// along x, y and z; a cylinder its points round, cells along and cap
// rings. Every field's zero is "automatic" -- what the model did before
// the tab existed -- and the tab says so beside it. Edits mark the draft
// like any other tab's; Apply hands them to the entity, which rebuilds.
//
// A read-out at the bottom builds the draft's bodies at the viewport's
// tolerance and reports the triangle count, so the cost of a setting is
// seen before Apply.
TSharedRef<SWidget> MakeMeshTab(aerotelic::ISolidModel* Model, aerotelic::project_yaml::EntityType Type, FSimpleDelegate OnChanged);

// The tab's description, for the panels' Help text.
FText MeshTabHelpText(aerotelic::project_yaml::EntityType Type);
