#pragma once

#include "CoreMinimal.h"

#include "aerotelic/mass_properties.hpp"
#include "aerotelic/units.hpp"
#include "AerotelicBooleanModel.h"

// Analysis -> Mass Properties: the mass, centre of mass and inertia of
// each chosen scene body and of all of them as one assembly. Each body is
// its entity's placed mesh (the mirrored half reflected), weighed the way
// the body's structure says (body_structure.hpp) -- a density times the
// enclosed volume, a skin as density times area times thickness with the
// ribs and infill estimated from the figures, or an asserted total spread
// over the surface -- or, once its structure has been GENERATED, by its
// layers exactly (one line per layer, each a solid of its own material).
// The assembly is the parallel-axis sum of the lines about their joint
// centre. Headless, so the document commandlet can check it; the dialog
// only shows the text.

// One body of the report.
struct FAerotelicMassLine
{
	// "<entity>", "<entity> / <body>" for a multi-body entity, with
	// " (mirror)" for the mirrored half.
	FString Label;
	aerotelic::MassMode Mode = aerotelic::MassMode::Unset;
	aerotelic::MassProperties Props;
	// Why it was left out, or a caveat on a body that was kept (an open
	// surface weighed by density).
	FString Note;
	bool bIncluded = false;
};

struct FAerotelicMassReport
{
	double Tolerance = 0.0;
	TArray<FAerotelicMassLine> Bodies;
	// The included bodies as one: inertia about the assembly's own centre.
	aerotelic::MassProperties Assembly;
	int32 IncludedCount = 0;
};

// Meshes every body at Tolerance (the STL / lattice tolerance) and weighs
// it in the project's units (a material's SI density is converted to
// them, since the meshes are in them). A body whose entity has no
// MassSpec, or no geometry, is listed with the reason and left out of
// the assembly rather than counted as zero.
FAerotelicMassReport ComputeAerotelicMassReport(const TArray<FAerotelicBooleanOperand>& Bodies, double Tolerance,
	const aerotelic::UnitSystem& Units);

// The report as a table in the project's units (which the integrals were
// done in): one row per component, the assembly under a rule. What the
// dialog shows, copies and saves.
FString FormatAerotelicMassReport(const FAerotelicMassReport& Report, const aerotelic::UnitSystem& Units);
