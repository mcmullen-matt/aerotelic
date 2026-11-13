#pragma once

#include "CoreMinimal.h"
#include <vector>

#include "aerotelic/body_structure.hpp"
#include "aerotelic/units.hpp"
#include "aerotelic/vec3.hpp"

#include "AerotelicBooleanModel.h"
#include "AerotelicStructuralFea.h"

class AAerotelicEntityActor;

// The structural model of a BODY from its structure (body_structure.hpp)
// and its generated layers, shared by the Structural dialog, the
// commandlets and the smoke run (2026-11-08) so the three cannot drift
// apart. The model is SHELLS ONLY -- the application dropped tetrahedral
// FEA and lattice infill on 2026-11-08 -- so a body is:
//
//   - its SKIN: shells of the skin thickness and material on the body's
//     own grid (the triangles its Mesh tab defines and the viewport
//     shows, GetPlacedMeshes at tolerance 0), nothing inside;
//   - its GENERATED LAYERS (2026-11-10; the plane networks meshed here
//     until then are gone): the ribs, spars and stringers the body's
//     parameterisation built with it (AerotelicBodyStructure.h), each
//     layer a region of unloaded shells of its own thickness and material.
//     They were built on the same grid, so their rims are the skin's own
//     vertices and a spar's web carries the chord nodes of the ribs it
//     crosses; the assembly matches nodes by position (to a billionth of
//     the body's diagonal) and so joins them to the skin and to one
//     another.
//
// A body with NO skin thickness cannot be modelled -- there is nothing to
// make shells of -- and is refused with a Problem that says to give it one
// under Structure... in the Scene tree. (Until 2026-11-08 such a body was
// filled with solid tetrahedra; that path is gone.)
//
// Every length is the placed (replica) body's, project units; the skin
// and layer thicknesses are the structure's physical figures.
namespace AerotelicFeaModelBuilder
{
	struct FOptions
	{
		// Progress lines ("meshing ...").
		TFunction<void(const FString&)> Status;
	};

	// Kept for the callers' sake: nothing is cached since the layers come
	// ready-built from the bodies (2026-11-10).
	struct FCache
	{
	};

	// What AppendBody accumulates: one node list over every body, the skin
	// faces, the rib faces, the regions, and what was left out.
	struct FBuilt
	{
		std::vector<aerotelic::Vec3> Nodes;
		TArray<AerotelicStructuralFea::FShellFace> ShellFaces;
		TArray<AerotelicStructuralFea::FShellFace> RibFaces;
		// The material REFERENCE DIRECTION of every face (2026-11-12), one
		// per ShellFaces / RibFaces entry: in a body's skin the along-the-
		// loft direction of its triangle (the gradient of the loft's u over
		// the triangle, from the plan's vertex parameters: a wing's span, a
		// fuselage's length), or the body's longest extent when it is no
		// loft; in a layer its mesh's longest extent. Projected into the
		// face by the solver. A laminate's 0 degree ply runs along it; an
		// isotropic region ignores it.
		TArray<aerotelic::Vec3> ShellAxes;
		TArray<aerotelic::Vec3> RibAxes;
		TArray<AerotelicStructuralFea::FRegion> Regions;
		TArray<FString> Problems;
		int32 NextId = 1;
	};

	// The structure of a body of the entity by role (the first body when
	// the role is empty), or null when the entity has no such body.
	const aerotelic::BodyStructure* StructureOf(const AAerotelicEntityActor* Entity, const FString& Role);
	// The role itself, resolved the same way.
	FString RoleOf(const AAerotelicEntityActor* Entity, const FString& Role);

	// The Set-up tab's line for a body: what it is and how it will be
	// modelled, or why it cannot be. bOutOk false when it cannot.
	FString Describe(const FAerotelicBooleanOperand& Operand, const FString& Role, const FString& Label, const FOptions& Options,
		const aerotelic::UnitSystem& Units, bool* bOutOk = nullptr);

	// Appends the body (its own grid, placed or mirrored) as regions of
	// Built. BodyIndex is the picker's index of the body (FRegion::Body).
	// False, with the reason in Built.Problems, when nothing was added.
	bool AppendBody(const FAerotelicBooleanOperand& Operand, const FString& Role, const FString& Label, int32 BodyIndex,
		const FOptions& Options, FCache& Cache, FBuilt& Built);
}
