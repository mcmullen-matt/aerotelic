#pragma once

#include "CoreMinimal.h"
#include <vector>

#include "aerotelic/body_structure.hpp"
#include "aerotelic/units.hpp"
#include "aerotelic/vec3.hpp"

#include "AerotelicBooleanModel.h"
#include "AerotelicRibGmsh.h"
#include "AerotelicStructuralFea.h"

class AAerotelicEntityActor;

// The structural model of a BODY from its structure (body_structure.hpp),
// shared by the Structural dialog, the commandlets and the smoke run
// (2026-11-08) so the three cannot drift apart. The model is SHELLS ONLY
// -- the application dropped tetrahedral FEA and lattice infill on
// 2026-11-08 -- so a body is:
//
//   - its SKIN: shells of the skin thickness and material on the body's
//     own grid (the triangles its Mesh tab defines and the viewport
//     shows, GetPlacedMeshes at tolerance 0), nothing inside;
//   - its PLANE NETWORKS: the surface is first cut by every plane (core
//     mesh_cut.hpp), so the rims of the ribs are the skin's own nodes;
//     the ribs are then meshed by gmsh as planar discs on those rims
//     (AerotelicRibGmsh.h), one region per network with its material and
//     thickness, unloaded.
//
// A body with NO skin thickness cannot be modelled -- there is nothing to
// make shells of -- and is refused with a Problem that says to give it one
// under Structure... in the Scene tree. (Until 2026-11-08 such a body was
// filled with solid tetrahedra; that path is gone.)
//
// Every length is the placed (replica) body's, project units; the skin
// and rib thicknesses and heights are the structure's physical figures.
namespace AerotelicFeaModelBuilder
{
	struct FOptions
	{
		// Target edge inside the ribs (0: the surface's median edge).
		double RibSize = 0.0;
		// Progress lines ("meshing ...").
		TFunction<void(const FString&)> Status;
	};

	// gmsh's rib meshes by the surface's bytes, the rib size and the
	// planes, so Build Model after a support edit runs gmsh again for
	// nothing. Owned by the caller (the dialog's state) across builds.
	struct FCache
	{
		TMap<FString, TSharedPtr<AerotelicRibGmsh::FResult>> Ribs;
	};

	// What AppendBody accumulates: one node list over every body, the skin
	// faces, the rib faces, the regions, and what was left out.
	struct FBuilt
	{
		std::vector<aerotelic::Vec3> Nodes;
		TArray<AerotelicStructuralFea::FShellFace> ShellFaces;
		TArray<AerotelicStructuralFea::FShellFace> RibFaces;
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
