#pragma once

#include "CoreMinimal.h"
#include <memory>
#include <vector>

#include "aerotelic/body_structure.hpp"
#include "aerotelic/lattice.hpp"
#include "aerotelic/tet.hpp"
#include "aerotelic/units.hpp"

#include "AerotelicBooleanModel.h"
#include "AerotelicStructuralFea.h"
#include "AerotelicTetGmsh.h"

class AAerotelicEntityActor;

// The structural model of a BODY from its structure (body_structure.hpp),
// shared by the Structural dialog, the commandlets and the smoke run
// (2026-11-08) so the three cannot drift apart:
//
//   - skin 0: SOLID tetrahedra of the material at the element size;
//   - skin > 0, no infill in the model: SHELLS of the skin on the body's
//     own grid, nothing inside;
//   - skin > 0 with an infill: shells of the skin material over 4-node
//     tetrahedra at the lattice cell carrying the HOMOGENISED cell of the
//     INFILL material (the two materials distinct); or, at full
//     resolution, the generated structure's skin / core tetrahedra;
//   - the body's PLANE NETWORKS: the body's surface is first cut by every
//     plane (core mesh_cut.hpp), so the rims of the ribs are the skin's own
//     nodes; the ribs are then meshed by gmsh as planar shells embedded in
//     the tetrahedra (or hanging on the skin shells alone), one region per
//     network with its material and thickness, unloaded. Not in the
//     full-resolution model (the generation's offset surface knows nothing
//     of them): said so in Problems.
//
// Every length is the placed (replica) body's, project units; the skin,
// rib thicknesses and heights are the structure's physical figures.
namespace AerotelicFeaModelBuilder
{
	enum class EModel : uint8
	{
		AsDesigned = 0,       // shells, homogenised infill, rib shells
		FullResolution = 1,   // the generated structure's skin / core tetrahedra
		IgnoreInterior = 2,   // the body as its skin and material alone say
	};

	struct FOptions
	{
		EModel Model = EModel::AsDesigned;
		// Voxels across the infill cell for the homogeniser.
		double Voxels = 32.0;
		// Element size of a solid body (0: a twentieth of the chosen bodies'
		// largest extent, which the caller resolves to a figure).
		double ElementSize = 0.0;
		// Target edge inside the ribs (0: the surface's median edge).
		double RibSize = 0.0;
		// The request's second-order flag, for the description only.
		bool bSecondOrder = false;
		// Progress lines ("meshing ...").
		TFunction<void(const FString&)> Status;
	};

	// gmsh's tetrahedral meshes by the surface's bytes, the cell size and
	// the ribs, so Build Model after a support edit runs gmsh again for
	// nothing. Owned by the caller (the dialog's state) across builds.
	struct FCache
	{
		TMap<FString, TSharedPtr<AerotelicTetGmsh::FResult>> Tets;
	};

	// What AppendBody accumulates: one mesh over every body, the shell-only
	// bodies' faces, the rib faces, the regions, and what was left out.
	struct FBuilt
	{
		aerotelic::TetMesh Mesh;
		TArray<AerotelicStructuralFea::FShellFace> ShellFaces;
		TArray<AerotelicStructuralFea::FShellFace> RibFaces;
		std::vector<std::unique_ptr<aerotelic::UnitCellSamples>> Cells;
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
