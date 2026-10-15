#pragma once

#include "CoreMinimal.h"
#include <vector>

#include "aerotelic/lattice.hpp"
#include "aerotelic/solid_model.hpp"
#include "aerotelic/tet.hpp"

#include "AerotelicBooleanModel.h"
#include "AerotelicTetGmsh.h"

class AAerotelicEntityActor;

// The model of a lattice infill (Lattice -> Generate Lattice...): the
// OpenLattice lattice of `Spec` (lattice.hpp) generated inside the placed
// bodies of `Targets` (an entity, its mirrored half, or a boolean result
// -- the same operands a boolean takes) and kept out of the placed
// `Exclusions` (the primitives marking payload bays and other reserved
// volumes). A normal scene item once built: body, colour, visibility,
// section, styles, STL export.
//
// The hollowing and the infill are MeshLib's (AerotelicMeshLib.h,
// tools/lattice_meshlib.py, run as a separate process): each body is
// offset inward by the skin thickness on a voxel distance field, the
// "Skin" body being the body's own surface outside and the offset surface
// inside; the lattice field (sampled here, from the core's LatticeField)
// is extracted by marching cubes and cut to the union of the offset
// interiors minus the reserved volumes by an exact mesh boolean, giving
// the "Lattice" body. The two are kept apart so a downstream analysis can
// give the shell and the infill different materials; their union is the
// part. The same offset surface is the skin's inner boundary in the
// tetrahedral mesh (GetTetMeshes below), so the finite-element skin is
// the viewport's.
//
// Unlike a boolean result it does NOT regenerate itself when a target is
// edited: generation takes seconds, which would make every keystroke in a
// wing's panel wait. Lattice -> Regenerate Lattices (or editing the
// lattice) rebuilds it; the parameters are saved with the project and the
// lattice is regenerated on load.
class FAerotelicLatticeModel : public aerotelic::ISolidModel
{
public:
	aerotelic::LatticeSpec Spec;
	TArray<FAerotelicBooleanOperand> Targets;
	TArray<FAerotelicBooleanOperand> Exclusions;

	// A lattice has no B-rep solid (it is a mesh): throws std::runtime_error.
	// Booleans over a lattice and STEP export are therefore not offered.
	virtual TopoDS_Shape BuildSolid() const override;

	// Generates the lattice (ignoring `tolerance`: the resolution is the
	// spec's voxels per cell), or returns the last generated meshes until
	// Invalidate(). Two bodies: "Skin" (the hollowed shell of each filled
	// body) and "Lattice" (the infill inside it). Throws std::runtime_error
	// with the reason when a target is gone, no target has geometry, a
	// target or reserved volume is not a closed outward surface, the skin
	// thickness is not positive, the grid is too large, MeshLib is not
	// installed, or one of its operations fails.
	virtual std::vector<aerotelic::NamedMesh> BuildMeshes(double tolerance) const override;

	// A filled body as the tetrahedral mesher sees it, kept from the last
	// generation: its placed surface triangulation (the OML, exactly what
	// MeshLib hollowed) and MeshLib's inward offset of it at the skin depth
	// (the skin's inner boundary; empty where the body is all skin). The
	// offset is usable as the mesher's inner constraint when it is closed,
	// outward and clear of the body's surface; InnerNote says why not.
	struct FTetBody
	{
		FString Label;
		aerotelic::TriMesh Outer;
		aerotelic::TriMesh Inner;
		bool bInnerUsable = false;
		FString InnerNote;
		// The volume below which a pocket of the offset is a sliver, as the
		// generation computed it from the offset's voxel. Carried here
		// because the tetrahedral mesh is built later, on request.
		double MinPocketVolume = 0.0;
	};
	const std::vector<FTetBody>& GetTetBodies() const { return TetBodies; }

	// The skin/core tetrahedral mesh of each filled body (gmsh, through
	// AerotelicTetGmsh: material 1 = skin, between the body's surface and
	// the offset; 2 = core, inside the offset), with the body's scene
	// label, for Lattice -> Export Skin Tetrahedra / Export Tetrahedral
	// Mesh. gmsh takes both surfaces as discrete entities and adds
	// interior nodes only, so the skin tetrahedra's outer faces ARE the
	// body's own triangles and skin + core = the body's volume to
	// rounding. Built on first request (not with the lattice) and kept
	// until Invalidate(); a body whose offset is unusable or that gmsh
	// refuses is left out, with the reason in GetTetNotes().
	struct FTetMesh
	{
		FString Label;
		AerotelicTetGmsh::FResult Result;
	};
	const std::vector<FTetMesh>& GetTetMeshes() const;
	const TArray<FString>& GetTetNotes() const { GetTetMeshes(); return TetNotes; }
	// The tetrahedra of every body of one material (1 skin, 2 core, -1 all)
	// as one mesh holding only the nodes they use, for WriteTetMeshVtk.
	aerotelic::TetMesh MergedTets(int Material) const;

	// Drops the generated mesh so the next build regenerates.
	void Invalidate() { bValid = false; TetMeshes.clear(); bTetMeshesBuilt = false; TetNotes.Reset(); }
	bool IsValid() const { return bValid; }

	// True if this lattice fills or excludes Entity (directly, or a boolean
	// result that depends on it).
	bool DependsOn(const AAerotelicEntityActor* Entity) const;

	// What the last generation produced.
	struct FStats
	{
		double AchievedFraction = 0.0; // lattice volume over the infill domain's
		int64 Triangles = 0;           // skin + lattice
		int64 SkinTriangles = 0;
		int64 LatticeTriangles = 0;
		int64 GridNodes = 0;           // lattice field samples
		double SkinVolume = 0.0;
		double DomainVolume = 0.0;
		double LatticeVolume = 0.0;
		double Seconds = 0.0;
		int32 GridX = 0, GridY = 0, GridZ = 0;
		TArray<FString> Warnings;      // MeshLib's notes (pockets filled solid, a body all skin, ...)
	};
	const FStats& GetStats() const { return Stats; }

	// The bounds of the targets' placed bodies (what the grid will cover),
	// invalid if none can be built. For the dialog's cost estimate.
	FBox TargetBounds() const;

	// The surface tolerance the bodies are meshed at for a spec: a quarter
	// of the lattice voxel or of the skin, whichever is finer.
	static double BodyTolerance(const aerotelic::LatticeSpec& Spec);

private:
	mutable bool bValid = false;
	mutable std::vector<aerotelic::NamedMesh> Cached;
	mutable std::vector<FTetBody> TetBodies;
	mutable bool bTetMeshesBuilt = false;
	mutable std::vector<FTetMesh> TetMeshes;
	mutable TArray<FString> TetNotes;
	mutable FStats Stats;
};
