#pragma once

#include "CoreMinimal.h"
#include <vector>

#include "aerotelic/lattice.hpp"
#include "aerotelic/solid_model.hpp"
#include "aerotelic/tet.hpp"

#include "AerotelicBooleanModel.h"

class AAerotelicEntityActor;

// The model of a lattice infill (Lattice -> Generate Lattice...): the
// OpenLattice lattice of `Spec` (lattice.hpp) generated inside the placed
// bodies of `Targets` (an entity, its mirrored half, or a boolean result
// -- the same operands a boolean takes) and kept out of the placed
// `Exclusions` (the primitives marking payload bays and other reserved
// volumes). The bodies enter as the signed distance fields of their
// triangle meshes (sdf.hpp), so any body the viewport can show can be
// filled. A normal scene item once built: body, colour, visibility,
// section, styles, STL export.
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
	// Invalidate(). Two bodies: "Skin", the solid shell of each filled body
	// as the tetrahedral mesher makes it (tet.hpp BuildSkinCoreMesh: the
	// body's surface triangles outside, the exact iso-surface of its
	// distance field at the skin depth inside -- which rounds a concave
	// crease such as a tailplane meeting a boom from the inside and cannot
	// self-intersect, where an offset copy of the surface cuts through the
	// corner), as the closed boundary of the skin tetrahedra; and
	// "Lattice", the infill sampled where the body's own distance field is
	// deeper than the skin thickness -- separate so a downstream analysis
	// can give the shell and the infill their own materials; their union is
	// the part. The skin's tetrahedra are kept (GetSkinMeshes) for export
	// to an analysis. Throws std::runtime_error with the reason when a
	// target is gone, no target has geometry, a target or reserved volume
	// is not a closed outward surface (the distance field's sign would be
	// wrong around it), the skin thickness is not positive, the mesher
	// cannot mesh a body, or the grid is too large.
	virtual std::vector<aerotelic::NamedMesh> BuildMeshes(double tolerance) const override;

	// The skin/core tetrahedral mesh of each filled body (built by the last
	// generation; material 1 = skin, 2 = core), with the body's scene label.
	struct FSkinMesh
	{
		FString Label;
		aerotelic::TetMesh Mesh;
	};
	const std::vector<FSkinMesh>& GetSkinMeshes() const { return SkinMeshes; }
	// The skin tetrahedra of every body as one mesh (material 1 only), for
	// WriteTetMeshVtk.
	aerotelic::TetMesh MergedSkinTets() const;

	// The placed surface meshes of the lattice's targets (one per body,
	// labelled), built at `Tolerance`; the names of targets that produced
	// nothing go to OutMissing. What the tetrahedral mesher (tet.hpp)
	// conforms to.
	std::vector<aerotelic::NamedMesh> TargetBodies(double Tolerance, TArray<FString>& OutMissing) const;

	// Drops the generated mesh so the next build regenerates.
	void Invalidate() { bValid = false; }
	bool IsValid() const { return bValid; }

	// True if this lattice fills or excludes Entity (directly, or a boolean
	// result that depends on it).
	bool DependsOn(const AAerotelicEntityActor* Entity) const;

	// What the last generation produced.
	struct FStats
	{
		double AchievedFraction = 0.0; // of the interior domain (inside the skin)
		int64 Triangles = 0;
		int64 SkinTets = 0;
		int64 GridNodes = 0;
		int64 BodyEvaluations = 0;
		double Seconds = 0.0;
		int32 GridX = 0, GridY = 0, GridZ = 0;
	};
	const FStats& GetStats() const { return Stats; }

	// The bounds of the targets' placed bodies (what the grid will cover),
	// invalid if none can be built. For the dialog's cost estimate.
	FBox TargetBounds() const;

private:
	mutable bool bValid = false;
	mutable std::vector<aerotelic::NamedMesh> Cached;
	mutable std::vector<FSkinMesh> SkinMeshes;
	mutable FStats Stats;
};
