#pragma once

#include "CoreMinimal.h"
#include <vector>

#include "aerotelic/lattice.hpp"
#include "aerotelic/shell.hpp"
#include "aerotelic/solid_model.hpp"

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
	// built explicitly from its surface triangulation (shell.hpp: the
	// surface offset inward by the spec's skin thickness as a layer of
	// prism tetrahedra, exact at any thickness, thin regions solid), as a
	// closed mesh of the outer surface and the reversed inner surface; and
	// "Lattice", the infill sampled inside the shell's interior domain --
	// separate so a downstream analysis can give the shell and the infill
	// their own materials; their union is the part. The skin tetrahedra
	// are kept (GetShells) for export to an analysis. Throws
	// std::runtime_error with the reason when a target is gone, no target
	// has geometry, a target or reserved volume is not a closed outward
	// surface (the distance field's sign would be wrong around it), the
	// skin thickness is not positive, or the grid is too large.
	virtual std::vector<aerotelic::NamedMesh> BuildMeshes(double tolerance) const override;

	// The skin of each filled body as tetrahedra (built by the last
	// generation), with the body's scene label.
	struct FSkinShell
	{
		FString Label;
		aerotelic::ShellResult Shell;
	};
	const std::vector<FSkinShell>& GetShells() const { return Shells; }
	// All the shells' tetrahedra as one mesh (nodes and tets only), for
	// WriteShellVtk.
	aerotelic::ShellResult MergedShell() const;

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
	mutable std::vector<FSkinShell> Shells;
	mutable FStats Stats;
};
