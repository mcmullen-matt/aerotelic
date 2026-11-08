#pragma once

#include "CoreMinimal.h"
#include <array>
#include <vector>

#include "aerotelic/body_structure.hpp"
#include "aerotelic/mesh.hpp"

#include "AerotelicTetGmsh.h"

// One layer of a body's GENERATED structure: the skin shell, the infill
// lattice, or one network's ribs -- a closed triangle mesh in the body's
// own (parameterisation) frame, with the material it is made of. The
// layers of a body are disjoint, so their volumes add to the body's
// interior plus skin; the actor draws each as a section of its structure
// meshes, the Scene tree lists each under the body, STL export writes
// each as a `solid` and the mass report weighs each by its material.
struct FAerotelicStructureLayer
{
	enum class EKind : uint8 { Skin, Infill, Ribs };

	// "Skin", "Infill", or the network's name (made unique within the body).
	FString Name;
	EKind Kind = EKind::Skin;
	// The network this layer is the ribs of (Kind == Ribs), else INDEX_NONE.
	int32 NetworkIndex = INDEX_NONE;
	// Closed, outward, in the body's local frame (the parameterisation's,
	// at full scale): place it with the entity's transform to get the
	// world geometry, mirror it about Y = 0 for the mirrored half.
	aerotelic::TriMesh Mesh;
	aerotelic::Material Material;
	// The skin's or the ribs' thickness (project units); 0 for the infill.
	double Thickness = 0.0;
	// The layer's volume as MeshLib measured it, in PLACED (replica) units.
	double Volume = 0.0;
	// Why the layer is empty, when it is.
	FString Note;

	// The colour a layer is drawn in by default: the skin the body's own
	// colour (it IS the body's surface), the infill amber, ribs steel blue.
	static FLinearColor DefaultColor(EKind Kind, const FLinearColor& BodyColor);
	static const TCHAR* KindName(EKind Kind);
};

// The generated structure of one body (AAerotelicEntityActor's bodies
// hold one each once Generate has run): the skin shell, the infill and the
// ribs that a BodyStructure describes, built by the MeshLib job
// (AerotelicMeshLib.h, tools/lattice_meshlib.py) over the body's placed
// surface, and the skin / core tetrahedral mesh of the body for the
// analyses (gmsh through AerotelicTetGmsh, built on first request).
//
// This replaces the FAerotelicLatticeModel scene entity (2026-11-08): the
// structure is an attribute of the body, not a body of its own. The
// generation runs in WORLD space -- the skin thickness and the ribs'
// thicknesses are physical build figures, so they must be taken on the
// placed (replica) body -- and the results are brought back into the
// body's local frame, so the layers follow the body through a move or a
// rotation without a job; a change of model scale, like any change of
// shape, regenerates them (the document's rebuild queue).
class FAerotelicBodyStructure
{
public:
	struct FReserved
	{
		FString Label;
		aerotelic::TriMesh Mesh;   // placed, closed
	};

	struct FInputs
	{
		// "<entity>[ / <body>]" for the log and the notes.
		FString Label;
		// The body's placed (world) surface, closed and outward, fine enough
		// for the offset's distance field (BodyTolerance).
		aerotelic::TriMesh Body;
		aerotelic::BodyStructure Structure;
		// The placed volumes the interior structure leaves alone.
		std::vector<FReserved> Reserved;
		// The placement the planes are carried by and the results are
		// brought back through (AAerotelicEntityActor::GetPlacementMatrix).
		std::array<double, 9> R{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
		aerotelic::Vec3 T;
		double Scale = 1.0;
	};

	// What the last generation produced.
	struct FStats
	{
		int64 Triangles = 0;
		int64 GridNodes = 0;
		double SkinVolume = 0.0;
		double DomainVolume = 0.0;
		double InfillVolume = 0.0;
		double RibsVolume = 0.0;
		double AchievedFraction = 0.0;
		double Seconds = 0.0;
		TArray<FString> Warnings;
	};

	// The surface tolerance the body is meshed at for a structure: a
	// quarter of the skin, and no coarser than a quarter of the infill's
	// voxel. Placed units.
	static double BodyTolerance(const aerotelic::BodyStructure& S);

	// Generates the layers. False with the reason when the structure has no
	// skin (nothing to generate: a solid body), the body is not a closed
	// outward surface, the grid is too large, MeshLib is not installed or
	// a boolean failed. On failure the previous layers are kept.
	bool Generate(const FInputs& Inputs, FString& OutError);

	bool IsGenerated() const { return bGenerated; }
	const std::vector<FAerotelicStructureLayer>& GetLayers() const { return Layers; }
	const FAerotelicStructureLayer* FindLayer(const FString& Name) const;
	const FStats& GetStats() const { return Stats; }
	// The structure the layers were generated from.
	const aerotelic::BodyStructure& GetStructure() const { return Generated; }
	// Set when the body's geometry changed after the generation (the
	// document regenerates shortly; the Scene tree can say "stale").
	bool IsStale() const { return bStale; }
	void MarkStale() { bStale = true; }

	// The skin / core tetrahedral mesh of the body (material 1 = skin,
	// between the body's surface and MeshLib's offset; 2 = core, inside
	// it), WORLD space as generated, built on first request with gmsh and
	// kept until the next Generate. Null with the reason when the offset is
	// unusable or gmsh refuses the body. CoreSize is the core's target edge
	// (0: the infill's cell, else a few skins).
	struct FTetMesh
	{
		AerotelicTetGmsh::FResult Result;
		double SkinThickness = 0.0;
	};
	const FTetMesh* GetTetMesh(FString& OutNote, double CoreSize = 0.0) const;
	// The body's placed surface and inner offset the tetrahedra conform to.
	const aerotelic::TriMesh& GetOuter() const { return Outer; }
	const aerotelic::TriMesh& GetInner() const { return Inner; }
	bool IsInnerUsable() const { return bInnerUsable; }

private:
	bool bGenerated = false;
	bool bStale = false;
	aerotelic::BodyStructure Generated;
	std::vector<FAerotelicStructureLayer> Layers;
	FStats Stats;
	// The hollowing's surfaces (world space) and the figures the tetrahedral
	// mesh is built with.
	aerotelic::TriMesh Outer;
	aerotelic::TriMesh Inner;
	bool bInnerUsable = false;
	FString InnerNote;
	double MinPocketVolume = 0.0;
	double InfillCell = 0.0;
	double SkinThickness = 0.0;
	// The offset cache: the inner surface is reused when the body, the skin
	// and the voxel are what they were (editing the networks or the infill
	// then costs the booleans alone, not the distance field).
	uint64 InnerKey = 0;
	mutable bool bTetBuilt = false;
	mutable bool bTetOk = false;
	mutable FTetMesh Tet;
	mutable FString TetNote;
};
