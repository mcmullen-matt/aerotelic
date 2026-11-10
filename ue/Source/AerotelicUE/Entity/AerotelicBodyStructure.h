#pragma once

#include "CoreMinimal.h"
#include <array>
#include <vector>

#include "aerotelic/body_structure.hpp"
#include "aerotelic/mesh.hpp"
#include "aerotelic/wing_model.hpp"

// One layer of a body's GENERATED structure: its skin, or one network's
// ribs. Every layer is a THIN SHELL (2026-11-08, evening): the skin is
// the body's own surface with the skin thickness as an attribute -- the
// Skin layer carries NO mesh of its own, the body's surface stands in --
// and a Ribs layer is the planar discs the network's planes cut out of
// the body, bounded by the body's surface, as an OPEN triangle mesh in the
// body's local (parameterisation) frame. A layer's material volume is its
// area times its thickness and its mass that times the density; the
// viewport draws the ribs as sections of the actor's structure meshes
// (both sides), the Scene tree lists each layer under the body, STL
// export writes the ribs' triangles, and the mass report weighs each as a
// shell.
struct FAerotelicStructureLayer
{
	enum class EKind : uint8 { Skin, Ribs, Spars, Stringers };

	// "Skin", or the network's name (made unique within the body).
	FString Name;
	EKind Kind = EKind::Skin;
	// The network this layer is the ribs of (Kind == Ribs), else INDEX_NONE.
	int32 NetworkIndex = INDEX_NONE;
	// Ribs: the discs, open, in the body's local frame (the
	// parameterisation's, at full scale): place it with the entity's
	// transform for the world geometry, mirror it about Y = 0 for the
	// mirrored half. Skin: EMPTY -- the body's surface is the skin.
	aerotelic::TriMesh Mesh;
	aerotelic::Material Material;
	// The skin's or the ribs' thickness (project units).
	double Thickness = 0.0;
	// The shell's area in PLACED (replica) units: the body's surface area
	// for the skin, the discs' summed triangle area for ribs.
	double Area = 0.0;
	// The shell's MATERIAL volume, Area x Thickness, placed units (the name
	// predates the shell model, when it was an enclosed volume).
	double Volume = 0.0;
	// Why the layer is empty, when it is, and what the mesher had to say
	// about its ribs.
	FString Note;

	// The colour a layer is drawn in by default: the skin the body's own
	// colour (it IS the body's surface), ribs steel blue.
	static FLinearColor DefaultColor(EKind Kind, const FLinearColor& BodyColor);
	static const TCHAR* KindName(EKind Kind);
};

// The generated structure of one body (AAerotelicEntityActor's bodies
// hold one each once Generate has run): the skin and the ribs a
// BodyStructure describes, as shells.
//
// WHY THERE ARE NO OFFSET SURFACES (the owner's decision, 2026-11-08
// night). Until that evening the generation hollowed the body -- MeshLib
// offset the surface inward by the skin and cut rib slabs from the hollow
// as closed solids, with a distance field, booleans and a remesh in the
// way (tools/structure_meshlib.py, gone). Now a body's skin is its own
// surface with the thickness as an attribute, and a rib is the planar
// disc its plane cuts out of the body, bounded by the body's surface and
// not by an inner offset: mass = area x thickness x density, the same
// shell model the structural analysis solves (AerotelicFeaModelBuilder
// builds the same discs on the same cut surface, with the same mesher).
// Two errors come with that and are accepted: the ribs overlap the skin
// along their rims (a strip half a skin wide, both counted), and the skin's
// area is taken on the outer surface rather than its mid-surface -- a few
// percent at most for the thicknesses in question.
//
// HOW. The body's placed surface (its OWN grid, the one the viewport shows
// and the analysis solves on) is cut by every network's placed planes
// (core mesh_cut.hpp CutMeshByPlanes), so each plane's section is a loop
// of the surface's own vertices; the reserved volumes the structure's
// exclusions name are cut by the same planes; gmsh (AerotelicRibGmsh.h,
// tools/rib_gmsh.py) triangulates the disc inside each loop, with the
// reserved volumes' sections as holes, and the faces come back labelled by
// rib id (network index + 1). Under a second for a wing. The generation
// runs in WORLD space -- the thicknesses are physical build figures, so
// the planes are placed with the body -- and the discs are brought back
// into the body's local frame, so they follow the body through a move or a
// rotation without a job; a change of model scale, like any change of
// shape, regenerates them (the document's rebuild queue).
//
// History: this replaced the FAerotelicLatticeModel scene entity on
// 2026-11-08 (the structure is an attribute of the body, not a body of its
// own); the same day the lattice infill and the tetrahedral mesh went, and
// that night the hollowing.
class FAerotelicBodyStructure
{
public:
	struct FInputs
	{
		// "<entity>[ / <body>]" for the log and the notes.
		FString Label;
		// The body's placed surface on its OWN grid (GetPlacedMeshes at
		// tolerance 0), closed and outward.
		aerotelic::TriMesh Body;
		aerotelic::BodyStructure Structure;
		// The placed volumes the ribs leave alone (own grids too): each
		// closed; one that is not is skipped with a warning.
		std::vector<aerotelic::TriMesh> Reserved;
		// The placement the planes are carried by and the results are
		// brought back through (AAerotelicEntityActor::GetPlacementMatrix).
		std::array<double, 9> R{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
		aerotelic::Vec3 T;
		double Scale = 1.0;
	};

	// What the last generation produced (placed units).
	struct FStats
	{
		// The ribs' triangles (the skin has none of its own).
		int64 Triangles = 0;
		double SkinArea = 0.0;
		double RibsArea = 0.0;
		double Seconds = 0.0;
		// The mesher's notes and anything skipped on the way.
		TArray<FString> Warnings;
	};

	// Generates the layers. False with the reason when the structure has no
	// skin (nothing to generate: a solid body), the body is not a closed
	// outward surface, the cut surface is not a closed manifold, or the rib
	// mesher is missing or refused. On failure the previous layers are kept.
	bool Generate(const FInputs& Inputs, FString& OutError);

	// A WING'S structure (2026-11-09): the plan its parameterisation made
	// on the body's own grid (wing_model.hpp WingBodyStructurePlan) -- rib
	// rims that are rings of the body mesh, spar webs and stringers as
	// ready meshes -- turned into layers: Skin, one Ribs layer holding every
	// rib (meshed as discs by the rib mesher, the spar chords across them),
	// one layer per spar and per stringer. Everything is in the body's
	// LOCAL frame, as the plan is; areas are reported in placed units.
	struct FPlanInputs
	{
		FString Label;
		// The body's mesh in its own frame (the model's triangles, the ones
		// the plan's vertex ids index).
		aerotelic::TriMesh Body;
		aerotelic::WingBodyStructurePlan Plan;
		// The body's material and skin thickness; an element whose material
		// has no density takes the body's.
		aerotelic::BodyStructure Structure;
		// The model scale, so a layer's area is reported in placed units.
		double Scale = 1.0;
	};
	bool GenerateFromPlan(const FPlanInputs& Inputs, FString& OutError);

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

private:
	bool bGenerated = false;
	bool bStale = false;
	aerotelic::BodyStructure Generated;
	std::vector<FAerotelicStructureLayer> Layers;
	FStats Stats;
};
