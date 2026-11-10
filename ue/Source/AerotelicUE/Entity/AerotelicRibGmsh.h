#pragma once

#include "CoreMinimal.h"

#include <vector>

#include "aerotelic/mesh.hpp"
#include "aerotelic/vec3.hpp"

// The RIB SHELLS of a body, on gmsh (tools/rib_gmsh.py, run as a separate
// process in the WingDesigner micromamba environment where gmsh is
// pip-installed -- the same bridge pattern as the other Python backends
// over AerotelicMeshLib.h's job runner, and the STEP writer).
//
// A body's structure (body_structure.hpp) holds PLANE NETWORKS -- ribs,
// spars, frames: families of parallel planes, each with a material and a
// thickness. The structural model (AerotelicFeaModelBuilder.h) is shells
// only: the skin on the body's own grid, and one planar disc per plane,
// its rim on the skin's nodes so the two share their degrees of freedom.
// This is the mesher of those discs -- and since the evening of
// 2026-11-08 also the mesher of the discs a body's Generate Structure
// shows and weighs (AerotelicBodyStructure.h): the viewport, the mass
// report and the analysis build the same ribs.
//
// RESERVED VOLUMES (a payload bay the ribs must leave alone) travel with
// the request as Holes: closed surfaces, cut by the same planes as the
// body, whose section loops become holes of the discs they fall inside
// (see the script's docstring for the cases it handles and the notes it
// leaves).
//
// The surface arrives ALREADY CUT by every plane (core mesh_cut.hpp
// CutMeshByPlanes), so each plane's section of it is a closed loop (or
// several) of the surface's own vertices. For each plane the script
// collects those rim loops, builds a scratch gmsh model of the planar
// face(s) they bound, and triangulates it with the rim fixed -- gmsh adds
// nodes INSIDE the disc only and never moves or re-triangulates a
// boundary. That is what makes the rib's rim exactly the skin's nodes.
// Where two networks cross (a rib through a spar) the discs share the
// chord along which their planes meet: the chord's nodes are made once
// and both discs are built against them, so the ribs are joined to one
// another as they are to the skin. Height > 0 keeps only the part of a
// disc within that distance of its rim (a rib flange rather than a full
// web).
//
// The result's nodes are the SURFACE'S VERTICES FIRST, in their order,
// then the ribs' interior nodes: the caller's shell faces index the first
// block as they index the surface, and the rib faces index both. The
// model builder checks that ordering (SurfaceNodesFirst) rather than
// trusting it.
//
// History: this grew out of tools/tet_gmsh.py and AerotelicTetGmsh, the
// skin / core tetrahedral mesher (2026-10-15) that filled a body between
// its surface and MeshLib's inward offset, and later embedded the ribs in
// those tetrahedra. The application dropped tetrahedral FEA and lattice
// infill on 2026-11-08 -- every structural element is a shell -- and the
// mesher was cut down to the ribs alone and renamed. Its sizing,
// optimisation and dihedral-quality options went with the tetrahedra.
namespace AerotelicRibGmsh
{
	struct FRequest
	{
		// The body's surface: closed, outward, in the scene's placement, and
		// already cut by every rib plane so the rims are its own vertices.
		const aerotelic::TriMesh* Outer = nullptr;

		// One rib per plane of a body's networks. Id is the caller's (the
		// model builder uses network index + 1 within the body) and comes
		// back on every face of the disc.
		struct FRib
		{
			int32 Id = 0;
			aerotelic::Vec3 Normal{0.0, 1.0, 0.0};   // unit, scene frame
			double Offset = 0.0;                     // Normal . x = Offset
			double Height = 0.0;                     // > 0: only within this of the rim
			// An EXPLICIT rim (2026-11-09, a wing's rib on its own ring): the
			// closed loop of Outer's vertex ids round the disc, the loops of
			// its holes (2026-11-10, a nacelle frame's inner skin), and the
			// chords across it (two rim vertices and the node count across, a
			// spar's web). With a rim the plane is read off it; Normal and
			// Offset are not sent. Height applies as for a plane.
			TArray<int32> Rim;
			TArray<TArray<int32>> Holes;
			TArray<FIntVector> Chords;
		};
		TArray<FRib> Ribs;
		// Target edge length inside the discs (0: the surface's median edge).
		double RibSize = 0.0;
		// A vertex within this of a plane is on it (0: 1e-7 of the diagonal).
		double PlaneTolerance = 0.0;
		// Reserved volumes (closed surfaces, ALREADY CUT by the same planes
		// as Outer, in the same frame): their section loops on a rib's plane
		// are meshed as holes of that disc where they lie inside it. Written
		// as hole_<i>.tri and listed in job.json as "holes"; nothing is
		// written or listed when empty. Null entries are skipped.
		TArray<const aerotelic::TriMesh*> Holes;
	};

	// One rib shell: its nodes (into FResult::Nodes) and the rib it belongs
	// to (FRib::Id).
	struct FRibFace
	{
		int32 N[3] = { 0, 0, 0 };
		int32 Id = 0;
	};

	struct FResult
	{
		// The surface's vertices first, in order, then the ribs' interior
		// nodes. Scene placement, project units.
		std::vector<aerotelic::Vec3> Nodes;
		// How many of Nodes are the surface's (its vertex count when all is
		// well; the model builder checks the coordinates too).
		int32 SurfaceNodes = 0;
		// The body's enclosed volume as gmsh measured it, project units^3 --
		// a sanity figure, not used by the model.
		double OuterVolume = 0.0;
		// The rib shells and their area (project units^2).
		TArray<FRibFace> RibFaces;
		double RibArea = 0.0;
		// How many reserved-volume loops were meshed as holes of the discs,
		// over all ribs (the script's "holes"; 0 when none were sent).
		int32 Holes = 0;
		// What the script had to say per rib (a plane that misses the body,
		// a loop it could not close, a disc it trimmed to its height, a
		// reserved volume it could not take as a hole). Each begins
		// "rib <id>: ", so a caller can file it under the rib.
		TArray<FString> Notes;
		double Seconds = 0.0;
	};

	// tools/rib_gmsh.py under the project (staged with the packaged
	// build). Empty if missing.
	FString FindScript();

	// Runs the mesher. False with the reason on failure (no interpreter,
	// no gmsh, a surface that is not closed, a plane whose section is not a
	// closed loop of the surface's vertices).
	bool Run(const FRequest& Request, FResult& Out, FString& OutError);
}
