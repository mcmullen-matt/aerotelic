#pragma once

#include "CoreMinimal.h"

#include "aerotelic/mesh.hpp"
#include "aerotelic/tet.hpp"

// The skin / core tetrahedral mesh of a body, on gmsh (tools/tet_gmsh.py,
// run as a separate process in the WingDesigner micromamba environment
// where gmsh is pip-installed -- the same bridge pattern as the lattice's
// MeshLib backend, AerotelicMeshLib.h, and the STEP writer).
//
// The two surfaces travel as .tri files: the body's own triangulation and
// MeshLib's inward offset of it at the skin depth. gmsh takes both as
// DISCRETE entities and meshes the two volumes bounded by them, so it adds
// interior nodes only and never re-triangulates a boundary. That is what
// makes the outer faces of the skin tetrahedra the body's own triangles
// exactly, the skin/core interface the offset's own triangles, and
// skin + core volume = the body's enclosed volume to rounding.
//
// It replaces core/src/tet.cpp's constrained Delaunay mesher for this job
// (2026-10-15). That one recovered the surfaces into a Delaunay
// tetrahedralization of their vertices, which cascaded wherever the two
// surfaces were closer together than their own triangles are wide: on
// p38a's hollowed airframe it abandoned 295,157 of about 414,580
// constraint faces, and the exported boundary was then not the outer
// mould line. gmsh meshes the same pair exactly.
namespace AerotelicTetGmsh
{
	struct FRequest
	{
		// The body's surface: closed, outward, in the scene's placement.
		const aerotelic::TriMesh* Outer = nullptr;
		// The offset surface strictly inside it; null or empty makes the
		// whole body skin.
		const aerotelic::TriMesh* Inner = nullptr;
		// Target edge length deep inside the body. 0 lets gmsh grow the
		// size from the surfaces alone, which on a finely remeshed offset
		// means millions of tetrahedra in the core.
		double CoreSize = 0.0;
		// Floor on the edge length (0: none).
		double MinSize = 0.0;
		// Pockets of the offset smaller than this are filled solid rather
		// than meshed as core: a remesh leaves slivers, and gmsh refuses a
		// whole body over one bad shell.
		double MinPocketVolume = 0.0;
		// gmsh's Mesh.Algorithm3D: 1 Delaunay, 10 HXT.
		int32 Algorithm = 1;
		// gmsh's own quality optimisation pass, and Netgen's after it.
		// Netgen roughly triples the time and takes the worst element of
		// p38a's airframe from 0.0001 to 0.0030: worth it for a mesh that
		// goes into a solver.
		bool bOptimize = true;
		bool bOptimizeNetgen = true;
	};

	struct FResult
	{
		// material 1 = skin (between the two surfaces), 2 = core (inside
		// the offset). Neighbours are rebuilt; `constrained` is not set
		// (gmsh keeps the boundary triangles, so the mesh boundary is the
		// surfaces by construction rather than by a per-face flag).
		aerotelic::TetMesh Mesh;
		int32 Nodes = 0;
		int32 SkinTets = 0, CoreTets = 0;
		double OuterVolume = 0.0, InnerVolume = 0.0;
		double SkinVolume = 0.0, CoreVolume = 0.0;
		// |skin + core - body| / body, and |core - offset| / body: the
		// conformity check, zero to rounding when all is well.
		double VolumeError = 0.0, CoreVolumeError = 0.0;
		// Signed inverse condition number: 1 is the regular tetrahedron,
		// 0 is flat, negative is inverted.
		double MinQuality = 0.0, MeanQuality = 0.0;
		int32 PoorQuality = 0;   // under 0.1
		int32 Flipped = 0;       // handed back the other way round, swapped here
		int32 Degenerate = 0;    // zero volume even after that
		double Seconds = 0.0;
	};

	// tools/tet_gmsh.py under the project (staged with the packaged
	// build). Empty if missing.
	FString FindScript();

	// Runs the mesher. False with the reason on failure (no interpreter,
	// no gmsh, a surface that is not closed, an offset that is not inside
	// the body, a volume gmsh could not fill).
	bool Run(const FRequest& Request, FResult& Out, FString& OutError);
}
