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
		// Whether the size grows from the surfaces' own triangle sizes
		// (gmsh's Mesh.MeshSizeExtendFromBoundary; the default) or is
		// CoreSize everywhere inside, graded to the surface only by the
		// Delaunay insertion itself. Keep it ON for a solver's mesh: tried
		// off for the homogenised core on 2026-11-05 (fewer tetrahedra, and
		// on a sphere no worse), it put a fan of slivers along the whole
		// skin of a fuselage whose 5 cm surface edges met a 60 cm cell --
		// 12% of the body's tetrahedra with a dihedral angle under 10
		// degrees against 2% seeded, the median smallest dihedral 26
		// against 45 degrees -- for 33k tetrahedra against 83k. Quality
		// beats count; the DOF budget is the Mesh tab's business.
		bool bSizeFromBoundary = true;
		// > 0: a graded size field instead (tools/tet_gmsh.py `grading`):
		// the surface's median edge at the surface, growing at this rate
		// per unit distance until it reaches CoreSize. Measured on the same
		// fuselage: 0.25 gives 63k tetrahedra at 7.4% under 10 degrees
		// (median 39), 0.5 gives 50k at 9.6% (median 33), 1.0 gives 43k at
		// 11.6% (median 30) -- each a step down from seeded, so it is an
		// option, not the default.
		double Grading = 0.0;
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

		// Ribs (2026-11-08): planar shells inside the volume, one per plane
		// of a body's networks. Outer must already have vertices along each
		// plane (core mesh_cut.hpp CutMeshByPlanes), so the rib's rim is the
		// surface's own nodes and the rib shells share them with the skin
		// shells; each disc walls the body into compartments the tetrahedra
		// conform to. Height > 0 reports only the part of the rib within that
		// distance of its rim. Ribs and an Inner offset together are refused.
		struct FRib
		{
			int32 Id = 0;
			aerotelic::Vec3 Normal{0.0, 1.0, 0.0};   // unit, scene frame
			double Offset = 0.0;                     // Normal . x = Offset
			double Height = 0.0;
		};
		TArray<FRib> Ribs;
		// Target edge length inside the ribs (0: the surface's median edge).
		double RibSize = 0.0;
		// A vertex within this of a plane is on it (0: 1e-7 of the diagonal).
		double PlaneTolerance = 0.0;
		// False: no tetrahedra at all, the rib discs alone (a thin-walled
		// body of shells with ribs). The mesh's nodes are then the surface's
		// followed by the ribs' interior nodes.
		bool bVolume = true;
	};

	// One rib shell: its nodes (into FResult::Mesh's nodes) and the rib it
	// belongs to.
	struct FRibFace
	{
		int32 N[3] = { 0, 0, 0 };
		int32 Id = 0;
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
		// The dihedral angles, degrees, and the tetrahedra outside the
		// usual acceptance band for a linear element (Shewchuk 2002: a
		// small dihedral wrecks the conditioning, a large one the
		// gradient's accuracy). AerotelicFeaPreprocess computes the same
		// over whatever mesh the solver is handed; these are gmsh's run.
		double MinDihedral = 0.0, MaxDihedral = 0.0;
		int32 TetsDihedralUnder10 = 0, TetsDihedralOver160 = 0;
		int32 Flipped = 0;       // handed back the other way round, swapped here
		int32 Degenerate = 0;    // zero volume even after that
		// The rib shells, their area (project units^2), and how many of them
		// are NOT a face of exactly two tetrahedra (0 when the embedding
		// conformed; the script retries once without Netgen when it did not).
		TArray<FRibFace> RibFaces;
		double RibArea = 0.0;
		int32 RibFacesMissing = 0;
		TArray<FString> Notes;
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
