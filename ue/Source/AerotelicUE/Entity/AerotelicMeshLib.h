#pragma once

#include "CoreMinimal.h"
#include <vector>

#include "aerotelic/mesh.hpp"
#include "aerotelic/vec3.hpp"

// The MeshLib backend of the lattices' hollowing and infill
// (tools/lattice_meshlib.py), run as a separate process in the
// WingDesigner micromamba environment where MeshLib is installed with pip.
// The bodies travel as .tri files (the application's own indexed format,
// so the loft's shared vertices survive), the lattice field as raw
// float32 samples on the generator's grid, and the two result bodies come
// back as .tri files with a result.json of statistics -- the same
// separate-process pattern as the STEP writer (Export/AerotelicExport.h).
namespace AerotelicMeshLib
{
	// A placed, closed body with the name it is reported under.
	struct FBody
	{
		FString Label;
		const aerotelic::TriMesh* Mesh = nullptr;
	};

	// The lattice field sampled on a regular grid: node (i, j, k) at
	// Origin + H * (i, j, k), Values[(k * Ny + j) * Nx + i] <= 0 inside the
	// lattice material.
	struct FField
	{
		int32 Nx = 0, Ny = 0, Nz = 0;
		aerotelic::Vec3 Origin;
		double H = 0.0;
		std::vector<float> Values;
	};

	struct FRequest
	{
		TArray<FBody> Targets;
		TArray<FBody> Reserved;
		// The skin depth (> 0).
		double SkinThickness = 0.0;
		// Voxel size of the distance field behind the inward offset.
		double OffsetVoxel = 0.0;
		// "standard" (dual marching cubes), "smooth" or "sharpening".
		FString OffsetMode = TEXT("standard");
		// The offset surface is decimated within this of itself (0: not).
		double DecimateTolerance = 0.0;
		// Pockets of the offset interior smaller than this are filled solid.
		double MinPocketVolume = 0.0;
		// The offset interior is opened by this radius (a double offset:
		// parts thinner than twice it are made solid, so the core is
		// nowhere thinner than that; 0: no opening).
		double InnerOpening = 0.0;
		// The offset surface is remeshed to isotropic triangles of about
		// this edge length before it is returned as a body's inner surface
		// (0: the decimated surface as it is).
		double InnerRemeshEdge = 0.0;
		// The infill's field; null for a hollow part with no infill.
		const FField* Field = nullptr;
	};

	// A target body's offset interior on its own: the inner boundary of
	// its skin, what the tetrahedral mesher (tet.hpp TetrahedralizeNested)
	// conforms to inside. Empty where the body is all skin.
	struct FInner
	{
		FString Label;
		aerotelic::TriMesh Mesh;
		// Closed, outward and clear of the body's own surface (MeshLib's
		// collision test): usable as the mesher's inner constraint. Note
		// says why not otherwise.
		bool bUsable = false;
		FString Note;
		double Volume = 0.0;
		int32 Components = 0;
	};

	struct FResult
	{
		aerotelic::TriMesh Skin;
		aerotelic::TriMesh Lattice;
		// One per request target, in order.
		TArray<FInner> Inners;
		double SkinVolume = 0.0;
		double DomainVolume = 0.0;
		double LatticeVolume = 0.0;
		// Lattice volume over the infill domain's.
		double AchievedFraction = 0.0;
		int32 Interiors = 0;
		TArray<FString> Warnings;
		double Seconds = 0.0;
	};

	// The Python interpreter to run the backend with: AEROTELIC_MESHLIB_PYTHON
	// if set, else the WingDesigner micromamba environment's python.exe.
	// Empty if none exists.
	FString FindPython();
	// tools/lattice_meshlib.py under the project (staged with the packaged
	// build). Empty if missing.
	FString FindScript();

	// Runs the backend. False with the reason on failure (no interpreter, a
	// body that is not closed, a boolean MeshLib refused, ...).
	bool Run(const FRequest& Request, FResult& Out, FString& OutError);

	// The .tri files (exposed for tests).
	bool WriteTri(const aerotelic::TriMesh& Mesh, const FString& Path);
	bool ReadTri(const FString& Path, aerotelic::TriMesh& OutMesh, FString& OutError);
}
