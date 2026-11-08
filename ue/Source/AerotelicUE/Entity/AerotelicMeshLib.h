#pragma once

#include "CoreMinimal.h"
#include <atomic>
#include <vector>

#include "aerotelic/body_structure.hpp"
#include "aerotelic/mesh.hpp"
#include "aerotelic/vec3.hpp"

class FJsonObject;

// The MeshLib backend of a body's hollowing and ribs
// (tools/structure_meshlib.py), run as a separate process in the
// WingDesigner micromamba environment where MeshLib is installed with pip.
// The bodies travel as .tri files (the application's own indexed format,
// so the loft's shared vertices survive), and the skin, each body's inner
// offset and each network's ribs come back as .tri files with a
// result.json of statistics -- the same separate-process pattern as the
// STEP writer (Export/AerotelicExport.h). Until 2026-11-08 the same job
// also carried a sampled lattice field and returned an infill body; the
// lattice infill was removed from the application that day.
namespace AerotelicMeshLib
{
	// A placed, closed body with the name it is reported under. A target
	// may carry its own skin thickness and remesh edge (each > 0; 0 takes
	// the request's); a reserved volume carries neither.
	struct FBody
	{
		FString Label;
		const aerotelic::TriMesh* Mesh = nullptr;
		double SkinThickness = 0.0;
		double InnerRemeshEdge = 0.0;
		// A target's inward offset from an earlier run (the application's
		// cache of the hollow): when set and non-empty the script takes it
		// as it is instead of offsetting the body again.
		const aerotelic::TriMesh* PrecomputedInner = nullptr;
	};

	// A family of parallel planes cut from the hollow as ribs, spars or
	// frames (body_structure.hpp PlaneNetwork, placed): the planes in the
	// job's frame (the world's: the bodies are placed meshes), a thickness,
	// a height (0: wall to wall; else the ribs stop that far in from the
	// body's surface) and the slab's extent -- the slabs are boxes
	// 2 * HalfExtent across centred on Centre projected onto each plane,
	// so Centre and HalfExtent must cover the body.
	struct FNetwork
	{
		FString Label;
		TArray<aerotelic::Plane> Planes;
		double Thickness = 0.0;
		double Height = 0.0;
		aerotelic::Vec3 Centre;
		double HalfExtent = 0.0;
	};

	struct FRequest
	{
		TArray<FBody> Targets;
		TArray<FBody> Reserved;
		// The skin depth (> 0) of a target that carries none of its own.
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
		// (0: the decimated surface as it is); a target's own takes
		// precedence.
		double InnerRemeshEdge = 0.0;
		// The plane networks, in priority order (an earlier one is
		// continuous, a later one is notched by it).
		TArray<FNetwork> Networks;
	};

	// A target body's offset interior on its own: the inner boundary of
	// its skin, the hollow the ribs are cut from and the surface the
	// structural analysis lays its rib shells inside. Empty where the body
	// is all skin.
	struct FInner
	{
		FString Label;
		aerotelic::TriMesh Mesh;
		// Closed, outward, clear of the body's own surface (MeshLib's
		// collision test) and not crossing itself: usable as a bounding
		// surface downstream. Note says why not otherwise.
		bool bUsable = false;
		FString Note;
		double Volume = 0.0;
		int32 Components = 0;
	};

	// One network's ribs: the slabs cut to the hollow (less the networks
	// before it), closed and disjoint from the other layers. Empty, with
	// the reason in Note, when the planes miss the interior or MeshLib
	// refused the boolean.
	struct FRibs
	{
		FString Label;
		aerotelic::TriMesh Mesh;
		double Volume = 0.0;
		int32 Planes = 0;
		FString Note;
	};

	struct FResult
	{
		aerotelic::TriMesh Skin;
		// One per request target, in order.
		TArray<FInner> Inners;
		// One per request network, in order.
		TArray<FRibs> Ribs;
		double SkinVolume = 0.0;
		// The hollow's volume (the union of the offset interiors less the
		// reserved volumes), before the ribs are taken out of it.
		double DomainVolume = 0.0;
		double RibsVolume = 0.0;
		int32 Interiors = 0;
		TArray<FString> Warnings;
		double Seconds = 0.0;
	};

	// The Python interpreter to run the backend with: AEROTELIC_MESHLIB_PYTHON
	// if set, else the WingDesigner micromamba environment's python.exe.
	// Empty if none exists.
	FString FindPython();
	// tools/structure_meshlib.py under the project (staged with the packaged
	// build). Empty if missing.
	FString FindScript();

	// Runs the backend. False with the reason on failure (no interpreter, a
	// body that is not closed, a boolean MeshLib refused, ...).
	bool Run(const FRequest& Request, FResult& Out, FString& OutError);

	// What a caller may watch while a job runs (2026-11-05): every complete
	// line the script prints, as it prints it, on the thread RunJob is
	// called on (a worker thread when the dialog solves asynchronously);
	// and a flag that, once set from any thread, ends the process and makes
	// RunJob return false with "cancelled". Null means neither.
	struct FJobProgress
	{
		TFunction<void(const FString&)> OnLine;
		std::atomic<bool> bCancel{ false };
	};

	// Runs `Script` on a job directory in that interpreter, collects its
	// output into the log and hands back the `result.json` it wrote. False
	// with the reason the script gave (its "error" field, else its last
	// "error:" line, else its exit code). Shared by every backend that
	// works this way -- the MeshLib backend's own, the rib mesher
	// (AerotelicRibGmsh.h), the potential flow solver
	// (AerotelicPotentialFlow.h) and the structural solver -- so that the
	// process plumbing, the logging and the way a failure is reported are
	// in one place. Blocks the calling thread until the script exits (or
	// Progress->bCancel is set): callers that must stay responsive run it
	// on a worker thread and watch Progress.
	bool RunJob(const FString& Script, const FString& Dir, TSharedPtr<FJsonObject>& OutResult, FString& OutError, FJobProgress* Progress = nullptr);

	// The .tri files (exposed for tests).
	bool WriteTri(const aerotelic::TriMesh& Mesh, const FString& Path);
	bool ReadTri(const FString& Path, aerotelic::TriMesh& OutMesh, FString& OutError);
}
