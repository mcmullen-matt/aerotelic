#pragma once

#include "CoreMinimal.h"
#include <atomic>

#include "aerotelic/mesh.hpp"

class FJsonObject;

// The Python job runner shared by the backends. Every Python backend of
// the application -- the rib mesher (AerotelicRibGmsh.h, gmsh), the
// potential flow solver (AerotelicPotentialFlow.h, Kratos) and the
// structural solver (AerotelicStructuralFea.h, Kratos) -- works the same
// way: a job directory with a job.json and the bodies as .tri files (the
// application's own indexed format, so the loft's shared vertices
// survive), a script run as a separate process in the WingDesigner
// micromamba environment, and a result.json read back -- the same
// separate-process pattern as the STEP writer (Export/AerotelicExport.h).
// This file holds that plumbing: finding the interpreter, writing and
// reading .tri, running the script with its output in the log and a
// cancel flag.
//
// History: until the night of 2026-11-08 this was the MeshLib backend of
// a body's hollowing (tools/structure_meshlib.py: the inward offset of the
// skin, the ribs as slabs cut from the hollow), with the job runner
// alongside; the owner decided that night that there are no offset
// surfaces anywhere -- a skin is the body's surface with a thickness, a
// rib a planar disc -- and the hollowing went (AerotelicBodyStructure.h).
// The namespace keeps its name, as does the AEROTELIC_MESHLIB_PYTHON
// variable users have set.
namespace AerotelicMeshLib
{
	// The Python interpreter to run a backend with: AEROTELIC_MESHLIB_PYTHON
	// if set, else the WingDesigner micromamba environment's python.exe.
	// Empty if none exists.
	FString FindPython();

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
	// "error:" line, else its exit code). Blocks the calling thread until
	// the script exits (or Progress->bCancel is set): callers that must
	// stay responsive run it on a worker thread and watch Progress.
	bool RunJob(const FString& Script, const FString& Dir, TSharedPtr<FJsonObject>& OutResult, FString& OutError, FJobProgress* Progress = nullptr);

	// The .tri files: "AEROTRI1", uint32 vertex and triangle counts, float64
	// xyz per vertex, int32 abc per triangle (exposed for tests).
	bool WriteTri(const aerotelic::TriMesh& Mesh, const FString& Path);
	bool ReadTri(const FString& Path, aerotelic::TriMesh& OutMesh, FString& OutError);
}
