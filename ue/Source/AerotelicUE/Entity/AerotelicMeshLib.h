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

	// An interpreter a backend runs in (2026-11-17): a Windows python.exe,
	// or a Linux python inside the Windows Subsystem for Linux -- TACS has
	// no Windows build, so Analysis > TACS (FEA) runs its script through
	// wsl.exe -d <Distro> -e <Python> with the job directory's path mapped
	// to /mnt/<drive>/... (ToWslPath). Distro empty = WSL's default
	// distribution; bWsl false = a native python.exe.
	struct FInterpreter
	{
		FString Python;
		FString Distro;
		bool bWsl = false;
		bool IsValid() const { return !Python.IsEmpty(); }
		// "wsl:<distro>:<python>" or the python.exe path, for messages.
		FString Describe() const;
	};

	// The TACS environment's interpreter: AEROTELIC_TACS_PYTHON if set --
	// "wsl:<distro>:/path/to/python" (distro may be empty for the default),
	// or a Windows python.exe that has tacs -- else the default WSL
	// distribution's ~/micromamba/envs/tacs/bin/python (or miniforge3 /
	// miniconda3 / mambaforge), found by asking WSL once and remembered;
	// bRefresh asks again (Check Installation). Invalid if none.
	FInterpreter FindTacsInterpreter(bool bRefresh = false);

	// C:\Users\x\y -> /mnt/c/Users/x/y (relative paths are made full first).
	FString ToWslPath(const FString& WindowsPath);

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
	// The same in a given interpreter (a WSL one maps the script and the
	// job directory to /mnt paths and runs python -u for live output).
	bool RunJob(const FInterpreter& Interpreter, const FString& Script, const FString& Dir, TSharedPtr<FJsonObject>& OutResult, FString& OutError, FJobProgress* Progress = nullptr);

	// The .tri files: "AEROTRI1", uint32 vertex and triangle counts, float64
	// xyz per vertex, int32 abc per triangle (exposed for tests).
	bool WriteTri(const aerotelic::TriMesh& Mesh, const FString& Path);
	bool ReadTri(const FString& Path, aerotelic::TriMesh& OutMesh, FString& OutError);
}
