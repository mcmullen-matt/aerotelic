#pragma once

#include "CoreMinimal.h"

// Analysis -> TACS (FEA) -> Check Installation (2026-11-17): is TACS
// reachable from the application? Looks for the TACS environment again
// (AerotelicMeshLib::FindTacsInterpreter(true): AEROTELIC_TACS_PYTHON, else
// the default WSL distribution's ~/micromamba/envs/tacs), then runs
// tools/tacs_check.py in it on a worker thread -- the interpreter, the
// platform, the versions of tacs, mpi4py, pyNastran and numpy, and a
// two-triangle cantilever solved through the real solver path with its
// time -- and shows the report in a modeless window, or the reason
// nothing could run with the installation recipe. Status receives the
// status-bar line.
void ShowAerotelicTacsCheckDialog(TFunction<void(const FText&)> Status);
