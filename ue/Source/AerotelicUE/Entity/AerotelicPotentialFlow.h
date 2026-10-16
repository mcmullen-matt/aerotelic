#pragma once

#include "CoreMinimal.h"

#include "aerotelic/mesh.hpp"

// The potential flow around a selected set of bodies, on
// KratosMultiphysics' CompressiblePotentialFlowApplication
// (tools/potential_flow.py, run as a separate process in the WingDesigner
// micromamba environment where Kratos is pip-installed -- the same bridge
// pattern as the lattice's MeshLib backend, AerotelicMeshLib.h, the
// tetrahedral mesher, AerotelicTetGmsh.h, and the STEP writer).
//
// The bodies travel as .tri files: the application's own triangulation,
// placed as it sits in the scene. The script builds the FLUID domain
// around them -- a far-field box with the bodies as holes, meshed by gmsh
// with the bodies as DISCRETE surfaces, so the wetted boundary of the
// fluid mesh is the designer's own surface and the pressures come out on
// it -- solves for the velocity potential, and writes the grid and the
// solution to the VTK file the user named.
//
// LIMITATION: with no wake surface the circulation around a lifting
// section is zero, so this is the NON-LIFTING potential flow. The
// velocity field and the pressure distribution are real; the lift is zero
// by construction, and so (d'Alembert's paradox) is the net force -- which
// is why the force that comes back is reported as an error measure rather
// than as a load.
namespace AerotelicPotentialFlow
{
	// A placed, closed body with the name it is reported under. Several
	// closed shells in one mesh (a propeller) are allowed.
	struct FBody
	{
		FString Label;
		const aerotelic::TriMesh* Mesh = nullptr;
	};

	struct FRequest
	{
		TArray<FBody> Bodies;

		// --- the free stream (model units; 1 unit = 1 m for the pressures
		// to be pascals and the forces newtons) ---
		double Speed = 50.0;
		double AngleOfAttackDeg = 0.0;
		// Only the ratio to the speed matters, and only to the compressible
		// formulation; it is what Kratos is given as the Mach number.
		double SpeedOfSound = 340.0;
		double Density = 1.225;
		// "incompressible" (one linear solve) or "compressible".
		FString ElementType = TEXT("incompressible");

		// --- the domain ---
		// The far-field box's half-extent, in diagonals of the bodies'
		// bounding box.
		double FarFieldFactor = 5.0;
		// Element size on the box (0: a tenth of that half-extent).
		double FarElementSize = 0.0;
		// Element size at the bodies (0: the mean edge of their own
		// triangles, which is what sets how well the surface pressures are
		// resolved).
		double NearElementSize = 0.0;
		// How far that size is held before it grows (0: twice the size),
		// and how much size is added per unit distance after that (0: 0.3).
		double NearDistance = 0.0;
		double GrowthRate = 0.0;

		// Where to write the solution.
		FString VtkPath;
	};

	struct FResult
	{
		int32 Nodes = 0, Tets = 0;
		int32 WettedTriangles = 0, FarFieldTriangles = 0;
		double BodyVolume = 0.0, WettedVolume = 0.0, FluidVolume = 0.0, BoxVolume = 0.0;
		double FarFieldSize = 0.0, NearElementSize = 0.0, FarElementSize = 0.0;
		// Signed inverse condition number of the worst / mean element.
		double MinQuality = 0.0, MeanQuality = 0.0;
		int32 PoorQuality = 0;

		double FreeStreamSpeed = 0.0, MaxSpeed = 0.0;
		// Over the whole field, and on the bodies alone.
		double MinCp = 0.0, MaxCp = 0.0;
		double MinCpBody = 0.0, MaxCpBody = 0.0;
		// The integral of -Cp n dA over the bodies, and its size against
		// the dynamic pressure on their frontal area: zero in an exact
		// potential flow, so this is the discretisation error.
		double Force[3] = {0.0, 0.0, 0.0};
		double ForceCoefficient = 0.0, FrontalArea = 0.0;

		double Seconds = 0.0, SecondsMesh = 0.0, SecondsSolve = 0.0;
		FString VtkPath;
	};

	// tools/potential_flow.py under the project (staged with the packaged
	// build). Empty if missing.
	FString FindScript();

	// Runs the solver. False with the reason on failure (no interpreter,
	// no Kratos, a body that is not closed, bodies that overlap each other
	// so there is no fluid domain between them, a volume gmsh could not
	// fill, a solve that did not converge).
	bool Run(const FRequest& Request, FResult& Out, FString& OutError);
}
