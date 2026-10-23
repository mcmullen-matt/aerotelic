#pragma once

#include "CoreMinimal.h"

#include "aerotelic/materials.hpp"
#include "aerotelic/tet.hpp"
#include "aerotelic/vec3.hpp"

// A linear static finite-element analysis of a lattice-filled body's
// skin / core tetrahedra, on KratosMultiphysics' StructuralMechanics
// application (tools/structural_fea.py, run as a separate process in the
// WingDesigner micromamba environment -- the same bridge pattern as the
// potential flow, AerotelicPotentialFlow.h).
//
// The mesh is the one Lattice -> Export Tetrahedral Mesh writes: gmsh's
// skin and core tetrahedra, whose outer faces are the body's own surface
// triangles exactly. Each tetrahedron carries a material id; the request
// says what each id is made of (the skin its body's material, the core
// its lattice's, from the Scene tree's Material Properties). Everything
// is handed to the solver in SI: the mesh scaled by LengthToSI, the
// materials as they are (materials.hpp is SI), the loads in newtons and
// pascals. What comes back is SI too; the dialog converts for display.
//
// PHASE 1: dummy loads -- an inertial load factor, a uniform pressure on
// the wetted surface, a point force -- and a support on a coordinate
// plane. The job takes the pressure per wetted face already, so a
// pressure field from the potential flow or DUST (phase 2) is a file in
// place of a constant.
namespace AerotelicStructuralFea
{
	// What the tetrahedra of one material id are made of.
	struct FMaterialRegion
	{
		int32 Id = 0;
		FString Label;
		aerotelic::Material Material;   // SI
	};

	struct FRequest
	{
		// Skin / core tetrahedra in PROJECT units, material id per tetrahedron.
		const aerotelic::TetMesh* Mesh = nullptr;
		double LengthToSI = 1.0;
		TArray<FMaterialRegion> Materials;

		// Second-order (10-node) tetrahedra: mid-edge nodes added by the
		// solver script. A skin one tetrahedron thick in bending needs them;
		// first-order tetrahedra lock.
		bool bSecondOrder = true;

		// --- support: every DOF of the nodes within SupportTolerance of
		// the plane SupportAxis = SupportValue (project units). "min" / "max"
		// put the plane at the mesh's own extreme instead of a value. ---
		TCHAR SupportAxis = TEXT('y');
		double SupportValue = 0.0;
		bool bSupportAtMin = false;
		bool bSupportAtMax = false;
		double SupportTolerance = 0.0;   // 0: a millionth of the mesh's diagonal

		// --- loads, SI ---
		// Inertial load factor: the body's own mass under LoadFactor x g
		// along -z (1 is its weight; 0 switches gravity off).
		double LoadFactor = 1.0;
		// Uniform pressure on the wetted (outer) surface, pascals, positive
		// pressing inwards. Or a pressure per wetted face (phase 2), which
		// takes precedence when non-empty; the faces are the outer surface's
		// triangles in the order the script reports them.
		double Pressure = 0.0;
		TArray<double> PressurePerFace;
		// A point force, newtons, at the node nearest the point (project units).
		bool bPointForce = false;
		aerotelic::Vec3 PointAt;
		aerotelic::Vec3 PointForce;

		// Where to write the solution (displacement, von Mises).
		FString VtkPath;
	};

	struct FResult
	{
		int32 Nodes = 0, Elements = 0, Dofs = 0;
		int32 SupportNodes = 0, WettedFaces = 0;
		bool bSecondOrder = false;

		double Mass = 0.0;                      // kg, over every element
		double MaxDisplacement = 0.0;           // m
		double MaxDisplacementAt[3] = {0, 0, 0};
		double MaxVonMises = 0.0;               // Pa
		int32 MaxVonMisesMaterial = 0;
		// Per material id (index = position in FRequest::Materials): the
		// peak von Mises, and the margin allowable / peak - 1 (0 when the
		// material has no allowable).
		TArray<double> MaxVonMisesByMaterial;
		TArray<double> MarginByMaterial;

		// The force balance: the reactions at the support against what
		// was applied. Equal and opposite in a converged solution.
		double Reaction[3] = {0, 0, 0};
		double AppliedGravity[3] = {0, 0, 0};
		double AppliedPressure[3] = {0, 0, 0};
		double AppliedPoint[3] = {0, 0, 0};
		// Mean of u . n over the wetted nodes: negative is inward, which
		// is what a positive pressure must do.
		double MeanNormalDisplacement = 0.0;

		double Seconds = 0.0, SecondsSolve = 0.0;
		FString VtkPath;
	};

	// tools/structural_fea.py under the project (staged with the packaged
	// build). Empty if missing.
	FString FindScript();

	// Runs the analysis. False with the reason on failure (no interpreter,
	// no Kratos, no element, a material without stiffness, no node on the
	// support plane, a solve that did not converge).
	bool Run(const FRequest& Request, FResult& Out, FString& OutError);
}
