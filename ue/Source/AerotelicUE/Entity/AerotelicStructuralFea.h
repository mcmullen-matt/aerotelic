#pragma once

#include "CoreMinimal.h"

#include "aerotelic/lattice.hpp"
#include "aerotelic/materials.hpp"
#include "aerotelic/tet.hpp"
#include "aerotelic/vec3.hpp"

// A linear static finite-element analysis of a lattice-filled body on
// KratosMultiphysics' StructuralMechanics application
// (tools/structural_fea.py, run as a separate process in the WingDesigner
// micromamba environment -- the same bridge pattern as the potential flow,
// AerotelicPotentialFlow.h).
//
// Two models of the same part:
//
//   ShellCore (the default, the one a desktop can carry). The core is
//   tetrahedra bounded by the body's own surface at the lattice cell size
//   (FAerotelicLatticeModel::GetStructuralTetMeshes), carrying the
//   lattice's HOMOGENISED elasticity tensor and effective density: the
//   script voxelises one unit cell (aerotelic::SampleUnitCell, handed over
//   in the job) and solves it under the six unit strains
//   (tools/homogenize.py). The skin is a shell element on every surface
//   triangle with the skin thickness as a property. The skin's material is
//   its body's; the lattice's material is the BASE material its struts or
//   walls are made of -- not an effective figure -- and the core's margin
//   is against that material's allowable using the estimated micro stress
//   in the struts, not the smeared macro stress of the homogenised core.
//
//   SolidTets. The skin / core tetrahedra Lattice > Export Tetrahedral
//   Mesh writes, each tetrahedron a material by id (the skin its body's,
//   the core the lattice's, here read as an EFFECTIVE material). The
//   full-resolution reference, for small bodies.
//
// Everything is handed to the solver in SI: the mesh scaled by LengthToSI,
// the materials as they are (materials.hpp is SI), the loads in newtons
// and pascals. What comes back is SI too; the dialog converts for display.
//
// Loads: an inertial load factor, a uniform pressure on the wetted
// surface or a pressure per wetted face (the flow solvers' field), a point
// force; and a support on a coordinate plane.
namespace AerotelicStructuralFea
{
	enum class EModel { ShellCore, SolidTets };

	// SolidTets: what the tetrahedra of one material id are made of.
	struct FMaterialRegion
	{
		int32 Id = 0;
		FString Label;
		aerotelic::Material Material;   // SI
	};

	// ShellCore: one filled body -- the region id its tetrahedra carry, its
	// skin's material and thickness, and the lattice's base material with
	// the voxelised unit cell the core is homogenised from.
	struct FRegion
	{
		int32 Id = 0;
		FString Label;
		aerotelic::Material Skin;       // SI
		double SkinThickness = 0.0;     // PROJECT units (scaled with the mesh)
		aerotelic::Material Core;       // SI: the lattice's BASE material
		const aerotelic::UnitCellSamples* Cell = nullptr;
	};

	struct FRequest
	{
		EModel Model = EModel::ShellCore;

		// The tetrahedra in PROJECT units, a region / material id per
		// tetrahedron: SolidTets' skin and core tetrahedra, or ShellCore's
		// body-only tetrahedra (whose boundary faces take the shells).
		const aerotelic::TetMesh* Mesh = nullptr;
		double LengthToSI = 1.0;
		TArray<FMaterialRegion> Materials;   // SolidTets, one per material id
		TArray<FRegion> Regions;             // ShellCore, one per region id

		// SolidTets: second-order (10-node) tetrahedra, mid-edge nodes added
		// by the solver script. A skin one tetrahedron thick in bending
		// needs them; first-order tetrahedra lock. (ShellCore uses 4-node
		// tetrahedra: the skin, the bending problem, is shells.)
		bool bSecondOrder = true;

		// The linear solver: "amgcl" (iterative, the default) or
		// "sparse_lu" (direct; sure, and fine up to a few hundred thousand
		// degrees of freedom).
		FString LinearSolver = TEXT("amgcl");
		// Reuse a homogenisation of the same cell and material from an
		// earlier run (the script keeps them under the user's temp folder).
		bool bHomogenizeCache = true;

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
		// pressing inwards. Or a pressure per wetted face, which takes
		// precedence when non-empty; the faces are the outer surface's
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

	// ShellCore: what came of one region.
	struct FRegionResult
	{
		int32 Id = 0;
		FString Label;
		// The homogenised core.
		double RelativeDensity = 0.0;        // solid fraction of the cell's voxels
		double E[3] = {0, 0, 0};             // effective Young's moduli, Pa
		double G[3] = {0, 0, 0};             // effective shear moduli, Pa (xy, yz, xz)
		double NuXY = 0.0, NuYZ = 0.0, NuXZ = 0.0;
		double Amplification = 0.0;          // peak strut von Mises over |macro stress| bound
		double HomogenizeSeconds = 0.0;
		bool bHomogenizeCached = false;
		// Masses, kg: the shells' rho t A and the core's (effective density
		// less the skin overlap) over the volume.
		double SkinMass = 0.0, CoreMass = 0.0;
		double CoreDensityUsed = 0.0;        // kg/m^3, after the overlap correction
		// Stresses, Pa, and margins allowable / peak - 1 (0 without an allowable).
		double SkinMaxVonMises = 0.0, SkinMargin = 0.0;
		double CoreMaxVonMises = 0.0;        // macro (smeared)
		double CoreMaxMicroVonMises = 0.0;   // estimated in the struts
		double CoreMargin = 0.0;             // base allowable / micro peak - 1
	};

	struct FResult
	{
		EModel Model = EModel::ShellCore;
		int32 Nodes = 0, Elements = 0, Dofs = 0;
		int32 Tets = 0, Shells = 0;
		int32 SupportNodes = 0, WettedFaces = 0;
		bool bSecondOrder = false;

		double Mass = 0.0;                      // kg, over every element
		double MaxDisplacement = 0.0;           // m
		double MaxDisplacementAt[3] = {0, 0, 0};
		double MaxVonMises = 0.0;               // Pa (ShellCore: shells, and cores by their micro stress)
		int32 MaxVonMisesMaterial = 0;
		// Per entry of the script's per_material list, in order: SolidTets
		// one per FRequest::Materials; ShellCore two per region, the skin
		// then the core (the core by its MICRO stress).
		TArray<FString> PerMaterialLabel;
		TArray<double> MaxVonMisesByMaterial;
		TArray<double> MarginByMaterial;
		TArray<FRegionResult> Regions;          // ShellCore

		// The force balance: the reactions at the support against what
		// was applied. Equal and opposite in a converged solution.
		double Reaction[3] = {0, 0, 0};
		double AppliedGravity[3] = {0, 0, 0};
		double AppliedPressure[3] = {0, 0, 0};
		double AppliedPoint[3] = {0, 0, 0};
		// Mean of u . n over the wetted nodes: negative is inward, which
		// is what a positive pressure must do.
		double MeanNormalDisplacement = 0.0;

		double Seconds = 0.0, SecondsSolve = 0.0, SecondsHomogenize = 0.0;
		FString VtkPath;
	};

	// tools/structural_fea.py under the project (staged with the packaged
	// build). Empty if missing.
	FString FindScript();

	// Runs the analysis. False with the reason on failure (no interpreter,
	// no Kratos, no element, a material without stiffness, a region without
	// a cell or a skin thickness, no node on the support plane, a solve
	// that did not converge).
	bool Run(const FRequest& Request, FResult& Out, FString& OutError);
}
