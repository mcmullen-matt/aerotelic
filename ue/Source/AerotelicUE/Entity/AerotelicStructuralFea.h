#pragma once

#include "CoreMinimal.h"

#include "aerotelic/lattice.hpp"
#include "aerotelic/materials.hpp"
#include "aerotelic/tet.hpp"
#include "aerotelic/vec3.hpp"

#include "AerotelicMeshLib.h"

// A linear static finite-element analysis of scene bodies on
// KratosMultiphysics' StructuralMechanics application
// (tools/structural_fea.py, run as a separate process in the WingDesigner
// micromamba environment -- the same bridge pattern as the potential flow,
// AerotelicPotentialFlow.h).
//
// The model is a set of REGIONS, each a body (or a part of one) with what
// its material properties say it is (2026-10-31: any body with a
// structural material can be analysed, lattice-filled or not):
//
//   - a body with a skin thickness and no lattice: SHELLS of that thickness
//     on its own surface triangles, nothing inside (a thin-walled part);
//   - a body with no skin thickness and no lattice: SOLID tetrahedra of
//     its material, meshed by gmsh at an element size;
//   - a lattice-filled body: shells of its skin thickness on ITS OWN
//     surface triangles (the body's Mesh tab grid, the one the viewport
//     shows) over tetrahedra at the lattice cell size carrying the
//     lattice's HOMOGENISED tensor (one unit cell,
//     aerotelic::SampleUnitCell, voxelised and solved under the six unit
//     strains by tools/homogenize.py) -- the struts being of the body's
//     own material, which the lattice inherits. The lattice lends its
//     material card only; its private, finer copy of the surface (what it
//     hollows on) never reaches the solver (2026-11-05). Or, at full
//     resolution, the lattice's own skin tetrahedra as a solid of the
//     material over core tetrahedra with the homogenised tensor.
//
// Everything is handed to the solver in SI: the mesh scaled by LengthToSI,
// the materials as they are (materials.hpp is SI), the loads in newtons
// and pascals. What comes back is SI too; the dialog converts for display.
//
// Loads: an inertial load factor, a uniform pressure on the wetted
// surface or a pressure per wetted face (the flow solvers' field), any
// number of point forces; and any number of supports, each a coordinate
// plane whose nodes are clamped.
namespace AerotelicStructuralFea
{
	// "auto" takes the direct solver (Eigen's SparseLU, one thread) up to
	// this many degrees of freedom and amgcl beyond -- the script's
	// AUTO_DIRECT_DOFS, mirrored so the dialog can say before a solve what
	// will run. A desktop's figure: a 64,056-dof sphere took two minutes
	// direct; a 540,000-dof fuselage (10-node tetrahedra on a 24,571-node
	// core, 2026-11-05) ran a 32 GB machine and its page file dry.
	constexpr int32 kAutoDirectDofs = 150000;

	// One region: the tetrahedra and wetted faces carrying its id.
	struct FRegion
	{
		int32 Id = 0;
		FString Label;
		// Shells of this material and thickness (project units) on the
		// region's wetted faces; SkinThickness 0 means no shells.
		aerotelic::Material Skin;      // SI
		double SkinThickness = 0.0;
		// What the region's tetrahedra are: a solid of Core (bSolid), or a
		// lattice homogenised from Cell with Core as its base material
		// (Cell != null). Neither: a shell-only region with no tetrahedra.
		aerotelic::Material Core;      // SI
		const aerotelic::UnitCellSamples* Cell = nullptr;
		bool bSolid = false;
		// The order of THIS region's tetrahedra: 1 second-order (10-node,
		// the mid-edge nodes added by the script), 0 first-order (4-node),
		// -1 the request's bSecondOrder. A homogenised core is 0: it carries
		// a material card, the shells on its faces carry the bending, and
		// promoting it multiplied a fuselage's nodes by seven for nothing.
		int32 SecondOrder = -1;
		// The chosen body this region was meshed from (the Structural
		// dialog's picker index), so a support plane "at the mesh's minimum
		// / maximum" can be resolved against that body alone (FSupport::Body).
		// -1: not from a picked body.
		int32 Body = -1;
		// Rib shells (2026-11-08): shells of this material and thickness on
		// the region's faces in FRequest::RibFaces -- the cross-sections of a
		// body's plane networks, embedded in its tetrahedra (or hanging on
		// its skin shells alone) by tools/tet_gmsh.py. Unloaded: no pressure
		// lands on them, they are not wetted; their mass counts. A region of
		// ribs alone is one network.
		aerotelic::Material Rib;       // SI
		double RibThickness = 0.0;

		bool HasShells() const { return SkinThickness > 0.0; }
		bool HasRibs() const { return RibThickness > 0.0; }
		bool HasHomogenisedCore() const { return Cell != nullptr; }
		bool HasTetrahedra() const { return bSolid || Cell != nullptr; }
		bool IsSecondOrder(bool bRequestDefault) const { return SecondOrder < 0 ? bRequestDefault : SecondOrder > 0; }
	};

	// A wetted face of a shell-only region (or a rib face, in
	// FRequest::RibFaces): its nodes (into the request mesh's nodes), wound
	// outward, and its region id.
	struct FShellFace
	{
		int32 N[3] = { 0, 0, 0 };
		int32 Region = 0;
	};

	// A support plane: the nodes within Tolerance of Axis = Value (project
	// units; "min" / "max" put it at the mesh's own extreme) are fixed in
	// every degree of freedom -- a clamp. Bounded (2026-11-05), it is a
	// PATCH of that plane: only the nodes inside the rectangle Lo..Hi
	// along the plane's two in-plane axes -- the other two axes in cyclic
	// order, (Axis + 1) % 3 then (Axis + 2) % 3, so a plane x = c has the
	// patch in (y, z), y = c in (z, x), z = c in (x, y) -- are fixed. An
	// infinite plane through a fuselage at x = 3 also took the wing root
	// and anything else crossing x = 3; the patch is what the user drags
	// in the Support tab's picture.
	struct FSupport
	{
		TCHAR Axis = TEXT('y');
		double Value = 0.0;
		bool bAtMin = false;
		bool bAtMax = false;
		double Tolerance = 0.0;   // 0: a ten-thousandth of the mesh's diagonal (a lofted section is planar to about that)
		bool bBounded = false;
		double Lo[2] = { 0.0, 0.0 };   // project units, in-plane axes as above
		double Hi[2] = { 0.0, 0.0 };
		// The in-plane axes (0 x, 1 y, 2 z) of a plane normal to Axis.
		static int32 InPlaneAxis(int32 PlaneAxis, int32 Which) { return (PlaneAxis + 1 + Which) % 3; }
		// "min" / "max" resolved over the nodes of the regions meshed from
		// this chosen body (FRegion::Body) rather than over the whole mesh,
		// so the body that sticks out furthest does not capture a support
		// meant for another. -1: the whole mesh. The script only knows the
		// whole mesh, so the dialog resolves the plane to a value before the
		// request is handed over (AerotelicFeaPreprocess gives the value).
		int32 Body = -1;
	};

	// A point force in newtons at the corner node nearest At (project units).
	struct FPointForce
	{
		aerotelic::Vec3 At;
		aerotelic::Vec3 Force;
	};

	struct FRequest
	{
		// The tetrahedra in PROJECT units with a region id per tetrahedron
		// (none at all when every body is shells), the nodes of every
		// region's faces among the mesh's nodes.
		const aerotelic::TetMesh* Mesh = nullptr;
		TArray<FShellFace> ShellFaces;
		// The rib faces (FRegion::Rib): shells inside the body, not wetted.
		TArray<FShellFace> RibFaces;
		double LengthToSI = 1.0;
		TArray<FRegion> Regions;

		// Second-order (10-node) tetrahedra, the mid-edge nodes added by the
		// solver script (the shells stay 3-node on the corners), for the
		// regions that do not say otherwise (FRegion::SecondOrder). A skin
		// one tetrahedron thick in bending needs them; first-order
		// tetrahedra lock. Ignored without tetrahedra.
		bool bSecondOrder = false;

		// The linear solver: "auto" (the default: the direct solver up to
		// kAutoDirectDofs degrees of freedom, amgcl beyond), "sparse_lu"
		// (direct; exact) or "amgcl" (iterative, every core; it can stop
		// short on thin shells and return a solution whose reactions do not
		// balance the loads -- FResult::bBalanced says so).
		FString LinearSolver = TEXT("auto");
		// The shell element: "ShellThinElement3D3N" (the linear thin
		// triangle, the default: a linear static analysis is then
		// self-consistent whatever the displacement -- the reactions balance
		// the loads exactly) or "ShellThinElementCorotational3D3N" (a
		// geometrically nonlinear element solved in one step: fine for small
		// displacements, inconsistent reactions and stresses once they are
		// large). The script also takes "ShellThickElementCorotational3D3N",
		// which the dialog no longer offers: on a 1 x 10 x 1 thin-walled box
		// (2026-11-02) it was too stiff and did not converge with refinement
		// (0.82, 0.94, 0.98, 0.93 of beam theory) while its force balance
		// error grew from 1e-4 to 1e-1.
		FString ShellElement = TEXT("ShellThinElement3D3N");
		// Reuse a homogenisation of the same cell and material from an
		// earlier run (the script keeps them under the user's temp folder).
		bool bHomogenizeCache = true;
		// How many natural frequencies to compute after the static solve
		// (2026-11-04): the same model -- mesh, materials, clamps, no loads
		// -- as the generalised eigenvalue problem K phi = omega^2 M phi,
		// its lowest Modes modes. 0: none. The frequencies come back in
		// FResult::NaturalFrequencies (Hz) and the mode shapes go into the
		// VTK as point vectors mode_1 ... scaled to a unit largest
		// displacement.
		int32 Modes = 0;

		// --- supports: every DOF of the nodes within Tolerance of each
		// plane Axis = Value (project units) is fixed; a node on any of the
		// planes is fixed. "min" / "max" put a plane at the mesh's own
		// extreme instead of a value. ---
		TArray<FSupport> Supports;

		// --- loads, SI ---
		// Inertial load factor: the body's own mass under LoadFactor x g
		// along -z (1 is its weight; 0 switches gravity off).
		double LoadFactor = 0.0;
		// Uniform pressure on the wetted (outer) surface, pascals, positive
		// pressing inwards. Or a pressure per wetted face, which takes
		// precedence when non-empty; the faces are the tetrahedra's boundary
		// faces in the order the script reports them, then ShellFaces.
		double Pressure = 0.0;
		TArray<double> PressurePerFace;
		// Point forces, newtons, each at the node nearest its point (project
		// units).
		TArray<FPointForce> PointForces;

		// Where to write the solution (displacement, von Mises).
		FString VtkPath;
	};

	// One line of the script's per_material list: the skin, the core (by
	// its micro stress) or the solid of a region.
	struct FEntryResult
	{
		int32 Region = 0;
		FString Label;
		FString Kind;                 // "skin", "core", "solid", "rib"
		double MaxVonMises = 0.0;     // Pa (a core: estimated in the struts)
		double MacroVonMises = 0.0;   // Pa, a core's smeared figure
		double Margin = 0.0;          // allowable / peak - 1 (0 without an allowable)
		double Allowable = 0.0;       // Pa
		double Mass = 0.0;            // kg
	};

	// What came of one homogenised core.
	struct FRegionResult
	{
		int32 Id = 0;
		FString Label;
		double RelativeDensity = 0.0;        // solid fraction of the cell's voxels
		double E[3] = {0, 0, 0};             // effective Young's moduli, Pa
		double G[3] = {0, 0, 0};             // effective shear moduli, Pa (xy, yz, xz)
		double NuXY = 0.0, NuYZ = 0.0, NuXZ = 0.0;
		double Amplification = 0.0;          // peak strut von Mises over |macro stress| bound
		double HomogenizeSeconds = 0.0;
		bool bHomogenizeCached = false;
		double SkinMass = 0.0, CoreMass = 0.0;
		double CoreDensityUsed = 0.0;        // kg/m^3, after the overlap correction
		double SkinMaxVonMises = 0.0, SkinMargin = 0.0;
		double CoreMaxVonMises = 0.0;        // macro (smeared)
		double CoreMaxMicroVonMises = 0.0;   // estimated in the struts
		double CoreMargin = 0.0;             // base allowable / micro peak - 1
	};

	struct FResult
	{
		int32 Nodes = 0, Elements = 0, Dofs = 0;
		int32 Tets = 0, Shells = 0;   // Shells counts the rib shells too
		int32 RibShells = 0;
		// The nodes the solver clamped: on 10-node tetrahedra the mid-edge
		// nodes on the support planes as well as the corner nodes, which
		// SupportCornerNodes counts alone (the pre-processor's figure).
		int32 SupportNodes = 0, SupportCornerNodes = 0, WettedFaces = 0;
		bool bSecondOrder = false;

		double Mass = 0.0;                      // kg, over every element
		double MaxDisplacement = 0.0;           // m
		double MaxDisplacementAt[3] = {0, 0, 0};
		double MaxVonMises = 0.0;               // Pa (shells, solids, and cores by their micro stress)
		int32 MaxVonMisesRegion = 0;
		TArray<FEntryResult> Entries;
		TArray<FRegionResult> Regions;          // the homogenised cores

		// The force balance: the reactions at the support against what
		// was applied. Equal and opposite in a converged solution:
		// BalanceError is |sum R + sum F| over the loads' magnitude, and
		// bBalanced that it is within the script's tolerance (1%). An
		// unbalanced solve is a linear solver that did not converge; its
		// displacements and stresses mean nothing. Warnings carries the
		// script's words for it.
		double Reaction[3] = {0, 0, 0};
		double AppliedGravity[3] = {0, 0, 0};
		double AppliedPressure[3] = {0, 0, 0};
		double AppliedPoint[3] = {0, 0, 0};
		double BalanceError = 0.0;
		bool bBalanced = true;
		TArray<FString> Warnings;
		// The linear solver that ran ("sparse_lu" or "amgcl"; what "auto" chose).
		FString LinearSolver;
		// Mean of u . n over the wetted nodes: negative is inward, which
		// is what a positive pressure must do.
		double MeanNormalDisplacement = 0.0;

		// The natural frequencies asked for (FRequest::Modes), hertz, lowest
		// first -- of the replica when a model scale is set, like every
		// other figure here. Empty when none were asked for, or when the
		// eigensolve failed (then Warnings says so and the static result
		// stands). Eigensolver names the one that ran.
		TArray<double> NaturalFrequencies;
		FString Eigensolver;

		double Seconds = 0.0, SecondsSolve = 0.0, SecondsHomogenize = 0.0, SecondsModes = 0.0;
		FString VtkPath;
	};

	// tools/structural_fea.py under the project (staged with the packaged
	// build). Empty if missing.
	FString FindScript();

	// Runs the analysis. False with the reason on failure (no interpreter,
	// no Kratos, nothing to analyse, a material without stiffness, a region
	// without a cell or a skin thickness, no node on the support plane, a
	// solve that did not converge). Blocks until the script is done: the
	// dialog calls it on a worker thread and watches Progress (the
	// script's stage lines as it prints them; a cancel flag that ends the
	// process and returns false with "cancelled"). Request and Out must
	// outlive the call; nothing here touches the engine's game thread.
	bool Run(const FRequest& Request, FResult& Out, FString& OutError, AerotelicMeshLib::FJobProgress* Progress = nullptr);
}
