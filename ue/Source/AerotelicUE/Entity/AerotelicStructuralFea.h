#pragma once

#include "CoreMinimal.h"

#include <vector>

#include "aerotelic/materials.hpp"
#include "aerotelic/vec3.hpp"

#include "AerotelicMeshLib.h"

// A linear static finite-element analysis of scene bodies on
// KratosMultiphysics' StructuralMechanics application
// (tools/structural_fea.py, run as a separate process in the WingDesigner
// micromamba environment -- the same bridge pattern as the potential flow,
// AerotelicPotentialFlow.h).
//
// The model is SHELLS ONLY (2026-11-08: the application dropped
// tetrahedral FEA and lattice infill; every structural element is a
// shell). It is a set of REGIONS, each a body or a part of one with what
// its structure (body_structure.hpp) says it is:
//
//   - a body's SKIN: shells of its skin thickness and material on its own
//     surface triangles (the body's Mesh tab grid, the one the viewport
//     shows), wetted -- the pressure lands on them;
//   - a body's RIBS: one region per plane network, shells of the network's
//     thickness and material on the planar discs tools/rib_gmsh.py meshes
//     inside the body (AerotelicRibGmsh.h), their rims on the skin's own
//     nodes so skin and ribs share degrees of freedom. Unloaded -- not
//     wetted -- but carrying mass and the load path.
//
// Everything is handed to the solver in SI: the nodes scaled by LengthToSI,
// the materials as they are (materials.hpp is SI), the loads in newtons
// and pascals. What comes back is SI too; the dialog converts for display.
//
// Loads: an inertial load factor, a uniform pressure on the wetted
// surface or a pressure per wetted face (the flow solvers' field), any
// number of point forces; and any number of supports, each a coordinate
// plane (or a patch of one) whose nodes are clamped.
namespace AerotelicStructuralFea
{
	// "auto" takes the direct solver (Eigen's SparseLU, one thread) up to
	// this many degrees of freedom and amgcl beyond -- the script's
	// AUTO_DIRECT_DOFS, mirrored so the dialog can say before a solve what
	// will run. A desktop's figure: a 64,056-dof sphere took two minutes
	// direct; a 540,000-dof fuselage (2026-11-05, in the days of
	// tetrahedra) ran a 32 GB machine and its page file dry. Every node is
	// a shell node now, six degrees of freedom each, so 150,000 DOF is
	// 25,000 nodes.
	constexpr int32 kAutoDirectDofs = 150000;

	// One region: the faces carrying its id, in FRequest::ShellFaces (a
	// skin) or FRequest::RibFaces (ribs). A region is one or the other.
	struct FRegion
	{
		int32 Id = 0;
		FString Label;
		// Shells of this material and thickness (project units) on the
		// region's wetted faces; SkinThickness 0 means no skin shells.
		aerotelic::Material Skin;      // SI
		double SkinThickness = 0.0;
		// The chosen body this region was built from (the Structural
		// dialog's picker index), so a support plane "at the mesh's minimum
		// / maximum" can be resolved against that body alone (FSupport::Body).
		// -1: not from a picked body.
		int32 Body = -1;
		// Rib shells (2026-11-08): shells of this material and thickness on
		// the region's faces in FRequest::RibFaces -- the cross-sections of
		// one of a body's plane networks, meshed by tools/rib_gmsh.py with
		// their rims on the skin's nodes. Unloaded: no pressure lands on
		// them, they are not wetted; their mass counts.
		aerotelic::Material Rib;       // SI
		double RibThickness = 0.0;
		// A LAMINATE skin's drape (2026-11-12; empty otherwise): per face of
		// this region in FRequest::ShellFaces order, per ply of the expanded
		// stack, (deviation, shear) in degrees -- the angle from the ply's
		// nominal direction to its draped warp, and how far warp and weft
		// have closed from 90 (core drape.hpp, computed by the model
		// builder). The script turns the plies and thickens the woven ones
		// by it. Faces x plies x 2 floats.
		TArray<float> SkinDrape;
		TArray<float> RibDrape;   // the same for a laminate rib layer (planar: normally none)

		bool HasShells() const { return SkinThickness > 0.0; }
		bool HasRibs() const { return RibThickness > 0.0; }
	};

	// A wetted face of a skin region (or a rib face, in FRequest::RibFaces):
	// its nodes (into FRequest::Nodes), wound outward, and its region id.
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
		// "min" / "max" resolved over the nodes of the regions built from
		// this chosen body (FRegion::Body) rather than over the whole mesh,
		// so the body that sticks out furthest does not capture a support
		// meant for another. -1: the whole mesh. The script only knows the
		// whole mesh, so the dialog resolves the plane to a value before the
		// request is handed over (AerotelicFeaPreprocess gives the value).
		int32 Body = -1;
	};

	// A point force in newtons at the node nearest At (project units).
	struct FPointForce
	{
		aerotelic::Vec3 At;
		aerotelic::Vec3 Force;
	};

	struct FRequest
	{
		// The nodes in PROJECT units; every face below indexes them. Each
		// node is a shell node (six degrees of freedom). Must outlive the
		// call.
		const std::vector<aerotelic::Vec3>* Nodes = nullptr;
		// The skin shells: wetted, the pressure lands on them.
		TArray<FShellFace> ShellFaces;
		// The rib faces (FRegion::Rib): shells inside the body, not wetted.
		TArray<FShellFace> RibFaces;
		// The REFERENCE DIRECTION of every shell (2026-11-12), one per
		// ShellFaces / RibFaces entry or empty: the fibre direction of a
		// laminate's 0 degree ply, as the model builder takes it from the
		// body's loft (the along-the-loft direction in each triangle) or a
		// planar member's longest extent. Any vector; the script projects it
		// into the face. Only matters to laminate regions; a zero vector
		// leaves the face to the script's own fallback.
		TArray<aerotelic::Vec3> ShellAxes;
		TArray<aerotelic::Vec3> RibAxes;
		double LengthToSI = 1.0;
		TArray<FRegion> Regions;

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
		// precedence when non-empty; the faces are ShellFaces in order.
		double Pressure = 0.0;
		TArray<double> PressurePerFace;
		// Point forces, newtons, each at the node nearest its point (project
		// units).
		TArray<FPointForce> PointForces;

		// Where to write the solution (displacement, von Mises).
		FString VtkPath;
	};

	// One line of the script's per_material list: the skin or the ribs of
	// a region.
	struct FEntryResult
	{
		int32 Region = 0;
		FString Label;
		FString Kind;                 // "skin" or "rib"
		double MaxVonMises = 0.0;     // Pa
		double Margin = 0.0;          // reserve factor - 1 (0 without an allowable / strengths)
		double Allowable = 0.0;       // Pa (an isotropic material's)
		double Mass = 0.0;            // kg
		// The criterion the margin follows (2026-11-12): "von Mises" for an
		// isotropic material (reserve factor = allowable / peak), "Tsai-Wu"
		// for a laminate (the smallest reserve factor over its plies and
		// elements; "Tsai-Wu (no strengths: no margin)" when a lamina lacks
		// them). ReserveFactor 0 when none.
		FString Criterion = TEXT("von Mises");
		double ReserveFactor = 0.0;
		// A laminate's stack as analysed: plies, thickness (m), distinct
		// draped stacks, and the drape's extremes (degrees) when one was
		// given.
		int32 Plies = 0;
		double Thickness = 0.0;
		int32 Stacks = 0;
		bool bDraped = false;
		double DrapeMaxShear = 0.0, DrapeMaxDeviation = 0.0;
		int32 DrapeFacesPastLocking = 0;
		bool IsLaminate() const { return Plies > 0; }
	};

	struct FResult
	{
		int32 Nodes = 0, Elements = 0, Dofs = 0;
		int32 Shells = 0;   // every shell, the rib shells included
		int32 RibShells = 0;
		// The nodes the solver clamped. SupportCornerNodes is the same count
		// (kept under its old name for the callers that compare it with the
		// pre-processor's figure; with shells alone there are no other nodes).
		int32 SupportNodes = 0, SupportCornerNodes = 0, WettedFaces = 0;

		double Mass = 0.0;                      // kg, over every element
		double MaxDisplacement = 0.0;           // m
		double MaxDisplacementAt[3] = {0, 0, 0};
		double MaxVonMises = 0.0;               // Pa
		int32 MaxVonMisesRegion = 0;
		TArray<FEntryResult> Entries;
		int32 Laminates = 0;                    // regions analysed as laminates

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

		double Seconds = 0.0, SecondsSolve = 0.0, SecondsModes = 0.0;
		FString VtkPath;
	};

	// tools/structural_fea.py under the project (staged with the packaged
	// build). Empty if missing.
	FString FindScript();

	// Runs the analysis. False with the reason on failure (no interpreter,
	// no Kratos, nothing to analyse, a material without stiffness, a region
	// with neither a skin nor a rib thickness, no node on the support
	// plane, a solve that did not converge). Blocks until the script is
	// done: the dialog calls it on a worker thread and watches Progress
	// (the script's stage lines as it prints them; a cancel flag that ends
	// the process and returns false with "cancelled"). Request and Out must
	// outlive the call; nothing here touches the engine's game thread.
	bool Run(const FRequest& Request, FResult& Out, FString& OutError, AerotelicMeshLib::FJobProgress* Progress = nullptr);
}
