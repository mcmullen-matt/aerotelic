#pragma once

#include "CoreMinimal.h"

#include "AerotelicStructuralFea.h"
#include "aerotelic/mesh.hpp"
#include "aerotelic/vec3.hpp"

#include <cmath>

// The structural analysis's inputs as the solver will see them, derived
// here -- headless, from an AerotelicStructuralFea::FRequest and without
// running anything -- by the same rules tools/structural_fea.py applies
// when it builds the Kratos model (shells only since 2026-11-08):
//
//   - the wetted faces: the skin regions' faces as given, in order, so
//     that a pressure per face maps face for face;
//   - the support: every node within the tolerance of the plane
//     axis = value, "min" / "max" resolved on the mesh, the zero tolerance
//     resolved to a ten-thousandth of the mesh's diagonal;
//   - the point force: the node nearest the point;
//   - the loads as force vectors in newtons: the pressure on each wetted
//     face (-p n A), the inertial load on the part's mass at its centre
//     (the shells' rho t A and the ribs'), the point force;
//   - the system's size: every node is a shell node, six degrees of
//     freedom, so Dofs = 6 x Nodes.
//
// The Structural dialog's pre-processor tabs draw this (the mesh, the
// loads as vectors, the fixed nodes), and the document commandlet checks
// its counts against what the script reports after a solve, so the two
// cannot drift apart unnoticed.
namespace AerotelicFeaPreprocess
{
	// One face: its nodes wound outward (a wetted face) or as the rib mesher
	// gave them (a rib face), and its region id.
	struct FFace
	{
		int32 N[3] = { 0, 0, 0 };
		int32 Region = 0;
	};

	// What one region id of the mesh amounts to.
	struct FRegionSummary
	{
		int32 Id = 0;
		FString Label;
		FString Kind;          // "shells" or "rib shells"
		int32 Faces = 0;       // wetted faces (skin shells) of the region
		int32 RibFaces = 0;    // rib shells of the region (FRequest::RibFaces)
		double Area = 0.0;     // of its wetted faces, project units^2
		double RibArea = 0.0;  // of its rib faces, project units^2
		double Mass = 0.0;     // kg
	};

	struct FModel
	{
		// ---- the mesh, project units ----
		int32 Nodes = 0;
		aerotelic::Box3 Bounds;
		double Diagonal = 0.0;
		TArray<FFace> WettedFaces;
		// The rib faces (2026-11-08): shells inside the body, not wetted --
		// no pressure, no part in the surface displacement -- but carrying
		// mass, degrees of freedom and the load path like any shell. Region
		// is the rib region's.
		TArray<FFace> RibFaces;
		// ---- the system the solver will build (2026-11-05), by the
		// script's own rules, so the cost is known BEFORE the solve: every
		// node carries a shell (six degrees of freedom), ShellNodes counts
		// the ones a face of a skin or rib region actually touches. ----
		int32 ShellNodes = 0;
		int64 Dofs = 0;
		// Which solver "auto" would run at that size (the script's rule).
		bool DirectSolverAuto() const { return Dofs <= AerotelicStructuralFea::kAutoDirectDofs; }
		TArray<FVector> FaceCentre;
		TArray<FVector> FaceNormal;     // unit, outward
		TArray<double> FaceArea;        // project units^2
		TArray<FRegionSummary> Regions; // by region id, ascending

		// ---- the supports, resolved: each plane's axis 0/1/2, value and
		// tolerance in project units, and the nodes they fix together ----
		struct FPlane
		{
			int32 Axis = 1;
			double Value = 0.0;
			double Tolerance = 0.0;
			// The patch (FSupport::bBounded): Lo..Hi along the in-plane axes
			// (Axis + 1) % 3 and (Axis + 2) % 3, project units.
			bool bBounded = false;
			double Lo[2] = { 0.0, 0.0 };
			double Hi[2] = { 0.0, 0.0 };
			int32 Nodes = 0;   // on this plane (and inside the patch)
			double Furthest = 0.0;   // the furthest of them from the plane, project units
			// The chosen body "min" / "max" was resolved against (FSupport::Body),
			// -1 for the whole mesh; true when that body had no nodes in the
			// mesh and the whole mesh was used instead.
			int32 Body = -1;
			bool bBodyFallback = false;
		};
		TArray<FPlane> SupportPlanes;
		TArray<int32> SupportNodes;   // the union, each node once
		// The median edge length of the wetted faces, project units: the scale
		// a support tolerance is judged against. A tolerance of more than
		// about a quarter of it can take a second ring of nodes and shorten
		// the part (a 1 x 10 x 1 beam lost 3.7% of its deflection to one
		// extra ring, 2026-11-02).
		double MedianEdge = 0.0;
		bool ToleranceIsCoarse(const FPlane& P) const { return MedianEdge > 0.0 && P.Tolerance > 0.25 * MedianEdge; }
		// The load path: the mesh's connected pieces (nodes joined by a
		// face), and the regions none of whose pieces holds a support node.
		// Such a region FLOATS -- a linear static solve on it is singular,
		// and an iterative solver returns rubbish for the whole model (a
		// mirrored wing chosen beside the boolean that already contains it
		// was the case that found this).
		int32 Components = 0;
		TArray<int32> NodeComponent;      // per node
		TArray<bool> ComponentSupported;  // per component: holds a support node
		TArray<int32> FloatingRegions;    // region ids with no path to the support
		int32 FloatingNodes = 0;

		// ---- the loads, SI ----
		TArray<double> FacePressure;    // Pa, one per wetted face
		FVector PressureResultant = FVector::ZeroVector;   // N, sum of -p n A
		double MaxFacePressure = 0.0;   // max |p| over the faces, Pa
		// The point forces, each at its nearest node.
		struct FPointLoad
		{
			int32 Node = -1;
			FVector At = FVector::ZeroVector;      // the node, project units
			FVector Asked = FVector::ZeroVector;   // the point asked for, project units
			double SnapDistance = 0.0;             // from Asked to the node, project units
			FVector Force = FVector::ZeroVector;   // N
		};
		TArray<FPointLoad> PointLoads;
		// A point force that landed further from its point than a mesh edge
		// is not where it was typed: the caption and the Apply status warn.
		bool SnapIsFar(const FPointLoad& P) const { return MedianEdge > 0.0 && P.SnapDistance > MedianEdge; }
		FVector PointResultant = FVector::ZeroVector;      // N
		double Mass = 0.0;              // kg, the part's: the shells' and the ribs'
		FVector MassCentre = FVector::ZeroVector;           // project units, mass-weighted
		FVector GravityForce = FVector::ZeroVector;         // N, (0, 0, -m n g)
	};

	// Derives the model. False with the reason when the request has no
	// nodes, no region for a face, or no wetted face at all. A support
	// with fewer than three nodes is NOT an error here (the script refuses
	// it; the dialog shows what there is and says so).
	bool Build(const AerotelicStructuralFea::FRequest& Request, FModel& Out, FString& OutError);

	// Whether a node lies on a resolved plane: within its tolerance of it,
	// and inside its patch when bounded. The one rule, shared by Build and
	// by the dialog's live redraw while a patch handle is dragged.
	inline bool NodeOnPlane(const aerotelic::Vec3& P, const FModel::FPlane& Plane)
	{
		const double C[3] = { P.x, P.y, P.z };
		if (std::abs(C[Plane.Axis] - Plane.Value) > Plane.Tolerance) return false;
		if (!Plane.bBounded) return true;
		for (int32 k = 0; k < 2; ++k)
		{
			const double V = C[(Plane.Axis + 1 + k) % 3];
			if (V < FMath::Min(Plane.Lo[k], Plane.Hi[k]) || V > FMath::Max(Plane.Lo[k], Plane.Hi[k])) return false;
		}
		return true;
	}
}
