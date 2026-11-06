#pragma once

#include "CoreMinimal.h"

#include "AerotelicStructuralFea.h"
#include "aerotelic/tet.hpp"
#include "aerotelic/vec3.hpp"

// The structural analysis's inputs as the solver will see them, derived
// here -- headless, from an AerotelicStructuralFea::FRequest and without
// running anything -- by the same rules tools/structural_fea.py applies
// when it builds the Kratos model:
//
//   - the wetted faces: the boundary faces of the tetrahedra (a face that
//     belongs to one tetrahedron only), outward, in the script's order
//     (lexicographic by sorted node ids, np.unique's), then the shell-only
//     regions' faces as given, so that a pressure per face maps face for
//     face;
//   - the support: every node within the tolerance of the plane
//     axis = value, "min" / "max" resolved on the mesh, the zero tolerance
//     resolved to a ten-thousandth of the mesh's diagonal;
//   - the point force: the corner node nearest the point;
//   - the loads as force vectors in newtons: the pressure on each wetted
//     face (-p n A), the inertial load on the part's mass at its centre
//     (a homogenised core's mass estimated from the cell's solid
//     fraction, before the homogeniser has run), the point force.
//
// The Structural dialog's pre-processor tabs draw this (the mesh, the
// loads as vectors, the fixed nodes), and the document commandlet checks
// its counts against what the script reports after a solve, so the two
// cannot drift apart unnoticed.
namespace AerotelicFeaPreprocess
{
	// One wetted face: its nodes wound outward, the tetrahedron it belongs
	// to (-1 for a shell-only region's face) and its region id.
	struct FFace
	{
		int32 N[3] = { 0, 0, 0 };
		int32 Tet = -1;
		int32 Region = 0;
	};

	// What one region id of the mesh amounts to.
	struct FRegionSummary
	{
		int32 Id = 0;
		FString Label;
		FString Kind;          // "shells", "solid", "shells over a homogenised core", "homogenised core"
		int32 Tets = 0;
		int32 Faces = 0;
		bool bSecondOrder = false;   // its tetrahedra 10-node (FRegion::IsSecondOrder)
		double Volume = 0.0;   // project units^3
		double Area = 0.0;     // of its wetted faces, project units^2
		double Mass = 0.0;     // kg (a homogenised core: estimated)
	};

	struct FModel
	{
		// ---- the mesh, project units ----
		int32 Nodes = 0, Tets = 0;
		aerotelic::Box3 Bounds;
		double Diagonal = 0.0;
		TArray<FFace> WettedFaces;
		// ---- the system the solver will build (2026-11-05), by the
		// script's own rules, so the cost is known BEFORE the solve: the
		// tetrahedra of the second-order regions, the mid-edge nodes the
		// script adds for them (one per distinct edge of those tetrahedra),
		// the corner nodes carrying shells (six degrees of freedom; the
		// rest three), and the degrees of freedom in all. ----
		int32 SecondOrderTets = 0;
		int32 MidEdgeNodes = 0;
		int32 ShellNodes = 0;
		int64 Dofs = 0;
		// Which solver "auto" would run at that size (the script's rule).
		bool DirectSolverAuto() const { return Dofs <= AerotelicStructuralFea::kAutoDirectDofs; }
		// ---- the tetrahedra's shape (2026-11-05), over whatever mesh the
		// solver is handed: the extreme dihedral angles (degrees) and the
		// tetrahedra outside the acceptance band of a linear element --
		// a dihedral under 10 degrees (the stiffness matrix's conditioning,
		// Shewchuk 2002) or over 160 (the gradient's, so the stress's,
		// accuracy). The fraction outside the band against kPoorTetFraction
		// is the dialog's verdict: shown at Build Model, asked about at
		// Solve. Found on Center at 100 x 100 (tools in the diagnosis of
		// 2026-11-05): the nose and tail pole fans of the fuselage grid
		// give needle triangles no volume mesher can fix, and an interior
		// not seeded by the surface added a fan of slivers along the skin.
		double MinDihedral = 0.0, MaxDihedral = 0.0;
		int32 TetsDihedralUnder10 = 0, TetsDihedralOver160 = 0;
		static constexpr double kPoorTetFraction = 0.02;
		double PoorTetFraction() const { return Tets > 0 ? static_cast<double>(TetsDihedralUnder10 + TetsDihedralOver160) / Tets : 0.0; }
		bool MeshIsPoor() const { return Tets > 0 && PoorTetFraction() > kPoorTetFraction; }
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
			int32 Nodes = 0;   // on this plane
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
		// tetrahedron or a face), and the regions none of whose pieces holds
		// a support node. Such a region FLOATS -- a linear static solve on
		// it is singular, and an iterative solver returns rubbish for the
		// whole model (a mirrored wing chosen beside the boolean that
		// already contains it was the case that found this).
		int32 Components = 0;
		TArray<int32> NodeComponent;      // per node
		TArray<bool> ComponentSupported;  // per component: holds a support node
		TArray<int32> FloatingRegions;    // region ids with no path to the support
		int32 FloatingNodes = 0;

		// ---- the loads, SI ----
		TArray<double> FacePressure;    // Pa, one per wetted face
		FVector PressureResultant = FVector::ZeroVector;   // N, sum of -p n A
		double MaxFacePressure = 0.0;   // max |p| over the faces, Pa
		// The point forces, each at its nearest corner node.
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
		double Mass = 0.0;              // kg, the part's (see bMassEstimated)
		bool bMassEstimated = false;    // a homogenised core's density from the cell's solid fraction
		FVector MassCentre = FVector::ZeroVector;           // project units, mass-weighted
		FVector GravityForce = FVector::ZeroVector;         // N, (0, 0, -m n g)
	};

	// Derives the model. False with the reason when the request has no
	// mesh, no region for a tetrahedron or face, or no wetted face at all.
	// A support with fewer than three nodes is NOT an error here (the
	// script refuses it; the dialog shows what there is and says so).
	bool Build(const AerotelicStructuralFea::FRequest& Request, FModel& Out, FString& OutError);
}
