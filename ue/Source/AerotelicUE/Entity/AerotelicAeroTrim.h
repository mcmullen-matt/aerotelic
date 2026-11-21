#pragma once

#include "CoreMinimal.h"

#include <atomic>
#include <vector>

#include "aerotelic/aero_trim.hpp"
#include "aerotelic/tri_panel.hpp"
#include "aerotelic/project_yaml.hpp"
#include "aerotelic/units.hpp"
#include "aerotelic/vec3.hpp"

class AAerotelicEntityActor;
struct FAerotelicAeroLoadCase;

// The panel method ON THE BODY TRIANGULATION (core tri_panel.hpp) over the
// scene's entities, and the rigid trim on it, as a headless module shared
// by Analysis > Panel Method, Model > Load Cases and the Kratos dialog
// (2026-11-15). What is paneled is what the viewport draws:
//
//   - a wing's AERO COPY: the sealed one-piece wing on the wing's own grid
//     (WingModel::BuildAeroMesh), the trim's flap kinked in the section --
//     which, without flaps, is the very body the viewport shows;
//   - every other lofted entity's own placed triangles (GetPlacedMeshes at
//     tolerance 0, the Mesh tab's grid), with the trailing edge its loft
//     tags;
//   - a boolean result: its operands' aero copies;
//   - bodies that overlap are FUSED by the mesh boolean into one closed
//     surface before paneling, so nothing of one body lies inside another
//     (the buried panels of the network method, which could not be fused,
//     are gone). A fuse the boolean refuses leaves the bodies separate,
//     with a note.
//
// FSetup says which entities, the plane and the references; BuildCase
// assembles the bodies (game thread: it reads the actors) and describes the
// mesh; SolveCaseNow / FRunner solve; Derive reads the wind-axis
// coefficients, per entity through the triangles it owns in a fused body;
// MakeLoadCase hands the pressures to the structure on the panels' own
// centres; ShowAeroMeshInViewport puts the mesh that will be solved into
// the main 3D view, edges drawn, so the user sees what is analysed.
namespace AerotelicAeroTrim
{
	using aerotelic::project_yaml::EntityType;
	namespace tripanel = aerotelic::tripanel;

	// The entities the method can panel: every lofted solid and a boolean
	// result over them. A propeller is left out (rotating blades are not a
	// steady potential flow).
	bool IsPanelable(EntityType T);

	// The span loading is cut at the centres of this many bands of the
	// semi-span (every 5%): never on the root ring, the tip cap or, for a
	// grid with as many stations, a vertex ring of the mesh, where a cut
	// finds a cap's triangles or nothing at all.
	constexpr int32 kLoadCuts = 20;

	// One entity's (or a mirrored copy's) share of a case: the solved body
	// it ended in and the panels of that body which are its own triangles.
	struct FShare
	{
		FString Name;                 // the entity (" (mirror)" for the reflected copy)
		int32 Body = -1;              // index into FCase::Bodies / Sol.bodies
		std::vector<char> Mask;       // one flag per panel of that body (filled by Derive)
		int32 Triangles = 0;          // its own aero mesh's triangles
		bool bWing = false;
		bool bTipToTip = false;       // a symmetric wing: the body spans both halves
		bool bMirrorCopy = false;
		FVector SpanAxis = FVector(0, 1, 0);
		FVector RootPoint = FVector::ZeroVector, TipPoint = FVector::ZeroVector;
		TArray<double> CutChords;
		bool bImaged = false;         // the plane was on: its image is the other half
	};

	// Wind-axis coefficients.
	struct FCoefs
	{
		double CL = 0.0, CD = 0.0, CY = 0.0, Cl = 0.0, Cm = 0.0, Cn = 0.0;
	};

	// A control the trim turns: a flap of a chosen wing.
	struct FControl
	{
		TWeakObjectPtr<AAerotelicEntityActor> Wing;
		int32 Flap = -1;
		FString Label;   // "<wing> / <flap>"
		bool IsSet() const { return Flap >= 0 && Wing.IsValid(); }
	};

	// One operating condition, built and solved.
	struct FCase
	{
		double AlphaDeg = 0.0, BetaDeg = 0.0;
		FString Control;
		double DeltaDeg = 0.0;
		aerotelic::Vec3 Stream, LiftDir, SideDir;
		std::vector<tripanel::Body> Bodies;      // as handed to the solver (fused where they overlapped)
		// Per body, per INPUT triangle, the share that owns it (the fused
		// triangles classified by the nearest original surface).
		std::vector<std::vector<int32>> Owner;
		TArray<FShare> Shares;
		TArray<FString> Skipped;
		TArray<FString> Notes;
		TArray<FString> FusedGroups;             // "Wing + Fuselage + Tail Boom"
		TArray<FString> Warnings;                // meshes not closed, bodies left overlapping
		FString MeshReport;                      // the statistics text, built with the case
		int32 Triangles = 0;                     // of every body together
		aerotelic::Vec3 MomentReference;         // the solve's
		bool bImagedSolve = false;               // the solve had the plane: every share is imaged
		double ReferenceAreaSolved = 1.0;        // the solve's reference area
		tripanel::Solution Sol;
		bool bSolved = false;
		double CL = 0.0, CD = 0.0, CY = 0.0, Cl = 0.0, Cm = 0.0, Cn = 0.0;
		TArray<FCoefs> ByBody;                   // per share
		double CpMin = 1.0, CpMax = -1e9;
		int32 NBody = 0, NWake = 0;
	};

	// What the paneling needs to know.
	struct FSetup
	{
		TArray<TWeakObjectPtr<AAerotelicEntityActor>> Entities;
		bool bSymmetry = false;
		bool bFuse = true;                      // fuse overlapping bodies before paneling
		double WakeChords = 20.0;               // wake length in reference lengths
		double ReferenceArea = 1.0, ReferenceLength = 1.0;   // project units
		TArray<AAerotelicEntityActor*> LiveEntities() const;
	};

	// The panelable entities among the scene's (IsPanelable, with a model).
	TArray<AAerotelicEntityActor*> PanelableEntities();
	// The planform area of the chosen wings (both halves each) and the
	// LARGEST wing's root chord (2026-11-20; the first wing's in the scene
	// order before, which a tailplane listed above the wing got wrong); with
	// no wing, the first body's frontal ellipse and its length. Project
	// units (the replica's). A boolean counts its operands' wings.
	void DefaultReferences(const TArray<AAerotelicEntityActor*>& Entities, double& OutArea, double& OutLength);
	// On when anything chosen is mirrored, a wing is symmetric (built tip to
	// tip about y = 0) or has its root on y = 0.
	bool DefaultSymmetry(const TArray<AAerotelicEntityActor*>& Entities);
	// The flaps of the chosen wings (a boolean's operands included);
	// OutDefault is the aftmost (an elevator).
	TArray<FControl> Controls(const TArray<AAerotelicEntityActor*>& Entities, int32& OutDefault);
	// The mass report of the entities (both halves of a mirrored one) with
	// the project's point masses: mass in kg, the centre in project units as
	// placed. False with no mass.
	bool MassAndCentre(const TArray<AAerotelicEntityActor*>& Entities, const aerotelic::UnitSystem& Units, double& OutMassKg, aerotelic::Vec3& OutCentre, FString& OutText);

	// Assembles the bodies of the chosen entities for one free stream, placed
	// as the entities sit, fused where they overlap, with the mesh report.
	// With a Control, that wing's flap is built at DeltaDeg instead of the
	// model's own (the model is put back as it was). Game thread. Throws on
	// a model that will not build or when nothing has a mesh.
	TSharedPtr<FCase> BuildCase(const FSetup& Setup, double AlphaDeg, double BetaDeg, const FControl* Control = nullptr, double DeltaDeg = 0.0);
	// The solver options a setup's cases are solved with.
	tripanel::Options SolveOptions(const FSetup& Setup, const aerotelic::Vec3* MomentReference = nullptr);
	// Solves a built case on the calling thread (the dialogs use FRunner or
	// their own worker; the tests this).
	void SolveCaseNow(FCase& C, const tripanel::Options& Options);
	// What is read off a solved case: wind-axis coefficients in all and per
	// share, the Cp percentile range, the shares' panel masks.
	void Derive(FCase& C, double ReferenceLength);

	// The mesh the solver will take, in the main 3D view with its edges
	// drawn (grey, lit), until hidden: the user's check of what is analysed.
	void ShowAeroMeshInViewport(const FCase& C);
	void HideAeroMeshInViewport();
	bool IsAeroMeshShown();

	// A trim's flight condition and the states its derivatives are taken
	// between.
	struct FCondition
	{
		double LoadFactor = 1.0;
		double Speed = 20.0, Density = 1.225;   // SI
		double MassKg = 0.0;
		aerotelic::Vec3 MassCentre;             // project units (the replica's)
		FControl Control;                        // unset: the angle of attack alone
		double Alpha0 = 2.0, Delta0 = 0.0;      // the base state
		double Step = 1.0;                      // degrees, both derivatives
		FString Name;                           // the load case's name

		// THE ITERATION (core aero_trim.hpp TrimDriver, 2026-11-19). One
		// Newton step lands where the LINEAR model is trimmed, which over a
		// step of fifteen degrees is not where the configuration is: Case 1 of
		// my_drone was stored from one at C_L -0.60 against the 1.24 it needed.
		// The defaults here are the trim; a caller only touches them to loosen
		// a case it cannot converge.
		// 12 and 6 because my_drone's Case 1 -- a V-n corner that wants a
		// twenty-degree excursion from the base state -- spent 8 corrections
		// and ran out of Jacobians while still 23% off, then oscillated on the
		// stale one (2026-11-19).
		int32 MaxIterations = 12;                // corrections solved after the first state
		int32 MaxJacobians = 6;                  // times the derivatives are measured
		double TolCLFraction = 0.005;            // of the required C_L
		double TolCLFloor = 0.002;
		double TolCm = 0.002;                    // about the mass centre
		double MaxStepDeg = 8.0;                 // trust region on one correction
		double JacobianMoveDeg = 5.0;            // re-measure once the state has moved this far
		double StallRatio = 1.5;                 // ... or when the residual stops falling

		// WHAT IS STORED. A converged trim is stored. One that did not
		// converge but whose lift is within this fraction of what the
		// condition needs is stored WITH A WARNING. Beyond it nothing is
		// stored: a structural analysis of a case that is not in equilibrium
		// is a waste of a solve (0: store whatever came out).
		double StoreLimitFraction = 0.05;

		// A state whose lift from the PRESSURES and from the WAKE CIRCULATION
		// disagree by more than this fraction of the latter (plus the floor)
		// is a broken panel solution, not an operating point: the trim stops
		// there rather than iterating on it or storing it. This is the
		// signature Case 1 showed (-0.60 against 3.5) when the wing's wake ran
		// through the tailplane (0: no check).
		double WakeCheckFraction = 0.10, WakeCheckFloor = 0.03;
	};

	// One solved state of the iteration, for the report.
	struct FTrimIterate
	{
		double AlphaDeg = 0.0, DeltaDeg = 0.0;
		double CL = 0.0, Cm = 0.0, CLWake = 0.0;
		bool bWake = false;                      // CLWake was reported (anything sheds a wake)
		double ResidualNorm = 0.0;
		bool bJacobian = false;                  // the derivatives were measured here
		bool bClamped = false;                   // the correction that led here was clamped
	};

	struct FOutcome
	{
		bool bOk = false;
		FString Report;                          // what happened, for the user
		TSharedPtr<FAerotelicAeroLoadCase> Case; // the stored pressure field, when bOk
		TSharedPtr<FCase> Trimmed;               // the best state solved (set even when nothing is stored)
		aerotelic::aero::TrimJacobian Jacobian;  // the last measured
		double CLRequired = 0.0, DynamicPressure = 0.0, Weight = 0.0, ReferenceAreaSI = 0.0;

		// The iteration.
		bool bConverged = false;                 // the trim reached its tolerance
		int32 Iterations = 0, Jacobians = 0, Solves = 0;
		double LiftErrorFraction = 0.0;          // of the state reported, against the required C_L
		double ResidualNorm = 0.0;               // of that state, in tolerances
		FString Warning;                         // set when a case is stored off target
		TArray<FTrimIterate> History;
	};

	// A solved case's panels as a load case (the image included when the
	// plane was on). Trim may be null for an untrimmed case.
	TSharedPtr<FAerotelicAeroLoadCase> MakeLoadCase(const FCase& C, const FSetup& Setup, const FString& Name, double DynamicPressure, const FCondition* Trim, const FOutcome* Outcome);

	// The trim, run asynchronously: the iteration of core's TrimDriver, whose
	// states this builds (game thread: it reads the actors) and solves (a
	// worker). Keep the runner alive (a shared pointer) until OnDone; Cancel
	// drops a run between solves.
	class FRunner : public TSharedFromThis<FRunner>
	{
	public:
		void Start(const FSetup& InSetup, const FCondition& InCondition, const aerotelic::UnitSystem& InUnits,
			TFunction<void(const FText&)> InOnStatus, TFunction<void(const FOutcome&)> InOnDone);
		void Cancel();
		bool IsRunning() const { return bRunning; }

	private:
		FSetup Setup;
		FCondition Condition;
		aerotelic::UnitSystem Units;
		TFunction<void(const FText&)> OnStatus;
		TFunction<void(const FOutcome&)> OnDone;
		FOutcome Outcome;
		bool bRunning = false;
		std::atomic<bool> bCancel{ false };
		int32 Generation = 0;

		TSharedPtr<aerotelic::aero::TrimDriver> Driver;
		TSharedPtr<FCase> LastCase;   // the state solved most recently (a re-measurement's base)
		TSharedPtr<FCase> BestCase;   // the case of the driver's best iterate
		FString MeshText;             // the mesh report, from the first case built
		// Why the iteration stopped early: a state the solver returned that is
		// not an operating point (see Accept). The best HEALTHY iterate up to
		// then is still reported and may still be stored.
		FString StopReason;

		void SolveCases(TArray<TSharedPtr<FCase>> Cases, const tripanel::Options& Options, TFunction<void(bool, const FString&)> Then);
		// Acts on what the driver asks for next; the three below are its arms.
		void Advance();
		void RequestDerivatives();
		void RequestState();
		void FinishFromDriver();
		// Derives a solved case and checks it is an operating point at all
		// (the pressure lift against the wake circulation's). False with why.
		bool Accept(FCase& C, FString& OutWhy) const;
		// Records an iterate in the report's history and keeps BestCase in step.
		void Note(const TSharedPtr<FCase>& C, bool bJacobianHere, bool bClamped);
		FString HistoryText() const;
		void Finish(bool bOk, const FString& Why);
	};
}
