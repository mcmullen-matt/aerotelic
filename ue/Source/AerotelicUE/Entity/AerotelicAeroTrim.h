#pragma once

#include "CoreMinimal.h"

#include <atomic>
#include <vector>

#include "aerotelic/aero_trim.hpp"
#include "aerotelic/panel_method.hpp"
#include "aerotelic/project_yaml.hpp"
#include "aerotelic/units.hpp"
#include "aerotelic/vec3.hpp"

class AAerotelicEntityActor;
struct FAerotelicAeroLoadCase;

// The high order panel method on the scene's lofted entities, and the
// rigid trim on it (stage 4 of the flight-loads plan), as a headless
// module (2026-11-15; lifted out of the High Order Panel dialog so that
// Model > Load Cases and the Kratos dialog trim with the same code):
//
//   - FSetup: which entities, the paneling counts, the symmetry plane and
//     the references;
//   - BuildCase: the networks of every chosen entity for one free stream,
//     placed as the entities sit (the game thread: it reads the actors);
//     Derive: the wind-axis coefficients of a solved case;
//   - MakeLoadCase: a solved case's body panels as a load case for the
//     structural analysis (AerotelicAeroLoads.h);
//   - FRunner: the trim -- base, alpha stepped, flap stepped, the 2 x 2
//     step, the trimmed state -- the panels built on the game thread, the
//     solves on a worker, the outcome delivered on the game thread.
namespace AerotelicAeroTrim
{
	using aerotelic::project_yaml::EntityType;
	using aerotelic::panel::Network;
	using aerotelic::panel::NetworkResult;
	using aerotelic::panel::NetworkType;
	using aerotelic::panel::PanelResult;
	using aerotelic::panel::Solution;

	// The bodies the method can panel: every lofted solid. Boolean results
	// are triangle soups with no sections to spline; a propeller is left
	// out (rotating blades are not a steady potential flow).
	bool IsPanelable(EntityType T);

	// The span loading is cut every 5% of the semi-span.
	constexpr int32 kLoadCuts = 20;

	// One entity's share of a case's network list.
	struct FShare
	{
		FString Name;
		int32 First = 0, Count = 0, OwnCount = 0;
		bool bWing = false;
		FVector SpanAxis = FVector(0, 1, 0);
		FVector RootPoint = FVector::ZeroVector, TipPoint = FVector::ZeroVector;
		TArray<double> CutChords;
		bool bImaged = false;
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
		std::vector<Network> Nets;
		TArray<FShare> Shares;
		TArray<FString> Skipped;
		TArray<FString> Notes;
		Solution Sol;
		bool bSolved = false;
		double CL = 0.0, CD = 0.0, CY = 0.0, Cl = 0.0, Cm = 0.0, Cn = 0.0;
		TArray<FCoefs> ByBody;
		double CpMin = 1.0, CpMax = -1e9;
		int32 NBody = 0, NWake = 0;
	};

	// What the paneling needs to know.
	struct FSetup
	{
		TArray<TWeakObjectPtr<AAerotelicEntityActor>> Entities;
		bool bSymmetry = false;
		double Chordwise = 12.0, Spanwise = 10.0, WakeChords = 20.0;
		double Along = 16.0, Around = 12.0;
		double ReferenceArea = 1.0, ReferenceLength = 1.0;   // project units
		TArray<AAerotelicEntityActor*> LiveEntities() const;
	};

	// The panelable entities among the scene's (IsPanelable, with a model).
	TArray<AAerotelicEntityActor*> PanelableEntities();
	// The planform area of the chosen wings (both halves each) and the first
	// wing's root chord; with no wing, the first body's frontal ellipse and
	// its length. Project units (the replica's).
	void DefaultReferences(const TArray<AAerotelicEntityActor*>& Entities, double& OutArea, double& OutLength);
	// On when anything chosen is mirrored, or a wing has its root on y = 0.
	bool DefaultSymmetry(const TArray<AAerotelicEntityActor*>& Entities);
	// The flaps of the chosen wings; OutDefault is the aftmost (an elevator).
	TArray<FControl> Controls(const TArray<AAerotelicEntityActor*>& Entities, int32& OutDefault);
	// The mass report of the entities (both halves of a mirrored one) with
	// the project's point masses: mass in kg, the centre in project units as
	// placed. False with no mass.
	bool MassAndCentre(const TArray<AAerotelicEntityActor*>& Entities, const aerotelic::UnitSystem& Units, double& OutMassKg, aerotelic::Vec3& OutCentre, FString& OutText);

	// Panels the chosen entities for one free stream. With a Control, that
	// wing's flap is built at DeltaDeg instead of the model's own (the
	// model is put back as it was). Game thread. Throws on a model that
	// will not build or when nothing has a loft.
	TSharedPtr<FCase> BuildCase(const FSetup& Setup, double AlphaDeg, double BetaDeg, const FControl* Control = nullptr, double DeltaDeg = 0.0);
	// The solver options a setup's cases are solved with.
	aerotelic::panel::Options SolveOptions(const FSetup& Setup, const aerotelic::Vec3* MomentReference = nullptr);
	// What is read off a solved case: wind-axis coefficients, per body, the
	// Cp percentile range.
	void Derive(FCase& C, double ReferenceLength);

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
	};

	struct FOutcome
	{
		bool bOk = false;
		FString Report;                          // what happened, for the user
		TSharedPtr<FAerotelicAeroLoadCase> Case; // the stored pressure field, when bOk
		TSharedPtr<FCase> Trimmed;               // the trimmed state, when bOk
		TArray<TSharedPtr<FCase>> Derivatives;   // base, alpha step, (delta step)
		aerotelic::aero::TrimResult Result;
		double CLRequired = 0.0, DynamicPressure = 0.0, Weight = 0.0, ReferenceAreaSI = 0.0;
	};

	// A solved case's body panels as a load case (the image included when
	// the plane was on). Trim may be null for an untrimmed case.
	TSharedPtr<FAerotelicAeroLoadCase> MakeLoadCase(const FCase& C, const FSetup& Setup, const FString& Name, double DynamicPressure, const FCondition* Trim, const FOutcome* Outcome);

	// The trim, run asynchronously. Keep the runner alive (a shared pointer)
	// until OnDone; Cancel drops a run between solves.
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

		void SolveCases(TArray<TSharedPtr<FCase>> Cases, const aerotelic::panel::Options& Options, TFunction<void(bool, const FString&)> Then);
		void OnDerivativesSolved();
		void Finish(bool bOk, const FString& Why);
	};
}
