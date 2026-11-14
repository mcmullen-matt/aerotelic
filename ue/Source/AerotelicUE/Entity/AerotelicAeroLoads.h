#pragma once

#include "CoreMinimal.h"

#include <vector>

#include "aerotelic/aero_loads.hpp"
#include "aerotelic/vec3.hpp"

#include "AerotelicStructuralFea.h"

// An AERODYNAMIC LOAD CASE (stage 4 of the flight-loads plan, 2026-11-14):
// the surface pressure of one high order panel solution, kept on the
// document for the structural analysis to pick up -- the bridge from
// Analysis > High Order Panel (which makes them: its Trim tab) to Analysis
// > Structural (which applies them: the Loads tab's aerodynamic pressure).
//
// What is kept is the panel field itself -- every body panel's centre,
// outward normal, area and pressure coefficient, placed in the world as the
// bodies sit (project units, the replica's when a model scale is set) --
// with the dynamic pressure that turns Cp into pascals, so the structural
// shells can be loaded whatever grid they are built on later. A solution
// with the symmetry plane carries only the y >= 0 half; the case holds the
// image too, so the whole airframe is covered.
//
// Session data: cleared with the scene, not saved (a trimmed case is a few
// seconds to redo from the panel dialog, and its panel field would double
// the project file).
struct FAerotelicAeroLoadCase
{
	FString Name;
	double DynamicPressure = 0.0;   // Pa
	double AlphaDeg = 0.0, BetaDeg = 0.0;
	// The control deflection the case was solved with (degrees, trailing
	// edge down) and which flap it is ("<entity> / <flap>"; empty: none).
	double DeltaDeg = 0.0;
	FString Control;
	// A TRIMMED case: the load factor and weight (N) it was trimmed to, its
	// lift and moment coefficients as solved (Cm about the mass centre), and
	// the mass centre (project units).
	bool bTrimmed = false;
	double LoadFactor = 0.0;
	double Weight = 0.0;
	double CL = 0.0, Cm = 0.0;
	aerotelic::Vec3 MassCentre;
	double ReferenceArea = 0.0;     // project units^2
	aerotelic::Vec3 Stream;         // unit, world
	TArray<FString> Entities;       // the scene names paneled
	std::vector<aerotelic::aero::PressureSample> Samples;

	// The samples' own resultant, -sum Cp n A (the image included), over
	// the reference area: should agree with the solver's force coefficient.
	aerotelic::Vec3 ForceCoefficient() const;
	int32 BodyPanels() const { return static_cast<int32>(Samples.size()); }
};

namespace AerotelicAeroLoads
{
	struct FMapResult
	{
		TArray<double> Pressure;        // Pa, one per shell face
		TArray<uint8> Mapped;           // 1 where a panel pressure landed
		aerotelic::aero::MapStats Stats;
		int32 FacesCovered = 0;         // faces whose region the case covers
		FVector Resultant = FVector::ZeroVector;   // N, -sum p n A over the mapped faces (project-unit areas scaled by LengthToSI^2)
	};

	// The case's pressure onto the shell faces (FRequest::ShellFaces over
	// Nodes, project units): q Cp on the faces of regions the case covers
	// (`Covered(RegionId)`, the dialog's rule: the region's entity, or an
	// operand of its boolean result, is one the case paneled), plus
	// UniformPressure (Pa) on every face; faces no panel describes get the
	// uniform pressure alone and are counted in Stats.unmapped.
	void MapToFaces(const FAerotelicAeroLoadCase& Case, const std::vector<aerotelic::Vec3>& Nodes, const TArray<AerotelicStructuralFea::FShellFace>& Faces,
		TFunctionRef<bool(int32 RegionId)> Covered, double UniformPressure, double LengthToSI, FMapResult& Out);
}
