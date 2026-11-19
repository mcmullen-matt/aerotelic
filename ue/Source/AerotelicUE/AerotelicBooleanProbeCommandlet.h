#pragma once

#include "CoreMinimal.h"
#include "Commandlets/Commandlet.h"
#include "AerotelicBooleanProbeCommandlet.generated.h"

// A headless probe of the mesh boolean on a real project (Matt, 2026-11-18:
// "try a boolean fuse on all the solid bodies within my_drone except the
// propellers; see if it throws an error"). Opens the project, takes every
// non-derived entity but the propellers (a mirrored one twice: itself and
// its mirror copy, as the Boolean dialog lists them) in file order, and
// fuses them through UAerotelicDocumentSubsystem::AddBoolean -- the very
// call the dialog makes. The error text, when there is one, is printed; on
// a failure the fuse is retried on growing prefixes of the operand list so
// the first operand that breaks it is named. A result is described body by
// body (closed, outward, triangles, volume).
//
//   -run=AerotelicBooleanProbe -Project=<path to .yaml> [-Op=fuse|cut|common]
//        [-Include=<comma separated entity names>] [-Exclude=<names>]
//
// Project defaults to ue/Projects/my_drone.yaml; the propellers are always
// left out unless named in -Include.
//
// A second mode runs the PANEL METHOD headlessly on one entity, as the
// Panel Method dialog would with its defaults (symmetry plane by the
// default rule, fuse on, 20 reference lengths of wake), and prints what the
// dialog's Span Loading tab computes for every wing share -- each cut's
// station, chord, l/q, cl, the cut outline's length and its Cp range -- plus
// how close every wake sheet runs to each share's panels (Matt, 2026-11-18:
// "the Cl on the aft boom looks erroneous"):
//
//   -run=AerotelicBooleanProbe -Panel=<entity name> [-Alpha=4] [-Beta=0]
//        [-Project=<path>] [-NoSymmetry] [-Share=<share name substring>]
UCLASS()
class UAerotelicBooleanProbeCommandlet : public UCommandlet
{
	GENERATED_BODY()

public:
	virtual int32 Main(const FString& Params) override;
};
