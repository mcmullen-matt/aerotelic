#pragma once

#include "CoreMinimal.h"

#include "AerotelicBooleanModel.h"
#include "AerotelicEntityActor.h"

// One solid body of a scene entity, or of its mirrored half, by role name
// ("Main", "Flap 1", "Blade 2", "Body 1"): what the structural analysis,
// the mass report and the Structure dialog address since the structure
// moved onto the bodies (2026-11-08). The operand form -- entity + mirror,
// FAerotelicBooleanOperand -- is what the booleans, the exports and the
// flow solvers take, and a ref gives it back. An empty Body means the
// entity's every body.
struct FAerotelicBodyRef
{
	TWeakObjectPtr<AAerotelicEntityActor> Entity;
	bool bMirror = false;
	FString Body;

	FAerotelicBooleanOperand Operand() const
	{
		FAerotelicBooleanOperand Op;
		Op.Entity = Entity;
		Op.bMirror = bMirror;
		return Op;
	}

	// The body's structure (its own: a mirrored half shares the body's),
	// or null when the entity is gone or has no such body.
	const aerotelic::BodyStructure* Structure() const
	{
		return Entity.IsValid() ? Entity->GetBodyStructure(Body) : nullptr;
	}

	// "<entity>[ (mirror)][ / <body>]", the label every report uses.
	FString Label() const
	{
		if (!Entity.IsValid()) return TEXT("(deleted entity)");
		FString L = Entity->EntityName;
		if (bMirror) L += TEXT(" (mirror)");
		if (!Body.IsEmpty() && Entity->GetBodies().Num() > 1) L += TEXT(" / ") + Body;
		return L;
	}

	bool operator==(const FAerotelicBodyRef& Other) const
	{
		return Entity == Other.Entity && bMirror == Other.bMirror && Body == Other.Body;
	}
};
