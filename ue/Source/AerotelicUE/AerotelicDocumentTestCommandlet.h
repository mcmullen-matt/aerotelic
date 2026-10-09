#pragma once

#include "CoreMinimal.h"
#include "Commandlets/Commandlet.h"
#include "AerotelicDocumentTestCommandlet.generated.h"

// Headless functional check of UAerotelicDocumentSubsystem: adds one entity
// of each type, saves to a temp file, starts a new (empty) project, reopens
// the saved file, and confirms the entity count/types/placements round-trip.
// Run via `-run=AerotelicDocumentTest`. Exercises the Phase-A entity/document
// layer before any dialog UI is built on top of it.
UCLASS()
class UAerotelicDocumentTestCommandlet : public UCommandlet
{
	GENERATED_BODY()

public:
	virtual int32 Main(const FString& Params) override;
};
