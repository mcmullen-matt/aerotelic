#pragma once

#include "CoreMinimal.h"
#include "Commandlets/Commandlet.h"
#include "AerotelicLicenceTestCommandlet.generated.h"

// Drives the licence verifier against the server's half of the scheme
// generated in process (an Ed25519 pair, signed envelopes), as
// WingDesigner's tests/test_licence.py does through its in-process wire:
// round trip, tamper, wrong key, key rotation, type confusion, machine id,
// candidate preference, offline grace, clock rollback, expiry against the
// trusted time. Run via `-run=AerotelicLicenceTest`.
UCLASS()
class UAerotelicLicenceTestCommandlet : public UCommandlet
{
	GENERATED_BODY()

public:
	virtual int32 Main(const FString& Params) override;
};
