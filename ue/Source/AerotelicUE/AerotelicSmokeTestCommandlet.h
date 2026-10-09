#pragma once

#include "CoreMinimal.h"
#include "Commandlets/Commandlet.h"
#include "AerotelicSmokeTestCommandlet.generated.h"

// Throwaway runtime smoke test: loads a real Python-saved project file
// through the full aerotelic_core -> project_yaml chain, inside the real
// UE module (not just the standalone *_fixture_check.exe programs), and
// builds one real solid. Confirms the Build.cs linkage/DLL staging is
// correct at runtime, not just at link time.
UCLASS()
class UAerotelicSmokeTestCommandlet : public UCommandlet
{
	GENERATED_BODY()

public:
	virtual int32 Main(const FString& Params) override;
};
