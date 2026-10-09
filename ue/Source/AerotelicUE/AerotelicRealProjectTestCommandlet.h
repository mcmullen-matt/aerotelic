#pragma once

#include "CoreMinimal.h"
#include "Commandlets/Commandlet.h"
#include "AerotelicRealProjectTestCommandlet.generated.h"

// Opens a real Python-saved project file (aerotelic/projects/p38.yaml)
// through the full UAerotelicDocumentSubsystem::OpenProjectPath path --
// not just this port's own round-trip -- and checks the entity count/types
// match the file's own `type:` lines. Run via `-run=AerotelicRealProjectTest`.
UCLASS()
class UAerotelicRealProjectTestCommandlet : public UCommandlet
{
	GENERATED_BODY()

public:
	virtual int32 Main(const FString& Params) override;
};
