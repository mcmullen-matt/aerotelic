#include "AerotelicRealProjectTestCommandlet.h"

#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"

DEFINE_LOG_CATEGORY_STATIC(LogAerotelicRealProjectTest, Log, All);

namespace
{
	int32 g_pass = 0, g_fail = 0;

	void Check(bool bCond, const FString& Msg)
	{
		if (bCond) { ++g_pass; return; }
		++g_fail;
		UE_LOG(LogAerotelicRealProjectTest, Error, TEXT("FAIL: %s"), *Msg);
	}
}

int32 UAerotelicRealProjectTestCommandlet::Main(const FString& Params)
{
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	Check(Doc != nullptr, "document subsystem is available");
	if (Doc == nullptr)
	{
		UE_LOG(LogAerotelicRealProjectTest, Display, TEXT("RESULT: %d passed, %d failed"), g_pass, g_fail);
		return 1;
	}

	// The Python app's repository (sibling of this one) ships the sample project.
	const FString Path = TEXT("C:\\Users\\Matt\\devel_1\\aerotelic_python\\projects\\p38.yaml");
	Check(Doc->OpenProjectPath(Path), "opened the real Python-saved p38.yaml");
	Check(Doc->GetEntities().Num() == 7, FString::Printf(TEXT("entity count is 7 (got %d)"), Doc->GetEntities().Num()));

	int32 nFuselage = 0, nWing = 0, nNacelle = 0, nPropeller = 0;
	for (auto& WeakEntity : Doc->GetEntities())
	{
		if (!WeakEntity.IsValid()) continue;
		switch (WeakEntity->GetEntityType())
		{
		case aerotelic::project_yaml::EntityType::Fuselage: ++nFuselage; break;
		case aerotelic::project_yaml::EntityType::Wing: ++nWing; break;
		case aerotelic::project_yaml::EntityType::Nacelle: ++nNacelle; break;
		case aerotelic::project_yaml::EntityType::Propeller: ++nPropeller; break;
		default: break;
		}
	}
	Check(nFuselage == 2 && nWing == 3 && nNacelle == 1 && nPropeller == 1,
		FString::Printf(TEXT("entity type mix matches the file (fuselage=%d wing=%d nacelle=%d propeller=%d)"), nFuselage, nWing, nNacelle, nPropeller));

	UE_LOG(LogAerotelicRealProjectTest, Display, TEXT("RESULT: %d passed, %d failed"), g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}
