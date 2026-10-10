#include "AerotelicRealProjectTestCommandlet.h"

#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"
#include "Entity/ReferenceImagePaths.h"
#include "Misc/Paths.h"

#include "aerotelic/fuselage_model.hpp"

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
	const FString Path = TEXT("C:\\Users\\Matt\\devel_1\\WingDesigner\\projects\\p38.yaml");
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

	// Display state the file carries: every entity but Center is mirrored,
	// and four carry a colour override (the Wing's is navy). Both must
	// survive a save and a reload -- the first saver dropped them.
	auto Find = [Doc](const TCHAR* Name) -> AAerotelicEntityActor*
	{
		for (auto& WeakEntity : Doc->GetEntities())
		{
			if (WeakEntity.IsValid() && WeakEntity->EntityName == Name) return WeakEntity.Get();
		}
		return nullptr;
	};
	auto CheckState = [&](const TCHAR* Stage)
	{
		AAerotelicEntityActor* Center = Find(TEXT("Center"));
		AAerotelicEntityActor* Wing = Find(TEXT("Wing"));
		AAerotelicEntityActor* Boom = Find(TEXT("Boom"));
		AAerotelicEntityActor* Prop = Find(TEXT("Propeller 7"));
		Check(Center && Wing && Boom && Prop, FString::Printf(TEXT("%s: Center, Wing, Boom and Propeller 7 are present"), Stage));
		if (!(Center && Wing && Boom && Prop)) return;
		Check(!Center->IsMirrored() && Wing->IsMirrored() && Boom->IsMirrored() && Prop->IsMirrored(),
			FString::Printf(TEXT("%s: mirror flags (Center off; Wing, Boom, Propeller 7 on)"), Stage));
		const TOptional<FLinearColor> WingColor = Wing->GetColorOverride();
		Check(WingColor.IsSet() && FMath::IsNearlyEqual(WingColor->B, 0.498f, 0.01f) && WingColor->R < 0.01f && WingColor->G < 0.01f,
			FString::Printf(TEXT("%s: Wing colour override is navy (0, 0, 0.498)"), Stage));
		Check(!Center->GetColorOverride().IsSet(), FString::Printf(TEXT("%s: Center has no colour override (per-kind default applies)"), Stage));
		// (Bodies are not checked: a commandlet never tessellates, so the
		// body list is only filled in the interactive app.)
	};
	CheckState(TEXT("after open"));

	// Reference-image paths: beside the project file they are written
	// relative to it, elsewhere absolute; relative paths resolve on load.
	{
		const FString Dir = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir());
		aerotelic::FuselageSpec Fs;
		Fs.reference_images["profile"].path = TCHAR_TO_UTF8(*(Dir / TEXT("three_view.png")));
		Fs.reference_images["width"].path = TCHAR_TO_UTF8(*FString(TEXT("D:/elsewhere/top.png")));
		aerotelic::project_yaml::EntityModelSpec Spec = Fs;
		RelativiseReferenceImagePaths(Spec, Dir);
		const aerotelic::FuselageSpec& Out = std::get<aerotelic::FuselageSpec>(Spec);
		Check(Out.reference_images.at("profile").path == "three_view.png",
			FString::Printf(TEXT("an image beside the project file is saved relative to it (got %s)"), UTF8_TO_TCHAR(Out.reference_images.at("profile").path.c_str())));
		Check(Out.reference_images.at("width").path == "D:/elsewhere/top.png", "an image elsewhere stays absolute");
		aerotelic::FuselageModel Model(Out);
		ResolveReferenceImagePaths(&Model, aerotelic::project_yaml::EntityType::Fuselage, Dir);
		Check(FString(UTF8_TO_TCHAR(Model.reference_images.at("profile").path.c_str())).Equals(Dir / TEXT("three_view.png"), ESearchCase::IgnoreCase),
			FString::Printf(TEXT("a relative path resolves against the project directory on load (got %s)"), UTF8_TO_TCHAR(Model.reference_images.at("profile").path.c_str())));
	}

	// The other shipped tutorial is in WingDesigner's legacy format (entities
	// keyed by id + label, a raw features graph): it must load too.
	const FString Cessna = FPaths::Combine(FPaths::ProjectDir(), TEXT("Projects"), TEXT("cessna_172.yaml"));
	Check(Doc->OpenProjectPath(Cessna, /*bAdopt*/ false), "opened the legacy-format cessna_172.yaml tutorial");
	Check(Doc->GetEntities().Num() == 2, FString::Printf(TEXT("the Cessna has 2 entities (got %d)"), Doc->GetEntities().Num()));
	Check(!Doc->HasCurrentPath(), "a tutorial is not adopted as the current file");
	Check(Find(TEXT("Wing 2")) != nullptr && Find(TEXT("Wing 2"))->IsMirrored(), "the legacy label is the entity name and its mirror flag loaded");
	Check(Doc->OpenProjectPath(Path), "reopened p38");

	const FString TempPath = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("p38_roundtrip.yaml"));
	Check(Doc->SaveProjectToPath(TempPath), "saved the project to Saved/p38_roundtrip.yaml");
	Check(Doc->OpenProjectPath(TempPath), "reopened the saved copy");
	Check(Doc->GetEntities().Num() == 7, FString::Printf(TEXT("reopened entity count is 7 (got %d)"), Doc->GetEntities().Num()));
	CheckState(TEXT("after save+reload"));

	UE_LOG(LogAerotelicRealProjectTest, Display, TEXT("RESULT: %d passed, %d failed"), g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}
