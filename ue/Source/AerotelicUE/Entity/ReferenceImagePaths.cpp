#include "ReferenceImagePaths.h"

#include "Misc/Paths.h"

#include "aerotelic/wing_model.hpp"
#include "aerotelic/fuselage_model.hpp"
#include "aerotelic/nacelle_model.hpp"
#include "aerotelic/lofted_nacelle_model.hpp"
#include "aerotelic/propeller_model.hpp"

using namespace aerotelic;
using namespace aerotelic::project_yaml;

namespace
{
	FString Normalised(FString Path)
	{
		FPaths::NormalizeFilename(Path);
		return Path;
	}

	void Relativise(ReferenceImageMap& Images, const FString& ProjectDir)
	{
		if (ProjectDir.IsEmpty()) return;
		const FString Dir = Normalised(ProjectDir) / TEXT("");
		for (auto& [Key, Spec] : Images)
		{
			FString Path = Normalised(UTF8_TO_TCHAR(Spec.path.c_str()));
			if (Path.IsEmpty() || FPaths::IsRelative(Path)) continue;
			// Only when the image sits inside the project's directory; a
			// result starting with ".." (or on another drive) stays absolute.
			if (Path.StartsWith(Dir, ESearchCase::IgnoreCase))
			{
				Spec.path = TCHAR_TO_UTF8(*Path.Mid(Dir.Len()));
			}
		}
	}

	void Resolve(ReferenceImageMap& Images, const FString& ProjectDir)
	{
		for (auto& [Key, Spec] : Images)
		{
			FString Path = UTF8_TO_TCHAR(Spec.path.c_str());
			if (Path.IsEmpty() || !FPaths::IsRelative(Path) || ProjectDir.IsEmpty()) continue;
			Spec.path = TCHAR_TO_UTF8(*Normalised(FPaths::Combine(ProjectDir, Path)));
		}
	}
}

void RelativiseReferenceImagePaths(EntityModelSpec& Spec, const FString& ProjectDir)
{
	std::visit([&ProjectDir](auto& S)
	{
		using T = std::decay_t<decltype(S)>;
		if constexpr (!std::is_same_v<T, PrimitiveSpec>)
		{
			Relativise(S.reference_images, ProjectDir);
			if constexpr (std::is_same_v<T, PropellerSpec>)
			{
				if (S.blade) Relativise(S.blade->reference_images, ProjectDir);
			}
		}
	}, Spec);
}

void ResolveReferenceImagePaths(ISolidModel* Model, EntityType Type, const FString& ProjectDir)
{
	if (Model == nullptr) return;
	switch (Type)
	{
	case EntityType::Wing: Resolve(static_cast<WingModel*>(Model)->reference_images, ProjectDir); break;
	case EntityType::Fuselage: Resolve(static_cast<FuselageModel*>(Model)->reference_images, ProjectDir); break;
	case EntityType::Nacelle: Resolve(static_cast<NacelleModel*>(Model)->reference_images, ProjectDir); break;
	case EntityType::LoftedNacelle: Resolve(static_cast<LoftedNacelleModel*>(Model)->reference_images, ProjectDir); break;
	case EntityType::Propeller:
	{
		PropellerModel* P = static_cast<PropellerModel*>(Model);
		Resolve(P->reference_images, ProjectDir);
		Resolve(P->blade().reference_images, ProjectDir);
		break;
	}
	default: break;
	}
}
