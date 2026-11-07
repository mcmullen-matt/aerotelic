#include "AerotelicMaterialLibrary.h"

#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"

#include "Licensing/AerotelicLicence.h"

DEFINE_LOG_CATEGORY_STATIC(LogAerotelicMaterials, Log, All);

namespace AerotelicMaterialLibrary
{
	namespace
	{
		struct FState
		{
			bool bLoaded = false;
			FString PathOverride;
			TArray<aerotelic::Material> Custom;
			FSimpleMulticastDelegate Changed;
		};

		FState& State()
		{
			static FState S;
			return S;
		}

		void Load()
		{
			FState& S = State();
			S.bLoaded = true;
			S.Custom.Reset();
			const FString Path = FilePath();
			if (!FPaths::FileExists(Path))
			{
				return;
			}
			FString Text;
			if (!FFileHelper::LoadFileToString(Text, *Path))
			{
				UE_LOG(LogAerotelicMaterials, Warning, TEXT("Material library: could not read %s"), *Path);
				return;
			}
			try
			{
				for (const aerotelic::Material& M : aerotelic::ReadMaterialsYaml(TCHAR_TO_UTF8(*Text)))
				{
					S.Custom.Add(M);
				}
			}
			catch (const std::exception& e)
			{
				// Keep the unreadable file where it is and set it aside under
				// another name, so the next save cannot bury whatever is in it.
				const FString Aside = Path + TEXT(".unreadable");
				IFileManager::Get().Copy(*Aside, *Path, /*Replace*/ true);
				UE_LOG(LogAerotelicMaterials, Warning, TEXT("Material library: %s is not readable (%s); a copy was set aside as %s and the library starts empty"),
					*Path, UTF8_TO_TCHAR(e.what()), *Aside);
				S.Custom.Reset();
			}
		}

		void EnsureLoaded()
		{
			if (!State().bLoaded)
			{
				Load();
			}
		}
	}

	FString FilePath()
	{
		const FState& S = State();
		if (!S.PathOverride.IsEmpty())
		{
			return S.PathOverride;
		}
		return FPaths::Combine(AerotelicLicence::UserDataDir(), TEXT("materials.yaml"));
	}

	void SetFilePathOverride(const FString& Path)
	{
		State().PathOverride = Path;
		Load();
	}

	const TArray<aerotelic::Material>& Custom()
	{
		EnsureLoaded();
		return State().Custom;
	}

	TArray<aerotelic::Material> All()
	{
		TArray<aerotelic::Material> Out;
		for (const aerotelic::Material& M : aerotelic::MaterialPresets())
		{
			Out.Add(M);
		}
		Out.Append(Custom());
		return Out;
	}

	const aerotelic::Material* Find(const std::string& Name)
	{
		if (const aerotelic::Material* P = aerotelic::FindMaterialPreset(Name))
		{
			return P;
		}
		for (const aerotelic::Material& M : Custom())
		{
			if (M.name == Name)
			{
				return &M;
			}
		}
		return nullptr;
	}

	const aerotelic::Material* FindMatching(const aerotelic::Material& M)
	{
		const aerotelic::Material* Found = Find(M.name);
		return Found != nullptr && aerotelic::SameMaterial(*Found, M) ? Found : nullptr;
	}

	bool SetCustom(const TArray<aerotelic::Material>& Materials, FString& OutError)
	{
		EnsureLoaded();
		for (int32 i = 0; i < Materials.Num(); ++i)
		{
			const aerotelic::Material& M = Materials[i];
			const std::string Problem = aerotelic::MaterialProblem(M);
			if (!Problem.empty())
			{
				OutError = FString::Printf(TEXT("'%s': %s"), UTF8_TO_TCHAR(M.name.c_str()), UTF8_TO_TCHAR(Problem.c_str()));
				return false;
			}
			if (aerotelic::FindMaterialPreset(M.name) != nullptr)
			{
				OutError = FString::Printf(TEXT("'%s' is the name of a preset; give yours another name"), UTF8_TO_TCHAR(M.name.c_str()));
				return false;
			}
			for (int32 j = 0; j < i; ++j)
			{
				if (Materials[j].name == M.name)
				{
					OutError = FString::Printf(TEXT("two materials are named '%s'; names must be unique"), UTF8_TO_TCHAR(M.name.c_str()));
					return false;
				}
			}
		}

		std::vector<aerotelic::Material> ToWrite;
		for (const aerotelic::Material& M : Materials)
		{
			ToWrite.push_back(M);
		}
		const FString Path = FilePath();
		IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), /*Tree*/ true);
		const FString Text = UTF8_TO_TCHAR(aerotelic::WriteMaterialsYaml(ToWrite).c_str());
		if (!FFileHelper::SaveStringToFile(Text, *Path, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
		{
			OutError = FString::Printf(TEXT("could not write %s"), *Path);
			return false;
		}
		State().Custom = Materials;
		State().Changed.Broadcast();
		return true;
	}

	FSimpleMulticastDelegate& OnChanged()
	{
		return State().Changed;
	}
}
