#include "AerotelicSettings.h"

#include "Misc/ConfigCacheIni.h"

namespace
{
	const TCHAR* kSection = TEXT("Aerotelic");
	const TCHAR* kShowSplash = TEXT("ShowSplashAtStartup");
}

namespace AerotelicSettings
{
	bool ShowSplashAtStartup()
	{
		bool bShow = true;
		if (GConfig != nullptr)
		{
			GConfig->GetBool(kSection, kShowSplash, bShow, GGameUserSettingsIni);
		}
		return bShow;
	}

	void SetShowSplashAtStartup(bool bShow)
	{
		if (GConfig != nullptr)
		{
			GConfig->SetBool(kSection, kShowSplash, bShow, GGameUserSettingsIni);
			GConfig->Flush(false, GGameUserSettingsIni);
		}
	}
}
