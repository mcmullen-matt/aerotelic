#pragma once

#include "CoreMinimal.h"

// The few user preferences the application keeps between runs -- the
// QSettings keys of WingDesigner (startup/show_splash) -- stored in the
// game user settings ini under [Aerotelic].
namespace AerotelicSettings
{
	bool ShowSplashAtStartup();        // default true
	void SetShowSplashAtStartup(bool bShow);
}
