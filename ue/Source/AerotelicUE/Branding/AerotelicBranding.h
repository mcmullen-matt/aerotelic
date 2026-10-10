#pragma once

#include "CoreMinimal.h"

// Product identity and the legal text shown on the welcome screen and in
// Help -> About -- WingDesigner's ui/branding.py. Both read from here so the
// two cannot drift apart.
//
// THE DISCLAIMER TEXT IS A PLACEHOLDER, as it is in WingDesigner: drafted to
// cover the ground an engineering pre-design tool obviously needs, not
// reviewed by counsel, and not to ship as written. Replace Disclaimers()
// wholesale with the reviewed wording; the welcome screen lays out whatever
// clauses it finds.
namespace AerotelicBranding
{
	// The distributor, separate from the product so renaming one never
	// silently renames the other.
	const TCHAR* const Company = TEXT("MS2 Aerodynamics");
	const TCHAR* const Product = TEXT("Aerotelic");
	const TCHAR* const Tagline = TEXT("Parametric aircraft surface modelling");
	// Relative to the project (resource) root; the author's own artwork.
	const TCHAR* const SplashImage = TEXT("Resources/WingDesigner_splash_screen.png");

	// From the VERSION file the release workflow stamps (Resources/VERSION,
	// then the project root); a source checkout has none and reports itself
	// as a dev build.
	FString Version();
	FString Copyright();
	// Each clause's heading is the upper-case run up to the first ". ".
	TArray<FString> Disclaimers();
	FString DisclaimerText();      // the clauses joined by blank lines
	FString AboutText();           // Help -> About body
}
