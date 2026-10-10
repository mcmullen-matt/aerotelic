#pragma once

#include "CoreMinimal.h"
#include "AerotelicLicence.h"

// The licence state as the welcome screen shows it -- WingDesigner's
// ui/licensing.py LicenseInfo: display rows only, no decisions.
struct FAerotelicLicenseInfo
{
	enum class ESeverity : uint8 { Ok, Warn, Expired };

	FString Licensee, Organisation, Edition, Serial, Seats;
	FString Status;              // the check's message, or a note
	bool bChecked = false;       // Expires / Time remaining shown only when true
	TOptional<FDateTime> Expires;
	FString TimeRemaining;
	ESeverity Severity = ESeverity::Ok;

	static FAerotelicLicenseInfo Development();   // running from source / the Editor binaries
	static FAerotelicLicenseInfo Unlicensed();
	static FAerotelicLicenseInfo FromResult(const AerotelicLicence::FCheckResult& Result);

	// Label/value rows in display order; optional rows only when non-empty.
	struct FRow { FText Label; FText Value; bool bHighlight = false; };
	TArray<FRow> AsRows() const;
};

// The result of this run's start-up check, for the welcome screen.
void SetCurrentAerotelicLicenseInfo(const FAerotelicLicenseInfo& Info);
const FAerotelicLicenseInfo& CurrentAerotelicLicenseInfo();
