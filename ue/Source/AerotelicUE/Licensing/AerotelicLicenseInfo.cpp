#include "AerotelicLicenseInfo.h"

#define LOCTEXT_NAMESPACE "AerotelicLicenseInfo"

namespace
{
	FAerotelicLicenseInfo GCurrent = FAerotelicLicenseInfo::Development();
}

FAerotelicLicenseInfo FAerotelicLicenseInfo::Development()
{
	FAerotelicLicenseInfo I;
	I.Licensee = TEXT("Development build");
	I.Status = TEXT("Licence not checked — running from source");
	return I;
}

FAerotelicLicenseInfo FAerotelicLicenseInfo::Unlicensed()
{
	FAerotelicLicenseInfo I;
	I.Licensee = TEXT("Unlicensed");
	I.Status = TEXT("No valid licence");
	return I;
}

FAerotelicLicenseInfo FAerotelicLicenseInfo::FromResult(const AerotelicLicence::FCheckResult& R)
{
	using namespace AerotelicLicence;
	if (!R.Licence.IsSet())
	{
		FAerotelicLicenseInfo I = Unlicensed();
		I.Status = R.Message;
		return I;
	}
	const FLicence& L = R.Licence.GetValue();
	FAerotelicLicenseInfo I;
	I.Licensee = L.Licensee;
	I.Organisation = L.Organisation;
	I.Edition = L.Edition;
	I.Serial = L.Serial;
	I.Seats = L.Seats.IsSet() ? FString::FromInt(L.Seats.GetValue()) : FString();
	I.Status = R.Message;
	if (!R.bOnline && R.LastVerified.IsSet())
	{
		I.Status += FString::Printf(TEXT("  Offline — last verified %s; connect within %d day(s)"),
			*FormatDate(R.LastVerified.GetValue()), R.OfflineDaysLeft().Get(0));
	}
	I.bChecked = true;
	I.Expires = L.Expires;
	if (!L.Expires.IsSet())
	{
		I.TimeRemaining = TEXT("Perpetual — does not expire");
	}
	else
	{
		// Measured from the trusted (server) date, never the PC clock alone.
		const int32 Days = (int32)(L.Expires.GetValue() - R.TrustedNow.GetDate()).GetTotalDays();
		if (Days < 0) { I.TimeRemaining = FString::Printf(TEXT("Expired %d day(s) ago"), -Days); I.Severity = ESeverity::Expired; }
		else if (Days == 0) { I.TimeRemaining = TEXT("Expires today"); I.Severity = ESeverity::Warn; }
		else if (Days >= 60) { I.TimeRemaining = FString::Printf(TEXT("%d days remaining  (~%d months)"), Days, FMath::RoundToInt(Days / 30.44)); }
		else { I.TimeRemaining = FString::Printf(TEXT("%d day%s remaining"), Days, Days == 1 ? TEXT("") : TEXT("s")); if (Days <= 30) I.Severity = ESeverity::Warn; }
	}
	return I;
}

TArray<FAerotelicLicenseInfo::FRow> FAerotelicLicenseInfo::AsRows() const
{
	TArray<FRow> Rows;
	auto Add = [&Rows](const FText& Label, const FString& Value, bool bHighlight = false)
	{
		if (!Value.IsEmpty()) Rows.Add({ Label, FText::FromString(Value), bHighlight });
	};
	Add(LOCTEXT("LicensedTo", "Licensed to"), Licensee.IsEmpty() ? FString(TEXT("—")) : Licensee);
	Add(LOCTEXT("Organisation", "Organisation"), Organisation);
	Add(LOCTEXT("Edition", "Edition"), Edition);
	Add(LOCTEXT("Serial", "Serial"), Serial);
	Add(LOCTEXT("Seats", "Seats"), Seats);
	Add(LOCTEXT("Status", "Status"), Status);
	if (bChecked)
	{
		Add(LOCTEXT("Expires", "Expires"), Expires.IsSet() ? AerotelicLicence::FormatDate(Expires.GetValue()) : FString(TEXT("—")));
		Add(LOCTEXT("TimeRemaining", "Time remaining"), TimeRemaining, true);
	}
	return Rows;
}

void SetCurrentAerotelicLicenseInfo(const FAerotelicLicenseInfo& Info) { GCurrent = Info; }
const FAerotelicLicenseInfo& CurrentAerotelicLicenseInfo() { return GCurrent; }

#undef LOCTEXT_NAMESPACE
