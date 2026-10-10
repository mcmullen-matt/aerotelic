#include "AerotelicLicenceTestCommandlet.h"
#include "Licensing/AerotelicLicence.h"
#include "Licensing/AerotelicLicenseInfo.h"

#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"

DEFINE_LOG_CATEGORY_STATIC(LogAerotelicLicenceTest, Log, All);

namespace
{
	int32 g_pass = 0, g_fail = 0;

	void Check(bool bCond, const FString& Msg)
	{
		if (bCond) { ++g_pass; UE_LOG(LogAerotelicLicenceTest, Display, TEXT("  ok    %s"), *Msg); }
		else { ++g_fail; UE_LOG(LogAerotelicLicenceTest, Error, TEXT("  FAIL  %s"), *Msg); }
	}

	// The server's json.dumps(sort_keys=True, separators=(",", ":")) for a
	// licence payload.
	FString LicencePayload(const FString& Serial, const FString& Machine, const FString& Issued, const FString& Expires, const FString& Product = TEXT("WingDesigner"))
	{
		return FString::Printf(TEXT("{\"edition\":\"Professional\",\"expires\":%s,\"issued\":\"%s\",\"licensee\":\"Jane Smith\",\"machine\":\"%s\",\"organisation\":\"Acme Aero\",\"product\":\"%s\",\"seats\":1,\"serial\":\"%s\",\"type\":\"licence\"}"),
			Expires.IsEmpty() ? TEXT("null") : *FString::Printf(TEXT("\"%s\""), *Expires), *Issued, *Machine, *Product, *Serial);
	}

	FString CheckPayload(const FString& Serial, const FString& Machine, const FString& Time, const FString& Status)
	{
		return FString::Printf(TEXT("{\"machine\":\"%s\",\"nonce\":\"abc\",\"product\":\"WingDesigner\",\"serial\":\"%s\",\"status\":\"%s\",\"time\":\"%s\",\"type\":\"check\"}"),
			*Machine, *Serial, *Status, *Time);
	}
}

int32 UAerotelicLicenceTestCommandlet::Main(const FString& Params)
{
	using namespace AerotelicLicence;

	// ---- the server's keys, in process ----
	TArray<uint8> Pub, Priv, Pub2, Priv2;
	Check(Ed25519GenerateKeyPair(Pub, Priv) && Ed25519GenerateKeyPair(Pub2, Priv2), "generated two Ed25519 pairs");
	const TArray<FString> Keys = { HexOf(Pub) };
	const TArray<FString> Rotated = { HexOf(Pub2), HexOf(Pub) };

	// ---- machine id ----
	const FString Machine = MachineIdFromGuid(TEXT("  ABCDEF01-2345-6789-ABCD-EF0123456789 "));
	Check(Machine.Len() == 64 && Machine == Machine.ToLower(), "machine id is 64 lowercase hex");
	Check(Machine == MachineIdFromGuid(TEXT("abcdef01-2345-6789-abcd-ef0123456789")), "machine id is case- and whitespace-insensitive");
	{
		FString Hex;
		Sha256Hex(TEXT("abc"), Hex);
		Check(Hex == TEXT("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"), "sha256(\"abc\") matches the known digest");
	}

	// ---- envelopes ----
	const FString Good = MakeEnvelope(LicencePayload(TEXT("AT-7K2QDM9X"), Machine, TEXT("2026-10-01T12:00:00Z"), TEXT("2027-10-01")), Priv);
	Check(!Good.IsEmpty() && Good.StartsWith(TEXT("{\"v\":1,")), "a signed envelope was produced");
	{
		FLicence L; EReason Reason; FString Err;
		Check(ParseLicence(Good, Keys, L, Reason, Err), FString::Printf(TEXT("the envelope verifies and parses (%s)"), *Err));
		Check(L.Serial == TEXT("AT-7K2QDM9X") && L.Licensee == TEXT("Jane Smith") && L.Edition == TEXT("Professional") && L.Seats.Get(0) == 1, "licence fields decoded");
		Check(L.Expires.IsSet() && L.Expires->GetYear() == 2027 && L.Expires->GetMonth() == 10 && L.Expires->GetDay() == 1, "expiry date decoded");
		Check(L.Issued.GetYear() == 2026 && L.Issued.GetHour() == 12, "issued timestamp decoded");
		Check(ParseLicence(Good, Rotated, L, Reason, Err), "key rotation: an older key later in the list still verifies");
		Check(!ParseLicence(Good, { HexOf(Pub2) }, L, Reason, Err) && Reason == EReason::Invalid, "a different key refuses the file");
		Check(!ParseLicence(Good, {}, L, Reason, Err), "no keys: nothing verifies");
	}
	{
		// Tamper: flip one character of the payload.
		FString Tampered = Good;
		const int32 At = Tampered.Find(TEXT("\"payload\":\"")) + 12;
		Tampered[At] = Tampered[At] == 'A' ? 'B' : 'A';
		FLicence L; EReason Reason; FString Err;
		Check(!ParseLicence(Tampered, Keys, L, Reason, Err) && Reason == EReason::Invalid, "an edited payload fails the signature");
	}
	{
		const FString CheckEnv = MakeEnvelope(CheckPayload(TEXT("AT-7K2QDM9X"), Machine, TEXT("2026-10-10T00:00:00Z"), TEXT("active")), Priv);
		FLicence L; EReason Reason; FString Err;
		Check(!ParseLicence(CheckEnv, Keys, L, Reason, Err), "a check reply is not a licence (type discriminator)");
		TSharedPtr<FJsonObject> P;
		Check(VerifyEnvelope(CheckEnv, TEXT("check"), Keys, P, Reason, Err), "the same reply verifies as a check");
		const FString OtherProduct = MakeEnvelope(LicencePayload(TEXT("AT-1"), Machine, TEXT("2026-10-01T12:00:00Z"), TEXT(""), TEXT("OtherApp")), Priv);
		Check(!ParseLicence(OtherProduct, Keys, L, Reason, Err), "another product's licence is refused");
		for (const TCHAR* Junk : { TEXT(""), TEXT("not json"), TEXT("{}"), TEXT("{\"v\":2,\"payload\":\"\",\"sig\":\"\"}"), TEXT("{\"v\":1,\"payload\":\"!!\",\"sig\":\"??\"}") })
		{
			Check(!ParseLicence(Junk, Keys, L, Reason, Err), FString::Printf(TEXT("junk input refused: %s"), Junk));
		}
	}
	{
		TArray<uint8> Bytes = { 0xfb, 0xff, 0x7e, 0x01 };
		TArray<uint8> Back;
		const FString Enc = Base64UrlEncode(Bytes);
		Check(!Enc.Contains(TEXT("+")) && !Enc.Contains(TEXT("/")) && !Enc.Contains(TEXT("=")), "base64url uses - and _ and strips padding");
		Check(Base64UrlDecode(Enc, Back) && Back == Bytes, "base64url round trip re-pads");
	}

	// ---- candidate preference ----
	{
		TArray<FLicence> Found;
		FLicence A, B, C; EReason R; FString E;
		ParseLicence(MakeEnvelope(LicencePayload(TEXT("A"), Machine, TEXT("2026-01-01T00:00:00Z"), TEXT("2027-01-01")), Priv), Keys, A, R, E);
		ParseLicence(MakeEnvelope(LicencePayload(TEXT("B"), Machine, TEXT("2026-02-01T00:00:00Z"), TEXT("2028-01-01")), Priv), Keys, B, R, E);
		ParseLicence(MakeEnvelope(LicencePayload(TEXT("C"), Machine, TEXT("2025-01-01T00:00:00Z"), TEXT("")), Priv), Keys, C, R, E);
		Found = { A, B };
		Check(ChooseBest(Found)->Serial == TEXT("B"), "the later expiry wins");
		Found = { A, B, C };
		Check(ChooseBest(Found)->Serial == TEXT("C"), "a perpetual licence beats any date");
	}

	// ---- the start-up decision, offline, with files on disk ----
	const FString Dir = FPaths::ProjectSavedDir() / TEXT("LicenceTest");
	IFileManager::Get().DeleteDirectory(*Dir, false, true);
	IFileManager::Get().MakeDirectory(*Dir, true);
	const FString LicPath = Dir / TEXT("licence.lic");
	const FString ChkPath = Dir / TEXT("licence-check.json");
	FFileHelper::SaveStringToFile(Good, *LicPath);

	FCheckOptions O;
	O.Keys = Keys;
	O.Candidates = TArray<FString>{ LicPath };
	O.CheckPath = ChkPath;
	O.Machine = Machine;
	O.bNetwork = false;
	O.Now = FDateTime(2026, 10, 12, 9, 0, 0);

	{
		FCheckOptions NoKeys = O; NoKeys.Keys = TArray<FString>{};
		Check(CheckLicence(NoKeys).Reason == EReason::Misconfigured, "no public key -> misconfigured");
		FCheckOptions NoFile = O; NoFile.Candidates = TArray<FString>{ Dir / TEXT("none.lic") };
		Check(CheckLicence(NoFile).Reason == EReason::Missing, "no licence file -> missing");
		FCheckOptions Other = O; Other.Machine = MachineIdFromGuid(TEXT("other"));
		Check(CheckLicence(Other).Reason == EReason::WrongMachine, "a licence for another computer -> wrong_machine");
		Check(CheckLicence(O).Reason == EReason::NeverVerified, "offline with no kept check reply -> never_verified");
	}
	// A kept check reply dated two days ago.
	FFileHelper::SaveStringToFile(MakeEnvelope(CheckPayload(TEXT("AT-7K2QDM9X"), Machine, TEXT("2026-10-10T00:00:00Z"), TEXT("active")), Priv), *ChkPath);
	{
		const FCheckResult R = CheckLicence(O);
		Check(R.Ok() && !R.bOnline, FString::Printf(TEXT("offline within the grace -> ok (%s)"), *R.Message));
		Check(R.OfflineDaysLeft().Get(-1) == 4, FString::Printf(TEXT("4 days of grace left (got %d)"), R.OfflineDaysLeft().Get(-1)));
		const FAerotelicLicenseInfo Info = FAerotelicLicenseInfo::FromResult(R);
		Check(Info.bChecked && Info.TimeRemaining.Contains(TEXT("days remaining")) && Info.Status.Contains(TEXT("Offline")), "welcome-screen rows: expiry countdown and the offline note");
		FCheckOptions Late = O; Late.Now = FDateTime(2026, 10, 20);
		Check(CheckLicence(Late).Reason == EReason::OfflineTooLong, "offline past the grace -> offline_too_long");
		FCheckOptions Early = O; Early.Now = FDateTime(2026, 10, 9, 23, 0, 0);
		Check(CheckLicence(Early).Reason == EReason::ClockRollback, "clock earlier than the kept reply -> clock_rollback");
		FCheckOptions Expired = O; Expired.Now = FDateTime(2027, 10, 5);
		FFileHelper::SaveStringToFile(MakeEnvelope(CheckPayload(TEXT("AT-7K2QDM9X"), Machine, TEXT("2027-10-03T00:00:00Z"), TEXT("active")), Priv), *ChkPath);
		Check(CheckLicence(Expired).Reason == EReason::Expired, "past the expiry by the trusted time -> expired");
	}
	{
		FFileHelper::SaveStringToFile(MakeEnvelope(CheckPayload(TEXT("AT-7K2QDM9X"), Machine, TEXT("2026-10-10T00:00:00Z"), TEXT("revoked")), Priv), *ChkPath);
		Check(CheckLicence(O).Reason == EReason::Revoked, "a revoked status -> revoked");
		FFileHelper::SaveStringToFile(MakeEnvelope(CheckPayload(TEXT("AT-7K2QDM9X"), Machine, TEXT("2026-10-10T00:00:00Z"), TEXT("released")), Priv), *ChkPath);
		Check(CheckLicence(O).Reason == EReason::Deactivated, "a released status -> deactivated");
		// A reply forged with another key, or for another serial, is ignored.
		FFileHelper::SaveStringToFile(MakeEnvelope(CheckPayload(TEXT("AT-7K2QDM9X"), Machine, TEXT("2026-10-10T00:00:00Z"), TEXT("active")), Priv2), *ChkPath);
		Check(CheckLicence(O).Reason == EReason::NeverVerified, "a forged check reply is ignored");
		FFileHelper::SaveStringToFile(MakeEnvelope(CheckPayload(TEXT("AT-OTHER"), Machine, TEXT("2026-10-10T00:00:00Z"), TEXT("active")), Priv), *ChkPath);
		Check(CheckLicence(O).Reason == EReason::NeverVerified, "a check reply for another serial is ignored");
	}
	{
		// A renewal beside the installer's licence wins.
		const FString Renewal = Dir / TEXT("renewal.lic");
		FFileHelper::SaveStringToFile(MakeEnvelope(LicencePayload(TEXT("AT-RENEW"), Machine, TEXT("2026-10-05T00:00:00Z"), TEXT("2029-01-01")), Priv), *Renewal);
		FFileHelper::SaveStringToFile(MakeEnvelope(CheckPayload(TEXT("AT-RENEW"), Machine, TEXT("2026-10-10T00:00:00Z"), TEXT("active")), Priv), *ChkPath);
		FCheckOptions Two = O; Two.Candidates = TArray<FString>{ Renewal, LicPath };
		const FCheckResult R = CheckLicence(Two);
		Check(R.Ok() && R.Licence->Serial == TEXT("AT-RENEW"), "the renewal beside the installer's licence wins");
	}
	Check(LooksLikeKey(TEXT("7K2QD-M9XTA-0RB4C-WZ8EH-5NJPV")) && !LooksLikeKey(TEXT("7K2QD-M9XTA")), "key format: 25 letters and digits");
	Check(ReasonCode(EReason::OfflineTooLong) == FString(TEXT("offline_too_long")), "reason codes match the Python strings");

	IFileManager::Get().DeleteDirectory(*Dir, false, true);
	UE_LOG(LogAerotelicLicenceTest, Display, TEXT("RESULT: %d passed, %d failed"), g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}
