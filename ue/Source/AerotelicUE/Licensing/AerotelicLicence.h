#pragma once

#include "CoreMinimal.h"
#include "Misc/DateTime.h"

class FJsonObject;

// Licence verification -- the C++ twin of WingDesigner's core/licence.py and
// core/licence_config.py, accepting the very same licence files.
//
// Three artefacts: a 25-character licence KEY the user types (exchanged once
// with the server, never stored); a licence FILE (licence.lic), an
// Ed25519-signed JSON envelope bound to a machine id; and the last signed
// CHECK REPLY (licence-check.json), the trusted time source and the basis of
// the offline grace. Envelope: {"v":1,"payload":"<base64url JSON>",
// "sig":"<base64url Ed25519>"}; the signature covers the decoded payload
// bytes, never a re-serialised JSON.
//
// Nothing here raises; every outcome is a FCheckResult with a reason code
// and a message for the user. Enforced() decides whether the start-up gate
// runs at all.
namespace AerotelicLicence
{
	// ---- configuration (core/licence_config.py) ----
	const TCHAR* const Product = TEXT("WingDesigner"); // the licence server's product name; licences are shared with the Python app
	const TCHAR* const DefaultServerUrl = TEXT("https://licence.ms2aero.example.com");
	constexpr int32 KeyLength = 25;
	const TCHAR* const LicenceFilename = TEXT("licence.lic");
	const TCHAR* const CheckFilename = TEXT("licence-check.json");
	const TCHAR* const MachineSalt = TEXT("WingDesigner-machine-v1:");
	constexpr double ClockSkewMinutes = 10.0;
	constexpr float HttpTimeoutSeconds = 6.0f;

	// Ed25519 public keys, 64 hex characters each, tried in order (key
	// rotation). EMPTY until a key is generated with the licence server's
	// `admin keygen`; with none, no licence can be accepted and an enforced
	// build cannot start (REQ-LIC-006).
	TArray<FString> PublicKeys();
	FString ServerUrl();   // WINGDESIGNER_LICENCE_SERVER / AEROTELIC_LICENCE_SERVER override
	int32 GraceDays();     // 7

	// A packaged build always enforces; a dev/editor-hosted run only with
	// AEROTELIC_ENFORCE_LICENCE=1 (or WINGDESIGNER_ENFORCE_LICENCE=1).
	bool Enforced();

	// ---- outcomes ----
	enum class EReason : uint8
	{
		Ok, Missing, Invalid, WrongMachine, Expired, Revoked, Deactivated,
		NeverVerified, OfflineTooLong, ClockRollback, Misconfigured,
	};
	const TCHAR* ReasonCode(EReason Reason);

	struct FLicence
	{
		FString Serial, Licensee, Organisation, Edition, Machine;
		TOptional<int32> Seats;
		FDateTime Issued;
		TOptional<FDateTime> Expires; // date only; unset = perpetual
		FString Text;                 // the envelope as read, for re-saving
	};

	struct FCheckResult
	{
		EReason Reason = EReason::Missing;
		FString Message;
		TOptional<FLicence> Licence;
		FDateTime TrustedNow;
		bool bOnline = false;
		TOptional<FDateTime> LastVerified;
		int32 Grace = 7;

		bool Ok() const { return Reason == EReason::Ok; }
		// Days of grace left when offline; unset when online.
		TOptional<int32> OfflineDaysLeft() const;
	};

	// Everything the check reads from the outside, overridable by tests.
	struct FCheckOptions
	{
		TOptional<TArray<FString>> Keys;          // else PublicKeys()
		TOptional<TArray<FString>> Candidates;    // else LicenceCandidates()
		TOptional<FString> CheckPath;             // else the per-user check file
		TOptional<FString> Machine;               // else MachineId()
		TOptional<FDateTime> Now;                 // else UtcNow
		bool bNetwork = true;                     // false: behave as if the server were unreachable
		int32 GraceDays = 7;
	};

	// ---- machine binding ----
	// sha256("WingDesigner-machine-v1:" + lower(trim(guid))), lowercase hex.
	FString MachineIdFromGuid(const FString& Guid);
	// Windows: HKLM\SOFTWARE\Microsoft\Cryptography\MachineGuid, 64-bit view.
	bool MachineId(FString& OutId, FString& OutError);

	// ---- files ----
	FString UserDataDir();                // %LOCALAPPDATA%\MS2 Aerodynamics\WingDesigner
	TArray<FString> LicenceCandidates();  // per-user first, then the install directory
	FString CheckPath();

	// ---- envelopes ----
	bool Base64UrlDecode(const FString& Text, TArray<uint8>& Out);
	FString Base64UrlEncode(const TArray<uint8>& Bytes);
	bool Sha256Hex(const FString& Ascii, FString& OutHex);
	bool Ed25519Verify(const TArray<uint8>& PublicKey32, const TArray<uint8>& Message, const TArray<uint8>& Signature64);
	// Verifies the signature against any of Keys and the type/product
	// discriminators; the payload is returned parsed.
	bool VerifyEnvelope(const FString& Text, const FString& ExpectedType, const TArray<FString>& Keys,
		TSharedPtr<FJsonObject>& OutPayload, EReason& OutReason, FString& OutError);
	bool ParseLicence(const FString& Text, const TArray<FString>& Keys, FLicence& Out, EReason& OutReason, FString& OutError);
	// Perpetual beats any date, then the later expiry, then the newer issue.
	const FLicence* ChooseBest(const TArray<FLicence>& Found);

	// ---- the start-up decision (core/licence.py:check_licence) ----
	FCheckResult CheckLicence(const FCheckOptions& Options = FCheckOptions());

	// ---- activation ----
	bool LooksLikeKey(const FString& Key);
	// POST /v1/activate; the reply must verify as a licence for this machine.
	bool Activate(const FString& Key, FString& OutLicenceText, FString& OutError);
	bool InstallLicence(const FString& Text, FString& OutError);

	FString FormatDate(const FDateTime& When); // "%d %B %Y"

	// ---- test support (the server's half, in process) ----
	bool Ed25519GenerateKeyPair(TArray<uint8>& OutPublic32, TArray<uint8>& OutPrivate32);
	bool Ed25519Sign(const TArray<uint8>& Private32, const TArray<uint8>& Message, TArray<uint8>& OutSignature64);
	FString MakeEnvelope(const FString& PayloadJson, const TArray<uint8>& Private32);
	FString HexOf(const TArray<uint8>& Bytes);
}
