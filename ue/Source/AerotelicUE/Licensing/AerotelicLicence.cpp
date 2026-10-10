#include "AerotelicLicence.h"

#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "HttpModule.h"
#include "HttpManager.h"
#include "Interfaces/IHttpRequest.h"
#include "Interfaces/IHttpResponse.h"
#include "Misc/Base64.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/Guid.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformMisc.h"
#include "HAL/FileManager.h"

#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#include <winreg.h>
#include "Windows/HideWindowsPlatformTypes.h"
#endif

// OpenSSL's `UI` typedef collides with an engine namespace; the engine's
// own SSL module masks it the same way.
#define UI UI_ST
THIRD_PARTY_INCLUDES_START
#include <openssl/evp.h>
#include <openssl/sha.h>
THIRD_PARTY_INCLUDES_END
#undef UI

namespace AerotelicLicence
{
	// ---------------------------------------------------------- config

	TArray<FString> PublicKeys()
	{
		// Paste the hex the licence server's `admin keygen` prints here.
		return {};
	}

	FString ServerUrl()
	{
		for (const TCHAR* Var : { TEXT("AEROTELIC_LICENCE_SERVER"), TEXT("WINGDESIGNER_LICENCE_SERVER") })
		{
			const FString V = FPlatformMisc::GetEnvironmentVariable(Var);
			if (!V.IsEmpty()) return V;
		}
		return DefaultServerUrl;
	}

	int32 GraceDays() { return 7; }

	bool Enforced()
	{
		for (const TCHAR* Var : { TEXT("AEROTELIC_ENFORCE_LICENCE"), TEXT("WINGDESIGNER_ENFORCE_LICENCE") })
		{
			if (FPlatformMisc::GetEnvironmentVariable(Var) == TEXT("1")) return true;
		}
#if WITH_EDITOR
		return false; // a dev run from the Editor binaries, like Python from source
#else
		return !IsRunningCommandlet();
#endif
	}

	const TCHAR* ReasonCode(EReason Reason)
	{
		switch (Reason)
		{
		case EReason::Ok: return TEXT("ok");
		case EReason::Missing: return TEXT("missing");
		case EReason::Invalid: return TEXT("invalid");
		case EReason::WrongMachine: return TEXT("wrong_machine");
		case EReason::Expired: return TEXT("expired");
		case EReason::Revoked: return TEXT("revoked");
		case EReason::Deactivated: return TEXT("deactivated");
		case EReason::NeverVerified: return TEXT("never_verified");
		case EReason::OfflineTooLong: return TEXT("offline_too_long");
		case EReason::ClockRollback: return TEXT("clock_rollback");
		case EReason::Misconfigured: return TEXT("misconfigured");
		}
		return TEXT("?");
	}

	TOptional<int32> FCheckResult::OfflineDaysLeft() const
	{
		if (bOnline || !LastVerified.IsSet()) return TOptional<int32>();
		const FTimespan Left = FTimespan::FromDays(Grace) - (TrustedNow - LastVerified.GetValue());
		return FMath::Max(0, (int32)Left.GetTotalDays());
	}

	FString FormatDate(const FDateTime& When)
	{
		return When.ToFormattedString(TEXT("%d %B %Y"));
	}

	// ------------------------------------------------------ primitives

	FString HexOf(const TArray<uint8>& Bytes)
	{
		return BytesToHex(Bytes.GetData(), Bytes.Num()).ToLower();
	}

	bool Base64UrlDecode(const FString& Text, TArray<uint8>& Out)
	{
		FString Std = Text.Replace(TEXT("-"), TEXT("+")).Replace(TEXT("_"), TEXT("/"));
		while (Std.Len() % 4 != 0) Std += TEXT("=");
		return FBase64::Decode(Std, Out);
	}

	FString Base64UrlEncode(const TArray<uint8>& Bytes)
	{
		FString Std = FBase64::Encode(Bytes);
		Std = Std.Replace(TEXT("+"), TEXT("-")).Replace(TEXT("/"), TEXT("_"));
		while (Std.EndsWith(TEXT("="))) Std.LeftChopInline(1);
		return Std;
	}

	bool Sha256Hex(const FString& Ascii, FString& OutHex)
	{
		const FTCHARToUTF8 Utf8(*Ascii);
		uint8 Digest[SHA256_DIGEST_LENGTH];
		SHA256(reinterpret_cast<const unsigned char*>(Utf8.Get()), Utf8.Length(), Digest);
		OutHex = BytesToHex(Digest, SHA256_DIGEST_LENGTH).ToLower();
		return true;
	}

	bool Ed25519Verify(const TArray<uint8>& PublicKey32, const TArray<uint8>& Message, const TArray<uint8>& Signature64)
	{
		if (PublicKey32.Num() != 32 || Signature64.Num() != 64) return false;
		EVP_PKEY* Key = EVP_PKEY_new_raw_public_key(EVP_PKEY_ED25519, nullptr, PublicKey32.GetData(), 32);
		if (Key == nullptr) return false;
		EVP_MD_CTX* Ctx = EVP_MD_CTX_new();
		bool bOk = false;
		if (Ctx != nullptr && EVP_DigestVerifyInit(Ctx, nullptr, nullptr, nullptr, Key) == 1)
		{
			bOk = EVP_DigestVerify(Ctx, Signature64.GetData(), 64, Message.GetData(), Message.Num()) == 1;
		}
		if (Ctx != nullptr) EVP_MD_CTX_free(Ctx);
		EVP_PKEY_free(Key);
		return bOk;
	}

	bool Ed25519GenerateKeyPair(TArray<uint8>& OutPublic32, TArray<uint8>& OutPrivate32)
	{
		EVP_PKEY_CTX* Ctx = EVP_PKEY_CTX_new_id(EVP_PKEY_ED25519, nullptr);
		if (Ctx == nullptr) return false;
		EVP_PKEY* Key = nullptr;
		bool bOk = EVP_PKEY_keygen_init(Ctx) == 1 && EVP_PKEY_keygen(Ctx, &Key) == 1 && Key != nullptr;
		if (bOk)
		{
			size_t Len = 32;
			OutPublic32.SetNumUninitialized(32);
			OutPrivate32.SetNumUninitialized(32);
			bOk = EVP_PKEY_get_raw_public_key(Key, OutPublic32.GetData(), &Len) == 1 && Len == 32;
			Len = 32;
			bOk = bOk && EVP_PKEY_get_raw_private_key(Key, OutPrivate32.GetData(), &Len) == 1 && Len == 32;
		}
		if (Key != nullptr) EVP_PKEY_free(Key);
		EVP_PKEY_CTX_free(Ctx);
		return bOk;
	}

	bool Ed25519Sign(const TArray<uint8>& Private32, const TArray<uint8>& Message, TArray<uint8>& OutSignature64)
	{
		if (Private32.Num() != 32) return false;
		EVP_PKEY* Key = EVP_PKEY_new_raw_private_key(EVP_PKEY_ED25519, nullptr, Private32.GetData(), 32);
		if (Key == nullptr) return false;
		EVP_MD_CTX* Ctx = EVP_MD_CTX_new();
		bool bOk = false;
		if (Ctx != nullptr && EVP_DigestSignInit(Ctx, nullptr, nullptr, nullptr, Key) == 1)
		{
			size_t Len = 64;
			OutSignature64.SetNumUninitialized(64);
			bOk = EVP_DigestSign(Ctx, OutSignature64.GetData(), &Len, Message.GetData(), Message.Num()) == 1 && Len == 64;
		}
		if (Ctx != nullptr) EVP_MD_CTX_free(Ctx);
		EVP_PKEY_free(Key);
		return bOk;
	}

	FString MakeEnvelope(const FString& PayloadJson, const TArray<uint8>& Private32)
	{
		const FTCHARToUTF8 Utf8(*PayloadJson);
		TArray<uint8> Payload((const uint8*)Utf8.Get(), Utf8.Length());
		TArray<uint8> Sig;
		if (!Ed25519Sign(Private32, Payload, Sig)) return FString();
		// The server's json.dumps(separators=(",", ":")).
		return FString::Printf(TEXT("{\"v\":1,\"payload\":\"%s\",\"sig\":\"%s\"}"), *Base64UrlEncode(Payload), *Base64UrlEncode(Sig));
	}

	// -------------------------------------------------------- machine

	FString MachineIdFromGuid(const FString& Guid)
	{
		FString Hex;
		Sha256Hex(FString(MachineSalt) + Guid.TrimStartAndEnd().ToLower(), Hex);
		return Hex;
	}

	bool MachineId(FString& OutId, FString& OutError)
	{
#if PLATFORM_WINDOWS
		HKEY Key = nullptr;
		// The explicit 64-bit view, as the Python app and the installer read
		// it: WOW6432Node carries a different MachineGuid.
		if (RegOpenKeyExW(HKEY_LOCAL_MACHINE, L"SOFTWARE\\Microsoft\\Cryptography", 0, KEY_READ | KEY_WOW64_64KEY, &Key) != ERROR_SUCCESS)
		{
			OutError = TEXT("the Cryptography registry key could not be opened");
			return false;
		}
		wchar_t Buffer[256] = {};
		DWORD Size = sizeof(Buffer) - sizeof(wchar_t);
		DWORD Type = 0;
		const LSTATUS Status = RegQueryValueExW(Key, L"MachineGuid", nullptr, &Type, reinterpret_cast<LPBYTE>(Buffer), &Size);
		RegCloseKey(Key);
		if (Status != ERROR_SUCCESS || Type != REG_SZ)
		{
			OutError = TEXT("MachineGuid is not present");
			return false;
		}
		OutId = MachineIdFromGuid(FString(Buffer));
		return true;
#else
		FString Text;
		for (const TCHAR* Path : { TEXT("/etc/machine-id"), TEXT("/var/lib/dbus/machine-id") })
		{
			if (FFileHelper::LoadFileToString(Text, Path) && !Text.TrimStartAndEnd().IsEmpty())
			{
				OutId = MachineIdFromGuid(Text);
				return true;
			}
		}
		OutError = TEXT("no machine id file");
		return false;
#endif
	}

	// ---------------------------------------------------------- files

	FString UserDataDir()
	{
		FString Base = FPlatformMisc::GetEnvironmentVariable(TEXT("LOCALAPPDATA"));
		if (Base.IsEmpty())
		{
			Base = FPlatformProcess::UserSettingsDir();
		}
		return FPaths::Combine(Base, TEXT("MS2 Aerodynamics"), TEXT("WingDesigner"));
	}

	TArray<FString> LicenceCandidates()
	{
		return {
			FPaths::Combine(UserDataDir(), LicenceFilename),
			FPaths::Combine(FPaths::ProjectDir(), LicenceFilename),
			FPaths::Combine(FPlatformProcess::BaseDir(), LicenceFilename),
		};
	}

	FString CheckPath()
	{
		return FPaths::Combine(UserDataDir(), CheckFilename);
	}

	// ------------------------------------------------------ envelopes

	namespace
	{
		bool ParseIsoUtc(const FString& Text, FDateTime& Out)
		{
			// "%Y-%m-%dT%H:%M:%SZ"
			return FDateTime::ParseIso8601(*Text, Out);
		}

		bool ParseDate(const FString& Text, FDateTime& Out)
		{
			// "YYYY-MM-DD"
			TArray<FString> Parts;
			Text.ParseIntoArray(Parts, TEXT("-"));
			if (Parts.Num() != 3) return false;
			const int32 Y = FCString::Atoi(*Parts[0]), M = FCString::Atoi(*Parts[1]), D = FCString::Atoi(*Parts[2]);
			if (!FDateTime::Validate(Y, M, D, 0, 0, 0, 0)) return false;
			Out = FDateTime(Y, M, D);
			return true;
		}

		FString GetStr(const TSharedPtr<FJsonObject>& O, const TCHAR* Field)
		{
			FString V;
			if (O->TryGetStringField(FString(Field), V)) return V;
			return FString();
		}

		// A blocking JSON POST, pumping the HTTP manager until the reply or
		// the timeout: the gate runs before anything else is up.
		bool PostJson(const FString& Url, const FString& Body, int32& OutCode, FString& OutBody, FString& OutError)
		{
			TSharedRef<IHttpRequest, ESPMode::ThreadSafe> Request = FHttpModule::Get().CreateRequest();
			Request->SetURL(Url);
			Request->SetVerb(TEXT("POST"));
			Request->SetHeader(TEXT("Content-Type"), TEXT("application/json"));
			Request->SetHeader(TEXT("Accept"), TEXT("application/json, text/plain"));
			Request->SetContentAsString(Body);
			Request->SetTimeout(HttpTimeoutSeconds);
			bool bDone = false, bSucceeded = false;
			Request->OnProcessRequestComplete().BindLambda([&](FHttpRequestPtr, FHttpResponsePtr Response, bool bOk)
			{
				bDone = true;
				if (bOk && Response.IsValid())
				{
					bSucceeded = true;
					OutCode = Response->GetResponseCode();
					OutBody = Response->GetContentAsString();
				}
			});
			if (!Request->ProcessRequest())
			{
				OutError = TEXT("the request could not be started");
				return false;
			}
			const double Deadline = FPlatformTime::Seconds() + HttpTimeoutSeconds + 1.0;
			while (!bDone && FPlatformTime::Seconds() < Deadline)
			{
				FHttpModule::Get().GetHttpManager().Tick(0.05f);
				FPlatformProcess::Sleep(0.02f);
			}
			if (!bDone)
			{
				Request->CancelRequest();
				OutError = TEXT("the licence server did not answer in time");
				return false;
			}
			if (!bSucceeded)
			{
				OutError = TEXT("the licence server could not be reached");
				return false;
			}
			return true;
		}
	}

	bool VerifyEnvelope(const FString& Text, const FString& ExpectedType, const TArray<FString>& Keys,
		TSharedPtr<FJsonObject>& OutPayload, EReason& OutReason, FString& OutError)
	{
		OutReason = EReason::Invalid;
		TSharedPtr<FJsonObject> Outer;
		if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Outer) || !Outer.IsValid())
		{
			OutError = TEXT("Not a licence file (the envelope is not JSON).");
			return false;
		}
		double V = 0;
		if (!Outer->TryGetNumberField(TEXT("v"), V) || V != 1.0)
		{
			OutError = FString::Printf(TEXT("The licence file is corrupt (unsupported envelope version %g)."), V);
			return false;
		}
		TArray<uint8> Payload, Sig;
		if (!Base64UrlDecode(GetStr(Outer, TEXT("payload")), Payload) || !Base64UrlDecode(GetStr(Outer, TEXT("sig")), Sig))
		{
			OutError = TEXT("The licence file is corrupt.");
			return false;
		}
		bool bVerified = false;
		for (const FString& Hex : Keys)
		{
			TArray<uint8> Key;
			Key.SetNumUninitialized(32);
			if (Hex.Len() == 64 && HexToBytes(Hex, Key.GetData()) == 32 && Ed25519Verify(Key, Payload, Sig))
			{
				bVerified = true;
				break;
			}
		}
		if (!bVerified)
		{
			OutError = TEXT("The licence file's signature does not verify. It has been altered, or was not issued by MS2 Aerodynamics.");
			return false;
		}
		const FString PayloadText = FString(FUTF8ToTCHAR((const ANSICHAR*)Payload.GetData(), Payload.Num()));
		if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(PayloadText), OutPayload) || !OutPayload.IsValid())
		{
			OutError = TEXT("The licence file is corrupt.");
			return false;
		}
		// The type discriminator is load-bearing: a check reply is signed by
		// the same key and must not pass as a licence.
		if (GetStr(OutPayload, TEXT("type")) != ExpectedType || GetStr(OutPayload, TEXT("product")) != Product)
		{
			OutError = TEXT("The file is not a WingDesigner licence.");
			return false;
		}
		OutReason = EReason::Ok;
		return true;
	}

	bool ParseLicence(const FString& Text, const TArray<FString>& Keys, FLicence& Out, EReason& OutReason, FString& OutError)
	{
		TSharedPtr<FJsonObject> P;
		if (!VerifyEnvelope(Text, TEXT("licence"), Keys, P, OutReason, OutError)) return false;
		Out.Text = Text;
		Out.Serial = GetStr(P, TEXT("serial"));
		Out.Licensee = GetStr(P, TEXT("licensee"));
		Out.Organisation = GetStr(P, TEXT("organisation"));
		Out.Edition = GetStr(P, TEXT("edition"));
		Out.Machine = GetStr(P, TEXT("machine"));
		double Seats = 0;
		if (P->TryGetNumberField(TEXT("seats"), Seats)) Out.Seats = (int32)Seats;
		if (Out.Serial.IsEmpty() || Out.Machine.IsEmpty() || !ParseIsoUtc(GetStr(P, TEXT("issued")), Out.Issued))
		{
			OutReason = EReason::Invalid;
			OutError = TEXT("The licence file is incomplete (serial, machine or issued).");
			return false;
		}
		const FString Expires = GetStr(P, TEXT("expires"));
		if (!Expires.IsEmpty())
		{
			FDateTime D;
			if (!ParseDate(Expires, D))
			{
				OutReason = EReason::Invalid;
				OutError = TEXT("The licence file is incomplete (expires).");
				return false;
			}
			Out.Expires = D;
		}
		OutReason = EReason::Ok;
		return true;
	}

	const FLicence* ChooseBest(const TArray<FLicence>& Found)
	{
		const FLicence* Best = nullptr;
		for (const FLicence& L : Found)
		{
			if (Best == nullptr) { Best = &L; continue; }
			const bool bPerp = !L.Expires.IsSet(), bBestPerp = !Best->Expires.IsSet();
			if (bPerp != bBestPerp) { if (bPerp) Best = &L; continue; }
			if (!bPerp && L.Expires.GetValue() != Best->Expires.GetValue()) { if (L.Expires.GetValue() > Best->Expires.GetValue()) Best = &L; continue; }
			if (L.Issued > Best->Issued) Best = &L;
		}
		return Best;
	}

	// ------------------------------------------------------- the check

	namespace
	{
		struct FCheckReply { FDateTime Time; FString Status; FString Text; };

		bool ParseCheckReply(const FString& Text, const TArray<FString>& Keys, const FString& Serial, const FString& Machine,
			const FString& Nonce, FCheckReply& Out, FString& OutError)
		{
			TSharedPtr<FJsonObject> P;
			EReason Reason;
			if (!VerifyEnvelope(Text, TEXT("check"), Keys, P, Reason, OutError)) return false;
			if (GetStr(P, TEXT("serial")) != Serial || GetStr(P, TEXT("machine")) != Machine || (!Nonce.IsEmpty() && GetStr(P, TEXT("nonce")) != Nonce))
			{
				OutError = TEXT("the server's reply was for a different licence");
				return false;
			}
			if (!ParseIsoUtc(GetStr(P, TEXT("time")), Out.Time))
			{
				OutError = TEXT("the server's reply carries no time");
				return false;
			}
			Out.Status = GetStr(P, TEXT("status"));
			Out.Text = Text;
			return true;
		}

		bool WriteAtomically(const FString& Path, const FString& Text)
		{
			IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), true);
			const FString Tmp = Path + TEXT(".tmp");
			if (!FFileHelper::SaveStringToFile(Text, *Tmp, FFileHelper::EEncodingOptions::ForceAnsi)) return false;
			return IFileManager::Get().Move(*Path, *Tmp, true, true);
		}
	}

	FCheckResult CheckLicence(const FCheckOptions& Options)
	{
		FCheckResult R;
		R.Grace = Options.GraceDays;
		const TArray<FString> Keys = Options.Keys.IsSet() ? Options.Keys.GetValue() : PublicKeys();
		const FDateTime Now = Options.Now.IsSet() ? Options.Now.GetValue() : FDateTime::UtcNow();
		R.TrustedNow = Now;

		if (Keys.Num() == 0)
		{
			R.Reason = EReason::Misconfigured;
			R.Message = TEXT("This build of Aerotelic carries no licence verification key, so no licence can be accepted. Contact MS2 Aerodynamics.");
			return R;
		}
		FString Machine, Err;
		if (Options.Machine.IsSet()) Machine = Options.Machine.GetValue();
		else if (!MachineId(Machine, Err))
		{
			R.Reason = EReason::Misconfigured;
			R.Message = FString::Printf(TEXT("This computer could not be identified (%s)."), *Err);
			return R;
		}

		// Every candidate file is read; all the valid ones compete.
		TArray<FLicence> Found;
		TOptional<TPair<EReason, FString>> Problem, WrongMachine;
		for (const FString& Path : Options.Candidates.IsSet() ? Options.Candidates.GetValue() : LicenceCandidates())
		{
			FString Text;
			if (!FPaths::FileExists(Path)) continue;
			if (!FFileHelper::LoadFileToString(Text, *Path))
			{
				if (!Problem.IsSet()) Problem = TPair<EReason, FString>(EReason::Invalid, FString::Printf(TEXT("%s could not be read."), *Path));
				continue;
			}
			FLicence L;
			EReason Reason;
			if (!ParseLicence(Text, Keys, L, Reason, Err))
			{
				if (!Problem.IsSet()) Problem = TPair<EReason, FString>(Reason, Err);
				continue;
			}
			if (L.Machine != Machine)
			{
				WrongMachine = TPair<EReason, FString>(EReason::WrongMachine,
					TEXT("This licence was activated on a different computer. Enter your licence key to activate it here; if it is still active on the other computer, MS2 Aerodynamics can release it."));
				continue;
			}
			Found.Add(L);
		}
		if (Found.Num() == 0)
		{
			if (WrongMachine.IsSet()) { R.Reason = WrongMachine->Key; R.Message = WrongMachine->Value; }
			else if (Problem.IsSet()) { R.Reason = Problem->Key; R.Message = Problem->Value; }
			else { R.Reason = EReason::Missing; R.Message = TEXT("No Aerotelic licence is installed on this computer."); }
			return R;
		}
		const FLicence Best = *ChooseBest(Found);
		R.Licence = Best;

		// Trusted time: the server's, else the kept reply within the grace.
		const FString CheckFile = Options.CheckPath.IsSet() ? Options.CheckPath.GetValue() : CheckPath();
		FCheckReply Reply;
		bool bHaveReply = false;
		FString Unavailable;
		if (Options.bNetwork)
		{
			const FString Nonce = FGuid::NewGuid().ToString(EGuidFormats::Digits).ToLower();
			const FString Body = FString::Printf(TEXT("{\"serial\":\"%s\",\"machine\":\"%s\",\"nonce\":\"%s\",\"product\":\"%s\"}"), *Best.Serial, *Machine, *Nonce, Product);
			int32 Code = 0;
			FString ReplyText;
			if (PostJson(ServerUrl() + TEXT("/v1/check"), Body, Code, ReplyText, Unavailable) && Code == 200)
			{
				if (ParseCheckReply(ReplyText, Keys, Best.Serial, Machine, Nonce, Reply, Err))
				{
					bHaveReply = true;
					R.bOnline = true;
					R.TrustedNow = Reply.Time;
					R.LastVerified = Reply.Time;
					WriteAtomically(CheckFile, ReplyText);
				}
				else
				{
					Unavailable = Err;
				}
			}
			else if (Unavailable.IsEmpty())
			{
				Unavailable = FString::Printf(TEXT("the licence server returned %d"), Code);
			}
		}
		else
		{
			Unavailable = TEXT("offline");
		}
		if (!bHaveReply)
		{
			FString Stored;
			if (!FPaths::FileExists(CheckFile) || !FFileHelper::LoadFileToString(Stored, *CheckFile)
				|| !ParseCheckReply(Stored, Keys, Best.Serial, Machine, FString(), Reply, Err))
			{
				R.Reason = EReason::NeverVerified;
				R.Message = FString::Printf(TEXT("The licence could not be verified (%s), and it has not been verified on this computer before. Connect to the Internet and try again."), *Unavailable);
				return R;
			}
			R.LastVerified = Reply.Time;
			if (Now < Reply.Time - FTimespan::FromMinutes(ClockSkewMinutes))
			{
				R.Reason = EReason::ClockRollback;
				R.Message = FString::Printf(TEXT("This computer's clock is set earlier than the last time the licence was verified (%s). Correct the date and time, or connect to the Internet."), *FormatDate(Reply.Time));
				return R;
			}
			if (Now - Reply.Time > FTimespan::FromDays(Options.GraceDays))
			{
				R.Reason = EReason::OfflineTooLong;
				R.Message = FString::Printf(TEXT("The licence was last verified on %s, more than %d days ago. Connect to the Internet so it can be checked again."), *FormatDate(Reply.Time), Options.GraceDays);
				return R;
			}
			R.TrustedNow = FMath::Max(Now, Reply.Time);
			R.bOnline = false;
		}

		if (Reply.Status == TEXT("revoked"))
		{
			R.Reason = EReason::Revoked;
			R.Message = TEXT("This licence has been revoked. Contact MS2 Aerodynamics.");
			return R;
		}
		if (Reply.Status != TEXT("active"))
		{
			R.Reason = EReason::Deactivated;
			R.Message = TEXT("This licence is no longer active on this computer: it has been released, or moved to another computer. Enter a licence key to activate it here.");
			return R;
		}
		if (Best.Expires.IsSet() && R.TrustedNow.GetDate() > Best.Expires.GetValue())
		{
			R.Reason = EReason::Expired;
			R.Message = FString::Printf(TEXT("This licence expired on %s. Enter a renewal key to continue."), *FormatDate(Best.Expires.GetValue()));
			return R;
		}
		R.Reason = EReason::Ok;
		R.Message = TEXT("Licence valid.");
		return R;
	}

	// ------------------------------------------------------ activation

	bool LooksLikeKey(const FString& Key)
	{
		int32 Count = 0;
		for (TCHAR C : Key) if (FChar::IsAlnum(C)) ++Count;
		return Count == KeyLength;
	}

	bool Activate(const FString& Key, FString& OutLicenceText, FString& OutError)
	{
		FString Machine;
		if (!MachineId(Machine, OutError))
		{
			OutError = FString::Printf(TEXT("This computer could not be identified (%s)."), *OutError);
			return false;
		}
		FString Clean;
		for (TCHAR C : Key) if (FChar::IsAlnum(C)) Clean.AppendChar(FChar::ToUpper(C));
		const FString Body = FString::Printf(TEXT("{\"key\":\"%s\",\"machine\":\"%s\",\"product\":\"%s\",\"version\":\"%s\"}"),
			*Clean, *Machine, Product, FApp::GetBuildVersion());
		int32 Code = 0;
		FString Reply;
		if (!PostJson(ServerUrl() + TEXT("/v1/activate"), Body, Code, Reply, OutError))
		{
			return false;
		}
		if (Code != 200)
		{
			// The server's 4xx bodies are plain text written for the user.
			OutError = Reply.TrimStartAndEnd().IsEmpty() ? FString::Printf(TEXT("The licence server returned %d."), Code) : Reply.TrimStartAndEnd();
			return false;
		}
		FLicence L;
		EReason Reason;
		FString Err;
		if (!ParseLicence(Reply, PublicKeys(), L, Reason, Err))
		{
			OutError = FString::Printf(TEXT("The server's reply was not a valid licence: %s"), *Err);
			return false;
		}
		if (L.Machine != Machine)
		{
			OutError = TEXT("The server's reply was for a different computer.");
			return false;
		}
		OutLicenceText = Reply;
		return true;
	}

	bool InstallLicence(const FString& Text, FString& OutError)
	{
		const FString Path = FPaths::Combine(UserDataDir(), LicenceFilename);
		if (!WriteAtomically(Path, Text))
		{
			OutError = FString::Printf(TEXT("The licence could not be saved: %s"), *Path);
			return false;
		}
		return true;
	}
}
