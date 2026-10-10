#include "SeligDatabase.h"
#include "MiniZip.h"

#include "HAL/FileManager.h"
#include "HAL/PlatformMisc.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HttpModule.h"
#include "Interfaces/IHttpRequest.h"
#include "Interfaces/IHttpResponse.h"

#include "aerotelic/airfoil.hpp"
#include "aerotelic/airfoil_dat.hpp"

DEFINE_LOG_CATEGORY_STATIC(LogAerotelicSelig, Log, All);

namespace AerotelicSelig
{
	const TCHAR* ZipUrl = TEXT("https://m-selig.ae.illinois.edu/ads/archives/coord_seligFmt.zip");

	namespace
	{
		TArray<FEntry> GIndex;
		bool GIndexLoaded = false;
		TArray<FCustomAirfoil> GCustom;

		FString LocalAppData()
		{
			FString Dir = FPlatformMisc::GetEnvironmentVariable(TEXT("LOCALAPPDATA"));
			if (Dir.IsEmpty())
			{
				Dir = FPaths::ProjectSavedDir();
			}
			return Dir;
		}

		// (stem, display) from one index line; "stem" alone when there is no tab.
		bool ParseIndexLine(const FString& Line, FEntry& Out)
		{
			FString Stem, Display;
			if (Line.Split(TEXT("\t"), &Stem, &Display))
			{
				Out.Stem = Stem.TrimStartAndEnd();
				Out.Display = Display.TrimStartAndEnd();
				return !Out.Stem.IsEmpty();
			}
			Out.Stem = Line.TrimStartAndEnd();
			Out.Display = Out.Stem;
			return !Out.Stem.IsEmpty();
		}

		std::shared_ptr<aerotelic::Airfoil> ParseDat(const FString& Text, const FString& FallbackName, FString& OutError)
		{
			try
			{
				std::unique_ptr<aerotelic::Airfoil> A = aerotelic::AirfoilFromDatText(TCHAR_TO_UTF8(*Text), TCHAR_TO_UTF8(*FallbackName));
				return std::shared_ptr<aerotelic::Airfoil>(A.release());
			}
			catch (const std::exception& e)
			{
				OutError = UTF8_TO_TCHAR(e.what());
				return nullptr;
			}
		}
	}

	FString CacheDir()
	{
		const FString Base = LocalAppData();
		const FString Current = FPaths::Combine(Base, TEXT("WingDesigner"), TEXT("selig_db"));
		const FString Old = FPaths::Combine(Base, TEXT("Aerotelic"), TEXT("selig_db"));
		if (!FPaths::DirectoryExists(Current) && FPaths::DirectoryExists(Old))
		{
			return Old;
		}
		return Current;
	}

	FString DatDir()
	{
		return FPaths::Combine(CacheDir(), TEXT("coord_seligFmt"));
	}

	FString IndexFile()
	{
		return FPaths::Combine(CacheDir(), TEXT("index.txt"));
	}

	bool IsDownloaded()
	{
		return FPaths::FileExists(IndexFile()) && FPaths::DirectoryExists(DatDir());
	}

	const TArray<FEntry>& GetIndex()
	{
		if (!GIndexLoaded)
		{
			GIndexLoaded = true;
			GIndex.Reset();
			FString Text;
			if (FFileHelper::LoadFileToString(Text, *IndexFile()))
			{
				TArray<FString> Lines;
				Text.ParseIntoArrayLines(Lines, /*bCullEmpty*/ true);
				for (const FString& Line : Lines)
				{
					FEntry E;
					if (ParseIndexLine(Line, E))
					{
						GIndex.Add(E);
					}
				}
			}
		}
		return GIndex;
	}

	void InvalidateIndex()
	{
		GIndexLoaded = false;
		GIndex.Reset();
	}

	TArray<FEntry> Search(const FString& Query, int32 MaxResults)
	{
		TArray<FEntry> Out;
		const FString Q = Query.ToLower();
		for (const FEntry& E : GetIndex())
		{
			if (Q.IsEmpty() || E.Stem.ToLower().Contains(Q) || E.Display.ToLower().Contains(Q))
			{
				Out.Add(E);
				if (Out.Num() >= MaxResults)
				{
					break;
				}
			}
		}
		return Out;
	}

	std::shared_ptr<aerotelic::Airfoil> LoadAirfoil(const FString& Stem, FString& OutError)
	{
		const FString Path = FPaths::Combine(DatDir(), Stem + TEXT(".dat"));
		FString Text;
		if (!FFileHelper::LoadFileToString(Text, *Path))
		{
			OutError = FString::Printf(TEXT("Airfoil file not found: %s"), *Path);
			return nullptr;
		}
		return ParseDat(Text, Stem, OutError);
	}

	std::shared_ptr<aerotelic::Airfoil> LoadDatFile(const FString& InPath, FString& OutError)
	{
		const FString Path = FPaths::ConvertRelativePathToFull(InPath);
		FString Text;
		if (!FFileHelper::LoadFileToString(Text, *Path))
		{
			OutError = FString::Printf(TEXT("Could not read %s"), *Path);
			return nullptr;
		}
		return ParseDat(Text, FPaths::GetBaseFilename(Path), OutError);
	}

	const TArray<FCustomAirfoil>& CustomAirfoils()
	{
		return GCustom;
	}

	void AddCustomAirfoil(const FString& Key, std::shared_ptr<aerotelic::Airfoil> Airfoil)
	{
		for (FCustomAirfoil& C : GCustom)
		{
			if (C.Key == Key)
			{
				C.Airfoil = Airfoil;
				return;
			}
		}
		GCustom.Add({ Key, Airfoil });
	}

	int32 BuildIndex()
	{
		TArray<FString> Files;
		IFileManager::Get().FindFiles(Files, *FPaths::Combine(DatDir(), TEXT("*.dat")), true, false);
		Files.Sort();
		FString Out;
		for (const FString& File : Files)
		{
			const FString Stem = FPaths::GetBaseFilename(File);
			FString Text, Display;
			if (FFileHelper::LoadFileToString(Text, *FPaths::Combine(DatDir(), File)))
			{
				Display = UTF8_TO_TCHAR(aerotelic::FirstDatNameLine(TCHAR_TO_UTF8(*Text)).c_str());
			}
			if (Display.IsEmpty())
			{
				Display = Stem;
			}
			Out += Stem + TEXT("\t") + Display + TEXT("\n");
		}
		FFileHelper::SaveStringToFile(Out, *IndexFile(), FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
		InvalidateIndex();
		return Files.Num();
	}

	// ------------------------------------------------------------ download

	void FDownload::Start(FOnProgress InProgress, FOnFinished InFinished)
	{
		Progress = InProgress;
		Finished = InFinished;
		bRunning = true;
		bCancelled = false;
		Progress.ExecuteIfBound(0, TEXT("Connecting to UIUC server..."));

		Request = FHttpModule::Get().CreateRequest();
		Request->SetURL(ZipUrl);
		Request->SetVerb(TEXT("GET"));
		Request->SetTimeout(300.0f);
		TWeakPtr<FDownload> Weak = AsShared();
		Request->OnRequestProgress64().BindLambda([Weak](FHttpRequestPtr Req, uint64 BytesSent, uint64 BytesReceived)
		{
			TSharedPtr<FDownload> Self = Weak.Pin();
			if (!Self.IsValid() || Self->bCancelled)
			{
				return;
			}
			uint64 Total = 0;
			if (Req.IsValid() && Req->GetResponse().IsValid())
			{
				Total = (uint64)Req->GetResponse()->GetContentLength();
			}
			const int32 Percent = Total > 0 ? FMath::Min<int32>((int32)(BytesReceived * 100 / Total), 99) : 0;
			const FString Status = Total > 0
				? FString::Printf(TEXT("Downloading... %s KB / %s KB"), *FText::AsNumber((int64)(BytesReceived / 1024)).ToString(), *FText::AsNumber((int64)(Total / 1024)).ToString())
				: FString::Printf(TEXT("Downloading... %s KB"), *FText::AsNumber((int64)(BytesReceived / 1024)).ToString());
			Self->Progress.ExecuteIfBound(Percent, Status);
		});
		Request->OnProcessRequestComplete().BindSP(this, &FDownload::OnComplete);
		if (!Request->ProcessRequest())
		{
			bRunning = false;
			Finished.ExecuteIfBound(false, TEXT("Error: the download could not be started."));
		}
	}

	void FDownload::Cancel()
	{
		bCancelled = true;
		if (Request.IsValid())
		{
			Request->CancelRequest();
		}
	}

	void FDownload::OnComplete(TSharedPtr<IHttpRequest, ESPMode::ThreadSafe> Req, TSharedPtr<IHttpResponse, ESPMode::ThreadSafe> Response, bool bOk)
	{
		bRunning = false;
		if (bCancelled)
		{
			Finished.ExecuteIfBound(false, TEXT("Download cancelled."));
			return;
		}
		if (!bOk || !Response.IsValid())
		{
			Finished.ExecuteIfBound(false, TEXT("Error: the UIUC server could not be reached."));
			return;
		}
		if (Response->GetResponseCode() != 200)
		{
			Finished.ExecuteIfBound(false, FString::Printf(TEXT("Error: the server answered HTTP %d."), Response->GetResponseCode()));
			return;
		}
		Progress.ExecuteIfBound(99, TEXT("Extracting archive..."));
		int32 Count = 0;
		FString Error;
		if (!Unpack(Response->GetContent(), Count, Error))
		{
			Finished.ExecuteIfBound(false, FString::Printf(TEXT("Error: %s"), *Error));
			return;
		}
		Progress.ExecuteIfBound(99, TEXT("Building index..."));
		const int32 Indexed = BuildIndex();
		Progress.ExecuteIfBound(100, TEXT("Done"));
		Finished.ExecuteIfBound(true, FString::Printf(TEXT("Done -- %s airfoils indexed."), *FText::AsNumber(Indexed).ToString()));
	}

	bool FDownload::Unpack(const TArray<uint8>& Zip, int32& OutCount, FString& OutError)
	{
		TArray<FMiniZipEntry> Entries;
		if (!MiniZipList(Zip, Entries, OutError))
		{
			return false;
		}
		IFileManager::Get().MakeDirectory(*DatDir(), true);
		OutCount = 0;
		for (const FMiniZipEntry& E : Entries)
		{
			if (!E.Name.EndsWith(TEXT(".dat")))
			{
				continue;
			}
			TArray<uint8> Data;
			if (!MiniZipExtract(Zip, E, Data, OutError))
			{
				return false;
			}
			// Entries are "coord_seligFmt/<name>.dat"; keep just the file
			// name under our own coord_seligFmt directory.
			const FString Target = FPaths::Combine(DatDir(), FPaths::GetCleanFilename(E.Name));
			if (!FFileHelper::SaveArrayToFile(Data, *Target))
			{
				OutError = FString::Printf(TEXT("could not write %s"), *Target);
				return false;
			}
			++OutCount;
		}
		UE_LOG(LogAerotelicSelig, Log, TEXT("Selig database: unpacked %d .dat files to %s"), OutCount, *DatDir());
		return true;
	}
}
