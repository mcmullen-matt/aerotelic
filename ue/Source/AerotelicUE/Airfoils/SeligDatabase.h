#pragma once

#include "CoreMinimal.h"
#include <memory>

namespace aerotelic { class Airfoil; }
class IHttpRequest;
class IHttpResponse;

// Port of core/selig_db.py: download, cache, index and lazily load the
// UIUC / Selig airfoil coordinate database (~1,650 airfoils, one zip at
// m-selig.ae.illinois.edu). Nothing here is persisted in a project: a
// chosen airfoil is saved as its fitted curve, so projects stay
// self-contained and need no database.
//
// Cache: %LOCALAPPDATA%\WingDesigner\selig_db (shared with the Qt app, so
// one download serves both) when it exists, else the older
// %LOCALAPPDATA%\Aerotelic\selig_db if that exists, else the WingDesigner
// location for a fresh download. Inside: coord_seligFmt\*.dat and
// index.txt ("stem<TAB>Display Name" per line).
namespace AerotelicSelig
{
	struct FEntry
	{
		FString Stem;
		FString Display;
	};

	extern const TCHAR* ZipUrl;

	FString CacheDir();
	FString DatDir();
	FString IndexFile();

	// True once the database has been downloaded and indexed.
	bool IsDownloaded();

	// The full index (stem, display), cached after the first read; empty
	// until downloaded.
	const TArray<FEntry>& GetIndex();
	// Forgets the cached index (after a download).
	void InvalidateIndex();

	// Case-insensitive substring search over stem and display name, up to
	// MaxResults hits (search_index).
	TArray<FEntry> Search(const FString& Query, int32 MaxResults = 200);

	// Parses coord_seligFmt\<stem>.dat; null with a message on failure.
	std::shared_ptr<aerotelic::Airfoil> LoadAirfoil(const FString& Stem, FString& OutError);

	// Parses any Selig-format file on disk (the "Load .dat..." buttons;
	// Airfoil.from_dat), taking its name from the file or else its stem.
	std::shared_ptr<aerotelic::Airfoil> LoadDatFile(const FString& Path, FString& OutError);

	// The session's custom (.dat) library (airfoil_panel.py _local_library):
	// what "Load .dat..." has loaded, keyed "custom_<stem>", shared by the
	// Airfoils panel and every section picker. Loading a key again replaces it.
	struct FCustomAirfoil
	{
		FString Key;
		std::shared_ptr<aerotelic::Airfoil> Airfoil;
	};
	const TArray<FCustomAirfoil>& CustomAirfoils();
	void AddCustomAirfoil(const FString& Key, std::shared_ptr<aerotelic::Airfoil> Airfoil);

	// The whole download: fetch the zip (HTTP, progress 0..99), unpack the
	// .dat entries, build the index (DownloadWorker.run). Finished reports
	// success and a message ("Done -- 1,650 airfoils indexed." / the error).
	DECLARE_DELEGATE_TwoParams(FOnProgress, int32 /*Percent*/, const FString& /*Status*/);
	DECLARE_DELEGATE_TwoParams(FOnFinished, bool /*bSuccess*/, const FString& /*Message*/);

	class FDownload : public TSharedFromThis<FDownload>
	{
	public:
		void Start(FOnProgress InProgress, FOnFinished InFinished);
		void Cancel();
		bool IsRunning() const { return bRunning; }

	private:
		FOnProgress Progress;
		FOnFinished Finished;
		TSharedPtr<IHttpRequest, ESPMode::ThreadSafe> Request;
		bool bRunning = false;
		bool bCancelled = false;

		void OnComplete(TSharedPtr<IHttpRequest, ESPMode::ThreadSafe> Req, TSharedPtr<IHttpResponse, ESPMode::ThreadSafe> Response, bool bOk);
		bool Unpack(const TArray<uint8>& Zip, int32& OutCount, FString& OutError);
	};

	// Rebuilds index.txt from the .dat files on disk (_build_index); returns
	// the number of entries.
	int32 BuildIndex();
}
