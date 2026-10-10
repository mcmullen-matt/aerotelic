#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include <memory>

namespace aerotelic { class Airfoil; }
class SAerotelicAirfoilPreview;
class STextBlock;
class SEditableTextBox;
template <typename ItemType> class SListView;
class ITableRow;
class STableViewBase;

// Where a listed airfoil comes from (section_panel.py AirfoilPickerDialog's
// sources and airfoil_panel.py's).
enum class EAirfoilSource : uint8
{
	Project,   // already on a wing or blade in the scene ("◆", gold)
	Naca,      // NACA 4-digit, generated
	Selig,     // the UIUC database on disk
	Custom,    // loaded from a .dat file this session ("★", green)
	Download,  // the "Download the Selig database..." row, shown while it is missing
};

struct FAirfoilListItem
{
	EAirfoilSource Source = EAirfoilSource::Selig;
	FString Label;
	// Selig stem, NACA code, custom key; empty for a project airfoil.
	FString Key;
	// Resolved lazily (a database file is parsed only when selected).
	std::shared_ptr<aerotelic::Airfoil> Airfoil;
};

// The airfoil library browser shared by the section picker and the
// Airfoils panel: a search box, a source combo, the result list, the
// preview and the statistics, with "Load .dat..." and the database
// download. Which sources the combo offers is the caller's choice.
class SAerotelicAirfoilBrowser : public SCompoundWidget
{
public:
	DECLARE_DELEGATE_OneParam(FOnAirfoilChosen, std::shared_ptr<aerotelic::Airfoil>);

	SLATE_BEGIN_ARGS(SAerotelicAirfoilBrowser)
		: _Compact(false), _ListHeight(200.0f), _PreviewSize(FVector2D(300, 300)) {}
		// The sources to offer, in combo order after "All sources".
		SLATE_ARGUMENT(TArray<EAirfoilSource>, Sources)
		SLATE_ARGUMENT(TArray<std::shared_ptr<aerotelic::Airfoil>>, ProjectAirfoils)
		// Compact preview (no legend / grid), for the modal picker.
		SLATE_ARGUMENT(bool, Compact)
		SLATE_ARGUMENT(float, ListHeight)
		SLATE_ARGUMENT(FVector2D, PreviewSize)
		// Double-click on a row (the picker's accept).
		SLATE_EVENT(FOnAirfoilChosen, OnActivated)
		SLATE_EVENT(FOnAirfoilChosen, OnSelectionChanged)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	// The selected, resolved airfoil (null when nothing usable is selected).
	std::shared_ptr<aerotelic::Airfoil> GetSelectedAirfoil() const { return Selected; }
	// Re-reads the database index and rebuilds the list (after a download).
	void Refresh();
	// The "Load .dat..." action: a file dialog, each file into the custom
	// library, then the list rebuilt and the first loaded one selected.
	void LoadDatFiles();

private:
	TArray<EAirfoilSource> Sources;
	TArray<std::shared_ptr<aerotelic::Airfoil>> ProjectAirfoils;
	bool bCompact = false;
	FOnAirfoilChosen OnActivated;
	FOnAirfoilChosen OnSelectionChanged;

	TArray<TSharedPtr<FString>> SourceOptions;
	TSharedPtr<FString> CurrentSource;
	TArray<TSharedPtr<FAirfoilListItem>> Items;
	TSharedPtr<SListView<TSharedPtr<FAirfoilListItem>>> ListView;
	TSharedPtr<SEditableTextBox> SearchBox;
	TSharedPtr<STextBlock> ResultLabel;
	TSharedPtr<STextBlock> StatName;
	TSharedPtr<STextBlock> StatThickness;
	TSharedPtr<STextBlock> StatCamber;
	TSharedPtr<SAerotelicAirfoilPreview> Preview;
	std::shared_ptr<aerotelic::Airfoil> Selected;
	FString PendingSelectKey;

	bool Offers(EAirfoilSource Source) const;
	void RebuildList();
	TSharedRef<ITableRow> MakeRow(TSharedPtr<FAirfoilListItem> Item, const TSharedRef<STableViewBase>& Owner);
	void OnSelect(TSharedPtr<FAirfoilListItem> Item, ESelectInfo::Type);
	void OnDoubleClick(TSharedPtr<FAirfoilListItem> Item);
	void StartDownload();
};

// The modal "Downloading Selig Airfoil Database" window
// (airfoil_panel.py DownloadDialog): status, progress bar, Cancel. OnDone
// runs after it closes, with the worker's success flag and message.
void ShowSeligDownloadDialog(TFunction<void(bool, const FString&)> OnDone);

// "2412", "naca 2412", "NACA-2412" -> "2412"; empty when the text is not
// a NACA 4-digit query (section_panel.py _naca_query).
FString ParseNacaQuery(const FString& Text);
