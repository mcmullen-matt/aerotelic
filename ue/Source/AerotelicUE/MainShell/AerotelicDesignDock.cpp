#include "AerotelicDesignDock.h"
#include "AerotelicDesignPanel.h"

#include "Framework/Docking/TabManager.h"
#include "Framework/Application/SlateApplication.h"
#include "Widgets/Docking/SDockTab.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/AerotelicDialogWindow.h"

#define LOCTEXT_NAMESPACE "AerotelicDesignDock"

DEFINE_LOG_CATEGORY_STATIC(LogAerotelicDock, Log, All);

namespace
{
	const FName DesignTabId("AerotelicDesign");

	TSharedPtr<FTabManager> GTabManager;
	// The private manager needs an owner major tab; it is never shown.
	TSharedPtr<SDockTab> GOwnerTab;
	// The column's content host. Empty until the first Edit; thereafter it
	// holds the docking area the Design tab lives in (or was torn out of).
	TSharedPtr<SBox> GHost;
	TWeakPtr<SWindow> GHostWindow;

	TSharedRef<SDockTab> SpawnDesignTab(const FSpawnTabArgs& Args)
	{
		UE_LOG(LogAerotelicDock, Log, TEXT("Spawned Design tab"));
		// Closable (the default): the tab's own close button, and the close
		// button of a floating window holding it, both work. A tab that can
		// never close also pins its floating window open, since the window
		// asks every tab for permission before it is destroyed.
		return SNew(SDockTab)
			.TabRole(ETabRole::PanelTab)
			.Label(LOCTEXT("DesignTab", "Design"))
			[
				SNew(SAerotelicDesignPanel)
			];
	}

	void EnsureTabManager()
	{
		if (GTabManager.IsValid())
		{
			return;
		}
		GOwnerTab = SNew(SDockTab).TabRole(ETabRole::MajorTab);
		GTabManager = FGlobalTabmanager::Get()->NewTabManager(GOwnerTab.ToSharedRef());
		GTabManager->RegisterTabSpawner(DesignTabId, FOnSpawnTab::CreateStatic(&SpawnDesignTab))
			.SetDisplayName(LOCTEXT("DesignTab", "Design"));
	}

	TSharedPtr<SDockTab> FindLiveDesignTab()
	{
		return GTabManager.IsValid() ? GTabManager->FindExistingLiveTab(FTabId(DesignTabId)) : nullptr;
	}

	// Builds a fresh docking area with the Design tab open and makes it the
	// column's content. Used whenever no Design tab is live: at the first
	// Edit, and after the tab was closed (docked or floating).
	//
	// Why not FTabManager::TryInvokeTab for the re-open: when the tab is
	// closed in a torn-off window the manager files that window's layout
	// under its collapsed areas and TryInvokeTab searches those first, so
	// the tab would come back as a floating window rather than in the
	// column. Restoring a new primary area always lands it in the column.
	void OpenDesignTabInColumn()
	{
		TSharedRef<FTabManager::FLayout> Layout = FTabManager::NewLayout("AerotelicDesignDock_v1")
			->AddArea
			(
				FTabManager::NewPrimaryArea()
				->SetOrientation(Orient_Vertical)
				->Split
				(
					FTabManager::NewStack()
					->AddTab(DesignTabId, ETabState::OpenedTab)
					->SetForegroundTab(FTabId(DesignTabId))
				)
			);

		TSharedPtr<SWindow> HostWindow = GHostWindow.Pin();
		UE_LOG(LogAerotelicDock, Log, TEXT("Opening the Design tab in the dock column; host window %s"), HostWindow.IsValid() ? TEXT("found") : TEXT("NOT found"));
		TSharedPtr<SWidget> Content = GTabManager->RestoreFrom(Layout, HostWindow);
		if (!Content.IsValid())
		{
			UE_LOG(LogAerotelicDock, Warning, TEXT("RestoreFrom returned no widget; dock column stays empty"));
			return;
		}
		if (GHost.IsValid())
		{
			GHost->SetContent(Content.ToSharedRef());
		}
	}
}

TSharedRef<SWidget> BuildAerotelicDesignDock(const TSharedPtr<SWindow>& HostWindow)
{
	EnsureTabManager();
	GHostWindow = HostWindow;
	// Nothing is docked until the user edits an entity, so the frame keeps
	// the column collapsed and the 3D view has the full width.
	GHost = SNew(SBox);
	return GHost.ToSharedRef();
}

void ShowAerotelicDesignTab()
{
	if (!GTabManager.IsValid())
	{
		return;
	}
	if (FindLiveDesignTab().IsValid())
	{
		// Docked or floating: bring it to the front where it is.
		GTabManager->TryInvokeTab(FTabId(DesignTabId));
		return;
	}
	OpenDesignTabInColumn();
}

bool IsAerotelicDesignTabDocked()
{
	if (!FSlateApplication::IsInitialized())
	{
		return false;
	}
	TSharedPtr<SDockTab> Tab = FindLiveDesignTab();
	if (!Tab.IsValid())
	{
		return false;
	}
	// The only dock area inside the main window is the frame's column, so
	// "docked" is simply "lives in the main window".
	TSharedPtr<SWindow> TabWindow = FSlateApplication::Get().FindWidgetWindow(Tab.ToSharedRef());
	TSharedPtr<SWindow> MainWindow = FindAerotelicMainWindow();
	return TabWindow.IsValid() && TabWindow == MainWindow;
}

void TearDownAerotelicDesignDock()
{
	if (!GTabManager.IsValid())
	{
		return;
	}
	UE_LOG(LogAerotelicDock, Log, TEXT("Tearing down design dock"));
	GTabManager->CloseAllAreas();
	GTabManager->UnregisterAllTabSpawners();
	GTabManager.Reset();
	GOwnerTab.Reset();
	GHost.Reset();
	GHostWindow.Reset();
}

#undef LOCTEXT_NAMESPACE
