#include "AerotelicDesignDock.h"
#include "AerotelicDesignPanel.h"

#include "Framework/Docking/TabManager.h"
#include "Framework/Application/SlateApplication.h"
#include "Widgets/Docking/SDockTab.h"
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

	TSharedRef<SDockTab> SpawnDesignTab(const FSpawnTabArgs& Args)
	{
		UE_LOG(LogAerotelicDock, Log, TEXT("Spawned Design tab"));
		return SNew(SDockTab)
			.TabRole(ETabRole::PanelTab)
			.Label(LOCTEXT("DesignTab", "Design"))
			// Always present, like the Qt dock column: tear it off or split
			// it, but there is no empty state to recover from.
			.CanEverClose(false)
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
}

TSharedRef<SWidget> BuildAerotelicDesignDock(const TSharedPtr<SWindow>& HostWindow)
{
	EnsureTabManager();

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

	UE_LOG(LogAerotelicDock, Log, TEXT("Restoring design dock layout; host window %s"), HostWindow.IsValid() ? TEXT("found") : TEXT("NOT found"));
	TSharedPtr<SWidget> Content = GTabManager->RestoreFrom(Layout, HostWindow);
	if (!Content.IsValid())
	{
		UE_LOG(LogAerotelicDock, Warning, TEXT("RestoreFrom returned no widget; dock column will be empty"));
		return SNew(STextBlock).Text(LOCTEXT("RestoreFailed", "Dock layout failed to restore"));
	}
	return Content.ToSharedRef();
}

void ShowAerotelicDesignTab()
{
	if (GTabManager.IsValid())
	{
		GTabManager->TryInvokeTab(FTabId(DesignTabId));
	}
}

bool IsAerotelicDesignTabDocked()
{
	if (!GTabManager.IsValid() || !FSlateApplication::IsInitialized())
	{
		return false;
	}
	TSharedPtr<SDockTab> Tab = GTabManager->FindExistingLiveTab(FTabId(DesignTabId));
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
}

#undef LOCTEXT_NAMESPACE
