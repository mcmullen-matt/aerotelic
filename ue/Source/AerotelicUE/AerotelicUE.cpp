#include "AerotelicUE.h"

#include "MainShell/AerotelicCommands.h"

#if WITH_EDITOR
#include "Widgets/Docking/SDockTab.h"
#include "WorkspaceMenuStructure.h"
#include "WorkspaceMenuStructureModule.h"
#include "Framework/Docking/TabManager.h"
#include "MainShell/AerotelicMainShell.h"
#endif

#define LOCTEXT_NAMESPACE "FAerotelicUEModule"

#if WITH_EDITOR
const FName FAerotelicUEModule::MainShellTabName(TEXT("AerotelicMainShell"));
#endif

void FAerotelicUEModule::StartupModule()
{
	// RuntimeDependencies in AerotelicUE.Build.cs stages every OCCT DLL next
	// to the output binary, which Windows finds via its "search the loading
	// module's directory" rule -- no AddDllDirectory() call needed here.

	// Menu / toolbar / shortcut commands shared by the standalone frame and
	// the Editor-hosted fallback.
	FAerotelicCommands::Register();

#if WITH_EDITOR
	// Only when hosted by the real Editor (not a commandlet run, not the
	// standalone game): offer the shell as a Window-menu tab.
	if (GIsEditor && !IsRunningCommandlet())
	{
		FGlobalTabmanager::Get()->RegisterNomadTabSpawner(
			MainShellTabName,
			FOnSpawnTab::CreateRaw(this, &FAerotelicUEModule::SpawnMainShellTab))
			.SetDisplayName(LOCTEXT("MainShellTabTitle", "Aerotelic"))
			.SetTooltipText(LOCTEXT("MainShellTabTooltip", "Parametric aircraft-geometry CAD"))
			.SetMenuType(ETabSpawnerMenuType::Enabled)
			.SetGroup(WorkspaceMenu::GetMenuStructure().GetToolsCategory());
		bRegisteredEditorTab = true;
	}
#endif
}

void FAerotelicUEModule::ShutdownModule()
{
	FAerotelicCommands::Unregister();
#if WITH_EDITOR
	if (bRegisteredEditorTab)
	{
		FGlobalTabmanager::Get()->UnregisterNomadTabSpawner(MainShellTabName);
		bRegisteredEditorTab = false;
	}
#endif
}

#if WITH_EDITOR
TSharedRef<SDockTab> FAerotelicUEModule::SpawnMainShellTab(const FSpawnTabArgs& Args)
{
	return SNew(SDockTab)
		.TabRole(ETabRole::NomadTab)
		[
			SNew(SAerotelicMainShell)
		];
}
#endif

#undef LOCTEXT_NAMESPACE

IMPLEMENT_PRIMARY_GAME_MODULE(FAerotelicUEModule, AerotelicUE, "AerotelicUE");
