#pragma once

#include "Modules/ModuleManager.h"

// Primary game module of the standalone Aerotelic application. In the
// standalone AerotelicUE.exe there is nothing to do at module scope: the
// game mode (Standalone/AerotelicGameMode) builds the scene lighting and
// the player controller (Standalone/AerotelicPlayerController) puts the
// shell on screen. When the same module is loaded by the Unreal Editor
// (the AerotelicUEEditor target, kept for the test commandlets), it also
// registers the legacy "Window -> Aerotelic" nomad tab.
class FAerotelicUEModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

private:
#if WITH_EDITOR
	TSharedRef<class SDockTab> SpawnMainShellTab(const class FSpawnTabArgs& Args);
	static const FName MainShellTabName;
	bool bRegisteredEditorTab = false;
#endif
};
