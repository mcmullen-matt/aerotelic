#pragma once

#include "CoreMinimal.h"

class SWidget;
class SWindow;

// The right-hand dock column: a private FTabManager (child of the global
// one) hosting the Design tab -- the Qt version's right-hand column of
// tabified design docks. Tabs can be dragged between stacks, torn off into
// floating windows and docked back (verified 2026-10-09; see README).
//
// Why a private manager: tabs spawned from the global tab manager's nomad
// spawners re-dock into tab wells but FDockingDragOperation::CanDockInNode
// refuses to let them split a global-manager area.
//
// HostWindow is the window the main viewport lives in (tear-off windows are
// placed relative to it); may be null.
TSharedRef<SWidget> BuildAerotelicDesignDock(const TSharedPtr<SWindow>& HostWindow);

// Brings the Design tab to the front (re-spawning it if it was closed).
void ShowAerotelicDesignTab();

// True while the Design tab lives in the main window's dock column (as
// opposed to a torn-off floating window). The frame collapses the column
// to zero width when this is false, so the 3D view takes the space.
bool IsAerotelicDesignTabDocked();

// Closes every docking area (including torn-off windows) and unregisters
// the spawners. Call from EndPlay.
void TearDownAerotelicDesignDock();
