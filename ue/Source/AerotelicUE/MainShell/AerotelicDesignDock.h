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
// The Design tab exists only while the user is editing an entity: it is
// opened by Edit (tree double-click, context menu, or adding an entity) and
// closed with its own close button or that of the window it was torn off
// into. With no live tab the frame collapses the column to zero width.
//
// HostWindow is the window the main viewport lives in (tear-off windows are
// placed relative to it); may be null. Returns the column's content host,
// empty until the first Edit.
TSharedRef<SWidget> BuildAerotelicDesignDock(const TSharedPtr<SWindow>& HostWindow);

// Brings the Design tab to the front where it lives, or opens it in the
// dock column if there is none.
void ShowAerotelicDesignTab();

// True while the Design tab lives in the main window's dock column (as
// opposed to a torn-off floating window). The frame collapses the column
// to zero width when this is false, so the 3D view takes the space.
bool IsAerotelicDesignTabDocked();

// Closes every docking area (including torn-off windows) and unregisters
// the spawners. Call from EndPlay.
void TearDownAerotelicDesignDock();
