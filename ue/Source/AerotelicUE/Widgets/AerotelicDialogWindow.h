#pragma once

#include "CoreMinimal.h"
#include <string>

// A std::string from the core library as an FString -- every dialog needs
// this for a model's `name` field, an Airfoil's name, etc. Plain FString(x)
// alone does not do (char* and TCHAR* are different types; a caller's own
// ternary between one of these and a TEXT() literal fails to compile with a
// confusing cascade of operator[] errors, not a clear "incompatible pointer
// types" one -- wrap at the call site instead of re-deriving this each time).
inline FString StdToFString(const std::string& S) { return FString(S.c_str()); }

// Opens Content in a standalone top-level window (non-modal -- the editor
// viewport and every other open dialog stay usable), per the plan's "no
// docking, standalone windows" decision for the entity dialogs. Shared by
// all of them so the window chrome (size, title, close behaviour) is
// consistent rather than each dialog rolling its own.
void OpenAerotelicDialogWindow(const FText& Title, const TSharedRef<class SWidget>& Content,
	FVector2D ClientSize = FVector2D(460, 620));

// Shows an already-built dialog window, positioning it beside the left control
// panel and cascading each successive dialog so they do not cover the 3D view
// or stack exactly on top of one another. Entity dialogs that build their own
// SWindow (to track a single instance per entity, or to keep tab state alive)
// call this in place of FSlateApplication::AddWindow so they share the same
// placement. ClientSize must match the window's ClientSize, for the on-screen
// clamp. Safe to call on the game thread only.
void ShowAerotelicDialogWindow(const TSharedRef<class SWindow>& Window, FVector2D ClientSize);

// The main application window: in the standalone game, the window hosting the
// game viewport widget; in the Editor host, the active top-level window. Null
// only if neither can be found. Used to anchor dialogs and the dock column.
TSharedPtr<class SWindow> FindAerotelicMainWindow();
