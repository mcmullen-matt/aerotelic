#pragma once

#include "CoreMinimal.h"

// The application's own title bar (main_window.py _TitleBar on a frameless
// window): the product and tagline on the left, Minimise / Maximise-or-
// Restore / Close on the right, 32 px high. Built on the engine's
// SWindowTitleBarArea, so the OS handles dragging, snapping and the
// double-click to maximise through the window's title-bar zone, and the
// window buttons are the engine's own. Collapsed when the hosting window
// has an OS border (the Editor host), where the OS supplies all of this.
TSharedRef<class SWidget> MakeAerotelicTitleBar();
