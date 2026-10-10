#pragma once

#include "CoreMinimal.h"

// The welcome screen -- WingDesigner's ui/splash.py: a frameless modal card
// shown once the main window is up, with the splash artwork fitted into a
// band, the product identity, the licence block, the legal small print, a
// link to the tutorial projects, a "Show this screen at start-up" tick and
// Close. Returns false without showing when the tick is off and bForce is
// false (Help -> Welcome Screen... forces it). When the user chooses the
// tutorials, the matrix opens after the card closes -- sequential, not
// nested, modals.
bool ShowAerotelicWelcomeScreen(bool bForce);
