#pragma once

#include "CoreMinimal.h"
#include "AerotelicLicence.h"

// The start-up gate -- WingDesigner's ui/activation_dialog.py: runs the
// licence check and, when it fails, shows a modal dialog ("Aerotelic needs a
// valid licence to start.") with the reason, a key entry, Check again /
// Activate / Quit. Returns the passing result, or unset when the user quit
// -- the caller then exits; there is no read-only mode and no trial.
TOptional<AerotelicLicence::FCheckResult> EnsureAerotelicLicensed();
