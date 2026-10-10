#pragma once

#include "CoreMinimal.h"

// Geometry -> Boolean Operation... (main_window.py
// _show_boolean_operation_dialog): pick the operands -- every entity, the
// mirrored half of each mirrored one, every earlier boolean result -- in
// order (the first is the base; Cut subtracts the rest from it), the
// operation (Fuse / Cut / Common) and a unique name. Modal. On OK the
// result is added to the scene and selected; a failed operation is
// reported and adds nothing. Returns the status line to show.
FText ShowAerotelicBooleanOperationDialog();
