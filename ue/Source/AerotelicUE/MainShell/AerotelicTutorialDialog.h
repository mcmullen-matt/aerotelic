#pragma once

#include "CoreMinimal.h"

// "Tutorial Projects" -- WingDesigner's ui/tutorials.py: a modal matrix of
// the example projects delivered with the application (Projects/*.yaml),
// each row's contents read off disk when the dialog opens so a row can
// never claim what the file does not have; a missing file is greyed and
// unopenable. Open Project loads the row without adopting its path, so the
// first Save asks for a location rather than overwriting the shipped file.
void ShowAerotelicTutorialDialog();
