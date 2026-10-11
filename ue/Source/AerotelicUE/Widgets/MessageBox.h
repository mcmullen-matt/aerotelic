#pragma once

#include "CoreMinimal.h"
#include "Misc/MessageDialog.h"

// The application's message box: a Slate modal window with the message
// and OK, or Yes / No, in place of FMessageDialog's native Windows box.
// In the game the native box does not take the first click -- the frame
// window keeps the mouse until that click releases it, so every
// confirmation needed two -- while a Slate modal, like every other dialog
// here, answers at once. Same signature as FMessageDialog::Open so call
// sites read the same; outside an interactive Slate session (commandlets,
// -unattended) it defers to FMessageDialog, which logs and answers the
// default.
EAppReturnType::Type AerotelicMessageBox(EAppMsgType::Type Type, const FText& Message, const FText& Title = FText::GetEmpty());
