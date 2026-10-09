#pragma once

#include "CoreMinimal.h"

// A single-field numeric modal prompt -- the Slate equivalent of
// QInputDialog.getDouble (CurveEditor's "Set Weight...") and
// ui/panel_base.py's ask_circle_radius (shared by both nacelle dialogs'
// "Reset to Circle..." buttons). Blocks until the user accepts or cancels;
// returns unset on cancel.
TOptional<double> PromptForDouble(const FText& Title, const FText& Label,
	double InitialValue, double MinValue, double MaxValue, double Delta = 0.01);

// A single-line text prompt (QInputDialog.getText), used by the scene tree's
// Rename. Blocks until the user accepts or cancels; returns unset on cancel.
TOptional<FString> PromptForString(const FText& Title, const FText& Label, const FString& InitialValue);
