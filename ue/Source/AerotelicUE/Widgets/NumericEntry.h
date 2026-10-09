#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/Input/SSpinBox.h"

DECLARE_DELEGATE_TwoParams(FOnAerotelicValueCommitted, double /*NewValue*/, ETextCommit::Type);

// The one numeric field every dialog uses -- the Slate equivalent of
// ui/numeric_input.py's FloatSpin: a wide default range (no ceiling that
// blocks typing a few more digits) and bounds passed only when they are
// definitional (a span fraction really does live in [0, 1]), never merely
// for cosmetic tidiness. SSpinBox<double> already avoids FloatSpin's other
// two fixes (it does not zero-pad past what was typed, and places no cap on
// typed precision), so this wrapper only needs to set sensible defaults.
class SAerotelicNumericEntry : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SAerotelicNumericEntry)
		: _Value(0.0)
		, _MinValue(-1.0e9)
		, _MaxValue(1.0e9)
		, _Delta(1.0)
		, _Suffix()
		, _LiveUpdate(true)
		{}
		SLATE_ATTRIBUTE(double, Value)
		SLATE_ARGUMENT(double, MinValue)
		SLATE_ARGUMENT(double, MaxValue)
		SLATE_ARGUMENT(double, Delta)
		SLATE_ARGUMENT(FText, Suffix)
		// True (default): commits on every keystroke/drag, matching every
		// FloatSpin that never set setKeyboardTracking(False). False: commits
		// only on Enter/focus-loss/arrow-step, matching the panels that did
		// (chevron count/depth, every primitive field, propeller layout...).
		SLATE_ARGUMENT(bool, LiveUpdate)
		SLATE_EVENT(FOnAerotelicValueCommitted, OnValueCommitted)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

private:
	FOnAerotelicValueCommitted OnValueCommittedDelegate;
	void HandleCommitted(double NewValue, ETextCommit::Type CommitType);
	void HandleChanged(double NewValue);
	bool bLiveUpdate = true;
};
