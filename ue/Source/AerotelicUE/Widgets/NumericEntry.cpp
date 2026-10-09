#include "NumericEntry.h"
#include "Misc/MessageDialog.h"

namespace
{
	// Every dialog's numeric fields funnel through here, so this is the one
	// place that needs to catch a backend validation throw (SetLength,
	// SetRadius, AddSection, ...) before it unwinds as an unhandled C++
	// exception and takes the whole Editor down with it -- confirmed the
	// hard way (NacelleModel::SetLength() rejecting a bad value crashed the
	// process outright, since nothing between here and there was
	// try/catch-guarded).
	void InvokeGuarded(const FOnAerotelicValueCommitted& Delegate, double NewValue, ETextCommit::Type CommitType)
	{
		try
		{
			Delegate.ExecuteIfBound(NewValue, CommitType);
		}
		catch (const std::exception& e)
		{
			FMessageDialog::Open(EAppMsgType::Ok,
				FText::FromString(FString::Printf(TEXT("That value was rejected:\n%s"), UTF8_TO_TCHAR(e.what()))));
		}
	}
}

void SAerotelicNumericEntry::Construct(const FArguments& InArgs)
{
	bLiveUpdate = InArgs._LiveUpdate;
	OnValueCommittedDelegate = InArgs._OnValueCommitted;

	ChildSlot
	[
		SNew(SSpinBox<double>)
		.Value(InArgs._Value)
		.MinValue(InArgs._MinValue)
		.MaxValue(InArgs._MaxValue)
		.MinSliderValue(InArgs._MinValue)
		.MaxSliderValue(InArgs._MaxValue)
		.Delta(InArgs._Delta)
		// FloatSpin's precision: "at least 9" fractional digits, no padding
		// beyond what's meaningful. SSpinBox defaults to 1-6, which pads a
		// whole number to "40.0" and truncates past 6 places -- both of
		// ui/numeric_input.py's fixes, just with different default numbers.
		.MinFractionalDigits(0)
		.MaxFractionalDigits(9)
		.MinDesiredWidth(90.0f)
		.Justification(ETextJustify::Left)
		.OnValueChanged(this, &SAerotelicNumericEntry::HandleChanged)
		.OnValueCommitted(this, &SAerotelicNumericEntry::HandleCommitted)
	];
}

void SAerotelicNumericEntry::HandleChanged(double NewValue)
{
	if (bLiveUpdate)
	{
		InvokeGuarded(OnValueCommittedDelegate, NewValue, ETextCommit::Default);
	}
}

void SAerotelicNumericEntry::HandleCommitted(double NewValue, ETextCommit::Type CommitType)
{
	if (!bLiveUpdate)
	{
		InvokeGuarded(OnValueCommittedDelegate, NewValue, CommitType);
	}
}
