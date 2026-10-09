#include "CurveTabControls.h"
#include "CurveEditor.h"

#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Layout/SSpacer.h"
#include "Widgets/Layout/SWrapBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SSpinBox.h"
#include "Styling/CoreStyle.h"

#define LOCTEXT_NAMESPACE "CurveTabControls"

using namespace aerotelic;

TSharedRef<SWidget> MakeDegreeSpin(const FText& Label, TFunction<NurbsCurve*()> GetCurve,
	TSharedPtr<SCurveEditor> Editor, FSimpleDelegate OnChanged, FText ToolTip)
{
	if (ToolTip.IsEmpty())
	{
		ToolTip = LOCTEXT("DegreeTip", "Curve degree  (1 = piecewise-linear, 2 = quadratic, 3 = cubic ...).\nMaximum is number-of-control-points - 1.");
	}
	auto Apply = [GetCurve, Editor, OnChanged](int32 V)
	{
		NurbsCurve* C = GetCurve();
		if (C == nullptr || V == C->degree())
		{
			return;
		}
		try
		{
			C->SetDegree(V);
		}
		catch (const std::exception&)
		{
			// An existing knot multiplicity forbids this degree (periodic
			// curves): leave it, as the Python panels do.
			return;
		}
		if (Editor.IsValid())
		{
			Editor->RequestRedraw();
		}
		OnChanged.ExecuteIfBound();
	};
	auto MaxDegree = [GetCurve]() -> TOptional<int32>
	{
		NurbsCurve* C = GetCurve();
		return TOptional<int32>(C ? FMath::Max(C->max_degree(), 1) : 1);
	};

	return SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 6, 0)
		[ SNew(STextBlock).Text(Label) ]
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
		[
			SNew(SBox).WidthOverride(56)
			[
				SNew(SSpinBox<int32>)
				.ToolTipText(ToolTip)
				.MinValue(1)
				.MaxValue_Lambda(MaxDegree)
				.MinSliderValue(1)
				.MaxSliderValue_Lambda(MaxDegree)
				.Delta(1)
				.Value_Lambda([GetCurve]() { NurbsCurve* C = GetCurve(); return C ? C->degree() : 1; })
				.OnValueChanged_Lambda([Apply](int32 V) { Apply(V); })
				.OnValueCommitted_Lambda([Apply](int32 V, ETextCommit::Type) { Apply(V); })
			]
		];
}

TSharedRef<SWidget> MakeControlButton(const FText& Label, const FText& ToolTip, FOnClicked OnClicked)
{
	return SNew(SButton)
		.Text(Label)
		.ToolTipText(ToolTip)
		.ContentPadding(FMargin(8, 3))
		.OnClicked(OnClicked);
}

TSharedRef<SWidget> MakeControlsRow(const TArray<TSharedRef<SWidget>>& Leading, const TArray<TSharedRef<SWidget>>& Trailing)
{
	// A wrap box rather than a single row: the dock column is narrow, so
	// the inputs and the reset buttons flow onto further lines as needed.
	TSharedRef<SWrapBox> Row = SNew(SWrapBox).UseAllottedSize(true).InnerSlotPadding(FVector2D(8, 4));
	for (const TSharedRef<SWidget>& W : Leading)
	{
		Row->AddSlot().VAlign(VAlign_Center) [ W ];
	}
	for (const TSharedRef<SWidget>& W : Trailing)
	{
		Row->AddSlot().VAlign(VAlign_Center) [ W ];
	}
	return SNew(SBox).Padding(FMargin(0, 4, 0, 0)) [ Row ];
}

TSharedRef<SWidget> MakeInfoLabel(const FText& Text)
{
	return SNew(STextBlock)
		.Text(Text)
		.AutoWrapText(true)
		.Font(FCoreStyle::GetDefaultFontStyle("Regular", 8))
		.ColorAndOpacity(FSlateColor(FLinearColor(0.53f, 0.53f, 0.53f)));
}

TSharedRef<SWidget> MakeMetricRow(const FText& Label, TAttribute<FText> Value)
{
	return SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 1, 8, 1)
		[ SNew(STextBlock).Text(Label) ]
		+ SHorizontalBox::Slot().FillWidth(1.0f).HAlign(HAlign_Right).VAlign(VAlign_Center).Padding(0, 1)
		[ SNew(STextBlock).Text(Value).ColorAndOpacity(FSlateColor(FLinearColor(0.85f, 0.85f, 0.85f))) ];
}

TSharedRef<SWidget> MakeGroup(const FText& Title, TSharedRef<SWidget> Content)
{
	return SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 2)
		[ SNew(STextBlock).Text(Title).Font(FCoreStyle::GetDefaultFontStyle("Bold", 9)) ]
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SBorder)
			.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
			.BorderBackgroundColor(FLinearColor(0.14f, 0.14f, 0.15f, 1.0f))
			.Padding(FMargin(8, 6))
			[ Content ]
		];
}

TSharedRef<SWidget> MakeHelpTab(const FText& HelpText)
{
	return SNew(SScrollBox)
		+ SScrollBox::Slot().Padding(8)
		[
			SNew(STextBlock)
			.Text(HelpText)
			.AutoWrapText(true)
			.LineHeightPercentage(1.15f)
		];
}

void ResetCurveFromSpec(NurbsCurve& Curve, const NurbsSpec& Spec, FVector2D Scale)
{
	Mat P = Spec.poles;
	for (int i = 0; i < P.rows(); ++i)
	{
		P(i, 0) *= Scale.X;
		if (P.cols() > 1) P(i, 1) *= Scale.Y;
	}
	std::vector<double> W = Spec.weights ? *Spec.weights : std::vector<double>(P.rows(), 1.0);
	Curve.SetPolesAndWeights(P, W);
	Curve.ResetKnotsPublic();
	try
	{
		Curve.SetDegree(Spec.degree);
	}
	catch (const std::exception&) {}
	Curve.ResetKnotsPublic();
}

void ResetCurveToLine(NurbsCurve& Curve, double Y0, double Y1)
{
	Curve.SetPolesAndWeights(Mat::FromRows({ { 0.0, Y0 }, { 1.0, Y1 } }), std::vector<double>{ 1.0, 1.0 });
	Curve.SetDegree(1);
	Curve.ResetKnotsPublic();
}

FText FormatG(double V, int32 Significant)
{
	return FText::FromString(FString::Printf(TEXT("%.*g"), Significant, V));
}

FText FormatF(double V, int32 Decimals)
{
	return FText::FromString(FString::Printf(TEXT("%.*f"), Decimals, V));
}

FText DashText()
{
	return FText::FromString(TEXT("—"));
}

#undef LOCTEXT_NAMESPACE
