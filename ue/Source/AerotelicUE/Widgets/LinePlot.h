#pragma once

#include "CoreMinimal.h"
#include "Widgets/SLeafWidget.h"

// One curve of a plot: points in data units, in x order as given.
struct FAerotelicPlotSeries
{
	FString Label;
	FLinearColor Color = FLinearColor::White;
	TArray<FVector2D> Points;
	bool bLine = true;
	bool bMarkers = true;
	bool bDashed = false;
	float LineWidth = 1.5f;
	// Plotted against the right-hand y axis (its own range and ticks) --
	// a second quantity in other units against the same x, e.g. a section
	// coefficient beside a load per unit span.
	bool bRightAxis = false;
};

// A plain x-y chart painted with Slate: axes with "nice" ticks fitted to
// the data, a grid, a zero line when zero is in range, the series as
// polylines with optional markers (dashed for a reference curve), an
// optional right-hand y axis for series in other units (its own range and
// ticks, no grid, its zero on the same line as the left axis's zero), a
// legend when there is more than one series, a title and axis labels. Read-only; what an
// analysis dialog shows a result with (the span loading, a coefficient
// against the angle of attack). Not the NURBS curve editor, which is an
// editor; this is only a picture of numbers.
class SAerotelicLinePlot : public SLeafWidget
{
public:
	SLATE_BEGIN_ARGS(SAerotelicLinePlot)
		: _Title(), _XLabel(), _YLabel(), _DesiredSize(FVector2D(320.0, 220.0)), _Compact(false) {}
		SLATE_ARGUMENT(FText, Title)
		SLATE_ARGUMENT(FText, XLabel)
		SLATE_ARGUMENT(FText, YLabel)
		SLATE_ARGUMENT(FVector2D, DesiredSize)
		// Smaller margins and fonts, for a grid of small multiples.
		SLATE_ARGUMENT(bool, Compact)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	void SetSeries(TArray<FAerotelicPlotSeries> InSeries);
	void SetTitle(const FText& InTitle);
	// The right-hand label is drawn when any series is on the right axis.
	void SetAxisLabels(const FText& InX, const FText& InY, const FText& InYRight = FText::GetEmpty());
	// A note drawn in the plot area (e.g. why it is empty).
	void SetMessage(const FText& InMessage);

	virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
		FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const override;
	virtual FVector2D ComputeDesiredSize(float) const override { return DesiredSize; }

private:
	TArray<FAerotelicPlotSeries> Series;
	FText Title, XLabel, YLabel, YRightLabel, Message;
	FVector2D DesiredSize;
	bool bCompact = false;
};
