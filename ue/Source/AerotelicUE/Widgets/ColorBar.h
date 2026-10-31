#pragma once

#include "CoreMinimal.h"
#include "Widgets/SLeafWidget.h"

namespace aerotelic { struct Colormap; }

DECLARE_DELEGATE_OneParam(FOnAerotelicColorBarClicked, const FPointerEvent&);

// The colour bar beside a field picture: a vertical ramp of the colour map
// between the minimum (bottom) and maximum (top), tick labels along it and
// a title above. Clicking it fires OnClicked, which the owner answers with
// a popup to change the range and the map (see HighOrderPanelDialog).
// Attributes, so the bar follows the owner's settings without being told.
class SAerotelicColorBar : public SLeafWidget
{
public:
	SLATE_BEGIN_ARGS(SAerotelicColorBar)
		: _Colormap(nullptr), _MinValue(0.0), _MaxValue(1.0), _Title(), _DesiredSize(FVector2D(92.0, 280.0)) {}
		SLATE_ATTRIBUTE(const aerotelic::Colormap*, Colormap)
		SLATE_ATTRIBUTE(double, MinValue)
		SLATE_ATTRIBUTE(double, MaxValue)
		SLATE_ATTRIBUTE(FText, Title)
		SLATE_ARGUMENT(FVector2D, DesiredSize)
		SLATE_EVENT(FOnAerotelicColorBarClicked, OnClicked)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
		FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const override;
	virtual FVector2D ComputeDesiredSize(float) const override { return DesiredSize; }
	virtual FReply OnMouseButtonDown(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent) override;
	virtual FCursorReply OnCursorQuery(const FGeometry& MyGeometry, const FPointerEvent& CursorEvent) const override;

private:
	TAttribute<const aerotelic::Colormap*> Colormap;
	TAttribute<double> MinValue;
	TAttribute<double> MaxValue;
	TAttribute<FText> Title;
	FVector2D DesiredSize;
	FOnAerotelicColorBarClicked OnClicked;
};
