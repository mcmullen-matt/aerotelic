#pragma once

#include "CoreMinimal.h"
#include "Widgets/SLeafWidget.h"
#include <memory>

namespace aerotelic { class Airfoil; }

// The 2-D airfoil preview (airfoil_panel.py _draw_preview, on a matplotlib
// canvas there): the raw fit points as dots, the upper and lower NURBS
// surfaces, the camber line dashed, the chord line dash-dotted, a legend
// top-right and the name as the title; equal aspect, x over 0..1 with a
// margin, a faint grid. "Preview unavailable" when the airfoil cannot be
// sampled. Compact mode (the section picker) drops the legend and grid.
class SAerotelicAirfoilPreview : public SLeafWidget
{
public:
	SLATE_BEGIN_ARGS(SAerotelicAirfoilPreview)
		: _Compact(false), _DesiredSize(FVector2D(300, 300)) {}
		SLATE_ARGUMENT(bool, Compact)
		SLATE_ARGUMENT(FVector2D, DesiredSize)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	void SetAirfoil(std::shared_ptr<aerotelic::Airfoil> InAirfoil);
	std::shared_ptr<aerotelic::Airfoil> GetAirfoil() const { return Airfoil; }

	virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
		FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const override;
	virtual FVector2D ComputeDesiredSize(float) const override { return DesiredSize; }

private:
	std::shared_ptr<aerotelic::Airfoil> Airfoil;
	bool bCompact = false;
	FVector2D DesiredSize;
};
