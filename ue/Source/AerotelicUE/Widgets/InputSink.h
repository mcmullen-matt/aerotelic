#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"

// Wraps a panel of the application frame so that mouse buttons and wheel
// events its children leave unhandled stop here instead of bubbling on to
// the game viewport. The frame is a viewport overlay, i.e. a *child* of the
// SViewport, so without this a right-click on a scene-tree row (which the
// row only claims on release) reaches the viewport, which captures the
// mouse in GameAndUI mode and swallows the release -- and the context menu
// never opens. Likewise a wheel turn over a panel would zoom the 3D view.
class SAerotelicInputSink : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SAerotelicInputSink) {}
		SLATE_DEFAULT_SLOT(FArguments, Content)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs)
	{
		ChildSlot [ InArgs._Content.Widget ];
	}

	virtual FReply OnMouseButtonDown(const FGeometry&, const FPointerEvent&) override { return FReply::Handled(); }
	virtual FReply OnMouseButtonUp(const FGeometry&, const FPointerEvent&) override { return FReply::Handled(); }
	virtual FReply OnMouseButtonDoubleClick(const FGeometry&, const FPointerEvent&) override { return FReply::Handled(); }
	virtual FReply OnMouseWheel(const FGeometry&, const FPointerEvent&) override { return FReply::Handled(); }
};
