#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"

#include "aerotelic/reference_image.hpp"

class SCurveEditor;

// The "Reference image" group under a curve editor -- WingDesigner's
// ui/reference_image_bar.py. Keeps only what cannot be dragged on the
// picture itself: Import..., Remove, Fit, Flip H, Flip V, Opacity, and a
// status line. The image record lives in the model's reference_images
// map under Key; the bar creates and removes it and hands it to the
// editor, which draws and drags it.
class SReferenceImageBar : public SCompoundWidget
{
public:
	// How a freshly imported (or Fit) image is placed, given the loaded
	// source size. The default spans x/L 0..1 vertically centred on 0.
	DECLARE_DELEGATE_TwoParams(FPlaceReferenceImage, aerotelic::ReferenceImageSpec&, FIntPoint);

	SLATE_BEGIN_ARGS(SReferenceImageBar)
		: _Units(FText::FromString(TEXT("x/L")))
		{}
		SLATE_ARGUMENT(TSharedPtr<SCurveEditor>, Editor)
		SLATE_ARGUMENT(aerotelic::ReferenceImageMap*, Images)
		SLATE_ARGUMENT(FString, Key)
		// "side view", "top view", "plan view", "front view"
		SLATE_ARGUMENT(FText, ViewLabel)
		SLATE_ARGUMENT(FText, Units)
		SLATE_EVENT(FPlaceReferenceImage, Placement)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

private:
	TSharedPtr<SCurveEditor> Editor;
	aerotelic::ReferenceImageMap* Images = nullptr;
	FString Key;
	FText ViewLabel;
	FText Units;
	FPlaceReferenceImage Placement;
	TSharedPtr<class STextBlock> StatusText;

	aerotelic::ReferenceImageSpec* Current() const;
	FString BaseDir() const;
	void Rebind();
	void Sync();
	FReply OnImport();
	FReply OnRemove();
	FReply OnFit();
	void Place(aerotelic::ReferenceImageSpec& Spec);
};
