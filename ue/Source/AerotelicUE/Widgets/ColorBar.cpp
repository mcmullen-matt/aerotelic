#include "ColorBar.h"

#include "Framework/Application/SlateApplication.h"
#include "Rendering/DrawElements.h"
#include "Fonts/FontMeasure.h"
#include "Styling/CoreStyle.h"

#include "aerotelic/colormaps.hpp"

namespace
{
	constexpr int32 kSlices = 96;
	constexpr double kBarWidth = 18.0;
	constexpr double kPadTop = 22.0;    // title above the bar
	constexpr double kPadBottom = 10.0;
	constexpr double kPadLeft = 8.0;
	constexpr int32 kTicks = 6;

	FString TickLabel(double V)
	{
		// Four significant figures, trailing zeros dropped, as the metric
		// read-outs do.
		FString S = FString::Printf(TEXT("%.4g"), V);
		if (S == TEXT("-0")) S = TEXT("0");
		return S;
	}
}

void SAerotelicColorBar::Construct(const FArguments& InArgs)
{
	Colormap = InArgs._Colormap;
	MinValue = InArgs._MinValue;
	MaxValue = InArgs._MaxValue;
	Title = InArgs._Title;
	DesiredSize = InArgs._DesiredSize;
	OnClicked = InArgs._OnClicked;
}

int32 SAerotelicColorBar::OnPaint(const FPaintArgs& Args, const FGeometry& Geometry, const FSlateRect& MyCullingRect,
	FSlateWindowElementList& Out, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const
{
	const FVector2D Size = Geometry.GetLocalSize();
	const FSlateBrush* White = FCoreStyle::Get().GetBrush("WhiteBrush");
	const FSlateFontInfo Font = FCoreStyle::GetDefaultFontStyle("Regular", 8);
	const FSlateFontInfo TitleFont = FCoreStyle::GetDefaultFontStyle("Bold", 9);
	const TSharedRef<FSlateFontMeasure> Measure = FSlateApplication::Get().GetRenderer()->GetFontMeasureService();

	// A translucent plate under everything so the labels read over any picture.
	FSlateDrawElement::MakeBox(Out, LayerId, Geometry.ToPaintGeometry(), White, ESlateDrawEffect::None, FLinearColor(0.0f, 0.0f, 0.0f, 0.35f));

	const double BarTop = kPadTop;
	const double BarBottom = Size.Y - kPadBottom;
	const double BarHeight = FMath::Max(BarBottom - BarTop, 1.0);
	const aerotelic::Colormap* Map = Colormap.Get();
	const double Lo = MinValue.Get(), Hi = MaxValue.Get();

	// The ramp, top = maximum.
	for (int32 i = 0; i < kSlices; ++i)
	{
		const double T0 = static_cast<double>(i) / kSlices, T1 = static_cast<double>(i + 1) / kSlices;
		const double Y0 = BarBottom - T1 * BarHeight, Y1 = BarBottom - T0 * BarHeight;
		FLinearColor C(0.5f, 0.5f, 0.5f);
		if (Map != nullptr)
		{
			const aerotelic::Rgb Rgb = Map->Sample(0.5 * (T0 + T1));
			C = FLinearColor(FColor(static_cast<uint8>(Rgb.r * 255.0f + 0.5f), static_cast<uint8>(Rgb.g * 255.0f + 0.5f), static_cast<uint8>(Rgb.b * 255.0f + 0.5f)));
		}
		FSlateDrawElement::MakeBox(Out, LayerId + 1,
			Geometry.ToPaintGeometry(FVector2f(static_cast<float>(kBarWidth), static_cast<float>(Y1 - Y0 + 0.5)), FSlateLayoutTransform(FVector2f(static_cast<float>(kPadLeft), static_cast<float>(Y0)))),
			White, ESlateDrawEffect::None, C);
	}
	// Outline.
	{
		TArray<FVector2f> L = {
			FVector2f((float)kPadLeft, (float)BarTop), FVector2f((float)(kPadLeft + kBarWidth), (float)BarTop),
			FVector2f((float)(kPadLeft + kBarWidth), (float)BarBottom), FVector2f((float)kPadLeft, (float)BarBottom), FVector2f((float)kPadLeft, (float)BarTop) };
		FSlateDrawElement::MakeLines(Out, LayerId + 2, Geometry.ToPaintGeometry(), L, ESlateDrawEffect::None, FLinearColor(0.85f, 0.85f, 0.85f), true, 1.0f);
	}
	// Ticks and labels.
	for (int32 k = 0; k < kTicks; ++k)
	{
		const double T = static_cast<double>(k) / (kTicks - 1);
		const double Y = BarBottom - T * BarHeight;
		TArray<FVector2f> Tick = { FVector2f((float)(kPadLeft + kBarWidth), (float)Y), FVector2f((float)(kPadLeft + kBarWidth + 4.0), (float)Y) };
		FSlateDrawElement::MakeLines(Out, LayerId + 2, Geometry.ToPaintGeometry(), Tick, ESlateDrawEffect::None, FLinearColor(0.85f, 0.85f, 0.85f), false, 1.0f);
		const FText Label = FText::FromString(TickLabel(Lo + T * (Hi - Lo)));
		const FVector2D TS = Measure->Measure(Label, Font);
		FSlateDrawElement::MakeText(Out, LayerId + 3, Geometry.ToOffsetPaintGeometry(FVector2D(kPadLeft + kBarWidth + 7.0, Y - TS.Y * 0.5)), Label, Font, ESlateDrawEffect::None, FLinearColor::White);
	}
	// Title.
	{
		const FText T = Title.Get();
		if (!T.IsEmpty())
		{
			FSlateDrawElement::MakeText(Out, LayerId + 3, Geometry.ToOffsetPaintGeometry(FVector2D(kPadLeft, 4.0)), T, TitleFont, ESlateDrawEffect::None, FLinearColor::White);
		}
	}
	return LayerId + 4;
}

FReply SAerotelicColorBar::OnMouseButtonDown(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent)
{
	if (MouseEvent.GetEffectingButton() == EKeys::LeftMouseButton)
	{
		OnClicked.ExecuteIfBound(MouseEvent);
		return FReply::Handled();
	}
	return FReply::Unhandled();
}

FCursorReply SAerotelicColorBar::OnCursorQuery(const FGeometry& MyGeometry, const FPointerEvent& CursorEvent) const
{
	return FCursorReply::Cursor(EMouseCursor::Hand);
}
