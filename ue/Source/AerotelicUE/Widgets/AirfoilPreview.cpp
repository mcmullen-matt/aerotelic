#include "AirfoilPreview.h"

#include "Framework/Application/SlateApplication.h"
#include "Rendering/DrawElements.h"
#include "Fonts/FontMeasure.h"
#include "Styling/CoreStyle.h"

#include "aerotelic/airfoil.hpp"

namespace
{
	// airfoil_panel.py's colours (sRGB hex, converted to linear by FColor).
	const FLinearColor kBg(FColor::FromHex(TEXT("#0d1117")));
	const FLinearColor kPoints(FColor::FromHex(TEXT("#c8d53a")));
	const FLinearColor kUpper(FColor::FromHex(TEXT("#d56b3a")));
	const FLinearColor kLower(FColor::FromHex(TEXT("#3a7bd5")));
	const FLinearColor kCamber(FColor::FromHex(TEXT("#30f05a")));
	const FLinearColor kChord(1.0f, 1.0f, 1.0f, 0.8f);
	const FLinearColor kGrid(FColor::FromHex(TEXT("#2a2f38")));
	const FLinearColor kTitle(FColor::FromHex(TEXT("#cccccc")));

	FVector2f ToF(const FVector2D& V) { return FVector2f((float)V.X, (float)V.Y); }
}

void SAerotelicAirfoilPreview::Construct(const FArguments& InArgs)
{
	bCompact = InArgs._Compact;
	DesiredSize = InArgs._DesiredSize;
}

void SAerotelicAirfoilPreview::SetAirfoil(std::shared_ptr<aerotelic::Airfoil> InAirfoil)
{
	Airfoil = InAirfoil;
	Invalidate(EInvalidateWidget::Paint);
}

int32 SAerotelicAirfoilPreview::OnPaint(const FPaintArgs& Args, const FGeometry& Geometry, const FSlateRect& MyCullingRect,
	FSlateWindowElementList& Out, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const
{
	const FVector2D Size = Geometry.GetLocalSize();
	FSlateDrawElement::MakeBox(Out, LayerId, Geometry.ToPaintGeometry(), FCoreStyle::Get().GetBrush("WhiteBrush"), ESlateDrawEffect::None, kBg);
	if (!Airfoil)
	{
		return LayerId + 1;
	}
	const FSlateFontInfo Font = FCoreStyle::GetDefaultFontStyle("Regular", 8);
	const TSharedRef<FSlateFontMeasure> Measure = FSlateApplication::Get().GetRenderer()->GetFontMeasureService();

	// Equal aspect: x in [-0.05, 1.05] across the width, y centred on the
	// chord line with the same scale.
	const double Margin = 12.0;
	const double Top = bCompact ? 8.0 : 18.0;
	const double Scale = (Size.X - 2.0 * Margin) / 1.1;
	const double X0 = Margin + 0.05 * Scale;
	const double YMid = Top + (Size.Y - Top) * 0.5;
	auto Map = [&](double X, double Y) { return FVector2f((float)(X0 + X * Scale), (float)(YMid - Y * Scale)); };

	try
	{
		constexpr int32 N = 200;
		std::vector<double> T(N);
		for (int32 i = 0; i < N; ++i) T[i] = (double)i / (N - 1);

		if (!bCompact)
		{
			// Grid every 0.1 in x and y over the visible band.
			const double YSpan = (Size.Y - Top) * 0.5 / Scale;
			for (double X = 0.0; X <= 1.0001; X += 0.1)
			{
				TArray<FVector2f> L = { Map(X, -YSpan), Map(X, YSpan) };
				FSlateDrawElement::MakeLines(Out, LayerId + 1, Geometry.ToPaintGeometry(), L, ESlateDrawEffect::None, kGrid, false, 1.0f);
			}
			for (double Y = -FMath::FloorToDouble(YSpan * 10.0) / 10.0; Y <= YSpan; Y += 0.1)
			{
				TArray<FVector2f> L = { Map(-0.05, Y), Map(1.05, Y) };
				FSlateDrawElement::MakeLines(Out, LayerId + 1, Geometry.ToPaintGeometry(), L, ESlateDrawEffect::None, kGrid, false, 1.0f);
			}
		}

		// Chord line (dash-dot in the Python; dashed here).
		for (double X = 0.0; X < 1.0; X += 0.05)
		{
			TArray<FVector2f> L = { Map(X, 0.0), Map(FMath::Min(X + 0.03, 1.0), 0.0) };
			FSlateDrawElement::MakeLines(Out, LayerId + 2, Geometry.ToPaintGeometry(), L, ESlateDrawEffect::None, kChord, true, 1.0f);
		}

		// Camber, dashed.
		{
			const std::vector<double> C = Airfoil->Camber(T);
			for (int32 i = 0; i + 1 < N; i += 2)
			{
				TArray<FVector2f> L = { Map(T[i], C[i]), Map(T[i + 1], C[i + 1]) };
				FSlateDrawElement::MakeLines(Out, LayerId + 2, Geometry.ToPaintGeometry(), L, ESlateDrawEffect::None, kCamber, true, 1.0f);
			}
		}

		// The fit points as small dots, under the curves as the Python plots them.
		{
			const aerotelic::Mat& P = Airfoil->fit_points();
			for (int32 i = 0; i < P.rows(); ++i)
			{
				const FVector2f C = Map(P(i, 0), P(i, 1));
				FSlateDrawElement::MakeBox(Out, LayerId + 3, Geometry.ToPaintGeometry(FVector2f(3.0f, 3.0f), FSlateLayoutTransform(FVector2f(C.X - 1.5f, C.Y - 1.5f))),
					FCoreStyle::Get().GetBrush("WhiteBrush"), ESlateDrawEffect::None, kPoints);
			}
		}

		// Upper and lower surfaces.
		{
			const aerotelic::Mat Upper = Airfoil->UpperXY(T);
			const aerotelic::Mat Lower = Airfoil->LowerXY(T);
			TArray<FVector2f> U, L;
			for (int32 i = 0; i < Upper.rows(); ++i) U.Add(Map(Upper(i, 0), Upper(i, 1)));
			for (int32 i = 0; i < Lower.rows(); ++i) L.Add(Map(Lower(i, 0), Lower(i, 1)));
			FSlateDrawElement::MakeLines(Out, LayerId + 4, Geometry.ToPaintGeometry(), U, ESlateDrawEffect::None, kUpper, true, 1.5f);
			FSlateDrawElement::MakeLines(Out, LayerId + 4, Geometry.ToPaintGeometry(), L, ESlateDrawEffect::None, kLower, true, 1.5f);
		}

		if (!bCompact)
		{
			// Legend top-right.
			struct FItem { const TCHAR* Label; FLinearColor Color; };
			const FItem Items[] = { { TEXT("Training Point"), kPoints }, { TEXT("Upper NURB"), kUpper }, { TEXT("Lower NURB"), kLower }, { TEXT("Camber"), kCamber } };
			double Y = Top + 4.0;
			for (const FItem& It : Items)
			{
				const FText Text = FText::FromString(It.Label);
				const FVector2D TS = Measure->Measure(Text, Font);
				const double X = Size.X - Margin - TS.X;
				TArray<FVector2f> Swatch = { FVector2f((float)(X - 22.0), (float)(Y + TS.Y * 0.5)), FVector2f((float)(X - 6.0), (float)(Y + TS.Y * 0.5)) };
				FSlateDrawElement::MakeLines(Out, LayerId + 5, Geometry.ToPaintGeometry(), Swatch, ESlateDrawEffect::None, It.Color, true, 1.5f);
				FSlateDrawElement::MakeText(Out, LayerId + 5, Geometry.ToOffsetPaintGeometry(FVector2D(X, Y)), Text, Font, ESlateDrawEffect::None, FLinearColor::White);
				Y += TS.Y + 2.0;
			}
			// Title.
			const FText Title = FText::FromString(UTF8_TO_TCHAR(Airfoil->name.c_str()));
			const FVector2D TS = Measure->Measure(Title, Font);
			FSlateDrawElement::MakeText(Out, LayerId + 5, Geometry.ToOffsetPaintGeometry(FVector2D((Size.X - TS.X) * 0.5, 3.0)), Title, Font, ESlateDrawEffect::None, kTitle);
		}
	}
	catch (const std::exception&)
	{
		const FText Text = FText::FromString(TEXT("Preview unavailable"));
		const FVector2D TS = Measure->Measure(Text, Font);
		FSlateDrawElement::MakeText(Out, LayerId + 1, Geometry.ToOffsetPaintGeometry((Size - TS) * 0.5), Text, Font, ESlateDrawEffect::None, FLinearColor(0.53f, 0.53f, 0.53f));
	}
	return LayerId + 6;
}
