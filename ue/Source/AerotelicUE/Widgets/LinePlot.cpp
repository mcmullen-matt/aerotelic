#include "LinePlot.h"

#include "Framework/Application/SlateApplication.h"
#include "Rendering/DrawElements.h"
#include "Fonts/FontMeasure.h"
#include "Styling/CoreStyle.h"

namespace
{
	// The airfoil preview's palette, so the charts read as one family.
	const FLinearColor kBg(FColor::FromHex(TEXT("#0d1117")));
	const FLinearColor kGrid(FColor::FromHex(TEXT("#2a2f38")));
	const FLinearColor kAxis(0.7f, 0.7f, 0.7f);
	const FLinearColor kZero(0.5f, 0.5f, 0.55f);
	const FLinearColor kText(FColor::FromHex(TEXT("#cccccc")));

	// The step between ticks: 1, 2 or 5 times a power of ten, giving about
	// `target` intervals over `range`.
	double NiceStep(double Range, int32 Target)
	{
		if (!(Range > 0.0)) return 1.0;
		const double Raw = Range / FMath::Max(Target, 1);
		const double Mag = FMath::Pow(10.0, FMath::FloorToDouble(FMath::LogX(10.0, Raw)));
		const double F = Raw / Mag;
		const double Nice = F < 1.5 ? 1.0 : (F < 3.5 ? 2.0 : (F < 7.5 ? 5.0 : 10.0));
		return Nice * Mag;
	}

	FString TickText(double V, double Step)
	{
		// Enough decimals for the step, none more.
		int32 Decimals = 0;
		if (Step < 1.0) Decimals = FMath::Clamp(static_cast<int32>(FMath::CeilToDouble(-FMath::LogX(10.0, Step) - 1e-9)), 0, 6);
		FString S;
		switch (Decimals)
		{
		case 0: S = FString::Printf(TEXT("%.0f"), V); break;
		case 1: S = FString::Printf(TEXT("%.1f"), V); break;
		case 2: S = FString::Printf(TEXT("%.2f"), V); break;
		case 3: S = FString::Printf(TEXT("%.3f"), V); break;
		case 4: S = FString::Printf(TEXT("%.4f"), V); break;
		case 5: S = FString::Printf(TEXT("%.5f"), V); break;
		default: S = FString::Printf(TEXT("%.6f"), V); break;
		}
		if (S.StartsWith(TEXT("-")) && FMath::Abs(V) < Step * 1e-6) S = S.Mid(1);
		return S;
	}
}

void SAerotelicLinePlot::Construct(const FArguments& InArgs)
{
	Title = InArgs._Title;
	XLabel = InArgs._XLabel;
	YLabel = InArgs._YLabel;
	DesiredSize = InArgs._DesiredSize;
	bCompact = InArgs._Compact;
}

void SAerotelicLinePlot::SetSeries(TArray<FAerotelicPlotSeries> InSeries)
{
	Series = MoveTemp(InSeries);
	Invalidate(EInvalidateWidget::Paint);
}

void SAerotelicLinePlot::SetTitle(const FText& InTitle)
{
	Title = InTitle;
	Invalidate(EInvalidateWidget::Paint);
}

void SAerotelicLinePlot::SetAxisLabels(const FText& InX, const FText& InY, const FText& InYRight)
{
	XLabel = InX;
	YLabel = InY;
	YRightLabel = InYRight;
	Invalidate(EInvalidateWidget::Paint);
}

void SAerotelicLinePlot::SetMessage(const FText& InMessage)
{
	Message = InMessage;
	Invalidate(EInvalidateWidget::Paint);
}

int32 SAerotelicLinePlot::OnPaint(const FPaintArgs& Args, const FGeometry& Geometry, const FSlateRect& MyCullingRect,
	FSlateWindowElementList& Out, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const
{
	const FVector2D Size = Geometry.GetLocalSize();
	const FSlateBrush* White = FCoreStyle::Get().GetBrush("WhiteBrush");
	FSlateDrawElement::MakeBox(Out, LayerId, Geometry.ToPaintGeometry(), White, ESlateDrawEffect::None, kBg);

	const FSlateFontInfo Font = FCoreStyle::GetDefaultFontStyle("Regular", bCompact ? 7 : 8);
	const FSlateFontInfo TitleFont = FCoreStyle::GetDefaultFontStyle("Bold", bCompact ? 8 : 10);
	const TSharedRef<FSlateFontMeasure> Measure = FSlateApplication::Get().GetRenderer()->GetFontMeasureService();
	const double LineH = Measure->Measure(FText::FromString(TEXT("0")), Font).Y;

	// Data range over every point: x over all series, y per axis.
	FBox2D Range(ForceInit), RangeRight(ForceInit);
	int32 NPoints = 0;
	bool bRight = false;
	for (const FAerotelicPlotSeries& S : Series)
	{
		for (const FVector2D& P : S.Points)
		{
			if (FMath::IsFinite(P.X) && FMath::IsFinite(P.Y))
			{
				(S.bRightAxis ? RangeRight : Range) += P;
				++NPoints;
			}
		}
		bRight = bRight || (S.bRightAxis && S.Points.Num() > 0);
	}
	if (bRight && !Range.bIsValid)
	{
		// Only right-axis data: draw it against the left axis after all.
		Range = RangeRight;
		bRight = false;
	}
	else if (bRight)
	{
		Range.Min.X = FMath::Min(Range.Min.X, RangeRight.Min.X);
		Range.Max.X = FMath::Max(Range.Max.X, RangeRight.Max.X);
	}
	auto DrawCentred = [&](const FText& T, const FSlateFontInfo& F, double Y, const FLinearColor& C)
	{
		const FVector2D TS = Measure->Measure(T, F);
		FSlateDrawElement::MakeText(Out, LayerId + 6, Geometry.ToOffsetPaintGeometry(FVector2D((Size.X - TS.X) * 0.5, Y)), T, F, ESlateDrawEffect::None, C);
	};
	if (!Title.IsEmpty())
	{
		DrawCentred(Title, TitleFont, 3.0, kText);
	}
	if (NPoints == 0)
	{
		DrawCentred(Message.IsEmpty() ? FText::FromString(TEXT("No data")) : Message, Font, Size.Y * 0.5 - LineH * 0.5, FLinearColor(0.53f, 0.53f, 0.53f));
		return LayerId + 7;
	}
	// Pad a y range; a flat series gets a band about its value; zero is
	// pulled in when it is close, so a coefficient reads against zero;
	// then the range snaps out to its ticks.
	const int32 TargetTicks = bCompact ? 4 : 6;
	auto NiceRange = [&](double& Y0, double& Y1, double& YStep)
	{
		if (Y1 - Y0 < 1e-12) { const double D = FMath::Max(FMath::Abs(Y0) * 0.1, 0.1); Y0 -= D; Y1 += D; }
		const double Pad = (Y1 - Y0) * 0.08;
		Y0 -= Pad; Y1 += Pad;
		if (Y0 > 0.0 && Y0 < (Y1 - Y0) * 0.5) Y0 = 0.0;
		if (Y1 < 0.0 && -Y1 < (Y1 - Y0) * 0.5) Y1 = 0.0;
		YStep = NiceStep(Y1 - Y0, TargetTicks);
		Y0 = FMath::FloorToDouble(Y0 / YStep) * YStep;
		Y1 = FMath::CeilToDouble(Y1 / YStep) * YStep;
	};
	double X0 = Range.Min.X, X1 = Range.Max.X, Y0 = Range.Min.Y, Y1 = Range.Max.Y, YStep = 1.0;
	if (X1 - X0 < 1e-12) { const double D = FMath::Max(FMath::Abs(X0) * 0.1, 0.5); X0 -= D; X1 += D; }
	const double XStep = NiceStep(X1 - X0, TargetTicks);
	double R0 = 0.0, R1 = 1.0, RStep = 1.0;
	if (bRight)
	{
		// Two axes share their zero: both ranges take the same proportion
		// below zero -- the larger one either axis's data demands -- so a
		// value of zero sits on one horizontal line whichever axis it is
		// read on, and each axis stretches only as far as its own data
		// needs (plus the usual pad). The ranges are not snapped to their
		// ticks (that would move the zeros apart); the ticks fall on nice
		// values inside them instead.
		auto Split = [](const FBox2D& B, double& Neg, double& Pos) { Neg = FMath::Max(0.0, -B.Min.Y); Pos = FMath::Max(0.0, B.Max.Y); };
		double NegL, PosL, NegR, PosR;
		Split(Range, NegL, PosL);
		Split(RangeRight, NegR, PosR);
		auto Share = [](double Neg, double Pos) { return Neg + Pos > 1e-300 ? Neg / (Neg + Pos) : 0.0; };
		const double F = FMath::Max(Share(NegL, PosL), Share(NegR, PosR));
		auto Extent = [F](double Neg, double Pos)
		{
			double D = 0.0;
			if (F < 1.0) D = FMath::Max(D, Pos / (1.0 - F));
			if (F > 0.0) D = FMath::Max(D, Neg / F);
			if (D < 1e-12) D = FMath::Max(FMath::Max(Neg, Pos) * 0.2, 0.1);
			return D * 1.08;
		};
		const double DL = Extent(NegL, PosL), DR = Extent(NegR, PosR);
		Y0 = -F * DL; Y1 = (1.0 - F) * DL;
		R0 = -F * DR; R1 = (1.0 - F) * DR;
		YStep = NiceStep(Y1 - Y0, TargetTicks);
		RStep = NiceStep(R1 - R0, TargetTicks);
	}
	else
	{
		NiceRange(Y0, Y1, YStep);
	}
	// The first tick at or above the bottom of each range (the bottom
	// itself when the range is snapped to its ticks).
	const double YTick0 = FMath::CeilToDouble(Y0 / YStep - 1e-9) * YStep;
	const double RTick0 = FMath::CeilToDouble(R0 / RStep - 1e-9) * RStep;

	// Margins: y tick labels on the left (and the right), x labels below,
	// title above.
	double LeftW = 0.0, RightW = 0.0;
	for (double V = YTick0; V <= Y1 + YStep * 1e-6; V += YStep) LeftW = FMath::Max(LeftW, Measure->Measure(FText::FromString(TickText(V, YStep)), Font).X);
	if (bRight) for (double V = RTick0; V <= R1 + RStep * 1e-6; V += RStep) RightW = FMath::Max(RightW, Measure->Measure(FText::FromString(TickText(V, RStep)), Font).X);
	const double Left = LeftW + 10.0;
	const double Right = bRight ? RightW + 10.0 : 10.0;
	const double Top = (Title.IsEmpty() ? 6.0 : LineH + 10.0) + ((YLabel.IsEmpty() && (!bRight || YRightLabel.IsEmpty())) ? 0.0 : LineH + 2.0);
	const double Bottom = LineH + 6.0 + (XLabel.IsEmpty() ? 0.0 : LineH + 2.0);
	const double PlotW = FMath::Max(Size.X - Left - Right, 10.0), PlotH = FMath::Max(Size.Y - Top - Bottom, 10.0);
	auto Map = [&](double X, double Y)
	{
		return FVector2f(static_cast<float>(Left + (X - X0) / (X1 - X0) * PlotW), static_cast<float>(Top + (Y1 - Y) / (Y1 - Y0) * PlotH));
	};
	auto MapRight = [&](double X, double Y)
	{
		return FVector2f(static_cast<float>(Left + (X - X0) / (X1 - X0) * PlotW), static_cast<float>(Top + (R1 - Y) / (R1 - R0) * PlotH));
	};

	// Grid and tick labels.
	for (double V = FMath::CeilToDouble(X0 / XStep) * XStep; V <= X1 + XStep * 1e-6; V += XStep)
	{
		TArray<FVector2f> L = { Map(V, Y0), Map(V, Y1) };
		FSlateDrawElement::MakeLines(Out, LayerId + 1, Geometry.ToPaintGeometry(), L, ESlateDrawEffect::None, kGrid, false, 1.0f);
		const FText T = FText::FromString(TickText(V, XStep));
		const FVector2D TS = Measure->Measure(T, Font);
		FSlateDrawElement::MakeText(Out, LayerId + 5, Geometry.ToOffsetPaintGeometry(FVector2D(Map(V, Y0).X - TS.X * 0.5, Top + PlotH + 3.0)), T, Font, ESlateDrawEffect::None, kText);
	}
	for (double V = YTick0; V <= Y1 + YStep * 1e-6; V += YStep)
	{
		TArray<FVector2f> L = { Map(X0, V), Map(X1, V) };
		FSlateDrawElement::MakeLines(Out, LayerId + 1, Geometry.ToPaintGeometry(), L, ESlateDrawEffect::None, kGrid, false, 1.0f);
		const FText T = FText::FromString(TickText(V, YStep));
		const FVector2D TS = Measure->Measure(T, Font);
		FSlateDrawElement::MakeText(Out, LayerId + 5, Geometry.ToOffsetPaintGeometry(FVector2D(Left - 6.0 - TS.X, Map(X0, V).Y - TS.Y * 0.5)), T, Font, ESlateDrawEffect::None, kText);
	}
	// The right axis: its own ticks down the right edge, no grid of its own.
	if (bRight)
	{
		for (double V = RTick0; V <= R1 + RStep * 1e-6; V += RStep)
		{
			const FText T = FText::FromString(TickText(V, RStep));
			const FVector2D TS = Measure->Measure(T, Font);
			const FVector2f P = MapRight(X1, V);
			TArray<FVector2f> Tick = { P, FVector2f(P.X + 4.0f, P.Y) };
			FSlateDrawElement::MakeLines(Out, LayerId + 2, Geometry.ToPaintGeometry(), Tick, ESlateDrawEffect::None, kAxis, false, 1.0f);
			FSlateDrawElement::MakeText(Out, LayerId + 5, Geometry.ToOffsetPaintGeometry(FVector2D(Left + PlotW + 6.0, P.Y - TS.Y * 0.5)), T, Font, ESlateDrawEffect::None, kText);
		}
	}
	// Zero lines and the frame.
	if (Y0 < 0.0 && Y1 > 0.0)
	{
		TArray<FVector2f> L = { Map(X0, 0.0), Map(X1, 0.0) };
		FSlateDrawElement::MakeLines(Out, LayerId + 2, Geometry.ToPaintGeometry(), L, ESlateDrawEffect::None, kZero, false, 1.0f);
	}
	if (X0 < 0.0 && X1 > 0.0)
	{
		TArray<FVector2f> L = { Map(0.0, Y0), Map(0.0, Y1) };
		FSlateDrawElement::MakeLines(Out, LayerId + 2, Geometry.ToPaintGeometry(), L, ESlateDrawEffect::None, kZero, false, 1.0f);
	}
	{
		TArray<FVector2f> Frame = { Map(X0, Y0), Map(X1, Y0), Map(X1, Y1), Map(X0, Y1), Map(X0, Y0) };
		FSlateDrawElement::MakeLines(Out, LayerId + 2, Geometry.ToPaintGeometry(), Frame, ESlateDrawEffect::None, kAxis, false, 1.0f);
	}
	// Axis labels.
	if (!XLabel.IsEmpty())
	{
		const FVector2D TS = Measure->Measure(XLabel, Font);
		FSlateDrawElement::MakeText(Out, LayerId + 5, Geometry.ToOffsetPaintGeometry(FVector2D(Left + (PlotW - TS.X) * 0.5, Size.Y - TS.Y - 2.0)), XLabel, Font, ESlateDrawEffect::None, kText);
	}
	if (!YLabel.IsEmpty())
	{
		FSlateDrawElement::MakeText(Out, LayerId + 5, Geometry.ToOffsetPaintGeometry(FVector2D(Left, Top - LineH - 2.0)), YLabel, Font, ESlateDrawEffect::None, kText);
	}
	if (bRight && !YRightLabel.IsEmpty())
	{
		const FVector2D TS = Measure->Measure(YRightLabel, Font);
		FSlateDrawElement::MakeText(Out, LayerId + 5, Geometry.ToOffsetPaintGeometry(FVector2D(Left + PlotW + Right - TS.X, Top - LineH - 2.0)), YRightLabel, Font, ESlateDrawEffect::None, kText);
	}

	// The series, clipped to the plot area.
	Out.PushClip(FSlateClippingZone(Geometry.ToPaintGeometry(FVector2f((float)PlotW, (float)PlotH), FSlateLayoutTransform(FVector2f((float)Left, (float)Top)))));
	for (const FAerotelicPlotSeries& S : Series)
	{
		TArray<FVector2f> Pts;
		const bool bOnRight = bRight && S.bRightAxis;
		for (const FVector2D& P : S.Points)
		{
			if (FMath::IsFinite(P.X) && FMath::IsFinite(P.Y)) Pts.Add(bOnRight ? MapRight(P.X, P.Y) : Map(P.X, P.Y));
		}
		if (S.bLine && Pts.Num() >= 2)
		{
			if (S.bDashed)
			{
				// Dashes of 6 px with 4 px gaps along the polyline.
				double Carry = 0.0;
				bool bOn = true;
				for (int32 i = 0; i + 1 < Pts.Num(); ++i)
				{
					const FVector2f A = Pts[i], B = Pts[i + 1];
					const double Len = FVector2f::Distance(A, B);
					double T = 0.0;
					while (T < Len)
					{
						const double Seg = FMath::Min((bOn ? 6.0 : 4.0) - Carry, Len - T);
						if (bOn)
						{
							TArray<FVector2f> D = { A + (B - A) * (float)(T / Len), A + (B - A) * (float)((T + Seg) / Len) };
							FSlateDrawElement::MakeLines(Out, LayerId + 3, Geometry.ToPaintGeometry(), D, ESlateDrawEffect::None, S.Color, true, S.LineWidth);
						}
						T += Seg;
						Carry += Seg;
						if (Carry >= (bOn ? 6.0 : 4.0) - 1e-9) { Carry = 0.0; bOn = !bOn; }
					}
				}
			}
			else
			{
				FSlateDrawElement::MakeLines(Out, LayerId + 3, Geometry.ToPaintGeometry(), Pts, ESlateDrawEffect::None, S.Color, true, S.LineWidth);
			}
		}
		if (S.bMarkers)
		{
			const float M = bCompact ? 4.0f : 5.0f;
			for (const FVector2f& P : Pts)
			{
				FSlateDrawElement::MakeBox(Out, LayerId + 4, Geometry.ToPaintGeometry(FVector2f(M, M), FSlateLayoutTransform(FVector2f(P.X - M * 0.5f, P.Y - M * 0.5f))), White, ESlateDrawEffect::None, S.Color);
			}
		}
	}
	Out.PopClip();

	// Legend, top-right of the plot area, when there is more than one series.
	if (Series.Num() > 1)
	{
		double Y = Top + 4.0;
		for (const FAerotelicPlotSeries& S : Series)
		{
			if (S.Label.IsEmpty()) continue;
			const FText T = FText::FromString(S.Label);
			const FVector2D TS = Measure->Measure(T, Font);
			const double X = Left + PlotW - 8.0 - TS.X;
			TArray<FVector2f> Swatch = { FVector2f((float)(X - 22.0), (float)(Y + TS.Y * 0.5)), FVector2f((float)(X - 6.0), (float)(Y + TS.Y * 0.5)) };
			FSlateDrawElement::MakeLines(Out, LayerId + 6, Geometry.ToPaintGeometry(), Swatch, ESlateDrawEffect::None, S.Color, true, S.LineWidth);
			FSlateDrawElement::MakeText(Out, LayerId + 6, Geometry.ToOffsetPaintGeometry(FVector2D(X, Y)), T, Font, ESlateDrawEffect::None, FLinearColor::White);
			Y += TS.Y + 2.0;
		}
	}
	return LayerId + 7;
}
