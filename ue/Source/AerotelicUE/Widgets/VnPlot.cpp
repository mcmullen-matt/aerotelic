#include "VnPlot.h"

#include "Framework/Application/SlateApplication.h"
#include "Rendering/DrawElements.h"
#include "Fonts/FontMeasure.h"
#include "Styling/CoreStyle.h"

namespace
{
	const FLinearColor kBg(FColor::FromHex(TEXT("#0d1117")));
	const FLinearColor kGrid(FColor::FromHex(TEXT("#2a2f38")));
	const FLinearColor kAxis(0.7f, 0.7f, 0.7f);
	const FLinearColor kZero(0.5f, 0.5f, 0.55f);
	const FLinearColor kText(FColor::FromHex(TEXT("#cccccc")));
	const FLinearColor kEnvelope(FColor::FromHex(TEXT("#3a7bd5")));
	const FLinearColor kEnvelopeFill(0.23f, 0.48f, 0.84f, 0.12f);
	const FLinearColor kStall(FColor::FromHex(TEXT("#d56b3a")));
	const FLinearColor kPoint(FColor::FromHex(TEXT("#c8d53a")));
	const FLinearColor kPointOutside(FColor::FromHex(TEXT("#d53a7b")));
	const FLinearColor kSelected(FColor::FromHex(TEXT("#ffffff")));
	constexpr double kPickPixels = 10.0;

	double NiceStep(double Range, int32 Target)
	{
		if (!(Range > 0.0)) return 1.0;
		const double Raw = Range / FMath::Max(Target, 1);
		const double Mag = FMath::Pow(10.0, FMath::FloorToDouble(FMath::LogX(10.0, Raw)));
		const double F = Raw / Mag;
		const double Nice = F < 1.5 ? 1.0 : (F < 3.5 ? 2.0 : (F < 7.5 ? 5.0 : 10.0));
		return Nice * Mag;
	}

	FString Fmt(double V)
	{
		if (FMath::Abs(V - FMath::RoundToDouble(V)) < 1e-9) return FString::Printf(TEXT("%.0f"), V);
		return FString::Printf(TEXT("%.2g"), V);
	}
}

void SAerotelicVnPlot::Construct(const FArguments& InArgs)
{
	DesiredSize = InArgs._DesiredSize;
}

void SAerotelicVnPlot::SetEnvelope(const aerotelic::VnDiagramSpec& InSpec, double InWeight, double InArea)
{
	Spec = InSpec;
	Weight = InWeight;
	Area = InArea;
	Invalidate(EInvalidateWidget::Paint);
}

void SAerotelicVnPlot::SetPoints(const TArray<FAerotelicVnPoint>& InPoints)
{
	Points = InPoints;
	if (Selected >= Points.Num()) Selected = -1;
	Invalidate(EInvalidateWidget::Paint);
}

void SAerotelicVnPlot::SetSelected(int32 Index)
{
	Selected = Points.IsValidIndex(Index) ? Index : -1;
	Invalidate(EInvalidateWidget::Paint);
}

double SAerotelicVnPlot::StallSpeed() const { return aerotelic::VnDiagramSpec::StallSpeed(Spec.cl_max, 1.0, Spec.density, Weight, Area); }
double SAerotelicVnPlot::ManoeuvreSpeed() const { return aerotelic::VnDiagramSpec::StallSpeed(Spec.cl_max, Spec.n_max, Spec.density, Weight, Area); }
double SAerotelicVnPlot::NegativeStallSpeed() const { return aerotelic::VnDiagramSpec::StallSpeed(Spec.cl_min, -1.0, Spec.density, Weight, Area); }
double SAerotelicVnPlot::NegativeCornerSpeed() const { return aerotelic::VnDiagramSpec::StallSpeed(Spec.cl_min, Spec.n_min, Spec.density, Weight, Area); }

bool SAerotelicVnPlot::Inside(double V, double N) const
{
	if (V < 0.0 || V > Spec.v_ne) return false;
	if (N > Spec.n_max || N < Spec.n_min) return false;
	if (Weight > 0.0 && Area > 0.0)
	{
		if (N > StallN(Spec.cl_max, V)) return false;
		if (N < StallN(Spec.cl_min, V)) return false;
	}
	return true;
}

FVector2D SAerotelicVnPlot::FFrame::ToPixel(double V, double N) const
{
	const double X = Origin.X + (V - VMin) / FMath::Max(VMax - VMin, 1e-12) * Size.X;
	const double Y = Origin.Y + Size.Y - (N - NMin) / FMath::Max(NMax - NMin, 1e-12) * Size.Y;
	return FVector2D(X, Y);
}

void SAerotelicVnPlot::FFrame::FromPixel(const FVector2D& P, double& V, double& N) const
{
	V = VMin + (P.X - Origin.X) / FMath::Max(Size.X, 1.0) * (VMax - VMin);
	N = NMin + (Origin.Y + Size.Y - P.Y) / FMath::Max(Size.Y, 1.0) * (NMax - NMin);
}

SAerotelicVnPlot::FFrame SAerotelicVnPlot::Frame(const FVector2D& LocalSize) const
{
	FFrame F;
	F.VMin = 0.0;
	F.VMax = FMath::Max(Spec.v_ne * 1.12, 1.0);
	for (const FAerotelicVnPoint& P : Points) F.VMax = FMath::Max(F.VMax, P.Speed * 1.08);
	F.NMax = FMath::Max(Spec.n_max, 1.0) + 0.6;
	F.NMin = FMath::Min(Spec.n_min, -1.0) - 0.6;
	for (const FAerotelicVnPoint& P : Points) { F.NMax = FMath::Max(F.NMax, P.LoadFactor + 0.4); F.NMin = FMath::Min(F.NMin, P.LoadFactor - 0.4); }
	const double Left = 52.0, Right = 14.0, Top = 14.0, Bottom = 34.0;
	F.Origin = FVector2D(Left, Top);
	F.Size = FVector2D(FMath::Max(LocalSize.X - Left - Right, 10.0), FMath::Max(LocalSize.Y - Top - Bottom, 10.0));
	return F;
}

int32 SAerotelicVnPlot::PointAt(const FGeometry& Geometry, const FVector2D& Pixel) const
{
	const FFrame F = Frame(Geometry.GetLocalSize());
	int32 Best = -1;
	double BestD = kPickPixels;
	for (int32 i = 0; i < Points.Num(); ++i)
	{
		const double D = FVector2D::Distance(F.ToPixel(Points[i].Speed, Points[i].LoadFactor), Pixel);
		if (D < BestD) { BestD = D; Best = i; }
	}
	return Best;
}

int32 SAerotelicVnPlot::OnPaint(const FPaintArgs& Args, const FGeometry& Geometry, const FSlateRect& MyCullingRect,
	FSlateWindowElementList& Out, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const
{
	const FVector2D Size = Geometry.GetLocalSize();
	const FSlateBrush* White = FCoreStyle::Get().GetBrush("WhiteBrush");
	FSlateDrawElement::MakeBox(Out, LayerId, Geometry.ToPaintGeometry(), White, ESlateDrawEffect::None, kBg);
	const FFrame F = Frame(Size);
	const FSlateFontInfo Font = FCoreStyle::GetDefaultFontStyle("Regular", 8);
	const FSlateFontInfo Bold = FCoreStyle::GetDefaultFontStyle("Bold", 8);
	const TSharedRef<FSlateFontMeasure> Measure = FSlateApplication::Get().GetRenderer()->GetFontMeasureService();
	auto Line = [&](const TArray<FVector2D>& Pts, const FLinearColor& C, float W, int32 Layer)
	{
		if (Pts.Num() >= 2) FSlateDrawElement::MakeLines(Out, Layer, Geometry.ToPaintGeometry(), Pts, ESlateDrawEffect::None, C, true, W);
	};
	auto Text = [&](const FString& S, const FVector2D& At, const FLinearColor& C, const FSlateFontInfo& Fi, int32 Layer)
	{
		FSlateDrawElement::MakeText(Out, Layer, Geometry.ToPaintGeometry(FVector2f(1.0f, 1.0f), FSlateLayoutTransform(FVector2f(At))), S, Fi, ESlateDrawEffect::None, C);
	};

	// Grid and ticks.
	const double VStep = NiceStep(F.VMax - F.VMin, 8), NStep = NiceStep(F.NMax - F.NMin, 7);
	for (double V = 0.0; V <= F.VMax + 1e-9; V += VStep)
	{
		const FVector2D A = F.ToPixel(V, F.NMin), B = F.ToPixel(V, F.NMax);
		Line({ A, B }, kGrid, 1.0f, LayerId + 1);
		const FString S = Fmt(V);
		const FVector2D W = Measure->Measure(S, Font);
		Text(S, FVector2D(A.X - W.X * 0.5, A.Y + 4), kText, Font, LayerId + 6);
	}
	for (double N = FMath::CeilToDouble(F.NMin / NStep) * NStep; N <= F.NMax + 1e-9; N += NStep)
	{
		const FVector2D A = F.ToPixel(F.VMin, N), B = F.ToPixel(F.VMax, N);
		Line({ A, B }, FMath::Abs(N) < 1e-9 ? kZero : kGrid, 1.0f, LayerId + 1);
		const FString S = Fmt(N);
		const FVector2D W = Measure->Measure(S, Font);
		Text(S, FVector2D(A.X - W.X - 6, A.Y - W.Y * 0.5), kText, Font, LayerId + 6);
	}
	Line({ F.ToPixel(F.VMin, F.NMin), F.ToPixel(F.VMax, F.NMin) }, kAxis, 1.0f, LayerId + 2);
	Line({ F.ToPixel(F.VMin, F.NMin), F.ToPixel(F.VMin, F.NMax) }, kAxis, 1.0f, LayerId + 2);
	Text(TEXT("V, m/s"), FVector2D(F.Origin.X + F.Size.X - 40, F.Origin.Y + F.Size.Y + 18), kText, Font, LayerId + 6);
	Text(TEXT("n"), FVector2D(6, F.Origin.Y), kText, Bold, LayerId + 6);

	// The envelope.
	const bool bStall = Weight > 0.0 && Area > 0.0;
	const double VA = bStall ? FMath::Min(ManoeuvreSpeed(), Spec.v_ne) : 0.0;
	const double VNegCorner = bStall ? FMath::Min(NegativeCornerSpeed(), Spec.v_ne) : 0.0;
	TArray<FVector2D> Envelope;
	// The positive stall boundary from the origin to V_A (or to V_ne where it never reaches n_max).
	constexpr int32 Samples = 48;
	if (bStall)
	{
		const double VEnd = VA > 0.0 ? VA : Spec.v_ne;
		for (int32 i = 0; i <= Samples; ++i)
		{
			const double V = VEnd * i / Samples;
			Envelope.Add(F.ToPixel(V, FMath::Min(StallN(Spec.cl_max, V), Spec.n_max)));
		}
	}
	else
	{
		Envelope.Add(F.ToPixel(0.0, 0.0));
		Envelope.Add(F.ToPixel(0.0, Spec.n_max));
	}
	Envelope.Add(F.ToPixel(Spec.v_ne, Spec.n_max));
	Envelope.Add(F.ToPixel(Spec.v_ne, Spec.n_min));
	if (bStall)
	{
		const double VEnd = VNegCorner > 0.0 ? VNegCorner : Spec.v_ne;
		for (int32 i = Samples; i >= 0; --i)
		{
			const double V = VEnd * i / Samples;
			Envelope.Add(F.ToPixel(V, FMath::Max(StallN(Spec.cl_min, V), Spec.n_min)));
		}
	}
	else
	{
		Envelope.Add(F.ToPixel(0.0, Spec.n_min));
		Envelope.Add(F.ToPixel(0.0, 0.0));
	}
	// A translucent fill: fans from the origin pixel over the outline.
	{
		const FVector2D O = F.ToPixel(0.0, 0.0);
		TArray<FSlateVertex> Verts;
		TArray<SlateIndex> Indices;
		const FSlateRenderTransform& RT = Geometry.ToPaintGeometry().GetAccumulatedRenderTransform();
		const FColor Fill = kEnvelopeFill.ToFColor(false);
		auto Vtx = [&](const FVector2D& P) { Verts.Add(FSlateVertex::Make<ESlateVertexRounding::Disabled>(RT, FVector2f(P), FVector2f(0.0f, 0.0f), Fill)); };
		Vtx(O);
		for (const FVector2D& P : Envelope) Vtx(P);
		for (int32 i = 1; i + 1 < Verts.Num(); ++i) { Indices.Add(0); Indices.Add(i); Indices.Add(i + 1); }
		if (Indices.Num() >= 3) FSlateDrawElement::MakeCustomVerts(Out, LayerId + 3, White->GetRenderingResource(), Verts, Indices, nullptr, 0, 0);
	}
	Line(Envelope, kEnvelope, 2.0f, LayerId + 4);
	// The stall boundaries beyond the limits, dashed-looking (thin, dim), and the marks.
	if (bStall)
	{
		TArray<FVector2D> Pos, Neg;
		for (int32 i = 0; i <= Samples; ++i)
		{
			const double V = F.VMax * i / Samples;
			const double Np = StallN(Spec.cl_max, V), Nn = StallN(Spec.cl_min, V);
			if (Np <= F.NMax) Pos.Add(F.ToPixel(V, Np));
			if (Nn >= F.NMin) Neg.Add(F.ToPixel(V, Nn));
		}
		Line(Pos, kStall * FLinearColor(1, 1, 1, 0.5f), 1.0f, LayerId + 3);
		Line(Neg, kStall * FLinearColor(1, 1, 1, 0.5f), 1.0f, LayerId + 3);
		const double VS = StallSpeed();
		if (VS > 0.0 && VS < F.VMax)
		{
			Line({ F.ToPixel(VS, F.NMin), F.ToPixel(VS, 1.0) }, kStall * FLinearColor(1, 1, 1, 0.6f), 1.0f, LayerId + 3);
			Text(FString::Printf(TEXT("V_S %.1f"), VS), F.ToPixel(VS, F.NMin) + FVector2D(3, -14), kStall, Font, LayerId + 6);
		}
		if (VA > 0.0 && VA < Spec.v_ne)
		{
			Line({ F.ToPixel(VA, F.NMin), F.ToPixel(VA, Spec.n_max) }, kEnvelope * FLinearColor(1, 1, 1, 0.5f), 1.0f, LayerId + 3);
			Text(FString::Printf(TEXT("V_A %.1f"), VA), F.ToPixel(VA, Spec.n_max) + FVector2D(3, -14), kEnvelope, Font, LayerId + 6);
		}
	}
	Text(FString::Printf(TEXT("V_ne %.1f"), Spec.v_ne), F.ToPixel(Spec.v_ne, F.NMax) + FVector2D(3, 2), kEnvelope, Font, LayerId + 6);
	Text(FString::Printf(TEXT("n_max %g"), Spec.n_max), F.ToPixel(F.VMin, Spec.n_max) + FVector2D(4, -13), kEnvelope, Font, LayerId + 6);
	Text(FString::Printf(TEXT("n_min %g"), Spec.n_min), F.ToPixel(F.VMin, Spec.n_min) + FVector2D(4, 2), kEnvelope, Font, LayerId + 6);

	// The points.
	for (int32 i = 0; i < Points.Num(); ++i)
	{
		const FAerotelicVnPoint& P = Points[i];
		const FVector2D C = F.ToPixel(P.Speed, P.LoadFactor);
		const bool bIn = Inside(P.Speed, P.LoadFactor);
		const FLinearColor Col = i == Selected ? kSelected : (bIn ? kPoint : kPointOutside);
		const double R = (i == Selected || i == Hover) ? 7.0 : 5.0;
		FSlateDrawElement::MakeBox(Out, LayerId + 7, Geometry.ToPaintGeometry(FVector2f(2 * R, 2 * R), FSlateLayoutTransform(FVector2f(C - FVector2D(R, R)))), White, ESlateDrawEffect::None, Col);
		if (!P.bSolved)
		{
			FSlateDrawElement::MakeBox(Out, LayerId + 8, Geometry.ToPaintGeometry(FVector2f(2 * R - 4, 2 * R - 4), FSlateLayoutTransform(FVector2f(C - FVector2D(R - 2, R - 2)))), White, ESlateDrawEffect::None, kBg);
		}
		Text(P.Name, C + FVector2D(R + 3, -R - 2), i == Selected ? kSelected : kText, i == Selected ? Bold : Font, LayerId + 9);
	}
	if (!bStall)
	{
		Text(TEXT("No weight or area: the stall boundaries need both (the mass report of the chosen bodies, or typed)"), FVector2D(F.Origin.X + 10, F.Origin.Y + 10), kStall, Font, LayerId + 9);
	}
	return LayerId + 10;
}

FReply SAerotelicVnPlot::OnMouseButtonDown(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent)
{
	if (MouseEvent.GetEffectingButton() != EKeys::LeftMouseButton) return FReply::Unhandled();
	const FVector2D Pixel = MyGeometry.AbsoluteToLocal(MouseEvent.GetScreenSpacePosition());
	const int32 Hit = PointAt(MyGeometry, Pixel);
	if (Hit >= 0)
	{
		Selected = Hit;
		Dragging = Hit;
		if (OnSelectPoint) OnSelectPoint(Hit);
		Invalidate(EInvalidateWidget::Paint);
		return FReply::Handled().CaptureMouse(SharedThis(this));
	}
	const FFrame F = Frame(MyGeometry.GetLocalSize());
	if (Pixel.X < F.Origin.X || Pixel.Y < F.Origin.Y || Pixel.X > F.Origin.X + F.Size.X || Pixel.Y > F.Origin.Y + F.Size.Y) return FReply::Unhandled();
	double V, N;
	F.FromPixel(Pixel, V, N);
	if (OnAddPoint) OnAddPoint(FMath::Max(V, 0.0), N);
	return FReply::Handled();
}

FReply SAerotelicVnPlot::OnMouseButtonUp(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent)
{
	if (Dragging >= 0)
	{
		const int32 Was = Dragging;
		Dragging = -1;
		if (OnMoveEnd) OnMoveEnd(Was);
		return FReply::Handled().ReleaseMouseCapture();
	}
	return FReply::Unhandled();
}

FReply SAerotelicVnPlot::OnMouseMove(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent)
{
	const FVector2D Pixel = MyGeometry.AbsoluteToLocal(MouseEvent.GetScreenSpacePosition());
	if (Dragging >= 0 && Points.IsValidIndex(Dragging))
	{
		const FFrame F = Frame(MyGeometry.GetLocalSize());
		double V, N;
		F.FromPixel(Pixel, V, N);
		V = FMath::Max(V, 0.0);
		Points[Dragging].Speed = V;
		Points[Dragging].LoadFactor = N;
		if (OnMovePoint) OnMovePoint(Dragging, V, N);
		Invalidate(EInvalidateWidget::Paint);
		return FReply::Handled();
	}
	const int32 H = PointAt(MyGeometry, Pixel);
	if (H != Hover)
	{
		Hover = H;
		Invalidate(EInvalidateWidget::Paint);
	}
	return FReply::Unhandled();
}

void SAerotelicVnPlot::OnMouseLeave(const FPointerEvent& MouseEvent)
{
	if (Hover != -1) { Hover = -1; Invalidate(EInvalidateWidget::Paint); }
}

FCursorReply SAerotelicVnPlot::OnCursorQuery(const FGeometry& MyGeometry, const FPointerEvent& CursorEvent) const
{
	return FCursorReply::Cursor(Hover >= 0 || Dragging >= 0 ? EMouseCursor::GrabHand : EMouseCursor::Crosshairs);
}
