#include "NavCube.h"

#include "Framework/Application/SlateApplication.h"
#include "Rendering/DrawElements.h"
#include "Rendering/SlateRenderer.h"
#include "Fonts/FontMeasure.h"
#include "Styling/CoreStyle.h"

#include "Standalone/AerotelicCameraPawn.h"

namespace
{
	// AIS_ViewCube settings from view_gizmo.py: size 62 px, parked 85 px
	// in from the top-right corner, face zone 0.72.
	constexpr float kWidgetSize = 150.0f;
	constexpr double kHalfEdgePx = 31.0;
	constexpr double kFaceZone = 0.72;
	const FLinearColor kBoxColor(0.32f, 0.36f, 0.44f);
	const FLinearColor kHoverColor(0.80f, 0.88f, 1.0f);
	const FLinearColor kInnerColor(0.18f, 0.21f, 0.27f);
	const FLinearColor kTextColor(0.92f, 0.93f, 0.95f);

	struct FFace
	{
		int32 Axis;   // 0 = X, 1 = Y, 2 = Z
		int32 Sign;   // +1 / -1
		const TCHAR* Label;
	};
	// Labels follow the application's presets: Front looks from -X at the
	// -X face, Top from +Z, Side from -Y (the left-hand side).
	const FFace kFaces[6] = {
		{ 0, -1, TEXT("FRONT") }, { 0, +1, TEXT("BACK") },
		{ 1, -1, TEXT("LEFT") },  { 1, +1, TEXT("RIGHT") },
		{ 2, +1, TEXT("TOP") },   { 2, -1, TEXT("BOTTOM") },
	};

	FVector Unit(int32 Axis)
	{
		FVector V = FVector::ZeroVector;
		V[Axis] = 1.0;
		return V;
	}

	// Projects a cube-space point orthographically onto the widget: the
	// cube is seen exactly as the camera sees the world.
	FVector2D Project(const FVector& P, const FVector& Right, const FVector& Up, const FVector2D& Centre)
	{
		return Centre + FVector2D(FVector::DotProduct(P, Right) * kHalfEdgePx, -FVector::DotProduct(P, Up) * kHalfEdgePx);
	}

	// Ray / unit-cube slab test; returns the entry point.
	bool RayUnitCube(const FVector& O, const FVector& D, FVector& OutHit)
	{
		double TMin = -1e9, TMax = 1e9;
		for (int32 i = 0; i < 3; ++i)
		{
			if (FMath::Abs(D[i]) < 1e-9)
			{
				if (O[i] < -1.0 || O[i] > 1.0) return false;
				continue;
			}
			double T1 = (-1.0 - O[i]) / D[i];
			double T2 = (1.0 - O[i]) / D[i];
			if (T1 > T2) Swap(T1, T2);
			TMin = FMath::Max(TMin, T1);
			TMax = FMath::Min(TMax, T2);
			if (TMin > TMax) return false;
		}
		if (TMax < 0.0) return false;
		OutHit = O + D * TMin;
		return true;
	}
}

void SAerotelicNavCube::Construct(const FArguments& InArgs)
{
	// Redrawn every frame: it follows the camera.
	ForceVolatile(true);
}

FVector2D SAerotelicNavCube::ComputeDesiredSize(float) const
{
	return FVector2D(kWidgetSize, kWidgetSize);
}

bool SAerotelicNavCube::GetBasis(FVector& Forward, FVector& Right, FVector& Up) const
{
	AAerotelicCameraPawn* Camera = AAerotelicCameraPawn::FindInGameWorld();
	if (Camera == nullptr)
	{
		return false;
	}
	Camera->GetViewBasis(Forward, Right, Up);
	return true;
}

bool SAerotelicNavCube::ClassifyCursor(const FGeometry& Geometry, const FVector2D& ScreenPos, FIntVector& OutSigns) const
{
	FVector F, R, U;
	if (!GetBasis(F, R, U))
	{
		return false;
	}
	const FVector2D Local = Geometry.AbsoluteToLocal(ScreenPos);
	const FVector2D Centre = Geometry.GetLocalSize() * 0.5;
	const double X = (Local.X - Centre.X) / kHalfEdgePx;
	const double Y = -(Local.Y - Centre.Y) / kHalfEdgePx;
	// A ray along the view direction through the cursor, cast from well in
	// front of the cube.
	const FVector O = R * X + U * Y - F * 4.0;
	FVector Hit;
	if (!RayUnitCube(O, F, Hit))
	{
		return false;
	}
	OutSigns = FIntVector::ZeroValue;
	for (int32 i = 0; i < 3; ++i)
	{
		if (FMath::Abs(Hit[i]) > kFaceZone)
		{
			OutSigns[i] = Hit[i] > 0.0 ? 1 : -1;
		}
	}
	return OutSigns != FIntVector::ZeroValue;
}

int32 SAerotelicNavCube::OnPaint(const FPaintArgs& Args, const FGeometry& Geometry, const FSlateRect& MyCullingRect,
	FSlateWindowElementList& Out, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const
{
	FVector F, R, U;
	if (!GetBasis(F, R, U))
	{
		return LayerId;
	}
	const FVector2D Centre = Geometry.GetLocalSize() * 0.5;
	const FSlateRenderTransform& Transform = Geometry.GetAccumulatedRenderTransform();
	const FSlateResourceHandle White = FSlateApplication::Get().GetRenderer()->GetResourceHandle(*FCoreStyle::Get().GetBrush("WhiteBrush"));
	const FSlateFontInfo Font = FCoreStyle::GetDefaultFontStyle("Bold", 7);
	const TSharedRef<FSlateFontMeasure> Measure = FSlateApplication::Get().GetRenderer()->GetFontMeasureService();
	// A headlight a little above and to the right of the eye.
	const FVector Light = (-F * 0.8 + U * 0.5 + R * 0.25).GetSafeNormal();

	TArray<FSlateVertex> Verts;
	TArray<SlateIndex> Indices;
	struct FLabel { FVector2D Pos; const TCHAR* Text; };
	TArray<FLabel> Labels;
	TArray<TArray<FVector2f>> Outlines;

	for (const FFace& Face : kFaces)
	{
		const FVector N = Unit(Face.Axis) * Face.Sign;
		const double Facing = -FVector::DotProduct(N, F);
		if (Facing <= 0.0)
		{
			continue; // back face
		}
		const int32 J = (Face.Axis + 1) % 3, K = (Face.Axis + 2) % 3;
		const FVector EJ = Unit(J), EK = Unit(K);
		const FVector Corners[4] = { N - EJ - EK, N + EJ - EK, N + EJ + EK, N - EJ + EK };

		const bool bHot = bHovering && Hover[Face.Axis] == Face.Sign;
		const double Shade = 0.55 + 0.45 * FMath::Max(0.0, FVector::DotProduct(N, Light));
		const FLinearColor Base = bHot ? kHoverColor : kBoxColor;
		const FColor Color = (Base * (float)Shade).ToFColor(false);

		const int32 Base0 = Verts.Num();
		TArray<FVector2f> Outline;
		for (const FVector& C : Corners)
		{
			const FVector2D P = Project(C, R, U, Centre);
			Verts.Add(FSlateVertex::Make<ESlateVertexRounding::Disabled>(Transform, FVector2f((float)P.X, (float)P.Y), FVector2f(0.5f, 0.5f), Color));
			Outline.Add(FVector2f((float)P.X, (float)P.Y));
		}
		// (A copy: appending a reference to the array's own element
		// reallocates under it.)
		const FVector2f First = Outline[0];
		Outline.Add(First);
		Outlines.Add(Outline);
		Indices.Append({ (SlateIndex)Base0, (SlateIndex)(Base0 + 1), (SlateIndex)(Base0 + 2), (SlateIndex)Base0, (SlateIndex)(Base0 + 2), (SlateIndex)(Base0 + 3) });

		// Label the face when it is turned enough toward the eye to read.
		if (Facing > 0.35)
		{
			Labels.Add({ Project(N, R, U, Centre), Face.Label });
		}
	}

	if (Verts.Num() > 0)
	{
		FSlateDrawElement::MakeCustomVerts(Out, LayerId, White, Verts, Indices, nullptr, 0, 0);
	}
	for (const TArray<FVector2f>& Outline : Outlines)
	{
		FSlateDrawElement::MakeLines(Out, LayerId + 1, Geometry.ToPaintGeometry(), Outline, ESlateDrawEffect::None, kInnerColor, true, 1.0f);
	}
	for (const FLabel& L : Labels)
	{
		const FText Text = FText::FromString(L.Text);
		const FVector2D Size = Measure->Measure(Text, Font);
		FSlateDrawElement::MakeText(Out, LayerId + 2, Geometry.ToOffsetPaintGeometry(L.Pos - Size * 0.5), Text, Font, ESlateDrawEffect::None, kTextColor);
	}

	// The XYZ triad at the cube's -X -Y -Z corner (SetDrawAxes).
	const FVector Origin(-1.2, -1.2, -1.2);
	const FLinearColor AxisColors[3] = { FLinearColor(0.95f, 0.25f, 0.2f), FLinearColor(0.3f, 0.85f, 0.3f), FLinearColor(0.3f, 0.5f, 1.0f) };
	const TCHAR* AxisNames[3] = { TEXT("X"), TEXT("Y"), TEXT("Z") };
	const FVector2D O2 = Project(Origin, R, U, Centre);
	for (int32 i = 0; i < 3; ++i)
	{
		const FVector2D E2 = Project(Origin + Unit(i) * 0.9, R, U, Centre);
		TArray<FVector2f> Line = { FVector2f((float)O2.X, (float)O2.Y), FVector2f((float)E2.X, (float)E2.Y) };
		FSlateDrawElement::MakeLines(Out, LayerId + 3, Geometry.ToPaintGeometry(), Line, ESlateDrawEffect::None, AxisColors[i], true, 1.5f);
		const FText Text = FText::FromString(AxisNames[i]);
		const FVector2D Size = Measure->Measure(Text, Font);
		const FVector2D Tip = Project(Origin + Unit(i) * 1.15, R, U, Centre);
		FSlateDrawElement::MakeText(Out, LayerId + 3, Geometry.ToOffsetPaintGeometry(Tip - Size * 0.5), Text, Font, ESlateDrawEffect::None, AxisColors[i]);
	}
	return LayerId + 4;
}

FReply SAerotelicNavCube::OnMouseButtonDown(const FGeometry& Geometry, const FPointerEvent& Event)
{
	FIntVector Signs = FIntVector::ZeroValue;
	if (Event.GetEffectingButton() == EKeys::LeftMouseButton && ClassifyCursor(Geometry, Event.GetScreenSpacePosition(), Signs))
	{
		if (AAerotelicCameraPawn* Camera = AAerotelicCameraPawn::FindInGameWorld())
		{
			// The signs give the eye's direction from the model; the camera
			// looks the opposite way.
			const FVector Eye = FVector((double)Signs.X, (double)Signs.Y, (double)Signs.Z).GetSafeNormal();
			Camera->AnimateToForward(-Eye);
		}
	}
	// Swallowed either way: a press on the cube never orbits or selects.
	return FReply::Handled();
}

FReply SAerotelicNavCube::OnMouseButtonUp(const FGeometry& Geometry, const FPointerEvent& Event)
{
	return FReply::Handled();
}

FReply SAerotelicNavCube::OnMouseMove(const FGeometry& Geometry, const FPointerEvent& Event)
{
	FIntVector Signs;
	bHovering = ClassifyCursor(Geometry, Event.GetScreenSpacePosition(), Signs);
	Hover = bHovering ? Signs : FIntVector::ZeroValue;
	return FReply::Handled();
}

void SAerotelicNavCube::OnMouseEnter(const FGeometry& Geometry, const FPointerEvent& Event)
{
	SLeafWidget::OnMouseEnter(Geometry, Event);
}

void SAerotelicNavCube::OnMouseLeave(const FPointerEvent& Event)
{
	bHovering = false;
	Hover = FIntVector::ZeroValue;
	SLeafWidget::OnMouseLeave(Event);
}

FCursorReply SAerotelicNavCube::OnCursorQuery(const FGeometry& Geometry, const FPointerEvent& Event) const
{
	return FCursorReply::Cursor(bHovering ? EMouseCursor::Hand : EMouseCursor::Default);
}
