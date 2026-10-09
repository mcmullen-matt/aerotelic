#include "CurveEditor.h"
#include "PromptDialogs.h"

#include "Widgets/Layout/SSplitter.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/SBoxPanel.h"
#include "Framework/Application/SlateApplication.h"
#include "Framework/MultiBox/MultiBoxBuilder.h"
#include "Rendering/DrawElements.h"
#include "Fonts/FontMeasure.h"
#include "Styling/AppStyle.h"
#include "Styling/CoreStyle.h"
#include "Algo/AllOf.h"
#include "Math/Box2D.h"

using namespace aerotelic;

// =========================================================================
// NurbsCurve edit dispatch -- AddPole/DeletePole/InsertKnot/DeleteKnot/
// ChangeKnot are declared separately (same signature) on ClampedNurbsCurve
// and PeriodicNurbsCurve rather than as virtuals on NurbsCurve (see
// nurbs.hpp); the editor only ever has a NurbsCurve*, so it dispatches on
// periodic() itself. ResetKnots() is already virtual and needs no helper.
// =========================================================================

namespace
{
	void NurbsAddPole(NurbsCurve* Curve, int32 Index, const std::vector<double>& Pole, double Weight = 1.0)
	{
		if (Curve->periodic()) static_cast<PeriodicNurbsCurve*>(Curve)->AddPole(Index, Pole, Weight);
		else static_cast<ClampedNurbsCurve*>(Curve)->AddPole(Index, Pole, Weight);
	}
	void NurbsDeletePole(NurbsCurve* Curve, int32 Index)
	{
		if (Curve->periodic()) static_cast<PeriodicNurbsCurve*>(Curve)->DeletePole(Index);
		else static_cast<ClampedNurbsCurve*>(Curve)->DeletePole(Index);
	}
	void NurbsInsertKnot(NurbsCurve* Curve, double Value)
	{
		if (Curve->periodic()) static_cast<PeriodicNurbsCurve*>(Curve)->InsertKnot(Value);
		else static_cast<ClampedNurbsCurve*>(Curve)->InsertKnot(Value);
	}
	void NurbsDeleteKnot(NurbsCurve* Curve, int32 Index)
	{
		if (Curve->periodic()) static_cast<PeriodicNurbsCurve*>(Curve)->DeleteKnot(Index);
		else static_cast<ClampedNurbsCurve*>(Curve)->DeleteKnot(Index);
	}
	void NurbsChangeKnot(NurbsCurve* Curve, int32 Index, double Value)
	{
		if (Curve->periodic()) static_cast<PeriodicNurbsCurve*>(Curve)->ChangeKnot(Index, Value);
		else static_cast<ClampedNurbsCurve*>(Curve)->ChangeKnot(Index, Value);
	}

	// (value, edit index or unset) per occurrence, in knot order -- the
	// Slate port of nurbs_editor.py's _knot_entries().
	struct FKnotEntry { double Value; TOptional<int32> EditIndex; };

	TArray<FKnotEntry> ComputeKnotEntries(const NurbsCurve& Curve)
	{
		TArray<FKnotEntry> Out;
		if (Curve.periodic())
		{
			const auto& Pc = static_cast<const PeriodicNurbsCurve&>(Curve);
			std::vector<double> Knots = Pc.knots();
			const int32 Pinned = Pc.n_pinned_knots();
			for (int32 i = 0; i < (int32)Knots.size(); ++i)
			{
				Out.Add({ Knots[i], i >= Pinned ? TOptional<int32>(i) : TOptional<int32>() });
			}
			for (int32 i = 0; i < Pc.seam_multiplicity(); ++i)
			{
				Out.Add({ 1.0, TOptional<int32>() });
			}
		}
		else
		{
			const auto& Cc = static_cast<const ClampedNurbsCurve&>(Curve);
			const int32 p = Curve.degree();
			for (int32 i = 0; i <= p; ++i) Out.Add({ 0.0, TOptional<int32>() });
			std::vector<double> Interior = Cc.interior_knots();
			for (int32 i = 0; i < (int32)Interior.size(); ++i) Out.Add({ Interior[i], TOptional<int32>(i) });
			for (int32 i = 0; i <= p; ++i) Out.Add({ 1.0, TOptional<int32>() });
		}
		return Out;
	}

	struct FKnotStack { double Value; TArray<TOptional<int32>> Indices; };

	TArray<FKnotStack> ComputeKnotStacks(const NurbsCurve& Curve)
	{
		TArray<FKnotStack> Stacks;
		for (const FKnotEntry& E : ComputeKnotEntries(Curve))
		{
			FKnotStack* Found = Stacks.FindByPredicate([&](const FKnotStack& S) { return FMath::IsNearlyEqual(S.Value, E.Value, 1e-9); });
			if (Found == nullptr)
			{
				Stacks.Add({ E.Value, {} });
				Found = &Stacks.Last();
			}
			Found->Indices.Add(E.EditIndex);
		}
		Stacks.Sort([](const FKnotStack& A, const FKnotStack& B) { return A.Value < B.Value; });
		return Stacks;
	}

	FLinearColor PoleColor(const FCurveEditorSlot& Slot, int32 Index, int32 Count)
	{
		const float T = Count > 1 ? (float)Index / (float)(Count - 1) : 0.0f;
		return FLinearColor::LerpUsingHSV(Slot.Color, FLinearColor::White, 0.55f * T);
	}

	std::vector<double> ToStdVec(FVector2D V) { return { V.X, V.Y }; }

	// Data-space <-> pane-local-pixel-space, shared by the pane (painting,
	// hit-testing) and SCurveEditor (pole/knot hit-testing lives there so it
	// can be reused by both the mouse handlers and the context-menu builders).
	// Slate's Y grows downward, so the Y axis is flipped relative to the data
	// box, which reads bottom-to-top like matplotlib's.
	FVector2D DataToLocalStatic(FVector2D DataXY, const FBox2D& Box, FVector2D SizePx)
	{
		const double u = (DataXY.X - Box.Min.X) / FMath::Max(Box.Max.X - Box.Min.X, 1e-12);
		const double v = (DataXY.Y - Box.Min.Y) / FMath::Max(Box.Max.Y - Box.Min.Y, 1e-12);
		return FVector2D(u * SizePx.X, (1.0 - v) * SizePx.Y);
	}
	FVector2D LocalToDataStatic(FVector2D Local, const FBox2D& Box, FVector2D SizePx)
	{
		const double u = Local.X / FMath::Max(SizePx.X, 1.0);
		const double v = 1.0 - Local.Y / FMath::Max(SizePx.Y, 1.0);
		return FVector2D(Box.Min.X + u * (Box.Max.X - Box.Min.X), Box.Min.Y + v * (Box.Max.Y - Box.Min.Y));
	}
}

TOptional<FVector2D> FCurveEditorSlot::FixedPole(int32 Index) const
{
	if (Curve == nullptr) return TOptional<FVector2D>();
	const int32 n = Curve->n_poles();
	if (const FVector2D* P = FixedPoles.Find(Index)) return *P;
	if (const FVector2D* P = FixedPoles.Find(Index - n)) return *P;
	return TOptional<FVector2D>();
}

// =========================================================================
// SCurveEditorPane -- one graph (the curve pane, or one slot's basis pane).
// =========================================================================

class SCurveEditorPane : public SLeafWidget
{
public:
	SLATE_BEGIN_ARGS(SCurveEditorPane) {}
	SLATE_END_ARGS()

	enum class ERole { Curve, Basis };

	void Construct(const FArguments& InArgs) {}

	void Initialize(SCurveEditor* InOwner, ERole InRole, int32 InSlotIndex)
	{
		Owner = InOwner;
		Role = InRole;
		SlotIndex = InSlotIndex;
	}

	virtual FVector2D ComputeDesiredSize(float) const override { return FVector2D(220.0f, Role == ERole::Curve ? 220.0f : 110.0f); }

	// The plot area is the pane less a margin on each side for the title and
	// legend (top), the rotated y-axis label (left) and the x-axis label
	// (bottom), so the labels never sit over the curves. Everything in
	// pixel space below -- painting and hit-testing -- is relative to the
	// plot area's origin; see PlotOrigin / LastPaneSizePx.
	FVector2D ComputePlotOrigin() const
	{
		const double Top = Role == ERole::Curve ? (HasLegend() ? 32.0 : 18.0) : 16.0;
		return FVector2D(16.0, Top);
	}
	static constexpr double kMarginRight = 6.0;
	static constexpr double kMarginBottom = 16.0;

	bool HasLegend() const
	{
		for (const FCurveEditorSlot& Slot : Owner->Slots)
		{
			if (!Slot.Label.IsEmpty()) return true;
		}
		return false;
	}

	FVector2D ToPlotLocal(const FGeometry& Geometry, const FVector2D& ScreenPos) const
	{
		return Geometry.AbsoluteToLocal(ScreenPos) - PlotOrigin;
	}

	virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
		FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const override
	{
		const FVector2D SizePx = AllottedGeometry.GetLocalSize();
		PlotOrigin = ComputePlotOrigin();
		LastPaneSizePx = FVector2D(FMath::Max(SizePx.X - PlotOrigin.X - kMarginRight, 1.0), FMath::Max(SizePx.Y - PlotOrigin.Y - kMarginBottom, 1.0));

		// Background.
		FSlateDrawElement::MakeBox(OutDrawElements, LayerId, AllottedGeometry.ToPaintGeometry(),
			FAppStyle::Get().GetBrush("WhiteBrush"), ESlateDrawEffect::None, FLinearColor(0.051f, 0.067f, 0.09f));

		// Curves and poles are clipped to the plot area so a zoomed view
		// does not spill over the labels in the margins.
		// (Grown by a pole marker's radius so a pole on the frame is whole.)
		OutDrawElements.PushClip(FSlateClippingZone(AllottedGeometry.ToPaintGeometry(
			FVector2f(LastPaneSizePx) + FVector2f(16.0f, 16.0f), FSlateLayoutTransform(FVector2f(PlotOrigin) - FVector2f(8.0f, 8.0f)))));
		if (Role == ERole::Curve)
		{
			LastDrawnBox = Owner->GetOrComputeViewBoxForPane(LastPaneSizePx);
			PaintCurvePane(AllottedGeometry, OutDrawElements, LayerId + 1);
		}
		else
		{
			LastDrawnBox = ComputeBasisBox();
			PaintBasisPane(AllottedGeometry, OutDrawElements, LayerId + 1);
		}
		OutDrawElements.PopClip();
		PaintLabels(AllottedGeometry, OutDrawElements, LayerId + 2);
		return LayerId + 3;
	}

	// Title, axis labels and a slot legend, as the Python canvases draw
	// (title and axis labels in grey, the legend top-left under the title).
	void PaintLabels(const FGeometry& Geometry, FSlateWindowElementList& Out, int32 Layer) const
	{
		const FSlateFontInfo Font = FCoreStyle::GetDefaultFontStyle("Regular", 8);
		const FLinearColor TitleColor(0.8f, 0.8f, 0.8f);
		const FLinearColor LabelColor(0.53f, 0.53f, 0.53f);
		const TSharedRef<FSlateFontMeasure> Measure = FSlateApplication::Get().GetRenderer()->GetFontMeasureService();
		const FVector2D Size = Geometry.GetLocalSize();

		auto Draw = [&](const FText& Text, FVector2D Pos, const FLinearColor& Color)
		{
			if (Text.IsEmpty()) return;
			FSlateDrawElement::MakeText(Out, Layer, Geometry.ToOffsetPaintGeometry(Pos), Text, Font, ESlateDrawEffect::None, Color);
		};
		// The x label centred under the plot area; the y label rotated a
		// quarter turn anticlockwise and centred beside it, as matplotlib
		// lays out axis labels.
		auto DrawXLabel = [&](const FText& Text)
		{
			if (Text.IsEmpty()) return;
			const FVector2D TextSize = Measure->Measure(Text, Font);
			const double X = PlotOrigin.X + (LastPaneSizePx.X - TextSize.X) * 0.5;
			Draw(Text, FVector2D(FMath::Max(X, 2.0), Size.Y - TextSize.Y - 2), LabelColor);
		};
		auto DrawYLabel = [&](const FText& Text)
		{
			if (Text.IsEmpty()) return;
			const FVector2D TextSize = Measure->Measure(Text, Font);
			// Rotating by -90 degrees about the layout origin maps the text's
			// baseline (+x) onto -y, so the glyphs read bottom to top from
			// the origin; the origin is therefore the label's bottom-left.
			const double Y = PlotOrigin.Y + (LastPaneSizePx.Y + TextSize.X) * 0.5;
			const FSlateRenderTransform Rotate(FQuat2D(-UE_HALF_PI));
			FSlateDrawElement::MakeText(Out, Layer,
				Geometry.ToPaintGeometry(FVector2f(TextSize), FSlateLayoutTransform(FVector2f(2.0f, (float)Y)), Rotate, FVector2f::ZeroVector),
				Text, Font, ESlateDrawEffect::None, LabelColor);
		};
		// A faint frame around the plot area so the axes read as a graph.
		{
			const FLinearColor FrameColor(0.22f, 0.24f, 0.28f);
			const FVector2f O((float)PlotOrigin.X, (float)PlotOrigin.Y);
			const FVector2f S((float)LastPaneSizePx.X, (float)LastPaneSizePx.Y);
			TArray<FVector2f> Pts = { O, FVector2f(O.X + S.X, O.Y), O + S, FVector2f(O.X, O.Y + S.Y), O };
			FSlateDrawElement::MakeLines(Out, Layer, Geometry.ToPaintGeometry(), Pts, ESlateDrawEffect::None, FrameColor, false, 1.0f);
		}

		if (Role == ERole::Curve)
		{
			Draw(Owner->Title, FVector2D(6, 3), TitleColor);
			// Legend: one coloured swatch + label per named slot.
			double X = 6.0;
			const double LegendY = 17.0;
			for (const FCurveEditorSlot& Slot : Owner->Slots)
			{
				if (Slot.Label.IsEmpty()) continue;
				FSlateDrawElement::MakeBox(Out, Layer, Geometry.ToPaintGeometry(FVector2f(10, 3), FSlateLayoutTransform(FVector2f(X, LegendY + 6))),
					FAppStyle::Get().GetBrush("WhiteBrush"), ESlateDrawEffect::None, Slot.Color);
				Draw(Slot.Label, FVector2D(X + 14, LegendY), LabelColor);
				X += 14 + Measure->Measure(Slot.Label, Font).X + 12;
			}
			DrawYLabel(Owner->YLabel);
			DrawXLabel(Owner->XLabel);
		}
		else if (Owner->Slots.IsValidIndex(SlotIndex))
		{
			const FCurveEditorSlot& Slot = Owner->Slots[SlotIndex];
			const FText Title = Slot.Label.IsEmpty()
				? NSLOCTEXT("CurveEditor", "BasisTitle", "blending functions and knots  ·  drag knots along the line  ·  right-click to insert / delete")
				: FText::Format(NSLOCTEXT("CurveEditor", "BasisTitleLabelled", "{0}  ·  blending functions and knots  ·  drag knots along the line  ·  right-click to insert / delete"), Slot.Label);
			Draw(Title, FVector2D(6, 3), TitleColor);
			DrawYLabel(NSLOCTEXT("CurveEditor", "BasisY", "N(s)"));
			DrawXLabel(NSLOCTEXT("CurveEditor", "BasisX", "Curve parameter s"));
		}
	}

	// ---- mouse ----

	virtual FReply OnMouseButtonDown(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent) override
	{
		const FVector2D Local = ToPlotLocal(MyGeometry, MouseEvent.GetScreenSpacePosition());
		if (MouseEvent.GetEffectingButton() == EKeys::LeftMouseButton)
		{
			if (Role == ERole::Curve) return PressCurve(Local);
			return PressBasis(Local);
		}
		if (MouseEvent.GetEffectingButton() == EKeys::RightMouseButton)
		{
			if (Role == ERole::Curve) ShowCurveContextMenu(Local, MouseEvent.GetScreenSpacePosition());
			else ShowBasisContextMenu(Local, MouseEvent.GetScreenSpacePosition());
			return FReply::Handled();
		}
		return FReply::Unhandled();
	}

	virtual FReply OnMouseButtonDoubleClick(const FGeometry& InMyGeometry, const FPointerEvent& InMouseEvent) override
	{
		if (Role != ERole::Curve || InMouseEvent.GetEffectingButton() != EKeys::LeftMouseButton)
		{
			return FReply::Unhandled();
		}
		const FVector2D Local = ToPlotLocal(InMyGeometry, InMouseEvent.GetScreenSpacePosition());
		if (Owner->PoleAt(Local, LastDrawnBox, LastPaneSizePx).IsSet())
		{
			return FReply::Handled(); // a pole under the cursor: the first click already selected/dragged it
		}
		Owner->AddPoleAtLocal(Local, LastDrawnBox, LastPaneSizePx);
		Invalidate(EInvalidateWidget::Paint);
		return FReply::Handled();
	}

	virtual FReply OnMouseMove(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent) override
	{
		const FVector2D Local = ToPlotLocal(MyGeometry, MouseEvent.GetScreenSpacePosition());
		if (Role == ERole::Curve && Owner->IsDraggingPole())
		{
			Owner->DragPoleTo(Local, LastDrawnBox, LastPaneSizePx);
			Invalidate(EInvalidateWidget::Paint);
			return FReply::Handled();
		}
		if (Role == ERole::Basis && Owner->IsDraggingKnot(SlotIndex))
		{
			Owner->DragKnotTo(SlotIndex, Local, LastDrawnBox, LastPaneSizePx);
			Invalidate(EInvalidateWidget::Paint);
			return FReply::Handled();
		}
		return FReply::Unhandled();
	}

	virtual FReply OnMouseButtonUp(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent) override
	{
		if (MouseEvent.GetEffectingButton() != EKeys::LeftMouseButton)
		{
			return FReply::Unhandled();
		}
		if (Role == ERole::Curve && Owner->IsDraggingPole())
		{
			Owner->ReleasePole();
			return FReply::Handled();
		}
		if (Role == ERole::Basis && Owner->IsDraggingKnot(SlotIndex))
		{
			Owner->ReleaseKnot();
			return FReply::Handled();
		}
		return FReply::Unhandled();
	}

	virtual FReply OnMouseWheel(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent) override
	{
		if (Role != ERole::Curve)
		{
			return FReply::Unhandled();
		}
		const FVector2D Local = ToPlotLocal(MyGeometry, MouseEvent.GetScreenSpacePosition());
		Owner->ZoomAboutLocal(Local, LastDrawnBox, LastPaneSizePx, MouseEvent.GetWheelDelta());
		Invalidate(EInvalidateWidget::Paint);
		return FReply::Handled();
	}

private:
	SCurveEditor* Owner = nullptr;
	ERole Role = ERole::Curve;
	int32 SlotIndex = -1;

	mutable FBox2D LastDrawnBox = FBox2D(FVector2D(0, 0), FVector2D(1, 1));
	// Plot-area size and its origin within the pane (see ComputePlotOrigin).
	mutable FVector2D LastPaneSizePx = FVector2D(220, 220);
	mutable FVector2D PlotOrigin = FVector2D(16, 18);

	// Data space to pane-local pixels (plot-area pixels plus the origin).
	FVector2D DataToLocal(FVector2D DataXY) const
	{
		return DataToLocalStatic(DataXY, LastDrawnBox, LastPaneSizePx) + PlotOrigin;
	}

	FReply PressCurve(FVector2D Local)
	{
		TOptional<FIntPoint> Hit = Owner->PoleAt(Local, LastDrawnBox, LastPaneSizePx);
		if (Hit.IsSet())
		{
			Owner->BeginDragPole(Hit.GetValue(), LastDrawnBox);
			Invalidate(EInvalidateWidget::Paint);
			return FReply::Handled();
		}
		Owner->DeselectAll();
		Invalidate(EInvalidateWidget::Paint);
		return FReply::Handled();
	}

	FReply PressBasis(FVector2D Local)
	{
		TOptional<int32> KnotEditIndex = Owner->KnotAt(SlotIndex, Local, LastDrawnBox, LastPaneSizePx);
		if (KnotEditIndex.IsSet())
		{
			Owner->BeginDragKnot(SlotIndex, KnotEditIndex.GetValue());
			return FReply::Handled();
		}
		Owner->SelectBasisFunction(SlotIndex, Local, LastDrawnBox, LastPaneSizePx);
		Invalidate(EInvalidateWidget::Paint);
		return FReply::Handled();
	}

	void ShowCurveContextMenu(FVector2D Local, FVector2D ScreenPos)
	{
		TOptional<FIntPoint> Hit = Owner->PoleAt(Local, LastDrawnBox, LastPaneSizePx);
		FMenuBuilder MenuBuilder(true, nullptr);
		if (Hit.IsSet())
		{
			Owner->BuildPoleMenu(MenuBuilder, Hit.GetValue());
		}
		else
		{
			MenuBuilder.AddMenuEntry(NSLOCTEXT("CurveEditor", "FitView", "Fit view to contents"),
				FText::GetEmpty(), FSlateIcon(), FUIAction(FExecuteAction::CreateSP(Owner, &SCurveEditor::FitView)));
		}
		FSlateApplication::Get().PushMenu(AsShared(), FWidgetPath(), MenuBuilder.MakeWidget(),
			ScreenPos, FPopupTransitionEffect(FPopupTransitionEffect::ContextMenu));
	}

	void ShowBasisContextMenu(FVector2D Local, FVector2D ScreenPos)
	{
		TOptional<int32> KnotEditIndex = Owner->KnotAt(SlotIndex, Local, LastDrawnBox, LastPaneSizePx);
		const double ClickS = Local.X / FMath::Max(LastPaneSizePx.X, 1.0) * (LastDrawnBox.Max.X - LastDrawnBox.Min.X) + LastDrawnBox.Min.X;
		FMenuBuilder MenuBuilder(true, nullptr);
		Owner->BuildKnotMenu(MenuBuilder, SlotIndex, KnotEditIndex, ClickS);
		FSlateApplication::Get().PushMenu(AsShared(), FWidgetPath(), MenuBuilder.MakeWidget(),
			ScreenPos, FPopupTransitionEffect(FPopupTransitionEffect::ContextMenu));
	}

	FBox2D ComputeBasisBox() const
	{
		if (SlotIndex < 0 || SlotIndex >= Owner->Slots.Num() || Owner->Slots[SlotIndex].Curve == nullptr)
		{
			return FBox2D(FVector2D(0, 0), FVector2D(1, 1));
		}
		const NurbsCurve& Curve = *Owner->Slots[SlotIndex].Curve;
		TArray<FKnotStack> Stacks = ComputeKnotStacks(Curve);
		int32 MaxStack = 1;
		for (const FKnotStack& S : Stacks) MaxStack = FMath::Max(MaxStack, S.Indices.Num());
		MaxStack = FMath::Max(MaxStack, Curve.degree() + 1);
		const double StackDy = 0.11;
		const double YLo = -0.06 - StackDy * (MaxStack - 1) - 0.08;
		return FBox2D(FVector2D(-0.03, YLo), FVector2D(1.05, 1.08));
	}

	void PaintCurvePane(const FGeometry& Geo, FSlateWindowElementList& OutDrawElements, int32 LayerId) const
	{
		const FBox2D& Box = LastDrawnBox;
		const double Sx = Owner->XScale, Sy = Owner->YScale;

		// y = 0 axis.
		DrawDataLine(Geo, OutDrawElements, LayerId, FVector2D(Box.Min.X, 0.0), FVector2D(Box.Max.X, 0.0), FLinearColor(0.27f, 0.27f, 0.27f), 0.8f);
		const bool bAllFreeX = Owner->Slots.Num() > 0 && Algo::AllOf(Owner->Slots, [](const FCurveEditorSlot& S) { return S.bFreeX; });
		if (bAllFreeX)
		{
			DrawDataLine(Geo, OutDrawElements, LayerId, FVector2D(0.0, Box.Min.Y), FVector2D(0.0, Box.Max.Y), FLinearColor(0.27f, 0.27f, 0.27f), 0.8f);
		}

		// Outline paths (read-only context / display-only shapes).
		if (Owner->OutlineProvider.IsBound())
		{
			for (const FCurveEditorOutlinePath& Path : Owner->OutlineProvider.Execute())
			{
				TArray<FVector2f> Pts;
				Pts.Reserve(Path.Points.Num() + 1);
				for (const FVector2D& P : Path.Points)
				{
					const FVector2D L = DataToLocal(FVector2D(P.X * Sx, P.Y * Sy));
					Pts.Add(FVector2f((float)L.X, (float)L.Y));
				}
				// Pts.Add(Pts[0]) asserts in UE's TArray -- passing one of the
				// array's own elements back into Add() is treated as a hazard
				// regardless of whether this specific call would reallocate.
				if (Path.bClosed && Pts.Num() > 0) { const FVector2f First = Pts[0]; Pts.Add(First); }
				if (Pts.Num() >= 2)
				{
					FSlateDrawElement::MakeLines(OutDrawElements, LayerId, Geo.ToPaintGeometry(), Pts, ESlateDrawEffect::None, Path.Color, true, Path.LineWidth);
				}
			}
		}

		for (int32 k = 0; k < Owner->Slots.Num(); ++k)
		{
			const FCurveEditorSlot& Slot = Owner->Slots[k];
			if (Slot.Curve == nullptr) continue;
			Mat Samples = Slot.Curve->Sample(96);
			TArray<FVector2f> Pts;
			Pts.Reserve(Samples.rows() + 1);
			for (int32 i = 0; i < Samples.rows(); ++i)
			{
				const FVector2D L = DataToLocal(FVector2D(Samples(i, 0) * Sx, Samples(i, 1) * Sy));
				Pts.Add(FVector2f((float)L.X, (float)L.Y));
			}
			if (Slot.bClosedFill && Pts.Num() > 0) { const FVector2f First = Pts[0]; Pts.Add(First); }
			if (Pts.Num() >= 2)
			{
				FSlateDrawElement::MakeLines(OutDrawElements, LayerId, Geo.ToPaintGeometry(), Pts, ESlateDrawEffect::None, Slot.Color, true, 1.6f);
			}

			// Control polygon.
			Mat Poles = Slot.Curve->poles();
			TArray<FVector2f> Poly;
			Poly.Reserve(Poles.rows() + 1);
			for (int32 i = 0; i < Poles.rows(); ++i)
			{
				const FVector2D L = DataToLocal(FVector2D(Poles(i, 0) * Sx, Poles(i, 1) * Sy));
				Poly.Add(FVector2f((float)L.X, (float)L.Y));
			}
			if (Slot.Curve->periodic() && Poly.Num() > 0) { const FVector2f First = Poly[0]; Poly.Add(First); }
			if (Poly.Num() >= 2)
			{
				FSlateDrawElement::MakeLines(OutDrawElements, LayerId, Geo.ToPaintGeometry(), Poly, ESlateDrawEffect::None, Slot.Color.CopyWithNewOpacity(0.5f), true, 0.8f);
			}

			// Poles.
			Mat PolesAndWeights = Slot.Curve->poles_and_weights();
			const int32 n = Poles.rows();
			for (int32 i = 0; i < n; ++i)
			{
				const FVector2D DataPos(Poles(i, 0) * Sx, Poles(i, 1) * Sy);
				const FVector2D Pos = DataToLocal(DataPos);
				const bool bSel = Owner->Selected.IsSet() && Owner->Selected->X == k && Owner->Selected->Y == i;
				const bool bFixed = Slot.FixedPole(i).IsSet();
				const bool bEnd = SCurveEditor::IsEndPole(Slot, i);
				FLinearColor Fill = bFixed ? FLinearColor(0.6f, 0.627f, 0.667f) : PoleColor(Slot, i, n);
				FLinearColor Edge = bEnd ? FLinearColor(0.94f, 0.627f, 0.188f) : (bSel ? FLinearColor::White : FLinearColor(0.1f, 0.1f, 0.1f));
				const float Radius = bSel ? 6.0f : 5.0f;
				FSlateDrawElement::MakeBox(OutDrawElements, LayerId + 1,
					Geo.ToPaintGeometry(FVector2D(Radius * 2 + 2), FSlateLayoutTransform(Pos - FVector2D(Radius + 1))),
					FAppStyle::Get().GetBrush("WhiteBrush"), ESlateDrawEffect::None, Edge);
				FSlateDrawElement::MakeBox(OutDrawElements, LayerId + 2,
					Geo.ToPaintGeometry(FVector2D(Radius * 2), FSlateLayoutTransform(Pos - FVector2D(Radius))),
					FAppStyle::Get().GetBrush("WhiteBrush"), ESlateDrawEffect::None, Fill);
			}
		}
	}

	void PaintBasisPane(const FGeometry& Geo, FSlateWindowElementList& OutDrawElements, int32 LayerId) const
	{
		if (SlotIndex < 0 || SlotIndex >= Owner->Slots.Num() || Owner->Slots[SlotIndex].Curve == nullptr) return;
		const FCurveEditorSlot& Slot = Owner->Slots[SlotIndex];
		const NurbsCurve& Curve = *Slot.Curve;

		DrawDataLine(Geo, OutDrawElements, LayerId, FVector2D(LastDrawnBox.Min.X, 0.0), FVector2D(LastDrawnBox.Max.X, 0.0), FLinearColor(0.53f, 0.53f, 0.53f), 0.9f);

		std::vector<double> S(65);
		for (int32 i = 0; i < 65; ++i) S[i] = (double)i / 64.0;
		Mat R = Curve.BasisMatrix(S, true);
		const int32 n = R.cols();
		const bool bHasSel = Owner->Selected.IsSet() && Owner->Selected->X == SlotIndex;
		for (int32 i = 0; i < n; ++i)
		{
			const bool bSel = bHasSel && Owner->Selected->Y == i;
			const bool bDim = bHasSel && !bSel;
			TArray<FVector2f> Pts;
			Pts.Reserve(65);
			for (int32 j = 0; j < 65; ++j)
			{
				const FVector2D L = DataToLocal(FVector2D(S[j], R(j, i)));
				Pts.Add(FVector2f((float)L.X, (float)L.Y));
			}
			FLinearColor Color = PoleColor(Slot, i, n);
			if (bDim) Color = Color.CopyWithNewOpacity(0.25f);
			FSlateDrawElement::MakeLines(OutDrawElements, LayerId, Geo.ToPaintGeometry(), Pts, ESlateDrawEffect::None, Color, true, bSel ? 2.2f : 1.2f);
		}

		for (const FKnotStack& Stack : ComputeKnotStacks(Curve))
		{
			for (int32 j = 0; j < Stack.Indices.Num(); ++j)
			{
				const double MarkerY = -0.06 - 0.11 * j;
				const FVector2D Pos = DataToLocal(FVector2D(Stack.Value, MarkerY));
				const bool bPinned = !Stack.Indices[j].IsSet();
				FLinearColor Color = bPinned ? FLinearColor(0.467f, 0.467f, 0.467f) : FLinearColor(0.314f, 0.753f, 0.376f);
				FSlateDrawElement::MakeBox(OutDrawElements, LayerId + 1,
					Geo.ToPaintGeometry(FVector2D(9, 9), FSlateLayoutTransform(Pos - FVector2D(4.5f))),
					FAppStyle::Get().GetBrush("WhiteBrush"), ESlateDrawEffect::None, Color);
			}
		}
	}

	void DrawDataLine(const FGeometry& Geo, FSlateWindowElementList& OutDrawElements, int32 LayerId,
		FVector2D DataA, FVector2D DataB, FLinearColor Color, float Thickness) const
	{
		const FVector2D A = DataToLocal(DataA), B = DataToLocal(DataB);
		TArray<FVector2f> Pts = { FVector2f((float)A.X, (float)A.Y), FVector2f((float)B.X, (float)B.Y) };
		FSlateDrawElement::MakeLines(OutDrawElements, LayerId, Geo.ToPaintGeometry(), Pts, ESlateDrawEffect::None, Color, true, Thickness);
	}
};

// =========================================================================
// SCurveEditor
// =========================================================================

void SCurveEditor::Construct(const FArguments& InArgs)
{
	XLabel = InArgs._XLabel;
	YLabel = InArgs._YLabel;
	Title = InArgs._Title;
	XScale = InArgs._XScale;
	YScale = InArgs._YScale;
	bEqualAspect = InArgs._EqualAspect;
	bSupportsUserLinks = InArgs._SupportsUserLinks;
	LeadingName = InArgs._LeadingName;
	TrailingName = InArgs._TrailingName;
	OnChangedDelegate = InArgs._OnChanged;

	ChildSlot
	[
		SAssignNew(Splitter, SSplitter)
		.Orientation(Orient_Vertical)
	];
	RebuildPanes();
}

void SCurveEditor::RebuildPanes()
{
	if (!Splitter.IsValid())
	{
		return;
	}
	while (Splitter->GetChildren()->Num() > 0)
	{
		Splitter->RemoveAt(0);
	}
	BasisPanes.SetNum(Slots.Num());
	for (int32 k = 0; k < Slots.Num(); ++k)
	{
		BasisPanes[k] = nullptr;
	}

	for (int32 k = 0; k < Slots.Num(); ++k)
	{
		if (Slots[k].Basis == ECurveBasisPlacement::Above)
		{
			TSharedRef<SCurveEditorPane> Pane = SNew(SCurveEditorPane);
			Pane->Initialize(this, SCurveEditorPane::ERole::Basis, k);
			Splitter->AddSlot().Value(0.4f)[Pane];
			BasisPanes[k] = Pane;
		}
	}
	{
		TSharedRef<SCurveEditorPane> Pane = SNew(SCurveEditorPane);
		Pane->Initialize(this, SCurveEditorPane::ERole::Curve, -1);
		Splitter->AddSlot().Value(1.0f)[Pane];
		CurvePane = Pane;
	}
	for (int32 k = 0; k < Slots.Num(); ++k)
	{
		if (Slots[k].Basis == ECurveBasisPlacement::Below)
		{
			TSharedRef<SCurveEditorPane> Pane = SNew(SCurveEditorPane);
			Pane->Initialize(this, SCurveEditorPane::ERole::Basis, k);
			Splitter->AddSlot().Value(0.4f)[Pane];
			BasisPanes[k] = Pane;
		}
	}
}

void SCurveEditor::SetSlots(TArray<FCurveEditorSlot> InSlots)
{
	Slots = MoveTemp(InSlots);
	Selected.Reset();
	RebuildPanes();
}

void SCurveEditor::SetScales(double NewXScale, double NewYScale)
{
	XScale = NewXScale;
	YScale = NewYScale;
	RequestRedraw();
}

void SCurveEditor::FitView()
{
	ViewRequest.Reset();
	RequestRedraw();
}

void SCurveEditor::RequestRedraw()
{
	if (CurvePane.IsValid())
	{
		CurvePane->Invalidate(EInvalidateWidget::Paint);
	}
	for (auto& Pane : BasisPanes)
	{
		if (Pane.IsValid())
		{
			Pane->Invalidate(EInvalidateWidget::Paint);
		}
	}
}

void SCurveEditor::NotifyChanged()
{
	// Mirrors nurbs_editor.py's _notify(): OnChanged typically rebuilds the
	// solid and pumps the Slate tick, so a drag-release delivered from
	// inside that call must not start a second, nested rebuild.
	if (bNotifying)
	{
		return;
	}
	bNotifying = true;
	OnChangedDelegate.ExecuteIfBound();
	bNotifying = false;
}

// ---- topology / constraints ----

FVector2D SCurveEditor::ConstrainPole(const FCurveEditorSlot& Slot, int32 PoleIndex, FVector2D Requested) const
{
	TOptional<FVector2D> Fixed = Slot.FixedPole(PoleIndex);
	if (Fixed.IsSet())
	{
		return Fixed.GetValue();
	}
	const NurbsCurve* C = Slot.Curve;
	const int32 n = C->n_poles();
	double NewX;
	if (Slot.bFreeX)
	{
		NewX = Requested.X;
	}
	else if (C->periodic())
	{
		NewX = (PoleIndex == n - 1) ? 1.0 : FMath::Clamp(Requested.X, 0.0, 1.0 - 1e-4);
	}
	else if (PoleIndex == 0)
	{
		NewX = 0.0;
	}
	else if (PoleIndex == n - 1)
	{
		NewX = 1.0;
	}
	else
	{
		NewX = FMath::Clamp(Requested.X, 1e-4, 1.0 - 1e-4);
	}
	double Y = Requested.Y;
	if (Slot.YClampLo.IsSet()) Y = FMath::Max(Y, Slot.YClampLo.GetValue());
	if (Slot.YClampHi.IsSet()) Y = FMath::Min(Y, Slot.YClampHi.GetValue());
	return FVector2D(NewX, Y);
}

bool SCurveEditor::IsEndPole(const FCurveEditorSlot& Slot, int32 PoleIndex)
{
	if (Slot.bFreeX)
	{
		return false;
	}
	const int32 n = Slot.Curve->n_poles();
	if (Slot.Curve->periodic())
	{
		return PoleIndex == n - 1;
	}
	return PoleIndex == 0 || PoleIndex == n - 1;
}

TArray<FIntPoint> SCurveEditor::Partners(int32 SlotIndex, int32 PoleIndex) const
{
	auto Norm = [this](FIntPoint End)
	{
		const int32 n = Slots[End.X].Curve->n_poles();
		return FIntPoint(End.X, ((End.Y % n) + n) % n);
	};
	const FIntPoint Me(SlotIndex, PoleIndex);
	TArray<FIntPoint> Out;
	for (const FCurveEditorLink& L : Links)
	{
		const FIntPoint A = Norm(L.A), B = Norm(L.B);
		if (A == Me) Out.Add(B);
		else if (B == Me) Out.Add(A);
	}
	return Out;
}

bool SCurveEditor::CanRemovePole(const FCurveEditorSlot& Slot, int32 SlotIndex, int32 PoleIndex) const
{
	if (Slot.FixedPole(PoleIndex).IsSet()) return false;
	if (Partners(SlotIndex, PoleIndex).Num() > 0) return false;
	const NurbsCurve* C = Slot.Curve;
	const int32 n = C->n_poles();
	if (C->periodic())
	{
		if (Slot.bFreeX) return n > C->degree() + 1;
		return PoleIndex != n - 1 && n > C->degree() + 1;
	}
	return PoleIndex > 0 && PoleIndex < n - 1 && n > 2;
}

void SCurveEditor::MovePole(int32 SlotIndex, int32 PoleIndex, FVector2D Requested)
{
	if (TargetOf(FIntPoint(SlotIndex, PoleIndex)).IsSet())
	{
		return; // a follower moves only when its target moves
	}
	FCurveEditorSlot& Slot = Slots[SlotIndex];
	const FVector2D P = ConstrainPole(Slot, PoleIndex, Requested);
	Slot.Curve->UpdatePole(PoleIndex, ToStdVec(P));
	TArray<FIntPoint> Moved;
	Moved.Add(FIntPoint(SlotIndex, PoleIndex));
	for (FIntPoint Partner : Partners(SlotIndex, PoleIndex))
	{
		FCurveEditorSlot& Other = Slots[Partner.X];
		Other.Curve->UpdatePole(Partner.Y, ToStdVec(ConstrainPole(Other, Partner.Y, P)));
		Moved.Add(Partner);
	}
	for (FIntPoint Src : Moved)
	{
		for (FIntPoint Follower : FollowersOf(Src))
		{
			FCurveEditorSlot& Other = Slots[Follower.X];
			Other.Curve->UpdatePole(Follower.Y, ToStdVec(ConstrainPole(Other, Follower.Y, P)));
		}
	}
}

bool SCurveEditor::Linkable(int32 SlotIndex) const
{
	return Slots.IsValidIndex(SlotIndex) && !Slots[SlotIndex].Curve->periodic() && !Slots[SlotIndex].bFreeX;
}

TOptional<FIntPoint> SCurveEditor::TargetOf(FIntPoint FollowerSlotPole) const
{
	for (const FCurveEditorUserLink& L : UserLinks)
	{
		if (L.Follower.X == L.Target.X || !Linkable(L.Follower.X) || !Linkable(L.Target.X)) continue;
		const int32 Nf = Slots[L.Follower.X].Curve->n_poles();
		const int32 Fi = ((L.Follower.Y % Nf) + Nf) % Nf;
		if (FIntPoint(L.Follower.X, Fi) == FollowerSlotPole)
		{
			const int32 Nt = Slots[L.Target.X].Curve->n_poles();
			const int32 Ti = ((L.Target.Y % Nt) + Nt) % Nt;
			return FIntPoint(L.Target.X, Ti);
		}
	}
	return TOptional<FIntPoint>();
}

TArray<FIntPoint> SCurveEditor::FollowersOf(FIntPoint TargetSlotPole) const
{
	TArray<FIntPoint> Out;
	for (const FCurveEditorUserLink& L : UserLinks)
	{
		if (L.Follower.X == L.Target.X || !Linkable(L.Follower.X) || !Linkable(L.Target.X)) continue;
		const int32 Nt = Slots[L.Target.X].Curve->n_poles();
		const int32 Ti = ((L.Target.Y % Nt) + Nt) % Nt;
		if (FIntPoint(L.Target.X, Ti) == TargetSlotPole)
		{
			const int32 Nf = Slots[L.Follower.X].Curve->n_poles();
			const int32 Fi = ((L.Follower.Y % Nf) + Nf) % Nf;
			Out.Add(FIntPoint(L.Follower.X, Fi));
		}
	}
	return Out;
}

TOptional<int32> SCurveEditor::EndOf(int32 SlotIndex, int32 PoleIndex) const
{
	const int32 n = Slots[SlotIndex].Curve->n_poles();
	if (PoleIndex == 0) return 0;
	if (PoleIndex == n - 1) return -1;
	return TOptional<int32>();
}

bool SCurveEditor::EnforceLinks()
{
	bool bChanged = false;
	for (const FCurveEditorUserLink& L : UserLinks)
	{
		if (L.Follower.X == L.Target.X || !Linkable(L.Follower.X) || !Linkable(L.Target.X)) continue;
		const int32 Nf = Slots[L.Follower.X].Curve->n_poles();
		const int32 Fi = ((L.Follower.Y % Nf) + Nf) % Nf;
		const int32 Nt = Slots[L.Target.X].Curve->n_poles();
		const int32 Ti = ((L.Target.Y % Nt) + Nt) % Nt;
		Mat TPoles = Slots[L.Target.X].Curve->poles();
		const FVector2D TargetXY(TPoles(Ti, 0), TPoles(Ti, 1));
		const FVector2D Want = ConstrainPole(Slots[L.Follower.X], Fi, TargetXY);
		Mat FPoles = Slots[L.Follower.X].Curve->poles();
		const FVector2D CurXY(FPoles(Fi, 0), FPoles(Fi, 1));
		if (!CurXY.Equals(Want, 1e-12))
		{
			Slots[L.Follower.X].Curve->UpdatePole(Fi, ToStdVec(Want));
			bChanged = true;
		}
	}
	return bChanged;
}

void SCurveEditor::LinkPole(int32 SlotIndex, int32 PoleIndex, int32 TargetSlot, int32 TargetPole)
{
	TOptional<int32> End = EndOf(SlotIndex, PoleIndex), EndT = EndOf(TargetSlot, TargetPole);
	if (!End.IsSet() || !EndT.IsSet() || !Linkable(SlotIndex) || !Linkable(TargetSlot))
	{
		return;
	}
	UserLinks.RemoveAll([&](const FCurveEditorUserLink& L)
	{
		if (!Linkable(L.Follower.X)) return false;
		const int32 n = Slots[L.Follower.X].Curve->n_poles();
		return L.Follower.X == SlotIndex && (((L.Follower.Y % n) + n) % n) == PoleIndex;
	});
	UserLinks.Add({ FIntPoint(SlotIndex, End.GetValue()), FIntPoint(TargetSlot, EndT.GetValue()) });
	EnforceLinks();
	RequestRedraw();
	NotifyChanged();
}

void SCurveEditor::UnlinkPole(int32 SlotIndex, int32 PoleIndex)
{
	UserLinks.RemoveAll([&](const FCurveEditorUserLink& L)
	{
		if (!Linkable(L.Follower.X)) return false;
		const int32 n = Slots[L.Follower.X].Curve->n_poles();
		return L.Follower.X == SlotIndex && (((L.Follower.Y % n) + n) % n) == PoleIndex;
	});
	RequestRedraw();
}

// ---- view box ----

FBox2D SCurveEditor::ComputeFitBox() const
{
	const double Sx = XScale, Sy = YScale;
	TArray<double> AllY;
	const bool bAllFreeX = Slots.Num() > 0 && Algo::AllOf(Slots, [](const FCurveEditorSlot& S) { return S.bFreeX; });
	double XLo = bAllFreeX ? TNumericLimits<double>::Max() : -0.05 * Sx;
	double XHi = bAllFreeX ? -TNumericLimits<double>::Max() : 1.08 * Sx;

	if (OutlineProvider.IsBound())
	{
		for (const FCurveEditorOutlinePath& Path : OutlineProvider.Execute())
		{
			for (const FVector2D& P : Path.Points)
			{
				AllY.Add(P.Y * Sy);
				XLo = FMath::Min(XLo, P.X * Sx);
				XHi = FMath::Max(XHi, P.X * Sx);
			}
		}
	}
	for (const FCurveEditorSlot& Slot : Slots)
	{
		if (Slot.Curve == nullptr) continue;
		Mat Pts = Slot.Curve->Sample(60);
		for (int32 i = 0; i < Pts.rows(); ++i) AllY.Add(Pts(i, 1) * Sy);
		Mat Poles = Slot.Curve->poles();
		for (int32 i = 0; i < Poles.rows(); ++i) AllY.Add(Poles(i, 1) * Sy);
		if (Slot.bFreeX)
		{
			double lo = TNumericLimits<double>::Max(), hi = -TNumericLimits<double>::Max();
			for (int32 i = 0; i < Pts.rows(); ++i) { lo = FMath::Min(lo, Pts(i, 0) * Sx); hi = FMath::Max(hi, Pts(i, 0) * Sx); }
			for (int32 i = 0; i < Poles.rows(); ++i) { lo = FMath::Min(lo, Poles(i, 0) * Sx); hi = FMath::Max(hi, Poles(i, 0) * Sx); }
			const double Pad = 0.12 * FMath::Max(hi - lo, 1e-9);
			XLo = FMath::Min(XLo, lo - Pad);
			XHi = FMath::Max(XHi, hi + Pad);
		}
	}
	if (!(XHi > XLo) || !FMath::IsFinite(XLo) || !FMath::IsFinite(XHi))
	{
		XLo = -0.05 * Sx;
		XHi = 1.08 * Sx;
	}

	double YMin = 0.0, YMax = bEqualAspect ? 0.5 * Sy : 0.5;
	if (AllY.Num() > 0)
	{
		YMin = AllY[0];
		YMax = AllY[0];
		for (double y : AllY) { YMin = FMath::Min(YMin, y); YMax = FMath::Max(YMax, y); }
	}
	const double YRange = FMath::Max(bEqualAspect ? 0.5 * Sy : 0.5, YMax - YMin);
	const double Margin = YRange * 0.2;
	return FBox2D(FVector2D(XLo, YMin - Margin), FVector2D(XHi, YMax + Margin));
}

FBox2D SCurveEditor::Equalized(const FBox2D& Box, FVector2D PaneSizePx) const
{
	if (PaneSizePx.X < 1.0 || PaneSizePx.Y < 1.0)
	{
		return Box;
	}
	const double XSpan = Box.Max.X - Box.Min.X, YSpan = Box.Max.Y - Box.Min.Y;
	FBox2D Out = Box;
	if (XSpan / PaneSizePx.X < YSpan / PaneSizePx.Y)
	{
		const double Want = YSpan * PaneSizePx.X / PaneSizePx.Y;
		const double Mid = 0.5 * (Box.Min.X + Box.Max.X);
		Out.Min.X = Mid - 0.5 * Want;
		Out.Max.X = Mid + 0.5 * Want;
	}
	else
	{
		const double Want = XSpan * PaneSizePx.Y / PaneSizePx.X;
		const double Mid = 0.5 * (Box.Min.Y + Box.Max.Y);
		Out.Min.Y = Mid - 0.5 * Want;
		Out.Max.Y = Mid + 0.5 * Want;
	}
	return Out;
}

FBox2D SCurveEditor::GetOrComputeViewBoxForPane(FVector2D PaneSizePx)
{
	FitBox = ComputeFitBox();
	if (!ViewRequest.IsSet())
	{
		ViewRequest = FitBox;
	}
	FBox2D Box = LimLock.IsSet() ? LimLock.GetValue() : ViewRequest.GetValue();
	if (bEqualAspect)
	{
		Box = Equalized(Box, PaneSizePx);
	}
	return Box;
}

// ---- pane-facing hit testing / editing ----

TOptional<FIntPoint> SCurveEditor::PoleAt(FVector2D Local, const FBox2D& Box, FVector2D SizePx) const
{
	struct FHit { double d; int32 k, i; };
	TArray<FHit> Hits;
	for (int32 k = 0; k < Slots.Num(); ++k)
	{
		const FCurveEditorSlot& Slot = Slots[k];
		if (Slot.Curve == nullptr) continue;
		Mat Poles = Slot.Curve->poles();
		for (int32 i = 0; i < Poles.rows(); ++i)
		{
			const FVector2D Px = DataToLocalStatic(FVector2D(Poles(i, 0) * XScale, Poles(i, 1) * YScale), Box, SizePx);
			const double d = FVector2D::Distance(Px, Local);
			if (d < 10.0) Hits.Add({ d, k, i });
		}
	}
	if (Hits.Num() == 0)
	{
		return TOptional<FIntPoint>();
	}
	Hits.Sort([](const FHit& A, const FHit& B) { return A.d < B.d; });
	TArray<FIntPoint> Tied;
	for (const FHit& H : Hits)
	{
		if (H.d - Hits[0].d < 1.0) Tied.Add(FIntPoint(H.k, H.i));
	}
	if (Tied.Num() > 1 && LastPoleHit.IsSet())
	{
		const int32 Idx = Tied.IndexOfByKey(LastPoleHit.GetValue());
		if (Idx != INDEX_NONE)
		{
			return Tied[(Idx + 1) % Tied.Num()];
		}
	}
	return Tied[0];
}

TOptional<int32> SCurveEditor::KnotAt(int32 SlotIndex, FVector2D Local, const FBox2D& Box, FVector2D SizePx) const
{
	if (!Slots.IsValidIndex(SlotIndex) || Slots[SlotIndex].Curve == nullptr)
	{
		return TOptional<int32>();
	}
	struct FHit { bool bPinned; double d; TOptional<int32> idx; };
	TArray<FHit> Hits;
	for (const FKnotStack& Stack : ComputeKnotStacks(*Slots[SlotIndex].Curve))
	{
		for (int32 j = 0; j < Stack.Indices.Num(); ++j)
		{
			const double MarkerY = -0.06 - 0.11 * j;
			const FVector2D Px = DataToLocalStatic(FVector2D(Stack.Value, MarkerY), Box, SizePx);
			const double d = FVector2D::Distance(Px, Local);
			if (d < 10.0) Hits.Add({ !Stack.Indices[j].IsSet(), d, Stack.Indices[j] });
		}
	}
	if (Hits.Num() == 0)
	{
		return TOptional<int32>();
	}
	Hits.Sort([](const FHit& A, const FHit& B) { if (A.bPinned != B.bPinned) return !A.bPinned; return A.d < B.d; });
	return Hits[0].idx;
}

void SCurveEditor::SelectBasisFunction(int32 SlotIndex, FVector2D Local, const FBox2D& Box, FVector2D SizePx)
{
	if (!Slots.IsValidIndex(SlotIndex) || Slots[SlotIndex].Curve == nullptr)
	{
		Selected.Reset();
		return;
	}
	std::vector<double> S(65);
	for (int32 i = 0; i < 65; ++i) S[i] = (double)i / 64.0;
	Mat R = Slots[SlotIndex].Curve->BasisMatrix(S, true);
	int32 Best = -1;
	double BestD = 10.0;
	for (int32 i = 0; i < R.cols(); ++i)
	{
		for (int32 j = 0; j < 65; ++j)
		{
			const FVector2D Px = DataToLocalStatic(FVector2D(S[j], R(j, i)), Box, SizePx);
			const double d = FVector2D::Distance(Px, Local);
			if (d < BestD) { BestD = d; Best = i; }
		}
	}
	Selected = Best >= 0 ? TOptional<FIntPoint>(FIntPoint(SlotIndex, Best)) : TOptional<FIntPoint>();
}

int32 SCurveEditor::InsertIndexFor(const FCurveEditorSlot& Slot, FVector2D DataPos, FVector2D Local, const FBox2D& Box, FVector2D SizePx) const
{
	Mat Poles = Slot.Curve->poles();
	const int32 n = Poles.rows();
	if (!Slot.Curve->periodic())
	{
		int32 Idx = 0;
		while (Idx < n && Poles(Idx, 0) < DataPos.X) ++Idx;
		return FMath::Clamp(Idx, 1, n - 1);
	}
	int32 Best = 0;
	double BestD = TNumericLimits<double>::Max();
	for (int32 i = 0; i < n; ++i)
	{
		const FVector2D A = DataToLocalStatic(FVector2D(Poles(i, 0) * XScale, Poles(i, 1) * YScale), Box, SizePx);
		const FVector2D B = DataToLocalStatic(FVector2D(Poles((i + 1) % n, 0) * XScale, Poles((i + 1) % n, 1) * YScale), Box, SizePx);
		const FVector2D AB = B - A;
		const double T = FMath::Clamp(FVector2D::DotProduct(Local - A, AB) / FMath::Max(FVector2D::DotProduct(AB, AB), 1e-12), 0.0, 1.0);
		const double d = FVector2D::Distance(Local, A + T * AB);
		if (d < BestD) { BestD = d; Best = i; }
	}
	return (Best + 1) % n;
}

void SCurveEditor::AddPoleAtLocal(FVector2D Local, const FBox2D& Box, FVector2D SizePx)
{
	if (Slots.Num() == 0) return;
	int32 Best = 0;
	double BestD = TNumericLimits<double>::Max();
	for (int32 k = 0; k < Slots.Num(); ++k)
	{
		if (Slots[k].Curve == nullptr) continue;
		Mat Pts = Slots[k].Curve->Sample(96);
		for (int32 i = 0; i < Pts.rows(); ++i)
		{
			const FVector2D Px = DataToLocalStatic(FVector2D(Pts(i, 0) * XScale, Pts(i, 1) * YScale), Box, SizePx);
			const double d = FVector2D::Distance(Px, Local);
			if (d < BestD) { BestD = d; Best = k; }
		}
	}
	FCurveEditorSlot& Slot = Slots[Best];
	if (Slot.Curve == nullptr) return;
	const FVector2D DataXY = LocalToDataStatic(Local, Box, SizePx);
	double X = DataXY.X / XScale;
	if (!Slot.bFreeX)
	{
		const double Lo = Slot.Curve->periodic() ? 0.0 : 1e-4;
		X = FMath::Clamp(X, Lo, 1.0 - 1e-4);
	}
	const double Y = DataXY.Y / YScale;
	const int32 Idx = InsertIndexFor(Slot, FVector2D(X, Y), Local, Box, SizePx);
	const FVector2D P = ConstrainPole(Slot, Idx, FVector2D(X, Y));
	try
	{
		NurbsAddPole(Slot.Curve, Idx, ToStdVec(P));
	}
	catch (const std::exception&)
	{
		return;
	}
	Selected.Reset();
	RequestRedraw();
	NotifyChanged();
}

void SCurveEditor::DeselectAll()
{
	Selected.Reset();
}

void SCurveEditor::BeginDragPole(FIntPoint Hit, const FBox2D& CurrentBox)
{
	LastPoleHit = Hit;
	TOptional<FIntPoint> Target = TargetOf(Hit);
	if (Target.IsSet())
	{
		Hit = Target.GetValue();
	}
	if (Slots[Hit.X].FixedPole(Hit.Y).IsSet())
	{
		Selected = Hit;
		RequestRedraw();
		return;
	}
	DraggingPole = Hit;
	bDragMoved = false;
	LimLock = CurrentBox;
}

void SCurveEditor::DragPoleTo(FVector2D Local, const FBox2D& Box, FVector2D SizePx)
{
	if (!DraggingPole.IsSet()) return;
	const FVector2D DataXY = LocalToDataStatic(Local, Box, SizePx);
	MovePole(DraggingPole->X, DraggingPole->Y, FVector2D(DataXY.X / XScale, DataXY.Y / YScale));
	bDragMoved = true;
}

void SCurveEditor::ReleasePole()
{
	if (!DraggingPole.IsSet()) return;
	const FIntPoint Dragging = DraggingPole.GetValue();
	const bool bMoved = bDragMoved;
	DraggingPole.Reset();
	bDragMoved = false;
	LimLock.Reset();
	if (!bMoved)
	{
		Selected = Dragging;
		RequestRedraw();
	}
	else
	{
		RequestRedraw();
		NotifyChanged();
	}
}

void SCurveEditor::DragKnotTo(int32 SlotIndex, FVector2D Local, const FBox2D& Box, FVector2D SizePx)
{
	if (!DraggingKnot.IsSet() || !Slots.IsValidIndex(SlotIndex) || Slots[SlotIndex].Curve == nullptr) return;
	const FVector2D DataXY = LocalToDataStatic(Local, Box, SizePx);
	TArray<FKnotStack> Stacks = ComputeKnotStacks(*Slots[SlotIndex].Curve);
	const int32 EditIdx = DraggingKnot->Y;
	int32 StackPos = INDEX_NONE;
	for (int32 s = 0; s < Stacks.Num() && StackPos == INDEX_NONE; ++s)
	{
		for (const TOptional<int32>& Idx : Stacks[s].Indices)
		{
			if (Idx.IsSet() && Idx.GetValue() == EditIdx) { StackPos = s; break; }
		}
	}
	if (StackPos == INDEX_NONE) return;
	const double Lo = StackPos > 0 ? Stacks[StackPos - 1].Value + 2e-3 : 0.0;
	const double Hi = StackPos < Stacks.Num() - 1 ? Stacks[StackPos + 1].Value - 2e-3 : 1.0;
	const double NewVal = FMath::Clamp(DataXY.X, Lo, Hi);
	try
	{
		NurbsChangeKnot(Slots[SlotIndex].Curve, EditIdx, NewVal);
	}
	catch (const std::exception&)
	{
		return;
	}
	RequestRedraw();
}

void SCurveEditor::ReleaseKnot()
{
	if (!DraggingKnot.IsSet()) return;
	DraggingKnot.Reset();
	NotifyChanged();
}

void SCurveEditor::ZoomAboutLocal(FVector2D Local, const FBox2D& Box, FVector2D SizePx, float WheelDelta)
{
	const double K = FMath::Pow(1.12, -(double)WheelDelta);
	const FVector2D DataXY = LocalToDataStatic(Local, Box, SizePx);
	const FBox2D PreBox = ViewRequest.IsSet() ? ViewRequest.GetValue() : FitBox;
	const double Span = FMath::Max(PreBox.Max.X - PreBox.Min.X, PreBox.Max.Y - PreBox.Min.Y) * K;
	if (Span < 5e-3 || Span > 50.0 * XScale) return;
	const double XLo = DataXY.X + (PreBox.Min.X - DataXY.X) * K, XHi = DataXY.X + (PreBox.Max.X - DataXY.X) * K;
	const double YLo = DataXY.Y + (PreBox.Min.Y - DataXY.Y) * K, YHi = DataXY.Y + (PreBox.Max.Y - DataXY.Y) * K;
	ViewRequest = FBox2D(FVector2D(XLo, YLo), FVector2D(XHi, YHi));
}

// ---- edits ----

void SCurveEditor::OnPoleWeightEdited(int32 SlotIndex, int32 PoleIndex, double NewWeight)
{
	Slots[SlotIndex].Curve->SetPoleWeight(PoleIndex, NewWeight);
	RequestRedraw();
	NotifyChanged();
}

void SCurveEditor::RemovePole(int32 SlotIndex, int32 PoleIndex)
{
	NurbsDeletePole(Slots[SlotIndex].Curve, PoleIndex);
	Selected.Reset();
	RequestRedraw();
	NotifyChanged();
}

void SCurveEditor::InsertKnot(int32 SlotIndex, double Value)
{
	try
	{
		NurbsInsertKnot(Slots[SlotIndex].Curve, Value);
	}
	catch (const std::exception&)
	{
		return;
	}
	Selected.Reset();
	RequestRedraw();
	NotifyChanged();
}

void SCurveEditor::DeleteKnot(int32 SlotIndex, int32 EditIndex)
{
	try
	{
		NurbsDeleteKnot(Slots[SlotIndex].Curve, EditIndex);
	}
	catch (const std::exception&)
	{
		return;
	}
	Selected.Reset();
	RequestRedraw();
	NotifyChanged();
}

void SCurveEditor::ResetKnotsForSlot(int32 SlotIndex)
{
	Slots[SlotIndex].Curve->ResetKnotsPublic();
	RequestRedraw();
	NotifyChanged();
}

// ---- context menus ----

void SCurveEditor::BuildPoleMenu(FMenuBuilder& MenuBuilder, FIntPoint Hit)
{
	const int32 k = Hit.X, i = Hit.Y;
	FCurveEditorSlot& Slot = Slots[k];
	if (Slot.FixedPole(i).IsSet())
	{
		MenuBuilder.AddMenuEntry(NSLOCTEXT("CurveEditor", "Fixed", "This point is fixed"), FText::GetEmpty(), FSlateIcon(),
			FUIAction(FExecuteAction(), FCanExecuteAction::CreateLambda([] { return false; })));
		return;
	}
	Mat PW = Slot.Curve->poles_and_weights();
	const double CurrentWeight = PW(i, 2);
	MenuBuilder.AddMenuEntry(NSLOCTEXT("CurveEditor", "SetWeight", "Set Weight..."), FText::GetEmpty(), FSlateIcon(),
		FUIAction(FExecuteAction::CreateLambda([this, k, i, CurrentWeight]()
		{
			TOptional<double> Result = PromptForDouble(
				NSLOCTEXT("CurveEditor", "SetWeightTitle", "Set Weight"),
				NSLOCTEXT("CurveEditor", "SetWeightLabel", "Weight (1.0 = neutral, >1 pulls closer, <1 pushes away):"),
				CurrentWeight, 0.01, 100.0, 0.05);
			if (Result.IsSet())
			{
				OnPoleWeightEdited(k, i, Result.GetValue());
			}
		})));
	if (CanRemovePole(Slot, k, i))
	{
		MenuBuilder.AddMenuEntry(NSLOCTEXT("CurveEditor", "RemovePole", "Remove Control Point"), FText::GetEmpty(), FSlateIcon(),
			FUIAction(FExecuteAction::CreateSP(this, &SCurveEditor::RemovePole, k, i)));
	}
	if (bSupportsUserLinks && Linkable(k))
	{
		TOptional<int32> End = EndOf(k, i);
		if (End.IsSet() && Partners(k, i).Num() == 0)
		{
			TOptional<FIntPoint> Target = TargetOf(FIntPoint(k, i));
			TArray<FIntPoint> Followers = FollowersOf(FIntPoint(k, i));
			MenuBuilder.AddMenuSeparator();
			if (Target.IsSet())
			{
				MenuBuilder.AddMenuEntry(NSLOCTEXT("CurveEditor", "Unlink", "Unlink from the linked point"), FText::GetEmpty(), FSlateIcon(),
					FUIAction(FExecuteAction::CreateSP(this, &SCurveEditor::UnlinkPole, k, i)));
			}
			else
			{
				for (FIntPoint F : Followers)
				{
					MenuBuilder.AddMenuEntry(NSLOCTEXT("CurveEditor", "Release", "Release the following point"), FText::GetEmpty(), FSlateIcon(),
						FUIAction(FExecuteAction::CreateSP(this, &SCurveEditor::UnlinkPole, F.X, F.Y)));
				}
				if (Followers.Num() == 0)
				{
					for (int32 k2 = 0; k2 < Slots.Num(); ++k2)
					{
						if (k2 == k || !Linkable(k2)) continue;
						const int32 j = (End.GetValue() == 0) ? 0 : Slots[k2].Curve->n_poles() - 1;
						if (TargetOf(FIntPoint(k2, j)).IsSet()) continue;
						MenuBuilder.AddMenuEntry(
							FText::Format(NSLOCTEXT("CurveEditor", "LinkTo", "Link to {0} (follow it)"), Slots[k2].Label),
							FText::GetEmpty(), FSlateIcon(),
							FUIAction(FExecuteAction::CreateSP(this, &SCurveEditor::LinkPole, k, i, k2, j)));
					}
				}
			}
		}
	}
}

void SCurveEditor::BuildKnotMenu(FMenuBuilder& MenuBuilder, int32 SlotIndex, TOptional<int32> KnotEditIndex, double ClickS)
{
	if (!Slots.IsValidIndex(SlotIndex) || Slots[SlotIndex].Curve == nullptr) return;
	const NurbsCurve* C = Slots[SlotIndex].Curve;
	const double S = FMath::Clamp(ClickS, 1e-4, 1.0 - 1e-4);
	MenuBuilder.AddMenuEntry(
		FText::Format(NSLOCTEXT("CurveEditor", "InsertKnot", "Insert knot at s = {0}"), FText::AsNumber(S)),
		FText::GetEmpty(), FSlateIcon(), FUIAction(FExecuteAction::CreateSP(this, &SCurveEditor::InsertKnot, SlotIndex, S)));
	if (KnotEditIndex.IsSet())
	{
		const int32 Idx = KnotEditIndex.GetValue();
		const bool bCanDelete = C->n_poles() > C->degree() + 1;
		MenuBuilder.AddMenuEntry(NSLOCTEXT("CurveEditor", "DeleteKnot", "Delete this knot"), FText::GetEmpty(), FSlateIcon(),
			FUIAction(FExecuteAction::CreateSP(this, &SCurveEditor::DeleteKnot, SlotIndex, Idx),
				FCanExecuteAction::CreateLambda([bCanDelete] { return bCanDelete; })));
	}
	MenuBuilder.AddMenuSeparator();
	MenuBuilder.AddMenuEntry(NSLOCTEXT("CurveEditor", "ResetKnots", "Reset knots"), FText::GetEmpty(), FSlateIcon(),
		FUIAction(FExecuteAction::CreateSP(this, &SCurveEditor::ResetKnotsForSlot, SlotIndex)));
}
