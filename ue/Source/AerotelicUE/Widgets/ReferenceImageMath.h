#pragma once

#include "CoreMinimal.h"
#include "aerotelic/reference_image.hpp"

// The geometry of core/reference_image.py's ReferenceImage, over the saved
// ReferenceImageSpec plus the source's pixel size. Everything is in the
// editor's display units; y grows upward, rows grow downward.
namespace ReferenceImageMath
{
	using FSpec = aerotelic::ReferenceImageSpec;

	inline double Width(const FSpec& S, FIntPoint Size) { return Size.X * S.scale; }
	inline double Height(const FSpec& S, FIntPoint Size) { return Size.Y * S.scale; }

	// Pivot of the rotation: the centre of the WHOLE source, so a change of
	// crop cannot swing the pixels that remain.
	inline FVector2D Centre(const FSpec& S, FIntPoint Size)
	{
		return FVector2D(S.x + 0.5 * Width(S, Size), S.y - 0.5 * Height(S, Size));
	}

	// (left, right, bottom, top) of the visible (cropped) rectangle, in the
	// image's own unrotated frame. Note crop is stored (left, top, right,
	// bottom) as source fractions.
	struct FExtent { double Left, Right, Bottom, Top; };
	inline FExtent Extent(const FSpec& S, FIntPoint Size)
	{
		const double W = Width(S, Size), H = Height(S, Size);
		return { S.x + S.crop[0] * W, S.x + S.crop[2] * W, S.y - S.crop[3] * H, S.y - S.crop[1] * H };
	}

	inline FVector2D Rotate(FVector2D P, FVector2D About, double AngleDeg)
	{
		const double A = FMath::DegreesToRadians(AngleDeg);
		const double C = FMath::Cos(A), Sn = FMath::Sin(A);
		const FVector2D D = P - About;
		return About + FVector2D(C * D.X - Sn * D.Y, Sn * D.X + C * D.Y);
	}
	inline FVector2D ToWorld(const FSpec& S, FIntPoint Size, FVector2D Local) { return Rotate(Local, Centre(S, Size), S.angle); }
	inline FVector2D ToLocal(const FSpec& S, FIntPoint Size, FVector2D World) { return Rotate(World, Centre(S, Size), -S.angle); }

	// Visible corners, world frame, counter-clockwise from bottom-left.
	inline void Corners(const FSpec& S, FIntPoint Size, FVector2D Out[4])
	{
		const FExtent E = Extent(S, Size);
		Out[0] = ToWorld(S, Size, FVector2D(E.Left, E.Bottom));
		Out[1] = ToWorld(S, Size, FVector2D(E.Right, E.Bottom));
		Out[2] = ToWorld(S, Size, FVector2D(E.Right, E.Top));
		Out[3] = ToWorld(S, Size, FVector2D(E.Left, E.Top));
	}
	inline FVector2D CropCentre(const FSpec& S, FIntPoint Size)
	{
		const FExtent E = Extent(S, Size);
		return ToWorld(S, Size, FVector2D(0.5 * (E.Left + E.Right), 0.5 * (E.Bottom + E.Top)));
	}

	inline void Translate(FSpec& S, FVector2D Delta) { S.x += Delta.X; S.y += Delta.Y; }

	// Scales by Factor about a world point (kept where it is).
	inline void ScaleAbout(FSpec& S, FIntPoint Size, double Factor, FVector2D About)
	{
		// Scale about the anchor in the image's own (unrotated) frame, then
		// put the anchor back where it was in the world: the rotation pivot
		// (the source's centre) moved with the scale.
		const FVector2D L = ToLocal(S, Size, About);
		S.scale *= Factor;
		S.x = L.X + (S.x - L.X) * Factor;
		S.y = L.Y + (S.y - L.Y) * Factor;
		const FVector2D After = ToWorld(S, Size, L);
		Translate(S, About - After);
	}

	// Moves one crop edge to the world point, clamped to the source and to
	// the opposite edge; dragging back out un-crops.
	inline void SetCropEdgeAt(FSpec& S, FIntPoint Size, const FString& Side, FVector2D World)
	{
		const FVector2D L = ToLocal(S, Size, World);
		const double W = FMath::Max(Width(S, Size), 1e-12), H = FMath::Max(Height(S, Size), 1e-12);
		const double Min = aerotelic::kReferenceImageMinCropFraction;
		if (Side == TEXT("left"))        S.crop[0] = FMath::Clamp((L.X - S.x) / W, 0.0, S.crop[2] - Min);
		else if (Side == TEXT("right"))  S.crop[2] = FMath::Clamp((L.X - S.x) / W, S.crop[0] + Min, 1.0);
		else if (Side == TEXT("top"))    S.crop[1] = FMath::Clamp((S.y - L.Y) / H, 0.0, S.crop[3] - Min);
		else if (Side == TEXT("bottom")) S.crop[3] = FMath::Clamp((S.y - L.Y) / H, S.crop[1] + Min, 1.0);
	}

	// Sets the angle holding Pivot (a world point, the visible centre by
	// default) exactly where it is: turn, then translate -- exact, because a
	// translation commutes with a rotation about the image's own centre.
	inline void RotateTo(FSpec& S, FIntPoint Size, double AngleDeg, TOptional<FVector2D> Pivot = TOptional<FVector2D>())
	{
		const FVector2D P = Pivot.IsSet() ? Pivot.GetValue() : CropCentre(S, Size);
		const FVector2D LocalP = ToLocal(S, Size, P);
		S.angle = AngleDeg;
		const FVector2D After = ToWorld(S, Size, LocalP);
		Translate(S, P - After);
	}

	// The whole source spans x/L X0..X0+WidthUnits, vertically centred on
	// CentreY, level.
	inline void FitWidth(FSpec& S, FIntPoint Size, double WidthUnits = 1.0, double CentreY = 0.0, double X0 = 0.0)
	{
		S.scale = WidthUnits / FMath::Max(Size.X, 1);
		S.x = X0;
		S.y = CentreY + 0.5 * Height(S, Size);
		S.angle = 0.0;
	}
	inline void ResetCrop(FSpec& S) { S.crop = { 0.0, 0.0, 1.0, 1.0 }; }
}
