#include "SceneView.h"

#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Text/STextBlock.h"
#include "Styling/CoreStyle.h"
#include "Misc/CoreMisc.h"

#include "Standalone/AerotelicSceneViewActor.h"

namespace
{
	// The main view's constants (AerotelicCameraPawn.cpp).
	constexpr double kOrbitDegreesPerPixel = 0.35;
	constexpr double kZoomStep = 0.85;
	constexpr double kMinDistance = 0.05;
	constexpr double kMaxDistance = 1.0e6;
	constexpr int32 kMinTarget = 32;
	constexpr int32 kMaxTarget = 4096;

	UWorld* GameWorld()
	{
		if (GEngine == nullptr)
		{
			return nullptr;
		}
		for (const FWorldContext& Context : GEngine->GetWorldContexts())
		{
			if (Context.WorldType == EWorldType::Game && Context.World() != nullptr)
			{
				return Context.World();
			}
		}
		return nullptr;
	}
}

void SAerotelicSceneView::Construct(const FArguments& InArgs)
{
	FieldOfViewDeg = InArgs._FieldOfViewDeg;
	Brush.DrawAs = ESlateBrushDrawType::Image;
	Brush.ImageSize = FVector2f(64.0f, 64.0f);
	Brush.TintColor = FLinearColor::White;

	ChildSlot
	[
		SNew(SOverlay)
		+ SOverlay::Slot()
		[
			// A dark ground under the picture, for the frames before the
			// first capture lands (and for the Editor host, which has no
			// game world to capture).
			SNew(SBorder).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(FLinearColor(0.05f, 0.06f, 0.07f)).Padding(0)
			[
				SAssignNew(Image, SImage).Image(&Brush).Visibility(EVisibility::Collapsed)
			]
		]
		+ SOverlay::Slot()
		[
			InArgs._Overlay.Widget
		]
	];
}

SAerotelicSceneView::~SAerotelicSceneView()
{
	if (ViewActor.IsValid() && !IsEngineExitRequested())
	{
		ViewActor->Destroy();
	}
	ViewActor.Reset();
}

bool SAerotelicSceneView::EnsureActor()
{
	if (ViewActor.IsValid())
	{
		return true;
	}
	UWorld* World = GameWorld();
	if (World == nullptr)
	{
		return false;
	}
	FActorSpawnParameters Params;
	Params.ObjectFlags |= RF_Transient;
	AAerotelicSceneViewActor* A = World->SpawnActor<AAerotelicSceneViewActor>(FVector::ZeroVector, FRotator::ZeroRotator, Params);
	if (A == nullptr)
	{
		return false;
	}
	ViewActor = A;
	TArray<AActor*> Actors;
	for (const TWeakObjectPtr<AActor>& W : Shown)
	{
		if (W.IsValid()) Actors.Add(W.Get());
	}
	A->SetShownActors(Actors);
	if (RenderTarget.IsValid())
	{
		A->SetRenderTarget(RenderTarget.Get());
	}
	if (BackdropColor.IsSet())
	{
		A->SetBackdropColor(BackdropColor.GetValue());
	}
	bDirty = true;
	return true;
}

void SAerotelicSceneView::SetBackdropColor(const FLinearColor& Color)
{
	BackdropColor = Color;
	if (ViewActor.IsValid())
	{
		ViewActor->SetBackdropColor(Color);
	}
	bDirty = true;
}

void SAerotelicSceneView::EnsureTarget(const FIntPoint& Size)
{
	if (!RenderTarget.IsValid())
	{
		UTextureRenderTarget2D* RT = NewObject<UTextureRenderTarget2D>(GetTransientPackage(), NAME_None, RF_Transient);
		RT->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
		RT->ClearColor = FLinearColor(0.05f, 0.06f, 0.07f, 1.0f);
		RT->bAutoGenerateMips = false;
		RT->InitAutoFormat(Size.X, Size.Y);
		RT->UpdateResourceImmediate(true);
		RenderTarget.Reset(RT);
		TargetSize = Size;
		Brush.SetResourceObject(RT);
		Brush.ImageSize = FVector2f(static_cast<float>(Size.X), static_cast<float>(Size.Y));
		if (ViewActor.IsValid())
		{
			ViewActor->SetRenderTarget(RT);
		}
		if (Image.IsValid())
		{
			Image->SetVisibility(EVisibility::Visible);
		}
		bDirty = true;
	}
	else if (Size != TargetSize)
	{
		RenderTarget->ResizeTarget(Size.X, Size.Y);
		TargetSize = Size;
		Brush.ImageSize = FVector2f(static_cast<float>(Size.X), static_cast<float>(Size.Y));
		bDirty = true;
	}
}

void SAerotelicSceneView::Tick(const FGeometry& AllottedGeometry, const double InCurrentTime, const float InDeltaTime)
{
	SCompoundWidget::Tick(AllottedGeometry, InCurrentTime, InDeltaTime);
	if (!EnsureActor())
	{
		return;
	}
	const FVector2D Px = AllottedGeometry.GetAbsoluteSize();
	const FIntPoint Size(
		FMath::Clamp(FMath::RoundToInt(Px.X), kMinTarget, kMaxTarget),
		FMath::Clamp(FMath::RoundToInt(Px.Y), kMinTarget, kMaxTarget));
	EnsureTarget(Size);
	const double Now = FPlatformTime::Seconds();
	if (bDirty)
	{
		bDirty = false;
		CaptureUntilSeconds = Now + 1.5;
	}
	if (Now < CaptureUntilSeconds && ViewActor.IsValid())
	{
		ViewActor->SetCamera(FocusPoint, Distance, Yaw, Pitch, FieldOfViewDeg);
		ViewActor->RequestCapture();
	}
}

void SAerotelicSceneView::SetShownActors(const TArray<AActor*>& Actors)
{
	Shown.Reset();
	for (AActor* A : Actors)
	{
		Shown.Add(A);
	}
	if (ViewActor.IsValid())
	{
		ViewActor->SetShownActors(Actors);
	}
	bDirty = true;
}

void SAerotelicSceneView::FrameBox(const FBox& Box)
{
	if (!Box.IsValid)
	{
		return;
	}
	FocusPoint = Box.GetCenter();
	const double Radius = FMath::Max(Box.GetExtent().Size(), 1e-3);
	Distance = FMath::Clamp(Radius / FMath::Sin(FMath::DegreesToRadians(FieldOfViewDeg * 0.5)) * 1.1, kMinDistance, kMaxDistance);
	bDirty = true;
}

void SAerotelicSceneView::FrameShown()
{
	FBox All(ForceInit);
	for (const TWeakObjectPtr<AActor>& W : Shown)
	{
		if (W.IsValid())
		{
			const FBox B = W->GetComponentsBoundingBox(true);
			if (B.IsValid) All += B;
		}
	}
	FrameBox(All);
}

void SAerotelicSceneView::SetView(double InYaw, double InPitch)
{
	Yaw = InYaw;
	Pitch = FMath::Clamp(InPitch, -89.0, 89.0);
	bDirty = true;
}

void SAerotelicSceneView::Orbit(double DeltaX, double DeltaY)
{
	Yaw = FMath::Fmod(Yaw + DeltaX * kOrbitDegreesPerPixel, 360.0);
	Pitch = FMath::Clamp(Pitch + DeltaY * kOrbitDegreesPerPixel, -89.0, 89.0);
	bDirty = true;
}

void SAerotelicSceneView::Pan(double DeltaX, double DeltaY, const FVector2D& ViewSize)
{
	const double ViewHeightWorld = 2.0 * Distance * FMath::Tan(FMath::DegreesToRadians(FieldOfViewDeg * 0.5));
	const double UnitsPerPixel = ViewHeightWorld / FMath::Max(ViewSize.Y, 1.0);
	const FRotationMatrix Frame(FRotator(Pitch, Yaw, 0.0));
	const FVector Right = Frame.GetUnitAxis(EAxis::Y);
	const FVector Up = Frame.GetUnitAxis(EAxis::Z);
	FocusPoint += (-Right * DeltaX + Up * DeltaY) * UnitsPerPixel;
	bDirty = true;
}

void SAerotelicSceneView::Zoom(double Factor)
{
	Distance = FMath::Clamp(Distance * Factor, kMinDistance, kMaxDistance);
	bDirty = true;
}

FReply SAerotelicSceneView::OnMouseButtonDown(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent)
{
	const FKey Button = MouseEvent.GetEffectingButton();
	if (Button != EKeys::LeftMouseButton && Button != EKeys::RightMouseButton && Button != EKeys::MiddleMouseButton)
	{
		return FReply::Unhandled();
	}
	bDragging = true;
	bDragIsLeft = Button == EKeys::LeftMouseButton;
	return FReply::Handled().CaptureMouse(SharedThis(this)).SetUserFocus(SharedThis(this), EFocusCause::Mouse);
}

FReply SAerotelicSceneView::OnMouseButtonUp(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent)
{
	if (!bDragging)
	{
		return FReply::Unhandled();
	}
	bDragging = false;
	return FReply::Handled().ReleaseMouseCapture();
}

FReply SAerotelicSceneView::OnMouseMove(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent)
{
	if (!bDragging || !HasMouseCapture())
	{
		return FReply::Unhandled();
	}
	// Deltas in screen pixels, as the main view's are.
	const FVector2D Delta = MouseEvent.GetCursorDelta();
	if (bDragIsLeft && MouseEvent.IsMouseButtonDown(EKeys::LeftMouseButton))
	{
		Orbit(Delta.X, Delta.Y);
	}
	else if (MouseEvent.IsMouseButtonDown(EKeys::RightMouseButton) || MouseEvent.IsMouseButtonDown(EKeys::MiddleMouseButton))
	{
		Pan(Delta.X, Delta.Y, MyGeometry.GetAbsoluteSize());
	}
	return FReply::Handled();
}

FReply SAerotelicSceneView::OnMouseWheel(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent)
{
	if (MouseEvent.GetWheelDelta() > 0.0f)
	{
		Zoom(kZoomStep);
	}
	else if (MouseEvent.GetWheelDelta() < 0.0f)
	{
		Zoom(1.0 / kZoomStep);
	}
	return FReply::Handled();
}

FReply SAerotelicSceneView::OnKeyDown(const FGeometry& MyGeometry, const FKeyEvent& InKeyEvent)
{
	if (InKeyEvent.GetKey() == EKeys::Home || InKeyEvent.GetKey() == EKeys::F)
	{
		FrameShown();
		return FReply::Handled();
	}
	return FReply::Unhandled();
}

FCursorReply SAerotelicSceneView::OnCursorQuery(const FGeometry& MyGeometry, const FPointerEvent& CursorEvent) const
{
	return FCursorReply::Cursor(bDragging ? EMouseCursor::GrabHandClosed : EMouseCursor::Default);
}
