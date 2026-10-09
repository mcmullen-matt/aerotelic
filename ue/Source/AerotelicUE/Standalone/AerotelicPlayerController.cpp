#include "AerotelicPlayerController.h"

#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Engine/LocalPlayer.h"
#include "InputKeyEventArgs.h"
#include "Widgets/SViewport.h"
#include "Framework/Application/SlateApplication.h"

#include "MainShell/AerotelicMainShell.h"
#include "MainShell/AerotelicDesignDock.h"

DEFINE_LOG_CATEGORY_STATIC(LogAerotelicFrame, Log, All);

AAerotelicPlayerController::AAerotelicPlayerController()
{
	PrimaryActorTick.bCanEverTick = true;
	bShowMouseCursor = true;
	bEnableClickEvents = false;
	bEnableMouseOverEvents = false;
}

void AAerotelicPlayerController::BeginPlay()
{
	Super::BeginPlay();

	// Cursor always visible; the 3D view captures the mouse only while a
	// button is held over it (so a drag that starts there keeps orbiting
	// even if the cursor crosses the frame), and releases it otherwise so
	// the frame and the dialog windows get their clicks.
	FInputModeGameAndUI InputMode;
	InputMode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
	InputMode.SetHideCursorDuringCapture(false);
	SetInputMode(InputMode);
	bShowMouseCursor = true;

	if (GEngine != nullptr && GEngine->GameViewport != nullptr && !Shell.IsValid())
	{
		SAssignNew(Shell, SAerotelicMainShell);
		GEngine->GameViewport->AddViewportWidgetContent(Shell.ToSharedRef(), 100);
	}
}

void AAerotelicPlayerController::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	TearDownAerotelicDesignDock();
	if (Shell.IsValid() && GEngine != nullptr && GEngine->GameViewport != nullptr)
	{
		GEngine->GameViewport->RemoveViewportWidgetContent(Shell.ToSharedRef());
	}
	Shell.Reset();
	// Give the whole viewport back.
	if (GEngine != nullptr && GEngine->GameViewport != nullptr)
	{
		const int32 Config = static_cast<int32>(ESplitScreenType::None);
		if (GEngine->GameViewport->SplitscreenInfo.IsValidIndex(Config) && GEngine->GameViewport->SplitscreenInfo[Config].PlayerData.Num() > 0)
		{
			FPerPlayerSplitscreenData& PD = GEngine->GameViewport->SplitscreenInfo[Config].PlayerData[0];
			PD.OriginX = 0.f; PD.OriginY = 0.f; PD.SizeX = 1.f; PD.SizeY = 1.f;
		}
	}
	Super::EndPlay(EndPlayReason);
}

void AAerotelicPlayerController::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	UpdateViewRect();
}

bool AAerotelicPlayerController::InputKey(const FInputKeyEventArgs& Params)
{
	if (Shell.IsValid() && !Params.Key.IsMouseButton() && !Params.Key.IsGamepadKey()
		&& (Params.Event == IE_Pressed || Params.Event == IE_Repeat)
		&& FSlateApplication::IsInitialized())
	{
		if (Shell->ProcessShortcut(Params.Key, FSlateApplication::Get().GetModifierKeys(), Params.Event == IE_Repeat))
		{
			return true;
		}
	}
	return Super::InputKey(Params);
}

void AAerotelicPlayerController::UpdateViewRect()
{
	if (!Shell.IsValid() || GEngine == nullptr || GEngine->GameViewport == nullptr)
	{
		return;
	}
	ULocalPlayer* LP = GetLocalPlayer();
	TSharedPtr<SViewport> ViewportWidget = GEngine->GameViewport->GetGameViewportWidget();
	if (LP == nullptr || !ViewportWidget.IsValid())
	{
		return;
	}

	const FGeometry& VG = ViewportWidget->GetCachedGeometry();
	const FVector2D ViewportPos(VG.GetAbsolutePosition());
	const FVector2D ViewportSize(VG.GetAbsoluteSize());
	FVector2D HolePos, HoleSize;
	if (ViewportSize.X <= 1.0 || ViewportSize.Y <= 1.0 || !Shell->GetViewHoleRect(HolePos, HoleSize))
	{
		return;
	}

	FVector2D Origin = (HolePos - ViewportPos) / ViewportSize;
	FVector2D Size = HoleSize / ViewportSize;
	Origin.X = FMath::Clamp(Origin.X, 0.0, 1.0);
	Origin.Y = FMath::Clamp(Origin.Y, 0.0, 1.0);
	Size.X = FMath::Clamp(Size.X, 0.01, 1.0 - Origin.X);
	Size.Y = FMath::Clamp(Size.Y, 0.01, 1.0 - Origin.Y);

	// UGameViewportClient::LayoutPlayers() rewrites every local player's
	// Origin/Size from this table before each draw, so the table is where
	// the rectangle has to live -- setting LP->Origin/Size directly is
	// undone a frame later.
	UGameViewportClient* GV = GEngine->GameViewport;
	const int32 Config = static_cast<int32>(ESplitScreenType::None);
	if (!GV->SplitscreenInfo.IsValidIndex(Config) || GV->SplitscreenInfo[Config].PlayerData.Num() == 0)
	{
		return;
	}
	FPerPlayerSplitscreenData& PD = GV->SplitscreenInfo[Config].PlayerData[0];
	if (!FMath::IsNearlyEqual(PD.OriginX, (float)Origin.X, 1e-4f) || !FMath::IsNearlyEqual(PD.OriginY, (float)Origin.Y, 1e-4f) ||
		!FMath::IsNearlyEqual(PD.SizeX, (float)Size.X, 1e-4f) || !FMath::IsNearlyEqual(PD.SizeY, (float)Size.Y, 1e-4f))
	{
		PD.OriginX = Origin.X;
		PD.OriginY = Origin.Y;
		PD.SizeX = Size.X;
		PD.SizeY = Size.Y;
		LP->Origin = Origin;
		LP->Size = Size;
		UE_LOG(LogAerotelicFrame, Log, TEXT("3D view rect: origin (%.3f, %.3f) size (%.3f, %.3f) of %.0fx%.0f"),
			Origin.X, Origin.Y, Size.X, Size.Y, ViewportSize.X, ViewportSize.Y);
	}
}
