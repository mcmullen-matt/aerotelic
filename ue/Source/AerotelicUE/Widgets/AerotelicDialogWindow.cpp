#include "AerotelicDialogWindow.h"

#include "Widgets/SWindow.h"
#include "Framework/Application/SlateApplication.h"
#include "Math/UnrealMathUtility.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Widgets/SViewport.h"

DEFINE_LOG_CATEGORY_STATIC(LogAerotelicDialog, Log, All);

namespace
{
	// Width of the frame's left column (vertical toolbar + scene tree).
	// Dialogs (Transform) open just to the right of it so they never land on
	// top of the 3D view's centre, where the model is framed. Kept in sync
	// with kViewToolbarWidth + kSceneTreeWidth in MainShell/AerotelicMainShell.cpp.
	constexpr float kShellPanelWidth = 260.0f;

	// Gap between the panel's right edge (and the window's top edge) and the
	// first dialog, and the diagonal step between successive dialogs so a
	// stack of open editors stays legible instead of overlapping exactly.
	constexpr float kEdgeMargin = 16.0f;
	constexpr float kCascadeStep = 32.0f;
	constexpr int32 kCascadeWrap = 6;

	// Advances once per dialog opened; wraps so the cascade restarts near the
	// top-left rather than marching off the bottom of the screen.
	int32 GDialogCascadeIndex = 0;

	// Cascade position to the right of the control panel, clamped to stay
	// inside the main window's bounds and never back over the panel.
	FVector2D NextDialogPosition(const TSharedRef<SWindow>& MainWindow, FVector2D ClientSize)
	{
		const FVector2D MainPos(MainWindow->GetPositionInScreen());
		const FVector2D MainSize(MainWindow->GetSizeInScreen());
		const float Step = static_cast<float>(GDialogCascadeIndex % kCascadeWrap) * kCascadeStep;

		FVector2D Pos(
			MainPos.X + kShellPanelWidth + kEdgeMargin + Step,
			MainPos.Y + kEdgeMargin + Step);

		const float MaxX = MainPos.X + FMath::Max(MainSize.X - ClientSize.X - kEdgeMargin, kShellPanelWidth + kEdgeMargin);
		const float MaxY = MainPos.Y + FMath::Max(MainSize.Y - ClientSize.Y - kEdgeMargin, kEdgeMargin);
		Pos.X = FMath::Min(Pos.X, MaxX);
		Pos.Y = FMath::Min(Pos.Y, MaxY);
		return Pos;
	}
}

// In the standalone game the window that hosts the viewport widget is the
// reliable handle -- GameViewport->GetWindow(), GetActiveTopLevelWindow() and
// the top-level-window list all come back empty in -game mode, which is why an
// earlier attempt fell through to Slate's centre-of-screen default. The active
// window and the top-level list remain as fallbacks for the Editor host.
TSharedPtr<SWindow> FindAerotelicMainWindow()
{
	FSlateApplication& App = FSlateApplication::Get();

	if (GEngine != nullptr && GEngine->GameViewport != nullptr)
	{
		if (TSharedPtr<SViewport> ViewportWidget = GEngine->GameViewport->GetGameViewportWidget())
		{
			if (TSharedPtr<SWindow> HostWindow = App.FindWidgetWindow(ViewportWidget.ToSharedRef()))
			{
				return HostWindow;
			}
		}
		if (TSharedPtr<SWindow> ViewportWindow = GEngine->GameViewport->GetWindow())
		{
			return ViewportWindow;
		}
	}
	if (TSharedPtr<SWindow> Active = App.GetActiveTopLevelWindow())
	{
		return Active;
	}
	const TArray<TSharedRef<SWindow>> TopWindows = App.GetTopLevelWindows();
	return TopWindows.Num() > 0 ? TSharedPtr<SWindow>(TopWindows[0]) : TSharedPtr<SWindow>();
}

void ShowAerotelicDialogWindow(const TSharedRef<SWindow>& Window, FVector2D ClientSize)
{
	const TSharedPtr<SWindow> MainWindow = FindAerotelicMainWindow();

	// Create the native window first, then move it: the auto-centre default
	// is applied during creation, so positioning afterwards is what reliably
	// sticks across both the standalone game and the Editor host.
	FSlateApplication::Get().AddWindow(Window);

	if (MainWindow.IsValid())
	{
		const FVector2D Pos = NextDialogPosition(MainWindow.ToSharedRef(), ClientSize);
		Window->MoveWindowTo(Pos);
		UE_LOG(LogAerotelicDialog, Log, TEXT("Dialog '%s' placed at (%.0f, %.0f)"), *Window->GetTitle().ToString(), Pos.X, Pos.Y);
	}
	else
	{
		UE_LOG(LogAerotelicDialog, Warning, TEXT("Dialog '%s': no main window found; left at auto-centre"), *Window->GetTitle().ToString());
	}

	++GDialogCascadeIndex;
}

void OpenAerotelicDialogWindow(const FText& Title, const TSharedRef<SWidget>& Content, FVector2D ClientSize)
{
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(Title)
		.ClientSize(ClientSize)
		.SupportsMaximize(true)
		.SupportsMinimize(true);

	Window->SetContent(Content);
	ShowAerotelicDialogWindow(Window, ClientSize);
}
