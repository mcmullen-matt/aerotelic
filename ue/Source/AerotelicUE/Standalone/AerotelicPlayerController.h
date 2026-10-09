#pragma once

#include "CoreMinimal.h"
#include "GameFramework/PlayerController.h"

#include "AerotelicPlayerController.generated.h"

class SAerotelicMainShell;

// The standalone application's player controller: shows the OS cursor,
// routes mouse input to both the Slate frame and the 3D view (GameAndUI
// input mode, capture only while a button is held), puts the application
// frame (SAerotelicMainShell) over the game viewport, and confines the 3D
// rendering to the frame's view hole.
UCLASS()
class AAerotelicPlayerController : public APlayerController
{
	GENERATED_BODY()

public:
	AAerotelicPlayerController();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

	// Keyboard shortcuts (Ctrl+N, F5, ...): the frame is a child of the
	// viewport widget, so every key its widgets leave unhandled arrives here;
	// it is offered to the frame's command list before normal game input.
	virtual bool InputKey(const FInputKeyEventArgs& Params) override;

private:
	TSharedPtr<SAerotelicMainShell> Shell;

	// Sets the local player's normalised view rectangle (the split-screen
	// mechanism) to the frame's view hole, so the scene is framed and
	// click-selected relative to the visible 3D view rather than the whole
	// window. Cheap; runs every tick so window resizes are followed.
	void UpdateViewRect();
};
