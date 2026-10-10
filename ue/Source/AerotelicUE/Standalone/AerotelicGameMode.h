#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"

#include "AerotelicGameMode.generated.h"

// The standalone application's game mode: pairs the orbit camera pawn with
// the shell-hosting player controller, builds the fixed studio lighting and
// backdrop the 3D view needs (the level itself -- the engine's empty Entry
// map, see Config/DefaultEngine.ini -- contains nothing at all, so no map
// asset has to be authored in the Editor), and opens a project file named
// on the command line (-AerotelicProject=<path.yaml>), if any.
UCLASS()
class AAerotelicGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	AAerotelicGameMode();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	void SpawnLighting();
	void SpawnBackdrop();
	void SpawnGround();
	// Resizes the grid to the scene's footprint whenever the entity list
	// changes, so a model of any size lands on a sensibly scaled floor.
	void FitGroundToScene();
	void OpenProjectFromCommandLine();

	UPROPERTY()
	TObjectPtr<class AAerotelicGridGround> Ground;

	FDelegateHandle ListChangedHandle;
};
