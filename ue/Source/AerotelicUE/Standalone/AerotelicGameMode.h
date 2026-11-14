#pragma once

#include "CoreMinimal.h"
#include "Containers/Ticker.h"
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
	// Resizes the grid to the scene's footprint and drops its plane under
	// the scene's lowest point whenever the entity list changes or any
	// entity's placed geometry does (a rebuild, a move, a structure
	// generated), so a model of any size or height lands on a sensibly
	// placed floor.
	void FitGroundToScene();
	// Geometry changes come in bursts (one per entity of a rebuilt project,
	// one per update of a drag): the refit is coalesced to the next tick.
	void RequestGroundFit();
	void OnEntityGeometryChanged(class AAerotelicEntityActor* Entity) { RequestGroundFit(); }
	void OpenProjectFromCommandLine();
	// The backdrop sphere takes the theme's viewport background.
	void ApplyThemeToBackdrop();

	UPROPERTY()
	TObjectPtr<class AAerotelicGridGround> Ground;

	UPROPERTY()
	TObjectPtr<class UMaterialInstanceDynamic> BackdropMID;

	FDelegateHandle ListChangedHandle;
	FDelegateHandle GeometryChangedHandle;
	FTSTicker::FDelegateHandle GroundFitTick;
	FDelegateHandle ThemeHandle;
};
