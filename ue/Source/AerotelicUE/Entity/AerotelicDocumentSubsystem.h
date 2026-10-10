#pragma once

#include "CoreMinimal.h"
#include "Subsystems/EngineSubsystem.h"
#include <memory>

#include "aerotelic/project_yaml.hpp"
#include "AerotelicEntityActor.h"

#include "AerotelicDocumentSubsystem.generated.h"

class AAerotelicEntityActor;
class UWorld;
namespace aerotelic { class Airfoil; }

// Owns the current project: the entity list, the file path and the current
// selection, with New/Save/SaveAs/Open mirroring aerotelic/ui/main_window.py's
// _new_project/_save_project/_save_project_as/_load_project_path --
// including the "scene non-empty, replace it?" confirmation on New/Open,
// and no dirty-flag (neither does the Python app).
//
// An *engine* subsystem (not an editor one) so the very same object serves
// the standalone application (GEngine is a UGameEngine, entities live in the
// game world) and the Editor/commandlet hosts (GEngine is the editor engine,
// entities live in the editor world). GetTargetWorld() picks the right one.
//
// Deliberately does NOT own a FeatureTree in this phase: with no
// boolean-creation UI, nothing yet needs EntityShapeFeature's placed-shape
// computation -- an entity actor's own Unreal transform *is* its placement
// (see AerotelicEntityActor::GetEntityTransform/ApplyPlacement). Wire a
// FeatureTree in when boolean operations get a UI.
UCLASS()
class UAerotelicDocumentSubsystem : public UEngineSubsystem
{
	GENERATED_BODY()

public:
	// The one document, wherever this module is hosted. Null only before
	// the engine has initialised its subsystems.
	static UAerotelicDocumentSubsystem* Get();

	// Returns false (and leaves the scene alone) if the user cancels the
	// "replace the scene?" confirmation.
	bool NewProject();
	bool OpenProjectWithDialog();
	bool OpenProjectPath(const FString& Path);
	bool SaveProject();
	bool SaveProjectAsWithDialog();
	// Saves without a file dialog -- what SaveProject()/SaveProjectAsWithDialog()
	// both call once they have a path; also lets headless tests exercise
	// Save/Open without a native file picker in the loop.
	bool SaveProjectToPath(const FString& Path);

	bool HasCurrentPath() const { return !CurrentPath.IsEmpty(); }
	FString GetCurrentPath() const { return CurrentPath; }

	const TArray<TWeakObjectPtr<AAerotelicEntityActor>>& GetEntities() const { return Entities; }

	// Every section airfoil on any Wing or Propeller blade in the scene --
	// the Sections tab's airfoil picker offers these first, matching
	// section_panel.py's project_airfoils()/_project_airfoil_source.
	TArray<std::shared_ptr<aerotelic::Airfoil>> GatherProjectAirfoils() const;

	AAerotelicEntityActor* AddWing(const FString& PlanformMode);
	AAerotelicEntityActor* AddFuselage();
	AAerotelicEntityActor* AddNacelle();
	AAerotelicEntityActor* AddLoftedNacelle();
	AAerotelicEntityActor* AddPropeller();
	AAerotelicEntityActor* AddPrimitive(const FString& Shape);

	void RemoveEntity(AAerotelicEntityActor* Entity);

	// The application's own selection (the standalone app has no World
	// Outliner): at most one entity, highlighted in the 3D view, and the
	// target of Edit Selected / Delete Selected / the Placement fields /
	// Frame Selected. Null clears the selection.
	void SetSelectedEntity(AAerotelicEntityActor* Entity);
	AAerotelicEntityActor* GetSelectedEntity() const;

	// The scene-wide surface finish (View -> Surface...). Not saved with the
	// project: it is how the scene is viewed, not what it is.
	const FAerotelicSurfaceStyle& GetSurfaceStyle() const { return SurfaceStyle; }
	void SetSurfaceStyle(const FAerotelicSurfaceStyle& Style);

	// Fired after the entity list changes shape (add/remove/New/Open) --
	// not on every in-place edit, which is each entity's own OnChanged.
	DECLARE_MULTICAST_DELEGATE(FOnEntityListChanged);
	FOnEntityListChanged OnEntityListChanged;

	DECLARE_MULTICAST_DELEGATE(FOnSelectionChanged);
	FOnSelectionChanged OnSelectionChanged;

private:
	UWorld* GetTargetWorld() const;
	bool ConfirmReplaceSceneIfNeeded() const;
	void ClearScene();
	FString UniqueLabel(const FString& BaseName) const;
	AAerotelicEntityActor* SpawnEntity(aerotelic::project_yaml::EntityType Type,
		std::unique_ptr<aerotelic::ISolidModel> Model, const FString& Name);

	UPROPERTY()
	TArray<TWeakObjectPtr<AAerotelicEntityActor>> Entities;

	TWeakObjectPtr<AAerotelicEntityActor> SelectedEntity;

	FString CurrentPath;
	FAerotelicSurfaceStyle SurfaceStyle;
};
