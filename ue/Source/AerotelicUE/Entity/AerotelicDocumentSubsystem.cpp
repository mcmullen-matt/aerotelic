#include "AerotelicDocumentSubsystem.h"
#include "AerotelicEntityActor.h"

#include "ReferenceImagePaths.h"

#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Misc/MessageDialog.h"
#include "Misc/Paths.h"
#include "DesktopPlatformModule.h"
#include "IDesktopPlatform.h"
#include "Framework/Application/SlateApplication.h"

#if WITH_EDITOR
#include "Editor.h"
#endif

#include "aerotelic/wing_model.hpp"
#include "aerotelic/fuselage_model.hpp"
#include "aerotelic/nacelle_model.hpp"
#include "aerotelic/lofted_nacelle_model.hpp"
#include "aerotelic/propeller_model.hpp"
#include "aerotelic/primitive_model.hpp"

using namespace aerotelic;
using namespace aerotelic::project_yaml;

UAerotelicDocumentSubsystem* UAerotelicDocumentSubsystem::Get()
{
	return GEngine ? GEngine->GetEngineSubsystem<UAerotelicDocumentSubsystem>() : nullptr;
}

UWorld* UAerotelicDocumentSubsystem::GetTargetWorld() const
{
#if WITH_EDITOR
	// Hosted by the Editor or a commandlet: entities are placed in the
	// editor world, exactly as before the standalone port.
	if (GIsEditor && GEditor != nullptr)
	{
		return GEditor->GetEditorWorldContext().World();
	}
#endif
	// Standalone application: the one running game world.
	if (GEngine != nullptr)
	{
		for (const FWorldContext& Context : GEngine->GetWorldContexts())
		{
			if (Context.WorldType == EWorldType::Game && Context.World() != nullptr)
			{
				return Context.World();
			}
		}
	}
	return nullptr;
}

void UAerotelicDocumentSubsystem::SetSurfaceStyle(const FAerotelicSurfaceStyle& Style)
{
	SurfaceStyle = Style;
	for (auto& WeakEntity : Entities)
	{
		if (WeakEntity.IsValid())
		{
			WeakEntity->ApplySurfaceStyle(Style);
		}
	}
}

bool UAerotelicDocumentSubsystem::ConfirmReplaceSceneIfNeeded() const
{
	if (Entities.Num() == 0)
	{
		return true;
	}
	// No one to ask under a test commandlet or an unattended run.
	if (IsRunningCommandlet() || FApp::IsUnattended())
	{
		return true;
	}
	const EAppReturnType::Type Result = FMessageDialog::Open(EAppMsgType::YesNo,
		FText::FromString(TEXT("Clear the entire project and start with an empty scene?\nAll unsaved changes will be lost.")));
	return Result == EAppReturnType::Yes;
}

void UAerotelicDocumentSubsystem::ClearScene()
{
	SetSelectedEntity(nullptr);
	for (auto& WeakEntity : Entities)
	{
		if (WeakEntity.IsValid())
		{
			WeakEntity->Destroy();
		}
	}
	Entities.Empty();
	CurrentPath.Empty();
	OnEntityListChanged.Broadcast();
}

FString UAerotelicDocumentSubsystem::UniqueLabel(const FString& BaseName) const
{
	TSet<FString> Taken;
	for (auto& WeakEntity : Entities)
	{
		if (WeakEntity.IsValid())
		{
			Taken.Add(WeakEntity->EntityName);
		}
	}
	int32 n = Entities.Num() + 1;
	FString Candidate = FString::Printf(TEXT("%s %d"), *BaseName, n);
	while (Taken.Contains(Candidate))
	{
		++n;
		Candidate = FString::Printf(TEXT("%s %d"), *BaseName, n);
	}
	return Candidate;
}

AAerotelicEntityActor* UAerotelicDocumentSubsystem::SpawnEntity(EntityType Type,
	std::unique_ptr<ISolidModel> Model, const FString& Name)
{
	UWorld* World = GetTargetWorld();
	if (World == nullptr)
	{
		return nullptr;
	}
	AAerotelicEntityActor* Actor = World->SpawnActor<AAerotelicEntityActor>();
	if (Actor == nullptr)
	{
		return nullptr;
	}
#if WITH_EDITOR
	// The World Outliner's display name; editor-only API (the standalone app
	// lists entities by EntityName in its own shell instead).
	if (GIsEditor)
	{
		Actor->SetActorLabel(Name);
	}
#endif
	Actor->InitializeFrom(Type, std::move(Model), Name);
	Entities.Add(Actor);
	OnEntityListChanged.Broadcast();
	return Actor;
}

TArray<std::shared_ptr<Airfoil>> UAerotelicDocumentSubsystem::GatherProjectAirfoils() const
{
	TArray<std::shared_ptr<Airfoil>> Out;
	auto AddFrom = [&Out](const WingModel& Wing)
	{
		for (const WingSection& Sec : Wing.sections())
		{
			if (Sec.airfoil)
			{
				Out.Add(Sec.airfoil);
			}
		}
	};
	for (auto& WeakEntity : Entities)
	{
		if (!WeakEntity.IsValid())
		{
			continue;
		}
		AAerotelicEntityActor* Entity = WeakEntity.Get();
		switch (Entity->GetEntityType())
		{
		case EntityType::Wing:
			AddFrom(*Entity->GetModelAs<WingModel>());
			break;
		case EntityType::Propeller:
			AddFrom(Entity->GetModelAs<PropellerModel>()->blade());
			break;
		default:
			break;
		}
	}
	return Out;
}

void UAerotelicDocumentSubsystem::RemoveEntity(AAerotelicEntityActor* Entity)
{
	if (Entity == nullptr)
	{
		return;
	}
	if (SelectedEntity.Get() == Entity)
	{
		SetSelectedEntity(nullptr);
	}
	Entities.RemoveAll([Entity](const TWeakObjectPtr<AAerotelicEntityActor>& E) { return E.Get() == Entity; });
	Entity->Destroy();
	OnEntityListChanged.Broadcast();
}

void UAerotelicDocumentSubsystem::SetSelectedEntity(AAerotelicEntityActor* Entity)
{
	AAerotelicEntityActor* Previous = SelectedEntity.Get();
	if (Previous == Entity)
	{
		return;
	}
	if (Previous != nullptr)
	{
		Previous->SetSelected(false);
	}
	SelectedEntity = Entity;
	if (Entity != nullptr)
	{
		Entity->SetSelected(true);
	}
	OnSelectionChanged.Broadcast();
}

AAerotelicEntityActor* UAerotelicDocumentSubsystem::GetSelectedEntity() const
{
	return SelectedEntity.IsValid() ? SelectedEntity.Get() : nullptr;
}

namespace
{
	// Every Add* below constructs a model from a spec, which can throw
	// (ParseShape/MakePlanform reject an unrecognized string, etc.) --
	// uncaught, that unwinds as an unhandled SEH exception and takes down
	// the whole application (confirmed the hard way: AddPrimitive(TEXT("Sphere"))
	// crashed the process before this guard existed, since ParseShape
	// requires the lowercase key). A dialog instead of a crash, matching
	// every other user-facing error path in this module.
	void ReportAddEntityFailure(const std::exception& e)
	{
		FMessageDialog::Open(EAppMsgType::Ok,
			FText::FromString(FString::Printf(TEXT("Could not add entity:\n%s"), UTF8_TO_TCHAR(e.what()))));
	}
}

AAerotelicEntityActor* UAerotelicDocumentSubsystem::AddWing(const FString& PlanformMode)
{
	try
	{
		PlanformSpec Spec;
		Spec.type = TCHAR_TO_UTF8(*PlanformMode);
		auto Model = std::make_unique<WingModel>(Spec);
		const FString Label = UniqueLabel(TEXT("Wing"));
		Model->name = TCHAR_TO_UTF8(*Label);
		// A fresh WingModel has no sections at all -- BuildSolid() throws
		// "No Airfoil Sections provided to the wing loft" until it has at
		// least a root and a tip. Python's make_wing_model() (ui/adapters.py)
		// always seeds both with a NACA 0012 placeholder before handing the
		// model to any panel; do the same here.
		std::shared_ptr<Airfoil> DefaultAirfoil = Airfoil::Naca4("0012");
		Model->AddSection(DefaultAirfoil, 0.0);
		Model->AddSection(DefaultAirfoil, 1.0);
		return SpawnEntity(EntityType::Wing, std::move(Model), Label);
	}
	catch (const std::exception& e)
	{
		ReportAddEntityFailure(e);
		return nullptr;
	}
}

AAerotelicEntityActor* UAerotelicDocumentSubsystem::AddFuselage()
{
	try
	{
		auto Model = std::make_unique<FuselageModel>(FuselageSpec{});
		const FString Label = UniqueLabel(TEXT("Fuselage"));
		Model->name = TCHAR_TO_UTF8(*Label);
		return SpawnEntity(EntityType::Fuselage, std::move(Model), Label);
	}
	catch (const std::exception& e)
	{
		ReportAddEntityFailure(e);
		return nullptr;
	}
}

AAerotelicEntityActor* UAerotelicDocumentSubsystem::AddNacelle()
{
	try
	{
		auto Model = std::make_unique<NacelleModel>(NacelleSpec{});
		const FString Label = UniqueLabel(TEXT("Nacelle"));
		Model->name = TCHAR_TO_UTF8(*Label);
		return SpawnEntity(EntityType::Nacelle, std::move(Model), Label);
	}
	catch (const std::exception& e)
	{
		ReportAddEntityFailure(e);
		return nullptr;
	}
}

AAerotelicEntityActor* UAerotelicDocumentSubsystem::AddLoftedNacelle()
{
	try
	{
		auto Model = std::make_unique<LoftedNacelleModel>(LoftedNacelleSpec{});
		const FString Label = UniqueLabel(TEXT("Nacelle (lofted)"));
		Model->name = TCHAR_TO_UTF8(*Label);
		return SpawnEntity(EntityType::LoftedNacelle, std::move(Model), Label);
	}
	catch (const std::exception& e)
	{
		ReportAddEntityFailure(e);
		return nullptr;
	}
}

AAerotelicEntityActor* UAerotelicDocumentSubsystem::AddPropeller()
{
	try
	{
		auto Model = std::make_unique<PropellerModel>(PropellerSpec{});
		const FString Label = UniqueLabel(TEXT("Propeller"));
		Model->name = TCHAR_TO_UTF8(*Label);
		return SpawnEntity(EntityType::Propeller, std::move(Model), Label);
	}
	catch (const std::exception& e)
	{
		ReportAddEntityFailure(e);
		return nullptr;
	}
}

AAerotelicEntityActor* UAerotelicDocumentSubsystem::AddPrimitive(const FString& Shape)
{
	try
	{
		auto Model = std::make_unique<PrimitiveModel>();
		// ParseShape expects the lowercase key ("sphere"/"box"/"cylinder"),
		// not the capitalized label the "Add Entity" buttons show.
		Model->shape = ParseShape(TCHAR_TO_UTF8(*Shape.ToLower()));
		const FString Label = UniqueLabel(Shape);
		Model->name = TCHAR_TO_UTF8(*Label);
		return SpawnEntity(EntityType::Primitive, std::move(Model), Label);
	}
	catch (const std::exception& e)
	{
		ReportAddEntityFailure(e);
		return nullptr;
	}
}

bool UAerotelicDocumentSubsystem::NewProject()
{
	if (!ConfirmReplaceSceneIfNeeded())
	{
		return false;
	}
	ClearScene();
	return true;
}

bool UAerotelicDocumentSubsystem::OpenProjectWithDialog()
{
	if (!ConfirmReplaceSceneIfNeeded())
	{
		return false;
	}
	IDesktopPlatform* Desktop = FDesktopPlatformModule::Get();
	if (Desktop == nullptr)
	{
		return false;
	}
	TArray<FString> Filenames;
	const void* ParentWindowHandle = FSlateApplication::Get().FindBestParentWindowHandleForDialogs(nullptr);
	const bool bOpened = Desktop->OpenFileDialog(ParentWindowHandle, TEXT("Open Project"), TEXT(""), TEXT(""),
		TEXT("YAML Files (*.yaml;*.yml)|*.yaml;*.yml"), EFileDialogFlags::None, Filenames);
	if (!bOpened || Filenames.Num() == 0)
	{
		return false;
	}
	return OpenProjectPath(Filenames[0]);
}

bool UAerotelicDocumentSubsystem::OpenProjectPath(const FString& InPath, bool bAdopt)
{
	// The native file dialogs hand back paths relative to the engine's
	// binaries directory; everything downstream (the status bar, the
	// project directory that reference images are saved relative to) wants
	// the full path.
	const FString Path = FPaths::ConvertRelativePathToFull(InPath);
	if (!ConfirmReplaceSceneIfNeeded())
	{
		return false;
	}
	ProjectFile Project;
	try
	{
		Project = LoadProjectYaml(TCHAR_TO_UTF8(*Path));
	}
	catch (const std::exception& e)
	{
		FMessageDialog::Open(EAppMsgType::Ok,
			FText::FromString(FString::Printf(TEXT("Could not open project:\n%s"), UTF8_TO_TCHAR(e.what()))));
		return false;
	}

	ClearScene();
	for (const ProjectEntity& Entity : Project.entities)
	{
		std::unique_ptr<ISolidModel> Model;
		switch (Entity.type)
		{
		case EntityType::Wing:
			Model = std::make_unique<WingModel>(std::get<WingModelSpec>(Entity.model));
			break;
		case EntityType::Fuselage:
			Model = std::make_unique<FuselageModel>(std::get<FuselageSpec>(Entity.model));
			break;
		case EntityType::Nacelle:
			Model = std::make_unique<NacelleModel>(std::get<NacelleSpec>(Entity.model));
			break;
		case EntityType::LoftedNacelle:
			Model = std::make_unique<LoftedNacelleModel>(std::get<LoftedNacelleSpec>(Entity.model));
			break;
		case EntityType::Propeller:
			Model = std::make_unique<PropellerModel>(std::get<PropellerSpec>(Entity.model));
			break;
		case EntityType::Primitive:
		{
			const PrimitiveSpec& Ps = std::get<PrimitiveSpec>(Entity.model);
			auto Pm = std::make_unique<PrimitiveModel>();
			Pm->shape = Ps.shape;
			Pm->name = Ps.name;
			Pm->radius = Ps.radius;
			Pm->stretch = Ps.stretch;
			Pm->size = Ps.size;
			Pm->cyl_radius = Ps.cyl_radius;
			Pm->cyl_length = Ps.cyl_length;
			Model = std::move(Pm);
			break;
		}
		}
		if (Model == nullptr)
		{
			continue;
		}
		ResolveReferenceImagePaths(Model.get(), Entity.type, FPaths::GetPath(Path));
		const FString Name = UTF8_TO_TCHAR(Entity.name.c_str());
		AAerotelicEntityActor* Actor = SpawnEntity(Entity.type, std::move(Model), Name);
		if (Actor != nullptr)
		{
			Actor->ApplyPlacement(Entity.placement);
			Actor->RefreshProxy();
			// Display state saved with the entity: the mirrored half, a
			// colour override, hidden. Absent means the defaults.
			Actor->SetMirrored(Entity.mirror);
			if (Entity.color)
			{
				const auto& C = *Entity.color;
				Actor->SetColorOverride(FLinearColor((float)C[0], (float)C[1], (float)C[2]));
			}
			if (Entity.hidden)
			{
				Actor->SetEntityVisible(false);
			}
		}
	}
	CurrentPath = bAdopt ? Path : FString();
	OnEntityListChanged.Broadcast();
	return true;
}

bool UAerotelicDocumentSubsystem::SaveProjectToPath(const FString& InPath)
{
	const FString Path = FPaths::ConvertRelativePathToFull(InPath); // see OpenProjectPath
	if (Entities.Num() == 0)
	{
		FMessageDialog::Open(EAppMsgType::Ok, FText::FromString(TEXT("Nothing to save -- the scene is empty.")));
		return false;
	}

	ProjectFile Project;
	for (auto& WeakEntity : Entities)
	{
		if (!WeakEntity.IsValid())
		{
			continue;
		}
		AAerotelicEntityActor* Entity = WeakEntity.Get();
		ProjectEntity Pe;
		Pe.name = TCHAR_TO_UTF8(*Entity->EntityName);
		Pe.type = Entity->GetEntityType();
		Pe.mirror = Entity->IsMirrored();
		Pe.placement = Entity->GetPlacement();
		// Written only when set, as the Qt app does: defaults never reach
		// the file.
		if (const TOptional<FLinearColor> Color = Entity->GetColorOverride(); Color.IsSet())
		{
			Pe.color = std::array<double, 3>{ Color->R, Color->G, Color->B };
		}
		Pe.hidden = !Entity->IsEntityVisible();
		switch (Pe.type)
		{
		case EntityType::Wing:
			Pe.model = Entity->GetModelAs<WingModel>()->Serialize();
			break;
		case EntityType::Fuselage:
			Pe.model = Entity->GetModelAs<FuselageModel>()->Serialize();
			break;
		case EntityType::Nacelle:
			Pe.model = Entity->GetModelAs<NacelleModel>()->Serialize();
			break;
		case EntityType::LoftedNacelle:
			Pe.model = Entity->GetModelAs<LoftedNacelleModel>()->Serialize();
			break;
		case EntityType::Propeller:
			Pe.model = Entity->GetModelAs<PropellerModel>()->Serialize();
			break;
		case EntityType::Primitive:
		{
			PrimitiveModel* Pm = Entity->GetModelAs<PrimitiveModel>();
			PrimitiveSpec Ps;
			Ps.shape = Pm->shape;
			Ps.name = Pm->name;
			Ps.radius = Pm->radius;
			Ps.stretch = Pm->stretch;
			Ps.size = Pm->size;
			Ps.cyl_radius = Pm->cyl_radius;
			Ps.cyl_length = Pm->cyl_length;
			Pe.model = Ps;
			break;
		}
		}
		RelativiseReferenceImagePaths(Pe.model, FPaths::GetPath(Path));
		Project.entities.push_back(std::move(Pe));
	}

	try
	{
		SaveProjectYaml(Project, TCHAR_TO_UTF8(*Path));
	}
	catch (const std::exception& e)
	{
		FMessageDialog::Open(EAppMsgType::Ok,
			FText::FromString(FString::Printf(TEXT("Save failed:\n%s"), UTF8_TO_TCHAR(e.what()))));
		return false;
	}
	return true;
}

bool UAerotelicDocumentSubsystem::SaveProject()
{
	if (!HasCurrentPath())
	{
		return SaveProjectAsWithDialog();
	}
	return SaveProjectToPath(CurrentPath);
}

bool UAerotelicDocumentSubsystem::SaveProjectAsWithDialog()
{
	if (Entities.Num() == 0)
	{
		FMessageDialog::Open(EAppMsgType::Ok, FText::FromString(TEXT("Nothing to save -- the scene is empty.")));
		return false;
	}
	IDesktopPlatform* Desktop = FDesktopPlatformModule::Get();
	if (Desktop == nullptr)
	{
		return false;
	}
	TArray<FString> Filenames;
	const void* ParentWindowHandle = FSlateApplication::Get().FindBestParentWindowHandleForDialogs(nullptr);
	const bool bChosen = Desktop->SaveFileDialog(ParentWindowHandle, TEXT("Save Project As"), CurrentPath, TEXT(""),
		TEXT("YAML Files (*.yaml;*.yml)|*.yaml;*.yml"), EFileDialogFlags::None, Filenames);
	if (!bChosen || Filenames.Num() == 0)
	{
		return false;
	}
	FString Path = Filenames[0];
	if (!Path.EndsWith(TEXT(".yaml")) && !Path.EndsWith(TEXT(".yml")))
	{
		Path += TEXT(".yaml");
	}
	if (SaveProjectToPath(Path))
	{
		CurrentPath = Path;
		return true;
	}
	return false;
}
