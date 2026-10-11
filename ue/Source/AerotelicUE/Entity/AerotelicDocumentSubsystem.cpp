#include "AerotelicDocumentSubsystem.h"
#include "Widgets/MessageBox.h"
#include "AerotelicEntityActor.h"
#include "AerotelicBooleanModel.h"
#include "AerotelicLatticeModel.h"

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

void UAerotelicDocumentSubsystem::SetRenderStyle(EAerotelicRenderStyle Style)
{
	RenderStyle = Style;
	for (auto& WeakEntity : Entities)
	{
		if (WeakEntity.IsValid())
		{
			WeakEntity->ApplyRenderStyle(Style);
		}
	}
}

void UAerotelicDocumentSubsystem::ApplySectionPlaneToEntities()
{
	for (auto& WeakEntity : Entities)
	{
		if (WeakEntity.IsValid())
		{
			WeakEntity->ApplySectionPlane(SectionPlane);
		}
	}
	OnSectionChanged.Broadcast();
}

void UAerotelicDocumentSubsystem::SetSectionAxis(EAerotelicSectionAxis Axis)
{
	SectionPlane.Axis = Axis;
	if (Axis != EAerotelicSectionAxis::None)
	{
		// view_gizmo.py set_axis: start at the midpoint of the scene box
		// along the chosen axis (the origin when the scene is empty).
		const FBox Bounds = GetSceneBounds();
		SectionPlane.Position = Bounds.IsValid ? Bounds.GetCenter()[FAerotelicSectionPlane::AxisIndex(Axis)] : 0.0;
	}
	ApplySectionPlaneToEntities();
}

void UAerotelicDocumentSubsystem::SetSectionFlipped(bool bFlipped)
{
	SectionPlane.bFlipped = bFlipped;
	ApplySectionPlaneToEntities();
}

void UAerotelicDocumentSubsystem::SetSectionPosition(double Position)
{
	SectionPlane.Position = Position;
	ApplySectionPlaneToEntities();
}

FBox UAerotelicDocumentSubsystem::GetSceneBounds() const
{
	FBox All(ForceInit);
	for (const auto& WeakEntity : Entities)
	{
		if (WeakEntity.IsValid())
		{
			const FBox Box = WeakEntity->GetVisibleWorldBounds();
			if (Box.IsValid)
			{
				All += Box;
			}
		}
	}
	return All;
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
	const EAppReturnType::Type Result = AerotelicMessageBox(EAppMsgType::YesNo,
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
	Actor->ApplyRenderStyle(RenderStyle);
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
	// Cascade: a boolean result that used this as an operand, or a lattice
	// filling it, goes too (and anything built on that), as
	// FeatureTree.Remove does.
	TArray<AAerotelicEntityActor*> Dependents;
	for (auto& Weak : Entities)
	{
		if (Weak.IsValid() && Weak.Get() != Entity && DependsOn(Weak.Get(), Entity))
		{
			Dependents.Add(Weak.Get());
		}
	}
	if (SelectedEntity.Get() == Entity)
	{
		SetSelectedEntity(nullptr);
	}
	Entities.RemoveAll([Entity](const TWeakObjectPtr<AAerotelicEntityActor>& E) { return E.Get() == Entity; });
	Entity->Destroy();
	for (AAerotelicEntityActor* D : Dependents)
	{
		if (SelectedEntity.Get() == D)
		{
			SetSelectedEntity(nullptr);
		}
		Entities.RemoveAll([D](const TWeakObjectPtr<AAerotelicEntityActor>& E) { return E.Get() == D; });
		D->Destroy();
	}
	OnEntityListChanged.Broadcast();
}

bool UAerotelicDocumentSubsystem::IsNameTaken(const FString& Name, const AAerotelicEntityActor* Except) const
{
	for (const auto& Weak : Entities)
	{
		if (Weak.IsValid() && Weak.Get() != Except && Weak->EntityName == Name)
		{
			return true;
		}
	}
	return false;
}

FString UAerotelicDocumentSubsystem::NextBooleanName() const
{
	int32 Count = 0;
	for (const auto& Weak : Entities)
	{
		if (Weak.IsValid() && Weak->GetEntityType() == EntityType::Boolean)
		{
			++Count;
		}
	}
	int32 n = Count + 1;
	FString Candidate = FString::Printf(TEXT("BoolOp %d"), n);
	while (IsNameTaken(Candidate))
	{
		++n;
		Candidate = FString::Printf(TEXT("BoolOp %d"), n);
	}
	return Candidate;
}

bool UAerotelicDocumentSubsystem::DependsOn(const AAerotelicEntityActor* Entity, const AAerotelicEntityActor* Other)
{
	if (Entity == nullptr || Other == nullptr || Entity->GetModel() == nullptr)
	{
		return false;
	}
	if (Entity->GetEntityType() == EntityType::Boolean)
	{
		return static_cast<const FAerotelicBooleanModel*>(Entity->GetModel())->DependsOn(Other);
	}
	if (Entity->GetEntityType() == EntityType::Lattice)
	{
		return static_cast<const FAerotelicLatticeModel*>(Entity->GetModel())->DependsOn(Other);
	}
	return false;
}

FString UAerotelicDocumentSubsystem::NextLatticeName() const
{
	int32 Count = 0;
	for (const auto& Weak : Entities)
	{
		if (Weak.IsValid() && Weak->GetEntityType() == EntityType::Lattice)
		{
			++Count;
		}
	}
	int32 n = Count + 1;
	FString Candidate = FString::Printf(TEXT("Lattice %d"), n);
	while (IsNameTaken(Candidate))
	{
		++n;
		Candidate = FString::Printf(TEXT("Lattice %d"), n);
	}
	return Candidate;
}

AAerotelicEntityActor* UAerotelicDocumentSubsystem::AddLattice(const FString& Name, const aerotelic::LatticeSpec& Spec,
	const TArray<FAerotelicBooleanOperand>& Targets, const TArray<FAerotelicBooleanOperand>& Exclusions, FString& OutError)
{
	if (Targets.Num() == 0)
	{
		OutError = TEXT("Select at least one body to fill.");
		return nullptr;
	}
	if (IsNameTaken(Name))
	{
		OutError = FString::Printf(TEXT("'%s' is already in use. Names must be unique in the scene."), *Name);
		return nullptr;
	}
	for (const FAerotelicBooleanOperand& Op : Targets)
	{
		if (Op.Entity.IsValid() && Op.Entity->GetEntityType() == EntityType::Lattice)
		{
			OutError = TEXT("A lattice cannot be filled with another lattice.");
			return nullptr;
		}
	}
	auto Model = std::make_unique<FAerotelicLatticeModel>();
	Model->Spec = Spec;
	Model->Targets = Targets;
	Model->Exclusions = Exclusions;
	try
	{
		Model->BuildMeshes(0.0);
	}
	catch (const std::exception& e)
	{
		OutError = UTF8_TO_TCHAR(e.what());
		return nullptr;
	}
	return SpawnEntity(EntityType::Lattice, std::move(Model), Name);
}

bool UAerotelicDocumentSubsystem::UpdateLattice(AAerotelicEntityActor* Lattice, const aerotelic::LatticeSpec& Spec,
	const TArray<FAerotelicBooleanOperand>& Targets, const TArray<FAerotelicBooleanOperand>& Exclusions, FString& OutError)
{
	if (Lattice == nullptr || Lattice->GetEntityType() != EntityType::Lattice)
	{
		OutError = TEXT("Not a lattice.");
		return false;
	}
	if (Targets.Num() == 0)
	{
		OutError = TEXT("Select at least one body to fill.");
		return false;
	}
	FAerotelicLatticeModel* Model = Lattice->GetModelAs<FAerotelicLatticeModel>();
	const aerotelic::LatticeSpec OldSpec = Model->Spec;
	const TArray<FAerotelicBooleanOperand> OldTargets = Model->Targets, OldExclusions = Model->Exclusions;
	Model->Spec = Spec;
	Model->Targets = Targets;
	Model->Exclusions = Exclusions;
	Model->Invalidate();
	try
	{
		Model->BuildMeshes(0.0);
	}
	catch (const std::exception& e)
	{
		OutError = UTF8_TO_TCHAR(e.what());
		Model->Spec = OldSpec;
		Model->Targets = OldTargets;
		Model->Exclusions = OldExclusions;
		Model->Invalidate();
		return false;
	}
	Lattice->RefreshProxy();
	return true;
}

int32 UAerotelicDocumentSubsystem::RegenerateLattices(TArray<FString>& OutErrors)
{
	int32 Count = 0;
	for (auto& Weak : Entities)
	{
		if (!Weak.IsValid() || Weak->GetEntityType() != EntityType::Lattice)
		{
			continue;
		}
		FAerotelicLatticeModel* Model = Weak->GetModelAs<FAerotelicLatticeModel>();
		Model->Invalidate();
		try
		{
			Model->BuildMeshes(0.0);
			Weak->RefreshProxy();
			++Count;
		}
		catch (const std::exception& e)
		{
			OutErrors.Add(FString::Printf(TEXT("%s: %s"), *Weak->EntityName, UTF8_TO_TCHAR(e.what())));
		}
	}
	return Count;
}

AAerotelicEntityActor* UAerotelicDocumentSubsystem::AddBoolean(const FString& Name, const TArray<FAerotelicBooleanOperand>& Operands,
	const FString& Operation, FString& OutError)
{
	if (Operands.Num() < 2)
	{
		OutError = TEXT("Select at least two operands.");
		return nullptr;
	}
	if (IsNameTaken(Name))
	{
		OutError = FString::Printf(TEXT("'%s' is already in use. Names must be unique in the scene."), *Name);
		return nullptr;
	}
	auto Model = std::make_unique<FAerotelicBooleanModel>();
	Model->Operation = TCHAR_TO_UTF8(*Operation);
	Model->Operands = Operands;
	// Build once up front so a failing operation (OCCT could not fuse/cut
	// these) adds nothing, as fold_boolean cleans up after itself. The
	// model keeps this result for the actor's first mesh build.
	try
	{
		Model->BuildSolid();
	}
	catch (const std::exception& e)
	{
		OutError = UTF8_TO_TCHAR(e.what());
		return nullptr;
	}
	return SpawnEntity(EntityType::Boolean, std::move(Model), Name);
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
		AerotelicMessageBox(EAppMsgType::Ok,
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
		AerotelicMessageBox(EAppMsgType::Ok,
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
	// The boolean operations, in build order, over what is now in the
	// scene (main_window.py _load_boolean). One that cannot be built is
	// reported and skipped; the rest of the project still opens.
	TArray<FString> Failures;
	for (const ProjectBoolean& B : Project.booleans)
	{
		TArray<FAerotelicBooleanOperand> Operands;
		FString Error;
		for (const BooleanOperand& Ref : B.operands)
		{
			const FString RefName = UTF8_TO_TCHAR(Ref.name.c_str());
			FAerotelicBooleanOperand Op;
			for (auto& Weak : Entities)
			{
				if (Weak.IsValid() && Weak->EntityName == RefName)
				{
					Op.Entity = Weak;
					break;
				}
			}
			Op.bMirror = Ref.mirror;
			if (!Op.Entity.IsValid())
			{
				Error = FString::Printf(TEXT("'%s' does not exist"), *RefName);
				break;
			}
			Operands.Add(Op);
		}
		const FString Name = UTF8_TO_TCHAR(B.name.c_str());
		AAerotelicEntityActor* Actor = Error.IsEmpty() ? AddBoolean(Name, Operands, UTF8_TO_TCHAR(B.operation.c_str()), Error) : nullptr;
		if (Actor == nullptr)
		{
			Failures.Add(FString::Printf(TEXT("%s: %s"), *Name, *Error));
			continue;
		}
		Actor->RefreshProxy();
		if (B.color)
		{
			Actor->SetColorOverride(FLinearColor((float)(*B.color)[0], (float)(*B.color)[1], (float)(*B.color)[2]));
		}
		for (const ProjectBooleanBody& Body : B.bodies)
		{
			const FString Role = UTF8_TO_TCHAR(Body.name.c_str());
			for (int32 i = 0; i < Actor->GetBodies().Num(); ++i)
			{
				if (Actor->GetBodies()[i].Name == Role)
				{
					if (Body.color)
					{
						Actor->SetBodyColor(i, FLinearColor((float)(*Body.color)[0], (float)(*Body.color)[1], (float)(*Body.color)[2]));
					}
					if (Body.hidden)
					{
						Actor->SetBodyVisible(i, false);
					}
				}
			}
		}
		if (B.hidden)
		{
			Actor->SetEntityVisible(false);
		}
	}
	// The lattices, regenerated over the entities and boolean results now
	// in the scene.
	for (const ProjectLattice& L : Project.lattices)
	{
		FString Error;
		auto Resolve = [&](const std::vector<BooleanOperand>& Refs, TArray<FAerotelicBooleanOperand>& Out)
		{
			for (const BooleanOperand& Ref : Refs)
			{
				const FString RefName = UTF8_TO_TCHAR(Ref.name.c_str());
				FAerotelicBooleanOperand Op;
				for (auto& Weak : Entities)
				{
					if (Weak.IsValid() && Weak->EntityName == RefName)
					{
						Op.Entity = Weak;
						break;
					}
				}
				Op.bMirror = Ref.mirror;
				if (!Op.Entity.IsValid())
				{
					Error = FString::Printf(TEXT("'%s' does not exist"), *RefName);
					return;
				}
				Out.Add(Op);
			}
		};
		TArray<FAerotelicBooleanOperand> Targets, Exclusions;
		Resolve(L.targets, Targets);
		if (Error.IsEmpty()) Resolve(L.exclusions, Exclusions);
		const FString Name = UTF8_TO_TCHAR(L.name.c_str());
		AAerotelicEntityActor* Actor = Error.IsEmpty() ? AddLattice(Name, L.spec, Targets, Exclusions, Error) : nullptr;
		if (Actor == nullptr)
		{
			Failures.Add(FString::Printf(TEXT("%s: %s"), *Name, *Error));
			continue;
		}
		Actor->RefreshProxy();
		if (L.color)
		{
			Actor->SetColorOverride(FLinearColor((float)(*L.color)[0], (float)(*L.color)[1], (float)(*L.color)[2]));
		}
		if (L.hidden)
		{
			Actor->SetEntityVisible(false);
		}
	}
	if (Failures.Num() > 0 && !IsRunningCommandlet() && !FApp::IsUnattended())
	{
		AerotelicMessageBox(EAppMsgType::Ok,
			FText::FromString(TEXT("Some boolean operations or lattices could not be rebuilt and were left out:\n") + FString::Join(Failures, TEXT("\n"))));
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
		AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(TEXT("Nothing to save -- the scene is empty.")));
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
		if (Entity->GetEntityType() == EntityType::Boolean || Entity->GetEntityType() == EntityType::Lattice)
		{
			continue; // written under `booleans:` / `lattices:` below
		}
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
		case EntityType::Boolean:
		case EntityType::Lattice:
			break; // skipped above
		}
		RelativiseReferenceImagePaths(Pe.model, FPaths::GetPath(Path));
		Project.entities.push_back(std::move(Pe));
	}

	// The boolean operations as authored (_booleans_for_save): name,
	// operation, operands by name, and the display state that differs from
	// the defaults, in build order.
	for (auto& WeakEntity : Entities)
	{
		if (!WeakEntity.IsValid() || WeakEntity->GetEntityType() != EntityType::Boolean)
		{
			continue;
		}
		AAerotelicEntityActor* Entity = WeakEntity.Get();
		const FAerotelicBooleanModel* M = static_cast<const FAerotelicBooleanModel*>(Entity->GetModel());
		ProjectBoolean Pb;
		Pb.name = TCHAR_TO_UTF8(*Entity->EntityName);
		Pb.operation = M->Operation;
		for (const FAerotelicBooleanOperand& Op : M->Operands)
		{
			if (!Op.Entity.IsValid())
			{
				continue;
			}
			BooleanOperand Ref;
			Ref.name = TCHAR_TO_UTF8(*Op.Entity->EntityName);
			Ref.mirror = Op.bMirror;
			Pb.operands.push_back(Ref);
		}
		Pb.hidden = !Entity->IsEntityVisible();
		if (const TOptional<FLinearColor> Color = Entity->GetColorOverride(); Color.IsSet())
		{
			Pb.color = std::array<double, 3>{ Color->R, Color->G, Color->B };
		}
		for (const FAerotelicBody& Body : Entity->GetBodies())
		{
			if (!Body.bHasColor && Body.bVisible)
			{
				continue;
			}
			ProjectBooleanBody Bb;
			Bb.name = TCHAR_TO_UTF8(*Body.Name);
			if (Body.bHasColor)
			{
				Bb.color = std::array<double, 3>{ Body.Color.R, Body.Color.G, Body.Color.B };
			}
			Bb.hidden = !Body.bVisible;
			Pb.bodies.push_back(Bb);
		}
		Project.booleans.push_back(std::move(Pb));
	}

	// The lattices: parameters and the bodies they fill, by name.
	for (auto& WeakEntity : Entities)
	{
		if (!WeakEntity.IsValid() || WeakEntity->GetEntityType() != EntityType::Lattice)
		{
			continue;
		}
		AAerotelicEntityActor* Entity = WeakEntity.Get();
		const FAerotelicLatticeModel* M = static_cast<const FAerotelicLatticeModel*>(Entity->GetModel());
		ProjectLattice Pl;
		Pl.name = TCHAR_TO_UTF8(*Entity->EntityName);
		Pl.spec = M->Spec;
		auto Refs = [](const TArray<FAerotelicBooleanOperand>& Ops)
		{
			std::vector<BooleanOperand> Out;
			for (const FAerotelicBooleanOperand& Op : Ops)
			{
				if (!Op.Entity.IsValid())
				{
					continue;
				}
				BooleanOperand Ref;
				Ref.name = TCHAR_TO_UTF8(*Op.Entity->EntityName);
				Ref.mirror = Op.bMirror;
				Out.push_back(Ref);
			}
			return Out;
		};
		Pl.targets = Refs(M->Targets);
		Pl.exclusions = Refs(M->Exclusions);
		Pl.hidden = !Entity->IsEntityVisible();
		if (const TOptional<FLinearColor> Color = Entity->GetColorOverride(); Color.IsSet())
		{
			Pl.color = std::array<double, 3>{ Color->R, Color->G, Color->B };
		}
		Project.lattices.push_back(std::move(Pl));
	}

	try
	{
		SaveProjectYaml(Project, TCHAR_TO_UTF8(*Path));
	}
	catch (const std::exception& e)
	{
		AerotelicMessageBox(EAppMsgType::Ok,
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
		AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(TEXT("Nothing to save -- the scene is empty.")));
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
