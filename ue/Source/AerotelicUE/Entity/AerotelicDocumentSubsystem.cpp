#include "AerotelicDocumentSubsystem.h"
#include "Widgets/MessageBox.h"
#include "AerotelicEntityActor.h"
#include "AerotelicBooleanModel.h"
#include "AerotelicBodyStructure.h"
#include "AerotelicMeshing.h"

#include "aerotelic/sdf.hpp"

#include "ReferenceImagePaths.h"

#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Engine/DirectionalLight.h"
#include "Components/DirectionalLightComponent.h"
#include "EngineUtils.h"
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

namespace
{
	// Model <-> spec for the parameterised entity types: the one place that
	// knows which spec belongs to which type. Saving, loading and
	// copy/paste all go through these two, so a new entity type is taught
	// here once instead of in three switches that can drift apart.
	//
	// Boolean results are not among them: they are solids built FROM other
	// entities rather than parameterisations of their own (hence no entry
	// in EntityModelSpec, and hence `booleans:` being a separate section of
	// the project file).
	bool SpecFromModel(EntityType Type, const ISolidModel& Model, EntityModelSpec& OutSpec)
	{
		switch (Type)
		{
		case EntityType::Wing:
			OutSpec = static_cast<const WingModel&>(Model).Serialize();
			return true;
		case EntityType::Fuselage:
			OutSpec = static_cast<const FuselageModel&>(Model).Serialize();
			return true;
		case EntityType::Nacelle:
			OutSpec = static_cast<const NacelleModel&>(Model).Serialize();
			return true;
		case EntityType::LoftedNacelle:
			OutSpec = static_cast<const LoftedNacelleModel&>(Model).Serialize();
			return true;
		case EntityType::Propeller:
			OutSpec = static_cast<const PropellerModel&>(Model).Serialize();
			return true;
		case EntityType::Primitive:
		{
			const PrimitiveModel* Pm = static_cast<const PrimitiveModel*>(&Model);
			PrimitiveSpec Ps;
			Ps.shape = Pm->shape;
			Ps.name = Pm->name;
			Ps.radius = Pm->radius;
			Ps.stretch = Pm->stretch;
			Ps.size = Pm->size;
			Ps.cyl_radius = Pm->cyl_radius;
			Ps.cyl_length = Pm->cyl_length;
			OutSpec = Ps;
			return true;
		}
		case EntityType::Boolean:
			return false;
		}
		return false;
	}

	bool SpecFromEntityModel(const AAerotelicEntityActor& Entity, EntityModelSpec& OutSpec)
	{
		return Entity.GetModel() != nullptr && SpecFromModel(Entity.GetEntityType(), *Entity.GetModel(), OutSpec);
	}

	std::unique_ptr<ISolidModel> ModelFromEntitySpec(EntityType Type, const EntityModelSpec& Spec)
	{
		switch (Type)
		{
		case EntityType::Wing:
			return std::make_unique<WingModel>(std::get<WingModelSpec>(Spec));
		case EntityType::Fuselage:
			return std::make_unique<FuselageModel>(std::get<FuselageSpec>(Spec));
		case EntityType::Nacelle:
			return std::make_unique<NacelleModel>(std::get<NacelleSpec>(Spec));
		case EntityType::LoftedNacelle:
			return std::make_unique<LoftedNacelleModel>(std::get<LoftedNacelleSpec>(Spec));
		case EntityType::Propeller:
			return std::make_unique<PropellerModel>(std::get<PropellerSpec>(Spec));
		case EntityType::Primitive:
		{
			const PrimitiveSpec& Ps = std::get<PrimitiveSpec>(Spec);
			auto Pm = std::make_unique<PrimitiveModel>();
			Pm->shape = Ps.shape;
			Pm->name = Ps.name;
			Pm->radius = Ps.radius;
			Pm->stretch = Ps.stretch;
			Pm->size = Ps.size;
			Pm->cyl_radius = Ps.cyl_radius;
			Pm->cyl_length = Ps.cyl_length;
			return Pm;
		}
		case EntityType::Boolean:
			return nullptr;
		}
		return nullptr;
	}

	// Every spec carries the entity's name; a copy needs its own.
	void SetSpecName(EntityModelSpec& Spec, const FString& Name)
	{
		const std::string Utf8 = TCHAR_TO_UTF8(*Name);
		std::visit([&Utf8](auto& S) { S.name = Utf8; }, Spec);
	}
}

UAerotelicDocumentSubsystem* UAerotelicDocumentSubsystem::Get()
{
	return GEngine ? GEngine->GetEngineSubsystem<UAerotelicDocumentSubsystem>() : nullptr;
}

void UAerotelicDocumentSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
	GeometryChangedHandle = AAerotelicEntityActor::OnGeometryChanged.AddUObject(this, &UAerotelicDocumentSubsystem::OnEntityGeometryChanged);
	BodyStructureChangedHandle = AAerotelicEntityActor::OnBodyStructureChanged.AddUObject(this, &UAerotelicDocumentSubsystem::OnEntityBodyStructureChanged);
}

void UAerotelicDocumentSubsystem::Deinitialize()
{
	AAerotelicEntityActor::OnGeometryChanged.Remove(GeometryChangedHandle);
	AAerotelicEntityActor::OnBodyStructureChanged.Remove(BodyStructureChangedHandle);
	if (RebuildTick.IsValid())
	{
		FTSTicker::GetCoreTicker().RemoveTicker(RebuildTick);
		RebuildTick.Reset();
	}
	Super::Deinitialize();
}

// ---------------------------------------------------- the panels' Apply

std::unique_ptr<ISolidModel> UAerotelicDocumentSubsystem::CloneEntityModel(const AAerotelicEntityActor* Entity) const
{
	EntityModelSpec Spec;
	if (Entity == nullptr || !SpecFromEntityModel(*Entity, Spec))
	{
		return nullptr;
	}
	try
	{
		std::unique_ptr<ISolidModel> Clone = ModelFromEntitySpec(Entity->GetEntityType(), Spec);
		// The grid density travels beside the parameterisation (it is not
		// in the spec): the draft edits a copy of it on its Mesh tab.
		if (Clone != nullptr) Clone->mesh = Entity->GetModel()->mesh;
		return Clone;
	}
	catch (const std::exception& e)
	{
		UE_LOG(LogTemp, Warning, TEXT("AerotelicDocument: could not clone '%s' for editing: %s"), *Entity->EntityName, UTF8_TO_TCHAR(e.what()));
		return nullptr;
	}
}

bool UAerotelicDocumentSubsystem::ApplyEntityModel(AAerotelicEntityActor* Entity, const ISolidModel& Draft, FString& OutError)
{
	if (Entity == nullptr)
	{
		OutError = TEXT("The entity is no longer in the scene.");
		return false;
	}
	EntityModelSpec Spec;
	if (!SpecFromModel(Entity->GetEntityType(), Draft, Spec))
	{
		OutError = TEXT("This kind of entity has no parameterisation to apply.");
		return false;
	}
	// The draft was cloned under the name the entity had then; a Rename in
	// the scene tree since must not be undone by applying it.
	SetSpecName(Spec, Entity->EntityName);
	std::unique_ptr<ISolidModel> Model;
	try
	{
		Model = ModelFromEntitySpec(Entity->GetEntityType(), Spec);
	}
	catch (const std::exception& e)
	{
		OutError = UTF8_TO_TCHAR(e.what());
		return false;
	}
	if (Model != nullptr) Model->mesh = Draft.mesh;
	if (!Entity->ReplaceModel(std::move(Model), OutError))
	{
		return false;
	}
	// The entity itself is one parameterisation rebuilt, for the progress
	// bar. Its rebuild broadcast OnGeometryChanged, which queued everything
	// over it; queued again here in case it did not (a commandlet's
	// RefreshProxy builds nothing and broadcasts nothing).
	++RebuildTotal;
	++RebuildDone;
	QueueDependentRebuilds(Entity);
	if (RebuildQueue.Num() == 0)
	{
		FinishRebuilds();
	}
	return true;
}

// ------------------------------------------------ the bodies in the file

namespace
{
	// Every body of the entity whose display state or structure differs from
	// the defaults, as the file lists them (`bodies:`). The encoder drops the
	// entries with nothing set.
	std::vector<ProjectBody> ProjectBodiesOf(const AAerotelicEntityActor* Entity)
	{
		std::vector<ProjectBody> Out;
		for (const FAerotelicBody& Body : Entity->GetBodies())
		{
			ProjectBody Pb;
			Pb.name = TCHAR_TO_UTF8(*Body.Name);
			if (Body.bHasColor)
			{
				Pb.color = std::array<double, 3>{ Body.Color.R, Body.Color.G, Body.Color.B };
			}
			Pb.hidden = !Body.bVisible;
			Pb.structure = Body.Structure;
			if (!Pb.IsDefault()) Out.push_back(Pb);
		}
		return Out;
	}

	// The file's bodies onto the actor's, by role name (a role the entity
	// no longer produces is parked by SetBodyStructure's caller -- here it
	// is simply not applied). When no body carries a structure and the file
	// has the old entity-level `mass:`, every body takes that.
	void ApplyProjectBodies(AAerotelicEntityActor* Actor, const std::vector<ProjectBody>& Bodies, const MassSpec& LegacyMass)
	{
		bool bAnyStructure = false;
		for (const ProjectBody& Body : Bodies)
		{
			const FString Role = UTF8_TO_TCHAR(Body.name.c_str());
			const int32 Index = Actor->FindBodyIndex(Role);
			if (Index == INDEX_NONE) continue;
			if (Body.color)
			{
				Actor->SetBodyColor(Index, FLinearColor((float)(*Body.color)[0], (float)(*Body.color)[1], (float)(*Body.color)[2]));
			}
			if (Body.hidden)
			{
				Actor->SetBodyVisible(Index, false);
			}
			if (Body.structure.IsSet() || Body.structure.skin_thickness > 0.0 || !Body.structure.exclusions.empty())
			{
				Actor->SetBodyStructure(Role, Body.structure);
				bAnyStructure = true;
			}
		}
		if (!bAnyStructure && LegacyMass.mode != MassMode::Unset)
		{
			Actor->SetMassSpec(LegacyMass);
		}
	}
}

// ------------------------------------ rebuilds downstream of an Apply

void UAerotelicDocumentSubsystem::OnEntityGeometryChanged(AAerotelicEntityActor* Entity)
{
	QueueDependentRebuilds(Entity);
}

void UAerotelicDocumentSubsystem::OnEntityBodyStructureChanged(AAerotelicEntityActor* Entity, const FString& Role, bool bGeometry)
{
	// Only the geometry the structure describes matters here: a material
	// edit regenerates nothing. A body whose structure has been generated
	// regenerates to the new description; one that lost its skin (a solid
	// now) drops its layers; one never generated waits for Generate.
	if (!bGeometry || Entity == nullptr)
	{
		return;
	}
	const aerotelic::BodyStructure* S = Entity->GetBodyStructure(Role);
	const FAerotelicBodyStructure* G = Entity->GetGeneratedStructure(Role);
	if (S == nullptr)
	{
		return;
	}
	if (!S->IsSkin())
	{
		Entity->ClearGeneratedStructure(Role);
		return;
	}
	// Never generated: a body with structure to build (a wing's plan, a
	// boolean's operand layers) builds as soon as it has a skin; any other
	// waits for Generate.
	if (G == nullptr)
	{
		const int32 Index = Entity->FindBodyIndex(Role);
		if (!Entity->GetBodies().IsValidIndex(Index) || !WantsStructure(Entity, Entity->GetBodies()[Index])) return;
	}
	QueueRebuild({ Entity, Role });
	LastQueueTime = FPlatformTime::Seconds();
	StartRebuildTicker();
}

void UAerotelicDocumentSubsystem::QueueRebuild(const FRebuildItem& Item)
{
	if (!RebuildQueue.Contains(Item))
	{
		RebuildQueue.Add(Item);
		++RebuildTotal;
	}
}

void UAerotelicDocumentSubsystem::QueueDependentRebuilds(const AAerotelicEntityActor* Entity, bool bStructuresOnly)
{
	if (Entity == nullptr)
	{
		return;
	}
	// Entities is in build order -- a boolean result comes after its
	// operands -- so walking it gives the queue its dependency order for
	// free: a boolean over a boolean rebuilds second, and the structures
	// of a boolean's bodies after the boolean.
	for (auto& Weak : Entities)
	{
		if (!Weak.IsValid())
		{
			continue;
		}
		const bool bSelf = Weak.Get() == Entity;
		const bool bOver = !bSelf && DependsOn(Weak.Get(), Entity);
		if (bOver && !bStructuresOnly && Weak->GetEntityType() == EntityType::Boolean)
		{
			QueueRebuild({ Weak, FString() });
		}
		// The generated structures: the entity's own bodies', those of the
		// results over it, and any body whose exclusions name it (a payload
		// bay that moved).
		for (const FAerotelicBody& Body : Weak->GetBodies())
		{
			// Generated once already, or wanting a first generation: the
			// entity's own bodies (a wing whose build just planned one) and the
			// results over it (a boolean whose operand just grew layers).
			if (!Body.Generated.IsValid() && !((bSelf || bOver) && WantsStructure(Weak.Get(), Body))) continue;
			bool bExcludes = false;
			for (const std::string& Name : Body.Structure.exclusions)
			{
				if (FString(UTF8_TO_TCHAR(Name.c_str())) == Entity->EntityName) bExcludes = true;
			}
			if (bSelf || bOver || bExcludes)
			{
				QueueRebuild({ Weak, Body.Name });
			}
		}
	}
	LastQueueTime = FPlatformTime::Seconds();
	StartRebuildTicker();
}

void UAerotelicDocumentSubsystem::StartRebuildTicker()
{
	if (RebuildQueue.Num() > 0 && !RebuildTick.IsValid() && !IsRunningCommandlet())
	{
		RebuildTick = FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateWeakLambda(this, [this](float)
		{
			// One item per tick, so the window repaints -- and the status
			// bar's progress moves -- between them.
			const int32 Index = NextRebuild(true);
			if (Index != INDEX_NONE)
			{
				RebuildAt(Index);
			}
			if (RebuildQueue.Num() > 0)
			{
				return true;
			}
			RebuildTick.Reset();
			FinishRebuilds();
			return false;
		}));
	}
}

int32 UAerotelicDocumentSubsystem::NextRebuild(bool bRespectSettle)
{
	RebuildQueue.RemoveAll([](const FRebuildItem& I) { return !I.Entity.IsValid(); });
	// A boolean goes as soon as its turn comes. A structure (seconds of
	// work) waits for the scene to settle: one Apply changes an entity and
	// then every boolean over it, each broadcasting in turn, and an Editor
	// gizmo drag changes a transform every frame -- the structure is to
	// regenerate once, after all of that.
	const bool bSettled = !bRespectSettle || FPlatformTime::Seconds() - LastQueueTime >= kStructureSettleSeconds;
	for (int32 i = 0; i < RebuildQueue.Num(); ++i)
	{
		if (bSettled || RebuildQueue[i].Role.IsEmpty())
		{
			return i;
		}
	}
	return INDEX_NONE;
}

void UAerotelicDocumentSubsystem::RebuildAt(int32 Index)
{
	const FRebuildItem Item = RebuildQueue[Index];
	RebuildQueue.RemoveAt(Index);
	++RebuildDone;
	if (!Item.Entity.IsValid())
	{
		return;
	}
	if (!Item.Role.IsEmpty())
	{
		FString Error;
		if (!GenerateBodyStructure(Item.Entity.Get(), Item.Role, Error))
		{
			RebuildErrors.Add(FString::Printf(TEXT("%s / %s: %s"), *Item.Entity->EntityName, *Item.Role, *Error));
		}
	}
	else
	{
		// A boolean result over its operands as they now stand. One that
		// cannot be built shows nothing and says why in GetBuildError(),
		// as it always has; its own broadcast queues whatever is over it.
		Item.Entity->RefreshProxy();
	}
}

void UAerotelicDocumentSubsystem::FinishRebuilds()
{
	RebuildTotal = 0;
	RebuildDone = 0;
	if (RebuildErrors.Num() > 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("AerotelicDocument: structures could not follow their bodies: %s"), *FString::Join(RebuildErrors, TEXT("; ")));
		if (!IsRunningCommandlet() && !FApp::IsUnattended())
		{
			AerotelicMessageBox(EAppMsgType::Ok,
				FText::FromString(TEXT("Some bodies' structures could not be regenerated over the bodies as they now stand, and were left as they were:\n") + FString::Join(RebuildErrors, TEXT("\n"))),
				NSLOCTEXT("AerotelicDocument", "StructureFollowFailed", "Regenerate Structures"));
		}
		RebuildErrors.Reset();
	}
}

bool UAerotelicDocumentSubsystem::HasPendingRebuilds() const
{
	for (const FRebuildItem& I : RebuildQueue)
	{
		if (I.Entity.IsValid()) return true;
	}
	return false;
}

float UAerotelicDocumentSubsystem::GetRebuildProgress() const
{
	return RebuildTotal <= 0 ? 1.0f : FMath::Clamp(static_cast<float>(RebuildDone) / static_cast<float>(RebuildTotal), 0.0f, 1.0f);
}

void UAerotelicDocumentSubsystem::FlushPendingRebuilds()
{
	for (int32 Index = NextRebuild(false); Index != INDEX_NONE; Index = NextRebuild(false))
	{
		RebuildAt(Index);
	}
	if (RebuildTick.IsValid())
	{
		FTSTicker::GetCoreTicker().RemoveTicker(RebuildTick);
		RebuildTick.Reset();
	}
	FinishRebuilds();
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

void UAerotelicDocumentSubsystem::SetShadowStyle(const FAerotelicShadowStyle& Style)
{
	ShadowStyle = Style;
	ApplyShadowStyle();
}

void UAerotelicDocumentSubsystem::ApplyShadowStyle()
{
	UWorld* World = GetTargetWorld();
	if (World == nullptr)
	{
		return;
	}
	// The studio rig's key light is the one with ForwardShadingPriority set
	// (SpawnLighting), which is also the only one that casts shadows. The
	// biases go to every directional light so a rig that gains another
	// caster stays consistent, but the cast toggle is the key light's
	// alone: switching it on for the fill lights would put three competing
	// shadows on the model.
	for (TActorIterator<ADirectionalLight> It(World); It; ++It)
	{
		UDirectionalLightComponent* Light = Cast<UDirectionalLightComponent>(It->GetLightComponent());
		if (Light == nullptr)
		{
			continue;
		}
		Light->SetShadowBias(ShadowStyle.DepthBias);
		Light->SetShadowSlopeBias(ShadowStyle.SlopeBias);
		if (Light->ForwardShadingPriority > 0)
		{
			Light->SetCastShadows(ShadowStyle.bCastShadows);
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

int32 UAerotelicDocumentSubsystem::CopyStructuresFromOperand(AAerotelicEntityActor* Boolean)
{
	if (Boolean == nullptr || Boolean->GetEntityType() != EntityType::Boolean)
	{
		return 0;
	}
	const FAerotelicBooleanModel* M = static_cast<const FAerotelicBooleanModel*>(Boolean->GetModel());
	if (M == nullptr || M->Operands.Num() == 0 || !M->Operands[0].Entity.IsValid())
	{
		return 0;
	}
	const BodyStructure Source = M->Operands[0].Entity->GetBodyStructure(0);
	int32 Count = 0;
	for (const FAerotelicBody& Body : Boolean->GetBodies())
	{
		if (Boolean->SetBodyStructure(Body.Name, Source)) ++Count;
	}
	return Count;
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
	Units = aerotelic::UnitSystem{};
	ModelScale = 1.0;
	OnEntityListChanged.Broadcast();
}

bool UAerotelicDocumentSubsystem::SetModelScale(double InScale)
{
	if (!(FMath::IsFinite(InScale) && InScale > 0.0))
	{
		return false;
	}
	if (InScale == ModelScale)
	{
		return true;
	}
	ModelScale = InScale;
	// Every entity's actor transform is its full-scale placement under
	// the model scale: re-applying the placements moves and resizes them,
	// which (OnRootTransformed) refreshes the mirrors and queues every
	// boolean and generated structure over them, as a Transform dialog's
	// Apply does.
	for (auto& Weak : Entities)
	{
		if (Weak.IsValid() && !Weak->IsDerived())
		{
			Weak->ApplyPlacement(Weak->GetPlacement());
		}
	}
	return true;
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
	// Cascade: a boolean result that used this as an operand goes too (and
	// anything built on that), as FeatureTree.Remove does.
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
	return false;
}

// ------------------------------------------------- generated structures

const aerotelic::WingBodyStructurePlan* UAerotelicDocumentSubsystem::WingPlanFor(const AAerotelicEntityActor* Entity, const FString& Role)
{
	if (Entity == nullptr || Entity->GetEntityType() != EntityType::Wing) return nullptr;
	for (const aerotelic::WingBodyStructurePlan& P : Entity->GetLastStructurePlans())
	{
		if (FString(UTF8_TO_TCHAR(P.body.c_str())) == Role) return P.IsEmpty() ? nullptr : &P;
	}
	return nullptr;
}

bool UAerotelicDocumentSubsystem::WantsStructure(const AAerotelicEntityActor* Entity, const FAerotelicBody& Body)
{
	if (!Body.Structure.IsSkin()) return false;
	if (WingPlanFor(Entity, Body.Name) != nullptr) return true;
	return OperandsHaveLayers(Entity);
}

bool UAerotelicDocumentSubsystem::OperandsHaveLayers(const AAerotelicEntityActor* Boolean)
{
	if (Boolean == nullptr || Boolean->GetEntityType() != EntityType::Boolean || Boolean->GetModel() == nullptr) return false;
	const FAerotelicBooleanModel* BM = Boolean->GetModelAs<FAerotelicBooleanModel>();
	for (const FAerotelicBooleanOperand& Op : BM->Operands)
	{
		if (!Op.Entity.IsValid()) continue;
		for (const FAerotelicBody& B : Op.Entity->GetBodies())
		{
			const FAerotelicBodyStructure* G = B.Generated.Get();
			if (G == nullptr || !G->IsGenerated()) continue;
			for (const FAerotelicStructureLayer& L : G->GetLayers()) if (!L.Mesh.IsEmpty()) return true;
		}
	}
	return false;
}

bool UAerotelicDocumentSubsystem::GenerateBodyStructure(AAerotelicEntityActor* Entity, const FString& BodyRole, FString& OutError)
{
	if (Entity == nullptr)
	{
		OutError = TEXT("The entity is no longer in the scene.");
		return false;
	}
	const int32 Index = Entity->FindBodyIndex(BodyRole);
	if (Index == INDEX_NONE)
	{
		OutError = FString::Printf(TEXT("%s has no body '%s'."), *Entity->EntityName, *BodyRole);
		return false;
	}
	const FString Role = Entity->GetBodies()[Index].Name;
	const FString Label = Entity->GetBodies().Num() > 1 ? FString::Printf(TEXT("%s / %s"), *Entity->EntityName, *Role) : Entity->EntityName;
	const aerotelic::BodyStructure S = Entity->GetBodyStructure(Index);
	const std::string Problem = aerotelic::BodyStructureProblem(S);
	if (!Problem.empty())
	{
		OutError = FString::Printf(TEXT("%s: %s."), *Label, UTF8_TO_TCHAR(Problem.c_str()));
		return false;
	}
	if (!S.IsSkin())
	{
		// Nothing to generate: a solid, or no material. What there was goes.
		Entity->ClearGeneratedStructure(Role);
		OutError = S.material.HasDensity()
			? FString::Printf(TEXT("%s is a solid (no skin thickness): there is no interior structure to generate. Set a skin thickness under Structure first."), *Label)
			: FString::Printf(TEXT("%s has no material: set one, with a skin thickness, under Structure first."), *Label);
		return false;
	}

	// A WING's structure comes from its parameterisation (2026-11-09): the
	// plan the wing's last build made for this body on its own grid (rib
	// rims that are rings of the body mesh, spar webs, stringers).
	if (Entity->GetEntityType() == EntityType::Wing)
	{
		const aerotelic::WingBodyStructurePlan* Plan = WingPlanFor(Entity, Role);
		if (Plan == nullptr)
		{
			Entity->ClearGeneratedStructure(Role);
			return true;
		}
		const aerotelic::NamedMesh* Found = nullptr;
		for (const aerotelic::NamedMesh& M : Entity->GetLastMeshes())
		{
			if (FString(UTF8_TO_TCHAR(M.name.c_str())) == Role) { Found = &M; break; }
		}
		if (Found == nullptr || Found->mesh.IsEmpty())
		{
			OutError = FString::Printf(TEXT("%s: no geometry could be built for the body."), *Label);
			return false;
		}
		FAerotelicBodyStructure::FPlanInputs In;
		In.Label = Label;
		In.Body = Found->mesh;
		In.Plan = *Plan;
		In.Structure = S;
		{
			std::array<double, 9> R;
			aerotelic::Vec3 T;
			Entity->GetPlacementMatrix(R, T, In.Scale);
		}
		TSharedPtr<FAerotelicBodyStructure> G = MakeShared<FAerotelicBodyStructure>();
		if (!G->GenerateFromPlan(In, OutError))
		{
			return false;
		}
		Entity->SetGeneratedStructure(Role, G);
		// The boolean results built over this wing take its layers: their
		// bodies follow (first generation or not).
		for (auto& Weak : Entities)
		{
			if (!Weak.IsValid() || Weak.Get() == Entity || !DependsOn(Weak.Get(), Entity)) continue;
			for (const FAerotelicBody& Body : Weak->GetBodies())
			{
				if (Body.Generated.IsValid() || WantsStructure(Weak.Get(), Body)) QueueRebuild({ Weak, Body.Name });
			}
		}
		return true;
	}

	// A BOOLEAN RESULT's structure is its operands' (2026-11-10): every
	// layer the operands' bodies generated -- a wing's ribs, spars and
	// stringers, its mirrored half's too -- placed as the operands are, and
	// given to the result body that holds it (the body most of its vertices
	// fall inside or on; the fuse removed the skins between the operands,
	// so a wing's spars run on through the fuselage as a carry-through).
	// Each keeps its kind, thickness, material and mesh, named
	// "<operand> / <layer>", brought into the result's own frame.
	if (Entity->GetEntityType() == EntityType::Boolean)
	{
		const FAerotelicBooleanModel* BM = Entity->GetModelAs<FAerotelicBooleanModel>();
		const aerotelic::NamedMesh* Own = nullptr;
		for (const aerotelic::NamedMesh& M : Entity->GetLastMeshes())
		{
			if (FString(UTF8_TO_TCHAR(M.name.c_str())) == Role) { Own = &M; break; }
		}
		if (Own == nullptr || Own->mesh.IsEmpty())
		{
			OutError = FString::Printf(TEXT("%s: no geometry could be built for the body."), *Label);
			return false;
		}
		FAerotelicBodyStructure::FLayerInputs In;
		In.Label = Label;
		In.Structure = S;
		In.Body = Own->mesh;
		std::array<double, 9> R;
		aerotelic::Vec3 T;
		Entity->GetPlacementMatrix(R, T, In.Scale);
		// World -> the result's local frame: x = R^T (x' - T) / scale.
		auto Unplace = [&R, &T, &In](aerotelic::TriMesh& Mesh)
		{
			const double Sc = In.Scale != 0.0 ? 1.0 / In.Scale : 1.0;
			const std::array<double, 9> Rt{{R[0] * Sc, R[3] * Sc, R[6] * Sc, R[1] * Sc, R[4] * Sc, R[7] * Sc, R[2] * Sc, R[5] * Sc, R[8] * Sc}};
			const aerotelic::Vec3 Shift{-(Rt[0] * T.x + Rt[1] * T.y + Rt[2] * T.z), -(Rt[3] * T.x + Rt[4] * T.y + Rt[5] * T.z), -(Rt[6] * T.x + Rt[7] * T.y + Rt[8] * T.z)};
			Mesh.Transform(Rt, Shift);
		};
		aerotelic::TriMesh PlacedBody = Own->mesh;
		AerotelicMeshing::PlaceMesh(PlacedBody, Entity->GetActorTransform());
		const aerotelic::MeshSdf Sdf(PlacedBody);
		const double OnSkin = 1e-6 * FMath::Max(PlacedBody.Bounds().Diagonal(), 1e-12);
		int32 Elsewhere = 0;
		if (BM != nullptr)
		{
			for (const FAerotelicBooleanOperand& Op : BM->Operands)
			{
				if (!Op.Entity.IsValid()) continue;
				const AAerotelicEntityActor* OpEntity = Op.Entity.Get();
				const FString OpLabel = OpEntity->EntityName + (Op.bMirror ? TEXT(" (mirror)") : TEXT(""));
				for (const FAerotelicBody& OB : OpEntity->GetBodies())
				{
					const FAerotelicBodyStructure* OG = OB.Generated.Get();
					if (OG == nullptr || !OG->IsGenerated()) continue;
					for (const FAerotelicStructureLayer& L : OG->GetLayers())
					{
						if (L.Mesh.IsEmpty()) continue;
						aerotelic::TriMesh Placed = L.Mesh;
						AerotelicMeshing::PlaceMesh(Placed, OpEntity->GetActorTransform());
						if (Op.bMirror) OpEntity->ReflectMesh(Placed);
						// Inside this result body (or on its skin, as a rim is)?
						// Sampled, by majority.
						int32 Inside = 0, Total = 0;
						for (size_t v = 0; v < Placed.vertices.size(); v += 5)
						{
							++Total;
							if (Sdf.Distance(Placed.vertices[v]) <= OnSkin) ++Inside;
						}
						if (Total == 0 || 2 * Inside < Total) { ++Elsewhere; continue; }
						FAerotelicStructureLayer Copy = L;
						Copy.Name = FString::Printf(TEXT("%s / %s"), *OpLabel, *L.Name);
						Unplace(Placed);
						Copy.Mesh = std::move(Placed);
						In.Layers.push_back(std::move(Copy));
					}
				}
			}
		}
		if (In.Layers.empty())
		{
			Entity->ClearGeneratedStructure(Role);
			return true;
		}
		if (Elsewhere > 0) In.Notes.Add(FString::Printf(TEXT("%d operand layer(s) lie in another body of this result"), Elsewhere));
		TSharedPtr<FAerotelicBodyStructure> G = MakeShared<FAerotelicBodyStructure>();
		if (!G->GenerateFromLayers(In, OutError))
		{
			return false;
		}
		Entity->SetGeneratedStructure(Role, G);
		return true;
	}

	// Any other body has no interior structure of its own yet (a fuselage's
	// or nacelle's frames are to come from their parameterisations as the
	// wing's do; the plane networks that stood in went on 2026-11-10):
	// nothing to generate, and whatever layers it had go.
	Entity->ClearGeneratedStructure(Role);
	return true;
}

bool UAerotelicDocumentSubsystem::ApplyBodyStructure(AAerotelicEntityActor* Entity, const FString& BodyRole, const aerotelic::BodyStructure& Structure, FString& OutError)
{
	if (Entity == nullptr)
	{
		OutError = TEXT("The entity is no longer in the scene.");
		return false;
	}
	if (!Entity->SetBodyStructure(BodyRole, Structure))
	{
		OutError = FString::Printf(TEXT("%s has no body '%s'."), *Entity->EntityName, *BodyRole);
		return false;
	}
	const int32 Index = Entity->FindBodyIndex(BodyRole);
	const FString Role = Index != INDEX_NONE ? Entity->GetBodies()[Index].Name : BodyRole;
	// The change handler queued a regeneration for a generated body; it is
	// done right here instead.
	RebuildQueue.RemoveAll([&](const FRebuildItem& I) { return I.Entity.Get() == Entity && I.Role == Role; });
	if (!Structure.IsSkin())
	{
		// A solid: the handler dropped any layers; nothing to build.
		return true;
	}
	return GenerateBodyStructure(Entity, Role, OutError);
}

int32 UAerotelicDocumentSubsystem::RegenerateStructures(TArray<FString>& OutErrors)
{
	int32 Count = 0;
	// A manual regeneration covers every structure waiting its turn.
	RebuildQueue.RemoveAll([](const FRebuildItem& I) { return !I.Entity.IsValid() || !I.Role.IsEmpty(); });
	for (auto& Weak : Entities)
	{
		if (!Weak.IsValid()) continue;
		TArray<FString> Roles;
		for (const FAerotelicBody& Body : Weak->GetBodies())
		{
			if (Body.Generated.IsValid()) Roles.Add(Body.Name);
		}
		for (const FString& Role : Roles)
		{
			FString Error;
			if (GenerateBodyStructure(Weak.Get(), Role, Error))
			{
				++Count;
			}
			else
			{
				OutErrors.Add(FString::Printf(TEXT("%s / %s: %s"), *Weak->EntityName, *Role, *Error));
			}
		}
	}
	return Count;
}

void UAerotelicDocumentSubsystem::GenerateStructuresWithInterior(TArray<FString>& OutFailures)
{
	for (auto& Weak : Entities)
	{
		if (!Weak.IsValid()) continue;
		TArray<FString> Roles;
		for (const FAerotelicBody& Body : Weak->GetBodies())
		{
			// A skin with plane networks, or a wing body with a structure plan.
			if (WantsStructure(Weak.Get(), Body)) Roles.Add(Body.Name);
		}
		for (const FString& Role : Roles)
		{
			FString Error;
			if (!GenerateBodyStructure(Weak.Get(), Role, Error))
			{
				OutFailures.Add(FString::Printf(TEXT("%s / %s: %s"), *Weak->EntityName, *Role, *Error));
			}
		}
	}
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
	// Build once up front so a failing operation (a degenerate contact the
	// mesh boolean refuses, an operand with no geometry) adds nothing, as
	// fold_boolean cleaned up after itself.
	try
	{
		Model->BuildMeshes(FAerotelicBooleanModel::kValidationTolerance);
	}
	catch (const std::exception& e)
	{
		OutError = UTF8_TO_TCHAR(e.what());
		return nullptr;
	}
	return SpawnEntity(EntityType::Boolean, std::move(Model), Name);
}

// ------------------------------------------------------------ copy / paste

bool UAerotelicDocumentSubsystem::CanCopyEntity(const AAerotelicEntityActor* Entity)
{
	if (Entity == nullptr)
	{
		return false;
	}
	return Entity->GetEntityType() != EntityType::Boolean;
}

bool UAerotelicDocumentSubsystem::CopyEntityToClipboard(const AAerotelicEntityActor* Entity)
{
	if (!CanCopyEntity(Entity))
	{
		return false;
	}
	ProjectEntity Snapshot;
	if (!SpecFromEntityModel(*Entity, Snapshot.model))
	{
		return false;
	}
	Snapshot.name = TCHAR_TO_UTF8(*Entity->EntityName);
	Snapshot.type = Entity->GetEntityType();
	Snapshot.mirror = Entity->IsMirrored();
	{
		const FAerotelicMirrorPlane& Plane = Entity->GetMirrorPlane();
		Snapshot.mirror_normal = { Plane.Normal.X, Plane.Normal.Y, Plane.Normal.Z };
		Snapshot.mirror_offset = Plane.Offset;
	}
	Snapshot.placement = Entity->GetPlacement();
	if (const TOptional<FLinearColor> Color = Entity->GetColorOverride(); Color.IsSet())
	{
		Snapshot.color = std::array<double, 3>{ Color->R, Color->G, Color->B };
	}
	Snapshot.hidden = !Entity->IsEntityVisible();
	Snapshot.bodies = ProjectBodiesOf(Entity);
	Snapshot.mesh = Entity->GetModel()->mesh;
	Clipboard = MoveTemp(Snapshot);
	return true;
}

FString UAerotelicDocumentSubsystem::GetClipboardEntityName() const
{
	return Clipboard.IsSet() ? UTF8_TO_TCHAR(Clipboard->name.c_str()) : FString();
}

AAerotelicEntityActor* UAerotelicDocumentSubsystem::PasteEntity(FString& OutError)
{
	if (!Clipboard.IsSet())
	{
		OutError = TEXT("There is nothing to paste: copy an entity first (Edit > Copy, Ctrl+C).");
		return nullptr;
	}
	// A copy is named like a newly added entity of its kind rather than
	// "Wing 1 2": the source's trailing number, if it has one, is the
	// numbering UniqueLabel is about to redo.
	FString Base = UTF8_TO_TCHAR(Clipboard->name.c_str());
	int32 Space = INDEX_NONE;
	if (Base.FindLastChar(TEXT(' '), Space) && Space > 0 && Space + 1 < Base.Len())
	{
		const FString Tail = Base.Mid(Space + 1);
		if (!Tail.IsEmpty() && Tail.IsNumeric())
		{
			Base = Base.Left(Space);
		}
	}
	const FString Label = UniqueLabel(Base);

	// The spec is a value and this is our own copy of it, so naming it and
	// handing it to the constructor cannot reach back to the source.
	EntityModelSpec Spec = Clipboard->model;
	SetSpecName(Spec, Label);
	std::unique_ptr<ISolidModel> Model;
	try
	{
		Model = ModelFromEntitySpec(Clipboard->type, Spec);
	}
	catch (const std::exception& e)
	{
		OutError = UTF8_TO_TCHAR(e.what());
		return nullptr;
	}
	if (Model == nullptr)
	{
		OutError = TEXT("That kind of entity cannot be copied.");
		return nullptr;
	}
	Model->mesh = Clipboard->mesh;

	AAerotelicEntityActor* Actor = SpawnEntity(Clipboard->type, std::move(Model), Label);
	if (Actor == nullptr)
	{
		OutError = TEXT("Could not place the copy in the scene.");
		return nullptr;
	}
	// The same display state the loader applies, so a pasted entity and a
	// reloaded one are the same thing. Pasted where the source sits: the
	// copy is then dragged or transformed wherever it is wanted, and
	// inventing an offset here would only be a guess at how far.
	Actor->ApplyPlacement(Clipboard->placement);
	Actor->RefreshProxy();
	{
		FAerotelicMirrorPlane Plane;
		Plane.Normal = FVector(Clipboard->mirror_normal[0], Clipboard->mirror_normal[1], Clipboard->mirror_normal[2]);
		Plane.Offset = Clipboard->mirror_offset;
		Actor->SetMirrorPlane(Plane, Clipboard->mirror);
	}
	if (Clipboard->color)
	{
		const auto& C = *Clipboard->color;
		Actor->SetColorOverride(FLinearColor((float)C[0], (float)C[1], (float)C[2]));
	}
	if (Clipboard->hidden)
	{
		Actor->SetEntityVisible(false);
	}
	ApplyProjectBodies(Actor, Clipboard->bodies, Clipboard->mass);
	SetSelectedEntity(Actor);
	return Actor;
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
	Units = Project.units;
	// Set before any placement is applied: an actor's transform is its
	// placement under the model scale.
	ModelScale = Project.scale;
	// Anything that could not be built: entities whose model refuses (a
	// wing with a negative chord), boolean operations, body structures,
	// and what an older file carried that the application no longer
	// supports (2026-11-08): lattice entities (a kind that no longer
	// exists) and a body's lattice infill (the `infill:` key the loader
	// dropped; the body keeps the rest of its structure). Reported together
	// at the end; the rest of the project still opens.
	TArray<FString> Failures;
	for (const std::string& Name : Project.unsupported_lattices)
	{
		Failures.Add(FString::Printf(TEXT("%s: lattice entities are no longer supported and this one was skipped; a body's structure is its skin and its plane networks (Scene tree > body > Structure...)"), UTF8_TO_TCHAR(Name.c_str())));
	}
	for (const std::string& Body : Project.ignored_infill)
	{
		Failures.Add(FString::Printf(TEXT("%s: its lattice infill was dropped -- infill is no longer supported; the body keeps its skin and plane networks"), UTF8_TO_TCHAR(Body.c_str())));
	}
	for (const ProjectEntity& Entity : Project.entities)
	{
		std::unique_ptr<ISolidModel> Model = ModelFromEntitySpec(Entity.type, Entity.model);
		if (Model == nullptr)
		{
			continue;
		}
		Model->mesh = Entity.mesh;
		ResolveReferenceImagePaths(Model.get(), Entity.type, FPaths::GetPath(Path));
		const FString Name = UTF8_TO_TCHAR(Entity.name.c_str());
		AAerotelicEntityActor* Actor = SpawnEntity(Entity.type, std::move(Model), Name);
		if (Actor != nullptr)
		{
			Actor->ApplyPlacement(Entity.placement);
			Actor->RefreshProxy();
			if (!Actor->GetBuildError().IsEmpty())
			{
				Failures.Add(FString::Printf(TEXT("%s: %s"), *Name, *Actor->GetBuildError()));
			}
			// Display state saved with the entity: the mirrored half (and its
			// plane), a colour override, hidden. Absent means the defaults.
			{
				FAerotelicMirrorPlane Plane;
				Plane.Normal = FVector(Entity.mirror_normal[0], Entity.mirror_normal[1], Entity.mirror_normal[2]);
				Plane.Offset = Entity.mirror_offset;
				Actor->SetMirrorPlane(Plane, Entity.mirror);
			}
			if (Entity.color)
			{
				const auto& C = *Entity.color;
				Actor->SetColorOverride(FLinearColor((float)C[0], (float)C[1], (float)C[2]));
			}
			if (Entity.hidden)
			{
				Actor->SetEntityVisible(false);
			}
			// The bodies' colours, visibility and structure; a file from
			// before the bodies carried their own (one `mass:` for the
			// entity) gives every body that setting.
			ApplyProjectBodies(Actor, Entity.bodies, Entity.mass);
		}
	}
	// The boolean operations, in build order, over what is now in the
	// scene (main_window.py _load_boolean). One that cannot be built is
	// reported and skipped; the rest of the project still opens.
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
		// A `mass:` on a boolean in an older file is ignored (it inherited
		// its first operand's then); its bodies carry their own now.
		if (B.color)
		{
			Actor->SetColorOverride(FLinearColor((float)(*B.color)[0], (float)(*B.color)[1], (float)(*B.color)[2]));
		}
		ApplyProjectBodies(Actor, B.bodies, MassSpec());
		if (B.hidden)
		{
			Actor->SetEntityVisible(false);
		}
	}
	// The structures with an interior (plane networks behind a skin),
	// generated over the bodies now in the scene -- the generated geometry
	// is not saved, only what describes it.
	GenerateStructuresWithInterior(Failures);
	if (Failures.Num() > 0 && !IsRunningCommandlet() && !FApp::IsUnattended())
	{
		AerotelicMessageBox(EAppMsgType::Ok,
			FText::FromString(TEXT("Some of the project could not be built. Entities that failed are listed but show no geometry until fixed; boolean operations that failed were left out; a body whose structure failed shows the body alone:\n\n") + FString::Join(Failures, TEXT("\n"))),
			FText::FromString(TEXT("Project Opened With Problems")));
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
	Project.units = Units;
	Project.scale = ModelScale;
	for (auto& WeakEntity : Entities)
	{
		if (!WeakEntity.IsValid())
		{
			continue;
		}
		AAerotelicEntityActor* Entity = WeakEntity.Get();
		if (Entity->GetEntityType() == EntityType::Boolean)
		{
			continue; // written under `booleans:` below
		}
		ProjectEntity Pe;
		Pe.name = TCHAR_TO_UTF8(*Entity->EntityName);
		Pe.type = Entity->GetEntityType();
		Pe.mirror = Entity->IsMirrored();
		{
			const FAerotelicMirrorPlane& Plane = Entity->GetMirrorPlane();
			Pe.mirror_normal = { Plane.Normal.X, Plane.Normal.Y, Plane.Normal.Z };
			Pe.mirror_offset = Plane.Offset;
		}
		Pe.placement = Entity->GetPlacement();
		// Written only when set, as the Qt app does: defaults never reach
		// the file.
		if (const TOptional<FLinearColor> Color = Entity->GetColorOverride(); Color.IsSet())
		{
			Pe.color = std::array<double, 3>{ Color->R, Color->G, Color->B };
		}
		Pe.hidden = !Entity->IsEntityVisible();
		// The bodies carry the material, skin and structure (`bodies:`); the
		// entity-level `mass:` is no longer written.
		Pe.bodies = ProjectBodiesOf(Entity);
		if (Entity->GetModel() != nullptr) Pe.mesh = Entity->GetModel()->mesh;
		SpecFromEntityModel(*Entity, Pe.model);
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
		// Its bodies' display state and structure (a boolean result's bodies
		// own theirs).
		Pb.bodies = ProjectBodiesOf(Entity);
		Project.booleans.push_back(std::move(Pb));
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
