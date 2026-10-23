#include "EntityDesignContent.h"

#include "Dialogs/WingDialog.h"
#include "Dialogs/FuselageDialog.h"
#include "Dialogs/NacelleDialog.h"
#include "Dialogs/LoftedNacelleDialog.h"
#include "Dialogs/PropellerDialog.h"
#include "Dialogs/PrimitiveDialog.h"

#include "aerotelic/wing_model.hpp"
#include "aerotelic/fuselage_model.hpp"
#include "aerotelic/nacelle_model.hpp"
#include "aerotelic/lofted_nacelle_model.hpp"
#include "aerotelic/propeller_model.hpp"
#include "aerotelic/primitive_model.hpp"

// Last: OCCT's Handle() macro vs engine headers.
#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"

using namespace aerotelic;

FAerotelicDesignDraft::FAerotelicDesignDraft() = default;
FAerotelicDesignDraft::~FAerotelicDesignDraft() = default;

FAerotelicDesignContent MakeEntityDesignContent(AAerotelicEntityActor* Entity)
{
	FAerotelicDesignContent Out;
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	if (Entity == nullptr || Doc == nullptr)
	{
		return Out;
	}
	std::unique_ptr<ISolidModel> Model = Doc->CloneEntityModel(Entity);
	if (Model == nullptr)
	{
		return Out; // a boolean result or a lattice: nothing of its own to edit here
	}
	Out.Draft = MakeShared<FAerotelicDesignDraft>();
	Out.Draft->Model = std::move(Model);

	// Every edit in the panel lands here: the draft is marked, nothing in
	// the scene changes. (Weak: the widgets outlive nothing, but a delegate
	// captured into one must not keep the draft alive past the panel.)
	TWeakPtr<FAerotelicDesignDraft> WeakDraft = Out.Draft;
	FSimpleDelegate OnChanged = FSimpleDelegate::CreateLambda([WeakDraft]()
	{
		if (TSharedPtr<FAerotelicDesignDraft> D = WeakDraft.Pin())
		{
			D->bDirty = true;
		}
	});

	ISolidModel* Draft = Out.Draft->Model.get();
	switch (Entity->GetEntityType())
	{
	case project_yaml::EntityType::Wing: BuildWingDesignContent(Entity, static_cast<WingModel*>(Draft), OnChanged, Out); break;
	case project_yaml::EntityType::Fuselage: BuildFuselageDesignContent(Entity, static_cast<FuselageModel*>(Draft), OnChanged, Out); break;
	case project_yaml::EntityType::Nacelle: BuildNacelleDesignContent(Entity, static_cast<NacelleModel*>(Draft), OnChanged, Out); break;
	case project_yaml::EntityType::LoftedNacelle: BuildLoftedNacelleDesignContent(Entity, static_cast<LoftedNacelleModel*>(Draft), OnChanged, Out); break;
	case project_yaml::EntityType::Propeller: BuildPropellerDesignContent(Entity, static_cast<PropellerModel*>(Draft), OnChanged, Out); break;
	case project_yaml::EntityType::Primitive: BuildPrimitiveDesignContent(Entity, static_cast<PrimitiveModel*>(Draft), OnChanged, Out); break;
	case project_yaml::EntityType::Boolean: break;
	case project_yaml::EntityType::Lattice: break;
	}
	if (!Out.Widget.IsValid())
	{
		Out.Draft.Reset();
	}
	return Out;
}
