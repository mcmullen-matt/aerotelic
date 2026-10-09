#include "EntityDesignContent.h"

#include "Entity/AerotelicEntityActor.h"
#include "Dialogs/WingDialog.h"
#include "Dialogs/FuselageDialog.h"
#include "Dialogs/NacelleDialog.h"
#include "Dialogs/LoftedNacelleDialog.h"
#include "Dialogs/PropellerDialog.h"
#include "Dialogs/PrimitiveDialog.h"

using namespace aerotelic;

FAerotelicDesignContent MakeEntityDesignContent(AAerotelicEntityActor* Entity)
{
	FAerotelicDesignContent Out;
	if (Entity == nullptr)
	{
		return Out;
	}
	switch (Entity->GetEntityType())
	{
	case project_yaml::EntityType::Wing: BuildWingDesignContent(Entity, Out); break;
	case project_yaml::EntityType::Fuselage: BuildFuselageDesignContent(Entity, Out); break;
	case project_yaml::EntityType::Nacelle: BuildNacelleDesignContent(Entity, Out); break;
	case project_yaml::EntityType::LoftedNacelle: BuildLoftedNacelleDesignContent(Entity, Out); break;
	case project_yaml::EntityType::Propeller: BuildPropellerDesignContent(Entity, Out); break;
	case project_yaml::EntityType::Primitive: BuildPrimitiveDesignContent(Entity, Out); break;
	}
	return Out;
}
