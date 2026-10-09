#include "WingDialog.h"
#include "WingDesignTabs.h"
#include "EntityDesignContent.h"
#include "Widgets/TabStrip.h"

#include "Entity/AerotelicEntityActor.h"
#include "aerotelic/wing_model.hpp"

using namespace aerotelic;

void BuildWingDesignContent(AAerotelicEntityActor* Entity, FAerotelicDesignContent& Out)
{
	if (Entity == nullptr || Entity->GetEntityType() != project_yaml::EntityType::Wing)
	{
		return;
	}

	WingModel* Model = Entity->GetModelAs<WingModel>();
	TWeakObjectPtr<AAerotelicEntityActor> WeakEntity = Entity;
	TSharedRef<FWingDesignTabs> DesignTabs = MakeShared<FWingDesignTabs>(Model, FSimpleDelegate::CreateLambda([WeakEntity]()
	{
		if (WeakEntity.IsValid())
		{
			WeakEntity->RefreshProxy();
		}
	}));

	TArray<FAerotelicTab> Tabs;
	for (const auto& Tab : DesignTabs->GetTabs())
	{
		Tabs.Add({ Tab.Key, Tab.Value });
	}

	Out.Title = FText::Format(NSLOCTEXT("WingDialog", "Title", "Wing -- {0}"), FText::FromString(Entity->EntityName));
	// Keeps FWingDesignTabs (and the model-bound widgets capturing `this`
	// from it) alive exactly as long as the panel that references them.
	Out.WingTabs = DesignTabs;
	Out.Widget = SNew(SAerotelicTabStrip, Tabs);
}
