#include "WingDialog.h"
#include "WingDesignTabs.h"
#include "EntityDesignContent.h"
#include "Widgets/TabStrip.h"

#include "Entity/AerotelicEntityActor.h"
#include "Widgets/MessageBox.h"
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
	// The reason the last rebuild was refused, so the user is told once
	// when an edit makes the wing unbuildable (a negative chord), not on
	// every further edit until it is fixed.
	TSharedRef<FString> LastReported = MakeShared<FString>(Entity->GetBuildError());
	TSharedRef<FWingDesignTabs> DesignTabs = MakeShared<FWingDesignTabs>(Model, FSimpleDelegate::CreateLambda([WeakEntity, LastReported]()
	{
		if (WeakEntity.IsValid())
		{
			WeakEntity->RefreshProxy();
			const FString& Error = WeakEntity->GetBuildError();
			if (!Error.IsEmpty() && Error != *LastReported)
			{
				AerotelicMessageBox(EAppMsgType::Ok,
					FText::FromString(FString::Printf(TEXT("'%s' cannot be built as it now stands and shows no geometry until this is fixed:\n\n%s"), *WeakEntity->EntityName, *Error)),
					NSLOCTEXT("WingDialog", "BuildRefused", "Wing Cannot Be Built"));
			}
			*LastReported = Error;
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
