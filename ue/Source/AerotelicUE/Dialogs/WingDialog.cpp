#include "WingDialog.h"
#include "WingDesignTabs.h"
#include "EntityDesignContent.h"
#include "MeshTab.h"
#include "Widgets/TabStrip.h"
#include "Widgets/CurveTabControls.h"

#include "Entity/AerotelicEntityActor.h"
#include "aerotelic/wing_model.hpp"

using namespace aerotelic;

void BuildWingDesignContent(AAerotelicEntityActor* Entity, WingModel* Model, FSimpleDelegate OnChanged, FAerotelicDesignContent& Out)
{
	if (Entity == nullptr || Model == nullptr || Entity->GetEntityType() != project_yaml::EntityType::Wing)
	{
		return;
	}

	// Every edit marks the draft; a wing that cannot be built as drafted (a
	// negative chord) is reported when Apply refuses it, and the scene
	// keeps the wing as it was.
	TSharedRef<FWingDesignTabs> DesignTabs = MakeShared<FWingDesignTabs>(Model, OnChanged);

	// The Mesh tab goes before the wing's Help (the last of its tabs).
	TArray<FAerotelicTab> Tabs;
	const auto& WingTabs = DesignTabs->GetTabs();
	for (int32 i = 0; i < WingTabs.Num(); ++i)
	{
		if (i == WingTabs.Num() - 1)
		{
			Tabs.Add({ NSLOCTEXT("WingDialog", "MeshTab", "Mesh"), MakeMeshTab(Model, project_yaml::EntityType::Wing, OnChanged) });
			Tabs.Add({ WingTabs[i].Key, MakeHelpTab(FText::Join(FText::FromString(TEXT("\n\n")), DesignTabs->GetHelpText(), MeshTabHelpText(project_yaml::EntityType::Wing))) });
			break;
		}
		Tabs.Add({ WingTabs[i].Key, WingTabs[i].Value });
	}

	Out.Title = FText::Format(NSLOCTEXT("WingDialog", "Title", "Wing -- {0}"), FText::FromString(Entity->EntityName));
	// Keeps FWingDesignTabs (and the model-bound widgets capturing `this`
	// from it) alive exactly as long as the panel that references them.
	Out.WingTabs = DesignTabs;
	Out.Widget = SNew(SAerotelicTabStrip, Tabs);
}
