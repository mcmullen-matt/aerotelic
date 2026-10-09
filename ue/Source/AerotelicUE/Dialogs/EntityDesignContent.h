#pragma once

#include "CoreMinimal.h"

class AAerotelicEntityActor;
class SWidget;
class FWingDesignTabs;

// A design panel for one entity: what the Qt version shows in that entity
// kind's dock widget (Wing Design, Fuselage Design, ...). Built once per
// entity by the design dock and kept while the entity exists, so tab state
// survives switching between entities.
struct FAerotelicDesignContent
{
	TSharedPtr<SWidget> Widget;
	// e.g. "Wing -- Wing 8"; the panel's header line.
	FText Title;
	// Wing and Propeller panels bind their widgets to this object; it has to
	// live exactly as long as the widget does.
	TSharedPtr<FWingDesignTabs> WingTabs;
};

// Dispatches on Entity's type. Widget is null if the type is unknown.
FAerotelicDesignContent MakeEntityDesignContent(AAerotelicEntityActor* Entity);
