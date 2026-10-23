#pragma once

#include "CoreMinimal.h"
#include <memory>

class AAerotelicEntityActor;
class SWidget;
class FWingDesignTabs;
namespace aerotelic { class ISolidModel; }

// What a design panel edits: a DRAFT of the entity's parameterisation, an
// independent copy (UAerotelicDocumentSubsystem::CloneEntityModel). Every
// control in the panel reads and writes this model, never the entity's,
// and nothing in the scene changes until the panel's Apply hands it over
// (ApplyEntityModel); Revert throws the draft away and the panel is rebuilt
// on a fresh copy. bDirty: edited since the last Apply (or since it was
// cloned), which is what enables the two buttons.
//
// Owned by FAerotelicDesignContent beside the widgets that point into it,
// and destroyed after them.
struct FAerotelicDesignDraft
{
	std::unique_ptr<aerotelic::ISolidModel> Model;
	bool bDirty = false;

	FAerotelicDesignDraft();
	~FAerotelicDesignDraft();
};

// A design panel for one entity: what the Qt version shows in that entity
// kind's dock widget (Wing Design, Fuselage Design, ...). Built once per
// entity by the design dock and kept while the entity exists, so tab state
// (and the draft) survives switching between entities.
struct FAerotelicDesignContent
{
	// Declared first so it is destroyed last: the widgets below hold raw
	// pointers into the draft's model.
	TSharedPtr<FAerotelicDesignDraft> Draft;
	TSharedPtr<SWidget> Widget;
	// e.g. "Wing -- Wing 8"; the panel's header line.
	FText Title;
	// Wing and Propeller panels bind their widgets to this object; it has to
	// live exactly as long as the widget does.
	TSharedPtr<FWingDesignTabs> WingTabs;
};

// Dispatches on Entity's type, building the panel over a fresh draft of its
// model. Widget is null if the type has no panel (a boolean result or a
// lattice, edited through their own dialogs).
FAerotelicDesignContent MakeEntityDesignContent(AAerotelicEntityActor* Entity);
