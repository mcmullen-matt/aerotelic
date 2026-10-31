#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/Text/STextBlock.h"

class AAerotelicEntityActor;
struct FAerotelicDesignContent;
namespace aerotelic { class PrimitiveModel; }

// Slate port of ui/primitive_panel.py's Shape tab: the shape selector plus
// the dimension rows for whichever shape is current (the others are hidden,
// never rebuilt, and the model keeps every shape's numbers), with Volume /
// Surface area / Extents read-outs underneath. Edits the panel's DRAFT
// model (see FAerotelicDesignDraft), firing OnChanged on every edit;
// nothing reaches the entity until Apply.
class SPrimitiveDialog : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SPrimitiveDialog) {}
		SLATE_EVENT(FSimpleDelegate, OnChanged)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs, aerotelic::PrimitiveModel* InModel);

private:
	aerotelic::PrimitiveModel* DraftModel = nullptr;
	FSimpleDelegate OnChangedDelegate;

	TArray<TSharedPtr<FString>> ShapeOptions;
	TSharedPtr<STextBlock> VolumeLabel, AreaLabel, ExtentsLabel;

	EVisibility ShapeRowVisibility(int32 ShapeIndex) const;
	void RefreshReadouts();
	void Changed();
};

// Builds this entity's Primitive design panel (Shape + Help tabs) over
// Model, the panel's draft of the entity's primitive.
void BuildPrimitiveDesignContent(AAerotelicEntityActor* Entity, aerotelic::PrimitiveModel* Model, FSimpleDelegate OnChanged, FAerotelicDesignContent& Out);
