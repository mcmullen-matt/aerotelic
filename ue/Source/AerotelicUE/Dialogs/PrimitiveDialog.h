#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/Text/STextBlock.h"

class AAerotelicEntityActor;
struct FAerotelicDesignContent;

// Slate port of ui/primitive_panel.py's Shape tab: the shape selector plus
// the dimension rows for whichever shape is current (the others are hidden,
// never rebuilt, and the model keeps every shape's numbers), with Volume /
// Surface area / Extents read-outs underneath.
class SPrimitiveDialog : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SPrimitiveDialog) {}
		SLATE_EVENT(FSimpleDelegate, OnChanged)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs, AAerotelicEntityActor* InEntity);

private:
	AAerotelicEntityActor* Entity = nullptr;
	FSimpleDelegate OnChangedDelegate;

	TArray<TSharedPtr<FString>> ShapeOptions;
	TSharedPtr<STextBlock> VolumeLabel, AreaLabel, ExtentsLabel;

	EVisibility ShapeRowVisibility(int32 ShapeIndex) const;
	void RefreshReadouts();
	void Changed();
};

// Builds this entity's Primitive design panel (Shape + Help tabs).
void BuildPrimitiveDesignContent(AAerotelicEntityActor* Entity, FAerotelicDesignContent& Out);
