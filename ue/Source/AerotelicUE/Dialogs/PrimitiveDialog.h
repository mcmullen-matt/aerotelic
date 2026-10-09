#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/Text/STextBlock.h"

class AAerotelicEntityActor;
struct FAerotelicDesignContent;

// Slate port of ui/primitive_panel.py's Shape tab (the Help tab's static
// text isn't ported -- it has no editable state and nothing in this app's
// scope reads the Transform-dialog cross-reference it makes). Shape buttons
// plus the dimension rows for whichever shape is current, with Volume /
// Surface area / Extents readouts underneath, exactly as the Python panel
// shows them.
class SPrimitiveDialog : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SPrimitiveDialog) {}
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs, AAerotelicEntityActor* InEntity);

private:
	AAerotelicEntityActor* Entity = nullptr;

	TSharedPtr<STextBlock> VolumeLabel, AreaLabel, ExtentsLabel;

	FReply OnShapeButtonClicked(int32 ShapeIndex);
	EVisibility ShapeRowVisibility(int32 ShapeIndex) const;
	void RefreshReadouts();
	void Changed();
};

// Builds this entity's Primitive design panel.
void BuildPrimitiveDesignContent(AAerotelicEntityActor* Entity, FAerotelicDesignContent& Out);
