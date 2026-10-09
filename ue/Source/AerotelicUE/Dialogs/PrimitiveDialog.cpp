#include "PrimitiveDialog.h"

#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SSeparator.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Input/SButton.h"

#include "Entity/AerotelicEntityActor.h"
#include "Widgets/NumericEntry.h"
#include "EntityDesignContent.h"

#include "aerotelic/primitive_model.hpp"

using aerotelic::PrimitiveModel;
using aerotelic::PrimitiveShape;

namespace
{
	TSharedRef<SWidget> Row(const FText& Label, TSharedRef<SWidget> Field)
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
			[ SNew(STextBlock).Text(Label) ]
			+ SHorizontalBox::Slot().FillWidth(1.2f)
			[ Field ];
	}

	TSharedRef<STextBlock> Readout()
	{
		return SNew(STextBlock).Text(FText::FromString(TEXT("--")));
	}
}

void SPrimitiveDialog::Construct(const FArguments& InArgs, AAerotelicEntityActor* InEntity)
{
	Entity = InEntity;
	PrimitiveModel* Model = Entity->GetModelAs<PrimitiveModel>();

	auto ShapeAttr = [Model]() { return Model->shape; };

	auto NumRow = [this, Model](const FText& Label, TFunction<double()> Get, TFunction<void(double)> Set)
	{
		return Row(Label,
			SNew(SAerotelicNumericEntry)
			.Value_Lambda(MoveTemp(Get))
			.MinValue(1.0e-6)
			.MaxValue(1.0e9)
			.Delta(0.1)
			.LiveUpdate(false)
			.OnValueCommitted_Lambda([this, Set](double V, ETextCommit::Type) { Set(V); Changed(); }));
	};

	ChildSlot
	[
		SNew(SBorder).Padding(12)
		[
			SNew(SVerticalBox)

			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 4, 0)
				[ SNew(SButton).Text(FText::FromString(TEXT("Sphere"))).OnClicked(this, &SPrimitiveDialog::OnShapeButtonClicked, 0) ]
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 4, 0)
				[ SNew(SButton).Text(FText::FromString(TEXT("Box"))).OnClicked(this, &SPrimitiveDialog::OnShapeButtonClicked, 1) ]
				+ SHorizontalBox::Slot().AutoWidth()
				[ SNew(SButton).Text(FText::FromString(TEXT("Cylinder"))).OnClicked(this, &SPrimitiveDialog::OnShapeButtonClicked, 2) ]
			]

			// ---- Sphere rows ----
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
			[
				SNew(SBox).Visibility(TAttribute<EVisibility>::CreateLambda([this]() { return ShapeRowVisibility(0); }))
				[
					NumRow(FText::FromString(TEXT("Radius:")),
						[Model]() { return Model->radius; },
						[Model](double V) { Model->radius = V; })
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
			[
				SNew(SBox).Visibility(TAttribute<EVisibility>::CreateLambda([this]() { return ShapeRowVisibility(0); }))
				[
					NumRow(FText::FromString(TEXT("Stretch X:")),
						[Model]() { return Model->stretch[0]; },
						[Model](double V) { Model->stretch[0] = V; })
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
			[
				SNew(SBox).Visibility(TAttribute<EVisibility>::CreateLambda([this]() { return ShapeRowVisibility(0); }))
				[
					NumRow(FText::FromString(TEXT("Stretch Y:")),
						[Model]() { return Model->stretch[1]; },
						[Model](double V) { Model->stretch[1] = V; })
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
			[
				SNew(SBox).Visibility(TAttribute<EVisibility>::CreateLambda([this]() { return ShapeRowVisibility(0); }))
				[
					NumRow(FText::FromString(TEXT("Stretch Z:")),
						[Model]() { return Model->stretch[2]; },
						[Model](double V) { Model->stretch[2] = V; })
				]
			]

			// ---- Box rows ----
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
			[
				SNew(SBox).Visibility(TAttribute<EVisibility>::CreateLambda([this]() { return ShapeRowVisibility(1); }))
				[
					NumRow(FText::FromString(TEXT("Length (X):")),
						[Model]() { return Model->size[0]; },
						[Model](double V) { Model->size[0] = V; })
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
			[
				SNew(SBox).Visibility(TAttribute<EVisibility>::CreateLambda([this]() { return ShapeRowVisibility(1); }))
				[
					NumRow(FText::FromString(TEXT("Width (Y):")),
						[Model]() { return Model->size[1]; },
						[Model](double V) { Model->size[1] = V; })
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
			[
				SNew(SBox).Visibility(TAttribute<EVisibility>::CreateLambda([this]() { return ShapeRowVisibility(1); }))
				[
					NumRow(FText::FromString(TEXT("Height (Z):")),
						[Model]() { return Model->size[2]; },
						[Model](double V) { Model->size[2] = V; })
				]
			]

			// ---- Cylinder rows ----
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
			[
				SNew(SBox).Visibility(TAttribute<EVisibility>::CreateLambda([this]() { return ShapeRowVisibility(2); }))
				[
					NumRow(FText::FromString(TEXT("Radius:")),
						[Model]() { return Model->cyl_radius; },
						[Model](double V) { Model->cyl_radius = V; })
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
			[
				SNew(SBox).Visibility(TAttribute<EVisibility>::CreateLambda([this]() { return ShapeRowVisibility(2); }))
				[
					NumRow(FText::FromString(TEXT("Length (X):")),
						[Model]() { return Model->cyl_length; },
						[Model](double V) { Model->cyl_length = V; })
				]
			]

			+ SVerticalBox::Slot().AutoHeight().Padding(0, 8) [ SNew(SSeparator) ]

			+ SVerticalBox::Slot().AutoHeight() [ Row(FText::FromString(TEXT("Volume:")), (VolumeLabel = Readout()).ToSharedRef()) ]
			+ SVerticalBox::Slot().AutoHeight() [ Row(FText::FromString(TEXT("Surface area:")), (AreaLabel = Readout()).ToSharedRef()) ]
			+ SVerticalBox::Slot().AutoHeight() [ Row(FText::FromString(TEXT("Extents (X x Y x Z):")), (ExtentsLabel = Readout()).ToSharedRef()) ]

			+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0)
			[
				SNew(STextBlock)
				.AutoWrapText(true)
				.ColorAndOpacity(FLinearColor(0.55f, 0.55f, 0.55f))
				.Text(FText::FromString(TEXT(
					"The shape is built centred on the origin. To place it, right-click "
					"the entity in the Scene tree and choose Transform.")))
			]
		]
	];

	RefreshReadouts();
}

FReply SPrimitiveDialog::OnShapeButtonClicked(int32 ShapeIndex)
{
	PrimitiveModel* Model = Entity->GetModelAs<PrimitiveModel>();
	Model->shape = static_cast<PrimitiveShape>(ShapeIndex);
	Changed();
	return FReply::Handled();
}

EVisibility SPrimitiveDialog::ShapeRowVisibility(int32 ShapeIndex) const
{
	PrimitiveModel* Model = Entity->GetModelAs<PrimitiveModel>();
	return static_cast<int32>(Model->shape) == ShapeIndex ? EVisibility::Visible : EVisibility::Collapsed;
}

void SPrimitiveDialog::RefreshReadouts()
{
	PrimitiveModel* Model = Entity->GetModelAs<PrimitiveModel>();
	const std::array<double, 3> Extents = Model->Extents();
	VolumeLabel->SetText(FText::FromString(FString::Printf(TEXT("%.4g"), Model->Volume())));
	AreaLabel->SetText(FText::FromString(FString::Printf(TEXT("%.4g"), Model->SurfaceArea())));
	ExtentsLabel->SetText(FText::FromString(FString::Printf(TEXT("%.4g x %.4g x %.4g"), Extents[0], Extents[1], Extents[2])));
}

void SPrimitiveDialog::Changed()
{
	RefreshReadouts();
	Entity->RefreshProxy();
}

void BuildPrimitiveDesignContent(AAerotelicEntityActor* Entity, FAerotelicDesignContent& Out)
{
	if (Entity == nullptr || Entity->GetEntityType() != aerotelic::project_yaml::EntityType::Primitive)
	{
		return;
	}
	Out.Title = FText::FromString(FString::Printf(TEXT("Primitive -- %s"), *Entity->EntityName));
	Out.Widget = SNew(SPrimitiveDialog, Entity);
}
