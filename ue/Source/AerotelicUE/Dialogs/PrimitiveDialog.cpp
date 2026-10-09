#include "PrimitiveDialog.h"

#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Input/SComboBox.h"

#include "Entity/AerotelicEntityActor.h"
#include "Widgets/NumericEntry.h"
#include "Widgets/TabStrip.h"
#include "Widgets/CurveTabControls.h"
#include "EntityDesignContent.h"

#include "aerotelic/primitive_model.hpp"

#define LOCTEXT_NAMESPACE "PrimitiveDialog"

using aerotelic::PrimitiveModel;
using aerotelic::PrimitiveShape;

namespace
{
	const FText kHelp = LOCTEXT("Help",
		"WHAT A PRIMITIVE IS FOR\n"
		"A canonical solid — a sphere, a box or a cylinder — sized by a few numbers, for marking reserved interior regions (payload bays, equipment volumes, anywhere the lattice fill must leave alone). "
		"It has no aerodynamic meaning and no curves to edit.\n\n"
		"SHAPE\n"
		"Sphere: a Radius about the centre plus a Stretch along each of X, Y and Z scaling the radius that way (all three at 1 is a true sphere, anything else an ellipsoid). "
		"Box: Length along X (the fuselage axis), Width along Y (lateral), Height along Z (vertical); equal sides make the default cube. "
		"Cylinder: Radius and Length, with its axis along X. Changing the shape keeps the other shapes' numbers.\n\n"
		"PUTTING IT IN PLACE\n"
		"Every primitive is built centred on the origin. To move it, right-click its row in the Scene tree and choose Transform: the translation there is the centre's position and the rotation turns it about that centre.\n\n"
		"READOUTS\n"
		"Volume is exact. Surface area is exact too except for an ellipsoid, which has no closed form and is integrated numerically well beyond the digits shown. Extents are the overall size along X, Y and Z before any rotation.");
}

void SPrimitiveDialog::Construct(const FArguments& InArgs, AAerotelicEntityActor* InEntity)
{
	Entity = InEntity;
	OnChangedDelegate = InArgs._OnChanged;
	PrimitiveModel* Model = Entity->GetModelAs<PrimitiveModel>();

	ShapeOptions.Add(MakeShared<FString>(TEXT("Sphere")));
	ShapeOptions.Add(MakeShared<FString>(TEXT("Box")));
	ShapeOptions.Add(MakeShared<FString>(TEXT("Cylinder")));

	auto Row = [](const FText& Label, TSharedRef<SWidget> Field, TAttribute<EVisibility> Vis, const FText& Tip)
	{
		return SNew(SHorizontalBox)
			.Visibility(Vis)
			.ToolTipText(Tip)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 2, 8, 2)
			[ SNew(SBox).MinDesiredWidth(90) [ SNew(STextBlock).Text(Label) ] ]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 2)
			[ Field ];
	};
	// Every dimension: 1e-9..1e9, step 0.1, committed on Enter/focus-out.
	auto NumRow = [this, Model, Row](const FText& Label, int32 Shape, TFunction<double()> Get, TFunction<void(double)> Set, const FText& Tip)
	{
		return Row(Label,
			SNew(SAerotelicNumericEntry)
			.Value_Lambda([Get]() { return Get(); })
			.MinValue(1.0e-9).MaxValue(1.0e9).Delta(0.1).LiveUpdate(false)
			.OnValueCommitted_Lambda([this, Get, Set](double V, ETextCommit::Type) { if (!FMath::IsNearlyEqual(V, Get())) { Set(V); Changed(); } }),
			TAttribute<EVisibility>::CreateLambda([this, Shape]() { return ShapeRowVisibility(Shape); }), Tip);
	};

	TSharedRef<SWidget> ShapeGroup = MakeGroup(LOCTEXT("ShapeGroup", "Shape"),
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[ Row(LOCTEXT("Shape", "Shape:"),
			SNew(SBox).MinDesiredWidth(140)
			[
				SNew(SComboBox<TSharedPtr<FString>>)
				.OptionsSource(&ShapeOptions)
				.InitiallySelectedItem(ShapeOptions[FMath::Clamp(static_cast<int32>(Model->shape), 0, 2)])
				.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
				.OnSelectionChanged_Lambda([this, Model](TSharedPtr<FString> S, ESelectInfo::Type)
				{
					if (!S.IsValid()) return;
					const int32 Index = ShapeOptions.IndexOfByKey(S);
					if (Index != INDEX_NONE && Index != static_cast<int32>(Model->shape))
					{
						Model->SetShape(static_cast<PrimitiveShape>(Index));
						Changed();
					}
				})
				[ SNew(STextBlock).Text_Lambda([this, Model]() { return FText::FromString(*ShapeOptions[FMath::Clamp(static_cast<int32>(Model->shape), 0, 2)]); }) ]
			],
			EVisibility::Visible, LOCTEXT("ShapeTip", "Which canonical solid this primitive is.")) ]

		// ---- Sphere rows ----
		+ SVerticalBox::Slot().AutoHeight()
		[ NumRow(LOCTEXT("Radius", "Radius:"), 0, [Model]() { return Model->radius; }, [Model](double V) { Model->radius = V; }, LOCTEXT("SphereRadiusTip", "Radius of the sphere about its centre.")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumRow(LOCTEXT("StretchX", "Stretch X:"), 0, [Model]() { return Model->stretch[0]; }, [Model](double V) { Model->stretch[0] = V; }, LOCTEXT("StretchXTip", "Stretch of the sphere along X: its semi-axis that way is the radius times this. 1 on all three is a true sphere.")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumRow(LOCTEXT("StretchY", "Stretch Y:"), 0, [Model]() { return Model->stretch[1]; }, [Model](double V) { Model->stretch[1] = V; }, LOCTEXT("StretchYTip", "Stretch of the sphere along Y: its semi-axis that way is the radius times this. 1 on all three is a true sphere.")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumRow(LOCTEXT("StretchZ", "Stretch Z:"), 0, [Model]() { return Model->stretch[2]; }, [Model](double V) { Model->stretch[2] = V; }, LOCTEXT("StretchZTip", "Stretch of the sphere along Z: its semi-axis that way is the radius times this. 1 on all three is a true sphere.")) ]

		// ---- Box rows ----
		+ SVerticalBox::Slot().AutoHeight()
		[ NumRow(LOCTEXT("LengthX", "Length (X):"), 1, [Model]() { return Model->size[0]; }, [Model](double V) { Model->size[0] = V; }, LOCTEXT("BoxLengthTip", "Size of the box along X, the fuselage axis.")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumRow(LOCTEXT("WidthY", "Width (Y):"), 1, [Model]() { return Model->size[1]; }, [Model](double V) { Model->size[1] = V; }, LOCTEXT("BoxWidthTip", "Size of the box along Y, lateral.")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumRow(LOCTEXT("HeightZ", "Height (Z):"), 1, [Model]() { return Model->size[2]; }, [Model](double V) { Model->size[2] = V; }, LOCTEXT("BoxHeightTip", "Size of the box along Z, vertical.")) ]

		// ---- Cylinder rows ----
		+ SVerticalBox::Slot().AutoHeight()
		[ NumRow(LOCTEXT("Radius", "Radius:"), 2, [Model]() { return Model->cyl_radius; }, [Model](double V) { Model->cyl_radius = V; }, LOCTEXT("CylRadiusTip", "Radius of the cylinder about its axis.")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ NumRow(LOCTEXT("LengthX", "Length (X):"), 2, [Model]() { return Model->cyl_length; }, [Model](double V) { Model->cyl_length = V; }, LOCTEXT("CylLengthTip", "Length of the cylinder along its axis, which lies along X.")) ]

		// ---- Read-outs ----
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0)
		[ MakeMetricRow(LOCTEXT("Volume", "Volume:"), TAttribute<FText>::CreateLambda([this]() { return VolumeLabel.IsValid() ? VolumeLabel->GetText() : DashText(); })) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ MakeMetricRow(LOCTEXT("SurfaceArea", "Surface area:"), TAttribute<FText>::CreateLambda([this]() { return AreaLabel.IsValid() ? AreaLabel->GetText() : DashText(); })) ]
		+ SVerticalBox::Slot().AutoHeight()
		[ MakeMetricRow(LOCTEXT("Extents", "Extents (X × Y × Z):"), TAttribute<FText>::CreateLambda([this]() { return ExtentsLabel.IsValid() ? ExtentsLabel->GetText() : DashText(); })) ]);

	VolumeLabel = SNew(STextBlock);
	AreaLabel = SNew(STextBlock);
	ExtentsLabel = SNew(STextBlock);

	TSharedRef<SWidget> ShapeTab = SNew(SScrollBox)
		+ SScrollBox::Slot().Padding(8)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight() [ ShapeGroup ]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0)
			[ MakeInfoLabel(LOCTEXT("PlacementNote", "The shape is built centred on the origin. To place it, right-click its row in the Scene tree and choose Transform — the translation there is the position of the centre.")) ]
		];

	TArray<FAerotelicTab> Tabs;
	Tabs.Add({ LOCTEXT("ShapeTab", "Shape"), ShapeTab });
	Tabs.Add({ LOCTEXT("HelpTab", "Help"), MakeHelpTab(kHelp) });

	ChildSlot
	[
		SNew(SAerotelicTabStrip, Tabs)
	];

	RefreshReadouts();
}

EVisibility SPrimitiveDialog::ShapeRowVisibility(int32 ShapeIndex) const
{
	PrimitiveModel* Model = Entity->GetModelAs<PrimitiveModel>();
	return static_cast<int32>(Model->shape) == ShapeIndex ? EVisibility::Visible : EVisibility::Collapsed;
}

void SPrimitiveDialog::RefreshReadouts()
{
	PrimitiveModel* Model = Entity->GetModelAs<PrimitiveModel>();
	try
	{
		const std::array<double, 3> Extents = Model->Extents();
		VolumeLabel->SetText(FormatG(Model->Volume(), 4));
		AreaLabel->SetText(FormatG(Model->SurfaceArea(), 4));
		ExtentsLabel->SetText(FText::FromString(FString::Printf(TEXT("%.4g × %.4g × %.4g"), Extents[0], Extents[1], Extents[2])));
	}
	catch (const std::exception&)
	{
		VolumeLabel->SetText(DashText());
		AreaLabel->SetText(DashText());
		ExtentsLabel->SetText(DashText());
	}
}

void SPrimitiveDialog::Changed()
{
	RefreshReadouts();
	Entity->RefreshProxy();
	OnChangedDelegate.ExecuteIfBound();
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

#undef LOCTEXT_NAMESPACE
