#include "AerotelicMassSpecDialog.h"

#include "Widgets/SWindow.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SComboBox.h"
#include "Framework/Application/SlateApplication.h"

#include "Widgets/CurveTabControls.h"
#include "Widgets/NumericEntry.h"
#include "Entity/AerotelicEntityActor.h"
#include "Entity/AerotelicBooleanModel.h"
#include "Entity/AerotelicDocumentSubsystem.h"

#include "aerotelic/mass_properties.hpp"
#include "aerotelic/units.hpp"

#include <cmath>

#define LOCTEXT_NAMESPACE "AerotelicMassSpecDialog"

using aerotelic::MassMode;
using aerotelic::MassSpec;

namespace
{
	const MassMode kModes[3] = { MassMode::Unset, MassMode::Asserted, MassMode::Density };

	FText ModeLabel(MassMode Mode)
	{
		switch (Mode)
		{
		case MassMode::Unset: return LOCTEXT("ModeUnset", "Not set (left out of the analysis)");
		case MassMode::Asserted: return LOCTEXT("ModeAsserted", "Assert the mass");
		case MassMode::Density: return LOCTEXT("ModeDensity", "Compute from a density");
		}
		return FText::GetEmpty();
	}

	TSharedRef<SWidget> FormRow(const FText& Label, TSharedRef<SWidget> Control)
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 2, 8, 2)
			[ SNew(SBox).WidthOverride(150) [ SNew(STextBlock).Text(Label) ] ]
			+ SHorizontalBox::Slot().FillWidth(1).Padding(1) [ Control ];
	}
}

void OpenAerotelicMassSpecDialog(AAerotelicEntityActor* Entity)
{
	if (Entity == nullptr)
	{
		return;
	}

	// The geometry's own figures, once, so the dialog can say what a
	// density would weigh before the user commits to one -- and show the
	// zero that a surface with no thickness encloses. The display
	// tolerance is plenty for a read-out and reuses the viewport's meshes.
	double Volume = 0.0, Area = 0.0;
	bool bClosed = true;
	{
		const std::vector<aerotelic::NamedMesh> Meshes = Entity->GetPlacedMeshes(false, FAerotelicBooleanModel::kValidationTolerance);
		for (const aerotelic::NamedMesh& M : Meshes)
		{
			Volume += std::abs(M.mesh.SignedVolume());
			Area += M.mesh.SurfaceArea();
			bClosed = bClosed && M.mesh.IsClosedManifold();
		}
	}

	MassSpec Spec = Entity->GetMassSpec();
	bool bAccepted = false;
	const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	const aerotelic::UnitSystem Units = Doc != nullptr ? Doc->GetUnits() : aerotelic::UnitSystem{};
	auto Unit = [](const std::string& Name) { return FString(UTF8_TO_TCHAR(Name.c_str())); };

	// Modal, so these locals outlive every widget that points at them.
	TArray<TSharedPtr<FString>> ModeOptions;
	for (MassMode Mode : kModes)
	{
		ModeOptions.Add(MakeShared<FString>(ModeLabel(Mode).ToString()));
	}
	auto ModeIndex = [&Spec]() { return Spec.mode == MassMode::Asserted ? 1 : (Spec.mode == MassMode::Density ? 2 : 0); };

	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(FText::Format(LOCTEXT("Title", "Mass Properties -- {0}"), FText::FromString(Entity->EntityName)))
		.ClientSize(FVector2D(440, 400))
		.SupportsMaximize(false)
		.SupportsMinimize(false);

	auto Button = [](const FText& Label, TFunction<void()> Fn)
	{
		return SNew(SButton).Text(Label).HAlign(HAlign_Center).ContentPadding(FMargin(14, 3))
			.OnClicked_Lambda([Fn]() { Fn(); return FReply::Handled(); });
	};

	// The one value field: the mass or the density, by mode.
	auto ValueField = [&Spec](MassMode For)
	{
		return SNew(SAerotelicNumericEntry)
			.Value_Lambda([&Spec, For]() { return For == MassMode::Asserted ? Spec.mass : Spec.density; })
			.MinValue(0.0).MaxValue(1.0e12).Delta(For == MassMode::Asserted ? 1.0 : 10.0)
			.LiveUpdate(true)
			.OnValueCommitted_Lambda([&Spec, For](double V, ETextCommit::Type)
			{
				if (For == MassMode::Asserted) Spec.mass = V; else Spec.density = V;
			});
	};
	auto VisibleWhen = [&Spec](MassMode Mode)
	{
		return TAttribute<EVisibility>::CreateLambda([&Spec, Mode]() { return Spec.mode == Mode ? EVisibility::Visible : EVisibility::Collapsed; });
	};

	Window->SetContent(
		SNew(SBorder).Padding(12)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight()
			[
				FormRow(LOCTEXT("Approach", "Approach:"),
					SNew(SComboBox<TSharedPtr<FString>>)
					.OptionsSource(&ModeOptions)
					.InitiallySelectedItem(ModeOptions[ModeIndex()])
					.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
					.OnSelectionChanged_Lambda([&Spec, &ModeOptions](TSharedPtr<FString> S, ESelectInfo::Type)
					{
						const int32 Index = ModeOptions.IndexOfByKey(S);
						if (Index != INDEX_NONE) Spec.mode = kModes[Index];
					})
					[ SNew(STextBlock).Text_Lambda([&Spec]() { return ModeLabel(Spec.mode); }) ])
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
			[
				SNew(SBox).Visibility(VisibleWhen(MassMode::Asserted))
				[
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight()
					[ FormRow(FText::Format(LOCTEXT("Mass", "Mass ({0}):"), FText::FromString(Unit(Units.MassName()))), ValueField(MassMode::Asserted)) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
					[
						MakeInfoLabel(LOCTEXT("AssertedHint",
							"The total mass of this parameterisation, in the project's mass unit (Geometry > Units). When weighed it is spread uniformly over the surface, as a thin shell: the centre of mass and the inertia come from integrating the surface. A multi-body entity (a propeller) shares it between its bodies by area; a mirrored half carries the whole mass again as a part of its own."))
					]
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
			[
				SNew(SBox).Visibility(VisibleWhen(MassMode::Density))
				[
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight()
					[ FormRow(FText::Format(LOCTEXT("Density", "Density ({0}):"), FText::FromString(Unit(Units.DensityName()))), ValueField(MassMode::Density)) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
					[
						MakeInfoLabel(LOCTEXT("DensityHint",
							"In the project's units (Geometry > Units). The entity is weighed as a uniform solid: the density times the volume its closed surface encloses, with the centre and inertia integrated over that volume. A surface with no thickness encloses nothing and weighs nothing this way -- assert the mass instead."))
					]
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
			[
				SNew(SBox).Visibility(VisibleWhen(MassMode::Unset))
				[
					MakeInfoLabel(LOCTEXT("UnsetHint",
						"Nothing is assigned yet: Analysis > Mass Properties leaves this entity out of the assembly and says so. Choose an approach above."))
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 14, 0, 0)
			[
				MakeGroup(LOCTEXT("Geometry", "This entity's geometry"),
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight()
					[ MakeMetricRow(FText::Format(LOCTEXT("Volume", "Enclosed volume ({0}):"), FText::FromString(Unit(Units.VolumeName()))), FormatG(Volume, 5)) ]
					+ SVerticalBox::Slot().AutoHeight()
					[ MakeMetricRow(FText::Format(LOCTEXT("Area", "Surface area ({0}):"), FText::FromString(Unit(Units.AreaName()))), FormatG(Area, 5)) ]
					+ SVerticalBox::Slot().AutoHeight()
					[
						MakeMetricRow(LOCTEXT("Closed", "Surface:"), bClosed ? LOCTEXT("IsClosed", "closed") : LOCTEXT("IsOpen", "open -- the enclosed volume is not well defined"))
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
					[
						MakeMetricRow(FText::Format(LOCTEXT("Weighs", "Would weigh ({0}):"), FText::FromString(Unit(Units.MassName()))), TAttribute<FText>::CreateLambda([&Spec, Volume]()
						{
							switch (Spec.mode)
							{
							case MassMode::Asserted: return FormatG(Spec.mass, 6);
							case MassMode::Density: return FormatG(Spec.density * Volume, 6);
							case MassMode::Unset: break;
							}
							return DashText();
						}))
					])
			]
			+ SVerticalBox::Slot().FillHeight(1) [ SNew(SBox) ]
			+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[ Button(LOCTEXT("OK", "OK"), [&bAccepted, Window]() { bAccepted = true; Window->RequestDestroyWindow(); }) ]
				+ SHorizontalBox::Slot().AutoWidth()
				[ Button(LOCTEXT("Cancel", "Cancel"), [Window]() { Window->RequestDestroyWindow(); }) ]
			]
		]);

	FSlateApplication::Get().AddModalWindow(Window, FSlateApplication::Get().GetActiveTopLevelWindow());

	if (bAccepted)
	{
		Entity->SetMassSpec(Spec);
	}
}

#undef LOCTEXT_NAMESPACE
