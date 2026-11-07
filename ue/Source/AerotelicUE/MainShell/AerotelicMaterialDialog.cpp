#include "AerotelicMaterialDialog.h"

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
#include "Entity/AerotelicMaterialLibrary.h"

#include "aerotelic/mass_properties.hpp"
#include "aerotelic/materials.hpp"
#include "aerotelic/units.hpp"

#include <cmath>

#define LOCTEXT_NAMESPACE "AerotelicMaterialDialog"

using aerotelic::MassMode;
using aerotelic::MassSpec;
using aerotelic::Material;

namespace
{
	TSharedRef<SWidget> FormRow(const FText& Label, TSharedRef<SWidget> Control)
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 2, 8, 2)
			[ SNew(SBox).WidthOverride(170) [ SNew(STextBlock).Text(Label) ] ]
			+ SHorizontalBox::Slot().FillWidth(1).Padding(1) [ Control ];
	}

	FText WithUnit(const FText& Label, const std::string& Unit)
	{
		return FText::Format(LOCTEXT("Labelled", "{0} ({1}):"), Label, FText::FromString(UTF8_TO_TCHAR(Unit.c_str())));
	}

	// One entry of the list: nothing, an asserted mass, a library material,
	// or the figures this entity already carries that match no library
	// material (an edited copy, a material since changed or removed in the
	// library, a file from another machine).
	struct FChoice
	{
		enum EKind { Unset, Asserted, Library, Own } Kind = Unset;
		Material M;   // Library / Own
		FString Label;
	};
}

void OpenAerotelicMaterialDialog(AAerotelicEntityActor* Entity)
{
	if (Entity == nullptr)
	{
		return;
	}
	const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	const aerotelic::UnitSystem Units = Doc != nullptr ? Doc->GetUnits() : aerotelic::UnitSystem{};

	// The geometry's own figures, once, so the dialog can say what a
	// material would weigh before the user commits to one -- and show the
	// zero that a surface with no thickness encloses.
	double Volume = 0.0, Area = 0.0;
	bool bClosed = true;
	{
		// The body's own grid (its Mesh tab's, the viewport's): the same
		// triangles the analyses weigh and solve on.
		const std::vector<aerotelic::NamedMesh> Meshes = Entity->GetPlacedMeshes(false);
		for (const aerotelic::NamedMesh& M : Meshes)
		{
			Volume += std::abs(M.mesh.SignedVolume());
			Area += M.mesh.SurfaceArea();
			bClosed = bClosed && M.mesh.IsClosedManifold();
		}
	}

	MassSpec Spec = Entity->GetMassSpec();
	bool bAccepted = false;

	// Modal, so these locals outlive every widget that points at them.
	TArray<TSharedPtr<FChoice>> Choices;
	{
		TSharedPtr<FChoice> C = MakeShared<FChoice>();
		C->Kind = FChoice::Unset;
		C->Label = LOCTEXT("ChoiceUnset", "Not set (left out of every analysis)").ToString();
		Choices.Add(C);
		C = MakeShared<FChoice>();
		C->Kind = FChoice::Asserted;
		C->Label = LOCTEXT("ChoiceAsserted", "An asserted mass (no material)").ToString();
		Choices.Add(C);
		for (const Material& M : AerotelicMaterialLibrary::All())
		{
			C = MakeShared<FChoice>();
			C->Kind = FChoice::Library;
			C->M = M;
			C->Label = UTF8_TO_TCHAR(M.name.c_str());
			Choices.Add(C);
		}
		if (Spec.mode == MassMode::Material && AerotelicMaterialLibrary::FindMatching(Spec.material) == nullptr)
		{
			C = MakeShared<FChoice>();
			C->Kind = FChoice::Own;
			C->M = Spec.material;
			C->Label = FText::Format(LOCTEXT("ChoiceOwn", "{0} -- this entity's own figures (not in the library)"), FText::FromString(UTF8_TO_TCHAR(Spec.material.name.c_str()))).ToString();
			Choices.Add(C);
		}
	}
	auto CurrentChoice = [&Spec, &Choices]() -> TSharedPtr<FChoice>
	{
		switch (Spec.mode)
		{
		case MassMode::Unset: return Choices[0];
		case MassMode::Asserted: return Choices[1];
		case MassMode::Material:
			for (const TSharedPtr<FChoice>& C : Choices)
			{
				if ((C->Kind == FChoice::Library || C->Kind == FChoice::Own) && aerotelic::SameMaterial(C->M, Spec.material)) return C;
			}
			break;
		}
		return Choices[0];
	};
	auto SelectedMaterial = [&Spec]() -> const Material* { return Spec.mode == MassMode::Material ? &Spec.material : nullptr; };

	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(FText::Format(LOCTEXT("Title", "Material Properties -- {0}"), FText::FromString(Entity->EntityName)))
		.ClientSize(FVector2D(560, 600))
		.SupportsMaximize(false)
		.SupportsMinimize(false);

	auto Button = [](const FText& Label, TFunction<void()> Fn)
	{
		return SNew(SButton).Text(Label).HAlign(HAlign_Center).ContentPadding(FMargin(14, 3))
			.OnClicked_Lambda([Fn]() { Fn(); return FReply::Handled(); });
	};
	auto VisibleWhen = [&Spec](MassMode Mode)
	{
		return TAttribute<EVisibility>::CreateLambda([&Spec, Mode]() { return Spec.mode == Mode ? EVisibility::Visible : EVisibility::Collapsed; });
	};
	// A figure of the chosen material in the project's units; a dash when
	// there is no material.
	auto MaterialFigure = [SelectedMaterial](double Material::* Field, double ToSI, int32 Significant)
	{
		return TAttribute<FText>::CreateLambda([SelectedMaterial, Field, ToSI, Significant]()
		{
			const Material* M = SelectedMaterial();
			return M != nullptr ? FormatG((M->*Field) / ToSI, Significant) : DashText();
		});
	};
	auto MaterialText = [SelectedMaterial](std::string Material::* Field)
	{
		return TAttribute<FText>::CreateLambda([SelectedMaterial, Field]()
		{
			const Material* M = SelectedMaterial();
			return M != nullptr && !(M->*Field).empty() ? FText::FromString(UTF8_TO_TCHAR((M->*Field).c_str())) : DashText();
		});
	};

	Window->SetContent(
		SNew(SBorder).Padding(12)
		[
			SNew(SVerticalBox)
			// ------------------------------------------------------ the pick
			+ SVerticalBox::Slot().AutoHeight()
			[
				FormRow(LOCTEXT("MadeOf", "This entity is made of:"),
					SNew(SComboBox<TSharedPtr<FChoice>>)
					.OptionsSource(&Choices)
					.InitiallySelectedItem(CurrentChoice())
					.OnGenerateWidget_Lambda([](TSharedPtr<FChoice> C) { return SNew(STextBlock).Text(FText::FromString(C->Label)); })
					.OnSelectionChanged_Lambda([&Spec](TSharedPtr<FChoice> C, ESelectInfo::Type)
					{
						if (!C.IsValid()) return;
						switch (C->Kind)
						{
						case FChoice::Unset: Spec.mode = MassMode::Unset; break;
						case FChoice::Asserted: Spec.mode = MassMode::Asserted; break;
						case FChoice::Library:
						case FChoice::Own:
							Spec.mode = MassMode::Material;
							Spec.material = C->M;   // a copy: the name and the figures, as the project file keeps them
							break;
						}
					})
					[ SNew(STextBlock).Text_Lambda([CurrentChoice]() { return FText::FromString(CurrentChoice()->Label); }) ])
			]
			// A material: its skin thickness. A geometric figure, in the
			// project's length unit as typed (never converted), like the
			// asserted mass; it is not part of the material.
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
			[
				SNew(SBox).Visibility(VisibleWhen(MassMode::Material))
				[
					FormRow(WithUnit(LOCTEXT("SkinThickness", "Skin thickness, 0 = solid"), Units.LengthName()),
						SNew(SAerotelicNumericEntry)
						.Value_Lambda([&Spec]() { return Spec.skin_thickness; })
						.MinValue(0.0).MaxValue(1.0e9).Delta(0.001)
						.LiveUpdate(true)
						.OnValueCommitted_Lambda([&Spec](double V, ETextCommit::Type) { Spec.skin_thickness = FMath::Max(V, 0.0); }))
				]
			]
			// An asserted mass: the total, in the project's mass unit.
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
			[
				SNew(SBox).Visibility(VisibleWhen(MassMode::Asserted))
				[
					FormRow(WithUnit(LOCTEXT("Mass", "Mass"), Units.MassName()),
						SNew(SAerotelicNumericEntry)
						.Value_Lambda([&Spec]() { return Spec.mass; })
						.MinValue(0.0).MaxValue(1.0e12).Delta(1.0)
						.LiveUpdate(true)
						.OnValueCommitted_Lambda([&Spec](double V, ETextCommit::Type) { Spec.mass = V; }))
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
			[
				MakeInfoLabel(LOCTEXT("Hint",
					"Pick what this parameterisation is made of. The materials are defined under Model > Material Properties (the presets and your own); this entity takes a copy of the one picked. "
					"With the skin thickness at zero the body is a uniform solid of the material; a positive thickness makes every body of this entity a skin that thick (also what a lattice hollows it by and what Analysis > Structural gives its shells). "
					"An asserted mass is spread over the surface when weighed and has no stiffness. Boolean results and lattices built on this entity inherit the setting."))
			]

			// ------------------------------------------------------ results
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 0)
			[
				MakeGroup(LOCTEXT("MaterialGroup", "The material"),
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight()
					[ MakeMetricRow(LOCTEXT("Category", "Category:"), MaterialText(&Material::category)) ]
					+ SVerticalBox::Slot().AutoHeight()
					[ MakeMetricRow(WithUnit(LOCTEXT("Density", "Density"), Units.DensityName()), MaterialFigure(&Material::density, Units.DensityToSI(), 4)) ]
					+ SVerticalBox::Slot().AutoHeight()
					[ MakeMetricRow(WithUnit(LOCTEXT("Young", "Young's modulus"), Units.StressName()), MaterialFigure(&Material::youngs_modulus, Units.StressToSI(), 4)) ]
					+ SVerticalBox::Slot().AutoHeight()
					[ MakeMetricRow(LOCTEXT("Poisson", "Poisson's ratio:"), MaterialFigure(&Material::poisson_ratio, 1.0, 3)) ]
					+ SVerticalBox::Slot().AutoHeight()
					[ MakeMetricRow(WithUnit(LOCTEXT("Allowable", "Allowable stress"), Units.StressName()), MaterialFigure(&Material::allowable_stress, Units.StressToSI(), 4)) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
					[
						SNew(STextBlock).AutoWrapText(true)
						.Font(FCoreStyle::GetDefaultFontStyle("Regular", 8))
						.ColorAndOpacity(FSlateColor(FLinearColor(0.6f, 0.6f, 0.6f)))
						.Text_Lambda([SelectedMaterial]()
						{
							const Material* M = SelectedMaterial();
							return M != nullptr && !M->source.empty()
								? FText::Format(LOCTEXT("SourceLine", "Source: {0}"), FText::FromString(UTF8_TO_TCHAR(M->source.c_str())))
								: FText::GetEmpty();
						})
					])
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
			[
				MakeGroup(LOCTEXT("Geometry", "This entity's geometry"),
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight()
					[ MakeMetricRow(WithUnit(LOCTEXT("Volume", "Enclosed volume"), Units.VolumeName()), FormatG(Volume, 5)) ]
					+ SVerticalBox::Slot().AutoHeight()
					[ MakeMetricRow(WithUnit(LOCTEXT("Area", "Surface area"), Units.AreaName()), FormatG(Area, 5)) ]
					+ SVerticalBox::Slot().AutoHeight()
					[
						MakeMetricRow(LOCTEXT("Closed", "Surface:"), bClosed ? LOCTEXT("IsClosed", "closed") : LOCTEXT("IsOpen", "open -- the enclosed volume is not well defined"))
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
					[
						// What the material fills: the enclosed volume of a solid, the
						// surface area times the thickness of a skin.
						MakeMetricRow(WithUnit(LOCTEXT("MaterialVolume", "Material volume"), Units.VolumeName()), TAttribute<FText>::CreateLambda([&Spec, Volume, Area]()
						{
							if (Spec.mode != MassMode::Material) return DashText();
							return FormatG(Spec.IsSkin() ? Area * Spec.skin_thickness : Volume, 5);
						}))
					]
					+ SVerticalBox::Slot().AutoHeight()
					[
						MakeMetricRow(WithUnit(LOCTEXT("Weighs", "Would weigh"), Units.MassName()), TAttribute<FText>::CreateLambda([&Spec, Volume, Area, Units]()
						{
							switch (Spec.mode)
							{
							case MassMode::Asserted: return FormatG(Spec.mass, 6);
							case MassMode::Material: return FormatG(Spec.material.density / Units.DensityToSI() * (Spec.IsSkin() ? Area * Spec.skin_thickness : Volume), 6);
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
