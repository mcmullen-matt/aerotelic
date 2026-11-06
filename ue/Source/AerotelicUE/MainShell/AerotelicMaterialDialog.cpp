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
	const MassMode kModes[3] = { MassMode::Unset, MassMode::Material, MassMode::Asserted };
	const FString kCustom = TEXT("Custom");

	FText ModeLabel(MassMode Mode)
	{
		switch (Mode)
		{
		case MassMode::Unset: return LOCTEXT("ModeUnset", "Not set (left out of every analysis)");
		case MassMode::Material: return LOCTEXT("ModeMaterial", "A material (density, stiffness, strength)");
		case MassMode::Asserted: return LOCTEXT("ModeAsserted", "An asserted mass");
		}
		return FText::GetEmpty();
	}

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
	TArray<TSharedPtr<FString>> ModeOptions;
	for (MassMode Mode : kModes) ModeOptions.Add(MakeShared<FString>(ModeLabel(Mode).ToString()));
	auto ModeIndex = [&Spec]() { return Spec.mode == MassMode::Material ? 1 : (Spec.mode == MassMode::Asserted ? 2 : 0); };

	// The presets, then Custom. What the combo shows is derived from the
	// numbers each time: an edited preset reads as Custom.
	TArray<TSharedPtr<FString>> MaterialOptions;
	for (const Material& M : aerotelic::MaterialPresets()) MaterialOptions.Add(MakeShared<FString>(UTF8_TO_TCHAR(M.name.c_str())));
	MaterialOptions.Add(MakeShared<FString>(kCustom));
	auto MaterialName = [&Spec]() -> FString
	{
		return aerotelic::IsMaterialPreset(Spec.material) ? FString(UTF8_TO_TCHAR(Spec.material.name.c_str())) : kCustom;
	};
	auto MaterialOption = [&MaterialOptions, MaterialName]() -> TSharedPtr<FString>
	{
		const FString Name = MaterialName();
		for (const TSharedPtr<FString>& O : MaterialOptions) if (*O == Name) return O;
		return MaterialOptions.Last();
	};

	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(FText::Format(LOCTEXT("Title", "Material Properties -- {0}"), FText::FromString(Entity->EntityName)))
		.ClientSize(FVector2D(520, 640))
		.SupportsMaximize(false)
		.SupportsMinimize(false);

	auto Button = [](const FText& Label, TFunction<void()> Fn)
	{
		return SNew(SButton).Text(Label).HAlign(HAlign_Center).ContentPadding(FMargin(14, 3))
			.OnClicked_Lambda([Fn]() { Fn(); return FReply::Handled(); });
	};

	// A material figure: SI underneath, the project's unit on screen. An
	// edit makes the material Custom (by the numbers, so a value typed back
	// to the preset's is the preset again).
	auto MaterialField = [&Spec](double Material::* Field, double ToSI, double Delta, double Max)
	{
		return SNew(SAerotelicNumericEntry)
			.Value_Lambda([&Spec, Field, ToSI]() { return (Spec.material.*Field) / ToSI; })
			.MinValue(0.0).MaxValue(Max).Delta(Delta)
			.LiveUpdate(true)
			.OnValueCommitted_Lambda([&Spec, Field, ToSI](double V, ETextCommit::Type)
			{
				Spec.material.*Field = V * ToSI;
				if (!aerotelic::IsMaterialPreset(Spec.material)) Spec.material.name = TCHAR_TO_UTF8(*kCustom);
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
				FormRow(LOCTEXT("Approach", "This entity is:"),
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

			// ------------------------------------------------------ material
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0)
			[
				SNew(SBox).Visibility(VisibleWhen(MassMode::Material))
				[
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight()
					[
						FormRow(LOCTEXT("Material", "Material:"),
							SNew(SComboBox<TSharedPtr<FString>>)
							.OptionsSource(&MaterialOptions)
							.InitiallySelectedItem(MaterialOption())
							.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
							.OnSelectionChanged_Lambda([&Spec](TSharedPtr<FString> S, ESelectInfo::Type)
							{
								if (!S.IsValid()) return;
								if (const Material* P = aerotelic::FindMaterialPreset(TCHAR_TO_UTF8(**S)))
								{
									Spec.material = *P;   // the preset's numbers come in with its name
								}
								else
								{
									Spec.material.name = TCHAR_TO_UTF8(*kCustom);   // Custom: the numbers stay for editing
								}
							})
							[ SNew(STextBlock).Text_Lambda([MaterialName]() { return FText::FromString(MaterialName()); }) ])
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
					[ FormRow(WithUnit(LOCTEXT("Density", "Density"), Units.DensityName()), MaterialField(&Material::density, Units.DensityToSI(), 10.0, 1.0e9)) ]
					+ SVerticalBox::Slot().AutoHeight()
					[ FormRow(WithUnit(LOCTEXT("Young", "Young's modulus"), Units.StressName()), MaterialField(&Material::youngs_modulus, Units.StressToSI(), 1.0e9 / Units.StressToSI(), 1.0e15)) ]
					+ SVerticalBox::Slot().AutoHeight()
					[ FormRow(LOCTEXT("Poisson", "Poisson's ratio:"), MaterialField(&Material::poisson_ratio, 1.0, 0.01, 0.4999)) ]
					+ SVerticalBox::Slot().AutoHeight()
					[ FormRow(WithUnit(LOCTEXT("Allowable", "Allowable stress"), Units.StressName()), MaterialField(&Material::allowable_stress, Units.StressToSI(), 1.0e6 / Units.StressToSI(), 1.0e15)) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
					[
						// The skin thickness is a geometric figure, in the project's
						// length unit as typed (never converted), like the asserted
						// mass; it is not part of the material.
						FormRow(WithUnit(LOCTEXT("SkinThickness", "Skin thickness, 0 = solid"), Units.LengthName()),
							SNew(SAerotelicNumericEntry)
							.Value_Lambda([&Spec]() { return Spec.skin_thickness; })
							.MinValue(0.0).MaxValue(1.0e9).Delta(0.001)
							.LiveUpdate(true)
							.OnValueCommitted_Lambda([&Spec](double V, ETextCommit::Type) { Spec.skin_thickness = FMath::Max(V, 0.0); }))
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
					[
						MakeInfoLabel(LOCTEXT("MaterialHint",
							"A material is a property of the stuff, not of the project's units: the figures are kept in SI and shown here in the project's units (Geometry > Units). Pick a preset to take its handbook values; edit any figure and it becomes Custom. "
							"With the skin thickness at zero the density weighs the entity as a uniform solid over the volume its closed surface encloses -- a surface with no thickness weighs nothing this way. "
							"A positive skin thickness makes every solid body this parameterisation produces a skin of the material that thick: its material volume is the surface area times the thickness, its mass the density times that, spread over the surface as a thin shell (open surfaces included; a mirrored half carries its own). A lattice that fills this entity hollows it by this thickness (the lattice dialog's figure is only the default for bodies without one) and is regenerated over the new thickness when OK changes it, as it is after an Apply; Analysis > Structural gives its shells this thickness. "
							"Young's modulus, Poisson's ratio and the allowable are what Analysis > Structural uses; the allowable is what a margin is reported against (zero: no margin)."))
					]
				]
			]

			// ------------------------------------------------------ asserted
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0)
			[
				SNew(SBox).Visibility(VisibleWhen(MassMode::Asserted))
				[
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight()
					[
						FormRow(WithUnit(LOCTEXT("Mass", "Mass"), Units.MassName()),
							SNew(SAerotelicNumericEntry)
							.Value_Lambda([&Spec]() { return Spec.mass; })
							.MinValue(0.0).MaxValue(1.0e12).Delta(1.0)
							.LiveUpdate(true)
							.OnValueCommitted_Lambda([&Spec](double V, ETextCommit::Type) { Spec.mass = V; }))
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
					[
						MakeInfoLabel(LOCTEXT("AssertedHint",
							"The total mass of this parameterisation, in the project's mass unit. When weighed it is spread uniformly over the surface, as a thin shell: the centre of mass and the inertia come from integrating the surface. A multi-body entity (a propeller) shares it between its bodies by area; a mirrored half carries the whole mass again as a part of its own. An asserted mass has no stiffness, so Analysis > Structural leaves it out."))
					]
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0)
			[
				SNew(SBox).Visibility(VisibleWhen(MassMode::Unset))
				[
					MakeInfoLabel(LOCTEXT("UnsetHint",
						"Nothing is assigned yet: Analysis > Mass Properties and Analysis > Structural leave this entity out and say so. Choose above."))
				]
			]

			// ------------------------------------------------------ readouts
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 14, 0, 0)
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
