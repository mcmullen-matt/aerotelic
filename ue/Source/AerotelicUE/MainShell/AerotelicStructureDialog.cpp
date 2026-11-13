#include "AerotelicStructureDialog.h"

#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Framework/Application/SlateApplication.h"
#include "Styling/CoreStyle.h"

#include "Entity/AerotelicEntityActor.h"
#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicBodyStructureText.h"
#include "Widgets/NumericEntry.h"
#include "Widgets/TabStrip.h"
#include "Widgets/CurveTabControls.h"
#include "Widgets/AerotelicDialogWindow.h"
#include "Widgets/MaterialPicker.h"
#include "Widgets/MessageBox.h"

#include "aerotelic/body_structure.hpp"
#include "aerotelic/laminate.hpp"
#include "aerotelic/mass_properties.hpp"

#define LOCTEXT_NAMESPACE "AerotelicStructureDialog"

using aerotelic::BodyStructure;
using aerotelic::Material;

namespace
{
	// One window per (entity, body): re-opening brings it to the front.
	TMap<FString, TWeakPtr<SWindow>> GOpenStructureDialogs;

	FString DialogKey(const AAerotelicEntityActor* Entity, const FString& Role)
	{
		return FString::Printf(TEXT("%p|%s"), static_cast<const void*>(Entity), *Role);
	}

	TSharedRef<SWidget> FormRow(const FText& Label, TSharedRef<SWidget> Control, float LabelWidth = 170.0f)
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 2, 8, 2)
			[ SNew(SBox).WidthOverride(LabelWidth) [ SNew(STextBlock).Text(Label) ] ]
			+ SHorizontalBox::Slot().FillWidth(1).Padding(1) [ Control ];
	}

	FText WithUnit(const FText& Label, const std::string& Unit)
	{
		return FText::Format(LOCTEXT("Labelled", "{0} ({1}):"), Label, FText::FromString(UTF8_TO_TCHAR(Unit.c_str())));
	}

	// What the dialog edits: a DRAFT of the body's structure. Every field
	// reads and writes this, never the body; Apply hands it over and Revert
	// re-reads the body's. The dialog is modeless, so this lives in a shared
	// object the widgets' lambdas own.
	struct FStructureDraft
	{
		TWeakObjectPtr<AAerotelicEntityActor> Entity;
		FString Role;
		BodyStructure S;
		bool bDirty = false;
		aerotelic::UnitSystem Units;

		// The body's own geometry, once.
		double Volume = 0.0, Area = 0.0;
		bool bClosed = true;

		// Mass: the estimate, computed on demand.
		FText Estimate;

		void Touch() { bDirty = true; }

		void ReadFromEntity()
		{
			if (const BodyStructure* Own = Entity.IsValid() ? Entity->GetBodyStructure(Role) : nullptr)
			{
				S = *Own;
			}
			else
			{
				S = BodyStructure{};
			}
			bDirty = false;
		}
	};

	FText EstimateText(const FStructureDraft& D)
	{
		if (!D.Entity.IsValid()) return LOCTEXT("Gone", "The entity is no longer in the scene.");
		const std::vector<aerotelic::NamedMesh> Meshes = D.Entity->GetPlacedMeshes(false);
		const aerotelic::TriMesh* Mesh = nullptr;
		for (const aerotelic::NamedMesh& M : Meshes)
		{
			if (FString(UTF8_TO_TCHAR(M.name.c_str())) == D.Role || (Meshes.size() == 1 && D.Role.IsEmpty())) Mesh = &M.mesh;
		}
		if (Mesh == nullptr) return LOCTEXT("NoMesh", "No geometry for this body.");
		std::array<double, 9> R;
		aerotelic::Vec3 T;
		double Scale = 1.0;
		D.Entity->GetPlacementMatrix(R, T, Scale);
		const std::vector<aerotelic::StructureMassPart> Parts = aerotelic::StructureMassParts(*Mesh, D.S, R, T, Scale, false, D.Units.DensityToSI());
		FString Out;
		double Total = 0.0;
		for (const aerotelic::StructureMassPart& P : Parts)
		{
			if (P.included)
			{
				Out += FString::Printf(TEXT("%s: %.5g %s%s\n"), UTF8_TO_TCHAR(P.label.c_str()), P.props.mass, UTF8_TO_TCHAR(D.Units.MassName()), P.estimated ? TEXT(" (estimated)") : TEXT(""));
				Total += P.props.mass;
			}
			else
			{
				Out += FString::Printf(TEXT("%s: left out -- %s\n"), UTF8_TO_TCHAR(P.label.c_str()), UTF8_TO_TCHAR(P.note.c_str()));
			}
		}
		// The generated layers, as the mass report weighs them.
		if (const FAerotelicBodyStructure* G = D.Entity->GetGeneratedStructure(D.Role); G != nullptr && G->IsGenerated() && D.S.material.HasDensity() && !D.S.asserted_mass)
		{
			for (const FAerotelicStructureLayer& L : G->GetLayers())
			{
				if (L.Kind == FAerotelicStructureLayer::EKind::Skin) continue;
				const aerotelic::Material& M = L.Material.HasDensity() ? L.Material : D.S.material;
				const double Mass = M.density / D.Units.DensityToSI() * L.Area * L.Thickness;
				Out += FString::Printf(TEXT("%s: %.5g %s\n"), *L.Name, Mass, UTF8_TO_TCHAR(D.Units.MassName()));
				Total += Mass;
			}
		}
		Out += FString::Printf(TEXT("Total: %.5g %s"), Total, UTF8_TO_TCHAR(D.Units.MassName()));
		if (D.Entity->IsMirrored()) Out += TEXT("  (the mirrored half weighs the same again)");
		return FText::FromString(Out);
	}
}

void OpenAerotelicStructureDialog(AAerotelicEntityActor* Entity, const FString& BodyRole)
{
	if (Entity == nullptr) return;
	const int32 BodyIndex = Entity->FindBodyIndex(BodyRole);
	if (BodyIndex == INDEX_NONE) return;
	const FString Role = Entity->GetBodies()[BodyIndex].Name;
	const FString Key = DialogKey(Entity, Role);
	if (TWeakPtr<SWindow>* Existing = GOpenStructureDialogs.Find(Key))
	{
		if (TSharedPtr<SWindow> Win = Existing->Pin())
		{
			Win->BringToFront();
			FSlateApplication::Get().SetKeyboardFocus(Win);
			return;
		}
	}

	TSharedRef<FStructureDraft> D = MakeShared<FStructureDraft>();
	D->Entity = Entity;
	D->Role = Role;
	if (const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get()) D->Units = Doc->GetUnits();
	{
		const std::vector<aerotelic::NamedMesh> Meshes = Entity->GetPlacedMeshes(false);
		for (const aerotelic::NamedMesh& M : Meshes)
		{
			if (FString(UTF8_TO_TCHAR(M.name.c_str())) != Role && Meshes.size() > 1) continue;
			D->Volume = std::abs(M.mesh.SignedVolume());
			D->Area = M.mesh.SurfaceArea();
			D->bClosed = M.mesh.IsClosedManifold();
		}
	}
	D->ReadFromEntity();
	const aerotelic::UnitSystem Units = D->Units;

	auto Button = [](const FText& Label, const FText& Tip, TFunction<void()> Fn)
	{
		return SNew(SButton).Text(Label).ToolTipText(Tip).HAlign(HAlign_Center).ContentPadding(FMargin(12, 3))
			.OnClicked_Lambda([Fn]() { Fn(); return FReply::Handled(); });
	};
	// A length of the structure, in the project's unit as typed.
	auto Length = [D](TFunction<double*()> Field, double Min, double Delta)
	{
		return SNew(SAerotelicNumericEntry)
			.Value_Lambda([Field]() { double* V = Field(); return V ? *V : 0.0; })
			.MinValue(Min).MaxValue(1.0e9).Delta(Delta)
			.LiveUpdate(true)
			.OnValueCommitted_Lambda([D, Field](double V, ETextCommit::Type) { if (double* F = Field()) { *F = V; D->Touch(); } });
	};

	// ------------------------------------------------------------------ Skin
	// A LAMINATE's thickness is its stack's (2026-11-12): picking one sets
	// the skin thickness to the plies' sum in the project's unit and locks
	// the field; the stack is edited under Model > Material Properties.
	auto SyncLaminateThickness = [D]()
	{
		if (D->S.material.IsLaminate())
		{
			D->S.skin_thickness = aerotelic::StackThickness(aerotelic::ExpandedStack(D->S.material)) / D->Units.LengthToSI();
		}
	};
	SyncLaminateThickness();
	TSharedRef<SWidget> SkinTab =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[
			FormRow(LOCTEXT("Material", "Made of:"),
				SNew(SAerotelicMaterialPicker)
				.Current_Lambda([D]() { return D->S.material; })
				.AllowNone(true)
				.OnPicked_Lambda([D, SyncLaminateThickness](const Material& M) { D->S.material = M; SyncLaminateThickness(); D->Touch(); }))
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
		[
			FormRow(WithUnit(LOCTEXT("SkinThickness", "Skin thickness, 0 = solid"), Units.LengthName()),
				SNew(SBox).IsEnabled_Lambda([D]() { return !D->S.material.IsLaminate(); })
				[ Length([D]() { return &D->S.skin_thickness; }, 0.0, 0.001) ])
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2, 0, 0)
		[
			SNew(STextBlock).AutoWrapText(true)
			.Visibility_Lambda([D]() { return D->S.material.IsLaminate() ? EVisibility::Visible : EVisibility::Collapsed; })
			.ColorAndOpacity(FSlateColor(FLinearColor(0.95f, 0.85f, 0.55f)))
			.Text_Lambda([D]()
			{
				const std::vector<aerotelic::Ply> Stack = aerotelic::ExpandedStack(D->S.material);
				return FText::FromString(FString::Printf(TEXT("A laminate: the thickness is the stack's, %d plies %s = %.4g %s (read-only; edit the stack under Model > Material Properties). "
					"Fibres at 0 degrees run along the body's loft (a wing's span, a fuselage's length); the analysis drapes each ply over the body and reports Tsai-Wu margins."),
					(int32)Stack.size(), UTF8_TO_TCHAR(aerotelic::StackingNotation(D->S.material).c_str()), D->S.skin_thickness, UTF8_TO_TCHAR(D->Units.LengthName())));
			})
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
		[
			MakeInfoLabel(LOCTEXT("SkinHint",
				"The material this body is made of -- its solid when the skin thickness is zero, its skin otherwise. Materials are defined under Model > Material Properties (the presets and your own); the body takes a copy of the one picked. "
				"A skin thickness is a build figure in the project's length unit, not scaled with the model: the body's surface is the skin, that thick -- what Analysis > Structural gives its shells, what the mass report weighs as rho t A, and what the Scene tree lists as the Skin layer. "
				"Behind a skin the body's PARAMETERISATION puts its structure -- a wing's spars, ribs and stringers from the wing dialog's Structure tab -- built with the body on its own grid as soon as the skin is set, and listed as layers under the body; a solid body has none."))
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 0)
		[
			MakeGroup(LOCTEXT("GeometryGroup", "This body's geometry"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(WithUnit(LOCTEXT("Volume", "Enclosed volume"), Units.VolumeName()), FormatG(D->Volume, 5)) ]
				+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(WithUnit(LOCTEXT("Area", "Surface area"), Units.AreaName()), FormatG(D->Area, 5)) ]
				+ SVerticalBox::Slot().AutoHeight() [ MakeMetricRow(LOCTEXT("Closed", "Surface:"), D->bClosed ? LOCTEXT("IsClosed", "closed") : LOCTEXT("IsOpen", "open -- the enclosed volume is not well defined")) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					MakeMetricRow(WithUnit(LOCTEXT("SkinVolume", "Skin material volume"), Units.VolumeName()), TAttribute<FText>::CreateLambda([D]()
					{
						return D->S.skin_thickness > 0.0 ? FormatG(D->Area * D->S.skin_thickness, 5) : FormatG(D->Volume, 5);
					}))
				])
		];

	// ------------------------------------------------------------------ Mass
	TSharedRef<SWidget> MassTab =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SCheckBox)
			.IsChecked_Lambda([D]() { return D->S.asserted_mass ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
			.OnCheckStateChanged_Lambda([D](ECheckBoxState State)
			{
				if (State == ECheckBoxState::Checked) D->S.asserted_mass = D->S.asserted_mass.value_or(0.0);
				else D->S.asserted_mass.reset();
				D->Touch();
			})
			[ SNew(STextBlock).Text(LOCTEXT("AssertOn", "Assert this body's total mass instead of computing it")) ]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
		[
			SNew(SBox).IsEnabled_Lambda([D]() { return D->S.asserted_mass.has_value(); })
			[
				FormRow(WithUnit(LOCTEXT("AssertedMass", "Mass"), Units.MassName()),
					SNew(SAerotelicNumericEntry)
					.Value_Lambda([D]() { return D->S.asserted_mass.value_or(0.0); })
					.MinValue(0.0).MaxValue(1.0e12).Delta(0.01)
					.LiveUpdate(true)
					.OnValueCommitted_Lambda([D](double V, ETextCommit::Type) { if (D->S.asserted_mass) { D->S.asserted_mass = V; D->Touch(); } }))
			]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
		[
			MakeInfoLabel(LOCTEXT("MassHint", "An asserted mass is spread over the body's surface when weighed and has no stiffness; it replaces every computed figure. Otherwise the body weighs by its structure: the skin or solid exactly, and each generated layer (ribs, spars, stringers) as a shell of its area, thickness and material."))
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 0)
		[
			MakeGroup(LOCTEXT("EstimateGroup", "What the draft weighs"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight()
				[ Button(LOCTEXT("Estimate", "Estimate"), LOCTEXT("EstimateTip", "Weigh the draft as the mass report would"), [D]() { D->Estimate = EstimateText(*D); }) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
				[ SNew(STextBlock).AutoWrapText(true).Text_Lambda([D]() { return D->Estimate; }) ])
		];

	// ---------------------------------------------------------------- footer
	auto IsDirty = [D]() { return D->bDirty; };
	TWeakObjectPtr<AAerotelicEntityActor> Weak = Entity;
	TSharedRef<SWidget> Footer =
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center).Padding(0, 0, 8, 0)
		[
			SNew(STextBlock).AutoWrapText(true)
			.ColorAndOpacity(FSlateColor(FLinearColor(0.6f, 0.6f, 0.6f)))
			.Text_Lambda([D, Weak]()
			{
				if (!Weak.IsValid()) return LOCTEXT("Gone", "The entity is no longer in the scene.");
				const std::string Why = aerotelic::BodyStructureProblem(D->S);
				if (!Why.empty()) return FText::FromString(FString::Printf(TEXT("Cannot apply: %s."), UTF8_TO_TCHAR(Why.c_str())));
				return D->bDirty ? LOCTEXT("Edited", "Edited here only -- press Apply to give the body this structure.") : LOCTEXT("Clean", "As the body is.");
			})
		]
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 6, 0)
		[
			SNew(SButton).Text(LOCTEXT("Revert", "Revert")).ToolTipText(LOCTEXT("RevertTip", "Put the fields back to the structure the body has."))
			.IsEnabled_Lambda([IsDirty, Weak]() { return IsDirty() && Weak.IsValid(); })
			.OnClicked_Lambda([D]() { D->ReadFromEntity(); return FReply::Handled(); })
		]
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 6, 0)
		[
			SNew(SButton).Text(LOCTEXT("Apply", "Apply")).ToolTipText(LOCTEXT("ApplyTip", "Give the body this material, skin and mass setting. A body with a skin builds its parameterisation's structure (a wing's spars and ribs) at once, listed as layers under it in the Scene tree; hide the Skin row to see them. Analysis > Mass Properties weighs by it and Analysis > Structural analyses with it."))
			.IsEnabled_Lambda([D, Weak]() { return D->bDirty && Weak.IsValid() && aerotelic::BodyStructureProblem(D->S).empty(); })
			.OnClicked_Lambda([D]()
			{
				UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
				if (!D->Entity.IsValid() || Doc == nullptr) return FReply::Handled();
				FString Error;
				const bool bOk = Doc->ApplyBodyStructure(D->Entity.Get(), D->Role, D->S, Error);
				if (D->Entity->GetBodyStructure(D->Role) != nullptr) D->bDirty = false;
				if (!bOk)
				{
					AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(Error), LOCTEXT("ApplyTitle", "Apply Structure"));
				}
				return FReply::Handled();
			})
		];

	TArray<FAerotelicTab> Tabs;
	Tabs.Add({ LOCTEXT("SkinTab", "Skin"), SkinTab });
	Tabs.Add({ LOCTEXT("MassTab", "Mass"), MassTab });

	const FVector2D Size(640, 520);
	const FString Title = Entity->GetBodies().Num() > 1
		? FString::Printf(TEXT("Structure -- %s / %s"), *Entity->EntityName, *Role)
		: FString::Printf(TEXT("Structure -- %s"), *Entity->EntityName);
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(FText::FromString(Title))
		.ClientSize(Size)
		.SupportsMaximize(false)
		[
			SNew(SBorder).Padding(12)
			[
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().FillHeight(1.0f) [ SNew(SAerotelicTabStrip, Tabs) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0) [ Footer ]
			]
		];
	Window->SetOnWindowClosed(FOnWindowClosed::CreateLambda([Key](const TSharedRef<SWindow>&)
	{
		GOpenStructureDialogs.Remove(Key);
	}));

	GOpenStructureDialogs.Add(Key, Window);
	ShowAerotelicDialogWindow(Window, Size);
}

#undef LOCTEXT_NAMESPACE
