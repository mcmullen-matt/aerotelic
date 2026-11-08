#include "AerotelicStructureDialog.h"

#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Views/SListView.h"
#include "Widgets/Views/STableRow.h"
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
#include "Widgets/LatticeSpecControls.h"

#include "aerotelic/body_structure.hpp"
#include "aerotelic/mass_properties.hpp"

#define LOCTEXT_NAMESPACE "AerotelicStructureDialog"

using aerotelic::BodyStructure;
using aerotelic::Material;
using aerotelic::PlaneNetwork;

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

		// Infill: the lattice controls' draft, mirrored into S.infill.
		TSharedRef<FAerotelicLatticeSpecDraft> Lattice = MakeShared<FAerotelicLatticeSpecDraft>();

		// Planes: the networks list and the one being edited.
		TArray<TSharedPtr<int32>> NetworkItems;
		TSharedPtr<SListView<TSharedPtr<int32>>> NetworkList;
		int32 Selected = INDEX_NONE;
		TArray<TSharedPtr<FString>> AxisOptions = { MakeShared<FString>(TEXT("X (fore/aft)")), MakeShared<FString>(TEXT("Y (left/right)")), MakeShared<FString>(TEXT("Z (up/down)")), MakeShared<FString>(TEXT("Custom")) };
		TSharedPtr<FString> AxisChoice;
		TSharedPtr<SComboBox<TSharedPtr<FString>>> AxisCombo;
		TSharedPtr<SEditableTextBox> NameBox;

		// Mass: the estimate, computed on demand.
		FText Estimate;

		PlaneNetwork* Net() { return S.networks.size() > static_cast<size_t>(Selected) && Selected >= 0 ? &S.networks[Selected] : nullptr; }

		void RebuildNetworkItems()
		{
			NetworkItems.Reset();
			for (size_t i = 0; i < S.networks.size(); ++i) NetworkItems.Add(MakeShared<int32>(static_cast<int32>(i)));
			if (Selected >= static_cast<int32>(S.networks.size())) Selected = static_cast<int32>(S.networks.size()) - 1;
			if (NetworkList.IsValid())
			{
				NetworkList->RequestListRefresh();
				for (const TSharedPtr<int32>& Item : NetworkItems)
				{
					if (*Item == Selected) NetworkList->SetSelection(Item, ESelectInfo::Direct);
				}
			}
			SyncNetworkWidgets();
		}

		void SyncNetworkWidgets()
		{
			const PlaneNetwork* N = Net();
			AxisChoice = AxisOptions[N ? FMath::Clamp(static_cast<int32>(N->axis), 0, 3) : 1];
			if (AxisCombo.IsValid()) AxisCombo->SetSelectedItem(AxisChoice);
			if (NameBox.IsValid()) NameBox->SetText(FText::FromString(N ? UTF8_TO_TCHAR(N->name.c_str()) : TEXT("")));
		}

		// The 3D view's translucent planes follow the draft's networks.
		void PushPreview()
		{
			UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
			if (Doc == nullptr || !Entity.IsValid()) return;
			FAerotelicStructurePreview P;
			P.Entity = Entity;
			P.BodyRole = Role;
			P.Networks = S.networks;
			P.ActiveIndex = Selected;
			Doc->SetStructurePreview(P);
		}

		void Touch()
		{
			bDirty = true;
			PushPreview();
		}

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
			if (S.infill) Lattice->Spec = S.infill->lattice;
			Lattice->SyncFromSpec();
			bDirty = false;
			RebuildNetworkItems();
			PushPreview();
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
	TSharedRef<SWidget> SkinTab =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[
			FormRow(LOCTEXT("Material", "Made of:"),
				SNew(SAerotelicMaterialPicker)
				.Current_Lambda([D]() { return D->S.material; })
				.AllowNone(true)
				.OnPicked_Lambda([D](const Material& M) { D->S.material = M; D->Touch(); }))
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
		[
			FormRow(WithUnit(LOCTEXT("SkinThickness", "Skin thickness, 0 = solid"), Units.LengthName()),
				Length([D]() { return &D->S.skin_thickness; }, 0.0, 0.001))
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
		[
			MakeInfoLabel(LOCTEXT("SkinHint",
				"The material this body is made of -- its solid when the skin thickness is zero, its skin otherwise. Materials are defined under Model > Material Properties (the presets and your own); the body takes a copy of the one picked. "
				"A skin thickness is a build figure in the project's length unit, not scaled with the model: it is what the body is hollowed by, what Analysis > Structural gives its shells, and what the mass report weighs as rho t A. Behind a skin the Infill and Planes tabs fill the interior; a solid body has none."))
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

	// ---------------------------------------------------------------- Infill
	TSharedRef<SWidget> LatticeControls = MakeAerotelicLatticeSpecControls(D->Lattice, FSimpleDelegate::CreateLambda([D]()
	{
		if (D->S.infill) { D->S.infill->lattice = D->Lattice->Spec; D->Touch(); }
	}));
	TSharedRef<SWidget> InfillTab =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SCheckBox)
			.IsChecked_Lambda([D]() { return D->S.infill ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
			.OnCheckStateChanged_Lambda([D](ECheckBoxState State)
			{
				if (State == ECheckBoxState::Checked)
				{
					aerotelic::InfillSpec I;
					I.lattice = D->Lattice->Spec;
					if (D->S.infill) I = *D->S.infill;
					D->S.infill = I;
				}
				else
				{
					D->S.infill.reset();
				}
				D->Touch();
			})
			[ SNew(STextBlock).Text(LOCTEXT("InfillOn", "Fill the interior with a lattice")) ]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
		[
			SNew(SBox).IsEnabled_Lambda([D]() { return D->S.infill.has_value(); })
			[
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight()
				[
					FormRow(LOCTEXT("InfillMaterial", "Infill material:"),
						SNew(SAerotelicMaterialPicker)
						.Current_Lambda([D]() { return D->S.infill ? D->S.infill->material : Material{}; })
						.AllowNone(false)
						.OnPicked_Lambda([D](const Material& M) { if (D->S.infill) { D->S.infill->material = M; D->Touch(); } }))
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0) [ LatticeControls ]
			]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
		[
			MakeInfoLabel(LOCTEXT("InfillHint", "The lattice fills what the skin encloses, less the ribs of the Planes tab and the reserved volumes, in a material of its own (a plastic infill under a balsa skin). It needs a skin thickness. Its struts weigh by the volume fraction; Analysis > Structural homogenises its unit cell into the core's material card."))
		];

	// ---------------------------------------------------------------- Planes
	auto NetField = [D, Length](double PlaneNetwork::* Member, double Min, double Delta)
	{
		return Length([D, Member]() -> double* { PlaneNetwork* N = D->Net(); return N ? &(N->*Member) : nullptr; }, Min, Delta);
	};
	auto NormalField = [D](int32 Component)
	{
		return SNew(SAerotelicNumericEntry)
			.Value_Lambda([D, Component]() { const PlaneNetwork* N = D->Net(); return N ? N->normal[Component] : 0.0; })
			.MinValue(-1e9).MaxValue(1e9).Delta(0.1)
			.LiveUpdate(true)
			.OnValueCommitted_Lambda([D, Component](double V, ETextCommit::Type) { if (PlaneNetwork* N = D->Net()) { N->normal[Component] = V; D->Touch(); } });
	};
	auto AddNetwork = [D](const PlaneNetwork& Template)
	{
		D->S.networks.push_back(Template);
		D->Selected = static_cast<int32>(D->S.networks.size()) - 1;
		D->RebuildNetworkItems();
		D->Touch();
	};

	TSharedRef<SWidget> PlanesTab =
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().FillWidth(0.36f).Padding(0, 0, 10, 0)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 2)
			[ SNew(STextBlock).Text(LOCTEXT("Networks", "Plane networks")).Font(FCoreStyle::GetDefaultFontStyle("Bold", 9)) ]
			+ SVerticalBox::Slot().FillHeight(1.0f)
			[
				SNew(SBorder).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(FLinearColor(0.05f, 0.05f, 0.06f)).Padding(2)
				[
					SAssignNew(D->NetworkList, SListView<TSharedPtr<int32>>)
					.ListItemsSource(&D->NetworkItems)
					.SelectionMode(ESelectionMode::Single)
					.OnGenerateRow_Lambda([D](TSharedPtr<int32> Item, const TSharedRef<STableViewBase>& Owner)
					{
						return SNew(STableRow<TSharedPtr<int32>>, Owner).Padding(FMargin(4, 2))
							[
								SNew(STextBlock).Text_Lambda([D, Item]()
								{
									if (!Item.IsValid() || *Item < 0 || *Item >= static_cast<int32>(D->S.networks.size())) return FText::GetEmpty();
									const PlaneNetwork& N = D->S.networks[*Item];
									return FText::FromString(FString::Printf(TEXT("%s (%d)"), UTF8_TO_TCHAR(N.name.c_str()), N.count));
								})
							];
					})
					.OnSelectionChanged_Lambda([D](TSharedPtr<int32> Item, ESelectInfo::Type Info)
					{
						if (Info == ESelectInfo::Direct) return;
						D->Selected = Item.IsValid() ? *Item : INDEX_NONE;
						D->SyncNetworkWidgets();
						D->PushPreview();
					})
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 4, 0)
				[ Button(LOCTEXT("AddRibs", "Add ribs"), LOCTEXT("AddRibsTip", "A network of planes normal to y: ribs across a wing"), [AddNetwork]() { PlaneNetwork N; N.name = "Ribs"; N.axis = aerotelic::PlaneAxis::Y; N.count = 5; AddNetwork(N); }) ]
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 4, 0)
				[ Button(LOCTEXT("AddSpar", "Add spar"), LOCTEXT("AddSparTip", "A single plane normal to x: a spar along a wing"), [AddNetwork]() { PlaneNetwork N; N.name = "Spar"; N.axis = aerotelic::PlaneAxis::X; N.count = 1; N.thickness = 0.003; AddNetwork(N); }) ]
				+ SHorizontalBox::Slot().AutoWidth()
				[ Button(LOCTEXT("RemoveNet", "Remove"), LOCTEXT("RemoveNetTip", "Remove the selected network"), [D]()
				{
					if (D->Net() == nullptr) return;
					D->S.networks.erase(D->S.networks.begin() + D->Selected);
					D->RebuildNetworkItems();
					D->Touch();
				}) ]
			]
		]
		+ SHorizontalBox::Slot().FillWidth(0.64f)
		[
			SNew(SScrollBox)
			+ SScrollBox::Slot()
			[
				SNew(SBox).IsEnabled_Lambda([D]() { return D->Net() != nullptr; })
				[
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight()
					[
						FormRow(LOCTEXT("NetName", "Name:"),
							SAssignNew(D->NameBox, SEditableTextBox)
							.Text(FText::FromString(D->Net() ? UTF8_TO_TCHAR(D->Net()->name.c_str()) : TEXT("")))
							.OnTextCommitted_Lambda([D](const FText& T, ETextCommit::Type)
							{
								if (PlaneNetwork* N = D->Net()) { N->name = TCHAR_TO_UTF8(*T.ToString()); D->Touch(); if (D->NetworkList.IsValid()) D->NetworkList->RequestListRefresh(); }
							}))
					]
					+ SVerticalBox::Slot().AutoHeight()
					[
						FormRow(LOCTEXT("NetAxis", "Planes normal to:"),
							SAssignNew(D->AxisCombo, SComboBox<TSharedPtr<FString>>)
							.OptionsSource(&D->AxisOptions)
							.InitiallySelectedItem(D->AxisChoice)
							.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
							.OnSelectionChanged_Lambda([D](TSharedPtr<FString> S, ESelectInfo::Type Info)
							{
								if (!S.IsValid() || Info == ESelectInfo::Direct) return;
								D->AxisChoice = S;
								if (PlaneNetwork* N = D->Net())
								{
									N->axis = static_cast<aerotelic::PlaneAxis>(FMath::Clamp(D->AxisOptions.IndexOfByKey(S), 0, 3));
									if (N->axis == aerotelic::PlaneAxis::Custom && !(aerotelic::Length(N->normal) > 0.0)) N->normal = aerotelic::Vec3{0.0, 1.0, 0.0};
									D->Touch();
								}
							})
							[ SNew(STextBlock).Text_Lambda([D]() { return FText::FromString(D->AxisChoice.IsValid() ? *D->AxisChoice : FString()); }) ])
					]
					+ SVerticalBox::Slot().AutoHeight()
					[
						SNew(SBox).Visibility_Lambda([D]() { const PlaneNetwork* N = D->Net(); return N && N->axis == aerotelic::PlaneAxis::Custom ? EVisibility::Visible : EVisibility::Collapsed; })
						[
							SNew(SVerticalBox)
							+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("NormalX", "Normal x:"), NormalField(0)) ]
							+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("NormalY", "Normal y:"), NormalField(1)) ]
							+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("NormalZ", "Normal z:"), NormalField(2)) ]
						]
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
					[ FormRow(WithUnit(LOCTEXT("Start", "First plane at"), Units.LengthName()), SNew(SAerotelicNumericEntry)
						.Value_Lambda([D]() { const PlaneNetwork* N = D->Net(); return N ? N->start : 0.0; })
						.MinValue(-1e9).MaxValue(1e9).Delta(0.01)
						.LiveUpdate(true)
						.OnValueCommitted_Lambda([D](double V, ETextCommit::Type) { if (PlaneNetwork* N = D->Net()) { N->start = V; D->Touch(); } })) ]
					+ SVerticalBox::Slot().AutoHeight()
					[ FormRow(WithUnit(LOCTEXT("Spacing", "Spacing"), Units.LengthName()), NetField(&PlaneNetwork::spacing, 0.0, 0.01)) ]
					+ SVerticalBox::Slot().AutoHeight()
					[ FormRow(LOCTEXT("Count", "Number of planes:"), SNew(SAerotelicNumericEntry)
						.Value_Lambda([D]() { const PlaneNetwork* N = D->Net(); return N ? static_cast<double>(N->count) : 1.0; })
						.MinValue(1).MaxValue(500).Delta(1)
						.LiveUpdate(false)
						.OnValueCommitted_Lambda([D](double V, ETextCommit::Type) { if (PlaneNetwork* N = D->Net()) { N->count = FMath::Clamp(FMath::RoundToInt(V), 1, 500); D->Touch(); if (D->NetworkList.IsValid()) D->NetworkList->RequestListRefresh(); } })) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
					[ FormRow(WithUnit(LOCTEXT("Thickness", "Rib thickness"), Units.LengthName()), NetField(&PlaneNetwork::thickness, 0.0, 0.0005)) ]
					+ SVerticalBox::Slot().AutoHeight()
					[ FormRow(WithUnit(LOCTEXT("Height", "Height in from the surface, 0 = wall to wall"), Units.LengthName()), NetField(&PlaneNetwork::height, 0.0, 0.001), 300.0f) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
					[
						FormRow(LOCTEXT("NetMaterial", "Rib material:"),
							SNew(SAerotelicMaterialPicker)
							.Current_Lambda([D]() { const PlaneNetwork* N = D->Net(); return N ? N->material : Material{}; })
							.AllowNone(false)
							.OnPicked_Lambda([D](const Material& M) { if (PlaneNetwork* N = D->Net()) { N->material = M; D->Touch(); } }))
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0)
					[
						SNew(STextBlock).AutoWrapText(true)
						.ColorAndOpacity(FSlateColor(FLinearColor(0.95f, 0.85f, 0.55f)))
						.Text_Lambda([D]()
						{
							const PlaneNetwork* N = D->Net();
							if (N == nullptr) return FText::GetEmpty();
							const std::string Why = aerotelic::PlaneNetworkProblem(*N);
							return Why.empty() ? FText::GetEmpty() : FText::FromString(FString::Printf(TEXT("This network cannot be built: %s."), UTF8_TO_TCHAR(Why.c_str())));
						})
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0)
					[
						MakeInfoLabel(LOCTEXT("PlanesHint",
							"A network is a set of parallel cutting planes: plane k lies at (first plane) + k x spacing along the normal, in the parameterisation's own frame -- so a wing's ribs are normal to y and stay with the wing when it moves, scales or mirrors. Each plane's intersection with the interior is a rib of the thickness, in the material, spanning wall to wall; a height makes it a band that reaches only that far in from the surface (a stringer, a spar cap). "
							"Earlier networks win where two cross: a spar runs through, the ribs are notched. The 3D view shows the planes as translucent rectangles while this window is open, the selected network brighter. Positions and spacing are geometry (scaled with the model); thickness and height are build figures (not scaled), like the skin."))
					]
				]
			]
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
			MakeInfoLabel(LOCTEXT("MassHint", "An asserted mass is spread over the body's surface when weighed and has no stiffness; it replaces every computed figure. Otherwise the body weighs by its structure: the skin or solid exactly, the ribs and the infill estimated from the planes' sections and the volume fraction until the structure is generated."))
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
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
		[
			SNew(SButton).Text(LOCTEXT("Apply", "Apply")).ToolTipText(LOCTEXT("ApplyTip", "Give the body this structure. Analysis > Mass Properties weighs by it and Analysis > Structural analyses with it; a lattice over the body regenerates when the geometry changed."))
			.IsEnabled_Lambda([D, Weak]() { return D->bDirty && Weak.IsValid() && aerotelic::BodyStructureProblem(D->S).empty(); })
			.OnClicked_Lambda([D]()
			{
				if (D->Entity.IsValid() && D->Entity->SetBodyStructure(D->Role, D->S))
				{
					D->bDirty = false;
				}
				return FReply::Handled();
			})
		];

	TArray<FAerotelicTab> Tabs;
	Tabs.Add({ LOCTEXT("SkinTab", "Skin"), SkinTab });
	Tabs.Add({ LOCTEXT("InfillTab", "Infill"), InfillTab });
	Tabs.Add({ LOCTEXT("PlanesTab", "Planes"), PlanesTab });
	Tabs.Add({ LOCTEXT("MassTab", "Mass"), MassTab });

	const FVector2D Size(720, 640);
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
	Window->SetOnWindowClosed(FOnWindowClosed::CreateLambda([D, Key](const TSharedRef<SWindow>&)
	{
		GOpenStructureDialogs.Remove(Key);
		if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get()) Doc->ClearStructurePreview();
	}));

	GOpenStructureDialogs.Add(Key, Window);
	ShowAerotelicDialogWindow(Window, Size);
}

#undef LOCTEXT_NAMESPACE
