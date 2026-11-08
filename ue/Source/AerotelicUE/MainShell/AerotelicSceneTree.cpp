#include "AerotelicSceneTree.h"
#include "Widgets/MessageBox.h"
#include "AerotelicDesignDock.h"
#include "AerotelicTransformDialog.h"
#include "AerotelicStructureDialog.h"

#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Views/STableRow.h"
#include "Widgets/Colors/SColorBlock.h"
#include "Widgets/Colors/SColorPicker.h"
#include "Framework/MultiBox/MultiBoxBuilder.h"
#include "Styling/CoreStyle.h"
#include "Misc/MessageDialog.h"

#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"
#include "Entity/AerotelicBooleanModel.h"
#include "Entity/AerotelicBodyStructureText.h"
#include "Branding/AerotelicTheme.h"
#include "Widgets/PromptDialogs.h"

#define LOCTEXT_NAMESPACE "AerotelicSceneTree"

using namespace aerotelic;

namespace
{
	FText EntityTypeLabel(project_yaml::EntityType Type)
	{
		switch (Type)
		{
		case project_yaml::EntityType::Wing: return LOCTEXT("TypeWing", "wing");
		case project_yaml::EntityType::Fuselage: return LOCTEXT("TypeFuselage", "fuselage");
		case project_yaml::EntityType::Nacelle: return LOCTEXT("TypeNacelle", "nacelle");
		case project_yaml::EntityType::LoftedNacelle: return LOCTEXT("TypeLoftedNacelle", "lofted nacelle");
		case project_yaml::EntityType::Propeller: return LOCTEXT("TypePropeller", "propeller");
		case project_yaml::EntityType::Primitive: return LOCTEXT("TypePrimitive", "primitive");
		case project_yaml::EntityType::Boolean: return LOCTEXT("TypeBoolean", "boolean");
		}
		return FText::GetEmpty();
	}

	const FLinearColor kDimText(0.55f, 0.55f, 0.55f);

	// Opens the engine's colour picker (AppFramework's SColorPicker, the
	// same one the Editor uses) on Initial, calling OnPicked live as the
	// user drags. Modeless, unlike QColorDialog.getColor: the game viewport
	// does not repaint while a Slate modal runs, so a modal picker would
	// hide its own live preview until OK; a floating picker lets the body
	// take each colour as it is chosen. No alpha: WingDesigner's colours
	// have none.
	void PickColor(const FText& Title, FLinearColor Initial, TSharedPtr<SWidget> Parent, TFunction<void(FLinearColor)> OnPicked)
	{
		FColorPickerArgs Args;
		Args.bIsModal = false;
		Args.bUseAlpha = false;
		Args.bExpandAdvancedSection = false;
		Args.InitialColor = Initial;
		Args.ParentWidget = Parent;
		Args.OnColorCommitted = FOnLinearColorValueChanged::CreateLambda([OnPicked](FLinearColor C) { OnPicked(C); });
		// Cancel restores what the picker opened on.
		Args.OnColorPickerCancelled = FOnColorPickerCancelled::CreateLambda([OnPicked, Initial](FLinearColor) { OnPicked(Initial); });
		OpenColorPicker(Args);
	}
}

void SAerotelicSceneTree::Construct(const FArguments& InArgs)
{
	OnStatusDelegate = InArgs._OnStatus;

	ChildSlot
	[
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(6, 4)
		[
			SNew(STextBlock)
			.Text(LOCTEXT("Header", "Scene"))
			.Font(FCoreStyle::GetDefaultFontStyle("Bold", 10))
		]
		+ SVerticalBox::Slot().FillHeight(1.0f).Padding(2)
		[
			SNew(SBorder)
			.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
			.BorderBackgroundColor_Lambda([]() { return FSlateColor(AerotelicTheme::Current().InputBg); })
			.Padding(2)
			[
				SAssignNew(Tree, STreeView<FItem>)
				.TreeItemsSource(&Items)
				.SelectionMode(ESelectionMode::Single)
				.OnGenerateRow(this, &SAerotelicSceneTree::GenerateRow)
				.OnGetChildren(this, &SAerotelicSceneTree::GetChildren)
				.OnSelectionChanged(this, &SAerotelicSceneTree::OnSelectionChanged)
				.OnMouseButtonDoubleClick(this, &SAerotelicSceneTree::OnDoubleClick)
				.OnContextMenuOpening(this, &SAerotelicSceneTree::OnContextMenuOpening)
			]
		]
	];

	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		ListChangedHandle = Doc->OnEntityListChanged.AddSP(this, &SAerotelicSceneTree::RefreshItems);
		SelectionChangedHandle = Doc->OnSelectionChanged.AddSP(this, &SAerotelicSceneTree::SyncSelectionFromDocument);
	}
	BodiesChangedHandle = AAerotelicEntityActor::OnBodiesChanged.AddSP(this, &SAerotelicSceneTree::OnBodiesChanged);
	RefreshItems();
}

SAerotelicSceneTree::~SAerotelicSceneTree()
{
	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		Doc->OnEntityListChanged.Remove(ListChangedHandle);
		Doc->OnSelectionChanged.Remove(SelectionChangedHandle);
	}
	AAerotelicEntityActor::OnBodiesChanged.Remove(BodiesChangedHandle);
}

void SAerotelicSceneTree::Status(const FText& Text)
{
	OnStatusDelegate.ExecuteIfBound(Text);
}

void SAerotelicSceneTree::EditEntity(AAerotelicEntityActor* Entity)
{
	if (Entity == nullptr)
	{
		return;
	}
	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		Doc->SetSelectedEntity(Entity);
	}
	// A boolean result has no parameters of its own to edit: its operands
	// do. Selecting it is all there is.
	if (Entity->GetEntityType() != project_yaml::EntityType::Boolean)
	{
		ShowAerotelicDesignTab();
	}
}

// ------------------------------------------------------------------- rows

TSharedRef<ITableRow> SAerotelicSceneTree::GenerateRow(FItem Item, const TSharedRef<STableViewBase>& OwnerTable)
{
	using EKind = FAerotelicSceneRow::EKind;
	const EKind Kind = Item->Kind;
	const int32 Body = Item->BodyIndex;
	const int32 Layer = Item->LayerIndex;
	TWeakObjectPtr<AAerotelicEntityActor> Entity = Item->Entity;

	// The checkbox: the entity's, a body's, a mirrored body's, or a
	// generated layer's visibility.
	auto IsChecked = [Entity, Kind, Body, Layer]()
	{
		if (!Entity.IsValid()) return ECheckBoxState::Unchecked;
		switch (Kind)
		{
		case EKind::Entity: return Entity->IsEntityVisible() ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
		case EKind::Body: return Entity->IsBodyVisible(Body) ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
		case EKind::Mirror: return Entity->IsMirrorBodyVisible(Body) ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
		case EKind::Layer: return Entity->IsLayerVisible(Layer) ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
		case EKind::MirrorLayer: return Entity->IsMirrorLayerVisible(Layer) ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
		}
		return ECheckBoxState::Unchecked;
	};
	auto OnChecked = [Entity, Kind, Body, Layer](ECheckBoxState State)
	{
		if (!Entity.IsValid()) return;
		const bool bOn = State == ECheckBoxState::Checked;
		switch (Kind)
		{
		case EKind::Entity: Entity->SetEntityVisible(bOn); break;
		case EKind::Body: Entity->SetBodyVisible(Body, bOn); break;
		case EKind::Mirror: Entity->SetMirrorBodyVisible(Body, bOn); break;
		case EKind::Layer: Entity->SetLayerVisible(Layer, bOn); break;
		case EKind::MirrorLayer: Entity->SetMirrorLayerVisible(Layer, bOn); break;
		}
	};
	auto Name = [Entity, Kind, Body, Layer]()
	{
		if (!Entity.IsValid()) return FText::GetEmpty();
		switch (Kind)
		{
		case EKind::Entity: return FText::FromString(Entity->EntityName);
		case EKind::Body: return FText::FromString(Entity->GetBodyDisplayName(Body));
		case EKind::Mirror: return FText::FromString(Entity->GetMirrorBodyDisplayName(Body));
		case EKind::Layer: return Entity->GetLayers().IsValidIndex(Layer) ? FText::FromString(Entity->GetLayers()[Layer].Name) : FText::GetEmpty();
		case EKind::MirrorLayer: return Entity->GetLayers().IsValidIndex(Layer) ? FText::FromString(Entity->GetLayers()[Layer].Name) : FText::GetEmpty();
		}
		return FText::GetEmpty();
	};
	// The grey annotation: an entity's kind; a body's role (Main, Spinner,
	// Blade 2 ...); a mirror's role plus "mirror"; a layer's kind (and
	// "stale" while its body changed under it, "empty" when the generation
	// produced nothing for it).
	auto Annotation = [Entity, Kind, Body, Layer]()
	{
		if (!Entity.IsValid()) return FText::GetEmpty();
		switch (Kind)
		{
		case EKind::Entity: return EntityTypeLabel(Entity->GetEntityType());
		case EKind::Body:
			return Entity->GetBodies().IsValidIndex(Body) ? FText::FromString(Entity->GetBodies()[Body].Name) : FText::GetEmpty();
		case EKind::Mirror:
			// The name already ends in "-mirror"; the annotation says what
			// is mirrored.
			return Entity->GetBodies().IsValidIndex(Body) ? FText::FromString(Entity->GetBodies()[Body].Name) : LOCTEXT("MirrorOnly", "mirror");
		case EKind::Layer:
		case EKind::MirrorLayer:
		{
			if (!Entity->GetLayers().IsValidIndex(Layer)) return FText::GetEmpty();
			const FAerotelicLayerView& L = Entity->GetLayers()[Layer];
			FString Text = FAerotelicStructureLayer::KindName(L.Kind);
			if (L.Section == INDEX_NONE) Text += TEXT(", empty");
			const FAerotelicBodyStructure* G = Entity->GetGeneratedStructure(L.BodyRole);
			if (G != nullptr && G->IsStale()) Text += TEXT(", stale");
			return FText::FromString(Text);
		}
		}
		return FText::GetEmpty();
	};
	auto Swatch = [Entity, Kind, Body, Layer]()
	{
		if (!Entity.IsValid()) return FLinearColor::Transparent;
		if (Kind == EKind::Entity) return Entity->GetDisplayColor();
		if (Kind == EKind::Layer || Kind == EKind::MirrorLayer) return Entity->GetLayerDisplayColor(Layer);
		return Entity->GetBodyDisplayColor(Body);
	};
	// The tooltip: what the body is made of and built with (an entity's,
	// one line per body); a layer's material, thickness and volume.
	auto Tip = [Entity, Kind, Body, Layer]()
	{
		if (!Entity.IsValid()) return FText::GetEmpty();
		const UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
		const aerotelic::UnitSystem Units = Doc != nullptr ? Doc->GetUnits() : aerotelic::UnitSystem{};
		if (Kind == EKind::Entity) return FText::FromString(DescribeAerotelicEntityStructures(Entity.Get(), Units));
		if (Kind == EKind::Layer || Kind == EKind::MirrorLayer)
		{
			if (!Entity->GetLayers().IsValidIndex(Layer)) return FText::GetEmpty();
			const FAerotelicLayerView& V = Entity->GetLayers()[Layer];
			const FAerotelicBodyStructure* G = Entity->GetGeneratedStructure(V.BodyRole);
			if (G == nullptr || V.LayerIndex < 0 || V.LayerIndex >= static_cast<int32>(G->GetLayers().size())) return FText::GetEmpty();
			const FAerotelicStructureLayer& L = G->GetLayers()[V.LayerIndex];
			FString Text = FString::Printf(TEXT("%s: %s"), FAerotelicStructureLayer::KindName(L.Kind), UTF8_TO_TCHAR(L.Material.name.c_str()));
			if (L.Thickness > 0.0) Text += FString::Printf(TEXT(", %.4g %s thick"), L.Thickness, UTF8_TO_TCHAR(Units.LengthName()));
			Text += FString::Printf(TEXT("; %s triangles, volume %.5g %s^3"), *FText::AsNumber(static_cast<int64>(L.Mesh.triangles.size())).ToString(), L.Volume, UTF8_TO_TCHAR(Units.LengthName()));
			if (!L.Note.IsEmpty()) Text += TEXT("\n") + L.Note;
			return FText::FromString(Text);
		}
		return FText::FromString(DescribeAerotelicBodyStructure(Entity->GetBodyStructure(Body), Units));
	};

	return SNew(STableRow<FItem>, OwnerTable)
		.Padding(FMargin(2, 1))
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 4, 0)
			[
				SNew(SCheckBox)
				.ToolTipText(LOCTEXT("VisibleTip", "Show or hide in the 3D view"))
				.IsChecked_Lambda(IsChecked)
				.OnCheckStateChanged_Lambda(OnChecked)
			]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 6, 0)
			[
				// The colour the row's geometry is drawn in.
				SNew(SColorBlock)
				.Color_Lambda(Swatch)
				.Size(FVector2D(10, 10))
			]
			+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
			[
				SNew(STextBlock)
				.Text_Lambda(Name)
				.ToolTipText_Lambda(Tip)
				.OverflowPolicy(ETextOverflowPolicy::Ellipsis)
				.ColorAndOpacity(Kind == EKind::Entity ? FSlateColor::UseForeground() : FSlateColor(FLinearColor(0.8f, 0.8f, 0.8f)))
			]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(6, 0, 2, 0)
			[
				SNew(STextBlock)
				.ColorAndOpacity(FSlateColor(kDimText))
				.Text_Lambda(Annotation)
			]
		];
}

void SAerotelicSceneTree::GetChildren(FItem Item, TArray<FItem>& OutChildren)
{
	OutChildren = Item->Children;
}

void SAerotelicSceneTree::OnSelectionChanged(FItem Item, ESelectInfo::Type SelectInfo)
{
	if (bSyncingSelection)
	{
		return;
	}
	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		// A body or mirror row selects the parameterisation it belongs to.
		Doc->SetSelectedEntity(Item.IsValid() && Item->Entity.IsValid() ? Item->Entity.Get() : nullptr);
	}
}

void SAerotelicSceneTree::OnDoubleClick(FItem Item)
{
	if (Item.IsValid() && Item->Entity.IsValid())
	{
		EditEntity(Item->Entity.Get());
		Status(FText::Format(LOCTEXT("StatusEditing", "Editing {0}"), FText::FromString(Item->Entity->EntityName)));
	}
}

void SAerotelicSceneTree::RefreshItems()
{
	using EKind = FAerotelicSceneRow::EKind;
	// Keep the user's expansion where a row survives the rebuild.
	TSet<AAerotelicEntityActor*> WasExpanded;
	// ... and of the body rows that have layers under them (keyed by
	// entity, kind and body), which open the first time they get layers.
	TSet<FString> BodyWasExpanded, BodyWasSeen;
	auto BodyKey = [](const FItem& Row) { return FString::Printf(TEXT("%p|%d|%d"), Row->Entity.Get(), static_cast<int32>(Row->Kind), Row->BodyIndex); };
	for (const FItem& Old : Items)
	{
		if (Old->Entity.IsValid() && Tree.IsValid() && Tree->IsItemExpanded(Old))
		{
			WasExpanded.Add(Old->Entity.Get());
		}
		for (const FItem& OldBody : Old->Children)
		{
			if (OldBody->Children.Num() == 0) continue;
			BodyWasSeen.Add(BodyKey(OldBody));
			if (Tree.IsValid() && Tree->IsItemExpanded(OldBody)) BodyWasExpanded.Add(BodyKey(OldBody));
		}
	}
	const bool bFirstFill = Items.Num() == 0;

	Items.Reset();
	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		for (const TWeakObjectPtr<AAerotelicEntityActor>& E : Doc->GetEntities())
		{
			if (!E.IsValid())
			{
				continue;
			}
			FItem Row = MakeShared<FAerotelicSceneRow>();
			Row->Kind = EKind::Entity;
			Row->Entity = E;
			const int32 NumBodies = E->GetBodies().Num();
			const TArray<FAerotelicLayerView>& Layers = E->GetLayers();
			auto AddLayers = [&](const FItem& Parent, int32 BodyIndex, EKind LayerKind)
			{
				for (int32 l = 0; l < Layers.Num(); ++l)
				{
					if (Layers[l].BodyIndex != BodyIndex) continue;
					FItem LayerRow = MakeShared<FAerotelicSceneRow>();
					LayerRow->Kind = LayerKind;
					LayerRow->Entity = E;
					LayerRow->BodyIndex = BodyIndex;
					LayerRow->LayerIndex = l;
					Parent->Children.Add(LayerRow);
				}
			};
			for (int32 i = 0; i < NumBodies; ++i)
			{
				FItem Body = MakeShared<FAerotelicSceneRow>();
				Body->Kind = EKind::Body;
				Body->Entity = E;
				Body->BodyIndex = i;
				AddLayers(Body, i, EKind::Layer);
				Row->Children.Add(Body);
			}
			if (E->IsMirrored())
			{
				for (int32 i = 0; i < NumBodies; ++i)
				{
					FItem Mirror = MakeShared<FAerotelicSceneRow>();
					Mirror->Kind = EKind::Mirror;
					Mirror->Entity = E;
					Mirror->BodyIndex = i;
					AddLayers(Mirror, i, EKind::MirrorLayer);
					Row->Children.Add(Mirror);
				}
			}
			Items.Add(Row);
		}
	}
	if (Tree.IsValid())
	{
		Tree->RequestTreeRefresh();
		// WingDesigner's policy: a row stays collapsed while it has only one
		// body; with more it opens, and after that the user's state holds. A
		// body with generated layers opens to show them, then holds its state.
		for (const FItem& Row : Items)
		{
			bool bHasLayers = false;
			for (const FItem& Body : Row->Children) bHasLayers = bHasLayers || Body->Children.Num() > 0;
			const bool bExpand = WasExpanded.Contains(Row->Entity.Get()) || (bFirstFill || !WasExpanded.Contains(Row->Entity.Get())) && (Row->Children.Num() > 1 || bHasLayers);
			Tree->SetItemExpansion(Row, bExpand);
			for (const FItem& Body : Row->Children)
			{
				if (Body->Children.Num() == 0) continue;
				const FString Key = BodyKey(Body);
				Tree->SetItemExpansion(Body, !BodyWasSeen.Contains(Key) || BodyWasExpanded.Contains(Key));
			}
		}
	}
	SyncSelectionFromDocument();
}

void SAerotelicSceneTree::OnBodiesChanged(AAerotelicEntityActor*)
{
	RefreshItems();
}

void SAerotelicSceneTree::SyncSelectionFromDocument()
{
	if (!Tree.IsValid())
	{
		return;
	}
	TGuardValue<bool> Guard(bSyncingSelection, true);
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	AAerotelicEntityActor* Selected = Doc ? Doc->GetSelectedEntity() : nullptr;
	if (Selected == nullptr)
	{
		Tree->ClearSelection();
		return;
	}
	// A body row of the selected entity already selected: leave it.
	for (const FItem& Current : Tree->GetSelectedItems())
	{
		if (Current.IsValid() && Current->Entity.Get() == Selected)
		{
			return;
		}
	}
	for (const FItem& Item : Items)
	{
		if (Item->Entity.Get() == Selected)
		{
			Tree->SetSelection(Item, ESelectInfo::Direct);
			Tree->RequestScrollIntoView(Item);
			return;
		}
	}
	Tree->ClearSelection();
}

// ----------------------------------------------------------- context menu

TSharedPtr<SWidget> SAerotelicSceneTree::OnContextMenuOpening()
{
	TArray<FItem> Selected = Tree->GetSelectedItems();
	if (Selected.Num() == 0 || !Selected[0].IsValid() || !Selected[0]->Entity.IsValid())
	{
		return nullptr;
	}
	switch (Selected[0]->Kind)
	{
	case FAerotelicSceneRow::EKind::Entity: return MakeEntityMenu(Selected[0]);
	case FAerotelicSceneRow::EKind::Layer:
	case FAerotelicSceneRow::EKind::MirrorLayer: return MakeLayerMenu(Selected[0]);
	default: return MakeBodyMenu(Selected[0]);
	}
}

TSharedPtr<SWidget> SAerotelicSceneTree::MakeEntityMenu(FItem Item)
{
	TWeakObjectPtr<AAerotelicEntityActor> Entity = Item->Entity;
	const bool bVisible = Entity->IsEntityVisible();
	// A boolean result is shown, coloured, renamed and deleted like any
	// row, but has no design panel, placement or mirror of its own
	// (main_window.py's result-row menu).
	const bool bBoolean = Entity->GetEntityType() == project_yaml::EntityType::Boolean;
	const bool bDerived = bBoolean;

	FMenuBuilder Menu(true, nullptr, nullptr, false, &FCoreStyle::Get(), /*bSearchable*/ false);

	Menu.AddMenuEntry(
		bVisible ? LOCTEXT("Hide", "Hide") : LOCTEXT("Show", "Show"),
		LOCTEXT("HideTip", "Show or hide this entity in the 3D view"),
		FSlateIcon(),
		FUIAction(FExecuteAction::CreateLambda([Entity, bVisible]()
		{
			if (Entity.IsValid()) Entity->SetEntityVisible(!bVisible);
		})));

	if (!bBoolean)
	{
		Menu.AddMenuEntry(LOCTEXT("Edit", "Edit"), LOCTEXT("EditTip", "Open this entity's design panel"), FSlateIcon(),
			FUIAction(FExecuteAction::CreateLambda([this, Entity]()
			{
				if (Entity.IsValid())
				{
					EditEntity(Entity.Get());
					Status(FText::Format(LOCTEXT("StatusEditing", "Editing {0}"), FText::FromString(Entity->EntityName)));
				}
			})));
	}

	Menu.AddMenuEntry(LOCTEXT("Rename", "Rename"), LOCTEXT("RenameTip", "Give this entity a new, unique name"), FSlateIcon(),
		FUIAction(FExecuteAction::CreateLambda([this, Entity]() { if (Entity.IsValid()) RenameEntity(Entity.Get()); })));

	// Only on a parameterisation: a derived row has nothing to hand a
	// constructor (see UAerotelicDocumentSubsystem::CanCopyEntity).
	if (!bDerived)
	{
		Menu.AddMenuEntry(LOCTEXT("Copy", "Copy"),
			LOCTEXT("CopyTip", "Copy this parameterisation. Edit > Paste (Ctrl+V) then adds a second, independent entity from it -- edits to one never reach the other."),
			FSlateIcon(),
			FUIAction(FExecuteAction::CreateLambda([this, Entity]()
			{
				UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
				if (!Entity.IsValid() || Doc == nullptr || !Doc->CopyEntityToClipboard(Entity.Get()))
				{
					return;
				}
				Status(FText::Format(LOCTEXT("StatusCopied", "Copied {0} -- Edit > Paste (Ctrl+V) adds an independent copy"),
					FText::FromString(Entity->EntityName)));
			})));
	}

	if (!bDerived)
	{
		Menu.AddMenuEntry(LOCTEXT("Transform", "Transform"), LOCTEXT("TransformTip", "Translate and rotate this entity"), FSlateIcon(),
			FUIAction(FExecuteAction::CreateLambda([Entity]() { if (Entity.IsValid()) OpenAerotelicTransformDialog(Entity.Get()); })));
	}

	// The structure -- material, skin, plane networks, asserted mass --
	// belongs to each BODY (2026-11-08): an entity with one body
	// offers it here too; one with several has it on each body's row. A
	// boolean result's bodies own theirs and can start from an operand's.
	if (Entity->GetBodies().Num() == 1)
	{
		const FString Role = Entity->GetBodies()[0].Name;
		Menu.AddMenuEntry(LOCTEXT("Structure", "Structure..."),
			LOCTEXT("StructureTip", "What this body is made of and built with: its material and skin thickness, networks of cutting planes (ribs, spars, frames) and their materials, or an asserted mass. Analysis > Mass Properties weighs by it; Analysis > Structural analyses with it."),
			FSlateIcon(),
			FUIAction(FExecuteAction::CreateLambda([Entity, Role]() { if (Entity.IsValid()) OpenAerotelicStructureDialog(Entity.Get(), Role); })));
		Menu.AddMenuEntry(Entity->GetGeneratedStructure(Role) != nullptr ? LOCTEXT("RegenerateStructure", "Regenerate Structure") : LOCTEXT("GenerateStructure", "Generate Structure"),
			LOCTEXT("GenerateStructureTip", "Build this body's skin shell and ribs from its structure (seconds of MeshLib work): they appear as layers under the body, and its mirrored half's, and follow every later edit of the body."),
			FSlateIcon(),
			FUIAction(FExecuteAction::CreateLambda([this, Entity, Role]() { if (Entity.IsValid()) GenerateStructure(Entity.Get(), Role); })));
	}
	if (bBoolean)
	{
		Menu.AddMenuEntry(LOCTEXT("CopyStructure", "Copy Structure From Operand"),
			LOCTEXT("CopyStructureTip", "Give every body of this result the structure of its first operand's first body, as a starting point (a boolean result's bodies own their structure; nothing is inherited)."),
			FSlateIcon(),
			FUIAction(FExecuteAction::CreateLambda([this, Entity]()
			{
				UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
				if (!Entity.IsValid() || Doc == nullptr) return;
				const int32 N = Doc->CopyStructuresFromOperand(Entity.Get());
				Status(FText::Format(LOCTEXT("StatusCopiedStructure", "{0}: {1} bodies took the first operand's structure"), FText::FromString(Entity->EntityName), FText::AsNumber(N)));
			})));
	}

	Menu.AddMenuEntry(LOCTEXT("Color", "Color"), LOCTEXT("ColorTip", "Pick the colour every body of this entity is drawn in (the mirrored ones included)"), FSlateIcon(),
		FUIAction(FExecuteAction::CreateLambda([this, Entity]() { if (Entity.IsValid()) PickEntityColor(Entity.Get()); })));

	Menu.AddMenuEntry(LOCTEXT("DefaultColor", "Default Color"), LOCTEXT("DefaultColorTip", "Back to the colour this kind of entity is drawn in by default"), FSlateIcon(),
		FUIAction(FExecuteAction::CreateLambda([Entity]() { if (Entity.IsValid()) Entity->SetColorOverride(TOptional<FLinearColor>()); }),
			FCanExecuteAction::CreateLambda([Entity]() { return Entity.IsValid() && Entity->GetColorOverride().IsSet(); })));

	if (!bDerived)
	{
		Menu.AddMenuSeparator();

		Menu.AddMenuEntry(LOCTEXT("Mirrored", "Mirrored"),
			LOCTEXT("MirroredTip", "Derive a port-side copy by reflecting about Y = 0. The mirrored half is never edited on its own -- it always follows this entity."),
			FSlateIcon(),
			FUIAction(
				FExecuteAction::CreateLambda([this, Entity]()
				{
					if (!Entity.IsValid()) return;
					Entity->SetMirrored(!Entity->IsMirrored());
					Status(FText::Format(Entity->IsMirrored() ? LOCTEXT("StatusMirrored", "{0} is now mirrored") : LOCTEXT("StatusUnmirrored", "{0} is no longer mirrored"),
						FText::FromString(Entity->EntityName)));
				}),
				FCanExecuteAction(),
				FIsActionChecked::CreateLambda([Entity]() { return Entity.IsValid() && Entity->IsMirrored(); })),
			NAME_None, EUserInterfaceActionType::ToggleButton);
	}

	Menu.AddMenuSeparator();

	Menu.AddMenuEntry(LOCTEXT("Delete", "Delete"), LOCTEXT("DeleteTip", "Remove this entity from the scene"), FSlateIcon(),
		FUIAction(FExecuteAction::CreateLambda([this, Entity]() { if (Entity.IsValid()) DeleteEntity(Entity.Get()); })));

	return Menu.MakeWidget();
}

TSharedPtr<SWidget> SAerotelicSceneTree::MakeBodyMenu(FItem Item)
{
	// A solid body's menu changes how that one body is shown, nothing
	// about how it was made: Hide/Show and Color (WingDesigner's
	// _body_context_menu, less Rename).
	using EKind = FAerotelicSceneRow::EKind;
	TWeakObjectPtr<AAerotelicEntityActor> Entity = Item->Entity;
	const int32 Body = Item->BodyIndex;
	const bool bMirror = Item->Kind == EKind::Mirror;
	const bool bVisible = bMirror ? Entity->IsMirrorBodyVisible(Body) : Entity->IsBodyVisible(Body);

	FMenuBuilder Menu(true, nullptr, nullptr, false, &FCoreStyle::Get(), /*bSearchable*/ false);

	Menu.AddMenuEntry(
		bVisible ? LOCTEXT("Hide", "Hide") : LOCTEXT("Show", "Show"),
		LOCTEXT("HideBodyTip", "Show or hide this body in the 3D view"),
		FSlateIcon(),
		FUIAction(FExecuteAction::CreateLambda([Entity, Body, bMirror, bVisible]()
		{
			if (!Entity.IsValid()) return;
			if (bMirror) Entity->SetMirrorBodyVisible(Body, !bVisible);
			else Entity->SetBodyVisible(Body, !bVisible);
		})));

	if (!bMirror)
	{
		// What the body is made of and built with -- its own, per body; a
		// mirrored half shares the body's -- and its generation.
		if (Entity->GetBodies().IsValidIndex(Body))
		{
			const FString Role = Entity->GetBodies()[Body].Name;
			Menu.AddMenuEntry(LOCTEXT("BodyStructure", "Structure..."),
				LOCTEXT("BodyStructureTip", "What this body is made of and built with: its material and skin thickness, networks of cutting planes (ribs, spars, frames) and their materials, or an asserted mass. Its mirrored half shares it. Analysis > Mass Properties weighs by it; Analysis > Structural analyses with it."),
				FSlateIcon(),
				FUIAction(FExecuteAction::CreateLambda([Entity, Role]() { if (Entity.IsValid()) OpenAerotelicStructureDialog(Entity.Get(), Role); })));
			const bool bGenerated = Entity->GetGeneratedStructure(Role) != nullptr;
			Menu.AddMenuEntry(bGenerated ? LOCTEXT("RegenerateStructure", "Regenerate Structure") : LOCTEXT("GenerateStructure", "Generate Structure"),
				LOCTEXT("GenerateBodyStructureTip", "Build this body's skin shell and ribs from its structure (seconds of MeshLib work): they appear as layers under the body, and its mirrored half's, and follow every later edit of the body."),
				FSlateIcon(),
				FUIAction(FExecuteAction::CreateLambda([this, Entity, Role]() { if (Entity.IsValid()) GenerateStructure(Entity.Get(), Role); })));
			if (bGenerated)
			{
				Menu.AddMenuEntry(LOCTEXT("ClearStructure", "Clear Generated Structure"),
					LOCTEXT("ClearStructureTip", "Drop the generated layers (the structure's description stays; Generate rebuilds them)."),
					FSlateIcon(),
					FUIAction(FExecuteAction::CreateLambda([Entity, Role]() { if (Entity.IsValid()) Entity->ClearGeneratedStructure(Role); })));
			}
		}

		Menu.AddMenuEntry(LOCTEXT("Color", "Color"), LOCTEXT("BodyColorTip", "Pick this body's colour. Only this body changes -- not its siblings; its mirror follows it."), FSlateIcon(),
			FUIAction(FExecuteAction::CreateLambda([this, Entity, Body]() { if (Entity.IsValid()) PickBodyColor(Entity.Get(), Body); })));

		Menu.AddMenuEntry(LOCTEXT("EntityColor", "Entity's Color"), LOCTEXT("EntityColorTip", "Drop this body's own colour and take the entity's"), FSlateIcon(),
			FUIAction(FExecuteAction::CreateLambda([Entity, Body]() { if (Entity.IsValid()) Entity->SetBodyColor(Body, TOptional<FLinearColor>()); }),
				FCanExecuteAction::CreateLambda([Entity, Body]()
				{
					return Entity.IsValid() && Entity->GetBodies().IsValidIndex(Body) && Entity->GetBodies()[Body].bHasColor;
				})));
	}

	return Menu.MakeWidget();
}

TSharedPtr<SWidget> SAerotelicSceneTree::MakeLayerMenu(FItem Item)
{
	// A generated layer's menu: shown or hidden (on this half), its colour.
	using EKind = FAerotelicSceneRow::EKind;
	TWeakObjectPtr<AAerotelicEntityActor> Entity = Item->Entity;
	const int32 Layer = Item->LayerIndex;
	const bool bMirror = Item->Kind == EKind::MirrorLayer;
	const bool bVisible = bMirror ? Entity->IsMirrorLayerVisible(Layer) : Entity->IsLayerVisible(Layer);

	FMenuBuilder Menu(true, nullptr, nullptr, false, &FCoreStyle::Get(), /*bSearchable*/ false);
	Menu.AddMenuEntry(
		bVisible ? LOCTEXT("Hide", "Hide") : LOCTEXT("Show", "Show"),
		LOCTEXT("HideLayerTip", "Show or hide this layer in the 3D view (the body's own surface shows again while its skin layer is hidden)"),
		FSlateIcon(),
		FUIAction(FExecuteAction::CreateLambda([Entity, Layer, bMirror, bVisible]()
		{
			if (!Entity.IsValid()) return;
			if (bMirror) Entity->SetMirrorLayerVisible(Layer, !bVisible);
			else Entity->SetLayerVisible(Layer, !bVisible);
		})));
	if (!bMirror)
	{
		Menu.AddMenuEntry(LOCTEXT("Color", "Color"), LOCTEXT("LayerColorTip", "Pick this layer's colour (its mirror follows it)."), FSlateIcon(),
			FUIAction(FExecuteAction::CreateLambda([this, Entity, Layer]() { if (Entity.IsValid()) PickLayerColor(Entity.Get(), Layer); })));
		Menu.AddMenuEntry(LOCTEXT("LayerDefaultColor", "Default Color"), LOCTEXT("LayerDefaultColorTip", "Back to the layer kind's colour: the skin the body's, ribs steel blue"), FSlateIcon(),
			FUIAction(FExecuteAction::CreateLambda([Entity, Layer]() { if (Entity.IsValid()) Entity->SetLayerColor(Layer, TOptional<FLinearColor>()); }),
				FCanExecuteAction::CreateLambda([Entity, Layer]() { return Entity.IsValid() && Entity->GetLayers().IsValidIndex(Layer) && Entity->GetLayers()[Layer].bHasColor; })));
	}
	return Menu.MakeWidget();
}

// ---------------------------------------------------------------- actions

void SAerotelicSceneTree::GenerateStructure(AAerotelicEntityActor* Entity, const FString& Role)
{
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	if (Doc == nullptr || Entity == nullptr) return;
	const FString Label = Entity->GetBodies().Num() > 1 ? FString::Printf(TEXT("%s / %s"), *Entity->EntityName, *Role) : Entity->EntityName;
	Status(FText::Format(LOCTEXT("StatusGenerating", "Generating the structure of {0}..."), FText::FromString(Label)));
	FString Error;
	if (Doc->GenerateBodyStructure(Entity, Role, Error))
	{
		const FAerotelicBodyStructure* G = Entity->GetGeneratedStructure(Role);
		Status(FText::Format(LOCTEXT("StatusGenerated", "Generated the structure of {0}: {1} layer(s), {2} triangles in {3} s"), FText::FromString(Label),
			FText::AsNumber(G != nullptr ? static_cast<int32>(G->GetLayers().size()) : 0), FText::AsNumber(G != nullptr ? G->GetStats().Triangles : 0),
			FText::AsNumber(G != nullptr ? FMath::RoundToInt(G->GetStats().Seconds) : 0)));
	}
	else
	{
		Status(LOCTEXT("Ready", "Ready"));
		AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(Error), LOCTEXT("GenerateTitle", "Generate Structure"));
	}
}

void SAerotelicSceneTree::PickLayerColor(AAerotelicEntityActor* Entity, int32 LayerIndex)
{
	TWeakObjectPtr<AAerotelicEntityActor> Weak = Entity;
	PickColor(FText::Format(LOCTEXT("LayerColorTitle", "Color -- {0}"), FText::FromString(Entity->GetLayerDisplayName(LayerIndex))),
		Entity->GetLayerDisplayColor(LayerIndex), AsShared(),
		[Weak, LayerIndex](FLinearColor C) { if (Weak.IsValid()) Weak->SetLayerColor(LayerIndex, C); });
	Status(FText::Format(LOCTEXT("StatusLayerColor", "Choosing a colour for {0}"), FText::FromString(Entity->GetLayerDisplayName(LayerIndex))));
}

void SAerotelicSceneTree::PickEntityColor(AAerotelicEntityActor* Entity)
{
	TWeakObjectPtr<AAerotelicEntityActor> Weak = Entity;
	// Opened on what is on screen, default included, as the Qt app does.
	PickColor(FText::Format(LOCTEXT("ColorTitle", "Color -- {0}"), FText::FromString(Entity->EntityName)),
		Entity->GetDisplayColor(), AsShared(),
		[Weak](FLinearColor C) { if (Weak.IsValid()) Weak->SetColorOverride(C); });
	Status(FText::Format(LOCTEXT("StatusColor", "Choosing a colour for {0}"), FText::FromString(Entity->EntityName)));
}

void SAerotelicSceneTree::PickBodyColor(AAerotelicEntityActor* Entity, int32 BodyIndex)
{
	TWeakObjectPtr<AAerotelicEntityActor> Weak = Entity;
	PickColor(FText::Format(LOCTEXT("BodyColorTitle", "Color -- {0}"), FText::FromString(Entity->GetBodyDisplayName(BodyIndex))),
		Entity->GetBodyDisplayColor(BodyIndex), AsShared(),
		[Weak, BodyIndex](FLinearColor C) { if (Weak.IsValid()) Weak->SetBodyColor(BodyIndex, C); });
	Status(FText::Format(LOCTEXT("StatusBodyColor", "Choosing a colour for {0}"), FText::FromString(Entity->GetBodyDisplayName(BodyIndex))));
}

void SAerotelicSceneTree::RenameEntity(AAerotelicEntityActor* Entity)
{
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	if (Doc == nullptr || Entity == nullptr)
	{
		return;
	}
	TOptional<FString> NewName = PromptForString(
		LOCTEXT("RenameTitle", "Rename"),
		FText::Format(LOCTEXT("RenameLabel", "New name for {0}:"), FText::FromString(Entity->EntityName)),
		Entity->EntityName);
	if (!NewName.IsSet())
	{
		return;
	}
	FString Name = NewName.GetValue().TrimStartAndEnd();
	if (Name.IsEmpty() || Name == Entity->EntityName)
	{
		return;
	}
	// Names are what the project file refers to entities by, so they must
	// be unique; a collision is refused rather than silently suffixed.
	for (const TWeakObjectPtr<AAerotelicEntityActor>& Other : Doc->GetEntities())
	{
		if (Other.IsValid() && Other.Get() != Entity && Other->EntityName == Name)
		{
			AerotelicMessageBox(EAppMsgType::Ok,
				FText::Format(LOCTEXT("NameTaken", "There is already an entity named \"{0}\"."), FText::FromString(Name)));
			return;
		}
	}
	const FString Old = Entity->EntityName;
	Entity->EntityName = Name;
	Status(FText::Format(LOCTEXT("StatusRenamed", "Renamed {0} to {1}"), FText::FromString(Old), FText::FromString(Name)));
	if (Tree.IsValid())
	{
		Tree->RequestTreeRefresh();
	}
}

void SAerotelicSceneTree::DeleteEntity(AAerotelicEntityActor* Entity)
{
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	if (Doc == nullptr || Entity == nullptr)
	{
		return;
	}
	const FString Name = Entity->EntityName;
	// Boolean results built on this go with it; say so.
	TArray<FString> Dependents;
	for (const TWeakObjectPtr<AAerotelicEntityActor>& Other : Doc->GetEntities())
	{
		if (Other.IsValid() && Other.Get() != Entity && UAerotelicDocumentSubsystem::DependsOn(Other.Get(), Entity))
		{
			Dependents.Add(Other->EntityName);
		}
	}
	const FText Question = Dependents.Num() == 0
		? FText::Format(LOCTEXT("ConfirmDelete", "Delete {0}?"), FText::FromString(Name))
		: FText::Format(LOCTEXT("ConfirmDeleteCascade", "Delete {0}?\nThe boolean operation(s) built on it go with it: {1}"), FText::FromString(Name), FText::FromString(FString::Join(Dependents, TEXT(", "))));
	const EAppReturnType::Type Answer = AerotelicMessageBox(EAppMsgType::YesNo, Question);
	if (Answer != EAppReturnType::Yes)
	{
		return;
	}
	Doc->RemoveEntity(Entity);
	Status(FText::Format(LOCTEXT("StatusDeleted", "Deleted {0}"), FText::FromString(Name)));
}

#undef LOCTEXT_NAMESPACE
