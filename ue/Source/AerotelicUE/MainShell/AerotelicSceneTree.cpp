#include "AerotelicSceneTree.h"
#include "AerotelicDesignDock.h"
#include "AerotelicTransformDialog.h"

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
		}
		return FText::GetEmpty();
	}

	const FLinearColor kDimText(0.55f, 0.55f, 0.55f);

	// Opens the engine's colour picker (AppFramework's SColorPicker, the
	// same one the Editor uses) on Initial, calling OnPicked live as the
	// user drags. Modal, as QColorDialog.getColor is: the window is always
	// in front and focused, and nothing else happens until OK or Cancel.
	// No alpha: WingDesigner's colours have none.
	void PickColor(const FText& Title, FLinearColor Initial, TSharedPtr<SWidget> Parent, TFunction<void(FLinearColor)> OnPicked)
	{
		FColorPickerArgs Args;
		Args.bIsModal = true;
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
			.BorderBackgroundColor(FLinearColor(0.03f, 0.03f, 0.035f, 1.0f))
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
	ShowAerotelicDesignTab();
}

// ------------------------------------------------------------------- rows

TSharedRef<ITableRow> SAerotelicSceneTree::GenerateRow(FItem Item, const TSharedRef<STableViewBase>& OwnerTable)
{
	using EKind = FAerotelicSceneRow::EKind;
	const EKind Kind = Item->Kind;
	const int32 Body = Item->BodyIndex;
	TWeakObjectPtr<AAerotelicEntityActor> Entity = Item->Entity;

	// The checkbox: the entity's, a body's, or a mirrored body's visibility.
	auto IsChecked = [Entity, Kind, Body]()
	{
		if (!Entity.IsValid()) return ECheckBoxState::Unchecked;
		switch (Kind)
		{
		case EKind::Entity: return Entity->IsEntityVisible() ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
		case EKind::Body: return Entity->IsBodyVisible(Body) ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
		case EKind::Mirror: return Entity->IsMirrorBodyVisible(Body) ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
		}
		return ECheckBoxState::Unchecked;
	};
	auto OnChecked = [Entity, Kind, Body](ECheckBoxState State)
	{
		if (!Entity.IsValid()) return;
		const bool bOn = State == ECheckBoxState::Checked;
		switch (Kind)
		{
		case EKind::Entity: Entity->SetEntityVisible(bOn); break;
		case EKind::Body: Entity->SetBodyVisible(Body, bOn); break;
		case EKind::Mirror: Entity->SetMirrorBodyVisible(Body, bOn); break;
		}
	};
	auto Name = [Entity, Kind, Body]()
	{
		if (!Entity.IsValid()) return FText::GetEmpty();
		switch (Kind)
		{
		case EKind::Entity: return FText::FromString(Entity->EntityName);
		case EKind::Body: return FText::FromString(Entity->GetBodyDisplayName(Body));
		case EKind::Mirror: return FText::FromString(Entity->GetMirrorBodyDisplayName(Body));
		}
		return FText::GetEmpty();
	};
	// The grey annotation: an entity's kind; a body's role (Main, Spinner,
	// Blade 2 ...); a mirror's role plus "mirror".
	auto Annotation = [Entity, Kind, Body]()
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
		}
		return FText::GetEmpty();
	};
	auto Swatch = [Entity, Kind, Body]()
	{
		if (!Entity.IsValid()) return FLinearColor::Transparent;
		return Kind == EKind::Entity ? Entity->GetDisplayColor() : Entity->GetBodyDisplayColor(Body);
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
	for (const FItem& Old : Items)
	{
		if (Old->Entity.IsValid() && Tree.IsValid() && Tree->IsItemExpanded(Old))
		{
			WasExpanded.Add(Old->Entity.Get());
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
			for (int32 i = 0; i < NumBodies; ++i)
			{
				FItem Body = MakeShared<FAerotelicSceneRow>();
				Body->Kind = EKind::Body;
				Body->Entity = E;
				Body->BodyIndex = i;
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
		// body; with more it opens, and after that the user's state holds.
		for (const FItem& Row : Items)
		{
			const bool bExpand = WasExpanded.Contains(Row->Entity.Get()) || (bFirstFill || !WasExpanded.Contains(Row->Entity.Get())) && Row->Children.Num() > 1;
			Tree->SetItemExpansion(Row, bExpand);
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
	return Selected[0]->Kind == FAerotelicSceneRow::EKind::Entity ? MakeEntityMenu(Selected[0]) : MakeBodyMenu(Selected[0]);
}

TSharedPtr<SWidget> SAerotelicSceneTree::MakeEntityMenu(FItem Item)
{
	TWeakObjectPtr<AAerotelicEntityActor> Entity = Item->Entity;
	const bool bVisible = Entity->IsEntityVisible();

	FMenuBuilder Menu(true, nullptr, nullptr, false, &FCoreStyle::Get(), /*bSearchable*/ false);

	Menu.AddMenuEntry(
		bVisible ? LOCTEXT("Hide", "Hide") : LOCTEXT("Show", "Show"),
		LOCTEXT("HideTip", "Show or hide this entity in the 3D view"),
		FSlateIcon(),
		FUIAction(FExecuteAction::CreateLambda([Entity, bVisible]()
		{
			if (Entity.IsValid()) Entity->SetEntityVisible(!bVisible);
		})));

	Menu.AddMenuEntry(LOCTEXT("Edit", "Edit"), LOCTEXT("EditTip", "Open this entity's design panel"), FSlateIcon(),
		FUIAction(FExecuteAction::CreateLambda([this, Entity]()
		{
			if (Entity.IsValid())
			{
				EditEntity(Entity.Get());
				Status(FText::Format(LOCTEXT("StatusEditing", "Editing {0}"), FText::FromString(Entity->EntityName)));
			}
		})));

	Menu.AddMenuEntry(LOCTEXT("Rename", "Rename"), LOCTEXT("RenameTip", "Give this entity a new, unique name"), FSlateIcon(),
		FUIAction(FExecuteAction::CreateLambda([this, Entity]() { if (Entity.IsValid()) RenameEntity(Entity.Get()); })));

	Menu.AddMenuEntry(LOCTEXT("Transform", "Transform"), LOCTEXT("TransformTip", "Translate and rotate this entity"), FSlateIcon(),
		FUIAction(FExecuteAction::CreateLambda([Entity]() { if (Entity.IsValid()) OpenAerotelicTransformDialog(Entity.Get()); })));

	Menu.AddMenuEntry(LOCTEXT("Color", "Color"), LOCTEXT("ColorTip", "Pick the colour every body of this entity is drawn in (the mirrored ones included)"), FSlateIcon(),
		FUIAction(FExecuteAction::CreateLambda([this, Entity]() { if (Entity.IsValid()) PickEntityColor(Entity.Get()); })));

	Menu.AddMenuEntry(LOCTEXT("DefaultColor", "Default Color"), LOCTEXT("DefaultColorTip", "Back to the colour this kind of entity is drawn in by default"), FSlateIcon(),
		FUIAction(FExecuteAction::CreateLambda([Entity]() { if (Entity.IsValid()) Entity->SetColorOverride(TOptional<FLinearColor>()); }),
			FCanExecuteAction::CreateLambda([Entity]() { return Entity.IsValid() && Entity->GetColorOverride().IsSet(); })));

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

// ---------------------------------------------------------------- actions

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
			FMessageDialog::Open(EAppMsgType::Ok,
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
	const EAppReturnType::Type Answer = FMessageDialog::Open(EAppMsgType::YesNo,
		FText::Format(LOCTEXT("ConfirmDelete", "Delete {0}?"), FText::FromString(Name)));
	if (Answer != EAppReturnType::Yes)
	{
		return;
	}
	Doc->RemoveEntity(Entity);
	Status(FText::Format(LOCTEXT("StatusDeleted", "Deleted {0}"), FText::FromString(Name)));
}

#undef LOCTEXT_NAMESPACE
