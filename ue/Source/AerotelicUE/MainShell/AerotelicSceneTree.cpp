#include "AerotelicSceneTree.h"
#include "AerotelicDesignDock.h"
#include "AerotelicTransformDialog.h"

#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Views/STableRow.h"
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
	RefreshItems();
}

SAerotelicSceneTree::~SAerotelicSceneTree()
{
	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		Doc->OnEntityListChanged.Remove(ListChangedHandle);
		Doc->OnSelectionChanged.Remove(SelectionChangedHandle);
	}
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
	return SNew(STableRow<FItem>, OwnerTable)
		.Padding(FMargin(2, 1))
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 4, 0)
			[
				SNew(SCheckBox)
				.ToolTipText(LOCTEXT("VisibleTip", "Show or hide this entity in the 3D view"))
				.IsChecked_Lambda([Item]()
				{
					return Item.IsValid() && Item->IsEntityVisible() ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
				})
				.OnCheckStateChanged_Lambda([Item](ECheckBoxState State)
				{
					if (Item.IsValid())
					{
						Item->SetEntityVisible(State == ECheckBoxState::Checked);
					}
				})
			]
			+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
			[
				SNew(STextBlock)
				.Text_Lambda([Item]() { return Item.IsValid() ? FText::FromString(Item->EntityName) : FText::GetEmpty(); })
			]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(6, 0, 2, 0)
			[
				SNew(STextBlock)
				.ColorAndOpacity(FSlateColor(FLinearColor(0.55f, 0.55f, 0.55f)))
				.Text_Lambda([Item]() { return Item.IsValid() ? EntityTypeLabel(Item->GetEntityType()) : FText::GetEmpty(); })
			]
		];
}

void SAerotelicSceneTree::GetChildren(FItem Item, TArray<FItem>& OutChildren)
{
	// No body rows yet: each entity is one mesh.
}

void SAerotelicSceneTree::OnSelectionChanged(FItem Item, ESelectInfo::Type SelectInfo)
{
	if (bSyncingSelection)
	{
		return;
	}
	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		Doc->SetSelectedEntity(Item.IsValid() ? Item.Get() : nullptr);
	}
}

void SAerotelicSceneTree::OnDoubleClick(FItem Item)
{
	if (Item.IsValid())
	{
		EditEntity(Item.Get());
		Status(FText::Format(LOCTEXT("StatusEditing", "Editing {0}"), FText::FromString(Item->EntityName)));
	}
}

void SAerotelicSceneTree::RefreshItems()
{
	Items.Reset();
	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		for (const FItem& E : Doc->GetEntities())
		{
			if (E.IsValid())
			{
				Items.Add(E);
			}
		}
	}
	if (Tree.IsValid())
	{
		Tree->RequestTreeRefresh();
	}
	SyncSelectionFromDocument();
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
	for (const FItem& Item : Items)
	{
		if (Item.Get() == Selected)
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
	UE_LOG(LogTemp, Verbose, TEXT("SceneTree: context menu requested, %d selected"), Selected.Num());
	if (Selected.Num() == 0 || !Selected[0].IsValid())
	{
		return nullptr;
	}
	FItem Item = Selected[0];
	AAerotelicEntityActor* Entity = Item.Get();
	const bool bVisible = Entity->IsEntityVisible();

	FMenuBuilder Menu(true, nullptr, nullptr, false, &FCoreStyle::Get(), /*bSearchable*/ false);

	Menu.AddMenuEntry(
		bVisible ? LOCTEXT("Hide", "Hide") : LOCTEXT("Show", "Show"),
		LOCTEXT("HideTip", "Show or hide this entity in the 3D view"),
		FSlateIcon(),
		FUIAction(FExecuteAction::CreateLambda([Item, bVisible]()
		{
			if (Item.IsValid()) Item->SetEntityVisible(!bVisible);
		})));

	Menu.AddMenuEntry(LOCTEXT("Edit", "Edit"), LOCTEXT("EditTip", "Open this entity's design panel"), FSlateIcon(),
		FUIAction(FExecuteAction::CreateLambda([this, Item]()
		{
			if (Item.IsValid())
			{
				EditEntity(Item.Get());
				Status(FText::Format(LOCTEXT("StatusEditing", "Editing {0}"), FText::FromString(Item->EntityName)));
			}
		})));

	Menu.AddMenuEntry(LOCTEXT("Rename", "Rename"), LOCTEXT("RenameTip", "Give this entity a new, unique name"), FSlateIcon(),
		FUIAction(FExecuteAction::CreateLambda([this, Item]() { if (Item.IsValid()) RenameEntity(Item.Get()); })));

	Menu.AddMenuEntry(LOCTEXT("Transform", "Transform"), LOCTEXT("TransformTip", "Translate and rotate this entity"), FSlateIcon(),
		FUIAction(FExecuteAction::CreateLambda([Item]() { if (Item.IsValid()) OpenAerotelicTransformDialog(Item.Get()); })));

	Menu.AddMenuEntry(LOCTEXT("Color", "Color"), LOCTEXT("ColorTip", "Not available yet in this version"), FSlateIcon(),
		FUIAction(FExecuteAction(), FCanExecuteAction::CreateLambda([]() { return false; })));

	Menu.AddMenuSeparator();

	Menu.AddMenuEntry(LOCTEXT("Mirrored", "Mirrored"),
		LOCTEXT("MirroredTip", "Derive a port-side copy by reflecting about Y = 0. Not available yet in this version."),
		FSlateIcon(),
		FUIAction(FExecuteAction(), FCanExecuteAction::CreateLambda([]() { return false; }),
			FIsActionChecked::CreateLambda([]() { return false; })),
		NAME_None, EUserInterfaceActionType::ToggleButton);

	Menu.AddMenuSeparator();

	Menu.AddMenuEntry(LOCTEXT("Delete", "Delete"), LOCTEXT("DeleteTip", "Remove this entity from the scene"), FSlateIcon(),
		FUIAction(FExecuteAction::CreateLambda([this, Item]() { if (Item.IsValid()) DeleteEntity(Item.Get()); })));

	return Menu.MakeWidget();
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
	for (const FItem& Other : Doc->GetEntities())
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
