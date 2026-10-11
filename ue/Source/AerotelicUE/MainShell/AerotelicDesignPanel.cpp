#include "AerotelicDesignPanel.h"
#include "Branding/AerotelicTheme.h"

#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Text/STextBlock.h"
#include "Styling/CoreStyle.h"

#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"

#define LOCTEXT_NAMESPACE "AerotelicDesignPanel"

void SAerotelicDesignPanel::Construct(const FArguments& InArgs)
{
	ChildSlot
	[
		SAssignNew(Host, SBox)
	];

	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		SelectionHandle = Doc->OnSelectionChanged.AddSP(this, &SAerotelicDesignPanel::Refresh);
		ListHandle = Doc->OnEntityListChanged.AddSP(this, &SAerotelicDesignPanel::Refresh);
	}
	Refresh();
}

SAerotelicDesignPanel::~SAerotelicDesignPanel()
{
	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		Doc->OnSelectionChanged.Remove(SelectionHandle);
		Doc->OnEntityListChanged.Remove(ListHandle);
	}
}

void SAerotelicDesignPanel::PruneDead()
{
	for (auto It = Panels.CreateIterator(); It; ++It)
	{
		if (!It->Key.IsValid())
		{
			Composed.Remove(It->Key);
			It.RemoveCurrent();
		}
	}
}

void SAerotelicDesignPanel::Refresh()
{
	PruneDead();

	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	AAerotelicEntityActor* Entity = Doc ? Doc->GetSelectedEntity() : nullptr;

	if (Entity == nullptr)
	{
		Shown.Reset();
		Host->SetContent(
			SNew(SBorder)
			.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
			.BorderBackgroundColor_Lambda([]() { return FSlateColor(AerotelicTheme::Current().WinBg); })
			.Padding(FMargin(12))
			[
				SNew(STextBlock)
				.AutoWrapText(true)
				.ColorAndOpacity_Lambda([]() { return FSlateColor(AerotelicTheme::Current().TextMuted); })
				.Text(LOCTEXT("NothingSelected", "No entity selected.\n\nClick an entity in the 3D view or in the Scene tree, or add one from the toolbar, and its design panel appears here."))
			]);
		return;
	}

	if (Shown == Entity && Composed.Contains(Entity))
	{
		return;
	}

	TSharedPtr<SWidget>* Existing = Composed.Find(Entity);
	if (Existing == nullptr)
	{
		FAerotelicDesignContent Content = MakeEntityDesignContent(Entity);
		if (!Content.Widget.IsValid())
		{
			Content.Widget = SNew(STextBlock).Text(LOCTEXT("NoPanel", "This entity type has no design panel."));
		}
		TWeakObjectPtr<AAerotelicEntityActor> Weak = Entity;
		TSharedRef<SWidget> Panel =
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight().Padding(8, 6, 8, 2)
			[
				SNew(STextBlock)
				.Font(FCoreStyle::GetDefaultFontStyle("Bold", 11))
				// Live, so a Rename in the scene tree shows here at once.
				.Text_Lambda([Weak, Type = Content.Title]()
				{
					if (!Weak.IsValid()) return FText::GetEmpty();
					FString Kind = Type.ToString();
					int32 Sep = INDEX_NONE;
					if (Kind.FindChar(TEXT('-'), Sep)) { Kind.LeftInline(Sep); Kind.TrimEndInline(); }
					return FText::FromString(FString::Printf(TEXT("%s -- %s"), *Kind, *Weak->EntityName));
				})
			]
			+ SVerticalBox::Slot().FillHeight(1.0f)
			[
				Content.Widget.ToSharedRef()
			];
		Panels.Add(Entity, Content);
		Existing = &Composed.Add(Entity, Panel);
	}

	Shown = Entity;
	Host->SetContent(Existing->ToSharedRef());
}

#undef LOCTEXT_NAMESPACE
