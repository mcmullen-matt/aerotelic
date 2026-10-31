#include "AerotelicDesignPanel.h"
#include "Branding/AerotelicTheme.h"
#include "Widgets/MessageBox.h"

#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Styling/CoreStyle.h"
#include "Containers/Ticker.h"

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

void SAerotelicDesignPanel::ApplyDraft(TWeakObjectPtr<AAerotelicEntityActor> Entity)
{
	FAerotelicDesignContent* Content = Panels.Find(Entity);
	if (!Entity.IsValid() || Content == nullptr || !Content->Draft.IsValid() || Content->Draft->Model == nullptr)
	{
		return;
	}
	UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
	FString Error;
	if (Doc == nullptr || !Doc->ApplyEntityModel(Entity.Get(), *Content->Draft->Model, Error))
	{
		// The draft stays as typed, still marked, so it can be fixed and
		// applied again; the scene keeps the entity as it was.
		AerotelicMessageBox(EAppMsgType::Ok,
			FText::FromString(FString::Printf(TEXT("'%s' cannot be built as drafted, so the scene keeps it as it was:\n\n%s"), *Entity->EntityName, *Error)),
			LOCTEXT("ApplyRefused", "Cannot Apply"));
		return;
	}
	Content->Draft->bDirty = false;
}

void SAerotelicDesignPanel::RevertDraft(TWeakObjectPtr<AAerotelicEntityActor> Entity)
{
	// The panel is rebuilt on a fresh copy of the entity's model: its
	// widgets point into the draft, so the draft cannot be replaced under
	// them. Done on the next tick, not from inside the Revert button's own
	// click: dropping the panel here would destroy that button while Slate
	// is still inside its mouse-up.
	FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateSPLambda(this, [this, Entity](float)
	{
		Host->SetContent(SNullWidget::NullWidget);
		Composed.Remove(Entity);
		Panels.Remove(Entity);
		Shown.Reset();
		Refresh();
		return false;
	}));
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
		TSharedRef<SVerticalBox> Panel =
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

		// The footer: what the panel edits is a draft, and these two are
		// the only way it reaches (or stops short of) the scene.
		if (Content.Draft.IsValid())
		{
			TWeakPtr<FAerotelicDesignDraft> WeakDraft = Content.Draft;
			auto IsDirty = [WeakDraft]()
			{
				TSharedPtr<FAerotelicDesignDraft> D = WeakDraft.Pin();
				return D.IsValid() && D->bDirty;
			};
			Panel->AddSlot().AutoHeight().Padding(8, 4, 8, 6)
			[
				SNew(SBorder)
				.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
				.BorderBackgroundColor_Lambda([]() { return FSlateColor(AerotelicTheme::Current().WinBg); })
				.Padding(FMargin(0, 6, 0, 0))
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center).Padding(0, 0, 8, 0)
					[
						SNew(STextBlock)
						.AutoWrapText(true)
						.ColorAndOpacity_Lambda([]() { return FSlateColor(AerotelicTheme::Current().TextMuted); })
						.Text_Lambda([IsDirty]()
						{
							return IsDirty()
								? LOCTEXT("Edited", "Edited here only -- press Apply to change the entity in the scene.")
								: LOCTEXT("Clean", "As in the scene.");
						})
					]
					+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 6, 0)
					[
						SNew(SButton)
						.Text(LOCTEXT("Revert", "Revert"))
						.ToolTipText(LOCTEXT("RevertTip", "Throw away the edits made here since the last Apply and reload the panel from the entity as it is in the scene."))
						.IsEnabled_Lambda(IsDirty)
						.OnClicked_Lambda([this, Weak]() { RevertDraft(Weak); return FReply::Handled(); })
					]
					+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
					[
						SNew(SButton)
						.Text(LOCTEXT("Apply", "Apply"))
						.ToolTipText(LOCTEXT("ApplyTip", "Make these edits the entity's: the viewport rebuilds it, every boolean over it rebuilds, and every lattice over it regenerates."))
						.IsEnabled_Lambda(IsDirty)
						.OnClicked_Lambda([this, Weak]() { ApplyDraft(Weak); return FReply::Handled(); })
					]
				]
			];
		}

		Panels.Add(Entity, Content);
		Existing = &Composed.Add(Entity, Panel);
	}

	Shown = Entity;
	Host->SetContent(Existing->ToSharedRef());
}

#undef LOCTEXT_NAMESPACE
