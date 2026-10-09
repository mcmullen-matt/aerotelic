#include "TabStrip.h"

#include "Widgets/Layout/SWidgetSwitcher.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Layout/SSeparator.h"

void SAerotelicTabStrip::Construct(const FArguments& InArgs, TArray<FAerotelicTab> InTabs)
{
	TSharedRef<SHorizontalBox> Headers = SNew(SHorizontalBox);
	for (int32 i = 0; i < InTabs.Num(); ++i)
	{
		Headers->AddSlot().AutoWidth().Padding(0, 0, 2, 0)
		[
			SNew(SButton)
			.Text(InTabs[i].Label)
			.OnClicked_Lambda([this, i]() { Switcher->SetActiveWidgetIndex(i); return FReply::Handled(); })
		];
	}

	TSharedRef<SWidgetSwitcher> SwitcherRef = SNew(SWidgetSwitcher);
	for (const FAerotelicTab& Tab : InTabs)
	{
		SwitcherRef->AddSlot() [ Tab.Content ];
	}
	Switcher = SwitcherRef;

	ChildSlot
	[
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 4) [ Headers ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6) [ SNew(SSeparator) ]
		+ SVerticalBox::Slot().FillHeight(1.0f) [ SwitcherRef ]
	];
}
