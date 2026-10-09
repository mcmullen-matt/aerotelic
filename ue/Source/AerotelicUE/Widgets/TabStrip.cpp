#include "TabStrip.h"

#include "Widgets/Layout/SWidgetSwitcher.h"
#include "Widgets/Layout/SWrapBox.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Layout/SSeparator.h"

void SAerotelicTabStrip::Construct(const FArguments& InArgs, TArray<FAerotelicTab> InTabs)
{
	// Headers wrap onto a second line when the strip is narrower than its
	// tabs (the propeller's seven tabs in the dock column).
	TSharedRef<SWrapBox> Headers = SNew(SWrapBox).UseAllottedSize(true).InnerSlotPadding(FVector2D(2, 2));
	for (int32 i = 0; i < InTabs.Num(); ++i)
	{
		Headers->AddSlot()
		[
			SNew(SButton)
			.Text(InTabs[i].Label)
			// The active tab's header is tinted so a six-tab strip (the
			// propeller's) reads at a glance, as QTabWidget's selected tab does.
			.ButtonColorAndOpacity_Lambda([this, i]()
			{
				return Switcher.IsValid() && Switcher->GetActiveWidgetIndex() == i
					? FLinearColor(0.3f, 0.5f, 0.8f) : FLinearColor::White;
			})
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
