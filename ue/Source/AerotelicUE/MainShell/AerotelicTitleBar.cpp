#include "AerotelicTitleBar.h"

#include "Widgets/SWindow.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SWindowTitleBarArea.h"
#include "Widgets/Text/STextBlock.h"
#include "Styling/CoreStyle.h"

#include "Branding/AerotelicBranding.h"
#include "Widgets/AerotelicDialogWindow.h"

namespace
{
	constexpr float kTitleBarHeight = 32.0f;
	// The Qt theme's titlebar_bg / titlebar_text.
	const FLinearColor kBarBg(FColor::FromHex(TEXT("#0b0d10")));
	const FLinearColor kBarText(FColor::FromHex(TEXT("#bbbbbb")));
}

TSharedRef<SWidget> MakeAerotelicTitleBar()
{
	TSharedPtr<SWindow> MainWindow = FindAerotelicMainWindow();
	const bool bFrameless = MainWindow.IsValid() && !MainWindow->HasOSWindowBorder();

	// The background and the label are hit-test invisible so the cursor
	// over them hits the area itself, whose window zone is TitleBar (the
	// OS then moves the window); the buttons the area overlays on the
	// right stay hit-testable and get their clicks.
	TSharedRef<SWindowTitleBarArea> Area = SNew(SWindowTitleBarArea)
		.Visibility(bFrameless ? EVisibility::Visible : EVisibility::Collapsed)
		[
			SNew(SBorder)
			.Visibility(EVisibility::HitTestInvisible)
			.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
			.BorderBackgroundColor(kBarBg)
			.Padding(FMargin(10, 0, 4, 0))
			[
				SNew(SBox).HeightOverride(kTitleBarHeight).VAlign(VAlign_Center)
				[
					SNew(STextBlock)
					.Text(FText::FromString(FString::Printf(TEXT("%s — %s"), AerotelicBranding::Product, AerotelicBranding::Tagline)))
					.Font(FCoreStyle::GetDefaultFontStyle("Bold", 11))
					.ColorAndOpacity(FSlateColor(kBarText))
				]
			]
		];
	Area->SetGameWindow(MainWindow);
	Area->SetWindowButtonsVisibility(bFrameless);
	return Area;
}
