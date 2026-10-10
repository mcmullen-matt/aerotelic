#include "AerotelicWelcomeScreen.h"
#include "AerotelicTutorialDialog.h"
#include "Branding/AerotelicBranding.h"
#include "Branding/AerotelicSettings.h"
#include "Licensing/AerotelicLicenseInfo.h"
#include "Widgets/AerotelicDialogWindow.h"

#include "Engine/Texture2D.h"
#include "ImageUtils.h"
#include "Misc/Paths.h"
#include "Framework/Application/SlateApplication.h"
#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SGridPanel.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Layout/SSeparator.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SHyperlink.h"
#include "Styling/CoreStyle.h"
#include "UObject/StrongObjectPtr.h"

#define LOCTEXT_NAMESPACE "AerotelicWelcomeScreen"

namespace
{
	constexpr float kImageW = 720.0f;
	constexpr float kBannerH = 300.0f;
	constexpr float kBannerHMin = 140.0f;
	constexpr float kBodyH = 480.0f;

	// The Dark Aerospace palette of ui/themes.py.
	const FLinearColor kPanelBg(FColor::FromHex(TEXT("#161b22")));
	const FLinearColor kWinBg(FColor::FromHex(TEXT("#0d1117")));
	const FLinearColor kBorderDim(FColor::FromHex(TEXT("#21262d")));
	const FLinearColor kMuted(FColor::FromHex(TEXT("#8b949e")));
	const FLinearColor kAccent(FColor::FromHex(TEXT("#58a6ff")));
	const FLinearColor kWarn(FColor::FromHex(TEXT("#d8a24a")));
	const FLinearColor kExpired(FColor::FromHex(TEXT("#d86a5a")));

	// The artwork, loaded once and kept.
	TStrongObjectPtr<UTexture2D> GBannerTexture;
	FSlateBrush GBannerBrush;

	TSharedRef<SWidget> Rule()
	{
		return SNew(SBox).HeightOverride(1)
		[ SNew(SBorder).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(kBorderDim).Padding(0) ];
	}

	TSharedRef<SWidget> Banner(float BannerH)
	{
		if (!GBannerTexture.IsValid())
		{
			const FString Path = FPaths::ProjectDir() / AerotelicBranding::SplashImage;
			if (UTexture2D* Tex = FImageUtils::ImportFileAsTexture2D(Path))
			{
				GBannerTexture.Reset(Tex);
				GBannerBrush.SetResourceObject(Tex);
				GBannerBrush.ImageSize = FVector2f((float)Tex->GetSizeX(), (float)Tex->GetSizeY());
				GBannerBrush.DrawAs = ESlateBrushDrawType::Image;
			}
		}
		if (!GBannerTexture.IsValid())
		{
			// Missing artwork: the product name, left-aligned, in a 90 px band.
			return SNew(SBox).HeightOverride(90).VAlign(VAlign_Center)
			[
				SNew(SBorder).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(kWinBg).Padding(FMargin(12, 0))
				[ SNew(STextBlock).Font(FCoreStyle::GetDefaultFontStyle("Bold", 24)).Text(FText::FromString(FString::Printf(TEXT("  %s"), AerotelicBranding::Product))) ]
			];
		}
		// Fitted into the band, never upscaled, centred.
		const FVector2f Src = GBannerBrush.ImageSize;
		const float Scale = FMath::Min(1.0f, FMath::Min(kImageW / Src.X, BannerH / Src.Y));
		return SNew(SBox).HeightOverride(BannerH).HAlign(HAlign_Center).VAlign(VAlign_Center)
		[
			SNew(SBorder).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(kWinBg).Padding(0)
			[ SNew(SImage).Image(&GBannerBrush).DesiredSizeOverride(FVector2D(Src.X * Scale, Src.Y * Scale)) ]
		];
	}

	TSharedRef<SWidget> Identity()
	{
		return SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight()
			[ SNew(STextBlock).Font(FCoreStyle::GetDefaultFontStyle("Bold", 22)).Text(FText::FromString(AerotelicBranding::Product)) ]
			+ SVerticalBox::Slot().AutoHeight()
			[
				SNew(STextBlock).Font(FCoreStyle::GetDefaultFontStyle("Regular", 11)).ColorAndOpacity(FSlateColor(kMuted))
				.Text(FText::FromString(FString::Printf(TEXT("%s  ·  Version %s"), AerotelicBranding::Tagline, *AerotelicBranding::Version())))
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth() [ SNew(STextBlock).Font(FCoreStyle::GetDefaultFontStyle("Regular", 12)).Text(LOCTEXT("DistributedBy", "Distributed by ")) ]
				+ SHorizontalBox::Slot().AutoWidth() [ SNew(STextBlock).Font(FCoreStyle::GetDefaultFontStyle("Bold", 12)).Text(FText::FromString(AerotelicBranding::Company)) ]
			];
	}

	TSharedRef<SWidget> LicenceBlock()
	{
		const FAerotelicLicenseInfo& Info = CurrentAerotelicLicenseInfo();
		TSharedRef<SGridPanel> Grid = SNew(SGridPanel);
		int32 Row = 0;
		for (const FAerotelicLicenseInfo::FRow& R : Info.AsRows())
		{
			FLinearColor Color = FLinearColor::White;
			if (R.bHighlight)
			{
				Color = Info.Severity == FAerotelicLicenseInfo::ESeverity::Expired ? kExpired
					: Info.Severity == FAerotelicLicenseInfo::ESeverity::Warn ? kWarn : kAccent;
			}
			Grid->AddSlot(0, Row).HAlign(HAlign_Right).Padding(0, 1.5f, 14, 1.5f)
			[ SNew(STextBlock).Font(FCoreStyle::GetDefaultFontStyle("Regular", 11)).ColorAndOpacity(FSlateColor(kMuted)).Text(R.Label) ];
			Grid->AddSlot(1, Row).HAlign(HAlign_Left).Padding(0, 1.5f)
			[ SNew(STextBlock).Font(FCoreStyle::GetDefaultFontStyle(R.bHighlight ? "Bold" : "Regular", 11)).ColorAndOpacity(FSlateColor(Color)).Text(R.Value).AutoWrapText(true) ];
			++Row;
		}
		return SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 3)
			[ SNew(STextBlock).Font(FCoreStyle::GetDefaultFontStyle("Bold", 9)).ColorAndOpacity(FSlateColor(kMuted)).Text(LOCTEXT("LicenceHeader", "L I C E N C E")) ]
			+ SVerticalBox::Slot().AutoHeight() [ Grid ];
	}
}

bool ShowAerotelicWelcomeScreen(bool bForce)
{
	if (!bForce && !AerotelicSettings::ShowSplashAtStartup())
	{
		return false;
	}

	// The banner band gives way to the body on a short screen.
	float ScreenH = 1080.0f;
	if (TSharedPtr<SWindow> Main = FindAerotelicMainWindow())
	{
		ScreenH = (float)Main->GetSizeInScreen().Y;
	}
	const float BannerH = FMath::Clamp(ScreenH * 0.88f - kBodyH, kBannerHMin, kBannerH);

	bool bWantsTutorials = false;
	bool bShowAtStartup = AerotelicSettings::ShowSplashAtStartup();

	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(FText::FromString(FString::Printf(TEXT("Welcome to %s"), AerotelicBranding::Product)))
		.CreateTitleBar(false)
		.SizingRule(ESizingRule::Autosized)
		.SupportsMaximize(false)
		.SupportsMinimize(false)
		.HasCloseButton(false);

	auto Close = [&]() { Window->RequestDestroyWindow(); };
	auto OpenTutorials = [&]() { bWantsTutorials = true; Window->RequestDestroyWindow(); };

	const FString Legal = AerotelicBranding::Copyright() + TEXT("\n\n") + AerotelicBranding::DisclaimerText();

	Window->SetContent(
		SNew(SBox).WidthOverride(kImageW)
		[
			SNew(SBorder)
			.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
			.BorderBackgroundColor(kPanelBg)
			.Padding(1)
			[
				SNew(SBorder).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(kBorderDim).Padding(1)
				[
					SNew(SBorder).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(kPanelBg).Padding(0)
					[
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight() [ Banner(BannerH) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(22, 16, 22, 14)
						[
							SNew(SVerticalBox)
							+ SVerticalBox::Slot().AutoHeight() [ Identity() ]
							+ SVerticalBox::Slot().AutoHeight().Padding(0, 12) [ Rule() ]
							+ SVerticalBox::Slot().AutoHeight() [ LicenceBlock() ]
							+ SVerticalBox::Slot().AutoHeight().Padding(0, 12) [ Rule() ]
							+ SVerticalBox::Slot().AutoHeight()
							[
								SNew(SVerticalBox)
								+ SVerticalBox::Slot().AutoHeight()
								[
									SNew(SHyperlink)
									.Text(LOCTEXT("TutorialLink", "▶  Start the tutorial — browse the delivered projects"))
									.TextStyle(&FCoreStyle::Get().GetWidgetStyle<FTextBlockStyle>("NormalText"))
									.OnNavigate_Lambda([&]() { OpenTutorials(); })
								]
								+ SVerticalBox::Slot().AutoHeight().Padding(0, 2, 0, 0)
								[
									SNew(STextBlock).AutoWrapText(true).Font(FCoreStyle::GetDefaultFontStyle("Regular", 10)).ColorAndOpacity(FSlateColor(kMuted))
									.Text(LOCTEXT("TutorialHint", "Opens a matrix of the example aircraft shipped with the application, from a single fuselage up to a complete twin-boom airframe."))
								]
							]
							+ SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 8)
							[
								SNew(SBox).MinDesiredHeight(96).MaxDesiredHeight(130)
								[
									SNew(SScrollBox)
									+ SScrollBox::Slot()
									[
										SNew(STextBlock).AutoWrapText(true).Font(FCoreStyle::GetDefaultFontStyle("Regular", 9)).ColorAndOpacity(FSlateColor(kMuted))
										.Text(FText::FromString(Legal))
									]
								]
							]
							+ SVerticalBox::Slot().AutoHeight().Padding(0, 2, 0, 0)
							[
								SNew(SHorizontalBox)
								+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
								[
									SNew(SCheckBox)
									.IsChecked_Lambda([&]() { return bShowAtStartup ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
									.OnCheckStateChanged_Lambda([&](ECheckBoxState S) { bShowAtStartup = S == ECheckBoxState::Checked; })
									[ SNew(STextBlock).Font(FCoreStyle::GetDefaultFontStyle("Regular", 10)).Text(LOCTEXT("ShowAtStartup", "Show this screen at start-up")) ]
								]
								+ SHorizontalBox::Slot().FillWidth(1.0f) [ SNullWidget::NullWidget ]
								+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
								[ SNew(SButton).Text(LOCTEXT("Tutorials", "Tutorials...")).OnClicked_Lambda([&]() { OpenTutorials(); return FReply::Handled(); }) ]
								+ SHorizontalBox::Slot().AutoWidth()
								[ SNew(SButton).Text(LOCTEXT("Close", "Close")).OnClicked_Lambda([&]() { Close(); return FReply::Handled(); }) ]
							]
						]
					]
				]
			]
		]);

	FSlateApplication::Get().AddModalWindow(Window, FindAerotelicMainWindow());

	// Persisted on every exit path, as SplashScreen.done() does.
	AerotelicSettings::SetShowSplashAtStartup(bShowAtStartup);

	if (bWantsTutorials)
	{
		ShowAerotelicTutorialDialog();
	}
	return true;
}

#undef LOCTEXT_NAMESPACE
