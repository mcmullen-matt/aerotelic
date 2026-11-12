#include "AirfoilLibraryDialog.h"
#include "Widgets/MessageBox.h"
#include "AirfoilBrowser.h"
#include "SectionAeroDialog.h"

#include "Framework/Application/SlateApplication.h"
#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Text/STextBlock.h"
#include "Misc/MessageDialog.h"

#include "Airfoils/SeligDatabase.h"
#include "Widgets/AerotelicDialogWindow.h"

#define LOCTEXT_NAMESPACE "AirfoilLibraryDialog"

namespace
{
	TWeakPtr<SWindow> GLibraryWindow;
}

void OpenAerotelicAirfoilLibrary()
{
	if (TSharedPtr<SWindow> Existing = GLibraryWindow.Pin())
	{
		Existing->BringToFront();
		FSlateApplication::Get().SetKeyboardFocus(Existing);
		return;
	}

	const FVector2D Size(400, 800);
	TSharedRef<SAerotelicAirfoilBrowser> BrowserRef = SNew(SAerotelicAirfoilBrowser)
		.Sources({ EAirfoilSource::Selig, EAirfoilSource::Custom })
		.ListHeight(200.0f)
		.PreviewSize(FVector2D(380, 340));
	// The window outlives this call: the buttons hold the browser weakly.
	TWeakPtr<SAerotelicAirfoilBrowser> Browser = BrowserRef;
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(LOCTEXT("Title", "Airfoils"))
		.ClientSize(Size)
		.SupportsMaximize(false);

	Window->SetContent(
		SNew(SBorder).Padding(8)
		[
			SNew(SScrollBox)
			+ SScrollBox::Slot()
			[
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 4)
				[
					SNew(SButton).HAlign(HAlign_Center)
					.Text_Lambda([]() { return AerotelicSelig::IsDownloaded() ? LOCTEXT("Redownload", "Re-download") : LOCTEXT("Download", "Download Selig DB"); })
					.OnClicked_Lambda([Browser]()
					{
						ShowSeligDownloadDialog([Browser](bool bOk, const FString& Message)
						{
							AerotelicMessageBox(EAppMsgType::Ok, FText::FromString(Message), FText::FromString(bOk ? TEXT("Download Complete") : TEXT("Download")));
							if (TSharedPtr<SAerotelicAirfoilBrowser> B = Browser.Pin()) B->Refresh();
						});
						return FReply::Handled();
					})
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 4)
				[
					SNew(SButton).HAlign(HAlign_Center)
					.Text(LOCTEXT("LoadDat", "Load .dat..."))
					.OnClicked_Lambda([Browser]() { if (TSharedPtr<SAerotelicAirfoilBrowser> B = Browser.Pin()) B->LoadDatFiles(); return FReply::Handled(); })
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
				[
					SNew(SButton).HAlign(HAlign_Center)
					.Text(LOCTEXT("Analyse", "Analyse section..."))
					.ToolTipText(LOCTEXT("AnalyseTip", "Open the selected airfoil in Section Aerodynamics: re-represent it with fewer poles, drag them with the aerodynamic sensitivities shown, and see its pressure distribution and coefficients"))
					.IsEnabled_Lambda([Browser]() { TSharedPtr<SAerotelicAirfoilBrowser> B = Browser.Pin(); return B.IsValid() && B->GetSelectedAirfoil() != nullptr; })
					.OnClicked_Lambda([Browser]()
					{
						if (TSharedPtr<SAerotelicAirfoilBrowser> B = Browser.Pin())
						{
							if (std::shared_ptr<aerotelic::Airfoil> A = B->GetSelectedAirfoil()) ShowAerotelicSectionAeroDialog([](const FText&) {}, A);
						}
						return FReply::Handled();
					})
				]
				+ SVerticalBox::Slot().AutoHeight()
				[
					BrowserRef
				]
			]
		]);

	GLibraryWindow = Window;
	ShowAerotelicDialogWindow(Window, Size);
}

#undef LOCTEXT_NAMESPACE
