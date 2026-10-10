#include "AirfoilPickerDialog.h"
#include "AirfoilBrowser.h"

#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Input/SButton.h"
#include "Framework/Application/SlateApplication.h"

using namespace aerotelic;

TOptional<std::shared_ptr<Airfoil>> PromptForAirfoil(const TArray<std::shared_ptr<Airfoil>>& ProjectAirfoils, const FString& InitialCode)
{
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(NSLOCTEXT("AirfoilPicker", "Title", "Pick Airfoil"))
		.ClientSize(FVector2D(380, 640))
		.SupportsMaximize(false)
		.SupportsMinimize(false);

	std::shared_ptr<Airfoil> Result;
	bool bAccepted = false;
	TSharedPtr<SAerotelicAirfoilBrowser> Browser;

	auto Accept = [&bAccepted, &Result, Window](std::shared_ptr<Airfoil> A)
	{
		if (A)
		{
			Result = A;
			bAccepted = true;
			Window->RequestDestroyWindow();
		}
	};

	// section_panel.py AirfoilPickerDialog: sources All / In this project /
	// NACA 4-digit / Selig database / Custom (.dat); double-click accepts.
	Window->SetContent(
		SNew(SBorder).Padding(10)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().FillHeight(1.0f)
			[
				SAssignNew(Browser, SAerotelicAirfoilBrowser)
				.Sources({ EAirfoilSource::Project, EAirfoilSource::Naca, EAirfoilSource::Selig, EAirfoilSource::Custom })
				.ProjectAirfoils(ProjectAirfoils)
				.Compact(true)
				.ListHeight(180.0f)
				.PreviewSize(FVector2D(360, 170))
				.OnActivated(SAerotelicAirfoilBrowser::FOnAirfoilChosen::CreateLambda(Accept))
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth()
				[
					SNew(SButton).Text(NSLOCTEXT("AirfoilPicker", "LoadDat", "Load .dat..."))
					.OnClicked_Lambda([&Browser]() { if (Browser.IsValid()) Browser->LoadDatFiles(); return FReply::Handled(); })
				]
				+ SHorizontalBox::Slot().FillWidth(1.0f)
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[
					SNew(SButton).Text(NSLOCTEXT("AirfoilPicker", "OK", "OK"))
					.IsEnabled_Lambda([&Browser]() { return Browser.IsValid() && Browser->GetSelectedAirfoil() != nullptr; })
					.OnClicked_Lambda([&Browser, Accept]() { if (Browser.IsValid()) Accept(Browser->GetSelectedAirfoil()); return FReply::Handled(); })
				]
				+ SHorizontalBox::Slot().AutoWidth()
				[
					SNew(SButton).Text(NSLOCTEXT("AirfoilPicker", "Cancel", "Cancel"))
					.OnClicked_Lambda([Window]() { Window->RequestDestroyWindow(); return FReply::Handled(); })
				]
			]
		]);

	FSlateApplication::Get().AddModalWindow(Window, FSlateApplication::Get().GetActiveTopLevelWindow());

	if (bAccepted)
	{
		return Result;
	}
	return TOptional<std::shared_ptr<Airfoil>>();
}
