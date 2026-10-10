#include "AirfoilBrowser.h"

#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Notifications/SProgressBar.h"
#include "Widgets/Views/SListView.h"
#include "Widgets/Views/STableRow.h"
#include "Framework/Application/SlateApplication.h"
#include "Styling/CoreStyle.h"
#include "Misc/MessageDialog.h"
#include "Misc/Paths.h"
#include "DesktopPlatformModule.h"
#include "IDesktopPlatform.h"

#include "Airfoils/SeligDatabase.h"
#include "Widgets/AirfoilPreview.h"
#include "Widgets/AerotelicDialogWindow.h"
#include "Widgets/CurveTabControls.h"

#include "aerotelic/airfoil.hpp"

#define LOCTEXT_NAMESPACE "AirfoilBrowser"

namespace
{
	// section_panel.py COMMON_NACA4.
	const TCHAR* kCommonNaca4[] = {
		TEXT("0006"), TEXT("0009"), TEXT("0012"), TEXT("0015"), TEXT("0018"), TEXT("0021"),
		TEXT("2412"), TEXT("2415"), TEXT("2418"), TEXT("4412"), TEXT("4415"), TEXT("4418"), TEXT("6412"),
	};
	const FLinearColor kProjectColor(FColor::FromHex(TEXT("#f0c060")));
	const FLinearColor kCustomColor(FColor::FromHex(TEXT("#22cc77")));
	const FLinearColor kDownloadColor(FColor::FromHex(TEXT("#7cc0ff")));

	FString SourceName(EAirfoilSource S)
	{
		switch (S)
		{
		case EAirfoilSource::Project: return TEXT("In this project");
		case EAirfoilSource::Naca:    return TEXT("NACA 4-digit");
		case EAirfoilSource::Selig:   return TEXT("Selig database");
		case EAirfoilSource::Custom:  return TEXT("Custom (.dat)");
		default:                      return TEXT("All sources");
		}
	}

	// section_panel.py unique_airfoils: the same airfoil on several
	// sections is listed once (same name and the same fit points).
	bool SameAirfoil(const aerotelic::Airfoil& A, const aerotelic::Airfoil& B)
	{
		if (A.name != B.name) return false;
		const aerotelic::Mat& PA = A.fit_points();
		const aerotelic::Mat& PB = B.fit_points();
		if (PA.rows() != PB.rows() || PA.cols() != PB.cols()) return false;
		for (int32 i = 0; i < PA.rows(); ++i)
			for (int32 j = 0; j < PA.cols(); ++j)
				if (FMath::Abs(PA(i, j) - PB(i, j)) > 1e-7) return false;
		return true;
	}

	FString Stats(const aerotelic::Airfoil& A, FString& OutThickness, FString& OutCamber)
	{
		try
		{
			const std::pair<double, double> T = A.MaxThickness();
			const std::pair<double, double> C = A.MaxCamber();
			OutThickness = FString::Printf(TEXT("%.2f%%  @  x/c=%.3f"), T.second * 100.0, T.first);
			OutCamber = FString::Printf(TEXT("%.2f%%  @  x/c=%.3f"), C.second * 100.0, C.first);
		}
		catch (const std::exception&)
		{
			OutThickness = OutCamber = TEXT("-");
		}
		return UTF8_TO_TCHAR(A.name.c_str());
	}
}

FString ParseNacaQuery(const FString& Text)
{
	FString S = Text.ToLower().TrimStartAndEnd();
	S.ReplaceInline(TEXT("naca"), TEXT(""));
	S.ReplaceInline(TEXT("-"), TEXT(""));
	S.ReplaceInline(TEXT(" "), TEXT(""));
	if (S.Len() != 4)
	{
		return FString();
	}
	for (TCHAR C : S)
	{
		if (!FChar::IsDigit(C)) return FString();
	}
	return S;
}

// --------------------------------------------------------------- browser

void SAerotelicAirfoilBrowser::Construct(const FArguments& InArgs)
{
	Sources = InArgs._Sources;
	ProjectAirfoils = InArgs._ProjectAirfoils;
	bCompact = InArgs._Compact;
	OnActivated = InArgs._OnActivated;
	OnSelectionChanged = InArgs._OnSelectionChanged;

	SourceOptions.Add(MakeShared<FString>(TEXT("All sources")));
	for (EAirfoilSource S : Sources)
	{
		SourceOptions.Add(MakeShared<FString>(SourceName(S)));
	}
	CurrentSource = SourceOptions[0];

	auto Stat = [](TSharedPtr<STextBlock>& Out) -> TSharedRef<SWidget>
	{
		TSharedRef<STextBlock> Block = SAssignNew(Out, STextBlock).Text(FText::FromString(TEXT("-"))).Justification(ETextJustify::Right)
			.ColorAndOpacity(FSlateColor(FLinearColor(0.85f, 0.85f, 0.85f)));
		return Block;
	};

	ChildSlot
	[
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 4)
		[
			SAssignNew(SearchBox, SEditableTextBox)
			.HintText(LOCTEXT("SearchHint", "Filter by name  (e.g. naca, s1223, fx...)"))
			.OnTextChanged_Lambda([this](const FText&) { RebuildList(); })
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 2)
		[
			SNew(SComboBox<TSharedPtr<FString>>)
			.OptionsSource(&SourceOptions)
			.InitiallySelectedItem(CurrentSource)
			.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
			.OnSelectionChanged_Lambda([this](TSharedPtr<FString> S, ESelectInfo::Type) { if (S.IsValid()) { CurrentSource = S; RebuildList(); } })
			[
				SNew(STextBlock).Text_Lambda([this]() { return FText::FromString(CurrentSource.IsValid() ? *CurrentSource : FString()); })
			]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 4)
		[
			SAssignNew(ResultLabel, STextBlock)
			.Font(FCoreStyle::GetDefaultFontStyle("Regular", 8))
			.ColorAndOpacity(FSlateColor(FLinearColor(0.545f, 0.58f, 0.62f)))
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
		[
			SNew(SBox).HeightOverride(InArgs._ListHeight)
			[
				SNew(SBorder).Padding(1).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(FLinearColor(0.06f, 0.06f, 0.07f))
				[
					SAssignNew(ListView, SListView<TSharedPtr<FAirfoilListItem>>)
					.ListItemsSource(&Items)
					.SelectionMode(ESelectionMode::Single)
					.OnGenerateRow(this, &SAerotelicAirfoilBrowser::MakeRow)
					.OnSelectionChanged(this, &SAerotelicAirfoilBrowser::OnSelect)
					.OnMouseButtonDoubleClick(this, &SAerotelicAirfoilBrowser::OnDoubleClick)
				]
			]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
		[
			SAssignNew(Preview, SAerotelicAirfoilPreview).Compact(bCompact).DesiredSize(InArgs._PreviewSize)
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			MakeGroup(LOCTEXT("Statistics", "Statistics"),
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight()
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().AutoWidth() [ SNew(STextBlock).Text(LOCTEXT("Name", "Name:")) ]
					+ SHorizontalBox::Slot().FillWidth(1.0f) [ Stat(StatName) ]
				]
				+ SVerticalBox::Slot().AutoHeight()
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().AutoWidth() [ SNew(STextBlock).Text(LOCTEXT("MaxT", "Max t/c (% chord):")) ]
					+ SHorizontalBox::Slot().FillWidth(1.0f) [ Stat(StatThickness) ]
				]
				+ SVerticalBox::Slot().AutoHeight()
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().AutoWidth() [ SNew(STextBlock).Text(LOCTEXT("MaxC", "Max camber (% chord):")) ]
					+ SHorizontalBox::Slot().FillWidth(1.0f) [ Stat(StatCamber) ]
				])
		]
	];

	RebuildList();
}

bool SAerotelicAirfoilBrowser::Offers(EAirfoilSource Source) const
{
	if (!Sources.Contains(Source))
	{
		return false;
	}
	return !CurrentSource.IsValid() || *CurrentSource == TEXT("All sources") || *CurrentSource == SourceName(Source);
}

void SAerotelicAirfoilBrowser::RebuildList()
{
	const FString Query = SearchBox.IsValid() ? SearchBox->GetText().ToString().TrimStartAndEnd() : FString();
	const FString Q = Query.ToLower();
	Items.Reset();
	int32 Count = 0;

	// In this project, de-duplicated (◆, gold).
	if (Offers(EAirfoilSource::Project))
	{
		TArray<std::shared_ptr<aerotelic::Airfoil>> Unique;
		for (const std::shared_ptr<aerotelic::Airfoil>& A : ProjectAirfoils)
		{
			if (!A) continue;
			bool bSeen = false;
			for (const std::shared_ptr<aerotelic::Airfoil>& U : Unique) if (SameAirfoil(*U, *A)) { bSeen = true; break; }
			if (bSeen) continue;
			Unique.Add(A);
			const FString Name = UTF8_TO_TCHAR(A->name.c_str());
			if (Q.IsEmpty() || Name.ToLower().Contains(Q))
			{
				TSharedPtr<FAirfoilListItem> It = MakeShared<FAirfoilListItem>();
				It->Source = EAirfoilSource::Project;
				It->Label = TEXT("◆ ") + Name;
				It->Airfoil = A;
				Items.Add(It);
				++Count;
			}
		}
	}

	// Custom .dat (★, green).
	if (Offers(EAirfoilSource::Custom))
	{
		for (const AerotelicSelig::FCustomAirfoil& C : AerotelicSelig::CustomAirfoils())
		{
			if (!C.Airfoil) continue;
			const FString Name = UTF8_TO_TCHAR(C.Airfoil->name.c_str());
			if (Q.IsEmpty() || C.Key.ToLower().Contains(Q) || Name.ToLower().Contains(Q))
			{
				TSharedPtr<FAirfoilListItem> It = MakeShared<FAirfoilListItem>();
				It->Source = EAirfoilSource::Custom;
				It->Label = TEXT("★ ") + Name;
				It->Key = C.Key;
				It->Airfoil = C.Airfoil;
				Items.Add(It);
				++Count;
			}
		}
	}

	// NACA 4-digit: the common codes, plus whatever valid code was typed.
	if (Offers(EAirfoilSource::Naca))
	{
		TArray<FString> Codes;
		const FString Typed = ParseNacaQuery(Query);
		if (!Typed.IsEmpty())
		{
			Codes.Add(Typed);
		}
		for (const TCHAR* Code : kCommonNaca4)
		{
			if (Typed == Code) continue;
			if (Q.IsEmpty() || !Typed.IsEmpty() || FString(TEXT("naca ")).Append(Code).Contains(Q) || FString(Code).Contains(Q))
			{
				Codes.Add(Code);
			}
		}
		for (const FString& Code : Codes)
		{
			TSharedPtr<FAirfoilListItem> It = MakeShared<FAirfoilListItem>();
			It->Source = EAirfoilSource::Naca;
			It->Label = TEXT("NACA ") + Code;
			It->Key = Code;
			Items.Add(It);
			++Count;
		}
	}

	// The Selig database, or the offer to download it.
	if (Offers(EAirfoilSource::Selig))
	{
		if (AerotelicSelig::IsDownloaded())
		{
			for (const AerotelicSelig::FEntry& E : AerotelicSelig::Search(Query, INT32_MAX))
			{
				TSharedPtr<FAirfoilListItem> It = MakeShared<FAirfoilListItem>();
				It->Source = EAirfoilSource::Selig;
				It->Label = (!E.Display.IsEmpty() && E.Display != E.Stem) ? E.Display : E.Stem;
				It->Key = E.Stem;
				Items.Add(It);
				++Count;
			}
		}
		else
		{
			TSharedPtr<FAirfoilListItem> It = MakeShared<FAirfoilListItem>();
			It->Source = EAirfoilSource::Download;
			It->Label = TEXT("Download the Selig database (~1,650 airfoils)...");
			Items.Add(It);
		}
	}

	if (ResultLabel.IsValid())
	{
		ResultLabel->SetText(FText::FromString(FString::Printf(TEXT("%d result(s)"), Count)));
	}
	if (ListView.IsValid())
	{
		ListView->RequestListRefresh();
		if (!PendingSelectKey.IsEmpty())
		{
			for (const TSharedPtr<FAirfoilListItem>& It : Items)
			{
				if (It->Key == PendingSelectKey)
				{
					ListView->SetSelection(It);
					ListView->RequestScrollIntoView(It);
					break;
				}
			}
			PendingSelectKey.Empty();
		}
	}
}

TSharedRef<ITableRow> SAerotelicAirfoilBrowser::MakeRow(TSharedPtr<FAirfoilListItem> Item, const TSharedRef<STableViewBase>& Owner)
{
	FLinearColor Color = FLinearColor::White;
	switch (Item->Source)
	{
	case EAirfoilSource::Project:  Color = kProjectColor; break;
	case EAirfoilSource::Custom:   Color = kCustomColor; break;
	case EAirfoilSource::Download: Color = kDownloadColor; break;
	default: break;
	}
	return SNew(STableRow<TSharedPtr<FAirfoilListItem>>, Owner)
		.Padding(FMargin(4, 2))
		[
			SNew(STextBlock).Text(FText::FromString(Item->Label)).ColorAndOpacity(FSlateColor(Color))
		];
}

void SAerotelicAirfoilBrowser::OnSelect(TSharedPtr<FAirfoilListItem> Item, ESelectInfo::Type)
{
	Selected = nullptr;
	if (Item.IsValid())
	{
		if (Item->Source == EAirfoilSource::Download)
		{
			StartDownload();
			return;
		}
		if (!Item->Airfoil)
		{
			FString Error;
			try
			{
				if (Item->Source == EAirfoilSource::Naca)
				{
					std::unique_ptr<aerotelic::Airfoil> A = aerotelic::Airfoil::Naca4(TCHAR_TO_UTF8(*Item->Key));
					Item->Airfoil = std::shared_ptr<aerotelic::Airfoil>(A.release());
				}
				else if (Item->Source == EAirfoilSource::Selig)
				{
					Item->Airfoil = AerotelicSelig::LoadAirfoil(Item->Key, Error);
				}
			}
			catch (const std::exception& e)
			{
				Error = UTF8_TO_TCHAR(e.what());
			}
			if (!Item->Airfoil && ResultLabel.IsValid())
			{
				ResultLabel->SetText(FText::FromString(FString::Printf(TEXT("Load error: %s"), *Error)));
			}
		}
		Selected = Item->Airfoil;
	}
	Preview->SetAirfoil(Selected);
	if (Selected)
	{
		FString T, C;
		const FString Name = Stats(*Selected, T, C);
		StatName->SetText(FText::FromString(Name));
		StatThickness->SetText(FText::FromString(T));
		StatCamber->SetText(FText::FromString(C));
	}
	else
	{
		StatName->SetText(FText::FromString(TEXT("-")));
		StatThickness->SetText(FText::FromString(TEXT("-")));
		StatCamber->SetText(FText::FromString(TEXT("-")));
	}
	OnSelectionChanged.ExecuteIfBound(Selected);
}

void SAerotelicAirfoilBrowser::OnDoubleClick(TSharedPtr<FAirfoilListItem> Item)
{
	if (Item.IsValid() && Item->Source != EAirfoilSource::Download && Selected)
	{
		OnActivated.ExecuteIfBound(Selected);
	}
}

void SAerotelicAirfoilBrowser::Refresh()
{
	AerotelicSelig::InvalidateIndex();
	RebuildList();
}

void SAerotelicAirfoilBrowser::StartDownload()
{
	TWeakPtr<SAerotelicAirfoilBrowser> Weak = SharedThis(this);
	ShowSeligDownloadDialog([Weak](bool bOk, const FString& Message)
	{
		FMessageDialog::Open(bOk ? EAppMsgType::Ok : EAppMsgType::Ok, FText::FromString(Message),
			FText::FromString(bOk ? TEXT("Download Complete") : TEXT("Download")));
		if (TSharedPtr<SAerotelicAirfoilBrowser> Self = Weak.Pin())
		{
			Self->Refresh();
		}
	});
}

void SAerotelicAirfoilBrowser::LoadDatFiles()
{
	IDesktopPlatform* Desktop = FDesktopPlatformModule::Get();
	if (Desktop == nullptr)
	{
		return;
	}
	TArray<FString> Files;
	const void* Parent = FSlateApplication::Get().FindBestParentWindowHandleForDialogs(AsShared());
	if (!Desktop->OpenFileDialog(Parent, TEXT("Load Airfoil .dat"), TEXT(""), TEXT(""),
		TEXT("DAT Files (*.dat)|*.dat|All Files (*.*)|*.*"), EFileDialogFlags::Multiple, Files) || Files.Num() == 0)
	{
		return;
	}
	FString FirstKey;
	for (const FString& File : Files)
	{
		FString Error;
		std::shared_ptr<aerotelic::Airfoil> A = AerotelicSelig::LoadDatFile(File, Error);
		if (!A)
		{
			FMessageDialog::Open(EAppMsgType::Ok, FText::FromString(FString::Printf(TEXT("%s: %s"), *FPaths::GetCleanFilename(File), *Error)),
				FText::FromString(TEXT("Load Error")));
			continue;
		}
		const FString Key = TEXT("custom_") + FPaths::GetBaseFilename(File);
		AerotelicSelig::AddCustomAirfoil(Key, A);
		if (FirstKey.IsEmpty()) FirstKey = Key;
	}
	if (!FirstKey.IsEmpty())
	{
		// Show the custom source so the new entry is in view, and select it.
		for (const TSharedPtr<FString>& S : SourceOptions)
		{
			if (*S == SourceName(EAirfoilSource::Custom) || *S == TEXT("All sources")) { CurrentSource = S; }
			if (*S == TEXT("All sources")) break;
		}
		PendingSelectKey = FirstKey;
	}
	RebuildList();
}

// ------------------------------------------------------- download dialog

void ShowSeligDownloadDialog(TFunction<void(bool, const FString&)> OnDone)
{
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(LOCTEXT("DownloadTitle", "Downloading Selig Airfoil Database"))
		.ClientSize(FVector2D(460, 190))
		.SupportsMaximize(false)
		.SupportsMinimize(false)
		.SizingRule(ESizingRule::FixedSize);

	TSharedPtr<STextBlock> Status;
	TSharedPtr<SProgressBar> Bar;
	TSharedRef<AerotelicSelig::FDownload> Download = MakeShared<AerotelicSelig::FDownload>();
	// Shared so the completion can tell a user's Cancel from the window
	// closing after the worker finished.
	TSharedRef<bool> bFinished = MakeShared<bool>(false);

	Window->SetContent(
		SNew(SBorder).Padding(12)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 10)
			[
				SNew(STextBlock).AutoWrapText(true)
				.Text(FText::FromString(FString::Printf(TEXT("Fetching ~1,650 airfoils from the UIUC Airfoil Coordinates Database (m-selig.ae.illinois.edu). This only happens once; the files are cached in %s."), *AerotelicSelig::CacheDir())))
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
			[
				SAssignNew(Status, STextBlock).Text(LOCTEXT("Initialising", "Initialising..."))
				.ColorAndOpacity(FSlateColor(FLinearColor(0.85f, 0.85f, 0.85f)))
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 10)
			[
				SAssignNew(Bar, SProgressBar).Percent(0.0f)
			]
			+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
			[
				SNew(SButton).Text(LOCTEXT("Cancel", "Cancel"))
				.OnClicked_Lambda([Window, Download]() { Download->Cancel(); Window->RequestDestroyWindow(); return FReply::Handled(); })
			]
		]);

	TWeakPtr<SWindow> WeakWindow = Window;
	Window->SetOnWindowClosed(FOnWindowClosed::CreateLambda([Download, bFinished](const TSharedRef<SWindow>&)
	{
		if (!*bFinished)
		{
			Download->Cancel();
		}
	}));

	ShowAerotelicDialogWindow(Window, FVector2D(460, 190));

	Download->Start(
		AerotelicSelig::FOnProgress::CreateLambda([Status, Bar](int32 Percent, const FString& Text)
		{
			if (Status.IsValid()) Status->SetText(FText::FromString(Text));
			if (Bar.IsValid()) Bar->SetPercent(Percent / 100.0f);
		}),
		AerotelicSelig::FOnFinished::CreateLambda([WeakWindow, bFinished, OnDone, Download](bool bOk, const FString& Message)
		{
			*bFinished = true;
			if (TSharedPtr<SWindow> W = WeakWindow.Pin())
			{
				W->RequestDestroyWindow();
			}
			OnDone(bOk, Message);
		}));
}

#undef LOCTEXT_NAMESPACE
