#include "ThemeDialog.h"

#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Views/SListView.h"
#include "Widgets/Views/STableRow.h"
#include "Framework/Application/SlateApplication.h"
#include "Styling/CoreStyle.h"

#include "Branding/AerotelicTheme.h"
#include "Widgets/AerotelicDialogWindow.h"

#define LOCTEXT_NAMESPACE "ThemeDialog"

namespace
{
	struct FRow
	{
		FText Header; // set for a group header row, which cannot be chosen
		FString Theme;
	};
	using FRowPtr = TSharedPtr<FRow>;

	// One instance; the rows and the "previous" theme live as long as the
	// window does.
	TWeakPtr<SWindow> GThemeWindow;

	struct FState : public TSharedFromThis<FState>
	{
		TArray<FRowPtr> Rows;
		FString Previous;
		bool bAccepted = false;
		TSharedPtr<SListView<FRowPtr>> List;
	};
}

void ShowAerotelicThemeDialog()
{
	if (TSharedPtr<SWindow> Existing = GThemeWindow.Pin())
	{
		Existing->BringToFront();
		FSlateApplication::Get().SetKeyboardFocus(Existing);
		return;
	}

	TSharedRef<FState> State = MakeShared<FState>();
	State->Previous = AerotelicTheme::CurrentName();
	FRowPtr Initial;
	for (const AerotelicTheme::FGroup& G : AerotelicTheme::Groups())
	{
		FRowPtr H = MakeShared<FRow>();
		H->Header = G.Title;
		State->Rows.Add(H);
		for (const FString& Name : G.Names)
		{
			FRowPtr R = MakeShared<FRow>();
			R->Theme = Name;
			State->Rows.Add(R);
			if (Name == State->Previous)
			{
				Initial = R;
			}
		}
	}

	// Modeless, so the frame repaints in the chosen theme while the list is
	// open (the Qt dialog is modal, but Qt keeps painting behind it; the
	// game viewport does not while a Slate modal runs). Cancel, or closing
	// the window, puts the previous theme back; OK keeps the choice.
	const FVector2D Size(300, 420);
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(LOCTEXT("Title", "Color Theme"))
		.ClientSize(Size)
		.SupportsMaximize(false)
		.SupportsMinimize(false);
	TWeakPtr<SWindow> WeakWindow = Window;

	Window->SetContent(
		SNew(SBorder).Padding(12)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().FillHeight(1.0f).Padding(0, 0, 0, 8)
			[
				SNew(SBorder).Padding(1).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(FLinearColor(0.06f, 0.06f, 0.07f))
				[
					SAssignNew(State->List, SListView<FRowPtr>)
					.ListItemsSource(&State->Rows)
					.SelectionMode(ESelectionMode::Single)
					.OnGenerateRow_Lambda([](FRowPtr Row, const TSharedRef<STableViewBase>& Owner)
					{
						const bool bHeader = !Row->Header.IsEmpty();
						return SNew(STableRow<FRowPtr>, Owner).Padding(FMargin(4, 2))
							[
								SNew(STextBlock)
								.Text(bHeader ? Row->Header : FText::FromString(TEXT("  ") + Row->Theme))
								.Font(bHeader ? FCoreStyle::GetDefaultFontStyle("Bold", 8) : FCoreStyle::GetDefaultFontStyle("Regular", 9))
								.ColorAndOpacity(bHeader ? FSlateColor(FLinearColor(FColor::FromHex(TEXT("#6a7a9a")))) : FSlateColor::UseForeground())
							];
					})
					.OnSelectionChanged_Lambda([State](FRowPtr Row, ESelectInfo::Type)
					{
						if (!Row.IsValid()) return;
						if (!Row->Header.IsEmpty())
						{
							// A header cannot be chosen.
							State->List->ClearSelection();
							return;
						}
						AerotelicTheme::SetCurrent(Row->Theme);
					})
				]
			]
			+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[
					SNew(SButton).Text(LOCTEXT("OK", "OK"))
					.OnClicked_Lambda([State, WeakWindow]()
					{
						State->bAccepted = true;
						if (TSharedPtr<SWindow> W = WeakWindow.Pin()) W->RequestDestroyWindow();
						return FReply::Handled();
					})
				]
				+ SHorizontalBox::Slot().AutoWidth()
				[
					SNew(SButton).Text(LOCTEXT("Cancel", "Cancel"))
					.OnClicked_Lambda([WeakWindow]() { if (TSharedPtr<SWindow> W = WeakWindow.Pin()) W->RequestDestroyWindow(); return FReply::Handled(); })
				]
			]
		]);
	Window->SetOnWindowClosed(FOnWindowClosed::CreateLambda([State](const TSharedRef<SWindow>&)
	{
		if (!State->bAccepted)
		{
			AerotelicTheme::SetCurrent(State->Previous);
		}
	}));
	if (Initial.IsValid())
	{
		State->List->SetSelection(Initial, ESelectInfo::Direct);
	}

	GThemeWindow = Window;
	ShowAerotelicDialogWindow(Window, Size);
}

#undef LOCTEXT_NAMESPACE
