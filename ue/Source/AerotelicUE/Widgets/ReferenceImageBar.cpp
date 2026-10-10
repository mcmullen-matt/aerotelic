#include "ReferenceImageBar.h"
#include "CurveEditor.h"
#include "CurveTabControls.h"
#include "ReferenceImageMath.h"

#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SWrapBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SSlider.h"
#include "Framework/Application/SlateApplication.h"
#include "DesktopPlatformModule.h"
#include "IDesktopPlatform.h"
#include "Misc/MessageDialog.h"
#include "Misc/Paths.h"

#include "Entity/AerotelicDocumentSubsystem.h"

#define LOCTEXT_NAMESPACE "ReferenceImageBar"

void SReferenceImageBar::Construct(const FArguments& InArgs)
{
	Editor = InArgs._Editor;
	Images = InArgs._Images;
	Key = InArgs._Key;
	ViewLabel = InArgs._ViewLabel;
	Units = InArgs._Units;
	Placement = InArgs._Placement;

	auto HasImage = [this]() { return Current() != nullptr; };
	auto HasPixels = [this]() { return Current() != nullptr && Editor.IsValid() && !Editor->IsReferenceImageMissing(); };

	TSharedRef<SWrapBox> Row = SNew(SWrapBox).UseAllottedSize(true).InnerSlotPadding(FVector2D(6, 3));
	Row->AddSlot().VAlign(VAlign_Center)
	[
		MakeControlButton(LOCTEXT("Import", "Import..."),
			FText::Format(LOCTEXT("ImportTip", "Load a {0} image behind the curves, spanning the reference length. Then click it: drag its body to move, a corner handle to scale, an edge handle to crop, the handle on the stem above it to turn it level."), ViewLabel),
			FOnClicked::CreateSP(this, &SReferenceImageBar::OnImport))
	];
	Row->AddSlot().VAlign(VAlign_Center)
	[
		SNew(SButton).Text(LOCTEXT("Remove", "Remove")).ContentPadding(FMargin(8, 3))
		.IsEnabled_Lambda(HasImage)
		.OnClicked(this, &SReferenceImageBar::OnRemove)
	];
	Row->AddSlot().VAlign(VAlign_Center)
	[
		SNew(SButton).Text(LOCTEXT("Fit", "Fit")).ContentPadding(FMargin(8, 3))
		.ToolTipText(LOCTEXT("FitTip", "Back to the starting placement, uncropped and level: for a view along the body, spanning the reference length with its left edge at 0; for a view down its axis, centred on the axis."))
		.IsEnabled_Lambda(HasPixels)
		.OnClicked(this, &SReferenceImageBar::OnFit)
	];
	Row->AddSlot().VAlign(VAlign_Center)
	[
		SNew(SCheckBox)
		.IsEnabled_Lambda(HasPixels)
		.IsChecked_Lambda([this]() { const auto* S = Current(); return S && S->flip_h ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
		.OnCheckStateChanged_Lambda([this](ECheckBoxState State)
		{
			if (auto* S = Current()) { S->flip_h = State == ECheckBoxState::Checked; Editor->ReferenceImageChanged(); }
		})
		[ SNew(STextBlock).Text(LOCTEXT("FlipH", "Flip H")) ]
	];
	Row->AddSlot().VAlign(VAlign_Center)
	[
		SNew(SCheckBox)
		.IsEnabled_Lambda(HasPixels)
		.IsChecked_Lambda([this]() { const auto* S = Current(); return S && S->flip_v ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
		.OnCheckStateChanged_Lambda([this](ECheckBoxState State)
		{
			if (auto* S = Current()) { S->flip_v = State == ECheckBoxState::Checked; Editor->ReferenceImageChanged(); }
		})
		[ SNew(STextBlock).Text(LOCTEXT("FlipV", "Flip V")) ]
	];
	Row->AddSlot().VAlign(VAlign_Center)
	[
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 4, 0)
		[ SNew(STextBlock).Text(LOCTEXT("Opacity", "Opacity:")) ]
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
		[
			SNew(SBox).WidthOverride(90)
			[
				SNew(SSlider)
				.MinValue(0.05f).MaxValue(1.0f)
				.IsEnabled_Lambda(HasPixels)
				.Value_Lambda([this]() { const auto* S = Current(); return S ? (float)S->opacity : 0.5f; })
				.OnValueChanged_Lambda([this](float V)
				{
					if (auto* S = Current()) { S->opacity = V; Editor->RequestRedraw(); }
				})
				.OnMouseCaptureEnd_Lambda([this]() { Sync(); })
			]
		]
	];

	ChildSlot
	[
		MakeGroup(LOCTEXT("Group", "Reference image"),
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight() [ Row ]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
			[
				SAssignNew(StatusText, STextBlock)
				.AutoWrapText(true)
				.Font(FCoreStyle::GetDefaultFontStyle("Regular", 8))
				.ColorAndOpacity(FSlateColor(FLinearColor(0.53f, 0.53f, 0.53f)))
			])
	];

	if (Editor.IsValid())
	{
		Editor->SetOnReferenceChanged(FSimpleDelegate::CreateSP(this, &SReferenceImageBar::Sync));
	}
	Rebind();
}

aerotelic::ReferenceImageSpec* SReferenceImageBar::Current() const
{
	if (Images == nullptr) return nullptr;
	auto It = Images->find(TCHAR_TO_UTF8(*Key));
	return It == Images->end() ? nullptr : &It->second;
}

FString SReferenceImageBar::BaseDir() const
{
	// Images beside the project file are saved relative to it; resolve
	// against the project's directory when there is one.
	if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
	{
		if (Doc->HasCurrentPath()) return FPaths::GetPath(Doc->GetCurrentPath());
	}
	return FString();
}

void SReferenceImageBar::Rebind()
{
	if (Editor.IsValid())
	{
		Editor->SetReferenceImage(Current(), BaseDir());
	}
	Sync();
}

void SReferenceImageBar::Sync()
{
	if (!StatusText.IsValid()) return;
	const aerotelic::ReferenceImageSpec* S = Current();
	if (S == nullptr)
	{
		StatusText->SetText(LOCTEXT("NoImage", "No image."));
		return;
	}
	const FString Path = UTF8_TO_TCHAR(S->path.c_str());
	if (!Editor.IsValid() || Editor->IsReferenceImageMissing())
	{
		StatusText->SetText(FText::Format(LOCTEXT("Missing", "Missing: {0}"), FText::FromString(Path)));
		return;
	}
	const FIntPoint Size = Editor->GetReferenceImageSize();
	FString Line = FString::Printf(TEXT("%s  ·  %d × %d px  ·  spans %.3f × %.3f (%s)"),
		*FPaths::GetCleanFilename(Path), Size.X, Size.Y,
		ReferenceImageMath::Width(*S, Size), ReferenceImageMath::Height(*S, Size), *Units.ToString());
	if (S->crop[0] > 0.0 || S->crop[1] > 0.0 || S->crop[2] < 1.0 || S->crop[3] < 1.0)
	{
		Line += FString::Printf(TEXT("  ·  cropped to %.0f%% × %.0f%%"), 100.0 * (S->crop[2] - S->crop[0]), 100.0 * (S->crop[3] - S->crop[1]));
	}
	if (!FMath::IsNearlyZero(S->angle))
	{
		Line += FString::Printf(TEXT("  ·  turned %+.1f°"), S->angle);
	}
	StatusText->SetText(FText::FromString(Line));
}

void SReferenceImageBar::Place(aerotelic::ReferenceImageSpec& Spec)
{
	const FIntPoint Size = Editor.IsValid() ? Editor->GetReferenceImageSize() : FIntPoint(1, 1);
	if (Placement.IsBound())
	{
		Placement.Execute(Spec, Size);
	}
	else
	{
		ReferenceImageMath::FitWidth(Spec, Size, 1.0, 0.0, 0.0);
	}
}

FReply SReferenceImageBar::OnImport()
{
	if (Images == nullptr || !Editor.IsValid()) return FReply::Handled();
	IDesktopPlatform* Desktop = FDesktopPlatformModule::Get();
	if (Desktop == nullptr) return FReply::Handled();
	TArray<FString> Files;
	const void* Parent = FSlateApplication::Get().FindBestParentWindowHandleForDialogs(AsShared());
	const bool bOk = Desktop->OpenFileDialog(Parent,
		FText::Format(LOCTEXT("ImportTitle", "Import {0} image"), ViewLabel).ToString(),
		BaseDir(), TEXT(""),
		TEXT("Images (*.png;*.jpg;*.jpeg;*.bmp;*.tif;*.tiff)|*.png;*.jpg;*.jpeg;*.bmp;*.tif;*.tiff|All Files (*.*)|*.*"),
		EFileDialogFlags::None, Files);
	if (!bOk || Files.Num() == 0) return FReply::Handled();

	FString Path = FPaths::ConvertRelativePathToFull(Files[0]);
	FPaths::NormalizeFilename(Path);
	aerotelic::ReferenceImageSpec Fresh;
	Fresh.path = TCHAR_TO_UTF8(*Path);
	(*Images)[TCHAR_TO_UTF8(*Key)] = Fresh;
	Editor->SetReferenceImage(Current(), BaseDir());
	if (Editor->IsReferenceImageMissing())
	{
		Images->erase(TCHAR_TO_UTF8(*Key));
		Editor->SetReferenceImage(nullptr, FString());
		FMessageDialog::Open(EAppMsgType::Ok,
			FText::Format(LOCTEXT("CouldNotRead", "Could not read:\n{0}\n\nSupported formats are PNG, JPEG, BMP and TIFF."), FText::FromString(Path)),
			LOCTEXT("ImportTitleDlg", "Import Reference Image"));
		Sync();
		return FReply::Handled();
	}
	Place(*Current());
	Editor->ReferenceImageChanged();
	return FReply::Handled();
}

FReply SReferenceImageBar::OnRemove()
{
	if (Images != nullptr)
	{
		Images->erase(TCHAR_TO_UTF8(*Key));
	}
	Rebind();
	return FReply::Handled();
}

FReply SReferenceImageBar::OnFit()
{
	if (auto* S = Current())
	{
		ReferenceImageMath::ResetCrop(*S);
		Place(*S);
		Editor->ReferenceImageChanged();
	}
	return FReply::Handled();
}

#undef LOCTEXT_NAMESPACE
