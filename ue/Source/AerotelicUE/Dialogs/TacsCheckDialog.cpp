#include "TacsCheckDialog.h"

#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Notifications/SProgressBar.h"
#include "Framework/Application/SlateApplication.h"
#include "Styling/CoreStyle.h"
#include "Async/Async.h"
#include "Containers/Ticker.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformProcess.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "Dom/JsonObject.h"

#include "Widgets/AerotelicDialogWindow.h"
#include "Entity/AerotelicMeshLib.h"
#include "Entity/AerotelicStructuralFea.h"

#define LOCTEXT_NAMESPACE "TacsCheckDialog"

namespace
{
	TWeakPtr<SWindow> GWindow;

	struct FCheck : public TSharedFromThis<FCheck>
	{
		TFunction<void(const FText&)> Status;
		TSharedPtr<STextBlock> Report;
		TSharedPtr<SProgressBar> Bar;
		TSharedPtr<SButton> AgainButton;
		FTSTicker::FDelegateHandle Ticker;
		// Written by the worker, read by the ticker.
		std::atomic<bool> bDone{ false };
		std::atomic<bool> bRunning{ false };
		FCriticalSection Lock;
		FString Text;
		bool bOk = false;

		void SetText(const FString& T)
		{
			FScopeLock L(&Lock);
			Text = T;
		}
		FString GetText()
		{
			FScopeLock L(&Lock);
			return Text;
		}

		// The whole check on a worker thread: the probe for the interpreter,
		// then the script in it.
		void Run()
		{
			if (bRunning.load()) return;
			bRunning = true;
			bDone = false;
			if (Bar.IsValid()) Bar->SetVisibility(EVisibility::Visible);
			if (AgainButton.IsValid()) AgainButton->SetEnabled(false);
			if (Report.IsValid()) Report->SetText(LOCTEXT("Looking", "Looking for the TACS environment (wsl.exe, then ~/micromamba/envs/tacs)..."));
			Status(LOCTEXT("Checking", "Checking the TACS installation..."));
			TSharedRef<FCheck> Self = AsShared();
			Async(EAsyncExecution::Thread, [Self]()
			{
				FString Out;
				const AerotelicMeshLib::FInterpreter I = AerotelicMeshLib::FindTacsInterpreter(true);
				FString Why;
				if (!I.IsValid())
				{
					AerotelicStructuralFea::IsAvailable(AerotelicStructuralFea::EBackend::Tacs, Why);
					Out = TEXT("TACS: NOT FOUND.\n\n") + Why;
					Out += TEXT("\n\nWhat was looked at: the AEROTELIC_TACS_PYTHON environment variable, then wsl.exe -e sh asking the default distribution for ~/micromamba/envs/tacs/bin/python (and miniforge3, miniconda3, mambaforge). The log (Saved/Logs) has the probe's exit code and output.");
					Self->bOk = false;
					Self->SetText(Out);
					Self->bDone = true;
					return;
				}
				Out = FString::Printf(TEXT("TACS environment: %s\n"), *I.Describe());
				const FString Script = FPaths::Combine(FPaths::ProjectDir(), TEXT("tools"), TEXT("tacs_check.py"));
				if (!FPaths::FileExists(Script))
				{
					Out += TEXT("\ntools\\tacs_check.py is not installed beside the application; the solver cannot be exercised.");
					Self->bOk = false;
					Self->SetText(Out);
					Self->bDone = true;
					return;
				}
				const FString Dir = FPaths::Combine(FPlatformProcess::UserTempDir(), TEXT("aerotelic_tacs_check_") + FGuid::NewGuid().ToString(EGuidFormats::Short));
				IFileManager::Get().MakeDirectory(*Dir, true);
				FFileHelper::SaveStringToFile(TEXT("{\"outputs\": {\"result\": \"result.json\"}}\n"), *FPaths::Combine(Dir, TEXT("job.json")), FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
				TSharedPtr<FJsonObject> Result;
				FString Error;
				AerotelicMeshLib::FJobProgress Progress;
				Progress.OnLine = [Self](const FString& Line) { Self->SetText(Self->GetText() + Line + TEXT("\n")); };
				Self->SetText(Out);
				const double T0 = FPlatformTime::Seconds();
				const bool bRan = AerotelicMeshLib::RunJob(I, FPaths::ConvertRelativePathToFull(Script), Dir, Result, Error, &Progress);
				FString Report;
				if (Result.IsValid())
				{
					const TArray<TSharedPtr<FJsonValue>>* Lines = nullptr;
					if (Result->TryGetArrayField(TEXT("report"), Lines))
					{
						for (const TSharedPtr<FJsonValue>& V : *Lines)
						{
							FString S;
							if (V.IsValid() && V->TryGetString(S)) Report += S + TEXT("\n");
						}
					}
				}
				if (Report.IsEmpty()) Report = Self->GetText().Mid(Out.Len());
				Out += TEXT("\n") + Report;
				Out += bRan
					? FString::Printf(TEXT("\nTACS IS USABLE from this application (the check took %.1f s, the first call of a WSL session includes its start-up)."), FPlatformTime::Seconds() - T0)
					: FString::Printf(TEXT("\nTACS IS NOT USABLE: %s"), *Error);
				IFileManager::Get().DeleteDirectory(*Dir, false, true);
				Self->bOk = bRan;
				Self->SetText(Out);
				Self->bDone = true;
			});
			TWeakPtr<FCheck> Weak = AsShared();
			Ticker = FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda([Weak](float)
			{
				TSharedPtr<FCheck> P = Weak.Pin();
				if (!P.IsValid()) return false;
				if (P->Report.IsValid()) P->Report->SetText(FText::FromString(P->GetText()));
				if (!P->bDone.load()) return true;
				P->Finish();
				return false;
			}), 0.25f);
		}

		void Finish()
		{
			Ticker.Reset();
			bRunning = false;
			if (Bar.IsValid()) Bar->SetVisibility(EVisibility::Collapsed);
			if (AgainButton.IsValid()) AgainButton->SetEnabled(true);
			if (Report.IsValid()) Report->SetText(FText::FromString(GetText()));
			Status(bOk ? LOCTEXT("Usable", "TACS is installed and solves.") : LOCTEXT("NotUsable", "TACS is not usable from here; see the report."));
		}
	};
}

void ShowAerotelicTacsCheckDialog(TFunction<void(const FText&)> Status)
{
	if (TSharedPtr<SWindow> Existing = GWindow.Pin())
	{
		Existing->BringToFront();
		FSlateApplication::Get().SetKeyboardFocus(Existing);
		return;
	}
	TSharedRef<FCheck> C = MakeShared<FCheck>();
	C->Status = Status;
	const FVector2D Size(760, 520);
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(LOCTEXT("Title", "TACS (FEA): Check Installation"))
		.ClientSize(Size)
		.SupportsMaximize(true)
		.SupportsMinimize(false);
	TWeakPtr<SWindow> WeakWindow = Window;
	Window->SetContent(
		SNew(SBorder).Padding(12)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
			[
				SNew(STextBlock).AutoWrapText(true)
				.Text(LOCTEXT("Intro", "TACS -- the Toolkit for the Analysis of Composite Structures -- has no Windows build, so the application runs it inside the Windows Subsystem for Linux. This looks for that environment and solves a small plate in it."))
			]
			+ SVerticalBox::Slot().FillHeight(1.0f).Padding(0, 0, 0, 8)
			[
				SNew(SBorder).Padding(8).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(FLinearColor(0.06f, 0.06f, 0.07f))
				[
					SNew(SScrollBox)
					+ SScrollBox::Slot()
					[ SAssignNew(C->Report, STextBlock).AutoWrapText(true).Font(FCoreStyle::GetDefaultFontStyle("Mono", 9)) ]
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
			[ SNew(SBox).HeightOverride(8) [ SAssignNew(C->Bar, SProgressBar).Percent(TOptional<float>()).Visibility(EVisibility::Collapsed) ] ]
			+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[ SAssignNew(C->AgainButton, SButton).Text(LOCTEXT("Again", "Check Again")).OnClicked_Lambda([C]() { C->Run(); return FReply::Handled(); }) ]
				+ SHorizontalBox::Slot().AutoWidth()
				[ SNew(SButton).Text(LOCTEXT("Close", "Close")).OnClicked_Lambda([WeakWindow]() { if (TSharedPtr<SWindow> W = WeakWindow.Pin()) W->RequestDestroyWindow(); return FReply::Handled(); }) ]
			]
		]);
	Window->SetOnWindowClosed(FOnWindowClosed::CreateLambda([C](const TSharedRef<SWindow>&)
	{
		// The worker, if still running, finishes into a state nobody reads.
		C->Ticker.Reset();
		C->Report.Reset();
		C->Bar.Reset();
		C->AgainButton.Reset();
	}));
	GWindow = Window;
	ShowAerotelicDialogWindow(Window, Size);
	C->Run();
}

#undef LOCTEXT_NAMESPACE
