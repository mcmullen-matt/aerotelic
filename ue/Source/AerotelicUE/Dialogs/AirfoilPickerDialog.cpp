#include "AirfoilPickerDialog.h"

#include "Widgets/SWindow.h"
#include "Widgets/SLeafWidget.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SSeparator.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Input/SButton.h"
#include "Framework/Application/SlateApplication.h"
#include "Rendering/DrawElements.h"
#include "Styling/AppStyle.h"

using namespace aerotelic;

namespace
{
	// A static preview of a NACA4 contour -- no interaction, just OnPaint.
	class SAirfoilPreview : public SLeafWidget
	{
	public:
		SLATE_BEGIN_ARGS(SAirfoilPreview) {}
		SLATE_END_ARGS()

		void Construct(const FArguments&) {}

		void SetAirfoil(std::shared_ptr<Airfoil> InAirfoil)
		{
			AirfoilPtr = InAirfoil;
			Invalidate(EInvalidateWidget::Paint);
		}

		virtual FVector2D ComputeDesiredSize(float) const override { return FVector2D(260.0f, 140.0f); }

		virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
			FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const override
		{
			const FVector2D Size = AllottedGeometry.GetLocalSize();
			FSlateDrawElement::MakeBox(OutDrawElements, LayerId, AllottedGeometry.ToPaintGeometry(),
				FAppStyle::Get().GetBrush("WhiteBrush"), ESlateDrawEffect::None, FLinearColor(0.051f, 0.067f, 0.09f));
			if (!AirfoilPtr)
			{
				return LayerId + 1;
			}
			Mat Contour = AirfoilPtr->ContourPoints(120);
			const float Margin = 12.0f;
			const float Scale = (Size.X - 2 * Margin);
			TArray<FVector2f> Pts;
			Pts.Reserve(Contour.rows() + 1);
			for (int32 i = 0; i < Contour.rows(); ++i)
			{
				const float X = Margin + (float)Contour(i, 0) * Scale;
				const float Y = Size.Y * 0.5f - (float)Contour(i, 1) * Scale;
				Pts.Add(FVector2f(X, Y));
			}
			// Pts.Add(Pts[0]) asserts in UE's TArray -- passing one of the
			// array's own elements back into Add() is treated as a hazard
			// regardless of whether this specific call would reallocate.
			if (Pts.Num() > 0) { const FVector2f First = Pts[0]; Pts.Add(First); }
			if (Pts.Num() >= 2)
			{
				FSlateDrawElement::MakeLines(OutDrawElements, LayerId + 1, AllottedGeometry.ToPaintGeometry(), Pts,
					ESlateDrawEffect::None, FLinearColor(0.227f, 0.482f, 0.835f), true, 1.5f);
			}
			return LayerId + 2;
		}

	private:
		std::shared_ptr<Airfoil> AirfoilPtr;
	};
}

TOptional<std::shared_ptr<Airfoil>> PromptForAirfoil(const TArray<std::shared_ptr<Airfoil>>& ProjectAirfoils, const FString& InitialCode)
{
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(NSLOCTEXT("AirfoilPicker", "Title", "Pick Airfoil"))
		.SizingRule(ESizingRule::Autosized)
		.SupportsMaximize(false)
		.SupportsMinimize(false);

	std::shared_ptr<Airfoil> Result;
	bool bAccepted = false;
	TSharedPtr<SAirfoilPreview> Preview;
	TSharedPtr<STextBlock> ErrorText;
	TSharedPtr<SEditableTextBox> CodeBox;

	auto TryBuildNaca = [](const FString& Code) -> std::shared_ptr<Airfoil>
	{
		const std::string Std = TCHAR_TO_UTF8(*Code);
		if (!IsNaca4Code(Std)) return nullptr;
		try { return Airfoil::Naca4(Std); }
		catch (const std::exception&) { return nullptr; }
	};

	// Project airfoils already in the scene, offered first -- clicking one
	// accepts it outright, matching section_panel.py's merged source list.
	TSharedRef<SVerticalBox> ProjectList = SNew(SVerticalBox);
	for (const std::shared_ptr<Airfoil>& A : ProjectAirfoils)
	{
		if (!A) continue;
		std::shared_ptr<Airfoil> Captured = A;
		ProjectList->AddSlot().AutoHeight().Padding(0, 1)
		[
			SNew(SButton)
			.Text(FText::FromString(FString::Printf(TEXT("◆ %s"), UTF8_TO_TCHAR(A->name.c_str()))))
			.HAlign(HAlign_Left)
			.OnClicked_Lambda([&bAccepted, &Result, Captured, Window]()
			{
				Result = Captured;
				bAccepted = true;
				Window->RequestDestroyWindow();
				return FReply::Handled();
			})
		];
	}

	TSharedRef<SWidget> Content =
		SNew(SBorder)
		.Padding(12)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
			[
				SNew(STextBlock).Text(NSLOCTEXT("AirfoilPicker", "ProjectHeader", "Airfoils already in this project:"))
				.Visibility(ProjectAirfoils.Num() > 0 ? EVisibility::Visible : EVisibility::Collapsed)
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
			[ ProjectList ]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
			[
				SNew(SSeparator)
				.Visibility(ProjectAirfoils.Num() > 0 ? EVisibility::Visible : EVisibility::Collapsed)
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
			[
				SNew(STextBlock).Text(NSLOCTEXT("AirfoilPicker", "Prompt", "NACA 4-digit code (e.g. 2412):"))
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
			[
				SAssignNew(CodeBox, SEditableTextBox)
				.Text(FText::FromString(InitialCode))
				.OnTextChanged_Lambda([&Preview, &ErrorText, TryBuildNaca](const FText& NewText)
				{
					Preview->SetAirfoil(TryBuildNaca(NewText.ToString()));
					ErrorText->SetText(FText::GetEmpty());
				})
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
			[
				SAssignNew(ErrorText, STextBlock)
				.ColorAndOpacity(FLinearColor(0.85f, 0.3f, 0.3f))
				.Text(FText::GetEmpty())
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 10)
			[
				SAssignNew(Preview, SAirfoilPreview)
			]
			+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
				[
					SNew(SButton).Text(NSLOCTEXT("AirfoilPicker", "OK", "OK"))
					.OnClicked_Lambda([&]()
					{
						std::shared_ptr<Airfoil> A = TryBuildNaca(CodeBox->GetText().ToString());
						if (A)
						{
							Result = A;
							bAccepted = true;
							Window->RequestDestroyWindow();
						}
						else
						{
							ErrorText->SetText(NSLOCTEXT("AirfoilPicker", "BadCode", "Not a valid NACA 4-digit code."));
						}
						return FReply::Handled();
					})
				]
				+ SHorizontalBox::Slot().AutoWidth()
				[
					SNew(SButton).Text(NSLOCTEXT("AirfoilPicker", "Cancel", "Cancel"))
					.OnClicked_Lambda([Window]() { Window->RequestDestroyWindow(); return FReply::Handled(); })
				]
			]
		];

	Preview->SetAirfoil(TryBuildNaca(InitialCode));

	Window->SetContent(Content);
	FSlateApplication::Get().AddModalWindow(Window, FSlateApplication::Get().GetActiveTopLevelWindow());

	if (bAccepted)
	{
		return Result;
	}
	return TOptional<std::shared_ptr<Airfoil>>();
}
