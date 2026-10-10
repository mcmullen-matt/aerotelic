#include "AerotelicSurfaceDialog.h"

#include "Framework/Application/SlateApplication.h"
#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SUniformGridPanel.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SSlider.h"

#include "Entity/AerotelicDocumentSubsystem.h"
#include "Widgets/AerotelicDialogWindow.h"

#define LOCTEXT_NAMESPACE "AerotelicSurfaceDialog"

namespace
{
	TWeakPtr<SWindow> GSurfaceDialog;

	FAerotelicSurfaceStyle CurrentStyle()
	{
		UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
		return Doc ? Doc->GetSurfaceStyle() : FAerotelicSurfaceStyle();
	}

	void Apply(const FAerotelicSurfaceStyle& Style)
	{
		if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
		{
			Doc->SetSurfaceStyle(Style);
		}
	}

	// One labelled slider bound to a member of the style, applied live.
	TSharedRef<SWidget> SliderRow(const FText& Label, const FText& Hint, float FAerotelicSurfaceStyle::* Member)
	{
		return SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center) [ SNew(STextBlock).Text(Label) ]
				+ SHorizontalBox::Slot().FillWidth(1.0f).HAlign(HAlign_Right).VAlign(VAlign_Center)
				[
					SNew(STextBlock)
					.ColorAndOpacity(FSlateColor(FLinearColor(0.85f, 0.85f, 0.85f)))
					.Text_Lambda([Member]() { return FText::FromString(FString::Printf(TEXT("%.2f"), CurrentStyle().*Member)); })
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
			[
				SNew(SSlider)
				.MinValue(0.0f).MaxValue(1.0f)
				.Value_Lambda([Member]() { return CurrentStyle().*Member; })
				.OnValueChanged_Lambda([Member](float V)
				{
					FAerotelicSurfaceStyle S = CurrentStyle();
					S.*Member = V;
					Apply(S);
				})
			]
			+ SVerticalBox::Slot().AutoHeight()
			[
				SNew(STextBlock)
				.Text(Hint)
				.AutoWrapText(true)
				.Font(FCoreStyle::GetDefaultFontStyle("Regular", 8))
				.ColorAndOpacity(FSlateColor(FLinearColor(0.53f, 0.53f, 0.53f)))
			];
	}

	TSharedRef<SWidget> Preset(const FText& Label, float Metallic, float Specular, float Roughness)
	{
		return SNew(SButton)
			.Text(Label)
			.HAlign(HAlign_Center)
			.OnClicked_Lambda([Metallic, Specular, Roughness]()
			{
				FAerotelicSurfaceStyle S;
				S.Metallic = Metallic;
				S.Specular = Specular;
				S.Roughness = Roughness;
				Apply(S);
				return FReply::Handled();
			});
	}
}

void OpenAerotelicSurfaceDialog()
{
	if (TSharedPtr<SWindow> Existing = GSurfaceDialog.Pin())
	{
		Existing->BringToFront();
		FSlateApplication::Get().SetKeyboardFocus(Existing);
		return;
	}

	const FVector2D Size(360, 360);
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(LOCTEXT("Title", "Surface"))
		.ClientSize(Size)
		.SupportsMaximize(false)
		[
			SNew(SBorder).Padding(12)
			[
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight()
				[
					SNew(STextBlock)
					.AutoWrapText(true)
					.Text(LOCTEXT("Intro", "The finish every body in the scene is drawn with. Changes apply at once."))
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 4)
				[
					SNew(SUniformGridPanel).SlotPadding(FMargin(3, 0))
					+ SUniformGridPanel::Slot(0, 0) [ Preset(LOCTEXT("Matte", "Matte"), 0.0f, 0.3f, 0.85f) ]
					+ SUniformGridPanel::Slot(1, 0) [ Preset(LOCTEXT("Satin", "Satin"), 0.0f, 0.5f, 0.5f) ]
					+ SUniformGridPanel::Slot(2, 0) [ Preset(LOCTEXT("Glossy", "Glossy"), 0.0f, 0.8f, 0.15f) ]
					+ SUniformGridPanel::Slot(3, 0) [ Preset(LOCTEXT("Metal", "Metal"), 1.0f, 0.5f, 0.3f) ]
				]
				+ SVerticalBox::Slot().AutoHeight()
				[ SliderRow(LOCTEXT("Metallic", "Metallic"), LOCTEXT("MetallicHint", "0 is a painted or plastic surface, 1 is bare metal (the colour tints its reflections)."), &FAerotelicSurfaceStyle::Metallic) ]
				+ SVerticalBox::Slot().AutoHeight()
				[ SliderRow(LOCTEXT("Specular", "Glossiness (specular)"), LOCTEXT("SpecularHint", "How strong the highlights are."), &FAerotelicSurfaceStyle::Specular) ]
				+ SVerticalBox::Slot().AutoHeight()
				[ SliderRow(LOCTEXT("Roughness", "Blur (roughness)"), LOCTEXT("RoughnessHint", "How blurred the reflections and highlights are: 0 is a mirror polish, 1 is fully diffuse."), &FAerotelicSurfaceStyle::Roughness) ]
			]
		];

	GSurfaceDialog = Window;
	ShowAerotelicDialogWindow(Window, Size);
}

#undef LOCTEXT_NAMESPACE
