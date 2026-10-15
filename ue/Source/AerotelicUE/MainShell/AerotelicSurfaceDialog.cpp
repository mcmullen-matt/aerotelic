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
	// SSlider works in 0..1, so a member with another range (the Fresnel
	// exponent) is mapped onto it and shown in its own units.
	TSharedRef<SWidget> SliderRow(const FText& Label, const FText& Hint, float FAerotelicSurfaceStyle::* Member, float Min = 0.0f, float Max = 1.0f)
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
				.Value_Lambda([Member, Min, Max]() { return Max > Min ? (CurrentStyle().*Member - Min) / (Max - Min) : 0.0f; })
				.OnValueChanged_Lambda([Member, Min, Max](float V)
				{
					FAerotelicSurfaceStyle S = CurrentStyle();
					S.*Member = Min + V * (Max - Min);
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

	// A preset sets the three physical parameters and leaves the Fresnel
	// sheen alone: the sheen is a reading aid, not part of the finish, so a
	// chosen rim survives a hop between Matte and Metal.
	TSharedRef<SWidget> Preset(const FText& Label, float Metallic, float Specular, float Roughness)
	{
		return SNew(SButton)
			.Text(Label)
			.HAlign(HAlign_Center)
			.OnClicked_Lambda([Metallic, Specular, Roughness]()
			{
				FAerotelicSurfaceStyle S = CurrentStyle();
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

	// Tall enough for the sheen rows with no dead space under them
	// (measured on the running app at 1:1 UI scale).
	const FVector2D Size(360, 480);
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
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
				[
					SNew(STextBlock)
					.AutoWrapText(true)
					.Text(LOCTEXT("SheenIntro", "Fresnel sheen -- brightens the surface where it turns away from the eye, so a silhouette and a tight curvature stand out on an untextured body."))
					.ColorAndOpacity(FSlateColor(FLinearColor(0.75f, 0.75f, 0.75f)))
				]
				+ SVerticalBox::Slot().AutoHeight()
				[ SliderRow(LOCTEXT("Sheen", "Edge sheen"), LOCTEXT("SheenHint", "How bright the rim is. 0 switches the sheen off and leaves the plain lit shading."), &FAerotelicSurfaceStyle::FresnelIntensity) ]
				+ SVerticalBox::Slot().AutoHeight()
				[ SliderRow(LOCTEXT("Falloff", "Sheen falloff"), LOCTEXT("FalloffHint", "How tightly the rim hugs the silhouette: 1 washes the whole body, 8 is a thin edge line."), &FAerotelicSurfaceStyle::FresnelExponent, 1.0f, 8.0f) ]
			]
		];

	GSurfaceDialog = Window;
	ShowAerotelicDialogWindow(Window, Size);
}

#undef LOCTEXT_NAMESPACE
