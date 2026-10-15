#include "AerotelicShadowDialog.h"

#include "Framework/Application/SlateApplication.h"
#include "Widgets/SWindow.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SUniformGridPanel.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SSlider.h"
#include "Styling/CoreStyle.h"

#include "Entity/AerotelicDocumentSubsystem.h"
#include "Widgets/AerotelicDialogWindow.h"

#define LOCTEXT_NAMESPACE "AerotelicShadowDialog"

namespace
{
	TWeakPtr<SWindow> GShadowDialog;

	// The engine's own defaults for a light, which the studio rig leaves
	// alone (ULightComponent::ShadowBias / ShadowSlopeBias).
	constexpr float kDefaultDepthBias = 0.5f;
	constexpr float kDefaultSlopeBias = 0.5f;
	// Both biases are in the same 0..2 range the light's details panel uses.
	constexpr float kBiasMax = 2.0f;

	FAerotelicShadowStyle CurrentStyle()
	{
		UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get();
		return Doc ? Doc->GetShadowStyle() : FAerotelicShadowStyle();
	}

	void Apply(const FAerotelicShadowStyle& Style)
	{
		if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get())
		{
			Doc->SetShadowStyle(Style);
		}
	}

	// One labelled slider over a bias, in the light's own 0..2 units,
	// applied live. Disabled while shadows are off, where it does nothing.
	TSharedRef<SWidget> BiasRow(const FText& Label, const FText& Hint, float FAerotelicShadowStyle::* Member)
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
				.IsEnabled_Lambda([]() { return CurrentStyle().bCastShadows; })
				.Value_Lambda([Member]() { return CurrentStyle().*Member / kBiasMax; })
				.OnValueChanged_Lambda([Member](float V)
				{
					FAerotelicShadowStyle S = CurrentStyle();
					S.*Member = V * kBiasMax;
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

	TSharedRef<SWidget> Preset(const FText& Label, const FText& Tip, float DepthBias, float SlopeBias)
	{
		return SNew(SButton)
			.Text(Label)
			.ToolTipText(Tip)
			.HAlign(HAlign_Center)
			.OnClicked_Lambda([DepthBias, SlopeBias]()
			{
				FAerotelicShadowStyle S = CurrentStyle();
				S.DepthBias = DepthBias;
				S.SlopeBias = SlopeBias;
				Apply(S);
				return FReply::Handled();
			});
	}
}

void OpenAerotelicShadowDialog()
{
	if (TSharedPtr<SWindow> Existing = GShadowDialog.Pin())
	{
		Existing->BringToFront();
		FSlateApplication::Get().SetKeyboardFocus(Existing);
		return;
	}

	// Tall enough for the footnote with no dead space under it (measured on
	// the running app at 1:1 UI scale).
	const FVector2D Size(380, 390);
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(LOCTEXT("Title", "Shadows"))
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
					.Text(LOCTEXT("Intro", "The key light's shadows. The depth map the shadow is read from needs a bias: too little and a surface shadows itself in stripes, too much and the shadow slides away from the part casting it. Changes apply at once."))
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
				[
					SNew(SCheckBox)
					.IsChecked_Lambda([]() { return CurrentStyle().bCastShadows ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
					.OnCheckStateChanged_Lambda([](ECheckBoxState State)
					{
						FAerotelicShadowStyle S = CurrentStyle();
						S.bCastShadows = (State == ECheckBoxState::Checked);
						Apply(S);
					})
					[ SNew(STextBlock).Text(LOCTEXT("Cast", "Cast shadows")) ]
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 4)
				[
					SNew(SUniformGridPanel).SlotPadding(FMargin(3, 0))
					+ SUniformGridPanel::Slot(0, 0)
					[ Preset(LOCTEXT("PresetDefault", "Default"), LOCTEXT("PresetDefaultTip", "The engine's own bias for a light."), kDefaultDepthBias, kDefaultSlopeBias) ]
					+ SUniformGridPanel::Slot(1, 0)
					[ Preset(LOCTEXT("PresetClean", "No acne"), LOCTEXT("PresetCleanTip", "More bias: kills self-shadowing stripes on surfaces edge-on to the light, at the cost of shadows sitting slightly off their caster."), 1.0f, 1.5f) ]
					+ SUniformGridPanel::Slot(2, 0)
					[ Preset(LOCTEXT("PresetTight", "Tight contact"), LOCTEXT("PresetTightTip", "Less bias: shadows meet their caster exactly, at the risk of stripes on grazing surfaces."), 0.2f, 0.2f) ]
				]
				+ SVerticalBox::Slot().AutoHeight()
				[ BiasRow(LOCTEXT("SlopeBias", "Slope (normal) bias"), LOCTEXT("SlopeBiasHint", "Scales the offset with how steeply the surface is inclined to the light, so a face edge-on to it gets the most. This is the one to raise against stripes along a fuselage flank or a wing's upper surface."), &FAerotelicShadowStyle::SlopeBias) ]
				+ SVerticalBox::Slot().AutoHeight()
				[ BiasRow(LOCTEXT("DepthBias", "Depth bias"), LOCTEXT("DepthBiasHint", "A constant offset, the same over the whole model. Raise it only if the slope bias alone cannot clear the stripes."), &FAerotelicShadowStyle::DepthBias) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
				[
					SNew(STextBlock)
					.AutoWrapText(true)
					.Font(FCoreStyle::GetDefaultFontStyle("Regular", 8))
					.ColorAndOpacity(FSlateColor(FLinearColor(0.53f, 0.53f, 0.53f)))
					.Text(LOCTEXT("Note", "The scene renders cascaded shadow maps; virtual shadow maps are off, so the engine's r.Shadow.Virtual.NormalBias has no effect here. These settings are how the scene is viewed and are not saved with the project."))
				]
			]
		];

	GShadowDialog = Window;
	ShowAerotelicDialogWindow(Window, Size);
}

#undef LOCTEXT_NAMESPACE
