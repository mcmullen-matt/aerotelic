#include "FrameStructureWidgets.h"

#include "Widgets/NumericEntry.h"
#include "Widgets/AerotelicDialogWindow.h"
#include "Widgets/CurveTabControls.h"

#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Layout/SSeparator.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/SBoxPanel.h"
#include "Styling/CoreStyle.h"

#include "aerotelic/structure_plan.hpp"

#include <algorithm>
#include <string>
#include <vector>

#define LOCTEXT_NAMESPACE "FrameStructure"

using aerotelic::FrameStructureSpec;
using aerotelic::FrameStringerSpec;

namespace
{
	constexpr float kNumWidth = 74.0f;

	TSharedRef<SWidget> Header(const FText& Text, TOptional<float> Width = TOptional<float>())
	{
		TSharedRef<SBox> Box = SNew(SBox).Padding(0, 0, 6, 0) [ SNew(STextBlock).Text(Text) ];
		if (Width.IsSet()) Box->SetWidthOverride(Width.GetValue());
		return Box;
	}

	TSharedRef<SWidget> Label(const FText& Text)
	{
		return SNew(SBox).MinDesiredWidth(150).VAlign(VAlign_Center) [ SNew(STextBlock).Text(Text) ];
	}
}

void SFrameStructureWidget::Construct(const FArguments& InArgs, FrameStructureSpec* InSpec)
{
	Spec = InSpec;
	OnChangedDelegate = InArgs._OnChanged;

	ChildSlot
	[
		SNew(SVerticalBox)

		// ---- frames ----
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 4)
		[ SNew(STextBlock).Text(LOCTEXT("FramesTitle", "Frames  (in the section planes along the length)")).Font(FCoreStyle::GetDefaultFontStyle("Bold", 10)) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
		[ BuildFrameRules() ]

		+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 4) [ SNew(SSeparator) ]

		// ---- stringers ----
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 4)
		[ SNew(STextBlock).Text(LOCTEXT("StringersTitle", "Stringers and longerons  (standing in from the skin, along the length)")).Font(FCoreStyle::GetDefaultFontStyle("Bold", 10)) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 4)
		[ MakeInfoLabel(InArgs._AroundHint) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 2)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().FillWidth(1.0f) [ Header(LOCTEXT("ColName", "Name")) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Header(LOCTEXT("ColAround", "Around"), kNumWidth) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Header(LOCTEXT("ColStart", "Start x/L"), kNumWidth) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Header(LOCTEXT("ColEnd", "End x/L"), kNumWidth) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Header(LOCTEXT("ColHeight", "Height"), kNumWidth) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Header(LOCTEXT("ColThick", "Thickness"), kNumWidth) ]
			+ SHorizontalBox::Slot().AutoWidth() [ SNew(SBox).WidthOverride(64) ]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
		[ SAssignNew(StringerRows, SVerticalBox) ]
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SButton).Text(LOCTEXT("AddStringer", "+ Add Stringer"))
				.ToolTipText(LOCTEXT("AddStringerTip", "Adds a stringer half way round from the last one, running the whole length."))
				.OnClicked_Lambda([this]() { AddStringer(); return FReply::Handled(); })
			]
		]
	];

	RebuildStringerRows();
}

TSharedRef<SWidget> SFrameStructureWidget::BuildFrameRules()
{
	auto Num = [this](const FText& Tip, TFunction<double()> Get, TFunction<void(double)> Set, double Lo, double Hi, double Delta)
	{
		return SNew(SBox).WidthOverride(kNumWidth)
		[
			SNew(SAerotelicNumericEntry)
			.ToolTipText(Tip)
			.Value_Lambda([Get]() { return Get(); })
			.MinValue(Lo).MaxValue(Hi).Delta(Delta).LiveUpdate(false)
			.OnValueCommitted_Lambda([Get, Set, this](double V, ETextCommit::Type) { if (Get() != V) { Set(V); Changed(); } })
		];
	};
	return SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 1)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth() [ Label(LOCTEXT("Between", "Frames between the ends:")) ]
			+ SHorizontalBox::Slot().AutoWidth()
			[ Num(LOCTEXT("BetweenTip", "This many frames evenly spaced along the length, the two ends not counted."),
				[this]() { return static_cast<double>(Spec->frames.between); },
				[this](double V) { Spec->frames.between = FMath::Clamp(FMath::RoundToInt(V), 0, 400); }, 0.0, 400.0, 1.0) ]
			+ SHorizontalBox::Slot().AutoWidth().Padding(12, 0, 0, 0) [ Label(LOCTEXT("Height", "Height in from the skin:")) ]
			+ SHorizontalBox::Slot().AutoWidth()
			[ Num(LOCTEXT("HeightTip", "A frame is the band of the section within this distance of the skin (project units, not scaled with the model); 0 keeps the whole bulkhead."),
				[this]() { return Spec->frames.height; },
				[this](double V) { Spec->frames.height = V; }, 0.0, 1.0e9, 0.01) ]
			+ SHorizontalBox::Slot().AutoWidth().Padding(12, 0, 0, 0) [ Label(LOCTEXT("Thickness", "Frame thickness:")) ]
			+ SHorizontalBox::Slot().AutoWidth()
			[ Num(LOCTEXT("ThicknessTip", "The frames' shell thickness, project units; a build figure, not scaled with the model."),
				[this]() { return Spec->frames.thickness; },
				[this](double V) { Spec->frames.thickness = V; }, 0.0, 1.0e9, 0.0005) ]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 1)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth() [ Label(LOCTEXT("Extra", "More frame stations (x/L, comma-separated):")) ]
			+ SHorizontalBox::Slot().FillWidth(1.0f)
			[
				SNew(SEditableTextBox)
				.ToolTipText(LOCTEXT("ExtraTip", "Extra frame stations as fractions of the length, e.g. 0.15, 0.33 (a bulkhead at a wing spar)."))
				.Text_Lambda([this]()
				{
					TArray<FString> Parts;
					for (double E : Spec->frames.extra) Parts.Add(FString::Printf(TEXT("%.4g"), E));
					return FText::FromString(FString::Join(Parts, TEXT(", ")));
				})
				.OnTextCommitted_Lambda([this](const FText& T, ETextCommit::Type)
				{
					TArray<FString> Parts;
					T.ToString().ParseIntoArray(Parts, TEXT(","), true);
					std::vector<double> Extra;
					for (FString P : Parts)
					{
						P.TrimStartAndEndInline();
						if (P.IsNumeric()) Extra.push_back(FCString::Atod(*P));
					}
					if (Extra != Spec->frames.extra) { Spec->frames.extra = Extra; Changed(); }
				})
			]
		];
}

void SFrameStructureWidget::RebuildStringerRows()
{
	StringerRows->ClearChildren();
	const int32 Count = static_cast<int32>(Spec->stringers.size());
	for (int32 i = 0; i < Count; ++i)
	{
		auto Num = [this, i](double FrameStringerSpec::*Field, double Lo, double Hi, double Delta, const FText& Tip)
		{
			return SNew(SBox).WidthOverride(kNumWidth).Padding(0, 0, 6, 0)
			[
				SNew(SAerotelicNumericEntry)
				.ToolTipText(Tip)
				.Value_Lambda([this, i, Field]() { return i < static_cast<int32>(Spec->stringers.size()) ? Spec->stringers[i].*Field : 0.0; })
				.MinValue(Lo).MaxValue(Hi).Delta(Delta)
				.LiveUpdate(false)
				.OnValueCommitted_Lambda([this, i, Field](double V, ETextCommit::Type)
				{
					if (i < static_cast<int32>(Spec->stringers.size()) && Spec->stringers[i].*Field != V)
					{
						Spec->stringers[i].*Field = V;
						Changed();
					}
				})
			];
		};
		StringerRows->AddSlot().AutoHeight().Padding(0, 1)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(0, 0, 6, 0).VAlign(VAlign_Center)
			[
				SNew(SEditableTextBox)
				.ToolTipText(LOCTEXT("StringerNameTip", "The stringer's layer name under the body in the Scene tree."))
				.Text_Lambda([this, i]() { return i < static_cast<int32>(Spec->stringers.size()) ? FText::FromString(StdToFString(Spec->stringers[i].name)) : FText(); })
				.OnTextCommitted_Lambda([this, i](const FText& T, ETextCommit::Type)
				{
					if (i >= static_cast<int32>(Spec->stringers.size())) return;
					const std::string S = TCHAR_TO_UTF8(*T.ToString());
					if (S != Spec->stringers[i].name) { Spec->stringers[i].name = S; Changed(); }
				})
			]
			+ SHorizontalBox::Slot().AutoWidth() [ Num(&FrameStringerSpec::around, 0.0, 0.9999, 0.025, LOCTEXT("AroundTip", "Position round the section as a fraction of the way round, 0 to 1 (see the note above for where 0 is).")) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Num(&FrameStringerSpec::x_start, 0.0, 1.0, 0.05, LOCTEXT("StartTip", "Where it starts, as a fraction of the length (0 = nose or lip).")) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Num(&FrameStringerSpec::x_end, 0.0, 1.0, 0.05, LOCTEXT("EndTip", "Where it ends, as a fraction of the length (1 = tail or trailing edge).")) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Num(&FrameStringerSpec::height, 0.0, 1.0e9, 0.005, LOCTEXT("SHeightTip", "How far the stringer stands in from the skin, project units; not scaled with the model.")) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Num(&FrameStringerSpec::thickness, 0.0, 1.0e9, 0.0005, LOCTEXT("SThickTip", "The stringer's shell thickness, project units; not scaled with the model.")) ]
			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SBox).WidthOverride(64)
				[
					SNew(SButton).Text(LOCTEXT("Remove", "Remove"))
					.OnClicked_Lambda([this, i]() { RemoveStringer(i); return FReply::Handled(); })
				]
			]
		];
	}
}

void SFrameStructureWidget::AddStringer()
{
	FrameStringerSpec S;
	S.name = "Stringer " + std::to_string(Spec->stringers.size() + 1);
	// Half way round from the last one; the first at the top.
	S.around = Spec->stringers.empty() ? 0.0 : std::fmod(Spec->stringers.back().around + 0.5, 1.0);
	Spec->stringers.push_back(S);
	RebuildStringerRows();
	Changed();
}

void SFrameStructureWidget::RemoveStringer(int32 Index)
{
	if (Index < 0 || Index >= static_cast<int32>(Spec->stringers.size())) return;
	Spec->stringers.erase(Spec->stringers.begin() + Index);
	RebuildStringerRows();
	Changed();
}

void SFrameStructureWidget::Changed()
{
	OnChangedDelegate.ExecuteIfBound();
}

TSharedRef<SWidget> MakeFrameStructureTab(FrameStructureSpec* Spec, FSimpleDelegate OnChanged, const FText& AroundHint)
{
	return SNew(SScrollBox)
		+ SScrollBox::Slot().Padding(8)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight()
			[ SNew(SFrameStructureWidget, Spec).OnChanged(OnChanged).AroundHint(AroundHint) ]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
			[
				MakeInfoLabel(LOCTEXT("StructureInfo",
					"Built with the body on its own grid on Apply: every frame's rim is a ring of the skin mesh and every stringer's feet a vertex line of it, so the shells share their nodes with the skin. "
					"The layers appear under the body in the Scene tree (Skin, Frames, each stringer) once the body has a skin thickness (the body's Structure... dialog), and follow every later edit."))
			]
		];
}

FText FrameStructureHelpText()
{
	return LOCTEXT("HelpStructure",
		"STRUCTURE\nThe body's internal structure, defined in its own terms and built with it on its own grid. FRAMES lie in the section planes at stations along the length: a given number evenly spaced between the ends and any extra stations typed in (as fractions of the length). "
		"A frame is the band of the section within the given height of the skin -- for a nacelle, of both skins -- or the whole bulkhead (the whole annulus) at a height of 0. "
		"STRINGERS and longerons are strips standing a given height in from the skin, running between two stations along the length at a fixed position round the section. "
		"Each element is a shell with its own thickness (a build figure, not scaled with the model) and takes the body's material unless the project file gives it one. "
		"On Apply the frames' rims and the stringers' feet become vertex lines of the skin mesh, the frames are meshed on them, and the layers appear under the body in the Scene tree once it has a skin thickness. A nacelle with chevrons has no mesh of its own and builds no structure.");
}

#undef LOCTEXT_NAMESPACE
