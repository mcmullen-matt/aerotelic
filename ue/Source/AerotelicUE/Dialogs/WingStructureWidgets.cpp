#include "WingStructureWidgets.h"

#include "Widgets/NumericEntry.h"
#include "Widgets/AerotelicDialogWindow.h"

#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SSeparator.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/SBoxPanel.h"
#include "Styling/CoreStyle.h"

#include "aerotelic/wing_model.hpp"

#include <algorithm>
#include <string>
#include <vector>

#define LOCTEXT_NAMESPACE "WingStructure"

using aerotelic::WingModel;
using aerotelic::WingSparSpec;
using aerotelic::WingStringerSpec;

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

void SWingStructureWidget::Construct(const FArguments& InArgs, WingModel* InModel)
{
	Model = InModel;
	OnChangedDelegate = InArgs._OnChanged;

	ChildSlot
	[
		SNew(SVerticalBox)

		// ---- ribs ----
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 4)
		[ SNew(STextBlock).Text(LOCTEXT("RibsTitle", "Ribs  (in the section planes)")).Font(FCoreStyle::GetDefaultFontStyle("Bold", 10)) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
		[ BuildRibRules() ]

		+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 4) [ SNew(SSeparator) ]

		// ---- spars ----
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 4)
		[ SNew(STextBlock).Text(LOCTEXT("SparsTitle", "Spars  (webs between the skins along a chord-fraction line)")).Font(FCoreStyle::GetDefaultFontStyle("Bold", 10)) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 2)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().FillWidth(1.0f) [ Header(LOCTEXT("ColName", "Name")) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Header(LOCTEXT("ColXRoot", "x/c root"), kNumWidth) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Header(LOCTEXT("ColXTip", "x/c tip"), kNumWidth) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Header(LOCTEXT("ColRoot", "Root η"), kNumWidth) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Header(LOCTEXT("ColTip", "Tip η"), kNumWidth) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Header(LOCTEXT("ColThick", "Thickness"), kNumWidth) ]
			+ SHorizontalBox::Slot().AutoWidth() [ SNew(SBox).WidthOverride(64) ]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
		[ SAssignNew(SparRows, SVerticalBox) ]
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SButton).Text(LOCTEXT("AddSpar", "+ Add Spar"))
				.ToolTipText(LOCTEXT("AddSparTip", "Adds a spar half way between the last spar and the trailing edge (or at a quarter chord when there is none), running the whole span."))
				.OnClicked_Lambda([this]() { AddSpar(); return FReply::Handled(); })
			]
		]

		+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 4) [ SNew(SSeparator) ]

		// ---- stringers ----
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 4)
		[ SNew(STextBlock).Text(LOCTEXT("StringersTitle", "Stringers  (standing in from the skin)")).Font(FCoreStyle::GetDefaultFontStyle("Bold", 10)) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 2)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().FillWidth(1.0f) [ Header(LOCTEXT("ColName", "Name")) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Header(LOCTEXT("ColSurface", "Surface"), 78.0f) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Header(LOCTEXT("ColXRoot", "x/c root"), kNumWidth) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Header(LOCTEXT("ColXTip", "x/c tip"), kNumWidth) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Header(LOCTEXT("ColRoot", "Root η"), kNumWidth) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Header(LOCTEXT("ColTip", "Tip η"), kNumWidth) ]
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
				.ToolTipText(LOCTEXT("AddStringerTip", "Adds a stringer on the upper surface at mid chord, running the whole span."))
				.OnClicked_Lambda([this]() { AddStringer(); return FReply::Handled(); })
			]
		]
	];

	RebuildSparRows();
	RebuildStringerRows();
}

TSharedRef<SWidget> SWingStructureWidget::BuildRibRules()
{
	auto Flag = [this](const FText& Text, const FText& Tip, bool aerotelic::WingRibRules::*Field)
	{
		return SNew(SCheckBox)
			.ToolTipText(Tip)
			.IsChecked_Lambda([this, Field]() { return Model->structure.ribs.*Field ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
			.OnCheckStateChanged_Lambda([this, Field](ECheckBoxState S)
			{
				const bool bOn = S == ECheckBoxState::Checked;
				if (Model->structure.ribs.*Field != bOn) { Model->structure.ribs.*Field = bOn; Changed(); }
			})
			[ SNew(STextBlock).Text(Text) ];
	};
	return SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 1)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 12, 0) [ Flag(LOCTEXT("AtSections", "At every section"), LOCTEXT("AtSectionsTip", "A rib at each airfoil section's station (those strictly inside the span; the root and tip are the caps)."), &aerotelic::WingRibRules::at_sections) ]
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 12, 0) [ Flag(LOCTEXT("AtFlapEnds", "At flap ends"), LOCTEXT("AtFlapEndsTip", "A rib at each end of every flap's pocket in the main element (the flap body's own end caps stand as its ribs)."), &aerotelic::WingRibRules::at_flap_ends) ]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 1)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth() [ Label(LOCTEXT("Between", "Ribs between the caps:")) ]
			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SBox).WidthOverride(kNumWidth)
				[
					SNew(SAerotelicNumericEntry)
					.ToolTipText(LOCTEXT("BetweenTip", "This many ribs evenly spaced along the semi-span, the root and tip caps not counted."))
					.Value_Lambda([this]() { return static_cast<double>(Model->structure.ribs.between); })
					.MinValue(0.0).MaxValue(400.0).Delta(1.0).LiveUpdate(false)
					.OnValueCommitted_Lambda([this](double V, ETextCommit::Type)
					{
						const int32 N = FMath::Clamp(FMath::RoundToInt(V), 0, 400);
						if (Model->structure.ribs.between != N) { Model->structure.ribs.between = N; Changed(); }
					})
				]
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(12, 0, 0, 0) [ Label(LOCTEXT("RibThickness", "Rib thickness:")) ]
			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SBox).WidthOverride(kNumWidth)
				[
					SNew(SAerotelicNumericEntry)
					.ToolTipText(LOCTEXT("RibThicknessTip", "The ribs' shell thickness, project units; a build figure, not scaled with the model."))
					.Value_Lambda([this]() { return Model->structure.ribs.thickness; })
					.MinValue(0.0).MaxValue(1.0e9).Delta(0.0005).LiveUpdate(false)
					.OnValueCommitted_Lambda([this](double V, ETextCommit::Type)
					{
						if (Model->structure.ribs.thickness != V) { Model->structure.ribs.thickness = V; Changed(); }
					})
				]
			]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 1)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth() [ Label(LOCTEXT("Extra", "More rib stations (η, comma-separated):")) ]
			+ SHorizontalBox::Slot().FillWidth(1.0f)
			[
				SNew(SEditableTextBox)
				.ToolTipText(LOCTEXT("ExtraTip", "Extra rib stations as span fractions, e.g. 0.15, 0.33."))
				.Text_Lambda([this]()
				{
					TArray<FString> Parts;
					for (double E : Model->structure.ribs.extra) Parts.Add(FString::Printf(TEXT("%.4g"), E));
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
					if (Extra != Model->structure.ribs.extra) { Model->structure.ribs.extra = Extra; Changed(); }
				})
			]
		];
}

void SWingStructureWidget::RebuildSparRows()
{
	SparRows->ClearChildren();
	const int32 Count = static_cast<int32>(Model->structure.spars.size());
	for (int32 i = 0; i < Count; ++i)
	{
		auto Num = [this, i](double WingSparSpec::*Field, double Lo, double Hi, double Delta, const FText& Tip)
		{
			return SNew(SBox).WidthOverride(kNumWidth).Padding(0, 0, 6, 0)
			[
				SNew(SAerotelicNumericEntry)
				.ToolTipText(Tip)
				.Value_Lambda([this, i, Field]() { return i < static_cast<int32>(Model->structure.spars.size()) ? Model->structure.spars[i].*Field : 0.0; })
				.MinValue(Lo).MaxValue(Hi).Delta(Delta)
				.LiveUpdate(false)
				.OnValueCommitted_Lambda([this, i, Field](double V, ETextCommit::Type)
				{
					if (i < static_cast<int32>(Model->structure.spars.size()) && Model->structure.spars[i].*Field != V)
					{
						Model->structure.spars[i].*Field = V;
						Changed();
					}
				})
			];
		};
		SparRows->AddSlot().AutoHeight().Padding(0, 1)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(0, 0, 6, 0).VAlign(VAlign_Center)
			[
				SNew(SEditableTextBox)
				.ToolTipText(LOCTEXT("SparNameTip", "The spar's layer name under the body in the Scene tree."))
				.Text_Lambda([this, i]() { return i < static_cast<int32>(Model->structure.spars.size()) ? FText::FromString(StdToFString(Model->structure.spars[i].name)) : FText(); })
				.OnTextCommitted_Lambda([this, i](const FText& T, ETextCommit::Type)
				{
					if (i >= static_cast<int32>(Model->structure.spars.size())) return;
					const std::string S = TCHAR_TO_UTF8(*T.ToString());
					if (S != Model->structure.spars[i].name) { Model->structure.spars[i].name = S; Changed(); }
				})
			]
			+ SHorizontalBox::Slot().AutoWidth() [ Num(&WingSparSpec::x_root, 0.01, 0.99, 0.01, LOCTEXT("XRootTip", "Chord fraction at the inboard end of the spar (0 = leading edge, 1 = trailing edge). The fraction runs linearly to the outboard value along the span.")) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Num(&WingSparSpec::x_tip, 0.01, 0.99, 0.01, LOCTEXT("XTipTip", "Chord fraction at the outboard end of the spar.")) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Num(&WingSparSpec::eta_root, 0.0, 1.0, 0.01, LOCTEXT("RootTip", "Inboard end, as a fraction of the semi-span (0 = root).")) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Num(&WingSparSpec::eta_tip, 0.0, 1.0, 0.01, LOCTEXT("TipTip", "Outboard end, as a fraction of the semi-span (1 = tip). A spar must end clear of a flap's pocket or run through it ahead of the lips.")) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Num(&WingSparSpec::thickness, 0.0, 1.0e9, 0.0005, LOCTEXT("ThickTip", "The web's shell thickness, project units; a build figure, not scaled with the model.")) ]
			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SBox).WidthOverride(64)
				[
					SNew(SButton).Text(LOCTEXT("Remove", "Remove"))
					.OnClicked_Lambda([this, i]() { RemoveSpar(i); return FReply::Handled(); })
				]
			]
		];
	}
}

void SWingStructureWidget::RebuildStringerRows()
{
	StringerRows->ClearChildren();
	const int32 Count = static_cast<int32>(Model->structure.stringers.size());
	for (int32 i = 0; i < Count; ++i)
	{
		auto Num = [this, i](double WingStringerSpec::*Field, double Lo, double Hi, double Delta, const FText& Tip)
		{
			return SNew(SBox).WidthOverride(kNumWidth).Padding(0, 0, 6, 0)
			[
				SNew(SAerotelicNumericEntry)
				.ToolTipText(Tip)
				.Value_Lambda([this, i, Field]() { return i < static_cast<int32>(Model->structure.stringers.size()) ? Model->structure.stringers[i].*Field : 0.0; })
				.MinValue(Lo).MaxValue(Hi).Delta(Delta)
				.LiveUpdate(false)
				.OnValueCommitted_Lambda([this, i, Field](double V, ETextCommit::Type)
				{
					if (i < static_cast<int32>(Model->structure.stringers.size()) && Model->structure.stringers[i].*Field != V)
					{
						Model->structure.stringers[i].*Field = V;
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
				.Text_Lambda([this, i]() { return i < static_cast<int32>(Model->structure.stringers.size()) ? FText::FromString(StdToFString(Model->structure.stringers[i].name)) : FText(); })
				.OnTextCommitted_Lambda([this, i](const FText& T, ETextCommit::Type)
				{
					if (i >= static_cast<int32>(Model->structure.stringers.size())) return;
					const std::string S = TCHAR_TO_UTF8(*T.ToString());
					if (S != Model->structure.stringers[i].name) { Model->structure.stringers[i].name = S; Changed(); }
				})
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0).VAlign(VAlign_Center)
			[
				SNew(SBox).WidthOverride(72)
				[
					SNew(SButton)
					.ToolTipText(LOCTEXT("SurfaceTip", "Which skin the stringer stands in from; click to switch."))
					.Text_Lambda([this, i]() { return i < static_cast<int32>(Model->structure.stringers.size()) && Model->structure.stringers[i].upper ? LOCTEXT("Upper", "upper") : LOCTEXT("Lower", "lower"); })
					.OnClicked_Lambda([this, i]()
					{
						if (i < static_cast<int32>(Model->structure.stringers.size())) { Model->structure.stringers[i].upper = !Model->structure.stringers[i].upper; Changed(); }
						return FReply::Handled();
					})
				]
			]
			+ SHorizontalBox::Slot().AutoWidth() [ Num(&WingStringerSpec::x_root, 0.01, 0.99, 0.01, LOCTEXT("SXRootTip", "Chord fraction at the inboard end (running linearly to the outboard value along the span).")) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Num(&WingStringerSpec::x_tip, 0.01, 0.99, 0.01, LOCTEXT("SXTipTip", "Chord fraction at the outboard end.")) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Num(&WingStringerSpec::eta_root, 0.0, 1.0, 0.01, LOCTEXT("SRootTip", "Inboard end, as a fraction of the semi-span.")) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Num(&WingStringerSpec::eta_tip, 0.0, 1.0, 0.01, LOCTEXT("STipTip", "Outboard end, as a fraction of the semi-span.")) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Num(&WingStringerSpec::height, 0.0, 1.0e9, 0.005, LOCTEXT("HeightTip", "How far the stringer stands in from the skin, project units; not scaled with the model.")) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Num(&WingStringerSpec::thickness, 0.0, 1.0e9, 0.0005, LOCTEXT("SThickTip", "The stringer's shell thickness, project units; not scaled with the model.")) ]
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

void SWingStructureWidget::AddSpar()
{
	WingSparSpec S;
	S.name = "Spar " + std::to_string(Model->structure.spars.size() + 1);
	double Aft = 0.0;
	for (const WingSparSpec& Sp : Model->structure.spars) Aft = std::max(Aft, std::max(Sp.x_root, Sp.x_tip));
	const double X = Model->structure.spars.empty() ? 0.25 : std::min(0.9, 0.5 * (Aft + 1.0));
	S.x_root = S.x_tip = X;
	Model->structure.spars.push_back(S);
	RebuildSparRows();
	Changed();
}

void SWingStructureWidget::RemoveSpar(int32 Index)
{
	if (Index < 0 || Index >= static_cast<int32>(Model->structure.spars.size())) return;
	Model->structure.spars.erase(Model->structure.spars.begin() + Index);
	RebuildSparRows();
	Changed();
}

void SWingStructureWidget::AddStringer()
{
	WingStringerSpec S;
	S.name = "Stringer " + std::to_string(Model->structure.stringers.size() + 1);
	Model->structure.stringers.push_back(S);
	RebuildStringerRows();
	Changed();
}

void SWingStructureWidget::RemoveStringer(int32 Index)
{
	if (Index < 0 || Index >= static_cast<int32>(Model->structure.stringers.size())) return;
	Model->structure.stringers.erase(Model->structure.stringers.begin() + Index);
	RebuildStringerRows();
	Changed();
}

void SWingStructureWidget::Changed()
{
	OnChangedDelegate.ExecuteIfBound();
}

#undef LOCTEXT_NAMESPACE
