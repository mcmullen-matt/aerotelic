#include "FlapListDialog.h"

#include "Widgets/NumericEntry.h"
#include "Widgets/AerotelicDialogWindow.h"

#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SSeparator.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/SBoxPanel.h"

#include "aerotelic/wing_model.hpp"

#include <algorithm>
#include <string>
#include <vector>

#define LOCTEXT_NAMESPACE "FlapList"

using aerotelic::FlapSpec;
using aerotelic::WingModel;

namespace
{
	constexpr float kNumWidth = 74.0f;

	TSharedRef<SWidget> Header(const FText& Text, TOptional<float> Width = TOptional<float>())
	{
		TSharedRef<SBox> Box = SNew(SBox).Padding(0, 0, 6, 0) [ SNew(STextBlock).Text(Text) ];
		if (Width.IsSet()) Box->SetWidthOverride(Width.GetValue());
		return Box;
	}
}

void SFlapListWidget::Construct(const FArguments& InArgs, WingModel* InModel)
{
	Model = InModel;
	OnChangedDelegate = InArgs._OnChanged;

	ChildSlot
	[
		SNew(SVerticalBox)

		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 2)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().FillWidth(1.0f) [ Header(LOCTEXT("ColName", "Name")) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Header(LOCTEXT("ColRoot", "Root η"), kNumWidth) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Header(LOCTEXT("ColTip", "Tip η"), kNumWidth) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Header(LOCTEXT("ColHingeRoot", "Hinge root"), kNumWidth) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Header(LOCTEXT("ColHingeTip", "Hinge tip"), kNumWidth) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Header(LOCTEXT("ColGap", "Gap"), kNumWidth) ]
			+ SHorizontalBox::Slot().AutoWidth() [ SNew(SBox).WidthOverride(64) ]
		]

		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
		[ SAssignNew(RowsBox, SVerticalBox) ]

		+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 4) [ SNew(SSeparator) ]

		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SButton).Text(LOCTEXT("AddFlap", "+ Add Flap"))
				.ToolTipText(LOCTEXT("AddFlapTip", "Adds a flap in the widest stretch of span that has none yet; edit its extent and hinge in the row."))
				.OnClicked_Lambda([this]() { AddFlap(); return FReply::Handled(); })
			]
		]
	];

	RebuildRows();
}

void SFlapListWidget::RebuildRows()
{
	RowsBox->ClearChildren();
	const int32 Count = static_cast<int32>(Model->flaps.size());
	for (int32 i = 0; i < Count; ++i)
	{
		// A numeric column bound to one FlapSpec field of row i. The row index
		// is re-checked on every read: rows are rebuilt after Add/Remove, but
		// a stale widget may still be polled for a frame.
		auto Num = [this, i](double FlapSpec::*Field, double Lo, double Hi, double Delta, const FText& Tip)
		{
			return SNew(SBox).WidthOverride(kNumWidth).Padding(0, 0, 6, 0)
			[
				SNew(SAerotelicNumericEntry)
				.ToolTipText(Tip)
				.Value_Lambda([this, i, Field]() { return i < static_cast<int32>(Model->flaps.size()) ? Model->flaps[i].*Field : 0.0; })
				.MinValue(Lo).MaxValue(Hi).Delta(Delta)
				.LiveUpdate(false)
				.OnValueCommitted_Lambda([this, i, Field](double V, ETextCommit::Type)
				{
					if (i < static_cast<int32>(Model->flaps.size()) && Model->flaps[i].*Field != V)
					{
						Model->flaps[i].*Field = V;
						Changed();
					}
				})
			];
		};

		RowsBox->AddSlot().AutoHeight().Padding(0, 1)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(0, 0, 6, 0).VAlign(VAlign_Center)
			[
				SNew(SEditableTextBox)
				.ToolTipText(LOCTEXT("NameTip", "The flap body's name in the scene tree and the exports."))
				.Text_Lambda([this, i]() { return i < static_cast<int32>(Model->flaps.size()) ? FText::FromString(StdToFString(Model->flaps[i].name)) : FText(); })
				.OnTextCommitted_Lambda([this, i](const FText& T, ETextCommit::Type)
				{
					if (i >= static_cast<int32>(Model->flaps.size())) return;
					const std::string S = TCHAR_TO_UTF8(*T.ToString());
					if (S != Model->flaps[i].name)
					{
						Model->flaps[i].name = S;
						Changed();
					}
				})
			]
			+ SHorizontalBox::Slot().AutoWidth() [ Num(&FlapSpec::eta_root, 0.0, 1.0, 0.01, LOCTEXT("RootTip", "Inboard end, as a fraction of the semi-span (0 = root).")) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Num(&FlapSpec::eta_tip, 0.0, 1.0, 0.01, LOCTEXT("TipTip", "Outboard end, as a fraction of the semi-span (1 = tip).")) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Num(&FlapSpec::hinge_root, 0.02, 0.98, 0.01, LOCTEXT("HingeRootTip", "Hinge position at the inboard end, as a chord fraction (0 = leading edge, 1 = trailing edge). The hinge line is straight in plan view between the two ends.")) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Num(&FlapSpec::hinge_tip, 0.02, 0.98, 0.01, LOCTEXT("HingeTipTip", "Hinge position at the outboard end, as a chord fraction.")) ]
			+ SHorizontalBox::Slot().AutoWidth() [ Num(&FlapSpec::gap, 0.0, 1.0e9, 0.001, LOCTEXT("GapTip", "Clearance between the flap's nose and the main element's cavity, in model units; constant along the span.")) ]
			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SBox).WidthOverride(64)
				[
					SNew(SButton).Text(LOCTEXT("Remove", "Remove"))
					.OnClicked_Lambda([this, i]() { RemoveFlap(i); return FReply::Handled(); })
				]
			]
		];
	}
}

void SFlapListWidget::AddFlap()
{
	// Into the widest stretch of span with no flap, leaving a tenth of it
	// clear at each end so the new flap neither touches a neighbour nor the
	// root/tip unless the user asks.
	std::vector<std::pair<double, double>> Taken;
	for (const FlapSpec& F : Model->flaps) Taken.emplace_back(F.eta_root, F.eta_tip);
	std::sort(Taken.begin(), Taken.end());
	double BestA = 0.0, BestB = 1.0, Prev = 0.0;
	double BestLen = -1.0;
	auto Consider = [&](double A, double B) { if (B - A > BestLen) { BestLen = B - A; BestA = A; BestB = B; } };
	for (const auto& [A, B] : Taken) { Consider(Prev, A); Prev = std::max(Prev, B); }
	Consider(Prev, 1.0);

	FlapSpec F;
	F.name = "Flap " + std::to_string(Model->flaps.size() + 1);
	if (Taken.empty())
	{
		F.eta_root = 0.55; F.eta_tip = 0.95; // FlapSpec's own defaults
	}
	else
	{
		const double Margin = 0.1 * (BestB - BestA);
		F.eta_root = BestA + Margin;
		F.eta_tip = BestB - Margin;
	}
	Model->flaps.push_back(F);
	RebuildRows();
	Changed();
}

void SFlapListWidget::RemoveFlap(int32 Index)
{
	if (Index < 0 || Index >= static_cast<int32>(Model->flaps.size())) return;
	Model->flaps.erase(Model->flaps.begin() + Index);
	RebuildRows();
	Changed();
}

void SFlapListWidget::Changed()
{
	OnChangedDelegate.ExecuteIfBound();
}

#undef LOCTEXT_NAMESPACE
