#include "SectionListDialog.h"
#include "Widgets/MessageBox.h"

#include "Widgets/Layout/SSeparator.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"

#include "Widgets/NumericEntry.h"
#include "Widgets/AerotelicDialogWindow.h"
#include "AirfoilPickerDialog.h"
#include "Misc/MessageDialog.h"

#include "aerotelic/wing_model.hpp"
#include "aerotelic/airfoil.hpp"

using aerotelic::Airfoil;
using aerotelic::WingModel;
using aerotelic::WingSection;

void SSectionListWidget::Construct(const FArguments& InArgs, WingModel* InModel)
{
	Model = InModel;
	ProjectAirfoilsAttr = InArgs._ProjectAirfoils;
	OnChangedDelegate = InArgs._OnChanged;

	ChildSlot
	[
		SNew(SVerticalBox)

		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
		[ SAssignNew(RowsBox, SVerticalBox) ]

		+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 4) [ SNew(SSeparator) ]

		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 4, 0)
			[ SNew(STextBlock).Text(FText::FromString(TEXT("Add at s:"))) ]
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 8, 0)
			[
				SAssignNew(AddEtaEntry, SAerotelicNumericEntry)
				.Value_Lambda([this]() { return PendingAddEta; })
				.MinValue(0.0).MaxValue(1.0).Delta(0.05)
				.LiveUpdate(false)
				.OnValueCommitted_Lambda([this](double V, ETextCommit::Type) { PendingAddEta = V; })
			]
			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SButton).Text(FText::FromString(TEXT("+ Add Section")))
				.OnClicked_Lambda([this]() { AddSection(); return FReply::Handled(); })
			]
		]
	];

	RebuildRows();
}

void SSectionListWidget::RebuildRows()
{
	RowsBox->ClearChildren();
	const std::vector<WingSection>& Secs = Model->sections();
	for (int32 i = 0; i < static_cast<int32>(Secs.size()); ++i)
	{
		const WingSection& Sec = Secs[i];
		const bool bLocked = (Sec.s <= 0.0 || Sec.s >= 1.0);

		RowsBox->AddSlot().AutoHeight().Padding(0, 1)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
			[
				SNew(SAerotelicNumericEntry)
				.Value_Lambda([this, i]() { return Model->sections()[i].s; })
				.MinValue(0.0).MaxValue(1.0).Delta(0.01)
				.LiveUpdate(false)
				.OnValueCommitted_Lambda([this, i](double V, ETextCommit::Type)
				{
					Model->SetSectionS(i, V);
					RebuildRows();
					OnChangedDelegate.ExecuteIfBound();
				})
			]
			+ SHorizontalBox::Slot().FillWidth(1.0f).Padding(0, 0, 6, 0)
			[
				SNew(SButton)
				.Text(FText::FromString(Sec.airfoil ? StdToFString(Sec.airfoil->name) : FString(TEXT("(none)"))))
				.HAlign(HAlign_Left)
				.OnClicked_Lambda([this, i]() { PickAirfoil(i); return FReply::Handled(); })
			]
			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SButton)
				.Text(FText::FromString(TEXT("Remove")))
				.IsEnabled(!bLocked)
				.OnClicked_Lambda([this, i]() { RemoveSection(i); return FReply::Handled(); })
			]
		];
	}
}

namespace
{
	// Model->Add/RemoveSection() etc. can throw on an invalid state; uncaught,
	// that unwinds as an unhandled C++ exception and crashes the whole
	// Editor (confirmed the hard way on a different dialog's numeric field --
	// see NumericEntry.cpp's InvokeGuarded). These buttons don't go through
	// SAerotelicNumericEntry, so they need the same guard here.
	void ReportSectionEditFailure(const std::exception& e)
	{
		AerotelicMessageBox(EAppMsgType::Ok,
			FText::FromString(FString::Printf(TEXT("That change was rejected:\n%s"), UTF8_TO_TCHAR(e.what()))));
	}
}

void SSectionListWidget::PickAirfoil(int32 Index)
{
	TArray<std::shared_ptr<Airfoil>> Choices = ProjectAirfoilsAttr.Get();
	TOptional<std::shared_ptr<Airfoil>> Picked = PromptForAirfoil(Choices);
	if (Picked.IsSet())
	{
		try
		{
			Model->SetSectionAirfoil(Index, Picked.GetValue());
		}
		catch (const std::exception& e)
		{
			ReportSectionEditFailure(e);
			return;
		}
		RebuildRows();
		OnChangedDelegate.ExecuteIfBound();
	}
}

void SSectionListWidget::RemoveSection(int32 Index)
{
	try
	{
		Model->RemoveSection(Index);
	}
	catch (const std::exception& e)
	{
		ReportSectionEditFailure(e);
		return;
	}
	RebuildRows();
	OnChangedDelegate.ExecuteIfBound();
}

void SSectionListWidget::AddSection()
{
	const std::vector<WingSection>& Secs = Model->sections();
	if (Secs.empty())
	{
		return;
	}
	// Nearest existing section's airfoil, matching SectionPanel._add_section.
	const WingSection* Nearest = &Secs[0];
	for (const WingSection& Sec : Secs)
	{
		if (FMath::Abs(Sec.s - PendingAddEta) < FMath::Abs(Nearest->s - PendingAddEta))
		{
			Nearest = &Sec;
		}
	}
	try
	{
		Model->AddSection(Nearest->airfoil, PendingAddEta);
	}
	catch (const std::exception& e)
	{
		ReportSectionEditFailure(e);
		return;
	}
	RebuildRows();
	OnChangedDelegate.ExecuteIfBound();
}
