#include "AerotelicTransformDialog.h"

#include "Widgets/SWindow.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Framework/Application/SlateApplication.h"
#include "Styling/CoreStyle.h"

#include "Entity/AerotelicEntityActor.h"
#include "Widgets/NumericEntry.h"
#include "Widgets/AerotelicDialogWindow.h"

#define LOCTEXT_NAMESPACE "AerotelicTransformDialog"

namespace
{
	TMap<TWeakObjectPtr<AAerotelicEntityActor>, TWeakPtr<SWindow>> GOpenTransformDialogs;

	enum EPlacementField : int32 { TX = 0, TY, TZ, AX, AY, AZ, Angle };

	double GetPlacementValue(AAerotelicEntityActor* Entity, int32 Field)
	{
		if (Entity == nullptr) return 0.0;
		const aerotelic::Placement P = Entity->GetPlacement();
		switch (Field)
		{
		case TX: return P.translation[0];
		case TY: return P.translation[1];
		case TZ: return P.translation[2];
		case AX: return P.rotation_axis[0];
		case AY: return P.rotation_axis[1];
		case AZ: return P.rotation_axis[2];
		case Angle: return P.rotation_deg;
		}
		return 0.0;
	}

	void SetPlacementValue(AAerotelicEntityActor* Entity, int32 Field, double Value)
	{
		if (Entity == nullptr) return;
		aerotelic::Placement P = Entity->GetPlacement();
		switch (Field)
		{
		case TX: P.translation[0] = Value; break;
		case TY: P.translation[1] = Value; break;
		case TZ: P.translation[2] = Value; break;
		case AX: P.rotation_axis[0] = Value; break;
		case AY: P.rotation_axis[1] = Value; break;
		case AZ: P.rotation_axis[2] = Value; break;
		case Angle: P.rotation_deg = Value; break;
		}
		Entity->ApplyPlacement(P);
	}

	TSharedRef<SWidget> Heading(const FText& Label)
	{
		return SNew(STextBlock).Text(Label).Font(FCoreStyle::GetDefaultFontStyle("Bold", 10));
	}
}

void OpenAerotelicTransformDialog(AAerotelicEntityActor* Entity)
{
	if (Entity == nullptr)
	{
		return;
	}
	if (TWeakPtr<SWindow>* Existing = GOpenTransformDialogs.Find(Entity))
	{
		if (TSharedPtr<SWindow> Win = Existing->Pin())
		{
			Win->BringToFront();
			FSlateApplication::Get().SetKeyboardFocus(Win);
			return;
		}
	}

	TWeakObjectPtr<AAerotelicEntityActor> Weak = Entity;

	auto Field = [Weak](int32 Index, double Delta, double Min, double Max)
	{
		return SNew(SAerotelicNumericEntry)
			.Value_Lambda([Weak, Index]() { return GetPlacementValue(Weak.Get(), Index); })
			.MinValue(Min).MaxValue(Max).Delta(Delta)
			.LiveUpdate(false)
			.OnValueCommitted_Lambda([Weak, Index](double V, ETextCommit::Type) { SetPlacementValue(Weak.Get(), Index, V); });
	};
	auto Row = [](const FText& Label, TSharedRef<SWidget> Widget)
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 2, 8, 2)
			[ SNew(SBox).WidthOverride(28) [ SNew(STextBlock).Text(Label) ] ]
			+ SHorizontalBox::Slot().FillWidth(1).Padding(1) [ Widget ];
	};

	const FVector2D Size(340, 330);
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(FText::Format(LOCTEXT("Title", "Transform -- {0}"), FText::FromString(Entity->EntityName)))
		.ClientSize(Size)
		.SupportsMaximize(false)
		[
			SNew(SBorder).Padding(12)
			[
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 4) [ Heading(LOCTEXT("Translation", "Translation")) ]
				+ SVerticalBox::Slot().AutoHeight() [ Row(LOCTEXT("X", "X:"), Field(TX, 0.1, -1e9, 1e9)) ]
				+ SVerticalBox::Slot().AutoHeight() [ Row(LOCTEXT("Y", "Y:"), Field(TY, 0.1, -1e9, 1e9)) ]
				+ SVerticalBox::Slot().AutoHeight() [ Row(LOCTEXT("Z", "Z:"), Field(TZ, 0.1, -1e9, 1e9)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 4) [ Heading(LOCTEXT("Rotation", "Rotation")) ]
				+ SVerticalBox::Slot().AutoHeight() [ Row(LOCTEXT("AxisX", "X:"), Field(AX, 0.01, -1.0, 1.0)) ]
				+ SVerticalBox::Slot().AutoHeight() [ Row(LOCTEXT("AxisY", "Y:"), Field(AY, 0.01, -1.0, 1.0)) ]
				+ SVerticalBox::Slot().AutoHeight() [ Row(LOCTEXT("AxisZ", "Z:"), Field(AZ, 0.01, -1.0, 1.0)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0) [ Row(LOCTEXT("Angle", "deg:"), Field(Angle, 1.0, -360.0, 360.0)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
				[
					SNew(STextBlock).AutoWrapText(true)
					.ColorAndOpacity(FSlateColor(FLinearColor(0.55f, 0.55f, 0.55f)))
					.Text(LOCTEXT("Hint", "Rotation about the axis (normalised) is applied before the translation. Edits apply when a field is committed."))
				]
			]
		];

	GOpenTransformDialogs.Add(Entity, Window);
	ShowAerotelicDialogWindow(Window, Size);
}

#undef LOCTEXT_NAMESPACE
