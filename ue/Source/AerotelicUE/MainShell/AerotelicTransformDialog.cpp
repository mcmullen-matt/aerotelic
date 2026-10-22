#include "AerotelicTransformDialog.h"

#include "Widgets/SWindow.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SComboBox.h"
#include "Framework/Application/SlateApplication.h"
#include "Styling/CoreStyle.h"

#include "Entity/AerotelicEntityActor.h"
#include "Widgets/NumericEntry.h"
#include "Widgets/TabStrip.h"
#include "Widgets/CurveTabControls.h"
#include "Widgets/AerotelicDialogWindow.h"

#define LOCTEXT_NAMESPACE "AerotelicTransformDialog"

using aerotelic::EulerOrder;
using aerotelic::Placement;

namespace
{
	TMap<TWeakObjectPtr<AAerotelicEntityActor>, TWeakPtr<SWindow>> GOpenTransformDialogs;

	enum EPlacementField : int32 { TX = 0, TY, TZ, AX, AY, AZ, Angle };

	// Yaw-pitch-roll first: the order an aircraft is actually described in.
	const EulerOrder kOrders[6] = { EulerOrder::ZYX, EulerOrder::XYZ, EulerOrder::XZY,
		EulerOrder::YXZ, EulerOrder::YZX, EulerOrder::ZXY };

	FText OrderLabel(EulerOrder Order)
	{
		switch (Order)
		{
		case EulerOrder::ZYX: return LOCTEXT("OrderZYX", "Z, Y, X  (yaw, pitch, roll)");
		case EulerOrder::XYZ: return LOCTEXT("OrderXYZ", "X, Y, Z");
		case EulerOrder::XZY: return LOCTEXT("OrderXZY", "X, Z, Y");
		case EulerOrder::YXZ: return LOCTEXT("OrderYXZ", "Y, X, Z");
		case EulerOrder::YZX: return LOCTEXT("OrderYZX", "Y, Z, X");
		case EulerOrder::ZXY: return LOCTEXT("OrderZXY", "Z, X, Y");
		}
		return FText::GetEmpty();
	}

	// "1st (Z):" and so on: which axis a given angle turns about depends on
	// the chosen order, so the row labels follow it.
	FText AngleRowLabel(EulerOrder Order, int32 Index)
	{
		static const FText kOrdinals[3] = { LOCTEXT("First", "1st"), LOCTEXT("Second", "2nd"), LOCTEXT("Third", "3rd") };
		static const FText kAxes[3] = { LOCTEXT("AxisNameX", "X"), LOCTEXT("AxisNameY", "Y"), LOCTEXT("AxisNameZ", "Z") };
		const int32 AxisIndex = aerotelic::EulerOrderAxes(Order)[Index];
		return FText::Format(LOCTEXT("AngleRow", "{0} ({1}):"), kOrdinals[Index], kAxes[AxisIndex]);
	}

	// Which approach the Rotation tab is showing. This is not dialog state:
	// it is the entity's own record of how its rotation was authored, so
	// re-opening the dialog (or the project) comes back to the same tab.
	bool UsingEuler(AAerotelicEntityActor* Entity)
	{
		return Entity != nullptr && Entity->GetPlacement().euler.used;
	}

	double GetPlacementValue(AAerotelicEntityActor* Entity, int32 Field)
	{
		if (Entity == nullptr) return 0.0;
		const Placement P = Entity->GetPlacement();
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
		Placement P = Entity->GetPlacement();
		// Belt and braces: under the Euler approach the axis and angle are
		// derived, so a write here would be overwritten by the next sync.
		// The fields are not on screen in that mode, so this cannot fire.
		if (Field >= AX && P.euler.used) return;
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

	// What the Euler rows show. Once the triple is the authored rotation it
	// is simply those three numbers; before that it is the rotation
	// actually in effect, decomposed into the chosen order -- switching
	// approach and finding three zeros beside a live rotation would be the
	// same kind of lie the axis fields used to tell.
	std::array<double, 3> DisplayedEulerAngles(AAerotelicEntityActor* Entity)
	{
		if (Entity == nullptr) return {0.0, 0.0, 0.0};
		const Placement P = Entity->GetPlacement();
		if (P.euler.used) return P.euler.angles_deg;
		return aerotelic::EulerAnglesFromMatrix(
			aerotelic::RotationMatrix(P.rotation_axis, P.rotation_deg), P.euler.order, P.euler.intrinsic);
	}

	void SetEulerAngle(AAerotelicEntityActor* Entity, int32 Index, double Value)
	{
		if (Entity == nullptr) return;
		Placement P = Entity->GetPlacement();
		P.euler.angles_deg = DisplayedEulerAngles(Entity);
		P.euler.angles_deg[Index] = Value;
		Entity->ApplyPlacement(P);
	}

	// Changing the order or the frame keeps the three angles and turns the
	// entity differently: the user is declaring which convention their
	// numbers are in, not re-reading the same orientation. (Under the
	// axis/angle approach there is nothing to keep -- the rotation stays
	// put and the Euler rows, if the user switches, re-derive.)
	void SetEulerConvention(AAerotelicEntityActor* Entity, EulerOrder Order, bool bIntrinsic)
	{
		if (Entity == nullptr) return;
		Placement P = Entity->GetPlacement();
		P.euler.order = Order;
		P.euler.intrinsic = bIntrinsic;
		Entity->ApplyPlacement(P);
	}

	// Switching approach never moves the entity. To Euler: adopt the three
	// angles the rows are already showing. To axis/angle: keep the axis and
	// angle those composed to, which the placement already holds.
	void SetApproach(AAerotelicEntityActor* Entity, bool bEuler)
	{
		if (Entity == nullptr) return;
		Placement P = Entity->GetPlacement();
		if (P.euler.used == bEuler) return;
		if (bEuler) P.euler.angles_deg = DisplayedEulerAngles(Entity);
		P.euler.used = bEuler;
		Entity->ApplyPlacement(P);
	}

	TSharedRef<SWidget> Heading(const FText& Label)
	{
		return SNew(STextBlock).Text(Label).Font(FCoreStyle::GetDefaultFontStyle("Bold", 10));
	}

	TSharedRef<SWidget> OnlyWhen(TAttribute<EVisibility> Vis, TSharedRef<SWidget> Content)
	{
		return SNew(SBox).Visibility(Vis) [ Content ];
	}

	// "Label:" on the left, the control filling the rest -- every row in
	// this dialog, whether it holds a number or a combo.
	TSharedRef<SWidget> FormRow(TAttribute<FText> Label, TSharedRef<SWidget> Control)
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 2, 8, 2)
			[ SNew(SBox).WidthOverride(62) [ SNew(STextBlock).Text(Label) ] ]
			+ SHorizontalBox::Slot().FillWidth(1).Padding(1) [ Control ];
	}

	// A combo over a fixed list of labels whose selection lives in the
	// entity, not in the widget: the displayed text is read back every
	// frame, so nothing can drift out of step with the placement.
	//
	// The options array is owned HERE and captured into the lambdas the
	// widget owns, so it lives exactly as long as the combo. SComboBox
	// keeps only a raw pointer to it and the text below indexes it on
	// every paint, so an array living in the caller's stack frame is a
	// use-after-free the moment the dialog function returns.
	TSharedRef<SWidget> ChoiceCombo(const TArray<FText>& Labels,
		TFunction<int32()> GetIndex, TFunction<void(int32)> SetIndex)
	{
		TSharedRef<TArray<TSharedPtr<FString>>> Options = MakeShared<TArray<TSharedPtr<FString>>>();
		for (const FText& Label : Labels)
		{
			Options->Add(MakeShared<FString>(Label.ToString()));
		}
		auto At = [Options](int32 Index) { return (*Options)[FMath::Clamp(Index, 0, Options->Num() - 1)]; };

		return SNew(SComboBox<TSharedPtr<FString>>)
			.OptionsSource(&Options.Get())
			.InitiallySelectedItem(At(GetIndex()))
			.OnGenerateWidget_Lambda([](TSharedPtr<FString> S) { return SNew(STextBlock).Text(FText::FromString(*S)); })
			.OnSelectionChanged_Lambda([Options, SetIndex](TSharedPtr<FString> S, ESelectInfo::Type)
			{
				if (!S.IsValid()) return;
				const int32 Index = Options->IndexOfByKey(S);
				if (Index != INDEX_NONE) SetIndex(Index);
			})
			[
				SNew(STextBlock).Text_Lambda([At, GetIndex]() { return FText::FromString(*At(GetIndex())); })
			];
	}

	const FText kHelp = LOCTEXT("Help",
		"WHERE THE ENTITY SITS\n"
		"Translation is where the entity's origin goes. The rotation is applied first, about the origin, and the translation after it, so an entity turns in place and then moves. The two have a tab each; edits apply when a field is committed.\n\n"
		"APPROACH\n"
		"The Rotation tab offers two ways to say the same thing, and the entity remembers which one you used -- the project file stores it, so the tab comes back the way you left it. Switching approach never moves the entity: each one opens on the rotation already in effect, expressed its way.\n\n"
		"AXIS AND ANGLE\n"
		"One turn of so many degrees about one axis. This is what the project file stores and what the geometry uses under either approach. The axis is a direction, not a unit vector: (0, 2, 1) and (0, 0.894, 0.447) are the same axis, and it is kept exactly as typed, so a 1 you type stays a 1. It is also kept while the angle is still zero, so you can set the axis first and wind the angle on afterwards.\n\n"
		"EULER ANGLES\n"
		"The same rotation as three turns in sequence, for an aircraft usually yaw then pitch then roll. Order lists the axes in the order the turns happen and the three angles run with it: the 1st row is the first turn. Axes chooses whether each turn is about the axes the earlier turns already moved (moving, the aircraft convention) or about the fixed world axes. Those are the same rotations read in opposite order, so moving Z,Y,X equals fixed X,Y,Z with the angles reversed. Changing Order or Axes keeps your three numbers and turns the entity differently -- you are declaring which convention they are in.\n"
		"\"Composes to\" is the single axis and angle the three turns amount to, which is what gets stored beside them.\n\n"
		"WHY BOTH ARE KEPT\n"
		"Three angles cannot be recovered from an axis and an angle uniquely, and at gimbal lock (middle angle at plus or minus 90 degrees) not at all: the first and third turns are then about the same axis and only their sum means anything, so a decomposition there puts the whole sum on the first row and leaves the third at zero. That is why the three angles are stored as well as the axis and angle they compose to -- and why, under the axis/angle approach, the Euler rows show only one of the decompositions that would do.");
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

	const TArray<FText> ApproachLabels = {
		LOCTEXT("ApproachAxis", "Axis and angle"), LOCTEXT("ApproachEuler", "Euler angles") };
	const TArray<FText> FrameLabels = {
		LOCTEXT("FrameMoving", "Moving (intrinsic)"), LOCTEXT("FrameFixed", "Fixed world axes") };
	TArray<FText> OrderLabels;
	for (EulerOrder Order : kOrders)
	{
		OrderLabels.Add(OrderLabel(Order));
	}

	// A placement field. Wide bounds throughout: an axis is a direction, so
	// the old -1..1 clamp only stopped honest ones being typed.
	auto Field = [Weak](int32 Index, double Delta, double Min, double Max)
	{
		return SNew(SAerotelicNumericEntry)
			.Value_Lambda([Weak, Index]() { return GetPlacementValue(Weak.Get(), Index); })
			.MinValue(Min).MaxValue(Max).Delta(Delta)
			.LiveUpdate(false)
			.OnValueCommitted_Lambda([Weak, Index](double V, ETextCommit::Type) { SetPlacementValue(Weak.Get(), Index, V); });
	};

	// --------------------------------------------------------- translation
	TSharedRef<SWidget> TranslationTab =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 4) [ Heading(LOCTEXT("Position", "Position of the entity's origin")) ]
		+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("X", "X:"), Field(TX, 0.1, -1.0e9, 1.0e9)) ]
		+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("Y", "Y:"), Field(TY, 0.1, -1.0e9, 1.0e9)) ]
		+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("Z", "Z:"), Field(TZ, 0.1, -1.0e9, 1.0e9)) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 0)
		[
			MakeInfoLabel(LOCTEXT("TranslationHint",
				"X is the fuselage axis, Y lateral, Z vertical. The rotation on the other tab is applied first, about the origin, so the entity turns in place and then moves here."))
		];

	// ---------------------------------------------- rotation: axis + angle
	TSharedRef<SWidget> AxisAnglePanel =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 4) [ Heading(LOCTEXT("Axis", "Axis to turn about")) ]
		+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("AxisX", "X:"), Field(AX, 0.01, -1.0e9, 1.0e9)) ]
		+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("AxisY", "Y:"), Field(AY, 0.01, -1.0e9, 1.0e9)) ]
		+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("AxisZ", "Z:"), Field(AZ, 0.01, -1.0e9, 1.0e9)) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 4) [ Heading(LOCTEXT("AngleHeading", "Angle")) ]
		+ SVerticalBox::Slot().AutoHeight() [ FormRow(LOCTEXT("Angle", "Degrees:"), Field(Angle, 1.0, -360.0, 360.0)) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 0)
		[
			MakeInfoLabel(LOCTEXT("AxisHint",
				"The axis is a direction, not a unit vector: it is kept exactly as typed, and kept while the angle is still zero, so you can set it first and wind the angle on afterwards."))
		];

	// --------------------------------------------------- rotation: euler
	auto CurrentOrder = [Weak]()
	{
		return Weak.IsValid() ? Weak->GetPlacement().euler.order : EulerOrder::ZYX;
	};
	auto AngleRow = [Weak, CurrentOrder](int32 Index)
	{
		return FormRow(TAttribute<FText>::CreateLambda([CurrentOrder, Index]() { return AngleRowLabel(CurrentOrder(), Index); }),
			SNew(SAerotelicNumericEntry)
			.Value_Lambda([Weak, Index]() { return DisplayedEulerAngles(Weak.Get())[Index]; })
			.MinValue(-360.0).MaxValue(360.0).Delta(1.0)
			.LiveUpdate(false)
			.OnValueCommitted_Lambda([Weak, Index](double V, ETextCommit::Type) { SetEulerAngle(Weak.Get(), Index, V); }));
	};

	TSharedRef<SWidget> EulerPanel =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 4) [ Heading(LOCTEXT("Sequence", "Sequence of turns")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[
			FormRow(LOCTEXT("Order", "Order:"), ChoiceCombo(OrderLabels,
				[CurrentOrder]()
				{
					for (int32 i = 0; i < 6; ++i) { if (kOrders[i] == CurrentOrder()) return i; }
					return 0;
				},
				[Weak](int32 Index)
				{
					const bool bIntrinsic = !Weak.IsValid() || Weak->GetPlacement().euler.intrinsic;
					SetEulerConvention(Weak.Get(), kOrders[Index], bIntrinsic);
				}))
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			FormRow(LOCTEXT("Frame", "Axes:"), ChoiceCombo(FrameLabels,
				[Weak]() { return (!Weak.IsValid() || Weak->GetPlacement().euler.intrinsic) ? 0 : 1; },
				[Weak, CurrentOrder](int32 Index) { SetEulerConvention(Weak.Get(), CurrentOrder(), Index == 0); }))
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 4) [ Heading(LOCTEXT("Angles", "Angles (degrees)")) ]
		+ SVerticalBox::Slot().AutoHeight() [ AngleRow(0) ]
		+ SVerticalBox::Slot().AutoHeight() [ AngleRow(1) ]
		+ SVerticalBox::Slot().AutoHeight() [ AngleRow(2) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 0)
		[
			MakeMetricRow(LOCTEXT("Equivalent", "Composes to:"), TAttribute<FText>::CreateLambda([Weak]()
			{
				if (!Weak.IsValid()) return DashText();
				const Placement P = Weak->GetPlacement();
				aerotelic::EulerRotation E;
				E.used = true;
				E.intrinsic = P.euler.intrinsic;
				E.order = P.euler.order;
				E.angles_deg = DisplayedEulerAngles(Weak.Get());
				const std::pair<std::array<double, 3>, double> AxisAngle = aerotelic::AxisAngleFromEuler(E);
				return FText::Format(LOCTEXT("EquivalentValue", "{0}° about ({1}, {2}, {3})"),
					FormatG(AxisAngle.second, 5), FormatG(AxisAngle.first[0], 4),
					FormatG(AxisAngle.first[1], 4), FormatG(AxisAngle.first[2], 4));
			}))
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
		[
			MakeInfoLabel(LOCTEXT("EulerHint",
				"The 1st row is the first turn. Moving axes means each turn is about the axes the earlier turns already moved -- the aircraft convention. Changing Order or Axes keeps these numbers and turns the entity differently."))
		];

	// ------------------------------------------------------ rotation tab
	TSharedRef<SWidget> RotationTab =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[
			FormRow(LOCTEXT("Approach", "Approach:"), ChoiceCombo(ApproachLabels,
				[Weak]() { return UsingEuler(Weak.Get()) ? 1 : 0; },
				[Weak](int32 Index) { SetApproach(Weak.Get(), Index == 1); }))
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
		[
			MakeInfoLabel(LOCTEXT("ApproachHint",
				"Both describe the same rotation and switching does not move the entity: each opens on the rotation already in effect, expressed its way."))
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 0)
		[
			OnlyWhen(TAttribute<EVisibility>::CreateLambda([Weak]()
				{ return UsingEuler(Weak.Get()) ? EVisibility::Collapsed : EVisibility::Visible; }), AxisAnglePanel)
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 0)
		[
			OnlyWhen(TAttribute<EVisibility>::CreateLambda([Weak]()
				{ return UsingEuler(Weak.Get()) ? EVisibility::Visible : EVisibility::Collapsed; }), EulerPanel)
		];

	TArray<FAerotelicTab> Tabs;
	Tabs.Add({ LOCTEXT("TranslationTab", "Translation"), TranslationTab });
	Tabs.Add({ LOCTEXT("RotationTab", "Rotation"), RotationTab });
	Tabs.Add({ LOCTEXT("HelpTab", "Help"), MakeHelpTab(kHelp) });

	const FVector2D Size(430, 470);
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(FText::Format(LOCTEXT("Title", "Transform -- {0}"), FText::FromString(Entity->EntityName)))
		.ClientSize(Size)
		.SupportsMaximize(false)
		[
			SNew(SBorder).Padding(12)
			[
				SNew(SAerotelicTabStrip, Tabs)
			]
		];

	GOpenTransformDialogs.Add(Entity, Window);
	ShowAerotelicDialogWindow(Window, Size);
}

#undef LOCTEXT_NAMESPACE
