#include "AerotelicTransformDialog.h"

#include "Widgets/SWindow.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
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

	// What the dialog edits: a DRAFT of the entity's placement. Every field
	// reads and writes this, never the entity; Apply hands it over
	// (ApplyPlacement, which is when the entity moves, the booleans over
	// it rebuild and the lattices over it regenerate) and Revert re-reads
	// the entity's. The dialog is modeless, so this lives in a shared
	// object the widgets' lambdas own, not on the stack of the function
	// that opened the window.
	struct FTransformDraft
	{
		Placement P;
		bool bDirty = false;
	};

	// After any edit: keep the axis and angle the geometry would use in
	// step with the Euler triple (when that is the authored form), exactly
	// as ApplyPlacement does on the entity, so the "Composes to" read-out
	// and a later switch of approach see what Apply will see.
	void Touched(FTransformDraft& D)
	{
		aerotelic::SyncPlacementRotation(D.P);
		D.bDirty = true;
	}

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

	// Which approach the Rotation tab is showing. Not a separate dialog
	// setting: it is the placement's own record of how its rotation was
	// authored (applied with the rest, saved with the project), so
	// re-opening the dialog comes back to the same tab.
	bool UsingEuler(const Placement& P)
	{
		return P.euler.used;
	}

	double GetPlacementValue(const Placement& P, int32 Field)
	{
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

	void SetPlacementValue(FTransformDraft& D, int32 Field, double Value)
	{
		// Belt and braces: under the Euler approach the axis and angle are
		// derived, so a write here would be overwritten by the next sync.
		// The fields are not on screen in that mode, so this cannot fire.
		if (Field >= AX && D.P.euler.used) return;
		switch (Field)
		{
		case TX: D.P.translation[0] = Value; break;
		case TY: D.P.translation[1] = Value; break;
		case TZ: D.P.translation[2] = Value; break;
		case AX: D.P.rotation_axis[0] = Value; break;
		case AY: D.P.rotation_axis[1] = Value; break;
		case AZ: D.P.rotation_axis[2] = Value; break;
		case Angle: D.P.rotation_deg = Value; break;
		}
		Touched(D);
	}

	// What the Euler rows show. Once the triple is the authored rotation it
	// is simply those three numbers; before that it is the rotation
	// actually in effect, decomposed into the chosen order -- switching
	// approach and finding three zeros beside a live rotation would be the
	// same kind of lie the axis fields used to tell.
	std::array<double, 3> DisplayedEulerAngles(const Placement& P)
	{
		if (P.euler.used) return P.euler.angles_deg;
		return aerotelic::EulerAnglesFromMatrix(
			aerotelic::RotationMatrix(P.rotation_axis, P.rotation_deg), P.euler.order, P.euler.intrinsic);
	}

	void SetEulerAngle(FTransformDraft& D, int32 Index, double Value)
	{
		D.P.euler.angles_deg = DisplayedEulerAngles(D.P);
		D.P.euler.angles_deg[Index] = Value;
		Touched(D);
	}

	// Changing the order or the frame keeps the three angles and turns the
	// entity differently: the user is declaring which convention their
	// numbers are in, not re-reading the same orientation. (Under the
	// axis/angle approach there is nothing to keep -- the rotation stays
	// put and the Euler rows, if the user switches, re-derive.)
	void SetEulerConvention(FTransformDraft& D, EulerOrder Order, bool bIntrinsic)
	{
		D.P.euler.order = Order;
		D.P.euler.intrinsic = bIntrinsic;
		Touched(D);
	}

	// Switching approach never moves the entity. To Euler: adopt the three
	// angles the rows are already showing. To axis/angle: keep the axis and
	// angle those composed to, which the placement already holds.
	void SetApproach(FTransformDraft& D, bool bEuler)
	{
		if (D.P.euler.used == bEuler) return;
		if (bEuler) D.P.euler.angles_deg = DisplayedEulerAngles(D.P);
		D.P.euler.used = bEuler;
		Touched(D);
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
	// draft, not in the widget: the displayed text is read back every
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
		"Translation is where the entity's origin goes. The rotation is applied first, about the origin, and the translation after it, so an entity turns in place and then moves. The two have a tab each.\n\n"
		"APPLY AND REVERT\n"
		"The fields edit a copy of the placement held by this dialog: nothing moves until Apply, which hands the copy to the entity -- the entity moves, every boolean over it rebuilds and every lattice over it regenerates. Revert puts the fields back to the placement the entity has. Closing the dialog with edits unapplied discards them.\n\n"
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
	TSharedRef<FTransformDraft> Draft = MakeShared<FTransformDraft>();
	Draft->P = Entity->GetPlacement();

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
	auto Field = [Draft](int32 Index, double Delta, double Min, double Max)
	{
		return SNew(SAerotelicNumericEntry)
			.Value_Lambda([Draft, Index]() { return GetPlacementValue(Draft->P, Index); })
			.MinValue(Min).MaxValue(Max).Delta(Delta)
			.LiveUpdate(false)
			.OnValueCommitted_Lambda([Draft, Index](double V, ETextCommit::Type) { SetPlacementValue(*Draft, Index, V); });
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
	auto CurrentOrder = [Draft]()
	{
		return Draft->P.euler.order;
	};
	auto AngleRow = [Draft, CurrentOrder](int32 Index)
	{
		return FormRow(TAttribute<FText>::CreateLambda([CurrentOrder, Index]() { return AngleRowLabel(CurrentOrder(), Index); }),
			SNew(SAerotelicNumericEntry)
			.Value_Lambda([Draft, Index]() { return DisplayedEulerAngles(Draft->P)[Index]; })
			.MinValue(-360.0).MaxValue(360.0).Delta(1.0)
			.LiveUpdate(false)
			.OnValueCommitted_Lambda([Draft, Index](double V, ETextCommit::Type) { SetEulerAngle(*Draft, Index, V); }));
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
				[Draft](int32 Index)
				{
					SetEulerConvention(*Draft, kOrders[Index], Draft->P.euler.intrinsic);
				}))
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			FormRow(LOCTEXT("Frame", "Axes:"), ChoiceCombo(FrameLabels,
				[Draft]() { return Draft->P.euler.intrinsic ? 0 : 1; },
				[Draft, CurrentOrder](int32 Index) { SetEulerConvention(*Draft, CurrentOrder(), Index == 0); }))
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 4) [ Heading(LOCTEXT("Angles", "Angles (degrees)")) ]
		+ SVerticalBox::Slot().AutoHeight() [ AngleRow(0) ]
		+ SVerticalBox::Slot().AutoHeight() [ AngleRow(1) ]
		+ SVerticalBox::Slot().AutoHeight() [ AngleRow(2) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 0)
		[
			MakeMetricRow(LOCTEXT("Equivalent", "Composes to:"), TAttribute<FText>::CreateLambda([Draft]()
			{
				const Placement& P = Draft->P;
				aerotelic::EulerRotation E;
				E.used = true;
				E.intrinsic = P.euler.intrinsic;
				E.order = P.euler.order;
				E.angles_deg = DisplayedEulerAngles(P);
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
				[Draft]() { return UsingEuler(Draft->P) ? 1 : 0; },
				[Draft](int32 Index) { SetApproach(*Draft, Index == 1); }))
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
		[
			MakeInfoLabel(LOCTEXT("ApproachHint",
				"Both describe the same rotation and switching does not move the entity: each opens on the rotation already in effect, expressed its way."))
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 0)
		[
			OnlyWhen(TAttribute<EVisibility>::CreateLambda([Draft]()
				{ return UsingEuler(Draft->P) ? EVisibility::Collapsed : EVisibility::Visible; }), AxisAnglePanel)
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 0)
		[
			OnlyWhen(TAttribute<EVisibility>::CreateLambda([Draft]()
				{ return UsingEuler(Draft->P) ? EVisibility::Visible : EVisibility::Collapsed; }), EulerPanel)
		];

	TArray<FAerotelicTab> Tabs;
	Tabs.Add({ LOCTEXT("TranslationTab", "Translation"), TranslationTab });
	Tabs.Add({ LOCTEXT("RotationTab", "Rotation"), RotationTab });
	Tabs.Add({ LOCTEXT("HelpTab", "Help"), MakeHelpTab(kHelp) });

	// ------------------------------------------------- Apply / Revert
	// The only way the draft reaches the entity. Apply re-reads the
	// placement afterwards: ApplyPlacement stores it as given, so this is
	// a no-op today, but the dialog must show what the entity holds.
	auto IsDirty = [Draft]() { return Draft->bDirty; };
	TSharedRef<SWidget> Footer =
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center).Padding(0, 0, 8, 0)
		[
			SNew(STextBlock)
			.AutoWrapText(true)
			.ColorAndOpacity(FSlateColor(FLinearColor(0.6f, 0.6f, 0.6f)))
			.Text_Lambda([IsDirty, Weak]()
			{
				if (!Weak.IsValid()) return LOCTEXT("Gone", "The entity is no longer in the scene.");
				return IsDirty()
					? LOCTEXT("Edited", "Edited here only -- press Apply to move the entity.")
					: LOCTEXT("Clean", "As the entity is placed.");
			})
		]
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 6, 0)
		[
			SNew(SButton)
			.Text(LOCTEXT("Revert", "Revert"))
			.ToolTipText(LOCTEXT("RevertTip", "Put the fields back to the placement the entity has."))
			.IsEnabled_Lambda([IsDirty, Weak]() { return IsDirty() && Weak.IsValid(); })
			.OnClicked_Lambda([Draft, Weak]()
			{
				if (Weak.IsValid()) Draft->P = Weak->GetPlacement();
				Draft->bDirty = false;
				return FReply::Handled();
			})
		]
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
		[
			SNew(SButton)
			.Text(LOCTEXT("Apply", "Apply"))
			.ToolTipText(LOCTEXT("ApplyTip", "Move the entity to this placement: every boolean over it rebuilds and every lattice over it regenerates."))
			.IsEnabled_Lambda([IsDirty, Weak]() { return IsDirty() && Weak.IsValid(); })
			.OnClicked_Lambda([Draft, Weak]()
			{
				if (!Weak.IsValid()) return FReply::Handled();
				Weak->ApplyPlacement(Draft->P);
				Draft->P = Weak->GetPlacement();
				Draft->bDirty = false;
				return FReply::Handled();
			})
		];

	const FVector2D Size(430, 520);
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(FText::Format(LOCTEXT("Title", "Transform -- {0}"), FText::FromString(Entity->EntityName)))
		.ClientSize(Size)
		.SupportsMaximize(false)
		[
			SNew(SBorder).Padding(12)
			[
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().FillHeight(1.0f)
				[
					SNew(SAerotelicTabStrip, Tabs)
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
				[
					Footer
				]
			]
		];

	GOpenTransformDialogs.Add(Entity, Window);
	ShowAerotelicDialogWindow(Window, Size);
}

#undef LOCTEXT_NAMESPACE
