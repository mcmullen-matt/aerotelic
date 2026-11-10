#include "AerotelicMirrorDialog.h"

#include "Widgets/SWindow.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SComboBox.h"
#include "Framework/Application/SlateApplication.h"
#include "Styling/CoreStyle.h"

#include "Entity/AerotelicEntityActor.h"
#include "Widgets/NumericEntry.h"
#include "Widgets/TabStrip.h"
#include "Widgets/CurveTabControls.h"
#include "Widgets/AerotelicDialogWindow.h"

#define LOCTEXT_NAMESPACE "AerotelicMirrorDialog"

namespace
{
	TMap<TWeakObjectPtr<AAerotelicEntityActor>, TWeakPtr<SWindow>> GOpenMirrorDialogs;

	// What the dialog edits: a DRAFT of the entity's mirror (on or off, and
	// the plane). Every field reads and writes this, never the entity; Apply
	// hands it over (SetMirrorPlane: the copy redraws and the booleans over
	// it rebuild) and Revert re-reads the entity's. Modeless, so it lives in
	// a shared object the widgets' lambdas own.
	struct FMirrorDraft
	{
		bool bMirrored = false;
		FAerotelicMirrorPlane Plane;
		bool bDirty = false;
	};

	// The three planes of the axes and anything else.
	enum EPlaneChoice : int32 { PlaneYZ = 0, PlaneXZ, PlaneXY, PlaneCustom };

	int32 PlaneChoiceOf(const FVector& N)
	{
		if (N.Y == 0.0 && N.Z == 0.0 && N.X != 0.0) return PlaneYZ;
		if (N.X == 0.0 && N.Z == 0.0 && N.Y != 0.0) return PlaneXZ;
		if (N.X == 0.0 && N.Y == 0.0 && N.Z != 0.0) return PlaneXY;
		return PlaneCustom;
	}

	TSharedRef<SWidget> Heading(const FText& Label)
	{
		return SNew(STextBlock).Text(Label).Font(FCoreStyle::GetDefaultFontStyle("Bold", 10));
	}

	TSharedRef<SWidget> FormRow(const FText& Label, TSharedRef<SWidget> Control)
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 2, 8, 2)
			[ SNew(SBox).WidthOverride(110) [ SNew(STextBlock).Text(Label) ] ]
			+ SHorizontalBox::Slot().FillWidth(1).Padding(1) [ Control ];
	}

	// A combo over fixed labels whose selection lives in the draft (see the
	// Transform dialog's note on the options' lifetime).
	TSharedRef<SWidget> ChoiceCombo(const TArray<FText>& Labels, TFunction<int32()> GetIndex, TFunction<void(int32)> SetIndex)
	{
		TSharedRef<TArray<TSharedPtr<FString>>> Options = MakeShared<TArray<TSharedPtr<FString>>>();
		for (const FText& Label : Labels) Options->Add(MakeShared<FString>(Label.ToString()));
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
			[ SNew(STextBlock).Text_Lambda([At, GetIndex]() { return FText::FromString(*At(GetIndex())); }) ];
	}

	const FText kHelp = LOCTEXT("Help",
		"WHAT THE MIRROR IS\n"
		"A reflected copy of this entity in a plane: the entity as placed in the scene, reflected point for point, its generated structure (ribs, spars, stringers) with it. The copy is DERIVED -- it is never edited on its own and follows every edit, move and regeneration of the entity -- and is a body of its own everywhere else: the Scene tree lists it as \"<entity>-<n>-mirror\", a boolean takes it as an operand (\"<entity> (mirror)\"), the exports write it, the mass report and the analyses weigh and solve it.\n\n"
		"THE PLANE\n"
		"Plane picks one of the three planes of the axes -- YZ (reflecting X, nose to tail), XZ (reflecting Y, the aircraft's symmetry plane, the default), XY (reflecting Z, up and down) -- or Custom, with any normal typed as three numbers (a direction, kept as typed). Offset moves the plane along its normal from the origin, in project units: the plane is normal . p = offset, so XZ with offset 2 is y = 2. The offset scales with the model like a placement.\n\n"
		"A SYMMETRIC WING\n"
		"A wing whose Symmetric option is on is already one body tip to tip; mirroring it in y = 0 would lay a copy over itself. Mirror is for the half wing, and for any other body: a boom, a nacelle, a fin, a propeller (which comes out the opposite hand).\n\n"
		"APPLY AND REVERT\n"
		"The fields edit a copy held by this dialog: nothing changes until Apply, which hands the copy to the entity -- the mirrored half redraws and every boolean over it rebuilds. Revert puts the fields back to what the entity has. Closing the dialog with edits unapplied discards them.");
}

void OpenAerotelicMirrorDialog(AAerotelicEntityActor* Entity)
{
	if (Entity == nullptr)
	{
		return;
	}
	if (TWeakPtr<SWindow>* Existing = GOpenMirrorDialogs.Find(Entity))
	{
		if (TSharedPtr<SWindow> Win = Existing->Pin())
		{
			Win->BringToFront();
			FSlateApplication::Get().SetKeyboardFocus(Win);
			return;
		}
	}

	TWeakObjectPtr<AAerotelicEntityActor> Weak = Entity;
	TSharedRef<FMirrorDraft> Draft = MakeShared<FMirrorDraft>();
	Draft->bMirrored = Entity->IsMirrored();
	Draft->Plane = Entity->GetMirrorPlane();

	const TArray<FText> PlaneLabels = {
		LOCTEXT("PlaneYZ", "YZ plane  (normal X: reflects nose to tail)"),
		LOCTEXT("PlaneXZ", "XZ plane  (normal Y: the symmetry plane)"),
		LOCTEXT("PlaneXY", "XY plane  (normal Z: reflects up and down)"),
		LOCTEXT("PlaneCustom", "Custom normal") };

	auto NormalField = [Draft](int32 Axis)
	{
		return SNew(SAerotelicNumericEntry)
			.Value_Lambda([Draft, Axis]() { return Draft->Plane.Normal[Axis]; })
			.MinValue(-1.0e9).MaxValue(1.0e9).Delta(0.01)
			.LiveUpdate(false)
			.OnValueCommitted_Lambda([Draft, Axis](double V, ETextCommit::Type)
			{
				if (Draft->Plane.Normal[Axis] == V) return;
				Draft->Plane.Normal[Axis] = V;
				Draft->bDirty = true;
			});
	};
	auto CustomVisible = TAttribute<EVisibility>::CreateLambda([Draft]() { return PlaneChoiceOf(Draft->Plane.Normal) == PlaneCustom ? EVisibility::Visible : EVisibility::Collapsed; });

	TSharedRef<SWidget> PlaneTab =
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
		[
			SNew(SCheckBox)
			.IsChecked_Lambda([Draft]() { return Draft->bMirrored ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
			.OnCheckStateChanged_Lambda([Draft](ECheckBoxState S)
			{
				const bool bOn = S == ECheckBoxState::Checked;
				if (Draft->bMirrored == bOn) return;
				Draft->bMirrored = bOn;
				Draft->bDirty = true;
			})
			[ SNew(STextBlock).Text(LOCTEXT("Mirrored", "Mirrored: a reflected copy of this entity, derived from it")) ]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 4) [ Heading(LOCTEXT("PlaneHeading", "Plane of reflection")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[
			FormRow(LOCTEXT("Plane", "Plane:"), ChoiceCombo(PlaneLabels,
				[Draft]() { return PlaneChoiceOf(Draft->Plane.Normal); },
				[Draft](int32 Index)
				{
					FVector N = Draft->Plane.Normal;
					switch (Index)
					{
					case PlaneYZ: N = FVector(1.0, 0.0, 0.0); break;
					case PlaneXZ: N = FVector(0.0, 1.0, 0.0); break;
					case PlaneXY: N = FVector(0.0, 0.0, 1.0); break;
					default:
						// Custom: keep the normal, nudged off an axis so the
						// combo stays on Custom while the fields are typed.
						if (PlaneChoiceOf(N) != PlaneCustom) N = FVector(N.X != 0.0 ? N.X : 0.0, N.Y != 0.0 ? N.Y : 0.0, N.Z != 0.0 ? N.Z : 0.0) + FVector(0.0, 0.0, 0.0);
						if (PlaneChoiceOf(N) != PlaneCustom) N = FVector(0.0, 1.0, 1.0);
						break;
					}
					if (Draft->Plane.Normal.Equals(N, 0.0)) return;
					Draft->Plane.Normal = N;
					Draft->bDirty = true;
				}))
		]
		+ SVerticalBox::Slot().AutoHeight()
		[ SNew(SBox).Visibility(CustomVisible) [ FormRow(LOCTEXT("NormalX", "Normal X:"), NormalField(0)) ] ]
		+ SVerticalBox::Slot().AutoHeight()
		[ SNew(SBox).Visibility(CustomVisible) [ FormRow(LOCTEXT("NormalY", "Normal Y:"), NormalField(1)) ] ]
		+ SVerticalBox::Slot().AutoHeight()
		[ SNew(SBox).Visibility(CustomVisible) [ FormRow(LOCTEXT("NormalZ", "Normal Z:"), NormalField(2)) ] ]
		+ SVerticalBox::Slot().AutoHeight()
		[
			FormRow(LOCTEXT("Offset", "Offset along normal:"),
				SNew(SAerotelicNumericEntry)
				.Value_Lambda([Draft]() { return Draft->Plane.Offset; })
				.MinValue(-1.0e9).MaxValue(1.0e9).Delta(0.1)
				.LiveUpdate(false)
				.OnValueCommitted_Lambda([Draft](double V, ETextCommit::Type)
				{
					if (Draft->Plane.Offset == V) return;
					Draft->Plane.Offset = V;
					Draft->bDirty = true;
				}))
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
		[
			MakeMetricRow(LOCTEXT("PlaneIs", "The plane:"), TAttribute<FText>::CreateLambda([Draft]()
			{
				const FVector N = Draft->Plane.Normal;
				const double O = Draft->Plane.Offset;
				switch (PlaneChoiceOf(N))
				{
				case PlaneYZ: return FText::Format(LOCTEXT("PlaneX", "x = {0}"), FormatG(O / N.X, 5));
				case PlaneXZ: return FText::Format(LOCTEXT("PlaneY", "y = {0}"), FormatG(O / N.Y, 5));
				case PlaneXY: return FText::Format(LOCTEXT("PlaneZ", "z = {0}"), FormatG(O / N.Z, 5));
				default:
					if (N.IsNearlyZero(0.0)) return LOCTEXT("PlaneNone", "no plane: the normal is zero");
					return FText::Format(LOCTEXT("PlaneGeneral", "({0}, {1}, {2}) . p = {3}"), FormatG(N.X, 4), FormatG(N.Y, 4), FormatG(N.Z, 4), FormatG(O, 5));
				}
			}))
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 0)
		[
			MakeInfoLabel(LOCTEXT("PlaneHint",
				"The copy is the entity as placed, reflected in this plane, its generated structure with it. XZ at offset 0 is the aircraft's symmetry plane y = 0. The offset is in project units and scales with the model."))
		];

	TArray<FAerotelicTab> Tabs;
	Tabs.Add({ LOCTEXT("PlaneTab", "Mirror"), PlaneTab });
	Tabs.Add({ LOCTEXT("HelpTab", "Help"), MakeHelpTab(kHelp) });

	auto IsDirty = [Draft]() { return Draft->bDirty; };
	auto CanApply = [Draft, Weak]() { return Draft->bDirty && Weak.IsValid() && !Draft->Plane.Normal.IsNearlyZero(0.0); };
	TSharedRef<SWidget> Footer =
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center).Padding(0, 0, 8, 0)
		[
			SNew(STextBlock)
			.AutoWrapText(true)
			.ColorAndOpacity(FSlateColor(FLinearColor(0.6f, 0.6f, 0.6f)))
			.Text_Lambda([Draft, IsDirty, Weak]()
			{
				if (!Weak.IsValid()) return LOCTEXT("Gone", "The entity is no longer in the scene.");
				if (Draft->Plane.Normal.IsNearlyZero(0.0)) return LOCTEXT("ZeroNormal", "The normal is zero: give the plane a direction.");
				return IsDirty()
					? LOCTEXT("Edited", "Edited here only -- press Apply to change the mirror.")
					: (Draft->bMirrored ? LOCTEXT("CleanOn", "As the entity is mirrored.") : LOCTEXT("CleanOff", "The entity is not mirrored."));
			})
		]
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 6, 0)
		[
			SNew(SButton)
			.Text(LOCTEXT("Revert", "Revert"))
			.ToolTipText(LOCTEXT("RevertTip", "Put the fields back to the mirror the entity has."))
			.IsEnabled_Lambda([IsDirty, Weak]() { return IsDirty() && Weak.IsValid(); })
			.OnClicked_Lambda([Draft, Weak]()
			{
				if (Weak.IsValid())
				{
					Draft->bMirrored = Weak->IsMirrored();
					Draft->Plane = Weak->GetMirrorPlane();
				}
				Draft->bDirty = false;
				return FReply::Handled();
			})
		]
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
		[
			SNew(SButton)
			.Text(LOCTEXT("Apply", "Apply"))
			.ToolTipText(LOCTEXT("ApplyTip", "Give the entity this mirror: the reflected copy redraws (or goes) and every boolean over it rebuilds."))
			.IsEnabled_Lambda(CanApply)
			.OnClicked_Lambda([Draft, Weak]()
			{
				if (!Weak.IsValid()) return FReply::Handled();
				Weak->SetMirrorPlane(Draft->Plane, Draft->bMirrored);
				Draft->bMirrored = Weak->IsMirrored();
				Draft->Plane = Weak->GetMirrorPlane();
				Draft->bDirty = false;
				return FReply::Handled();
			})
		];

	const FVector2D Size(460, 400);
	TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(FText::Format(LOCTEXT("Title", "Mirror -- {0}"), FText::FromString(Entity->EntityName)))
		.ClientSize(Size)
		.SupportsMaximize(false)
		[
			SNew(SBorder).Padding(12)
			[
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().FillHeight(1.0f) [ SNew(SAerotelicTabStrip, Tabs) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0) [ Footer ]
			]
		];

	GOpenMirrorDialogs.Add(Entity, Window);
	ShowAerotelicDialogWindow(Window, Size);
}

#undef LOCTEXT_NAMESPACE
