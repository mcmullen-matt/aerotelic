#include "AerotelicMainShell.h"
#include "AerotelicCommands.h"
#include "AerotelicSceneTree.h"
#include "AerotelicDesignDock.h"
#include "AerotelicSurfaceDialog.h"
#include "AerotelicWelcomeScreen.h"
#include "AerotelicTutorialDialog.h"
#include "Branding/AerotelicBranding.h"

#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "EngineUtils.h"

#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SSpacer.h"
#include "Widgets/Layout/SSeparator.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SComboButton.h"
#include "Widgets/SBoxPanel.h"
#include "Framework/Commands/UICommandList.h"
#include "Framework/MultiBox/MultiBoxBuilder.h"
#include "Framework/Application/SlateApplication.h"
#include "Styling/CoreStyle.h"
#include "Misc/MessageDialog.h"
#include "HAL/PlatformMisc.h"

#include "Entity/AerotelicDocumentSubsystem.h"
#include "Entity/AerotelicEntityActor.h"
#include "Standalone/AerotelicCameraPawn.h"
#include "Standalone/AerotelicGridGround.h"
#include "Widgets/AerotelicDialogWindow.h"
#include "Widgets/InputSink.h"

#define LOCTEXT_NAMESPACE "AerotelicMainShell"

namespace
{
	// The ground lattice the game mode spawned (null in the Editor host).
	AAerotelicGridGround* FindGround()
	{
		UWorld* World = (GEngine != nullptr && GEngine->GameViewport != nullptr) ? GEngine->GameViewport->GetWorld() : nullptr;
		if (World == nullptr)
		{
			return nullptr;
		}
		for (TActorIterator<AAerotelicGridGround> It(World); It; ++It)
		{
			return *It;
		}
		return nullptr;
	}

	// Widths of the frame's columns. The Qt window (1600 wide by default)
	// gives the scene tree ~200 px and the design dock column a quarter of
	// the window. Keep kViewToolbarWidth + kSceneTreeWidth in sync with
	// kLeftColumnWidth in Widgets/AerotelicDialogWindow.cpp.
	constexpr float kViewToolbarWidth = 40.0f;
	// 280 (the Qt tree's maximum) rather than ~200: body and mirror rows
	// carry "<entity>-<n>-mirror" names plus a role annotation.
	constexpr float kSceneTreeWidth = 280.0f;
	constexpr float kIconSize = 30.0f;

	const FLinearColor kPanelBg(0.09f, 0.09f, 0.10f, 1.0f);
	const FLinearColor kBarBg(0.07f, 0.07f, 0.08f, 1.0f);

	FLinearColor Hex(const TCHAR* HexCode)
	{
		return FLinearColor(FColor::FromHex(HexCode));
	}

	FCanExecuteAction Never()
	{
		return FCanExecuteAction::CreateLambda([]() { return false; });
	}

	// A thin vertical grip on the dock column's left edge: dragging it
	// resizes the column (QDockWidget's separator). Dragging left widens.
	class SDockResizeHandle : public SCompoundWidget
	{
	public:
		SLATE_BEGIN_ARGS(SDockResizeHandle) {}
			SLATE_ARGUMENT(float*, Width)
		SLATE_END_ARGS()

		void Construct(const FArguments& InArgs)
		{
			Width = InArgs._Width;
			SetCursor(EMouseCursor::ResizeLeftRight);
			ChildSlot
			[
				SNew(SBox).WidthOverride(6)
				[
					SNew(SBorder)
					.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
					.BorderBackgroundColor_Lambda([this]() { return bDragging || IsHovered() ? FLinearColor(0.23f, 0.48f, 0.84f) : FLinearColor(0.12f, 0.13f, 0.16f); })
				]
			];
		}

		virtual FReply OnMouseButtonDown(const FGeometry& Geo, const FPointerEvent& Event) override
		{
			if (Event.GetEffectingButton() != EKeys::LeftMouseButton) return FReply::Unhandled();
			bDragging = true;
			DragStartX = Event.GetScreenSpacePosition().X;
			DragStartWidth = *Width;
			return FReply::Handled().CaptureMouse(SharedThis(this));
		}
		virtual FReply OnMouseMove(const FGeometry& Geo, const FPointerEvent& Event) override
		{
			if (!bDragging) return FReply::Unhandled();
			*Width = FMath::Clamp(DragStartWidth - (Event.GetScreenSpacePosition().X - DragStartX), 300.0f, 1000.0f);
			return FReply::Handled();
		}
		virtual FReply OnMouseButtonUp(const FGeometry& Geo, const FPointerEvent& Event) override
		{
			if (!bDragging) return FReply::Unhandled();
			bDragging = false;
			return FReply::Handled().ReleaseMouseCapture();
		}

	private:
		float* Width = nullptr;
		bool bDragging = false;
		float DragStartX = 0.f, DragStartWidth = 0.f;
	};
}

UAerotelicDocumentSubsystem* SAerotelicMainShell::GetDocument() const
{
	return UAerotelicDocumentSubsystem::Get();
}

// ------------------------------------------------------------ construction

void SAerotelicMainShell::Construct(const FArguments& InArgs)
{
	StatusText = LOCTEXT("StatusReady", "Ready");
	BindCommands();

	ChildSlot
	[
		SNew(SVerticalBox)
		.Visibility(EVisibility::SelfHitTestInvisible)

		// Menu bar + horizontal toolbar.
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SAerotelicInputSink)
			[
				SNew(SBorder)
				.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
				.BorderBackgroundColor(kBarBg)
				.Padding(0)
				[
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight() [ BuildMenuBar() ]
					+ SVerticalBox::Slot().AutoHeight() [ BuildEntityToolbar() ]
				]
			]
		]

		// Body: vertical toolbar | scene tree | 3D view hole | design dock.
		+ SVerticalBox::Slot().FillHeight(1.0f)
		[
			SNew(SHorizontalBox)
			.Visibility(EVisibility::SelfHitTestInvisible)

			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SAerotelicInputSink)
				[
					SNew(SBox).WidthOverride(kViewToolbarWidth)
					[
						SNew(SBorder)
						.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
						.BorderBackgroundColor(kBarBg)
						.Padding(FMargin(4, 6))
						[ BuildViewToolbar() ]
					]
				]
			]

			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SAerotelicInputSink)
				[
					SNew(SBox).WidthOverride(kSceneTreeWidth)
					[
						SNew(SBorder)
						.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
						.BorderBackgroundColor(kPanelBg)
						.Padding(FMargin(2))
						[
							SNew(SAerotelicSceneTree)
							.OnStatus(FOnAerotelicStatus::CreateSP(this, &SAerotelicMainShell::SetStatus))
						]
					]
				]
			]

			+ SHorizontalBox::Slot().FillWidth(1.0f)
			[
				SAssignNew(ViewHole, SSpacer)
				.Visibility(EVisibility::HitTestInvisible)
			]

			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SAerotelicInputSink)
				// The column takes no space while the Design tab is torn off
				// into its own window, so the 3D view spans to the right edge.
				// It reappears for the duration of any drag so a tab can be
				// dropped back into it.
				.Visibility_Lambda([]()
				{
					return IsAerotelicDesignTabDocked() || FSlateApplication::Get().IsDragDropping()
						? EVisibility::Visible : EVisibility::Collapsed;
				})
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().AutoWidth()
					[ SNew(SDockResizeHandle).Width(&DockColumnWidth) ]
					+ SHorizontalBox::Slot().AutoWidth()
					[
						SNew(SBox).WidthOverride_Lambda([this]() { return FOptionalSize(DockColumnWidth); })
						[
							SNew(SBorder)
							.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
							.BorderBackgroundColor(kPanelBg)
							.Padding(FMargin(2))
							[
								BuildAerotelicDesignDock(FindAerotelicMainWindow())
							]
						]
					]
				]
			]
		]

		// Status bar.
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SAerotelicInputSink)
			[ BuildStatusBar() ]
		]
	];

#if WITH_EDITOR
	KeyListenerHandle = FSlateApplication::Get().OnApplicationPreInputKeyDownListener().AddSP(this, &SAerotelicMainShell::OnPreKeyDown);
#endif
}

SAerotelicMainShell::~SAerotelicMainShell()
{
#if WITH_EDITOR
	if (FSlateApplication::IsInitialized())
	{
		FSlateApplication::Get().OnApplicationPreInputKeyDownListener().Remove(KeyListenerHandle);
	}
#endif
}

bool SAerotelicMainShell::ProcessShortcut(const FKey& Key, const FModifierKeysState& Modifiers, bool bRepeat)
{
	return Commands.IsValid() && Commands->ProcessCommandBindings(Key, Modifiers, bRepeat);
}

bool SAerotelicMainShell::GetViewHoleRect(FVector2D& OutAbsPos, FVector2D& OutAbsSize) const
{
	if (!ViewHole.IsValid())
	{
		return false;
	}
	const FGeometry& G = ViewHole->GetCachedGeometry();
	OutAbsPos = FVector2D(G.GetAbsolutePosition());
	OutAbsSize = FVector2D(G.GetAbsoluteSize());
	return OutAbsSize.X > 1.0 && OutAbsSize.Y > 1.0;
}

void SAerotelicMainShell::SetStatus(const FText& Text)
{
	StatusText = Text;
}

#if WITH_EDITOR
void SAerotelicMainShell::OnPreKeyDown(const FKeyEvent& KeyEvent)
{
	if (Commands.IsValid())
	{
		Commands->ProcessCommandBindings(KeyEvent);
	}
}
#endif

// ---------------------------------------------------------------- commands

void SAerotelicMainShell::BindCommands()
{
	const FAerotelicCommands& C = FAerotelicCommands::Get();
	Commands = MakeShared<FUICommandList>();

	// File
	Commands->MapAction(C.NewProject, FExecuteAction::CreateSP(this, &SAerotelicMainShell::OnNew));
	Commands->MapAction(C.Save, FExecuteAction::CreateSP(this, &SAerotelicMainShell::OnSave));
	Commands->MapAction(C.SaveAs, FExecuteAction::CreateSP(this, &SAerotelicMainShell::OnSaveAs));
	Commands->MapAction(C.OpenProject, FExecuteAction::CreateSP(this, &SAerotelicMainShell::OnOpen));
	Commands->MapAction(C.ExportStep, FExecuteAction(), Never());
	Commands->MapAction(C.ExportStl, FExecuteAction(), Never());
	Commands->MapAction(C.Quit, FExecuteAction::CreateSP(this, &SAerotelicMainShell::OnQuit));

	// Edit
	Commands->MapAction(C.ColorTheme, FExecuteAction(), Never());

	// Geometry
	Commands->MapAction(C.Airfoils, FExecuteAction(), Never());
	Commands->MapAction(C.BooleanOperation, FExecuteAction(), Never());

	// View
	Commands->MapAction(C.ViewIsometric, FExecuteAction::CreateSP(this, &SAerotelicMainShell::OnViewPreset, EAerotelicViewPreset::Isometric));
	Commands->MapAction(C.ViewTop, FExecuteAction::CreateSP(this, &SAerotelicMainShell::OnViewPreset, EAerotelicViewPreset::Top));
	Commands->MapAction(C.ViewFront, FExecuteAction::CreateSP(this, &SAerotelicMainShell::OnViewPreset, EAerotelicViewPreset::Front));
	Commands->MapAction(C.ViewSide, FExecuteAction::CreateSP(this, &SAerotelicMainShell::OnViewPreset, EAerotelicViewPreset::Side));
	Commands->MapAction(C.GroundGrid,
		FExecuteAction::CreateLambda([]() { if (AAerotelicGridGround* G = FindGround()) G->SetActorHiddenInGame(!G->IsHidden()); }),
		FCanExecuteAction::CreateLambda([]() { return FindGround() != nullptr; }),
		FIsActionChecked::CreateLambda([]() { AAerotelicGridGround* G = FindGround(); return G != nullptr && !G->IsHidden(); }));
	Commands->MapAction(C.NavigationCube, FExecuteAction(), Never(), FIsActionChecked::CreateLambda([]() { return false; }));
	Commands->MapAction(C.ShowIntermediateFeatures, FExecuteAction(), Never(), FIsActionChecked::CreateLambda([]() { return false; }));
	Commands->MapAction(C.SurfaceStyle, FExecuteAction::CreateLambda([]() { OpenAerotelicSurfaceDialog(); }));
	Commands->MapAction(C.Rebuild, FExecuteAction::CreateSP(this, &SAerotelicMainShell::OnRebuild));

	// Style / Section dropdowns (occ_viewer.py set_render_style /
	// view_gizmo.py SectionPlane), radio groups ticked from the document.
	auto StyleAction = [this](TSharedPtr<FUICommandInfo> Cmd, EAerotelicRenderStyle Style)
	{
		Commands->MapAction(Cmd,
			FExecuteAction::CreateLambda([Style]() { if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get()) Doc->SetRenderStyle(Style); }),
			FCanExecuteAction(),
			FIsActionChecked::CreateLambda([Style]() { UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get(); return Doc != nullptr && Doc->GetRenderStyle() == Style; }));
	};
	StyleAction(C.StyleShaded, EAerotelicRenderStyle::Shaded);
	StyleAction(C.StyleShadedEdges, EAerotelicRenderStyle::ShadedEdges);
	StyleAction(C.StyleWireframe, EAerotelicRenderStyle::Wireframe);
	auto SectionAction = [this](TSharedPtr<FUICommandInfo> Cmd, EAerotelicSectionAxis Axis)
	{
		Commands->MapAction(Cmd,
			FExecuteAction::CreateLambda([Axis]() { if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get()) Doc->SetSectionAxis(Axis); }),
			FCanExecuteAction(),
			FIsActionChecked::CreateLambda([Axis]() { UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get(); return Doc != nullptr && Doc->GetSectionPlane().Axis == Axis; }));
	};
	SectionAction(C.SectionNone, EAerotelicSectionAxis::None);
	SectionAction(C.SectionX, EAerotelicSectionAxis::X);
	SectionAction(C.SectionY, EAerotelicSectionAxis::Y);
	SectionAction(C.SectionZ, EAerotelicSectionAxis::Z);
	Commands->MapAction(C.SectionFlip,
		FExecuteAction::CreateLambda([]() { if (UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get()) Doc->SetSectionFlipped(!Doc->GetSectionPlane().bFlipped); }),
		FCanExecuteAction(),
		FIsActionChecked::CreateLambda([]() { UAerotelicDocumentSubsystem* Doc = UAerotelicDocumentSubsystem::Get(); return Doc != nullptr && Doc->GetSectionPlane().bFlipped; }));

	// Help
	// Forced: Help -> Welcome Screen... shows it even when ticked off at
	// start-up (the Qt app's own menu item lost that argument).
	Commands->MapAction(C.WelcomeScreen, FExecuteAction::CreateLambda([]() { ShowAerotelicWelcomeScreen(/*bForce*/ true); }));
	Commands->MapAction(C.TutorialProjects, FExecuteAction::CreateLambda([]() { ShowAerotelicTutorialDialog(); }));
	Commands->MapAction(C.About, FExecuteAction::CreateSP(this, &SAerotelicMainShell::OnAbout));

	// Add entity
	Commands->MapAction(C.AddWingTrapezoid, FExecuteAction::CreateSP(this, &SAerotelicMainShell::OnAddWing, FString(TEXT("standard"))));
	Commands->MapAction(C.AddWingCranked, FExecuteAction::CreateSP(this, &SAerotelicMainShell::OnAddWing, FString(TEXT("double_trap"))));
	Commands->MapAction(C.AddWingNurbs, FExecuteAction::CreateSP(this, &SAerotelicMainShell::OnAddWing, FString(TEXT("nurbs"))));
	Commands->MapAction(C.AddFuselage, FExecuteAction::CreateSP(this, &SAerotelicMainShell::OnAddFuselage));
	Commands->MapAction(C.AddNacelleRevolved, FExecuteAction::CreateSP(this, &SAerotelicMainShell::OnAddNacelle));
	Commands->MapAction(C.AddNacelleLofted, FExecuteAction::CreateSP(this, &SAerotelicMainShell::OnAddLoftedNacelle));
	Commands->MapAction(C.AddPropeller, FExecuteAction::CreateSP(this, &SAerotelicMainShell::OnAddPropeller));
	Commands->MapAction(C.AddSphere, FExecuteAction::CreateSP(this, &SAerotelicMainShell::OnAddPrimitive, FString(TEXT("Sphere"))));
	Commands->MapAction(C.AddBox, FExecuteAction::CreateSP(this, &SAerotelicMainShell::OnAddPrimitive, FString(TEXT("Box"))));
	Commands->MapAction(C.AddCylinder, FExecuteAction::CreateSP(this, &SAerotelicMainShell::OnAddPrimitive, FString(TEXT("Cylinder"))));
}

// ---------------------------------------------------------------- menu bar

TSharedRef<SWidget> SAerotelicMainShell::BuildMenuBar()
{
	// Each pull-down builds its own FMenuBuilder with searching off: the
	// Qt menus have no search box, and these are short.
	auto PullDown = [this](void (SAerotelicMainShell::*Fill)(FMenuBuilder&))
	{
		return FOnGetContent::CreateLambda([this, Fill]()
		{
			FMenuBuilder Menu(true, Commands, nullptr, false, &FCoreStyle::Get(), /*bSearchable*/ false);
			(this->*Fill)(Menu);
			return Menu.MakeWidget();
		});
	};

	FMenuBarBuilder Bar(Commands);
	Bar.AddPullDownMenu(LOCTEXT("FileMenu", "File"), LOCTEXT("FileMenuTip", "Project files"), PullDown(&SAerotelicMainShell::FillFileMenu));
	Bar.AddPullDownMenu(LOCTEXT("EditMenu", "Edit"), LOCTEXT("EditMenuTip", "Preferences"), PullDown(&SAerotelicMainShell::FillEditMenu));
	Bar.AddPullDownMenu(LOCTEXT("GeometryMenu", "Geometry"), LOCTEXT("GeometryMenuTip", "Airfoils and boolean operations"), PullDown(&SAerotelicMainShell::FillGeometryMenu));
	Bar.AddPullDownMenu(LOCTEXT("ViewMenu", "View"), LOCTEXT("ViewMenuTip", "3D view"), PullDown(&SAerotelicMainShell::FillViewMenu));
	Bar.AddPullDownMenu(LOCTEXT("HelpMenu", "Help"), LOCTEXT("HelpMenuTip", "Help"), PullDown(&SAerotelicMainShell::FillHelpMenu));
	return Bar.MakeWidget();
}

void SAerotelicMainShell::FillFileMenu(FMenuBuilder& Menu)
{
	const FAerotelicCommands& C = FAerotelicCommands::Get();
	Menu.AddMenuEntry(C.NewProject);
	Menu.AddMenuSeparator();
	Menu.AddMenuEntry(C.Save);
	Menu.AddMenuEntry(C.SaveAs);
	Menu.AddMenuEntry(C.OpenProject);
	Menu.AddMenuSeparator();
	Menu.AddMenuEntry(C.ExportStep);
	Menu.AddMenuEntry(C.ExportStl);
	Menu.AddMenuSeparator();
	Menu.AddMenuEntry(C.Quit, NAME_None, TAttribute<FText>(), TAttribute<FText>(), FSlateIcon(), NAME_None);
}

void SAerotelicMainShell::FillEditMenu(FMenuBuilder& Menu)
{
	Menu.AddSubMenu(LOCTEXT("Preferences", "Preferences"), LOCTEXT("PreferencesTip", "Application preferences"),
		FNewMenuDelegate::CreateLambda([](FMenuBuilder& Sub)
		{
			Sub.AddMenuEntry(FAerotelicCommands::Get().ColorTheme);
		}));
}

void SAerotelicMainShell::FillGeometryMenu(FMenuBuilder& Menu)
{
	const FAerotelicCommands& C = FAerotelicCommands::Get();
	Menu.AddMenuEntry(C.Airfoils);
	Menu.AddMenuSeparator();
	Menu.AddMenuEntry(C.BooleanOperation);
}

void SAerotelicMainShell::FillViewMenu(FMenuBuilder& Menu)
{
	const FAerotelicCommands& C = FAerotelicCommands::Get();
	Menu.AddMenuEntry(C.ViewIsometric);
	Menu.AddMenuEntry(C.ViewTop);
	Menu.AddMenuEntry(C.ViewFront);
	Menu.AddMenuEntry(C.ViewSide);
	Menu.AddMenuSeparator();
	Menu.AddMenuEntry(C.GroundGrid);
	Menu.AddMenuEntry(C.NavigationCube);
	Menu.AddMenuSeparator();
	Menu.AddMenuEntry(C.ShowIntermediateFeatures);
	Menu.AddMenuSeparator();
	Menu.AddMenuEntry(C.SurfaceStyle);
	Menu.AddMenuSeparator();
	Menu.AddMenuEntry(C.Rebuild);
}

void SAerotelicMainShell::FillHelpMenu(FMenuBuilder& Menu)
{
	const FAerotelicCommands& C = FAerotelicCommands::Get();
	Menu.AddMenuEntry(C.WelcomeScreen);
	Menu.AddMenuEntry(C.TutorialProjects);
	Menu.AddMenuSeparator();
	Menu.AddMenuEntry(C.About);
}

// ---------------------------------------------------------------- toolbars

TSharedRef<SWidget> SAerotelicMainShell::MakeIcon(const TCHAR* Abbrev, const TCHAR* FgHex, const TCHAR* BgHex, const TCHAR* BorderHex)
{
	return SNew(SBox).WidthOverride(kIconSize).HeightOverride(kIconSize)
	[
		SNew(SBorder)
		.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
		.BorderBackgroundColor(Hex(BorderHex))
		.Padding(FMargin(1))
		[
			SNew(SBorder)
			.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
			.BorderBackgroundColor(Hex(BgHex))
			.HAlign(HAlign_Center).VAlign(VAlign_Center)
			.Padding(0)
			[
				SNew(STextBlock)
				.Text(FText::FromString(Abbrev))
				.Font(FCoreStyle::GetDefaultFontStyle("Bold", 8))
				.ColorAndOpacity(FSlateColor(Hex(FgHex)))
			]
		]
	];
}

TSharedRef<SWidget> SAerotelicMainShell::MakeIconButton(TSharedPtr<FUICommandInfo> Command, const TCHAR* Abbrev,
	const TCHAR* FgHex, const TCHAR* BgHex, const TCHAR* BorderHex)
{
	TSharedRef<const FUICommandInfo> Cmd = Command.ToSharedRef();
	TSharedPtr<FUICommandList> List = Commands;
	return SNew(SButton)
		.ButtonStyle(FCoreStyle::Get(), "NoBorder")
		.ContentPadding(FMargin(2))
		.ToolTipText(Command->GetDescription())
		.IsEnabled_Lambda([List, Cmd]() { return List->CanExecuteAction(Cmd); })
		.OnClicked_Lambda([List, Cmd]() { List->TryExecuteAction(Cmd); return FReply::Handled(); })
		[
			MakeIcon(Abbrev, FgHex, BgHex, BorderHex)
		];
}

TSharedRef<SWidget> SAerotelicMainShell::MakeGroupLabel(const TCHAR* Text, const TCHAR* ColorHex)
{
	return SNew(SBox).VAlign(VAlign_Center).Padding(FMargin(8, 0, 6, 0))
	[
		SNew(STextBlock)
		.Text(FText::FromString(Text))
		.Font(FCoreStyle::GetDefaultFontStyle("Bold", 7))
		.ColorAndOpacity(FSlateColor(Hex(ColorHex)))
	];
}

TSharedRef<SWidget> SAerotelicMainShell::BuildEntityToolbar()
{
	const FAerotelicCommands& C = FAerotelicCommands::Get();
	auto Sep = []() { return SNew(SBox).WidthOverride(1).Padding(FMargin(4, 4)) [ SNew(SBorder).BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush")).BorderBackgroundColor(Hex(TEXT("#30363d"))) ]; };

	return SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().AutoWidth() [ MakeGroupLabel(TEXT("WING"), TEXT("#4a90d9")) ]
		+ SHorizontalBox::Slot().AutoWidth() [ MakeIconButton(C.AddWingTrapezoid, TEXT("TRAP"), TEXT("#7cc0ff"), TEXT("#0d1a2a"), TEXT("#4a7aa0")) ]
		+ SHorizontalBox::Slot().AutoWidth() [ MakeIconButton(C.AddWingCranked, TEXT("2TRP"), TEXT("#40d8d8"), TEXT("#001a1a"), TEXT("#208888")) ]
		+ SHorizontalBox::Slot().AutoWidth() [ MakeIconButton(C.AddWingNurbs, TEXT("NRB"), TEXT("#c080ff"), TEXT("#100820"), TEXT("#7040b0")) ]
		+ SHorizontalBox::Slot().AutoWidth() [ Sep() ]
		+ SHorizontalBox::Slot().AutoWidth() [ MakeGroupLabel(TEXT("FUSELAGE"), TEXT("#50a050")) ]
		+ SHorizontalBox::Slot().AutoWidth() [ MakeIconButton(C.AddFuselage, TEXT("CON"), TEXT("#90d878"), TEXT("#071a07"), TEXT("#407030")) ]
		+ SHorizontalBox::Slot().AutoWidth() [ Sep() ]
		+ SHorizontalBox::Slot().AutoWidth() [ MakeGroupLabel(TEXT("NACELLE"), TEXT("#c89a48")) ]
		+ SHorizontalBox::Slot().AutoWidth() [ MakeIconButton(C.AddNacelleRevolved, TEXT("NAC"), TEXT("#f0c878"), TEXT("#1a1206"), TEXT("#8a6428")) ]
		+ SHorizontalBox::Slot().AutoWidth() [ MakeIconButton(C.AddNacelleLofted, TEXT("LFT"), TEXT("#f0a0b8"), TEXT("#1a0610"), TEXT("#8a2848")) ]
		+ SHorizontalBox::Slot().AutoWidth() [ Sep() ]
		+ SHorizontalBox::Slot().AutoWidth() [ MakeGroupLabel(TEXT("PROPELLER"), TEXT("#a8b0c0")) ]
		+ SHorizontalBox::Slot().AutoWidth() [ MakeIconButton(C.AddPropeller, TEXT("PRP"), TEXT("#dde4f0"), TEXT("#10141c"), TEXT("#6a7488")) ]
		+ SHorizontalBox::Slot().AutoWidth() [ Sep() ]
		+ SHorizontalBox::Slot().AutoWidth() [ MakeGroupLabel(TEXT("PRIMITIVE"), TEXT("#c8b848")) ]
		+ SHorizontalBox::Slot().AutoWidth() [ MakeIconButton(C.AddSphere, TEXT("SPH"), TEXT("#f0e088"), TEXT("#1a1806"), TEXT("#8a7c28")) ]
		+ SHorizontalBox::Slot().AutoWidth() [ MakeIconButton(C.AddBox, TEXT("BOX"), TEXT("#f0e088"), TEXT("#1a1806"), TEXT("#8a7c28")) ]
		+ SHorizontalBox::Slot().AutoWidth() [ MakeIconButton(C.AddCylinder, TEXT("CYL"), TEXT("#f0e088"), TEXT("#1a1806"), TEXT("#8a7c28")) ];
}

TSharedRef<SWidget> SAerotelicMainShell::MakeDropdown(const TCHAR* Abbrev, const FText& ToolTip, TSharedRef<SWidget> (SAerotelicMainShell::*MakeMenu)())
{
	return SNew(SComboButton)
		.ButtonStyle(FCoreStyle::Get(), "NoBorder")
		.ContentPadding(FMargin(0))
		.HasDownArrow(false)
		.ToolTipText(ToolTip)
		.OnGetMenuContent(FOnGetContent::CreateSP(this, MakeMenu))
		.ButtonContent()
		[
			MakeIcon(Abbrev, TEXT("#c8ccd4"), TEXT("#14161a"), TEXT("#30363d"))
		];
}

TSharedRef<SWidget> SAerotelicMainShell::MakeStyleMenu()
{
	const FAerotelicCommands& C = FAerotelicCommands::Get();
	FMenuBuilder Menu(true, Commands, nullptr, false, &FCoreStyle::Get(), false);
	Menu.AddMenuEntry(C.StyleShaded);
	Menu.AddMenuEntry(C.StyleShadedEdges);
	Menu.AddMenuEntry(C.StyleWireframe);
	return Menu.MakeWidget();
}

TSharedRef<SWidget> SAerotelicMainShell::MakeOrientationMenu()
{
	const FAerotelicCommands& C = FAerotelicCommands::Get();
	FMenuBuilder Menu(true, Commands, nullptr, false, &FCoreStyle::Get(), false);
	Menu.AddMenuEntry(C.ViewTop);
	Menu.AddMenuEntry(C.ViewSide);
	Menu.AddMenuEntry(C.ViewFront);
	Menu.AddMenuEntry(C.ViewIsometric);
	return Menu.MakeWidget();
}

TSharedRef<SWidget> SAerotelicMainShell::MakeSectionMenu()
{
	const FAerotelicCommands& C = FAerotelicCommands::Get();
	FMenuBuilder Menu(true, Commands, nullptr, false, &FCoreStyle::Get(), false);
	Menu.AddMenuEntry(C.SectionNone);
	Menu.AddMenuEntry(C.SectionX);
	Menu.AddMenuEntry(C.SectionY);
	Menu.AddMenuEntry(C.SectionZ);
	Menu.AddMenuSeparator();
	Menu.AddMenuEntry(C.SectionFlip);
	return Menu.MakeWidget();
}

TSharedRef<SWidget> SAerotelicMainShell::BuildViewToolbar()
{
	const FAerotelicCommands& C = FAerotelicCommands::Get();
	return SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
		[ MakeDropdown(TEXT("STY"), LOCTEXT("StyleTip", "Style: how the bodies are drawn"), &SAerotelicMainShell::MakeStyleMenu) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
		[ MakeDropdown(TEXT("ORI"), LOCTEXT("OrientationTip", "Orientation: look from a preset direction"), &SAerotelicMainShell::MakeOrientationMenu) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
		[ MakeDropdown(TEXT("SEC"), LOCTEXT("SectionTip", "Section: cut the bodies with a plane"), &SAerotelicMainShell::MakeSectionMenu) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6)
		[ SNew(SSeparator) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 2)
		[ MakeIconButton(C.Rebuild, TEXT("RBLD"), TEXT("#ffd080"), TEXT("#1a1406"), TEXT("#8a6c28")) ];
}

TSharedRef<SWidget> SAerotelicMainShell::BuildStatusBar()
{
	return SNew(SBorder)
		.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
		.BorderBackgroundColor(kBarBg)
		.Padding(FMargin(8, 3))
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
			[
				SNew(STextBlock).Text(this, &SAerotelicMainShell::GetStatusText)
			]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
			[
				SNew(STextBlock)
				.Text(this, &SAerotelicMainShell::GetProjectPathText)
				.ColorAndOpacity(FSlateColor(FLinearColor(0.6f, 0.6f, 0.6f)))
			]
		];
}

FText SAerotelicMainShell::GetProjectPathText() const
{
	UAerotelicDocumentSubsystem* Doc = GetDocument();
	if (Doc != nullptr && Doc->HasCurrentPath())
	{
		return FText::FromString(Doc->GetCurrentPath());
	}
	return LOCTEXT("Untitled", "(untitled project)");
}

// ----------------------------------------------------------------- actions

void SAerotelicMainShell::OnNew()
{
	if (UAerotelicDocumentSubsystem* Doc = GetDocument())
	{
		if (Doc->NewProject())
		{
			StatusText = LOCTEXT("StatusNew", "New project -- empty scene");
		}
	}
}

void SAerotelicMainShell::OnOpen()
{
	if (UAerotelicDocumentSubsystem* Doc = GetDocument())
	{
		if (Doc->OpenProjectWithDialog())
		{
			StatusText = FText::Format(LOCTEXT("StatusOpened", "Opened {0}"), FText::FromString(Doc->GetCurrentPath()));
		}
	}
}

void SAerotelicMainShell::OnSave()
{
	if (UAerotelicDocumentSubsystem* Doc = GetDocument())
	{
		if (Doc->SaveProject())
		{
			StatusText = FText::Format(LOCTEXT("StatusSaved", "Saved {0}"), FText::FromString(Doc->GetCurrentPath()));
		}
	}
}

void SAerotelicMainShell::OnSaveAs()
{
	if (UAerotelicDocumentSubsystem* Doc = GetDocument())
	{
		if (Doc->SaveProjectAsWithDialog())
		{
			StatusText = FText::Format(LOCTEXT("StatusSaved", "Saved {0}"), FText::FromString(Doc->GetCurrentPath()));
		}
	}
}

void SAerotelicMainShell::OnQuit()
{
	FPlatformMisc::RequestExit(false);
}

void SAerotelicMainShell::OnAbout()
{
	// The same identity and legal text as the welcome screen (branding.py's
	// about_html), so the two cannot drift apart.
	FMessageDialog::Open(EAppMsgType::Ok, FText::FromString(AerotelicBranding::AboutText()),
		FText::FromString(FString::Printf(TEXT("About %s"), AerotelicBranding::Product)));
}

void SAerotelicMainShell::OnRebuild()
{
	if (AAerotelicCameraPawn::RebuildSelectedEntity())
	{
		AAerotelicEntityActor* Entity = GetDocument() ? GetDocument()->GetSelectedEntity() : nullptr;
		StatusText = FText::Format(LOCTEXT("StatusRebuilt", "Rebuilt {0}"), FText::FromString(Entity ? Entity->EntityName : FString()));
	}
	else
	{
		StatusText = LOCTEXT("StatusNoSelection", "Select an entity first (click it in the view or in the Scene tree).");
	}
}

void SAerotelicMainShell::OnViewPreset(EAerotelicViewPreset Preset)
{
	if (AAerotelicCameraPawn* Camera = AAerotelicCameraPawn::FindInGameWorld())
	{
		Camera->SetPresetView(Preset);
	}
}

void SAerotelicMainShell::AfterAdd(AAerotelicEntityActor* Entity)
{
	if (Entity == nullptr)
	{
		return;
	}
	StatusText = FText::Format(LOCTEXT("StatusAdded", "Added {0}"), FText::FromString(Entity->EntityName));
	// Select the new entity and show its design panel, as "Add Wing" opens
	// the planform dock in the Qt version.
	SAerotelicSceneTree::EditEntity(Entity);
}

void SAerotelicMainShell::OnAddWing(FString PlanformMode)
{
	if (UAerotelicDocumentSubsystem* Doc = GetDocument()) AfterAdd(Doc->AddWing(PlanformMode));
}

void SAerotelicMainShell::OnAddFuselage()
{
	if (UAerotelicDocumentSubsystem* Doc = GetDocument()) AfterAdd(Doc->AddFuselage());
}

void SAerotelicMainShell::OnAddNacelle()
{
	if (UAerotelicDocumentSubsystem* Doc = GetDocument()) AfterAdd(Doc->AddNacelle());
}

void SAerotelicMainShell::OnAddLoftedNacelle()
{
	if (UAerotelicDocumentSubsystem* Doc = GetDocument()) AfterAdd(Doc->AddLoftedNacelle());
}

void SAerotelicMainShell::OnAddPropeller()
{
	if (UAerotelicDocumentSubsystem* Doc = GetDocument()) AfterAdd(Doc->AddPropeller());
}

void SAerotelicMainShell::OnAddPrimitive(FString Shape)
{
	if (UAerotelicDocumentSubsystem* Doc = GetDocument()) AfterAdd(Doc->AddPrimitive(Shape));
}

#undef LOCTEXT_NAMESPACE
