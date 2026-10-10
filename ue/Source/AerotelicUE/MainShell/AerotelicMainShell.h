#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "GenericPlatform/GenericApplication.h"
#include "InputCoreTypes.h"

class AAerotelicEntityActor;
class UAerotelicDocumentSubsystem;
class FUICommandList;
class FUICommandInfo;
class FMenuBuilder;
struct FKeyEvent;
enum class EAerotelicViewPreset : uint8;

// The application frame -- the Slate port of main_window.py's QMainWindow
// layout, laid over the game viewport:
//
//   * Menu bar: File / Edit / Geometry / View / Help, same items and
//     shortcuts as the Qt version (items without a backend yet are disabled).
//   * Horizontal "Panels" toolbar under it: the add-entity launchers,
//     grouped WING / FUSELAGE / NACELLE / PROPELLER / PRIMITIVE.
//   * Vertical "Controls" toolbar on the left edge: Style / Orientation /
//     Section dropdowns and Rebuild.
//   * Scene tree beside the 3D view (MainShell/AerotelicSceneTree).
//   * The 3D view in the middle: a hit-test-invisible hole the viewport
//     shows and receives clicks through; the player controller confines the
//     3D rendering to it (GetViewHoleRect).
//   * Design dock column on the right (MainShell/AerotelicDesignDock).
//   * Status bar along the bottom.
//
// In the Editor fallback host (the Window -> Aerotelic tab) the same frame
// is shown; the hole is simply empty there.
class SAerotelicMainShell : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SAerotelicMainShell) {}
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);
	virtual ~SAerotelicMainShell();

	// Screen-space rectangle of the 3D view hole (absolute Slate
	// coordinates). False until the frame has been laid out once.
	bool GetViewHoleRect(FVector2D& OutAbsPos, FVector2D& OutAbsSize) const;

	void SetStatus(const FText& Text);

	// Offers a key press to the menu/toolbar shortcuts (Ctrl+N, F5, ...).
	// In the standalone game every key the frame's widgets leave unhandled
	// reaches the player controller (the frame is a child of the viewport
	// widget), which calls this. Returns true if a command ran.
	bool ProcessShortcut(const FKey& Key, const FModifierKeysState& Modifiers, bool bRepeat);

private:
	TSharedPtr<FUICommandList> Commands;
	TSharedPtr<SWidget> ViewHole;
	FText StatusText;
	// The dock column's width, dragged with the handle on its left edge.
	float DockColumnWidth = 400.0f;
	// View -> Navigation Cube.
	bool bNavCubeVisible = true;
	FDelegateHandle KeyListenerHandle;

	// Construction.
	void BindCommands();
	TSharedRef<SWidget> BuildMenuBar();
	void FillFileMenu(FMenuBuilder& Menu);
	void FillEditMenu(FMenuBuilder& Menu);
	void FillGeometryMenu(FMenuBuilder& Menu);
	void FillViewMenu(FMenuBuilder& Menu);
	void FillHelpMenu(FMenuBuilder& Menu);
	TSharedRef<SWidget> BuildEntityToolbar();
	TSharedRef<SWidget> BuildViewToolbar();
	TSharedRef<SWidget> BuildStatusBar();
	TSharedRef<SWidget> MakeStyleMenu();
	TSharedRef<SWidget> MakeOrientationMenu();
	TSharedRef<SWidget> MakeSectionMenu();

	// A 30x30 text-label icon button (the Qt version's _label_icon) bound
	// to a command.
	TSharedRef<SWidget> MakeIconButton(TSharedPtr<FUICommandInfo> Command, const TCHAR* Abbrev,
		const TCHAR* FgHex, const TCHAR* BgHex, const TCHAR* BorderHex);
	TSharedRef<SWidget> MakeIcon(const TCHAR* Abbrev, const TCHAR* FgHex, const TCHAR* BgHex, const TCHAR* BorderHex);
	TSharedRef<SWidget> MakeGroupLabel(const TCHAR* Text, const TCHAR* ColorHex);
	TSharedRef<SWidget> MakeDropdown(const TCHAR* Abbrev, const FText& ToolTip, TSharedRef<SWidget> (SAerotelicMainShell::*MakeMenu)());

	// Actions.
	void OnNew();
	void OnOpen();
	void OnSave();
	void OnSaveAs();
	void OnQuit();
	void OnAbout();
	void OnRebuild();
	void OnViewPreset(EAerotelicViewPreset Preset);
	void OnAddWing(FString PlanformMode);
	void OnAddFuselage();
	void OnAddNacelle();
	void OnAddLoftedNacelle();
	void OnAddPropeller();
	void OnAddPrimitive(FString Shape);
	void AfterAdd(AAerotelicEntityActor* Entity);

#if WITH_EDITOR
	// Editor host only: Slate's pre-input listener (an Editor-only API)
	// offers every key-down to the command list before Slate routes it.
	void OnPreKeyDown(const FKeyEvent& KeyEvent);
#endif

	FText GetStatusText() const { return StatusText; }
	FText GetProjectPathText() const;
	UAerotelicDocumentSubsystem* GetDocument() const;
};
