# Aerotelic (C++ / Unreal) -- standalone application

Aerotelic is the C++ port of the Python/Qt parametric aircraft-geometry CAD
tool. The geometry backend is `../core` (OCCT + yaml-cpp, in this same
repository); this project is the user interface and 3D view, built on Unreal Engine 5.8 and
delivered as a **standalone application** -- the Unreal Editor is not part of
the workflow.

## Running

| Script | What it does |
|---|---|
| `run_aerotelic.bat [project.yaml]` | Launches the app. Uses the packaged build in `Packaged\` if present, otherwise the developer launcher. |
| `run_aerotelic_dev.bat [project.yaml]` | Developer launcher: hosts the app in the editor binaries in `-game` mode (no Editor UI, reads uncooked content). Needs `build_editor.bat`. |
| `build_editor.bat` | Compiles the module for the developer launcher and the tests. |
| `package_standalone.bat` | Cooks and packages the self-contained `Packaged\Windows\AerotelicUE.exe`. Slow the first time (shader compilation). |
| `run_tests.bat` | Headless regression: document round-trip (17 checks) and loading the real `p38.yaml` (4 checks). |

Both launchers accept an optional project file to open at startup
(`-AerotelicProject=<path>` on the executable's command line).

## The application window

The layout follows the Python/Qt version's main window (`aerotelic/ui/main_window.py`):

* **Menu bar**: File (New, Save, Save As, Open, Export STEP/STL, Quit), Edit
  (Preferences), Geometry (Airfoils, Boolean Operation), View (Isometric /
  Top / Front / Side, Ground Grid, Navigation Cube, Show Intermediate
  Features, Rebuild), Help. Same shortcuts as the Qt app (Ctrl+N/S/O,
  Ctrl+Shift+S, F5...). Items whose backend is not in the C++ port yet
  (exports, theme, airfoil dock, booleans, grid, nav cube, tutorials) are
  present but **disabled**, with a tooltip saying so.
* **Toolbar** under the menu: the add-entity launchers, grouped WING /
  FUSELAGE / NACELLE / PROPELLER / PRIMITIVE with the Qt version's
  text-label icons. Adding an entity selects it and opens its design panel.
* **Left edge**: Style / Orientation / Section dropdowns (Orientation works;
  the others are placeholders) and Rebuild.
* **Scene tree**: one row per entity with a visibility checkbox. Click to
  select, double-click to edit, right-click for Hide/Show, Edit, Rename,
  Transform, Color (placeholder), Mirrored (placeholder), Delete.
* **3D view** in the middle: left-drag orbits, right/middle-drag pans, wheel
  zooms, `F` frames the selected entity, `Home` frames everything. Clicking
  an object selects it (orange tint); clicking empty space clears the
  selection. Rendering is confined to this region (the frame sets the local
  player's split-screen rectangle), so framing and picking are relative to
  the visible view, not the whole window.
* **Design dock** on the right: the selected entity's design panel (Wing,
  Fuselage, Nacelle, Lofted Nacelle, Propeller, Primitive) in a dockable tab
  that can be torn off into its own window and docked back. The tab exists
  only while an entity is being edited: it opens on Edit (tree
  double-click, context menu, or adding an entity) and closes with its own
  close button or the close button of the window it was torn off into.
  Whenever nothing is docked (at startup, after a close, or while the tab
  floats) the column collapses to zero width and the 3D view takes the
  space; a 6-px grip on the column's left edge drags its width (300..1000).
  Panels are kept per entity while the tab lives, so switching selection
  keeps each panel's tab state.
* **Design panels** carry the WingDesigner panel features: every curve tab
  has a degree spin box (1..points-1, applied through `SetDegree`), a
  "Reset Knots" button and a reset-to-default (Straight / Flat / Initial /
  Default / Circle...), a Help tab with the WingDesigner text, and
  read-only derived metrics. Specifics: wing Standard tab has a Type combo
  (Tip Chord vs Taper Ratio) and a planform outline view; fuselage has a
  Sections tab with a station slider, section preview and parameter table;
  nacelle Front and lofted-nacelle guide curves have "Reset to Circle..."
  with a radius prompt; lofted nacelle has an "Editing:" guide combo with
  Add/Remove; propeller shows derived metrics and a Spinner tab.
* **Transform dialog** (from the tree's context menu): one modeless window
  per entity with position, rotation axis and angle.
* **Status bar**: last action on the left, project path on the right.

## How the standalone build is put together

* `Source/AerotelicUE.Target.cs` -- the game target that becomes `AerotelicUE.exe`.
  The module is `Runtime` type; editor-only code (the legacy Window -> Aerotelic
  tab, World-Outliner selection fallback) is compiled only for the editor target.
* `Source/AerotelicUE/Standalone/` -- `AerotelicGameMode` (studio lighting,
  grey backdrop, command-line project), `AerotelicCameraPawn` (orbit camera,
  click-to-select, view presets), `AerotelicPlayerController` (cursor/input
  mode, hosts the frame over the viewport and confines the 3D rendering to
  the frame's view hole via the viewport client's split-screen table).
* `Source/AerotelicUE/MainShell/` -- the frame: `AerotelicMainShell` (menu
  bar, toolbars, layout, status bar), `AerotelicCommands` (every action with
  label/tooltip/shortcut), `AerotelicSceneTree`, `AerotelicDesignDock`
  (private `FTabManager` hosting the Design tab), `AerotelicDesignPanel`
  (selected entity's panel), `AerotelicTransformDialog`.
* `Source/AerotelicUE/Dialogs/` -- one `Build*DesignContent` per entity
  type, dispatched by `EntityDesignContent`; these are the panel contents
  shown in the design dock.
* `Widgets/CurveTabControls` -- the shared panel chrome (degree spin, control
  buttons in a wrapping row, info label, metric row, group box, Help tab,
  `ResetCurveFromSpec` / `ResetCurveToLine`). `Widgets/PromptDialogs` has
  the modal string and circle-radius prompts.
* `Widgets/InputSink` -- wraps each frame panel so unhandled mouse buttons
  and wheel turns stop there instead of bubbling to the game viewport (the
  frame is a viewport overlay, i.e. a child of the SViewport; without this a
  right-click on a tree row reached the viewport, which captured the mouse,
  and the context menu never opened).
* `Entity/AerotelicDocumentSubsystem` is an engine subsystem, so the same
  document object serves the standalone app, the Editor host and the test
  commandlets.
* `Config/DefaultEngine.ini` -- starts in the engine's empty `Entry` map (no
  project map asset is needed), fixed exposure, no Lumen/bloom/motion blur,
  near clip 0.5 (models are a few units across), UI at 1:1 scale.
* Docking note (spike, 2026-10-09): Slate's docking system works in the
  standalone game target -- tabs stack, tear off into floating windows,
  re-dock into a tab well and split the column -- provided the tabs live in
  a private `FTabManager` (child of the global one). Tabs spawned from the
  global manager's nomad spawners can re-dock but may not split. Two more
  findings: a tab with `CanEverClose(false)` also pins its floating window
  open (the window asks every tab before closing); and after a tab is
  closed in a floating window, `TryInvokeTab` restores that window from the
  manager's collapsed areas, so the dock re-opens the tab by restoring a
  fresh primary area into the column instead.
* `tools/mouse.ps1` -- blind UI automation (click, right-click,
  double-click, drag, held drag, key presses, window list, close main
  window) used with `tools/screenshot.ps1` to verify the app without a human
  at the keyboard.

## Known limits of this prototype

* Only what the Editor-hosted version had is carried over: no new modelling
  functionality. Mirrored entities (`mirror: true` in the project file) are
  still rendered un-mirrored, as before.
* Transform dialogs open just to the right of the scene tree, so they do not
  cover the 3D view's centre; a second, third and later dialog cascade down
  and to the right. Drag any of them wherever you like.
* Not yet carried over from the Qt version: the custom frameless title bar,
  render styles (edges / wireframe), section planes, ground grid, navigation
  cube, per-body rows and colours in the scene tree, mirrored entities,
  reference images, the Selig airfoil database / `.dat` import, and the
  wing Control Surfaces tab (the C++ core has no control-surface model yet;
  the tab is a placeholder note).
* Build gotchas: a TArray must not be appended with a reference to its own
  element (`Pts.Add(Pts[0])` reallocates under the reference); if a crash
  leaves `CrashReportClientEditor.exe` running it holds the module DLL and
  the next link fails with LNK1104 -- kill it first.
* The packaged build is `Development` configuration (native file dialogs via
  the DesktopPlatform module are not available in `Shipping`).
