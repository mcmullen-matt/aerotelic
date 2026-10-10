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
* **Left edge**: Style / Orientation / Section dropdowns and Rebuild.
  **Style** (occ_viewer.py's render styles): Shaded; Shaded with Edges
  (the triangulation's edges in near-black over the surface); Wireframe
  (those edges alone, in the body's colour). The edges are a second,
  wireframe-material copy of each body's mesh (`M_AerotelicWire`, unlit,
  nudged a little towards the camera so it sits on the surface); the
  surface mesh stays the pick geometry even while hidden. **Section**
  (view_gizmo.py SectionPlane): None / X Plane / Y Plane / Z Plane and
  Flip Side. One clip plane normal to a world axis, applied to every body
  (never the ground or the gizmo) through the materials' ClipPoint /
  ClipNormal parameters, starting at the middle of the scene along the
  axis; the positive side is kept unless flipped. An amber frame outlines
  the plane and a double-headed arrow along the axis, drawn on top of the
  bodies so it stays reachable inside one, moves it when dragged (clamped
  to the scene plus 2 %). The engine has no capping, so a cut body shows
  the inside of its far wall, darkened, where WingDesigner caps the cut.
  Neither style nor section is saved with the project.
* **Navigation cube** (View -> Navigation Cube, on by default): a shaded,
  labelled cube in the top-right corner of the 3D view, turning with the
  camera, with an XYZ triad at its corner (WingDesigner's AIS_ViewCube).
  Faces are named after the application's own presets: FRONT is the face
  the Front preset looks at (-X), TOP is +Z, LEFT is -Y. Hovering lights
  the face, edge or corner under the cursor; clicking swings the camera to
  look from there over 0.35 s -- the centre of a face for a face-on view,
  near an edge for a two-axis view, a corner for three (face zone 0.72).
  Drawn entirely in Slate, so styles and sections never touch it.
* **Airfoils** (Geometry -> Airfoils..., and the picker behind a section's
  airfoil button on a wing's Sections tab): WingDesigner's library.
  The UIUC / Selig database (~1,650 airfoils) is downloaded once, on
  request, into `%LOCALAPPDATA%\WingDesigner\selig_db` (shared with the Qt
  app; an older `Aerotelic\selig_db` is used if that is what exists) and
  indexed; files are parsed only when selected. `Load .dat...` reads any
  Selig-format file (the core's `ParseSeligDat` port: comments, Lednicer
  count headers, LE-to-origin / TE-onto-+x normalisation, unit chord) into
  the session's custom library. The panel is preview only: search box,
  source combo (All / Selig / Custom), result list, the 2-D preview
  (training points, upper and lower NURBS, dashed camber, chord line,
  legend, equal aspect) and the statistics (max t/c and camber, with
  their x/c). The picker adds the sources In this project (gold, one entry
  per distinct airfoil across every wing and blade) and NACA 4-digit (the
  common codes, plus whatever valid code is typed -- "2412", "naca 2412",
  "NACA-2412"); double-click or OK assigns. A chosen airfoil is saved as
  its fitted curve, so projects need no database.
* **Scene tree**: the list of parameterisations, as in WingDesigner. Each
  entity row has a visibility checkbox, a colour swatch, its name and kind,
  and child rows for the solid bodies it produces (`<entity>-<n>` with the
  role: Main, or a propeller's Spinner and Blade n) plus, when mirrored,
  one `<entity>-<n>-mirror` row per body. Every row has its own checkbox.
  Click to select (a body row selects its entity), double-click to edit,
  right-click an entity for Hide/Show, Edit, Rename, Transform, Color,
  Default Color, Mirrored (toggle), Delete; a body for Hide/Show, Color,
  Entity's Color.
* **Mirroring** (entity menu, Mirrored): a port-side copy reflected about
  world Y = 0 from the *placed* shape, exactly WingDesigner's
  MirrorFeature -- an off-centre boom lands on the other side and a
  mirrored propeller comes out the opposite hand. It follows every move
  and edit of its entity, inherits its colours, and is never edited on
  its own. Saved as the entity's `mirror:` flag.
* **Colour**: per entity and per body, chosen with the engine's own colour
  picker (AppFramework's SColorPicker, no alpha). Defaults are
  WingDesigner's per-kind colours (steel blue wing, green fuselage, bronze
  nacelle, rose lofted nacelle, aluminium propeller, ochre primitive);
  resolution is body override, else entity override, else default. Setting
  an entity colour drops its bodies' overrides. Only overrides are saved
  (`color: [r, g, b]`, 0..1), as the Qt app does; `hidden: true` is
  saved too.
* **Surface** (View -> Surface...): a modeless window with Metallic,
  Glossiness (specular) and Blur (roughness) sliders and Matte / Satin /
  Glossy / Metal presets, setting the finish of every body in the scene at
  once. The bodies are drawn with `Content/Materials/M_AerotelicSurface`
  (Color, Metallic, Specular and Roughness parameters, the section clip,
  darker back faces), an asset authored by `setup_materials.bat` (the
  `AerotelicMaterialSetup` commandlet, Editor binaries) because shaders
  cannot be compiled inside a packaged game; the same commandlet authors
  `M_AerotelicWire` (the edge styles) and `M_AerotelicGizmo` /
  `M_AerotelicGizmoTop` (the section gizmo's frame and its on-top arrow).
  All four are committed. If the surface asset is missing the bodies fall
  back to BasicShapeMaterial and the dialog has no effect; without the
  wire asset the edge styles draw no edges. View -> Ground Grid (Ctrl+G)
  shows or hides the ground.
* **Reference images** (WingDesigner's REQ-IO-011 / REQ-FUTURE-020, -024,
  -025, -027): every side/top/plan/front-view curve editor has a
  "Reference image" group with Import..., Remove, Fit, Flip H/V and
  Opacity. The picture draws under the curves in the editor's own
  non-dimensional units (scale = units per source pixel, position of the
  source's top-left corner). Click it to select: drag the body to move, a
  corner handle to scale about the opposite corner (aspect fixed, floor
  5 % of the scale at press), an edge handle to crop, the knob on the
  stem above it to turn (snapping to 15 degrees within 0.75; offered only
  on equal-aspect editors, where a turn is rigid); double-click the knob
  or use the graph's right-click "Level reference image" to level. Poles
  win over the body, a selected image's handles win over poles. Saved on
  the model under `reference_images:` keyed by view (profile, width,
  planform, dihedral, side, front, centreline, section, skins, spinner)
  with path, scale, x, y, crop, opacity, flip_h, flip_v, angle; legacy
  mpp/x_m/y_m files are converted on load. An image beside the project
  file is saved relative to it, others absolute; a missing file draws
  nothing and keeps its placement. PNG, JPEG, BMP and TIFF (no SVG: the
  engine has no SVG rasteriser).
* **Boolean operations** (Geometry -> Boolean Operation...): WingDesigner's
  dialog -- the operands in order (every entity, the mirrored half of each
  mirrored one as its own row, every earlier boolean result), Add / Select
  All / Remove / Up / Down, the operation (Fuse, Cut = first minus the
  rest, Common) and a unique name ("BoolOp n" by default). The result is a
  new scene item of kind boolean: the operands' placed shapes folded left
  to right with the core's `BooleanOperation`, drawn in lilac with one
  "Body n" row per solid, shown, coloured, renamed and deleted like any
  row (no design panel, placement or mirror of its own). The operands stay
  in the scene, so hide them to see the result alone. It follows its
  operands: editing or moving one rebuilds it, deleting one deletes it
  (with a warning). An operation OCCT cannot complete is reported and adds
  nothing. A fuse of lofted bodies can take tens of seconds, as in the Qt
  app. Saved under `booleans:` by name (`operands: [Wing, {mirror: Wing}]`,
  `operation`, `hidden`, `color`, per-body `bodies:`), validated on load
  as WingDesigner does; a boolean that cannot be rebuilt is left out with
  a message. No seam fillet (removed from WingDesigner's UI 2026-09-29).
  View -> Show Intermediate Features stays disabled: the fold's steps are
  not materialised here.
* **Welcome screen** (Help -> Welcome Screen..., and at start-up unless
  ticked off; `-NoWelcome` or `-unattended` skips it): WingDesigner's
  splash card -- the artwork from `Resources/`, product identity and
  version (from `Resources/VERSION` or `VERSION`, else `0.9.0-dev`), the
  licence block, the legal small print, the tutorial link, "Show this
  screen at start-up" (saved in the game user settings) and Close.
  Branding and the disclaimer text live in `Branding/AerotelicBranding`
  and are, as in WingDesigner, placeholders not reviewed by counsel.
* **Tutorial Projects** (Help -> Tutorial Projects..., F1, or from the
  welcome screen): the matrix of the two shipped projects in `Projects/`
  (Cessna 172, Lockheed P-38 Lightning) with each row's contents read off
  disk; a missing or unreadable file is greyed. Opening a row loads it
  without adopting its path, so the first Save asks for a location.
* **Licensing** (`Licensing/`): WingDesigner's scheme, accepting the same
  licence files. A packaged build (or a dev run with
  `AEROTELIC_ENFORCE_LICENCE=1`) verifies `licence.lic` -- an
  Ed25519-signed JSON envelope bound to this computer's MachineGuid --
  before the frame exists, checks the licence with the server
  (`/v1/check`, trusted time) or falls back to the kept signed reply
  within a 7-day grace, and otherwise shows the activation dialog (enter
  a 25-character key -> `/v1/activate`) or exits. `PublicKeys()` in
  `AerotelicLicence.cpp` is EMPTY until a key pair is generated with the
  licence server's `admin keygen`; with none, an enforced build cannot
  start (REQ-LIC-006). The server URL is overridable with
  `AEROTELIC_LICENCE_SERVER`. No feature gating, no trial, no read-only
  mode, exactly as the Qt app. `run_tests.bat` runs
  `AerotelicLicenceTest`: 42 checks driving the verifier against a key
  pair generated in process.
* **Ground**: the GridCube-UE5.8 reference project's grid lattice (thin cube
  instances over a dark base plane, engine basic shapes only) ported in
  `Standalone/AerotelicGridGround`, scaled to model units and refitted
  whenever the entity list changes: square and symmetric about the origin,
  from -L to +L in X and Y with L = 1.2 x the scene's largest |X| or |Y|,
  the lines falling exactly on 0 and +/-L, and the plane sitting at
  Z = -0.1 L below the origin. It has no collision, so clicks on it still
  clear the selection.
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
  read-only derived metrics. Editing tabs hold only controls: every
  descriptive paragraph lives in the panel's Help tab (the wing has one of
  its own; the propeller's Help includes the blade tabs' text).
* **Curve editors** draw a titled, framed plot with the x-axis label
  centred below it and the y-axis label rotated along its left edge, in
  non-dimensional units (x/L, y/L, z/L; x/R and r/R on the propeller
  spinner; the twist editor's y axis is in degrees). Curves are clipped to
  the plot area; hit-testing is relative to it.
* **Ticks and limits.** Major ticks with numeric labels and minor ticks
  sit on the left and bottom edges, with a faint grid through the majors,
  placed as matplotlib's automatic locator does (steps of 1, 2, 2.5 or 5
  times a power of ten; 5 minors per major, 4 when the step is 2). The
  x axis shows 0..1 (small grab padding); the y axis starts at 0 and
  shows up to a per-editor default ceiling (`DefaultYMax`: 0.5 planform
  chord, 0.1 blade chord / dihedral / fuselage heights / lofted offsets,
  0.25 nacelle and spinner radius, 1.0 conic rho, 10 degrees twist), grown
  to take in content above it or below zero. Equal-aspect editors treat
  the ceiling as a minimum. Dragging a pole past an edge grows the view
  on that side by the cursor's travel beyond it and the pole rides the new
  edge; the view never shrinks during a drag and stays grown afterwards.
  Shapes about an origin (the nacelle lip ring) frame their content.
  Basis panes have fixed y ticks at 0, 0.5 and 1.
* **Tooltips are off** for now (`SetAllowTooltips(false)` in the player
  controller). The widgets keep their tooltip text, so one line turns
  them back on. Specifics: wing Standard tab has a Type combo
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

* No new modelling functionality beyond WingDesigner's: this is a port.
* Transform dialogs open just to the right of the scene tree, so they do not
  cover the 3D view's centre; a second, third and later dialog cascade down
  and to the right. Drag any of them wherever you like.
* Not yet carried over from the Qt version: the custom frameless title bar,
  Show Intermediate Features, Export STEP/STL (the OCCT STEP toolkits
  collide with the engine's own zlib/png/tiff DLLs -- see
  `AerotelicUE.Build.cs`), the colour theme, and the wing Control Surfaces
  tab (the C++ core has no control-surface model yet; the tab is a
  placeholder note). A section plane cuts without capping (see above), and
  the engine cannot rasterise SVG reference images.
* Legacy project files: WingDesigner's older format keys entities by `id`
  with a display `label` (`Projects/cessna_172.yaml`); the label becomes
  the name and the raw `features` graph is ignored (the mirror nodes it
  holds are covered by the entity's own `mirror` flag; its booleans are
  not recovered -- only the current `booleans:` list is read).
* Build gotchas: a TArray must not be appended with a reference to its own
  element (`Pts.Add(Pts[0])` reallocates under the reference); if a crash
  leaves `CrashReportClientEditor.exe` running it holds the module DLL and
  the next link fails with LNK1104 -- kill it first.
* The packaged build is `Development` configuration (native file dialogs via
  the DesktopPlatform module are not available in `Shipping`).
