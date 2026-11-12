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

* **Title bar**: the window is frameless (`bUseBorderlessWindow` in
  `Config/DefaultGame.ini`, WingDesigner's Qt.FramelessWindowHint) and the
  frame draws its own 32-px bar -- product and tagline on the left, the
  engine's Minimise / Maximise-or-Restore / Close buttons on the right --
  on the engine's `SWindowTitleBarArea`, so the OS handles dragging,
  snapping, the double-click to maximise and the edge resizing. In the
  Editor host, which has an OS border, the bar is collapsed.
* **Colour theme** (Edit -> Preferences -> Color Theme): WingDesigner's
  thirteen palettes (`ui/themes.py`) in their Dark / Light / Operational
  groups; selecting one applies it at once, Cancel restores the previous
  one, and the choice persists in the game user settings (the Qt app's
  `appearance/theme`; a name saved under the old product name is mapped).
  The theme colours the frame's chrome -- title bar, menu and tool bars,
  scene tree, panels, status bar, the inherited text colour -- and the 3D
  view's backdrop. Dialog windows, menus and the design panels' controls
  keep the engine's own dark style, so a light theme lightens the frame
  around them rather than the whole application.
* **Menu bar**: File (New, Save, Save As, Open, Export STEP/STL/3D PDF, Quit), Analysis
  (Mass Properties, Structural (Kratos FEA), Potential Flow (Kratos), High Order
  Panel, DUST (Vortex Particle Method), Section Aerodynamics), Edit
  (Preferences), Model (Airfoils, Boolean Operation, Material Properties,
  Units, Scale; it was called Geometry until 2026-11-07), View (Isometric /
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
* **Airfoils** (Model -> Airfoils, and the picker behind a section's
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
* **Copy and Paste** (Edit menu, Ctrl+C / Ctrl+V, and Copy on the tree
  row's own menu): duplicates the selected *parameterisation* as a second,
  wholly **independent** entity -- not a link. The copy is made the way a
  save and a reload are: the source model's own `Serialize()` spec goes to
  a fresh model's constructor (`ModelFromEntitySpec`), so the two share no
  state and neither one's edits, moves or deletion reach the other. The
  clipboard holds a whole `ProjectEntity` *by value*, which is already the
  value form of everything a copy needs (type, spec, placement, mirror,
  colour, hidden), so it is a snapshot: editing the source after copying
  does not change what pastes, and the clipboard survives the source being
  deleted, and New/Open -- an entity can be carried between projects.
  Pasting twice gives two separate copies.
  The copy is named like a newly added entity of its kind ("Wing 1" ->
  "Wing 2", the source's trailing number being the numbering `UniqueLabel`
  redoes), lands **where the original sits** (inventing an offset would
  only be a guess at how far; it is selected and its design panel opened,
  so Transform or an edit is the next move), and keeps the source's
  placement, mirror flag, colour and hidden state.
  **Only parameterisations can be copied.** A boolean result (and, until
  2026-11-08, a lattice) is a solid built *from* other entities with no
  parameterisation of its own -- there is nothing to hand a constructor, and a "copy" could only
  point at the same operands, which is the link this is meant not to be.
  Copy stays *enabled* for them so Ctrl+C can say why in the status bar
  rather than doing nothing without a word; the tree row's menu simply
  omits Copy for a derived row.
  `SpecFromEntityModel`/`ModelFromEntitySpec` are now the one place that
  knows which spec belongs to which entity type: save, load and copy/paste
  all go through them, so a new entity type is taught once instead of in
  three switches that can drift apart.
  Two input details, both found by driving the real application:
  the Editor host listens for keys *before* the focused widget
  (`OnApplicationPreInputKeyDownListener`), so the clipboard chords are
  skipped while a text field has the keyboard -- otherwise Ctrl+C in a
  numeric entry would copy the selected entity instead of the number; and
  editor binaries hosting the game (`-game`, `run_aerotelic_dev.bat`) have
  **both** that listener and the player controller's `ProcessShortcut`
  live, which ran every shortcut twice -- invisible for Ctrl+S, but one
  Ctrl+V pasted two copies. `AlreadyHandledThisFrame` now swallows the
  second, which also cures Ctrl+N/Ctrl+O prompting twice in that host.
  `AerotelicDocumentTest` covers the refusals, the unique name, that the
  copy has a model object of its own, that editing either side leaves the
  other alone in *both* directions, that moving or deleting the source
  does not disturb the copy, and that the clipboard pastes a snapshot
  rather than the source's later edits.
* **Mirroring** (entity menu, Mirrored): a port-side copy reflected about
  world Y = 0 from the *placed* shape, exactly WingDesigner's
  MirrorFeature -- an off-centre boom lands on the other side and a
  mirrored propeller comes out the opposite hand (independently of its own
  Spin direction, which the mirror reverses in turn). It follows every move
  and edit of its entity, inherits its colours, and is never edited on
  its own. Saved as the entity's `mirror:` flag.
* **Colour**: per entity and per body, chosen with the engine's own colour
  picker (AppFramework's SColorPicker, no alpha), floating beside the
  frame so the body takes each colour live as it is chosen; Cancel puts
  the previous one back. Defaults are
  WingDesigner's per-kind colours (steel blue wing, green fuselage, bronze
  nacelle, rose lofted nacelle, aluminium propeller, ochre primitive);
  resolution is body override, else entity override, else default. Setting
  an entity colour drops its bodies' overrides. Only overrides are saved
  (`color: [r, g, b]`, 0..1), as the Qt app does; `hidden: true` is
  saved too.
* **Surface** (View -> Surface): a modeless window with Metallic,
  Glossiness (specular) and Blur (roughness) sliders, Matte / Satin /
  Glossy / Metal presets, and two sliders for the Fresnel sheen below
  them, setting the finish of every body in the scene at once. The bodies
  are drawn with `Content/Materials/M_AerotelicSurface`
  (Color, Metallic, Specular and Roughness parameters, the Fresnel sheen,
  the section clip,
  darker back faces), an asset authored by `setup_materials.bat` (the
  `AerotelicMaterialSetup` commandlet, Editor binaries) because shaders
  cannot be compiled inside a packaged game; the same commandlet authors
  `M_AerotelicWire` (the edge styles) and `M_AerotelicGizmo` /
  `M_AerotelicGizmoTop` (the section gizmo's frame and its on-top arrow).
  All four are committed. If the surface asset is missing the bodies fall
  back to BasicShapeMaterial and the dialog has no effect; without the
  wire asset the edge styles draw no edges. View -> Ground Grid (Ctrl+G)
  shows or hides the ground.
* **Shadows** (View -> Shadows, 2026-10-15): a modeless window over the
  key light's shadow settings (`FAerotelicShadowStyle`, applied live by
  `UAerotelicDocumentSubsystem::ApplyShadowStyle` to the studio rig's
  directional lights): a Cast shadows checkbox, Default / No acne / Tight
  contact presets, and sliders for the slope (normal) bias and the depth
  bias in the light's own 0..2 units. The rig has one shadow-casting
  movable directional light, so its shadows are cascaded shadow maps, read
  from a depth map that needs a bias: too little and a surface shadows
  itself in stripes where it is nearly parallel to the light, too much and
  the shadow slides off its caster, which on a thin wing reads as a
  detached shadow. The slope bias is the normal-facing term, scaling the
  offset with the surface's inclination to the light, so it is the one to
  raise against stripes on a fuselage flank; the depth bias is a constant
  offset. Both go to every directional light, while the cast toggle is the
  key light's alone (`ForwardShadingPriority > 0`), since switching the
  two fill lights on would put three competing shadows on the model. The
  game mode re-applies the settings after it spawns the rig, because the
  style lives in the engine subsystem and outlives any one world. Virtual
  shadow maps are off in this project (`r.Shadow.Virtual.Enable` defaults
  to 0 and `Config/DefaultEngine.ini` does not set it), so the engine's
  `r.Shadow.Virtual.NormalBias` is not the knob here and the dialog says
  so.
* **Fresnel sheen** (2026-10-15): the surface material adds the engine's
  `Fresnel` node to its emissive output, scaled by `FresnelIntensity`,
  sharpened by `FresnelExponent`, lifted by `FresnelBaseReflect` and
  tinted by `FresnelColor` (a little cooler than white, so it reads as sky
  light rather than a halo). The factor is zero where the surface faces
  the eye and rises to one at the silhouette, so an untextured monochrome
  body shows its form: a fuselage stops fading into the backdrop and a
  leading edge separates from the skin behind it. Emissive rather than a
  scene light, so it needs no lighting rig, cannot blow out the lit
  response, and is still discarded by the section clip -- a cut body does
  not glow along the cut. The intensity and the exponent are the two
  bottom sliders of View -> Surface (`FAerotelicSurfaceStyle::
  FresnelIntensity` and `FresnelExponent`, defaults 0.35 and 3, intensity
  0 switching it off); the presets leave them alone, since the sheen is a
  reading aid and not part of the part's finish. The sheen needs the
  material asset re-authored (`setup_materials.bat`); with an older asset
  the two parameters simply do not exist and setting them is a no-op, so
  the viewport keeps the plain lit shading.
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
* **Export STL / Export STEP** (File menu; main_window.py's
  `_export_stl_dialog` / `_export_step_dialog`): both start from the same
  body picker as the boolean dialog -- every entity, each mirrored half as
  a row of its own, every boolean result -- so "what can I export" and
  "what can I operate on" are one question. STL: one mesh density (linear
  deflection 0.01, angular 20 degrees), the mesh statistics (bodies,
  nodes, triangles) recomputed on every change, binary or ASCII; copies of
  the placed shapes are meshed into one compound and the file holds
  exactly that triangulation, written by the application itself. STEP:
  the loft station tolerance (WingDesigner's EXPORT_STATION_TOL, 0.2 mm,
  finer than the interactive preview) with "Rebuild bodies at this
  tolerance" (wings, fuselages, lofted nacelles and propellers are
  re-lofted; booleans and mirrors' sources likewise, then reflected), and
  "Write named parts (STEP assembly)" for an XCAF product structure. The
  STEP file is written by `aerotelic_step_writer.exe`, a console program
  built by the core's CMake (`core/src/tools/step_writer.cpp`) and run as
  a separate process over temporary .brep files: OCCT's STEP toolkits
  drag in zlib/libpng/freetype/tiff DLLs under the names the engine loads
  its own copies of, which must never enter the engine's process (see
  `AerotelicUE.Build.cs`). `tools\stage_step_writer.ps1` copies the writer
  and its DLL closure (found with dumpbin) into
  `Binaries\Win64\StepWriter\`, which the packaged build stages; a
  developer checkout falls back to the core build tree. Without the
  writer, Export STEP says so and does nothing.
* **Export 3D PDF** (File menu, 2026-10-16): the chosen bodies as one
  interactive model on a PDF page -- drag to orbit, and the reader's
  model tree lists the bodies by name, each in its scene colour and
  separately hideable. Same dialog bones as Export STL: the shared body
  picker (boolean results included -- a 3D PDF carries
  triangles, so anything that meshes exports), one mesh density with the
  statistics recomputed on every change, then the document title (the
  project's name by default), the page size (A4 / Letter / A3, portrait
  or landscape) and "Compress the 3D model stream".
  The file is written by the application itself:
  `core/include/aerotelic/pdf3d.hpp` builds a **U3D** model
  (ECMA-363) and wraps it in a PDF 3D annotation (ISO 32000-1 13.6),
  with a default view aimed at the model's bounding sphere from an
  isometric direction and `/3DA` set to activate the model when the page
  opens. Each body becomes its own CLOD mesh generator, model node,
  shader and material, which is what gives the model tree a named row per
  body; positions are welded and corner normals averaged within a 30
  degree crease, so a fuselage shades smoothly while a trailing edge
  keeps its edge. Written in U3D's **no-compression profile** (9.4.1.2,
  profile bit 0x4), where every field the block definitions mark
  compressed is written as the plain value of its type: that keeps the
  encoder to little-endian field writes -- no arithmetic coder, and no
  histogram state that has to evolve in lockstep with the reader's --
  and the PDF's own Flate filter takes the size back. The deflate is the
  **engine's** zlib (`FCompression`, `NAME_Zlib`), passed in as a
  callback, because the core must not link a second copy of that name
  (the collision `AerotelicUE.Build.cs` documents at length).
  `core/tests/pdf3d_check.cpp` walks the U3D blocks against their
  declared sizes, checks the declaration and the base mesh agree, checks
  the welding and the crease angle, and checks the PDF's cross-reference
  table points at its objects; it writes
  `core/build/aerotelic_3d_sample.pdf` to open in Acrobat, which is the
  only thing that can say whether a *reader* draws it.
  KNOWN OPEN: that last step has not been done -- see "Known limits".
* **Boolean operations** (Model -> Boolean Operation): WingDesigner's
  dialog -- the operands in order (every entity, the mirrored half of each
  mirrored one as its own row, every earlier boolean result), Add / Select
  All / Remove / Remove All (2026-11-04, the reverse of Select All; the
  same picker serves the exports and the analyses) / Up / Down, the
  operation (Fuse, Cut = first minus the
  rest, Common) and a unique name ("BoolOp n" by default). The result is a
  new scene item of kind boolean: the operands' placed MESHES folded left
  to right through the core's mesh boolean (`core/include/aerotelic/
  mesh_boolean.hpp`, 2026-10-12, replacing the OpenCascade boolean):
  triangle-triangle intersections on Shewchuk's exact predicates, the cut
  triangles re-triangulated in their planes with the intersection curve
  as constraints, pieces kept by the sign of the other body's distance
  field, vertices on the curve shared to the bit. Two bodies sharing a
  wall vertex for vertex (a wing and its mirror image on the symmetry
  plane) are joined along the wall's rim; a wall only approximately
  shared (the root of a wing with dihedral overshoots the plane by its
  thickness times the dihedral's sine) is a genuine intersection and is
  cut. Refused, with the place named: two triangles overlapping in one
  plane, a vertex of one body exactly on a face of the other, an edge
  exactly in a face. (Mirror-image edges that cross each other exactly on
  the symmetry plane are the common case and are fine: they meet in one
  shared vertex of the curve.) Where the curve passes a hair from a vertex
  or an edge of a triangle it cuts, the cut leaves sliver pieces (the
  airframe had 495 under one degree); a seam pass (2026-10-13) removes
  them worst first by collapsing the sliver's shortest edge -- an original
  vertex onto a seam point, two seam points onto one, never a seam point
  off the seam -- when that moves the surface by under 1e-4 of the
  bounding diagonal, or by flipping its longest edge when the flip raises
  the pair's smaller angle and moves the surface by less than that (3 of
  the 495 remain, where neither is possible). Pieces are classified by
  connected region rather than one by one, so a sliver hugging the seam
  cannot be misjudged. The whole P-38 airframe fuses in half a second
  (`core/tests/mesh_boolean_check.cpp`). (Known open while the
  tetrahedral mesher existed, to 2026-11-08: it degraded on the union --
  the coarse body's cut triangles fan from a far corner to the dense seam,
  edges no Delaunay sphere can hold; the experimental in-plane refinement
  behind `MeshBooleanOptions::refine_min_angle_deg` (off) fixed some
  junctions and worsened others -- see the header. The fans are still
  there and still matter to a shell grid, which is why the option stays.)
  The result is drawn in
  lilac as one body, shown, coloured, renamed and deleted like any row
  (no design panel, placement or mirror of its own), carries a per-body
  structure (skin, ribs) like any other body, and has no B-rep: it is not
  exported to STEP (export the operands and combine them downstream). The
  operands stay in the scene, so hide them to see the result alone. It
  follows its operands: editing or moving one rebuilds it, deleting one
  deletes it (with a warning). An operation that is refused is reported
  and adds nothing. Saved under `booleans:` by name (`operands: [Wing, {mirror: Wing}]`,
  `operation`, `hidden`, `color`, per-body `bodies:`), validated on load
  as WingDesigner does; a boolean that cannot be rebuilt is left out with
  a message. No seam fillet (removed from WingDesigner's UI 2026-09-29).
  View -> Show Intermediate Features stays disabled: the fold's steps are
  not materialised here.
* **Welcome screen** (Help -> Welcome Screen, and at start-up unless
  ticked off; `-NoWelcome` or `-unattended` skips it): WingDesigner's
  splash card -- the artwork from `Resources/`, product identity and
  version (from `Resources/VERSION` or `VERSION`, else `0.9.0-dev`), the
  licence block, the legal small print, the tutorial link, "Show this
  screen at start-up" (saved in the game user settings) and Close.
  Branding and the disclaimer text live in `Branding/AerotelicBranding`
  and are, as in WingDesigner, placeholders not reviewed by counsel.
* **Tutorial Projects** (Help -> Tutorial Projects, F1, or from the
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
  an object selects it: its triangulation edges are drawn in orange over
  its surface, which keeps its own colour (so a colour picked for it shows
  at once, as in WingDesigner, whose viewer has no selection highlight at
  all); clicking empty space clears the selection. Rendering is confined to this region (the frame sets the local
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
  its own; the propeller's Help includes the blade tabs' text). Every
  panel edits a *draft* of its entity and ends in an Apply / Revert
  footer: nothing in the scene changes until Apply (see *Apply in the
  design panels and the Transform dialog* below).
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
* **Optional spinner** (propeller panel, 2026-10-16): a **Build the
  spinner** checkbox on the Propeller tab, **on by default**
  (`PropellerSpec::spinner`). Cleared, only the blades are built -- a
  bare hub, or a rotor whose hub is modelled by other entities -- and
  the body list starts at `Blade 1` instead of `Spinner`. The profile
  and its length are kept while it is off and the Spinner tab stays
  editable, so switching it back on restores the shape unchanged. Two
  checks exist only to keep the blade roots buried in the spinner --
  that the profile clears the axis, and that the blade root radius is
  inside the spinner radius -- and both are skipped when there is no
  spinner, because then the roots are *meant* to be bare. The derived
  Spinner diameter and Spinner length read `none` rather than a number,
  since the profile still has a size but nothing is built from it.
  A fused export of a spinner-less propeller may come back as several
  solids: the blades only ever meet at the hub.
  Saved as `spinner: true|false`; **absent means on**, so a project
  written before the flag existed keeps its spinner rather than quietly
  losing a body. Bodies are matched by name on rebuild
  (`AerotelicEntityActor`) and on load, so toggling it does not shift
  the blades' own colours or visibility onto the wrong body -- the
  Spinner's own colour is simply forgotten while it is off.
  `propeller_fixture_check` covers the body list, the blades being
  byte-identical either way, the relaxed validation, and the round
  trip; `project_yaml_check` covers both flag states and a legacy node
  with no `spinner` key at all.
* **Spin direction** (propeller panel): a **Spin direction** combo on the
  Propeller tab, **Positive** or **Negative**
  (`PropellerSpec::spin_direction`). The convention is the right-hand
  rule with the thumb along the shaft pointing *from the propeller
  towards the spinner* -- that is, forward, the spinner's tip being the
  front of the hub. Since `+x` is aft, that thumb points along `-x`, so
  a positive spin is the **negative** sense about the model's `+x`:
  clockwise seen from behind, the usual right-hand tractor propeller,
  and the only hand the model had before this field -- hence the
  default, and hence `spin_direction:` **absent means positive**, so an
  older project opens turning as it was drawn.
  Negative is the mirror image. The blade is reflected in the plane of
  the axis and the span -- `z = 0` after `BuildMeshes`' turn, the chord
  lying along `z` there -- which reverses its camber and twist, so it
  still leads with the leading edge and still thrusts forward while
  turning the other way. The reflection is folded into that turn
  (negating its `z` row) rather than applied after, so the blades keep
  their azimuths and `Blade 1` still sits at zero, which the DUST writer
  relies on; `TriMesh::Transform` reverses the winding on its own, the
  matrix now having determinant -1. `BladeSolid` does the same with a
  `gp_Trsf` mirror about `gp_Ax2(origin, (0,0,1))`, composed as
  `mirror * turn`.
  Nothing else moves: the spinner is a body of revolution, and the
  blade's twist curve and every derived metric read the same either way
  -- the hand is in the geometry, not in the sign of beta. It composes
  with mirroring the *entity*, which also reverses the hand, so a
  mirrored negative-spin propeller turns like an unmirrored positive
  one. `propeller_fixture_check` checks that `Blade 1` negative is
  `Blade 1` positive reflected in `z` vertex for vertex, that both
  enclose the same volume outward (a reflection that forgot the winding
  would come back inside out), that the blade really is handed in `z` at
  all (else the setting would do nothing), that the metrics do not
  budge, and -- on a one-blade propeller, so the blade opposite does not
  cancel it -- that the OCCT solid's centroid flips in `z` and not along
  the axis, cross-checking the `gp_Trsf` mirror against the matrix one.
* **Transform dialog** (from the tree's context menu): one modeless window
  per entity, a tab each for Translation and Rotation, the rotation tab
  offering a choice of approach -- an axis and an angle, or three Euler
  angles. See *The Transform dialog* below.
* **Status bar**: last action on the left, project path on the right.

## Meshing and distance fields (2026-10-11)

* **OCCT-free triangulation** (`core/include/aerotelic/mesh.hpp`,
  `src/mesh.cpp`): every lofted body is a chain of ruled surfaces between
  its section curves, so `TriangulateLoft` discretises the section curves
  at one shared, ordered set of along-curve parameters, connects point `j`
  of consecutive stations by rulings, and splits each quadrilateral along
  its shorter diagonal. Parameters are added wherever the chord between two
  consecutive points strays more than the tolerance from the curve, checked
  on *every* station (the error of a ruled surface lives on its end curves);
  stations are added wherever the ruled strip strays from the surface
  between them. Ends are capped by ear clipping; apexes (a fuselage nose, a
  spinner tip) collapse to a vertex; a rim of zero wall thickness shares
  its vertices; a section squeezed flat (a fuselage end of zero width but
  some height -- the P-38 booms' knife-edge tails) shares the vertices it
  retraces, gets no cap, and the strip on either side is split away from
  the flat section, so the knife edge is a true shared edge. The result is
  a closed, outward-oriented manifold within the tolerance of the surface
  (`project_yaml_check` asserts this for every body of the p38 project). `ISolidModel::BuildMeshes(tolerance)` is
  implemented by every model: wing (section curves placed through
  `AirfoilCoordsToXyz`, stations from `LoftStations`), fuselage (the conic
  section as a curve, `ConicCrossSectionPoint`), both nacelles (one
  periodic chain: outer skin, trailing-edge rim, inner skin, lip rim),
  propeller (spinner profile revolved as rings; blades from the hub station
  outward, turned -- and reflected, for a negative spin direction -- and
  placed as `BuildSolid` does) and the primitives.
  The viewport, the mirror and STL export use these meshes; the OCCT solid
  is only built on demand (booleans, STEP export) or for a model without a
  mesh builder (a boolean result; a nacelle with chevrons, which needs the
  boolean cut). `tests/mesh_check.cpp` compares every body's volume with
  its definition (integrated section areas) and OCCT's tessellation.
  Measured: the default wing meshes in 0.02 s; OCCT's own loft comes out a
  few percent larger because `ThruSections` pairs the section curves by its
  own reparametrisation, and `BRepGProp::VolumeProperties` is unreliable on
  it (2.52 against 2.95-3.05 from every other measure).
* **Signed distance field** (`sdf.hpp`, `src/sdf.cpp`): `MeshSdf` holds a
  BVH (binary tree of boxes over the triangles, median split on the longest
  axis, leaves of 4) and answers closest point and signed distance with a
  nearest-box-first descent that never enters a box farther than the best
  distance found; the sign is the angle-weighted pseudo-normal of the
  feature the closest point lies on (Baerentzen & Aanaes). Measured: 2.3 us
  per query on a 2,700-triangle wing, BVH equal to brute force on every
  tested point, sign equal to ray parity on every tested point
  (`tests/sdf_check.cpp`). The distance field stays because the mesh
  boolean needs it; the lattice fields that were its first customer
  (`lattice.hpp`, OpenLattice's TPMS / strut / honeycomb cells, the
  marching-tetrahedra extractor) were removed on 2026-11-08 -- see
  *Shells only* under *Materials and the structural analysis*.
* **Per-body structure** (2026-11-08; replaces the Lattice menu and the
  lattice scene entity of 2026-10-11..11-04). The
  structure of a solid BODY -- a wing's Main and each flap, a propeller's
  spinner and blades, a boolean result's Body k -- is an attribute of
  that body (`core/include/aerotelic/body_structure.hpp`, saved under the
  entity's `bodies:` list): its material and skin thickness (0 = a
  solid), any number of PLANE NETWORKS (ribs, spars,
  frames: planes normal to x, y, z or a custom axis, a start, a spacing,
  a count, a thickness, a material, and a height -- 0 spans the interior
  wall to wall, positive stops that far in from the surface), reserved
  volumes by scene name, or an asserted mass. Start and spacing are
  geometry in the parameterisation's full-scale frame (placed, scaled and
  mirrored with the body); skin, rib thickness and height are physical
  build figures in project units, not scaled by the model scale. Boolean
  result bodies own their structure (no inheritance; *Copy Structure From
  Operand* on the result's row is the starting point). The Scene tree's
  body rows (and a single-body entity's row) have *Structure...*: a
  modeless dialog with Skin / Planes / Mass tabs, Apply, Revert
  and *Generate*; while the Planes tab is edited the 3D view shows every
  plane as a translucent rectangle over the body (the one being edited
  amber), through `Standalone/AerotelicPlanePreviewActor` and the
  document's structure preview. *Generate* (also on the row menu) builds
  that body's structure alone and gives it LAYERS -- "Skin" and one
  per network -- listed under its row with their own visibility and
  colour, exported as STL parts and weighed by the mass report. Since the
  evening of 2026-11-08 the layers are SHELLS (see *Structure as shells:
  no offset surfaces* under *Materials and the structural analysis*): the
  Skin layer is the body's own surface with the thickness as an attribute
  and has no mesh of its own -- its row shows and hides the body's
  surface, so the ribs can be viewed alone -- and each network's layer is
  the planar discs its planes cut out of the body, meshed by gmsh
  (`tools/rib_gmsh.py`) on the body's own cut grid with the reserved
  volumes' sections as holes, drawn two-sided under the body (and its
  mirrored half) in the body's frame. No scene entity is created. A
  generated body
  follows every later edit of its body, of the booleans over it and of
  the volumes its exclusions name (the document's rebuild queue, by
  (entity, body)); a structure edit that changes the geometry regenerates
  a generated body; bodies with a skin are regenerated on load. As first
  written that morning the generation hollowed the body on MeshLib and
  cut the ribs as slabs from the hollow (`tools/structure_meshlib.py`,
  gone the same night -- see *Hollowing and ribs on MeshLib* below for
  the record). Old files' `lattices:` entries are skipped
  by name, and a body's `infill:` key (the lattice infill the structure
  carried for a few days) is dropped; both are listed in "Project Opened
  With Problems".
* **Structural analysis of a structured body** (Analysis > Structural):
  `Entity/AerotelicFeaModelBuilder` builds every chosen body the same way
  for the dialog and the commandlets, and every element it builds is a
  SHELL. The body's skin is shells of the skin's material and thickness
  on the body's own grid. Its plane networks enter as RIB SHELLS: the
  body's own grid is first cut by every plane (`core/mesh_cut.hpp`
  `CutMeshByPlanes`: triangles split at the planes, one crossing point per
  edge, vertices within a tolerance snapped onto the plane, closed in and
  out), so each rib's rim is a loop of skin nodes; `tools/rib_gmsh.py`
  (`Entity/AerotelicRibGmsh`, a separate process in the WingDesigner
  environment where gmsh is pip-installed) meshes each rib's disc in 2-D
  in a scratch model from the rim (transfinite rim
  lines, no new rim nodes) and splits it along the chords where networks
  cross (the same transfinite node count on both discs, so they share
  their chord nodes). A limited height reports only the part of the disc
  within it as rib faces. Since the evening of 2026-11-08 *Generate*
  builds the SAME discs on the same cut surface with the same mesher for
  the viewport and the mass report (plus the reserved volumes' sections
  as holes), so what is drawn, weighed and solved is one rib. In
  `structural_fea.py` rib faces are shells
  of their region's `rib` material and thickness and nothing else: not
  wetted, no surface load, no part in the mean normal displacement; their
  mass counts; their peak stress is a `kind: "rib"` entry and the VTK
  marks them `kind` 2. One region per body and one per network. A body
  whose skin thickness is 0 (a solid) is refused -- "shells only; give it
  a skin thickness" -- there being no solid elements left to give it.
  Checks:
  `core/tests/mesh_cut_check.cpp`, `tools/rib_gmsh_check.py` (the 1 x 10 x
  1 box with three ribs and a crossing spar: area 13 exact, chord nodes
  shared, every rim node a skin node), and the document
  commandlet's "a box beam with ribs" step (the cut's volume, one
  unit-square rim loop per plane on the skin, the mass rho t A of skin
  plus ribs, the clamp's reactions, the tip deflection a little under the
  unribbed Timoshenko box, a "rib" entry, kind-2 VTK cells).
* **Hollowing and ribs on MeshLib** (2026-10-14; removed 2026-11-08,
  late; kept as the record). From 2026-10-14 until the evening of
  2026-11-08 a body's layers were made by MeshLib (pip `meshlib` in the
  WingDesigner micromamba environment; `tools/structure_meshlib.py` and
  its self-check `structure_meshlib_check.py`, named `lattice_meshlib*.py`
  until that morning), run as a separate process the way the STEP writer
  is -- the placed bodies out as `.tri` files, the layers back -- because
  the sampled skin and the tetrahedra-bounded skin had both shown visible
  faults (uneven thickness, stair-steps where the interior met the offset,
  a ragged interior). Each body was offset inward by its skin thickness
  on MeshLib's OpenVDB distance field (voxel a quarter of the skin under a
  voxel budget), which cannot self-intersect and rounds a concave crease
  from the inside -- the lesson of *Explicit skin layer* below; the offset
  was decimated, small pockets filled, and "Skin" was the body's surface
  outside and the inverted offset inside, a closed solid; the union of
  the offset interiors minus the reserved volumes was the hollow the rib
  SLABS were cut from (slabs about the planes intersected with the
  hollow, a limited height by subtracting a deeper offset, the slab
  subtracted from the domain and never the rib, since MeshLib's boolean
  fails on coincident faces), and until the morning of 2026-11-08 the
  lattice infill was marching-cubed and booleaned to that hollow as well.
  What ended it is in *Structure as shells: no offset surfaces* under
  *Materials and the structural analysis*: the closed Skin layer looked
  like an offset copy of the body and hid the ribs, and on the 1 mm skin
  an airframe actually has the voxel budget made the offset useless.
  What survives of it is the process plumbing -- `Entity/AerotelicMeshLib.h`
  is now only the shared Python job runner (`FindPython`, `RunJob`,
  `WriteTri` / `ReadTri`, `FJobProgress`) for the rib mesher, the
  structural analysis and the potential flow -- and the `.tri` format.
* **The offset remesh cannot run away** (2026-11-01; removed with the
  hollowing 2026-11-08, late). Generating a lattice on p38's `BoolOp 1`
  (six shells, every body carrying the FEA work's 1 mm skin) hung the
  application -- 15 minutes, 18 GB, the machine in swap -- because the
  inner surface's remesh edge was two skins = 2 mm while the voxel budget
  had capped the offset voxel at 19.5 mm over the airframe's bounds, so a
  19.5 mm marching-cubes surface was being remeshed to a 2 mm edge over
  ~148 m^2 (~85 M triangles, MeshLib's split cap stopping at 20 M per
  body). Two guards went in: a floor on the remesh edge of two offset
  voxels (`kInnerRemeshEdgeOverVoxel`; the surface carries no detail
  finer than its voxel) and a job-wide `MAX_INNER_FACES` budget of 1 M
  in the script; the same job then took 5 s. The same budget is what,
  on 2026-11-08, made a 1 mm skin's offset voxel 7 mm and the inner
  surface unusable ("the inner surface touches the body's surface") --
  the guards made the offset finish, not resolve a thin skin -- and the
  offset route went with it. The job folders `%TEMP%\aerotelic_structure_*`
  (then `aerotelic_lattice_*`) are no longer written; a killed editor's
  child `python.exe` still keeps running until stopped, for the jobs that
  remain.
* **Explicit skin layer** (`shell.hpp`, `src/shell.cpp`, 2026-10-11
  afternoon; the first step of the tetrahedral route Matt chose; REMOVED
  with that route on 2026-11-08, kept here as the record): the
  body's surface triangulation offset inward by the skin thickness as a
  layer of prisms split into tetrahedra (Dompierre's lowest-index rule, so
  the tets are conformal across prisms), the inner surface left as the
  closed boundary of the interior (lattice) domain. No voxels: each offset
  node is placed by walking inward along the vertex normal on the body's
  signed-distance field and stopping where the field reaches the skin
  depth, where it stops deepening (the mid-surface of a region thinner
  than two skins: the node stays there, the vertex is "collapsed", and the
  skin fills the thickness -- a trailing edge), or where the surface is
  far nearer than the distance walked (a vertex already on the
  mid-surface, e.g. on the trailing edge itself, which stays put). The
  walk only moves to more negative field values, so no node can land in
  the opposite skin or outside the body. Then: an edge-slope bound on the
  walks, welding of coincident offset nodes (a symmetric section's two
  sides meet on the same mid-surface points), cancellation of the
  coincident opposite triangles of the collapsed double sheet, sealing of
  the remaining pinch lenses (Euler circuit of the open edges, split into
  simple cycles, each capped in its plane), and a relaxation that scales
  back the walks of any prism whose three tets disagree in sign (the exact
  fold test) -- a leading edge whose radius is near the skin thickness is
  where that happens. `tests/shell_check.cpp`: a sphere's shell to 0.01%,
  a plate thinner than two skins (all skin, no interior), and the P-38's
  port wing (1% root-chord skin: 5,919 tets, no folds, every tet inside
  the body, skin volume within 1.1% of the field-integrated shell, the
  interior domain a closed manifold enclosing wing minus skin; writes
  `build/p38_port_wing_skin.vtk` and the inner/outer STLs). SUPERSEDED in
  the application (2026-10-12 night) by the skin tetrahedra of the
  tetrahedral mesher (`tet.hpp`, since removed): the body's own
  surface triangles outside, the exact iso-surface of its distance field
  at the skin depth inside. The offset copy could not represent an inward
  offset across a concave crease -- where the tailplane's leading edge
  meets a boom on the fused airframe its straight inner edges bridged the
  corner and came out through the leading edge by 10 to 13 mm (the wing's
  trailing edge at the boom, 9 mm) -- nor at a leading edge sharper than
  the skin, where it folded. The iso-surface rounds a concave corner from
  the inside and cannot self-intersect, which is the lesson that
  survives: an inward offset is a distance-field operation, not a copy of
  the surface, and that is what MeshLib's OpenVDB hollowing did from
  2026-10-14 until the hollowing itself went on the evening of 2026-11-08
  (see *Hollowing and ribs on MeshLib* above; the structure is shells
  with a thickness attribute now, and nothing is offset at all). With the
  tetrahedra gone on 2026-11-08 `shell.hpp` had no caller left and went
  with them.
  Also fixed on the way: `WingModel::AirfoilAt` now recognises
  two sections holding the same curve (as a reloaded project has them) and
  no longer re-fits a blend whose different parameterisation made the
  loft bunch stations at the root and tip.
* **Negative chord** -- a planform whose trailing-edge curve crosses ahead
  of its leading-edge curve (the P-38 tailplane's tip as first saved, by
  10 mm) turns the section inside out and folds the loft; the wing's
  distance field then reports the wrong side near the fold. `WingModel`
  refuses to build (`RequirePositiveChord()`, every build path) naming the
  span range; a zero chord (a pointed tip, the P-38 fin) is allowed. The
  entity actor keeps the reason (`GetBuildError()`), the project loader
  lists every entity that could not be built in one message, and the wing
  editor reports it once when an edit causes it. The shipped p38.yaml has
  the tailplane's tip pole corrected.
* **STL export** now writes the models' own meshes at the linear
  deflection (OCCT's tessellation, with the angular deflection, only for
  bodies that need it). An ASCII STL holds one `solid <entity>/<body>` per
  body (spaces as underscores), so a body's Skin and Rib layers arrive as
  separate parts; binary STL has no parts and holds the union. Since
  2026-11-08 (late) the structure's parts are OPEN shell surfaces -- the
  skin is the body's surface, a rib a planar disc -- not printable solids;
  see *Structure as shells: no offset surfaces*.

### Tetrahedral mesh of the bodies (2026-10-12; removed 2026-11-08)

The constrained Delaunay tetrahedral mesher of 2026-10-12 (`tet.hpp`,
Bowyer-Watson on Shewchuk's predicates, segment and face recovery, the
skin / core split on the distance field's iso-surface, its Delaunay
refinement and improvement passes, the nested-surface mode of 2026-10-14
and the gmsh route of 2026-10-15 that replaced it on the application's
path) was removed on 2026-11-08 (see *Shells only* under *Materials and
the structural analysis*); its matched-row loft rule
(`LoftSource::mirror_v` / `mirror_u`) survives in `mesh.hpp` because the
mesh boolean and the panel grids still benefit from it.

## High order panel method (2026-10-25)

**Analysis -> High Order Panel** solves the incompressible flow about the
lofted bodies of the scene with the higher-order panel method of Johnson
(NASA CR-3079), which lives entirely in the core (`panel_integrals.cpp`,
`panel_method.cpp`, `wing_panels.cpp`; see "Known limits" for what of the
report is not there). It is independent of the Kratos route below, which
is untouched.

The dialog (`Dialogs/HighOrderPanelDialog.cpp`) opens with the two-list
body picker the export dialogs use, offering every wing, fuselage, nacelle
and primitive (one row per entity; a boolean result is a
triangle mesh with no sections to spline, and a propeller is left out).
A wing is paneled by `WingNetworks` -- the sections round as a closed
surface with the interior potential held at zero and the sources
prescribed -- with a wake shed along the free stream, which is what
carries the lift. (The dialog's earlier "mean camber surface" option, a
doublet sheet loaded by Cp_lower − Cp_upper, was removed 2026-10-28 at
Matt's request: the app integrates Cp on the faceted closed geometry,
and the post-processing has no two-sided path at all. The solver keeps
its sheet support only for the report's thin-wing validation cases in
`panel_method_check.cpp`.) Every other body is a closed source network built by
`LoftNetwork` from the model's `BuildLofts()` -- the same section function
its triangulation comes from, sampled on the loft's own stations and
section corners plus the dialog's "along" and "around" counts, apexes as
rows collapsed to a point (triangular panels), a periodic chain (a
nacelle) closed on a seam, flat panels for a box. Networks are built in
the body's frame and placed as it sits (`AerotelicMeshing::PlaceMesh`),
or for a body with the plane on, built from the loft placed into the world
first.

**Symmetry plane at y = 0** is a checkbox, on by default when a chosen
entity is mirrored or a wing root sits on the plane. On, every network is
solved with its image (`Options::symmetry_y`): a mirrored entity appears
once and the image is its other half; a wing root on the plane gets the
∂μ/∂n = 0 condition there (the root station is laid in the plane exactly,
`WingPaneling::pin_root_to_plane`: a section is placed normal to the
dihedral curve, so a dihedral slope at the root tilts the root section
and its corners would otherwise stick ± thickness · sin(dihedral) through
the plane -- 0.017 on the P-38 -- cross their own image and blow the root
column up); a body that straddles the plane (a fuselage on the
centreline) is cut at y = 0 and only its y >= 0 half paneled
(`BodyPaneling::half_plane_y`), since paneling it whole would double it
with its image. Off, a mirrored entity's reflected copy is paneled as well
(`MirroredNetwork`: y -> -y, columns reversed to keep the normals
outward). Bodies whose placed bounds overlap are flagged above the picker:
they solve, but a wing root buried in a fuselage leaves panels inside the
other body that are not a wetted surface, so pressures near the junction
are not to be trusted.

Nothing dimensional is asked for: a potential flow is similar at every
speed and density, Cp = 1 - (V/V∞)² and C_F = -∮ Cp n dA / S_ref, so the
dialog deals in coefficients only, on a reference area (default: the
planform area of the chosen wings, both halves; with no wing the first
body's frontal ellipse) and a reference length for the moments (the first
root chord, or the body's length). A wing and a fuselage, a thousand
unknowns, take a second or two per angle.

**Post-processing (2026-10-26).** The dialog is modeless and tabbed:

* *Set-up* -- the bodies, the paneling, the operating conditions and the
  references. The angle of attack and the angle of sideslip each take one
  value or a comma separated list (`0, 2, 4, 6`); a list in BOTH is
  refused (it would be a grid of cases, not a sweep), and sideslip is
  held at zero for now (the symmetry plane and the mirrored loads assume a
  stream in the plane of symmetry; a non-zero value is refused with a
  note). One case is paneled per angle -- the wake sheds along each
  stream -- and the solves run on a worker thread one after another, the
  status bar counting them; the dialog stays live. A solution-file path
  writes one legacy VTK polydata file per case (`_aoa4` suffixes when
  there are several; SI lengths, as the Kratos file) with `Cp`,
  `doublet`, `source`, `curvature_delta`, `network` and
  `velocity_upper` per panel. The tab's only read-out is a summary
  (panels, unknowns, curvature measure, time); the coefficients live in
  the fourth tab.
* *Surface Pressure* -- the bodies coloured by Cp, rendered by the engine
  itself through a scene capture (`Widgets/SceneView`,
  `Standalone/AerotelicSceneViewActor`) into a texture the tab shows,
  with the main view's mouse conventions (left-drag orbits,
  right/middle-drag pans, wheel zooms, Home frames). The capture carries
  its own lighting rig fixed to the eye (2026-10-27): a key light along
  the line of sight and four fills from either side, above and below, as
  children of the capture component, so every face the picture shows is
  lit whichever way the result is turned -- the scene's studio rig, fixed
  in the world, left the far side in shadow. The rig is on lighting
  channel 2 alone and the result mesh is put on channel 2 alone, so the
  rig lights nothing in the main scene (channel 0) and the studio rig
  lights nothing of the result. The mesh is
  `Standalone/AerotelicColoredMeshActor`: every non-wake network's corner
  grid with the panel pressures averaged onto the corners, both windings,
  visible to captures only (never in the main view),
  casting no shadow, drawn with `M_AerotelicVertexColor` (lit, two sided,
  base colour from the vertex colour, which the actor linearises --
  vertex streams get no sRGB decode). The colour bar
  (`Widgets/ColorBar`) on the right is clickable: a popup with the
  minimum, the maximum, an "auto range" over every solution (the default,
  so the colours mean the same from one case to the next; the 2nd to 98th
  percentile of the pressures, so the few junction panels of a fin buried
  in a boom, at Cp -100, do not swamp the scale) and the colour map, from
  the core's catalogue (`core/include/aerotelic/colormaps.hpp`: Turbo,
  Jet, Rainbow, Viridis, Plasma, Inferno, Magma, Cividis, Cool-Warm,
  Spectral, Red-Blue, Red-Yellow-Blue, Blues, Yellow-Orange-Red, Hot,
  Cool, Grey, Cubehelix, Twilight; all free tables or formulae,
  subsampled to 33 stops). A stepper at the bottom left moves between the
  solutions.
* *Span Loading* -- a stepper picks the view (another the solution): one
  per wing, then with more than one body *All bodies*. For a wing, on
  the left axis, the lift per unit span over the dynamic pressure, l / q
  = cl·c, a length in the project's units (Matt: the distribution itself,
  not cl·c / c_ref, "a surrogate scaled by a constant"), against the span
  fraction. No strips (Matt, 2026-10-28: "finite strip widths fall apart
  where the chord goes to zero"): the wing's own network is cut by a
  plane normal to its placed span axis every 5% of the semi-span
  (`kLoadCuts`), root station to tip station, and `CutSection`
  (`core/.../panel_loads.hpp`) intersects the plane with the panels --
  one segment per panel crossed, its ends on the panel's two spanwise
  edges, Cp interpolated along the span between the panel's centre and
  its neighbour's -- and integrates ∮ −Cp (t × axis) dl along the
  section's closed outline. That is the force per unit span exactly: for
  a surface parametrised by the cut and the span, n dA = (t × ∂P/∂y) dl
  dy, and the components normal to the axis do not depend on ∂P/∂y's
  sweep and taper. Resolved normal to the stream and to the axis -- lift
  for a wing, side load for a fin -- it is plotted as is; a cut at a
  collapsed tip has zero length and carries nothing. Dashed beside it,
  the elliptic distribution of the same integral (`EllipticLoad`: 4 L /
  (π b) √(1 − (2s/b)²) over the cuts' extent; the total is the trapezoid
  rule over the cuts), so the two curves enclose the same total lift,
  which the title gives as L / q and C_L. On the right axis
  (`FAerotelicPlotSeries::bRightAxis`, its own range and ticks, its zero
  on the left axis's zero line), the section lift coefficient cl, the
  same load on the chord the planform defines at that station
  (`Planform::ChordAt(eta)`, queried when the case is built -- not the
  twisted section's projection, which read 0.3% short at 4° of twist):
  how hard each section works, read against the airfoil's stall margin.
  With the symmetry plane on the image half is cut as well; without it a
  mirrored entity's reflected copy is already there. `StripLoads` stays
  in the core for the tests (its strips sum to the network's force
  exactly, which the cuts are checked against) and the document
  commandlet; the dialog no longer uses it. *All bodies* is l / q of the whole configuration along
  the lateral axis y: every non-wake network's panels -- wings,
  fuselages, nacelles, the images and reflected copies -- summed into
  uniform bins (twice the spanwise count, 16 to 400) by `BinnedLoads`,
  each panel's force spread over the bins its corners' extent overlaps in
  proportion to the overlap (so a fuselage's ring of twelve gives a
  smooth, conservative distribution), resolved along the lift, with its
  own elliptic curve. All plotted by `Widgets/LinePlot`.
* *Forces and Moments* -- six small charts of the wind-axis coefficients
  against the swept angle: C_L, C_D (induced), C_Y; C_l about the stream,
  C_m about the side axis, C_n about the lift axis, about the origin on
  the reference length. With more than one body each chart carries a
  dashed curve per body (its own networks' force and moment, a mirrored
  entity's reflected copy included; the image side is already in the
  network sums) and the total in white. With a single solution a table
  replaces the charts -- a column per body and the total -- with the lift
  from the wake doublet strength (the exact circulation).

`core/tests/panel_loads_check.cpp` checks the strips against the network
force, their order, the chords, the plane cuts (at a strip's centre within
1.3% of the strip's lift per span; integrated over 400 cuts within 2% of
the network's lift; a cut on a station taken once; the cut's perimeter
the section's; none beyond the tip), the binned loads (on the strip edges
they are the strips; the image half mirrors the own half; a strip cut in
two bins gives each half the same share), the elliptic integral and the
colour maps; `panel_p38_check.cpp` and `panel_p38_config_check.cpp` solve
the P-38 of `Projects/p38.yaml` (see "Known limits" for what they guard);
the document commandlet repeats the strip and elliptic checks on the
scene's placed wing. Nothing in the repository can exercise the widgets
(see "Known limits").

## Potential flow on Kratos (2026-10-15)

**Analysis -> Potential Flow (Kratos)** computes the potential flow
around selected geometry and writes the grid and the solution to a VTK
file the user names. The dialog opens with the same question the Boolean
Operation dialog asks -- *which geometry?* -- and the same two-list picker
answers it (every entity, the mirrored half of each mirrored one, every
boolean result); then the free stream, the size of the air around it, and
the solution file.

The route, and where each piece lives:

* `Dialogs/PotentialFlowDialog.cpp` -- the dialog. The chosen operands are
  triangulated with `GetPlacedMeshes` at the surface deflection, which is
  the application's own loft triangulation placed as it sits in the scene.
* `Entity/AerotelicPotentialFlow.cpp` -- the bridge: the bodies as `.tri`
  files, the settings as `job.json`, `tools/potential_flow.py` run in the
  WingDesigner micromamba environment, statistics back as `result.json`.
  The same pattern as the rib mesher (`AerotelicRibGmsh`; the
  tetrahedral mesher, when this was written) and, until 2026-11-08, the
  body structure's MeshLib backend; the process plumbing those three
  shared by copy is now one function, `AerotelicMeshLib::RunJob`, which
  is all `AerotelicMeshLib.h` holds today.
* `tools/potential_flow.py` -- the fluid domain, the solve and the output.

**The fluid domain is the other mesh.** The structural route puts shells
on a body's surface and rib discs inside it; this one meshes the air, so
the domain is a far-field box with the bodies as holes -- the one place
the application still builds tetrahedra, and a CFD mesh, nothing to do
with the structure. The box is triangulated at the far-field element
size and each body goes in as a gmsh DISCRETE surface, through the
`.tri` reader and discrete-surface helpers of `tools/gmsh_common.py`
(shared with `rib_gmsh.py` and `structural_fea.py`; they were
`tet_gmsh.py`'s until 2026-11-08), so gmsh adds interior nodes only: **the wetted
boundary of the fluid mesh is the body's own triangulation**, which is both
what makes the pressures land on the surface the designer sees and the
reason the bodies must not intersect each other (fuse them with a boolean
first). The element size is a distance field off the bodies -- their own
mean triangle edge at the wall, held for twice that, then growing by 0.3
per unit distance to the far-field size. Without the field gmsh grows
straight off the wetted surface and the suction peaks come out smeared.

**The solve** is Kratos's `CompressiblePotentialFlowApplication` over
tetrahedra: one velocity potential per node, grad(phi) = u, written as a
`.mdpa` with generic `Element3D4N` / `SurfaceCondition3D3N` that the solver
replaces with its own. Only the far field gets a process
(`apply_far_field_and_wake_process`, which fixes the potential on the
inflow part of the box from the sign of u_inf . n and applies the flux on
the rest); **the body needs no condition at all**, because d(phi)/dn = 0 is
this formulation's natural boundary condition and that is exactly the
non-penetration wall. `ComputeNodalValueProcess` projects the elemental
velocity and pressure coefficient onto the nodes.

**The output** is one legacy VTK unstructured grid in the same dialect as
the structural analysis writes: the tetrahedra of the air, with
`VELOCITY_POTENTIAL`, `VELOCITY` and `PRESSURE_COEFFICIENT` at every node,
and the bodies' own surface triangles alongside them with cell data
`region` = 1 against the fluid's 0 -- threshold on that in ParaView and the
pressure distribution on the aircraft is on its own, from the same file.

**No wake, so no lift.** This is the non-lifting potential flow: with no
wake surface the circulation around a lifting section is zero. The velocity
field, the suction peaks and the pressure distribution are real; the lift
is zero by construction, and so, by d'Alembert's paradox, is the net force
-- which is why the force the dialog reports is a measure of the
discretisation (as a fraction of the dynamic pressure on the frontal area),
not a load. Kratos does have a 3-D wake operation
(`Define3DWakeOperation`), which needs the trailing edge identified and a
wake sheet shed from it; that is the next step, not this prototype.

**What it was marked against.** `tools/potential_flow_check.py` runs the
real job on a sphere, where the exact solution is known: Cp = 1 -
(9/4) sin^2(theta), from +1 at the stagnation points to -1.25 round the
equator. It checks the wetted boundary encloses the body's own volume, the
tetrahedra fill the box minus the bodies, the far field is undisturbed, the
net force vanishes, and every wetted node against the formula:

      triangles   h        Cp rms error   net force / q*area
        1280      0.150      0.179            3.5e-03
        5120      0.075      0.106            7.0e-04
       20480      0.038      0.065            6.6e-04

The error is fore-aft symmetric and is a uniform 3-4% deficit in the
surface speed, worst where the speed is highest. That is the first-order
bias of the postprocessing, not of the solve: the velocity is constant over
a linear tetrahedron, and a wall node gets the average over the tetrahedra
around it, all of which lie inside the fluid where the flow has already
slowed towards the free stream. It falls off with the element size at the
wall, so **the resolution of the body's own triangulation is what sets the
accuracy of the pressures on it**.

Install, once, into the WingDesigner environment (all at one version):

    pip install KratosMultiphysics KratosFluidDynamicsApplication \
                KratosCompressiblePotentialFlowApplication

## DUST, vortex particle method (2026-10-16)

**Analysis -> DUST (Vortex Particle Method)** writes a complete
[DUST](https://public.gitlab.polimi.it/DAER/dust) case for the chosen
bodies. DUST is Politecnico di Milano's mid-fidelity unsteady solver: the
bodies carry surface panels and the vorticity they shed is carried away
as free vortex PARTICLES, so the wake rolls up and blows back over the
geometry. That is the thing the Kratos potential flow above cannot do --
it has no wake, so no circulation and no lift. This one lifts.

**The dialog writes input files; it does not run DUST.** DUST is a
separate Fortran program with its own HDF5/CGNS/LAPACK chain, all of
which must be built by the same Fortran compiler; keeping it at arm's
length means a case can be written here and run anywhere, and that the
feature works on a machine where DUST is not installed. The dialog's
**Help tab** gives the three commands (`dust_pre`, `dust`, `dust_post`),
explains what each does, and covers how to read the result; the case
folder gets a `README.txt` repeating it.

* `Dialogs/DustDialog.cpp` -- five tabs over the same two-list body
  picker the boolean, export and potential-flow dialogs use: **Geometry**
  (picker, surface deflection, panel count, panel-system size, overlap
  check, trailing-edge detection), **Flow** (free stream, time), **Wake**
  (particles, models, octree), **Output** (case folder, basename) and
  **Help**. Everything derived -- time step, run length, particle core
  radius, particle box, octree -- is shown live as it is edited, because
  the blank-means-automatic fields would otherwise be invisible.
* `core/include/aerotelic/dust_case.hpp`, `src/dust_case.cpp` -- the
  writer. `DeriveDustCase` works everything out and writes nothing (what
  the dialog calls on every change); `WriteDustCase` derives then writes.

**The meshes go out in DUST's "basic" format**: two ASCII files per
component, `<prefix>rr.dat` with a node per line and `<prefix>ee.dat`
with an element per line. DUST's element array is four wide and reads a
zero fourth index as a triangle (`mod_build_geo.f90`: `if (ee(4,i) .eq.
0) nsides1 = 3`), which is exactly what the loft triangulation produces.
The alternative, CGNS, would mean linking the CGNS library into this
application for no gain.

**Element type is DUST's `p`, surface panels** -- and that is not a
choice. Vortex lattice (`v`) and lifting line (`l`) need a structured
chordwise/spanwise mesh, which DUST only builds from its own
`parametric` and `pointwise` inputs, never from a triangle mesh.

**Bodies must not lie on top of one another.** Two surfaces in the same
place give the panel method two unknowns with identical influence: the
matrix is singular and the run diverges on step 1, not gradually. The
everyday way to cause it here is to pick a wing AND its mirror -- each is
a closed body and their root caps are the same wall seen from both sides.
This was found the hard way, by running the generated case: loads reached
1e35 at the first step, while the same two wings pulled half a unit apart
converged normally. The writer therefore hashes panel centroids and
reports bodies found overlapping, and the Geometry tab shows the warning
in amber naming both bodies; the fix is to fuse them with a boolean
operation and select the result. The check only finds surfaces lying ON
each other, not one body poking through another.

**The comparison happens where DUST mounts each body**, blade by blade.
A body's mesh is stored in its own reference frame's coordinates, so a
rotor's blade is hub-local while the airframe is already in world
coordinates, and the two have to be placed before they can be compared at
all. Leaving that out is how a four-propeller aircraft got told that 912
panels of Propeller 8 and Propeller 9 lay on top of each other: the props
are the same model, so their hub-local blades are the same mesh, and the
hubs they actually hang from are 4 m apart. It also hid the real thing --
a blade sitting inside a wing was never compared against it. The same
placement sizes the particle box, which otherwise enclosed an aircraft
with all its propellers piled up at the origin. Blades are compared at
their mounted azimuth only: a blade that sweeps *through* a body further
round is a collision over the run, not a singular matrix on step 1, and
this warning is about step 1.

**Cost is quadratic.** DUST's panel method builds a dense influence
matrix, so the Geometry tab shows its size in MB next to the panel count:
a few thousand panels is comfortable, tens of thousands will not run.
This is why surface deflection matters more here than in an export.

**Derived settings.** The reference length is the model's extent *along
the flow* (a wing's chord, a fuselage's length); the time step is the
time to cross it divided by steps-per-length, and the run lasts that many
crossings. The particle core radius defaults to the mean panel size, the
particle box to the model's bounds plus margin and a long downstream tail,
and the octree is sized to cover that box. Every one of those can be
overridden; zero means "work it out".

**Spinning propellers** (2026-10-17). A propeller given a non-zero speed
on the dialog's **Rotors** tab is written as DUST's own rotor: the frame
is `multiple = T` with `mult_type = rotor`, and DUST is handed **one
blade**, which it replicates `n_blades` times about the axis and spins at
`rot_rate`. That is the form DUST's post-processing is built around.
Everything else still rides reference frame 0, and a propeller left at
zero is written exactly as before -- bodies where they sit.

**This changes nothing in the geometry engine.** Rotational speed is an
analysis setting and is deliberately *not* a property of `PropellerSpec`,
so it is not saved with the project and is forgotten when the dialog
closes. The writer only *reads* three things that were already public:
`GetModel()->BuildMeshes()` for hub-local geometry (`GetPlacedMeshes()`
is that plus the placement), `PropellerModel::BuildMeshes`'s existing
`Blade 1` at azimuth 0, and `GetEntityTransform()`.

Details that matter:

* The blade's mesh goes out in **hub-local** coordinates and the frame
  carries the placement. Writing a placed mesh onto a placed frame would
  apply the placement twice.
* The **spinner stays on frame 0**, placed. It is a body of revolution
  about the same axis, so spinning it would change nothing, and this
  avoids a second multiple frame. `dust_pre` duly reports 0 trailing-edge
  elements for it and 18 for the blade.
* **`rot_rate` goes out negative for a positive RPM** (2026-10-17). A
  positive speed on the Rotors tab must drive the blade *leading edge*
  first, and the blade's own geometry decides which way that is:
  `PropellerModel::BuildMeshes` turns the blade -90 degrees about y,
  carrying the wing's chord (leading edge at low x, since +x is aft) onto
  local **+z**. So the leading edge sits at low z and the blade has to
  travel towards -z -- while a blade at radius r along +y under rate w
  about +x has `v = (w,0,0) x (0,r,0) = (0,0,+w*r)`, which points at -z
  only for `w < 0`. Written positive, as it was at first, every propeller
  in the case turned trailing-edge first. Measured on the Zipline P2
  blade to be sure: the chord-perpendicular thickness 15% in from each
  end is 3:1 in favour of the low-z end at every span station, with a
  textbook twist of 60.9 degrees at the root falling to 13.9 at the tip.
  The sign lives in `DustDialog.cpp` and nowhere else -- the writer takes
  it as given, `dust_case.cpp` only ever reads `fabs(rot_rate)` for the
  time step, and no geometry depends on it -- so the whole convention is
  one negation at the one place that knows the propeller's handedness.
  That reasoning is for a **positive-spin** propeller. A negative-spin
  one (`PropellerSpec::spin_direction`) is that blade reflected in `z`,
  so its leading edge sits at high `z` instead of low and it has to
  travel towards `+z`: the sign flips back. The two flips are
  independent and compose -- a mirrored negative-spin propeller gets
  both and ends up back where it started, which is right, since each
  reversal reverses the hand.
* A **mirrored** propeller turns the opposite way and its placement
  contains a reflection, which cannot be a DUST orientation matrix
  (determinant -1). Writing M for the reflection, `M*R = (M*R*M)*M`: the
  proper rotation `M*R*M` becomes the frame and the leftover `M` is
  folded into the blade's own coordinates, with `rot_rate` negated. That
  negation is right for two reasons that agree: a reflection turns a
  rotation around, `M*R_x(t)*M = R_x(-t)`, and the reflected blade spans
  -y with its edges still where they were in z, so `v = (0,0,-w*r)` and
  the sign it needs flips back.
* `orientation` is read by DUST with a Fortran `reshape(...,(/3,3/))`,
  which fills **column major**, so the nine numbers are the frame's x, y
  and z axes -- the transpose of the row-major spelling.
* `n_dofs` is **required and has no default**; zero is right for a rigid
  propeller. Omitting it lets `dust_pre` pass and makes the *solver*
  refuse the case, because the preprocessor never reads `References.in`.
* The derived time step now resolves the **rotation** as well as the
  flow: `DustCaseOptions::degrees_per_step` (5 by default) caps how far
  the fastest blade sweeps per step, and the step is the smaller of that
  and the flow-crossing one. On the P-38 propeller at 2400 rev/min that
  took the step from 15.6 deg/step to 5, i.e. 72 steps a revolution.
  Zero disables the rule. The panel count also counts a blade once per
  blade, since DUST replicates it -- otherwise the dense-system estimate
  would be out by a factor of `n_blades`.

`References.in` still carries a commented-out template for a hand-written
moving frame, for motion the dialog does not offer.

`core/tests/dust_case_check.cpp` (97 checks) covers the file set, the
`ee`/`rr` formats down to the zero fourth index and 1-based node indices,
the derived-versus-written agreement, the octree covering the particle
box, the overlap detection, and the refusals. **It was also validated
against the real solver**: the generated case runs through the DUST built
at `C:\Users\Matt\dust-install` -- `dust_pre` detects the trailing edge
(30 elements on a default wing), `dust` solves 321 steps in 61 s, and
`dust_post` produces the VTK series and load tables.

## The Transform dialog (2026-10-22)

**Translation and Rotation have a tab each**, and the Rotation tab leads
with an **Approach** choice -- *Axis and angle* or *Euler angles* --
which switches the fields below it. The approach is not dialog state: it
is `Placement::euler.used`, the entity's own record of how its rotation
was authored, so re-opening the dialog (or the project) comes back to
the approach it was left on. Switching never moves the entity; each
approach opens on the rotation already in effect, expressed its way.

### What was wrong with it

The dialog kept no state of its own: every edit read the placement back
out of the actor's `FQuat`, changed one field and wrote the quaternion
again. A quaternion holds a rotation, not the parameters that describe
one, so two things the dialog had to keep did not survive the trip.
(The dialog has since gained state of a different kind: a draft
placement, held until Apply -- see *Apply in the design panels and the
Transform dialog* below.)

* **An axis set while the angle was zero was lost.** `ApplyPlacement`
  stored `FQuat::Identity` for a zero angle, and reading that back gave
  UE's fallback axis from `FQuat::GetRotationAxis()` -- so every axis
  field repainted as X = 1 whatever had been typed, and setting the axis
  before winding the angle on was impossible.
* **A typed axis was rescaled.** The round trip normalised it, so typing
  `1` into Z beside an existing X axis read back as 0.707, and the axis
  rows were clamped to -1..1, which blocked honest directions like
  `(0, 2, 1)` as well.

### Storing the placement instead of re-reading it

`AAerotelicEntityActor` now **stores** `aerotelic::Placement` and derives
the actor transform from it (`ApplyPlacement`), never the other way
about; `GetPlacement()` returns what was authored and
`GetEntityTransform()` -- what the feature tree places geometry with --
reads the same stored values. Normalising was never needed anyway: both
`gp_Dir` in `core` and `FQuat` here normalise for themselves. A
zero-length axis is now treated as no rotation in `Place()` and in
`GetEntityTransform()` rather than reaching `gp_Dir`, which throws on a
null direction. The axis rows have the same wide bounds as every other
field, since an axis is a direction and not a unit vector.

### The Euler approach

A rotation can instead be authored as three turns in sequence: **Order**
picks which axes and in what order, **Axes** whether each turn is about
the axes the earlier turns already moved (*Moving (intrinsic)*, the
aircraft convention) or about the fixed world axes. `EulerMatrix`
composes the three
single-axis matrices -- intrinsic post-multiplies, extrinsic
pre-multiplies, which is why intrinsic Z,Y,X is extrinsic X,Y,Z
reversed -- and `AxisAngleFromMatrix` reduces the product to the one
axis and angle it amounts to. **Nothing downstream knows Euler angles
exist**: `rotation_axis`/`rotation_deg` remain the whole rotation for
the feature tree, the mesh paths and every older reader of a project
file.

What is stored beside them is only authoring provenance:
`Placement::euler` (`used`, `intrinsic`, `order`, `angles_deg`),
serialised as an optional `rotation_euler` block under an entity's
`placement`. It has to be stored because a matrix does not decompose
into a triple uniquely -- and not at all at gimbal lock -- so the three
numbers could not be recovered to re-open the dialog on. `used` is the
Approach choice: with it set, `SyncPlacementRotation` derives the axis
and angle from the triple on every edit and on load (so a hand-edited
`rotation_deg` beside an Euler block cannot disagree with the angles),
and the "Composes to" read-out shows what they amount to. Under the
axis/angle approach the Euler rows would show `EulerAnglesFromMatrix` of
the current rotation -- one valid decomposition, not three zeros beside
a live rotation -- which is what switching approach adopts. Changing
Order or Axes keeps the three numbers and turns the entity differently:
the user is declaring which convention they are in.

`feature_tree_check` covers the composition (orthonormal, det +1, equal
to the three matrices multiplied by hand, the intrinsic/extrinsic
equivalence), known values (a 90-degree roll from Z,Y,X `(0, 0, 90)`;
fixed-axis Z90 then X90 as 120 degrees about `(1, -1, 1)/sqrt(3)`), the
axis+angle round trip at 0.5, 45, 90, 179.5 and 180 degrees, and
decompose/recompose over all six orders in both frames including two
gimbal-locked sets. `project_yaml_check` covers the `rotation_euler`
round trip and that an axis survives storage un-normalised at zero
angle; the document commandlet covers the same three promises through
`ApplyPlacement`/`GetPlacement` on a real entity.

## Apply in the design panels and the Transform dialog (2026-10-22)

Until now every design panel wrote straight into the entity's live model
and called `RefreshProxy()` on each edit -- per keystroke on the wing
tabs, per commit on the others -- and the Transform dialog applied each
field as it was committed. Every edit therefore rebuilt the entity, then
every boolean over it; and the lattice over those was deliberately
excluded from that chain, because a generation that takes seconds cannot
run on every keystroke. The visible symptom was the one that prompted
the change: edit the fuselage's ρ-upper curve, and the boolean over it
follows but the lattice filling that boolean does not.

The paradigm is now **edit a draft, Apply on a button**, in both places.

* **Design panels edit a draft.** `MakeEntityDesignContent` clones the
  entity's model (`UAerotelicDocumentSubsystem::CloneEntityModel`: the
  spec round-trip copy/paste and save/load take, so the copy is
  independent and the panel's curve editors can hold pointers into it)
  into a `FAerotelicDesignDraft` owned by the panel, and every
  `Build*DesignContent` is handed that draft and an `OnChanged` that
  only marks it dirty. Nothing in the scene changes as fields are typed
  into or poles dragged. The panel ends in a footer -- a status line,
  **Revert** and **Apply**, both enabled only while the draft is dirty.
  Apply (`ApplyEntityModel`) builds a fresh model from the draft's spec
  and swaps it in with `AAerotelicEntityActor::ReplaceModel`, which
  keeps each body's display state by role and, if the draft refuses to
  build (a wing with a negative chord), puts the old model back and
  reports why; the draft stays as typed so it can be fixed. Revert drops
  the draft and rebuilds the panel on a fresh clone. A Rename in the
  scene tree after the clone is kept: Apply writes the entity's current
  name into the spec. Switching selection keeps each panel's draft (the
  panels were already kept per entity); deleting the entity drops it.
* **The Transform dialog edits a draft placement** the same way: its
  fields read and write a `Placement` owned by the dialog's widgets (it
  is modeless, so the state lives in the widget tree), kept in step with
  `SyncPlacementRotation` so the Euler rows, the axis fields and the
  "Composes to" read-out agree as they did when every edit was applied;
  Apply calls `ApplyPlacement`, Revert re-reads the entity's placement.
* **What follows an Apply is one queue, drained one item per frame.**
  Boolean results used to subscribe to `OnGeometryChanged` themselves and
  rebuild synchronously inside the broadcast -- so a chain of booleans
  rebuilt the second one twice (once from the first's nested broadcast,
  once from the entity's outer one), and the whole chain ran inside the
  Apply click with no repaint. Lattices were excluded from that chain
  altogether. Now the document subsystem alone subscribes: it queues
  every boolean result and generated body structure (the lattices, when
  this was written) that `DependsOn` the changed entity
  (directly, or through a boolean target), walking `Entities` so the
  queue is in build order, and a core ticker rebuilds one item per tick.
  A boolean goes as soon as its turn comes; a structure generation
  (seconds of work)
  waits until 0.3 s have passed since the last queueing, so one Apply
  that changes an entity and then the booleans over it regenerates the
  structure over those once, and an Editor gizmo drag does not regenerate
  it every frame. `ApplyEntityModel` queues explicitly as well, since a
  commandlet's `RefreshProxy` builds and broadcasts nothing;
  `FlushPendingRebuilds` drains at once, which is what the test uses.
  Generation failures are reported in one message box and the layers left
  as they were.
* **The status bar's rebuild bar** (`SAerotelicMainShell::BuildStatusBar`)
  is a thin progress bar at the left of the bottom border, directly under
  the scene tree (the status text sits to its right), full while nothing
  is waiting. An Apply counts the entity itself as one parameterisation
  rebuilt and each queued boolean or structure generation as one more, so the bar
  drops to `done / total` and fills back as the queue drains, one step
  per frame. It is a count, not a timing: every parameterisation is one
  step whatever it costs, nothing is measured, and the only cost is one
  float read per paint (`GetRebuildProgress`).

The document commandlet covers the draft's independence before and after
Apply, that Apply gives the entity a fresh model (neither the old one nor
the draft), the wing's parameters surviving the spec round trip, that a
boolean result has no draft, the boolean and the generated structure over
the sphere (the lattice, when this was written)
being queued (the bar at one third) and rebuilt by the flush (the bar
full), and the structure regenerated over the sphere as changed.

## Mass properties (2026-10-22)

Two pieces, joined by one setting per parameterisation.

**Scene tree > Material Properties** (`MainShell/AerotelicMaterialDialog`;
it was "Mass Properties" for a few hours on 2026-10-22) on any row,
derived ones included, sets what that entity is made of --
`Placement`-style, stored on the actor (`AAerotelicEntityActor::MassSpec`)
and written with it as `mass: {asserted: <total>}` or
`mass: {material: {name, density, youngs_modulus, poisson_ratio,
allowable_stress}, skin_thickness: <t>}` on the entity or boolean (and,
until 2026-11-08, a lattice; absent means unset, so older files read unchanged; the interim
`mass: {density: <rho>}` reads as a custom material with only a density;
`skin_thickness` is written only when positive). *An asserted mass*
states the total; when weighed it is spread uniformly over the surface
as a thin shell, with the centre and inertia integrated over that
surface. *A material* weighs the entity as a uniform solid, its density
times the volume its closed surface encloses, and is what Analysis >
Structural analyses with; see *Materials and the structural analysis*
below. *A material with a skin thickness* (2026-10-31; the dialog's
"Skin thickness, 0 = solid", in the project's length unit as typed, never
converted) makes every solid body the parameterisation produces a skin of
the material that thick: its material volume is the surface area times
the thickness, its mass the density times that, spread over the surface
as a thin shell (`aerotelic::SkinMassProperties`) -- open surfaces
included, so a zero-thickness skin no longer has to be asserted. The
report's Note column says "skin t thick" on such a row. The same figure
is what the structural analysis gives its skin shells (since 2026-11-08
the skin thickness is also a per-body attribute -- see *Per-body
structure*; until that evening it was also what the structure's
generation hollowed the entity by).

**Generated structures weigh as shells (2026-11-08, late).** A body whose
structure has been generated weighs by its layers, one report line each,
and every layer is a THIN SHELL of its own material and thickness --
density x area x thickness, `SkinMassProperties` -- the Skin on the
body's own placed surface (the Skin layer has no mesh; the body's surface
is the skin), each Ribs layer on its placed discs. The Note says "skin t
thick, <material>" or "rib shells t thick, <material>", and "generated
before the body's last change" when the layers are stale. Until that
evening the layers were closed solids (the hollowed skin, the rib slabs)
weighed by enclosed volume; the shell rule is the one the structural
analysis uses, so the mass report and the solver agree by construction.
The overlap of rib and skin along the rims and the skin's area taken on
the outer surface are counted twice and over respectively, by a few
percent at most -- accepted, see *Structure as shells: no offset
surfaces*. An asserted mass still stands in for all of it.

**Inheritance, and no defaults (2026-10-31, Matt's three rules).** A
parameterisation's Material Properties are the one source of what it is
made of. A DERIVED entity -- a boolean result (and, until 2026-11-08, a
lattice) -- carries no
`mass:` of its own (an older file's is ignored on load, none is written)
and has no Material Properties menu: it inherits, a boolean result from
its first operand (a lattice from its first filled body), recursively
(`AAerotelicEntityActor::GetEffectiveMassSpec`, `GetMassSpecSource`,
`IsDerived`). Every analysis reads the effective spec: the mass report
("inherited from X" in the Note), the structure's generation (each
body's skin at its own thickness; until 2026-11-08 late, each body
hollowed by it) and the structural analysis. There is no global
default anywhere (the lattice dialog of the time had no thickness field:
the list under the parameters showed each chosen body's figure and named
one that lacked it, which the generation refused by name), and the
document commandlet checks the sphere's structure is generated from its
own 0.05 skin (the Skin layer's volume its area times 0.05) and
that a wing without a material is refused.
The dialog shows the entity's enclosed volume, surface area, whether the
surface is closed, the material volume (enclosed, or area times
thickness) and what the current setting would weigh, so the zero a
zero-thickness skin encloses is on screen before anyone wonders why.
Since 2026-11-07 the dialog only PICKS a material (one list: not set, an
asserted mass, every material of the library) and takes the thickness or
the mass; the figures themselves are defined under Model > Material
Properties -- see *The Model menu and the material library* below.

**Analysis > Mass Properties** (`Dialogs/MassPropertiesDialog`) picks
bodies with the shared two-list picker, meshes each at the chosen
tolerance (the STL tolerance, 2e-3 by default) and reports a table: one
row per component and, under a rule, the assembly; the columns the mass,
the three components of the centre of mass, the three moments Ixx, Iyy,
Izz and the three products Ixy, Ixz, Iyz -- the six independent entries
of the tensor about that component's own centre (the assembly's about
the assembly's centre, each body's tensor carried there by the
parallel-axis theorem and summed). An entity with no setting keeps its
row with dashes and the reason rather than being counted as zero; a body
weighed by density whose surface is open is kept but flagged in its Note
column, since its enclosed volume is not well defined. A multi-body
entity (a propeller) shares an asserted mass between its bodies by area;
a mirrored half is a second part and carries the whole mass again. The
report is text: shown in the dialog, copied, or saved beside the project.

The integrals are `core/include/aerotelic/mass_properties.hpp`, OCCT-free
so a boolean result -- which has no B-rep solid -- weighs the
same way a wing does, and a generated layer (an open shell) by the same
code's area rule: every triangle and the origin make a tetrahedron
whose signed volume, first moment and second-moment matrix add up over a
closed surface to the body's (the divergence theorem one simplex at a
time; the one simplex formula serves the shell with area/12 in place of
volume/20), and the tensor is `tr(S) 1 - S` moved to the centre by the
parallel-axis theorem run backwards. `PrincipalInertia` is a Jacobi
eigen-solve. `mass_properties_check` (73 checks) holds the box solid and
the box shell to their closed forms to 1e-9 -- the shell against six
plates carried in by the parallel-axis theorem, a different derivation --
a single triangle to `m(a^2+b^2+c^2)/36`, the sphere and cylinder to
their analytic values within the tessellation's own error, translation
and rotation invariance (principal moments unchanged, principal axes the
turned body axes), and two unit cubes combined against the 2x1x1 box they
make. The document commandlet weighs real entities: a sphere by density
against the model's exact volume, an asserted fuselage and its mirror,
an unset wing left out and named, and the settings through save/reopen.

## Units (2026-10-22)

Every number typed into a parameterisation is unit-agnostic to the
geometry: a wing with a span of 12 is the same shape in feet or metres.
What the units decide is what those numbers MEAN when they leave the
geometry -- for a mass report, a STEP file, or an analysis code that
computes forces. **Model > Units** sets them per project, length and
mass chosen separately (metres / millimetres / feet; kilograms /
pounds-mass), time always seconds; the status bar shows the three.

Two rules, both deliberate. **Nothing is converted when the setting
changes**: every typed length, mass and density keeps its value and is
read in the new unit from then on (the menu says so, out loud, when there
is anything in the scene to be re-read). And **everything handed to an
analysis code is converted to SI first**, from the two base units --
density as mass/length^3, speed as length/s, pressure and viscosity as
mass/length, inertia as mass length^2 (`core/include/aerotelic/units.hpp`,
with the exact definitions 1 ft = 0.3048 m, 1 lbm = 0.45359237 kg). No
compound unit is ever chosen on its own, so the system cannot be
inconsistent, and because the codes get kilograms no slug ever appears:
the user types lbm, and lbm/ft^3 straight from a materials table.

Where the setting reaches:

* **The project file**: `units: {length: m, mass: kg, time: s}`, first in
  the file and always written, the defaults included, so a reader sees
  what the numbers mean. Absent on load means metres and kilograms --
  what every project before the key existed was taken to be. A length
  or mass unit the application does not offer, or a time unit other than
  seconds, is refused rather than misread.
* **Mass Properties**: the per-entity dialog labels its mass and density
  fields and its volume/area read-outs; the report's header names the
  system and its second header row gives each column's unit. The
  integrals themselves are done in project units and stay there.
* **STEP export**: the file now DECLARES the project's length unit
  (`write.step.unit` = M, MM or FT, passed to `aerotelic_step_writer` as
  `--unit`) and the geometry is written as it is, so CAD opens it at the
  right size. Before this nothing set it and every export left declared
  in OCCT's default, millimetres, whatever the project was drawn in.
  STL and VTK are unitless by format and stay in model units.
* **Potential flow and DUST**: the dialogs' fields are in project units
  with the unit on each; at Solve / Write the bodies are scaled to
  metres and the stream (speed, density, speed of sound, pressure,
  viscosity) converted, so the solver and the case are SI. DUST's
  derived step and run length are a length over a speed and come out
  the same either way, so what the Time tab shows still holds. The
  dialogs' SI defaults (1.225 kg/m^3, 340 m/s, ...) are converted the
  other way on opening, which is what keeps 1.225 from sitting in a
  lbm/ft^3 field and reaching the solver as 19.6 kg/m^3. Results come
  back in SI; lengths, speeds and areas are shown in project units
  again, forces stay in newtons and say so -- a force in lbm ft/s^2 is a
  poundal, which nobody wants.

`units_check` covers the factors, every compound factor against the base
ones, the sea-level air figures each system will show, the name
spellings both ways and the refusals (inches, slugs, minutes);
`project_yaml_check` the round trip, the key coming first, the absent
key, and the refusals on load; the document commandlet sets feet and
pounds, saves, reopens, and reads the file's `units:` line.

## Scale: a replica of the full-scale aircraft (2026-11-03)

The parameterisations are always the **full-scale aircraft**: a wing's
span is the real wing's, a fuselage's length the real fuselage's, and
that is what every panel edits and every project file saves. **Model >
Scale** sets the *model scale* -- the size the vehicle is BUILT at as a
fraction of the size it is drawn at -- so a 1:4 replica is the same
project with the scale at 0.25, and nothing in any parameterisation
changes. A radio per common model scale (1:1, 1:2 ... 1:72) and
*Custom...* for any other number (a double-size model is 2). The status
bar's units readout gains ", 1:4 scale" while the vehicle is not full
size.

How it is done: the document holds one number
(`UAerotelicDocumentSubsystem::GetModelScale` / `SetModelScale`) and an
entity actor's transform is its FULL-SCALE placement under that scale --
the translation multiplied by it and a uniform scale of it
(`AAerotelicEntityActor::ApplyPlacement`). The backend's placement
transform carries it the same way (`feature_tree.hpp EntityTransform::
scale`, `Place(..., scale)`: the solid scaled about the origin, then
rotated, then translated by the scaled translation). So everything that
reads placed geometry is the replica's without knowing it: the viewport,
the mirrors, `GetPlacedMeshes` / `GetPlacedBounds` / `GetPlacedShape`,
the boolean results and the body structures generated over them, STL / STEP / 3D PDF
export, mass properties, the FEA, the potential flow, the panel method
(its default reference area and length are scaled with the wing) and the
DUST case (a spinning propeller's hub-local blade and radius are scaled).
Setting the scale re-places every entity, which rebuilds the booleans and
regenerates the body structures as a Transform dialog's Apply does, and reframes
the view. The Transform dialog and the design panels keep showing the
full-scale numbers, which is what they are.

**Not scaled, on purpose**: the materials -- a skin thickness and a
density are what the replica is to be built FROM, so a 1:4 model with a
2 mm skin weighs its area at 2 mm, and the stresses and strains the FEA
reports are the replica's under its own loads; the structure's build
figures -- skin thickness, rib thickness and height -- stay as set and
the structure regenerates over the smaller bodies (rib start and spacing
are geometry and scale with the body; while there were lattices their
cell size stayed as set too, so a 1:10 replica at the same cell had a
thousandth of the cells); and the units -- a 1:4 replica of a vehicle drawn in feet is in
feet. Every flow quantity with a length scale in it (Reynolds number, the
forces at a given speed) is the replica's.

The project file: `scale: 0.25` between `units:` and `entities:`,
written only when the vehicle is not full size so a 1:1 file reads as it
always did; absent on load means 1; anything but a positive number is
refused on load and on save. The Python app does not know the key and
ignores it (it reads `entities`, `booleans` and `lattices` by name).

`feature_tree_check`: `Place` at 1:4 scales a box's volume by 1/64 and
its centre by 1/4, 1:1 is the identity, zero and negative are refused, an
`EntityTransform` built from the seven placement numbers is full scale.
`project_yaml_check`: the key absent at 1:1, written and read back at
0.25 with the full-scale parameterisation and placement beneath it, the
refusals. The document commandlet sets 1:4 on the scene: the wing's
parameterisation and authored placement unchanged, the actor at a quarter
scale a quarter as far out, the feature-tree transform carrying the scale
with the full-scale translation, the placed bounds, meshes and OCCT solid
a quarter the size, the sphere's skin mass 1/16 (area) with its thickness
as typed, the boolean and the structure over the sphere (the lattice,
then) queued and rebuilt; saves, finds
`scale: 0.25` in the file, reopens, and finds the replica as it was.

### What the model scale broke, and the fixes (2026-11-03, evening)

A replica a fraction of a unit across found every depth-related constant
in the renderer and every coordinate in the FEA dialog assuming the
vehicle was its full 6 to 16 units. Fixed:

* **Clipping.** The near clip plane was the engine's one global value
  (`NearClipPlane=0.5` in DefaultEngine.ini, still there as the fallback)
  and the camera could come within 0.05 units of its focus, so a 1:40
  replica framed behind the near plane and vanished, and any replica
  clipped a few wheel notches in. Both are now proportional: the near
  plane sits at 0.2% of the camera's distance to its focus
  (`AAerotelicCameraPawn::CalcCamera` sets the view's own near plane;
  the dialogs' picture views set the capture's custom near plane the
  same way) and the camera may come no closer than 0.1% of the radius
  last framed (`SceneRadius`). The far plane is the engine's infinite
  reversed-Z one and needed nothing.
* **Edges floating off the surface.** M_AerotelicWire nudges its lines
  0.02 world units towards the camera -- 15% of a 1:72 replica. The
  entity actors and the result meshes now set the material instance's
  `EdgeOffset` to 0.15% of the placed diagonal, after every rebuild and
  every move or rescale (`ApplyEdgeOffset`).
* **The ground grid** had a half-unit floor; its floor is now the
  content's own size.
* **The FEA pictures framed once per dialog**; every Build and Apply
  reframes them now (the panel-method picture reframes on every solve).
* **FEA coordinates are typed FULL-SCALE**, as the parameterisations are:
  a support plane's value and tolerance and a point force's position are
  divided by the model scale on the way into the request, whose mesh is
  the replica's; the captions show full-scale figures back and say so;
  the field labels read "(full scale)" while a scale is set. Forces,
  pressures, thicknesses and the load factor are untouched by scale.
* **"At the mesh's minimum / maximum" has an "Of" choice**: the whole
  mesh, or one picked body. Before, the extreme of all chosen bodies
  decided, so a body that stuck out furthest captured a support meant for
  another and everything else was dropped as floating. The pre-processor
  resolves the plane against that body's nodes (`FSupport::Body`,
  `FRegion::Body`), falls back to the whole mesh and says so if the body
  has none, and the dialog hands the solver the resolved value (the
  script only knows the whole mesh), so the two fix the same nodes.
* **A point force that snaps further than a mesh edge is flagged**: the
  pre-processor records the snap distance (`FPointLoad::SnapDistance`,
  `SnapIsFar` against the median edge); the Loads caption prints the
  point asked for, the distance and the warning, the Apply status counts
  them, and Solve asks before running on one.
* The smoke run's default skin thickness is one percent of the full-scale
  extent, not the replica's.

The document commandlet's beam step now also checks the snap distance (a
point 3 above the top face snaps 2.5 to it and is flagged; the tip loads
are not), the per-body minimum (body 0's is the beam's root; a body with
no nodes falls back and is flagged).

## Materials and the structural analysis (2026-10-22)

**A material is a property of the stuff, not of the units.** The
presets (since 2026-11-07 the rows of `core/data/materials_presets.yaml`,
public-domain sources only -- see *The split, and presets as YAML* under
*The Model menu and the material library*; as first written,
`core/include/aerotelic/materials.hpp` carried 2024-T3, 7075-T6,
6061-T6, Ti-6Al-4V, 4130, 17-4PH, AZ31B, quasi-isotropic CFRP and GFRP,
spruce, a PMI foam; since 2026-11-06 the 3D-printing polymers too --
seven photopolymer resins of SLA / LCD printers (Formlabs Grey, Clear,
Tough 2000, Durable, High Temp, Rigid 10K; Siraya Tech Blu) at their
data sheets' post-cured figures, seventeen Anycubic LCD / DLP resins at
the conservative bounds of their data sheets' ranges (lowest modulus
and strength, highest density), and ten FDM thermoplastics (PLA, PETG,
ABS, ASA, polycarbonate, nylon, two carbon-filled nylons, ULTEM 9085,
PEEK) at their data sheets' as-printed figures along the layers, with
the caveat in `materials.cpp` that a printed part is weaker across its
layers and that the allowable is the sheet's yield or tensile strength)
and every custom material are SI -- kg/m^3, Pa --
whatever the project is drawn in; the Material Properties dialog shows
them in the project's units and converts back on edit, and the analysis
bridges hand them on as they are. Picking a preset copies its figures in
with its name; editing a figure makes the material "Custom" (by the
numbers: a figure typed back to the preset's is the preset again). The
project file writes the figures beside the name so a later change to a
preset cannot reach into saved projects. Because stress is a force per
area it is its own unit -- lbm/(ft s^2) would be a poundal per square
foot -- so **Model > Units** gained a *Stress* choice (Pa, MPa, psi,
ksi; `stress:` in the file's `units:` line) that Young's modulus, the
allowables and every reported stress use.

**Analysis > Structural (Kratos FEA)** (`Dialogs/StructuralFeaDialog`,
`Entity/AerotelicStructuralFea`, `Entity/AerotelicFeaModelBuilder`,
`tools/structural_fea.py`) is a linear static analysis of the chosen
bodies as SHELLS -- skin shells on each body's own grid, rib shells on
the discs of its plane networks -- on KratosMultiphysics'
StructuralMechanicsApplication, run as a
separate process in the WingDesigner environment like the potential flow
(`pip install KratosMultiphysics KratosStructuralMechanicsApplication
KratosLinearSolversApplication scipy`). Everything the solver sees is SI:
the mesh scaled by the length unit, the materials as they are, loads in
newtons and pascals; the dialog converts the results back. Since
2026-11-08 there is one model, shells (see *Shells only* below). As
first written, when bodies were lattice-filled, there were two models of
the same part, chosen in the dialog; both are gone and stay here as the
record of what was measured:

* **Shells over a homogenised lattice core (the default 2026-10-30 to
  2026-11-08; removed).**
  The skin / core tetrahedra below ran to 609k on p38a's airframe --
  millions of degrees of freedom at second order -- and the count was the
  SKIN: one tetrahedron thick on an offset remeshed at two skin
  thicknesses, promoted to 10 nodes so as not to lock. So the skin is now
  shells and the core a homogenised continuum. The core is tetrahedra
  bounded by the body's OWN surface at the lattice cell size
  (`FAerotelicLatticeModel::GetStructuralTetMeshes`, gmsh with no offset
  surface at all), carrying the lattice's homogenised 6 x 6 elasticity
  tensor through Kratos's `UserProvidedLinearElastic3DLaw` and its
  effective density; the skin is a shell element on every one of those
  surface triangles with the skin thickness as a property
  (`ShellThinElementCorotational3D3N`). The shells sit on the outer
  surface and the core fills the whole body, so they overlap by the
  skin's volume: the core is far softer than the skin, so the overlap's
  stiffness is nothing, and the core density is reduced by the overlap's
  share so the mass and the inertial load are the part's. First-order
  tetrahedra are fine, the bending problem being the shells'. The pressure
  still maps face for face, the shells being the wetted triangles. The
  lattice row's material is the BASE material its struts or walls are
  printed in, as Mass Properties already reads it (density times the
  lattice body's actual volume); the effective figures are computed.

  *Homogenisation* (`tools/homogenize.py`, removed 2026-11-08; after
  Andreassen & Andreasen
  2014, extended to 3-D voxels of any aspect): one period of the lattice
  field is voxelised at the voxel centres (`aerotelic::SampleUnitCell` --
  the cubic cell for TPMS and struts; for a honeycomb one period of the
  cross-section, a x 2h for the triangular lattice of a Hexagon or
  Triangle, by two voxels along the prism axis where the field does not
  vary), each solid voxel a trilinear hexahedron, the nodes numbered
  periodically so the cell has no boundary, the six unit macro strains
  applied as equivalent nodal forces, one sparse factorisation (scipy's
  SuperLU) for the six right-hand sides, three translations pinned, and
  the tensor read off as the strain energy of the corrected fields. The
  result depends on the voxel pattern and the base material alone, not on
  the cell size, so one homogenisation serves every body the lattice
  fills (and is cached under the user's temp folder by the cell's bytes
  and the material). 32 voxels across the cell by default: the gyroid
  takes ~16 s there, the octet 3 s. Checked (`tools/homogenize_check.py`)
  against what has a closed form: a solid cell returns the base tensor
  to rounding; a prismatic cell attains the Voigt bound along the prisms,
  E_z = rho* E, exactly; a square honeycomb's in-plane modulus is (t/l) E
  within 8%; a grid of bars (a/l)^2 E within 10% and cubic; a regular
  hexagonal honeycomb's in-plane modulus is Gibson & Ashby's 2.3 (t/l)^3 E
  within 8%; the octet is stretch-dominated near rho*/9 (Deshpande, Fleck
  & Ashby), the BCC bending-dominated at 5% of rho*, the gyroid network
  between (E/E_s = 0.07 at rho* = 0.30) and cubic.

  *Micro stress.* The von Mises a homogenised element reports is the
  average over a cell; the struts carry more. The same six solves give
  every voxel's stress per unit macro strain, from which come a
  conservative scalar bound (the largest ratio of peak voxel von Mises to
  the Frobenius norm of the macro stress over 2000 sampled strain
  directions, 20-60 for the lattices here, 240 for a thin-walled
  honeycomb in plane) and the influence tensors of the voxels that set
  that peak in any direction, the *critical set*, against which the
  solver evaluates each core element's OWN strain state. The core's
  margin is the base material's allowable over that estimated strut
  stress; the smeared figure is reported beside it. Homogenisation
  assumes the core is several cells thick, which a trailing edge is not:
  the error there is local.

  *Which shell element*, measured rather than assumed, and still the
  reason the thin triangle is the one in use
  (`tools/structural_fea_check.py`, a sandwich box beam L x 0.1 x 0.1
  clamped at one end under its own weight, against Timoshenko and against
  the same box as 10-node tetrahedra -- tetrahedral models, since
  removed): with a core of 1% of the skin's
  modulus the thin corotational triangle (Kirchhoff bending, ANDES
  membrane) deflects 1.002 of Timoshenko and the 10-node tetrahedra 1.06;
  with a void core, where a hollow box really is softer than beam theory,
  the tetrahedra give 1.19 and the thin shell 1.13. Kratos's THICK
  (Reissner-Mindlin DSG) triangle gave 0.77 with the core and 0.91
  without, and did not move with mesh refinement -- too stiff once a core
  shares its nodes -- so the thin one is the default and the thick one a
  `shell_element` option in the job only. The box's shell model is 5,877
  degrees of freedom against 169,959 for the tetrahedra. Also checked
  there: a shell on a unit sphere under 1 bar carries the membrane
  stress p a / 2t within 3% (median away from the support) and moves
  inward by p a^2 (1 - nu) / 2 E t within 8%; the mixed system (six
  degrees of freedom on shell nodes, three on the rest -- Kratos builds
  the system from what each element asks for) solves, the shells carry
  `VOLUME_ACCELERATION` (the reactions balance the whole weight), and
  the clamp fixes the rotations too (`ROTATION` constrained on the
  Support alongside `DISPLACEMENT`). AMGCL needs
  `use_block_matrices_if_possible: false` for that mixed system; the
  direct solver (`sparse_lu`, Eigen's SparseLU from
  KratosLinearSolversApplication) is what the default `auto` runs up to a
  million degrees of freedom (see "The solver that stopped short" below).

* **Skin and core tetrahedra (full resolution; removed 2026-11-08).** The
  mesh Lattice >
  Export Tetrahedral Mesh wrote, whose outer faces were the body's own
  surface triangles. The skin tetrahedra were made of their body's
  material, the core tetrahedra of the lattice's row read as an EFFECTIVE
  material. Second-order (10-node) tetrahedra were the default and had to
  stay on: a skin one tetrahedron thick is a bending problem, and
  first-order tetrahedra lock. The script added the mid-edge nodes itself
  (straight edges; the corner nodes and so the surface untouched). This
  was the reference the shell model was checked against on small bodies.

A region without a structural material stops the solve and says which;
so does a body with no skin thickness (a solid): "shells only; give it a
skin thickness".

Phase 1 is dummy loads. The support fixes every degree of freedom of the
nodes on a coordinate plane -- y = 0 by default, the root of a mirrored
half and its symmetry plane; or the mesh's own minimum / maximum along an
axis. The loads are an inertial load factor (the body's own mass under
that many g along -z; 1 is its weight, a 4 g pull-up a real case), a
uniform pressure on the wetted faces (positive presses inward:
`POSITIVE_FACE_PRESSURE` acts against the face normal, and the faces go
in outward), and a point force at the node nearest a point. The job
already takes the pressure **per wetted face** (`pressure_file`), so
phase 2 -- the pressure field from the potential flow or DUST, at the
same tolerance and so face for face -- is a file in place of a constant.
Forces are newtons whatever the project's units.

Reported: nodes, elements and DOF; the peak displacement
and where; the peak von Mises overall and per region (each body's skin,
each network's ribs), with the
margin allowable / peak - 1; the mass; the force balance -- the reactions
at the support against gravity, the pressure's resultant and the point
force, which must be equal and opposite; and the mean of u.n over the
wetted nodes, so an inward pressure can be seen to move the surface
inward. The VTK holds the displacement (and the shells' rotations) and the
von Mises stress (element peaks over the integration points -- a shell's
over its top, middle and bottom surfaces -- averaged to the nodes), as
triangles, in SI, for ParaView. (Until 2026-11-08 the report also carried
the homogenised core's relative density, effective moduli, amplification
bound and estimated strut stress, and the VTK its tetrahedra.)

**Any body, as its Material Properties say (2026-10-31, later the same
day; the lattice and solid cases removed 2026-11-08).** The dialog picks
BODIES with the shared picker, and each becomes a REGION: with a skin
thickness, a thin-walled part -- shells of that thickness on its own
surface triangles and nothing inside -- which since 2026-11-08 is the
only case. As first written there were three more: no skin thickness, a
solid of its material meshed by gmsh at an element size (a
twentieth of the largest extent unless set); filled by a lattice, shells
of its skin thickness over tetrahedra at the cell size carrying the
lattice's homogenised tensor; or at full resolution the skin tetrahedra
as a solid of the material over core tetrahedra with the homogenised
lattice. The job (`AerotelicStructuralFea::FRequest`: `Regions`, each
with its skin and its `ShellFaces` in `faces.bin`) is one
form, `model: regions`; the script (`_regions_from_job`) still reads the
two earlier forms. The shells are 3-node on the
corner nodes, and a shell-only body's faces are 3-node conditions. The
document commandlet solved the sphere four ways then (full resolution,
shells over the homogenised core, thin-walled shell, solid); the
thin-walled shell (rho t A, the weight carried, p a / 2t) is the one that
remains, against the pre-processor.

**Bodies, loads and supports (2026-10-31, night; Matt's four points).**
(1) concerned the Lattice menu (it left the menu bar for a per-body
"Generate Lattice" on the Scene tree), all of it since removed -- see
*Shells only*. (2) A boolean result is reported as the
solid bodies it really is: `AerotelicMeshing::SplitShells` splits the
folded mesh into its connected shells (seam vertices welded within 1e-6
for the connectivity), one shell stays "Main", several are "Body 1" (the
largest) onwards -- a flap fused alongside its wing, parted by its gap,
is a body of its own. (3) The Structural dialog picks SOLID BODIES, one
row per body of every operand (`FAerotelicBodyChoice::Body`), and whatever
has no load path to any support is LEFT OUT automatically rather than
refused: `FBuilt` keeps the full mesh, each derivation (`Derive`) runs
the pre-processor, drops the elements on unsupported connected pieces
(`FModel::ComponentSupported`), renumbers, runs it again, and names what
went on the Mesh and Support tabs -- a control surface in its gap, a bay
inside the fuselage. (4) Any number of point forces and of support planes
(`FRequest::PointForces`, `FRequest::Supports`; the job's `point_forces`
and `supports` lists, one Kratos sub model part and process per point
force, the planes' node sets united; the older single keys are still
read): the Loads tab has a panel beside its picture -- the load factor,
the pressure, the point forces as a column of cards, each editable and
deletable, Add and Apply -- and the Support tab the same for the planes
(axis, where, value, tolerance). Apply pushes the inputs onto the built
model without re-meshing and redraws; Build Model and Solve take them
too.

**The wing-tip load, and the shell element (2026-10-31, evening).** Matt
loaded the P-38, put a test load on the starboard wing tip, clamped the
centre and saw stresses on the UNLOADED side. Reproduced in the
real-project commandlet (`AerotelicRealProjectTestCommandlet`, step
"p38a wing-tip load": `ue/Projects/p38a.yaml`'s fused BoolOp 1 alone as
a 5 mm 2024-T3 shell, clamped on the centre nacelle's ring within 5 cm
of y = 0, 5 kN down on the tip node, no other load, the direct solver),
which found TWO things. First, the corotational thin shell -- the
default until then -- is a geometrically nonlinear element: solved in the
one step of a linear analysis its internal forces are evaluated on the
displaced configuration while the step was linearised on the undeformed
one, so once the displacements are large (this skin-only airframe moves
2.7 m at the tip, the outer wing's moment entering the nacelle cowl with
no spar to carry it) the reactions no longer balance the loads
((-10, -11, 7) kN against 5 kN) and the stresses are nonsense (8.6e10 Pa).
Ruled out on the way, by probes on the kept job (`AEROTELIC_FEA_KEEP=1`
keeps the job directory; the probes are in `ue/tools/fea_probes/`, each
taking that directory as its argument):
floating pieces (one connected piece, clamped), sliver triangles
(dropping them changed nothing), the knife trailing edges, an isotropic
MeshLib remesh, the load path (a sphere shell carries a point load to
0.1%), and the linear solver. The LINEAR thin triangle
(`ShellThinElement3D3N`) closes the balance to (0.008, 0.000, 5000) N on
the same mesh, with the same displacement field, and along the starboard
outer panel (clear of the boom) the band peaks of von Mises are 1.08 to
1.44 times beam theory's M z / I of the actual thin-walled section (the
medians just under it); the port wing reads exactly zero, the clamped
ring at y = 0 isolating it. So the linear thin shell is now the default
in the script, the bridge (`FRequest::ShellElement`) and the dialog
(a "Shell element" combo keeps the two corotational triangles for small-
displacement comparisons). Second, the dialog's picker had started with
EVERY body chosen, the boolean's operands beside the boolean that already
contains them: the mirrored wing operand then sits in the model as a
second, unconnected shell that no support reaches -- a singular system,
rubbish everywhere. The picker now starts with the bodies no boolean is
built on, the pre-processor computes the mesh's connected pieces
(`FModel::Components`, `FloatingRegions`), the Support tab names any
region with no load path to the support, and Solve refuses it. The
inertial load factor now starts at 0, so a test load acts alone.

**The pre-processor tabs (2026-10-31).** The dialog is modeless and
tabbed (`Widgets/TabStrip`, the High Order Panel pattern): *Set-up* holds
the choices and the result read-outs; *Build Model* builds the bodies'
shells (the skin on the body's own grid, the rib discs by gmsh) and
derives, without
solving, what the solver will be handed; *Solve* builds the same model
and runs it. The derivation is `Entity/AerotelicFeaPreprocess` --
headless, from the `FRequest` alone, by the script's own rules: the
wetted faces are the skin's triangles (the tetrahedra's boundary faces,
while there were tetrahedra), outward, in
`np.unique`'s order (lexicographic by sorted node ids) so a pressure per
face would map face for face; the fixed nodes are the corner nodes
within the tolerance of the plane (`min` / `max` resolved on the mesh, a
zero tolerance a ten-thousandth of the diagonal); the point force goes to the
nearest corner node; the inertial load is the load factor times g on the
part's mass at the
mass-weighted centre. Three tabs draw it through `SAerotelicSceneView`
and `AAerotelicColoredMeshActor` (which gained an edge overlay through
`M_AerotelicWire`, `SetEdges`): *Mesh*, the wetted faces coloured by
region with their edges -- the shell elements themselves -- with the counts
per region; *Loads*, the surface in grey and the forces as arrows whose
heads sit at the point of application: orange the pressure as the
traction on up to ~1500 evenly spaced faces (direction and relative
magnitude, not the face force, so a uniform pressure draws uniform
arrows), red the point force at its node, green the inertial load at the
centre of mass; *Support*, a red marker on every fixed node and the plane
as a blue frame, with "fewer than three" flagged since the script
refuses it. Every figure is in the caption. After a solve the dialog
logs a warning if the solver's counts of support nodes and wetted faces
differ from the pre-processor's; the document commandlet asserts they
are equal on the sphere both ways, with the mass, the applied gravity,
the zero pressure resultant and the outward winding. There is no
post-processing view yet: the solution still goes to the VTK for
ParaView. `run_aerotelic_dev.bat -AerotelicFeaSmoke
-AerotelicProject=<path>\p38a.yaml` opens
the dialog, builds the model with every load on, saves
`Saved/Screenshots/fea_smoke_{mesh,loads,support}.png` and quits;
`-AerotelicFeaSmokeDelay=<s>` waits longer before the first picture.

**The solver that stopped short (2026-11-01).** A 1 kN tip load on
`hershey_bar.yaml` (NACA 0012, 10 m semi-span, 20 mm 2024-T3 skin, 1,980
nodes / 3,956 linear thin shells) came back from the dialog with 0.07 mm
of tip deflection, 4 MPa of stress and reactions of 300 N -- a stiff,
lightly loaded wing, and nonsense. Two causes, found by rebuilding the
job from the solution VTK and comparing with beam theory
(`tools/fea_probes/fea_beam_probe.py`): AMGCL (cg + ilu0, the default
then) does not converge on a shell-only system and says nothing -- with
the clamp right it still took 75 s to reach 0.1 mm and reactions of
(-2932, -294, -3351) N; `sparse_lu` on the same 11,880 dofs took 0.55 s,
balanced the load to 1e-8 and matched the cantilever formulas: tip
deflection 0.998 x P L^3 / 3EI, the spanwise shape within 4%, the
chordwise deflection from the 4-degree twist within 4%, M c / I at 0.94
to 1.14 on the straight part (the loft's washout curls the outer 1.5 m
upward by 0.8 m, so beam theory applies inboard of y = 8.5 with the
horizontal lever arm). And the loft's root section is planar only to
+-0.3 mm, so the zero tolerance of a millionth of the diagonal (0.01 mm)
clamped 2 of its 30 nodes. Hence: the dialog's solver choice is now
*Automatic* -- `sparse_lu` up to a million degrees of freedom
(`AUTO_DIRECT_DOFS`), amgcl beyond -- with the two named solvers still
offered; the script measures the force balance (|sum R + sum F| over the
loads' gross magnitude, so a closed-surface pressure still sets a scale),
reports `balance_error`, `balanced` and `warnings` in the result and the
solver it ran, and a miss of more than 1% (`BALANCE_TOLERANCE`) is
reported as NOT CONVERGED on the Force balance line, in the status bar
and in a message box; and a zero support tolerance is a ten-thousandth
of the diagonal (`SUPPORT_TOLERANCE_OVER_DIAGONAL`, mirrored in
`AerotelicFeaPreprocess`). `structural_fea_check.py` asserts the solver
used and the balance; the document commandlet asserts the result's
verdict on the sphere.

Two Kratos details worth knowing before touching the script: a load
assigned with `assign_vector_by_direction_process` /
`assign_vector_variable_process` must say `"constrained": false`, or the
process tries to fix a degree of freedom the load is not; and the
constraint process FREES the support's degrees of freedom again when
the step ends, so reactions are read from the Support sub model part,
not by asking which nodes are fixed.

The document commandlet solved the lattice-filled sphere both ways (both
tetrahedral models removed 2026-11-08; the figures stay as the record of
what the solver and the iterative-solver guard were checked against). As
skin / core tetrahedra -- an aluminium skin 0.05 thick on a unit sphere
over a very soft core, stood on its bottom under its own weight and 1 bar
of external pressure (64,056 DOF) -- it holds to closed forms: the mass is
the densities over the regions' volumes to 1e-6; the support carries the
weight and nothing else, the pressure's resultant over a closed surface
being zero to 1e-11; the surface moves inward; the skin's peak von Mises
(1.9 MPa) is Lame's thick-shell hoop stress for a/b = 0.95 (1.05 MPa)
plus the concentration where the weight enters the fixed cap; the soft
core carries under a tenth of that; and the displacement is the core
sagging under its own weight, millimetres at E = 1e5 Pa, while the skin
moves microns. This case runs on the direct solver (~2 min against 18 s):
with a skin 7e5 times stiffer than its core AMGCL stopped short, closing
the force balance to 1.7% only and reporting a peak of 1.27 MPa that the
converged solve does not reproduce -- the "residual not chased" of the
first cut was the iterative solver, not the model. As shells over the
homogenised gyroid (made of PMI foam; 2,628 DOF, 0.7 s to solve):
the body alone meshes at the cell size with fewer tetrahedra, the cell
voxelises near its 0.35 fraction, the effective modulus is cubic and
between the bounds, the strut stress estimate sits above the smeared
one, the mass is rho t A of the skin plus the overlap-corrected core,
the support carries the weight with the shells' share, the surface moves
inward, and the skin's peak von Mises is the thin-shell membrane stress
p a / 2t = 1.0 MPa up to the support's concentration.

**The beam study, three fixes and the Mesh tab (2026-11-02).** Matt asked
for the 1 x 10 x 1 m box of `Projects/simple_beam.yaml` (10 mm aluminium
skin) as a cantilever against beam theory, every element type, with the
inputs varied (the driver, every job and the results are in
`C:\temp\beam_fea`). Two symmetric 500 N loads at the tip on the webs'
mid-height, the root clamped; an end couple as the pure-bending case.
Found: the linear thin shell converges cleanly (tip 0.988 of Timoshenko
at h = 0.5 m, 1.000 at 0.0625; M c / I to 1%; the couple to 0.3%; amgcl
agrees with sparse_lu); the thick corotational shell is too stiff and
does NOT converge (0.82, 0.94, 0.98, 0.93 over the series) with its force
balance error growing 1e-4 -> 1e-1; the 10-node tetrahedra (tetrahedral
models, since removed) came out 8%
then 4% too flexible because `_support_nodes` ran on the corner nodes
BEFORE `second_order()` added the mid-edge nodes, so every mid-edge node
on the clamp plane was free (clamping them all gave 0.99 on both meshes);
and the box primitive was 12 triangles, which both routes used as given --
48 degrees of freedom of shells at 0.69 of the right deflection (0.05
with the thick shell), and the solid route's `core_size` had no effect at
all (gmsh keeps the boundary triangles, so 0.5, 0.25 and 0.125 all gave
the same 9-node mesh: Tet4 0.015, Tet10 0.76 of theory -- tetrahedral
models, since removed) -- all reported
as converged, the force balance being exact. Also measured: a support
tolerance wider than the element size clamps the next ring and shortens
the beam (0.963 = (9.875 / 10)^3); the shells sit on the outer surface
rather than t/2 inward, 3% stiff at 10 mm and 16% at 50 mm (Matt chose to
leave this); where the tip loads are put does not matter (the end cap is
a diaphragm). The fixes, as Matt chose them:

* **The clamp held the mid-edge nodes** (`structural_fea.py`; a fix to
  the second-order tetrahedra, removed with them on 2026-11-08): the
  support search ran again on `nodes_all` after `second_order()`; the
  result reported `support_nodes` (all) and `support_corner_nodes` (the
  pre-processor's figure), which the commandlet compared on 10-node runs,
  and `structural_fea_check.py` solved a 10-node bar and asserted no node
  on the clamped face moved. With shells only the two counts are one.
* **The thick corotational shell left the dialog** (the script still
  takes it in a job). The thin linear triangle remains the default, the
  corotational thin one the alternative.
* **Support tolerance guard**: the pre-processor reports each plane's
  furthest fixed node and the mesh's median wetted edge; a tolerance over
  a quarter of that edge is flagged on the Support tab and in the Apply
  status as liable to clamp a second ring.
* **A Mesh tab on every parameterisation** (wing, fuselage, nacelle,
  lofted nacelle, propeller, primitive; `Dialogs/MeshTab`), in place of
  remeshing in the analysis: the grid is defined with the parameterisation
  and every analysis uses it. `aerotelic::MeshSpec` (mesh.hpp) lives on
  `ISolidModel::mesh`, travels with clone / Apply / copy / paste / load /
  save (an entity-level `mesh:` key with only the non-automatic fields;
  WingDesigner ignores keys it does not know), and every `BuildMeshes`
  goes through `mesh.Apply(base, tolerance)` over its own defaults -- so
  the viewport, the FEA shells, the potential flow's and
  DUST's wetted surfaces and the STL / 3D PDF exports all see one grid.
  Fields, each 0 = automatic (what the model did before): *Panels along*
  and *Panels round* (`LoftOptions::grid_u / grid_v`: uniform intervals
  merged with the loft's seeds by `GridParameters`, the very construction
  `LoftNetwork` uses for its rows and columns, so the same numbers give
  the panel method the same stations and ring points), *Refine to
  tolerance* (off: the grid as it stands, no chordal or aspect
  refinement, and the triangles are the panel method's quads split --
  `mesh_check` asserts every corner of a structured ellipsoid's network
  is a vertex of its triangulation), *Tolerance*, *Longest strip times
  spacing*; a wing's *chordwise / spanwise* panel counts (the Panel Method
  dialog takes them from the entity where set, its own spins otherwise --
  the body spins likewise); a box's *cells* along x, y, z (`BoxMesh` with
  counts; automatic is cells about the shortest side, so the 1 x 10 x 1
  bar is 1 x 10 x 1 cells); a cylinder's *points round*, *cells along*
  and *cap rings* (`CylinderMesh` with counts, structured caps closing on
  the axis; automatic from the chordal tolerance). A read-out builds the
  draft and shows the triangle count. On a flapped wing the stations are
  shared out over the chain's segments by span. Not yet one grid: a
  wing's panel networks (cosine-spaced on the planform, `WingNetworks`)
  and the box's and cylinder's loft networks (`BoxLoft` / `CylinderLoft`
  panel their own u in [0, 3]) do not coincide with the triangulation.
* **Tests**: `mesh_check` (box cells, cylinder counts, the structured
  loft against `LoftNetwork`), `project_yaml_check` (`mesh:` round trip),
  `structural_fea_check.py` (the thin-walled box beam within 2% of
  Timoshenko and M c / I; the 10-node bar's clamp, while there were
  tetrahedra), and the document
  commandlet's step "structural fea (a box beam)": a box primitive given
  2 x 20 x 2 cells through a draft, the pre-processor's 9 root nodes and
  the coarse-tolerance flag, the solve within 3% of Timoshenko.

### Natural frequencies (2026-11-04)

The structural dialog's Model group gained *Natural frequencies to
compute* (6 by default; 0 switches it off) and its Result group a
*Natural frequencies* line. After the static solve `structural_fea.py`
solves the SAME model a second time -- the same mesh, materials and
clamps, no loads -- as the generalised eigenvalue problem K phi =
omega^2 M phi on StructuralMechanicsApplication's `eigen_value` solver
(the dynamic scheme: K the stiffness the static case used, M the elements'
own mass matrices from the densities and shell thicknesses), for the
lowest modes asked. The eigensolver
is LinearSolversApplication's Spectra shift-invert
(`spectra_sym_g_eigs_shift`), with the subspace solver
`eigen_eigensystem` as the fallback (either falls back to the other); on
the 1 x 10 x 1 m box beam Spectra takes 0.2 s against 1.1 s. The result
carries `natural_frequencies` (hertz, lowest first -- hertz whatever the
project's units, and a replica's under a model scale), the eigensolver
used and its seconds; `FResult::NaturalFrequencies`; the VTK carries each
mode shape as the point vector `mode_k` scaled to a unit largest
displacement, for ParaView. A failed eigensolve is a warning on a static
result that stands (the Result line says NOT COMPUTED).

What came out, and why it is right: the thin-walled 1 x 10 x 1 m
aluminium box (10 mm skin, clamped at one end) gives the degenerate first
bending pair at **10.81 Hz twice**. Euler-Bernoulli for the bare beam is
11.71 Hz; the shells are 7.7% under it, and that is three real effects,
not an error: the tip cap (27.8 kg on a mode whose effective mass is a
quarter of the beam's 1112 kg) takes 4.7%, the webs' shear flexibility
and rotary inertia (Timoshenko) 1.8%, and together the estimate is
10.97 Hz, so the shells are within 1.5%. The modes after the pair -- 26.1,
27.0, 28.4, 30.3 Hz -- are the 1 m skin panels' own plate modes, well
below the second beam mode at 73 Hz: a thin-walled part goes local long
before it bends again, which is the kind of thing this output is for. (The
10-node solid bar of the same proportions -- a tetrahedral model, since
removed -- had no cap and landed on the
Timoshenko cantilever frequency.)

* **Tests**: `structural_fea_check.py` -- `cantilever_f1` (beta^2 / (2 pi
  L^2) sqrt(EI / rho A) lowered for shear and rotary inertia to first
  order and for a tip mass by Rayleigh) against the box beam's pair within
  4% (and the 10-node bar's within 3%, while there were tetrahedra), the
  pair degenerate to 0.5%, the `mode_k` vectors in the VTK scaled to unit
  peak (the sandwich box's eigensolve on mixed six- and
  three-degree-of-freedom nodes went with the core); the document
  commandlet's box-beam step asks for four modes and checks the same.
* **Also 2026-11-04**: every body picker (booleans, STL / STEP / 3D PDF
  exports, mass properties, panel method, potential flow, DUST,
  structural) has *Remove All*, the reverse of *Select All*; and a changed
  skin thickness under Material Properties regenerates the generated
  structure over the entity (the lattices, then; see *Meshing and
  distance fields*), checked in the
  document commandlet (0.05 to 0.08 and back; the same spec
  again, or a density edit, queues nothing).

### The body's own grid, and a solve you can watch (2026-11-05)

(The lattice, the core tetrahedra and the element order this section
describes were removed on 2026-11-08 -- see *Shells only* below. What
stands from it is the own-grid rule, the DOF budget, the cost known
before the solve, and the watched, cancellable solve.)

Matt ran the structural analysis on the P-38's Center fuselage with a
honeycomb lattice inside it and it "stalled": no feedback, then a failure
after eight minutes. The log, the page-file counters and the job
directory it left behind said what happened. The solve was not hung: it
was a 540,000-degree-of-freedom model on the single-threaded direct
solver, which ran 32 GB of memory and a 23 GB page file dry. Three things
had multiplied a model that should have been a few thousand shells and
tetrahedra:

1. **The solver was not given the body the user picked.** The picker
   lists solid bodies; a body filled by a lattice was silently
   reclassified and its geometry taken from the lattice, which keeps a
   PRIVATE copy of each target's surface rebuilt at a quarter of the skin
   thickness so that MeshLib's distance field could resolve the hollowing
   (both gone since 2026-11-08).
   For Center (1 mm skin) that copy is 28,074 triangles where the Mesh tab
   grid the viewport shows is 1,574. The core tetrahedra were built on
   that copy (`GetStructuralTetMeshes`, now gone), so the FEA shells were
   eighteen times the grid the user had set, and the Mesh tab's whole
   purpose -- the user defines the discretisation every analysis uses --
   was bypassed. Worse, nothing in the UI said so.
2. **Second order was on by default for everything**, the homogenised
   core included, which the script's own header says needs no such thing
   (the shells carry the bending; the core carries a material card). That
   turned 24,571 nodes into 165,900.
3. **The automatic solver choice took the direct solver to a million
   DOF**, four times what a desktop can factor, and the dialog ran it on
   the game thread, so the window could not even paint the "Solving..."
   line it had set.

What changed:

* **Every analysis asks for the body's own grid.**
  `AAerotelicEntityActor::GetPlacedMeshes(bMirror, Tolerance = 0)`: a
  tolerance of zero means the viewport's meshes as they stand (the Mesh
  tab spec at the viewport's rule, 0.15% of the diagonal; built at that
  rule if the viewport has not yet). The structural dialog, the material
  dialog and mass properties (whose tolerance box now takes 0 for this,
  and starts there; the report says so) all ask for it. A caller with a
  tolerance of its own -- STL export (and, until 2026-11-08 late, the
  structure's hollowing; the structure's Generate now cuts the body's own
  grid) -- gets a private copy and must not hand it to a solver.
* **The lattice lent its material card only; the Model combo; an element
  order per region** -- three changes of this day that went with the
  tetrahedra on 2026-11-08: the dialog meshing the body's own grid with
  gmsh at the lattice cell (`TetMeshFor`, the `size_from_boundary`
  seeding choice), the Set-up tab's "FILLED BY Lattice 1" line and the
  *Ignore the lattices* choice, and `FRegion::SecondOrder` with the
  script's two element blocks. See *Shells only*.
* **The cost is known before the solve.** `AerotelicFeaPreprocess::FModel`
  counts, by the script's own rules, the nodes
  carrying shells and the degrees of freedom (six a node); Build Model's
  status line
  and the Mesh caption show it with which solver the automatic choice
  takes; past the budget Solve asks first. The document commandlet
  asserts the estimate equals the solver's `dofs` (as first written, on
  the 10-node sphere with its mid-edge nodes counted, the 4-node
  homogenised sphere and the 10-node solid as well).
* **The direct solver's budget is 150,000 DOF**, `AUTO_DIRECT_DOFS` in
  the script and `AerotelicStructuralFea::kAutoDirectDofs` in the
  application, amgcl beyond on every core. Checked first:
  `structural_fea_check.py --solver amgcl` passed every case with the
  reactions balanced to 1e-12 (as the cases then were: the mixed shell /
  core box, the shell-only
  beam, the pressurised sphere, the 10-node bar), so the 2026-11-01
  failure on hershey_bar was the clamp's tolerance and the shell element,
  not amgcl as such; the force balance still guards every solve.
* **A solve you can watch and stop.** `AerotelicMeshLib::RunJob` takes
  an `FJobProgress` (every line the script prints, as it prints it; a
  cancel flag that terminates the process); `AerotelicStructuralFea::Run`
  passes it through; the dialog runs the solve on a worker thread
  (`FSolveJob`, `Async(EAsyncExecution::Thread)`), a ticker shows the
  script's stage and the elapsed time in the status bar and in a line
  with a marquee above the buttons, Build and Solve go quiet, Cancel
  ends it, and the window may be closed while it runs. The script prints
  its stages (mesh read, writing the Kratos model, Kratos
  reading, assembling and solving with which solver, post-processing,
  the modes, the VTK). Meshing in Build Model is still on the game thread
  (it touches the entity actors; seconds, not minutes).
* **Tests** (as of this day; the second-order and homogenised-core cases
  went with the tetrahedra): `structural_fea_check.py` -- a region
  declining second order
  under a job that asked for it stayed 4-node and solved the same; a
  two-region solid with a 10-node skin over a 4-node core wrote both
  element kinds. The document commandlet's
  sphere meshes the body's own grid (tolerance 0, coarser
  than a finer tolerance's mesh) and checks the
  pre-processor's DOF and budget verdict against the solver's.

### Support patches, dragged in the picture (2026-11-05, night)

Matt: the infinite support plane clamps parts that should be free (x = 3
through the fuselage also takes the wing root), so let the plane's extent
be limited, by mouse, with live feedback. Done as an axis-aligned PATCH,
consistent with the planes the Support panel already has:

* **Model**: `FSupport::bBounded`, `Lo[2]`, `Hi[2]` along the plane's two
  in-plane axes -- the other two axes in cyclic order,
  `FSupport::InPlaneAxis`: x = c has its patch in (y, z), y = c in (z, x),
  z = c in (x, y) -- project units; the job's support entries carry
  `bounds: [u0, u1, w0, w1]` (SI) or null; `structural_fea.py
  _support_nodes` and `AerotelicFeaPreprocess` (`NodeOnPlane`, the one
  rule, shared with the dialog's live redraw) fix only the nodes within
  the tolerance of the plane AND inside the rectangle; the resolved
  planes report their bounds.
* **Card**: a "Limit to a patch" tick, the rectangle's four numbers with
  their axes named live from the plane's axis, *Fit to body* (the 'Of'
  body's extent on the plane, else the whole mesh's, with a 1% margin)
  and *Whole plane*. Full-scale like Value and Within.
* **Picture**: each plane's frame is drawn at its patch (amber) or at the
  mesh's extent plus 5% (blue, the whole plane) with eight handles -- the
  corners and the edges' midpoints (`FPatchHandle`). `SAerotelicSceneView`
  gained picking: `OnPickHover / Down / Move / Up` callbacks with the
  cursor's pixel and the view ray through it, `PixelToRay` and
  `WorldToPixel` in the capture camera's own convention (the eye at the
  focus minus the view direction times the distance; the field of view
  across the picture's WIDTH whatever its aspect -- checked by picking in
  the portrait-shaped Support picture, where the height rule misses the
  handles), and the hand cursor over a handle. A press within 14 px of a
  handle takes the drag (else the view orbits as before); a whole-plane
  support becomes a patch starting from its drawn frame; the cursor's
  point on the plane (ray / plane intersection) moves the corner's two
  sides or the edge's one, never past the opposite side; every move
  recomputes the fixed nodes by the pre-processor's rule and redraws the
  markers' actor alone (the surface stays), with the caption's counts
  marked provisional; release re-derives the model (`Derive(false)`: no
  reframing, the view stays where the user had it).
* **Checked live** with `tools/mouse.ps1` and screen captures on the
  box of the FEA smoke test (gyroid-filled at the time;
  `Saved/Screenshots/fea_patch_drag.png`): a corner
  dragged on the end face y = -2 sits under the cursor mid-drag, the
  patch's numbers and the count (20 to 6 nodes) follow, and release keeps
  them. Also seen: a left press that misses a handle orbits the view as
  before, so aim at the octahedra.
* **Tests**: `structural_fea_check.py` -- the beam's clamp limited to
  z >= 0 fixes exactly the root nodes with z >= 0 and the half clamp
  carries the tip loads; the document commandlet -- the thin-walled
  sphere clamped on a quadrant patch of z = min: the script and the
  pre-processor fix the same, smaller set, every fixed node inside the
  rectangle, the quadrant carrying the weight (293 passed).

### Shells only: the tetrahedra and the lattices removed (2026-11-08)

Matt's decision, after the per-body structure of the morning had put
skins, ribs, spars and frames on every body: the structure of an airframe
IS those things -- thin shells, a skin stiffened by plane members -- and
the application's structural model should be nothing else. The
tetrahedral route (the constrained Delaunay mesher of 2026-10-12, its gmsh
replacement, the skin / core split, the homogenised core, the second
order) and the lattice infill (OpenLattice's 36 cell types, their
calibration, the marching-tetrahedra and marching-cubes extractions, the
homogeniser) had grown, over four weeks, into a large part of the code
base -- several thousand lines of core, a Python tool chain and half the
structural dialog -- in the service of a printed-lattice interior that the
mission does not need. A solid body has no place in an airframe's
structural model either; if it needs analysing, it gets a skin thickness
and becomes one.

**Removed.** In the core: `tet.hpp` / `tet.cpp`, `tet_skin`, `tet_improve`
(the constrained Delaunay tetrahedral mesher, its skin / core split and
its refinement and improvement passes), `lattice.hpp` / `lattice.cpp`
(the OpenLattice TPMS, strut and honeycomb fields and their calibration),
`shell.hpp` (the offset-copy skin of 2026-10-11) and their tests. In the
tools: `homogenize.py` and `homogenize_check.py` (the Andreassen &
Andreasen homogeniser), `tet_gmsh.py` and `tet_gmsh_check.py` -- the
`.tri` reader, the DLL bootstrap and the discrete-surface helpers they
carried moved to `tools/gmsh_common.py`, which `potential_flow.py`,
`rib_gmsh.py` and `structural_fea.py` import. In the application:
`AerotelicTetGmsh` (now `AerotelicRibGmsh`, which meshes rib discs and
nothing else), the structural dialog's Model combo (as designed / full
resolution / ignore the lattices), the second-order (10-node) option, the
voxels and element-size rows, File's *Export Skin Tetrahedra (VTK)* and
*Export Tetrahedral Mesh (VTK)*, the Structure dialog's Infill tab and
`LatticeSpecControls`. The `lattice_meshlib*.py` scripts were renamed
`structure_meshlib*.py`, since hollowing and rib slabs were what they
did -- for the rest of the day; they went that evening (next section).

**The structural model now.** Every element is a shell. A body's skin is
shells of its skin material and thickness on the body's OWN grid (the
Mesh tab's, tolerance 0 -- the rule of 2026-11-05 stands). Each plane
network is rib shells: the skin is cut by every plane with the core's
`mesh_cut.hpp`, so a rib's rim is the skin's own nodes; the disc is
meshed in 2-D by gmsh through `tools/rib_gmsh.py` from that rim with no
new rim nodes, and crossing networks share their chord nodes. One region
per body and one per network, each with its material and thickness; rib
faces are not wetted and carry no surface load, their mass counts and
their peak stress is reported. A body whose skin thickness is 0 (a
solid) is refused by name: "shells only; give it a skin thickness". The
pre-processor, the DOF estimate, the 150,000-DOF direct-solver budget,
the watched and cancellable solve, the natural frequencies and the
support patches are as they were.

**Old files.** A project's legacy top-level `lattices:` list is skipped
and each entry reported by name; a body's `infill:` key (the lattice
infill the per-body structure carried for a few days) is dropped and
reported. Both appear under "Project Opened With Problems"; everything
else in such a file loads as before.

**Kept, and why.** `sdf.hpp` (the BVH distance field) and Shewchuk's
`predicates`: the mesh boolean needs both. MeshLib was kept at this point
for the hollowing (the Skin layer) and the rib slabs (the Ribs layers) of
a body's viewport and mass structure, `tools/structure_meshlib.py` -- for
a few hours: it went the same evening, see *Structure as shells: no
offset surfaces* next. gmsh: the rib discs and the
potential flow's fluid domain -- a tetrahedral mesh of the AIR, a CFD
mesh unrelated to the structure, which is the one place tetrahedra are
still built. The matched-row loft rule (`LoftSource::mirror_v` /
`mirror_u`) in `mesh.hpp`: the mesh boolean and the panel grids benefit
from it. The viewport's structure layers of a body are Skin and Ribs
only.

**Which earlier sections are now history.** *Tetrahedral mesh of the
bodies (2026-10-12)*, reduced to a marker above; the *Lattices* and
*Lattice menu* entries of *Meshing and distance fields* (deleted; the
*Per-body structure* entry is the present); the two models under
*Materials and the structural analysis* (shells over a homogenised core,
skin and core tetrahedra), kept as the record of what was measured; the
lattice, reclassification and element-order parts of *The body's own
grid*; *The interior grid's quality (2026-11-05, late)* (deleted -- its
one finding about the SURFACE survives: a fuselage whose nose is an apex
with a full ring just behind it, or whose tail is a slit of 100 points 1
mm apart, makes needle triangles, and shells on needle triangles are as
poor as the tetrahedra were; the polar cap it asked for is still
wanted). What those sections learned stays where it is still relevant:
the matched-row lofts in *Meshing and distance fields*, the mesh
boolean's seam pass under *Boolean operations*, the shell-element
measurements (thin against thick, against beam theory) in *Materials and
the structural analysis*, and the Delaunay refinement references
(Ruppert 1995, Üngör's off-centres, Shewchuk's 2002 survey) for whoever
next needs a quality bound on a 2-D rib disc -- gmsh's own 2-D algorithms
do that job today.

### Structure as shells: no offset surfaces (2026-11-08, late)

The same evening, after the editor build of the morning's work came up
and Matt looked at a generated body in the viewport. What he saw: hiding
and showing the Skin layer made it look like an offset COPY of the body
sitting on it (which it was -- the body's surface outside and the
inverted offset inside, a closed solid a skin thick); hiding the Skin
layer brought the body's own display mesh back, so the ribs could never
be seen on their own; and on a 1 mm skin the voxel budget had made the
offset voxel 7 mm, so the inner surface was unusable -- the result the
guards of 2026-11-01 report as "the inner surface touches the body's
surface", on the one skin thickness an airframe actually has. His
decision: no offset surfaces anywhere. The structure of a body is a pure
SHELL model, and the viewport, the mass report and the structural
analysis all read the same one.

**The model.** The SKIN is the body's own surface -- the Mesh tab's grid,
the triangles the viewport shows -- with the skin thickness as an
attribute of the body. The Skin layer has no mesh of its own any more.
A RIB, SPAR or FRAME is the planar shell disc its plane cuts out of the
body, bounded by the body's surface, with the network's thickness and
material as attributes. The thickness of a shell is a number carried
beside its area, never a second surface; nothing is offset, hollowed,
voxelised or booleaned to build a structure.

**Generate now.** `FAerotelicBodyStructure::Generate` cuts the body's
own grid (`GetPlacedMeshes`, tolerance 0; the job runs in world space,
since the thicknesses are physical build figures and the planes are
placed with the body) by
every plane of every network with the core's `mesh_cut.hpp`
`CutMeshByPlanes` -- the same cut the FEA model builder makes, so the
viewport's ribs and the solver's rib shells are the same discs -- and
cuts the reserved volumes (exclusions, payload bays) by the same planes.
Both go to `tools/rib_gmsh.py` through `Entity/AerotelicRibGmsh`
(`outer.tri`, the bays as `hole_k.tri` under a new `holes` job key, the
planes as before). The script reads each rib's rim loops off the cut
surface (the largest loop the outer boundary, any others holes of the
section itself), then places each bay's section on that plane: a bay
whose loop lies inside the rim is a HOLE of the disc; a bay crossing the rim is left out, with a
note in the result ("crosses the rib's boundary"); a bay that contains the
whole disc empties it, with a note. The disc is meshed in 2-D by gmsh
with the surface's own vertices as the rim nodes (transfinite rim lines,
no new rim nodes), crossing networks share their chord nodes, and a
height-limited network keeps only the triangles within that height of
the rim. The discs come back as the body's Rib layers, drawn two-sided
(a shell has no inside) and brought back into the body's local frame, so
they follow a move or a rotation without a job and are placed and
mirrored with the body; a change of shape or of model scale regenerates
them through the rebuild queue. `FStats` is now `Triangles`, `SkinArea`, `RibsArea`,
`Seconds` and `Warnings` (the script's notes).

**What the Scene tree shows.** The "Skin" row under a body controls the
body's own surface -- there is nothing else for it to control -- so
hiding it leaves the ribs alone in view, which is the picture Matt
wanted and the hollowed layer could not give. The Rib layers keep their
own visibility and colour. The rule of the morning ("the body's own
section hides while its Skin layer shows") is gone with the layer it
described.

**Apply builds; the preview shows one network (later that night).** Two
usability notes from Matt's first session with it. The Generate button
"did nothing": it did generate, but the discs are inside the skin and so
invisible until the Skin row is hidden, and once a body had been generated
Apply regenerated it anyway -- two buttons for one action. The dialog's
Apply now gives the body the structure and builds its layers in one step
(`UAerotelicDocumentSubsystem::ApplyBodyStructure`, which also takes the
regeneration the change had queued back out of the queue so the work is
not done twice); the Generate button is gone, the Scene tree's Generate /
Regenerate Structure entries remain for building without the dialog. And
the plane preview draws only the network selected in the Planes list --
every other network's planes were clutter on a wing with several. A third
proposal, hiding the body's surface while the dialog is open so the applied
ribs show, Matt declined.

**Mass.** Density x area x thickness for skin and ribs alike,
`aerotelic::SkinMassProperties` on the skin's grid and on each rib disc,
the same formula whether the structure has been generated or is estimated
from the body's surface and the planes' sections. A layer's "Volume" in
the dialog and the report is area x thickness. The mass report's rows
for generated layers are therefore shells spread over their surface, not
closed solids -- see *Mass properties*.

**Accepted errors** (Matt's call; typically under a few percent on an
airframe part). The rib disc runs out to the OUTER surface, so rib and
skin overlap along the rim: the double-counted material is the rim's
perimeter x the skin thickness x the rib thickness per rib. The skin's
area is taken on the outer surface rather than the mid-surface, so a
curved panel is over by about t/R (a 1 mm skin on a 100 mm radius, 1%;
a flat panel, nothing). Both are the price of a shell model with one
surface, and both are far smaller than the uncertainty in the figures
they multiply.

**Removed.** MeshLib entirely: the hollowing, the inward offset and its
voxel budget (the guards of *The offset remesh cannot run away*), the
offset cache, `tools/structure_meshlib.py` and
`tools/structure_meshlib_check.py`, the `pip meshlib` dependency of the
WingDesigner environment and the "without MeshLib, generation is
refused" message. `Entity/AerotelicMeshLib.h` keeps its name and is now
only the shared Python job runner -- `FindPython`, `RunJob`,
`WriteTri` / `ReadTri`, `FJobProgress` -- used by the rib mesher, the
structural analysis and the potential flow. `FAerotelicBodyStructure`
lost `Outer`, `Inner`, `IsInnerUsable` and `BodyTolerance`. Why the whole
route and not a repair: an offset surface is what a SOLID model of a
thin part needs, and the structural model is not one; a shell carries
its thickness as a number, so the inner surface was work (a distance
field, a voxel budget, a remesh, a cache) spent making a second surface
whose only reader was the mass report -- and what it read there, the
volume between two surfaces a skin apart, is area x thickness to within
the t/R above. The solver never saw it.

**Deferred.** STL export of the structure now writes OPEN shell
surfaces: the body's skin and each rib disc as a part. The thick-solid
layers for 3-D printing (a closed skin a thickness thick, closed rib
slabs) will be revisited when printing comes back; that code is in git
history at commit `2f48173`.

**Verified.** `tools/rib_gmsh_check.py`, with new cases for the bays (a
bay inside the rim: two hole loops, one per disc, no notes; a bay that
misses the plane: the full disc; a bay crossing the rim: no hole, one
note; a bay enclosing the disc: no faces, one note); the UE editor
build; the Document (271), RealProject (43) and Licence (42) commandlets.

## The Model menu and the material library (2026-11-07)

**Geometry became Model.** The menu that holds the airfoils, the
booleans, the units and the scale is called **Model** now (every "Geometry
> Units" / "Geometry > Scale" in the tooltips, the hints and this README
reads Model); `FillModelMenu` in `MainShell/AerotelicMainShell.cpp`. It
gained **Model > Material Properties**, the material library.

**What a material is, where it lives.** `aerotelic::Material`
(`core/include/aerotelic/materials.hpp`) carries, besides the four
figures (density, Young's modulus, Poisson's ratio, allowable stress --
SI as ever), a `category` ("Metal", "Composite", "Wood", "Foam", "SLA
resin", "FDM thermoplastic", or whatever the user types) and a `source`:
where the figures came from and the convention they follow, which every
preset now states (the handbook or data sheet, the condition, what the
allowable is). The library is the presets followed by the user's own
materials, kept in ONE file per user --
`%LOCALAPPDATA%\MS2 Aerodynamics\WingDesigner\materials.yaml`, beside the
licence (`materials:` a list of `{name, category, density, youngs_modulus,
poisson_ratio, allowable_stress, source}`; `WriteMaterialsYaml` /
`ReadMaterialsYaml` in core, `Entity/AerotelicMaterialLibrary` in the UE
module: `All`, `Custom`, `Find`, `FindMatching`, `SetCustom`, a path
override for the tests). An entity keeps a COPY of the material it is
given (name and figures, in the project file as before), so editing or
removing a library material changes nothing already assigned until it is
picked again, and a project opens the same on a machine without the
library. A user's material may not take a preset's name, names are
unique, and a material needs a positive density and modulus, a Poisson's
ratio in [0, 0.5) and a non-negative allowable
(`aerotelic::MaterialProblem`); the file is written on every change, and
an unreadable one is set aside as `materials.yaml.unreadable` rather than
overwritten.

**Model > Material Properties** (`Dialogs/MaterialLibraryDialog`) is
three tabs:

* **Materials** -- one row per material (the user's in bold): name,
  category, density, Young's modulus, Poisson's ratio, allowable stress
  (in the project's units; the header says which) and the source. Any
  column header sorts the table (`SHeaderRow` sort mode, ascending then
  descending; ties by name), the figures compared as numbers and the
  text case-insensitively. Hover a row for its full source; double-click
  one to take it to the Custom tab as a starting point. The woods the
  hobby trade builds from were added the same day: balsa in three grades
  (contest-light 100, medium 160, hard 240 kg/m^3 -- the Wood Handbook's
  balsa figures scaled in proportion to density after Borrega & Gibson
  2015), American basswood, Eastern white pine, yellow-poplar, Coast
  Douglas-fir (Wood Handbook Table 5-3, 12% moisture, nu_LR from Table
  5-1, allowable = compression parallel to grain) and two three-ply
  plywoods -- aircraft birch and poplar lite-ply -- at two-thirds of
  the solid wood along the face grain (ANC-18), beside the spruce that
  was already there.
* **Custom** -- the user's materials on the left; on the right a "Start
  from" list that copies any material's figures into the fields, the
  name, the category, the four figures in the project's units, a
  source / notes box, and Add as new / Update selected / Remove selected
  / Clear fields. Each of the three writes the file at once and says so
  on the status bar (and in the tab), or says why not.
* **Help** -- how to use the two tabs, how a material is assigned, the
  conventions behind each family of presets, and the citations: MMPDS-01
  and the ASM Handbooks for the metals, the USDA Wood Handbook
  (FPL-GTR-190) for spruce, Evonik's ROHACELL sheet for the foam, the
  Formlabs / Siraya Tech / Anycubic data sheets (post-cured; Anycubic's
  ranges taken at their conservative bounds) for the resins and the
  Prusament / Bambu Lab / Polymaker / Markforged / Stratasys / 3DXTech
  sheets (as printed, flat orientation) for the FDM thermoplastics, with
  the caveat that these are typical values, not design allowables.

**The alloy tables (2026-11-07, evening).** Matt asked for at least ten
alloys each of aluminium, steel, magnesium and titanium, and for the
sources to be ones whose figures can be redistributed publicly and
commercially. Every new row comes from one source: MIL-HDBK-5J
(*Metallic Materials and Elements for Aerospace Vehicle Structures*, US
DoD, 31 January 2003), a US Government work with Distribution Statement
A, public domain -- the FAA re-issued the same data as MMPDS-01, and
Battelle's later MMPDS editions are copyrighted and sold, which is why
the handbook's 2003 edition is the one cited. The PDF was downloaded
from everyspec.com and its 1733 pages text-extracted with pypdf; each
row's `source` names the handbook table, specification, product form,
condition, thickness range and basis, and the two strengths in ksi
(ksi x 6.894757 = MPa, lb/in^3 x 27679.9 = kg/m^3). Convention, unlike
the seven original metal rows (typical values): the allowable is the
handbook's design *minimum* tensile yield, A-basis where given else
S-basis, longitudinal; so these allowables sit 5-15% below the typical
yield of the same alloy, and the magnesium castings' "any area" minimums
sit far below test-bar figures. Where the handbook leaves Poisson's
ratio blank (CP titanium, Ti-5Al-2.5Sn, SP-700 and the three beta
alloys) a literature value is used and the source says so. Added:
aluminium 2014-T6, 2090-T83, 2124-T851, 2219-T87, 5052-H34, 5083-H321,
6013-T6, 7050-T7451, 7055-T77511, 7475-T761, A356.0-T6, A357.0-T6;
steels 1025, 4130 (125 ksi), 4140 (180), 4340 (200), 300M, D6AC,
4330V, H-11, maraging 250, AerMet 100, PH13-8Mo H1000, 15-5PH H1025,
17-4PH H1025, 17-7PH TH1050, 301 annealed and half hard, A-286;
magnesium AZ31B-F, AZ61A-F, ZK60A-T5, AM100A-T6, AZ91C-T6, AZ91E-T6,
AZ92A-T6, EZ33A-T5, QE22A-T6, ZE41A-T5; titanium CP grade 2 and 4,
Ti-5Al-2.5Sn, Ti-8Al-1Mo-1V, Ti-6Al-2Sn-4Zr-2Mo, Ti-6Al-4V STA,
Ti-6Al-6V-2Sn, SP-700, Ti-13V-11Cr-3Al, Ti-15V-3Cr-3Sn-3Al, Ti-10V-2Fe-3Al.
`mass_properties_check` counts at least ten of each and checks the
Ti-6Al-4V STA row against the handbook.

**Sources and redistribution (audit of 2026-11-07).** Of the sources the
presets cited, the public-domain ones are MIL-HDBK-5J, the USDA Wood
Handbook (FPL-GTR-190) and ANC-18 (US Government works), and MMPDS-01
was released as an FAA report through NTIS with no copyright notice
(prepared under contract, so not formally a 17 USC 105 work, but
equivalent to MIL-HDBK-5J, which is). The ASM Handbooks are copyrighted
by ASM International, whose terms require written consent to reproduce
any of their material (the 4130, 17-4PH and AZ31B rows and the co-cited
aluminium and Ti-6Al-4V rows). Every manufacturer data sheet (Formlabs,
Siraya Tech, Anycubic, Prusament, Bambu Lab, Polymaker, Markforged,
Stratasys, 3DXTech, Evonik, AK Steel/Cleveland-Cliffs) is copyrighted
and most of the sites' terms of use say personal / non-commercial use
only; individual factual values are not copyrightable in the US (Feist
v. Rural, 17 USC 103(b)), but the terms are a contract and the EU has a
separate database right.

**The split, and presets as YAML (2026-11-07, night).** Matt's decision:
ship only what may be redistributed, move the rest into the user's own
library. So the presets are now DATA -- `core/data/materials_presets.yaml`,
69 rows, every one from MIL-HDBK-5J, the Wood Handbook / ANC-18 or no
one's figures (the two representative laminates) -- and
`core/src/materials.cpp` holds no material at all: `core/CMakeLists.txt`
reads the file at configure time into a byte array
(`build/generated/materials_presets_yaml.inc`, a configure dependency,
so an edit to the YAML re-runs configure and rebuilds; `configure_file
COPYONLY` keeps the timestamp still when the bytes did not change) that
`MaterialPresets()` parses once with `ReadMaterialsYaml`. The seven
original metal rows lost nothing: public-domain MIL-HDBK-5J rows of the
same alloys were added under new names (`Aluminium 2024-T3 sheet`,
`7075-T6 sheet`, `6061-T6 sheet`, `Steel 4130, normalised`, `Steel 17-4PH
H900`, `Magnesium AZ31B-H24 sheet`, `Titanium Ti-6Al-4V annealed sheet`),
since a preset's name may never change and the old names now belong to
the restricted rows. The 42 restricted rows (the seven ASM-typical metals,
the ROHACELL foam, 24 resins, 10 filaments) went to
`core/data/materials_restricted.yaml` -- NOT embedded, NOT staged, kept in
the tree so they are not lost; delete it before publishing the repository
-- and were merged into Matt's `%LOCALAPPDATA%\MS2
Aerodynamics\WingDesigner\materials.yaml` (his one existing custom entry
kept, the previous file set aside as `materials.yaml.before-2026-11-07`),
where the dialog lists them in bold like any material of his. A one-off
`aerotelic_materials_split` tool (classify by source keyword, write both
files, merge into the user's) did the migration and was then removed.
`WriteMaterialsYaml` now streams the figures through the emitter at 12
significant digits -- a double assigned to a `YAML::Node` is stringified
at full precision on assignment, which is why the files used to read
`0.33000000000000002`; the already-written files were rounded in place.
Every reference to the old names moved to the new ones
(`StructuralFeaDialog`'s smoke-test default, the commandlet tests' 2024-T3
skins; the shells-over-core test builds its own round-figure foam instead
of the ROHACELL preset). `mass_properties_check`: the embedded presets
parse (>= 60), no preset cites the ASM Handbook or a data sheet, the
restricted file parses to its 42 complete rows with their figures, and
the library YAML writes 0.3 as 0.3.

**The ratio columns (2026-11-07, night).** The Materials table gained
E/ρ, √E/ρ, G/ρ, √G/ρ and σ/ρ -- Ashby's indices for a tie, a beam, a
shaft (section fixed; radius free) and specific strength, G the isotropic
E / (2 (1 + ν)) -- computed from the four figures in the project's units
(`IndicesOf` in `MaterialLibraryDialog.cpp`), each sortable like the other
columns (the order is unit-independent); the window grew to 1640 px to
hold them and the Help tab explains them.

**Scene tree > Material Properties slimmed.** The per-entity dialog no
longer edits figures. One list -- *Not set*, *An asserted mass*, then
every material of the library, plus the entity's own figures when they
match nothing in the library (an edited copy, a material since changed
or removed, a file from elsewhere) -- a skin thickness field (or, for an
asserted mass, the mass), and two read-only tables: the picked
material's card (category, the four figures, the source) and the
entity's geometry (enclosed volume, surface area, closed or open,
material volume, what it would weigh). Picking copies the library
material onto the entity; OK stores it as before.

**Tests.** `mass_properties_check` (102): every preset has a category and
a source; a material round-trips through the library YAML with every
field; an empty file is an empty library; a nameless entry and a figure
that is not a number are refused; `SameMaterial` and `MaterialProblem`.
The document commandlet's new "material library" step points the
library at a temp file: empty without a file, the presets alone in
`All`, a user's material accepted, written, found by name and matched by
figures (an edited copy not), a preset's name / a repeated name / a
missing modulus refused leaving the library as it was, and the file read
back with every field.

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
* `Widgets/SceneView` -- an engine-rendered 3D view inside a dialog: a scene
  capture (`Standalone/AerotelicSceneViewActor`, with its own backdrop)
  draws a chosen set of actors into a render target the widget shows,
  orbit/pan/zoom as the main camera, overlays on top. The capture is on
  demand -- for 1.5 s after any change (camera, size, content), nothing
  while idle -- and deliberately not a single frame: a procedural mesh
  created this frame reaches the capture before its vertex colour stream
  does and comes out white for several frames (seen 2026-10-26; the main
  view, redrawn every frame, never shows it). `-AerotelicSceneViewEveryFrame`
  switches to continuous capture for comparison. `Widgets/ColorBar`
  and `Widgets/LinePlot` are the colour scale and the x-y chart the
  analysis dialogs draw results with; `Standalone/AerotelicColoredMeshActor`
  is a vertex-coloured mesh (a field on a surface) seen by captures only.
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
* Not yet carried over from the Qt version: Show Intermediate Features and
  the STL dialog's mesh preview in the viewport. A section plane cuts without
  capping (see above), and the engine cannot rasterise SVG reference
  images.
* The core carries a higher-order panel method (Johnson, NASA CR-3079:
  curved panels, linear sources, quadratic doublets, closed-form influence
  coefficients, wakes) in `core/src/panel_integrals.cpp`,
  `panel_method.cpp` and `wing_panels.cpp`, checked against the report's
  cylinder, sphere, circular and rectangular wing cases
  (`tests/panel_method_check.cpp`, `wing_panels_check.cpp`), and drives
  Analysis -> High Order Panel. Not implemented from the report: the design
  networks (types 3-6) and the continuous doublet spline of its appendix
  B.3, so surface pressures on coarse thin wings read a few percent high
  (lift from the wake doublet strength is exact). A tip whose chord is
  under 2% of the root (`WingPaneling::collapse_tip_fraction`) is
  collapsed to a point -- one apex unknown under the potential condition,
  a column of triangular panels -- rather than paneled as a sliver of two
  dozen corner unknowns a millimetre apart (which gave the P-38 tip Cp of
  -260 and a negative tip load) or pulled in short of the point (which
  leaves the tube open at a finite chord, where the ending doublet sheet
  acts as a bound vortex: the tip strip then carried three times its
  neighbour's load). The collapsed thick tip is sound at the dialog's
  defaults and up to about 12 chordwise panels; at 16 or 20 chordwise the
  tiny leading-edge panels of the last strips still go wrong (Cp of -40
  to -2000 at the very tip), so refine a pointed-tip wing spanwise rather
  than chordwise. Bodies other than wings are source networks built from
  their lofts, not from their triangulations (a triangle mesh has no
  structured grid for the method's splines), so boolean results
  and propellers cannot be paneled; a loft whose end ring has collapsed to
  a line (the P-38 boom's nose and tail: zero half-width, finite height)
  gets no cap there, since a fan on a line is a ring of panels of no area
  whose frames are NaN and took the whole solution with them; bodies that
  penetrate each other solve, with the junction panels inside the other
  body left as they are (the P-38's fin root inside its boom reads Cp
  -120 there, which is why the colour range is a percentile one).
  Sideslip is accepted as an input but must be zero for now: a non-zero
  angle would need the symmetry image dropped, the solver's zeroing of
  the mirrored side force and roll/yaw moments (`panel_method.cpp`, the
  `mirrored` branch of the loads) removed, and the wake lift integrated
  along the real span rather than |dy|.
* Conditions on the end columns of a wing under the potential
  (interior-potential-zero) formulation, 2026-10-27, after the P-38 wing
  showed Cp -30 on its root and tip columns with a smooth μ and a root
  strip of three times its neighbour's load. The edge-midpoint and corner
  unknowns of a type-2 network only exist on its boundary, and under the
  potential condition they were collocated 1e-4 of a panel from the edge
  (`Options::edge_epsilon`), where the near-field potential of the
  neighbouring panel loses precision: the centre strengths hid it, the
  spline gradient -- the surface velocity -- did not. Now (a) those
  points are collocated half way from their lattice point to the owning
  panel's centre (`potential_edge_inset`; a fraction of the way, not of
  the diameter, which overshot the centre of a narrow column and landed
  by the opposite edge); (b) an edge on the symmetry plane or abutting
  another doublet network takes that edge's own exact condition -- zero
  normal gradient, continuity -- instead (`potential_edge_conditions`),
  since two potential conditions a fraction of a panel apart are nearly
  the same equation and the narrow root column's μ oscillated at 12 x 16
  (Cp -800); a free edge keeps the potential condition (μ = 0 is wrong on
  a thick body, whose μ is the outer potential), and an apex takes it too
  even on a wake-shedding edge (a gradient condition where the surface
  pinches to nothing is meaningless; the wake column it heads carries the
  apex's zero jump, as a pointed tip should); (c) where the influence-sum
  velocity's interior side misses V∞ by more than
  `interior_velocity_tolerance` (1.0 V∞ -- a thin section's healthy
  panels miss it by a few tenths) the panel's velocities are taken from
  the identity V_lower = V∞, V_upper = V∞ + σ n + ∇μ instead, which
  catches the tip panels sitting by tiny neighbours where the B.2
  spline's jumps act as vortex lines (the sum there was 3 to 6 V∞ with μ
  smooth); the sum stays the default because it is the more accurate
  where sound (the default wing's pressure lift lands 2% from its wake
  lift, against 7% by the identity throughout). With all of that the
  P-38 wing's loading is smooth and positive to the tip at the dialog's
  12 x 10, the pressure lift within 2% of the wake lift, the image
  within 1% of the explicit full wing. One residual: with the root
  station pinned to the plane the root strip reads 7% under its
  neighbour at 12 x 10 and 22% at 12 x 16 (the pinned root and the
  tilted next station make the root panels skewed; the solver's thin
  sheet, run in the core tests, shows no dip; without the pin the root is smooth at 10
  spanwise but blows up at 16), so take the root strip of a wing with
  root dihedral with a pinch of salt. `edge_epsilon` = 1e-4 still
  governs velocity-condition networks and wake heads (at 3e-3 and above
  the points no longer register on the wake abutment and the lift is
  lost).
* The post-processing widgets (the capture view, the colour bar, the
  plots) cannot be exercised by the commandlets (no Slate there). The one
  automated run is `run_aerotelic_dev.bat`'s switch
  `-AerotelicPanelSmoke` (with or without a project; an empty scene gets a
  default wing; no welcome screen): the High Order Panel dialog opens,
  solves at 0 and 4 degrees, steps through its three result tabs saving
  `Saved/Screenshots/panel_smoke_{surface,loads,polars}.png`, and quits.
  Look at the pictures. With `-AerotelicProject=<path>\p38.yaml` it
  solves the whole P-38 (seven bodies, ~2400 unknowns, a few seconds a
  case) and the pictures show the multi-body views: the eye-lit Cp on
  every body, the wing's loading with cl, the per-body polars with the
  total.
* The wing Control Surfaces tab carries trailing-edge flaps only (a table
  of span extent, hinge chord fraction at each end, gap and side gap, drawn
  on a plan view). The cut is made in every section plane with no boolean
  operations (`core/src/section_cut.cpp`; the main element stays one closed
  body with a pocket per flap, `TriangulateLoftChain` in `mesh.cpp`); the
  pocket is the flap's span widened by the side gap at each end. Not yet:
  dragging the hinge on the plan view, deflection, slats, and a flapped
  wing's STEP export (still the one-piece wing).
* A propeller blade's mesh is trimmed at the hub
  by its span range rather than a box cut (exact without rake). A nacelle
  with chevrons still goes through OCCT. (The lattice limits that stood
  here -- reconstructed strut cells, axis-aligned honeycombs, the cost of
  a thin wall on a large part -- went with the lattices on 2026-11-08.)
* Legacy project files: WingDesigner's older format keys entities by `id`
  with a display `label` (`Projects/cessna_172.yaml`); the label becomes
  the name and the raw `features` graph is ignored (the mirror nodes it
  holds are covered by the entity's own `mirror` flag; its booleans are
  not recovered -- only the current `booleans:` list is read).
* 3D PDF: **not yet opened in Adobe Acrobat.** The writer is checked
  against ECMA-363 and ISO 32000-1 clause by clause, and
  `pdf3d_check` validates every size and offset in both containers, but
  no structural check can tell whether Acrobat's U3D loader accepts the
  file -- in particular whether it honours the no-compression profile
  bit, which the specification requires of every reader but which few
  writers exercise. Open `core/build/aerotelic_3d_sample.pdf` (a sphere,
  a box and a half-transparent cylinder) before trusting the feature; if
  the model does not appear, the fallback is to implement the
  specification's arithmetic coder (ECMA-363 clause 10 and Annex A.3.1,
  about 120 lines) and write the compressed profile instead. Only
  Acrobat and Reader display 3D annotations at all; other PDF viewers
  show the page and the frame but no model, which is a property of the
  format, not of this writer. The PDF-side dictionaries were checked
  against a known-good SOLIDWORKS 3D PDF, which differs only in
  carrying PRC where this carries U3D.
* Build gotchas: a TArray must not be appended with a reference to its own
  element (`Pts.Add(Pts[0])` reallocates under the reference); if a crash
  leaves `CrashReportClientEditor.exe` running it holds the module DLL and
  the next link fails with LNK1104 -- kill it first.
* The packaged build is `Development` configuration (native file dialogs via
  the DesktopPlatform module are not available in `Shipping`).

## Section aerodynamics: conformal map, re-fit, sensitivities (2026-11-11)

**Analysis -> Section Aerodynamics** (and **Analyse section...** in the
Airfoil library) is the two-dimensional aerodynamics of one airfoil per
unit span, and the tool for re-representing a library airfoil with fewer
design variables while watching what that costs. Everything lives in the
core (`fft.hpp`, `fourier_periodic.hpp`, `periodic_spline.hpp`,
`conformal_airfoil.hpp`, `section_aero.hpp`, the `ApproximateCurve` in
`nurbs.hpp`, `Airfoil::Refit` / `CompareAirfoilFit`); the dialog is
`Dialogs/SectionAeroDialog`.

**The method registry.** `section_aero.hpp` fixes one request (the
contour, the angle of attack, the Reynolds number, the resolution) and one
result (surface distributions from the stagnation point, the coefficients,
the cues, diagnostics) that every analysis method fills; the dialog runs
every ticked method on the same closed section and overlays their pressure
distributions on one chart with a row per method in the table, so methods
are juxtaposed directly. The trailing-edge closure and the viscous cues are
layered on every method's inviscid result identically (`SolveSection`).
The first method is the conformal map; the next (a 2-D strip of the Johnson
panel method, a Kratos 2-D potential flow with a wake, XFOIL if installed)
are registry entries.

**The conformal map** is Theodorsen's (NACA TR 411; Theodorsen & Garrick
TR 452) in the form David Ives gave it in *A Modern Look at Conformal
Mapping, Including Multiply Connected Regions*, AIAA Journal 14(8):1006-1011,
1976. A Karman-Trefftz pre-map with exponent `n = 2 - tau/pi` (`tau` the
trailing-edge angle, measured by a least-squares quadratic through the
vertices within 1.5% of chord of the trailing edge so a cusp reads as a
cusp) takes the section to a near-circle; its second singular point sits
inside the nose at half the leading-edge radius. Theodorsen's map to the
exact circle is the fixed point of Garrick's conjugate-function relation
`eps = K[psi(phi + eps)]` -- `psi(theta)` a periodic cubic spline through
the near-circle's vertices, `K` the exterior harmonic conjugate evaluated
with the FFT each sweep (a 512-point grid converges to 1e-10 in about ten
sweeps, well under a millisecond). The circle flow at incidence with the
Kutta condition at the trailing edge's image gives the surface speed
`q = |w_circle| / (|dz/dzeta'| |dzeta'/dzeta|)`, with
`|dzeta'/dzeta| = exp(psi - psi0) sqrt((1 + eps')^2 + psi'^2)`. Cl comes
both from Kutta-Joukowski (exact) and from integrating Cp round the grid
polygon; the integrated pressure drag is the d'Alembert residual. The
checks: `conformal_check` reproduces the exact Cp of a Joukowski airfoil
(cusp) to 3e-5 and of a van de Vooren airfoil (10 deg trailing edge) to
3e-5 away from the trailing edge; `section_aero_check` gives NACA 0012 a
lift slope of 6.92/rad and `Cp_min = -0.414` (Abbott & von Doenhoff's
v/V = 1.196), NACA 2412 `alpha_0 = -2.11 deg` and `Cm_c/4 = -0.059`.

**The FFT is in-house and segregated on purpose.** FFTW was the first
choice and was withdrawn when its GPL licence turned out to be incompatible
with the application's distribution; `fft.hpp` is the only interface the
rest of core sees (FFTW's sign convention, in-place, power-of-two sizes)
and `src/fft.cpp` is a plain iterative radix-2 transform that an optimised
library (MKL's FFTW3 wrappers, pocketfft, KissFFT) replaces by swapping
the three function bodies and nothing else.

**Trailing edge.** Every section is closed before analysis in XFOIL's
manner: the gap vector between the two trailing-edge points is blended out
over the aft 30% of chord (half the gap each way, weight
`((x - 0.7)/0.3)^2`); the gap removed and the trailing-edge angle are
reported. A finite angle is an inviscid stagnation point (Cp 1 there); a
cusp keeps a finite speed.

**Viscous cues, Mach 0, Reynolds number the user's.** On each surface from
the stagnation point: the *transition cue* is the earlier of the station
where `Re_x = Re (V/U)(s/c)` reaches 3.5e6 and the first adverse pressure
gradient (the suction peak); the *skin-friction drag* integrates the
turbulent flat-plate law `Cf = 0.0592 Re_s^-0.2` at the local edge speed
from the stagnation point, projected on the wind (0.0108 for NACA 0012 at
Re 1e6 and 0 deg; a plate gives 0.0094); the *separation cue* is
Stratford's criterion (J. Fluid Mech. 5:1-16, 1959), fully turbulent from
stagnation given the transition cue's fidelity: from the suction peak,
`Cpbar = 1 - (V/Vmax)^2`, `x` from the equivalent flat-plate origin
`x_EL = integral (V/Vmax)^3 ds` over the favourable run, separation where
`Cpbar (x dCpbar/dx)^1/2 (1e-6 Re_x)^-0.1` reaches 0.39 (0.35 where
`d2p/dx2 < 0`), valid while `Cpbar <= 4/7` (the grey marker is where that
ends). Because an inviscid distribution recovers to stagnation at a
finite-angle trailing edge, the criterion usually trips in the last few
percent of chord at low incidence -- read that as "no separation ahead of
the trailing-edge recovery". Stratford's second expression for
`Cpbar > 4/7` is not implemented.

**Re-fit.** A library airfoil is a B-spline interpolating every one of its
points (one pole per point, 60-160 poles). `ApproximateCurve` is Piegl &
Tiller's least-squares approximation with a fixed number of control points
(9.4.1, A9.7: end points interpolated, normal equations over the interior
poles, dense Cholesky) followed by three rounds of Hoschek's parameter
correction (9.4.3), so the residual at the assigned parameters is the
normal distance. Knots by averaging over the data parameters (every span
holds data), uniform, or given (the dialog's "Keep current knots").
`CompareAirfoilFit` reports the largest and RMS normal distance of the
library points to the new curve (closest points by sampling plus
golden-section search), where the largest is, the change of maximum
thickness and camber, and the trailing-edge gap. Measured on NACA 2412
(161 points): 10 poles 1.1e-2, 20 poles 1.9e-3, 30 poles 5.8e-4, the worst
point always the leading edge, where the knot density by data count is
lowest relative to the curvature -- curvature-adapted knot placement is
the known improvement.

**The dialog.** Left, the section's control-point editor (the shared
`SCurveEditor`, free-x, equal aspect, the library contour as a grey
outline, the trailing-edge poles locked by default) over the
Representation group (degree, pole count, knot placement, Refit) and the
error read-outs; at each pole an arrow shows the central-difference
sensitivity of a chosen scalar (Cl, Cm c/4, friction drag, Cp at a probe on
either surface) to that pole, drawn through the editor's outline provider
(no editor change), the longest 0.08 c with its magnitude stated. Right,
the conditions (angle, Reynolds, points, methods), the Cp chart
(`SAerotelicLinePlot` with the new `InvertY`; upper solid, lower dashed;
yellow transition, red separation and grey range-end markers; Stratford's
S optionally on the right axis), the coefficient table and the cue notes.
A Polar tab sweeps -6..16 deg. Every edit re-solves at once (sub-ms); the
fit report, the sensitivities and the polar run on a ticker once the poles
have been still for 0.2 s. Apply clones the edited section into the
session's custom airfoil library under the given name, where every section
picker lists it; the library airfoil is never changed.
