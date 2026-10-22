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
* **Colour theme** (Edit -> Preferences -> Color Theme...): WingDesigner's
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
  (Potential Flow (Kratos), DUST (Vortex Particle Method)), Edit
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
  **Only parameterisations can be copied.** A boolean result or a lattice
  is a solid built *from* other entities with no parameterisation of its
  own -- there is nothing to hand a constructor, and a "copy" could only
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
* **Surface** (View -> Surface...): a modeless window with Metallic,
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
* **Shadows** (View -> Shadows..., 2026-10-15): a modeless window over the
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
  bottom sliders of View -> Surface... (`FAerotelicSurfaceStyle::
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
  picker (lattices and boolean results included -- a 3D PDF carries
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
* **Boolean operations** (Geometry -> Boolean Operation...): WingDesigner's
  dialog -- the operands in order (every entity, the mirrored half of each
  mirrored one as its own row, every earlier boolean result), Add / Select
  All / Remove / Up / Down, the operation (Fuse, Cut = first minus the
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
  (`core/tests/mesh_boolean_check.cpp`). KNOWN OPEN: the tetrahedral
  mesher still degrades on the union (the coarse body's cut triangles fan
  from a far corner to the dense seam, edges no Delaunay sphere can hold);
  the experimental in-plane refinement behind
  `MeshBooleanOptions::refine_min_angle_deg` (off) fixes some junctions
  and worsens others -- see the header. The result is drawn in
  lilac as one body, shown, coloured, renamed and deleted like any row
  (no design panel, placement or mirror of its own), is a lattice target
  and a tetrahedral-mesh body like any other, and has no B-rep: it is not
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
  per entity with position, rotation axis and angle.
* **Status bar**: last action on the left, project path on the right.

## Meshing, distance fields and lattices (2026-10-11)

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
  (`tests/sdf_check.cpp`).
* **Lattices** (`lattice.hpp`, `src/lattice.cpp`), after Gayir, Ozdilek,
  Ozdemir, Simsek, Sendur and Kiziltas, *OpenLattice* (SSRN 6210543),
  section 2: the 27 TPMS level-set functions of the paper's Table 3 in
  network, sheet (matrix) and iso-surface (wall thickness) phases, with the
  level-set parameter calibrated to the volume fraction by bisection on a
  sampled unit cell; the strut-based and auxetic cells (BCC, FCC, G7, Grid,
  Octet, Tesseract, Vintiles, X-lattice, Re-entrant) as the paper's
  periodic strut distance field (nearest periodic copy by rounding,
  distance to the segment, normalised by the strut radius, minimum over
  struts), the radius calibrated the same way. Honeycomb (prismatic) cells
  are ours, not the paper's: a 2D tiling's walls extruded along a chosen
  axis -- Square, Rectangle, Hexagon and Triangle, the cell size being the
  side of a square or triangle, the width of a rectangle or a hexagon's
  size across the flats, with a cell aspect stretching the cross-section's
  second dimension. Square, Rectangle and Triangle are the exact distance
  to two or three families of parallel lines through the tiling's lattice
  points; Hexagon is the distance to those points' Voronoi edges, taken as
  the nearest point's bisector half-planes (inside a convex cell the
  distance to the boundary is the least distance to its edges' own lines),
  so every cell's field is a true signed distance and the aspect stays
  exact. The wall is either calibrated to the volume fraction like
  everything else or given outright -- what a printed or bonded core is
  actually specified by. Unlike the other two families these cells are
  anisotropic on purpose, so the prism axis is part of the spec.
  `GenerateLattice` samples
  the lattice field intersected with the body's distance field (and minus
  the reserved volumes) on a grid of N voxels per cell, skipping whole
  blocks far from the body surface (a 1-Lipschitz field lets one
  evaluation classify a block), and extracts the surface with marching
  tetrahedra (vertex-shared, outward, parallel by slabs). With a skin
  thickness it extracts TWO bodies from the same samples: `skin`, the solid
  shell between each body's surface and its inward offset (a reserved
  volume keeps a wall too), and `mesh`, the lattice clipped to that offset;
  they meet on the offset surface vertex for vertex, and are kept apart so
  an analysis can give shell and infill different materials. A region
  thinner than two skins comes out solid shell, which is what a thin
  trailing edge should be: the sampled field cannot represent a wedge
  thinner than a voxel, and without a skin the infill's struts simply end
  wherever they last met the skin, raggedly on a swept edge.
  A wall set apart from its cell (a honeycomb's, or a TPMS iso-surface's)
  can be asked for thinner than a voxel, and then it falls between the
  samples and is simply not there; `LatticeResolutionWarning` says so, the
  dialog shows it while the numbers are being chosen, and generation throws
  rather than return a holed mesh.
  `tests/lattice_check.cpp` calibrates every type, checks the marching
  tetrahedra on a sphere, and generates in a box and in a sphere with a
  reserved sphere and a skin (both bodies closed, the shell's volume right).
  The honeycomb cells are checked against closed forms -- the void is each
  cell shrunk inward by half the wall, so a cell with an incircle gives
  (1 - t/2r)^2 and a rectangle (A-t)(B-t)/AB -- plus a stretched hexagon
  against a patch many cells across (converging on the one-tile figure, so
  the tile is the right fundamental domain), the field's being prismatic,
  periodic and 1-Lipschitz, and a hexagonal fill of a box whose mesh volume
  matches the field's own share of that box. Note that the share of a small
  box runs under the fraction asked for by order 1 / cells across, the
  boundary cutting the tiling; the calibration is exact, the box is not.
* **Lattice menu** -- *Generate Lattice...* opens the dialog: family (TPMS /
  Strut-based / Auxetic / Honeycomb), type, TPMS phase, prism axis and cell
  aspect (honeycomb), unit cell size, volume fraction or wall thickness
  (whichever the honeycomb's "wall set by" chooses; wall thickness for the
  iso-surface phase), voxels per cell (the
  paper's mesh density), solid skin thickness (always positive: one percent
  of the largest root chord among the wings filled by default, else of the
  largest body's longest extent; a saved lattice without one gets the same
  default on load), the bodies to fill (every non-primitive body, mirrored
  halves and boolean results included, chosen by default), the reserved
  volumes (every primitive, chosen by default -- WingDesigner's primitives
  mark payload bays), a name, and a live estimate of the sample grid (with
  a honeycomb's wall thickness and how many voxels span it). The
  result is a scene item of kind *lattice* with two bodies, "Skin" (the
  outer shell) and "Lattice" (the infill): each shown, coloured, hidden,
  sectioned, exported to STL (not STEP: they are meshes; an ASCII STL
  carries them as separate named `solid`s) and saved under `lattices:` in
  the project file (parameters and body names), regenerated on load. Every
  body filled or reserved must be a closed, outward mesh: the distance
  field's sign is meaningless on an open one (it reported "inside" in
  whole regions behind the P-38's boom tails before the knife-edge fix),
  so generation is refused naming the body; and each body's field is
  clamped positive outside the body's own bounding box as a second line of
  defence. Edit (double-click or the row menu) reopens the dialog and
  regenerates; *Regenerate Lattices* rebuilds every lattice over its bodies
  as they now stand -- a lattice does NOT follow edits to its bodies by
  itself (generation takes seconds, which would make every keystroke in a
  wing panel wait). Deleting a filled body deletes the lattice. Lattices
  carry no collision (click-select them in the tree). Measured on p38: the
  default cell (a tenth of the largest extent) generates in 0.2 s; a
  0.4-unit cell (19.5 M samples) in well under a minute.
* **Hollowing and infill on MeshLib** (2026-10-14; `Entity/AerotelicMeshLib.h`,
  `tools/lattice_meshlib.py`, `tools/lattice_meshlib_check.py`). The
  sampled skin and the tetrahedra-bounded skin both showed visible faults
  (uneven skin thickness, stair-step artefacts where the infill met the
  offset surface, a ragged interior volume), so the lattice's two bodies
  are now made by MeshLib (pip `meshlib` in the WingDesigner micromamba
  environment), run as a separate process the way the STEP writer is:
  the application writes the placed bodies as `.tri` files (its own
  indexed format, so the loft's shared vertices survive) and the lattice
  field -- still the core's `LatticeField`, all 36 OpenLattice types,
  calibrated as before -- as raw float32 on the generator's grid, and
  reads the two bodies back. *Hollowing*: each body is offset inward by
  the skin thickness on MeshLib's OpenVDB distance field (voxel a quarter
  of the skin, no coarser than the lattice voxel, capped by a voxel
  budget), which cannot self-intersect and rounds a concave crease from
  the inside; the offset surface is decimated within a fifth of its voxel
  and pockets smaller than four voxels cubed (slivers along a trailing
  edge barely thicker than two skins) are filled solid; "Skin" is the
  body's own surface outside and the inverted offset inside, so its
  outer surface is exact. *Infill domain*: the union of the offset
  interiors (a mesh boolean where they overlap) minus the reserved
  volumes (mesh boolean difference). *Infill*: the field goes through
  MeshLib's marching cubes, padded so the surface closes, and is cut to
  the domain by an exact mesh boolean, so "Lattice" meets the offset
  surface along a true intersection curve rather than a resampled one.
  The achieved fraction reported is lattice volume over domain volume.
  The interpreter is `AEROTELIC_MESHLIB_PYTHON` if set, else the
  WingDesigner environment's `python.exe` (the script puts the
  environment's `Library\bin` on its own DLL path: numpy's BLAS lives
  there); without MeshLib, generation is refused with a message saying
  so. The script's self-check (`python tools\lattice_meshlib_check.py
  [body.stl] [--keep DIR]`) runs the pipeline on a unit sphere or an STL
  and checks both bodies are closed and outward, the skin's volume is
  body minus interior, the infill stays inside the skin and the fraction
  is near the one asked: the sphere in 0.35 s, the P-38 port wing in
  0.19 s. *Export Skin Tetrahedra* still writes the skin as the
  tetrahedral mesher's skin tetrahedra, built on request rather than with
  the lattice.
* **Explicit skin layer** (`shell.hpp`, `src/shell.cpp`, 2026-10-11
  afternoon; the first step of the tetrahedral route Matt chose): the
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
  the application (2026-10-12 night): the lattice's "Skin" body is now the
  boundary of the skin tetrahedra the tetrahedral mesher builds
  (`tet.hpp`, see "Tetrahedral mesh of the bodies"): the body's own
  surface triangles outside, the exact iso-surface of its distance field
  at the skin depth inside. The offset copy could not represent an inward
  offset across a concave crease -- where the tailplane's leading edge
  meets a boom on the fused airframe its straight inner edges bridged the
  corner and came out through the leading edge by 10 to 13 mm (the wing's
  trailing edge at the boom, 9 mm) -- nor at a leading edge sharper than
  the skin, where it folded. The iso-surface rounds a concave corner from
  the inside and cannot self-intersect. The lattice is sampled where the
  body's field is deeper than the skin (the generator's own sampled skin,
  which needed the voxel finer than half the thickness, is no longer
  used), the achieved fraction reported is that of the interior, and
  *Lattice -> Export Skin Tetrahedra (VTK)...* writes every lattice's skin
  tetrahedra (material 1 of those meshes) as one legacy VTK unstructured
  grid. `shell.hpp` remains the source of the mesher's offset nodes only.
  SUPERSEDED AGAIN (2026-10-14): the viewport's "Skin" and "Lattice" are
  MeshLib's (see "Hollowing and infill on MeshLib" above); the skin
  tetrahedra are built only for *Export Skin Tetrahedra*.
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
  body (spaces as underscores), so a lattice's shell and infill arrive as
  separate parts; binary STL has no parts and holds the union.

### Tetrahedral mesh of the bodies (2026-10-12)

`core/include/aerotelic/tet.hpp` is a tetrahedral mesher for the finite-
element route: **Lattice -> Export Tetrahedral Mesh (VTK)...** meshes every
body a lattice fills and writes one legacy VTK unstructured grid with cell
data `material` = 1 for the skin (the lattice's skin thickness) and 2 for
the core (the volume the lattice's orthotropic material card applies to).

* **Exact conformity.** The outer faces of the mesh are the body's own
  surface triangles (or sub-triangles of them: Steiner points lie on
  surface edges only), no node lies outside the body, and the kept
  tetrahedra's volume equals the surface's enclosed volume to rounding --
  `core/tests/tet_check.cpp` asserts this on the P-38's bodies. It is a
  constrained Delaunay tetrahedralization: incremental Bowyer-Watson on
  Shewchuk's exact predicates (`core/third_party/predicates`, public
  domain, the double version compiled as C with `/fp:strict`), segment
  recovery by splitting at the reference point (Si / TetGen), face
  recovery by cavity re-tetrahedralization (Si & Gartner), exterior
  removal by crossing parity. Planar regions (end caps) are facets: their
  interior edges are free and the mesh's own in-plane triangulation is
  adopted. Three things the surface itself needed: planar caps are
  re-triangulated to their in-plane Delaunay triangulation; the loft
  samples each ring at its own density and adds stations wherever a
  strip is much longer than its rings' point spacing (`LoftOptions::
  max_aspect`), which also cuts triangle counts (the Boom from 17k to 7k);
  and a thin body's two sheets are sampled at MATCHED rows
  (`LoftSource::mirror_v` for an airfoil's upper and lower surfaces at the
  same chordwise station, `mirror_u` for a cowl's outer and inner skins at
  the same x). A vertex of one sheet under the middle of an edge of the
  other, nearer than that edge's own sag, leaves the edge no empty
  circumsphere, and the mesher splits it, and its pieces, down the wedge to
  the length floor -- which is what degraded the nacelle (2026-10-12: it
  now conforms exactly, with no Steiner point at all). The mesher keeps
  rows matched as it adds points: an edge point planted on one sheet of a
  thin region gets a twin straight across the thickness on the other
  (a BVH ray cast, `MeshSdf::Raycast`).
* **Skin / core.** The surface's inward offset nodes (the shell walk) and a
  body-centred cubic lattice of core points (spacing = the lattice cell)
  are in the point set; after meshing, every edge straddling the distance-
  field iso-surface b = -t is split where it crosses (or an interior node
  near the crossing is snapped onto it), so the skin/core interface is a
  conforming surface on the iso-surface and skin + core = the body's
  volume exactly.
* **Degradation.** Where no splitting rule conforms, segments are abandoned
  under a Steiner budget, their triangles left unrecovered, the interior
  classified by the distance field instead of parity, and the export
  reports how many faces were abandoned. The P-38's wing, tails, both
  fuselages, the nacelle and the spinner conform exactly; the propeller
  blades still degrade (volumes match to 1e-7): a blade is thin, twisted
  and tapered, so consecutive rings shear against each other and the strip
  edges between matched parameters are not locally Delaunay within their
  sheet while the opposite sheet is nearer than their sag. A flip pass over
  the loft and a thickness-driven station refinement were both tried and
  rejected (the first breaks the boom's mirror-symmetric knife edge, the
  second triples the point counts without curing the blade); the open
  direction is a surface graded to the local thickness along the span.
* **Quality (2026-10-12).** Two passes, both keeping the surface, the
  skin/core interface and the material volumes exactly as they are:
  Delaunay refinement of the interior (`ConformingOptions::
  refine_radius_edge`, 2 in the export): tetrahedra whose circumradius
  exceeds twice their shortest edge are split at their circumcentre; one
  whose circumcentre falls outside the body splits the surface face its
  circumsphere escapes through instead (a point on the face -- still the
  input's sub-triangles), after cushioning a near-flat tetrahedron on
  either side of that face with a point half a face size away (exterior
  cushions are dropped with the exterior). Then `ImproveTetMesh`: 2-3 and
  3-2 flips that never cross a constrained or interface face, and
  optimisation smoothing of the free nodes toward the centroid of their
  neighbours, accepted only where the worst dihedral angle of their
  tetrahedra improves. On the P-38's port wing (skin 0.029) the pass
  removes about a quarter of the tetrahedra under 15 degrees and half the
  slivers under 5. The minimum dihedral angle stays near zero: the skin is
  one tetrahedron thick under surface triangles ten times wider than it,
  and a sliver whose four nodes all lie on the surface or the interface
  can neither be flipped nor moved. The way forward there is surface
  sizing (triangles a few skin thicknesses wide), not more passes.
* **The tetrahedral mesher is gmsh (2026-10-15).** The application no
  longer tetrahedralizes with `core/src/tet.cpp`. *Export Skin Tetrahedra*
  and *Export Tetrahedral Mesh* now go through `tools/tet_gmsh.py`
  (`Entity/AerotelicTetGmsh.cpp`, a separate process in the WingDesigner
  environment where gmsh is pip-installed, the same bridge pattern as the
  MeshLib lattice backend). The body's surface and MeshLib's offset go in
  as gmsh DISCRETE entities, one per closed shell, and the two volumes are
  bounded by them: the skin is the body's loop with every pocket as a
  hole, each pocket is a core volume. gmsh therefore adds interior nodes
  ONLY and never re-triangulates a boundary, which is what the old mesher
  could not guarantee. The outer faces of the skin tetrahedra are the
  body's own triangles, with no Steiner points on them at all; the
  skin/core interface is the offset's own triangles; and skin + core =
  the body's enclosed volume to rounding. Measured on the case that
  defeated the old mesher, p38a's 13-operand airframe fuse: exact to
  3.5e-15 in 15 s, 609k tetrahedra, no inverted or flat elements, where
  the old mesher abandoned 295,157 of about 414,580 constraint faces and
  was still cascading after 11 minutes. The sphere of the document test
  meshes in 0.16 s, exact to 2e-16, worst element quality 0.284.
  Settings that came out of measuring, not guessing: gmsh's Delaunay
  algorithm beat HXT on both time and quality on every body tried; the
  core's target edge length is the lattice cell, since the core carries
  the lattice's material card rather than its geometry; and the MeshLib
  offset is remeshed at TWO skin thicknesses, because the raw
  marching-cubes offset and a three-skin remesh both touch the body or
  cross themselves and are refused ("a segment and a facet intersect"),
  while one skin costs 350 s against 15 s for the same exact answer. The
  lattice job's usability check now also tests the offset for
  self-intersection, so a surface gmsh would refuse is reported by name
  first. Two workarounds that existed only for the old mesher are gone:
  the hollow is no longer opened by a skin thickness (that cost up to 94%
  of the hollow on a thick-skinned wing), and the mesh is no longer
  refined to a radius-edge bound. `core/src/tet.cpp` and its tests stay as
  they are, no longer on the application's path.
* **Nested surfaces: the skin's inner boundary is MeshLib's offset
  (2026-10-14, SUPERSEDED the next day by the gmsh route above).**
  `TetrahedralizeNested(outer, inner)` /
  `BuildNestedSkinCoreMesh` take the body's own triangulation and a closed
  surface strictly inside it as ONE constraint set (the inner inverted, so
  the pair is a hollow body for steps 1-3), conform to both exactly, and
  classify by crossing depth instead of parity: depth one is the skin
  (material 1), depth two the core (material 2). The skin/core interface
  is therefore the inner surface's own (sub-)triangles, and skin + core
  tile the body: skin + core = the outer's volume, core = the inner's, to
  rounding (`tet_check`: a sphere in a sphere, a box in a box, two pockets
  in a box, an all-skin body, an inner poking out refused). The inner
  surface the application uses is the lattice job's inward offset
  (`tools/lattice_meshlib.py` steps 1 and 4), which at the time was
  opened by a skin thickness and remeshed at one skin (both settings
  changed when gmsh took over; `inner_opening` is now zero and
  `inner_remesh_edge` two skins). It is remeshed by MeshLib to
  isotropic triangles (`inner_remesh_edge`),
  checked closed, outward and clear of the body (MeshLib's triangle
  collision test) and returned as `inner_<i>.tri`;
  `FAerotelicLatticeModel` keeps each body's surface and offset from the
  generation (`GetTetBodies`) and meshes them on first request
  (`GetTetMeshes`: core points at the lattice cell, radius-edge
  refinement 2, improvement; a mesh that comes out invalid is left out
  with a note). Both *Export Skin Tetrahedra (VTK)* and
  *Export Tetrahedral Mesh (VTK)* now write that one mesh (material 1 only,
  or both), and the status line prints skin + core against the bodies'
  volume with the worst relative mismatch, so the conformity can be read
  off; a message lists bodies left out (an offset that touches its body)
  or faces not conformed exactly. `tet_check nested <outer.tri>
  <inner.tri> [cell] [out.vtk] [radius-edge]` runs the nested mesher on
  two application .tri files; `write-wing` / `write-bodies` write the
  P-38's bodies as double-precision .tri fixtures.
  Two pitfalls found on the way, both in the mesher's segment recovery,
  not in the nesting: (1) any surface that went through float32 (an STL,
  or MeshLib's own output) loses the planar-cap detection (1e-9 of the
  diagonal), so a loft's ear-clipped caps become needle constraints and
  recovery splits them to the length floor -- the outer surface must be
  the app's double-precision loft (it is), only the inner may come from
  MeshLib; (2) an inner surface whose two sheets are closer than about
  twice its triangles (the offset of a wing near the trailing edge, a thin
  cowl wall) or that has pole fans splits without end too -- hence the
  opening (the P-38 nacelle conformed exactly opened by one skin and
  cascaded at half a skin) and the isotropic remesh -- and a segment lost
  during face recovery is now recovered again rather than aborting the
  mesh. Restricting the snap rule to adjacent nodes and casting twins on
  the combined surface were both tried and rejected: they cured nothing
  and made the Boom cascade. With the final settings every P-38 body a
  lattice fills conforms exactly in nested mode except the propeller
  blades, which degrade as they do in plain mode.

## Potential flow on Kratos (2026-10-15)

**Analysis -> Potential Flow (Kratos)...** computes the potential flow
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
  The same pattern as the lattice's MeshLib backend and the tetrahedral
  mesher; the process plumbing those three shared by copy is now one
  function, `AerotelicMeshLib::RunJob`.
* `tools/potential_flow.py` -- the fluid domain, the solve and the output.

**The fluid domain is the other mesh.** The structural route meshes the
inside of a body; this one meshes the air, so the domain is a far-field box
with the bodies as holes. The box is triangulated at the far-field element
size and each body goes in as a gmsh DISCRETE surface, exactly as
`tet_gmsh.py` does it, so gmsh adds interior nodes only: **the wetted
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
the structural export: the tetrahedra of the air, with
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

**Analysis -> DUST (Vortex Particle Method)...** writes a complete
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
* Not yet carried over from the Qt version: Show Intermediate Features,
  the STL dialog's mesh preview in the viewport, and
  the wing Control Surfaces tab (the C++ core has no control-surface model
  yet; the tab is a placeholder note). A section plane cuts without
  capping (see above), and the engine cannot rasterise SVG reference
  images.
* Lattices: the paper gives pictures, not node tables, for its strut
  cells; G7 (BCC with a vertical pillar), Vintiles (the cuboctahedral
  frame on the edge midpoints) and Re-entrant (vertical edges bowed a
  quarter cell inward) are reconstructions in `lattice.cpp`'s tables and
  are plain data to edit. The honeycomb family is not in the paper at all
  and claims no authority from it. Its cells are axis-aligned: the prism
  axis picks x, y or z, not an arbitrary direction, and the tiling is not
  rotated within its plane, so a honeycomb that should follow a swept spar
  has to be approximated by the axis nearest it. A thin wall on a large
  part is expensive, the sampling having to resolve the wall everywhere
  (a 0.6 mm wall across a 600 mm part is a 1000:1 span); the grid cap and
  the resolution warning both bite well before anything silently degrades.
  A propeller blade's mesh is trimmed at the hub
  by its span range rather than a box cut (exact without rake). A nacelle
  with chevrons still goes through OCCT. The lattice dialog is modal and
  generation runs on the game thread, so the window does not repaint until
  it is done.
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
