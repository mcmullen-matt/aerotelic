# Aerotelic (C++ / Unreal Engine)

Parametric aircraft-geometry CAD: the C++ port of the Python/Qt application
WingDesigner (which lives in its own repository, checked out as a sibling
directory `WingDesigner`; its `projects/p38.yaml` is the sample project the
tests load). Two parts, kept in one repository because they change together:

| Directory | What it is |
|---|---|
| `core/` | The geometry backend: NURBS, airfoils, wing / fuselage / nacelle / propeller / primitive models, feature tree, project-file (YAML) I/O. Open CASCADE Technology 7.9.3 + yaml-cpp, built with CMake into a static library plus fixture-check executables. |
| `ue/` | The application: Unreal Engine 5.8 standalone game target that hosts the UI (menu bar, toolbars, scene tree, dockable design panels) and the 3D view, linking `core/`. See `ue/README.md` for the launchers, build scripts and tests. |

## Getting the source

yaml-cpp is a git submodule (pinned to upstream tag 0.8.0), so clone with it:

    git clone --recurse-submodules <this repo>

or, after a plain clone, `git submodule update --init`.

## Building

1. OCCT 7.9.3 is taken from the micromamba environment used by the Python
   app (`C:\Users\Matt\micromamba\envs\aerotelic\Library`); the path is set
   in `core/CMakeLists.txt` and `ue/Source/AerotelicUE/AerotelicUE.Build.cs`.
2. `core/`: configure and build with CMake (Release), which also builds the
   vendored yaml-cpp. The UE project links `core/build/Release/aerotelic_core.lib`.
3. `ue/`: `build_editor.bat` (developer launcher + tests) or
   `package_standalone.bat` (self-contained `Packaged\Windows\AerotelicUE.exe`).

Build output (`core/build`, `ue/Binaries`, `ue/Intermediate`, `ue/Saved`,
`ue/Packaged`, ...) is not tracked.
