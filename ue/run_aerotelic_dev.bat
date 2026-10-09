@echo off
rem Developer launcher: runs the standalone Aerotelic application WITHOUT
rem packaging, by hosting the game in the (already built) editor binaries in
rem "-game" mode. No Editor UI appears -- just the application window -- but
rem it reads the uncooked project/engine content directly, so it is the quick
rem way to iterate after build_editor.bat. End users should use the packaged
rem build (package_standalone.bat -> run_aerotelic.bat).
rem
rem Optional first argument: a project .yaml to open at startup, e.g.
rem   run_aerotelic_dev.bat C:\Users\Matt\devel_1\aerotelic_python\projects\p38.yaml
setlocal
set UE_ROOT=C:\Program Files\Epic Games\UE_5.8
set PROJECT_ARG=
if not "%~1"=="" set PROJECT_ARG=-AerotelicProject="%~1"
start "" "%UE_ROOT%\Engine\Binaries\Win64\UnrealEditor.exe" "%~dp0AerotelicUE.uproject" -game -windowed -ResX=1600 -ResY=900 %PROJECT_ARG%
endlocal
