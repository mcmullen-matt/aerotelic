@echo off
rem Runs the application from the SNAPSHOT copy of the runtime files
rem (..\..\aerotelic_snapshot\ue, made by refresh_snapshot.bat), so that it
rem can stay open for testing while the main tree is being built: the build
rem links UnrealEditor-AerotelicUE.dll in THIS tree's Binaries, which a
rem session run from here would hold open (LNK1104).
rem
rem -LiveCoding=0 matters: Live Coding holds a mutex named after the engine's
rem UnrealEditor.exe, and UnrealBuildTool refuses to build ANY editor target
rem of this engine while that mutex exists ("Unable to build while Live
rem Coding is active"), whichever project the session was started from.
rem
rem Optional first argument: a project .yaml to open at startup, e.g.
rem   run_aerotelic_snapshot.bat C:\Users\Matt\devel_1\aerotelic\ue\Projects\p38.yaml
setlocal
set UE_ROOT=C:\Program Files\Epic Games\UE_5.8
set SNAPSHOT=%~dp0..\..\aerotelic_snapshot\ue
if not exist "%SNAPSHOT%\AerotelicUE.uproject" (
	echo No snapshot at %SNAPSHOT% -- run refresh_snapshot.bat first.
	exit /b 1
)
set PROJECT_ARG=
if not "%~1"=="" set PROJECT_ARG=-AerotelicProject="%~1"
start "" "%UE_ROOT%\Engine\Binaries\Win64\UnrealEditor.exe" "%SNAPSHOT%\AerotelicUE.uproject" -game -windowed -ResX=2560 -ResY=1440 -LiveCoding=0 %PROJECT_ARG%
endlocal
