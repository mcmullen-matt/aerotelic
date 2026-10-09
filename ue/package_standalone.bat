@echo off
rem Cooks and packages the standalone Aerotelic application into
rem   Packaged\Windows\AerotelicUE.exe   (self-contained; needs no Editor to run)
rem
rem Takes a while the first time (it compiles the engine's shaders for the
rem packaged target). Prerequisites: UE 5.8 at the path below, and the
rem ..\core (and its yaml-cpp submodule) Release libraries already built.
setlocal
set UE_ROOT=C:\Program Files\Epic Games\UE_5.8
call "%UE_ROOT%\Engine\Build\BatchFiles\RunUAT.bat" BuildCookRun ^
	-project="%~dp0AerotelicUE.uproject" ^
	-platform=Win64 -clientconfig=Development ^
	-build -cook -stage -pak -archive ^
	-archivedirectory="%~dp0Packaged" ^
	-nop4 -utf8output
endlocal
