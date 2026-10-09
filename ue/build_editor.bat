@echo off
rem Builds the editor-hosted binaries (UnrealEditor-AerotelicUE.dll). Needed
rem for run_aerotelic_dev.bat, for the headless test commandlets
rem (run_tests.bat), and as the first step of package_standalone.bat.
setlocal
set UE_ROOT=C:\Program Files\Epic Games\UE_5.8
call "%UE_ROOT%\Engine\Build\BatchFiles\Build.bat" AerotelicUEEditor Win64 Development -project="%~dp0AerotelicUE.uproject" -WaitMutex
endlocal
