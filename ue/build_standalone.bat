@echo off
rem Builds the standalone Aerotelic application (Binaries\Win64\AerotelicUE.exe).
rem Prerequisites: UE 5.8 installed at the path below; ..\core and its
rem yaml-cpp submodule already built in Release (see core\CMakeLists.txt).
setlocal
set UE_ROOT=C:\Program Files\Epic Games\UE_5.8
call "%UE_ROOT%\Engine\Build\BatchFiles\Build.bat" AerotelicUE Win64 Development -project="%~dp0AerotelicUE.uproject" -WaitMutex
endlocal
