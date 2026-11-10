@echo off
rem Copies the files the application needs at run time into the SNAPSHOT
rem folder ..\..\aerotelic_snapshot\ue (outside the repository), for
rem run_aerotelic_snapshot.bat: the editor-hosted binaries (not the UnrealGame
rem AerotelicUE.exe or Live Coding patches), Config, Resources, tools and the
rem .uproject. Content is empty (the UI is Slate), Source and Intermediate are
rem not needed to run, and Projects (gigabytes of results) is opened by path
rem from wherever it lives. Run it after build_editor.bat, with the snapshot
rem application CLOSED (its DLL would be locked); the copy takes seconds.
setlocal
set SRC=%~dp0
set DST=%~dp0..\..\aerotelic_snapshot\ue
if not exist "%SRC%Binaries\Win64\UnrealEditor-AerotelicUE.dll" (
	echo No editor binaries in %SRC%Binaries\Win64 -- run build_editor.bat first.
	exit /b 1
)
if not exist "%DST%" mkdir "%DST%"
robocopy "%SRC%Binaries\Win64" "%DST%\Binaries\Win64" /E /XF AerotelicUE.exe AerotelicUE.pdb AerotelicUE.lib AerotelicUE.exp AerotelicUE.target UnrealEditor-AerotelicUE.patch_*.* /NFL /NDL /NJH /NP
if errorlevel 8 (
	echo The binaries could not all be copied -- is the snapshot application still running?
	exit /b 1
)
robocopy "%SRC%Config" "%DST%\Config" /E /NFL /NDL /NJH /NJS /NP
robocopy "%SRC%Resources" "%DST%\Resources" /E /NFL /NDL /NJH /NJS /NP
robocopy "%SRC%tools" "%DST%\tools" /E /XD __pycache__ /NFL /NDL /NJH /NJS /NP
copy /Y "%SRC%AerotelicUE.uproject" "%DST%\AerotelicUE.uproject" >nul
echo Snapshot refreshed at %DST%
exit /b 0
