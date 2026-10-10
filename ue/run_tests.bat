@echo off
rem Headless regression checks of the document layer (add every entity type,
rem save, reopen, round-trip placements) and of loading a real Python-saved
rem project. Requires build_editor.bat first. Results are printed as
rem "RESULT: N passed, M failed" lines.
setlocal
set UE_ROOT=C:\Program Files\Epic Games\UE_5.8
set CMD="%UE_ROOT%\Engine\Binaries\Win64\UnrealEditor-Cmd.exe"
%CMD% "%~dp0AerotelicUE.uproject" -run=AerotelicDocumentTest -unattended -nullrhi -nosplash -stdout -FullStdOutLogOutput | findstr /C:"RESULT" /C:"FAIL"
%CMD% "%~dp0AerotelicUE.uproject" -run=AerotelicRealProjectTest -unattended -nullrhi -nosplash -stdout -FullStdOutLogOutput | findstr /C:"RESULT" /C:"FAIL"
%CMD% "%~dp0AerotelicUE.uproject" -run=AerotelicLicenceTest -unattended -nullrhi -nosplash -stdout -FullStdOutLogOutput | findstr /C:"RESULT" /C:"FAIL"
endlocal
