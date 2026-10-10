@echo off
rem Creates Content\Materials\M_AerotelicSurface.uasset, the surface material
rem the bodies are drawn with (Color + Metallic / Specular / Roughness
rem parameters, driven by View -> Surface...). Needs build_editor.bat first.
rem Run once after cloning (the asset is committed, so normally not needed)
rem and again whenever AerotelicMaterialSetupCommandlet changes.
setlocal
set UE_ROOT=C:\Program Files\Epic Games\UE_5.8
"%UE_ROOT%\Engine\Binaries\Win64\UnrealEditor-Cmd.exe" "%~dp0AerotelicUE.uproject" -run=AerotelicMaterialSetup -unattended -nosplash -stdout -FullStdOutLogOutput | findstr /C:"RESULT" /C:"Error:"
endlocal
