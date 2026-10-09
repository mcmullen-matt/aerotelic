@echo off
rem Launches the standalone Aerotelic application.
rem   1. Prefers the packaged, self-contained build (package_standalone.bat).
rem   2. Otherwise falls back to the developer launcher (editor binaries
rem      hosting the app in -game mode; needs build_editor.bat first).
rem
rem Optional first argument: a project .yaml to open at startup, e.g.
rem   run_aerotelic.bat C:\Users\Matt\devel_1\aerotelic_python\projects\p38.yaml
setlocal
set PROJECT_ARG=
if not "%~1"=="" set PROJECT_ARG=-AerotelicProject="%~1"
if exist "%~dp0Packaged\Windows\AerotelicUE.exe" (
	start "" "%~dp0Packaged\Windows\AerotelicUE.exe" -windowed -ResX=1600 -ResY=900 %PROJECT_ARG%
	goto :eof
)
if exist "%~dp0Packaged\Win64\AerotelicUE.exe" (
	start "" "%~dp0Packaged\Win64\AerotelicUE.exe" -windowed -ResX=1600 -ResY=900 %PROJECT_ARG%
	goto :eof
)
echo No packaged build found in Packaged\ -- falling back to the developer launcher.
call "%~dp0run_aerotelic_dev.bat" %1
endlocal
