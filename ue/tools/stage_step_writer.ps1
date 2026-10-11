# Copies aerotelic_step_writer.exe (built by the core's CMake) and every
# DLL it needs, found by following dumpbin /dependents through the OCCT
# install, into ue\Binaries\Win64\StepWriter\. The application runs the
# writer from there (File -> Export STEP) and the packaged build stages the
# folder (see AerotelicUE.Build.cs). Kept out of Binaries\Win64 itself: the
# STEP chain's zlib/libpng/freetype/tiff must never be found by the engine.
#
#   powershell -File stage_step_writer.ps1
param(
	[string] $Exe = (Join-Path $PSScriptRoot "..\..\core\build\Release\aerotelic_step_writer.exe"),
	[string] $OcctBin = "C:\Users\Matt\micromamba\envs\aerotelic\Library\bin",
	[string] $Out = (Join-Path $PSScriptRoot "..\Binaries\Win64\StepWriter")
)
$ErrorActionPreference = "Stop"
$dumpbin = Get-ChildItem "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Tools\MSVC\*\bin\Hostx64\x64\dumpbin.exe" | Select-Object -First 1 -ExpandProperty FullName
if (-not (Test-Path $Exe)) { throw "writer not built: $Exe (cmake --build core\build --config Release --target aerotelic_step_writer)" }

function Dependents($path) {
	& $dumpbin /nologo /dependents $path | Where-Object { $_ -match '^\s+\S+\.dll$' } | ForEach-Object { $_.Trim() }
}

$seen = @{}
$queue = New-Object System.Collections.Generic.Queue[string]
$queue.Enqueue($Exe)
$files = @($Exe)
while ($queue.Count -gt 0) {
	$cur = $queue.Dequeue()
	foreach ($dep in Dependents $cur) {
		$key = $dep.ToLower()
		if ($seen.ContainsKey($key)) { continue }
		$seen[$key] = $true
		$candidate = Join-Path $OcctBin $dep
		# Only what lives in the OCCT install travels; system DLLs stay where they are.
		if (Test-Path $candidate) {
			$files += $candidate
			$queue.Enqueue($candidate)
		}
	}
}

New-Item -ItemType Directory -Force $Out | Out-Null
foreach ($f in $files) { Copy-Item $f -Destination $Out -Force }
Write-Output ("staged {0} files to {1}" -f $files.Count, (Resolve-Path $Out))
$files | ForEach-Object { Write-Output ("  " + (Split-Path $_ -Leaf)) }
