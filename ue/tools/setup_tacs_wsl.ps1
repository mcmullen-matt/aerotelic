<#
setup_tacs_wsl.ps1 -- the TACS runtime for Analysis > TACS (FEA), in one go.

TACS (smdogroup.github.io/tacs) has no Windows build, so the application
runs tools/structural_fea_tacs.py inside WSL. This script, run AFTER the
reboot that `wsl --install` asks for:
  1. installs the WSL distribution if it is missing (--no-launch: no
     first-run user prompt; commands run as the distribution's default user),
  2. installs micromamba under ~ in the distribution and creates the
     environment ~/micromamba/envs/tacs (python 3.11 + tacs from the
     smdogroup channel), which is where the application looks for it
     (AerotelicMeshLib.cpp ProbeWslTacsPython) -- or set
     AEROTELIC_TACS_PYTHON=wsl:<distro>:<python> for another place,
  3. pip-installs gmsh there (only the check script's rib case needs it),
  4. runs tools/structural_fea_check.py --backend tacs through WSL.

    powershell -ExecutionPolicy Bypass -File ue\tools\setup_tacs_wsl.ps1 [-Distro Ubuntu-24.04] [-SkipCheck]

Written 2026-11-17 before the machine had rebooted, so it is UNTESTED end to
end: each step prints the command it runs; fix and rerun (every step is
idempotent). Never `pip install tacs` on Windows: PyPI's "tacs" is an
unrelated TOML library.
#>
param(
    [string]$Distro = "Ubuntu-24.04",
    [switch]$SkipCheck
)

$ErrorActionPreference = "Stop"

function Step($Message) { Write-Host ""; Write-Host "== $Message" -ForegroundColor Cyan }

function Invoke-Wsl([string[]]$Arguments) {
    # wsl.exe prints UTF-16; read it as such or every other character is a NUL.
    $previous = [Console]::OutputEncoding
    [Console]::OutputEncoding = [System.Text.Encoding]::Unicode
    try { $out = & wsl.exe @Arguments 2>&1 | ForEach-Object { "$_" } } finally { [Console]::OutputEncoding = $previous }
    return $out
}

function Invoke-InDistro([string]$Shell) {
    # One sh -c command in the distribution, output streamed (UTF-8).
    Write-Host "   wsl -d $Distro -e sh -c '...'" -ForegroundColor DarkGray
    Write-Host $Shell -ForegroundColor DarkGray
    & wsl.exe -d $Distro -e sh -c $Shell
    if ($LASTEXITCODE -ne 0) { throw "the command in $Distro failed with exit code $LASTEXITCODE" }
}

Step "WSL state"
$status = Invoke-Wsl @("--status")
$status | ForEach-Object { Write-Host "   $_" }
if (($status -join " ") -match "virtualization is not enabled|Virtual Machine Platform") {
    Write-Host ""
    Write-Host "WSL 2 cannot start yet. If 'wsl --install' was just run, REBOOT first; if the machine has rebooted, enable" -ForegroundColor Yellow
    Write-Host "virtualization (Intel VT-x / AMD-V) in the firmware and the 'Virtual Machine Platform' Windows feature." -ForegroundColor Yellow
    exit 2
}

Step "Distributions"
$installed = Invoke-Wsl @("-l", "-q") | ForEach-Object { $_.Trim() } | Where-Object { $_ }
if ($installed) { $installed | ForEach-Object { Write-Host "   $_" } } else { Write-Host "   (none)" }
if ($installed -notcontains $Distro) {
    Step "Installing $Distro (no first-run launch)"
    & wsl.exe --install -d $Distro --no-launch
    if ($LASTEXITCODE -ne 0) { throw "wsl --install -d $Distro failed with exit code $LASTEXITCODE (a reboot may still be pending)" }
    $installed = Invoke-Wsl @("-l", "-q") | ForEach-Object { $_.Trim() } | Where-Object { $_ }
    if ($installed -notcontains $Distro) { throw "$Distro is not listed after the install; run 'wsl -l -v' and look" }
}

Step "Hello from $Distro"
Invoke-InDistro 'echo "user $(id -un), home $HOME, $(uname -sr)"'

Step "micromamba + the tacs environment (idempotent)"
$linux = @'
set -e
cd "$HOME"
if ! command -v bzip2 >/dev/null 2>&1 || ! command -v curl >/dev/null 2>&1; then
  if [ "$(id -u)" = "0" ]; then apt-get update && apt-get install -y curl bzip2; else sudo apt-get update && sudo apt-get install -y curl bzip2; fi
fi
if [ ! -x "$HOME/bin/micromamba" ]; then
  mkdir -p "$HOME/bin"
  curl -Ls https://micro.mamba.pm/api/micromamba/linux-64/latest | tar -xj -C "$HOME" bin/micromamba
fi
"$HOME/bin/micromamba" --version
if [ ! -x "$HOME/micromamba/envs/tacs/bin/python" ]; then
  "$HOME/bin/micromamba" create -y -r "$HOME/micromamba" -n tacs -c conda-forge -c smdogroup python=3.11 tacs
fi
PY="$HOME/micromamba/envs/tacs/bin/python"
"$PY" -c "import gmsh" 2>/dev/null || "$PY" -m pip install --quiet gmsh
"$PY" -c "import sys, tacs, mpi4py, numpy, pyNastran; print('python', sys.version.split()[0]); print('tacs', tacs.__version__); print('pyNastran', pyNastran.__version__); print('numpy', numpy.__version__)"
echo "TACS python: $PY"
'@
Invoke-InDistro $linux

if ($SkipCheck) { Write-Host ""; Write-Host "Done (check skipped)."; exit 0 }

Step "structural_fea_check.py --backend tacs"
$toolsWin = $PSScriptRoot
$toolsLinux = (Invoke-Wsl @("-d", $Distro, "-e", "wslpath", "-a", $toolsWin) | Select-Object -First 1).Trim()
if (-not $toolsLinux) { throw "wslpath could not map $toolsWin" }
Invoke-InDistro ('cd "{0}" && "$HOME/micromamba/envs/tacs/bin/python" -u structural_fea_check.py --backend tacs' -f $toolsLinux)

Write-Host ""
Write-Host "Done. The application finds ~/micromamba/envs/tacs/bin/python in the default distribution by itself;" -ForegroundColor Green
Write-Host "if $Distro is not the default (wsl --set-default $Distro), set AEROTELIC_TACS_PYTHON=wsl:${Distro}:<that python>." -ForegroundColor Green
Write-Host "Then Analysis > TACS (FEA) > Check Installation... in Aerotelic." -ForegroundColor Green
