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

Each step prints the command it runs; fix and rerun (every step is
idempotent). First run for real 2026-11-18. Never `pip install tacs` on Windows: PyPI's "tacs" is an
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

function ConvertTo-WslPath([string]$WindowsPath) {
    # C:\a\b -> /mnt/c/a/b (the default automount). Not wslpath: its output
    # is UTF-8 while wsl.exe's own messages are UTF-16, so one encoding
    # cannot read both.
    $full = [System.IO.Path]::GetFullPath($WindowsPath)
    return "/mnt/" + $full.Substring(0, 1).ToLower() + $full.Substring(2).Replace('\', '/')
}

function Invoke-InDistro([string]$Shell) {
    # The shell text goes through a file: double quotes inside one wsl.exe
    # argument do not survive the Windows command line (sh -c "..." lost
    # every quote and $(...) on the first run). LF endings, no BOM.
    $file = Join-Path $env:TEMP "aerotelic_tacs_setup.sh"
    $text = ($Shell -replace "`r`n", "`n") + "`n"
    [System.IO.File]::WriteAllText($file, $text, (New-Object System.Text.UTF8Encoding($false)))
    $linux = ConvertTo-WslPath $file
    Write-Host "   wsl -d $Distro -e sh $linux" -ForegroundColor DarkGray
    Write-Host $Shell -ForegroundColor DarkGray
    & wsl.exe -d $Distro -e sh $linux
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
if ! command -v curl >/dev/null 2>&1; then
  if [ "$(id -u)" = "0" ]; then apt-get update && apt-get install -y curl; else sudo apt-get update && sudo apt-get install -y curl; fi
fi
if [ ! -x "$HOME/bin/micromamba" ]; then
  # The static binary from the releases: the micro.mamba.pm tarball needs
  # bzip2, which the stock Ubuntu WSL image lacks (and sudo cannot prompt here).
  mkdir -p "$HOME/bin"
  curl -Ls -o "$HOME/bin/micromamba" https://github.com/mamba-org/micromamba-releases/releases/latest/download/micromamba-linux-64
  chmod +x "$HOME/bin/micromamba"
fi
"$HOME/bin/micromamba" --version
if [ ! -x "$HOME/micromamba/envs/tacs/bin/python" ]; then
  "$HOME/bin/micromamba" create -y -r "$HOME/micromamba" -n tacs -c conda-forge -c smdogroup python=3.11 tacs
fi
PY="$HOME/micromamba/envs/tacs/bin/python"
# conda-forge's python-gmsh, not the pip wheel: the wheel dlopens the system
# libGLU.so.1 and friends, which the stock WSL image lacks; conda-forge ships them.
"$PY" -c "import gmsh" 2>/dev/null || "$HOME/bin/micromamba" install -y -r "$HOME/micromamba" -n tacs -c conda-forge python-gmsh
"$PY" -c "import sys, tacs, mpi4py, numpy, pyNastran, importlib.metadata as m; print('python', sys.version.split()[0]); print('tacs', m.version('tacs')); print('pyNastran', pyNastran.__version__); print('numpy', numpy.__version__)"
echo "TACS python: $PY"
'@
Invoke-InDistro $linux

if ($SkipCheck) { Write-Host ""; Write-Host "Done (check skipped)."; exit 0 }

Step "structural_fea_check.py --backend tacs"
$toolsLinux = ConvertTo-WslPath $PSScriptRoot
Invoke-InDistro ('cd "{0}" && "$HOME/micromamba/envs/tacs/bin/python" -u structural_fea_check.py --backend tacs' -f $toolsLinux)

Write-Host ""
Write-Host "Done. The application finds ~/micromamba/envs/tacs/bin/python in the default distribution by itself;" -ForegroundColor Green
Write-Host "if $Distro is not the default (wsl --set-default $Distro), set AEROTELIC_TACS_PYTHON=wsl:${Distro}:<that python>." -ForegroundColor Green
Write-Host "Then Analysis > TACS (FEA) > Check Installation... in Aerotelic." -ForegroundColor Green
