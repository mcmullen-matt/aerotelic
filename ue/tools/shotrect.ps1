# Captures an explicit screen rectangle to a PNG. screenshot.ps1 captures a
# process's MAIN window, which follows the foreground/first window -- once a
# modeless dialog is open that is the dialog, not the viewport. Use this when
# you need one particular rectangle (e.g. the app window while a dialog of the
# same process is open, or a window on a second monitor).
#
#   powershell -File shotrect.ps1 -Out shot.png -X 3040 -Y 246 -W 1600 -H 900
param(
	[Parameter(Mandatory = $true)] [string] $Out,
	[Parameter(Mandatory = $true)] [int] $X,
	[Parameter(Mandatory = $true)] [int] $Y,
	[Parameter(Mandatory = $true)] [int] $W,
	[Parameter(Mandatory = $true)] [int] $H
)
Add-Type -AssemblyName System.Drawing
$bmp = New-Object System.Drawing.Bitmap($W, $H)
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.CopyFromScreen((New-Object System.Drawing.Point($X, $Y)), [System.Drawing.Point]::Empty, (New-Object System.Drawing.Size($W, $H)))
$bmp.Save($Out, [System.Drawing.Imaging.ImageFormat]::Png)
$g.Dispose(); $bmp.Dispose()
Write-Output "saved $Out (${W}x${H} at $X,$Y)"
