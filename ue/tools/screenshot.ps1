# Captures the primary screen (or, with -ProcessId, that process's main window
# rectangle) to a PNG. Used to eyeball the standalone app without a human at
# the keyboard. Usage: powershell -File screenshot.ps1 -Out shot.png [-ProcessId 1234]
param(
	[Parameter(Mandatory = $true)] [string] $Out,
	[int] $ProcessId = 0
)
Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName System.Windows.Forms
Add-Type @"
using System;
using System.Runtime.InteropServices;
public static class Win32Shot {
	[StructLayout(LayoutKind.Sequential)] public struct RECT { public int Left, Top, Right, Bottom; }
	[DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr hWnd, out RECT rect);
	[DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr hWnd);
}
"@
$bounds = [System.Windows.Forms.Screen]::PrimaryScreen.Bounds
if ($ProcessId -ne 0) {
	$proc = Get-Process -Id $ProcessId -ErrorAction Stop
	$h = $proc.MainWindowHandle
	if ($h -ne [IntPtr]::Zero) {
		[void][Win32Shot]::SetForegroundWindow($h)
		Start-Sleep -Milliseconds 400
		$r = New-Object Win32Shot+RECT
		if ([Win32Shot]::GetWindowRect($h, [ref]$r)) {
			$bounds = New-Object System.Drawing.Rectangle($r.Left, $r.Top, ($r.Right - $r.Left), ($r.Bottom - $r.Top))
		}
	}
}
$bmp = New-Object System.Drawing.Bitmap($bounds.Width, $bounds.Height)
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.CopyFromScreen($bounds.Location, [System.Drawing.Point]::Empty, $bounds.Size)
$bmp.Save($Out, [System.Drawing.Imaging.ImageFormat]::Png)
$g.Dispose(); $bmp.Dispose()
Write-Output "saved $Out ($($bounds.Width)x$($bounds.Height))"
