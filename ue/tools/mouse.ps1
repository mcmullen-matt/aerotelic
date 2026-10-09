# Blind UI automation for the standalone app: move/click/drag the real mouse
# and list a process's visible top-level windows.
#
#   powershell -File mouse.ps1 -Action click -X 500 -Y 300
#   powershell -File mouse.ps1 -Action drag  -X 500 -Y 300 -X2 900 -Y2 400 [-Steps 40] [-HoldMs 120]
#   powershell -File mouse.ps1 -Action move  -X 500 -Y 300
#   powershell -File mouse.ps1 -Action windows -ProcessId 1234
#
# Coordinates are screen pixels. A drag presses, moves in -Steps increments
# with short pauses (Slate needs real intermediate motion to start a drag
# operation), then releases.
param(
	[Parameter(Mandatory = $true)] [ValidateSet("click", "rclick", "dclick", "drag", "move", "down", "up", "path", "windows", "close", "key")] [string] $Action,
	[string] $Keys = "",
	[int] $X = 0, [int] $Y = 0, [int] $X2 = 0, [int] $Y2 = 0,
	[int] $Steps = 40, [int] $HoldMs = 120,
	[int] $ProcessId = 0
)
Add-Type @"
using System;
using System.Text;
using System.Collections.Generic;
using System.Runtime.InteropServices;
public static class Win32Mouse {
	[DllImport("user32.dll")] public static extern bool SetCursorPos(int x, int y);
	[DllImport("user32.dll")] public static extern void mouse_event(uint flags, int dx, int dy, uint data, UIntPtr extra);
	public const uint LEFTDOWN = 0x0002, LEFTUP = 0x0004, MOVE = 0x0001, RIGHTDOWN = 0x0008, RIGHTUP = 0x0010;

	[StructLayout(LayoutKind.Sequential)] public struct RECT { public int Left, Top, Right, Bottom; }
	public delegate bool EnumProc(IntPtr hWnd, IntPtr lParam);
	[DllImport("user32.dll")] public static extern bool EnumWindows(EnumProc cb, IntPtr lParam);
	[DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr hWnd);
	[DllImport("user32.dll")] public static extern uint GetWindowThreadProcessId(IntPtr hWnd, out uint pid);
	[DllImport("user32.dll", CharSet = CharSet.Unicode)] public static extern int GetWindowText(IntPtr hWnd, StringBuilder sb, int max);
	[DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr hWnd, out RECT rect);
	[DllImport("user32.dll")] public static extern bool PostMessage(IntPtr hWnd, uint msg, IntPtr wParam, IntPtr lParam);
	public const uint WM_CLOSE = 0x0010;

	// Posts WM_CLOSE to the process's visible window whose title starts with
	// the prefix (the main app window, not a torn-off tab window).
	public static string CloseWindow(uint pid, string titlePrefix) {
		string closed = null;
		EnumWindows((h, l) => {
			uint p; GetWindowThreadProcessId(h, out p);
			if (p != pid || !IsWindowVisible(h)) return true;
			var sb = new StringBuilder(256); GetWindowText(h, sb, 256);
			if (!sb.ToString().StartsWith(titlePrefix)) return true;
			PostMessage(h, WM_CLOSE, IntPtr.Zero, IntPtr.Zero);
			closed = sb.ToString();
			return false;
		}, IntPtr.Zero);
		return closed;
	}

	public static List<string> ListWindows(uint pid) {
		var result = new List<string>();
		EnumWindows((h, l) => {
			uint p; GetWindowThreadProcessId(h, out p);
			if (p != pid || !IsWindowVisible(h)) return true;
			var sb = new StringBuilder(256); GetWindowText(h, sb, 256);
			RECT r; GetWindowRect(h, out r);
			result.Add(string.Format("{0}\t({1},{2})-({3},{4}) {5}x{6}", sb.ToString(), r.Left, r.Top, r.Right, r.Bottom, r.Right - r.Left, r.Bottom - r.Top));
			return true;
		}, IntPtr.Zero);
		return result;
	}
}
"@

switch ($Action) {
	"move" {
		[void][Win32Mouse]::SetCursorPos($X, $Y)
		Write-Output "moved to ($X,$Y)"
	}
	# down / path / up let a caller hold a drag open (e.g. to screenshot the
	# drop indicators) before releasing: down at X,Y; path moves X,Y -> X2,Y2
	# in steps with the button still held; up releases.
	"down" {
		[void][Win32Mouse]::SetCursorPos($X, $Y)
		Start-Sleep -Milliseconds 120
		[Win32Mouse]::mouse_event([Win32Mouse]::LEFTDOWN, 0, 0, 0, [UIntPtr]::Zero)
		Write-Output "button down at ($X,$Y)"
	}
	"path" {
		for ($i = 1; $i -le $Steps; $i++) {
			$cx = [int]($X + ($X2 - $X) * $i / $Steps)
			$cy = [int]($Y + ($Y2 - $Y) * $i / $Steps)
			[void][Win32Mouse]::SetCursorPos($cx, $cy)
			[Win32Mouse]::mouse_event([Win32Mouse]::MOVE, 0, 0, 0, [UIntPtr]::Zero)
			Start-Sleep -Milliseconds 16
		}
		Write-Output "moved (held) ($X,$Y) -> ($X2,$Y2)"
	}
	"up" {
		[Win32Mouse]::mouse_event([Win32Mouse]::LEFTUP, 0, 0, 0, [UIntPtr]::Zero)
		Write-Output "button up"
	}
	"click" {
		[void][Win32Mouse]::SetCursorPos($X, $Y)
		Start-Sleep -Milliseconds 80
		[Win32Mouse]::mouse_event([Win32Mouse]::LEFTDOWN, 0, 0, 0, [UIntPtr]::Zero)
		Start-Sleep -Milliseconds 60
		[Win32Mouse]::mouse_event([Win32Mouse]::LEFTUP, 0, 0, 0, [UIntPtr]::Zero)
		Write-Output "clicked ($X,$Y)"
	}
	"rclick" {
		[void][Win32Mouse]::SetCursorPos($X, $Y)
		Start-Sleep -Milliseconds 80
		[Win32Mouse]::mouse_event([Win32Mouse]::RIGHTDOWN, 0, 0, 0, [UIntPtr]::Zero)
		Start-Sleep -Milliseconds 60
		[Win32Mouse]::mouse_event([Win32Mouse]::RIGHTUP, 0, 0, 0, [UIntPtr]::Zero)
		Write-Output "right-clicked ($X,$Y)"
	}
	"dclick" {
		[void][Win32Mouse]::SetCursorPos($X, $Y)
		Start-Sleep -Milliseconds 80
		for ($i = 0; $i -lt 2; $i++) {
			[Win32Mouse]::mouse_event([Win32Mouse]::LEFTDOWN, 0, 0, 0, [UIntPtr]::Zero)
			Start-Sleep -Milliseconds 40
			[Win32Mouse]::mouse_event([Win32Mouse]::LEFTUP, 0, 0, 0, [UIntPtr]::Zero)
			Start-Sleep -Milliseconds 80
		}
		Write-Output "double-clicked ($X,$Y)"
	}
	"key" {
		# SendKeys syntax, e.g. "{ESC}", "^s", "{F5}". Goes to the foreground window.
		Add-Type -AssemblyName System.Windows.Forms
		[System.Windows.Forms.SendKeys]::SendWait($Keys)
		Write-Output "sent keys $Keys"
	}
	"drag" {
		[void][Win32Mouse]::SetCursorPos($X, $Y)
		Start-Sleep -Milliseconds 120
		[Win32Mouse]::mouse_event([Win32Mouse]::LEFTDOWN, 0, 0, 0, [UIntPtr]::Zero)
		Start-Sleep -Milliseconds $HoldMs
		for ($i = 1; $i -le $Steps; $i++) {
			$cx = [int]($X + ($X2 - $X) * $i / $Steps)
			$cy = [int]($Y + ($Y2 - $Y) * $i / $Steps)
			[void][Win32Mouse]::SetCursorPos($cx, $cy)
			[Win32Mouse]::mouse_event([Win32Mouse]::MOVE, 0, 0, 0, [UIntPtr]::Zero)
			Start-Sleep -Milliseconds 16
		}
		Start-Sleep -Milliseconds 200
		[Win32Mouse]::mouse_event([Win32Mouse]::LEFTUP, 0, 0, 0, [UIntPtr]::Zero)
		Write-Output "dragged ($X,$Y) -> ($X2,$Y2)"
	}
	"close" {
		# -X unused; title prefix fixed to the app's window title.
		if ($ProcessId -eq 0) { throw "-ProcessId required" }
		$c = [Win32Mouse]::CloseWindow([uint32]$ProcessId, "Aerotelic")
		if ($c) { Write-Output "posted WM_CLOSE to '$c'" } else { Write-Output "no window titled Aerotelic* for pid $ProcessId" }
	}
	"windows" {
		if ($ProcessId -eq 0) { throw "-ProcessId required" }
		$list = [Win32Mouse]::ListWindows([uint32]$ProcessId)
		Write-Output "$($list.Count) visible top-level window(s) for pid $ProcessId"
		$list | ForEach-Object { Write-Output "  $_" }
	}
}
