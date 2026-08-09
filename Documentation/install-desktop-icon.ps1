# install-desktop-icon.ps1
# Put an "Arm Console" icon on the desktop that starts the arm console.
#
# Run it once. Run it again any time the repository moves, the shortcut is
# deleted, or the icon goes stale - it overwrites rather than duplicating.
#
#   powershell -NoProfile -ExecutionPolicy Bypass -File Documentation\install-desktop-icon.ps1
#
# The shortcut points at START ARM GUI.bat, which decides for itself whether the
# Arduino is on this laptop or on the Pi and takes the right route. There is
# deliberately ONE icon: a second one labelled "...(Pi)" would be a question the
# operator has to answer every time, and the USB cable already answers it.
#
# Nothing here touches the arm.

$ErrorActionPreference = 'Stop'

$root     = Split-Path -Parent $PSScriptRoot
$target   = Join-Path $root 'START ARM GUI.bat'
$icon     = Join-Path $PSScriptRoot 'assets\arm-console.ico'
$desktop  = [Environment]::GetFolderPath('Desktop')
$linkPath = Join-Path $desktop 'Arm Console.lnk'

foreach ($p in @($target, $icon)) {
    if (-not (Test-Path $p)) {
        Write-Host ("PROBLEM: missing {0}" -f $p) -ForegroundColor Red
        Write-Host 'Nothing was changed.' -ForegroundColor Red
        exit 1
    }
}

$shell = New-Object -ComObject WScript.Shell
$lnk   = $shell.CreateShortcut($linkPath)
$lnk.TargetPath       = $target
$lnk.WorkingDirectory = $root
$lnk.IconLocation     = "$icon,0"
$lnk.Description      = 'Start the FactoryLM arm console. Leave the black window open while you use it.'
$lnk.WindowStyle      = 1          # normal window - it must be visible, it holds the connection
$lnk.Save()

Write-Host ''
Write-Host ('  Desktop icon written: {0}' -f $linkPath) -ForegroundColor Green
Write-Host ('  It runs: {0}' -f $target) -ForegroundColor Gray
Write-Host ''
Write-Host '  Double-click it. A black window opens and stays open - that window' -ForegroundColor Gray
Write-Host '  is the connection. The browser opens by itself a few seconds later.' -ForegroundColor Gray
Write-Host ''
exit 0
