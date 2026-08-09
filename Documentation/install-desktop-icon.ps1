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

# NAMED TO SIT NEXT TO ITS SIBLING. The desktop already has
# "Robot Arm - Check Setup", so "Robot Arm - Console" sorts beside it instead of
# landing under A among everything else. On a desktop with 77 icons, where a
# shortcut SORTS is most of whether it can be found.
$LINKNAME = 'Robot Arm - Console.lnk'
$OLDNAMES = @('Arm Console.lnk')       # earlier name, cleaned up so there is only ever one

foreach ($p in @($target, $icon)) {
    if (-not (Test-Path $p)) {
        Write-Host ("PROBLEM: missing {0}" -f $p) -ForegroundColor Red
        Write-Host 'Nothing was changed.' -ForegroundColor Red
        exit 1
    }
}

function Write-Link($path) {
    $shell = New-Object -ComObject WScript.Shell
    $lnk   = $shell.CreateShortcut($path)
    $lnk.TargetPath       = $target
    $lnk.WorkingDirectory = $root
    $lnk.IconLocation     = "$icon,0"
    $lnk.Description      = 'Start the FactoryLM arm console. Leave the black window open while you use it.'
    $lnk.WindowStyle      = 1      # normal window - it must be visible, it holds the connection
    $lnk.Save()
}

# TWO PLACES, because one icon among many is not findable and the Start menu is.
#   Desktop    - what you look for when you already know it is there
#   Start menu - what actually works: press Windows, type "arm", press Enter
$targets = @{}
$desktop = [Environment]::GetFolderPath('Desktop')
$start   = Join-Path $env:APPDATA 'Microsoft\Windows\Start Menu\Programs'
$targets['Desktop']    = Join-Path $desktop $LINKNAME
$targets['Start menu'] = Join-Path $start   $LINKNAME

# ONEDRIVE SPLITS THE DESKTOP IN TWO. When folder backup is on, the desktop you
# SEE can be %USERPROFILE%\OneDrive\Desktop while GetFolderPath still answers
# %USERPROFILE%\Desktop, so a shortcut written to the reported path lands in a
# folder nobody is looking at. Write to the other one too when it exists -
# a duplicate icon is a far smaller problem than an invisible one.
$alt = Join-Path $env:USERPROFILE 'OneDrive\Desktop'
if ((Test-Path $alt) -and ($alt -ne $desktop)) { $targets['OneDrive desktop'] = Join-Path $alt $LINKNAME }

Write-Host ''
foreach ($k in $targets.Keys | Sort-Object) {
    $p = $targets[$k]
    $dir = Split-Path -Parent $p
    foreach ($old in $OLDNAMES) {
        $o = Join-Path $dir $old
        if (Test-Path $o) { Remove-Item $o -Force -ErrorAction SilentlyContinue
                            Write-Host ("  removed the old {0} shortcut in {1}" -f $old, $k) -ForegroundColor Yellow }
    }
    Write-Link $p
    if (Test-Path $p) { Write-Host ("  {0,-16} {1}" -f ($k + ':'), $p) -ForegroundColor Green }
    else              { Write-Host ("  {0,-16} FAILED to write {1}" -f ($k + ':'), $p) -ForegroundColor Red }
}

# Tell Explorer the desktop changed, so the icon appears without a sign-out or a
# manual refresh. SHCNE_ASSOCCHANGED (0x08000000), SHCNF_IDLIST (0x0000).
if (-not ('Shell32Notify' -as [type])) {
    Add-Type -Namespace Win32 -Name Shell32Notify -MemberDefinition @'
[System.Runtime.InteropServices.DllImport("shell32.dll")]
public static extern void SHChangeNotify(int eventId, uint flags, System.IntPtr item1, System.IntPtr item2);
'@ -ErrorAction SilentlyContinue
}
try { [Win32.Shell32Notify]::SHChangeNotify(0x08000000, 0, [IntPtr]::Zero, [IntPtr]::Zero) } catch { }

Write-Host ''
Write-Host '  Easiest way in: press the Windows key, type  arm  , press Enter.' -ForegroundColor Green
Write-Host '  A black window opens and stays open - that window IS the connection' -ForegroundColor Gray
Write-Host '  to the Pi. The browser opens by itself a few seconds later.' -ForegroundColor Gray
Write-Host ''
Write-Host '  If the desktop icon still is not visible, right-click the desktop' -ForegroundColor Gray
Write-Host '  and choose Refresh.' -ForegroundColor Gray
Write-Host ''
exit 0
