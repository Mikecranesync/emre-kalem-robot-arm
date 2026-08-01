# servo-console.ps1
# A plain terminal for the robot arm bench-test sketch.
#
# Opens the Arduino's serial port, prints everything it says, and sends every key
# you press straight to it. No Arduino IDE needed, no Send button to hunt for.
#
# Run it by double-clicking  SERVO CONSOLE.bat  in C:\RobotArm
#
# IMPORTANT: close the Arduino IDE's Serial Monitor first. Only one program can
# hold the port at a time.

$ErrorActionPreference = 'Stop'

# --- find the board ------------------------------------------------------------
$port = $args[0]
if (-not $port) {
    $dev = Get-CimInstance Win32_PnPEntity -ErrorAction SilentlyContinue |
           Where-Object { $_.Name -match 'COM\d+' -and $_.Status -eq 'OK' } |
           Select-Object -First 1
    if ($dev -and $dev.Name -match '\((COM\d+)\)') { $port = $Matches[1] }
}

if (-not $port) {
    Write-Host ''
    Write-Host 'No board found. Plug the Arduino in and try again.' -ForegroundColor Red
    Write-Host ''
    Read-Host 'Press Enter to close'
    exit 1
}

# --- open it -------------------------------------------------------------------
$sp = New-Object System.IO.Ports.SerialPort $port, 115200, 'None', 8, 'One'
$sp.ReadTimeout = 200
$sp.DtrEnable   = $true
$sp.RtsEnable   = $true

try {
    $sp.Open()
} catch {
    Write-Host ''
    Write-Host ("Could not open {0}." -f $port) -ForegroundColor Red
    Write-Host 'Almost always this means the Arduino IDE Serial Monitor is still open.' -ForegroundColor Yellow
    Write-Host 'Close it (or close the whole IDE) and run this again.' -ForegroundColor Yellow
    Write-Host ''
    Read-Host 'Press Enter to close'
    exit 1
}

Clear-Host
Write-Host '=========================================================' -ForegroundColor Cyan
Write-Host (" SERVO CONSOLE  -  connected on {0} at 115200" -f $port) -ForegroundColor Cyan
Write-Host '=========================================================' -ForegroundColor Cyan
Write-Host ' Just press a key. It goes straight to the Arduino.' -ForegroundColor Gray
Write-Host ''
Write-Host '   0-6  pick a pin      c  centre 90      a  ATTACH (signal on)'
Write-Host '   +  -  one degree     ]  [  five deg    w  wide test 90-110-70-90'
Write-Host '   d    DETACH          s  status         h  help'
Write-Host ''
Write-Host ' Press ESC to quit. It sends "d" on the way out so the servo' -ForegroundColor Gray
Write-Host ' is never left driven.' -ForegroundColor Gray
Write-Host '---------------------------------------------------------' -ForegroundColor Cyan
Write-Host ''

# --- pump --------------------------------------------------------------------
try {
    while ($true) {

        # anything the Arduino said
        try {
            $incoming = $sp.ReadExisting()
            if ($incoming.Length) { Write-Host -NoNewline $incoming }
        } catch { }

        # anything you typed
        if ([Console]::KeyAvailable) {
            $key = [Console]::ReadKey($true)

            if ($key.Key -eq 'Escape') {
                Write-Host ''
                Write-Host '>>> detaching and closing...' -ForegroundColor Yellow
                try { $sp.Write('d'); Start-Sleep -Milliseconds 600 } catch { }
                try { Write-Host -NoNewline $sp.ReadExisting() } catch { }
                break
            }

            $ch = $key.KeyChar
            if ($ch) {
                Write-Host ''
                Write-Host (">>> sent '{0}'" -f $ch) -ForegroundColor Cyan
                $sp.Write([string]$ch)
            }
        }

        Start-Sleep -Milliseconds 40
    }
}
finally {
    if ($sp.IsOpen) { $sp.Close() }
    Write-Host ''
    Write-Host 'Port closed. Signal detached.' -ForegroundColor Green
    Write-Host ''
    Read-Host 'Press Enter to close this window'
}
