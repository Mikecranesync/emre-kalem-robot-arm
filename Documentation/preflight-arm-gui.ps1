# preflight-arm-gui.ps1
# Run by START ARM GUI.bat before the bridge starts.
#
# Only one program can hold a serial port. The Arduino IDE Serial Monitor, an old
# bridge, a browser tab using Web Serial, and SERVO CONSOLE.bat all want the same
# port and will lock each other out. Whichever grabbed it first wins and everything
# else reports "port is blocked", which reads like a hardware fault and is not one.
#
# This clears what it safely can and NAMES what it cannot.

$PORT = 8770

Write-Host ''
Write-Host ' Checking nothing else is already holding the board...' -ForegroundColor Gray

# --- 1. stale bridges from a previous run -------------------------------------
# Match python.exe running arm-bridge.py ONLY. Matching on command-line text alone
# also catches shells that merely mention it.
$stale = @(Get-CimInstance Win32_Process -ErrorAction SilentlyContinue |
           Where-Object { $_.Name -match '^python' -and $_.CommandLine -match 'arm-bridge' })

foreach ($p in $stale) {
    Write-Host ("   closing an old bridge that was still running (PID {0})" -f $p.ProcessId) -ForegroundColor Yellow
    Stop-Process -Id $p.ProcessId -Force -ErrorAction SilentlyContinue
}
if ($stale.Count) { Start-Sleep -Seconds 2 }

# --- 2. is our port still bound? ----------------------------------------------
$bound = Get-NetTCPConnection -LocalPort $PORT -State Listen -ErrorAction SilentlyContinue
if ($bound) {
    $owner = (Get-Process -Id $bound.OwningProcess -ErrorAction SilentlyContinue).ProcessName
    Write-Host ''
    Write-Host ("   PROBLEM: something is already using port {0}" -f $PORT) -ForegroundColor Red
    Write-Host ("   It is: {0} (PID {1})" -f $owner, $bound.OwningProcess) -ForegroundColor Red
    Write-Host '   Close it, then run this again.' -ForegroundColor Red
    Write-Host ''
    exit 1
}

# --- 3. can a serial port actually be opened? ---------------------------------
# Use SetupAPI enumeration, never [System.IO.Ports.SerialPort]::GetPortNames(),
# which reports stale registry entries for boards that are not plugged in.
$devs = @(Get-CimInstance Win32_PnPEntity -ErrorAction SilentlyContinue |
          Where-Object { $_.Name -match 'COM\d+' -and $_.Status -eq 'OK' })

if (-not $devs.Count) {
    Write-Host ''
    Write-Host '   No board found. Plug the Arduino in with a DATA cable.' -ForegroundColor Yellow
    Write-Host '   Starting anyway - the console still works with no board attached.' -ForegroundColor Gray
    Write-Host ''
    exit 0
}

$blocked = @()
foreach ($d in $devs) {
    if ($d.Name -notmatch '\((COM\d+)\)') { continue }
    $name = $Matches[1]
    $sp = New-Object System.IO.Ports.SerialPort $name, 115200
    try { $sp.Open(); $sp.Close(); Write-Host ("   {0} is free." -f $name) -ForegroundColor Green }
    catch {
        if ($_.Exception.Message -match 'denied') { $blocked += $name }
        else { Write-Host ("   {0} present but not usable: {1}" -f $name, $_.Exception.Message.Trim()) -ForegroundColor Yellow }
    }
}

if ($blocked.Count) {
    Write-Host ''
    Write-Host ('   PROBLEM: {0} is held by another program.' -f ($blocked -join ', ')) -ForegroundColor Red
    Write-Host ''
    Write-Host '   Only ONE program can use the board at a time. Close whichever of' -ForegroundColor Yellow
    Write-Host '   these you have open, then run this again:' -ForegroundColor Yellow
    Write-Host ''
    Write-Host '     - the Arduino IDE Serial Monitor (the magnifying-glass panel)' -ForegroundColor Yellow
    Write-Host '     - any browser TAB showing arm-console.html or 127.0.0.1:8770' -ForegroundColor Yellow
    Write-Host '       (a tab using Web Serial keeps the port until the TAB is closed)' -ForegroundColor Yellow
    Write-Host '     - SERVO CONSOLE.bat' -ForegroundColor Yellow
    Write-Host ''
    $running = @(Get-Process -ErrorAction SilentlyContinue |
                 Where-Object { $_.ProcessName -match 'arduino|chrome|msedge' } |
                 Group-Object ProcessName | ForEach-Object { $_.Name })
    if ($running.Count) {
        Write-Host ('   Currently running that could be holding it: ' + ($running -join ', ')) -ForegroundColor Gray
        Write-Host ''
    }
    exit 1
}

Write-Host ''
exit 0
