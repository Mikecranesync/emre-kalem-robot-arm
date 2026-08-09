# arm-console-pi.ps1
# Open the arm console when the Arduino is plugged into the PI, not this laptop.
#
# Called by START ARM GUI.bat, which picks this route automatically when no
# Arduino is on a local COM port. Can also be run on its own.
#
# WHAT IT DOES, in order:
#   1. if the console already answers on 127.0.0.1:8770, just opens it
#   2. otherwise checks the bridge is running on the Pi, and starts it if not
#   3. opens an SSH tunnel so 127.0.0.1:8770 on this laptop reaches the Pi
#   4. opens the browser
#   5. holds the window open, because the window IS the tunnel
#
# IT DOES NOT TOUCH THE ARM. Starting the bridge does not open the serial port
# and does not enable a joint - the port is opened by CONNECT on the page, and
# opening it resets the board. Servo power is the rocker switch and is yours.
#
# THE HOLD DAEMON IS NOT STARTED HERE, deliberately. hold_arm_pi.py reports
# EN=1 for joints whose rail is dead, so a daemon left running when the rocker
# goes back on snaps every joint to the setpoint it was holding. Starting it is
# a decision made at the bench with a hand on the arm, never by a launcher.
#
# This is a supervised hobby bench tool. The Arduino, this script, and the
# on-screen stop button are NOT safety devices. The rocker switch and the
# inline fuse are the real stop.

$PORT      = 8770
$URL       = "http://127.0.0.1:$PORT/"
$HOSTALIAS = 'arm'
$SSH       = Join-Path $env:SystemRoot 'System32\OpenSSH\ssh.exe'
$REMOTEDIR = '~/arm/console'
$REMOTEPY  = '~/arm-venv/bin/python'

function Say($text, $colour) {
    if ($colour) { Write-Host $text -ForegroundColor $colour } else { Write-Host $text }
}

# Does the console actually ANSWER? A listening port is not the same thing: a
# tunnel can be up while the bridge behind it is dead, and that failure looks
# identical to a working setup until the page refuses to load.
function Test-Console {
    try {
        $r = Invoke-WebRequest -Uri $URL -UseBasicParsing -TimeoutSec 5 `
                               -MaximumRedirection 3 -ErrorAction Stop
        return ($r.StatusCode -eq 200)
    } catch {
        return $false
    }
}

Say ''
Say ' ============================================================' 'Gray'
Say '  FACTORYLM ARM CONSOLE  (arm is on the Pi)' 'Gray'
Say ' ============================================================' 'Gray'
Say ''

if (-not (Test-Path $SSH)) {
    Say '  PROBLEM: Windows OpenSSH is missing.' 'Red'
    Say ("  Expected it here: {0}" -f $SSH) 'Red'
    Say '  Install it from Settings > System > Optional features > OpenSSH Client.' 'Red'
    Say ''
    exit 1
}

# --- 1. already up? -----------------------------------------------------------
if (Test-Console) {
    Say '  The console is already running. Opening it.' 'Green'
    Start-Process $URL
    Say ''
    Say '  A tunnel from an earlier run is holding the connection, so this' 'Gray'
    Say '  window is not needed and will close. Do not close the OTHER window.' 'Gray'
    Say ''
    Start-Sleep -Seconds 3
    exit 0
}

# --- 2. is the port held by something that is NOT answering? ------------------
$bound = Get-NetTCPConnection -LocalPort $PORT -State Listen -ErrorAction SilentlyContinue
if ($bound) {
    $owner = (Get-Process -Id $bound[0].OwningProcess -ErrorAction SilentlyContinue).ProcessName
    Say ("  PROBLEM: port {0} is held by {1} (PID {2}), but nothing answers on it." -f `
         $PORT, $owner, $bound[0].OwningProcess) 'Red'
    Say '  That is usually a tunnel left over from a Pi that went away.' 'Red'
    Say '  Close that window (or end that process), then run this again.' 'Red'
    Say ''
    pause
    exit 1
}

# --- 3. is the Pi there, and is the bridge running? ---------------------------
Say ("  Checking the Pi ({0})..." -f $HOSTALIAS) 'Gray'

# THE PGREP SELF-MATCH TRAP. Over ssh the search pattern lives in the command
# string that ssh itself runs, so a plain `pgrep -f arm-bridge.py` matches that
# shell and reports the bridge alive when it is not. The [a] bracket stops the
# pattern matching its own text.
$check = & $SSH -o BatchMode=yes -o ConnectTimeout=10 $HOSTALIAS `
         "pgrep -f '[a]rm-bridge.py' >/dev/null && echo RUNNING || echo STOPPED" 2>&1
if ($LASTEXITCODE -ne 0) {
    Say '  PROBLEM: could not reach the Pi over SSH.' 'Red'
    Say ("  It said: {0}" -f ($check -join ' ')) 'Red'
    Say ''
    Say '  Check in this order:' 'Yellow'
    Say '    1. is the Pi powered on' 'Yellow'
    Say '    2. is Tailscale up on this laptop and on the Pi' 'Yellow'
    Say '    3. try the garage LAN instead:  ssh arm-lan' 'Yellow'
    Say ''
    pause
    exit 1
}

if ($check -match 'STOPPED') {
    Say '  The bridge was not running on the Pi. Starting it.' 'Yellow'
    # setsid + nohup + closed stdin so the bridge survives this ssh session
    # ending. Without that it dies the moment the check returns.
    & $SSH -o BatchMode=yes -o ConnectTimeout=10 $HOSTALIAS `
      "cd $REMOTEDIR && setsid nohup $REMOTEPY arm-bridge.py > bridge.out 2>&1 < /dev/null & sleep 3" | Out-Null
    Start-Sleep -Seconds 1
} else {
    Say '  The bridge is already running on the Pi.' 'Green'
}

# --- 4. tunnel ----------------------------------------------------------------
Say '  Opening the tunnel...' 'Gray'
# ExitOnForwardFailure means a tunnel that cannot bind the port FAILS instead of
# sitting there looking connected while nothing is forwarded.
$tunnelArgs = @('-N', '-o', 'BatchMode=yes', '-o', 'ExitOnForwardFailure=yes',
                '-o', 'ServerAliveInterval=30', '-o', 'ConnectTimeout=10',
                '-L', "${PORT}:127.0.0.1:$PORT", $HOSTALIAS)
$tunnel = Start-Process -FilePath $SSH -ArgumentList $tunnelArgs `
                        -WindowStyle Hidden -PassThru

# --- 5. wait for it to actually answer ---------------------------------------
$ok = $false
foreach ($i in 1..20) {
    Start-Sleep -Seconds 1
    if ($tunnel.HasExited) { break }
    if (Test-Console) { $ok = $true; break }
}

if (-not $ok) {
    Say ''
    Say '  PROBLEM: the tunnel opened but the console never answered.' 'Red'
    Say '  Read what the bridge logged on the Pi:' 'Yellow'
    Say ("     ssh {0} ""tail -20 {1}/bridge.out""" -f $HOSTALIAS, $REMOTEDIR) 'Yellow'
    Say ''
    if (-not $tunnel.HasExited) { Stop-Process -Id $tunnel.Id -Force -ErrorAction SilentlyContinue }
    pause
    exit 1
}

# --- 6. in -------------------------------------------------------------------
Start-Process $URL
Say ''
Say '  Ready. The console is open in your browser.' 'Green'
Say ''
Say '  Press CONNECT, pick the Arduino, and the arm is yours.' 'Gray'
Say '  Servo power is the rocker switch, and it is still yours to flip.' 'Gray'
Say ''
Say ' ------------------------------------------------------------' 'Gray'
Say '  LEAVE THIS WINDOW OPEN. It is holding the connection to the' 'Gray'
Say '  Pi. Closing it stops the page working. Ctrl-C here to stop.' 'Gray'
Say ' ------------------------------------------------------------' 'Gray'
Say ''

try {
    Wait-Process -Id $tunnel.Id
} finally {
    if (-not $tunnel.HasExited) { Stop-Process -Id $tunnel.Id -Force -ErrorAction SilentlyContinue }
}

Say ''
Say '  The connection to the Pi has stopped. The console page will not work now.' 'Yellow'
Say '  Double-click the Arm Console icon again to restart it.' 'Yellow'
Say ''
pause
exit 0
