# bench-preflight.ps1
# Is the bench ready, and is it running the build you think it is?
#
# Run this BEFORE a bench session, with servo power still OFF.
#
#   powershell -NoProfile -ExecutionPolicy Bypass -File Documentation\bench-preflight.ps1
#
# READ-ONLY. It does not start the bridge, does not open the serial port, does
# not enable a joint, and never touches servo power. It reads and compares.
#
# WHAT IT IS FOR. Every bench session so far has started with the same three
# unasked questions: is the bridge alive, is the Pi serving the console I just
# built, and does the pose file on the bench still match the repository. Getting
# any of those wrong means testing a build that is not the one under review, or
# discovering afterwards that a taught pose was never pulled back. Answer them
# in one command instead of remembering.
#
# The rocker switch and the inline fuse are the real emergency stop. This
# script, the Arduino, and the console are NOT safety devices.

$PORT      = 8770
$URL       = "http://127.0.0.1:$PORT/"
$HOSTALIAS = 'arm'
$SSH       = Join-Path $env:SystemRoot 'System32\OpenSSH\ssh.exe'
$REMOTEDIR = '$HOME/arm/console'

$root    = Split-Path -Parent $PSScriptRoot
$repoHtml  = Join-Path $root 'Software\arm-console\arm-console.html'
$repoPoses = Join-Path $root 'Software\arm-console\arm-poses.csv'

$pass = 0; $fail = 0; $warn = 0
function Ok  ($m){ Write-Host ("  PASS  " + $m) -ForegroundColor Green;  $script:pass++ }
function No  ($m){ Write-Host ("  FAIL  " + $m) -ForegroundColor Red;    $script:fail++ }
function Hmm ($m){ Write-Host ("  WARN  " + $m) -ForegroundColor Yellow; $script:warn++ }

function Sha($path){ (Get-FileHash -Algorithm SHA256 -LiteralPath $path).Hash }

Write-Host ''
Write-Host ' ============================================================'
Write-Host '  BENCH PREFLIGHT - read-only, servo power should still be OFF'
Write-Host ' ============================================================'
Write-Host ''

# --- 1. the repo build --------------------------------------------------------
if (Test-Path $repoHtml) { Ok ("console in the repo: " + (Get-Item $repoHtml).Length + " bytes") }
else { No "cannot find Software\arm-console\arm-console.html"; }

# --- 2. can we reach the Pi? --------------------------------------------------
$hostname = & $SSH -o BatchMode=yes -o ConnectTimeout=10 $HOSTALIAS 'hostname' 2>&1
if ($LASTEXITCODE -eq 0) { Ok ("Pi reachable over SSH: " + ($hostname -join '')) }
else {
    No "cannot reach the Pi over SSH - check power, then Tailscale, then try 'ssh arm-lan'"
    Write-Host ''
    Write-Host ("  $fail failed. Nothing else can be checked without the Pi.") -ForegroundColor Red
    Write-Host ''
    exit 1
}

# --- 3. bridge alive? ---------------------------------------------------------
# [a] bracket so the pattern does not match the shell ssh is running it in.
$bridge = & $SSH -o BatchMode=yes -o ConnectTimeout=10 $HOSTALIAS `
          "pgrep -f '[a]rm-bridge.py' >/dev/null && echo RUNNING || echo STOPPED" 2>&1
if ($bridge -match 'RUNNING') { Ok 'bridge running on the Pi' }
else { Hmm 'bridge is NOT running - the Arm Console icon will start it for you' }

# --- 4. is the board plugged into the Pi? -------------------------------------
# Presence of the USB device only. Says nothing about servo power, which is
# physical and is yours to confirm at the rocker.
$tty = & $SSH -o BatchMode=yes -o ConnectTimeout=10 $HOSTALIAS `
       "ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -1" 2>&1
if ($tty -match '/dev/tty') { Ok ("board present on the Pi at " + ($tty -join '').Trim() + " (USB only - says nothing about servo power)") }
else { No 'no Arduino on the Pi - check the USB cable is a DATA cable and is seated' }

# --- 5. is the hold daemon stopped? -------------------------------------------
# It must be. It reports EN=1 for joints whose rail is dead, so one left running
# when the rocker goes back on snaps every joint to a stale setpoint.
$hold = & $SSH -o BatchMode=yes -o ConnectTimeout=10 $HOSTALIAS `
        "pgrep -f '[h]old_arm_pi.py' >/dev/null && echo RUNNING || echo STOPPED" 2>&1
if ($hold -match 'STOPPED') { Ok 'hold daemon stopped (this is the safe state before power-on)' }
else { No 'HOLD DAEMON IS RUNNING - stop it BEFORE switching servo power on, or every joint snaps to the setpoint it was last holding' }

# --- 6. is the Pi serving the build in the repo? ------------------------------
$served = $null
try {
    $r = Invoke-WebRequest -Uri $URL -UseBasicParsing -TimeoutSec 6 -MaximumRedirection 3 -ErrorAction Stop
    $served = $r.Content
} catch { $served = $null }

if ($null -eq $served) {
    Hmm 'console did not answer on 127.0.0.1:8770 - start it with the Arm Console icon, then re-run this'
} else {
    $tmp = [IO.Path]::GetTempFileName()
    [IO.File]::WriteAllText($tmp, $served)
    if ((Sha $tmp) -eq (Sha $repoHtml)) { Ok 'the Pi is serving EXACTLY the console in this repo' }
    else {
        No 'the Pi is serving a DIFFERENT console from the one in this repo'
        Write-Host ("        deploy it:  scp Software/arm-console/arm-console.html {0}:~/arm/console/" -f $HOSTALIAS) -ForegroundColor Yellow
    }
    Remove-Item $tmp -ErrorAction SilentlyContinue
}

# --- 7. does the bench pose file match the repo? ------------------------------
# So that after the session, any diff is what you taught and nothing else.
$benchPoses = & $SSH -o BatchMode=yes -o ConnectTimeout=10 $HOSTALIAS "cat $REMOTEDIR/arm-poses.csv" 2>$null
if (-not $benchPoses) { No 'cannot read arm-poses.csv on the Pi' }
else {
    $tmp2 = [IO.Path]::GetTempFileName()
    [IO.File]::WriteAllLines($tmp2, $benchPoses)
    $a = (Get-Content -LiteralPath $tmp2 -Raw) -replace "`r`n","`n"
    $b = (Get-Content -LiteralPath $repoPoses -Raw) -replace "`r`n","`n"
    $benchRows = ($a -split "`n" | Where-Object { $_ -and -not $_.StartsWith('#') }).Count - 1
    if ($a.TrimEnd() -eq $b.TrimEnd()) { Ok ("pose file matches the repo ({0} poses) - any diff afterwards is what you taught" -f $benchRows) }
    else { Hmm 'pose file on the bench DIFFERS from the repo - run Software/arm-console/sync-poses.sh and commit BEFORE teaching, or you will not be able to tell old work from new' }
    Remove-Item $tmp2 -ErrorAction SilentlyContinue
}

# --- 8. is there a rollback copy of the console on the Pi? --------------------
$baks = & $SSH -o BatchMode=yes -o ConnectTimeout=10 $HOSTALIAS `
        "ls -1 $REMOTEDIR/arm-console.html.bak-* 2>/dev/null | tail -1" 2>&1
if ($baks -match 'bak-') { Ok ("rollback copy on the Pi: " + ($baks -join '').Trim()) }
else { Hmm 'no rollback copy of the console on the Pi' }

Write-Host ''
Write-Host (" {0} passed, {1} warnings, {2} failed" -f $pass, $warn, $fail)
Write-Host ''
if ($fail) {
    Write-Host '  Fix the failures before switching servo power on.' -ForegroundColor Red
    Write-Host ''
    exit 1
}
Write-Host '  Bench is ready. Hand under the forearm BEFORE the rocker.' -ForegroundColor Green
Write-Host ''
exit 0
