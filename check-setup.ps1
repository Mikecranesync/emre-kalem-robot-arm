# check-setup.ps1
# Emre Kalem Robot Arm - Arduino Uno bench setup verifier
#
# Run it any time. Safe: it only reads, lists, and compiles. It never uploads
# anything to the board and never touches servo power.
#
# How to run:
#   1. Press the Windows key, type PowerShell, open Windows PowerShell.
#   2. Type:  powershell -ExecutionPolicy Bypass -File C:\RobotArm\check-setup.ps1
#   3. Press Enter.

$ErrorActionPreference = 'Continue'

$CLI      = 'C:\Program Files\Arduino CLI\arduino-cli.exe'
$IDE      = "$env:LOCALAPPDATA\Programs\Arduino IDE\Arduino IDE.exe"
$SOFTWARE = 'C:\RobotArm\Software'
$FQBN     = 'arduino:avr:uno'

$pass = 0
$fail = 0

# Direct Win32 serial access. Needed because .NET's SerialPort.Open() collapses
# "cannot open the port" and "the driver refuses this baud rate" into one
# misleading error ("A device attached to the system is not functioning").
if (-not ('ComProbe' -as [type])) {
Add-Type -Language CSharp -TypeDefinition @"
using System;
using System.Runtime.InteropServices;

public static class ComProbe {
    [StructLayout(LayoutKind.Sequential)]
    public struct DCB {
        public uint DCBlength, BaudRate, Flags;
        public ushort wReserved, XonLim, XoffLim;
        public byte ByteSize, Parity, StopBits;
        public sbyte XonChar, XoffChar, ErrorChar, EofChar, EvtChar;
        public ushort wReserved1;
    }
    [DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Unicode)]
    static extern IntPtr CreateFile(string n, uint a, uint s, IntPtr sec, uint d, uint f, IntPtr t);
    [DllImport("kernel32.dll", SetLastError = true)] static extern bool GetCommState(IntPtr h, ref DCB d);
    [DllImport("kernel32.dll", SetLastError = true)] static extern bool SetCommState(IntPtr h, ref DCB d);
    [DllImport("kernel32.dll", SetLastError = true)] static extern bool CloseHandle(IntPtr h);

    public static IntPtr Open(string port) {
        IntPtr h = CreateFile("\\\\.\\" + port, 0xC0000000, 0, IntPtr.Zero, 3, 0, IntPtr.Zero);
        return (h == new IntPtr(-1)) ? IntPtr.Zero : h;
    }
    public static bool TrySetBaud(IntPtr h, uint baud) {
        DCB d = new DCB();
        d.DCBlength = (uint)Marshal.SizeOf(typeof(DCB));
        if (!GetCommState(h, ref d)) return false;
        d.BaudRate = baud; d.ByteSize = 8; d.Parity = 0; d.StopBits = 0;
        return SetCommState(h, ref d);
    }
    public static void Close(IntPtr h) { if (h != IntPtr.Zero) CloseHandle(h); }
}
"@
}

function Report($label, $ok, $detail) {
    if ($ok) { $script:pass++; $tag = '[ OK   ]' } else { $script:fail++; $tag = '[ FAIL ]' }
    Write-Host ("{0} {1,-34} {2}" -f $tag, $label, $detail)
}

Write-Host ''
Write-Host '========================================================'
Write-Host ' EMRE KALEM ROBOT ARM - SETUP CHECK'
Write-Host '========================================================'
Write-Host ''
Write-Host '--- Dependencies ---'

# 1. Arduino CLI
if (Test-Path $CLI) {
    $v = (& $CLI version) -join ' '
    Report 'Arduino CLI' $true $v
} else {
    Report 'Arduino CLI' $false 'not found at C:\Program Files\Arduino CLI'
}

# 2. Arduino IDE
if (Test-Path $IDE) {
    $iv = (Get-Item $IDE).VersionInfo.FileVersion
    Report 'Arduino IDE' $true "version $iv"
    if ($iv -ne '2.3.10') {
        Write-Host ("[ NOTE ] {0,-34} {1}" -f 'Arduino IDE version', "walkthrough was written against 2.3.10; you have $iv")
    }
} else {
    Report 'Arduino IDE' $false 'not installed'
}

# 3. AVR board package
if (Test-Path $CLI) {
    $core = (& $CLI core list) | Select-String 'arduino:avr'
    if ($core) { Report 'Arduino AVR Boards package' $true ($core -join ' ').Trim() }
    else       { Report 'Arduino AVR Boards package' $false 'missing - run: arduino-cli core install arduino:avr' }

    # 4. Servo library
    $lib = (& $CLI lib list) | Select-String '^Servo'
    if ($lib) { Report 'Servo library' $true ($lib -join ' ').Trim() }
    else      { Report 'Servo library' $false 'missing - run: arduino-cli lib install Servo' }
}

Write-Host ''
Write-Host '--- Project files ---'

$expected = @(
    'emre_kalem_single_servo_bench_test\emre_kalem_single_servo_bench_test.ino',
    'blink_toolchain_check\blink_toolchain_check.ino',
    'wiring-map.csv',
    'conveyor-waypoints-template.csv'
)
foreach ($rel in $expected) {
    $p = Join-Path $SOFTWARE $rel
    Report $rel (Test-Path $p) $(if (Test-Path $p) { 'present' } else { 'MISSING' })
}

# Files the walkthrough names but that did NOT ship in the downloaded package.
foreach ($missing in @('emre_kalem_arm_calibrate', 'emre_kalem_arm_uno_controller')) {
    $p = Join-Path $SOFTWARE "$missing\$missing.ino"
    if (Test-Path $p) {
        Report "$missing.ino" $true 'present'
    } else {
        Write-Host ("[ NOTE ] {0,-34} {1}" -f "$missing.ino", 'not supplied by the vendor package - see SETUP-STATUS.md')
    }
}

Write-Host ''
Write-Host '--- Compile check (no board needed) ---'

if (Test-Path $CLI) {
    foreach ($sk in @('blink_toolchain_check', 'emre_kalem_single_servo_bench_test')) {
        $dir = Join-Path $SOFTWARE $sk
        if (-not (Test-Path $dir)) { Report "compile $sk" $false 'sketch folder missing'; continue }
        $out = & $CLI compile --fqbn $FQBN $dir 2>&1
        if ($LASTEXITCODE -eq 0) {
            $sz = ($out | Select-String 'program storage space')
            Report "compile $sk" $true ($sz -join ' ').Trim()
        } else {
            Report "compile $sk" $false 'compile FAILED - see output below'
            $out | Select-Object -Last 8 | ForEach-Object { Write-Host "         $_" }
        }
    }
}

Write-Host ''
Write-Host '--- Board detection ---'

# Ask Windows directly. This is more reliable than arduino-cli's discovery,
# which can return "No boards found" if it is queried too soon after plug-in.
$serialDevs = @(Get-CimInstance Win32_PnPEntity -ErrorAction SilentlyContinue |
                Where-Object { $_.Name -match 'COM\d+' })

if ($serialDevs.Count -eq 0) {
    Write-Host '  Windows sees no serial (COM) device at all.'
    Write-Host ''
    Write-Host '  If the Uno IS plugged in:'
    Write-Host '    1. Try a different USB cable. Charge-only cables cause most "mystery" failures.'
    Write-Host '    2. Plug directly into the laptop, not through an unpowered USB hub.'
    Write-Host '    3. Check that the green ON LED is lit.'
    Write-Host '    4. Do not install random internet drivers.'
    Report 'Board on a COM port' $false 'no serial device found (plug it in and re-run)'
} else {
    foreach ($d in $serialDevs) {
        Write-Host ("  {0}   [{1}]" -f $d.Name, $d.Status)
    }
    Report 'Board on a COM port' $true ("{0} serial device(s) found" -f $serialDevs.Count)

    # Which COM number, and can the port actually be opened?
    foreach ($d in $serialDevs) {
        if ($d.Name -notmatch '\((COM\d+)\)') { continue }
        $port  = $Matches[1]
        $clone = ($d.Name -match 'CH340|CH341|CP210|USB-SERIAL|FT232')

        # Two separate questions: can the port be OPENED, and can it be SET to the
        # 115200 the Arduino bootloader needs? .NET's SerialPort.Open() does both at
        # once and reports a misleading "device is not functioning" if either fails.
        $openOk = $false
        $h = [ComProbe]::Open($port)
        if ($h -ne [IntPtr]::Zero) {
            $openOk = $true
            Report "$port opens" $true 'the port itself is fine'

            $bad = @()
            foreach ($b in 9600, 19200, 57600, 115200) {
                if (-not [ComProbe]::TrySetBaud($h, $b)) { $bad += $b }
            }
            [ComProbe]::Close($h)

            if ($bad -notcontains 115200) {
                Report "$port supports 115200" $true 'ready to upload'
            } else {
                Report "$port supports 115200" $false ('driver refuses these rates: ' + ($bad -join ', '))
                Write-Host ''
                Write-Host '  The port opens, but the USB-serial driver will not run it at the speed'
                Write-Host '  the Arduino bootloader needs (115200). Uploading is impossible until'
                Write-Host '  this is fixed. The board is probably NOT broken.'
                Write-Host '  Full write-up + what to try: Documentation\USB-SERIAL-DIAGNOSIS.md'
                Write-Host ''
            }
        } else {
            Report "$port opens" $false 'cannot open the port at all'
            Write-Host ''
            Write-Host '  In order, try:'
            Write-Host '    1. UNPLUG the USB cable, wait 5 seconds, plug it back in, re-run.'
            Write-Host '    2. Use a different USB port on the laptop, directly, no hub.'
            Write-Host '    3. Use a different USB-B DATA cable.'
            Write-Host '    4. Close anything else holding the port (Serial Monitor, PuTTY).'
            Write-Host ''
        }

        if ($clone) {
            Write-Host '  [ NOTE ] This is a CLONE board (CH340-type USB chip), not a genuine Arduino.'
            Write-Host '           That is fine, but the IDE will NOT auto-identify it. You must pick'
            Write-Host '           the board by hand: Tools -> Board -> Arduino AVR Boards -> Arduino Uno,'
            Write-Host ("           then Tools -> Port -> {0}." -f $port)
            $drv = Get-CimInstance Win32_PnPSignedDriver -ErrorAction SilentlyContinue |
                   Where-Object { $_.DeviceID -like '*VID_1A86*' } | Select-Object -First 1
            if ($drv) {
                Write-Host ("           USB-serial driver: {0} v{1} (signed: {2})" -f `
                            $drv.DriverProviderName, $drv.DriverVersion, $drv.IsSigned)
                Write-Host '           If the 115200 check above FAILED, this driver version is a suspect.'
                Write-Host '           See Documentation\USB-SERIAL-DIAGNOSIS.md before changing anything.'
            }
        }
    }

    # arduino-cli's own view, for the IDE's benefit.
    if (Test-Path $CLI) {
        Write-Host ''
        Write-Host '  arduino-cli sees:'
        (& $CLI board list --discovery-timeout 5s 2>&1) | ForEach-Object { Write-Host "    $_" }
    }
}

Write-Host ''
Write-Host '========================================================'
Write-Host (" PASS: {0}    FAIL: {1}" -f $pass, $fail)
Write-Host '========================================================'
Write-Host ''
Write-Host ' REMINDER: KCD1 rocker OFF. No external +5 V to the Uno.'
Write-Host ' USB -> Uno only. External 5 V -> servo only. Grounds join.'
Write-Host ''
