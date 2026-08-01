$src = @"
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
    public static extern IntPtr CreateFile(string name, uint access, uint share,
        IntPtr sec, uint disp, uint flags, IntPtr tmpl);

    [DllImport("kernel32.dll", SetLastError = true)]
    public static extern bool GetCommState(IntPtr h, ref DCB dcb);

    [DllImport("kernel32.dll", SetLastError = true)]
    public static extern bool SetCommState(IntPtr h, ref DCB dcb);

    [DllImport("kernel32.dll", SetLastError = true)]
    public static extern bool CloseHandle(IntPtr h);

    public static string Probe(string port, uint baud) {
        IntPtr h = CreateFile("\\\\.\\" + port, 0xC0000000, 0, IntPtr.Zero, 3, 0, IntPtr.Zero);
        if (h == new IntPtr(-1))
            return "CreateFile FAILED  err=" + Marshal.GetLastWin32Error();

        DCB dcb = new DCB();
        dcb.DCBlength = (uint)Marshal.SizeOf(typeof(DCB));
        if (!GetCommState(h, ref dcb)) {
            int e = Marshal.GetLastWin32Error(); CloseHandle(h);
            return "CreateFile ok | GetCommState FAILED  err=" + e;
        }

        uint had = dcb.BaudRate;
        dcb.BaudRate = baud;
        dcb.ByteSize = 8; dcb.Parity = 0; dcb.StopBits = 0;
        bool ok = SetCommState(h, ref dcb);
        int err = Marshal.GetLastWin32Error();
        CloseHandle(h);

        return ok
            ? "CreateFile ok | GetCommState ok (was " + had + ") | SetCommState OK at " + baud
            : "CreateFile ok | GetCommState ok (was " + had + ") | SetCommState FAILED at " + baud + "  err=" + err;
    }
}
"@

Add-Type -TypeDefinition $src -Language CSharp

# Find the board's COM port automatically - it changes whenever you use a different
# USB socket. Pass a port name as an argument to override, e.g.  comprobe.ps1 COM7
$port = $args[0]
if (-not $port) {
    $dev = Get-CimInstance Win32_PnPEntity -ErrorAction SilentlyContinue |
           Where-Object { $_.Name -match 'COM\d+' -and $_.Status -eq 'OK' } |
           Select-Object -First 1
    if ($dev -and $dev.Name -match '\((COM\d+)\)') { $port = $Matches[1] }
}
if (-not $port) { Write-Host 'No serial port found. Plug the board in.'; exit 1 }

Write-Host "=== direct Win32 probe on $port ==="
foreach ($b in 1200, 9600, 14400, 19200, 38400, 57600, 115200) {
    Write-Host ("{0,7} : {1}" -f $b, [ComProbe]::Probe($port, [uint32]$b))
}
