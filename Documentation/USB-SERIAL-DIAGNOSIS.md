# COM3 will not upload — diagnosis log

Date: 2026-08-01, Windows 11 build 26200 travel laptop.
Board: clone Arduino Uno with a WCH CH340 USB-to-serial chip.

**Result: the port opens fine, but the driver refuses every baud rate at or above
19200. The Arduino bootloader needs 115200. So uploading is impossible until this is
fixed. Nothing has been changed on the laptop.**

---

## The measurement that settles it

A direct Win32 probe (`CreateFile` → `GetCommState` → `SetCommState`), bypassing both
the `mode` command and .NET, run against COM3:

```
   1200 : CreateFile ok | GetCommState ok | SetCommState OK
   9600 : CreateFile ok | GetCommState ok | SetCommState OK
  14400 : CreateFile ok | GetCommState ok | SetCommState OK
  19200 : CreateFile ok | GetCommState ok | SetCommState FAILED  err=31
  38400 : CreateFile ok | GetCommState ok | SetCommState FAILED  err=31
  57600 : CreateFile ok | GetCommState ok | SetCommState FAILED  err=31
 115200 : CreateFile ok | GetCommState ok | SetCommState FAILED  err=31
```

Error 31 is `ERROR_GEN_FAILURE` — "A device attached to the system is not functioning."

Read that carefully, because it reframes everything:

- **Opening the port always works.** `CreateFile` succeeds every time.
- **Reading the port config always works.** `GetCommState` succeeds every time.
- **Setting the speed works up to 14400 and fails from 19200 up.** A clean, sharp,
  repeatable cutoff.

The Arduino Uno bootloader communicates at **115200** (older clone bootloaders at
57600). Both are above the cutoff. That is the whole failure.

This also explains the confusing earlier symptoms: `avrdude` reported
`cannot set com-state for \\.\COM3` — literally the `SetCommState` call above. And
`mode COM3` could *read* the settings but not change them to 115200.

---

## What the board is

```
FriendlyName            USB-SERIAL CH340 (COM3)
Hardware ID             USB\VID_1A86&PID_7523&REV_0254
BusReportedDeviceDesc   USB2.0-Ser!
Status                  OK      Problem  CM_PROB_NONE      IsPresent  True
Driver                  wch.cn  3.9.2024.9  (09/16/2024)  signed WHQL, oem256.inf
Driver service          CH341SER_A64 -> C:\Windows\System32\Drivers\CH341S64.SYS
Device stack            \Driver\CH341SER_A64, \Driver\ACPI, \Driver\USBHUB3   (no filter drivers)
```

Not a genuine Arduino — a clone using a WCH CH340 instead of the ATmega16U2 that a real
Uno R3 uses. That is normal and usually fine.

---

## Ruled out, with the evidence

| Suspect | Ruled out by |
|---|---|
| Board not plugged in | Windows enumerates it; present, no problem code |
| Charge-only cable | A charge-only cable cannot enumerate a device at all; Windows read this device's USB string descriptors |
| Port held by another program | `CreateFile` succeeds — nothing else holds it |
| Wrong baud in software | Failure is the *driver refusing* the rate, measured directly at every rate |
| Driver missing / wrong vendor | WCH's own driver, signed, WHQL, current 3.9.2024.9 |
| Driver installed while board attached | Board was unplugged and replugged; `LastArrivalDate` = 08:26:29 proves re-enumeration. Unchanged |
| Device in an error state | `Problem = CM_PROB_NONE`, `HasProblem = False` |
| Insufficient rights | Error 31 (`ERROR_GEN_FAILURE`), not 5 (`ACCESS_DENIED`) |
| A registry baud limit on the port | `Device Parameters` contains only `PortName`, `PollingPeriod`, `EnumerationRetryCount` — no limit |
| A conflicting filter driver | Device stack is clean: CH341SER_A64, ACPI, USBHUB3 |
| A bad USB socket / hub port | Board moved to a different physical socket (hub 2 port 9 -> hub 2 port 1, re-enumerated as **COM4** at 09:39:03). **Byte-identical cutoff**, 2/2 runs |

Timeline:

```
07:55:26  FirstInstallDate  - board first seen
07:56:35  InstallDate       - CH341SER_A64 service installed
08:26:29  LastArrivalDate   - unplugged and replugged, re-enumerated cleanly
08:35     driver still refuses every rate >= 19200
```

---

## Update 09:40 — the USB port has been ruled out

The board was moved to a different physical USB socket. It re-enumerated cleanly as a
new device (`Port_#0001` instead of `Port_#0009`, new COM number **COM4**,
`LastArrivalDate` 09:39:03), so the swap definitely took effect.

The result was **byte-identical**, twice:

```
1200 / 9600 / 14400   ->  OK
19200 / 38400 / 57600 / 115200  ->  FAILED  err=31
```

Same sharp boundary, same error, different socket. This matters for the two candidates
below: a marginal electrical link is an *analog* fault and would be expected to move the
boundary or produce intermittent results. A boundary this sharp and this repeatable
across two independent sockets is the signature of a **deliberate capability decision in
software**, not a flaky wire.

Candidate B is therefore much weaker. Swapping the USB cable is the only free test left,
and is worth one attempt, but the evidence now points hard at candidate A.

## Update 09:47 — the USB cable has been ruled out too

A different USB-B cable was fitted. The device re-enumerated (`LastArrivalDate`
09:46:29). Result, twice more:

```
1200 / 9600 / 14400   ->  OK
19200 / 38400 / 57600 / 115200  ->  FAILED  err=31
```

Identical again. **Three independent physical configurations — two USB sockets and two
cables — produce exactly the same cutoff at exactly the same place.**

Candidate B (marginal USB link) is now eliminated. Every free, no-change test has been
run. What remains is candidate A: the driver is restricting this chip in software.

## Update 09:55 — ROOT CAUSE CONFIRMED against upstream sources

This is a **known defect in WCH's CH341SER driver from version 3.8 onward.** The laptop
has 3.9.2024.9. It is not a broken board, not a bad cable, and not (necessarily) a
counterfeit chip.

The avrdude project has an issue for this exact failure —
[avrdudes/avrdude#1328, "WCH CH340 serial port issue under Windows: can't set com-state
for \\.\COMx"](https://github.com/avrdudes/avrdude/issues/1328). Our error message is
character-for-character the one in that title. The maintainers labelled it **"not our
bug"**: it is a driver defect, and the only known fix is to downgrade the driver to
**3.5.2019.1** (30 Jan 2019).

Independent confirmation of the same failure and the same fix:
- [Arduino Forum — CH340 driver rollback on Windows 10 vs 11](https://forum.arduino.cc/t/ch340-driver-rollback-workaround-works-on-windows-10-but-not-11/1187954):
  same `can't set com-state` error; rollback **does** work on Windows 11. Notes that
  *"Windows 11 had always installed the newest driver! (But not on Windows 10)"* —
  which is exactly what happened here at 07:56:35.
- [Digital Town — Windows 11 CH340 driver problems](https://www.digitaltown.co.uk/66FakeCH340Chips.php):
  identifies 3.8.2023.2 as the first bad version and 3.5.2019.1 as the fix. Also notes
  that counterfeit CH340 chips can be spotted because **they have no printing on them**
  at all, whereas genuine chips are clearly marked.

Note the upstream sources do **not** describe it as a baud-rate ceiling — they report it
as a flat `SetCommState` failure. The measurement above (working ≤14400, failing ≥19200)
is a finer-grained view of the same defect than anyone upstream published.

**Fix if this board must be used:** downgrade the driver to 3.5.2019.1, sourced from
[wemos/ch340_driver](https://github.com/wemos/ch340_driver) (`CH341SER_WIN_3.5.ZIP`),
the source cited in the avrdude issue. Requires administrator rights, plus blocking
Windows Update from silently reinstalling 3.9 afterwards — every source warns it will
try.

## Update 09:56 — RESOLVED BY AVOIDANCE: a genuine Arduino Uno is available

Mike located a genuine Arduino Uno. A real Uno R3 uses an **ATmega16U2** for USB, not a
CH340, so it does not touch the CH341SER driver at all. The defect above simply does not
apply to it.

**Decision: use the genuine Uno. Do not modify system drivers.** The driver downgrade is
recorded here only in case the clone is needed later — it is no longer the plan.

---

## What was left before the genuine Uno turned up — two candidates

Both produce this signature. The port swap above weakened B considerably.

**A. The driver and this chip disagree about capabilities.**
The CH341SER driver asks the chip what it is before deciding which baud divisors to
allow. If the chip answers with something the driver does not recognise — because it is
a counterfeit CH340, or a variant, or the answer got corrupted — the driver falls back
to a restricted set and refuses the high rates. A genuine CH340 handles 2 Mbps, so a
14400 ceiling is not the chip's real limit.

An earlier note in this file claimed `USB2.0-Ser!` proves a counterfeit chip. **That was
wrong and has been removed** — that string appears on plenty of CH340 boards that work
fine. It is the stock WCH descriptor, not a forgery marker.

**B. A marginal USB link degraded the chip's capability negotiation.**
The driver's capability query rides on USB control transfers. A damaged cable, a worn
socket, or a low-power port can let a device enumerate and still corrupt those
transfers. Cheap to test, so test it first.

---

## What to try, cheapest first

1. **Different USB port on the laptop, plugged in directly, no hub.** Free, no system
   change. Then re-run `CHECK SETUP.bat`.
2. **A different USB-B data cable.** Free. Enumeration proves the data lines carry
   *something*, not that they carry it cleanly.
3. **Read the markings on the USB chip on the board** — the small square chip near the
   USB socket. The walkthrough asks for this before touching any driver. If it does not
   say `CH340`, everything above changes.
4. **Install an older WCH CH341SER driver** (3.5.2019.1 or 3.4.2016.9), which predates
   the capability restriction. Needs administrator rights. Still WCH's own official
   driver, just an earlier release — not a random internet driver. Only 3.9.2024.9 is in
   the driver store now, so there is nothing to roll back to; an older package must be
   added.
5. **Use a different board** — a genuine Uno R3, or a clone with an FTDI or CP2102
   chip — which avoids the CH340 driver entirely.

Steps 1–3 are free and reversible. Step 4 changes a system driver and is Mike's call.

---

## Reproducing the measurement

The probe script is at
`Documentation\comprobe.ps1`. Run:

```
powershell -ExecutionPolicy Bypass -File C:\RobotArm\Documentation\comprobe.ps1
```

A fixed board prints `SetCommState OK` on every line, including 115200.
