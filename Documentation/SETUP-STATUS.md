# Emre Kalem Robot Arm — Setup Status

Prepared 2026-08-01 on the Windows travel laptop, following
`Emre_Kalem_Arduino_Uno_Beginner_Walkthrough.docx`.

**Nothing has been uploaded to a board. No servo has been powered.**

---

## 1. What is installed and verified

| Item | Walkthrough expects | Installed | Verified how |
|---|---|---|---|
| Windows | 10 64-bit or newer | Windows 11, build 10.0.26200 (64-bit) | `ver` |
| Arduino IDE | 2.3.10 | **2.3.10** | `Arduino IDE.exe` file version |
| Arduino CLI | not required | **1.5.1** (latest) | `arduino-cli version` |
| Arduino AVR Boards | required | **1.8.8** | `arduino-cli core list` |
| Servo library | install only if missing | **1.3.0** (official, by Arduino) | it *was* missing — see §3 |

Both sketches compile clean for `arduino:avr:uno`:

```
blink_toolchain_check               924 bytes (2%)  flash,   9 bytes (0%)  RAM
emre_kalem_single_servo_bench_test 6902 bytes (21%) flash, 333 bytes (16%) RAM
```

Arduino IDE was installed to `%LOCALAPPDATA%\Programs\Arduino IDE` (per-user, no
administrator rights needed).

**The IDE really does see the board package and the Servo library — this was checked,
not assumed.** The IDE was launched once to complete its first run, then its own
built-in compiler back end was queried directly using the IDE's own configuration file
(`%USERPROFILE%\.arduinoIDE\arduino-cli.yaml`). It reports:

```
core list  ->  arduino:avr  1.8.8  Arduino AVR Boards
lib  list  ->  Servo        1.3.0  (user)
compile    ->  emre_kalem_single_servo_bench_test: 6902 bytes (21%) - OK
```

So when you press Verify inside the IDE, it uses the same AVR compiler and the same
Servo library that were installed and tested here. You should **not** see
`Servo.h: No such file or directory`. If you somehow do, the fix is the walkthrough's:
Tools → Manage Libraries → search `Servo` → the one published by Arduino → Install.

Arduino IDE 2.3.10 is also the current official release (checked against Arduino's
own release feed on 2026-08-01), so the walkthrough's version number is up to date.
The Windows package manager only offered 2.3.8, so the installer was taken from
Arduino's official release page instead.

---

## 1b. Board — Checkpoint B PASSED (2026-08-01 ~10:00)

**Use the genuine Arduino Uno. Do not use the clone board.**

| | |
|---|---|
| Board | Genuine Arduino Uno R3, `VID_2341 / PID_0043` |
| Port | **COM5** ("USB Serial Device" — Windows in-box driver, nothing to install) |
| Identified by arduino-cli as | `Arduino UNO`, `arduino:avr:uno`, core `arduino:avr` |
| Chip | ATmega328P, signature `1E 95 0F` |
| Bootloader | STK500 v1, HW 3 / FW 4.4, at 115200 |

Blink was uploaded and then **read back off the chip and byte-compared**:

```
Writing 924 bytes to flash   100%
924 bytes of flash written
Verifying 924 bytes of flash against input file
924 bytes of flash verified
```

That is the walkthrough's **Checkpoint B**, passed on real hardware. The whole chain —
laptop, cable, driver, COM port, board package, compiler, bootloader, chip — is proven.
The yellow `L` LED should now blink one second on, one second off.

### The clone board does not work on this laptop

The other Uno (a clone with a WCH CH340 USB chip) cannot be programmed here. Its USB
driver refuses every baud rate at or above 19200; the bootloader needs 115200. This is a
known defect in WCH's CH341SER driver 3.8 and later, which Windows 11 installed
automatically. It is not a broken board or a bad cable — a different USB socket and a
different cable were both tested and made no difference.

Full evidence, upstream references, and the driver-downgrade procedure if that board is
ever needed: `Documentation\USB-SERIAL-DIAGNOSIS.md`. **No system drivers were changed.**

---

## 2. Folder layout created

```
C:\RobotArm\
├── check-setup.ps1                      <- run this any time to re-verify everything
├── Software\
│   ├── blink_toolchain_check\
│   │   └── blink_toolchain_check.ino    <- local copy of the standard Blink example
│   ├── emre_kalem_single_servo_bench_test\
│   │   └── emre_kalem_single_servo_bench_test.ino
│   ├── wiring-map.csv
│   └── conveyor-waypoints-template.csv
├── Documentation\
│   ├── SETUP-STATUS.md                  <- this file
│   └── Emre_Kalem_Arduino_Uno_Beginner_Walkthrough.docx
├── Calibration_Notes\
│   └── calibration-log.csv              <- fill this in as you center each servo
└── Backups\
    └── Emre_Kalem_Arduino_Uno_Beginner_Walkthrough.docx   (untouched original)
```

Each `.ino` sits in a folder of the same name, as the walkthrough's
"Arduino folder rule" requires.

---

## 3. Two things the walkthrough assumes that turned out differently

### 3a. The Servo library was NOT already present

The walkthrough lists Servo as "usually already available". On a clean install it is
not — the AVR core 1.8.8 ships only EEPROM, HID, SoftwareSerial, SPI and Wire. The
bench-test sketch failed to compile with exactly the error the walkthrough predicts:

```
fatal error: Servo.h: No such file or directory
```

The official Arduino Servo library 1.3.0 was then installed, and the sketch compiles.
**You do not need to do this step yourself — it is already done.** If you ever see that
error again, the fix is the one in the walkthrough: Tools → Manage Libraries → search
`Servo` → the one published by Arduino → Install.

### 3b. Two sketches named in the walkthrough do not exist

The downloaded project package (`Robotic+Arm+with+Servo+&+Arduino.zip`) contains
**only 3D-printable STL parts**. These files, which the walkthrough refers to, were
never in it:

- `emre_kalem_arm_calibrate.ino` — the main calibration program
- `emre_kalem_arm_uno_controller.ino` — the final arm controller

**They have deliberately not been written for you.** Phase 2 of the walkthrough says
"select the test servo using the *actual* calibration sketch command" — if a
substitute carried that filename, you could believe you were following the vendor's
calibration procedure when you were not. That is a safety problem, not a convenience
problem.

The walkthrough already covers this case. It says of the bench-test sketch: *"Use this
included sketch only when the main calibration file is unavailable or unclear."* It is
unavailable, so the bench test is the sketch to use.

Ask the arm's author or source for the two missing `.ino` files. When they arrive, drop
each into `C:\RobotArm\Software\<same-name-as-file>\` and re-run `check-setup.ps1`.

---

## 4. What you can do right now

Checkpoints A and B are already passed (see §1b). The next step in the walkthrough is
Phase 2 — the software side of servo calibration, done with **servo power still OFF**.

1. **Open Arduino IDE.** The genuine Uno self-identifies, so Tools → Board and
   Tools → Port should already show **Arduino Uno** on **COM5**. Confirm both.
2. **Bench test**: File → Open →
   `C:\RobotArm\Software\emre_kalem_single_servo_bench_test\emre_kalem_single_servo_bench_test.ino`
   → Verify → Upload, **with the KCD1 rocker OFF and no servo powered**.
3. **Serial Monitor at 115200.** You should see the banner, the safety reminders, and
   the command list. Nothing is driven — the servo signal stays detached until you
   type `a`.
4. Practise the command sequence **with no servo connected at all**: `5` → `c` → `a`
   → `+` → `-` → `d`. It costs nothing and you learn the flow before any hardware can
   move.

Uploading with servo power off is safe: the sketch never attaches the servo signal in
`setup()`.

Re-run the checker any time with `CHECK SETUP.bat`, or:
```
powershell -ExecutionPolicy Bypass -File C:\RobotArm\check-setup.ps1
```

Uploading the bench test with the KCD1 rocker OFF is safe: the sketch keeps the servo
signal **detached** until you type `a`, so no pulses leave the Uno.

---

## 5. Bench-test sketch — command reference

Serial Monitor at **115200 baud**. Any line-ending setting works.

| Type | Meaning |
|---|---|
| `0`–`6` | Select a servo (0 Base D3, 1 Shoulder-L D4, 2 Shoulder-R D5, 3 Elbow D6, 4 Wrist-pitch D9, 5 Wrist-roll D10, 6 Gripper D11) |
| `c` | Store the center command: 90 degrees |
| `a` | Attach the signal and output 90 degrees |
| `+` / `-` | Move 1 degree and back |
| `]` / `[` | Move 5 degrees and back — only after the 1-degree test passes |
| `d` | Detach the signal before rewiring |
| `s` | Show status |
| `h` or `?` | Show help |

Normal order: `5` → `c` → `a` → turn rocker ON → `+` → `-` → `d`.

Built-in limits you cannot override from the keyboard:

- Every angle is clamped to **70–110 degrees**. `0` and `180` are impossible.
- The signal is **detached at power-up**. Nothing is driven until you type `a`.
- Selecting a different servo while one is attached **auto-detaches** it first.
- The shoulder pair is never driven as a pair. Center D4 and D5 separately.

---

## 6. Still blocked — hardware, not software

Physical servo motion still needs the safe external 5 V path from the walkthrough:
regulated 5 V supply, inline fuse in the positive line, KCD1 rocker or rated DC
disconnect, insulated terminals, a multimeter, and a ground jumper from the external
supply to an Arduino GND pin.

```
USB -> UNO ONLY    |    EXTERNAL 5 V -> SERVO ONLY    |    GROUNDS JOIN
```

Never connect external +5 V or a servo red wire to Uno `5V`, `VIN`, or the barrel jack.
Start with **wrist roll on D10** — it carries the least mechanical load.

---

## 7. Note on `wiring-map.csv`

The pin column is taken straight from the walkthrough and is reliable. The
**servo type** column is mostly inferred and is marked as such:

- Wrist roll = MG90S is stated in the walkthrough (`DOC-CONFIRMED`).
- The other six are split to match the bill of materials (4 × MG995/MG996R,
  3 × MG90S) using the usual big-joints/small-joints arrangement. Each is marked
  `INFERRED` — check them against your physical build before relying on them.

`conveyor-waypoints-template.csv` is a **header-only placeholder**. The walkthrough
names the file but never defines its columns, so the header is a reasonable guess, not
the vendor's schema. Replace it if the real one arrives.

---

## 8. Limits of this document

This is a supervised hobby-robot bench procedure, not a safety-rated industrial robot
commissioning standard. The Uno, the servo sketch, the rocker switch, and the software
stop are **not** safety devices.
