# Emre Kalem Robot Arm — Bench Setup

Windows laptop setup, Arduino toolchain, and safe single-servo bring-up for the Emre
Kalem 6-axis robot arm (Arduino Uno, 4 × MG995/MG996R, 3 × MG90S, 6 logical axes /
7 physical servos).

Working notes and verified procedures from actually getting this running — not a
reprint of the manufacturer's booklet.

## Quick start

Double-click **`CHECK SETUP.bat`** to verify the whole toolchain and board.

```
powershell -ExecutionPolicy Bypass -File C:\RobotArm\check-setup.ps1
```

It checks the Arduino IDE, the AVR board package, the Servo library, compiles both
sketches, finds the board, and confirms the port can run at the 115200 the bootloader
needs.

## Layout

```
Software/       Arduino sketches + the pin map
Documentation/  setup status, diagnosis write-ups, plain-English guide
Calibration_Notes/  per-servo calibration log
Backups/        untouched originals from the vendor package
```

## Read these

| File | What it covers |
|---|---|
| `Documentation/NEXT-STEPS-PLAIN-ENGLISH.md` | Start here if you just want to use it |
| `Documentation/SETUP-STATUS.md` | What is installed, what is verified, what is missing |
| `Documentation/V2-SERVO-POWER-AND-WIRING.md` | Correct power architecture, sourced |
| `Documentation/USB-SERIAL-DIAGNOSIS.md` | Why the clone Uno cannot be programmed |

## Safety

This is a supervised hobby bench procedure, not a safety-rated commissioning standard.
The Arduino, the sketch, the rocker switch, and the software stop are **not** safety
devices.

```
USB → UNO ONLY   |   EXTERNAL 5V → SERVO ONLY   |   GROUNDS JOIN
```

Never connect an external supply's positive to the Uno's `5V`, `VIN`, or barrel jack.
