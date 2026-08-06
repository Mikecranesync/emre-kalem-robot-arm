# Emre Kalem Robot Arm — Bench Setup

Cross-platform Arduino toolchain and safe single-servo bring-up for the Emre Kalem
6-axis robot arm (Arduino Uno, 4 × MG995/MG996R, 3 × MG90S, 6 logical axes /
7 physical servos). The arm console runs on Windows and macOS; the firmware is the
same on both hosts.

Working notes and verified procedures from actually getting this running — not a
reprint of the manufacturer's booklet.

## Quick start

On Windows, double-click **`CHECK SETUP.bat`**. On macOS, double-click
**`CHECK SETUP.command`** or run `./check-setup.sh` from Terminal. Both checks are
read-only: they do not upload firmware or enable motion.

To start the console, use **`START ARM GUI.bat`** on Windows or
**`START ARM GUI.command`** on macOS. The macOS launcher starts the same localhost
Python/pyserial bridge and opens the console in the default browser. Keep its
Terminal window open while the console is in use.

If Python/pyserial is unavailable, open `Software/arm-console/arm-console.html`
directly in Google Chrome or Microsoft Edge and use its Web Serial transport.
Safari and Firefox do not provide the Web Serial API required by that no-install
route. The bridge route works with the default macOS browser because it uses HTTP.

The bridge dependency can be installed with:

```
python3 -m pip install --user pyserial
```

The Windows check also inspects the Arduino IDE, AVR board package, Servo library,
sketch compilation, board discovery, and 115200 support. The portable macOS check
verifies Python/pyserial, the project files, Arduino CLI when installed, and visible
serial devices; it never uploads firmware.

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
