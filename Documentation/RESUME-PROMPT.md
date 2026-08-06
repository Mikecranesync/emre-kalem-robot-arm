# Resume prompt — Emre Kalem arm

Paste this into a fresh session. It exists because the same three questions kept
getting re-opened; the **Settled** section below is the fix. Last updated
2026-08-06.

---

## ⚠️ TWO REPOSITORIES. Read before any git command.

| | |
|---|---|
| **The work** | `C:\RobotArm` → `github.com/Mikecranesync/emre-kalem-robot-arm` |
| **Session cwd** | `C:\Users\hharp\Documents\MIRA` → **unrelated repo** |

Every command needs `cd /c/RobotArm` first. Run `git remote -v` before any commit.

**Branch:** `feat/lerobot-adapter-and-markers` → **PR #5, open.** Also open and
unrelated: PR #4 (envelope joystick), PR #2 (troubleshooting guide).

---

## 🛑 SETTLED — do not re-litigate these

Each of these was raised, answered by the operator, and is closed. Raising them
again wastes his time and is the specific failure this file exists to stop.

### The shoulder mirror is fine. Enable J1 normally.

`mirror_offset_deg` is 0 and unmeasured, and the docs warn that a wrong axis makes
the two MG996R fight each other. **That warning is not new information and it is
not a blocker.** The operator has already run this joint on exactly this
configuration. `joint-limits.csv` records `OK LIM J1 MIN=0 MAX=91 CAL=1` locked
2026-08-05 19:04 — **you cannot lock a range you have not driven.**

`MIR INV 0` must go out *after* joint 1's `LIM` and while J1 is disabled. That's an
ordering rule, not a hazard gate. Measuring the true offset is still worth doing
some day. It does not block anything.

### The fuse question is closed. Stop raising it.

The operator has ruled: *"Don't worry about the fuse. the power is fine."* Do not
re-raise it, do not caveat work with it, do not add it to a risk list.

### The base servo is dead. That is the whole D3 mystery.

Not a software bug. The firmware provably drove D3 at 29.2 °/s for 3.5 s and
nothing moved. Two software causes were found and fixed along the way — `DPS=1`
from the scroll-wheel hazard, and a destroyed 29–110 envelope — and **neither was
the cause.** A replacement was ordered 2026-08-06.

**When the new servo goes on, re-measure J0.** Horns seat on a splined shaft in
~18° steps on an MG996R, so 29–110 will not point where it used to.

### D11 is the gripper. Confirmed by observation.

The 2026-08-01 log row saying "the servo on D3 was the GRIPPER" records the
**single-servo bench sketch** — horn off, unloaded, where "index 0" means pin D3 on
a bench rig. Not the assembled arm. Two configurations, never a conflict.

---

## Where things stand right now

**A holder daemon is running and keeping the arm up.** Detached OS process,
`python hold_arm.py`, in the session scratchpad.

```
J0 base        EN=0   dead servo — deliberately excluded
J1 shoulder    EN=1   SET=1     (0–91)    MIR=INV OFF=0
J3 elbow       EN=1   SET=33    (0–66)
J4 wrist pitch EN=1   SET=90    (0–180)   ends UNCONFIRMED
J5 wrist roll  EN=1   SET=104   (31–178)
J6 gripper     EN=1   SET=40    (10–70)   proven today
ES=0  WD=0  MIR=INV  UNCAL=1
```

### Why it is a daemon and not a script

Two things detach every joint the moment a plain script exits — the watchdog trips
(and the firmware's recovery is a full **latched** detach), and closing the port
can toggle DTR and reset the board. So it holds the port and never leaves.

**Two bugs already cost a session here. Do not reintroduce them:**

1. **The heartbeat must not block.** Feeding it with a call that waits for a reply
   (up to 1.2 s) can overrun the 4000 ms watchdog. It writes `PNG\n` and never
   waits — writing is what feeds the watchdog.
2. **Something must notice a latch.** `PNG` does **not** clear one. Once tripped it
   stays tripped, and the loop will go on answering `STA` cheerfully while every
   joint is dead and the arm is on the bench. There is now a 5 s health check that
   detects `ES=1`/`WD=1`, sends `CLR`, and re-enables.

### Driving it

The console and any script are **mutually exclusive** — one owner per serial port,
and `GET /rx` is destructive so a second poller steals the console's replies.

While the holder runs, send commands by writing a protocol line to `arm_cmd.txt`
in the scratchpad; the reply lands in `arm_hold.log`. Full state is written to
`arm_status.txt` every 5 s.

For the console instead: double-click `START ARM GUI.bat`, use the tab it opens
(each launch mints a fresh access code — typing the URL will not work), leave the
black window open.

---

## Hard-won facts. Do not re-derive these.

- **`LIM` takes FOUR arguments** — `<j> <min> <max> <cal>`. Three gives `ERR E2`.
- **`STA` takes none.** `STA J0` is `ERR E2`.
- **Opening the port DTR-resets the board and the firmware keeps nothing.** Limits
  go back to 70–110, `MIR=UNKNOWN`, `CAL=0`. Any script must push `LIM` itself.
- **J6's 10–70 lies almost entirely BELOW the 70–110 boot default**, so after any
  reconnect everything under 70 clamps until `LIM 6 10 70 1` goes out.
- **`STA`'s `SET=` is fiction on a disabled joint.** The firmware seeds every joint
  to 90 at boot. Validity keys off `EN=`.
- **`ENA <j> <adopt>` drives the joint to `<adopt>`.** The firmware pre-loads that
  pulse *before* attaching. Nothing observes the shaft, so the number is always a
  human's estimate. Adopt where the joint *is* and it does not move; adopt at
  midpoint when it has sagged and it will lift the arm to get there.
- **`START ARM GUI.bat` cannot be launched from `cmd`** — `START` is a cmd builtin.
  Use PowerShell `Start-Process`.
- **Nothing in software is an emergency stop.** `STP` holds with joints driven;
  `EST` and the watchdog *detach* and a gravity-loaded arm falls. The rocker and
  the fuse are the only real stop.

---

## What is proven, and what is not

**Proven 2026-08-06 — the first motion on this arm confirmed against the shaft**
rather than against an ack. The operator watched the gripper open and close, and
the camera corroborated: 1799 and 1657 changed pixels between commanded 10° and
70° across two cycles, against a **272-pixel same-command control**, with global
shift under 0.5 px. The interpolator was also proven to *ramp* — `MOV=1` with `SET`
stepping `12 17 23 29 34 40 46 51 57 63 68 70`, ~11–12 °/s against a commanded 12.

**Every joint now has a lock.** `uncalibrated_ids()` is empty. 168 tests pass.

**Not proven:**

- Nothing in `Software/lerobot_robot_emre_arm/` has been imported against a real
  LeRobot install or run against a board. **Unproven at both ends.**
- **No camera has been calibrated**, so every marker size still rests on an assumed
  ~60° field of view. **Do not print stickers yet.**
- `_BENCH_SUPPLY_V = 5.0` is the **label** on the JCPOWER JC-25-5, not a
  measurement. The old unit was nominally 6 V and read 6.62.
- The numpy / pyserial BSD-3-Clause exception is unruled, so nothing is described
  as licence-compliant.

---

## The camera — solved, and how to unstick it next time

Device is `Integrated Camera` (`USB\VID_174F&PID_2469`), the laptop's **internal**
webcam and the only camera on this machine — the other PnP entries are the HP Envy
printer's scanner. So "replug it" is never the answer.

**If it returns frozen frames (identical MD5s, `max 0` between captures), another
app owns it.** Find the holder without guessing — Windows tracks per-app camera
use in the registry, and `LastUsedTimeStop = 0` means *in use right now*:

```powershell
$r='HKCU:\SOFTWARE\Microsoft\Windows\CurrentVersion\CapabilityAccessManager\ConsentStore\webcam'
Get-ChildItem $r -Recurse | ForEach-Object {
  $p = Get-ItemProperty $_.PSPath -ErrorAction SilentlyContinue
  if ($p.LastUsedTimeStop -eq 0) { Split-Path $_.PSPath -Leaf } }
```

On 2026-08-06 the holder was **`Microsoft.WindowsCamera`** — the operator had
opened the Camera app to aim the laptop at the arm. Closing it freed the device
immediately. No reboot, no admin needed. (Restarting the `FrameServer` service or
cycling the PnP device *would* need admin; this session had none.)

### The measurement lesson that outlives the bug

A `0 px` difference means **the capture is dead, not that the arm is steady.**
Always take a **same-command control frame** before believing any null result.
That control is what caught a false negative earlier the same day, where an
over-aggressive threshold plus a 5×5 morphological opening erased the thin edge
bands a moving finger produces, and the absence was nearly reported as a finding.

### Backdrop matters more than it sounds

Against the operator's **white sheet** behind the gripper, open-vs-closed measured
a clean **6× separation over a 272 px floor**. Against cluttered cardboard, with
the gripper smaller in frame, it could not be resolved at all. **Put the white
backdrop back for any camera-based gripper or marker verification.**

### The arm drifts while held

Observed 2026-08-06: with five joints energised, the whole gripper assembly
settles downward slowly. The joints hold, but frames captured minutes apart are
**not** comparable — an aggregate frame difference picks up the drift rather than
the thing you are measuring. Capture control and test frames close together in
time, and expect this to matter for the marker work.

**Board acks prove the firmware accepted a command. They do not prove a motor
turned** — that distinction is what cost the D3 afternoon.

---

## Working style

- **Ask what he observed.** Three runs once "passed" while nothing physically moved.
- State the expected behaviour and the test **before** each task; run the narrowest
  relevant test after; commit only on green.
- **Merges, pushes and purchases need their own explicit approval.** Commits do not.
- Plain English for anything decision-shaped. No jargon, one sentence per option.
- Caveman mode may be active — terse. Safety-critical and multi-step content drops
  out of it automatically.
- He invokes `ultracode` when he wants multi-agent orchestration. Not otherwise.
- **Do not use `graphify` for code navigation.**

---

## Next

1. **Test the gripper** with the arm held — the command channel is live.
2. **Get the camera back**, then re-verify the hold visually. Right now the hold is
   only attested by board acks.
3. **Base servo swap** when it arrives, then re-measure J0 and resolve its identity
   flag the same way J6's was: drive it and watch.
4. Longer-term: calibrate a camera so the marker sizes stop resting on an
   assumption, and measure the shoulder's true mirror offset.
