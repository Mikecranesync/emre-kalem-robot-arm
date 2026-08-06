# Session findings — 2026-08-06

Plain English. What we proved, what changed, what is still open, and exactly where
to pick up. Read the last section first if you are short on time.

---

## The headline

**The gripper moved, and it is the first time on this arm that a commanded motion
has been confirmed against the actual shaft rather than against an acknowledgement
from the board.**

Everything before today proved the firmware *accepted* a command. Nothing proved a
motor turned. That distinction is the whole reason the D3 mystery burned an
afternoon.

It was confirmed twice over:

- **You watched it** and reported it opened and closed. Under the project's
  vocabulary rule that is the approved exception — a value a human actually
  *observed*, recorded as observed.
- **The camera agreed.** 1799 and 1657 pixels changed between commanded 10° and
  70° across two cycles, against a **272-pixel control** taken between two frames
  at the *same* command. Phase correlation put the whole-assembly shift under
  0.5 px, so it is the fingers moving and not the arm swinging — which mattered,
  because the other five joints were detached and limp and reaction torque could
  have moved everything.

Evidence: `Calibration_Notes/evidence/2026-08-06_J6-gripper_*.png`.

**The interpolator was also proven to ramp, not just to book a target.** Sampling
`STA` *during* the move showed `MOV=1` with `SET` stepping
`12 17 23 29 34 40 46 51 57 63 68 70` — about 11–12 °/s against a commanded 12.
An earlier run only ever sampled *after* the move settled, so every reading was
`MOV=0` and could not tell the two apart.

---

## The base servo is dead, and that is the answer to the D3 mystery

You found it. It explains the thing that has been open since 2026-08-01: the
firmware provably drove D3 at 29.2 °/s for 3.5 seconds and nothing happened.

**It was not the software.** Two software causes were found and fixed along the
way and both were real, but neither was the cause:

- `DPS=1` on the J0 card — a three-second jog moved three degrees and read as
  dead. Cause: the SPEED box changed value on scroll-wheel. **Fixed today.**
- A 0–180 sweep had destroyed J0's measured 29–110 envelope. **Fixed on 2026-08-06
  in `6e95e69`.**

A replacement arrives tomorrow from Amazon.

> **When the new servo goes on, J0's 29–110 envelope probably does not survive.**
> Horns seat on a splined shaft in whole-tooth steps — about 18° apart on an
> MG996R. A new servo fitted at a different tooth means 29–110 no longer points
> where it used to. **Re-measure J0 after the swap. Do not trust the existing row.**

---

## The D3 / D11 identity dispute — resolved for the gripper

The 2026-08-01 log said the servo physically wired to D3 was the GRIPPER, not the
Base. That has been flagged in code and docs ever since as an unresolved dispute.

**It was never actually a conflict.** That row records the **single-servo bench
sketch** — horn off, servo unloaded — where "index 0" selects **pin D3 on a bench
rig**. That is not the assembled arm's wiring. Two different configurations, not
two claims about one thing.

Driving J6 and watching the gripper move proves the joint the firmware calls J6
*is* the gripper, so the wiring map is right about D11.

**J0 stays disputed** — the base servo is dead, so driving D3 proves nothing either
way. Resolve it after the replacement the same way: drive it and watch.

---

## The new power supply changes more than it looks

**JCPOWER JC-25-5 — +5 V, 5 A.** Transcribed from its own label.

It closes **two** blockers:

- **Current.** 5 A replaces the ~700 mA unit that could not hold an assembled arm.
  That was gating the whole marker slice, not just convenience.
- **Voltage, which is the bigger one.** The old supply measured **6.62 V** and the
  MG90S is rated **4.8–6.0 V**. J4, J5 and J6 are all MG90S. J6's own limits row
  said in as many words *"do NOT test it on the 6.62 V adapter"* — that block is
  what the new supply cleared, and it is why the gripper could be tested today at
  all.

### But the fuse now matters MORE, not less

The 700 mA unit was accidentally protective — it current-limited, so a stalled
servo just sagged the rail and the worst case was a brownout.

**5 A has no such mercy.** It will deliver 5 A into a stalled servo, a pinched
lead, or a short until something else gives way. A single stalled MG90S sits
comfortably under 5 A and will simply cook.

**The 1 A slow-blow inline fuse is now a hard prerequisite. It is still not
fitted, and J6 has now been driven repeatedly without it.**

Slow-blow specifically — the 4700 µF capacitor's inrush nuisance-blows a fast 1 A
and it will read as an intermittent hardware fault.

### Two more things

- **Meter the rail.** 5 V is the *label*. The old unit was nominally 6 V and read
  6.62 — that discrepancy is the entire reason J4 and J5 carry a stress history.
- **Expect ~10–15 % less holding torque.** 5 V is the bottom of the MG996R window.
  If a gravity-loaded joint sags where it did not before, that is the trade, not a
  fault. The fix would be a 6 V supply, not more current.

---

## Every joint on this arm now has a lock

J6 was the last hold-out. `uncalibrated_ids()` is now empty.

| joint | pin | range | home | note |
|---|---|---|---|---|
| 0 Base | D3 | 29–110 | 64 | **servo dead** — re-measure after the swap |
| 1 Shoulder | D4+D5 | 0–91 | 1 | mirror offset still **unmeasured** — see hazard below |
| 3 Elbow | D6 | 0–66 | 33 | |
| 4 Wrist pitch | D9 | 0–180 | 90 | **SUSPECT** — 0–180 is the whole electrical range |
| 5 Wrist roll | D10 | 31–178 | 104 | |
| 6 Gripper | D11 | **10–70** | **40** | **locked and proven today** |

> **Your J6 lock existed only in `Downloads/`.** `joint-limits.csv` still said
> 0–180. That is exactly the condition the file's own header warns about, and it
> is how the base row got silently widened on 2026-08-05. It has been copied into
> `Calibration_Notes/lock-artifacts/` where the H5 guard can see it.
> **Habit worth forming: after every LOCK, move the downloaded row into the repo.**

> **`10–70` sits almost entirely BELOW the firmware's `70–110` boot default.**
> Opening the serial port resets the board and it keeps nothing. After any
> reconnect, press **LOAD LIMITS FILE** before touching the gripper, or everything
> under 70 silently clamps and it will look broken.

---

## The scroll-wheel hazard is fixed

A focused number box in a browser steps its own value when the wheel turns over
it. No click, no keystroke, nothing on screen. That is how J0's speed became
`DPS=1` for an afternoon.

All three number boxes on every joint card are now guarded, and **SPEED was the
least dangerous of the three**:

- **the adopt box** is focused *and selected* the instant ENABLE opens, so a page
  scroll lands straight in it — and its value is the pulse the firmware pre-loads
  **before** attaching, which is the mechanism that snaps a joint.
- **the target box** is the angle SEND transmits.
- **the speed box** sends `SPD` on change. The `DPS=1` path.

Guarded only while focused, so scrolling the page past an unfocused box still
works. Regression test wired into `Software/tests/selftest.sh`, proven able to
fail against four mutants.

---

## Software fixes that shipped today

Six items had been recorded as KNOWN OPEN in commit `b4caedf`. All addressed.

- **151 tests written** for the LeRobot adapter's observation schema and
  calibration validators — `__init__.py` had claimed "is tested" with no test file
  anywhere. The headline test replays the 2026-08-05 J0 widening and asserts it is
  refused. Now 168 tests.
- **`send_action()` no longer emits six MOVs per tick.** It sends one only when a
  target changes. **This introduced a bug that was caught before it shipped:** the
  dedup turned a *loud* failure into a silent one — after the idle watcher's
  `DIS A`, the next tick recorded a confident angle for a detached, sagging joint.
  `get_observation()` now reconciles against the board's `EN=`.
- **A ChArUco board generator and an intrinsics gate** that refuses to emit a
  calibration that fails its own quality check. Every marker size still rests on
  an assumed ~60° field of view until a camera is actually calibrated.
- **Marker placement positions now derive from the surveyed geometry**, with the
  two that genuinely cannot be surveyed drawn differently and labelled NOT
  SURVEYED on the drawing itself.
- **The spec was telling the next person to delete a safety guard.** It had
  adjudicated that `calibrate()`'s raise "moved to `_save_calibration()`". Both
  raise, and they guard different things. Recorded as a visible retraction rather
  than quietly corrected.

### One of the six never existed

`b4caedf` listed *"the print sheet's single cut line is physically impossible for
the three smallest markers."* There have been **thirteen separate cut lines since
the file was created**. A previous pass recorded a defect it had not verified — in
the very commit message whose purpose was recording what remained.

**Treat inherited to-do lists as claims to check, not as facts.**

---

## Still open

### Hardware

- [ ] **1 A slow-blow inline fuse** — now a prerequisite, not a nice-to-have
- [ ] **Meter the actual rail voltage** — 5 V is the label, not a measurement
- [ ] **Base servo replacement** (Amazon, tomorrow) — then **re-measure J0**
- [ ] 4700 µF capacitor moved off the breadboard

### The shoulder — read before enabling the arm

**J1 is the joint that stops the arm falling, and it is the most dangerous one to
enable.**

After any port reset `MIR=UNKNOWN`, so `ENA 1` returns `E13` until `MIR INV 0` is
pushed first. **That command asserts a mirror offset nobody has ever measured.**
J1 is two MG996R driven in opposition; if the true mirror axis is not 90°, they
fight each other continuously — hold, get hot, and eventually strip a gear.

**The firmware's own E13 guard does not catch this.** It is satisfied the moment
`mirror_mode` is set, while the hazard is fully present. Two stalled MG996R draw
~2.5 A each — 5 A, exactly this supply's rating, on an unfused rail.

The correct fix is to **measure the offset**: linkage unbolted, horns off, drive
D4 and D5 separately, find the angles where both output shafts point the same
physical way, then `offset = (a + b) / 2 - 90`. That is a bench job.

### Software

- [ ] Nothing runs the 168 tests automatically — no CI, no pre-commit hook
- [ ] `_BENCH_SUPPLY_V = 5.0` is the label value, unmetered
- [ ] Nothing in the LeRobot package has been imported against a real LeRobot
      install or run against a board. Unproven at both ends.
- [ ] The numpy / pyserial BSD-3-Clause licence exception is still unruled, so
      nothing is described as licence-compliant anywhere

---

## Pick up here

**The goal when you are back: enable the other joints so the arm holds itself and
only one joint moves at a time.**

Three things are needed and two of them are yours:

1. **Confirm the dead base servo's 3-pin lead is unplugged from D3.** A failed
   servo commonly fails *shorted*, and an unfused 5 A rail into a short is the one
   genuinely bad outcome available on this bench. J0 will not be enabled either
   way — a dead motor holds nothing.

2. **Rough adopt angles, by eye, for where each joint is sitting right now.**
   `ENA` pre-loads the pulse *before* attaching, so the servo drives to whatever
   number it is given. Adopt at where the joint actually is and it does not move
   at all; adopt at midpoint when it has sagged to the bottom and it will *lift
   the arm* to get there. Within ~10° is fine — closer for J4, whose limits are
   unconfirmed.

   The arm has visibly moved since the last run (compare the evidence frames), so
   nothing can be inferred from the last commanded values.

3. **A decision on the shoulder.** Recommended: **try it without J1 first.**
   Enable J3, J4, J5, J6 — none of those need a mirror assertion, so no new
   hazard — and see whether the arm actually falls. It has been detached most of
   this session without collapsing, so the shoulder may hold on friction alone. If
   it does, the gear-strip risk is avoided entirely.

**Also worth doing:** pull the camera back to see the whole arm rather than just
the gripper. It is currently framed too tight to judge shoulder or elbow angles,
and a wider view means each enable can be verified visually instead of on trust.

---

## How to drive it

The console and any script are **mutually exclusive** — only one program can hold
the serial port, and `GET /rx` is destructive, so a second poller steals the
console's replies.

- **Console:** double-click `START ARM GUI.bat`. Use the browser tab it opens; each
  launch mints a fresh access code and typing the address by hand will not work.
  Leave the black window open.
- **Script:** the preflight in the launcher kills stale bridges. A raw script must
  push `LIM` itself after opening the port, because the DTR reset wipes the board's
  state. `LIM` takes **four** arguments — `<j> <min> <max> <cal>`; three gives
  `ERR E2`.

**Nothing in software is an emergency stop.** `STP` stops motion but keeps the
joints powered and holding. `EST` and the watchdog *detach*, and a gravity-loaded
arm falls. The rocker switch and the fuse are the only real stop.
