# Next Bench Run — User Tests

Bench card for the first session on the rebuilt console. Work down it in order.
Everything in Phase 0 runs with **servo power off**; nothing moves until Phase 1.

**What this run is for.** Three things, in this order of importance:

1. Prove that letting go of the jog bar really does record a step.
2. Produce a **park** pose that can actually be driven — the first one this arm
   has ever had.
3. Play it back. No taught pose has ever been played back on this arm.

Re-recording `storage` and `pick` is Phase 4 and is optional. If you run out of
time or patience, stop after Phase 3 and you have still got what matters.

---

## Before you start

| | |
|---|---|
| Servo power | **OFF** at the rocker until Phase 1 says otherwise |
| Hand | under the forearm **before** the rocker, every single time |
| The real stop | the rocker switch and the inline fuse. Not the screen. |
| Rollback | `cp ~/arm/console/arm-console.html.bak-* ~/arm/console/arm-console.html` on the Pi, then F5. No restart needed. |

**Joint limits in force.** Anything outside these gets clamped by the firmware.

| Joint | Limits | Home | Note |
|---|---|---|---|
| J0 Base | 0–180 | 90 | **Pin not verified.** A bench test found the gripper wired to D3, not the base. Watch which motor actually moves. |
| J1 Shoulder | 0–91 | 1 | Two servos through one link; mirror offset still unmeasured |
| J3 Elbow | 0–30 | 15 | Re-locked 2026-08-08, down from 0–66 |
| J4 Wrist pitch | 33–180 | 90 | Travel ends still unconfirmed |
| J5 Wrist roll | 0–180 | 90 | |
| J6 Gripper | 0–180 | 90 | Does not articulate yet |

---

## Phase 0 — power off

Run this first. It is read-only and takes a few seconds.

```
powershell -NoProfile -ExecutionPolicy Bypass -File Documentation\bench-preflight.ps1
```

- [ ] **P0.1** All checks pass. If it says the Pi is serving a *different* console, deploy before going further — otherwise you are testing a build nobody reviewed.
- [ ] **P0.2** Double-click **Arm Console** on the desktop. Browser opens, **TEACH** tab selected, six joints in the left rail.
- [ ] **P0.3** Click **RUN POSES**, **CALIBRATION**, **DIAGNOSTICS**, back to **TEACH**. The **E-STOP** and the safety line stay on screen on all four.
- [ ] **P0.4** Press **CONNECT**. Status goes green. No red notices.
- [ ] **P0.5** On **CALIBRATION**, click **J3 Elbow**. The envelope reads **0–30** and the pill says **ACKNOWLEDGED**, not DEFAULT.

> **P0.5 is the one that matters.** DEFAULT means the controller has not confirmed
> the limits and `joint-limits.csv` did not load — everything is on the firmware's
> 70–110 placeholder. Stop and fix that before powering anything.

---

## Phase 1 — power on, one joint, prove the recording

Hand under the forearm. Rocker on.

- [ ] **T1** Select **J1 Shoulder**, press **ENABLE**, type the angle it is sitting at by eye, **CONFIRM**. Nothing jumps.
- [ ] **T2** The teach table is empty and says *"No steps yet"* under SAVE POSE. **Touch the jog bar and hold it still for a second.** A row labelled **start** appears **before the arm has gone anywhere**.
- [ ] **T3** Jog about 10°, let go. A **second row** appears with the new angle.
- [ ] **T4** Nudge it barely — a degree or so — and let go. **No new row.**
- [ ] **T5** Hold the jog bar so the joint is moving, and **while still holding, click a different joint tile.** The joint **stops immediately**.
- [ ] **T6** Untick **Save a step each time I let go**. Jog, let go: **no row**. Tick it back on.

| Test | Proves | If it fails |
|---|---|---|
| T2 | The starting position is captured before the first move. Without it, playback jumps to step 1 from wherever the arm stands — every joint at once. | Stop. Recording is not safe to rely on. |
| T4 | A nudge is not a step, so the table stays readable. | Cosmetic. Carry on, delete spare rows. |
| T5 | Switching joints releases the one you were holding. Otherwise it keeps moving where you cannot see it. | **Stop and press E-STOP.** Report it. |

Delete the practice rows with **DELETE** before Phase 2.

---

## Phase 2 — make the park pose

Park is where the arm rests with the power off. It is the one pose worth being
fussy about.

**Rules, not angles.** Judge it with your eyes; nothing on this arm observes the
shaft.

- Stay **at least 3° inside** every limit. A joint against its own end has
  nowhere to back off to. Elbow travel is now 0–30, much less than when
  `storage` was recorded — the old fold does not exist any more.
- **Fold over the base** rather than reaching out over the bench.
- **Leave the wrist alone** unless you are sure of its travel. J4's ends are
  still unconfirmed.
- Think about power-off: the forearm swings down through the base area where the
  loom and the blue cable sit.

- [ ] **T7** Drive there in 8–12° steps, letting go at each step. Watch the arm, not the screen.
- [ ] **T8** Above **SAVE POSE** it reads *"Will save N steps as the path to this pose."* If it still says *"No steps yet"*, **do not save** — nothing was recorded and you would repeat the `Parked` row.
- [ ] **T9** Type `park`, press **SAVE POSE**. Green notice confirms it, and names the step count.
- [ ] **T10** **No** amber warning about a joint sitting on a soft limit. If you get one, you are parked on an end — back off a few degrees and save again.
- [ ] **T11** On **RUN POSES**, `park` shows **READY** with its step count, and **LOAD** is enabled.

---

## Phase 3 — the first playback

Nothing taught on this arm has ever been played back. Go slowly and keep a hand
free for the rocker.

- [ ] **T12** Jog the arm away from park, somewhere obviously different.
- [ ] **T13** On **RUN POSES**, press **LOAD** on `park`. It drops you on **TEACH** with the steps in the table. Read them.
- [ ] **T14** Press **PLAY**. The arm retraces the path and ends at park.

**Abort immediately — press E-STOP or the rocker — if:**

- any joint reports **CL=1** (clamped) or **JTO=1** (joint timeout)
- the watchdog latches, or a joint detaches on its own
- anything moves that you did not expect, in an order you did not expect
- the claw heads toward the bench, the mat, the loom or the base housing

If it aborts, note which step it was on. The step number and the table are the
whole diagnosis.

---

## Phase 4 — optional, if Phase 3 went cleanly

`storage` and `pick` are marked **OUT OF RANGE** because the elbow lock narrowed
from 0–66 to 0–30 after they were recorded. The arm cannot reach either any
more. Re-record them under today's limits, under new names.

- [ ] **T15** Teach and save `storage-v2` the same way as `park`.
- [ ] **T16** Teach and save `pick-v2`.

The old rows stay. They are history and they were true on the day they were
written; they are simply not targets any more.

---

## Phase 5 — get it into git

Do this before closing anything. A pose that lives only on the Pi is invisible
to git and to anyone else working on this arm.

```bash
Software/arm-console/sync-poses.sh
git diff Software/arm-console/arm-poses.csv
```

- [ ] **T17** The diff is **additions only**. The file is append-only; an edited row means something went wrong.
- [ ] **T18** Commit it.

---

## Powering down

- [ ] Hand under the forearm **before** the rocker. It swings down through the base area on power-off.
- [ ] Rocker off.
- [ ] Leave the hold daemon stopped. It reports EN=1 for joints whose rail is dead, so one left running when the power comes back snaps every joint to a stale setpoint.

---

## Record the run

Fill this in and paste it back. Blank is fine — "did not get to it" is a result.

```
Date/time:
Preflight:        pass / fail  (which check)
Phase 1 T1-T6:    pass / fail  (which test, what happened)
park pose saved:  yes / no     steps recorded: ___
Playback T14:     clean / aborted at step ___ / not attempted
storage-v2:       yes / no
pick-v2:          yes / no
Synced + committed: yes / no
Anything the console said that was wrong or confusing:
```

That last line is the most useful one. The console now makes claims about what
it recorded and what it will save — if any of them turned out not to be true at
the bench, that is a bug worth more than the pose.
