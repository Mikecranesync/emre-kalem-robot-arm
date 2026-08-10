# Powered bench test — 2026-08-09

First camera-supervised powered test of the arm after FW 1.2.0 was flashed.
Two incidents, six joints characterised, HOME reached. Everything below is
observation; where something is inferred it says so.

**Outcome:** the gripper is the only joint that does not work. The operator said
so before the test and the camera confirmed it independently.

---

## Result

```
POWER-UP:              PASS (2nd attempt)   — see Incident 1
CAMERA:                PASS
HOME/PARK:             PASS   J0=110 J1=91 J3=15 J4=90 J5=180 J6=90
J0 Base:               PASS   with backlash — first move after enable is dead
J1 Shoulder (pair):    PASS
J3 Elbow:              PASS
J4 Wrist pitch:        PASS   direction asymmetry, see below
J5 Wrist roll:         PASS   parks ON its limit at HOME
J6 Gripper:            FAIL   never clears the noise floor
DUAL-SERVO SHOULDER:   PASS   no evidence of the pair fighting
WATCHDOG/COMMS:        PASS   fired correctly; comms clean throughout
READY FOR TEACH/PLAYBACK:  NO — gripper only
```

Per-joint camera evidence, changed pixels against a 322 px floor:

| Joint | out+ | back+ | out− | back− | Reading |
|---|---|---|---|---|---|
| J1 Shoulder | 37\* | 2773 | 3487 | 10385 | moves cleanly both ways |
| J3 Elbow | 17080 | 7737 | 7156 | 7094 | best joint on the arm |
| J4 Wrist pitch | 12594 | 851 | **187** | 964 | strong one way, weak the other |
| J5 Wrist roll | —\* | —\* | 1474 | 1498 | moves; cannot go up, at limit |
| J0 Base | 134 | 7019 | 52 | 258 | dead band; proven at 10° |
| J6 Gripper | 534 | 413 | 96 | 51 | **never clears the floor** |

\* Not faults. Clamped moves: J1 sat at 90 with `MAX=91` (a 1° command) and J5
at 180 = `MAX` (a 0° command). Both were reported FAIL by the first harness,
which is a harness defect and is now rule 5.

**J0 disambiguation.** Re-tested at 10° rather than 3°: `301 / 10562 / 8954 /
16540` px. The base moves; the first commanded step after an enable is
absorbed by backlash. The 3° "failures" were inside the dead band.

---

## Incident 1 — the arm was dropped

**What happened.** With every joint detached, the arm was standing on gearbox
friction alone. The test enabled **J0 (the base)** first. The attach jolt broke
static friction on the still-detached shoulder and the arm collapsed onto the
bench.

**Why it was not noticed immediately.** The harness measured its "noise floor"
during the fall and recorded **36486 px** as normal. Every later step was then
judged against an impossible bar, so a real failure read as a pass. The fall was
found by *looking at the picture*, not by the numbers.

**Cause.** Enabling a joint that does not carry the load, while the joint that
does was detached. Nothing was holding the arm.

**Fixed by** rule 2 (shoulder captured first) and rule 4 (a scene must be
demonstrably still before any floor is trusted; an implausible floor is
rejected, not adopted).

## Incident 2 — the shoulder was driven over centre

**What happened.** With the arm collapsed, the harness adopted J1 at `MIN=0`,
reasoning from this file's own direction note — *"J1 HIGHER = folds onto the
base, LOWER = reaches out"* — that a collapsed arm must rest at low J1. It does
not. 0 is the **over-centre** end. The operator stopped the run.

**The calibration file agreed with the wrong answer.** `joint-limits.csv` had
`home_deg=1` for J1, so even "go to home" would have driven it over centre.

**Cause.** An adopt angle was *inferred* on the one joint where being wrong is
expensive. The direction note explicitly warns that the camera projection reads
backwards here and that *when the two disagree, the operator wins* — and it was
still used as the basis for an inference.

**Fixed by** rule 3: `--shoulder-deg` is a required argument supplied by a
human, and `joint-limits.csv` J1 `home_deg` corrected `1 → 90` with the
operator's statement recorded in the row.

Adopting at 90 was then verified: **21 changed pixels against a 322 px floor.**
That is what a correct adopt angle looks like.

---

## What landed in the repo

| File | Why |
|---|---|
| `Software/arm-vision/mjpeg_preview.py` | The camera tool existed **only on the Pi**. Its 180° rotation fix — which is what finally made the arm identifiable in frame — would have been lost. Now versioned. |
| `Software/tests/arm_bench_test.py` | The harness, with all six rules enforced in code rather than described in prose. |
| `Software/arm-console/joint-limits.csv` | J1 `home_deg` 1 → 90, evidence in the row. |

The six rules and the failure each one prevents are documented at the top of
`arm_bench_test.py`. They are scars, not preferences.

---

## Open items

1. **J4 wrist pitch asymmetry** — 12594 px one way, 187 the other. Looks like
   backlash in the joint that drives the small printed gear. Not characterised.
2. **J1 and J5 both park ON their limits at HOME** (91/91 and 180/180). A joint
   at its own end has nowhere to back off to — the mistake this file's own rows
   warn about. HOME arguably wants re-teaching a few degrees inside.
3. **Gripper** — known mechanical fault, gear slipping on the motor shaft
   (`Calibration_Notes/evidence/2026-08-08_J6-retry/`). Confirmed dead by camera.
4. **Bench light** — the camera still reports `TOO DARK` (52/255, ~60% dark).
   Everything worked, but the noise floor is light-bound; more light means a
   lower floor and smaller detectable movements.

---

## Phase 2 — proposed

Phase 1 answered *"does each joint move when told, and does HOME work."* It did
not answer *"does it move the right amount, repeatably, in the right
direction."* Nothing here has ever measured a joint against a scale.

**2a. Direction and magnitude, per joint.** For each joint drive a known ladder
(±5, ±10, ±20°) and record changed pixels and the centroid shift of the moving
region. Establishes *which way is positive on screen* — settling the direction
confusion that caused Incident 2 with data instead of prose — and whether pixel
movement scales with commanded degrees.

**2b. Backlash, quantified.** J0 and J4 both show a dead first move. Command
+N then −N repeatedly and measure the lost motion. Produces a number per joint
instead of "seems to have backlash", and tells us whether the wrist gear is
getting worse.

**2c. Repeatability.** Drive to the same target from both directions, ten times,
and compare the settled frames. This is the number that decides whether teach
and playback is worth anything, and it is measurable with the camera we have.

**2d. Reversal, on real hardware.** FW 1.2.0's reversal braking is proven in the
host-compiled test but has **never been observed on the arm**. Command a
mid-move target flip and confirm the joint eases through zero rather than
slamming. This is the change that exists to protect the wrist gear.

**2e. Hold quality.** The original question that started all this — does a
joint hunt or sag while holding? Now answerable: park each joint mid-travel,
enabled, and watch for several minutes against a proper floor. Needs the bench
light.

**2f. Gripper root cause.** Camera says the commanded gripper produces no
motion. Determine whether the servo turns at all (watch the horn, not the
fingers) to separate "gear slipping on the shaft" from "servo not driving".

**Not in phase 2:** teach/playback of new poses. That waits on 2c, and on the
gripper.
