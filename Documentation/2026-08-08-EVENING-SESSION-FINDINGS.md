# 2026-08-08 evening — session findings

Companion to `2026-08-06-EVENING-SESSION-FINDINGS.md`. Same bench, same arm, a
different machine: the arm was disassembled, recalibrated and rewired between the
two sessions, and the Uno moved from the Windows laptop to the Pi. **Most of what
the repo asserted about this arm on 08-06 stopped being true on 08-08 and did not
say so.** That is the headline finding, and it cost more of this session than any
mechanical fault.

---

## 1. The stale-authority problem

Every file below reads as current fact and is not. Anyone resuming will believe
them unless they are fixed:

| File | What it still asserts | Truth on 08-08 |
|---|---|---|
| `Software/arm-console/arm-poses.csv` | "J0 IS ABSENT FROM EVERY ROW ON PURPOSE. Its servo is dead" | Replaced 08-06, re-locked 0–180 on 08-08, driving fine |
| same | "As of 2026-08-06 evening the gripper does not articulate" | It articulates — proven this session, §3 |
| same | `storage` needs `J3 = 64` | Elbow re-locked 0–30 on 08-08; 64 clamps |
| `.claude/skills/arm-motion-verify` | gripper ROIs `330,425,485,565` and `325,455,425,560` "calibrated for this camera position" | Camera has moved twice since; both boxes now **fail**, §2 |
| `Software/arm-console/hold_arm.py` | elbow `0-66 adopt 33`, wrist pitch `0-180`, J0 "servo is DEAD", enables J1 | All four wrong; superseded by `hold_arm_pi.py` |
| `.claude/skills/arm-serial-control` §8 | old pin map | Firmware 1.1.1 is elbow D10 / pitch D6 / roll D9 |

**The lesson is structural, not clerical.** A hand-maintained copy of a measured
value drifts silently the moment the measurement changes. `hold_arm.py`'s limits
dict is the worst case — it had drifted in *both* directions at once, holding the
elbow 36° too wide (0–66 against a real 0–30) and the wrist pitch 33° too wide.
Commanding a joint into a region already measured as unreachable is precisely the
failure that stopped J0 driving in August. The fix was not to correct the dict but
to **delete it and read `joint-limits.csv`** — the file that is already the source
of truth. Prefer reading the authority over copying it.

---

## 2. Camera measurement: three ROIs, all void, and the test that catches it

A 75 s gripper film scored **65 of 96 intervals as MOTION, peaks 80,263 px**, and
none of it was the gripper. The white paper backdrop and its shadow were sweeping
across the scene and through the ROI. Full write-up:
`Calibration_Notes/evidence/2026-08-08_J6-film-void/`.

Then, testing the ROIs properly, **all three boxes this project has ever used fail
a backdrop negative control**:

| box | negative control |
|---|---|
| ad-hoc grader `700,350,900,480` | **5,978 / 4,318 px** → FAIL |
| skill gripper diff `330,425,485,565` | 417 / 232 px → FAIL |
| skill gripper geo `325,455,425,560` | 200 / 75 px → FAIL |

Worse: at the current camera position the skill's *geometry* box contains **zero
arm pixels** — 100 dark px, all in one row, a 1 px artifact line. It has been
measuring nothing and reporting a verdict about it.

**The missing test, now built:** disturb the scene, command nothing, and require
the candidate ROI to score near zero. This harness had gates for "is the signal
big enough" and "did the shape change" but never "does this box see things that
are not the joint". Add it before trusting any new ROI.

Two further measurement traps recorded so that they are not rediscovered:

- **A saturated metric silently carries no information.** A naive "topmost dark
  pixel" apex pinned at 0 whatever the arm did, because a 1 px prong tip reaches
  the frame edge. Same class as the skill's existing `bbox_h` warning. Use the
  topmost row with ≥20 px.
- **Phase correlation is not a motion discriminator inside a body-dominated ROI.**
  It returned under 0.75 px on every pair *including a real 60° move* (0.16 px),
  because the large static servo body dominates. A measure that reads ~0 on a
  known motion cannot be used to rule out motion.

---

## 3. J6 gripper: articulates, does not repeat

The 08-06 total-slip regression is **lifted** — opposite-direction prong fringing
and the gear teeth in a new rotational position. But three visits to the *same*
commanded 90° differ by **1,073–1,440 px**, more than a commanded 30° move
produces, drifting monotonically closed.

**Gravity creep was tested rather than assumed** — twice. Parked and left alone for
120 s: 0–5 px, no trend. A later 900 s watch: no oscillation at all, but **134 px
of cumulative drift**. Pro-rata that is ~20–30 px over the J6 run's 2–3 minutes,
against a 1,073–1,440 px effect. Creep is an order of magnitude too small, and
that is now an arithmetic bound rather than a short null.

So: J6 moves and is **not controllable**. A joint whose fingers land somewhere
different each time you command the same angle has no position contract, and
nothing downstream — a grasp width, a pick pose, a LeRobot action — can be built
on it. Full evidence: `Calibration_Notes/evidence/2026-08-08_J6-retry/`.

---

## 4. The elbow "bounce" was a wrong question, correctly caught by asking

The operator reported a servo bouncing and asked for a "smoothing program". The
firmware's interpolator *is* a rectangular velocity profile — constant `dps`, no
acceleration limit, dead stop at target — so a motion-profile fix looked obvious
and defensible.

**It was the wrong fix.** Asked *when* the bounce happens, the answer was **while
the joint is holding still**, on the elbow. Smoothing shapes a joint while it
travels and does nothing for a parked one. The symptom is hunting — a servo whose
internal loop cannot hold position in its deadband under load.

Two supporting facts: the elbow is an MG996R on a **5.0 V** rail, the bottom of its
window, and `joint-limits.csv` already predicts "roughly 10-15 percent less holding
torque"; and J1 was detached, so the elbow was carrying more of the arm than it
should. Ranked fixes — supply voltage (5.0 → **6.0 V max**, the MG90S ceiling),
mechanical unloading, then getting J1 to carry its share.

A hypothesis floated as "a one-line fix waiting to be confirmed" — that the
daemon's 5 s `STA` poll was jittering the AVR's servo pulses — was **ruled out**:
sampling at 0.4 s for 900 s would have caught a 5 s periodic excursion 180 times
and caught none. Recorded so it is not floated again.

---

## 5. Hardware and protocol facts worth not rediscovering

- **`LIM` is refused on a live joint.** Verified in firmware, not inferred from a
  comment: *"you cannot move the goalposts under a live joint."* Widening an
  envelope therefore requires `DIS`, which drops that joint — on a gravity-loaded
  arm that means a hand on it first.
- **`ENA j <adopt>` drives the joint to the adopt angle the instant it attaches.**
  The pulse is pre-loaded before `attach()`. `LIM` does not protect you: it clamps
  commanded values, not where the servo physically goes on adopt. If nobody knows
  where a detached joint is sitting, only the operator's hand makes this safe.
- **`STA` reports `SET`, the commanded angle — never a measured one.** Nothing on
  this arm observes an output shaft. Every "lock" is a record of a decision.
- **Firmware 1.1.1 never emits `CL` on a `STA` line.** `motion_verify.py` read it
  from there, so every move reported `clamped=false` including clamped ones. `CL`
  lives on the `MOV` reply. Fixed in `motion_verify_pi.py`; **the original still
  has the bug.**
- **`arm-bridge.py` deliberately does not feed the watchdog** — the browser sends
  `PNG` every 250 ms so that a crashed tab stops the arm. Consequence: a script
  driving through `/tx` with no console open leaves joints dropping mid-run. The
  daemon must own the port.

---

## 6. Rig limitation: the camera cannot see the working end

In every pose the arm could reach tonight, the claw sat **above the top of the
frame**. 40° of wrist pitch would not bring it back. The camera sees the arm's body
but not the thing being measured, which makes distal-joint verification blind.

Compounding it, `arm-poses.csv` already warns that a flat projection **lies about
fold direction**: *"the claw RISES in frame as the arm folds onto the base... when
the two disagree, the operator wins."* Confirmed tonight — the fold metric read
"flat" across a 46,000 px change.

Before the next teach or reteach session, **move the camera back or up** so the
whole arm including the claw is in frame. Until then the operator's eyes and ears
are the instrument for anything distal, and rows should say so.

---

## 7. Process lessons

- **Ask *when* and *where*, not just *what*.** One question about when the bounce
  happened turned a firmware rewrite into a supply-voltage conversation.
- **Deliver the outcome, not the instrument.** Asked to stow the arm, this session
  hit a blocked limit, pivoted to reteaching every joint, and *unfolded the arm* to
  get camera visibility for the reteach — reversing its own progress. The tell was
  undoing a change just made. Stop and re-read the goal at that moment.
- **Verify the artifact, not the exit code.** A backgrounded ssh wrapper exited 255
  and 1 on two occasions while the process it launched was running fine. Both times
  the answer came from `pgrep` and the log, not the status.
- **A null in one pose and one window is not a refutation.** The hunt watch caught
  nothing; that does not mean the operator did not see it.
