# Evening session findings — 2026-08-06

Companion to `2026-08-06-SESSION-FINDINGS.md` (the daytime session). That one ends
with the gripper newly proven and five joints locked. This one starts there, and the
first thing it does is take the gripper away again.

**Headline: every joint on this arm with a live servo is now camera-verified, the
gripper has regressed to a mechanical fault, and the arm cycles pick↔storage
cleanly at the firmware's top speed.**

---

## 1. Per-joint verification

Every result below is camera-verified, not inferred from a board ack. The pass bar
was fixed *before* each run: changed pixels inside the joint's ROI must beat that
same waypoint's own noise floor by ≥4×, where the floor is two frames 0.5 s apart at
the **same** command.

| Joint | Pin | Verdict | Signal px | Noise floor | Ratio |
|---|---|---|---|---|---|
| **J1 shoulder** (mirrored MG996R pair) | D4+D5 | **verified** 2/2 then 4/4 | 24707–33204 | 2–44 | 754–16369× |
| **J3 elbow** | D6 | **verified** 4/4 | 15963–17807 | 2–40 | 421–7981× |
| **J4 wrist pitch** | D9 | **verified** 4/4 twice | 1852–3390 | 0–27 | 96–3390× |
| **J5 wrist roll** | D10 | **verified** 3/3 twice | 10221–14362 | 0–20 | 600–14362× |
| **J6 gripper** | D11 | **REGRESSED — not articulating** | 21–609 | 1–677 | never clears |
| J0 base | D3 | dead servo, excluded | — | — | — |

`JTO=0` on every waypoint of every run. Every `MOV` returned `CL=0`.

**J3's result resolves an old open item:** the 04:43 calibration row noted that its
lock "contradicts the earlier no-motion report for the elbow." The elbow moves.

**J1's result is the strongest evidence available that the mirrored pair is not
fighting**, short of actually measuring the offset: positions repeat to within a
couple of pixels across two cycles (at 12°, upper-arm-box area 6310 then 6167,
bbox_h 133 then 135; back at 1°, area 8357/8153/8185, bbox_h 124/124/124), with no
stall, timeout, buzz or creep. Two MG996R in opposition would show exactly those.
**`mirror_offset_deg` is still 0 and still unmeasured** — repeatable motion over 11°
does not measure the true mirror axis.

**Scope of what was actually driven** (do not over-read the table): J1 only 1–88,
J3 only 18–64, J4 only 75–170, J5 only 60–165. Locked envelopes are not re-verified
by these runs.

---

## 2. The gripper regressed, and it is mechanical

At 14:53 the operator watched J6 open and close and the camera measured 1799/1657
changed px. By ~19:00 the same commands produced no finger movement at all. Four
independent lines agree, and none of them is a measurement failure:

1. **Frame differencing** — commanded 10 vs 70 gives 21–609 changed px against
   floors of 1–677. Never clears the bar except where the floor was freakishly quiet.
2. **Geometry, illumination-independent** — the prong silhouette is *identical* at
   both commands: area 3226/3264/3244/3274, bbox height 79/80/79/80, median finger
   span 61/61/62/61.
3. **A red/green channel overlay** — every edge of the whole assembly, both prongs
   plus body plus forearm, fringes the *same* direction (green top, magenta bottom).
   That is a uniform ~1 px downward sag. Articulating fingers would fringe in
   **opposite** directions and change the gap between the prongs.
4. **J4 and J5 resolve at 96–14362×** through the same harness, camera and ROI scale
   in the same session.

**Firmware is exonerated.** `STA` sampled through the travel shows `SET` ramping
13-15-18-21-24-26-31-34-37-39-42-46-49-53-56-59-65-68-70 with `MOV=1` throughout,
~5.9 °/s against a commanded 6. The pulse train on D11 is correct.

**Operator's diagnosis**, after checking by hand that it was not bound up and not
off the gear, and moving it manually: *"I think the gear is slipping around the
motor shaft."* Consistent with `wiring-map.csv`'s own `first_test_note` for this
joint — "Linkage loose at first."

**Do not record J6 as controllable until it is fixed and re-verified.** The
`confirmed` row from 14:53 stays true as of 14:53; a dated `REGRESSED-was-confirmed`
row supersedes it.

---

## 3. Poses — `Software/arm-console/arm-poses.csv`

| pose | J1 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|
| `storage` | 88 | 64 | 90 | 104 | 70 cmd |
| `pick` | 8 | 36 | 140 | 165 | 70 cmd |

`entry_path` is a **column, not a comment**: two safe poses can have an unsafe
straight line between them. To leave a pose, reverse its own path. Do not
interpolate between rows.

**Direction mapping, hard-won:**

- **J1 shoulder** — HIGHER folds it over *onto the base*; LOWER reaches *out*.
- **J3 elbow** — HIGHER folds/tucks and tilts the forearm *down*; LOWER extends.
  Re-test its screen direction after any large J1 change; the frame of reference
  rotates with the shoulder (~87° here).
- **Aiming the claw down is ROLL, not pitch.** J3 tucked gives the fingers-down
  geometry, then J5 rotates the claw to vertical (104 ≈ 45° off, 130 hangs down,
  165 vertical at the level-forearm pose). J4 pitch rotates the claw mostly *out of
  the camera plane* in these configurations — 80° of commanded pitch moved it ~11°
  in frame — so pitch alone cannot aim it.
- **Lowering the pick pose while keeping the forearm level is nearly out of room.**
  Higher J3 tilts the forearm down, so dropping J1 must be matched by dropping J3,
  and J3 bottoms at 0. Compensating the other way (J1 down + J3 up) *retracts* the
  arm upward — tested and photographed as `lowering_needs_J3_DOWN_not_up.png`.

---

## 4. Cycling pick ↔ storage

16 cycles total, **all clean** — no clamp, no joint timeout, no watchdog latch,
every joint reaching its commanded angle every time.

| dps | round trip |
|---|---|
| 12 | 30.2 s |
| 15 | 25.6 s |
| 25 | 18.9 s |
| 40 | 13.7 s |
| 60 | 11.0 s |
| **90** | **9.8–10.5 s** |

**90 dps is the firmware ceiling** — `ERR E12 SPD JOINT=1 REQ=120 MIN=1 MAX=90`.
Note 40–90 exceeds the `max_deg_per_sec=30` recorded per joint in
`joint-limits.csv`; that column is recorded intent, not a firmware limit, and
running above it is a deliberate act.

**Repeatability does not degrade with speed, and the naive reading said it did.**
The run's own drift column climbed 2089 → 10972 px, which looks like the arm losing
position as it speeds up. It is not — every frame was compared against a reference
captured five minutes earlier, and this arm sags while held. Measured cycle-to-cycle
at each speed instead (same speed, ~20 s apart, elapsed time controlled):

| dps | storage c1↔c2 | pick c1↔c2 |
|---|---|---|
| 15 | 5479 | 2089 |
| 25 | 4462 | 5661 |
| 40 | 5499 | 3932 |
| **60** | **2611** | **2395** |
| **90** | **2884** | **4002** |

60 and 90 are the *tightest*; 15 is no better. Five consecutive runs at 90 dps then
showed no degradation across repetitions either (pick adjacent pairs 3563 / 4244 /
1328 / 4075; storage 5979 / 3223 / 2120 / 2572 — flat and scattered, not climbing).

**Endurance is untested.** Five cycles proves repeatability, not what a hundred runs
at 90 dps does to MG996R gears on a 5 A rail.

---

## 5. The recurring measurement trap — read this before trusting any number

**Four times tonight a measurement saturated on something that was not the thing
being measured, and each time the number looked like a real physical finding.**

| What read wrong | Why |
|---|---|
| `bbox_h` frozen at 202 px across every pose | the ROI was filled edge to edge; the metric had no room to move |
| `lowest_y` frozen at 609 | pinned on the ROI's own bottom edge |
| a "claw" box that never changed | it was catching the black stand foot on the mat, not the claw |
| a J5 sweep reading −8° "nearly vertical" | the claw had swung *out* of the box; the figure came from a clipped fragment |
| endpoint drift "rising with speed" | frames compared against a reference five minutes old, measuring sag |

**Rule: when a measure stops changing, suspect the measure before believing the arm
stopped. Look at the image.** Every one of these was caught by looking, and none by
staring harder at the number.

The same asymmetry governs the harness's gates: a false positive records a wrong
claim as fact, a false negative costs a re-run. Gates are deliberately strict, and
a pixel/geometry disagreement reports `DISAGREES` rather than asserting a diagnosis.

---

## 6. What was built

| Path | What it is |
|---|---|
| `Software/tests/motion_verify.py` | camera-verified motion for one joint through named waypoints; control frames, two pixel gates, a blocking geometry gate, `--film` for travel windows |
| `Software/tests/cycle_poses.py` | phase-ordered pick↔storage cycling with speed escalation and endpoint repeatability |
| `Software/arm-console/arm-poses.csv` | named poses **plus their entry paths** |
| `Software/arm-console/hold_arm.py` | the holder daemon — now in the repo instead of a temp dir |
| `.claude/skills/arm-motion-verify/` | the verification doctrine |
| `.claude/skills/arm-serial-control/` | commanding the arm without dropping it |
| `.claude/skills/arm-bench-safety/` | no-software-e-stop, hands-clear, conservative limits |

Two defects in the harness were found by its own runs and fixed: duplicate waypoint
labels overwrote each other's evidence, and a ratio-only gate scored a 70 px edge
shimmer as "35× MOVED" with nothing articulating. A third — the harness reporting
the broken gripper as `2/3 MOVED` — was fixed by making geometry a blocking gate.

---

## 7. Where I was wrong

Recorded because each was reported as fact before being corrected.

1. **The shoulder direction, inverted.** I read it from the claw's pixel height,
   which *rises* as the arm folds onto the base, and told the operator the opposite
   of the truth. A flat projection cannot distinguish rising from folding. He read
   the 3D geometry correctly. **When the projection and the operator disagree about
   direction, the operator wins.**
2. **The overlay colour labels, swapped.** Green is dark in the *first* frame, not
   the second. Caught by a subagent reviewing the work. The J6 conclusion was
   unaffected — it rests on all edges fringing the *same* direction, which is
   colour-agnostic.
3. **"Repeatability degrades with speed."** It does not; that was the sag.
4. **Dismissing the operator's chosen pick pose** as "retracted upward" because I
   was optimising for a level forearm. He picked it anyway and was right — claw
   geometry over the work surface matters more.
5. **A malformed CSV row** — appended a comma-bearing field unquoted, which split it
   into 14 columns and silently made `pass` read "yes". Caught by a field-count
   validator, then re-appended through `csv.writer`.

---

## 8. Open at end of night

- **J6 gripper** — gear slipping on the motor shaft. Not controllable. Mechanical fix owed.
- **J0 base** — dead servo; replacement ordered. Re-measure when fitted; a new horn
  seats on a different spline tooth, so 29–110 will not transfer.
- **`mirror_offset_deg`** still 0 and unmeasured.
- **J4's travel ends** still UNCONFIRMED (0–180 is the whole electrical range);
  J5's 178 may be electrical rather than mechanical.
- **Camera uncalibrated** — every verdict here is motion / no-motion. No angle is
  derived from pixels. Do not print markers.
- **Endurance at 90 dps** untested beyond five cycles.
- **The arm is not self-supporting in any pose.** All five live joints detach the
  instant the daemon stops or the rocker goes off. Hand under the forearm, then the
  switch.

---

## 9. Postscript — how the session actually ended (21:55)

The holder daemon **crashed** rather than being stopped cleanly:

```
serial.serialutil.SerialException: WriteFile failed
  (PermissionError(13, 'The device does not recognize the command.', None, 22))
  ... in send: ser.write((line + "\n").encode("ascii"))
```

Cause was benign and external: the operator powered the bench down for the night.
`Get-CimInstance Win32_SerialPort` afterwards returns **no ports at all**, so the
board's USB serial device was gone — the daemon was mid-`PNG` heartbeat when it
vanished. All five joints detached at that instant. The arm was already parked
folded at `storage`, which is exactly why that pose was chosen: the drop is short
and away from the loom.

**Two things learned that were not known before:**

1. **`hold_arm.py` does not survive a serial disconnect** — it takes the exception
   out of `main()` and exits. That is arguably correct for an unplug (there is
   nothing to hold once the board is gone) but it means the process disappears
   silently, and `arm_status.txt` is left behind showing `EN=1` on every joint.
   **A stale `arm_status.txt` reads exactly like a healthy held arm.** Check the
   file's mtime, or that the PID is alive, before believing it. This is the same
   class as every other trap in section 5: a stale artifact that looks like live data.
2. The last recorded live state before the disconnect was the intended park —
   J1 88, J3 64, J4 90, J5 104, `ES=0 WD=0`, **zero watchdog latches across the
   entire ~3.7 hour session**.

**Next session:** the arm's physical resting position after the detach has NOT been
observed — the camera had been repositioned by then. Look at it before commanding
anything, and expect the adopt angles in `hold_arm.py` to be estimates of where the
joints *are*, not where they were left.
