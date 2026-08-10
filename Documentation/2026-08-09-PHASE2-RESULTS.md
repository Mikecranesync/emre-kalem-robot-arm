# Phase 2 results — 2026-08-09

Measuring the arm against a scale, rather than asking "did it move".
Companion to `2026-08-09-POWERED-BENCH-TEST.md` (phase 1).

Run with `Software/tests/arm_phase2_measure.py`, shoulder adopted at 91
(19 changed pixels — no jump). Raw log: `2026-08-09-phase2-raw-log.jsonl`.
Noise floor for the whole run: **992 px** — higher than phase 1's 322, the bench
light had changed.

---

## 2b Backlash — the cleanest result of the day

Three ±10° swings per joint, changed pixels per move:

| Joint | swing 1 | swing 2 | swing 3 |
|---|---|---|---|
| J0 Base | **91** / 9740 | 15768 / 15764 | 15978 / 13977 |
| J4 Wrist pitch | **198** / 6568 | 5463 / 5393 | 5393 / 5381 |
| J3 Elbow | **3779** / 11815 | 11443 / 11355 | 11610 / 12461 |

**The loss is all in the first move after enabling, not in every reversal.**
J0 and J4's first commanded move is effectively dead (91 and 198 px against a
992 floor); J3's is partial, about a third. After that the mechanism is
*extremely* repeatable — J4 returns 5463 / 5393 / 5393 / 5381, inside 1.5%.

Consequence: **the first move after any enable is a take-up move and should be
discarded**, not used for positioning.

## 2c Repeatability — directional, and the gap is large

J3 driven to the same target (15°) alternately from below (7°) and above (23°),
each arrival differenced against the first arrival:

```
arrival 2 (from above)   11035 px    different place
arrival 3 (from below)     982 px    SAME place  (floor = 992)
arrival 4 (from above)   10372 px    different place
```

Approach from the **same** direction and the joint lands in the same place to
within the noise floor. Approach from the **opposite** direction and it lands
~10–11k px away. Textbook backlash hysteresis, and the single most important
number here for teach and playback.

It independently confirms what `arm-poses.csv` already says in its own words —
*"TO LEAVE A POSE, REVERSE ITS OWN ENTRY PATH."* That is not a stylistic
preference; it is the only reason this arm is repeatable at all.

J1 was skipped: parked at 91 with `MAX=91`, no room either side. Another
consequence of HOME sitting on a limit.

## 2e Hold quality — hunting confirmed

Parked, all joints enabled, 45 s:

```
peak adjacent-frame    9571 px     (floor 992)
peak drift vs start    9983 px
```

About **10x the floor on both channels**. The arm does not sit still while
holding: it moves frame-to-frame *and* creeps away from where it started. This
matches the complaint that started this whole thread — "very bouncy going to its
position".

**Caveat, stated plainly:** the ROI is whole-arm and the run was unattended, so a
person or a lighting change in frame would also register. Repeat with the bench
confirmed empty before calling this a hard fault.

## 2f Gripper — not inert, directional

| Command | Δ | px | Moved? |
|---|---|---|---|
| 90 → 130 | +40 | 20 | no |
| → 90 | — | 177 | no |
| 90 → 50 | −40 | 2348 | **yes** |
| → 90 | — | 5697 | **yes** |

**The gripper moves toward lower angles and not toward higher ones.** A gear
simply slipping on its shaft would fail in both directions. A one-way failure
points instead at binding, a mechanical stop, or slipping only under the load of
one direction. Worth an eyes-on check of the horn while commanding +40.

## 2a Direction and magnitude — METHOD FAILED, data discarded

The ladder was meant to show pixels scaling with commanded degrees. It does not:

```
J4 wrist pitch:   5° -> 11820 px     10° -> 572 px     20° -> 3850 px
J0 base:          5° ->  5835 px     10° -> 409 px     20° -> 8600 px
```

Non-monotonic, and the same 5°-big / 10°-tiny shape repeats across joints, so it
is systematic rather than noisy.

**Cause:** the ROI covers the whole arm and there is **no scene-quiet gate
between ladder steps**, so each reading also caught the previous return-to-home
still settling. Per-joint, per-step attribution is therefore untrustworthy and
none of these numbers should be quoted.

The quiet gate already exists in `arm_bench_test.py` and simply was not carried
into the ladder. Fix for the re-run: gate on quiet before every step, and use a
per-joint sub-ROI so one joint's motion cannot be credited to another.

## 2d Reversal braking — not attempted

The braking ramp is a sub-second event and `/snapshot` delivers ~2.5 fps. It
cannot be sampled with this instrument. FW 1.2.0's reversal behaviour remains
proven only by the host-compiled property test in `interpolator_check.py`.
Observing it on hardware needs a higher-rate capture or an audio channel.
Deferred, not faked.

---

## Verdict

```
2a direction/magnitude   METHOD FAILED — needs quiet-gate + per-joint sub-ROI
2b backlash              PASS — quantified; dead first move after enable
2c repeatability         PASS — same-direction repeatable to the floor;
                                opposite-direction is NOT
2d reversal on hardware  NOT TESTABLE with a 2.5 fps camera
2e hold quality          HUNTING CONFIRMED (~10x floor) — repeat with bench empty
2f gripper               REFINED — one-way motion, not inert
```

**Teach/playback: still NO** — but the blocker is now specific rather than
vague. Poses must be approached from a consistent direction, and the first move
after an enable must be discarded.

## Phase 3, proposed

1. **Re-run 2a properly** — quiet gate per step, per-joint sub-ROIs. Needed
   before any claim about direction mapping can be made from pixels.
2. **Re-run 2e with the bench confirmed empty** and better light, to separate
   real hunting from scene contamination.
3. **Quantify hysteresis in degrees, not pixels** — currently we know
   same-direction is repeatable and opposite-direction is not; we do not know by
   how much. That number decides whether playback needs a one-way approach rule
   or genuine backlash compensation.
4. **Gripper teardown** on the one-way finding.
5. **Re-teach HOME a few degrees inside its limits** — J1 at 91/91 and J5 at
   180/180 leave nothing to back off to, and it already cost us the J1 arm of
   the 2c test.
