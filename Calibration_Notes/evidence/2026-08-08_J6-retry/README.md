# 2026-08-08 ~20:45 — J6 gripper: articulates, does NOT repeat

First run of the camera-verified harness on the Pi. Superseded tooling, new result.

Harness: `motion_verify_pi.py --joint 6 --dps 10 --roi 300,590,480,719 --film 8`
driven through the holder daemon (`hold_arm_pi.py`, joints 0/3/4/5/6 energised, J1
refused), frames from the Pi's MJPEG `/snapshot` endpoint. Waypoints
`90 → 60 → 90 → 120 → 90`. Camera framing is NOT the 2026-08-06 framing and not
tonight's earlier one — the arm changed pose when the daemon adopted its home angles,
so this ROI was derived fresh at this pose.

## Result 1 — the fingers move. The 2026-08-06 total-slip regression is lifted.

`overlay_60-vs-120_ARTICULATION.png` (green = dark at 60° only, magenta = dark at 120°
only, 5× nearest-neighbour, rotated 180° to read the right way up):

- The **upper prong fringes green along its top edge**; the **lower prong fringes green
  along its bottom edge** — opposite geometric directions, and the gap between them
  changes. Per `arm-motion-verify`, opposite-direction fringing is articulation;
  same-direction fringing everywhere is rigid displacement.
- The **gear teeth at the mesh are magenta** — the pinion is in a new rotational
  position. This is the same gear the operator diagnosed as slipping on the shaft.
- The servo body shows almost no fringe.

Pixel numbers, ROI `300,590,480,719`, threshold 25, no morphological opening:

| leg | signal px | floor px | global shift | verdict |
|---|---|---|---|---|
| 90 → 60 | 1032 | 0 | 0.50 px | MOVED |
| 60 → 90 | 1009 | 0 | 0.13 px | MOVED |
| 90 → 120 | 969 | 0 | 0.65 px | MOVED |
| 120 → 90 | **363** | 0 | 0.61 px | **NOT RESOLVED** |

Every control pair had **distinct frame MD5s** and `capture_dead: false`, so the 0 px
floors are real stability, not a dead capture — the operator taped the backdrop down
before this run, which is what removed the confound that voided the earlier film.

Note the 0 px floor makes the ratio gate vacuous (`ratio=inf` everywhere). The absolute
`MIN_SIGNAL_PX = 400` gate is carrying this run alone. That is the gate working as
designed, but it is worth saying out loud rather than quoting an infinite ratio.

## Result 2 — the commanded angle does not map to a repeatable position

Three visits to the SAME command, 90°:

| pair | changed px | dark px |
|---|---|---|
| 90 (1st) vs 90 (2nd) | 1439 | 10637 → 10613 |
| 90 (2nd) vs 90 (3rd) | 1073 | 10613 → 10376 |
| 90 (1st) vs 90 (3rd) | 1440 | 10637 → 10376 |

**A commanded 30° move produces ~1000 px. Two visits to the identical command produce
more than that.** The silhouette area drifts monotonically in the *closing* direction —
the same direction 60→120 moves it. `overlay_90-1st-vs-90-3rd_NONREPEAT.png` is that
pair.

### The confound was tested, not assumed

The obvious alternative is gravity creep: five joints energised, the arm standing
vertical, and **J1 detached** — the shoulder is not holding anything. So the joint was
parked at 90 and left completely alone for two minutes, capturing every 20 s with no
commands sent:

```
t=  0s     0 px changed   dark 10429
t= 20s     3 px           dark 10408
t= 40s     5 px           dark 10425
t= 60s     3 px           dark 10382
t= 80s     1 px           dark 10379
t=100s     5 px           dark 10423
t=120s     5 px           dark 10409
```

**0–5 px, no trend.** The arm does not creep while it is left alone. The 1000–1400 px
discrepancy is therefore introduced BY the commanded round-trip, not by gravity between
captures.

**The creep confound now has a measured BOUND, not just a null.** A later 900 s watch on
this same ROI (`Calibration_Notes/evidence/2026-08-08_elbow-hunt-watch/`, harness
`Software/tests/hunt_watch.py`) found no oscillation at all — 1600 samples, peak
adjacent-frame 26 px, nothing over the 30 px flag — but it did record **134 px of
cumulative drift versus the start frame over 15 minutes**, with the shoulder detached.
Pro-rata that is roughly **20–30 px across the 2–3 minutes this J6 run took**, against
the **1,073–1,440 px** measured between same-command visits. Creep is therefore an order
of magnitude too small to explain the non-repeatability, and this is now an arithmetic
bound rather than an assumption drawn from a short null. The two-minute test above says
creep is absent at that timescale; the fifteen-minute watch says it is present but tiny.
Both point the same way.

Phase correlation is deliberately not used as the discriminator here: it returns under
0.75 px on every pair *including a real 60° move* (0.16 px), because the large static
servo body dominates the correlation inside this ROI. A measure that reads ~0 on a known
motion cannot be used to rule out motion. The overlays and the idle test carry this.

## What this means, and what it does not

- J6 is **not** the dead joint of 2026-08-06 evening. It articulates.
- J6 is **not** controllable. A joint whose fingers land somewhere different each time
  you command the same angle has no usable position contract, and nothing downstream —
  a grasp width, a pick pose, a LeRobot action — can be built on it.
- The `120 → 90` leg scoring 363 px is consistent with slip on that leg specifically.
- Consistent with the operator's 2026-08-06 hands-on diagnosis, "the gear is slipping
  around the motor shaft", now presenting as partial rather than total slip.

**Not yet done: the operator has not looked.** Per `arm-motion-verify`, `pass=confirmed`
requires eyes on the shaft, and his hands are what produced the root cause last time. The
camera says the fingers move and do not repeat; whether the grub screw / gear is loose on
the output shaft is a thing to check by hand.
