# Phase 3 — backlash compensation: the premise did not survive measurement

**Outcome: no compensation was built, because measurement says there is nothing
to compensate.** Two independent methods agree that J3 has no meaningful
directional backlash, and Phase 2's headline hysteresis result was an artifact of
its own missing settling gate. The hunting result failed its control too.

This is a negative result with trustworthy measurements. It is recorded in full
rather than quietly dropped.

---

## What Phase 2 claimed, and what actually happened

| Phase 2 claim | Phase 3 measurement | Verdict |
|---|---|---|
| Opposite-direction arrivals split by ~10,000 px | 1442 px median, overlapping same-side | **artifact** |
| Same-direction lands at the noise floor (982) | 977 median vs a 403 floor | roughly holds |
| Arm hunts while holding (~10x floor) | detached joint moved *more* than driven | **not demonstrated** |

## 1. Backlash by onset sweep — under 1 degree

Dial-indicator method: load the gears one way, reverse, step 1° at a time, and
find the first settled frame that differs from the previous settled frame by
more than the ROI noise floor. No pixel-to-degree scale is needed, which is
exactly why it works where Phase 2's ladder failed.

```
J3, park 15 deg, ROI [221,28,372,346], floor 131 px, threshold 262
  up    onset at 1 deg  (1253 px)
  down  onset at 1 deg  (2224 px)
  up    onset at 1 deg  (1725 px)
  down  onset at 1 deg  (2211 px)
  up    onset at 1 deg  ( 655 px)
```

**Every single 1° step produced motion 2.5–8x over threshold.** J3's dead band is
below 1°, which is the resolution limit of a whole-degree command. There is no
lost motion to add back.

This alone kills CNC-style offset compensation for this joint: there is no
dead band to cross, so adding degrees at a reversal would simply command the
wrong angle.

## 2. The hysteresis control — Phase 2 measured its own settling

Phase 2's 2c had **no scene-quiet gate**, the same defect that invalidated 2a.
Repeated properly — every arrival quiet-gated, measured inside J3's own derived
ROI:

```
floor            403 px
same-side        [1784, 896, 2711]  and  [438, 1059, 752]   median  977
opposite-side    [51, 2183, 701, 2901]                       median 1442
```

The opposite-side arrivals **overlap the same-side range** and include values
(51, 701) well below the same-side median. Phase 2's 10,000 px split does not
reproduce. It was arrivals caught mid-settle: the approach from further away
was still moving when it was photographed.

What remains is real but direction-independent: same-side scatter of ~977 px
median against a 403 floor. Arrivals are **not** tightly repeatable — but the
error is not a function of approach direction, so preferred-direction approach
would not remove it.

## 3. The hold control — hunting not attributable to the servo

Driven, holding one unchanging command for 75 s:

```
J3 enabled at 15 deg   floor 129   peak adjacent 2992 px  (23x)
                       distinct (SET,TGT) pairs: [("15","15")]  -> host innocent
```

The host was proven not to be rewriting targets — a single SET/TGT pair for the
whole window. So the motion is not competing commands.

But the control says it is not the servo either:

```
J3 DETACHED, same ROI   floor 471   peak adjacent 4644 px  (9.9x)
```

**The unpowered joint moved MORE in absolute pixels than the powered one.** If
the servo were hunting, powered would be worse than unpowered. It is not.

Note also the floor moved 129 -> 471 between two runs minutes apart. Ambient
scene noise varies by 3.6x run to run, which is the same order as the effect
being chased.

## Conclusion

On the evidence available:

- **J3 backlash: < 1 degree.** Nothing to compensate.
- **Directional hysteresis: not present.** The Phase 2 number was an artifact.
- **Hunting: not demonstrated to be servo-driven.** Fails its own control.
- **Arrival scatter is real (~2-6x floor) but direction-independent**, so it is
  not backlash and preferred-direction approach would not fix it.

Building and tuning a compensation system against these numbers would be fitting
to noise. The correct engineering answer is to say so and fix the instrument
first.

## What was built anyway, and is worth keeping

`Software/tests/arm_precision.py` — imports `Link`/`Bench` from
`arm_bench_test.py` so the six safety rules are shared rather than forked, and
adds:

- **onset sweep** — backlash in degrees, no pixel scale required
- **self-derived per-joint ROIs** from real motion, via morphology + largest
  connected component. Anatomically sensible and stored as calibration:
  J1 `[116,289,372,602]` area 20332, J3 `[221,28,372,346]` 15525,
  J4 `[267,27,372,220]` 6706, J5 `[279,0,372,244]` 7318 — the nesting matches
  the kinematic chain, which is itself a check that the derivation works.
- **deliberate priming** — spends the known first-move take-up on purpose so a
  precision move is never the sacrificial one
- **quiet-gated, sanity-capped ROI floor.** The first version of this repeated
  the Phase 2 bug in a new function and returned 3860 px on a 48k-pixel ROI —
  four times the whole-arm floor, which is impossible for a smaller box. It had
  caught the arm settling after priming. Now gated and capped against the ROI's
  own area.
- **controls for both headline claims** — `repeat` (gated hysteresis) and
  `holdoff` (detached hold). Both are what turned a confident result into a
  negative one.

Compensation code was **not** written. There is no measured effect for it to act
on, and a preferred-direction approach layer tuned against this data would
encode noise as calibration.

## What would make these effects measurable

The binding constraint is the instrument, not the arm:

1. **Light.** The camera still reports `TOO DARK` (~52/255). The noise floor is
   light-bound and it drifted 3.6x between two runs minutes apart.
2. **A fiducial marker on the arm.** Changed-pixel counting answers "different",
   not "how far". A printed ArUco/checker target on the forearm would give
   sub-pixel absolute position, turning every measurement here from a
   threshold test into a real number. `Software/arm-vision/` already has
   `make_charuco_board.py` and `make_marker_sheet.py`.
3. **A still bench.** Ambient motion is the same magnitude as the effects.

With 1–3, backlash below 1° and hunting of a few tenths of a degree become
measurable. Without them, this instrument cannot resolve either, and any
compensation tuned on it is unfalsifiable.

## Recommendation

Do **not** build backlash compensation for this arm yet. Fit a fiducial marker
and fix the light first, then re-run `arm_precision.py backlash` and `repeat`.
If a real directional effect appears, the compensation design in that file's
header (preferred-direction approach, not offset injection) is the right shape
for hobby servos and can be implemented in an afternoon.

The gripper's one-way fault remains genuinely mechanical and is unaffected by
any of this.
