# 2026-08-08 ~19:24 — J6 gripper film: VOID, not a verdict

`grade.py film 75 J6` captured 97 frames (`J6_000`–`J6_096`, ~0.6 s cadence) while the
operator was asked to send `SPD 6 10` / `ENA 6 <observed>` / `MOV 6 90 / 60 / 90`.
Re-graded offline with `regrade.py` (kept here), using the same rules as the live
grader — threshold 25, no morphological opening, floor from `control.json`
(full 767 px / gripper-ROI 10 px → thresholds 3068 / 150), ROI `(700,350)-(900,480)`.

**Headline: 65 of 96 intervals scored MOTION, peaks 80,263 px full-frame (105× floor)
and 6,144 px in the ROI (614× floor). None of it is the gripper.**

## Why it is void

`diff_24_25.png` and `diff_38_39.png` overlay the changed pixels in red on the earlier
frame, with the ROI drawn in green. What changed is the **white paper backdrop and the
shadow sweeping across it** — a broad band that runs from the bottom-left of the frame
up through the claw and out to the right. It passes straight through the ROI, so the
ROI metric is contaminated too: inside a 200×130 box the changed pixels span the *full*
extent, y 0–129 and x 0–194.

The claw itself is unchanged. Compare `J6_024.jpg` / `J6_025.jpg` (and `038`/`039`): the
prongs sit in the same place with the same gap, and no red traces their own edges in the
overlays. The signal is behind the gripper, not in it.

Three independent reasons this run cannot be rescued by re-thresholding:

1. **The floor does not describe this window.** `control.json` was measured during a
   quiet moment; the backdrop was already moving at interval 0 (17,619 px full-frame in
   the very first pair). The denominator of every ratio is wrong, so the 4× gate is
   meaningless here — not merely "the numbers came out high".
2. **The ROI admits the confound.** The changed pixels fill the box corner to corner.
   The box was drawn around the claw, but the backdrop is visible inside it.
3. **The film was blind.** `grade.py` only photographs; it does not drive. There is no
   serial-side record correlated with these 97 frames, so even a clean capture could not
   attribute motion to a command. This is the structural reason to use
   `Software/tests/motion_verify.py` instead — it drives the waypoints itself, so every
   frame is attributable by construction, and the shape gate would have refused a
   verdict here rather than reporting 65 MOTION intervals.

Capture health was fine: 0 identical-byte adjacent pairs across all 96 transitions, so
the camera path was live. The fault is scene contamination, not a dead capture.

## What this changes

**Nothing.** J6's status stands where 2026-08-06 left it —
`REGRESSED-was-confirmed`, operator's diagnosis "the gear is slipping around the motor
shaft". This run neither confirms nor clears that. No row was added to
`Calibration_Notes/calibration-log.csv`, because a void run is not a joint verdict.

## What the retry needs

- Tape or weight the backdrop so it cannot move; keep hands out of frame for the whole
  window.
- **Re-derive the ROIs for the current camera position.** `arm-motion-verify` lists
  gripper diff `330,425,485,565` and geo `325,455,425,560` "calibrated 2026-08-06 for
  this camera position" — the camera has moved since, which is why tonight's box is
  `700,350,900,480`. The geometry gate's thresholds are uncalibrated at this framing and
  should be treated as advisory on the first run at it, not blocking.
- Add a **negative control** the harness has never had: disturb the backdrop, command
  nothing, and require the candidate ROI to score near zero. Tonight's numbers are the
  proof that this test was needed.
- Drive through `motion_verify.py`, not a blind film.
