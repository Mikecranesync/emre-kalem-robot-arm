---
name: arm-motion-verify
description: Use when verifying that a commanded joint on the Emre Kalem arm actually MOVED — camera-verified motion, control frames, the geometry gate and its limits. Trigger on "did the joint move", "verify the gripper/wrist", "prove it moved", any calibration-log motion claim, or before recording a joint as controllable.
---

# Camera-verified motion

## The core rule

**A board ack proves the firmware accepted a command. It does not prove a motor turned.**

Two afternoons paid for this sentence:

- **D3 / J0 base.** The firmware provably drove D3 at 29.2 °/s for 3.5 s and nothing
  moved. Two software causes were found and fixed along the way — `DPS=1` and a
  destroyed 29–110 envelope — and **neither was the cause.** The servo was dead.
- **D11 / J6 gripper, 2026-08-06 evening.** `OK MOV J6 REQ=10 SET=10 CL=0`,
  `STA` confirming `SET=10 MOV=0 JTO=0`, and `SET` ramping
  `13 15 18 21 24 26 31 34 37 39 42 46 49 53 56 59 65 68 70` with `MOV=1`
  throughout at ~5.9 °/s against a commanded 6. Textbook. The fingers did not move
  at all. Root cause per the operator's hands: the gear slips on the motor shaft.

So: never write "controllable" from acks. Verify against the shaft — camera, or the
operator's eyes, ideally both.

## Running it

`Software/tests/motion_verify.py` drives one joint through named waypoints and
photographs every settle point. It **never opens the serial port** — the holder
daemon (`hold_arm.py`) owns COM5 and must keep owning it, because the watchdog
latches and detaches every joint the moment nothing feeds it, and closing the port
can DTR-reset the board. Motion goes through the daemon's file command channel.

| Flag | Meaning |
|---|---|
| `--link` | dir holding `arm_cmd.txt` / `arm_hold.log`. **Required** — it is a session-temporary scratchpad, never hardcode it |
| `--out` | where PNGs, the contact strip and `<label>_result.json` land |
| `--joint` | joint id |
| `--dps` | the joint's speed, used for settle timing. Set the board's speed separately with `SPD <j> <dps>` |
| `--roi` | `x0,y0,x1,y1` for the pixel diff |
| `--geo-roi` | tighter box for silhouette geometry. **Turns on the blocking shape gate** |
| `--waypoint` | `label:degrees`, repeat per waypoint |
| `--film N` | capture up to N frames DURING each travel window |
| `--label` | filename prefix |
| `--camera` | device index, default 0 |

Gripper (J6), with the shape gate on:

```bash
python Software/tests/motion_verify.py --link "$LINK" --out "$OUT" \
  --joint 6 --dps 12 --roi 330,425,485,565 --geo-roi 325,455,425,560 \
  --label J6 --film 8 \
  --waypoint open:70 --waypoint closed:10 --waypoint open:70 --waypoint closed:10
```

Wrist roll (J5) — no `--geo-roi`, see the gate's limits below:

```bash
python Software/tests/motion_verify.py --link "$LINK" --out "$OUT" \
  --joint 5 --dps 20 --roi 315,370,530,600 --label J5 --film 8 \
  --waypoint home:104 --waypoint roll_ccw:60 --waypoint roll_cw:150 --waypoint home:104
```

Exit code is non-zero if any graded transition did not resolve as motion.

## Control frames — the noise floor is measured, never assumed

At every settle point the harness grabs **two frames 0.5 s apart at the SAME
command**. Their difference is that waypoint's noise floor. Rules:

- **Adjacent in time, always.** With five joints energised the whole gripper
  assembly still settles downward slowly. Frames captured minutes apart pick up
  the sag, not the thing being measured.
- **A 0 px result means the capture is dead, not that the arm is steady.** The
  camera returns identical buffered frames when another app owns the device. Check
  for distinct frame MD5s before believing any null.
- **Never trust a null without a same-command control.** A threshold of 45 plus a
  5×5 morphological opening once erased the thin edge bands a moving finger
  produces and a real motion was nearly reported as absent. The threshold is now
  25 with **no** opening — do not raise it.
- Webcams buffer. Read ~6 frames and keep the last, or you photograph the past,
  which looks exactly like "it did not move".

## ROI framing

- **Exclude the operator.** He stands in shot at the bench. On a whole-frame diff
  he dominates completely, and when he reaches in, noise floors go from 2 px to
  12,022 px and every number for that run is void. That is correct behaviour, not
  a bug — but measure inside a box he is not in.
- **Frame the geo ROI on the articulating feature, with white margin around it.**
  The operator's white backdrop is what makes a threshold honest. Against cluttered
  cardboard, open-vs-closed could not be resolved at all; against the white sheet
  it separated 6× over a 272 px floor.
- **`bbox_h` SATURATES when the ROI is filled edge to edge.** The wrist box
  `315,370,530,600` pins it at 202 px whatever the pose, leaving `med_span` as the
  only live signal. A saturated dimension silently carries no information.

Boxes calibrated 2026-08-06 for this camera position: gripper diff
`330,425,485,565`, gripper prongs (geo) `325,455,425,560`, wrist `315,370,530,600`.

## The three gates, and what each one caught

1. **Ratio ≥ 4×** the same-waypoint noise floor. Pre-committed before the run, not
   chosen after seeing the numbers.
2. **Absolute ≥ 400 px** (`MIN_SIGNAL_PX`). A ratio alone is not enough: a 70 px
   edge shimmer against a freakishly quiet 2 px floor scored **35× MOVED** on the
   first run of this harness with nothing articulating in frame.
3. **A changed silhouette**, when `--geo-roi` is given. Both pixel gates passed on
   J6's retry — 519 px at 519× and 609 px at 47× — for a joint that provably does
   not articulate. Pixels answer "did anything change". For a gripper that is the
   wrong question. Shape answers the right one.

## The geometry gate: thresholds, and its measured limit

`GEO_MIN_BBOX_H_DELTA = 2` px, `GEO_MIN_SPAN_DELTA = 3` px,
`GEO_MIN_AREA_FRAC = 0.03`. Calibrated against two joints in one session, one
working and one broken, same camera and ROI scale:

| | bbox_h Δ | med_span Δ | area Δ |
|---|---|---|---|
| J4, articulating | 3, 4, 5, 5 px | 1, 1, 4, 4 | 0.3–2.3 % |
| J6, **not** | 1, 1, 1 px | 0, 1, 1 | 0.6–1.2 % |

`bbox_h` separates cleanly with a pixel of slack either side. **Area alone does
not** — the two overlap.

**Known false negative, measured not guessed.** 15° of J4 pitch moves the
silhouette by only ~1 px at this camera distance, indistinguishable from J6's sag,
so some real J4 legs fail this gate — 1 of 3 in the prong ROI, 1 of 3 in the wrist
ROI. It is **kept strict anyway** because the two errors do not cost the same: a
false positive records a wrong claim as fact, which is the failure this whole
harness exists to stop; a false negative costs a re-run, with the pixel numbers
printed on the same line to show it was marginal. That is why the verdict is
`DISAGREES` and **not** a diagnosis — the measurement cannot tell which side is
right, so it refuses to certify instead of asserting.

**Re-calibrate these thresholds if the camera moves or the ROI scale changes.**
They are a bench heuristic, not a physical constant.

## Reading the output

| Verdict | Meaning |
|---|---|
| `baseline` | first waypoint, nothing to compare against |
| `MOVED` | all applicable gates passed |
| `NOT RESOLVED` | pixel gates failed — under 4× or under 400 px |
| `DISAGREES (pixels moved, shape unchanged)` | pixels passed, silhouette did not change. **Refuses to certify.** Read the `geo` line and decide |
| `INVALID (daemon re-enabled mid-waypoint)` | the daemon logged `LATCHED` or `re-ENA` inside this window, so it re-sent `ENA <j> <adopt>` and snapped joints to their adopt angles. **The waypoint is garbage** — re-run it, do not measure it. It reads exactly like "the joint moved on its own" |

`--film` also reports `travel_peak`: the largest change seen at any point during
travel. Settle frames alone cannot tell "never moved" from "moved and came back".

## Rigid displacement vs articulation — the move that settled J6

Two frames can differ because the whole assembly shifted while nothing
articulated. A channel overlay separates them at a glance:

```python
ga = cv2.cvtColor(crop(first),  cv2.COLOR_BGR2GRAY)
gb = cv2.cvtColor(crop(second), cv2.COLOR_BGR2GRAY)
cv2.imwrite("overlay.png", np.dstack([ga, gb, ga]))   # OpenCV is BGR
```

Sign convention, worth getting right: **green = dark in the FIRST frame only,
magenta = dark in the SECOND frame only**, grey = identical.

- **Every edge fringing the SAME direction** — both prongs, body and forearm all
  green on top and magenta on the bottom — is a uniform rigid displacement. On J6
  that was ~1 px of downward sag, and it accounted for the entire 519 px "signal".
- **Articulation fringes in OPPOSITE directions** and changes the gap between the
  prongs. If the two fingers do not move relative to each other, they did not open.

The direction is readable too: green on top, magenta below, first frame captured
earlier ⇒ the assembly moved *down* between captures.

## Afterwards: the calibration log

Append to `Calibration_Notes/calibration-log.csv`. **13 fields**, comma-quoted
notes. It is **append-only and dated** — J1 has three rows, J3 has two.

- **Never edit an older observation.** The J6 `confirmed` row from 14:53 stays true
  *as of 14:53*; the regression is a new row (`REGRESSED-was-confirmed`). This repo
  already paid for a stale canonical row once — the J0 envelope that "gave back 29
  degrees the operator had already measured as unreachable", after which the joint
  would not drive from the console.
- **`pass=confirmed` requires the operator to have actually watched it.** A camera
  corroborates; his eyes are what confirm. That is the approved exception to the
  vocabulary rule — a value a human observed.
- Put the numbers in the note: signal px, noise floor, ratio, global shift, and the
  evidence path under `Calibration_Notes/evidence/`.

## Do not

- ❌ **Derive an angle from pixels.** No camera on this bench has been calibrated,
  so every marker size still rests on an assumed ~60° FOV. These runs produce
  motion / no-motion verdicts, never degrees.
- ❌ Call a joint controllable because the board acked, because `MOV=0`, or because
  `JTO=0`. `JTO` stayed 0 through every J6 run.
- ❌ Open COM5 from the test. The daemon owns it; `GET /rx` is destructive and a
  second poller steals replies. Console and script are mutually exclusive.
- ❌ Trust a run the operator's hands were in. Check the strip before believing it.
- ❌ Skip asking him. **Three runs once "passed" while nothing physically moved.**
  Ask what he observed — it is the strongest evidence available, and on J6 it is
  what produced the root cause the camera could not reach.
