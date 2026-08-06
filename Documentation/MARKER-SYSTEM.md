# Marker system — observing the arm it cannot feel

The Emre Kalem arm has **no position feedback of any kind**. Every angle in
`Software/arm-console/joint-limits.csv` is an *accepted commanded soft limit* —
a number the operator drove to and the controller acknowledged. Nothing in the
system observes an output shaft.

This document designs the instrument that changes that: a set of printed
fiducial stickers, a camera, and two calibration passes. It is the physical
half of the **Option A observation contract** — `.pos` stays the *commanded*
joint position, and observed-from-markers values arrive in separate fields with
a residual, a validity flag and a timestamp.

The sticker table is `Software/vision/markers.csv`.
The geometry evidence is `Software/vision/stl-face-survey.csv`, produced by
`Software/vision/stl_face_survey.py` from the real STLs in `Backups/STL_parts/`.

---

## 1. What this design cannot know, stated first

The 21 STLs are **individual parts in print orientation**. There is no assembly
file in this repository. Therefore:

- **Link-to-link transforms are UNKNOWN.** Nothing tells us where `On_Kol`
  attaches to `Alt_Kol`, or at what offset.
- **Joint axes are UNKNOWN.** No axis direction can be read off a part.
- **Part → link assignment is INFERRED** from the Turkish names and the
  measured sizes (`Alt`=lower, `Ust`=upper, `Govde`=body, `Kasa`=case,
  `Kol`=arm, `On`=fore, `Bilek`=wrist, `El`=hand, `Parmak`=finger,
  `Disli`=gear, `Mil`=shaft, `Tabla`=plate). It is not verified.
- **Which face ends up outward** on symmetric parts (`El`, `Alt_Kapak`) is
  unknown. Where it matters the design gives numbers for both faces.
- **`D3`'s motor identity is DISPUTED.** `Calibration_Notes/calibration-log.csv`
  2026-08-01 says the servo physically wired to D3 was the **gripper**, not the
  base. Marker rows are labelled by *physical link*; the link→joint-id mapping
  inherits that dispute. **This marker set is the instrument that settles it** —
  command J0 alone and observe which link moves.
- **J4's `0-180` range is flagged SUSPECT** (it is the servo's whole electrical
  range). Occlusion analysis for the wrist is therefore run over a range that
  may not be physically reachable.
- **J6 is UNCALIBRATED.** There is no trustworthy travel to design against.

Consequence: **markers alone give a *change* in orientation, not an angle.**
Absolute angles need §6.

---

## 2. Dictionary: `cv2.aruco.DICT_4X4_50`

**ArUco 4×4, not AprilTag, and not ChArUco.** A 4×4 ArUco marker is 6 modules
across (4 data + 1 black border each side); AprilTag 36h11 is 8 — so at equal
physical size every 4×4 module is 33% larger, and on this arm *module size is
the binding constraint*, because the smallest carriers are tiny (`Bilek`
17.2 mm, `Parmak_2` 10.4 mm of usable flat). AprilTag's real advantage is
false-positive rejection and corner refinement, which buys little on a closed
bench set of 13 known ids where any unexpected id is dropped by a whitelist.

**ChArUco is a camera-calibration board, not a link sticker** — do not confuse
the two. It *is* required here, once: print a ChArUco board (5×7, 25 mm squares,
same `DICT_4X4_50`) and use it to obtain the camera intrinsics that every pose
estimate in §6 depends on. It is bench equipment, not something glued to a link.

OpenCV 4.13's `cv2.aruco` also ships APRILTAG dictionaries, so switching later
is a one-line change if false positives ever become the problem.

---

## 3. Quiet zone and sticker size

Quiet zone is **one module**, not a flat millimetre margin — a fixed margin does
not scale with marker size.

```
DICT_4X4_50 marker  = 6 modules across (4 data + 1 border ring)
+ 1 module quiet zone each side
= 8 modules total
total sticker = black_square × 8/6 = 1.3333 × black_square
```

Sizing rule actually applied to every row of `markers.csv`:

```
black_square_mm = floor( (measured_inscribed_square_mm - 2.0) / 1.3333 )
```

The 2.0 mm is application margin for a hand-placed sticker.

`measured_inscribed_square_mm` is **not** a bounding box and **not** a face
area. It is the largest axis-aligned square that fits inside the *actual
triangle coverage* of one planar patch, so holes and cutouts are already
subtracted. This matters: `Alt_Kapak` shows ~20 400 mm² of flat face but only
0.88 fill; `Tabla_Alt` looks like a 113.7 mm plate and takes an **11.6 mm**
square because it is a ring at 0.22 fill.

Rasterisation is at 0.4 mm/px, so every square is quoted **±0.4 mm**. Where a
reported square marginally exceeds the patch bbox (`Alt_Kapak` 114.0 vs 113.6)
that is exactly one pixel of rounding — take the bbox.

---

## 4. The size chain — derived, not asserted

**Assumption, unverified:** the built-in webcam is 1280×720 with HFOV ≈ 60°.
*Nothing in this repository records this camera's field of view.* It is stated
here as an assumption with the formula attached so it is re-derivable the moment
someone runs the ChArUco board. Every size below scales linearly with `f_px`.

```
f_px          = image_width_px / (2 · tan(HFOV/2))
              = 1280 / (2 · tan 30°)                    ≈ 1108 px

projected_px  = f_px · S_mm · cos θ / Z_mm
```

`S_mm` is the black square; `θ` is the angle between the face normal and the
line of sight; `Z_mm` is the standoff.

**Floors, expressed per module** (this is what makes the 4×4 choice pay):

| | px/module | 4×4 (6 mod) | 36h11 (8 mod) |
|---|---|---|---|
| detection | ≥ 4 | **24 px** | 32 px |
| stable pose | ≥ 6 | **36 px** | 48 px |

So `S_min(detect) = 24·Z / (f·cos θ)` and `S_min(pose) = 36·Z / (f·cos θ)`.

### The framing constraint that forces the answer

VFOV = 2·atan(360/1108) = 36.0°. To frame ~350 mm of arm vertically needs
`Z = 175/tan 18° ≈ 540 mm`. So a single camera that sees the whole arm sits at
**Z ≈ 550 mm**. At that standoff with a typical `cos θ = 0.7`:

| black square | projected px @ 550 mm | verdict |
|---|---|---|
| 36 mm (BASE) | 50.8 | pose ✓ |
| 25 mm (HAND-1) | 35.3 | detect ✓, pose marginal |
| 20 mm (TURRET) | 28.2 | detect ✓ |
| 17 mm (SHOULDER-1) | **23.99** | **below floor — by 0.01 px** |
| 13 mm (ELBOW-1) | 18.3 | **below floor** |
| 11 mm (WRIST-1) | 15.5 | **below floor** |
| 6 mm (FINGER) | 8.5 | **below floor** |

SHOULDER-1 deserves its own line, because rounding it to "24.0" would have
reported it as passing. `1108 × 17 × 0.7 / 550 = 23.99` px. It crosses the floor
at Z ≤ 549 mm. That is not a margin in either direction — treat the shoulder as
unobservable on CAM-A rather than as a pass.

**This is the headline finding, and it is a negative one:** on a single
1280×720 camera at the standoff required to frame the arm, **only the two BASE
markers reach pose grade and only the two TURRET markers reach detect grade.**
The shoulder, forearm, wrist and gripper are all below the detection floor —
they are not merely imprecise, they are *not seen*. The arm's own printed
surfaces are simply too small. Two honest ways out, both in §8; neither is
optional if any joint past the turret is to be observed at all.

---

## 5. Placement logic

### The rule that decides every face

> Put the marker on the face whose **normal is parallel to the joint axis**.

Then the marker rotates **in-plane** as the joint moves, instead of tilting
away from the camera. In-plane rotation is what a single tag measures best, and
it holds the projected size constant across the whole travel. This applies to
roll joints *and* bend joints — it is not a roll-only trick:

- **J5 wrist roll (31→178, 147° sweep).** `El` is a plate perpendicular to the
  roll axis → the marker spins in-plane over the full sweep. **Single marker is
  correct here**, and it is also forced: only one 36 mm square fits on `El`.
- **J0 base yaw (29→110, 81°).** Axis vertical → want a face with a vertical
  normal. `Alt_Govde` has none usable (§7) — this is the design's weak point.
- **J1 shoulder (0→91) and J3 elbow (0→66).** Axes horizontal-sideways → the
  *broad side faces* of `Alt_Kol` and `On_Kol` have normals along those axes, so
  they turn with the joint rather than away from it. No cosine penalty across
  travel.

Acceptance test for anything not covered by the rule: projected width is
`S·cos θ`; if `cos θ` drops below **0.35** at either travel extreme, that link
needs a second marker on a face rotated ~90°.

### Where two markers, and what they buy

Angle noise from a two-centroid baseline:

```
σ_angle ≈ σ_centroid_px · (Z / f_px) / baseline_mm      [radians]
```

At σ = 0.2 px, Z = 600 mm, f = 1108 → 0.108 mm of lateral centroid noise:

**A baseline is only real if both markers sit on the same part.** The survey
measures disjoint square pairs *within one planar patch of one STL*, so those
numbers are trustworthy. A pair spanning two parts has **no derivable baseline**
— there is no assembly file — and must be observed in Pass 1:

| link | baseline | σ_angle | note |
|---|---|---|---|
| UPPER_ARM (`Alt_Kol`) | **176.6 mm** | **0.035°** | same part — best in the set |
| GROUND (`Alt_Kasa`) | 83.2 mm | 0.075° | same part; up to ~100 mm achievable |
| FOREARM (`On_Kol`) | 22.2 mm | 0.28° | same part — **degraded, named not hidden** |
| HAND (`El` → `El_Ust`) | **unknown** | — | cross-part; Pass 1 must observe it |
| FINGERS (`Parmak_2` ×2) | **unknown** | — | cross-part; the two fingers are two printed copies |

The gripper observable is the *separation between the two installed fingers*.
That distance is **not** derivable from a single STL and no number for it
appears in this design.

Two markers are specified for GROUND (world datum + scale check), UPPER_ARM
(direction), FOREARM (direction, degraded), TURRET (**coverage, not baseline** —
across 81° of yaw a single wrapped marker rotates out of view), and the FINGERS
(their **separation** is the gripper observable; neither alone means anything).
`Bilek` gets one because it has no second island ≥ 8 mm. `El` gets one because
only one square fits and J5 does not need a pair.

### Orientation arrows

ArUco encodes its own orientation, so an arrow adds **nothing** to the
algorithm. Print a human label and an arrow **outside the quiet zone** anyway
(`SHOULDER-1 ↑`), so a sticker is re-applied identically after a re-print and
the residual sign convention does not silently flip. **Never** put ink inside
the quiet zone — that breaks detection.

### Where NOT to place — and three traps the geometry exposed

Excluded by inspection: all gears (`Disli`, `Servo_Disli`, `Mil_Disli`,
`Parmak_Disli` — they rotate relative to their link and must mesh), all shafts
(`Mil_1/2/3`, 6.8–17 mm cylinders), `Servo_Cable_Holder` (cables sweep over it),
`Jack_Cover` (removable).

Three placements that *look* ideal in the survey and are wrong:

1. **`Alt_Kol` "rails"** — `n=(±1,0,0.05)`, 198.5 × 20.1 mm at **fill 1.00**,
   the most inviting numbers on the part. They are **inner walls of a channel**:
   the face sits at x = −22.5 while the part runs out to x = −27.5, so there is
   material outboard of it. Not a surface you can reach with a sticker.
2. **`On_Kol` `n=(+1,0,0)` at x = −30.5** — 36.8 × 40.2 at fill 0.81, the
   largest square on the forearm (25.2 mm). Also an **inner** face; the part
   runs to x = +20. Same for `n=(0,+1,0)` at y = −15.2.
3. **`Alt_Kasa` top face** — 119.6 × 209.6 mm and the biggest bbox on the arm,
   but **fill 0.39**: it is open for servo access. Square is only 24.8 mm.

Also avoid `Alt_Govde`'s `z=0` face: 114 × 114 at fill 0.62 looks usable but it
is a **recessed ledge with 50 mm of structure above it**.

---

## 6. The two calibration passes — this is what makes the residual mean anything

A marker table on its own does **not** satisfy the Option A contract: without a
reference, `residual = observed − commanded` has nothing to subtract from. Both
passes below are required, in order.

### Pass 1 — axis identification (recovers what the STLs cannot give)

For each joint *j*, one at a time, all others held:

1. Command *j* across its calibrated range from `joint-limits.csv` in ~5° steps.
2. At each step capture the pose of the child-link marker **relative to** the
   parent-link marker.
3. Least-squares fit a rotation axis to that trajectory.

This yields the joint axis **and** the parent→child transform empirically. It is
the correct answer to "link-to-link geometry is unknown": you do not read it off
the STLs, you **observe it with the very marker system being designed**. It also
settles the D3 dispute — command J0 and watch which link moves.

Run it at reduced speed with the supply limitation in mind: the bench supply is
~700 mA and **cannot hold an assembled arm**; a 3–5 A supply and a 1 A slow-blow
fuse are outstanding.

### Pass 2 — datum capture

1. Drive every joint to its recorded `home_deg` (J0 64, J1 1, J3 33, J4 90,
   J5 104, J6 90).
2. Capture and store the relative marker geometry as the reference datum.
3. Thereafter `observed_angle = home_deg + Δ(about the fitted axis)`.

**The datum is observed at a *commanded* home, so it inherits that home's
uncertainty. It is not a kinematic zero.** `home_deg` is explicitly editorial in
`joint-limits.csv` — several values were moved off a hard end of travel by hand.
Every observed angle therefore carries the datum's offset error as a constant
bias. The residual is still useful (it detects *change* — sag, slip, stall)
without the bias being zero.

**Named side benefit, not overclaimed:** a persistent non-zero J1 residual is the
first observable that would expose the unmeasured `mirror_offset_deg`. Two
MG996R servos fighting each other by a fixed angle is exactly a constant
shoulder residual.

---

## 7. Camera plan

**CAM-A — overview.** Z ≈ 550 mm, elevated ~35–45°, off to one side so it sees
both the turret's vertical-axis rotation and the broad side faces of the arm
links. `cv2.VideoCapture` **must** use `CAP_DSHOW` on this machine, and the
built-in camera silently refuses 1080p — 1280×720 is the ceiling. Carries
BASE-1/2 (pose) and TURRET-1/2 (detect) — **and nothing else**; SHOULDER-1
misses the detection floor by 0.01 px at 550 mm (§4).

**CAM-B — wrist close-up.** Z ≈ 230–250 mm aimed at the wrist/hand region.
Carries HAND-1 (pose), WRIST-1 and HAND-2 (detect).

**J5 is carried by CAM-B, not CAM-A.** HAND-1 is *dual-visible* — CAM-A sees it
— but at 550 mm a 25 mm square projects 35.3 px, which is below this document's
own 36 px pose floor. CAM-A is therefore a detect-grade fallback for the hand,
and every pose-grade statement about J5 depends on CAM-B existing. Do not read
§8(a) as optional if J5 pose matters.

WRIST-1 needs CAM-B closer than ~237 mm for pose grade (34.1 px at 250 mm), and
the fingers need closer than ~190 mm merely to be detected. LeRobot supports
multiple cameras directly:
`make_cameras_from_configs(config.cameras)` with two `OpenCVCameraConfig`
entries, so a second camera costs configuration, not architecture.

A note this design must own: **CAM-B is a fixed bench camera, not a wrist
camera.** A camera mounted *on* the wrist moves with J4/J5 and cannot observe
them.

---

## 8. Two honest ways past the negative finding

The arm's own surfaces cannot carry pose-grade markers for FOREARM, WRIST and
GRIPPER from a single framing camera. Pick one:

**(a) Second camera.** Add CAM-B per §7. Costs one webcam and one config entry.
Changes nothing mechanical. Gripper still marginal at 6 mm.

**(b) Printed flat marker tabs — recommended.** Three flat plates ~40 × 40 × 2 mm
bolted or glued to `Alt_Kol`, `On_Kol` and `Bilek` would carry a 28 mm black
square, which projects 40 px at 550 mm and `cos θ` 0.7 — **pose-grade for the
whole chain from CAM-A alone**. Working backwards from the 36 px pose floor:
`S ≥ 36 × 550 / (1108 × 0.7) = 25.5 mm`, so `1.333 × 25.5 ≈ 34 mm` of sticker,
so a 40 mm tab. A 30 mm tab is *not* enough (29.6 px — detect only).

This is new geometry, so it is a proposal, not a measurement. It is flagged as
such rather than folded silently into `markers.csv`.

---

## 9. Assumptions register

| # | Assumption | Status | How to settle it |
|---|---|---|---|
| A1 | Webcam HFOV ≈ 60°, f_px ≈ 1108 | **UNVERIFIED** | ChArUco board → intrinsics |
| A2 | Part → link mapping | **INFERRED** from names/sizes | Pass 1 axis identification |
| A3 | Joint axes and link transforms | **UNKNOWN** | Pass 1 |
| A4 | D3 drives the base, not the gripper | **DISPUTED** | Command J0; observe |
| A5 | Which face of `El`/`Alt_Kapak` is outward | **UNKNOWN** | Look at the arm |
| A6 | `El_Ust` is rigid with `El` | **INFERRED** from shared footprint | Pass 1 |
| A7 | J4 reaches 0–180 | **SUSPECT** (full electrical range) | Re-drive to real stops |
| A8 | J6 travel | **UNCALIBRATED** | Bench-test the gripper |
| A9 | Centroid noise 0.2 px | Typical, not measured here | Static capture, N frames |

---

## 10. Licences

- **OpenCV** — Apache-2.0. Compliant.
- **numpy** — BSD-3-Clause. **FLAGGED**, not silently accepted: the stated rule
  is Apache-2.0 or MIT only. numpy is an unavoidable dependency of both this
  work and LeRobot. Raised for a decision rather than passed over.
- **pyserial** — BSD-3-Clause. Same flag; already unavoidable in `arm-bridge.py`.

---

## 11. Safety and vocabulary

Nothing in this document is an emergency stop. `STP` aborts motion and **holds**
with joints still driven; `EST` and the watchdog **detach**, and a gravity-loaded
arm **sags**. The rocker switch and the inline fuse are the only real stop.

Values recovered from these markers are **observed** — a camera actually saw
them, which is the one approved exception to the rule against "measured". They
are observations of a *printed part*, never of a servo output shaft. `.pos`
remains **commanded**. The two must never be collapsed into one field.
