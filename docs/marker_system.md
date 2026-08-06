# Marker system — print, place, and record

This is the **operator-facing** half of the fiducial marker work: which markers
to print, at what size, how to print them without silently corrupting the scale,
where to stick them, and what to write down afterwards.

**Provenance — two documents, on purpose.**

| File | Role |
|---|---|
| `Documentation/MARKER-SYSTEM.md` | **Design / derivation.** Why 4×4, the size chain, the two calibration passes, the assumptions register. |
| `Software/vision/markers.csv` | **Full 17-column table.** Every derived number plus its evidence columns. |
| `Software/vision/stl-face-survey.csv` | **Geometry evidence.** 59 measured planar patches from the real STLs. |
| **`docs/marker_system.md`** (this file) | **Print-and-place procedure.** Printing, ID scheme, callipering, placement, occlusion, limits. |
| **`docs/marker_placement_table.csv`** | **The 8-column placement table** — a projection of `markers.csv`, not a second source of truth. |

`docs/marker_placement_table.csv` is **generated**, never hand-edited. It is the
first 8 columns of `Software/vision/markers.csv`, unchanged. Regenerate it after
any change to that file:

```bash
python - <<'PY'
import csv, io
src = 'Software/vision/markers.csv'
dst = 'docs/marker_placement_table.csv'
kept = [l for l in open(src, encoding='utf-8').read().splitlines()
        if l.strip() and not l.lstrip().startswith('#')]
rows = [r[:8] for r in csv.reader(io.StringIO("\n".join(kept)))]
csv.writer(open(dst, 'w', newline='', encoding='utf-8'), lineterminator='\n').writerows(rows)
PY
```

Two files that can disagree is exactly how this repo lost J0's calibrated
`29–110` envelope on 2026-08-05 (a lock existed only in a Downloads folder while
the file that got edited did not know about it). Regenerating removes the
possibility rather than relying on discipline.

---

## 1. Marker family: `cv2.aruco.DICT_4X4_50`

**ArUco 4×4. Not AprilTag, not ChArUco.**

A `DICT_4X4_50` marker is **6 modules across** — 4 data modules plus a one-module
black border on each side. AprilTag 36h11 is **8 modules across**. At the same
physical size, every 4×4 module is therefore **33% larger**.

Module size is the binding constraint on this arm, not marker area. The smallest
carriers are genuinely tiny: `Bilek` offers 17.2 mm of usable flat, `Parmak_2`
offers 10.4 mm. A 6 mm black square is 1.0 mm per module in 4×4 and 0.75 mm per
module in 36h11 — the difference between marginal and hopeless.

Expressed as detection floors, which is what actually decides the design:

| | px per module | 4×4 (6 modules) | AprilTag 36h11 (8 modules) |
|---|---|---|---|
| detected at all | ≥ 4 | **24 px** | 32 px |
| stable 6-DOF pose | ≥ 6 | **36 px** | 48 px |

AprilTag's real advantages are false-positive rejection and corner refinement.
Neither buys much here: this is a **closed set of 13 known ids on a bench**, and
any id outside `0–12` is dropped by a whitelist before it reaches pose code.
OpenCV 4.13 also ships APRILTAG dictionaries, so switching later is a one-line
change if false positives ever become a real problem.

**ChArUco is not an alternative to the above — it is a different tool.** A
ChArUco board is *camera-calibration equipment*, not a link sticker. You still
need one, exactly once: print a 5×7 board with 25 mm squares in the same
`DICT_4X4_50` and use it to obtain the camera intrinsics. Every pose estimate
depends on those intrinsics, and the focal length in §5 is currently an
**assumption** until that board is run.

---

## 2. ID scheme

**Marker ids are `0–12` and are deliberately NOT joint ids.**

| id | label | link | joint it helps observe |
|---|---|---|---|
| 0 | BASE-1 | GROUND | — (world datum) |
| 1 | BASE-2 | GROUND | — (world datum + scale check) |
| 2 | TURRET-1 | TURRET | J0 base yaw |
| 3 | TURRET-2 | TURRET | J0 base yaw (coverage) |
| 4 | SHOULDER-1 | UPPER_ARM | J1 shoulder |
| 5 | SHOULDER-2 | UPPER_ARM | J1 shoulder (baseline) |
| 6 | ELBOW-1 | FOREARM | J3 elbow |
| 7 | ELBOW-2 | FOREARM | J3 elbow (baseline, degraded) |
| 8 | WRIST-1 | WRIST | J4 wrist pitch |
| 9 | HAND-1 | HAND | J5 wrist roll |
| 10 | HAND-2 | HAND | J5 (optional redundancy) |
| 11 | FINGER-A | FINGER_A | J6 gripper (separation) |
| 12 | FINGER-B | FINGER_B | J6 gripper (separation) |

Rules the scheme follows:

1. **Ids are contiguous `0–12`.** No gaps, so a missing detection is
   unambiguously a missing detection.
2. **Marker id ≠ joint id, on purpose.** Joint id **2 is reserved and
   unaddressable by construction** (it is D5, the shoulder pair's second servo).
   An id space aligned to joints would either imply a joint 2 exists or need a
   conspicuous hole. **Marker id 2 is TURRET-1 and has nothing to do with joint
   2.**
3. **Two markers per link are consecutive** (0/1, 2/3, 4/5, 6/7, 11/12), so a
   pair is obvious from the id alone.
4. **Link names are physical, not functional** — GROUND, TURRET, UPPER_ARM,
   FOREARM, WRIST, HAND, FINGER_A, FINGER_B. The link→joint mapping in the table
   above is **inferred**, because there is no assembly file and because the D3
   motor identity is disputed (see §8). Physical names stay true regardless of
   how that dispute resolves.
5. **One logical joint, one marker set.** J1 is one joint spanning D4+D5; it gets
   one UPPER_ARM pair, not two.
6. **Ids are permanent.** Re-print a damaged sticker with the same id. Never
   recycle an id onto a different link — every recorded dataset would silently
   re-interpret.

---

## 3. Sizes in mm, with quiet zones

### The quiet zone is one module, not a fixed margin

A fixed millimetre border does not scale with marker size. The quiet-zone
requirement is defined in **modules**:

```
DICT_4X4_50 marker  = 6 modules  (4 data + 1 border ring each side)
+ 1 module quiet zone on each side
= 8 modules total

printed sticker edge = black_square × 8/6 = 1.3333 × black_square
```

The `approximate_size_mm` column in `docs/marker_placement_table.csv` is the
**whole sticker including the white quiet zone** — the square you cut out.
The **black square** is what you pass to `cv2.aruco` / `solvePnP` as the marker
length. They are different numbers and confusing them scales every pose by 1.33.

### The sizing rule (an upper bound)

```
black_square_mm ≤ floor( (measured_inscribed_square_mm - 2.0) / 1.3333 )
```

The 2.0 mm is application margin for a hand-placed sticker.
`measured_inscribed_square_mm` is **not** a bounding box and **not** a face area
— it is the largest axis-aligned square that fits inside the actual triangle
coverage of one planar patch, so holes and cutouts are already subtracted.

It is written as **≤**, not **=**, because two rows were deliberately capped
below it. Of the 13 markers:

- **9 reproduce the rule exactly** (SHOULDER-1/2, ELBOW-1/2, WRIST-1, HAND-1/2,
  FINGER-A/B).
- **2 are capped** — BASE-1 and BASE-2. The rule permits 45 mm; the table uses
  **36 mm**. The source table does not record why. The effect is benign (36 mm
  already projects 50.8 px, the largest pose margin in the set) but the reason is
  a reconstruction, not a record — logged in §8.
- **2 do not apply** — TURRET-1 and TURRET-2 sit on a *cylinder*. There is no
  planar patch and therefore no inscribed square; 20 mm was chosen from the arc
  argument in §6.

### The printed set

| id | label | black square | sticker (incl. quiet zone) | carrier |
|---|---|---|---|---|
| 0 | BASE-1 | 36 mm | **48.0 mm** | `Alt_Kasa` side wall |
| 1 | BASE-2 | 36 mm | **48.0 mm** | `Alt_Kasa` side wall |
| 2 | TURRET-1 | 20 mm | **26.7 mm** | `Alt_Govde` cylinder |
| 3 | TURRET-2 | 20 mm | **26.7 mm** | `Alt_Govde` cylinder |
| 4 | SHOULDER-1 | 17 mm | **22.7 mm** | `Alt_Kol` side face |
| 5 | SHOULDER-2 | 8 mm | **10.7 mm** | `Alt_Kol` side face |
| 6 | ELBOW-1 | 13 mm | **17.3 mm** | `On_Kol` outer face |
| 7 | ELBOW-2 | 6 mm | **8.0 mm** | `On_Kol` outer face |
| 8 | WRIST-1 | 11 mm | **14.7 mm** | `Bilek` outer face |
| 9 | HAND-1 | 25 mm | **33.3 mm** | `El` plate |
| 10 | HAND-2 | 8 mm | **10.7 mm** | `El_Ust` face |
| 11 | FINGER-A | 6 mm | **8.0 mm** | `Parmak_2` outer face |
| 12 | FINGER-B | 6 mm | **8.0 mm** | `Parmak_2` outer face |

---

## 4. Printing instructions

**This section is where the design most easily fails silently.** A marker printed
at the wrong scale is still detected perfectly — `solvePnP` simply returns a pose
scaled by the print error and nothing warns you.

### Before printing

1. Generate the markers at a **generous pixel resolution** (e.g. 600 px per
   marker) and place them on the page at the physical sizes in §3. Do not let the
   generator's pixel size become the physical size by accident.
2. Leave the **quiet zone white and empty**. Ink inside the quiet zone breaks
   detection. Human labels go **outside** it (see §6).
3. Print on **matte** stock. Glossy paper and clear laminate produce specular
   blowout under bench lighting, which destroys corner refinement on exactly the
   small markers that can least afford it.

### At the print dialog

1. **Scale must be 100% / "Actual size".**
2. **NEVER "Fit to page", "Shrink oversized pages", "Scale to fit", or
   "Fit to printable area".** These are usually the *default*. Fit-to-page on
   A4/Letter typically applies ~94–96%, which is a ~5% scale error on every pose
   — a 25 mm marker becomes 23.8 mm and every distance the camera reports is
   wrong by 5% while looking completely healthy.
3. Turn off any driver-level "fit", "poster", or "booklet" mode as well. The
   application dialog and the driver dialog can each rescale independently.
4. Print **one calibration test page first**, calliper it (below), and only then
   print the full set.

### After printing — calliper and record

**Measure the printed black square with callipers and write the number down.
That measured number is what you pass to the pose solver — not the nominal size
from §3.**

For each marker:

1. Calliper the **black square only**, edge to edge — not the sticker, not the
   quiet zone. Include the outer black border ring; exclude all white.
2. Measure **both** axes (X and Y). Printers scale differently along and across
   the paper feed. If they differ by more than ~1%, reprint; a non-square marker
   corrupts pose in a way no single scale factor can correct.
3. Record to **0.1 mm**.
4. Log each measurement in `Calibration_Notes/` with the date, printer, and the
   nominal size it was supposed to be. Follow the existing lock-artifact habit:
   the raw number, the tool that produced it, and the timestamp.
5. **Reject any marker more than 2% off nominal.** Reprint it.

**Vocabulary — this is an observation, not a nominal.** The callipered figure is
a value a human actually measured with an instrument, so it is **observed** and
must be named as such in the log. The 36 mm in §3 is the *intended* size. Where
they differ, the observed number wins, always. This is the same rule that governs
the arm itself: `.pos` is commanded, and anything a camera or a human actually
saw is observed and lives in its own field.

### Applying the sticker

1. Clean the printed plastic surface — 3D-printed PLA carries release agent and
   finger oil.
2. Apply flat, no bubbles, no bridging over an edge or a hole. A marker that
   creases over a corner is not planar and its pose is meaningless.
3. Align to the **preferred orientation** in `docs/marker_placement_table.csv`,
   using the human arrow (§6) as the reference.
4. Photograph each placement before running anything. If a sticker is later
   knocked off, the photograph is the only record of where it was.

---

## 5. Webcam assumptions

**All of these are assumptions, and the first one is unverified.**

| | value | status |
|---|---|---|
| Resolution | 1280 × 720 | confirmed on this machine |
| 1080p | **silently refused** by the built-in camera | confirmed |
| Backend | `cv2.VideoCapture` **must** use `CAP_DSHOW` | confirmed on this machine |
| HFOV | ≈ 60° | **UNVERIFIED — assumption** |
| Focal length | `f_px = 1280 / (2·tan 30°) ≈ 1108 px` | derived from the above |
| Centroid noise | 0.2 px | typical, **not measured here** |
| View angle | `cos θ = 0.7` used throughout | design assumption |

**Every size and every grade in this design scales linearly with `f_px`.** Run
the ChArUco board and re-derive before committing to a print run.

```
projected_px = f_px · black_square_mm · cos θ / standoff_mm
```

### Two cameras, forced by arithmetic

VFOV = `2·atan(360/1108)` = 36.0°. Framing ~350 mm of arm vertically therefore
needs a standoff of `175 / tan 18° ≈ 540 mm`. So a single camera that sees the
whole arm sits at ~550 mm — and at that distance most of the markers vanish.

**CAM-A — overview.** Z ≈ 550 mm, elevated 35–45°, off to one side.
Carries BASE-1/2 (pose) and TURRET-1/2 (detect).

**CAM-B — close.** Z ≈ 230–250 mm on the wrist/hand region.
Carries HAND-1 (pose), WRIST-1 and HAND-2 (detect).
**CAM-B is a fixed bench camera, not a wrist camera** — a camera mounted *on* the
wrist moves with J4/J5 and therefore cannot observe them.

Grades at the design standoffs, `cos θ = 0.7`:

| black square | @ 550 mm (CAM-A) | @ 250 mm (CAM-B) |
|---|---|---|
| 36 mm | **50.8 px** — pose | — |
| 25 mm | 35.3 px — detect | **77.6 px** — pose |
| 20 mm | 28.2 px — detect | — |
| 17 mm | **23.97 px — below floor** | — |
| 13 mm | 18.3 px — below floor | 40.3 px — pose |
| 11 mm | 15.5 px — below floor | 34.1 px — detect |
| 8 mm | 11.3 px — below floor | 24.8 px — detect |
| 6 mm | 8.5 px — below floor | 18.6 px — below floor |

SHOULDER-1 gets its own line because rounding hides it: `1108 × 17 × 0.7 / 550`
= **23.97 px**, missing the 24 px detection floor by **0.03 px**. It crosses at
Z ≤ 549 mm. That is not a margin in either direction — treat the shoulder as
unobservable on CAM-A, not as a pass.

> **Correction to the committed artifacts:** `Software/vision/markers.csv` row 4
> and `Documentation/MARKER-SYSTEM.md` §4 both state **23.99 px** / "a hundredth
> of a pixel". The correct value is **23.97 px**, i.e. 0.03 px. The verdict
> (below-floor) and the 549 mm crossing are unchanged. Fix on the next edit of
> those files.

---

## 6. Placement rationale

### The rule that decides every face

> **Put the marker on the face whose normal is parallel to the joint axis.**

The marker then rotates **in-plane** as the joint moves, rather than tilting away
from the camera. In-plane rotation is what a single tag measures best, and it
holds the projected size constant across the entire travel — no cosine penalty at
either extreme.

This is **not** a roll-joint-only trick; it applies to bend joints too:

- **J5 wrist roll (31→178, a 147° sweep).** `El` is a plate perpendicular to the
  roll axis, so HAND-1 spins in-plane over the whole sweep. A single marker is
  correct here — and also forced, since only one 36 mm square fits on `El`.
- **J1 shoulder (0→91) and J3 elbow (0→66).** The axes are horizontal-sideways,
  and the **broad side faces** of `Alt_Kol` and `On_Kol` have normals along those
  axes. They turn *with* the joint, not away from it.
- **J0 base yaw (29→110, 81°).** The axis is vertical, so a vertical-normal face
  is wanted. `Alt_Govde` has none usable — this is the design's weak point (§7).

Acceptance test for anything the rule does not cover: projected width is
`S·cos θ`. If `cos θ` falls below **0.35** at either travel extreme, that link
needs a second marker on a face rotated ~90°.

### Where two markers, and what they buy

```
σ_angle ≈ σ_centroid_px · (standoff / f_px) / baseline_mm      [radians]
```

At 0.2 px, Z = 600 mm, f = 1108 px → 0.108 mm of lateral centroid noise:

| link | baseline | σ_angle | note |
|---|---|---|---|
| UPPER_ARM (`Alt_Kol`) | **176.6 mm** | **0.035°** | same part — best in the set |
| GROUND (`Alt_Kasa`) | 83.2 mm | 0.075° | same part; ~100 mm achievable |
| FOREARM (`On_Kol`) | 22.2 mm | 0.28° | same part — **degraded, named not hidden** |
| HAND (`El` → `El_Ust`) | **unknown** | — | cross-part; must be observed |
| FINGERS (`Parmak_2` ×2) | **unknown** | — | cross-part; two printed copies |

**A baseline is only real if both markers sit on the same part.** Those numbers
come from disjoint square pairs measured within one planar patch of one STL. A
pair spanning two parts has **no derivable baseline** — there is no assembly file
— and must be observed empirically.

Two markers are specified for: GROUND (world datum + scale check), UPPER_ARM
(direction, best baseline), FOREARM (direction, degraded baseline), TURRET
(**coverage, not baseline**), and the FINGERS (their **separation** is the
gripper observable — neither alone means anything for J6). `Bilek` gets one
because it has no second island ≥ 8 mm. `El` gets one because only one square
fits and J5 does not need a pair.

### Human labels and arrows

ArUco encodes its own orientation, so an arrow adds **nothing** to the algorithm.
Print a human label and arrow **outside the quiet zone** anyway — `SHOULDER-1 ↑`
— so a sticker is re-applied identically after a re-print and the residual sign
convention does not silently flip. **Never put ink inside the quiet zone.**

### Where NOT to place

Excluded by inspection: all gears (`Disli`, `Servo_Disli`, `Mil_Disli`,
`Parmak_Disli` — they rotate relative to their link and must mesh), all shafts
(`Mil_1/2/3`), `Servo_Cable_Holder` (cables sweep over it), `Jack_Cover`
(removable).

Four surfaces that *look* ideal in the geometry survey and are wrong:

1. **`Alt_Kol` "rails"** — 198.5 × 20.1 mm at **fill 1.00**, the most inviting
   numbers on the part. They are **inner walls of a channel**: the face sits at
   x = −22.5 while the part runs out to x = −27.5, so there is material outboard.
   Unreachable with a sticker.
2. **`On_Kol` at x = −30.5** — fill 0.81, the largest square on the forearm
   (25.2 mm). Also **inner**; the part runs to x = +20. Same for y = −15.2.
3. **`Alt_Kasa` top face** — 119.6 × 209.6 mm, the biggest bbox on the arm, but
   **fill 0.39**: it is open for servo access.
4. **`Alt_Govde` z=0 face** — 114 × 114 at fill 0.62 looks usable but it is a
   **recessed ledge with 50 mm of structure above it**.

---

## 7. Occlusion notes

| link | risk | mitigation |
|---|---|---|
| **GROUND** | none — never moves, never shadowed | — |
| **TURRET** | **high across the 81° J0 sweep.** A single marker wrapped on the cylinder rotates out of view. | TURRET-1 and TURRET-2 are **90° apart in azimuth**, so at least one stays within ~45° of facing the camera at all times. This pair exists for **coverage, not baseline.** |
| **UPPER_ARM** | moderate — the forearm can shadow the shoulder at extreme elbow angles | two markers 176.6 mm apart; losing one still gives a centroid |
| **FOREARM** | moderate — the hand assembly crosses it at high J4 | ELBOW-1/2 are redundancy against occlusion, **not** a precision baseline (22.2 mm) |
| **WRIST** | **highest in the set.** `Bilek` is 43 × 40 × 25 mm and has **no second usable island** (next best ≥ 8 mm does not exist). | Single-tag pose only. Degrades faster than every two-marker link. No mitigation available on the printed geometry. |
| **HAND** | low for J5 — the plate spins in-plane and stays presented | HAND-2 on `El_Ust` is **optional** redundancy; include only if the hand assembly shadows HAND-1 at some roll angles |
| **FINGERS** | **self-occlusion when the gripper closes** — the fingers approach and can overlap in projection | Both fingers are labelled so they read the same way when closed. The observable is their **separation**, so losing either kills the J6 estimate entirely |

Two cross-cutting notes:

- **The TURRET markers are on a curved surface.** A 20 mm sticker on a 60 mm
  radius spans ~19° of arc, about 0.8 mm of bow. Use **centroid and azimuth
  only** — do **not** trust single-tag pose from a bowed marker.
- **Lighting is an occlusion mode.** Specular glare off a glossy sticker removes
  a marker as effectively as a physical obstruction. Matte stock, diffuse light,
  no direct lamp in either camera's specular path.

---

## 8. Limitations — read before trusting any number here

### The headline finding is negative

**On a single 1280 × 720 camera at the standoff required to frame the arm, only
the two BASE markers reach pose grade and only the two TURRET markers reach
detect grade.** Shoulder, forearm, wrist and gripper are all **below the
detection floor** — not imprecise, *not seen*. The arm's own printed surfaces are
too small.

Two honest ways past it:

- **(a) Add CAM-B** (§5). One webcam, one config entry, nothing mechanical
  changes. The gripper remains marginal at 6 mm.
- **(b) Printed flat marker tabs — recommended.** Three flat plates ~40 × 40 ×
  2 mm on `Alt_Kol`, `On_Kol` and `Bilek` would carry a 28 mm black square →
  ≈39.5 px at 550 mm, i.e. **pose grade for the whole chain from CAM-A alone**.
  Working back from the 36 px floor: `S ≥ 36 × 550 / (1108 × 0.7) = 25.5 mm`, so
  ~34 mm of sticker, so a 40 mm tab. **A 30 mm tab is not enough** (29.6 px —
  detect only). This is new geometry, so it is a **proposal, not a measurement**.

### What this design structurally cannot know

The 21 STLs in `Backups/STL_parts/` are **individual parts in print orientation**
and there is **no assembly file** anywhere in the repository. Therefore:

- **Link-to-link transforms are UNKNOWN.** Nothing says where `On_Kol` attaches
  to `Alt_Kol`, or at what offset.
- **Joint axes are UNKNOWN.** No axis direction can be read off a part.
- **Part → link assignment is INFERRED** from the Turkish names and measured
  sizes (`Alt`=lower, `Ust`=upper, `Govde`=body, `Kasa`=case, `Kol`=arm,
  `On`=fore, `Bilek`=wrist, `El`=hand, `Parmak`=finger, `Tabla`=plate).
- **Which face is outward** on symmetric parts (`El`, `Alt_Kapak`) is unknown.
- **Cross-part baselines are UNKNOWN** — `El`→`El_Ust` and the finger separation
  are not derivable. The finger separation *is* the J6 observable, so **J6 has no
  predicted baseline anywhere in this design.**

**Consequence, stated plainly: markers alone give a *change* in orientation, not
an angle.** Absolute angles require two calibration passes documented in
`Documentation/MARKER-SYSTEM.md` §6 — axis identification (command each joint
alone across its range and fit a rotation axis to the child-marker trajectory,
which *recovers* the transforms the STLs cannot give) and datum capture at
commanded home. Neither has been run.

### Facts about the arm that bound this work

- **`D3`'s motor identity is DISPUTED.** `Calibration_Notes/calibration-log.csv`
  (2026-08-01) records the servo physically wired to D3 as the **gripper**, not
  the base. The link→joint mapping in §2 inherits that dispute. **This marker set
  is the instrument that settles it** — command J0 alone and observe which link
  moves.
- **J4's `0–180` range is flagged SUSPECT** (it is the servo's whole *electrical*
  range, so it was probably never driven to a real stop). Wrist occlusion is
  analysed over travel that may not be physically reachable.
- **J6 is UNCALIBRATED.** There is no trustworthy gripper travel to design
  against at all.
- **J1's `mirror_offset_deg` is 0 and unmeasured.** A persistent non-zero
  shoulder residual would be the first observable exposing it — two MG996R
  servos fighting each other by a fixed angle is exactly a constant residual.
- **Power blocks the calibration passes.** The bench supply is ~700 mA and
  **cannot hold an assembled arm**; a 3–5 A supply and a 1 A slow-blow fuse are
  outstanding. Axis identification requires driving joints on an assembled arm,
  so this is a hard prerequisite. The MG90S units are also being driven at
  6.62 V against a 6.0 V rating.

### Open items in this document

- **A1 — camera HFOV is unverified.** Every size and grade scales with it.
  Settle with the ChArUco board before printing.
- **BASE-1/2 cap is unexplained.** The sizing rule permits 45 mm; the table uses
  36 mm. The effect is benign but the reason is a reconstruction, not a record.
- **Centroid noise 0.2 px is assumed**, not measured on this camera. A static
  N-frame capture settles it.

### Vocabulary and safety

Values recovered from these markers are **observed** — a camera actually saw
them, which is the one approved exception to this project's ban on "measured"
for shaft state. They are observations of a **printed part**, never of a servo
output shaft. `.pos` remains **commanded**. The two must never be collapsed into
one field.

**Nothing in this document is an emergency stop.** `STP` aborts motion and
**holds** with joints still driven; `EST` and the watchdog **detach**, and a
gravity-loaded arm **sags**. The rocker switch and the inline fuse are the only
real stop.

### Licences

- **OpenCV** — Apache-2.0. Compliant.
- **numpy** — BSD-3-Clause. **FLAGGED**, not silently accepted: the stated rule
  is Apache-2.0 or MIT only. Unavoidable for this work and for LeRobot. Raised
  for a decision.
- **pyserial** — BSD-3-Clause. Same flag; already unavoidable in `arm-bridge.py`.
