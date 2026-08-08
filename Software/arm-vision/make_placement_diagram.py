#!/usr/bin/env python3
"""
make_placement_diagram.py - generate docs/marker_placement_diagram.svg, the
engineering drawing that says WHERE each fiducial sticker goes and, just as
loudly, where it must not.

WHY THIS EXISTS
    Software/vision/markers.csv gets the sizes right but describes positions in
    prose ("survey square centre on the y=+15 face"). Prose cannot show that the
    inviting fill-1.00 rails on Alt_Kol are inner channel walls you physically
    cannot reach, and it cannot show that a 6 mm finger sticker is a speck next
    to a 36 mm base sticker. A drawing can. Both of those are the difference
    between a marker rig that works and one that quietly does not.

WHAT IS TO SCALE AND WHAT IS NOT - read this before trusting any distance
    TO SCALE, because it is measured:
      * every link outline. Each link is drawn as the FACE ITS MARKER SITS ON:
        the two bounding-box extents perpendicular to that face's normal, read
        from the real STL in Backups/STL_parts/ at run time. The third extent
        (depth into the page) is NOT drawn.
      * every marker footprint, in the same scale as the links. That is
        deliberate: the size disparity across the arm IS the headline finding
        and it should be visible without reading a caption.
      * the three measured two-marker baselines, dimensioned.
      * ELEVEN OF THIRTEEN STICKER POSITIONS. See the next block - this used to
        be the weakest claim on the sheet and it is now derived, not draughted.

    WHERE EACH STICKER SITS - derived, and gated
        Sticker positions used to be hand-authored draughting constants that
        merely LOOKED like the survey, and the drawing did not admit it. They
        are now read from Software/vision/stl-face-survey.csv. For each marker
        the SurveyCell in PLACEMENT names one exact row (part_file + face normal
        + plane_offset_mm) and one exact column pair (square1_cx/cy/cz_mm or
        square2_cx/cy/cz_mm). That surveyed point, in the part's own coordinate
        frame, IS the drawn position:

            u = centre[axis_along]  - bbox_min[axis_along]
            v = centre[axis_across] - bbox_centre[axis_across]

        Three gates make the claim checkable rather than asserted:
          ROW GATE   the (part, normal, offset) address must select EXACTLY one
                     survey row. Alt_Kasa y+, Alt_Kol y+ and On_Kol z+ each have
                     several near-parallel faces; taking "the first match" would
                     have drawn SHOULDER from the wrong face and still looked
                     plausible. 0 or >1 matches is a hard error.
          CELL GATE  the selected row's fill_ratio and square size must agree
                     with markers.csv patch_fill_ratio and
                     measured_inscribed_square_mm for that marker. This proves
                     the row chosen is the row markers.csv sized the sticker
                     from - square1 checks largest_inscribed_square_mm, square2
                     checks second_square_mm.
          PAIR GATE  for every within-part pair, the distance between the two
                     DERIVED centres must reproduce markers.csv pair_baseline_mm.
                     The drawn spacing cannot drift from the dimension.

    A PANEL IS A COORDINATE RECTANGLE, NOT A VIEW
        Each link panel is the part's own bounding rectangle in the two
        part-coordinate axes perpendicular to its marker face: origin at the
        bbox minimum, +u toward increasing part coordinate. It is deliberately
        NOT "the face seen from outside", because VIEW A has no single view
        direction - every panel is a different part's face laid flat for
        legibility - so any left/right claim would be unsupportable. The
        consequence is stated on the sheet: which physical end is u=0 cannot be
        read off the drawing, so the schedule prints the surveyed centre as a
        part-coordinate triple. That triple, not the picture, is the instruction.

    NOT TO SCALE, because it is not known or does not fit:
      * every joint-to-joint offset. The STLs are individual parts in PRINT
        orientation. There is no assembly file, no URDF, no STEP - so the
        transform from one link to the next is UNKNOWN. Those gaps are drawn as
        dashed connectors with a "?" and they are schematic. Do not scale them.
      * the pose. The joint angles drawn are one plausible arrangement chosen to
        make the drawing legible, not a commanded or observed configuration.
      * the camera standoffs. 550 mm and 250 mm would dwarf the page, so those
        lines carry a break symbol and a written distance.
      * TURRET-1 and TURRET-2 - the two markers of thirteen that CANNOT be
        derived. Alt_Govde has no planar patch to sit on; markers.csv records
        patch_normal "cylindrical (no planar patch)" and n/a for both the square
        and the baseline, so there is no cell to trace to. Their drawn spots are
        schematic and are labelled NOT SURVEYED on the sheet and in the schedule.
      * the X do-not-place callouts. The FACE each one condemns is surveyed; the
        spot the X is struck at is chosen for legibility.

    INFERRED, not measured: which link belongs to which joint. Part-to-link
    assignment comes from the Turkish part names and the measured sizes. Worse,
    the D3 motor identity is DISPUTED (Calibration_Notes/calibration-log.csv,
    2026-08-01: the servo physically on D3 was the GRIPPER, not the Base). This
    marker rig is the instrument that settles it - command J0 alone and watch
    which link moves.

WHY BALLOONS AND A SCHEDULE, NOT LABELS ON THE GEOMETRY
    Thirteen markers each needing size, camera, standoff and grade is more text
    than the distal end of this arm has room for; laid out in place it collides
    and becomes unreadable exactly where the markers are smallest. So the
    geometry carries the footprint (at true scale, filled by grade) plus a
    numbered balloon, and the MARKER SCHEDULE carries the numbers. That is how
    an engineering drawing handles it, and it is why the layout gate below can
    guarantee nothing overflows.

LAYOUT IS GATED, NOT EYEBALLED
    Every panel is a Panel object and every string drawn into one is recorded
    with its estimated extent. build() fails loudly if anything lands outside
    the panel that owns it. A diagram that silently ran text off the sheet would
    be worse than no diagram, because it would look finished.

USAGE
    python Software/arm-vision/make_placement_diagram.py
    python Software/arm-vision/make_placement_diagram.py --out other.svg

DEPENDENCIES
    numpy   BSD-3-Clause. FLAGGED, not silently accepted - the repository rule
            is Apache-2.0 or MIT only. Already unavoidable for the vision work.
    stdlib  argparse, csv, math, os, struct, sys, xml.sax.saxutils.
    opencv is NOT needed here; this script draws, it does not detect.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import struct
from dataclasses import dataclass, field
from xml.sax.saxutils import escape as xml_escape

import numpy as np


# --------------------------------------------------------------------------
# Page. A3 landscape, viewBox in millimetres: 1 user unit == 1 mm.
# --------------------------------------------------------------------------

PAGE_W, PAGE_H = 420.0, 297.0

INK = "#111111"
THIN = "#6b6b6b"
GHOST = "#c2c2c2"
PAPER = "#ffffff"
WARN = "#b3261e"       # the one accent, reserved for "do not" and "below floor"
UNKNOWN = "#6a3fb5"    # the one other accent, reserved for "this is not known"

GLYPH_W = 0.54         # Helvetica character width as a fraction of font size

# Grade -> how a marker footprint is drawn. Grade is a property of the marker
# AND its camera standoff; both are in the schedule so the encoding cannot be
# read as an absolute property of the sticker.
GRADE_STYLE = {
    "pose":        dict(fill=INK,   stroke=INK,  dash="",        label="POSE   >= 36 px"),
    "detect":      dict(fill="#8e8e8e", stroke=INK, dash="",     label="DETECT >= 24 px"),
    "below-floor": dict(fill=PAPER, stroke=WARN, dash="0.9 0.7", label="BELOW FLOOR < 24 px"),
}


# --------------------------------------------------------------------------
# STL bounding boxes - measured at run time, never transcribed
# --------------------------------------------------------------------------

def stl_bounds(path: str) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """Binary STL bbox as (mins, maxs). Validated by the exact arithmetic
    84 + 50n == filesize, the same gate Software/vision/stl_face_survey.py uses:
    a mismatch is a hard error, never a silent misparse of an ASCII or truncated
    file.

    Returns MINS as well as extents because the panel origin is now the part's
    own bbox minimum - that is what makes a surveyed part-coordinate centre
    convertible into a drawn position at all.
    """
    size = os.path.getsize(path)
    with open(path, "rb") as fh:
        fh.seek(80)
        n = struct.unpack("<I", fh.read(4))[0]
        if 84 + 50 * n != size:
            raise SystemExit(
                f"ERROR: {path} is not a valid binary STL (header says {n} triangles "
                f"= {84 + 50 * n} bytes, file is {size})")
        raw = np.frombuffer(fh.read(50 * n), dtype=np.uint8).reshape(n, 50)
    tri = raw[:, 12:48].copy().view("<f4").reshape(n * 3, 3)
    return (tuple(float(v) for v in tri.min(0)), tuple(float(v) for v in tri.max(0)))


_BOUNDS_CACHE: dict[str, tuple[tuple[float, ...], tuple[float, ...]]] = {}


def bounds_for(stl_dir: str, part: str):
    if part not in _BOUNDS_CACHE:
        path = os.path.join(stl_dir, part)
        if not os.path.isfile(path):
            raise SystemExit(f"ERROR: STL not found: {path}")
        _BOUNDS_CACHE[part] = stl_bounds(path)
    return _BOUNDS_CACHE[part]


# --------------------------------------------------------------------------
# The kinematic chain. Link order and joint assignment are INFERRED - the
# `why` string on each row is the evidence, and it is printed on the drawing.
# `axes` names which two bbox axes are drawn: (along the link, across it).
# `angle` and `gap` are DRAWING POSE ONLY - neither is a commanded value.
# --------------------------------------------------------------------------

@dataclass
class Link:
    name: str
    part: str
    joint_above: str | None
    axes: tuple[int, int]
    why: str
    angle: float
    gap: float
    attach: tuple[float, float] = (1.0, 0.0)   # (u fraction, v fraction) on prev
    joint_note: str = ""
    branch_of: str | None = None
    length: float = 0.0
    across: float = 0.0
    anchor: tuple[float, float] = (0.0, 0.0)
    bbmin: tuple[float, ...] = (0.0, 0.0, 0.0)
    bbmax: tuple[float, ...] = (0.0, 0.0, 0.0)
    face: tuple[float, float, float] | None = None   # marker face normal, or None
                                                     # if the part has no planar
                                                     # patch (Alt_Govde)


LINKS: list[Link] = [
    Link("GROUND", "Alt_Kasa.stl", None, (0, 2),
         "marker face normal (0,1,0) -> the x-z face is drawn", 0.0, 0.0,
         face=(0.0, 1.0, 0.0)),
    Link("TURRET", "Alt_Govde.stl", "J0", (2, 0),
         "no flat outward face; body of revolution about J0 -> height x diameter",
         90.0, 14.0, attach=(0.5, 1.0), joint_note="29-110 deg", face=None),
    Link("UPPER_ARM", "Alt_Kol.stl", "J1", (2, 0),
         "marker face normal (0,1,0) -> the x-z face is drawn", 72.0, 16.0,
         joint_note="0-91 deg, D4+D5", face=(0.0, 1.0, 0.0)),
    Link("FOREARM", "On_Kol.stl", "J3", (0, 1),
         "marker face normal (0,0,1) -> the x-y face is drawn", 0.0, 18.0,
         joint_note="0-66 deg", face=(-0.0436, 0.0, 0.999)),
    Link("WRIST", "Bilek.stl", "J4", (1, 2),
         "marker face normal (-1,0,0) -> the y-z face is drawn", -34.0, 26.0,
         joint_note="0-180 SUSPECT", face=(-1.0, 0.0005, 0.0)),
    Link("HAND", "El.stl", "J5", (0, 1),
         "marker face normal (0,0,1) -> the x-y face is drawn", -34.0, 30.0,
         joint_note="31-178 deg", face=(0.0, 0.0, 1.0)),
    Link("FINGER_A", "Parmak_2 X 2.stl", "J6", (0, 1),
         "marker face normal (0,0,1) -> the x-y face is drawn", 2.0, 24.0,
         joint_note="UNCALIBRATED", face=(0.0, 0.0, 1.0)),
    Link("FINGER_B", "Parmak_2 X 2.stl", None, (0, 1),
         "second printed copy of the same part", -62.0, 24.0, branch_of="FINGER_A",
         face=(0.0, 0.0, 1.0)),
]

# Where each link's name/part/size caption sits, as a paper-mm offset from the
# link centre. Small links cannot hold their own caption without it landing on
# top of a marker, so those are pushed clear and given a leader.
LABEL_OFFSET = {
    "GROUND": (42.0, -14.0), "TURRET": (-34.0, -7.5), "UPPER_ARM": (0.0, 12.0),
    "FOREARM": (0.0, -13.0), "WRIST": (-17.0, 13.0), "HAND": (-13.0, 20.0),
    "FINGER_A": (25.0, -7.0), "FINGER_B": (17.0, 14.0),
}

# --------------------------------------------------------------------------
# WHERE EACH STICKER SITS. This table no longer holds positions - it holds the
# ADDRESS of the surveyed cell each position is read from. A SurveyCell names
# exactly one row of Software/vision/stl-face-survey.csv (part_file + face
# normal + plane_offset_mm) and exactly one column pair on that row
# (square1_cx/cy/cz_mm or square2_cx/cy/cz_mm). u and v are computed from that
# point in load-time code; there is deliberately nowhere here to type a
# millimetre by hand.
#
# cell=None means NOT SURVEYED - only Alt_Govde's two turret markers, which have
# no planar patch to sit on. Those keep an explicit schematic u/v and are drawn
# and scheduled as NOT SURVEYED. balloon is (bearing_deg, paper_mm) for the
# leader; it is pure draughting and moves nothing physical.
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class SurveyCell:
    part: str                                # part_file
    normal: tuple[float, float, float]       # normal_x/y/z of the row
    offset: float                            # plane_offset_mm of the row
    square: str                              # "square1" or "square2"

    def label(self) -> str:
        return f"{self.part} n={fmt_normal(self.normal)} off={self.offset:g} {self.square}"


def fmt_normal(n: tuple[float, float, float]) -> str:
    return "(" + ";".join(f"{v:+.2f}".rstrip("0").rstrip(".") if v else "0"
                          for v in n) + ")"


PLACEMENT = {
    # BASE-2 is the LOW-u square and BASE-1 the high-u one, so their balloons
    # splay outward (left / right) and clear the baseline dimension between them.
    "BASE-1":     dict(link="GROUND", balloon=(290, 16),
                       cell=SurveyCell("Alt_Kasa.stl", (0.0, 1.0, 0.0), 59.81, "square1")),
    "BASE-2":     dict(link="GROUND", balloon=(250, 16),
                       cell=SurveyCell("Alt_Kasa.stl", (0.0, 1.0, 0.0), 59.81, "square2")),
    "TURRET-1":   dict(link="TURRET", balloon=(340, 15), cell=None, u=30.0, v=0.0),
    "TURRET-2":   dict(link="TURRET", balloon=(20, 14), cell=None, u=30.0, v=48.0,
                       edge_on=True),
    "SHOULDER-1": dict(link="UPPER_ARM", balloon=(200, 17),
                       cell=SurveyCell("Alt_Kol.stl", (0.0, 1.0, 0.0), 15.00, "square1")),
    "SHOULDER-2": dict(link="UPPER_ARM", balloon=(195, 19),
                       cell=SurveyCell("Alt_Kol.stl", (0.0, 1.0, 0.0), 15.00, "square2")),
    # Both ELBOW balloons hang BELOW the forearm: the surveyed centres sit under
    # the link caption, and leaders drawn upward printed straight through it.
    "ELBOW-1":    dict(link="FOREARM", balloon=(300, 16),
                       cell=SurveyCell("On_Kol.stl", (-0.0436, 0.0, 0.999), 19.61,
                                       "square1")),
    "ELBOW-2":    dict(link="FOREARM", balloon=(240, 16),
                       cell=SurveyCell("On_Kol.stl", (-0.0436, 0.0, 0.999), 19.61,
                                       "square2")),
    "WRIST-1":    dict(link="WRIST", balloon=(150, 14),
                       cell=SurveyCell("Bilek.stl", (-1.0, 0.0005, 0.0), 21.50, "square1")),
    "HAND-1":     dict(link="HAND", balloon=(78, 18),
                       cell=SurveyCell("El.stl", (0.0, 0.0, 1.0), 3.00, "square1")),
    # El_Ust is a DIFFERENT part from El. Its surveyed centre is in El_Ust's own
    # frame and is drawn on El's panel only because the two share a 50 x 48.3 mm
    # footprint - an INFERENCE, checked at load time, flagged "? cross-part".
    "HAND-2":     dict(link="HAND", balloon=(28, 16), cross_part=True,
                       cell=SurveyCell("El_Ust.stl", (0.0, 0.0, -1.0), 0.00, "square2")),
    "FINGER-A":   dict(link="FINGER_A", balloon=(30, 16),
                       cell=SurveyCell("Parmak_2 X 2.stl", (0.0, 0.0, 1.0), 5.00,
                                       "square1")),
    # SAME cell as FINGER-A on purpose: the two fingers are two printed copies of
    # one part, not two islands on one part. The row's own square2 belongs to the
    # SAME finger and must not be borrowed for the opposing one.
    "FINGER-B":   dict(link="FINGER_B", balloon=(-55, 16),
                       cell=SurveyCell("Parmak_2 X 2.stl", (0.0, 0.0, 1.0), 5.00,
                                       "square1")),
}

# Drawn as x callouts ON the geometry. Every one is a face that looks usable in
# Software/vision/stl-face-survey.csv and is not. (link, u frac, v frac, bearing,
# leader mm, text)
DO_NOT_PLACE = [
    ("GROUND", 0.86, 1.0, 330, 17, 34,
     "TOP FACE of Alt_Kasa - fill 0.39, open for servo access. Biggest bbox on the arm; "
     "only a 24.8 mm square fits."),
    ("TURRET", 0.06, 0.30, 168, 46, 26,
     "z=0 face of Alt_Govde - 114 x 114 looks usable but it is a RECESSED LEDGE with "
     "50 mm of structure above it."),
    ("UPPER_ARM", 0.62, 1.0, 172, 46, 32,
     "x=+/-22.5 RAILS of Alt_Kol - 198.5 x 20.1 at fill 1.00, the most inviting numbers "
     "on the part. INNER CHANNEL WALLS: the part runs out to x=+/-27.5, so there is "
     "material outboard. A sticker cannot reach them."),
    ("FOREARM", 0.80, -1.0, 265, 26, 34,
     "x=-30.5 face of On_Kol - fill 0.81 and the largest square on the forearm "
     "(25.2 mm). Also an INNER face; the part runs to x=+20. Same for y=-15.2."),
]

EXCLUDED_PARTS = (
    "Disli / Servo_Disli / Mil_Disli / Parmak_Disli - GEARS. They rotate relative to "
    "their own link and must mesh.",
    "Mil_1/2/3 - SHAFTS. 6.8-17 mm cylinders; nothing flat, and they turn.",
    "Servo_Cable_Holder - cables sweep across it.",
    "Jack_Cover - removable. A marker on a removable part is a marker that moves.",
    "Tabla_Alt - 113.7 mm across but a RING at fill 0.22; takes only an 11.6 mm square.",
    "Alt_Kapak - a cover; not verified rigid with any link.",
    "Any face flagged with an X in VIEW A.",
)


# --------------------------------------------------------------------------
# markers.csv
# --------------------------------------------------------------------------

@dataclass
class Marker:
    marker_id: int
    label: str
    link: str
    part: str
    black_mm: float
    sticker_mm: float
    grade: str
    camera: str
    standoff: str
    baseline: str
    orientation: str
    fill: float | None            # markers.csv patch_fill_ratio ("n/a" -> None)
    square: float | None          # markers.csv measured_inscribed_square_mm
    pos: dict = field(default_factory=dict)
    u: float = 0.0                # derived (or schematic for TURRET)
    v: float = 0.0
    centre: tuple[float, float, float] | None = None   # surveyed, part frame
    src: str = ""                 # "sq1" / "sq2" / "" when not surveyed


ORIENT_MAX = 40      # fits the 45 mm ORIENTATION column; gated in draw_schedule


def short_orientation(s: str) -> str:
    """Condense markers.csv preferred_orientation to one schedule cell.

    Rule-based on purpose. A hand-written lookup would go stale the moment the
    CSV changed, and DERIVING an arrow glyph in the drawn frame would be worse
    still: mapping "toward the shoulder end (-z in part coords)" into this
    drawing's frame is an inference the STLs cannot support, and a wrong arrow
    looks authoritative in a way wrong text does not. So: keep the author's own
    words, drop the rationale clause, and truncate visibly if it still will not
    fit. The full sentence stays in markers.csv.
    """
    s = s.split(". ")[0].strip().rstrip(".")
    for clause in (" so the", " so that", " because"):
        i = s.find(clause)
        if i > 0:
            s = s[:i]
    for a, b in (("Marker top edge toward", "top edge ->"),
                 ("Same orientation as", "same as"), ("Same as", "same as"),
                 ("top edge toward", "top edge ->"), (" in part coords", "")):
        s = s.replace(a, b)
    s = " ".join(s.split())
    if len(s) > ORIENT_MAX:
        s = s[:ORIENT_MAX - 4].rsplit(" ", 1)[0] + " ..."
    return s


def load_markers(path: str) -> list[Marker]:
    with open(path, "r", encoding="utf-8", newline="") as fh:
        rows = list(csv.reader(
            l for l in fh if not l.lstrip().startswith("#") and l.strip()))
    if not rows:
        raise SystemExit(f"ERROR: {path} has no data rows")
    hdr = [c.strip() for c in rows[0]]
    need = ("marker_id", "human_label", "link_name", "black_square_mm",
            "approximate_size_mm", "grade_at_design_standoff", "camera",
            "design_standoff_mm", "pair_baseline_mm", "target_part_file",
            "preferred_orientation", "patch_fill_ratio",
            "measured_inscribed_square_mm")
    missing = [c for c in need if c not in hdr]
    if missing:
        raise SystemExit(f"ERROR: {path} missing column(s): {missing}")
    ix = {c: hdr.index(c) for c in need}

    out: list[Marker] = []
    for lineno, r in enumerate(rows[1:], start=2):
        if len(r) != len(hdr):
            raise SystemExit(
                f"ERROR: {path} row {lineno} has {len(r)} fields, header has {len(hdr)}")
        g = r[ix["grade_at_design_standoff"]].strip()
        if g not in GRADE_STYLE:
            raise SystemExit(f"ERROR: {path} row {lineno}: unknown grade {g!r}; "
                             f"expected one of {sorted(GRADE_STYLE)}")
        m = Marker(
            marker_id=int(r[ix["marker_id"]]),
            label=r[ix["human_label"]].strip(),
            link=r[ix["link_name"]].strip(),
            part=r[ix["target_part_file"]].strip(),
            black_mm=float(r[ix["black_square_mm"]]),
            sticker_mm=float(r[ix["approximate_size_mm"]]),
            grade=g,
            camera=r[ix["camera"]].strip(),
            standoff=r[ix["design_standoff_mm"]].strip(),
            baseline=r[ix["pair_baseline_mm"]].strip(),
            orientation=short_orientation(r[ix["preferred_orientation"]]),
            fill=_opt_float(r[ix["patch_fill_ratio"]]),
            square=_opt_float(r[ix["measured_inscribed_square_mm"]]),
        )
        if not m.orientation:
            raise SystemExit(
                f"ERROR: {path} row {lineno}: preferred_orientation is empty for "
                f"{m.label}. Sheet 1 prints an UP arrow on every sticker label and "
                "promises it means something; this column is the only thing that says "
                "what. A blank one silently breaks the residual sign convention.")
        if m.label not in PLACEMENT:
            raise SystemExit(
                f"ERROR: {path} row {lineno}: marker {m.label!r} has no PLACEMENT entry. "
                "Add one rather than letting it be silently omitted from the drawing.")
        m.pos = PLACEMENT[m.label]
        out.append(m)
    return out


# --------------------------------------------------------------------------
# stl-face-survey.csv -> drawn position. Three gates; see the module docstring.
# --------------------------------------------------------------------------

SURVEY_NEED = ("part_file", "normal_x", "normal_y", "normal_z", "plane_offset_mm",
               "fill_ratio", "largest_inscribed_square_mm", "second_square_mm",
               "square1_cx_mm", "square1_cy_mm", "square1_cz_mm",
               "square2_cx_mm", "square2_cy_mm", "square2_cz_mm")

NORMAL_DOT_MIN = 0.995      # a face is "the same face" only this parallel
OFFSET_TOL_MM = 0.05        # plane_offset_mm is written to 2 dp
FILL_TOL = 0.006            # markers.csv rounds fill to 2 dp, survey to 3
SQUARE_TOL_MM = 0.06        # markers.csv rounds the square to 1 dp
BASELINE_TOL_MM = 0.15      # markers.csv rounds pair_baseline_mm to 1 dp; the
                            # two tilted-normal faces add ~0.02 mm of projection


def _opt_float(s: str) -> float | None:
    s = s.strip()
    try:
        return float(s)
    except ValueError:
        return None


def load_survey(path: str) -> list[dict]:
    if not os.path.isfile(path):
        raise SystemExit(f"ERROR: survey not found: {path}")
    with open(path, "r", encoding="utf-8", newline="") as fh:
        rows = list(csv.DictReader(
            l for l in fh if not l.lstrip().startswith("#") and l.strip()))
    if not rows:
        raise SystemExit(f"ERROR: {path} has no data rows")
    missing = [c for c in SURVEY_NEED if c not in rows[0]]
    if missing:
        raise SystemExit(f"ERROR: {path} missing column(s): {missing}")
    return rows


def select_row(survey: list[dict], cell: SurveyCell, label: str) -> dict:
    """ROW GATE. The (part, normal, offset) address must select EXACTLY one row.

    Alt_Kasa y+, Alt_Kol y+ and On_Kol z+ each carry several near-parallel faces
    at different plane offsets; "first match wins" would silently draw a marker
    from the wrong face and the result would look entirely plausible. Nothing
    downstream can detect that, so it is caught here or not at all.
    """
    mag = math.sqrt(sum(v * v for v in cell.normal)) or 1.0
    want = tuple(v / mag for v in cell.normal)
    hits = []
    for r in survey:
        if r["part_file"] != cell.part:
            continue
        n = (float(r["normal_x"]), float(r["normal_y"]), float(r["normal_z"]))
        nm = math.sqrt(sum(v * v for v in n)) or 1.0
        if sum(a * b / nm for a, b in zip(want, n)) < NORMAL_DOT_MIN:
            continue
        if abs(float(r["plane_offset_mm"]) - cell.offset) > OFFSET_TOL_MM:
            continue
        hits.append(r)
    if len(hits) != 1:
        raise SystemExit(
            f"ERROR: {label}: survey address {cell.label()} selected {len(hits)} rows, "
            "expected exactly 1. A marker position must trace to ONE cell; refusing to "
            "guess which near-parallel face was meant.")
    return hits[0]


def derive_positions(markers: list[Marker], by: dict[str, Link], survey: list[dict],
                     stl_dir: str) -> list[str]:
    """Turn each SurveyCell into a drawn (u, v). Returns the audit lines."""
    audit: list[str] = []
    for m in markers:
        cell: SurveyCell | None = m.pos["cell"]
        lk = by[m.link]
        if cell is None:
            m.u, m.v, m.src, m.centre = m.pos["u"], m.pos["v"], "", None
            audit.append(f"  {m.label:<11} NOT SURVEYED - schematic "
                         f"(u={m.u:.1f} v={m.v:.1f} on {lk.part})")
            continue

        row = select_row(survey, cell, m.label)
        k = "1" if cell.square == "square1" else "2"
        c = (float(row[f"square{k}_cx_mm"]), float(row[f"square{k}_cy_mm"]),
             float(row[f"square{k}_cz_mm"]))
        sq = float(row["largest_inscribed_square_mm" if k == "1" else "second_square_mm"])
        fill = float(row["fill_ratio"])

        # CELL GATE - the row selected must be the row markers.csv sized this
        # sticker from. Without this, the ROW GATE only proves uniqueness, not
        # correctness: a unique but wrong face would still pass.
        if m.fill is not None and abs(m.fill - fill) > FILL_TOL:
            raise SystemExit(
                f"ERROR: {m.label}: markers.csv patch_fill_ratio={m.fill} but survey row "
                f"{cell.label()} has fill_ratio={fill}. The drawing would be positioned "
                "from a different face than the sticker was sized from.")
        if m.square is not None and abs(m.square - sq) > SQUARE_TOL_MM:
            raise SystemExit(
                f"ERROR: {m.label}: markers.csv measured_inscribed_square_mm={m.square} "
                f"but survey {cell.label()} gives {sq}. Same mismatch as above.")

        # The point is in the SURVEY part's frame. When that part is not the one
        # whose panel is drawn (HAND-2 on El_Ust vs the El panel) the two must at
        # least agree on the drawn axes, or the co-registration is fiction.
        smin, smax = bounds_for(stl_dir, cell.part)
        if cell.part != lk.part:
            for ax in lk.axes:
                if (abs(smin[ax] - lk.bbmin[ax]) > 0.05
                        or abs(smax[ax] - lk.bbmax[ax]) > 0.05):
                    raise SystemExit(
                        f"ERROR: {m.label}: {cell.part} is drawn on the {lk.part} panel but "
                        f"their axis-{ax} bounds differ ({smin[ax]:.2f}..{smax[ax]:.2f} vs "
                        f"{lk.bbmin[ax]:.2f}..{lk.bbmax[ax]:.2f}). The shared-footprint "
                        "inference does not hold; do not draw it as if it did.")

        a0, a1 = lk.axes
        m.u = c[a0] - smin[a0]
        m.v = c[a1] - (smin[a1] + smax[a1]) / 2.0
        m.centre = c
        m.src = "sq" + k
        audit.append(f"  {m.label:<11} {cell.label():<52} "
                     f"centre=({c[0]:.2f};{c[1]:.2f};{c[2]:.2f}) -> u={m.u:7.2f} v={m.v:+6.2f}")
    return audit


def check_baselines(markers: list[Marker]) -> list[str]:
    """PAIR GATE. Every within-part pair's DERIVED spacing must reproduce
    markers.csv pair_baseline_mm, so the drawn gap cannot drift from the
    dimension printed beside it."""
    out: list[str] = []
    seen: set[tuple[str, str]] = set()
    for m in markers:
        want = _opt_float(m.baseline)
        if want is None or m.centre is None:
            continue
        key = (m.link, m.baseline)
        if key in seen:
            continue
        sibs = [k for k in markers
                if k.link == m.link and k.baseline == m.baseline and k.centre is not None]
        if len(sibs) != 2:
            continue
        seen.add(key)
        a, b = sibs
        got = math.dist((a.u, a.v), (b.u, b.v))
        if abs(got - want) > BASELINE_TOL_MM:
            raise SystemExit(
                f"ERROR: {a.label}/{b.label}: derived spacing {got:.3f} mm but markers.csv "
                f"pair_baseline_mm={want}. Tolerance {BASELINE_TOL_MM} mm. Either the wrong "
                "survey cells are addressed or the CSV disagrees with itself.")
        out.append(f"  {a.label:>11} <-> {b.label:<11} derived {got:8.3f} mm   "
                   f"csv {want:7.2f} mm   delta {got - want:+.3f} mm")
    return out


# --------------------------------------------------------------------------
# Geometry. Computed in a y-UP millimetre frame and converted to SVG (y-down) at
# emit time, so no mirroring transform can flip a label.
# --------------------------------------------------------------------------

def unit(a_deg: float):
    a = math.radians(a_deg)
    return (math.cos(a), math.sin(a)), (-math.sin(a), math.cos(a))


def local_to_world(lk: Link, u: float, v: float) -> tuple[float, float]:
    d, p = unit(lk.angle)
    return (lk.anchor[0] + d[0] * u + p[0] * v, lk.anchor[1] + d[1] * u + p[1] * v)


def build_chain(stl_dir: str) -> tuple[list[Link], dict[str, Link]]:
    by: dict[str, Link] = {}
    for lk in LINKS:
        lk.bbmin, lk.bbmax = bounds_for(stl_dir, lk.part)
        lk.length = lk.bbmax[lk.axes[0]] - lk.bbmin[lk.axes[0]]
        lk.across = lk.bbmax[lk.axes[1]] - lk.bbmin[lk.axes[1]]
        by[lk.name] = lk

    LINKS[0].anchor = (-LINKS[0].length / 2.0, LINKS[0].across / 2.0)
    for i, lk in enumerate(LINKS):
        if i == 0:
            continue
        prev = by[lk.branch_of] if lk.branch_of else LINKS[i - 1]
        base = prev if not lk.branch_of else by[lk.branch_of]
        att_src = by[base.branch_of] if base.branch_of else (
            LINKS[LINKS.index(base) - 1] if lk.branch_of else prev)
        src = att_src if lk.branch_of else prev
        p = local_to_world(src, src.length * lk.attach[0],
                           src.across * 0.5 * lk.attach[1])
        d, _ = unit(lk.angle)
        lk.anchor = (p[0] + d[0] * lk.gap, p[1] + d[1] * lk.gap)
        lk.joint_from = p                                     # type: ignore[attr-defined]
    return LINKS, by


def link_corners(lk: Link):
    h = lk.across / 2.0
    return [local_to_world(lk, 0, -h), local_to_world(lk, lk.length, -h),
            local_to_world(lk, lk.length, h), local_to_world(lk, 0, h)]


# --------------------------------------------------------------------------
# SVG + the layout gate
# --------------------------------------------------------------------------

def t(x: float) -> str:
    return f"{x:.3f}".rstrip("0").rstrip(".") or "0"


class Overflow(Exception):
    pass


class Panel:
    """A framed region. Everything drawn through it is bounds-checked, so a
    layout that runs off its panel fails the build instead of shipping."""

    def __init__(self, name, x, y, w, h, title="", sub="", slack=1.0):
        self.name, self.x, self.y, self.w, self.h = name, x, y, w, h
        self.slack = slack
        self.out: list[str] = []
        self.bad: list[str] = []
        if title:
            self.out += [
                f'<rect x="{t(x)}" y="{t(y)}" width="{t(w)}" height="{t(h)}" '
                f'fill="none" stroke="{INK}" stroke-width="0.4"/>',
                f'<line x1="{t(x)}" y1="{t(y+7.5)}" x2="{t(x+w)}" y2="{t(y+7.5)}" '
                f'stroke="{INK}" stroke-width="0.4"/>']
            self.raw(self._text(x + 2.5, y + 5.3, title, 3.4, weight="bold"))
            if sub:
                self.raw(self._text(x + w - 2.5, y + 5.3, sub, 2.3,
                                    anchor="end", fill=THIN))

    # -- primitives ---------------------------------------------------------
    def raw(self, s: str) -> None:
        self.out.append(s)

    def _check(self, x0, x1, y0, y1, what) -> None:
        s = self.slack
        if (x0 < self.x - s or x1 > self.x + self.w + s
                or y0 < self.y - s or y1 > self.y + self.h + s):
            self.bad.append(f"{self.name}: {what} at ({x0:.1f}..{x1:.1f}, "
                            f"{y0:.1f}..{y1:.1f}) escapes "
                            f"({self.x:.1f}..{self.x+self.w:.1f}, "
                            f"{self.y:.1f}..{self.y+self.h:.1f})")

    def _text(self, x, y, s, size, *, anchor="start", fill=INK,
              weight="normal", style="") -> str:
        wpx = len(s) * size * GLYPH_W
        x0 = x if anchor == "start" else (x - wpx if anchor == "end" else x - wpx / 2)
        self._check(x0, x0 + wpx, y - size, y + size * 0.35, repr(s[:38]))
        st = f' font-style="{style}"' if style else ""
        return (f'<text x="{t(x)}" y="{t(y)}" font-family="Helvetica, Arial, sans-serif" '
                f'font-size="{t(size)}" font-weight="{weight}" fill="{fill}" '
                f'text-anchor="{anchor}"{st}>{xml_escape(s)}</text>')

    def text(self, x, y, s, size=2.3, **kw) -> None:
        self.out.append(self._text(x, y, s, size, **kw))

    def block(self, x, y, s, size=2.05, width=60, lh=2.75, **kw) -> float:
        for ln in wrap(s, width):
            self.text(x, y, ln, size, **kw)
            y += lh
        return y

    def circle(self, cx, cy, r, **kw):
        self._check(cx - r, cx + r, cy - r, cy + r, "circle")
        attr = " ".join(f'{k.replace("_", "-")}="{v}"' for k, v in kw.items())
        self.out.append(f'<circle cx="{t(cx)}" cy="{t(cy)}" r="{t(r)}" {attr}/>')


def wrap(s: str, width: int) -> list[str]:
    words, lines, cur = s.split(), [], ""
    for w in words:
        if cur and len(cur) + 1 + len(w) > width:
            lines.append(cur)
            cur = w
        else:
            cur = f"{cur} {w}".strip()
    if cur:
        lines.append(cur)
    return lines


class View:
    """Maps y-up model millimetres into a page rectangle, uniformly scaled."""

    def __init__(self, x, y, w, h, pts, pad=6.0):
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        mw, mh = max(xs) - min(xs), max(ys) - min(ys)
        self.s = min((w - 2 * pad) / mw, (h - 2 * pad) / mh)
        self.ox = x + pad + ((w - 2 * pad) - mw * self.s) / 2.0 - min(xs) * self.s
        self.oy = y + h - pad - ((h - 2 * pad) - mh * self.s) / 2.0 + min(ys) * self.s

    def __call__(self, p):
        return (self.ox + p[0] * self.s, self.oy - p[1] * self.s)

    def poly(self, pts) -> str:
        return " ".join(f"{t(a)},{t(b)}" for a, b in (self(p) for p in pts))


def x_mark(cx, cy, r, color=WARN) -> str:
    return (f'<path d="M {t(cx-r)} {t(cy-r)} L {t(cx+r)} {t(cy+r)} '
            f'M {t(cx+r)} {t(cy-r)} L {t(cx-r)} {t(cy+r)}" stroke="{color}" '
            f'stroke-width="0.55" fill="none"/>'
            f'<circle cx="{t(cx)}" cy="{t(cy)}" r="{t(r*1.6)}" fill="none" '
            f'stroke="{color}" stroke-width="0.35"/>')


def balloon(p: Panel, cx, cy, tx, ty, n, color=INK) -> None:
    p.raw(f'<line x1="{t(cx)}" y1="{t(cy)}" x2="{t(tx)}" y2="{t(ty)}" '
          f'stroke="{THIN}" stroke-width="0.3"/>')
    p.raw(f'<circle cx="{t(cx)}" cy="{t(cy)}" r="0.6" fill="{THIN}"/>')
    p.circle(tx, ty, 2.9, fill=PAPER, stroke=color, stroke_width="0.5")
    p.text(tx, ty + 1.0, str(n), 2.7, anchor="middle", weight="bold", fill=color)


# --------------------------------------------------------------------------
# VIEW A - side elevation of the whole chain
# --------------------------------------------------------------------------

def draw_view_a(x, y, w, h, links, by, markers) -> Panel:
    p = Panel("VIEW A", x, y, w, h, "VIEW A - LINK FACE PANELS",
              "panels + footprints + 11 of 13 sticker positions to scale; "
              "joint offsets are NOT")

    pts = []
    for lk in links:
        pts += link_corners(lk)
    xs = [q[0] for q in pts]
    ys = [q[1] for q in pts]
    # headroom so balloons and X callouts stay inside the frame. The left margin
    # is wide because the do-not-place callouts live in that column.
    pts += [(min(xs) - 150, min(ys) - 30), (max(xs) + 95, max(ys) + 55)]
    v = View(x, y + 8, w, h - 20, pts, pad=4.0)

    # ---- schematic joint connectors ---------------------------------------
    for lk in links:
        if lk.joint_above is None:
            continue
        a = v(getattr(lk, "joint_from"))
        b = v(lk.anchor)
        p.raw(f'<line x1="{t(a[0])}" y1="{t(a[1])}" x2="{t(b[0])}" y2="{t(b[1])}" '
              f'stroke="{UNKNOWN}" stroke-width="1.1" stroke-dasharray="1.6 1.2"/>')
        mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
        p.circle(mx, my, 2.4, fill=PAPER, stroke=UNKNOWN, stroke_width="0.4")
        p.text(mx, my + 1.0, "?", 3.0, anchor="middle", fill=UNKNOWN, weight="bold")
        p.text(mx + 3.4, my - 0.4, lk.joint_above, 2.7, weight="bold", fill=UNKNOWN)
        if lk.joint_note:
            p.text(mx + 3.4, my + 2.6, lk.joint_note, 1.95, fill=THIN)

    # ---- links -------------------------------------------------------------
    for lk in links:
        p.raw(f'<polygon points="{v.poly(link_corners(lk))}" fill="#f5f5f5" '
              f'stroke="{INK}" stroke-width="0.45"/>')
        c = v(local_to_world(lk, lk.length * 0.5, 0))
        dx, dy = LABEL_OFFSET.get(lk.name, (0.0, 0.0))
        lx, ly = c[0] + dx, c[1] + dy
        if dx or dy:            # caption pushed clear of the markers -> leader
            p.raw(f'<line x1="{t(c[0])}" y1="{t(c[1])}" x2="{t(lx)}" y2="{t(ly)}" '
                  f'stroke="{GHOST}" stroke-width="0.25"/>')
        p.text(lx, ly - 1.4, lk.name, 2.6, anchor="middle", weight="bold")
        p.text(lx, ly + 1.7, lk.part, 2.1, anchor="middle", fill=THIN, style="italic")
        # The face normal completes the survey-row address: part_file + this
        # normal + the schedule's sq1/sq2 names one cell of stl-face-survey.csv.
        face = (f"face n={fmt_normal(lk.face)}" if lk.face else "- no planar patch")
        p.text(lx, ly + 4.4, f"{lk.length:.0f} x {lk.across:.0f} mm {face}",
               1.95, anchor="middle", fill=THIN)

    # ---- dimensioned baselines --------------------------------------------
    seen = set()
    for m in markers:
        key = (m.link, m.baseline)
        if m.baseline in ("n/a", "unknown-cross-part", "") or key in seen:
            continue
        sibs = [k for k in markers if k.link == m.link and k.baseline == m.baseline]
        if len(sibs) != 2:
            continue
        seen.add(key)
        lk = by[m.link]
        u0, u1 = min(k.u for k in sibs), max(k.u for k in sibs)
        off = lk.across * 0.5 + 9.0
        a, b = v(local_to_world(lk, u0, -off)), v(local_to_world(lk, u1, -off))
        p.raw(f'<line x1="{t(a[0])}" y1="{t(a[1])}" x2="{t(b[0])}" y2="{t(b[1])}" '
              f'stroke="{INK}" stroke-width="0.3" marker-start="url(#dim)" '
              f'marker-end="url(#dim)"/>')
        mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
        p.raw(f'<rect x="{t(mx-13)}" y="{t(my-3.2)}" width="26" height="4.4" '
              f'fill="{PAPER}"/>')
        p.text(mx, my, f"baseline {m.baseline} mm", 2.05, anchor="middle")

    # ---- marker footprints + balloons --------------------------------------
    for m in markers:
        lk = by[m.link]
        st = GRADE_STYLE[m.grade]
        half = m.sticker_mm / 2.0
        # A position that is not fully established gets the sheet's one "unknown"
        # colour on BOTH its footprint outline and its balloon. The words that say
        # WHICH kind of unknown live in the gated note block below, not floating on
        # the geometry: two "NOT SURVEYED" captions on the 60 mm turret panel
        # overprinted each other and the part caption.
        unsure = m.centre is None or bool(m.pos.get("cross_part"))
        stroke = UNKNOWN if unsure else st["stroke"]
        if m.pos.get("edge_on"):
            a = v(local_to_world(lk, m.u - half, m.v))
            b = v(local_to_world(lk, m.u + half, m.v))
            p.raw(f'<line x1="{t(a[0])}" y1="{t(a[1])}" x2="{t(b[0])}" y2="{t(b[1])}" '
                  f'stroke="{stroke}" stroke-width="1.5"/>')
            cx, cy = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
        else:
            corners = [local_to_world(lk, m.u - half, m.v - half),
                       local_to_world(lk, m.u + half, m.v - half),
                       local_to_world(lk, m.u + half, m.v + half),
                       local_to_world(lk, m.u - half, m.v + half)]
            d = st["dash"] or ("1.2 0.9" if unsure else "")
            dash = f' stroke-dasharray="{d}"' if d else ""
            p.raw(f'<polygon points="{v.poly(corners)}" fill="{st["fill"]}" '
                  f'stroke="{stroke}" stroke-width="0.45"{dash}/>')
            cx, cy = v(local_to_world(lk, m.u, m.v))

        bearing, dist = m.pos["balloon"]
        ang = math.radians(bearing)
        col = UNKNOWN if unsure else (WARN if m.grade == "below-floor" else INK)
        balloon(p, cx, cy, cx + math.cos(ang) * dist, cy - math.sin(ang) * dist,
                m.marker_id, col)

    # ---- do-not-place ------------------------------------------------------
    for link_name, fu, fv, bearing, dist, cols, note in DO_NOT_PLACE:
        lk = by[link_name]
        q = v(local_to_world(lk, lk.length * fu, lk.across * 0.5 * fv))
        p.raw(x_mark(q[0], q[1], 1.6))
        ang = math.radians(bearing)
        lx, ly = q[0] + math.cos(ang) * dist, q[1] - math.sin(ang) * dist
        p.raw(f'<line x1="{t(q[0])}" y1="{t(q[1])}" x2="{t(lx)}" y2="{t(ly)}" '
              f'stroke="{WARN}" stroke-width="0.28"/>')
        anchor = "start" if math.cos(ang) >= 0 else "end"
        p.block(lx + (1.2 if anchor == "start" else -1.2), ly, note,
                1.95, width=cols, lh=2.5, anchor=anchor, fill=WARN)

    # ---- cameras (standoff broken - NOT to scale) --------------------------
    for cam, standoff, target_link, tu, bearing in (
            ("CAM-A", "~550 mm", "GROUND", 0.30, 205),
            ("CAM-B", "~240 mm", "WRIST", 0.50, 22)):
        lk = by[target_link]
        tgt = v(local_to_world(lk, lk.length * tu, 0))
        ang = math.radians(bearing)
        px, py = tgt[0] + math.cos(ang) * 40, tgt[1] - math.sin(ang) * 40
        p.raw(f'<line x1="{t(tgt[0])}" y1="{t(tgt[1])}" x2="{t(px)}" y2="{t(py)}" '
              f'stroke="{THIN}" stroke-width="0.3" stroke-dasharray="2.4 1.4"/>')
        mx, my = (tgt[0] + px) / 2, (tgt[1] + py) / 2
        n = (-math.sin(ang), -math.cos(ang))
        p.raw(f'<path d="M {t(mx-n[0]*2.4-math.cos(ang)*1.8)} '
              f'{t(my-n[1]*2.4+math.sin(ang)*1.8)} L {t(mx+n[0]*2.4)} {t(my+n[1]*2.4)} '
              f'L {t(mx-n[0]*2.4+math.cos(ang)*1.8)} {t(my-n[1]*2.4-math.sin(ang)*1.8)}" '
              f'stroke="{INK}" stroke-width="0.5" fill="none"/>')
        p.raw(f'<rect x="{t(px-5.5)}" y="{t(py-3.6)}" width="11" height="7.2" rx="1" '
              f'fill="{PAPER}" stroke="{INK}" stroke-width="0.45"/>')
        p.raw(f'<path d="M {t(px+5.5)} {t(py-2.3)} l 3.2 -1.8 l 0 8.2 l -3.2 -1.8 z" '
              f'fill="{PAPER}" stroke="{INK}" stroke-width="0.45"/>')
        p.text(px, py + 0.8, cam, 2.4, anchor="middle", weight="bold")
        p.text(px, py - 5.2, f"{standoff} - NOT TO SCALE", 2.0, anchor="middle",
               weight="bold")

    # Name the violet balloons by id rather than hard-coding "2, 3 and 10" - the
    # ids come from markers.csv and a hand-typed list would rot the moment it changes.
    ns = ", ".join(str(m.marker_id) for m in sorted(markers, key=lambda k: k.marker_id)
                   if m.centre is None)
    xp = ", ".join(str(m.marker_id) for m in sorted(markers, key=lambda k: k.marker_id)
                   if m.centre is not None and m.pos.get("cross_part"))
    p.text(x + 2.5, y + h - 11.6,
           "Every dashed violet link is a joint whose offset is UNKNOWN - no assembly file "
           "exists here. Do not scale those gaps; the pose is for legibility only.",
           2.05, fill=UNKNOWN)
    p.text(x + 2.5, y + h - 8.6,
           f"Violet balloon + dashed footprint: {ns} are NOT SURVEYED (schematic). "
           f"{xp} is surveyed in its OWN part's frame - which way round it mates with the "
           "part drawn is not established, so it may belong at the opposite end.",
           2.0, fill=UNKNOWN)
    p.text(x + 2.5, y + h - 5.4,
           "Each panel is its PART'S OWN coordinate rectangle: origin at the part bbox "
           "minimum, +u toward increasing part coordinate.", 1.95, weight="bold")
    p.text(x + 2.5, y + h - 2.6,
           "It is NOT a view from a defined direction, so which physical end is u=0 is NOT "
           "readable here - use the SURVEY CENTRE column in the schedule.", 1.95,
           weight="bold")
    return p


# --------------------------------------------------------------------------
# VIEW B - plan view: the J0 coverage argument
# --------------------------------------------------------------------------

def draw_view_b(x, y, w, h, by) -> Panel:
    p = Panel("VIEW B", x, y, w, h, "VIEW B - PLAN, BASE + TURRET",
              "why TURRET needs two markers")
    cx, cy = x + w * 0.44, y + 48.0
    s = 0.33

    bw, bd = by["GROUND"].length, 120.0        # Alt_Kasa x and y extents
    p.raw(f'<rect x="{t(cx-bw*s/2)}" y="{t(cy-bd*s/2)}" width="{t(bw*s)}" '
          f'height="{t(bd*s)}" fill="#f5f5f5" stroke="{INK}" stroke-width="0.45"/>')
    p.text(cx + bw * s / 2 + 3.0, cy - bd * s / 2 + 3.0, "Alt_Kasa.stl",
           2.0, fill=THIN, style="italic")
    p.text(cx + bw * s / 2 + 3.0, cy - bd * s / 2 + 6.0, "210 x 120 mm",
           1.95, fill=THIN)

    r = 60.0 * s
    p.circle(cx, cy, r, fill=PAPER, stroke=INK, stroke_width="0.45")
    p.text(cx, cy + 6.0, "Alt_Govde", 2.1, anchor="middle", fill=THIN, style="italic")
    p.text(cx, cy + 8.8, "dia 120", 1.95, anchor="middle", fill=THIN)

    def ray(deg, rad):
        a = math.radians(deg)
        return cx + math.cos(a) * rad, cy - math.sin(a) * rad

    # J0 travel 29..110 deg, home 64 - Software/arm-console/joint-limits.csv
    ra = r + 7
    a0, a1 = ray(29, ra), ray(110, ra)
    p.raw(f'<path d="M {t(a0[0])} {t(a0[1])} A {t(ra)} {t(ra)} 0 0 0 {t(a1[0])} '
          f'{t(a1[1])}" fill="none" stroke="{INK}" stroke-width="0.7"/>')
    for deg, lab, wt in ((29, "29", "normal"), (110, "110", "normal"),
                         (64, "home 64", "bold")):
        q1, q2 = ray(deg, r), ray(deg, ra + 2.2)
        p.raw(f'<line x1="{t(q1[0])}" y1="{t(q1[1])}" x2="{t(q2[0])}" y2="{t(q2[1])}" '
              f'stroke="{INK}" stroke-width="0.3"/>')
        q3 = ray(deg, ra + 5.0)
        p.text(q3[0], q3[1], lab, 2.0, anchor="middle", weight=wt)
    p.text(cx, y + 11.8, "J0 base yaw - 81 deg of travel (29-110)", 2.4,
           anchor="middle", weight="bold")

    # TURRET-1 / TURRET-2 at 90 deg of azimuth separation
    for deg, lab, mid in ((64, "TURRET-1", 2), (154, "TURRET-2", 3)):
        half = math.degrees(math.atan2(26.7 / 2.0, 60.0))
        pa, pb = ray(deg - half, r), ray(deg + half, r)
        p.raw(f'<path d="M {t(pa[0])} {t(pa[1])} A {t(r)} {t(r)} 0 0 0 {t(pb[0])} '
              f'{t(pb[1])}" fill="none" stroke="#8e8e8e" stroke-width="2.2"/>')
        lp = ray(deg, r - 8)
        balloon(p, lp[0], lp[1], *ray(deg, r - 8), mid)
        lp2 = ray(deg, r - 15.0)
        p.text(lp2[0], lp2[1], lab, 1.95, anchor="middle", weight="bold")
    arc = ray(160, r + 17)
    p.text(arc[0], arc[1] - 1.5, "90 deg apart", 1.95, anchor="end", fill=THIN)

    # BASE-1/2 drawn on the y=-59.8 wall. markers.csv notes the two side walls
    # are equally good (both fill 1.00, 63.2 mm square), and this one keeps the
    # ground datum clear of the J0 travel arc.
    wy = cy + bd * s / 2
    b1, b2 = cx + (-27.1) * s, cx + (-27.1 - 83.2) * s
    for xp, mid in ((b1, 0), (b2, 1)):
        p.raw(f'<rect x="{t(xp-2.0)}" y="{t(wy-1.3)}" width="4" height="2.6" '
              f'fill="{INK}"/>')
        balloon(p, xp, wy, xp, wy + 6.0, mid)
    dy = wy - 4.5
    p.raw(f'<line x1="{t(b2)}" y1="{t(dy)}" x2="{t(b1)}" y2="{t(dy)}" stroke="{INK}" '
          f'stroke-width="0.3" marker-start="url(#dim)" marker-end="url(#dim)"/>')
    p.text((b1 + b2) / 2, dy - 1.2, "83.2 mm", 1.95, anchor="middle")
    p.text(cx + bw * s / 2 + 3.0, wy + 1.0,
           "either side wall serves -", 1.9, fill=THIN)
    p.text(cx + bw * s / 2 + 3.0, wy + 3.8, "both are fill 1.00", 1.9, fill=THIN)

    cam = ray(48, r + 24)
    p.raw(f'<path d="M {t(cam[0])} {t(cam[1])} L {t(cx)} {t(cy)}" stroke="{THIN}" '
          f'stroke-width="0.3" stroke-dasharray="2 1.4"/>')
    p.text(cam[0] + 3.0, cam[1], "CAM-A", 2.2, weight="bold")

    ny = y + h - 22.0
    p.text(x + 2.5, ny, "CURVED SURFACE - centroid and azimuth only, NOT single-tag pose.",
           2.15, weight="bold", fill=WARN)
    p.block(x + 2.5, ny + 3.4,
            "Alt_Govde has no flat outward face: the top is open and the z=0 face is a "
            "recessed ledge. A 20 mm sticker wrapped on r=60 spans ~19 deg of arc, ~0.8 mm "
            "of bow. The pair is 90 deg apart for COVERAGE, not for a baseline - across "
            "81 deg of yaw a single wrapped marker rotates out of view; this way one of "
            "the two always faces CAM-A.", 1.95, width=76, lh=2.6, fill=THIN)
    return p


# --------------------------------------------------------------------------
# LEGEND
# --------------------------------------------------------------------------

def draw_legend(x, y, w, h, markers) -> Panel:
    p = Panel("LEGEND", x, y, w, h, "LEGEND",
              "grade belongs to the marker AND its camera")
    cy = y + 12.5
    for grade, st in GRADE_STYLE.items():
        dash = f' stroke-dasharray="{st["dash"]}"' if st["dash"] else ""
        p.raw(f'<rect x="{t(x+3)}" y="{t(cy-3.2)}" width="6" height="6" '
              f'fill="{st["fill"]}" stroke="{st["stroke"]}" stroke-width="0.45"{dash}/>')
        n = sum(1 for m in markers if m.grade == grade)
        p.text(x + 12, cy - 0.4, st["label"], 2.35, weight="bold",
               fill=WARN if grade == "below-floor" else INK)
        p.text(x + 12, cy + 2.5, f"{n} of {len(markers)} markers at design standoff",
               1.95, fill=THIN)
        cy += 8.6

    p.raw(f'<line x1="{t(x+3)}" y1="{t(cy-2.4)}" x2="{t(x+w-3)}" y2="{t(cy-2.4)}" '
          f'stroke="{GHOST}" stroke-width="0.3"/>')
    p.raw(f'<line x1="{t(x+3)}" y1="{t(cy+1.5)}" x2="{t(x+9)}" y2="{t(cy+1.5)}" '
          f'stroke="{UNKNOWN}" stroke-width="1.1" stroke-dasharray="1.6 1.2"/>')
    p.text(x + 12, cy + 2.4, "joint - OFFSET UNKNOWN, not to scale", 2.25,
           weight="bold", fill=UNKNOWN)
    cy += 7.0
    p.raw(x_mark(x + 6, cy + 1.5, 1.6))
    p.text(x + 12, cy + 2.4, "DO NOT PLACE a marker here", 2.25, weight="bold", fill=WARN)
    p.text(x + 12, cy + 5.3, "the FACE is surveyed; where the X is struck is not",
           1.9, fill=THIN)
    cy += 9.4
    p.circle(x + 6, cy + 1.5, 2.9, fill=PAPER, stroke=INK, stroke_width="0.5")
    p.text(x + 6, cy + 2.5, "7", 2.7, anchor="middle", weight="bold")
    p.text(x + 12, cy + 2.4, "marker id - see MARKER SCHEDULE", 2.25, weight="bold")
    cy += 8.0

    p.block(x + 3, cy,
            "Marker footprints are drawn at the SAME scale as the links. The 36 mm BASE "
            "sticker beside the 6 mm FINGER sticker is the finding, not a drawing "
            "artefact.", 2.0, width=42, lh=2.75)

    # Right column - the headline result gets its own space rather than being
    # squeezed under the swatches, where it used to run off the panel.
    rx = x + 78
    p.raw(f'<line x1="{t(rx-4)}" y1="{t(y+9.5)}" x2="{t(rx-4)}" y2="{t(y+h-3)}" '
          f'stroke="{GHOST}" stroke-width="0.3"/>')
    ry = y + 13.5
    p.text(rx, ry, "THE NEGATIVE RESULT", 2.6, weight="bold", fill=WARN)
    ry += 4.0
    ry = p.block(rx, ry,
                 "One 1280x720 camera at the ~550 mm standoff needed to frame the whole "
                 "arm resolves only BASE-1/2 at pose grade and TURRET-1/2 at detect "
                 "grade. SHOULDER, ELBOW, WRIST, HAND and FINGER are BELOW the detection "
                 "floor there - not imprecise, NOT SEEN.", 2.0, width=48, lh=2.75)
    ry += 1.4
    p.text(rx, ry, "Two honest ways past it:", 2.05, weight="bold")
    ry += 3.2
    ry = p.block(rx, ry,
                 "(a) a second close camera, CAM-B at ~240 mm. One webcam and one config "
                 "entry; nothing mechanical changes. The gripper stays marginal at 6 mm.",
                 2.0, width=48, lh=2.75)
    ry += 0.8
    p.block(rx, ry,
            "(b) 40 x 40 x 2 mm printed flat tabs on Alt_Kol, On_Kol and Bilek - carries "
            "a 28 mm square, pose-grade for the whole chain from CAM-A alone. A 30 mm tab "
            "is NOT enough (29.6 px - detect only). New geometry, so a proposal.",
            2.0, width=48, lh=2.75)
    return p


# --------------------------------------------------------------------------
# MARKER SCHEDULE
# --------------------------------------------------------------------------

# (header, width mm, align). Widths were re-cut to make room for SURVEY CENTRE
# without losing a column - the provenance of a position belongs on the sheet,
# not only in the generator. draw_schedule gates every cell against its width, so
# a future edit cannot silently overlap two columns.
COLS = [("ID", 6, "middle"), ("LABEL", 18, "start"), ("STL PART", 20, "start"),
        ("SURVEY CENTRE part mm", 27, "start"),
        ("BLACK", 10, "end"), ("STICKER", 10, "end"), ("CAM", 9, "start"),
        ("STANDOFF", 12, "end"), ("GRADE", 17, "start"),
        ("PAIR BASELINE", 22, "start"), ("ORIENTATION - which way is UP", 45, "start")]

CELL_PAD_MM = 1.5           # breathing room a cell must leave inside its column


def draw_schedule(x, y, w, h, markers) -> Panel:
    p = Panel("SCHEDULE", x, y, w, h, "MARKER SCHEDULE",
              "positions from Software/vision/stl-face-survey.csv; sizes from markers.csv")
    cy = y + 12.0
    cxs, acc = [], x + 3.0
    for _, cw, _ in COLS:
        cxs.append(acc)
        acc += cw

    def cell(cxp, cw, al, s, size, **kw) -> None:
        """Place one cell AND gate it against its column width. The panel gate
        only catches text leaving the sheet; two columns quietly overprinting
        each other would still look like a finished table."""
        if len(s) * size * GLYPH_W > cw - CELL_PAD_MM:
            p.bad.append(f"SCHEDULE: {s!r} is {len(s) * size * GLYPH_W:.1f} mm wide, "
                         f"column allows {cw - CELL_PAD_MM:.1f} mm")
        ax = cxp if al == "start" else (cxp + cw - CELL_PAD_MM if al == "end"
                                        else cxp + cw / 2)
        p.text(ax, cy, s, size, anchor=al, **kw)

    for (name, cw, al), cxp in zip(COLS, cxs):
        cell(cxp, cw, al, name, 2.05, weight="bold")
    p.raw(f'<line x1="{t(x+3)}" y1="{t(cy+1.3)}" x2="{t(acc)}" y2="{t(cy+1.3)}" '
          f'stroke="{INK}" stroke-width="0.35"/>')
    cy += 4.6

    for m in sorted(markers, key=lambda k: k.marker_id):
        col = WARN if m.grade == "below-floor" else INK
        base = m.baseline if m.baseline not in ("", "n/a") else "-"
        if base == "unknown-cross-part":
            base = "UNKNOWN cross-part"
        if m.centre is None:
            centre = "NOT SURVEYED"
        else:
            # "-0.0" is a faithful echo of the CSV's "-0.00" but reads as a bug.
            trio = ",".join(f"{c:.1f}".replace("-0.0", "0.0") for c in m.centre)
            centre = f"{m.src} {trio}"
        vals = [str(m.marker_id), m.label, m.part, centre, f"{m.black_mm:.0f} mm",
                f"{m.sticker_mm:.0f} mm", m.camera, f"{m.standoff} mm",
                m.grade.upper(), base, m.orientation]
        for (name, cw, al), cxp, val in zip(COLS, cxs, vals):
            fill = col if name in ("ID", "LABEL", "GRADE") else (
                UNKNOWN if val.startswith(("UNKNOWN", "NOT SURVEYED")) else THIN)
            cell(cxp, cw, al, val, 1.95, fill=fill,
                 weight="bold" if name in ("ID", "LABEL") else "normal")
        cy += 3.15
    p.raw(f'<line x1="{t(x+3)}" y1="{t(cy-2.2)}" x2="{t(acc)}" y2="{t(cy-2.2)}" '
          f'stroke="{GHOST}" stroke-width="0.3"/>')
    cy += 0.8
    p.text(x + 3, cy,
           "SURVEY CENTRE is where the sticker goes: sq1 = square1_cx/cy/cz_mm, sq2 = "
           "square2_cx/cy/cz_mm on the stl-face-survey.csv row for that part and face "
           "normal.", 1.95, weight="bold")
    cy += 2.9
    p.text(x + 3, cy,
           "Part-frame millimetres - unambiguous without a view direction. The drawn spot "
           "is computed from it (u from the part bbox minimum, v from the bbox centre).",
           1.95, fill=THIN)
    cy += 2.9
    p.text(x + 3, cy,
           "NOT SURVEYED = TURRET-1/2 only: Alt_Govde has no planar patch (markers.csv "
           "patch_normal cylindrical; square and baseline n/a). All others are derived.",
           1.95, fill=UNKNOWN, weight="bold")
    cy += 2.9
    p.text(x + 3, cy,
           "BLACK is the ArUco square edge - the value you pass to solvePnP. "
           "STICKER is the white square you cut out (BLACK x 8/6, one module of quiet zone).",
           1.95, fill=THIN)
    cy += 2.9
    p.text(x + 3, cy,
           "GRADE assumes f_px ~= 1108 (HFOV 60 deg at 1280 px) - UNVERIFIED. "
           "Every grade moves if that changes; run a ChArUco board to settle it.",
           1.95, fill=WARN)
    cy += 2.9
    p.text(x + 3, cy,
           "PAIR BASELINE is only real within ONE part. Cross-part spacings have no "
           "assembly file behind them and must be observed in calibration Pass 1.",
           1.95, fill=THIN)
    cy += 2.9
    p.text(x + 3, cy,
           "ORIENTATION is condensed from preferred_orientation in markers.csv (full "
           "text there). It defines which way is UP - it is what the arrow on every "
           "sheet-1 sticker label means.",
           1.95, weight="bold")
    cy += 2.9
    p.text(x + 3, cy,
           "Apply each sticker that way up EVERY time. A re-applied sticker at a "
           "different rotation silently flips the sign of that joint's residual.",
           1.95, fill=THIN)
    return p


# --------------------------------------------------------------------------
# NOTES
# --------------------------------------------------------------------------

def draw_notes(x, y, w, h) -> Panel:
    p = Panel("NOTES", x, y, w, h, "NOTES", "MARKER-SYSTEM.md is the full argument")
    colw = (w - 10) / 2.0
    cols = [
        ("WHAT IS NOT KNOWN", INK, [
            "Joint-to-joint transforms. The 21 STLs are individual parts in PRINT "
            "orientation; there is no assembly file, URDF or STEP anywhere in the repo, "
            "so no joint axis can be read off this geometry.",
            "Which link belongs to which joint is INFERRED from the Turkish part names "
            "and the measured sizes.",
            "D3 is DISPUTED: calibration-log.csv 2026-08-01 says the servo on D3 was the "
            "GRIPPER, not the Base. This rig settles it - command J0 alone and watch "
            "which link moves.",
            "Until the two calibration passes run (MARKER-SYSTEM.md 6: axis "
            "identification, then datum capture at commanded home) a marker yields a "
            "CHANGE in orientation, NOT a joint angle.",
            "VOCABULARY: a value a camera actually saw is OBSERVED. Nothing here observes "
            "a servo output shaft - it observes the printed part the shaft is bolted to.",
        ]),
        ("NEVER PLACE A MARKER ON", WARN, list(EXCLUDED_PARTS)),
    ]
    for i, (head, col, items) in enumerate(cols):
        cxp = x + 4 + i * (colw + 6)
        cy = y + 12.5
        p.text(cxp, cy, head, 2.6, weight="bold", fill=col)
        cy += 4.0
        for it in items:
            for k, ln in enumerate(wrap(it, 56)):
                p.text(cxp + (0 if k == 0 else 2.2), cy,
                       ("- " if k == 0 else "") + ln, 1.98,
                       fill=INK if i == 1 else THIN)
                cy += 2.65
            cy += 0.8
    return p


# --------------------------------------------------------------------------

def build_svg(links, by, markers, csv_rel) -> tuple[str, list[str]]:
    panels: list[Panel] = []
    head_p = Panel("TITLE", 6, 6, PAGE_W - 12, 17)
    head_p.raw(f'<rect x="6" y="6" width="{t(PAGE_W-12)}" height="17" fill="none" '
               f'stroke="{INK}" stroke-width="0.5"/>')
    head_p.text(9, 15.2, "EMRE KALEM 6-AXIS ARM - FIDUCIAL MARKER PLACEMENT", 5.2,
                weight="bold")
    head_p.text(9, 20.4,
                f"ArUco DICT_4X4_50 - {len(markers)} markers - generated by "
                f"Software/arm-vision/make_placement_diagram.py from {csv_rel} "
                "and the STLs in Backups/STL_parts/", 2.15, fill=THIN)
    head_p.text(PAGE_W - 9, 12.6, "SHEET 2 OF 2", 2.6, anchor="end", weight="bold")
    head_p.text(PAGE_W - 9, 16.6, "sheet 1: docs/marker_print_sheet.svg", 2.1,
                anchor="end", fill=THIN)
    head_p.text(PAGE_W - 9, 20.6, "NOT AN ASSEMBLY DRAWING - see NOTES", 2.1,
                anchor="end", fill=WARN, weight="bold")
    panels.append(head_p)

    panels.append(draw_view_a(6, 26, 250, 176, links, by, markers))
    panels.append(draw_view_b(260, 26, 154, 100, by))
    panels.append(draw_legend(260, 130, 154, 72, markers))
    panels.append(draw_schedule(6, 206, 202, 85, markers))
    panels.append(draw_notes(212, 206, 202, 85))

    body = [f'<rect x="0" y="0" width="{t(PAGE_W)}" height="{t(PAGE_H)}" fill="{PAPER}"/>']
    problems: list[str] = []
    for pn in panels:
        body += pn.out
        problems += pn.bad

    defs = ('<defs><marker id="dim" viewBox="0 0 10 10" refX="5" refY="5" '
            'markerWidth="4" markerHeight="4" orient="auto-start-reverse">'
            f'<path d="M 1 1 L 9 5 L 1 9 z" fill="{INK}"/></marker></defs>')
    head = (f'<svg xmlns="http://www.w3.org/2000/svg" version="1.1" '
            f'width="{t(PAGE_W)}mm" height="{t(PAGE_H)}mm" '
            f'viewBox="0 0 {t(PAGE_W)} {t(PAGE_H)}">\n'
            f'<title>Emre Kalem arm - fiducial marker placement diagram '
            f'(A3 landscape)</title>\n{defs}\n')
    return head + "\n".join(body) + "\n</svg>\n", problems


def main(argv=None) -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.abspath(os.path.join(here, "..", ".."))
    ap = argparse.ArgumentParser(description="Emre arm marker placement diagram")
    ap.add_argument("--csv", default=os.path.join(repo, "Software", "vision", "markers.csv"))
    ap.add_argument("--survey", default=os.path.join(repo, "Software", "vision",
                                                     "stl-face-survey.csv"))
    ap.add_argument("--stl-dir", default=os.path.join(repo, "Backups", "STL_parts"))
    ap.add_argument("--out", default=os.path.join(repo, "docs",
                                                  "marker_placement_diagram.svg"))
    args = ap.parse_args(argv)

    markers = load_markers(args.csv)
    links, by = build_chain(args.stl_dir)
    unknown = sorted({m.link for m in markers if m.link not in by})
    if unknown:
        raise SystemExit(
            f"ERROR: markers.csv references link(s) with no LINKS entry: {unknown}. "
            "Add the link (with its part file and the reason for the drawn axes) rather "
            "than dropping the marker from the drawing.")

    survey = load_survey(args.survey)
    # A surveyed marker on its own link's part must use that link's declared face
    # normal, or the panel caption would advertise a cell the schedule does not use.
    for m in markers:
        c = m.pos["cell"]
        lk = by[m.link]
        if c is not None and c.part == lk.part and lk.face is not None:
            if fmt_normal(c.normal) != fmt_normal(lk.face):
                raise SystemExit(
                    f"ERROR: {m.label} addresses face {fmt_normal(c.normal)} on {c.part} but "
                    f"link {lk.name} is captioned {fmt_normal(lk.face)}. The drawing would "
                    "name one survey row and use another.")
    audit = derive_positions(markers, by, survey, args.stl_dir)
    pairs = check_baselines(markers)

    print(f"markers.csv : {args.csv}   ({len(markers)} markers)")
    print(f"survey      : {args.survey}   ({len(survey)} rows)")
    print("link outlines measured from the STLs at run time:")
    for lk in links:
        print(f"  {lk.name:<10} {lk.part:<20} drawn {lk.length:7.1f} x {lk.across:6.1f} mm"
              f"   ({lk.why})")
    print("sticker positions derived from stl-face-survey.csv "
          "(row gate + cell gate passed):")
    for ln in audit:
        print(ln)
    print("pair gate - derived spacing vs markers.csv pair_baseline_mm:")
    for ln in pairs:
        print(ln)

    svg, problems = build_svg(links, by, markers,
                              os.path.relpath(args.csv, repo).replace("\\", "/"))
    if problems:
        print("\nLAYOUT GATE FAILED - content escapes its panel:")
        for q in problems:
            print("  " + q)
        raise SystemExit(f"ERROR: {len(problems)} layout overflow(s); refusing to write "
                         "a diagram whose text runs off its own panel.")
    print("layout gate : PASS (nothing escapes its panel)")

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(svg)

    grades: dict[str, int] = {}
    for m in markers:
        grades[m.grade] = grades.get(m.grade, 0) + 1
    print(f"wrote       : {args.out}")
    print(f"bytes       : {os.path.getsize(args.out)}")
    print("grades      : " + "  ".join(f"{k}={v}" for k, v in sorted(grades.items())))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
