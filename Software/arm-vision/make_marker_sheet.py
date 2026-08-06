#!/usr/bin/env python3
"""
make_marker_sheet.py - generate docs/marker_print_sheet.svg, the printable A4
sheet carrying every fiducial sticker in the design.

WHY THIS EXISTS
    Software/vision/markers.csv specifies thirteen stickers and, for each one, a
    PHYSICAL size in millimetres that was derived from measured STL geometry.
    Those millimetres are the whole point: pass the wrong black-square edge to
    solvePnP and every pose is wrong by that ratio. A hand-made sheet, or one
    printed "to fit", silently destroys the derivation. This script emits the
    sheet at exact mm and prints the self-check that proves the printer did not
    rescale it.

WHAT IT EMITS AND WHY IT IS VECTOR, NOT A BITMAP
    The ArUco bitmaps ARE generated with cv2.aruco.generateImageMarker, exactly
    as the design says. They are then decomposed into their 6x6 module grid and
    re-emitted as SVG rectangles rather than embedded as a raster.

    That is deliberate. An <image> data URI gets smooth-scaled by most print
    pipelines unless image-rendering:crisp-edges is honoured, which is
    inconsistent across Chrome-headless-to-PDF and Inkscape. A blurred marker
    edge degrades precisely the corner refinement this whole design exists to
    protect. Vector modules cannot blur at any zoom or DPI.

    The decomposition is not trusted on faith. Two gates run on every marker:
      GATE 1  every NxN pixel block of the generated bitmap must be uniformly
              black or uniformly white. A misaligned grid fails here loudly
              instead of succeeding quietly, which is what sampling a module
              CENTRE pixel would do.
      GATE 2  the emitted module grid is expanded back to an image, padded with
              a white quiet zone, and fed to cv2.aruco.ArucoDetector. Exactly
              one marker must be found and its id must match. This catches a
              vertical flip (SVG y-down vs numpy row 0), which is the other way
              to emit a wrong-but-plausible marker.
    A third, stricter check compares the expansion against the original bitmap
    pixel for pixel.

WHAT THIS SHEET DOES NOT CLAIM
    Nothing here is an observation of the arm. These are blanks. A sticker only
    becomes an observation after the two calibration passes in
    Documentation/MARKER-SYSTEM.md section 6 (axis identification, then datum
    capture at commanded home). Detection grades are deliberately NOT printed on
    this sheet: a grade is meaningless without the camera standoff and the
    unverified focal-length assumption it was computed from, and a number that
    can be wrong while looking authoritative does not belong on a shop-floor
    printout. Grades live on docs/marker_placement_diagram.svg with their
    assumptions attached.

USAGE
    python Software/arm-vision/make_marker_sheet.py
    python Software/arm-vision/make_marker_sheet.py --out somewhere/else.svg
    python Software/arm-vision/make_marker_sheet.py --csv path/to/markers.csv

DEPENDENCIES
    opencv-python  Apache-2.0. Generates the marker bitmaps and re-detects them.
    numpy          BSD-3-Clause. FLAGGED, not silently accepted: the repository
                   rule is Apache-2.0 or MIT only. numpy is already an
                   unavoidable dependency of the vision and LeRobot work (cv2
                   requires it), so it is raised here for a decision rather than
                   passed over.
    stdlib         argparse, csv, math, os, sys, xml.etree, xml.sax.saxutils.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from dataclasses import dataclass
from xml.sax.saxutils import escape as xml_escape

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover - environment problem, not a code path
    sys.stderr.write(
        "ERROR: opencv-python is required to generate the marker bitmaps.\n"
        "       pip install opencv-python\n"
    )
    raise


# --------------------------------------------------------------------------
# Constants that encode the design. Change these ONLY together with
# Documentation/MARKER-SYSTEM.md - they are not free parameters.
# --------------------------------------------------------------------------

DICT_NAME = "DICT_4X4_50"
MARKER_MODULES = 6          # 4 data modules + 1 black border ring each side
QUIET_MODULES = 1           # quiet zone, per MARKER-SYSTEM.md section 3
STICKER_RATIO = (MARKER_MODULES + 2 * QUIET_MODULES) / MARKER_MODULES  # 8/6

PAGE_W_MM = 210.0           # A4 portrait
PAGE_H_MM = 297.0
MARGIN_MM = 10.0

# TWO boundaries per card, and they do different jobs. One line could not do
# both, which is the defect this pair fixes.
#
#   OUTER CUT LINE   what the scissors follow on the sheet. Held CUT_RING_MM off
#                    the sticker square so there is a real margin to cut inside
#                    of, and so the hairline is never ink at the quiet-zone
#                    boundary - MARKER-SYSTEM.md section 5 forbids that.
#
#   INNER TRIM LIMIT the line you must NOT cut inside. NINE of the thirteen
#                    stickers are printed on cards larger than the surveyed
#                    island they have to be applied to (a 28 mm card for a
#                    10.4 mm finger island), so those MUST be trimmed down after
#                    the first cut. Before this line existed there was nothing
#                    printed to trim to: you cut by eye, and by eye you cut into
#                    the one-module quiet zone that ArUco detection requires.
#                    It is drawn TRIM_LINE_MM OUTSIDE the sticker square, never
#                    on it, so the line itself is not ink in the quiet zone.
#
#                    NINE, not five. An earlier revision of this comment said
#                    five; the generator's own gate flags nine, and the footer it
#                    writes onto the sheet says nine. The gate is the authority
#                    here -- it counts, this comment does not. Do not re-hardcode
#                    a number without running it.
CUT_RING_MM = 3.2
# 0.8 mm, and the ceiling is 1.2. The binding case is the 6 mm marker on the
# 10.4 mm finger island: the trim square is sticker + 2*TRIM_LINE_MM, so
# 8.0 + 2T <= 10.4 gives T <= 1.2. 0.8 leaves 0.4 mm of white per side.
# A previous note justified 0.8 as "the largest value that still fits every
# surveyed island", citing the gate's 0.80 mm -- but that figure was a difference
# of two EDGE LENGTHS, i.e. twice the per-side room. The value 0.8 is fine and is
# unchanged; the REASONING was wrong, and the gate below now reports per-side.
TRIM_LINE_MM = 0.8
SCISSOR_MARGIN_MM = CUT_RING_MM - TRIM_LINE_MM      # 2.4 mm of cuttable white,
                                                    # per side -- both terms are
                                                    # per-side offsets already
LABEL_STRIP_MM = 5.0        # inside the cut line, so the label travels with the
                            # sticker and it can be re-applied the same way up.
                            # NOTE: a sticker trimmed to its island loses this -
                            # the footer says so, with the numbers.
MIN_CARD_W_MM = 28.0        # so a 6 mm finger sticker still carries a legible
                            # "FINGER-A id11 6mm" without the label overrunning
                            # the card it is printed on
CARD_GAP_MM = 5.0           # scissors path between two adjacent cut lines, so
                            # each is cuttable with +/- 2.5 mm of wander
LABEL_FS_MAX = 2.6
LABEL_FS_MIN = 1.5
GLYPH_W = 0.55              # width of one Helvetica character as a fraction of
                            # font size. Used ONLY to shrink a sticker label to
                            # fit its card: there it must be an OVER-estimate
                            # (too small a label is ugly, too large is trimmed
                            # off by the scissors), and it is ~25% high on
                            # lowercase prose, which is the safe direction.

# Helvetica advance widths, units per 1000 em (the standard AFM table). Used for
# the footer page gate, where GLYPH_W's 25% over-estimate would reject lines that
# actually fit. Checked against the rightmost ink observed on the rendered sheet:
# within +0.5% on every regular-weight line. Helvetica-Bold runs wider - BOLD_K.
_AFM = {' ': 278, '!': 278, '"': 355, '#': 556, '$': 556, '%': 889, '&': 667,
        "'": 191, '(': 333, ')': 333, '*': 389, '+': 584, ',': 278, '-': 333,
        '.': 278, '/': 278, ':': 278, ';': 278, '<': 584, '=': 584, '>': 584,
        '?': 556, '@': 1015, '[': 278, ']': 278, '^': 469, '_': 556, '`': 333,
        '{': 334, '|': 260, '}': 334, '~': 584}
_AFM.update({c: 556 for c in "0123456789"})
_AFM.update(zip("ABCDEFGHIJKLMNOPQRSTUVWXYZ",
                (667, 667, 722, 722, 667, 611, 778, 722, 278, 500, 667, 556, 833,
                 722, 778, 667, 778, 722, 667, 611, 722, 667, 944, 667, 667, 611)))
_AFM.update(zip("abcdefghijklmnopqrstuvwxyz",
                (556, 556, 500, 556, 556, 278, 556, 556, 222, 222, 500, 222, 833,
                 556, 556, 556, 556, 333, 500, 278, 556, 500, 722, 500, 500, 500)))
BOLD_K = 1.08


def helv_mm(s: str, size: float, bold: bool = False) -> float:
    """Rendered width of a Helvetica string, in millimetres."""
    w = sum(_AFM.get(c, 556) for c in s) / 1000.0 * size
    return w * (BOLD_K if bold else 1.0)

RULE_LEN_MM = 100.0         # the printer scale check
EPS_MM = 0.01               # sub-printer-resolution overlap that closes hairline
                            # seams between abutting black rects (600 dpi = 0.042 mm)

INK = "#000000"
PAPER = "#ffffff"
GUIDE = "#9a9a9a"           # cut lines and rule ticks - grey so it is obviously
                            # not part of any marker
LIMIT = "#c0c0c0"           # the inner trim-limit line - lighter still, so it
                            # never competes with the cut line for the scissors


# --------------------------------------------------------------------------
# markers.csv
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class Sticker:
    marker_id: int
    human_label: str
    link_name: str
    black_mm: float          # edge of the ArUco black square - the solvePnP value
    sticker_mm: float        # edge of the printed white square incl. quiet zone
    island_mm: float | None  # measured_inscribed_square_mm - the surveyed patch
                             # this sticker must physically fit on. None where
                             # markers.csv records n/a (the curved turret).

    @property
    def quiet_mm(self) -> float:
        """One module of quiet zone, in millimetres. This is the width that must
        survive every cut - not a rule of thumb, it is black_mm / 6."""
        return self.black_mm / MARKER_MODULES

    @property
    def trim_mm(self) -> float:
        """Edge of the inner trim-limit square."""
        return self.sticker_mm + 2 * TRIM_LINE_MM

    @property
    def card_w(self) -> float:
        return max(self.sticker_mm + 2 * CUT_RING_MM, MIN_CARD_W_MM)

    @property
    def card_h(self) -> float:
        return self.sticker_mm + 2 * CUT_RING_MM + LABEL_STRIP_MM

    @property
    def oversize(self) -> bool:
        """True when the printed card cannot be applied as cut - it does not fit
        inside the surveyed island. These are the ones that MUST be trimmed.

        BOTH dimensions, not just the width: card_h carries the label strip and
        overtakes card_w for any sticker above ~16.6 mm, so a width-only test
        would eventually call a too-tall card 'fits' and print a wrong count.
        """
        return self.island_mm is not None and max(self.card_w,
                                                  self.card_h) > self.island_mm


def load_stickers(path: str) -> list[Sticker]:
    """Parse markers.csv. Fails loudly on a shape it does not recognise rather
    than quietly producing a sheet with a wrong size on it."""
    if not os.path.isfile(path):
        raise SystemExit(f"ERROR: markers.csv not found at {path}")

    with open(path, "r", encoding="utf-8", newline="") as fh:
        # '#' lines are the file's long design preamble, not data.
        rows = list(csv.reader(l for l in fh if not l.lstrip().startswith("#") and l.strip()))

    if not rows:
        raise SystemExit(f"ERROR: {path} contains no data rows")

    header = [c.strip() for c in rows[0]]
    need = ("marker_id", "human_label", "link_name",
            "black_square_mm", "approximate_size_mm",
            "measured_inscribed_square_mm")
    missing = [c for c in need if c not in header]
    if missing:
        raise SystemExit(f"ERROR: {path} is missing required column(s): {missing}")
    idx = {name: header.index(name) for name in need}

    out: list[Sticker] = []
    for lineno, row in enumerate(rows[1:], start=2):
        if len(row) != len(header):
            raise SystemExit(
                f"ERROR: {path} data row {lineno} has {len(row)} fields, "
                f"header has {len(header)}. Refusing to guess which column is which."
            )
        black = float(row[idx["black_square_mm"]])
        stated = float(row[idx["approximate_size_mm"]])
        derived = black * STICKER_RATIO
        # The CSV rounds approximate_size_mm to 0.1 mm. Anything worse than that
        # means the two columns disagree about the quiet zone, which would put a
        # wrong physical size on the paper.
        if abs(stated - derived) > 0.06:
            raise SystemExit(
                f"ERROR: {path} row {lineno}: approximate_size_mm={stated} but "
                f"black_square_mm={black} implies {derived:.3f} "
                f"(x{STICKER_RATIO:.4f} for the 1-module quiet zone). "
                "Fix the CSV; do not print a sheet whose sizes disagree."
            )
        try:
            island = float(row[idx["measured_inscribed_square_mm"]])
        except ValueError:
            island = None       # markers.csv records n/a for the curved turret
        out.append(Sticker(
            marker_id=int(row[idx["marker_id"]]),
            human_label=row[idx["human_label"]].strip(),
            link_name=row[idx["link_name"]].strip(),
            black_mm=black,
            # Use the DERIVED value, not the rounded CSV value: this is the one
            # number the printed geometry must honour exactly.
            sticker_mm=derived,
            island_mm=island,
        ))

    ids = [s.marker_id for s in out]
    if len(set(ids)) != len(ids):
        raise SystemExit(f"ERROR: duplicate marker_id in {path}: {ids}")
    return out


# --------------------------------------------------------------------------
# ArUco -> module grid, with both gates
# --------------------------------------------------------------------------

def marker_grid(dictionary, marker_id: int, block_px: int = 8) -> np.ndarray:
    """Return a (6, 6) bool array, True where the module is BLACK.

    GATE 1 lives here: every block must be uniform. Sampling a centre pixel
    would succeed on a misaligned grid; this cannot.
    """
    side = MARKER_MODULES * block_px
    img = cv2.aruco.generateImageMarker(dictionary, marker_id, side)
    if img.shape != (side, side):
        raise SystemExit(
            f"ERROR: generateImageMarker returned {img.shape}, expected {(side, side)}"
        )

    blocks = img.reshape(MARKER_MODULES, block_px, MARKER_MODULES, block_px)
    lo = blocks.min(axis=(1, 3))
    hi = blocks.max(axis=(1, 3))
    if not np.array_equal(lo, hi):
        bad = np.argwhere(lo != hi)
        raise SystemExit(
            f"ERROR: marker {marker_id}: module grid is not aligned to the bitmap "
            f"({len(bad)} non-uniform block(s), first at row/col {tuple(bad[0])}). "
            "Refusing to emit a marker sampled from a misaligned grid."
        )
    if not np.isin(lo, (0, 255)).all():
        raise SystemExit(f"ERROR: marker {marker_id}: bitmap is not pure black/white")

    grid = lo == 0

    # Strict check: the grid, expanded back, must equal the bitmap exactly.
    # This catches any transpose or flip introduced by the reshape above.
    expanded = np.where(np.kron(grid, np.ones((block_px, block_px), bool)), 0, 255)
    if not np.array_equal(expanded.astype(np.uint8), img):
        raise SystemExit(
            f"ERROR: marker {marker_id}: module grid does not reproduce the bitmap"
        )
    return grid


def verify_detectable(dictionary, detector, grid: np.ndarray, marker_id: int) -> None:
    """GATE 2: render the grid we are about to emit and detect it back.

    The quiet zone must be present in the test image or detection fails for the
    wrong reason. Two modules of padding is used - more than the printed one -
    so a detection failure here means the MARKER is wrong, never the margin.
    """
    px = 50
    body = np.where(np.kron(grid, np.ones((px, px), bool)), 0, 255).astype(np.uint8)
    pad = 2 * px
    canvas = np.full((body.shape[0] + 2 * pad, body.shape[1] + 2 * pad), 255, np.uint8)
    canvas[pad:pad + body.shape[0], pad:pad + body.shape[1]] = body

    corners, ids, _ = detector.detectMarkers(canvas)
    if ids is None or len(ids) != 1:
        found = 0 if ids is None else len(ids)
        raise SystemExit(
            f"ERROR: marker {marker_id}: re-detection found {found} markers, expected 1. "
            "The emitted module grid is not a valid marker."
        )
    if int(ids[0][0]) != marker_id:
        raise SystemExit(
            f"ERROR: marker {marker_id}: re-detection returned id {int(ids[0][0])}. "
            "The emitted grid encodes the WRONG marker (a flip or transpose)."
        )


def verify_card_detectable(detector, s: Sticker, grid: np.ndarray,
                           px_per_mm: float = 20.0) -> None:
    """GATE 4: rasterise the WHOLE CARD as it is printed and detect it back.

    GATE 2 renders the bare module grid on clean white and has never seen a card,
    so it says nothing about the ink this sheet puts AROUND the marker. This gate
    draws the sticker square, the trim-limit square, the cut outline and the
    label strip at their real millimetre offsets and requires the detector to
    find this marker and no other. It is deliberately pessimistic against the
    real print: the guide squares are drawn SOLID rather than dashed and the
    label is a solid black bar rather than text, so both put more ink near the
    marker than the sheet actually does.

    WHAT IT DOES AND DOES NOT PROVE - measured, not assumed. Sweeping the guide
    offsets against this gate shows it fires only once ink physically OVERLAPS
    the black square (label ink at -1.0 mm relative to the black edge). A thin
    line drawn even in BLACK, 0.9 mm inside the quiet zone, still detects - at
    print resolution and at 24 px across the black square with blur, the whole
    graded range. So:
        catches   gross ink placement - a label over the marker, a guide square
                  crossing the black, a flipped or wrong-id grid.
        does NOT  establish the quiet-zone margin. That claim stays GEOMETRIC and
                  is the honest form of it: the trim line sits TRIM_LINE_MM
                  OUTSIDE the sticker square, so the full one-module quiet zone
                  (1.00 mm on the 6 mm markers) remains white by construction.
    Do not cite a green GATE 4 as evidence that a smaller quiet zone would work.
    """
    def px(v: float) -> int:
        return int(round(v * px_per_mm))

    w, h = s.card_w, s.card_h
    pad = 6.0                                   # white beyond the cut line
    img = np.full((px(h + 2 * pad), px(w + 2 * pad)), 255, np.uint8)

    def rect(x0, y0, x1, y1, val):
        img[px(y0 + pad):px(y1 + pad), px(x0 + pad):px(x1 + pad)] = val

    def frame(x0, y0, x1, y1, val, lw):
        rect(x0, y0, x1, y0 + lw, val)
        rect(x0, y1 - lw, x1, y1, val)
        rect(x0, y0, x0 + lw, y1, val)
        rect(x1 - lw, y0, x1, y1, val)

    # Greys come from the sheet's own colours, so this gate tests what is
    # actually printed and follows any change to them.
    frame(0, 0, w, h, int(GUIDE[1:3], 16), 0.15)           # outer cut outline
    sx = (w - s.sticker_mm) / 2.0
    sy = CUT_RING_MM
    frame(sx - TRIM_LINE_MM, sy - TRIM_LINE_MM,            # inner trim limit
          sx + s.sticker_mm + TRIM_LINE_MM, sy + s.sticker_mm + TRIM_LINE_MM,
          int(LIMIT[1:3], 16), 0.12)
    rect(1.0, h - LABEL_STRIP_MM + 1.0, w - 1.0, h - 1.0, 0)   # label, as solid ink

    module = s.black_mm / MARKER_MODULES
    bx, by = sx + QUIET_MODULES * module, sy + QUIET_MODULES * module
    for r, c0, c1 in black_runs(grid):
        rect(bx + c0 * module, by + r * module,
             bx + c1 * module, by + (r + 1) * module, 0)

    corners, ids, _ = detector.detectMarkers(img)
    found = 0 if ids is None else len(ids)
    if found != 1 or int(ids[0][0]) != s.marker_id:
        got = "none" if ids is None else ",".join(str(int(i[0])) for i in ids)
        raise SystemExit(
            f"ERROR: marker {s.marker_id} ({s.human_label}): the printed CARD detects as "
            f"{found} marker(s) [{got}], expected exactly this id. The guide lines or the "
            f"label are too close to a {s.black_mm:g} mm marker - increase CUT_RING_MM / "
            "TRIM_LINE_MM rather than shipping a sheet whose stickers do not detect.")


def black_runs(grid: np.ndarray) -> list[tuple[int, int, int]]:
    """Merge black modules into maximal horizontal runs -> (row, col0, col1).

    Fewer rects means fewer abutting edges for a renderer to antialias a seam
    into, and a smaller file.
    """
    runs: list[tuple[int, int, int]] = []
    for r in range(grid.shape[0]):
        c = 0
        while c < grid.shape[1]:
            if grid[r, c]:
                c0 = c
                while c < grid.shape[1] and grid[r, c]:
                    c += 1
                runs.append((r, c0, c))
            else:
                c += 1
    return runs


# --------------------------------------------------------------------------
# Shelf packing - re-runnable, so the sheet adapts if markers.csv changes
# --------------------------------------------------------------------------

def pack(stickers: list[Sticker], usable_w: float) -> list[list[Sticker]]:
    ordered = sorted(stickers, key=lambda s: (-s.card_w, -s.card_h, s.marker_id))
    rows: list[list[Sticker]] = []
    row: list[Sticker] = []
    width = 0.0
    for s in ordered:
        add = s.card_w if not row else s.card_w + CARD_GAP_MM
        if row and width + add > usable_w:
            rows.append(row)
            row, width = [s], s.card_w
        else:
            row.append(s)
            width += add
    if row:
        rows.append(row)
    return rows


# --------------------------------------------------------------------------
# SVG emission. viewBox is in millimetres: 1 user unit == 1 mm, exactly.
# --------------------------------------------------------------------------

def t(x: float) -> str:
    """Trim a float to 4 dp without exponent notation."""
    return f"{x:.4f}".rstrip("0").rstrip(".") or "0"


def text(x, y, s, size, *, anchor="middle", fill=INK, weight="normal", family=None):
    fam = family or "Helvetica, Arial, sans-serif"
    return (f'<text x="{t(x)}" y="{t(y)}" font-family="{fam}" font-size="{t(size)}" '
            f'font-weight="{weight}" fill="{fill}" text-anchor="{anchor}">'
            f'{xml_escape(s)}</text>')


def draw_card(s: Sticker, grid: np.ndarray, x: float, y: float) -> list[str]:
    """One sticker card: cut outline, white sticker square, marker, label."""
    out: list[str] = []
    w, h = s.card_w, s.card_h

    # Cut outline. Grey and thin so nobody mistakes it for marker ink.
    out.append(
        f'<rect x="{t(x)}" y="{t(y)}" width="{t(w)}" height="{t(h)}" rx="0.8" '
        f'fill="{PAPER}" stroke="{GUIDE}" stroke-width="0.15" '
        f'stroke-dasharray="1.2 1.0"/>'
    )

    # The sticker square (white, incl. the one-module quiet zone), centred
    # horizontally, sitting above the label strip.
    sx = x + (w - s.sticker_mm) / 2.0
    sy = y + CUT_RING_MM
    out.append(
        f'<rect x="{t(sx)}" y="{t(sy)}" width="{t(s.sticker_mm)}" '
        f'height="{t(s.sticker_mm)}" fill="{PAPER}"/>'
    )

    # INNER TRIM LIMIT. Sits TRIM_LINE_MM outside the sticker square, so cutting
    # ON it still leaves the full one-module quiet zone (plus that margin), and
    # so the line itself is never ink inside the quiet zone. Lighter and finer
    # than the cut line: it is a limit, not a path.
    out.append(
        f'<rect x="{t(sx - TRIM_LINE_MM)}" y="{t(sy - TRIM_LINE_MM)}" '
        f'width="{t(s.trim_mm)}" height="{t(s.trim_mm)}" fill="none" '
        f'stroke="{LIMIT}" stroke-width="0.12" stroke-dasharray="0.5 0.5"/>'
    )

    # The 6x6 black square, inset by exactly one module of quiet zone.
    module = s.black_mm / MARKER_MODULES
    bx = sx + QUIET_MODULES * module
    by = sy + QUIET_MODULES * module
    for r, c0, c1 in black_runs(grid):
        out.append(
            f'<rect x="{t(bx + c0 * module)}" y="{t(by + r * module)}" '
            f'width="{t((c1 - c0) * module + EPS_MM)}" '
            f'height="{t(module + EPS_MM)}" fill="{INK}"/>'
        )

    # Label strip - INSIDE the cut line so it travels with the sticker, OUTSIDE
    # the quiet zone so it never touches detection. The arrow fixes which way is
    # up, so a re-printed sticker is re-applied identically and the residual
    # sign convention cannot silently flip.
    #
    # Arrow and text are laid out as one centred group and the font is shrunk to
    # fit the card, because a label that overruns its own cut line is a label
    # that gets trimmed off by the scissors.
    label = f"{s.human_label}  id{s.marker_id}  {t(s.black_mm)}mm"
    pad = 2.0
    gap = 1.0
    avail = w - 2 * pad
    # arrow width is 1.1 * fs, so: fs * (1.1 + gap/fs + GLYPH_W*len) <= avail
    fs = min(LABEL_FS_MAX, (avail - gap) / (1.1 + GLYPH_W * len(label)))
    fs = max(fs, LABEL_FS_MIN)
    arrow_w = 1.1 * fs
    group_w = arrow_w + gap + GLYPH_W * fs * len(label)
    gx = x + max(pad, (w - group_w) / 2.0)
    ly = y + h - LABEL_STRIP_MM / 2.0 + fs * 0.35
    out.append(
        f'<path d="M {t(gx)} {t(ly - fs * 0.15)} l {t(arrow_w / 2)} {t(-fs * 0.85)} '
        f'l {t(arrow_w / 2)} {t(fs * 0.85)} z" fill="{INK}"/>'
    )
    out.append(text(gx + arrow_w + gap, ly, label, fs, anchor="start"))
    return out


def draw_rule(x: float, y: float) -> list[str]:
    """The 100 mm scale check. Ticks every 10 mm so a scale error is visible,
    not just detectable by a careful measurement."""
    out = [f'<line x1="{t(x)}" y1="{t(y)}" x2="{t(x + RULE_LEN_MM)}" y2="{t(y)}" '
           f'stroke="{INK}" stroke-width="0.3"/>']
    for i in range(0, int(RULE_LEN_MM) + 1, 5):
        major = i % 10 == 0
        out.append(f'<line x1="{t(x + i)}" y1="{t(y)}" x2="{t(x + i)}" '
                   f'y2="{t(y + (2.6 if major else 1.4))}" stroke="{INK}" '
                   f'stroke-width="{0.3 if major else 0.2}"/>')
        if major and i % 20 == 0:
            out.append(text(x + i, y + 5.6, str(i), 2.2))
    out.append(text(x + RULE_LEN_MM / 2.0, y - 1.8,
                    "100 mm - MEASURE THIS BEFORE CUTTING ANYTHING", 2.6, weight="bold"))
    out.append(text(x + RULE_LEN_MM + 4, y + 1.2,
                    "If this line is not exactly 100 mm, the printer rescaled the page.",
                    2.2, anchor="start"))
    out.append(text(x + RULE_LEN_MM + 4, y + 4.4,
                    "Reprint at Actual Size / 100% / Scale: None. Do NOT use Fit to Page.",
                    2.2, anchor="start"))
    return out


def build_svg(stickers: list[Sticker], grids: dict[int, np.ndarray],
              csv_path: str) -> tuple[str, list[tuple], float]:
    """Returns (svg, placed_cards, last_baseline_y).

    The two extra returns exist so main() can gate the geometry it just emitted -
    the placement diagram has had a layout gate since it was written and this
    sheet has not, which is how a widened trim margin could have pushed the last
    row or the footer off A4 without anything complaining.
    """
    usable_w = PAGE_W_MM - 2 * MARGIN_MM
    placed: list[tuple] = []           # (sticker, x, y, w, h)
    body: list[str] = [
        f'<rect x="0" y="0" width="{t(PAGE_W_MM)}" height="{t(PAGE_H_MM)}" fill="{PAPER}"/>'
    ]

    x0 = MARGIN_MM
    body.append(text(x0, 14, "FactoryLM / Emre Kalem arm - fiducial marker sheet",
                     5.0, anchor="start", weight="bold"))
    body.append(text(x0, 19.6,
                     f"ArUco {DICT_NAME}  -  {len(stickers)} stickers  -  "
                     f"quiet zone {QUIET_MODULES} module (already inside every white square)"
                     f"  -  cut on the OUTER line, never inside the INNER line",
                     2.6, anchor="start"))
    body.append(text(x0, 23.8,
                     "Generated by Software/arm-vision/make_marker_sheet.py from "
                     f"{csv_path}. Do not edit this SVG by hand.",
                     2.2, anchor="start", fill="#555555"))
    body.append(f'<line x1="{t(x0)}" y1="{t(26.5)}" x2="{t(PAGE_W_MM - MARGIN_MM)}" '
                f'y2="{t(26.5)}" stroke="{INK}" stroke-width="0.35"/>')

    body += draw_rule(x0, 36.0)

    y = 50.0
    for row in pack(stickers, usable_w):
        x = x0
        row_h = max(s.card_h for s in row)
        for s in row:
            body += draw_card(s, grids[s.marker_id], x, y)
            placed.append((s, x, y, s.card_w, s.card_h))
            x += s.card_w + CARD_GAP_MM
        y += row_h + CARD_GAP_MM + 2.0

    # ---- footer -----------------------------------------------------------
    fy = y + 6.0
    body.append(f'<line x1="{t(x0)}" y1="{t(fy)}" x2="{t(PAGE_W_MM - MARGIN_MM)}" '
                f'y2="{t(fy)}" stroke="{INK}" stroke-width="0.35"/>')
    fy += 5.2
    body.append(text(x0, fy, "BEFORE YOU CUT", 3.2, anchor="start", weight="bold"))
    # Every number in the trim warning is BUILT FROM THE DATA, not typed. If
    # markers.csv changes a size these sentences change with it rather than
    # quietly going stale - which is how the old sheet came to promise that the
    # printed label "travels with the sticker" for markers that must lose it.
    over = sorted((s for s in stickers if s.oversize), key=lambda k: k.marker_id)
    smallest = min(s.quiet_mm for s in stickers)
    tight = min((s for s in stickers if s.island_mm is not None),
                key=lambda k: k.island_mm - k.trim_mm)
    tight_ids = "/".join(str(s.marker_id) for s in stickers
                         if s.island_mm is not None
                         and abs((s.island_mm - s.trim_mm)
                                 - (tight.island_mm - tight.trim_mm)) < 1e-6)

    lines = [
        "1. Print at Actual Size / 100%. Check the 100 mm rule above with a steel rule. "
        "Everything on this sheet is wrong if that fails.",
        f"2. TWO lines per marker. Cut on the OUTER dashed line: it clears the inner line "
        f"by {SCISSOR_MARGIN_MM:.1f} mm and the next marker by {CARD_GAP_MM:.1f} mm.",
        "3. The INNER dotted square is the TRIM LIMIT. NEVER cut inside it and never let "
        "ink cross it - the white it encloses is the quiet zone",
        f"   ArUco needs, one module, only {smallest:.1f} mm on the smallest markers. Cut "
        "inside that line and the marker stops being detected at all.",
        f"4. TRIM TO FIT. {len(over)} of {len(stickers)} cards are LARGER than the surveyed "
        "surface they go on - the printed label needs the room, the arm does not",
        "   have it. Cut the outer line, offer the sticker up, then trim toward the inner "
        f"line and never past it. The inner square DOES fit: tightest is",
        f"   id{tight_ids} at {tight.trim_mm:.1f} mm inside a {tight.island_mm:.1f} mm "
        "island. Trimming that far removes the label AND ITS UP-ARROW, so mark the id and "
        "the up edge",
        "   on the back before you cut - note 5 is why the arrow matters more than the id.",
        "5. The arrow on each label is UP. Apply the sticker that way up every time - a "
        "re-applied sticker at a different rotation silently flips the residual sign.",
        "6. The number on the label (e.g. 36mm) is the BLACK SQUARE edge. That is the value "
        "you pass to cv2.aruco / solvePnP as the marker length -",
        "   NOT the size of the white square you cut out, and NOT the size of the paper card.",
        "7. Matte paper or matte label stock. A gloss finish specular-highlights under the "
        "bench lamp and blinds the detector at exactly the angles you care about.",
        "8. Stick each marker only where docs/marker_placement_diagram.svg says. Several "
        "faces that look ideal are inner channel walls you cannot reach.",
    ]
    fy += 4.6
    for ln in lines:
        # Footer lines are full-width prose; a line that overran the right margin
        # would simply be clipped by the printer with no warning at all.
        w_mm = helv_mm(ln, 2.35)
        if w_mm > usable_w:
            raise SystemExit(
                f"ERROR: footer line is {w_mm:.1f} mm wide, page allows {usable_w:.1f} mm: "
                f"{ln[:60]!r}...")
        body.append(text(x0, fy, ln, 2.35, anchor="start"))
        fy += 3.5

    fy += 2.0
    body.append(text(x0, fy,
                     "WHAT THIS SHEET IS NOT: a measurement. These are blanks. A sticker "
                     "yields a CHANGE in orientation, not a joint angle, until the two",
                     2.35, anchor="start", weight="bold"))
    fy += 3.5
    body.append(text(x0, fy,
                     "calibration passes in Documentation/MARKER-SYSTEM.md section 6 have "
                     "been run. Detection grades are on the placement diagram, not here:",
                     2.35, anchor="start"))
    fy += 3.5
    body.append(text(x0, fy,
                     "a grade depends on the camera standoff and on an UNVERIFIED focal "
                     "length, and a number that can be wrong should not look authoritative.",
                     2.35, anchor="start"))

    body.append(text(PAGE_W_MM / 2.0, PAGE_H_MM - 6.0,
                     "Sources: Software/vision/markers.csv - Software/vision/stl-face-survey.csv "
                     "- Documentation/MARKER-SYSTEM.md", 2.1, fill="#555555"))

    head = (
        f'<svg xmlns="http://www.w3.org/2000/svg" version="1.1" '
        f'width="{t(PAGE_W_MM)}mm" height="{t(PAGE_H_MM)}mm" '
        f'viewBox="0 0 {t(PAGE_W_MM)} {t(PAGE_H_MM)}" shape-rendering="crispEdges">\n'
        f'<title>Emre Kalem arm - ArUco {DICT_NAME} marker print sheet (A4, print at 100%)</title>\n'
    )
    return head + "\n".join(body) + "\n</svg>\n", placed, fy


# --------------------------------------------------------------------------
# GATE 3 - page geometry. The sheet had no layout gate at all; widening a trim
# margin could have pushed the last row or the footer off A4 in silence.
# --------------------------------------------------------------------------

def check_page(placed: list[tuple], last_y: float) -> list[str]:
    """Return the human-readable geometry report. Raises on anything off-page or
    on a cut line that cannot physically be cut."""
    problems: list[str] = []
    right, bottom = PAGE_W_MM - MARGIN_MM, PAGE_H_MM - MARGIN_MM
    for s, x, y, w, h in placed:
        if x < MARGIN_MM - 1e-6 or x + w > right + 1e-6:
            problems.append(f"id{s.marker_id} card spans x {x:.1f}..{x + w:.1f}, "
                            f"page allows {MARGIN_MM:.1f}..{right:.1f}")
        if y < MARGIN_MM - 1e-6 or y + h > bottom + 1e-6:
            problems.append(f"id{s.marker_id} card spans y {y:.1f}..{y + h:.1f}, "
                            f"page allows {MARGIN_MM:.1f}..{bottom:.1f}")
    if last_y > bottom:
        problems.append(f"footer text reaches y {last_y:.1f}, page allows {bottom:.1f}")

    # The whole point of the inner line is that trimming TO it leaves a sticker
    # that fits. If the trim square were bigger than the surveyed island the line
    # would be a lie and the fitter would have to cut into the quiet zone.
    clear = None
    for s, *_ in placed:
        if s.island_mm is None:
            continue
        # PER SIDE. `island_mm` and `trim_mm` are both EDGE LENGTHS, so their
        # difference is twice the room you actually get when the trim square is
        # centred on the island. Reported straight, it sat directly beneath
        # SCISSOR_MARGIN_MM -- which IS a per-side offset -- and the two read as
        # the same kind of number while differing by a factor of two.
        c = (s.island_mm - s.trim_mm) / 2.0
        if c < 0:
            problems.append(f"id{s.marker_id} trim limit {s.trim_mm:.2f} mm exceeds its "
                            f"{s.island_mm:.1f} mm island - cannot be applied without "
                            "cutting into the quiet zone")
        clear = c if clear is None else min(clear, c)

    # Smallest clear white gap between any two cut outlines. This is the number
    # behind "can this be cut with scissors" - not an opinion.
    gap = None
    for i, (si, xi, yi, wi, hi) in enumerate(placed):
        for sj, xj, yj, wj, hj in placed[i + 1:]:
            dx = max(xj - (xi + wi), xi - (xj + wj))
            dy = max(yj - (yi + hi), yi - (yj + hj))
            d = max(dx, dy)          # boxes are axis-aligned and never overlap
            if d < 0:
                problems.append(f"id{si.marker_id} and id{sj.marker_id} cut outlines OVERLAP")
                continue
            gap = d if gap is None else min(gap, d)
    if gap is not None and gap < 2.0:
        problems.append(f"closest two cut outlines are {gap:.2f} mm apart - "
                        "not cuttable with scissors")
    if problems:
        for p in problems:
            print("  " + p)
        raise SystemExit(f"ERROR: {len(problems)} page-geometry problem(s); refusing to "
                         "write a sheet that cannot be printed or cut as drawn.")

    report = [f"min gap between adjacent cut outlines : {gap:.2f} mm",
              f"scissors margin, cut line -> trim limit: {SCISSOR_MARGIN_MM:.2f} mm per side",
              f"min clearance, trim limit -> island    : {clear:.2f} mm per side",
              f"lowest ink on the page                : y {last_y:.1f} mm "
              f"(A4 usable to {bottom:.1f} mm)"]
    return report


# --------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.abspath(os.path.join(here, "..", ".."))

    ap = argparse.ArgumentParser(description=__doc__.split("\n")[2])
    ap.add_argument("--csv", default=os.path.join(repo, "Software", "vision", "markers.csv"))
    ap.add_argument("--out", default=os.path.join(repo, "docs", "marker_print_sheet.svg"))
    ap.add_argument("--block-px", type=int, default=8,
                    help="pixels per module when generating the bitmap (>=4)")
    args = ap.parse_args(argv)

    if args.block_px < 4:
        raise SystemExit("ERROR: --block-px must be >= 4")

    stickers = load_stickers(args.csv)
    print(f"markers.csv          : {args.csv}")
    print(f"stickers             : {len(stickers)}")

    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICT_NAME))
    if dictionary.markerSize != MARKER_MODULES - 2:
        raise SystemExit(
            f"ERROR: {DICT_NAME} has markerSize {dictionary.markerSize}; this script "
            f"assumes {MARKER_MODULES - 2} data modules + 1 border ring."
        )
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())

    grids: dict[int, np.ndarray] = {}
    for s in stickers:
        if s.marker_id >= dictionary.bytesList.shape[0]:
            raise SystemExit(
                f"ERROR: marker id {s.marker_id} is outside {DICT_NAME} "
                f"(0..{dictionary.bytesList.shape[0] - 1})"
            )
        g = marker_grid(dictionary, s.marker_id, args.block_px)   # GATE 1 + strict
        verify_detectable(dictionary, detector, g, s.marker_id)   # GATE 2
        verify_card_detectable(detector, s, g)                    # GATE 4
        grids[s.marker_id] = g
    print(f"gate 1 module align  : PASS ({len(stickers)}/{len(stickers)})")
    print(f"gate 2 re-detection  : PASS ({len(stickers)}/{len(stickers)})")
    print(f"gate 4 printed card  : PASS ({len(stickers)}/{len(stickers)}) - each card "
          "detects with its cut line, trim line and label ink in frame")
    print("                       (catches ink OVERLAPPING the marker; the quiet-zone "
          "margin itself is a geometric claim - see the docstring)")

    svg, placed, last_y = build_svg(
        stickers, grids, os.path.relpath(args.csv, repo).replace("\\", "/"))
    for ln in check_page(placed, last_y):                       # GATE 3
        print(f"gate 3 page geometry : {ln}")

    print("per-marker cut geometry (mm) - quiet zone is what every cut must leave:")
    print(f"  {'id':>3} {'label':<11} {'black':>6} {'sticker':>8} {'quiet':>6} "
          f"{'trim sq':>8} {'card':>12} {'island':>7}  applied as cut?")
    for s in sorted(stickers, key=lambda k: k.marker_id):
        isl = f"{s.island_mm:.1f}" if s.island_mm is not None else "n/a"
        verdict = ("MUST TRIM" if s.oversize else
                   "fits" if s.island_mm is not None else "curved - see diagram")
        print(f"  {s.marker_id:>3} {s.human_label:<11} {s.black_mm:6.1f} "
              f"{s.sticker_mm:8.2f} {s.quiet_mm:6.2f} {s.trim_mm:8.2f} "
              f"{s.card_w:5.1f} x {s.card_h:4.1f} {isl:>7}  {verdict}")

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(svg)

    total_black = sum(s.black_mm ** 2 for s in stickers)
    print(f"wrote                : {args.out}")
    print(f"bytes                : {os.path.getsize(args.out)}")
    print(f"total black-square   : {total_black:.0f} mm^2 of ink across {len(stickers)} markers")
    print("PRINT AT 100% / ACTUAL SIZE. Verify the 100 mm rule before cutting.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
