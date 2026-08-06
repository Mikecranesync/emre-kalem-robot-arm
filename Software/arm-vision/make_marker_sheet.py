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

# The cut line is held OFF the quiet-zone boundary. approximate_size_mm already
# includes the one-module quiet zone, so a hairline drawn at that edge would be
# ink at the quiet zone - which MARKER-SYSTEM.md section 5 forbids. The ring is
# spare white; more quiet zone never hurts detection.
CUT_RING_MM = 2.5
LABEL_STRIP_MM = 5.0        # inside the cut line, so the label travels with the
                            # sticker and it can be re-applied the same way up
MIN_CARD_W_MM = 28.0        # so a 6 mm finger sticker still carries a legible
                            # "FINGER-A id11 6mm" without the label overrunning
                            # the card it is printed on
CARD_GAP_MM = 4.0
LABEL_FS_MAX = 2.6
LABEL_FS_MIN = 1.5
GLYPH_W = 0.55              # width of one Helvetica character as a fraction of
                            # font size - close enough to fit a label, and the
                            # only consequence of being wrong is whitespace

RULE_LEN_MM = 100.0         # the printer scale check
EPS_MM = 0.01               # sub-printer-resolution overlap that closes hairline
                            # seams between abutting black rects (600 dpi = 0.042 mm)

INK = "#000000"
PAPER = "#ffffff"
GUIDE = "#9a9a9a"           # cut lines and rule ticks - grey so it is obviously
                            # not part of any marker


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

    @property
    def card_w(self) -> float:
        return max(self.sticker_mm + 2 * CUT_RING_MM, MIN_CARD_W_MM)

    @property
    def card_h(self) -> float:
        return self.sticker_mm + 2 * CUT_RING_MM + LABEL_STRIP_MM


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
            "black_square_mm", "approximate_size_mm")
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
        out.append(Sticker(
            marker_id=int(row[idx["marker_id"]]),
            human_label=row[idx["human_label"]].strip(),
            link_name=row[idx["link_name"]].strip(),
            black_mm=black,
            # Use the DERIVED value, not the rounded CSV value: this is the one
            # number the printed geometry must honour exactly.
            sticker_mm=derived,
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


def build_svg(stickers: list[Sticker], grids: dict[int, np.ndarray], csv_path: str) -> str:
    usable_w = PAGE_W_MM - 2 * MARGIN_MM
    body: list[str] = [
        f'<rect x="0" y="0" width="{t(PAGE_W_MM)}" height="{t(PAGE_H_MM)}" fill="{PAPER}"/>'
    ]

    x0 = MARGIN_MM
    body.append(text(x0, 14, "FactoryLM / Emre Kalem arm - fiducial marker sheet",
                     5.0, anchor="start", weight="bold"))
    body.append(text(x0, 19.6,
                     f"ArUco {DICT_NAME}  -  {len(stickers)} stickers  -  "
                     f"quiet zone {QUIET_MODULES} module (already inside every white square)",
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
            x += s.card_w + CARD_GAP_MM
        y += row_h + CARD_GAP_MM + 2.0

    # ---- footer -----------------------------------------------------------
    fy = y + 6.0
    body.append(f'<line x1="{t(x0)}" y1="{t(fy)}" x2="{t(PAGE_W_MM - MARGIN_MM)}" '
                f'y2="{t(fy)}" stroke="{INK}" stroke-width="0.35"/>')
    fy += 5.2
    body.append(text(x0, fy, "BEFORE YOU CUT", 3.2, anchor="start", weight="bold"))
    lines = [
        "1. Print at Actual Size / 100%. Check the 100 mm rule above with a steel rule. "
        "Everything on this sheet is wrong if that fails.",
        "2. Cut on the grey dashed line. The white ring between the line and the marker is "
        "SPARE quiet zone - keeping it is free and helps detection.",
        "3. Never write, stamp or stick anything inside the white square. Ink in the quiet "
        "zone breaks detection. The printed label is deliberately outside it.",
        "4. The arrow on each label is UP. Apply the sticker that way up every time - a "
        "re-applied sticker at a different rotation silently flips the residual sign.",
        "5. The number on the label (e.g. 36mm) is the BLACK SQUARE edge. That is the value "
        "you pass to cv2.aruco / solvePnP as the marker length -",
        "   NOT the size of the white square you cut out, and NOT the size of the paper card.",
        "6. Matte paper or matte label stock. A gloss finish specular-highlights under the "
        "bench lamp and blinds the detector at exactly the angles you care about.",
        "7. Stick each marker only where docs/marker_placement_diagram.svg says. Several "
        "faces that look ideal are inner channel walls you cannot reach.",
    ]
    fy += 4.6
    for ln in lines:
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
    return head + "\n".join(body) + "\n</svg>\n"


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
        grids[s.marker_id] = g
    print(f"gate 1 module align  : PASS ({len(stickers)}/{len(stickers)})")
    print(f"gate 2 re-detection  : PASS ({len(stickers)}/{len(stickers)})")

    svg = build_svg(stickers, grids, os.path.relpath(args.csv, repo).replace("\\", "/"))
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
