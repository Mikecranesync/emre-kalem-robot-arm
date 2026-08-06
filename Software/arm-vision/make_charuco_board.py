#!/usr/bin/env python3
"""
make_charuco_board.py - generate docs/charuco_calibration_board.svg, the
printable ChArUco board used to obtain real camera intrinsics.

WHY THIS EXISTS
    Every marker size in Software/vision/markers.csv, and every detection grade
    in docs/marker_placement_table.csv, was derived from an ASSUMED camera field
    of view of about 60 degrees (f_px = 1280 / (2*tan 30) ~ 1108 px). That
    assumption is recorded as open item A1 in docs/marker_system.md section 8
    and has never been checked against a camera. Until it is, the whole sticker
    set is unproven and printing it is speculative.

    docs/marker_system.md section 1 and Documentation/MARKER-SYSTEM.md section 2
    both say what to do about it - "print a 5x7 board with 25 mm squares in the
    same DICT_4X4_50" - but neither ships a generator, so the board did not
    exist and the gate could not be run. This script is the missing half of that
    sentence. calibrate_intrinsics.py is the other half.

    NOTHING HERE VERIFIES ANYTHING ABOUT A CAMERA. This emits a piece of paper.
    The ~60 degree field of view remains an ASSUMPTION and every size derived
    from it remains UNVERIFIED until a real camera has been run through
    calibrate_intrinsics.py and passed its gate.

WHAT IT EMITS AND WHY IT IS VECTOR, NOT A BITMAP
    Same reasoning as make_marker_sheet.py, and it matters more here. A ChArUco
    board is read by sub-pixel chessboard-corner refinement; the corner is
    the intersection of two black squares. An <image> data URI gets smooth-scaled
    by print pipelines that do not honour image-rendering:crisp-edges, and a
    softened corner biases exactly the refinement this board exists to feed.
    Vector rects cannot blur at any zoom or DPI.

    The geometry is not trusted on faith. Three gates run on every generation:
      GATE 1  each ArUco bitmap decomposes cleanly into its 6x6 module grid -
              every NxN pixel block must be uniformly black or uniformly white.
              A misaligned grid fails loudly instead of succeeding quietly,
              which is what sampling a module CENTRE pixel would do.
      GATE 2  the rects this script is about to write as SVG are rasterised and
              compared PIXEL FOR PIXEL against cv2.aruco.CharucoBoard's own
              generateImage(). This is the strong gate: it proves the SVG
              encodes the identical board OpenCV will later detect, including
              the chessboard parity, the white-square scan order, the marker
              inset, and the absence of any flip or transpose.

              ONE HONEST CAVEAT, because "pixel for pixel" overstates by a hair.
              The gate rasterises the RAW rect list; build_svg() then writes each
              rect grown by EPS_MM on all four sides, to stop hairline seams
              appearing between abutting squares in some renderers. So the board
              ink on disk measures 125.02 mm, not the 125.00 mm the gate proved.
              0.01 mm per side is far below one pixel at 600 dpi and rounds away
              at the gate's own raster, so nothing is physically wrong with the
              board -- but the gate validates the geometry BEFORE that expansion,
              and saying otherwise would be the kind of claim this file exists to
              avoid.
      GATE 3  that same raster, padded with a white quiet margin, is fed back to
              cv2.aruco.CharucoDetector. Every ChArUco corner and every ArUco
              marker must be found, and each recovered corner must land on its
              ideal grid location within a stated pixel tolerance.

BOARD SPEC, AND WHY THE MARKER IDS ARE OFFSET
    5 x 7 squares, 25.000 mm squares, 18.750 mm markers, DICT_4X4_50.
    24 interior ChArUco corners, 17 ArUco markers.

    The 5x7 / 25 mm / DICT_4X4_50 part is fixed by the two design documents and
    is not a free parameter here.

    The marker ids ARE a free parameter, and this script does NOT use the
    default 0..16. It uses 20..36. Reason: the arm's own stickers occupy ids
    0..12 and are accepted by an id whitelist. A default board would put ids
    0..12 on a sheet of calibration paper that can easily be in frame at the
    same time as the arm - a board marker with id 4 is indistinguishable from
    SHOULDER-1 to anything downstream of that whitelist. Offsetting the board
    out of the arm's id space removes the collision by construction and needs no
    change to any whitelist. 20..36 fits inside DICT_4X4_50 (0..49) with room
    to spare. The offset is recorded on the printed sheet and must be passed to
    calibrate_intrinsics.py as --first-id.

    Neither design document pins the marker length or the board ids, so neither
    choice contradicts them. Both are printed ON the sheet so that whoever is
    holding it can confirm what it is without consulting a file.

WHY 18.750 mm MARKERS
    The marker/square ratio is 0.75, which makes every feature land on an exact
    integer pixel at the raster sizes used by GATE 2: square 120 px, marker
    90 px, module 15 px, inset 15 px. A ratio that does not divide cleanly would
    force cv2's renderer and this script's renderer to round independently, and
    GATE 2 would then be comparing two different roundings rather than two
    descriptions of the same board. 0.75 also leaves a 3.125 mm white gutter
    around each marker, which is one full module of quiet zone.

WHAT TO CALLIPER, AND WHAT MIS-SCALE ACTUALLY BREAKS
    Print at 100% and calliper the board. But be precise about the risk, because
    it is NOT the same risk as the sticker sheet:

      - The camera intrinsics (fx, fy, cx, cy, distortion) are INVARIANT to the
        board's true square size. Scaling every object point by k scales the
        recovered translations by k and leaves the camera matrix untouched. A
        board printed at 96% still yields the correct focal length.
      - What a mis-scaled board corrupts is DISTANCE. Every tvec, every standoff
        this board is later used to check, and every marker-derived range is
        wrong by the print error while looking healthy.

    So: calliper 4 squares across (exactly 100.0 mm by construction), and pass
    the callipered value to calibrate_intrinsics.py --callipered-square-mm so it
    is stamped into the intrinsics file. That value is OBSERVED - a human read
    it off an instrument - and it wins over the nominal 25.000 mm.

USAGE
    python Software/arm-vision/make_charuco_board.py
    python Software/arm-vision/make_charuco_board.py --out somewhere/else.svg
    python Software/arm-vision/make_charuco_board.py --first-id 20 --square-mm 25

DEPENDENCIES
    opencv-python  Apache-2.0. Generates the marker bitmaps, the reference board
                   raster for GATE 2, and the detector for GATE 3.
    numpy          BSD-3-Clause. FLAGGED, not silently accepted: the repository
                   rule is Apache-2.0 or MIT only. numpy is already an
                   unavoidable dependency of the vision and LeRobot work (cv2
                   requires it), so it is raised here for a decision rather than
                   passed over.
    stdlib         argparse, math, os, sys, xml.sax.saxutils.

VOCABULARY
    This arm has no shaft feedback of any kind and this script does not change
    that. Nothing here observes a joint. The board is bench equipment; the only
    values it will ever produce that may be called OBSERVED are the callipered
    square size (a human read it) and the intrinsics from a real camera capture
    (a camera saw it). Neither exists yet.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass
from xml.sax.saxutils import escape as xml_escape

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover - environment problem, not a code path
    sys.stderr.write(
        "ERROR: opencv-python is required to generate and gate the board.\n"
        "       pip install opencv-python\n"
    )
    raise


# --------------------------------------------------------------------------
# Constants that encode the design. The first four are fixed by
# docs/marker_system.md section 1 and Documentation/MARKER-SYSTEM.md section 2 -
# they are NOT free parameters. Change them only together with those documents.
# --------------------------------------------------------------------------

DICT_NAME = "DICT_4X4_50"
SQUARES_X = 5                # columns
SQUARES_Y = 7                # rows
SQUARE_MM = 25.0             # chessboard square edge - the doc-mandated value

# Free parameters, chosen here and justified in the module docstring.
MARKER_RATIO = 0.75          # marker edge / square edge -> 18.750 mm
FIRST_MARKER_ID = 20         # keeps the board clear of the arm's ids 0..12

MARKER_MODULES = 6           # 4 data modules + 1 black border ring each side

# GATE 2 raster scale. Must keep square_px, marker_px, module_px and the marker
# inset all integral: marker_px = 0.75 * square_px must divide by 6, i.e.
# square_px must be a multiple of 8.
SQUARE_PX = 120
GATE3_MARGIN_PX = SQUARE_PX // 2     # white quiet margin for the re-detection
GATE3_CORNER_TOL_PX = 0.25           # recovered corner vs ideal grid location

PAGE_W_MM = 210.0            # A4 portrait
PAGE_H_MM = 297.0
MARGIN_MM = 10.0

# White paper that must stay completely clear all the way around the board. The
# outermost chessboard squares include black ones that touch the board edge; an
# ArUco marker in a corner square needs white beyond that edge or its border
# ring cannot be segmented. Nothing may be drawn inside this ring - the code
# asserts it rather than trusting the layout arithmetic below.
QUIET_RING_MM = 12.0

RULE_LEN_MM = 100.0          # the printer scale check - also exactly 4 squares
EPS_MM = 0.01                # sub-printer-resolution overlap that closes
                             # hairline seams between abutting black rects
                             # (600 dpi = 0.042 mm)

INK = "#000000"
PAPER = "#ffffff"
GUIDE = "#9a9a9a"            # rule ticks - grey so they are obviously not board

# Footer text. "" is a paragraph gap. Kept as a module constant so the page-fit
# assertion in build_svg() has something to point at when it fails.
FOOTER_PITCH_MM = 2.85
FOOTER_GAP_MM = 1.3
FOOTER_LINES = [
    "This is CAMERA-CALIBRATION EQUIPMENT - bench kit, never glued to the arm. Every marker size in Software/vision/markers.csv",
    "and every grade in docs/marker_placement_table.csv rests on an ASSUMED ~60 deg field of view (f_px ~ 1108) that has NEVER",
    "been checked against a camera - open item A1. This board is how you settle it.",
    "",
    "PROCEDURE  1. Print at 100% on matte stock; verify the 100 mm rule.  2. Mount the sheet FLAT on rigid board - a bowed sheet is",
    "not a plane and silently corrupts the solve.  3. Capture >= 12 stills: board filling about a third of the frame, tilted 25-45",
    "deg in varied directions, at varied distance, pushed into all four corners of the image.  4. Run:",
    "     python Software/arm-vision/calibrate_intrinsics.py --captures <dir> --capture-source camera --callipered-square-mm <mm>",
    "That script REFUSES to emit intrinsics unless the capture set passes its gate. Full procedure: docs/marker_system.md section 9.",
    "",
    "MIS-SCALE RISK HERE IS NOT THE STICKER-SHEET RISK. Camera intrinsics (fx, fy, cx, cy, distortion) do NOT depend on this",
    "board's true square size - a board printed at 96% still gives the correct focal length. What mis-scale corrupts is DISTANCE.",
    "",
    "NOTHING ON THIS SHEET IS AN OBSERVATION. It is a blank. The ~60 deg field of view stays an ASSUMPTION and every size derived",
    "from it stays UNVERIFIED until a real camera has passed the gate in calibrate_intrinsics.py.",
]


# --------------------------------------------------------------------------
# Board description
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class BoardSpec:
    squares_x: int
    squares_y: int
    square_mm: float
    marker_mm: float
    first_id: int
    dict_name: str

    @property
    def width_mm(self) -> float:
        return self.squares_x * self.square_mm

    @property
    def height_mm(self) -> float:
        return self.squares_y * self.square_mm

    @property
    def corner_count(self) -> int:
        """Interior ChArUco corners - the things calibration actually consumes."""
        return (self.squares_x - 1) * (self.squares_y - 1)

    @property
    def module_mm(self) -> float:
        return self.marker_mm / MARKER_MODULES

    @property
    def inset_mm(self) -> float:
        """White gutter between a square's edge and the marker inside it."""
        return (self.square_mm - self.marker_mm) / 2.0

    def cv_board(self, dictionary) -> "cv2.aruco.CharucoBoard":
        ids = np.arange(self.first_id, self.first_id + self.marker_count,
                        dtype=np.int32)
        return cv2.aruco.CharucoBoard(
            (self.squares_x, self.squares_y),
            float(self.square_mm), float(self.marker_mm), dictionary, ids,
        )

    @property
    def marker_count(self) -> int:
        return len(white_squares(self.squares_x, self.squares_y))


def white_squares(sx: int, sy: int) -> list[tuple[int, int]]:
    """(row, col) of every white square, in row-major scan order.

    OpenCV's CharucoBoard makes square (0,0) BLACK and alternates, then drops
    one ArUco marker into each white square in this order. Getting either the
    parity or the order wrong produces a board that still looks like a ChArUco
    board and detects as a DIFFERENT one. This is asserted by GATE 2, not
    assumed - see build_black_rects().
    """
    return [(r, c) for r in range(sy) for c in range(sx) if (r + c) % 2 == 1]


# --------------------------------------------------------------------------
# ArUco -> module grid. Identical gate to make_marker_sheet.py GATE 1.
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

    expanded = np.where(np.kron(grid, np.ones((block_px, block_px), bool)), 0, 255)
    if not np.array_equal(expanded.astype(np.uint8), img):
        raise SystemExit(
            f"ERROR: marker {marker_id}: module grid does not reproduce the bitmap"
        )
    return grid


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
# The single source of geometry. Everything downstream - the SVG and the GATE 2
# raster - is built from this ONE list, so the gate tests what gets printed.
# --------------------------------------------------------------------------

def build_black_rects(spec: BoardSpec, dictionary) -> tuple[list[tuple[float, float, float, float]],
                                                            dict[int, np.ndarray]]:
    """Return every black rectangle on the board as (x_mm, y_mm, w_mm, h_mm),
    with the board's top-left corner at (0, 0), plus the module grids used.

    Two kinds of rect: whole black chessboard squares, and the module runs of
    each ArUco marker sitting inside a white square.
    """
    rects: list[tuple[float, float, float, float]] = []
    grids: dict[int, np.ndarray] = {}

    sq = spec.square_mm
    for r in range(spec.squares_y):
        for c in range(spec.squares_x):
            if (r + c) % 2 == 0:
                rects.append((c * sq, r * sq, sq, sq))

    mod = spec.module_mm
    for i, (r, c) in enumerate(white_squares(spec.squares_x, spec.squares_y)):
        marker_id = spec.first_id + i
        if marker_id >= dictionary.bytesList.shape[0]:
            raise SystemExit(
                f"ERROR: marker id {marker_id} is outside {spec.dict_name} "
                f"(0..{dictionary.bytesList.shape[0] - 1}). Lower --first-id."
            )
        grid = marker_grid(dictionary, marker_id)      # GATE 1
        grids[marker_id] = grid
        ox = c * sq + spec.inset_mm
        oy = r * sq + spec.inset_mm
        for mr, c0, c1 in black_runs(grid):
            rects.append((ox + c0 * mod, oy + mr * mod, (c1 - c0) * mod, mod))

    return rects, grids


def rasterise(spec: BoardSpec, rects, square_px: int) -> np.ndarray:
    """Render the rect list to a uint8 image at exactly square_px per square.

    Deliberately refuses any scale that would need rounding: a rect that does
    not land on an integer pixel means the GATE 2 comparison below would be
    testing two different roundings rather than two descriptions of one board.
    """
    ppm = square_px / spec.square_mm
    h = spec.squares_y * square_px
    w = spec.squares_x * square_px
    img = np.full((h, w), 255, np.uint8)
    for x, y, rw, rh in rects:
        vals = [x * ppm, y * ppm, rw * ppm, rh * ppm]
        ivals = [int(round(v)) for v in vals]
        if any(abs(v - iv) > 1e-6 for v, iv in zip(vals, ivals)):
            raise SystemExit(
                f"ERROR: rect {(x, y, rw, rh)} mm does not land on an integer "
                f"pixel at {square_px} px/square. Refusing to rasterise a board "
                "whose gate would compare two different roundings."
            )
        ix, iy, iw, ih = ivals
        img[iy:iy + ih, ix:ix + iw] = 0
    return img


# --------------------------------------------------------------------------
# GATE 2 and GATE 3
# --------------------------------------------------------------------------

def gate2_matches_opencv(spec: BoardSpec, raster: np.ndarray, cv_board) -> None:
    """The board this script draws must be pixel-identical to OpenCV's own.

    This is the strong gate. It simultaneously proves the chessboard parity, the
    white-square scan order, the marker id assignment, the marker inset, and the
    absence of a flip or transpose - none of which are individually asserted
    anywhere else, and every one of which produces a plausible-looking board
    that detects as something other than what the SVG claims.
    """
    ref = cv_board.generateImage(
        (spec.squares_x * SQUARE_PX, spec.squares_y * SQUARE_PX), marginSize=0
    )
    if ref.shape != raster.shape:
        raise SystemExit(
            f"ERROR: GATE 2: OpenCV rendered {ref.shape}, this script {raster.shape}"
        )
    if not np.array_equal(ref, raster):
        diff = int(np.count_nonzero(ref != raster))
        ys, xs = np.where(ref != raster)
        raise SystemExit(
            f"ERROR: GATE 2 FAILED: {diff} pixel(s) differ from "
            "cv2.aruco.CharucoBoard.generateImage(), first at row/col "
            f"{(int(ys[0]), int(xs[0]))}. The SVG would encode a DIFFERENT board "
            "from the one OpenCV detects. Refusing to write it."
        )


def gate3_redetect(spec: BoardSpec, raster: np.ndarray, cv_board) -> tuple[int, int, float]:
    """Detect the board back out of the raster this script is about to emit.

    Returns (charuco_corners_found, markers_found, max_corner_error_px).
    """
    m = GATE3_MARGIN_PX
    canvas = np.full((raster.shape[0] + 2 * m, raster.shape[1] + 2 * m), 255, np.uint8)
    canvas[m:m + raster.shape[0], m:m + raster.shape[1]] = raster

    detector = cv2.aruco.CharucoDetector(
        cv_board,
        cv2.aruco.CharucoParameters(),
        cv2.aruco.DetectorParameters(),
        cv2.aruco.RefineParameters(),
    )
    cc, ci, _mc, mi = detector.detectBoard(canvas)

    n_corners = 0 if cc is None else int(cc.shape[0])
    n_markers = 0 if mi is None else int(len(mi))

    if n_corners != spec.corner_count:
        raise SystemExit(
            f"ERROR: GATE 3 FAILED: re-detection found {n_corners} ChArUco corners, "
            f"expected {spec.corner_count}. Refusing to emit a board that cannot "
            "be fully detected in its own ideal rendering."
        )
    if n_markers != spec.marker_count:
        raise SystemExit(
            f"ERROR: GATE 3 FAILED: re-detection found {n_markers} ArUco markers, "
            f"expected {spec.marker_count}."
        )
    if mi is not None:
        want = set(range(spec.first_id, spec.first_id + spec.marker_count))
        got = set(int(v) for v in mi.ravel())
        if got != want:
            raise SystemExit(
                f"ERROR: GATE 3 FAILED: detected ids {sorted(got)} != expected "
                f"{sorted(want)}. The board encodes the wrong ids."
            )

    # Ideal grid location of every interior corner, in the padded canvas.
    ppm = SQUARE_PX / spec.square_mm
    ideal = cv_board.getChessboardCorners()[:, :2] * ppm + m
    got_xy = cc.reshape(-1, 2)
    err = np.abs(got_xy - ideal[ci.ravel()])
    max_err = float(err.max())
    if max_err > GATE3_CORNER_TOL_PX:
        raise SystemExit(
            f"ERROR: GATE 3 FAILED: worst recovered corner is {max_err:.4f} px from "
            f"its ideal grid location (tolerance {GATE3_CORNER_TOL_PX} px). "
            "The board geometry is not what it claims."
        )
    return n_corners, n_markers, max_err


# --------------------------------------------------------------------------
# SVG emission. viewBox is in millimetres: 1 user unit == 1 mm, exactly.
# --------------------------------------------------------------------------

def t(x: float) -> str:
    """Trim a float to 4 dp without exponent notation."""
    return f"{x:.4f}".rstrip("0").rstrip(".") or "0"


def text(x, y, s, size, *, anchor="middle", fill=INK, weight="normal"):
    return (f'<text x="{t(x)}" y="{t(y)}" font-family="Helvetica, Arial, sans-serif" '
            f'font-size="{t(size)}" font-weight="{weight}" fill="{fill}" '
            f'text-anchor="{anchor}">{xml_escape(s)}</text>')


def draw_rule(x: float, y: float) -> list[str]:
    """The printer scale check. 100 mm is also exactly 4 board squares, so this
    line and the board itself fail the same way if the printer rescales."""
    out = [f'<line x1="{t(x)}" y1="{t(y)}" x2="{t(x + RULE_LEN_MM)}" y2="{t(y)}" '
           f'stroke="{INK}" stroke-width="0.4"/>']
    for i in range(0, int(RULE_LEN_MM) + 1, 5):
        major = i % 25 == 0
        out.append(f'<line x1="{t(x + i)}" y1="{t(y)}" x2="{t(x + i)}" '
                   f'y2="{t(y + (3.0 if major else 1.6))}" stroke="{GUIDE}" '
                   f'stroke-width="{0.3 if major else 0.2}"/>')
        if major:
            out.append(text(x + i, y + 5.8, str(i), 2.1))
    out.append(text(x, y - 1.6,
                    "100 mm - AND EXACTLY 4 BOARD SQUARES. MEASURE BOTH.",
                    2.5, anchor="start", weight="bold"))
    out.append(text(x + RULE_LEN_MM + 4, y + 0.4,
                    "If this is not 100.0 mm the printer rescaled the page.",
                    2.1, anchor="start"))
    out.append(text(x + RULE_LEN_MM + 4, y + 3.4,
                    "Reprint at Actual Size / 100%. Do NOT use Fit to Page.",
                    2.1, anchor="start"))
    out.append(text(x + RULE_LEN_MM + 4, y + 6.4,
                    "Then calliper 4 squares and pass it to --callipered-square-mm.",
                    2.1, anchor="start"))
    return out


def build_svg(spec: BoardSpec, rects, board_x: float, board_y: float,
              rule_y: float, footer_y: float) -> str:
    body: list[str] = [
        f'<rect x="0" y="0" width="{t(PAGE_W_MM)}" height="{t(PAGE_H_MM)}" fill="{PAPER}"/>'
    ]
    x0 = MARGIN_MM

    body.append(text(x0, 11.0,
                     "FactoryLM / Emre Kalem arm - ChArUco camera-intrinsics board",
                     4.6, anchor="start", weight="bold"))
    body.append(text(x0, 16.2,
                     f"{spec.dict_name}  -  {spec.squares_x} x {spec.squares_y} squares  -  "
                     f"square {spec.square_mm:.3f} mm  -  marker {spec.marker_mm:.3f} mm  -  "
                     f"marker ids {spec.first_id}-{spec.first_id + spec.marker_count - 1}",
                     2.5, anchor="start", weight="bold"))
    body.append(text(x0, 19.8,
                     f"{spec.corner_count} interior ChArUco corners  -  "
                     f"{spec.marker_count} ArUco markers  -  board "
                     f"{spec.width_mm:.1f} x {spec.height_mm:.1f} mm  -  A4 portrait, print at 100%",
                     2.3, anchor="start"))
    body.append(text(x0, 23.4,
                     "Generated by Software/arm-vision/make_charuco_board.py. Do not edit this SVG "
                     "by hand. Ids are 20+ on purpose: the arm's stickers are ids 0-12.",
                     2.1, anchor="start", fill="#555555"))
    body.append(f'<line x1="{t(x0)}" y1="{t(25.6)}" x2="{t(PAGE_W_MM - MARGIN_MM)}" '
                f'y2="{t(25.6)}" stroke="{INK}" stroke-width="0.35"/>')

    body += draw_rule(x0, rule_y)

    # ---- the board itself -------------------------------------------------
    body.append(f'<g transform="translate({t(board_x)},{t(board_y)})">')
    for x, y, w, h in rects:
        body.append(f'<rect x="{t(x - EPS_MM)}" y="{t(y - EPS_MM)}" '
                    f'width="{t(w + 2 * EPS_MM)}" height="{t(h + 2 * EPS_MM)}" fill="{INK}"/>')
    body.append('</g>')

    # ---- footer -----------------------------------------------------------
    fy = footer_y
    body.append(f'<line x1="{t(x0)}" y1="{t(fy)}" x2="{t(PAGE_W_MM - MARGIN_MM)}" '
                f'y2="{t(fy)}" stroke="{INK}" stroke-width="0.35"/>')
    fy += 4.6
    body.append(text(x0, fy, "WHAT THIS BOARD IS FOR", 3.1, anchor="start", weight="bold"))
    fy += 4.2
    last_baseline = fy
    for ln in FOOTER_LINES:
        if ln:
            body.append(text(x0, fy, ln, 2.15, anchor="start"))
            last_baseline = fy
            fy += FOOTER_PITCH_MM
        else:
            fy += FOOTER_GAP_MM

    # The most important sentences on this sheet are the LAST ones. A footer that
    # overruns the bottom margin gets silently clipped by the printer's
    # unprintable edge, and the warning that goes missing is exactly the one
    # saying the whole design is unverified. Fail here instead.
    ink_bottom = last_baseline + 1.0            # descender allowance
    if ink_bottom > PAGE_H_MM - MARGIN_MM:
        raise SystemExit(
            f"ERROR: the footer's last line sits at {ink_bottom:.1f} mm, past the "
            f"{PAGE_H_MM - MARGIN_MM:.1f} mm bottom margin. It would be clipped by "
            "most printers - and the clipped text is the UNVERIFIED warning. "
            "Shorten FOOTER_LINES or tighten the layout."
        )

    head = (
        f'<svg xmlns="http://www.w3.org/2000/svg" version="1.1" '
        f'width="{t(PAGE_W_MM)}mm" height="{t(PAGE_H_MM)}mm" '
        f'viewBox="0 0 {t(PAGE_W_MM)} {t(PAGE_H_MM)}" shape-rendering="crispEdges">\n'
        f'<title>Emre Kalem arm - ChArUco {spec.dict_name} {spec.squares_x}x{spec.squares_y} '
        f'{spec.square_mm:g} mm intrinsics board (A4, print at 100%)</title>\n'
    )
    return head + "\n".join(body) + "\n</svg>\n"


def assert_quiet_ring(spec: BoardSpec, board_x: float, board_y: float,
                      rule_y: float, footer_y: float) -> None:
    """Nothing may be drawn inside QUIET_RING_MM of the board.

    The rule ticks and their numerals sit above the board; the footer line sits
    below it. Both are checked against their true lowest/highest ink, not
    against their baseline.
    """
    rule_bottom = rule_y + 6.8          # tick numerals baseline + descender
    board_top = board_y
    board_bottom = board_y + spec.height_mm
    gap_above = board_top - rule_bottom
    gap_below = footer_y - board_bottom
    gap_left = board_x - MARGIN_MM
    gap_right = (PAGE_W_MM - MARGIN_MM) - (board_x + spec.width_mm)

    for name, gap in (("above (print rule)", gap_above), ("below (footer)", gap_below),
                      ("left", gap_left), ("right", gap_right)):
        if gap < QUIET_RING_MM:
            raise SystemExit(
                f"ERROR: layout puts ink {gap:.1f} mm from the board {name}; "
                f"{QUIET_RING_MM} mm of clear white is required around a ChArUco "
                "board or the border markers cannot be segmented."
            )
    if board_bottom > PAGE_H_MM - MARGIN_MM:
        raise SystemExit("ERROR: board runs off the bottom of the page")


# --------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.abspath(os.path.join(here, "..", ".."))

    ap = argparse.ArgumentParser(
        description="Generate the printable ChArUco camera-intrinsics board.")
    ap.add_argument("--out", default=os.path.join(repo, "docs", "charuco_calibration_board.svg"))
    ap.add_argument("--square-mm", type=float, default=SQUARE_MM,
                    help="chessboard square edge in mm (design value 25.0)")
    ap.add_argument("--marker-ratio", type=float, default=MARKER_RATIO,
                    help="marker edge / square edge (design value 0.75)")
    ap.add_argument("--first-id", type=int, default=FIRST_MARKER_ID,
                    help="lowest ArUco id on the board (default 20, clear of the arm's 0-12)")
    args = ap.parse_args(argv)

    if args.first_id < 0:
        raise SystemExit("ERROR: --first-id must be >= 0")
    if not 0.4 <= args.marker_ratio <= 0.9:
        raise SystemExit("ERROR: --marker-ratio outside a sane 0.4..0.9")

    spec = BoardSpec(
        squares_x=SQUARES_X,
        squares_y=SQUARES_Y,
        square_mm=args.square_mm,
        marker_mm=args.square_mm * args.marker_ratio,
        first_id=args.first_id,
        dict_name=DICT_NAME,
    )

    if args.first_id <= 12:
        sys.stderr.write(
            f"WARNING: --first-id {args.first_id} overlaps the arm's sticker ids 0-12. "
            "A board marker will be indistinguishable from a link marker to anything "
            "downstream of the id whitelist.\n"
        )

    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICT_NAME))
    if dictionary.markerSize != MARKER_MODULES - 2:
        raise SystemExit(
            f"ERROR: {DICT_NAME} has markerSize {dictionary.markerSize}; this script "
            f"assumes {MARKER_MODULES - 2} data modules + 1 border ring."
        )

    print(f"dictionary           : {spec.dict_name}")
    print(f"board                : {spec.squares_x} x {spec.squares_y} squares, "
          f"{spec.square_mm:.3f} mm square, {spec.marker_mm:.3f} mm marker")
    print(f"marker ids           : {spec.first_id}-{spec.first_id + spec.marker_count - 1} "
          f"({spec.marker_count} markers)  [arm stickers are 0-12]")
    print(f"charuco corners      : {spec.corner_count}")
    print(f"physical size        : {spec.width_mm:.1f} x {spec.height_mm:.1f} mm")

    rects, grids = build_black_rects(spec, dictionary)      # GATE 1 inside
    print(f"gate 1 module align  : PASS ({len(grids)}/{spec.marker_count} markers)")

    cv_board = spec.cv_board(dictionary)
    raster = rasterise(spec, rects, SQUARE_PX)
    gate2_matches_opencv(spec, raster, cv_board)
    print(f"gate 2 pixel-identity: PASS ({raster.shape[1]}x{raster.shape[0]} px vs "
          "cv2 CharucoBoard.generateImage, 0 differing pixels)")

    n_c, n_m, max_err = gate3_redetect(spec, raster, cv_board)
    print(f"gate 3 re-detection  : PASS ({n_c}/{spec.corner_count} charuco corners, "
          f"{n_m}/{spec.marker_count} markers, worst corner {max_err:.4f} px "
          f"<= {GATE3_CORNER_TOL_PX} px)")

    # ---- layout, then assert the quiet ring before writing anything -------
    board_x = (PAGE_W_MM - spec.width_mm) / 2.0
    rule_y = 30.0
    board_y = 51.0
    footer_y = board_y + spec.height_mm + QUIET_RING_MM
    assert_quiet_ring(spec, board_x, board_y, rule_y, footer_y)

    svg = build_svg(spec, rects, board_x, board_y, rule_y, footer_y)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(svg)

    print(f"wrote                : {args.out}")
    print(f"bytes                : {os.path.getsize(args.out)}")
    print(f"black rects          : {len(rects)}")
    print("PRINT AT 100% / ACTUAL SIZE, ON MATTE STOCK, THEN MOUNT IT FLAT ON RIGID BOARD.")
    print("This sheet verifies nothing. The ~60 deg field of view stays an ASSUMPTION")
    print("until calibrate_intrinsics.py has passed its gate on real camera captures.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
