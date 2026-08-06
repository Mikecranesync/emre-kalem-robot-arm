#!/usr/bin/env python3
"""
calibrate_intrinsics.py - turn captures of the ChArUco board into real camera
intrinsics, or REFUSE to.

WHY THIS EXISTS
    docs/marker_system.md section 5 states an HFOV of "about 60 degrees" and
    derives f_px = 1280 / (2*tan 30) ~ 1108 px from it. Section 8 records that
    as open item A1: UNVERIFIED. Every marker size in
    Software/vision/markers.csv and every detection grade in
    docs/marker_placement_table.csv scales linearly with that number, so until
    it is settled the whole sticker set is unproven.

    Both design documents said "run a ChArUco board" and neither shipped a way
    to do it. make_charuco_board.py emits the board; this script consumes
    captures of it.

THE POINT OF THE GATE - AND WHY RMS IS NOT IT
    A bad calibration does not look bad. Twelve views of a board held roughly
    flat-on to the camera will return an RMS reprojection error of a few tenths
    of a pixel and a focal length that is simply wrong, because a frontoparallel
    board cannot separate "the board is far away" from "the lens is long". The
    two trade off almost exactly, and RMS - which only asks whether the model
    reproduces the corners it was fitted to - is blind to it.

    So the headline criterion here is NOT RMS. It is the standard deviation of
    fx and fy that cv2.calibrateCameraExtended reports (G8). That number is the
    solver stating how well-determined the focal length actually is, which is
    precisely and only what open item A1 is asking about. RMS is kept as a
    secondary bound (G6/G7) to catch blur, motion and mis-detections.

    The pose-diversity criteria (G3, G4, G5) are not decoration either: they are
    the conditions that EARN a small sigma. They are checked separately from G8
    so that a FAIL names the capture you are missing rather than just saying
    "uncertain".

    THIS SCRIPT REFUSES. If any criterion fails it prints the full table and
    writes NOTHING. Emitting intrinsics with a warning attached is how an
    unverified number ends up cited as verified three files later - which is the
    exact failure this repository is already carrying in open item A1.

CAPTURE PROCEDURE (what the gate is actually asking for)
    1. Print docs/charuco_calibration_board.svg at 100% on matte stock. Verify
       the 100 mm rule. Calliper 4 squares.
    2. MOUNT IT FLAT on rigid board. A sheet held in the hand bows, and a bowed
       board is not a plane. Nothing downstream can detect that.
    3. Take >= 12 stills - not a video scrub, stills, so nothing is motion
       blurred. Keep the camera settings identical to the settings the arm will
       be observed with. Intrinsics are a property of the lens AND the mode:
       change the resolution, the zoom or the field-of-view setting and this
       calibration no longer applies.
    4. Fill roughly a third of the frame. This is CLOSER than the 550 mm CAM-A
       design standoff - at 550 mm the 125 mm board is a few hundred pixels and
       corner refinement has nothing to bite on. Intrinsics recovered close up
       apply at any distance; that is what makes them intrinsics.
    5. TILT. At least 6 views tilted 25 deg or more, tilted in several different
       directions, at least one beyond 35 deg. This is the single most common
       reason a calibration is quietly wrong.
    6. Vary the distance - the far views at least 1.6x the near ones.
    7. Push the board into all four corners of the image. Distortion is only
       observable where there is distortion, i.e. away from the centre.

THE OUTPUT FILE
    JSON, schema "emre-arm/camera-intrinsics" version 1. Keys:

      schema, schema_version        identify the format
      generated_by/_utc, versions   provenance of the run
      provenance.capture_source     "camera" or "synthetic" - MANDATORY, no
                                    default. A synthetic run is self-identifying
                                    forever; it can never be mistaken for a
                                    calibration of a real lens.
      provenance.capture_files      every file that contributed
      board                         dictionary, squares, nominal square_mm,
                                    callipered square_mm_observed, marker_mm,
                                    first_marker_id
      image_size                    [width, height] px - the intrinsics are only
                                    valid at this size
      camera_matrix                 3x3 row-major
      dist_coeffs                   as returned by OpenCV, in OpenCV's order
      observed                      fx_px, fy_px, cx_px, cy_px and the hfov/vfov
                                    they imply. Named "observed" because a
                                    camera actually saw the board - the one
                                    approved exception in this project's
                                    vocabulary rule.
      uncertainty                   sigma on each intrinsic, and sigma/value
      gate                          every criterion with its measured value,
                                    comparator and threshold. The file carries
                                    the evidence, not just the verdict.

    Scale note, stated because it is easy to get backwards: fx, fy, cx, cy and
    the distortion coefficients are INVARIANT to the board's true square size.
    Scaling every object point by k scales the recovered translations by k and
    leaves the camera matrix alone. A board printed at 96% therefore still gives
    the right focal length; what it corrupts is DISTANCE. The callipered square
    size is recorded because it is what makes recovered translations trustworthy,
    not because the focal length depends on it.

WHAT THIS SCRIPT DOES NOT DO
    It does not touch Software/vision/markers.csv. Re-deriving marker sizes and
    grades from a real focal length is regrade_markers.py, which reads that file
    and never writes it.

    It does not verify anything until it is run on real captures. Running
    --self-test proves the GATE works. It does not calibrate a camera and the
    file it writes says so in provenance.capture_source.

USAGE
    python Software/arm-vision/calibrate_intrinsics.py \
        --captures Calibration_Notes/charuco/ \
        --capture-source camera \
        --callipered-square-mm 24.9

    python Software/arm-vision/calibrate_intrinsics.py --self-test

DEPENDENCIES
    opencv-python  Apache-2.0. Detection and the calibration solve.
    numpy          BSD-3-Clause. FLAGGED, not silently accepted: the repository
                   rule is Apache-2.0 or MIT only. numpy is already an
                   unavoidable dependency of the vision and LeRobot work (cv2
                   requires it), so it is raised here for a decision rather than
                   passed over.
    stdlib         argparse, datetime, glob, json, math, os, sys, tempfile.

VOCABULARY
    This arm has no shaft feedback. Nothing in this file observes a joint, and
    intrinsics do not give one. "observed" here means a camera actually saw the
    board, or a human read a calliper. Joint state remains commanded.
"""

from __future__ import annotations

import argparse
import datetime
import glob
import json
import math
import os
import sys
import tempfile
from dataclasses import dataclass

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover - environment problem, not a code path
    sys.stderr.write(
        "ERROR: opencv-python is required.\n       pip install opencv-python\n"
    )
    raise


SCHEMA = "emre-arm/camera-intrinsics"
SCHEMA_VERSION = 1

# Board defaults - must match make_charuco_board.py. Passed explicitly rather
# than imported so this script still runs if it is copied out on its own, and so
# a mismatch shows up as a detection failure rather than a silent import.
DICT_NAME = "DICT_4X4_50"
SQUARES_X = 5
SQUARES_Y = 7
SQUARE_MM = 25.0
MARKER_MM = 18.75
FIRST_ID = 20

IMAGE_EXTS = ("*.png", "*.jpg", "*.jpeg", "*.bmp", "*.tif", "*.tiff",
              "*.PNG", "*.JPG", "*.JPEG")


# --------------------------------------------------------------------------
# Gate thresholds. These are CHOSEN values, not measured ones. Each carries the
# reasoning for its number so a future change is an argument, not a nudge.
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class Thresholds:
    # G1. Twelve is the conventional floor and gives 12 x 24 = 288 corner
    # observations against 9-ish unknowns. Fewer views makes sigma(fx) itself
    # unreliable, which would defeat the point of gating on it.
    min_views: int = 12

    # G2. Half the board. A view showing under half the corners is usually
    # clipped or badly lit; it contributes little and drags per-view RMS.
    min_corners_per_view: int = 12

    # G3. The conditioning criteria. Below ~20 deg the focal/distance trade-off
    # is barely broken; 25 deg is a comfortable margin that is still easy to
    # hold by hand. Requiring HALF the minimum view count to be tilted stops one
    # lucky oblique frame from carrying an otherwise flat set. The >= 35 deg
    # single view forces at least one genuinely strong perspective cue. The
    # azimuth spread stops every tilt being about the same axis, which leaves
    # the orthogonal focal axis just as poorly determined as before.
    min_tilt_deg: float = 25.0
    min_tilted_views: int = 6
    min_max_tilt_deg: float = 35.0
    min_tilt_azimuth_quadrants: int = 3

    # G4. Two distances that differ by less than ~1.6x are one distance as far
    # as the solver is concerned.
    min_z_ratio: float = 1.6

    # G5. Distortion is only observable off-centre, and cx/cy are only pinned
    # down if corners appear on both sides of the centre. 70% span plus a
    # presence floor in every quadrant is the cheap version of "cover the frame".
    min_span_frac: float = 0.70
    min_quadrant_frac: float = 0.05

    # G6/G7. Secondary bounds, catching blur, motion and mis-detection rather
    # than conditioning. 0.80 px is loose enough for a printed paper board on a
    # webcam and tight enough that a bowed board or a soft focus fails. The
    # per-view bound catches one bad frame that a mean would absorb.
    max_rms_px: float = 0.80
    max_per_view_rms_px: float = 1.50

    # G8. THE HEADLINE. 1% on the focal length. The design's whole exposure is
    # that grades scale linearly with f_px, so a 1% uncertainty moves a 24.0 px
    # grade by 0.24 px - smaller than the 0.03 px knife-edge the design already
    # documents for SHOULDER-1, and therefore not the thing that decides it.
    max_sigma_f_frac: float = 0.01

    # G9. Sanity. A non-square pixel aspect beyond 5% or a principal point more
    # than 15% off centre on a normal webcam means the solve wandered, whatever
    # the residual says.
    max_f_aspect_dev: float = 0.05
    max_pp_offset_frac: float = 0.15


@dataclass
class Criterion:
    name: str
    what: str
    measured: float
    comparator: str        # ">=" or "<="
    threshold: float

    @property
    def passed(self) -> bool:
        if self.comparator == ">=":
            return self.measured >= self.threshold
        if self.comparator == "<=":
            return self.measured <= self.threshold
        raise ValueError(self.comparator)

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "what": self.what,
            "measured": round(float(self.measured), 6),
            "comparator": self.comparator,
            "threshold": float(self.threshold),
            "passed": bool(self.passed),
        }


# --------------------------------------------------------------------------
# Board + detection
# --------------------------------------------------------------------------

def make_board(square_mm: float, marker_mm: float, first_id: int):
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICT_NAME))
    n_white = sum(1 for r in range(SQUARES_Y) for c in range(SQUARES_X) if (r + c) % 2 == 1)
    ids = np.arange(first_id, first_id + n_white, dtype=np.int32)
    if first_id + n_white > dictionary.bytesList.shape[0]:
        raise SystemExit(
            f"ERROR: --first-id {first_id} + {n_white} markers overruns {DICT_NAME}"
        )
    return cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), float(square_mm),
                                  float(marker_mm), dictionary, ids)


@dataclass
class View:
    path: str
    obj_pts: np.ndarray      # (N,1,3) float32, board mm
    img_pts: np.ndarray      # (N,1,2) float32, image px
    n_corners: int


def detect_views(paths: list[str], board, min_corners: int,
                 verbose: bool = True) -> tuple[list[View], tuple[int, int], list[str]]:
    detector = cv2.aruco.CharucoDetector(
        board, cv2.aruco.CharucoParameters(),
        cv2.aruco.DetectorParameters(), cv2.aruco.RefineParameters(),
    )
    views: list[View] = []
    rejected: list[str] = []
    size: tuple[int, int] | None = None

    for p in paths:
        img = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if img is None:
            rejected.append(f"{os.path.basename(p)}: unreadable")
            continue
        hw = (img.shape[1], img.shape[0])
        if size is None:
            size = hw
        elif hw != size:
            # Mixing resolutions silently averages two different lenses-as-configured.
            raise SystemExit(
                f"ERROR: {p} is {hw[0]}x{hw[1]} but earlier captures are "
                f"{size[0]}x{size[1]}. Intrinsics are only valid for one capture "
                "mode. Re-shoot the whole set at one resolution."
            )

        cc, ci, _mc, _mi = detector.detectBoard(img)
        n = 0 if cc is None else int(cc.shape[0])
        if n < min_corners:
            rejected.append(f"{os.path.basename(p)}: {n} corners (< {min_corners})")
            if verbose:
                print(f"  reject {os.path.basename(p):<28} {n:>3} corners")
            continue
        obj, imgp = board.matchImagePoints(cc, ci)
        if obj is None or len(obj) < min_corners:
            rejected.append(f"{os.path.basename(p)}: matchImagePoints returned too few")
            continue
        views.append(View(p, obj, imgp, n))
        if verbose:
            print(f"  accept {os.path.basename(p):<28} {n:>3} corners")

    if size is None:
        raise SystemExit("ERROR: no readable images found")
    return views, size, rejected


# --------------------------------------------------------------------------
# Pose diversity, computed from the calibration's own extrinsics
# --------------------------------------------------------------------------

def board_tilt_deg(rvec: np.ndarray) -> tuple[float, float]:
    """Return (tilt_deg, azimuth_deg) of the board plane in the camera frame.

    tilt is the angle between the board normal and the camera optical axis:
    0 deg is dead flat-on (frontoparallel), which is the degenerate case.
    azimuth is which way it is tilted, used to check the tilts are not all
    about one axis.
    """
    R, _ = cv2.Rodrigues(rvec)
    n = R @ np.array([0.0, 0.0, 1.0])
    tilt = math.degrees(math.acos(min(1.0, abs(float(n[2])))))
    azim = math.degrees(math.atan2(float(n[1]), float(n[0])))
    return tilt, azim


def coverage_stats(views: list[View], size: tuple[int, int]) -> tuple[float, float, float]:
    """(width span fraction, height span fraction, smallest quadrant fraction)."""
    pts = np.concatenate([v.img_pts.reshape(-1, 2) for v in views], axis=0)
    w, h = size
    span_w = (pts[:, 0].max() - pts[:, 0].min()) / w
    span_h = (pts[:, 1].max() - pts[:, 1].min()) / h
    cx, cy = w / 2.0, h / 2.0
    quads = [
        np.count_nonzero((pts[:, 0] < cx) & (pts[:, 1] < cy)),
        np.count_nonzero((pts[:, 0] >= cx) & (pts[:, 1] < cy)),
        np.count_nonzero((pts[:, 0] < cx) & (pts[:, 1] >= cy)),
        np.count_nonzero((pts[:, 0] >= cx) & (pts[:, 1] >= cy)),
    ]
    return float(span_w), float(span_h), float(min(quads) / len(pts))


# --------------------------------------------------------------------------
# Calibrate + gate
# --------------------------------------------------------------------------

def calibrate(views: list[View], size: tuple[int, int]):
    obj = [v.obj_pts for v in views]
    img = [v.img_pts for v in views]
    flags = 0
    (rms, K, dist, rvecs, tvecs,
     std_int, _std_ext, per_view) = cv2.calibrateCameraExtended(
        obj, img, size, None, None, flags=flags
    )
    return rms, K, dist, rvecs, tvecs, std_int.ravel(), per_view.ravel()


def build_criteria(th: Thresholds, views, size, rms, K, std_int, per_view,
                   rvecs, tvecs) -> list[Criterion]:
    w, h = size
    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    sfx, sfy = float(std_int[0]), float(std_int[1])

    tilts = [board_tilt_deg(r) for r in rvecs]
    tilt_deg = [t for t, _ in tilts]
    tilted = [(t, a) for t, a in tilts if t >= th.min_tilt_deg]
    quads = {int(((a + 360.0) % 360.0) // 90.0) for _, a in tilted}

    zs = [abs(float(t.ravel()[2])) for t in tvecs]
    z_ratio = (max(zs) / min(zs)) if zs and min(zs) > 0 else 0.0

    span_w, span_h, min_quad = coverage_stats(views, size)

    return [
        Criterion("G1_view_count", "accepted views",
                  len(views), ">=", th.min_views),
        Criterion("G2_min_corners_in_a_view", "fewest ChArUco corners in any accepted view",
                  min(v.n_corners for v in views), ">=", th.min_corners_per_view),
        Criterion("G3a_tilted_views", f"views tilted >= {th.min_tilt_deg:g} deg",
                  len(tilted), ">=", th.min_tilted_views),
        Criterion("G3b_max_tilt_deg", "strongest board tilt (deg)",
                  max(tilt_deg) if tilt_deg else 0.0, ">=", th.min_max_tilt_deg),
        Criterion("G3c_tilt_azimuth_quadrants", "distinct tilt directions (of 4 quadrants)",
                  len(quads), ">=", th.min_tilt_azimuth_quadrants),
        Criterion("G4_distance_ratio", "farthest board distance / nearest",
                  z_ratio, ">=", th.min_z_ratio),
        Criterion("G5a_span_width", "corner spread across image width (fraction)",
                  span_w, ">=", th.min_span_frac),
        Criterion("G5b_span_height", "corner spread across image height (fraction)",
                  span_h, ">=", th.min_span_frac),
        Criterion("G5c_emptiest_quadrant", "corners in the emptiest image quadrant (fraction)",
                  min_quad, ">=", th.min_quadrant_frac),
        Criterion("G6_rms_px", "overall RMS reprojection error (px)",
                  rms, "<=", th.max_rms_px),
        Criterion("G7_worst_view_rms_px", "worst single-view RMS reprojection error (px)",
                  float(per_view.max()), "<=", th.max_per_view_rms_px),
        Criterion("G8a_sigma_fx_frac", "uncertainty on fx (sigma/fx) - THE HEADLINE",
                  sfx / fx if fx else 1.0, "<=", th.max_sigma_f_frac),
        Criterion("G8b_sigma_fy_frac", "uncertainty on fy (sigma/fy) - THE HEADLINE",
                  sfy / fy if fy else 1.0, "<=", th.max_sigma_f_frac),
        Criterion("G9a_focal_aspect_dev", "|fx/fy - 1|",
                  abs(fx / fy - 1.0) if fy else 1.0, "<=", th.max_f_aspect_dev),
        Criterion("G9b_principal_point_offset", "principal point offset from image centre (fraction)",
                  max(abs(cx - w / 2.0) / w, abs(cy - h / 2.0) / h), "<=", th.max_pp_offset_frac),
    ]


def print_criteria(crits: list[Criterion]) -> None:
    print()
    print(f"  {'criterion':<28} {'measured':>12}  {'':2} {'threshold':>10}   verdict")
    print(f"  {'-' * 28} {'-' * 12}  {'-' * 2} {'-' * 10}   {'-' * 7}")
    for c in crits:
        print(f"  {c.name:<28} {c.measured:>12.4f}  {c.comparator:>2} "
              f"{c.threshold:>10.4f}   {'PASS' if c.passed else 'FAIL'}")
    print()
    for c in crits:
        if not c.passed:
            print(f"  FAILED {c.name}: {c.what}")


def hfov_vfov(K, size) -> tuple[float, float]:
    w, h = size
    return (2 * math.degrees(math.atan(w / (2 * float(K[0, 0])))),
            2 * math.degrees(math.atan(h / (2 * float(K[1, 1])))))


# --------------------------------------------------------------------------
# Run
# --------------------------------------------------------------------------

def run(paths: list[str], *, capture_source: str, out_path: str | None,
        square_mm: float, marker_mm: float, first_id: int,
        callipered_square_mm: float | None, th: Thresholds,
        note: str = "", verbose: bool = True) -> tuple[bool, dict | None]:
    """Returns (gate_passed, payload). Writes out_path ONLY if the gate passed."""
    board = make_board(square_mm, marker_mm, first_id)

    if verbose:
        print(f"captures supplied    : {len(paths)}")
    views, size, rejected = detect_views(paths, board, th.min_corners_per_view, verbose)
    if verbose:
        print(f"views accepted       : {len(views)} / {len(paths)}")
        print(f"image size           : {size[0]} x {size[1]}")

    if len(views) < 4:
        print(f"\nGATE: FAIL - only {len(views)} usable view(s); "
              "cv2.calibrateCamera cannot be run at all.")
        print("NO INTRINSICS WRITTEN.")
        return False, None

    rms, K, dist, rvecs, tvecs, std_int, per_view = calibrate(views, size)
    crits = build_criteria(th, views, size, rms, K, std_int, per_view, rvecs, tvecs)
    passed = all(c.passed for c in crits)

    if verbose:
        fx, fy = float(K[0, 0]), float(K[1, 1])
        hf, vf = hfov_vfov(K, size)
        print(f"\nobserved fx / fy     : {fx:.2f} / {fy:.2f} px "
              f"(sigma {std_int[0]:.2f} / {std_int[1]:.2f})")
        print(f"observed cx / cy     : {float(K[0, 2]):.2f} / {float(K[1, 2]):.2f} px")
        print(f"observed HFOV / VFOV : {hf:.2f} / {vf:.2f} deg")
        print(f"RMS reprojection     : {rms:.4f} px")
        print_criteria(crits)

    if not passed:
        print("GATE: FAIL")
        print("NO INTRINSICS WRITTEN. Add the captures the failed criteria name and re-run.")
        print("Refusing to emit a focal length that the capture set does not determine -")
        print("an unverified number that looks verified is exactly open item A1.")
        return False, None

    hf, vf = hfov_vfov(K, size)
    payload = {
        "schema": SCHEMA,
        "schema_version": SCHEMA_VERSION,
        "generated_by": "Software/arm-vision/calibrate_intrinsics.py",
        "generated_utc": datetime.datetime.now(datetime.timezone.utc)
                                  .replace(microsecond=0).isoformat(),
        "opencv_version": cv2.__version__,
        "numpy_version": np.__version__,
        "provenance": {
            "capture_source": capture_source,
            "note": note,
            "n_captures_supplied": len(paths),
            "n_views_accepted": len(views),
            "rejected": rejected,
            "capture_files": [os.path.basename(v.path) for v in views],
        },
        "board": {
            "dictionary": DICT_NAME,
            "squares_x": SQUARES_X,
            "squares_y": SQUARES_Y,
            "square_mm_nominal": square_mm,
            "square_mm_observed": callipered_square_mm,
            "marker_mm": marker_mm,
            "first_marker_id": first_id,
            "charuco_corner_count": (SQUARES_X - 1) * (SQUARES_Y - 1),
        },
        "image_size": [size[0], size[1]],
        "camera_matrix": [[float(x) for x in row] for row in K],
        "dist_coeffs": [float(x) for x in np.asarray(dist).ravel()],
        "dist_model": "OpenCV rational-free default: k1 k2 p1 p2 k3",
        "observed": {
            "fx_px": float(K[0, 0]), "fy_px": float(K[1, 1]),
            "cx_px": float(K[0, 2]), "cy_px": float(K[1, 2]),
            "hfov_deg": hf, "vfov_deg": vf,
            "rms_reprojection_px": float(rms),
        },
        "uncertainty": {
            "sigma_fx_px": float(std_int[0]), "sigma_fy_px": float(std_int[1]),
            "sigma_cx_px": float(std_int[2]), "sigma_cy_px": float(std_int[3]),
            "sigma_fx_frac": float(std_int[0]) / float(K[0, 0]),
            "sigma_fy_frac": float(std_int[1]) / float(K[1, 1]),
        },
        "gate": {"passed": True, "criteria": [c.as_dict() for c in crits]},
        "scale_note": (
            "fx, fy, cx, cy and dist_coeffs are INVARIANT to the board's true square "
            "size; only recovered translations scale with it. square_mm_observed is "
            "recorded so distances are trustworthy, not because the focal length "
            "depends on it."
        ),
        "vocabulary_note": (
            "'observed' means a camera saw the board or a human read a calliper. This "
            "arm has no shaft feedback; joint state remains commanded."
        ),
    }

    if out_path:
        os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
        with open(out_path, "w", encoding="utf-8", newline="\n") as fh:
            json.dump(payload, fh, indent=2)
            fh.write("\n")
        print(f"GATE: PASS -> wrote {out_path}")
    else:
        print("GATE: PASS (no --out given, nothing written)")
    return True, payload


# --------------------------------------------------------------------------
# Self-test: proves the GATE works, with no camera. Synthetic, and says so.
# --------------------------------------------------------------------------

def _synth_views(board, out_dir: str, K_true: np.ndarray, size: tuple[int, int],
                 poses: list[tuple[float, float, float, float, float, float]],
                 tag: str) -> list[str]:
    """Render the board through a known pinhole camera at the given poses.

    poses are (rot_x_deg, rot_y_deg, rot_z_deg, u, v, z_mm) where u and v are in
    [-1, 1] and say where in the frame to put the board: 0 is centred, +-1 is as
    far into that edge as the board's own projected extent allows. The sheet is
    modelled centred on its own origin so a rotation spins it about its middle
    rather than swinging it out of frame about a corner.

    No lens distortion is simulated - the aim is to exercise the GATE's
    conditioning logic, not to model a real lens.
    """
    sq_px = 60
    margin_px = 30
    ppm = sq_px / 25.0
    sheet = board.generateImage(
        (SQUARES_X * sq_px + 2 * margin_px, SQUARES_Y * sq_px + 2 * margin_px),
        marginSize=margin_px)
    sh, sw = sheet.shape
    margin_mm = margin_px / ppm
    W = SQUARES_X * 25.0 + 2 * margin_mm
    H = SQUARES_Y * 25.0 + 2 * margin_mm

    src = np.float32([[0, 0], [sw, 0], [sw, sh], [0, sh]])
    obj = np.float32([[-W / 2, -H / 2, 0], [W / 2, -H / 2, 0],
                      [W / 2, H / 2, 0], [-W / 2, H / 2, 0]])
    fx, fy = float(K_true[0, 0]), float(K_true[1, 1])
    img_w, img_h = size

    written: list[str] = []
    for i, (rx, ry, rz, u, v, tz) in enumerate(poses):
        Rx, _ = cv2.Rodrigues(np.array([math.radians(rx), 0.0, 0.0]))
        Ry, _ = cv2.Rodrigues(np.array([0.0, math.radians(ry), 0.0]))
        Rz, _ = cv2.Rodrigues(np.array([0.0, 0.0, math.radians(rz)]))
        rvec, _ = cv2.Rodrigues(Rz @ Ry @ Rx)

        # Pass 1: project centred on the optical axis to learn the true
        # projected extent of THIS pose (tilt shrinks it, in-plane spin grows it).
        tvec = np.array([[0.0], [0.0], [tz]], np.float64)
        proj = cv2.projectPoints(obj, rvec, tvec, K_true, np.zeros(5))[0].reshape(-1, 2)
        lo, hi = proj.min(axis=0), proj.max(axis=0)
        room_x = max(0.0, (img_w - (hi[0] - lo[0])) / 2.0 - 4.0)
        room_y = max(0.0, (img_h - (hi[1] - lo[1])) / 2.0 - 4.0)

        # Pass 2: shift so the projected centre lands where u, v ask for.
        cur = (lo + hi) / 2.0
        want = np.array([img_w / 2.0 + u * room_x, img_h / 2.0 + v * room_y])
        d = want - cur
        tvec = np.array([[d[0] * tz / fx], [d[1] * tz / fy], [tz]], np.float64)
        dst = cv2.projectPoints(obj, rvec, tvec, K_true,
                                np.zeros(5))[0].reshape(-1, 2).astype(np.float32)

        Hm = cv2.getPerspectiveTransform(src, dst)
        img = cv2.warpPerspective(sheet, Hm, size, flags=cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_CONSTANT, borderValue=255)
        p = os.path.join(out_dir, f"{tag}_{i:02d}.png")
        cv2.imwrite(p, img)
        written.append(p)
    return written


# Where in the frame each synthetic view puts the board. Corners first so both
# scenarios cover all four image quadrants - that keeps the ONLY difference
# between them the thing being demonstrated: tilt.
_PLACEMENTS = [(-1, -1), (1, -1), (-1, 1), (1, 1), (0, 0), (-1, 0),
               (1, 0), (0, -1), (0, 1), (-0.6, 0.6), (0.6, -0.6), (0, 0)]
_DISTANCES = [330.0, 380.0, 430.0, 480.0, 540.0]


def self_test(th: Thresholds) -> int:
    print("=" * 78)
    print("SELF-TEST - SYNTHETIC. THIS IS NOT A CAMERA CALIBRATION.")
    print("Images are rendered from a known pinhole matrix. The point is to prove")
    print("the GATE refuses a badly-conditioned capture set and accepts a good one.")
    print("=" * 78)

    board = make_board(SQUARE_MM, MARKER_MM, FIRST_ID)
    size = (1280, 720)
    fx_true = 900.0            # deliberately NOT the assumed 1108
    K_true = np.array([[fx_true, 0, 640.0], [0, fx_true, 360.0], [0, 0, 1.0]])
    print(f"\nsynthetic truth      : fx = fy = {fx_true:.1f} px, "
          f"cx/cy = 640/360, {size[0]}x{size[1]}, no distortion")
    print(f"                       (the design ASSUMES 1108 px - so this synthetic")
    print(f"                        camera is 23% away from the assumption on purpose)")

    tmp = tempfile.mkdtemp(prefix="charuco_selftest_")
    print(f"scratch dir          : {tmp}")

    ok = True

    # ---- Scenario A: 12 near-frontoparallel views. RMS will look fine. ----
    print("\n" + "-" * 78)
    print("SCENARIO A - 12 views, well spread over the frame and over distance,")
    print("             but all held nearly FLAT-ON. Must FAIL.")
    print("             This is the trap: RMS will look excellent and the focal")
    print("             length will still be undetermined.")
    print("-" * 78)
    poses_a = [(3.0 * math.sin(i), 3.0 * math.cos(i), 4.0 * i,
                u, v, _DISTANCES[i % len(_DISTANCES)])
               for i, (u, v) in enumerate(_PLACEMENTS)]
    paths_a = _synth_views(board, tmp, K_true, size, poses_a, "flat")
    passed_a, _ = run(paths_a, capture_source="synthetic", out_path=None,
                      square_mm=SQUARE_MM, marker_mm=MARKER_MM, first_id=FIRST_ID,
                      callipered_square_mm=None, th=th, verbose=True,
                      note="self-test scenario A")
    if passed_a:
        print("SELF-TEST BROKEN: scenario A was supposed to be refused.")
        ok = False
    else:
        print("scenario A refused as designed.")

    # ---- Scenario B: 16 diverse views. Must PASS. ----
    print("\n" + "-" * 78)
    print("SCENARIO B - 16 views, tilted in varied directions over varied distance. Must PASS.")
    print("-" * 78)
    tilt_sets = [(33, 0), (-33, 0), (0, 33), (0, -33), (27, 27), (-27, 27),
                 (27, -27), (-27, -27), (38, 12), (-38, -12), (12, 38), (-12, -38),
                 (22, -30), (-22, 30), (30, 18), (-30, -18)]
    places = _PLACEMENTS + [(-0.8, -0.4), (0.8, 0.4), (-0.4, 0.8), (0.4, -0.8)]
    poses_b = [(rx, ry, (i * 17) % 40 - 20, places[i][0], places[i][1],
                _DISTANCES[i % len(_DISTANCES)])
               for i, (rx, ry) in enumerate(tilt_sets)]
    out_json = os.path.join(tmp, "camera_intrinsics_SYNTHETIC.json")
    paths_b = _synth_views(board, tmp, K_true, size, poses_b, "tilt")
    passed_b, payload = run(paths_b, capture_source="synthetic", out_path=out_json,
                            square_mm=SQUARE_MM, marker_mm=MARKER_MM, first_id=FIRST_ID,
                            callipered_square_mm=None, th=th, verbose=True,
                            note="self-test scenario B - SYNTHETIC, not a real camera")
    if not passed_b:
        print("SELF-TEST BROKEN: scenario B was supposed to pass.")
        ok = False
    else:
        fx = payload["observed"]["fx_px"]
        print(f"scenario B accepted. recovered fx {fx:.2f} vs synthetic truth "
              f"{fx_true:.2f}  ->  {abs(fx - fx_true) / fx_true * 100:.3f}% error")
        print(f"provenance.capture_source in the written file: "
              f"{payload['provenance']['capture_source']!r}")

    print("\n" + "=" * 78)
    print("SELF-TEST RESULT:", "OK - the gate refuses A and accepts B." if ok else "BROKEN")
    print("This proved the GATE. It did NOT calibrate a camera, and the ~60 deg")
    print("field-of-view assumption behind every marker size is still UNVERIFIED.")
    print("=" * 78)
    return 0 if ok else 1


# --------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    here = os.path.dirname(os.path.abspath(__file__))

    ap = argparse.ArgumentParser(
        description="Recover camera intrinsics from ChArUco captures, or refuse to.")
    ap.add_argument("--captures", help="directory of capture images")
    ap.add_argument("--capture-source", choices=("camera", "synthetic"),
                    help="MANDATORY for a real run. Stamped into the output so a "
                         "synthetic run can never be mistaken for a real one.")
    ap.add_argument("--out", default=os.path.join(here, "camera_intrinsics.json"))
    ap.add_argument("--callipered-square-mm", type=float, default=None,
                    help="the square edge a human callipered on the print - an "
                         "OBSERVED value, and it wins over the nominal 25.000 mm")
    ap.add_argument("--square-mm", type=float, default=SQUARE_MM)
    ap.add_argument("--marker-mm", type=float, default=MARKER_MM)
    ap.add_argument("--first-id", type=int, default=FIRST_ID)
    ap.add_argument("--note", default="")
    ap.add_argument("--self-test", action="store_true",
                    help="prove the gate works using synthetic captures; writes "
                         "nothing outside a scratch dir and calibrates no camera")
    args = ap.parse_args(argv)

    th = Thresholds()

    if args.self_test:
        return self_test(th)

    if not args.captures:
        ap.error("--captures is required (or use --self-test)")
    if not args.capture_source:
        ap.error("--capture-source is required: 'camera' or 'synthetic'. "
                 "There is no default on purpose.")
    if not os.path.isdir(args.captures):
        raise SystemExit(f"ERROR: --captures {args.captures} is not a directory")

    paths: list[str] = []
    for ext in IMAGE_EXTS:
        paths.extend(glob.glob(os.path.join(args.captures, ext)))
    paths = sorted(set(paths))
    if not paths:
        raise SystemExit(f"ERROR: no images found in {args.captures}")

    if args.capture_source == "camera" and args.callipered_square_mm is None:
        sys.stderr.write(
            "WARNING: --callipered-square-mm not given. The focal length does not "
            "depend on it, but every distance this calibration is later used to "
            "recover does. Calliper 4 squares (nominally 100.0 mm) and re-run.\n"
        )

    passed, _ = run(paths, capture_source=args.capture_source, out_path=args.out,
                    square_mm=args.square_mm, marker_mm=args.marker_mm,
                    first_id=args.first_id,
                    callipered_square_mm=args.callipered_square_mm,
                    th=th, note=args.note, verbose=True)
    if passed:
        print("\nNEXT: re-derive the marker sizes and grades that were built on the")
        print("      assumed focal length:")
        print(f"      python Software/arm-vision/regrade_markers.py --intrinsics {args.out}")
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
