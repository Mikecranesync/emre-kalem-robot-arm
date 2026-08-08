#!/usr/bin/env python3
"""
regrade_markers.py - re-derive every marker detection grade from a REAL focal
length, and say exactly which committed numbers move.

WHY THIS EXISTS
    docs/marker_system.md section 5 says it plainly: "Every size and every grade
    in this design scales linearly with f_px." f_px is currently 1108, derived
    from an ASSUMED ~60 degree HFOV, recorded as UNVERIFIED open item A1.

    So the moment calibrate_intrinsics.py passes its gate on real captures, a
    pile of committed numbers become stale at once - and the stale ones look
    exactly like the fresh ones. This script is the arithmetic that closes that
    loop, so re-deriving is a command rather than a careful afternoon.

    Run it with no arguments and it shows the CURRENT state of the design under
    the assumed focal length. Run it with --intrinsics and it shows the same
    table under the observed focal length, side by side, with every grade change
    called out.

THE ONE FORMULA, FROM docs/marker_system.md SECTION 5
        projected_px = f_px * black_square_mm * cos_theta / standoff_mm
    with cos_theta = 0.7 (a design assumption about view angle, NOT something
    this script can settle) and the three-value grade vocabulary:
        pose         projected black square >= 36 px   stable 6-DOF pose
        detect       projected black square >= 24 px   detected, pose unstable
        below-floor  projected black square <  24 px   NOT DETECTED at all

    Which f? A square's horizontal extent scales with fx and its vertical extent
    with fy. The design collapsed both into one f_px because it assumed square
    pixels. A real calibration reports both. Default here is min(fx, fy), the
    conservative choice - the marker has to clear the floor on its WORSE axis to
    be honestly gradeable. --axis fx or --axis fy overrides.

WHAT THIS SCRIPT WILL NOT DO
    It does not write Software/vision/markers.csv. That file is the derivation
    of record and rewriting it from a script would destroy the audit trail its
    own preamble is built on. This prints what changes; a human edits the CSV
    and re-runs make_marker_sheet.py and the placement-table regeneration
    snippet in docs/marker_system.md.

    It also does not verify anything on its own. Without --intrinsics it is just
    re-stating the assumption.

WHICH COMMITTED COLUMNS ACTUALLY MOVE (and which do NOT)
    MOVE - these are downstream of f_px:
      markers.csv  grade_at_design_standoff   the grade itself
      markers.csv  camera                     CAM-A/CAM-B assignment follows
                                              from which grades are reachable
      markers.csv  design_standoff_mm         the two-camera split in section 5
                                              was forced by VFOV arithmetic
      markers.csv  visibility_notes           carries literal px figures in
                                              prose ("23.99 px", "18.6 px", ...)
      docs/marker_placement_table.csv         it is the first 8 columns of
                                              markers.csv, so visibility_notes
                                              changes propagate into it
      docs/marker_system.md  section 5 table, the 540 mm standoff derivation,
                             the SHOULDER-1 knife-edge, section 8 item (b)

    DO NOT MOVE - these come from STL geometry, not from the camera:
      markers.csv  black_square_mm            floor((inscribed - 2.0) / 1.3333)
      markers.csv  approximate_size_mm        black_square_mm * 8/6
      markers.csv  measured_inscribed_square_mm, patch_normal, patch_fill_ratio,
                   pair_baseline_mm
    A real focal length does NOT resize a single sticker. It re-grades them. The
    sizes are bounded by the plastic, and the plastic does not care about the
    lens. What a new f_px changes is whether those sizes are good ENOUGH.

USAGE
    python Software/arm-vision/regrade_markers.py
    python Software/arm-vision/regrade_markers.py --intrinsics Software/arm-vision/camera_intrinsics.json
    python Software/arm-vision/regrade_markers.py --intrinsics <f>.json --out /tmp/regrade.csv

DEPENDENCIES
    numpy is NOT required and cv2 is NOT required - this is arithmetic.
    stdlib only: argparse, csv, json, math, os, sys.

VOCABULARY
    A grade is a prediction, not an observation. It says what a camera SHOULD
    see. Nothing here observes the arm, and none of it gives a joint angle.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys

# From docs/marker_system.md section 5 and section 1.
ASSUMED_F_PX = 1280.0 / (2.0 * math.tan(math.radians(30.0)))   # ~1108, HFOV 60 deg
COS_THETA = 0.7
POSE_FLOOR_PX = 36.0
DETECT_FLOOR_PX = 24.0

# The design's framing requirement, section 5: about 350 mm of arm in view.
ARM_FRAME_HEIGHT_MM = 350.0
SENSOR_H_PX = 720.0
SENSOR_W_PX = 1280.0


def grade(px: float) -> str:
    """The grade vocabulary is exactly three values and nothing else."""
    if px >= POSE_FLOOR_PX:
        return "pose"
    if px >= DETECT_FLOOR_PX:
        return "detect"
    return "below-floor"


def projected_px(f_px: float, black_mm: float, standoff_mm: float,
                 cos_theta: float = COS_THETA) -> float:
    return f_px * black_mm * cos_theta / standoff_mm


def min_black_mm_for(px_floor: float, f_px: float, standoff_mm: float,
                     cos_theta: float = COS_THETA) -> float:
    """Invert the formula: smallest black square that reaches a grade floor."""
    return px_floor * standoff_mm / (f_px * cos_theta)


def crossing_standoff_mm(f_px: float, black_mm: float, px_floor: float,
                         cos_theta: float = COS_THETA) -> float:
    """The distance at which a given square crosses a grade floor."""
    return f_px * black_mm * cos_theta / px_floor


def framing_standoff_mm(f_px_vertical: float,
                        frame_mm: float = ARM_FRAME_HEIGHT_MM) -> float:
    """Standoff needed to fit frame_mm of arm vertically - section 5's '~540 mm'."""
    vfov = 2.0 * math.atan(SENSOR_H_PX / (2.0 * f_px_vertical))
    return (frame_mm / 2.0) / math.tan(vfov / 2.0)


def load_markers(path: str) -> list[dict]:
    if not os.path.isfile(path):
        raise SystemExit(f"ERROR: markers.csv not found at {path}")
    with open(path, "r", encoding="utf-8", newline="") as fh:
        rows = list(csv.reader(
            l for l in fh if not l.lstrip().startswith("#") and l.strip()))
    if not rows:
        raise SystemExit(f"ERROR: {path} contains no data rows")
    header = [c.strip() for c in rows[0]]
    need = ("marker_id", "human_label", "black_square_mm", "camera",
            "design_standoff_mm", "grade_at_design_standoff")
    missing = [c for c in need if c not in header]
    if missing:
        raise SystemExit(f"ERROR: {path} is missing required column(s): {missing}")
    out = []
    for lineno, row in enumerate(rows[1:], start=2):
        if len(row) != len(header):
            raise SystemExit(
                f"ERROR: {path} data row {lineno} has {len(row)} fields, header has "
                f"{len(header)}. Refusing to guess which column is which.")
        out.append(dict(zip(header, row)))
    return out


def load_intrinsics(path: str, axis: str) -> tuple[float, dict]:
    with open(path, "r", encoding="utf-8") as fh:
        d = json.load(fh)
    if d.get("schema") != "emre-arm/camera-intrinsics":
        raise SystemExit(
            f"ERROR: {path} is not an emre-arm/camera-intrinsics file "
            f"(schema={d.get('schema')!r})")
    if not d.get("gate", {}).get("passed"):
        raise SystemExit(
            f"ERROR: {path} does not record a PASSED gate. calibrate_intrinsics.py "
            "refuses to write a failing calibration, so this file was hand-made or "
            "edited. Refusing to re-derive the whole marker design from it.")
    fx = float(d["observed"]["fx_px"])
    fy = float(d["observed"]["fy_px"])
    f = {"min": min(fx, fy), "fx": fx, "fy": fy}[axis]
    return f, d


def main(argv: list[str] | None = None) -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.abspath(os.path.join(here, "..", ".."))

    ap = argparse.ArgumentParser(
        description="Re-derive marker detection grades from a real focal length.")
    ap.add_argument("--markers", default=os.path.join(repo, "Software", "vision", "markers.csv"))
    ap.add_argument("--intrinsics", default=None,
                    help="camera_intrinsics.json from calibrate_intrinsics.py. "
                         "Without it this only re-states the ASSUMED focal length.")
    ap.add_argument("--axis", choices=("min", "fx", "fy"), default="min",
                    help="which focal axis to grade against (default min = conservative)")
    ap.add_argument("--cos-theta", type=float, default=COS_THETA,
                    help="assumed cos of the view angle (design value 0.7)")
    ap.add_argument("--out", default=None,
                    help="optional CSV of the recomputed table. Nothing is written "
                         "unless you pass this. markers.csv is NEVER written.")
    args = ap.parse_args(argv)

    markers = load_markers(args.markers)

    f_new: float | None = None
    intr: dict | None = None
    if args.intrinsics:
        f_new, intr = load_intrinsics(args.intrinsics, args.axis)

    print(f"markers.csv          : {args.markers}  ({len(markers)} markers)")
    print(f"cos theta            : {args.cos_theta}  (design assumption, not settled here)")
    print(f"assumed f_px         : {ASSUMED_F_PX:.1f}  "
          f"(HFOV 60 deg over {SENSOR_W_PX:.0f} px - UNVERIFIED, open item A1)")

    if f_new is None:
        print()
        print("NO INTRINSICS SUPPLIED. Everything below is the design AS COMMITTED,")
        print("resting on the unverified 60 degree field of view. Nothing here is")
        print("evidence. To settle it:")
        print("  1. python Software/arm-vision/make_charuco_board.py")
        print("  2. print docs/charuco_calibration_board.svg at 100%, mount it flat")
        print("  3. python Software/arm-vision/calibrate_intrinsics.py \\")
        print("         --captures <dir> --capture-source camera --callipered-square-mm <mm>")
        print("  4. re-run this script with --intrinsics <the file that wrote>")
    else:
        src = intr["provenance"]["capture_source"]
        print(f"observed f_px        : {f_new:.1f}  (axis={args.axis}; "
              f"fx={intr['observed']['fx_px']:.1f} fy={intr['observed']['fy_px']:.1f})")
        print(f"observed HFOV        : {intr['observed']['hfov_deg']:.2f} deg  "
              f"(assumed 60.00 deg)")
        print(f"ratio observed/assumed: {f_new / ASSUMED_F_PX:.4f}  -> every projected "
              f"px figure scales by this")
        print(f"capture source       : {src}")
        if src != "camera":
            print("  *** capture_source is NOT 'camera'. These intrinsics did not come")
            print("      from a real lens and settle NOTHING about open item A1. ***")

    # ---- per-marker table -------------------------------------------------
    f_ref = f_new if f_new is not None else ASSUMED_F_PX
    print()
    hdr = (f"  {'id':>2} {'label':<11} {'blk':>5} {'cam':<6} {'stand':>6} "
           f"{'csv grade':<12} {'px@assumed':>11} {'grade':<12}")
    if f_new is not None:
        hdr += f" {'px@observed':>12} {'grade':<12} {'change':<8}"
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))

    changes: list[str] = []
    rows_out: list[dict] = []
    for m in markers:
        blk = float(m["black_square_mm"])
        stand = float(m["design_standoff_mm"])
        px_a = projected_px(ASSUMED_F_PX, blk, stand, args.cos_theta)
        g_a = grade(px_a)
        line = (f"  {m['marker_id']:>2} {m['human_label']:<11} {blk:>5.1f} "
                f"{m['camera']:<6} {stand:>6.0f} {m['grade_at_design_standoff']:<12} "
                f"{px_a:>11.2f} {g_a:<12}")
        row = {"marker_id": m["marker_id"], "human_label": m["human_label"],
               "black_square_mm": blk, "camera": m["camera"],
               "design_standoff_mm": stand,
               "grade_in_csv": m["grade_at_design_standoff"],
               "px_at_assumed_f": round(px_a, 2), "grade_at_assumed_f": g_a}
        if f_new is not None:
            px_n = projected_px(f_new, blk, stand, args.cos_theta)
            g_n = grade(px_n)
            ch = "" if g_n == g_a else "CHANGED"
            if g_n != g_a:
                changes.append(f"id {m['marker_id']} {m['human_label']}: "
                               f"{g_a} -> {g_n}  ({px_a:.2f} -> {px_n:.2f} px)")
            line += f" {px_n:>12.2f} {g_n:<12} {ch:<8}"
            row.update({"px_at_observed_f": round(px_n, 2),
                        "grade_at_observed_f": g_n, "grade_changed": bool(ch)})
        # The CSV's own grade should already agree with the assumed-f arithmetic.
        # Where it does not, the CSV is stale against its own stated formula.
        if m["grade_at_design_standoff"].strip() != g_a:
            changes.append(
                f"id {m['marker_id']} {m['human_label']}: markers.csv says "
                f"'{m['grade_at_design_standoff']}' but the assumed-f formula gives "
                f"'{g_a}' ({px_a:.2f} px) - the CSV is stale against its OWN formula")
        print(line)
        rows_out.append(row)

    # ---- the derived design numbers that also move ------------------------
    print()
    print("  DERIVED DESIGN NUMBERS")
    for label, f in (("assumed", ASSUMED_F_PX),) + \
                    ((("observed", f_new),) if f_new is not None else ()):
        fs = framing_standoff_mm(f)
        print(f"    [{label:>8}] f_px {f:>7.1f}  "
              f"HFOV {2 * math.degrees(math.atan(SENSOR_W_PX / (2 * f))):>5.2f} deg  "
              f"VFOV {2 * math.degrees(math.atan(SENSOR_H_PX / (2 * f))):>5.2f} deg")
        print(f"                 standoff to frame {ARM_FRAME_HEIGHT_MM:.0f} mm of arm: "
              f"{fs:>6.1f} mm   (section 5 says ~540 mm)")
        for stand in sorted({float(m["design_standoff_mm"]) for m in markers}):
            print(f"                 @ {stand:>5.0f} mm: smallest black square for "
                  f"pose {min_black_mm_for(POSE_FLOOR_PX, f, stand, args.cos_theta):>5.1f} mm, "
                  f"for detect {min_black_mm_for(DETECT_FLOOR_PX, f, stand, args.cos_theta):>5.1f} mm")
        tab = min_black_mm_for(POSE_FLOOR_PX, f, 550.0, args.cos_theta)
        print(f"                 section 8 item (b) flat tab: needs a {tab:.1f} mm black "
              f"square -> ~{tab * 8 / 6:.0f} mm sticker -> ~{math.ceil((tab * 8 / 6 + 6) / 5) * 5:.0f} mm tab")

    # ---- SHOULDER-1, the knife edge --------------------------------------
    sh = next((m for m in markers if m["human_label"].strip() == "SHOULDER-1"), None)
    if sh:
        blk = float(sh["black_square_mm"])
        stand = float(sh["design_standoff_mm"])
        print()
        print("  SHOULDER-1 KNIFE EDGE (section 5 records it as 0.03 px under the floor)")
        for label, f in (("assumed", ASSUMED_F_PX),) + \
                        ((("observed", f_new),) if f_new is not None else ()):
            px = projected_px(f, blk, stand, args.cos_theta)
            cross = crossing_standoff_mm(f, blk, DETECT_FLOOR_PX, args.cos_theta)
            print(f"    [{label:>8}] {px:>6.2f} px vs {DETECT_FLOOR_PX:.0f} px floor "
                  f"({px - DETECT_FLOOR_PX:+.2f} px), crosses at {cross:.0f} mm")

    # ---- what to edit -----------------------------------------------------
    print()
    if changes:
        print(f"  {len(changes)} THING(S) TO ACT ON")
        for c in changes:
            print(f"    - {c}")
    else:
        print("  No grade changes against markers.csv as committed.")

    print()
    print("  IF ANYTHING ABOVE CHANGED, EDIT THESE BY HAND - this script never writes them:")
    print("    Software/vision/markers.csv        grade_at_design_standoff, camera,")
    print("                                       design_standoff_mm, and the literal px")
    print("                                       figures inside visibility_notes")
    print("    docs/marker_placement_table.csv    REGENERATE - do not hand-edit; use the")
    print("                                       snippet in docs/marker_system.md")
    print("    docs/marker_system.md              section 5 table + standoff derivation,")
    print("                                       section 8 item (b), open item A1")
    print("    Documentation/MARKER-SYSTEM.md     section 4 grade table, assumption A1")
    print("    then re-run  python Software/arm-vision/make_marker_sheet.py")
    print()
    print("  NOT AFFECTED - these come from the STLs, not the lens:")
    print("    black_square_mm, approximate_size_mm, measured_inscribed_square_mm,")
    print("    patch_normal, patch_fill_ratio, pair_baseline_mm.")
    print("    A real focal length does not resize a sticker. It re-grades it.")

    if args.out:
        with open(args.out, "w", encoding="utf-8", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows_out[0].keys()), lineterminator="\n")
            w.writeheader()
            w.writerows(rows_out)
        print(f"\n  wrote {args.out}")

    if f_new is None:
        print()
        print("REMINDER: no camera has been calibrated. The ~60 deg field of view is")
        print("still an ASSUMPTION and every grade above is still UNVERIFIED.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
