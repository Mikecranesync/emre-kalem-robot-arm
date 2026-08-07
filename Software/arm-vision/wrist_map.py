#!/usr/bin/env python3
"""Sweep the wrist through a grid and record what the gripper camera sees.

WHAT THIS BUILDS. A map, indexed by wrist orientation, of the arm's own point of
view: for every (J4 pitch, J5 roll) cell it stores the wrist frame, the side
frame, and the commanded angles. The contact sheet at the end is the map made
visible -- one thumbnail per cell, laid out as the grid actually is, so the
relationship between wrist angle and what the camera is looking at can be read
off a single picture instead of inferred.

WHY IT IS THE WRIST AND NOT THE WHOLE ARM. J4 and J5 are the two joints that
change where the camera POINTS without changing where the camera IS. Shoulder and
elbow move the camera through space, which mixes translation into every frame and
confounds the thing being mapped. Map orientation first; position is a separate
sweep.

THE SAFE ENVELOPE IS NARROWER THAN THE SOFT LIMITS, ON PURPOSE.
  J4 pitch: joint-limits.csv says 0-180, but that is the servo's WHOLE ELECTRICAL
    RANGE and a joint locked at exactly the placeholder width has almost
    certainly never been driven to a mechanical stop at either end. A real stop
    may sit well inside it. arm-bench-safety section 3 says at most +/-15 deg off
    90, so this tool clamps to 75-105 and will not be argued out of it.
  J5 roll: 31 is a real found end and 178 is 2 deg off the electrical ceiling, so
    the top is probably electrical rather than mechanical. This stays inside
    40-170 -- off both ends.
Anything outside those is refused before a byte is written, not clamped quietly.

WHAT THIS DATA IS AND IS NOT. It is a record of what the camera saw at each
COMMANDED angle. Nothing observes a shaft, so it is not a record of where the
wrist was. It is enough to learn the SIGN and SCALE of each joint in image space,
which is what image-based visual servoing needs; it is not a demonstration of a
task and it is not, on its own, imitation-learning data.

SAFETY. Nothing in software is an emergency stop here. STP holds with joints
driven; EST and the watchdog DETACH and a gravity-loaded arm falls. The rocker
and the fuse are the only real stop. This tool sends only MOV and STA, only on
J4 and J5, and stops on the first clamp, stall, latch or unreadable reply.

Usage:
  python wrist_map.py --link DIR --out DIR [--dps 10] [--j4 75,90,105] [--j5 40,70,100,130,160]
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "tests"))

import cameras                                                      # noqa: E402
from goto_pose import CleanArm                                      # noqa: E402
from reply_cut import clamped                                       # noqa: E402

# Hard envelope. See the docstring -- these are tighter than joint-limits.csv and
# that is the point. J4's locked 0-180 is the placeholder width, not measured
# travel; J5's 178 is very likely an electrical end.
SAFE = {4: (75, 105), 5: (40, 170)}
MOVABLE = (4, 5)
LATCH_MARKERS = ("LATCHED", "re-ENA")


def check_envelope(joint: int, angles: list[int]) -> None:
    lo, hi = SAFE[joint]
    bad = [a for a in angles if not (lo <= a <= hi)]
    if bad:
        raise SystemExit(
            f"refusing J{joint} angles {bad}: outside the SAFE envelope {lo}-{hi}. "
            f"joint-limits.csv may allow more, but J4's locked range is the whole "
            f"electrical range with its mechanical ends never found, and J5's top "
            f"end is probably electrical. Widen this only after driving the joint "
            f"to a real stop and recording it."
        )


def move(arm, j: int, a: int, dps: int, settle_extra: float = 1.5) -> str | None:
    """One waypoint. Returns the reply, or None if anything was wrong."""
    before = arm.status().get(j, {}).get("SET")
    travel = abs(a - int(before)) if before is not None else 90
    off = arm.offset()
    reply = arm.send(f"MOV {j} {a}")
    window = arm.tail(off)
    if "ERR" in reply:
        print(f"      J{j}->{a}: {reply} -- STOP")
        return None
    cl = clamped(reply)
    if cl is None:
        print(f"      J{j}->{a}: reply unreadable (no CL=) -- NOT a pass. STOP. {reply!r}")
        return None
    if cl:
        print(f"      J{j}->{a}: CLAMPED -- STOP. {reply}")
        return None
    if any(m in window for m in LATCH_MARKERS):
        print(f"      J{j}->{a}: latch in window -- STOP")
        return None
    ok, rows, waited = arm.settle({j: a}, timeout=max(4.0, travel / max(dps, 1) * 1.8 + settle_extra))
    if not ok:
        print(f"      J{j}->{a}: did not settle in {waited}s -- STOP")
        return None
    if rows.get(j, {}).get("JTO") not in ("0", None):
        print(f"      J{j}->{a}: JTO set -- joint stalled. STOP")
        return None
    return reply


def contact_sheet(cells, out_path, j4s, j5s, thumb=(256, 144)):
    """The map, made visible. Rows are J4 pitch, columns are J5 roll.

    Laid out as the grid IS rather than as a filmstrip, because the whole value
    of a map is that adjacency on the page means adjacency in the world.
    """
    pad, label_h, label_w = 4, 22, 54
    W = label_w + len(j5s) * (thumb[0] + pad)
    H = label_h + len(j4s) * (thumb[1] + pad + label_h)
    sheet = np.full((H, W, 3), 24, dtype=np.uint8)
    for ci, j5 in enumerate(j5s):
        x = label_w + ci * (thumb[0] + pad)
        cv2.putText(sheet, f"J5 {j5}", (x + 4, 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (200, 200, 200), 1, cv2.LINE_AA)
    for ri, j4 in enumerate(j4s):
        y = label_h + ri * (thumb[1] + pad + label_h)
        cv2.putText(sheet, f"J4", (4, y + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(sheet, f"{j4}", (4, y + 78), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (200, 200, 200), 1, cv2.LINE_AA)
        for ci, j5 in enumerate(j5s):
            x = label_w + ci * (thumb[0] + pad)
            img = cells.get((j4, j5))
            if img is None:
                continue
            t = cv2.resize(cv2.imread(img), thumb, interpolation=cv2.INTER_AREA)
            sheet[y:y + thumb[1], x:x + thumb[0]] = t
    cv2.imwrite(out_path, sheet)
    return out_path


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--link", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--dps", type=int, default=10)
    p.add_argument("--j4", default="75,82,90,98,105")
    p.add_argument("--j5", default="40,60,80,100,120,140,160")
    p.add_argument("--settle", type=float, default=0.6,
                   help="extra seconds after settle before the frame is taken")
    args = p.parse_args()

    j4s = [int(x) for x in args.j4.split(",")]
    j5s = [int(x) for x in args.j5.split(",")]
    check_envelope(4, j4s)
    check_envelope(5, j5s)

    arm = CleanArm(args.link)
    sta = arm.send("STA")
    if "ES=1" in sta or "WD=1" in sta:
        print("  arm is LATCHED -- refusing to move.")
        return 1
    rows = arm.status()
    for j in MOVABLE:
        if rows.get(j, {}).get("EN") != "1":
            print(f"  J{j} is not enabled (EN={rows.get(j, {}).get('EN')}) -- refusing.")
            return 1
    start = {j: int(rows[j]["SET"]) for j in MOVABLE}
    print(f"  start: J4={start[4]} J5={start[5]}   grid {len(j4s)}x{len(j5s)} "
          f"= {len(j4s) * len(j5s)} cells at {args.dps} deg/s")

    os.makedirs(args.out, exist_ok=True)
    frames = os.path.join(args.out, "frames")
    os.makedirs(frames, exist_ok=True)
    index = os.path.join(args.out, "wrist_map.jsonl")
    open(index, "w").close()

    caps = {}
    for role in ("side", "wrist"):
        cap, spec, idx = cameras.open_role(role)
        caps[role] = (cap, spec)

    for j in MOVABLE:
        arm.send(f"SPD {j} {args.dps}")

    cells, n, rc = {}, 0, 0
    t0 = time.monotonic()
    for ri, j4 in enumerate(j4s):
        if move(arm, 4, j4, args.dps) is None:
            rc = 1
            break
        # Serpentine: alternate the roll direction each row so the wrist never
        # traverses the whole range just to start the next line. Less travel,
        # less cable wind, and the cable crosses J5.
        order = j5s if ri % 2 == 0 else list(reversed(j5s))
        for j5 in order:
            if move(arm, 5, j5, args.dps) is None:
                rc = 1
                break
            time.sleep(args.settle)
            rec = {"j4": j4, "j5": j5, "i": n, "images": {}}
            for role, (cap, spec) in caps.items():
                ok, frame = cameras.read(cap, spec)
                if not ok:
                    rec["images"][role] = {"file": None, "error": "no frame"}
                    continue
                name = f"j4-{j4}_j5-{j5}_{role}.png"
                path = os.path.join(frames, name)
                cv2.imwrite(path, frame)
                g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                rec["images"][role] = {
                    "file": f"frames/{name}",
                    "luma": round(float(g.mean()), 2),
                    "sharpness": round(float(cv2.Laplacian(g, cv2.CV_64F).var()), 2)}
                if role == "wrist":
                    cells[(j4, j5)] = path
            with open(index, "a", encoding="utf-8") as fh:
                fh.write(json.dumps(rec) + "\n")
            w = rec["images"].get("wrist", {})
            print(f"    J4 {j4:3d}  J5 {j5:3d}   wrist luma {w.get('luma')} "
                  f"sharp {w.get('sharpness')}")
            n += 1
        if rc:
            break

    print(f"\n  returning the wrist to where it started: J4={start[4]} J5={start[5]}")
    move(arm, 5, start[5], args.dps)
    move(arm, 4, start[4], args.dps)

    for cap, _ in caps.values():
        cap.release()

    sheet = ""
    if cells:
        sheet = contact_sheet(cells, os.path.join(args.out, "wrist_map_sheet.png"),
                              j4s, j5s)
    print(f"  {n} cells in {round(time.monotonic() - t0)}s -> {index}")
    if sheet:
        print(f"  contact sheet -> {sheet}")
    print("  Commanded angles only -- nothing here observes a shaft.")
    return rc


if __name__ == "__main__":
    sys.exit(main())
