#!/usr/bin/env python3
"""Drive a pose tour and save a labelled camera frame at every pose. RUNS ON THE PI.

    arm_pose_proof.py <bridge-token> <shoulder_deg> [--outdir DIR]

WHY THIS EXISTS
    "It moved" is not evidence. This walks the arm through a fixed tour and
    writes one annotated JPEG per pose - commanded angles, changed pixels
    against the previous pose, and the joint that moved burned into the image -
    plus a contact sheet of the whole tour. That is a proof-of-work artifact
    someone can look at, rather than a log someone has to trust.

ONE PROCESS, ONE CAMERA READER
    The frames are grabbed inside this process, through the same Link that is
    driving the arm. A second client on the preview's /stream makes it hand back
    DUPLICATE frames - that is what aborted endurance run 2 on 2026-08-09, where
    a stalled camera read as five joints failing at once. Do not run a browser on
    the stream while this is running.

WHAT THE TOUR RESPECTS
    - Shoulder first, at an operator-supplied adopt angle (rule 3). Never inferred.
    - Every target inside the bands this arm has actually traversed (SAFE below),
      which are the same bands arm_endurance.py uses.
    - J3 steps are 10 deg, never smaller. The elbow has a measured 8 deg dead band
      in the UP direction - see Documentation/2026-08-10-ELBOW-DIRECTIONAL-BACKLASH.md
      - so a 4 deg elbow command is invisible and would photograph as a failure
      that is really a gearbox property.
    - Speeds forced down, one joint moved at a time, always finishes with DIS A.

J0 and J6 are excluded: the operator reported them disengaged for this session.
"""

from __future__ import annotations

import os
import sys
import time
import urllib.request

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_bench_test import Link, Bench, frame, diff, log  # noqa: E402

CAM_FULL = "http://127.0.0.1:8781/snapshot"
SPD = 5

# Bands this arm has physically traversed. Same source as arm_endurance.SAFE.
SAFE = {1: (78, 89), 3: (5, 25), 4: (85, 105), 5: (165, 178)}
HOME = {1: 85, 3: 15, 4: 95, 5: 172}

#: (label, joint, target). Each entry is one photographed pose. The elbow legs
#: are 10 deg because anything smaller is inside its dead band going up.
TOUR = [
    ("01-home",             None, None),
    ("02-shoulder-low",     1, 78),
    ("03-shoulder-high",    1, 89),
    ("04-shoulder-home",    1, 85),
    ("05-elbow-up",         3, 25),
    ("06-elbow-down",       3, 5),
    ("07-elbow-home",       3, 15),
    ("08-wrist-pitch-up",   4, 105),
    ("09-wrist-pitch-down", 4, 85),
    ("10-wrist-roll-max",   5, 178),
    ("11-wrist-roll-min",   5, 165),
    ("12-home-return",      None, None),
]


def full_frame() -> np.ndarray:
    raw = urllib.request.urlopen(CAM_FULL, timeout=15).read()
    return cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)


def annotate(img, label, angles, moved_px, joint, target):
    h, w = img.shape[:2]
    cv2.rectangle(img, (0, h - 96), (w, h), (0, 0, 0), -1)
    line1 = f"{label}   " + "  ".join(f"J{j}={angles.get(j, '?')}" for j in sorted(HOME))
    what = "settled at start pose" if joint is None else f"moved J{joint} -> {target} deg"
    line2 = f"{what}   changed {moved_px} px vs previous pose   {time.strftime('%H:%M:%S')}"
    cv2.putText(img, line1, (14, h - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 255, 0), 2)
    cv2.putText(img, line2, (14, h - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 220, 255), 2)
    return img


def contact_sheet(paths, out, cols=3):
    tiles = []
    for p in paths:
        im = cv2.imread(p)
        if im is not None:
            tiles.append(cv2.resize(im, (640, 360)))
    if not tiles:
        return None
    rows = []
    for i in range(0, len(tiles), cols):
        row = tiles[i:i + cols]
        while len(row) < cols:
            row.append(np.zeros_like(tiles[0]))
        rows.append(np.hstack(row))
    cv2.imwrite(out, np.vstack(rows))
    return out


def main():
    token = sys.argv[1]
    shoulder = int(sys.argv[2])
    outdir = "/home/armnode/pose-proof/" + time.strftime("%Y%m%d-%H%M%S")
    if "--outdir" in sys.argv:
        outdir = sys.argv[sys.argv.index("--outdir") + 1]
    os.makedirs(outdir, exist_ok=True)

    L = Link(token)
    B = Bench(L)
    paths, angles = [], dict(HOME)

    try:
        B.clr()
        if not B.wait_quiet("before adopt"):
            raise SystemExit("scene never settled before start")
        B.capture_shoulder(shoulder)
        if L.sta().get(1, {}).get("EN") != "1":
            raise SystemExit("shoulder would not enable")

        # Shoulder adopts at its rest angle, which is outside the SAFE band.
        # Walk it into the band before anything else moves.
        B.wait_quiet("before entering the safe band")
        fl = B.floor() or 200
        L.send(f"SPD 1 {SPD}")
        L.send(f"MOV 1 {HOME[1]}")
        for _ in range(6):
            L.idle(1.0)
        angles[1] = HOME[1]

        for j in (3, 4, 5):
            s = B.clr()
            B.wait_quiet(f"before enabling J{j}")
            L.send(f"ENA {j} {int(s[j]['SET'])}")
            L.idle(2.0)
            L.send(f"SPD {j} {SPD}")
            L.send(f"MOV {j} {HOME[j]}")
            for _ in range(5):
                L.idle(1.0)
            angles[j] = HOME[j]

        prev = frame()
        for label, j, tgt in TOUR:
            if j is not None:
                lo, hi = SAFE[j]
                tgt = max(lo, min(hi, tgt))
                L.send(f"SPD {j} {SPD}")
                L.send(f"MOV {j} {tgt}")
                for _ in range(6):
                    L.idle(1.0)
                angles[j] = tgt
            B.wait_quiet(f"settle at {label}")
            now = frame()
            px = diff(prev, now)
            prev = now
            p = os.path.join(outdir, f"{label}.jpg")
            cv2.imwrite(p, annotate(full_frame(), label, angles, px, j, tgt))
            paths.append(p)
            log(step="pose", label=label, joint=j, target=tgt,
                angles={str(k): v for k, v in angles.items()},
                changed_px=px, floor_px=fl,
                moved=(j is None) or (px > max(fl * 3, 300)), image=p)

        sheet = contact_sheet(paths, os.path.join(outdir, "00-contact-sheet.jpg"))
        log(step="tour_done", poses=len(paths), outdir=outdir, contact_sheet=sheet)
    finally:
        log(step="off", result=" | ".join(L.send("DIS A")))


if __name__ == "__main__":
    main()
