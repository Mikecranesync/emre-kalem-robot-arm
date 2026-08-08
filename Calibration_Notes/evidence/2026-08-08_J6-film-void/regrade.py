"""Offline re-grade of an already-captured frame series.

Same rules as grade.py / arm-motion-verify: threshold 25, no morphological
opening, md5s must differ (identical bytes = dead capture, not a steady arm),
control pair sets the floor, 4x bar (floor minimums 400 full / 150 ROI).

usage: regrade.py <dir> <tag>
"""

import glob
import hashlib
import json
import os
import sys

import cv2

ROI = (700, 350, 900, 480)  # gripper / claw region in the current framing


def md5(path):
    return hashlib.md5(open(path, "rb").read()).hexdigest()[:8]


def diff(pa, pb):
    a = cv2.cvtColor(cv2.imread(pa), cv2.COLOR_BGR2GRAY)
    b = cv2.cvtColor(cv2.imread(pb), cv2.COLOR_BGR2GRAY)
    full = int((cv2.absdiff(a, b) > 25).sum())
    x0, y0, x1, y1 = ROI
    roi = int((cv2.absdiff(a[y0:y1, x0:x1], b[y0:y1, x0:x1]) > 25).sum())
    return full, roi


def main(d, tag):
    floor = json.load(open(os.path.join(d, "control.json")))
    tf, tr = max(floor["full"] * 4, 400), max(floor["roi"] * 4, 150)
    frames = sorted(glob.glob(os.path.join(d, f"{tag}_*.jpg")))
    print(f"{len(frames)} frames   floor full {floor['full']} roi {floor['roi']}"
          f"   thresholds: full {tf}  roi {tr}\n")

    hashes = [md5(p) for p in frames]
    dupes = sum(1 for x, y in zip(hashes, hashes[1:]) if x == y)
    print(f"identical-byte adjacent pairs: {dupes} / {len(frames) - 1}"
          f"   {'<-- DEAD CAPTURE somewhere' if dupes else '(none - capture was live)'}\n")

    moved = 0
    peak_full = peak_roi = 0
    for i, (p1, p2) in enumerate(zip(frames, frames[1:])):
        full, roi = diff(p1, p2)
        peak_full, peak_roi = max(peak_full, full), max(peak_roi, roi)
        flag = "MOTION" if (full >= tf or roi >= tr) else "  .   "
        if flag == "MOTION":
            moved += 1
        # ~0.6 s cadence in grade.py film
        print(f"  {i*0.6:5.1f}->{(i+1)*0.6:5.1f}s  full {full:8d}  roi {roi:7d}   {flag}")

    print(f"\npeak full {peak_full} ({peak_full/max(floor['full'],1):.1f}x floor)"
          f"   peak roi {peak_roi} ({peak_roi/max(floor['roi'],1):.1f}x floor)")
    print("RESULT:", f"motion in {moved} intervals" if moved
          else "NO MOTION ANYWHERE IN THIS WINDOW")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
