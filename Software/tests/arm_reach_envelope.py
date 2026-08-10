"""Measure where the arm can actually GO, by watching it be moved through its range.

WHY THIS EXISTS
---------------
The measurement ROI in arm_bench_test.py has twice been cut from a single frame
of a parked arm, and both times it was wrong. On 2026-08-10 a box derived from
the folded pose (x660-1250) was falsified within a minute by raising the arm:
the gripper landed near x460, two hundred pixels outside it. A joint that moves
outside the ROI is scored as a DEAD MOVE - the exact false claim this harness was
built to prevent - so an ROI cut from one pose is not a small error, it is a
manufactured finding.

One pose is not a reach envelope. This tool builds the envelope from motion:
it holds a reference frame, then accumulates every pixel that changes while the
arm is swept through its range. Whatever lit up is somewhere the arm can be.

HOW TO USE IT
-------------
    python arm_reach_envelope.py [seconds]

Run it, then move the arm through its full range for the duration - by hand with
the power off is fine and is the safest way. The envelope prints as it grows, so
you can watch it stop growing, which is how you know you have covered the range.

The operator's hands and arm are in frame while sweeping by hand, so the result
is an OVER-estimate. That is deliberate: a too-large ROI costs a little noise
floor (measured at ~90 px on a 400k-px box, against a QUIET_PX of 900), while a
too-small one invents dead moves. Err large.

Nothing here touches the arm, the serial link or the watchdog. It only reads
frames from the camera preview.
"""

from __future__ import annotations

import hashlib
import os
import sys
import time
import urllib.request

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_bench_test import CAM, ROI, THRESH  # noqa: E402

BANNER_ROWS = 70     # burnt-in status text; it redraws every frame
MARGIN = 40          # px of slack added to the observed envelope
SETTLE_FRAMES = 5    # frames averaged into the reference before sweeping starts

# A pixel must change in at least this many frames to count as swept. Without it
# the envelope is worthless: the first run pinned to the whole frame within two
# seconds because sensor speckle on high-contrast edges - gridlines, cables, the
# shelf - put a stray changed pixel in every corner, and a bounding box drawn
# around ANY changed pixel is a bounding box around the noise. Only 1.1% of the
# frame had actually changed. Real swept structure persists across frames;
# speckle does not.
PERSIST = 3
OPEN_K = 3           # morphological opening, kills isolated survivors


def grab() -> np.ndarray:
    raw = urllib.request.urlopen(CAM, timeout=15).read()
    img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def bounds(mask: np.ndarray) -> tuple[int, int, int, int] | None:
    cols = np.where(mask.any(axis=0))[0]
    rows = np.where(mask.any(axis=1))[0]
    if not len(cols) or not len(rows):
        return None
    return int(cols[0]), int(rows[0]) + BANNER_ROWS, int(cols[-1]), int(rows[-1]) + BANNER_ROWS


def main() -> None:
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0

    print("hold still - taking a reference of the scene", flush=True)
    ref = np.median([grab()[BANNER_ROWS:, :] for _ in range(SETTLE_FRAMES)], axis=0)
    ref = ref.astype(np.uint8)
    H, W = ref.shape

    print(f"sweep the arm through its FULL range for {secs:.0f}s - start now", flush=True)
    hits = np.zeros((H, W), np.int32)
    end, last, n = time.time() + secs, 0.0, 0
    seen: set[str] = set()

    def envelope() -> np.ndarray:
        m = (hits >= PERSIST).astype(np.uint8)
        return cv2.morphologyEx(m, cv2.MORPH_OPEN,
                                np.ones((OPEN_K, OPEN_K), np.uint8)).astype(bool)

    while time.time() < end:
        raw = urllib.request.urlopen(CAM, timeout=15).read()
        seen.add(hashlib.md5(raw).hexdigest())
        img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        hits += (cv2.absdiff(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)[BANNER_ROWS:, :],
                             ref) > THRESH)
        n += 1
        if time.time() - last >= 2.0:
            env = envelope()
            b = bounds(env)
            if b:
                print(f"  {end - time.time():5.1f}s left   envelope "
                      f"x{b[0]}-{b[2]}  y{b[1]}-{b[3]}   "
                      f"{env.mean() * 100:4.1f}% of frame", flush=True)
            last = time.time()

    env = envelope()
    b = bounds(env)
    if not b:
        print("\nNOTHING MOVED. The envelope is empty - was the arm actually swept?")
        raise SystemExit(1)

    x0 = max(0, b[0] - MARGIN)
    y0 = max(BANNER_ROWS, b[1] - MARGIN)
    x1 = min(W, b[2] + MARGIN)
    y1 = min(H + BANNER_ROWS, b[3] + MARGIN)

    print(f"\nframes            {n}, distinct {len(seen)}")
    if len(seen) < n * 0.8:
        print("  !! DUPLICATE FRAMES - the camera handed back stale images, so motion")
        print("     was missed and this envelope is an UNDER-estimate. Close any other")
        print("     viewer on /stream and re-run before trusting it.")
    print(f"observed envelope x{b[0]}-{b[2]}  y{b[1]}-{b[3]}")
    print(f"with {MARGIN}px margin  x{x0}-{x1}  y{y0}-{y1}")
    print(f"ROI in code now   x{ROI[0]}-{ROI[2]}  y{ROI[1]}-{ROI[3]}")

    miss = []
    if x0 < ROI[0]:
        miss.append(f"{ROI[0] - x0}px on the LEFT")
    if x1 > ROI[2]:
        miss.append(f"{x1 - ROI[2]}px on the RIGHT")
    if y0 < ROI[1]:
        miss.append(f"{ROI[1] - y0}px on the TOP")
    if y1 > ROI[3]:
        miss.append(f"{y1 - ROI[3]}px on the BOTTOM")
    print("verdict           " + ("current ROI COVERS the swept range"
                                  if not miss else
                                  "current ROI MISSES " + ", ".join(miss)))
    print(f"\n    ROI = ({x0}, {y0}, {x1}, {y1})")

    vis = cv2.cvtColor(ref, cv2.COLOR_GRAY2BGR)
    vis[env] = (0, 200, 255)
    cv2.rectangle(vis, (x0, y0 - BANNER_ROWS), (x1, y1 - BANNER_ROWS), (0, 255, 0), 2)
    cv2.rectangle(vis, (ROI[0], ROI[1] - BANNER_ROWS), (ROI[2], ROI[3] - BANNER_ROWS),
                  (80, 80, 255), 1)
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "reach-envelope.jpg")
    cv2.imwrite(out, vis)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
