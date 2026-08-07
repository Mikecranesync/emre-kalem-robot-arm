#!/usr/bin/env python3
"""Turn a recorded episode into the numbers that say how a joint moves the world.

THE QUESTION THIS ANSWERS. "Drive the target to the middle of the wrist image"
needs exactly two facts per joint: which WAY the image moves when the joint
angle increases, and how MUCH it moves per degree. Get the sign wrong and the
controller drives away from the target, faster the closer it gets. Nothing else
in this project can supply those facts, because nothing observes a shaft -- they
have to be measured by moving the joint and watching.

WHAT IT MEASURES, per consecutive pair of steps in an episode:

  dx, dy      image translation, in pixels, by phase correlation
  rot         image rotation, in degrees, by phase correlation in log-polar
  response    the correlation peak's own confidence, 0..1
  changed     thresholded changed-pixel count, as a blunt did-anything-happen

for BOTH cameras, against the commanded joint delta. The per-degree columns are
the usable output: px/deg and deg/deg.

WHY BOTH CAMERAS, AND WHY THEY DISAGREE ON PURPOSE. The side camera is fixed and
watches the arm, so it sees the ARM move across a still scene. The wrist camera
is bolted to the gripper, so it sees the WORLD sweep past a still arm. Their
signs are naturally opposite and their magnitudes are unrelated. That is not a
bug in the measurement, it is the whole reason two views are worth having.

WHAT A NUMBER HERE IS NOT. It is not a shaft angle, and it is not metric. It is
a relationship between a commanded angle and pixels, valid for THIS camera
mounting, THIS lens focus and THIS scene distance. Move the camera or change the
focus and it must be re-measured. Nothing here is calibrated -- there are no
intrinsics yet, so pixels do not convert to millimetres and this file never
pretends they do.

READ response BEFORE BELIEVING dx/dy. A low response means the correlation had
no confident peak -- typically a scene that moved too far between frames, or one
too blurred or too dark to correlate. A confident-looking dx with a response of
0.02 is noise wearing a number.

Usage:
  python correlate.py EPISODE_DIR
  python correlate.py EPISODE_DIR --roi 0.6 --csv out.csv
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys

import cv2
import numpy as np


def load(episode: str) -> list[dict]:
    path = os.path.join(episode, "transitions.jsonl")
    with open(path, "r", encoding="utf-8") as fh:
        return [json.loads(ln) for ln in fh if ln.strip()]


def prep(path: str, roi: float):
    """Grayscale float centre crop, Hann-windowed -- what phaseCorrelate wants.

    Centre crop because a wide lens is soft at the edges and, on the wrist
    camera, the gripper's own fingers sit in the bottom corners and do NOT move
    relative to the camera. Leaving them in anchors the correlation to a static
    object and biases every measurement toward zero.
    """
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        return None
    h, w = img.shape
    ch, cw = int(h * roi) // 2, int(w * roi) // 2
    crop = img[h // 2 - ch:h // 2 + ch, w // 2 - cw:w // 2 + cw]
    return np.float32(crop)


def translation(a, b):
    """dx, dy and the peak response. Positive dx = scene moved right."""
    win = cv2.createHanningWindow((a.shape[1], a.shape[0]), cv2.CV_32F)
    (dx, dy), resp = cv2.phaseCorrelate(a, b, win)
    return dx, dy, resp


def rotation(a, b):
    """Rotation in degrees via log-polar phase correlation.

    A rotation about the optical axis becomes a horizontal SHIFT in log-polar
    space, which phase correlation can find. The wrist camera is bolted to a
    ROLL joint, so this is the column that matters most for J5 -- and it is the
    one a translation-only measurement would miss entirely.
    """
    h, w = a.shape
    centre = (w / 2.0, h / 2.0)
    m = w / 2.0 / np.log(max(w, h) / 2.0)
    la = cv2.logPolar(a, centre, m, cv2.INTER_LINEAR + cv2.WARP_FILL_OUTLIERS)
    lb = cv2.logPolar(b, centre, m, cv2.INTER_LINEAR + cv2.WARP_FILL_OUTLIERS)
    win = cv2.createHanningWindow((w, h), cv2.CV_32F)
    (_, dyy), resp = cv2.phaseCorrelate(np.float32(la), np.float32(lb), win)
    return dyy * 360.0 / h, resp


def changed_px(a, b, thresh: int = 25) -> int:
    d = cv2.absdiff(np.uint8(a), np.uint8(b))
    return int((d > thresh).sum())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("episode")
    ap.add_argument("--roi", type=float, default=0.6,
                    help="centre crop fraction (default 0.6)")
    ap.add_argument("--csv", default="", help="also write the table here")
    args = ap.parse_args()

    steps = load(args.episode)
    if len(steps) < 2:
        print("  need at least two steps to compare")
        return 1

    meta_path = os.path.join(args.episode, "metadata.json")
    meta = json.load(open(meta_path, encoding="utf-8")) if os.path.exists(meta_path) else {}
    joint = meta.get("joint")
    print(f"  episode: {args.episode}")
    print(f"  mode={meta.get('mode')} joint={joint} roi={args.roi}\n")

    roles = [r for r in ("side", "wrist") if r in steps[0].get("images", {})]
    hdr = f"  {'from':>5} {'to':>5} {'d_ang':>6}"
    for r in roles:
        hdr += f" | {r:^33}"
    print(hdr)
    sub = " " * 19
    for _ in roles:
        sub += f" | {'dx':>6} {'dy':>6} {'rot':>6} {'resp':>5} {'chg':>6}"
    print(sub)

    rows = []
    for prev, cur in zip(steps, steps[1:]):
        pa = (prev.get("commanded") or {}).get("angle")
        ca = (cur.get("commanded") or {}).get("angle")
        d_ang = (ca - pa) if (pa is not None and ca is not None) else None
        line = f"  {str(pa):>5} {str(ca):>5} {str(d_ang):>6}"
        row = {"from": pa, "to": ca, "d_angle": d_ang}
        for r in roles:
            fa = prev["images"][r].get("file")
            fb = cur["images"][r].get("file")
            if not fa or not fb:
                line += f" | {'-- no frame --':^33}"
                continue
            a = prep(os.path.join(args.episode, fa), args.roi)
            b = prep(os.path.join(args.episode, fb), args.roi)
            dx, dy, resp = translation(a, b)
            rot, _ = rotation(a, b)
            chg = changed_px(a, b)
            line += f" | {dx:6.1f} {dy:6.1f} {rot:6.2f} {resp:5.2f} {chg:6d}"
            row.update({f"{r}_dx": round(dx, 2), f"{r}_dy": round(dy, 2),
                        f"{r}_rot": round(rot, 3), f"{r}_resp": round(resp, 3),
                        f"{r}_changed": chg})
            if d_ang:
                row[f"{r}_px_per_deg"] = round((dx ** 2 + dy ** 2) ** 0.5 / abs(d_ang), 2)
                row[f"{r}_rotdeg_per_deg"] = round(rot / d_ang, 3)
        print(line)
        rows.append(row)

    # Summary, only over steps with a real commanded delta.
    print()
    for r in roles:
        vals = [x[f"{r}_px_per_deg"] for x in rows if f"{r}_px_per_deg" in x]
        rots = [x[f"{r}_rotdeg_per_deg"] for x in rows if f"{r}_rotdeg_per_deg" in x]
        resp = [x[f"{r}_resp"] for x in rows if f"{r}_resp" in x]
        if not vals:
            continue
        print(f"  {r:5}: median {np.median(vals):6.2f} px per commanded degree, "
              f"{np.median(rots):+6.3f} image-deg per commanded degree, "
              f"median response {np.median(resp):.2f}")
        if np.median(resp) < 0.15:
            print(f"         ^ LOW RESPONSE -- treat these as unmeasured, not as "
                  f"small. The correlation found no confident peak.")

    if args.csv and rows:
        keys = sorted({k for x in rows for k in x})
        with open(args.csv, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)
        print(f"\n  wrote {args.csv}")

    print("\n  These are pixel relationships for THIS mounting, focus and scene "
          "distance.\n  No camera is calibrated, so nothing here converts to "
          "millimetres.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
