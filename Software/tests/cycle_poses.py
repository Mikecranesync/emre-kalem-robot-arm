"""Cycle the arm between two named poses, escalating speed until it stops being smooth.

WHY PHASES AND NOT A STRAIGHT LINE. Two poses can both be safe while the straight
line between them drives the claw through the bench or the base housing -- the same
reason arm-poses.csv records an entry_path. So a transition here is a SEQUENCE of
phases, and only the joints inside one phase move together. The ordering is not
cosmetic:

  pick -> storage   lift first (get the claw away from the mat), THEN neutralise the
                    wrist and tuck the elbow while the arm is high, THEN fold onto
                    the base.
  storage -> pick   swing out while still tucked and high, THEN aim the claw in free
                    space, THEN descend.

Reversing those orders drops a claw that is still pointing down onto the table, or
swings a wrist through the loom.

WHAT "SMOOTH" MEANS HERE, because it has to be measurable and not a vibe:
  1. every joint reaches its commanded angle -- SET == target with MOV=0
  2. no clamp (CL=1), no joint timeout (JTO != 0)
  3. no watchdog latch or daemon re-enable during the run
  4. the arm comes back to the SAME place each cycle -- an endpoint frame is
     compared against the first cycle's frame at that endpoint, and the changed-
     pixel count is the repeatability error. A rising error across cycles means the
     arm is overshooting or losing position as speed climbs, which is exactly the
     failure that "it looks smooth" would miss.

SPEED CEILING IS 90 DPS. The firmware rejects more: ERR E12 SPD JOINT=1 REQ=120
MIN=1 MAX=90. Note joint-limits.csv carries max_deg_per_sec=30 per joint as the
recorded intent -- this tool can drive past that, which is the point of the
escalation, but going above 30 is a deliberate act and the operator should know.

Usage:
  python cycle_poses.py --link <daemon-dir> --out <dir> --cycles 2 --speeds 15,25,40,60,90
"""

from __future__ import annotations

import argparse
import json
import os
import time

import cv2
import numpy as np

import sys as _sys, os as _os
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
from reply_cut import cut_reply, clamped  # noqa: E402

PICK = {1: 8, 3: 36, 4: 140, 5: 165}
STORAGE = {1: 88, 3: 64, 4: 90, 5: 104}
TRANSIT_J1 = 40  # shoulder height used to cross between the two, claw clear of the mat

TO_STORAGE = [("lift", {1: TRANSIT_J1}),
              ("neutralise", {3: 64, 4: 90, 5: 104}),
              ("fold", {1: 88})]
TO_PICK = [("swing_out", {1: TRANSIT_J1}),
           ("aim", {3: 36, 4: 140, 5: 165}),
           ("descend", {1: 8})]

ROI = (300, 200, 830, 680)   # whole arm, on the backdrop, operator excluded
DIFF_THRESHOLD = 25


class Arm:
    def __init__(self, link_dir):
        self.cmd = os.path.join(link_dir, "arm_cmd.txt")
        self.log = os.path.join(link_dir, "arm_hold.log")
        if not os.path.exists(self.log):
            raise SystemExit(f"no daemon log at {self.log} -- is hold_arm.py running?")

    def offset(self):
        return os.path.getsize(self.log)

    def tail(self, off):
        with open(self.log, "r", encoding="utf-8", errors="replace") as fh:
            fh.seek(off)
            return fh.read()

    def send(self, line, timeout=6.0):
        off = self.offset()
        with open(self.cmd, "w", encoding="utf-8") as fh:
            fh.write(line + "\n")
        t0 = time.monotonic()
        marker = f"CMD {line} -> "
        while time.monotonic() - t0 < timeout:
            new = self.tail(off)
            if marker in new:
                # CUT at the daemon's next timestamped line -- reply_cut.py. The
                # daemon logs a PNG heartbeat and a 5 s STA poll into this same
                # file, so without the cut they are returned as part of THIS
                # command's reply, and a reply read mid-flush can be truncated
                # before its own CL= field. Observed live 2026-08-07.
                return cut_reply(new.split(marker, 1)[1])
            time.sleep(0.05)
        raise SystemExit(f"no reply for {line!r}")

    def status(self):
        rows = {}
        for ln in self.send("STA").splitlines():
            if ln.startswith("STA J"):
                p = ln.split()
                rows[int(p[1][1:])] = {k: v for k, v in (t.split("=", 1) for t in p[2:])}
        return rows

    def settle(self, targets, timeout):
        """Wait until every joint in `targets` is parked where it was told."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rows = self.status()
            if all(rows.get(j, {}).get("MOV") == "0"
                   and rows.get(j, {}).get("SET") == str(v) for j, v in targets.items()):
                return True, rows, round(time.monotonic() - t0, 2)
            time.sleep(0.12)
        return False, self.status(), round(time.monotonic() - t0, 2)


def grab(cap):
    frame = None
    for _ in range(5):
        ok, fr = cap.read()
        if ok:
            frame = fr
        time.sleep(0.03)
    return frame


def changed_px(a, b):
    x0, y0, x1, y1 = ROI
    ga = cv2.GaussianBlur(cv2.cvtColor(a[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY), (3, 3), 0)
    gb = cv2.GaussianBlur(cv2.cvtColor(b[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY), (3, 3), 0)
    return int((cv2.absdiff(ga, gb) > DIFF_THRESHOLD).sum())


def run_leg(arm, phases, dps, cur):
    """Drive one direction. Returns (ok, seconds, notes)."""
    notes = []
    t0 = time.monotonic()
    for label, targets in phases:
        off = arm.offset()
        travel = max(abs(targets[j] - cur.get(j, targets[j])) for j in targets)
        for j, v in targets.items():
            reply = arm.send(f"MOV {j} {v}")
            cl = clamped(reply)
            if cl is None:
                # No CL= field survived. NOT a pass -- an unreadable reply is
                # exactly how a real clamp went unseen before the cut was added.
                notes.append(f"{label}: J{j} reply UNREADABLE (no CL=) -- {reply!r}")
                return False, round(time.monotonic() - t0, 2), notes
            if cl:
                # A clamp means the joint is NOT where it was told to go. The
                # docstring's own definition of smooth says "no clamp (CL=1)",
                # and arm-serial-control section 6 says treat it as a failed
                # command, not a warning. It used to only append a note and keep
                # driving; it now stops.
                notes.append(f"{label}: J{j} CLAMPED -- {reply}")
                return False, round(time.monotonic() - t0, 2), notes
            if "ERR" in reply:
                return False, round(time.monotonic() - t0, 2), notes + [f"{label}: {reply}"]
        ok, rows, waited = arm.settle(targets, timeout=max(3.0, travel / dps * 1.8 + 2.0))
        for j in targets:
            if rows.get(j, {}).get("JTO", "0") != "0":
                notes.append(f"{label}: J{j} JTO={rows[j]['JTO']}")
        window = arm.tail(off)
        if "LATCHED" in window or "re-ENA" in window:
            notes.append(f"{label}: DAEMON INTERVENED")
            return False, round(time.monotonic() - t0, 2), notes
        if not ok:
            notes.append(f"{label}: did not reach target in {waited}s")
            return False, round(time.monotonic() - t0, 2), notes
        cur.update(targets)
    return True, round(time.monotonic() - t0, 2), notes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--link", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--cycles", type=int, default=2)
    ap.add_argument("--speeds", default="15,25,40,60,90")
    ap.add_argument("--camera", type=int, default=0)
    args = ap.parse_args()
    speeds = [int(s) for s in args.speeds.split(",")]
    os.makedirs(args.out, exist_ok=True)

    arm = Arm(args.link)
    cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    for _ in range(10):
        cap.read()
        time.sleep(0.03)

    cur = dict(PICK)
    ref = {}
    results = []
    try:
        for dps in speeds:
            for j in (1, 3, 4, 5):
                r = arm.send(f"SPD {j} {dps}")
                if "ERR" in r:
                    raise SystemExit(f"SPD {j} {dps} rejected: {r}")
            print(f"\n=== {dps} deg/s ===")
            for c in range(1, args.cycles + 1):
                ok_s, t_s, n_s = run_leg(arm, TO_STORAGE, dps, cur)
                f_s = grab(cap)
                ok_p, t_p, n_p = run_leg(arm, TO_PICK, dps, cur)
                f_p = grab(cap)
                cv2.imwrite(f"{args.out}/d{dps}_c{c}_storage.png", f_s)
                cv2.imwrite(f"{args.out}/d{dps}_c{c}_pick.png", f_p)
                ref.setdefault("storage", f_s)
                ref.setdefault("pick", f_p)
                drift_s = changed_px(ref["storage"], f_s)
                drift_p = changed_px(ref["pick"], f_p)
                rec = dict(dps=dps, cycle=c, to_storage_s=t_s, to_pick_s=t_p,
                           round_trip_s=round(t_s + t_p, 2),
                           drift_storage_px=drift_s, drift_pick_px=drift_p,
                           notes=n_s + n_p, ok=ok_s and ok_p)
                results.append(rec)
                print(f"  cycle {c}: {rec['round_trip_s']}s round trip "
                      f"(to_storage {t_s}s / to_pick {t_p}s)  "
                      f"drift storage={drift_s} pick={drift_p} px"
                      + ("" if rec["ok"] else "   <<< FAILED"))
                for n in rec["notes"]:
                    print(f"      ! {n}")
                if not rec["ok"]:
                    print(f"  STOPPING ESCALATION at {dps} deg/s")
                    raise KeyboardInterrupt
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        with open(f"{args.out}/cycle_result.json", "w") as fh:
            json.dump(results, fh, indent=2)

    good = [r for r in results if r["ok"]]
    print(f"\n{len(good)}/{len(results)} cycles clean")
    if good:
        best = min(good, key=lambda r: r["round_trip_s"])
        print(f"fastest clean round trip: {best['round_trip_s']}s at {best['dps']} deg/s")
        top = max(r["dps"] for r in good)
        drifts = [r for r in good if r["dps"] == top]
        print(f"highest clean speed: {top} deg/s "
              f"(endpoint drift storage {[d['drift_storage_px'] for d in drifts]}, "
              f"pick {[d['drift_pick_px'] for d in drifts]})")


if __name__ == "__main__":
    main()
