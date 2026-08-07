#!/usr/bin/env python3
"""Record both camera feeds against commanded joint state, in one episode.

WHAT THIS IS FOR. Learning how a joint command maps to movement in the world.
Nothing on this arm observes a shaft, so the ONLY way to know what a command did
is to look. Two views answer two different questions:

    side  (laptop lid)   where the ARM is        -- did it move, and how far
    wrist (Arducam)      where the WORLD is      -- what the gripper is facing

Correlating them is what makes "drive the target to the middle of the wrist
image" possible without any 3D maths: you learn the SIGN and the SCALE of each
joint's effect in image space, per joint, by measurement. That is step 1 of the
development sequence in the telegram/voice PRD's ancestor document, and it needs
no calibration, no hand-eye solve, and no metric pose.

WHAT IT RECORDS PER STEP, which is the shape a learning dataset needs later:

    commanded angle  ->  board reply (REQ/SET/CL)  ->  side frame  ->  wrist frame

all timestamped, plus the mean-luma and sharpness of each frame so a dead or
defocused capture is visible in the data rather than discovered afterwards.

HOW IT TALKS TO THE ARM, AND WHY NOT DIRECTLY. It does NOT open the serial port.
hold_arm.py owns COM5 and must keep owning it -- the watchdog latches and every
joint detaches the moment nothing feeds it, and closing the port DTR-resets the
board. So commands go through the daemon's file channel and this process never
becomes a second port owner. Measured on 2026-08-07: a blocking serial read in
the same thread as camera capture drops both cameras from 29.9 to 22.3 fps, so
keeping them in separate processes is a measured decision and not a stylistic one.

SAFETY, STATED PLAINLY AND NOT SOFTENED
  * Nothing in software is an emergency stop on this arm. STP holds with joints
    driven. EST and the watchdog DETACH, and a gravity-loaded arm FALLS. The
    rocker switch and the fuse are the only real stop. This script sends none of
    those and cannot.
  * It may command only MOV, and only on joints 1/3/4/5. J0's servo is dead.
    J6's gripper gear slips -- it acks every command and the fingers do not move,
    so commanding it records a lie.
  * It never sends ENA. Enabling drives a joint to its adopt angle; that belongs
    to the daemon and to a human watching the arm.
  * A step is refused if the reply says CL=1 (the firmware clamped it, so the
    joint is NOT where it was asked to be) or if the log window shows a latch.

Usage:
  python dual_record.py --observe 20 --out EPISODE_DIR --link DAEMON_DIR
  python dual_record.py --sweep 5 --from 104 --to 140 --step 6 --out DIR --link DIR
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cameras                                                    # noqa: E402

# ArmLink lives in the telegram package because that is where the stdlib-only
# copy was lifted to; motion_verify.py's copy drags OpenCV in at import time.
_TG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "arm-telegram")
sys.path.insert(0, _TG)
from arm_link import ArmLink                                      # noqa: E402

# Joints this script may command. J0 is a dead servo; J6's gear slips and would
# record motion that did not happen.
MOVABLE = (1, 3, 4, 5)
LATCH_MARKERS = ("LATCHED", "re-ENA")


def frame_stats(frame) -> dict:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return {"luma": round(float(gray.mean()), 2),
            "sharpness": round(float(cv2.Laplacian(gray, cv2.CV_64F).var()), 2)}


class Episode:
    """One recording. Frames on disk, one JSON line per step."""

    def __init__(self, out_dir: str, meta: dict):
        self.dir = out_dir
        self.frames = os.path.join(out_dir, "frames")
        os.makedirs(self.frames, exist_ok=True)
        self.t0 = time.monotonic()
        with open(os.path.join(out_dir, "metadata.json"), "w",
                  encoding="utf-8") as fh:
            json.dump(meta, fh, indent=2)
        self.path = os.path.join(out_dir, "transitions.jsonl")
        open(self.path, "w").close()
        self.n = 0

    def step(self, caps, commanded: dict | None, reply: str, note: str = "") -> dict:
        rec = {"i": self.n, "t": round(time.monotonic() - self.t0, 3),
               "commanded": commanded, "reply": reply, "note": note,
               "images": {}}
        for role, (cap, spec) in caps.items():
            ok, frame = cameras.read(cap, spec)
            if not ok:
                rec["images"][role] = {"file": None, "error": "no frame"}
                continue
            name = f"{self.n:04d}_{role}.png"
            cv2.imwrite(os.path.join(self.frames, name), frame)
            rec["images"][role] = {"file": f"frames/{name}", **frame_stats(frame)}
        with open(self.path, "a", encoding="utf-8") as fh:
            fh.write(json.dumps(rec) + "\n")
        self.n += 1
        return rec


def open_cams():
    caps = {}
    for role in ("side", "wrist"):
        cap, spec, idx = cameras.open_role(role)
        caps[role] = (cap, spec)
        print(f"  {role:5} index {idx}  {spec.friendly_name}")
    return caps


def close_cams(caps):
    for cap, _ in caps.values():
        cap.release()


def cmd_observe(args) -> int:
    """Record both feeds with no commands at all. Safe with the arm at rest."""
    caps = open_cams()
    link = ArmLink(args.link) if args.link else None
    ep = Episode(args.out, {"mode": "observe", "seconds": args.seconds,
                            "link": args.link, "arm_commanded": False})
    print(f"  observing {args.seconds}s -> {args.out}")
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.seconds:
        reply = link.send("STA") if link else ""
        r = ep.step(caps, None, reply, note="observe")
        s = " ".join(f"{k}:luma {v.get('luma')} sharp {v.get('sharpness')}"
                     for k, v in r["images"].items())
        print(f"    [{r['i']:03d}] {s}")
        time.sleep(max(0.0, args.interval))
    close_cams(caps)
    print(f"  {ep.n} steps -> {ep.path}")
    return 0


def cmd_sweep(args) -> int:
    """Drive ONE joint through a range, capturing both views at every step.

    This is the measurement that teaches the sign convention: which way does the
    world move in each image when this joint increases. It is deliberately one
    joint at a time -- a coordinated move confounds the very thing being learned.
    """
    j = args.sweep
    if j not in MOVABLE:
        print(f"  refusing joint {j}: this script may command only {MOVABLE}. "
              f"J0's servo is dead; J6's gear slips and would record motion that "
              f"did not happen.")
        return 1
    if not args.link:
        print("  --sweep needs --link DIR (the running daemon's directory). This "
              "script never opens the serial port itself.")
        return 1

    link = ArmLink(args.link)
    sta = link.send("STA")
    if any(m in sta for m in LATCH_MARKERS) or "ES=1" in sta or "WD=1" in sta:
        print(f"  refusing to move: the arm is latched or was re-enabled.\n{sta}")
        return 1
    row = [ln for ln in sta.splitlines() if ln.startswith(f"STA J{j} ")]
    if not row or "EN=1" not in row[0]:
        print(f"  refusing to move joint {j}: it is not enabled.\n    {row}")
        return 1
    print(f"  joint {j} before: {row[0]}")

    caps = open_cams()
    ep = Episode(args.out, {"mode": "sweep", "joint": j, "from": args.frm,
                            "to": args.to, "step": args.step, "link": args.link,
                            "arm_commanded": True, "before": row[0]})

    step = abs(args.step) * (1 if args.to >= args.frm else -1)
    angles = list(range(args.frm, args.to + (1 if step > 0 else -1), step))
    print(f"  sweeping J{j}: {angles}")
    rc = 0
    for a in angles:
        off = link.offset()
        reply = link.send(f"MOV {j} {a}")
        window = link.tail(off)
        if "CL=1" in reply:
            print(f"    J{j} -> {a}: CLAMPED ({reply.strip()}) -- stopping. The "
                  f"joint is not where it was asked to be.")
            ep.step(caps, {"joint": j, "angle": a}, reply, note="clamped-stop")
            rc = 1
            break
        if any(m in window for m in LATCH_MARKERS):
            print(f"    J{j} -> {a}: LATCH in the log window -- stopping.")
            ep.step(caps, {"joint": j, "angle": a}, reply, note="latch-stop")
            rc = 1
            break
        time.sleep(args.settle)
        r = ep.step(caps, {"joint": j, "angle": a}, reply)
        s = "  ".join(f"{k} sharp {v.get('sharpness')}"
                      for k, v in r["images"].items())
        print(f"    J{j} -> {a:3d}  {reply.strip()[:52]:52}  {s}")

    close_cams(caps)
    print(f"\n  {ep.n} steps -> {ep.path}")
    print("  A board ack is not motion. Compare the frames before claiming the "
          "joint moved.")
    return rc


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--observe", type=float, metavar="SECONDS", dest="seconds",
                   help="record both feeds, send nothing")
    g.add_argument("--sweep", type=int, metavar="JOINT",
                   help="drive one joint through a range, capturing both views")
    p.add_argument("--out", required=True, help="episode directory")
    p.add_argument("--link", default="", help="the running daemon's directory")
    p.add_argument("--from", dest="frm", type=int, help="sweep start angle")
    p.add_argument("--to", type=int, help="sweep end angle")
    p.add_argument("--step", type=int, default=5, help="sweep step in degrees")
    p.add_argument("--settle", type=float, default=1.0,
                   help="seconds to wait after each move before capturing")
    p.add_argument("--interval", type=float, default=0.5,
                   help="observe mode: seconds between samples")
    args = p.parse_args()

    os.makedirs(args.out, exist_ok=True)
    if args.seconds is not None:
        return cmd_observe(args)
    if args.frm is None or args.to is None:
        p.error("--sweep needs --from and --to")
    return cmd_sweep(args)


if __name__ == "__main__":
    sys.exit(main())
