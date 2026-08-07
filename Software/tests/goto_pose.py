#!/usr/bin/env python3
"""Drive the arm to a named pose along its own recorded entry path. One way.

WHY THIS EXISTS ALONGSIDE cycle_poses.py. That tool cycles BETWEEN two poses and
escalates speed to find where motion stops being smooth. It assumes the arm is
already parked at one of them. After a drop, a reset, or a fresh enable, it is
not -- and the one thing arm-poses.csv capitalises is that you do NOT interpolate
between two poses and assume the middle is clear. So this tool covers the case
cycle_poses cannot: getting from the daemon's ADOPT state back onto a recorded
path, and then along it, one direction, at a speed you choose.

THE ADOPT STATE IS WHY THE RECOVERY PATH WORKS. hold_arm.py enables J1 at its
home angle of 1, and storage's recorded entry path in arm-poses.csv begins at
exactly J1 1: "J1 1>22>34>46>58>70>80>88 at 5 deg/s then J3 52>60>64 at 20 deg/s".
That is not a coincidence -- home_deg IS 1 -- and it means a freshly enabled arm
is already standing at the start of a verified route. Take it rather than
inventing a shortcut.

THE PHASE CONSTANTS ARE IMPORTED FROM cycle_poses.py, NEVER RETYPED. Two copies
of a waypoint list is how one of them silently goes stale, and the failure mode
is a claw driven through the bench. Same reason arm-poses.csv says to reverse a
pose's own entry path instead of interpolating.

SAFETY, NOT SOFTENED
  * Nothing in software is an emergency stop here. STP holds with joints driven.
    EST and the watchdog DETACH, and a gravity-loaded arm FALLS. The rocker
    switch and the fuse are the only real stop. This tool sends neither.
  * It never sends ENA. Enabling drives a joint to its adopt angle; that is the
    daemon's job and it needs a human watching the arm.
  * It stops on the first CL=1 (the firmware clamped, so the joint is NOT where
    it was asked to be), on any JTO, on any ERR, on a failed settle, and on any
    LATCHED / re-ENA in the log window.
  * A board ack is not motion. Frames are captured after every phase so the
    claim can be checked against a picture rather than against an ack.

Usage:
  python goto_pose.py --to storage --link DIR --out DIR [--dps 5]
  python goto_pose.py --to pick    --link DIR --out DIR [--dps 12]
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "arm-vision"))

import cameras                                                     # noqa: E402
from cycle_poses import PICK, STORAGE, TO_PICK, TO_STORAGE, Arm     # noqa: E402

# The recovery leg: from the daemon's adopt state onto storage, using the
# waypoint list recorded in arm-poses.csv. J4/J5 already sit at storage's values
# (90/104) after enable, so only J1 and J3 travel.
ADOPT_TO_STORAGE = [
    ("fold_j1", [(1, a) for a in (22, 34, 46, 58, 70, 80, 88)]),
    ("tuck_j3", [(3, a) for a in (52, 60, 64)]),
]
LATCH_MARKERS = ("LATCHED", "re-ENA")
_STAMP = re.compile(r"^\d\d:\d\d:\d\d ", re.M)


class CleanArm(Arm):
    """Arm.send() with the reply cut at the daemon's next timestamp.

    THE DEFECT THIS FIXES, OBSERVED ON A REAL RUN AND SAFETY-RELEVANT.
    cycle_poses.Arm.send() returns everything after its marker to the END of the
    log, and the daemon writes its own lines into that same log -- PNG
    heartbeats and a STA poll every 5 s. So an unrelated daemon line lands
    INSIDE the quoted reply, and worse, the reply can be read before the rest of
    its own line has been flushed. The 2026-08-07 pick->storage run produced:

        03_fold: J1->88  OK PNG UP=2480971 \\n\\n OK MOV J1 REQ=88 SET=88 C

    -- truncated immediately before the clamp flag. Every guard in this tool
    tests `"CL=1" in reply`. On that string a genuine CL=1 is INVISIBLE and the
    run continues past a joint that is not where it was told to go. The run
    happened to be clean; the check was not.

    Cut at the next line beginning HH:MM:SS, NOT at the next newline -- the
    daemon timestamps only the FIRST line of a multi-line message, so a naive
    newline cut truncates every STA to its J0 row. Same rule as
    arm-telegram/arm_link.py, which fixed this on the other surface first.

    A reply that still has no CL= field after cutting is treated as unreadable
    rather than as a pass -- silence is not consent.
    """

    def send(self, line, timeout=6.0):
        raw = super().send(line, timeout)
        m = _STAMP.search(raw)
        return (raw[:m.start()] if m else raw).strip()


def clamped(reply: str) -> bool | None:
    """True clamped, False clean, None unreadable. None is NOT a pass."""
    if "CL=" not in reply:
        return None
    return "CL=1" in reply


def check_ready(arm: Arm, joints) -> bool:
    rows = arm.status()
    ok = True
    for j in joints:
        r = rows.get(j)
        if not r:
            print(f"    J{j}: no status row")
            ok = False
        elif r.get("EN") != "1":
            print(f"    J{j}: EN={r.get('EN')} -- not enabled")
            ok = False
        elif r.get("JTO") not in ("0", None):
            print(f"    J{j}: JTO={r.get('JTO')} -- joint timeout already set")
            ok = False
    return ok


def snap(caps, out_dir, label):
    import cv2
    os.makedirs(out_dir, exist_ok=True)
    line = []
    for role, (cap, spec) in caps.items():
        ok, frame = cameras.read(cap, spec)
        if not ok:
            line.append(f"{role}:NO FRAME")
            continue
        path = os.path.join(out_dir, f"{label}_{role}.png")
        cv2.imwrite(path, frame)
        g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        line.append(f"{role}:luma {g.mean():.0f}")
    return "  ".join(line)


def drive(arm: Arm, caps, out_dir, label, moves, dps, settle_extra=2.0) -> bool:
    """One phase: command each waypoint, settle, verify, photograph."""
    targets = {}
    for j, v in moves:
        # Timeout must scale with TRAVEL, not be a constant. J1 88->40 at 5 deg/s
        # is 9.6 s of real motion; a fixed 8 s window would report "did not
        # settle" on a joint that was moving correctly and stop a good run.
        # Same 1.8x margin cycle_poses.py uses.
        before = arm.status().get(j, {}).get("SET")
        travel = abs(v - int(before)) if before is not None else 90
        budget = max(4.0, travel / max(dps, 1) * 1.8 + settle_extra + 1.0)
        off = arm.offset()
        reply = arm.send(f"MOV {j} {v}")
        window = arm.tail(off)
        if "ERR" in reply:
            print(f"    {label}: J{j}->{v}  {reply}  -- STOPPING")
            return False
        cl = clamped(reply)
        if cl is None:
            print(f"    {label}: J{j}->{v}  reply has no CL= field -- UNREADABLE, "
                  f"not a pass. STOPPING.  raw: {reply!r}")
            return False
        if cl:
            print(f"    {label}: J{j}->{v}  CLAMPED ({reply}) -- STOPPING. The "
                  f"joint is not where it was asked to be.")
            return False
        if any(m in window for m in LATCH_MARKERS):
            print(f"    {label}: J{j}->{v}  LATCH in log window -- STOPPING")
            return False
        targets[j] = v
        ok, rows, waited = arm.settle({j: v}, timeout=budget)
        jto = rows.get(j, {}).get("JTO")
        if not ok:
            print(f"    {label}: J{j}->{v}  DID NOT SETTLE in {waited}s "
                  f"(SET={rows.get(j, {}).get('SET')}) -- STOPPING")
            return False
        if jto not in ("0", None):
            print(f"    {label}: J{j}->{v}  JTO={jto} -- joint stalled. STOPPING")
            return False
        print(f"    {label}: J{j}->{v:3d}  settled {waited}s  {reply.strip()[:44]}")
    print(f"      {snap(caps, out_dir, label)}")
    return True


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--to", required=True, choices=("storage", "pick"))
    p.add_argument("--link", required=True, help="the running daemon's directory")
    p.add_argument("--out", required=True, help="where frames go")
    p.add_argument("--dps", type=int, default=5, help="speed for this run")
    p.add_argument("--from-adopt", action="store_true",
                   help="start with the adopt->storage recovery leg")
    args = p.parse_args()

    arm = CleanArm(args.link)
    sta = arm.send("STA")
    if "ES=1" in sta or "WD=1" in sta:
        print("  arm is LATCHED -- refusing to move. Clear it and re-enable first.")
        return 1
    joints = sorted(set(list(PICK) + list(STORAGE)))
    print("  preflight:")
    if not check_ready(arm, joints):
        print("  refusing to move.")
        return 1
    rows = arm.status()
    print("    " + "  ".join(f"J{j}={rows[j]['SET']}" for j in joints if j in rows))

    legs = []
    if args.from_adopt:
        legs += ADOPT_TO_STORAGE
    if args.to == "pick":
        legs += [(label, list(t.items())) for label, t in TO_PICK]
    elif not args.from_adopt:
        # pick -> storage. The ordering is the whole safety property: LIFT first
        # to get the claw off the mat, THEN neutralise the wrist while the arm
        # is high, THEN fold onto the base. Neutralising before lifting swings a
        # downward-pointing claw across the table.
        legs += [(label, list(t.items())) for label, t in TO_STORAGE]
    if not legs:
        print("  nothing to do")
        return 1

    caps = {}
    for role in ("side", "wrist"):
        cap, spec, idx = cameras.open_role(role)
        caps[role] = (cap, spec)
    print(f"      {snap(caps, args.out, '00_before')}")

    print(f"  setting speed {args.dps} dps on {joints}")
    for j in joints:
        arm.send(f"SPD {j} {args.dps}")

    rc = 0
    for i, (label, moves) in enumerate(legs, 1):
        print(f"  phase {i}/{len(legs)}: {label}")
        if not drive(arm, caps, args.out, f"{i:02d}_{label}", moves, args.dps):
            rc = 1
            break

    final = arm.status()
    print("\n  final: " + "  ".join(
        f"J{j}=SET {final[j]['SET']} JTO {final[j]['JTO']}"
        for j in joints if j in final))
    print(f"      {snap(caps, args.out, '99_final')}")
    for cap, _ in caps.values():
        cap.release()

    want = PICK if args.to == "pick" else STORAGE
    reached = all(final.get(j, {}).get("SET") == str(v) for j, v in want.items())
    print(f"\n  {'REACHED' if reached else 'DID NOT REACH'} {args.to} "
          f"(commanded angles; nothing here observes a shaft)")
    print("  A board ack is not motion -- check the frames in " + args.out)
    return rc if rc else (0 if reached else 1)


if __name__ == "__main__":
    sys.exit(main())
