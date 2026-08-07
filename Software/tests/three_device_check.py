#!/usr/bin/env python3
"""Both cameras and the board, alive at the same time. Read-only on the wire.

WHY THIS EXISTS. The dual-vision plan needs three USB devices up together: the
integrated camera (side view), the Arducam (wrist view), and the Arduino Uno.
Each has been measured alone. Nothing had measured them TOGETHER, and "it
should be fine" is not a measurement -- especially when one of the cameras
refuses MJPEG and streams uncompressed.

WHAT IT PROVES, AND WHAT IT CANNOT
  Proves: the three devices enumerate and stream/answer concurrently on this
  host, at a measured frame rate and a measured serial round-trip, for a
  measured duration. Frames are hashed, so a frozen capture cannot pass as a
  working one.
  Cannot prove: anything about a motor, a shaft, or a joint. No joint is
  enabled here and none can be -- see the verb allowlist below.

THIS SCRIPT NEVER COMMANDS A JOINT. It sends exactly two verbs, VER and STA,
both read-only, and refuses anything else at the point of writing. It is not a
control path, and it must never grow into one.

  * ENA drives a joint to its adopt angle. Not here.
  * MOV, JOG, SPD, LIM change state. Not here.
  * EST, DIS and the watchdog DETACH every joint, and a gravity-loaded arm
    FALLS. Those are not safety commands and they are not in the allowlist.

Nothing in software is an emergency stop on this arm. The rocker switch and the
fuse are the only real stop. If the arm is energised while this runs, a human
is standing at the bench -- that is a precondition, not a feature of the script.

OPENING THE PORT RESETS THE BOARD. DTR toggles on open, the firmware restarts,
and it keeps NOTHING: limits go back to the 70-110 default, MIR=UNKNOWN, CAL=0,
every joint detached. That is expected and harmless with no joint enabled, but
it means this check must not be run while a holder daemon is up -- the daemon
owns the port, this will fail to open, and that failure is the correct outcome.

Usage:
  python three_device_check.py --seconds 10
  python three_device_check.py --seconds 10 --no-serial     cameras only
  python three_device_check.py --cams 0 1 --port COM5 --out DIR
"""

from __future__ import annotations

import argparse
import hashlib
import statistics
import sys
import time

import cv2
import serial

# The only verbs this script may put on the wire. Both are read-only: VER
# reports firmware identity, STA reports state and takes no arguments (STA J0
# is ERR E2). Adding to this list makes this a control path, which it must not
# become -- write a new tool instead.
ALLOWED_VERBS = ("VER", "STA")


def guard(line: str) -> str:
    """Refuse anything outside the read-only allowlist, before it is written."""
    verb = line.strip().split()[0].upper() if line.strip() else ""
    if verb not in ALLOWED_VERBS:
        raise ValueError(
            f"refusing to send {verb!r}: this script is read-only and may send "
            f"only {ALLOWED_VERBS}. It does not enable, move or detach a joint."
        )
    return line


def fourcc_str(cap) -> str:
    v = int(cap.get(cv2.CAP_PROP_FOURCC))
    return "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4)) if v > 0 else "----"


def open_cam(index: int, w: int, h: int):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    for _ in range(5):                     # drain the buffer before measuring
        cap.read()
    return cap


def read_reply(ser, timeout=1.5, quiet=0.12) -> str:
    """Drain a whole reply, including a multi-line one.

    THE BUG THIS FIXES, because it was in here and it printed nonsense. The
    first version broke out the moment `in_waiting` hit zero after any data had
    arrived. A `STA` reply is eight lines and arrives in bursts at 115200, so a
    momentary gap mid-burst ended the read: the printed reply was truncated at
    `STA J1 EN` and the leftovers surfaced at the FRONT of the next one, which
    is why a stray `TO=0` appeared above the `SYS` line and made the whole block
    look scrambled.

    A truncated reply that still contains recognisable text is the dangerous
    shape -- it reads as success. So: require `quiet` seconds of genuine silence
    after the last byte before calling a reply complete, and stop early only on
    a line that can only be the end of one.
    """
    out, t0, last_byte = b"", time.monotonic(), None
    while time.monotonic() - t0 < timeout:
        n = ser.in_waiting
        if n:
            out += ser.read(n)
            last_byte = time.monotonic()
            # A single-line ack ends at its own newline; a STA block ends at the
            # last joint row. Either way, wait out `quiet` before believing it.
            continue
        if last_byte is not None and time.monotonic() - last_byte >= quiet:
            break
        time.sleep(0.01)
    return out.decode("ascii", "replace").strip()


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--cams", type=int, nargs=2, default=[0, 1],
                   metavar=("SIDE", "WRIST"))
    p.add_argument("--port", default="COM5")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--no-serial", action="store_true",
                   help="cameras only; do not open the port at all")
    p.add_argument("--out", default="", help="save one frame per camera here")
    args = p.parse_args()

    print("THREE-DEVICE CHECK -- read-only on the wire, no joint is enabled\n")

    caps = {}
    for role, idx in zip(("side", "wrist"), args.cams):
        cap = open_cam(idx, args.width, args.height)
        if cap is None:
            print(f"  {role:5} index {idx}: WILL NOT OPEN")
            for c in caps.values():
                c[0].release()
            return 1
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"  {role:5} index {idx}: {w}x{h} {fourcc_str(cap)}")
        caps[role] = (cap, idx)

    ser = None
    if not args.no_serial:
        try:
            print(f"\n  opening {args.port} -- THIS DTR-RESETS THE BOARD; the "
                  f"firmware keeps nothing")
            ser = serial.Serial(args.port, args.baud, timeout=0.3)
            time.sleep(2.2)                # Optiboot window; section 9 mandates it
            ser.reset_input_buffer()
            ser.write((guard("VER") + "\n").encode("ascii"))
            ser.flush()
            print(f"  VER -> {read_reply(ser) or '(no reply)'}")
        except Exception as exc:                       # noqa: BLE001
            print(f"  {args.port}: {exc}")
            print("  (a holder daemon or the console may own the port -- that is"
                  " the correct\n   outcome if one is running; stop it or pass"
                  " --no-serial)")
            for c, _ in caps.values():
                c.release()
            return 1

    stats = {r: {"n": 0, "h": []} for r in caps}
    sta_ok, sta_fail, rtts = 0, 0, []
    t0 = last_sta = time.monotonic()

    while time.monotonic() - t0 < args.seconds:
        for role, (cap, _) in caps.items():
            ok, frame = cap.read()
            if ok and frame is not None:
                stats[role]["n"] += 1
                stats[role]["h"].append(
                    hashlib.md5(frame.tobytes()).hexdigest()[:12])
        # Poll the board while both cameras stream. This is the interleaving
        # that matters: a serial round-trip competing with two video streams.
        if ser is not None and time.monotonic() - last_sta > 0.5:
            last_sta = time.monotonic()
            ser.reset_input_buffer()
            s0 = time.monotonic()
            ser.write((guard("STA") + "\n").encode("ascii"))
            ser.flush()
            reply = read_reply(ser)
            rtts.append((time.monotonic() - s0) * 1000.0)
            if reply.startswith("OK STA") or "STA J" in reply:
                sta_ok += 1
            else:
                sta_fail += 1

    elapsed = time.monotonic() - t0
    last_reply = ""
    if ser is not None:
        ser.reset_input_buffer()
        ser.write((guard("STA") + "\n").encode("ascii"))
        ser.flush()
        last_reply = read_reply(ser)
        ser.close()

    for role, (cap, idx) in caps.items():
        if args.out:
            ok, frame = cap.read()
            if ok and frame is not None:
                import os
                os.makedirs(args.out, exist_ok=True)
                path = os.path.join(args.out, f"three_{role}_cam{idx}.png")
                cv2.imwrite(path, frame)
                print(f"\n  saved {path}")
        cap.release()

    print(f"\n  {'role':6} {'idx':>4} {'frames':>7} {'fps':>6} {'distinct':>9}"
          f"  verdict")
    all_live = True
    for role, (cap, idx) in caps.items():
        s = stats[role]
        distinct = len(set(s["h"]))
        live = s["n"] > 0 and distinct > 1
        all_live &= live
        print(f"  {role:6} {idx:>4} {s['n']:>7} {s['n']/elapsed:>6.1f} "
              f"{distinct:>9}  {'live' if live else 'DEAD / frozen'}")

    serial_ok = True
    if ser is not None:
        serial_ok = sta_ok > 0 and sta_fail == 0
        print(f"\n  serial  {args.port}  STA ok={sta_ok} fail={sta_fail}  "
              f"rtt min/median/max = "
              f"{min(rtts):.0f}/{statistics.median(rtts):.0f}/{max(rtts):.0f} ms"
              if rtts else f"\n  serial  {args.port}  no round-trips attempted")
        if last_reply:
            print("\n  last STA (read-only; every joint should read EN=0 after a"
                  " reset):")
            for ln in last_reply.splitlines():
                print(f"    {ln}")

    ok = all_live and serial_ok
    print("\n  ALL THREE DEVICES RAN TOGETHER" if ok else
          "\n  NOT ALL DEVICES HELD UP -- see the verdicts above")
    print("  This says nothing about a motor. No joint was enabled.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
