#!/usr/bin/env python3
"""Open a camera BY ROLE, with every per-open setting re-applied.

WHY THIS MODULE EXISTS AT ALL. Every setting that makes these cameras usable --
the pixel format, the resolution, autofocus off, the pinned focus value -- is
forgotten by DirectShow the moment the capture closes. `cameras.csv` is
therefore not a record of how the cameras are configured; it is a list of things
that MUST be re-applied on every single open. A setting a human has to remember
is a setting that will be forgotten, so this module does it instead.

THE FAILURE IT EXISTS TO PREVENT. An OpenCV index is not an identity. Windows
hands `VideoCapture(0)` whichever device enumerated first, and that order
changes on a replug. Pin a config to an index and one day the WRIST camera is
the SIDE camera -- and a visual servo loop that believes it is looking at the
gripper while it is actually looking at the room will drive the arm somewhere
nobody asked for. So this module resolves a role to an index by CAPABILITY
PROBE, at open time, every time. If it cannot resolve the role it RAISES. It
never falls back to the index hint, because opening the wrong camera silently
is the whole thing being avoided.

NO SERIAL PORT IS OPENED ANYWHERE IN THIS FILE, and none should ever be. The
board has exactly one owner (hold_arm.py) and a blocking serial read in the same
thread as camera capture measurably costs frames -- 29.9 fps fell to 22.3 when
that was tried on 2026-08-07.

Usage:
    from cameras import open_role, read

    cap, spec, idx = open_role("wrist")
    ok, frame = read(cap, spec)        # rotation applied
    ...
    cap.release()

    python cameras.py                  # resolve both roles and report
"""

from __future__ import annotations

import csv
import os
import time
from dataclasses import dataclass

import cv2

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cameras.csv")

# The capability fingerprint that separates these two parts. The Arducam is a
# 12 MP module and offers 4K; the laptop lid camera refuses anything above
# 1280x720. THIS IS A HEURISTIC THAT FITS EXACTLY TWO CAMERAS, not a law -- if a
# third arrives that also does 4K, this stops working and must be replaced with
# a real device-path lookup. It fails loudly rather than quietly: an unresolved
# role raises.
_FOURK = (3840, 2160)


@dataclass(frozen=True)
class CameraSpec:
    role: str
    vid_pid: str
    friendly_name: str
    index_hint: int
    width: int
    height: int
    fourcc: str
    rotation_deg: int
    autofocus: str
    focus: int
    focus_set_at: str
    intrinsics: str
    notes: str


def load_specs(path: str = CONFIG) -> dict[str, CameraSpec]:
    """Read cameras.csv. Comment lines start with '#', as everywhere in this repo."""
    with open(path, "r", encoding="utf-8") as fh:
        rows = [ln for ln in fh if ln.strip() and not ln.lstrip().startswith("#")]
    specs = {}
    for r in csv.DictReader(rows):
        specs[r["role"]] = CameraSpec(
            role=r["role"],
            vid_pid=r["vid_pid"],
            friendly_name=r["friendly_name"],
            index_hint=int(r["index_hint"]),
            width=int(r["width"]),
            height=int(r["height"]),
            fourcc=r["fourcc"],
            rotation_deg=int(r["rotation_deg"]),
            autofocus=r["autofocus"],
            focus=int(r["focus"]),
            focus_set_at=r["focus_set_at"],
            intrinsics=r["intrinsics"],
            notes=r["notes"],
        )
    return specs


def _max_resolution(cap) -> tuple[int, int]:
    """What the device will actually hand back. Ask big, read what you got."""
    for w, h in (_FOURK, (1920, 1080), (1280, 720), (640, 480)):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
               int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if got == (w, h):
            return got
    return (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))


def resolve_index(spec: CameraSpec, max_index: int = 4) -> int:
    """Find the OpenCV index for this role by probing, hint first.

    Raises if it cannot be resolved. That is deliberate: returning the hint
    unverified would reintroduce exactly the silent-wrong-camera failure this
    module exists to stop.
    """
    wants_4k = spec.vid_pid == "0C40:0304"
    order = [spec.index_hint] + [i for i in range(max_index + 1)
                                 if i != spec.index_hint]
    for idx in order:
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap.release()
            continue
        mx = _max_resolution(cap)
        cap.release()
        is_4k = mx[0] * mx[1] >= _FOURK[0] * _FOURK[1]
        if is_4k == wants_4k:
            return idx
    raise RuntimeError(
        f"cannot resolve camera role {spec.role!r} ({spec.friendly_name}, "
        f"{spec.vid_pid}). Probed indices 0..{max_index} and none matched its "
        f"capability signature. Refusing to open an unverified index -- opening "
        f"the wrong camera silently is worse than not opening one. Check the "
        f"device is plugged in and that nothing else owns it."
    )


def warmup(cap, max_seconds: float = 3.0, settle_frac: float = 0.06) -> bool:
    """Read frames until exposure has actually ramped. Returns False on timeout.

    THE BUG THIS FIXES, because it produced a frame that lied. The first version
    drained five frames with no delay after opening, then handed the capture
    back. `cap.read()` returned ok=True with the correct 1280x720 shape -- and
    the image was BLACK. The sensor's auto-exposure had not ramped yet, so the
    call succeeded and the picture was worthless. That is the same shape as
    every other trap in this project: a result that reports success and contains
    nothing. An arriving frame is not an exposed frame.

    Draining a bigger fixed number would be a magic constant that is wrong on
    the next camera. So this measures instead: read until mean luma stops
    changing by more than `settle_frac` between consecutive samples, bounded by
    `max_seconds`. A genuinely dark scene settles dark and settles quickly --
    that is a true reading, not a timeout.
    """
    prev, t0 = None, time.monotonic()
    while time.monotonic() - t0 < max_seconds:
        ok, frame = cap.read()
        if not ok or frame is None:
            time.sleep(0.03)
            continue
        luma = float(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).mean())
        if prev is not None:
            # Settled when consecutive means agree. The +1 keeps a near-black
            # scene from dividing by ~0 and declaring instant success.
            if abs(luma - prev) / (prev + 1.0) < settle_frac and luma > 1.0:
                return True
        prev = luma
        time.sleep(0.03)
    return False


def open_role(role: str, path: str = CONFIG, verify: bool = True):
    """Open the camera for a role and apply every per-open setting.

    Returns (cap, spec, index). Caller releases. Raises rather than guessing.

    THE INDEX IS RETURNED RATHER THAN RE-DERIVABLE ON DEMAND, and that is not a
    convenience. These cameras are single-owner: calling resolve_index() again
    while the capture is open probes the same device, and the probe breaks the
    live capture -- the first version of main() did exactly that and every
    read() came back empty while both roles still "resolved" correctly. The
    resolution looked fine and the frames were gone. Ask once, keep the answer.
    """
    specs = load_specs(path)
    if role not in specs:
        raise KeyError(f"no camera role {role!r} in {path}; have {sorted(specs)}")
    spec = specs[role]
    idx = resolve_index(spec)

    cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise RuntimeError(f"role {role!r} resolved to index {idx} but it will "
                           f"not open -- another application may own it")

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*spec.fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, spec.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, spec.height)

    # Focus, when the device has it. Order matters: autofocus off first, or the
    # lens fights the manual value.
    if spec.autofocus == "off":
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        time.sleep(0.1)
    if spec.focus >= 0:
        cap.set(cv2.CAP_PROP_FOCUS, spec.focus)
        time.sleep(0.1)

    warmed = warmup(cap)

    if verify:
        if not warmed:
            cap.release()
            raise RuntimeError(
                f"role {role!r}: exposure never settled within the warm-up "
                f"window. The capture would return ok=True on a black frame, "
                f"which reads exactly like a working one -- refusing to hand "
                f"that back.")
        got_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        got_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if (got_w, got_h) != (spec.width, spec.height):
            cap.release()
            raise RuntimeError(
                f"role {role!r}: asked for {spec.width}x{spec.height}, got "
                f"{got_w}x{got_h}. Refusing to hand back a capture that is not "
                f"what the config says it is.")
        if spec.focus >= 0:
            got_f = cap.get(cv2.CAP_PROP_FOCUS)
            if abs(got_f - spec.focus) >= 2:
                cap.release()
                raise RuntimeError(
                    f"role {role!r}: focus {spec.focus} did not stick "
                    f"(read back {got_f:.0f}). Intrinsics are only valid at the "
                    f"focus they were measured at, so this is not ignorable.")
    return cap, spec, idx


def orient(frame, spec: CameraSpec):
    """Apply the configured rotation. The wrist camera is mounted upside down."""
    r = spec.rotation_deg % 360
    if r == 0:
        return frame
    if r == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if r == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    if r == 270:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    raise ValueError(f"rotation_deg must be 0/90/180/270, got {spec.rotation_deg}")


def read(cap, spec: CameraSpec):
    """cap.read() with the rotation applied. Use this, not cap.read()."""
    ok, frame = cap.read()
    if not ok or frame is None:
        return False, None
    return True, orient(frame, spec)


def main() -> int:
    import argparse
    ap = argparse.ArgumentParser(description="resolve and open every camera role")
    ap.add_argument("--save", default="", metavar="DIR",
                    help="write one oriented frame per role into DIR")
    args = ap.parse_args()

    specs = load_specs()
    print(f"{'role':6} {'vid:pid':12} {'idx':>4} {'size':>10} {'fmt':>5} "
          f"{'rot':>4} {'focus':>6}  device")
    rc = 0
    for role, spec in specs.items():
        try:
            cap, s, idx = open_role(role)
        except Exception as exc:                       # noqa: BLE001
            print(f"{role:6} {spec.vid_pid:12} {'--':>4}  FAILED: {exc}")
            rc = 1
            continue
        ok, frame = read(cap, s)
        shape = f"{frame.shape[1]}x{frame.shape[0]}" if ok else "no frame"
        extra = ""
        if ok and args.save:
            os.makedirs(args.save, exist_ok=True)
            path = os.path.join(args.save, f"{role}.png")
            cv2.imwrite(path, frame)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # Luma and sharpness reported together: a frame can be well exposed
            # and out of focus, and only one of those is visible in a thumbnail.
            extra = (f"  luma {gray.mean():5.1f}  "
                     f"sharp {cv2.Laplacian(gray, cv2.CV_64F).var():7.1f}")
        cap.release()
        print(f"{role:6} {s.vid_pid:12} {idx:>4} {shape:>10} {s.fourcc:>5} "
              f"{s.rotation_deg:>4} {s.focus:>6}  {s.friendly_name}{extra}")
        if not ok:
            rc = 1
    print("\nintrinsics: " + ", ".join(f"{r}={s.intrinsics}"
                                       for r, s in specs.items())
          + "   <- no camera is calibrated; do not print marker stickers yet")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
