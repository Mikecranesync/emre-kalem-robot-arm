#!/usr/bin/env python3
"""Find the cameras, prove they open, and prove they run at the same time.

WHY THIS EXISTS. The dual-vision plan needs three USB devices alive at once --
the integrated camera (side view), the Arducam (wrist view), and the Arduino
Uno. Every one of those has already bitten this project in a different way:

  * A camera index is NOT an identity. Windows hands `cv2.VideoCapture(0)`
    whichever device enumerated first, and that order changes when something is
    replugged or a hub wakes up. Pinning a config to an index is how the wrist
    camera silently becomes the side camera. This tool resolves index -> real
    device and writes the answer down.
  * A frozen capture looks EXACTLY like a working one. `cap.read()` returns
    ok=True with the same bytes forever when another application owns the
    device. On 2026-08-06 the holder was `Microsoft.WindowsCamera` -- the
    operator had opened the Camera app to aim the laptop at the arm. So every
    probe here grabs SEVERAL frames and hashes them: identical hashes mean the
    capture is dead, not that the scene is still.
  * Uncompressed video eats the bus. 1280x720 at 30 fps in YUY2 is about
    440 Mbit/s, and USB 2.0 tops out near 480 Mbit/s of theoretical bandwidth --
    one camera can starve the other. MJPEG is roughly a tenth of that. This tool
    reports the FOURCC actually in force after asking for MJPG, because asking
    is not the same as getting.
  * `CAP_DSHOW` is required on this machine. Repo note, already paid for.

WHAT IT DOES NOT DO. It does not calibrate anything, it does not detect a
marker, and it never opens the serial port. It answers four questions: what
cameras are here, which OpenCV index is which, what each will actually give you,
and do they survive being opened together.

Usage:
  python camera_probe.py --list                 devices + index mapping
  python camera_probe.py --probe 1              one index in detail
  python camera_probe.py --both 0 1 --out DIR   open both, grab from each
  python camera_probe.py --grab 1 --out DIR     one frame from one index
  python camera_probe.py --preview 1 --out DIR  live window for AIMING the camera
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import time

import cv2

# Resolutions asked for, largest first. The Arducam is a 12 MP part and the
# integrated camera silently refuses anything above 1280x720 (repo note,
# MARKER-SYSTEM.md section 7) -- which is exactly what makes them tellable
# apart without trusting an index.
LADDER = [
    (4032, 3024),
    (3840, 2160),
    (1920, 1080),
    (1280, 720),
    (640, 480),
]

# Known devices on this bench. VID:PID is the only stable identity Windows
# offers without a full device-path capture API.
KNOWN = {
    "0C40:0304": "Arducam-708-12MP-HDR  (wrist camera)",
    "174F:2469": "Integrated Camera     (side view / laptop lid)",
}


def pnp_cameras() -> list[dict]:
    """Ask Windows what camera-class devices exist. Identity, not capability."""
    ps = (
        "Get-PnpDevice -Class Camera -Status OK -ErrorAction SilentlyContinue "
        "| Select-Object FriendlyName,InstanceId | ConvertTo-Json -Compress"
    )
    try:
        out = subprocess.run(
            ["powershell", "-NoProfile", "-Command", ps],
            capture_output=True, text=True, timeout=30,
        ).stdout.strip()
        data = json.loads(out) if out else []
    except Exception as exc:                      # noqa: BLE001 - report, never raise
        print(f"  (PnP query failed: {exc})")
        return []
    if isinstance(data, dict):
        data = [data]
    rows = []
    for d in data:
        inst = d.get("InstanceId", "")
        vid = pid = ""
        for tok in inst.replace("\\", "&").split("&"):
            if tok.startswith("VID_"):
                vid = tok[4:]
            elif tok.startswith("PID_"):
                pid = tok[4:]
        rows.append({
            "name": d.get("FriendlyName", "?"),
            "vidpid": f"{vid}:{pid}" if vid else "",
            "instance": inst,
        })
    return rows


def fourcc_str(cap) -> str:
    v = int(cap.get(cv2.CAP_PROP_FOURCC))
    if v <= 0:
        return "----"
    return "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4))


def frame_hash(frame) -> str:
    return hashlib.md5(frame.tobytes()).hexdigest()[:12]


def probe(index: int, deep: bool = False) -> dict | None:
    """Open one index and find out what it really is. None if it will not open."""
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap.release()
        return None

    info: dict = {"index": index}
    info["default_wh"] = (
        int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
    )
    info["default_fourcc"] = fourcc_str(cap)

    # Ask for MJPEG. Asking is not getting -- read it back.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    info["mjpg_fourcc"] = fourcc_str(cap)
    info["mjpg_accepted"] = info["mjpg_fourcc"].upper().startswith("MJP")

    # Largest resolution the device will actually hand back, which is the
    # discriminator between a 12 MP module and a 720p lid camera.
    best = None
    for w, h in (LADDER if deep else LADDER[:4]):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        got = (
            int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        )
        if got == (w, h):
            best = got
            break
    info["max_wh"] = best or info["default_wh"]

    # Settle at 720p for the liveness check -- that is the working resolution
    # for both cameras and it keeps the bus quiet while the other one is open.
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    info["working_wh"] = (
        int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
    )
    info["fps_reported"] = round(cap.get(cv2.CAP_PROP_FPS), 1)

    # LIVENESS. Several frames, hashed. Webcams also buffer, so the first few
    # reads can be stale -- discard them before timing anything.
    for _ in range(5):
        cap.read()
    hashes, t0, n = [], time.monotonic(), 0
    for _ in range(10):
        ok, frame = cap.read()
        if ok and frame is not None:
            hashes.append(frame_hash(frame))
            n += 1
        time.sleep(0.02)
    elapsed = time.monotonic() - t0
    cap.release()

    info["frames_read"] = n
    info["distinct_frames"] = len(set(hashes))
    info["fps_measured"] = round(n / elapsed, 1) if elapsed > 0 else 0.0
    # One distinct hash across ten reads is a dead capture, not a still scene.
    info["live"] = info["distinct_frames"] > 1
    return info


def identify(info: dict, devices: list[dict]) -> str:
    """Map a probed index onto a real device by capability, never by order."""
    w, h = info["max_wh"]
    if w * h >= 3840 * 2160:
        want = "0C40:0304"
    elif (w, h) == (1280, 720):
        want = "174F:2469"
    else:
        return "unidentified"
    for d in devices:
        if d["vidpid"] == want:
            return f"{d['name']}  [{want}]"
    return KNOWN.get(want, "unidentified")


def cmd_list(args) -> int:
    devices = pnp_cameras()
    print("WINDOWS SEES")
    for d in devices:
        tag = KNOWN.get(d["vidpid"], "")
        print(f"  {d['vidpid'] or '??:??':10}  {d['name']:26} {tag}")
    if not devices:
        print("  (none)")

    print("\nOPENCV INDEX MAPPING  (CAP_DSHOW)")
    print(f"  {'idx':<4} {'max':>11} {'working':>10} {'fourcc':>7} {'fps':>6} "
          f"{'live':>5}  device")
    found = []
    for i in range(args.max_index + 1):
        info = probe(i)
        if info is None:
            continue
        who = identify(info, devices)
        found.append((i, who, info))
        mx = f"{info['max_wh'][0]}x{info['max_wh'][1]}"
        wk = f"{info['working_wh'][0]}x{info['working_wh'][1]}"
        live = "yes" if info["live"] else "DEAD"
        print(f"  {i:<4} {mx:>11} {wk:>10} {info['mjpg_fourcc']:>7} "
              f"{info['fps_measured']:>6} {live:>5}  {who}")
    if not found:
        print("  (no index opened -- another application may own the devices)")
        return 1

    dead = [i for i, _, inf in found if not inf["live"]]
    if dead:
        print(f"\n  WARNING: index {dead} returned identical frames. That is a"
              " dead capture,\n  not a still scene. Something else owns the"
              " device -- see arm-bench-safety section 9.")
    return 0


def cmd_probe(args) -> int:
    devices = pnp_cameras()
    info = probe(args.probe, deep=True)
    if info is None:
        print(f"index {args.probe}: will not open")
        return 1
    info["device"] = identify(info, devices)
    for k, v in info.items():
        print(f"  {k:18} {v}")
    return 0


def _grab_to(index: int, out_dir: str, label: str) -> tuple[bool, str]:
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        return False, f"index {index}: will not open"
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    frame = None
    for _ in range(8):                 # drain the buffer or you photograph the past
        ok, f = cap.read()
        if ok:
            frame = f
        time.sleep(0.03)
    cap.release()
    if frame is None:
        return False, f"index {index}: opened but returned no frame"
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"{label}.png")
    cv2.imwrite(path, frame)
    return True, f"{path}  {frame.shape[1]}x{frame.shape[0]}  hash={frame_hash(frame)}"


def cmd_grab(args) -> int:
    ok, msg = _grab_to(args.grab, args.out, f"cam{args.grab}")
    print(("  " if ok else "  FAILED ") + msg)
    return 0 if ok else 1


def cmd_both(args) -> int:
    """The question the whole dual-vision plan rests on: both at once, or not."""
    a, b = args.both
    print(f"opening index {a} and index {b} together, both 1280x720 MJPEG")
    caps = {}
    for i in (a, b):
        c = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if not c.isOpened():
            print(f"  index {i}: WILL NOT OPEN while the other is up")
            for x in caps.values():
                x.release()
            return 1
        c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        c.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        c.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        caps[i] = c
        print(f"  index {i}: open, fourcc={fourcc_str(c)}")

    for c in caps.values():
        for _ in range(5):
            c.read()

    stats = {i: {"n": 0, "hashes": []} for i in caps}
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0:          # three seconds of both, interleaved
        for i, c in caps.items():
            ok, frame = c.read()
            if ok and frame is not None:
                stats[i]["n"] += 1
                stats[i]["hashes"].append(frame_hash(frame))
    elapsed = time.monotonic() - t0

    for i, c in caps.items():
        ok, frame = c.read()
        if ok and frame is not None and args.out:
            os.makedirs(args.out, exist_ok=True)
            p = os.path.join(args.out, f"both_cam{i}.png")
            cv2.imwrite(p, frame)
            print(f"  index {i}: saved {p}")
        c.release()

    print(f"\n  {'idx':<4} {'frames':>7} {'fps':>6} {'distinct':>9}  verdict")
    all_ok = True
    for i, s in stats.items():
        fps = round(s["n"] / elapsed, 1)
        distinct = len(set(s["hashes"]))
        good = s["n"] > 0 and distinct > 1
        all_ok &= good
        print(f"  {i:<4} {s['n']:>7} {fps:>6} {distinct:>9}  "
              f"{'live' if good else 'DEAD / frozen'}")
    print("\n  BOTH CAMERAS RUN TOGETHER" if all_ok else
          "\n  THEY DO NOT BOTH RUN -- see the per-index verdict above")
    return 0 if all_ok else 1


def cmd_preview(args) -> int:
    """A live window, for pointing a camera at something by hand.

    This exists because aiming is a physical loop -- move the camera, look,
    move it again -- and a still frame captured after the fact is the slowest
    possible way to run it. The overlay carries the numbers that matter while
    aiming: what the device actually settled on, the measured frame rate, and
    how bright the scene is, because both of these cameras go dim indoors and
    a marker that is too dark will not be detected at all.

    's' saves a snapshot, 'q' or ESC quits. --seconds caps the run so this can
    never become a window nobody closes.
    """
    idx = args.preview
    cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"index {idx}: will not open")
        return 1
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cc = fourcc_str(cap)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    win = f"cam{idx}  {w}x{h}  {cc}   [s]=save  [q]=quit"
    print(f"  index {idx}: {w}x{h} {cc} -- window open, 's' saves, 'q' quits, "
          f"{args.seconds}s cap")
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, min(w, 1280), min(h, 720))

    saved, n, t0, last = 0, 0, time.monotonic(), time.monotonic()
    fps = 0.0
    while time.monotonic() - t0 < args.seconds:
        ok, frame = cap.read()
        if not ok or frame is None:
            continue
        n += 1
        now = time.monotonic()
        if now - last >= 0.5:
            fps = n / (now - t0)
            last = now

        view = frame.copy()
        # Mean luma, as a plain aiming aid. Marker detection wants a scene that
        # is not in the dark; this is the cheapest honest indicator of that.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        luma = float(gray.mean())
        ch, cw = view.shape[0] // 2, view.shape[1] // 2
        cv2.line(view, (cw - 30, ch), (cw + 30, ch), (0, 255, 0), 1)
        cv2.line(view, (cw, ch - 30), (cw, ch + 30), (0, 255, 0), 1)
        txt = f"cam{idx} {w}x{h} {cc}  {fps:4.1f} fps  luma {luma:5.1f}"
        if luma < 40:
            txt += "  -- DARK, markers may not detect"
        cv2.putText(view, txt, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(view, txt, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0), 1, cv2.LINE_AA)
        cv2.imshow(win, view)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("s") and args.out:
            os.makedirs(args.out, exist_ok=True)
            p = os.path.join(args.out, f"cam{idx}_aim_{saved:02d}.png")
            cv2.imwrite(p, frame)
            print(f"  saved {p}")
            saved += 1

    cap.release()
    cv2.destroyAllWindows()
    print(f"  closed. {n} frames, {fps:.1f} fps, {saved} snapshot(s)")
    return 0


def cmd_props(args) -> int:
    """Dump the properties this device will actually report, for tuning later."""
    idx = args.props
    cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"index {idx}: will not open")
        return 1
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    names = [
        "FRAME_WIDTH", "FRAME_HEIGHT", "FPS", "BRIGHTNESS", "CONTRAST",
        "SATURATION", "HUE", "GAIN", "EXPOSURE", "AUTO_EXPOSURE", "GAMMA",
        "SHARPNESS", "BACKLIGHT", "AUTOFOCUS", "FOCUS", "ZOOM",
        "AUTO_WB", "WB_TEMPERATURE", "BUFFERSIZE",
    ]
    print(f"  index {idx}  fourcc={fourcc_str(cap)}")
    for nm in names:
        prop = getattr(cv2, f"CAP_PROP_{nm}", None)
        if prop is None:
            continue
        v = cap.get(prop)
        # -1 is DSHOW's "this device does not expose that"
        print(f"    {nm:18} {v}" + ("   (not supported)" if v == -1 else ""))
    cap.release()
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--list", action="store_true", help="devices + index mapping")
    g.add_argument("--probe", type=int, metavar="IDX", help="one index in detail")
    g.add_argument("--grab", type=int, metavar="IDX", help="save one frame")
    g.add_argument("--both", type=int, nargs=2, metavar=("A", "B"),
                   help="open two indices at once and measure both")
    g.add_argument("--preview", type=int, metavar="IDX",
                   help="live window for aiming the camera by hand")
    g.add_argument("--props", type=int, metavar="IDX",
                   help="dump the device's adjustable properties")
    p.add_argument("--out", default="", help="output directory for frames")
    p.add_argument("--max-index", type=int, default=3,
                   help="highest OpenCV index to try (default 3)")
    p.add_argument("--seconds", type=float, default=120.0,
                   help="preview time cap in seconds (default 120)")
    p.add_argument("--width", type=int, default=1280, help="preview width")
    p.add_argument("--height", type=int, default=720, help="preview height")
    args = p.parse_args()

    if args.list:
        return cmd_list(args)
    if args.probe is not None:
        return cmd_probe(args)
    if args.props is not None:
        return cmd_props(args)
    if args.preview is not None:
        return cmd_preview(args)
    if args.grab is not None:
        if not args.out:
            p.error("--grab needs --out DIR")
        return cmd_grab(args)
    return cmd_both(args)


if __name__ == "__main__":
    sys.exit(main())
