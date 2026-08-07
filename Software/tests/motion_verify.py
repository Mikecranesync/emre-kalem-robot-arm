"""Drive one joint through named waypoints and photograph every settle point.

WHY THIS EXISTS. A board ack proves the firmware accepted a command. It does not
prove a motor turned -- that distinction cost a whole afternoon on D3, where the
firmware provably drove the joint and nothing moved. This harness closes the loop
with a camera and refuses to call a move proven on the ack alone.

HOW IT TALKS TO THE ARM. It does NOT open the serial port. A holder daemon
(`hold_arm.py`) owns COM5 and must keep owning it -- the watchdog latches and
every joint detaches the moment nothing feeds it, and closing the port can
DTR-reset the board. So motion goes through the daemon's file command channel:
write a protocol line into `arm_cmd.txt`, the daemon sends it, the reply lands in
`arm_hold.log`. That directory is session-temporary, so it is a required argument
rather than a constant.

SYNCHRONISATION. `arm_status.txt` is only rewritten on the daemon's 5 s poll, so
immediately after a MOV it still holds pre-move state -- which says MOV=0. Waiting
on that alone photographs mid-travel. Instead every command records
`arm_hold.log`'s byte length first and tails from that offset, which yields this
command's own reply synchronously.

WHAT IT REFUSES TO MEASURE. The daemon's health check re-sends `ENA <j> <adopt>`
after a latch, which snaps joints back to their adopt angles and reads exactly
like "the joint moved on its own". Any waypoint whose window contains `LATCHED`
or `re-ENA` is marked INVALID rather than measured.

THE MEASUREMENT. Two frames are taken at every settle point at the SAME command.
Their difference is that waypoint's noise floor -- taken adjacent in time because
the arm settles downward slowly while held, so frames minutes apart are not
comparable. Signal is the previous waypoint's frame against this one, inside an
ROI that excludes everything that is not the joint (an operator standing in shot
will otherwise dominate the diff). A phase correlation separates "the joint moved"
from "the whole arm or the wire loom swung".

Thresholding is deliberately plain -- threshold 25, no morphological opening. A
threshold of 45 plus a 5x5 opening erased the thin edge bands a moving finger
produces and nearly got a real motion reported as a null result.

Usage:
  python motion_verify.py --link <daemon-dir> --out <dir> --joint 6 --dps 12 \
      --roi 330,425,485,565 --waypoint closed:10 --waypoint open:70 ...
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

DIFF_THRESHOLD = 25  # grayscale levels; see docstring -- do not raise this
PASS_RATIO = 4.0  # signal must beat the same-waypoint noise floor by this much
#: A ratio alone is not enough. A freakishly quiet noise floor (2 px) turned a
#: 70 px edge-shimmer into a 35x "MOVED" on the first run of this harness, while
#: the frames showed no articulation at all. Both gates must pass.
MIN_SIGNAL_PX = 400
SETTLE_MARGIN_S = 1.4  # added to travel time before the joint is believed parked

#: GEOMETRY GATE. Pixel gates answer "did anything change". For a gripper that is
#: the wrong question, and answering it got J6 reported as MOVED at 519x while its
#: fingers were provably not articulating -- the changed pixels were a 1 px sag of
#: the whole assembly. So when --geo-roi is given, a waypoint must ALSO show a
#: changed silhouette, or it is called NOT ARTICULATING however big the diff is.
#:
#: Thresholds are calibrated on 2026-08-06 against two joints in one session, one
#: working and one broken, same camera and ROI scale (see
#: Calibration_Notes/evidence/2026-08-06_motion-verify/). Per waypoint transition:
#:   J4, articulating:  bbox_h delta 3,4,5,5 px   span delta 1,1,4,4   area 0.3-2.3%
#:   J6, NOT:           bbox_h delta 1,1,1 px     span delta 0,1,1     area 0.6-1.2%
#: bbox_h separates cleanly with a px of slack either side; area alone does not.
#: RE-CALIBRATE these if the camera moves or the ROI scale changes -- they are a
#: bench heuristic, not a physical constant.
#:
#: KNOWN FALSE NEGATIVE, measured not guessed. A 15 deg J4 pitch at this camera
#: distance moves the silhouette by ~1 px, which is indistinguishable from J6's
#: sag, so some real J4 legs fail this gate (1 of 3 in the prong ROI, 1 of 3 in the
#: wrist ROI). The gate is therefore NOT tight enough to certify small-angle pitch,
#: and it is kept strict anyway because the two errors do not cost the same: a
#: false positive records a wrong claim as fact -- which is the failure this whole
#: harness exists to stop -- while a false negative costs a re-run, with the pixel
#: numbers printed on the same line to show it was marginal. A disagreement is
#: reported as DISAGREES, not as a diagnosis.
#:
#: ROI GOTCHA: bbox_h saturates when the ROI is filled edge to edge (the wrist box
#: pins it at 202 px regardless of pose), leaving med_span as the only live signal.
#: Frame the geo ROI on the articulating feature with white margin around it.
GEO_MIN_BBOX_H_DELTA = 2  # px
GEO_MIN_SPAN_DELTA = 3  # px
GEO_MIN_AREA_FRAC = 0.03  # 3%


class ArmLink:
    """Command channel to the holder daemon. Never opens the serial port."""

    def __init__(self, link_dir: str):
        self.cmd = os.path.join(link_dir, "arm_cmd.txt")
        self.log = os.path.join(link_dir, "arm_hold.log")
        if not os.path.exists(self.log):
            raise SystemExit(f"no daemon log at {self.log} -- is hold_arm.py running?")

    def offset(self) -> int:
        return os.path.getsize(self.log)

    def tail(self, offset: int) -> str:
        with open(self.log, "r", encoding="utf-8", errors="replace") as fh:
            fh.seek(offset)
            return fh.read()

    def send(self, line: str, timeout: float = 6.0) -> str:
        """Write one protocol line, return the daemon-logged reply for it."""
        off = self.offset()
        with open(self.cmd, "w", encoding="utf-8") as fh:
            fh.write(line + "\n")
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            new = self.tail(off)
            marker = f"CMD {line} -> "
            if marker in new:
                # CUT at the daemon's next timestamped line -- see reply_cut.py.
                # Without it, everything the daemon logs after this command (a
                # PNG heartbeat, the 5 s STA poll) comes back as part of THIS
                # reply, and a reply read mid-flush can be truncated before its
                # own CL= field. Observed live 2026-08-07.
                return cut_reply(new.split(marker, 1)[1])
            time.sleep(0.08)
        raise SystemExit(f"daemon never logged a reply for {line!r} in {timeout}s")

    def sta_row(self, jid: int) -> str:
        for ln in self.send("STA").splitlines():
            if ln.startswith(f"STA J{jid} "):
                return ln.strip()
        return ""

    @staticmethod
    def field(row: str, key: str) -> str:
        for tok in row.split():
            if tok.startswith(key + "="):
                return tok.split("=", 1)[1]
        return ""


class Camera:
    def __init__(self, index: int, out_dir: str):
        self.out = out_dir
        self.cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        if not self.cap.isOpened():
            raise SystemExit("camera did not open -- another app may own it")
        for _ in range(12):  # let exposure/gain settle
            self.cap.read()
            time.sleep(0.04)

    def grab(self, name: str):
        # Webcams buffer several frames. Read a handful and keep the last, or you
        # photograph the past -- which looks exactly like "it did not move".
        frame = None
        for _ in range(6):
            ok, fr = self.cap.read()
            if ok:
                frame = fr
            time.sleep(0.035)
        if frame is None:
            raise SystemExit("camera returned no frame")
        cv2.imwrite(os.path.join(self.out, f"{name}.png"), frame)
        return frame

    def release(self):
        self.cap.release()


def crop(frame, roi):
    x0, y0, x1, y1 = roi
    return frame[y0:y1, x0:x1]


def changed_px(a, b, roi) -> int:
    ga = cv2.cvtColor(crop(a, roi), cv2.COLOR_BGR2GRAY)
    gb = cv2.cvtColor(crop(b, roi), cv2.COLOR_BGR2GRAY)
    ga = cv2.GaussianBlur(ga, (3, 3), 0)
    gb = cv2.GaussianBlur(gb, (3, 3), 0)
    d = cv2.absdiff(ga, gb)
    return int((d > DIFF_THRESHOLD).sum())


def silhouette(frame, roi, dark_below=95):
    """Geometry of the dark part of an ROI, against the operator's white backdrop.

    Frame differencing answers "did anything change". This answers "what shape is
    it in", which is the question that matters for a gripper: two frames can differ
    because the whole assembly shifted while the fingers never moved. Area, height
    and per-column span are all illumination-robust in a way a diff is not, and it
    was this measure -- identical at commanded 10 and 70 -- that settled the J6
    question when the diff was still arguable.
    """
    x0, y0, x1, y1 = roi
    g = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
    m = (g < dark_below).astype(np.uint8)
    ys, xs = np.nonzero(m)
    if len(ys) == 0:
        return {"area": 0, "note": "no dark pixels -- joint outside the geo ROI"}
    spans = [np.ptp(np.nonzero(m[:, c])[0])
             for c in range(m.shape[1]) if len(np.nonzero(m[:, c])[0]) > 1]
    return {"area": int(m.sum()), "bbox_h": int(np.ptp(ys)), "bbox_w": int(np.ptp(xs)),
            "med_span": int(np.median(spans)) if spans else 0}


def geometry_changed(prev, cur):
    """True/False if the silhouette changed shape; None when it cannot be judged."""
    if not prev or not cur or "note" in prev or "note" in cur:
        return None
    d_h = abs(cur["bbox_h"] - prev["bbox_h"])
    d_span = abs(cur["med_span"] - prev["med_span"])
    d_area = abs(cur["area"] - prev["area"]) / max(prev["area"], 1)
    return (d_h >= GEO_MIN_BBOX_H_DELTA or d_span >= GEO_MIN_SPAN_DELTA
            or d_area >= GEO_MIN_AREA_FRAC)


def global_shift(a, b, roi) -> float:
    """Pixels of whole-ROI translation. Large => the arm/loom swung, not the joint."""
    ga = cv2.cvtColor(crop(a, roi), cv2.COLOR_BGR2GRAY).astype(np.float64)
    gb = cv2.cvtColor(crop(b, roi), cv2.COLOR_BGR2GRAY).astype(np.float64)
    win = cv2.createHanningWindow((ga.shape[1], ga.shape[0]), cv2.CV_64F)
    (dx, dy), _ = cv2.phaseCorrelate(ga, gb, win)
    return float((dx * dx + dy * dy) ** 0.5)


def strip(frames, labels, roi, path, scale=3):
    tiles = []
    for fr, lab in zip(frames, labels):
        t = cv2.resize(crop(fr, roi), None, fx=scale, fy=scale,
                       interpolation=cv2.INTER_NEAREST)
        bar = np.zeros((34, t.shape[1], 3), np.uint8)
        cv2.putText(bar, lab, (6, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                    (255, 255, 255), 2)
        tiles.append(np.vstack([bar, t]))
    cv2.imwrite(path, np.hstack(tiles))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--link", required=True, help="dir holding arm_cmd.txt / arm_hold.log")
    ap.add_argument("--out", required=True)
    ap.add_argument("--joint", type=int, required=True)
    ap.add_argument("--dps", type=float, required=True, help="joint speed, for settle timing")
    ap.add_argument("--roi", required=True, help="x0,y0,x1,y1")
    ap.add_argument("--waypoint", action="append", required=True, help="label:degrees")
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--label", default="run")
    ap.add_argument("--film", type=int, default=0,
                    help="capture up to N frames DURING each travel window")
    ap.add_argument("--geo-roi", default=None,
                    help="x0,y0,x1,y1 -- tighter box for dark-silhouette geometry")
    args = ap.parse_args()

    roi = tuple(int(v) for v in args.roi.split(","))
    geo_roi = tuple(int(v) for v in args.geo_roi.split(",")) if args.geo_roi else None
    waypoints = [(w.split(":")[0], int(w.split(":")[1])) for w in args.waypoint]
    os.makedirs(args.out, exist_ok=True)

    link = ArmLink(args.link)
    jid = args.joint

    row = link.sta_row(jid)
    if not row:
        raise SystemExit(f"no STA row for J{jid}")
    if link.field(row, "EN") != "1":
        raise SystemExit(f"J{jid} is not enabled: {row}")
    print(f"start  {row}")
    start_deg = int(link.field(row, "SET"))

    cam = Camera(args.camera, args.out)
    results, frames, labels = [], [], []
    prev_frame = None
    prev_label = None
    prev_geo = None
    cur = start_deg
    try:
        for idx, (label, deg) in enumerate(waypoints, 1):
            # Index the name: two waypoints with the same label used to overwrite
            # each other's evidence, silently leaving 3 files for 5 waypoints.
            name = f"{args.label}_{idx:02d}_{label}_{deg}"
            travel = abs(deg - cur) / max(args.dps, 1e-6)
            off = link.offset()

            reply = link.send(f"MOV {jid} {deg}")

            # FILM THE TRAVEL. A joint can move and come back, or move and be
            # dragged back by the daemon, and two settle-point frames cannot tell
            # that from never having moved. Frames captured across the travel
            # window can.
            film = []
            if args.film:
                t_end = time.monotonic() + travel + SETTLE_MARGIN_S
                while time.monotonic() < t_end and len(film) < args.film:
                    film.append(cam.grab(f"{name}_film{len(film):02d}"))
            else:
                time.sleep(travel + SETTLE_MARGIN_S)

            # Confirm parked where asked, from a reply this call solicited itself.
            row = ""
            for _ in range(10):
                row = link.sta_row(jid)
                if link.field(row, "MOV") == "0" and link.field(row, "SET") == str(deg):
                    break
                time.sleep(0.5)

            a = cam.grab(name + "_a")
            time.sleep(0.5)
            b = cam.grab(name + "_b")  # same command => this pair is the noise floor

            # Did the daemon intervene inside this window?
            window = link.tail(off)
            intervened = ("LATCHED" in window) or ("re-ENA" in window)

            floor = changed_px(a, b, roi)
            signal = changed_px(prev_frame, a, roi) if prev_frame is not None else 0
            shift = global_shift(prev_frame, a, roi) if prev_frame is not None else 0.0
            ratio = (signal / floor) if floor else float("inf")
            # Peak change seen at any point DURING travel, against the frame the
            # joint started from. Catches motion that does not survive to the
            # settle point.
            in_travel = max((changed_px(a, f, roi) for f in film), default=0)

            geo = silhouette(a, geo_roi) if geo_roi else None
            geo_moved = geometry_changed(prev_geo, geo)
            pixels_pass = ratio >= PASS_RATIO and signal >= MIN_SIGNAL_PX
            if intervened:
                verdict = "INVALID (daemon re-enabled mid-waypoint)"
            elif prev_frame is None:
                verdict = "baseline"
            elif not pixels_pass:
                verdict = "NOT RESOLVED"
            elif geo_moved is False:
                # The diff passed and the shape did not: the two measures DISAGREE.
                # Deliberately not asserted as "not articulating" -- see the
                # asymmetry note on the geometry gate above. Refuse to certify and
                # make the disagreement visible; the pixel numbers are on the same
                # line for whoever reads it.
                verdict = "DISAGREES (pixels moved, shape unchanged)"
            else:
                verdict = "MOVED"

            rec = {
                "label": label, "deg": deg, "from": cur, "reply": reply,
                "sta": row, "clamped": link.field(row, "CL") == "1",
                "jto": link.field(row, "JTO"),
                "parked_at": link.field(row, "SET"),
                "moving": link.field(row, "MOV"),
                "noise_floor_px": floor, "signal_px": signal,
                "in_travel_peak_px": in_travel,
                "silhouette": geo, "geometry_changed": geo_moved,
                "ratio": None if prev_frame is None else round(ratio, 2),
                "global_shift_px": round(shift, 2),
                "daemon_intervened": intervened,
                "vs": prev_label,
                "verdict": verdict,
            }
            results.append(rec)
            print(f"  {label:<12} cmd={deg:<4} parked={rec['parked_at']:<4} "
                  f"MOV={rec['moving']} JTO={rec['jto']} "
                  f"signal={signal:<6} floor={floor:<5} ratio={rec['ratio']} "
                  f"travel_peak={in_travel:<6} shift={rec['global_shift_px']}px  "
                  f"{rec['verdict']}")
            if rec["silhouette"]:
                print(f"               geo {rec['silhouette']} changed={geo_moved}")

            frames.append(a)
            labels.append(f"{label} {deg}deg")
            prev_frame, prev_label, cur, prev_geo = a, f"{label}:{deg}", deg, geo
    finally:
        cam.release()

    if frames:
        strip(frames, labels, roi, os.path.join(args.out, f"{args.label}_strip.png"))
    summary = {
        "joint": jid, "dps": args.dps, "roi": roi, "threshold": DIFF_THRESHOLD,
        "pass_ratio": PASS_RATIO, "waypoints": results,
    }
    with open(os.path.join(args.out, f"{args.label}_result.json"), "w") as fh:
        json.dump(summary, fh, indent=2)

    graded = [r for r in results if r["verdict"] not in ("baseline",)]
    bad = [r for r in graded if r["verdict"] != "MOVED"]
    gate = (f">={PASS_RATIO}x noise floor, >={MIN_SIGNAL_PX} px"
            + (" AND a changed silhouette" if geo_roi else ""))
    print(f"\n{len(graded) - len(bad)}/{len(graded)} transitions resolved as motion "
          f"({gate})")
    if bad:
        print("NOT PROVEN:", ", ".join(f"{r['label']}({r['verdict']})" for r in bad))
    raise SystemExit(1 if bad else 0)


if __name__ == "__main__":
    main()
