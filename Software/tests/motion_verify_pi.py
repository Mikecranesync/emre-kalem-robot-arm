"""Drive one joint through named waypoints and photograph every settle point.

WHY THIS EXISTS. A board ack proves the firmware accepted a command. It does not
prove a motor turned -- that distinction cost a whole afternoon on D3, where the
firmware provably drove the joint and nothing moved. This harness closes the loop
with a camera and refuses to call a move proven on the ack alone.

HOW IT TALKS TO THE ARM. It does NOT open the serial port. A holder daemon
(`hold_arm.py`) owns the board's serial device and must keep owning it -- the
watchdog latches and every joint detaches the moment nothing feeds it, and
closing the port can DTR-reset the board. So motion goes through the daemon's
file command channel: write a protocol line into `arm_cmd.txt`, the daemon sends
it, the reply lands in `arm_hold.log`. That directory is session-temporary, so it
is a required argument rather than a constant.

WHERE THE FRAMES COME FROM. Two sources, one interface:

  --camera <index>     a local capture device this process opens itself.
  --snapshot-url <url> an HTTP endpoint that hands back a JPEG.

The second exists because the Arduino now hangs off a Raspberry Pi, and on that
Pi `/dev/video0` is ALREADY OWNED by `~/arm/tools/mjpeg_preview.py`. A second
process opening the same device gets stale buffered frames -- which is
indistinguishable from "the joint did not move", the exact false null this
harness exists to stop. So the preview process stays the sole owner of the
camera and this harness asks it for a picture over HTTP.

SYNCHRONISATION. `arm_status.txt` is only rewritten on the daemon's 5 s poll, so
immediately after a MOV it still holds pre-move state -- which says MOV=0. Waiting
on that alone photographs mid-travel. Instead every command records
`arm_hold.log`'s byte length first and tails from that offset, which yields this
command's own reply synchronously.

WHAT IT REFUSES TO MEASURE. The daemon's health check re-sends `ENA <j> <adopt>`
after a latch, which snaps joints back to their adopt angles and reads exactly
like "the joint moved on its own". Any waypoint whose window contains `LATCHED`
or `re-ENA` is marked INVALID rather than measured. A waypoint whose two control
frames are byte-identical inside the ROI is likewise INVALID -- see below.

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

  # frames from the Pi's preview process instead of a local device
  python motion_verify.py --link <daemon-dir> --out <dir> --joint 6 --dps 12 \
      --snapshot-url http://127.0.0.1:8781/snapshot \
      --roi ... --waypoint closed:10 --waypoint open:70 ...
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import time
import urllib.error
import urllib.request

import cv2
import numpy as np

DIFF_THRESHOLD = 25  # grayscale levels; see docstring -- do not raise this
PASS_RATIO = 4.0  # signal must beat the same-waypoint noise floor by this much
#: A ratio alone is not enough. A freakishly quiet noise floor (2 px) turned a
#: 70 px edge-shimmer into a 35x "MOVED" on the first run of this harness, while
#: the frames showed no articulation at all. Both gates must pass.
MIN_SIGNAL_PX = 400
SETTLE_MARGIN_S = 1.4  # added to travel time before the joint is believed parked
SNAPSHOT_TIMEOUT_S = 6.0  # per-GET timeout on the --snapshot-url path

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
#: THEY ARE NOT CALIBRATED FOR THE PI CAMERA. Every number above, and every ROI in
#: the skill docs, was framed on the laptop's internal 1280x720 webcam. The
#: Arducam sits somewhere else and mjpeg_preview.py serves whatever size it
#: serves, so an old ROI on the new source produces numbers that look valid and
#: mean nothing. Re-frame the boxes and re-run the two-joint calibration (one
#: working joint, one known-broken) before believing a verdict from this source.
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
                return new.split(marker, 1)[1].strip()
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
    """A capture device this process opens itself. Windows bench path."""

    def __init__(self, index: int, out_dir: str):
        self.out = out_dir
        # CAP_DSHOW is the DirectShow backend and is Windows-only -- passing it on
        # Linux fails to open the device and lands in the isOpened() exit below,
        # which reads like "another app owns it" and is not. Windows behaviour is
        # unchanged; everywhere else OpenCV picks its own backend (V4L2 on the Pi).
        backend = cv2.CAP_DSHOW if os.name == "nt" else cv2.CAP_ANY
        self.cap = cv2.VideoCapture(index, backend)
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


class SnapshotCamera:
    """Frames over HTTP from a process that already owns the camera.

    WHY IT EXISTS. On the Pi, `~/arm/tools/mjpeg_preview.py` owns /dev/video0 and
    serves JPEGs at http://127.0.0.1:8781/snapshot. Opening the same device from
    here would hand this process stale buffered frames -- indistinguishable from
    "the joint did not move". One owner, and everyone else asks it.

    THERE IS DELIBERATELY NO DRAIN LOOP. The device path above reads ~6 frames and
    keeps the last because cv2.VideoCapture maintains a FIFO INSIDE THIS PROCESS,
    so the first read returns the past. HTTP has no such queue here: a GET returns
    whatever the server holds at that moment, and re-fetching drains nothing --
    it just costs round trips and would paper over a stalled server rather than
    expose it. Freshness is therefore ASSERTED, not assumed: main() hashes the
    decoded pixels inside the measurement ROI and voids any waypoint whose two
    control frames are identical. See the note on that check -- it is load
    bearing, because a zero noise floor removes the ratio gate entirely.
    """

    def __init__(self, url: str, out_dir: str):
        self.url = url
        self.out = out_dir
        self.frames = 0

    def _fetch(self) -> bytes:
        req = urllib.request.Request(self.url, headers={"Cache-Control": "no-cache"})
        try:
            with urllib.request.urlopen(req, timeout=SNAPSHOT_TIMEOUT_S) as resp:
                data = resp.read()
        except (urllib.error.URLError, OSError) as exc:
            raise SystemExit(f"snapshot GET failed: {self.url} -- {exc}")
        if not data:
            raise SystemExit(f"snapshot endpoint returned an empty body: {self.url}")
        return data

    def grab(self, name: str):
        data = self._fetch()
        frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise SystemExit(
                f"snapshot from {self.url} did not decode as an image "
                f"({len(data)} bytes) -- is that URL the JPEG endpoint?")
        # Written from the decoded array, so the evidence directory has the same
        # shape as the device path: one lossless PNG per named capture.
        cv2.imwrite(os.path.join(self.out, f"{name}.png"), frame)
        self.frames += 1
        return frame

    def release(self):
        # Nothing to release -- this process never owned the device. Defined
        # because main() calls release() in a finally block, and an AttributeError
        # raised there would mask whatever actually went wrong.
        pass


def crop(frame, roi):
    x0, y0, x1, y1 = roi
    return frame[y0:y1, x0:x1]


def check_roi(frame, roi, flag: str):
    """Refuse an ROI this camera's frames do not actually contain.

    numpy SILENTLY CLIPS an out-of-range slice. `frame[425:565, 330:485]` on a
    480-row image returns 55 rows, not 140 -- no exception, no warning, and every
    pixel count downstream is then measured over a different region than the one
    the thresholds were calibrated on. MIN_SIGNAL_PX would be mis-scaled and the
    verdict would still print with confidence. A fully off-frame ROI does raise,
    inside cvtColor; the partial clip is the silent one, so it is checked here.
    """
    x0, y0, x1, y1 = roi
    h, w = frame.shape[:2]
    if not (0 <= x0 < x1 <= w and 0 <= y0 < y1 <= h):
        raise SystemExit(
            f"{flag} {x0},{y0},{x1},{y1} does not fit inside a {w}x{h} frame from "
            f"this camera. Re-frame it against a probe capture -- a slice past the "
            f"edge is silently clipped, not an error. Every ROI in the skill docs "
            f"was calibrated on a 1280x720 laptop webcam.")


def roi_digest(frame, roi) -> str:
    """MD5 of the DECODED pixels inside the measurement ROI.

    NOT of the JPEG bytes. If the preview process draws a clock, an FPS counter or
    any overlay, every response differs by construction and a raw-byte check
    proves only that the server re-encoded something -- not that the sensor was
    re-read. Hashing the pixels the measurement actually consumes survives that,
    and works identically for the local-device path, where identical hashes are
    already the documented signature of another app holding the camera.
    """
    return hashlib.md5(crop(frame, roi).tobytes()).hexdigest()


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
    ap.add_argument("--camera", type=int, default=None,
                    help="local capture device index (default 0). Mutually exclusive "
                         "with --snapshot-url")
    ap.add_argument("--snapshot-url", default=None,
                    help="HTTP endpoint returning one JPEG per GET, e.g. "
                         "http://127.0.0.1:8781/snapshot . Use this when another "
                         "process already owns the camera")
    ap.add_argument("--label", default="run")
    ap.add_argument("--film", type=int, default=0,
                    help="capture up to N frames DURING each travel window")
    ap.add_argument("--geo-roi", default=None,
                    help="x0,y0,x1,y1 -- tighter box for dark-silhouette geometry")
    args = ap.parse_args()

    if args.snapshot_url and args.camera is not None:
        raise SystemExit("pass --camera OR --snapshot-url, not both -- two frame "
                         "sources cannot be compared against each other")

    # J1 SHOULDER IS EXCLUDED, DELIBERATELY -- this harness will not drive it.
    # Enabling J1 requires `MIR INV <offset>`, and mirror_offset_deg in
    # joint-limits.csv is still 0 and still UNMEASURED (0 is a placeholder, not a
    # measurement). If the true mirror axis is not 90, the two MG996R servos are
    # commanded in opposition and fight each other continuously -- they hold, run
    # hot, and strip a gear. This harness parks a joint at a waypoint for seconds
    # at a time, which is exactly the sustained-load condition that does the
    # damage. This is a refusal only: it cannot start, stop or move anything.
    if args.joint == 1:
        raise SystemExit(
            "refusing --joint 1 (shoulder). mirror_offset_deg is 0 and UNMEASURED, "
            "so the two MG996R may be fighting each other the whole time J1 is "
            "driven. Measure it first -- procedure is in the joint-limits.csv "
            "header block.")

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

    if args.snapshot_url:
        cam = SnapshotCamera(args.snapshot_url, args.out)
        source = args.snapshot_url
    else:
        cam = Camera(0 if args.camera is None else args.camera, args.out)
        source = f"device:{0 if args.camera is None else args.camera}"
    print(f"camera {source}")

    results, frames, labels = [], [], []
    prev_frame = None
    prev_label = None
    prev_geo = None
    cur = start_deg
    try:
        # PROBE BEFORE COMMANDING ANYTHING. One capture up front proves the source
        # answers and that the ROIs actually fit the frames it returns -- checked
        # here rather than after the first MOV so a mis-framed box is found while
        # the arm is still parked, not halfway through a run.
        probe = cam.grab(f"{args.label}_00_probe")
        print(f"probe  {probe.shape[1]}x{probe.shape[0]}")
        check_roi(probe, roi, "--roi")
        if geo_roi:
            check_roi(probe, geo_roi, "--geo-roi")

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
            # Wait on MOV=0 ALONE, and report the parked angle separately.
            # The old condition also required SET == deg, which a CLAMPED move can
            # never satisfy: the firmware parks at the limit, not at the request.
            # That spun the full 10 x 0.5 s and then recorded a parked_at nobody
            # had flagged as clamped. Stopping is one question ("is it still
            # moving?"); landing where asked is a different one, answered below.
            row = ""
            for _ in range(10):
                row = link.sta_row(jid)
                if link.field(row, "MOV") == "0":
                    break
                time.sleep(0.5)

            a = cam.grab(name + "_a")
            time.sleep(0.5)
            b = cam.grab(name + "_b")  # same command => this pair is the noise floor

            # IS THE CAPTURE ALIVE? Two frames half a second apart, of a real scene,
            # are never identical -- sensor noise alone guarantees it. Identical ROI
            # pixels therefore mean the source handed back the SAME frame twice: a
            # dead device, or a preview process whose capture loop has stalled or is
            # slower than this 0.5 s gap.
            #
            # This is not hygiene, it is a gate. Look at what a duplicate pair does
            # below: floor == 0, so `ratio` becomes inf and the >=4x noise-floor test
            # passes unconditionally. The run is then decided by MIN_SIGNAL_PX alone
            # -- and 519 px of whole-assembly sag has already cleared that bar for a
            # joint that provably does not articulate. A dead capture does not just
            # produce a null; it removes a gate and biases toward FALSE MOVED.
            dig_a, dig_b = roi_digest(a, roi), roi_digest(b, roi)
            capture_dead = dig_a == dig_b

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
            elif capture_dead:
                # Ahead of the baseline case on purpose: a dead capture at the
                # first waypoint poisons every comparison that follows it.
                verdict = "INVALID (capture dead -- identical frames)"
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
                # CLAMPED COMES OFF THE `MOV` REPLY, NOT OFF `STA`.
                # Firmware 1.1.1's doSta emits EN/SET/TGT/MI/MOV/JTO -- there is
                # no CL on a STA line, so the old `field(row, "CL")` read an
                # absent key and reported clamped=False for EVERY move, including
                # the clamped ones. `OK MOV J<n> REQ=<a> SET=<b> CL=<0|1>` is
                # where the firmware actually says it.
                "sta": row, "clamped": link.field(reply, "CL") == "1",
                "parked_as_asked": link.field(row, "SET") == str(deg),
                "jto": link.field(row, "JTO"),
                "parked_at": link.field(row, "SET"),
                "moving": link.field(row, "MOV"),
                "noise_floor_px": floor, "signal_px": signal,
                "in_travel_peak_px": in_travel,
                "silhouette": geo, "geometry_changed": geo_moved,
                "ratio": None if prev_frame is None else round(ratio, 2),
                "global_shift_px": round(shift, 2),
                "daemon_intervened": intervened,
                "roi_md5_a": dig_a, "roi_md5_b": dig_b,
                "capture_dead": capture_dead,
                "vs": prev_label,
                "verdict": verdict,
            }
            results.append(rec)
            print(f"  {label:<12} cmd={deg:<4} parked={rec['parked_at']:<4} "
                  f"MOV={rec['moving']} JTO={rec['jto']} "
                  f"signal={signal:<6} floor={floor:<5} ratio={rec['ratio']} "
                  f"travel_peak={in_travel:<6} shift={rec['global_shift_px']}px  "
                  f"{rec['verdict']}")
            if capture_dead:
                print(f"               ROI pixels identical across the control pair "
                      f"({dig_a[:8]}) -- the source repeated a frame. Nothing about "
                      f"the arm can be concluded from this waypoint.")
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
        "pass_ratio": PASS_RATIO, "camera_source": source, "waypoints": results,
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
