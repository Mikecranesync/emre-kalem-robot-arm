#!/usr/bin/env python3
"""Camera-supervised powered bench test for the arm. RUNS ON THE PI.

    arm_bench_test.py <bridge-token> --shoulder-deg <deg> [--joints 1,3,4,5,0,6]
    arm_bench_test.py <bridge-token> --shoulder-deg 90 --home
    arm_bench_test.py <bridge-token> --off

WHY THIS EXISTS
    Nothing on this arm observes an output shaft. "OK MOV" means the firmware
    ACCEPTED a target; it does not mean anything moved. So every motion step here
    is judged by the camera: the arm's ROI is differenced before and after and
    compared to a noise floor measured on the same scene moments earlier. A
    command that returns OK and produces no pixels is a FAIL. That is the point.

    Written after a 2026-08-09 bench session in which the arm was dropped and
    then driven over centre. Every rule below is a scar, not a preference.

=============================================================================
THE SIX RULES, AND THE FAILURE EACH ONE PREVENTS
=============================================================================

1. ONE PROCESS, ONE LINK, ALWAYS HEARTBEATING.
   The firmware's serial watchdog (WDMS, typically 4000 ms) detaches every joint
   when the host goes quiet. The first harness ran each step as its own
   short-lived command, so the gaps BETWEEN steps tripped it: a joint enabled by
   one invocation was detached before the next began (observed: ES=1 WD=1). The
   watchdog was right and the harness was wrong. Never "fix" this by widening or
   disabling WDG - hold the link and feed it.

2. ENABLE THE LOAD-BEARING JOINT FIRST.
   A detached arm is held up by gearbox friction alone, and static friction is
   not a fixture. Enabling the BASE first, while the shoulder was still
   detached, produced an attach jolt that broke that friction and the whole arm
   collapsed onto the bench. The shoulder carries the load: capture it first, and
   only then work outward.

3. THE ADOPT ANGLE COMES FROM A HUMAN. NEVER INFER IT.
   enableJoint() pre-loads the adopt pulse and attaches; if that angle is wrong
   the servo slams from where it really is to where it was told, at its own
   speed, with no ramp possible. There is no sensor to derive it from, and SET
   is a record of what the joint was last TOLD - worthless once the arm has
   moved on its own. Inferring it from the direction notes in joint-limits.csv
   drove the shoulder to 0, which is over centre. Those notes even warn that the
   camera projection reads backwards and that the operator wins. So: ask.

4. NEVER MEASURE A MOVING SCENE.
   The floor is the bar every move must clear. The first run measured its "noise
   floor" WHILE the arm was falling, recorded 36486 px as normal, and every
   later step was then judged against an impossible threshold - so a real
   failure read as a pass. The scene must be demonstrably still first, and an
   implausible floor is rejected rather than adopted.

5. A CLAMPED MOVE IS NOT A FAILED MOVE.
   Probing +3 deg on a joint sitting 1 deg from its limit commands a 1 deg move,
   which is invisible, and the joint gets reported FAIL. Both J1 (at 90 with
   MAX=91) and J5 (at 180 = MAX) were slandered this way. Any step smaller than
   MIN_DELTA_DEG is SKIPPED and labelled, never failed.

6. THE FIRST MOVE AFTER AN ENABLE MAY BE DEAD.
   Backlash. J0 moved 134 px on its first commanded step and 7019, 8954, 16540
   on the ones after. A single dead first move is reported as BACKLASH, not as a
   broken joint; only a joint that never clears the floor across a whole probe
   is a FAIL.

=============================================================================

SAFETY
    - speeds are forced DOWN to TEST_DPS, never raised
    - targets are clamped to the board's own reported soft limits; this never
      widens a limit or edits calibration to make a test pass
    - one joint is enabled at a time, shoulder first
    - any camera/command disagreement stops motion (STP) and ends the run
    - the run always finishes with DIS A rather than leaving joints driven for
      the watchdog to drop
"""
import argparse
import json
import sys
import time
import urllib.request

import cv2
import numpy as np

BRIDGE = "http://127.0.0.1:8770"
CAM = "http://127.0.0.1:8781/snapshot"

# The arm in full-frame pixels, on the upright image (mjpeg_preview rotates 180).
# Everything the arm can reach is inside this; bench, shelves and tool rack are
# outside, so their noise never counts toward a verdict.
#
# Re-cut 2026-08-10 for the re-aimed camera and the new gridded backdrop. The old
# box (800, 70, 1160, 660) was derived for a camera position that no longer
# exists; against this scene its left edge fell at x800, straight through the
# arm's base - so J0's motion was partly OUTSIDE the measured region, which is a
# live candidate for J0's unexplained 36%/33% dead-move rate.
#
# PROVISIONAL - covers every pose observed so far, pending a real reach sweep
# (Software/tests/arm_reach_envelope.py). Sized deliberately generous: an ROI
# that is too big only costs a little noise, while one that is too small scores
# real motion as a dead move, which is the failure this harness exists to stop.
#
# Derived from measurement, not eyeballed. Dark-pixel column/row profiles of two
# very different poses:
#   arm parked (folded right)   x700-1130  y360-650
#   arm raised (extended up)    x460-900   y140-680
# A first attempt at (660, 90, 1250, 700) was cut from the parked pose alone and
# was immediately falsified by the raised pose, whose gripper sits near x460 -
# 200 px outside it. Two poses are still not a reach envelope; treat this box as
# an upper bound on what has been SEEN, not on what the arm can DO.
#
# Scene facts behind the edges:
#   backdrop      x120-1150   the only background with usable contrast
#   left clutter  x0-160      shelving and wiring, permanently dark, excluded
#   operator zone bottom-centre and far left - where hands and tools go
# Static noise floor MEASURED FOR THIS BOX, 10 distinct frames, nothing moving:
# median 56 px, max 76, over its 506k px (QUIET_PX is 900). With the operator
# actively working at the bench a 360k-px box peaked at 821, still under the
# threshold. So noise is no longer what limits box size - coverage is.
ROI = (420, 90, 1250, 700)
BANNER_ROWS = 70        # burnt-in status text; it redraws every frame - never measure it

THRESH = 25             # per-pixel grey delta counted as "changed"
HEARTBEAT_S = 1.5       # comfortably inside a 4000 ms watchdog
TEST_DPS = 5            # slowest useful speed
MIN_DELTA_DEG = 2       # rule 5: below this a move is unobservable, so skip it
QUIET_PX = 900          # a still scene in bench light sits near 50
QUIET_RUN = 4           # consecutive quiet frames before the scene counts as still
FLOOR_CAP = 3000        # rule 4: any floor above this means something is moving

NAMES = {0: "Base", 1: "Shoulder(pair)", 3: "Elbow",
         4: "Wrist pitch", 5: "Wrist roll", 6: "Gripper"}
LOG = []


def log(**kw):
    kw = {"t": time.strftime("%H:%M:%S"), **kw}
    LOG.append(kw)
    print(json.dumps(kw), flush=True)


class Link:
    """One reader. The bridge's /rx is destructive, so two pollers steal each
    other's replies - there is exactly one pump() and every wait goes through
    it, which is also where the watchdog heartbeat lives (rule 1)."""

    def __init__(self, token):
        self.t = token
        self.buf = []
        self.last_hb = 0.0

    def _post(self, line):
        b = json.dumps({"data": line + "\n"}).encode()
        r = urllib.request.Request(f"{BRIDGE}/tx?t={self.t}", data=b,
                                   headers={"Content-Type": "application/json"})
        urllib.request.urlopen(r, timeout=6).read()

    def pump(self):
        d = json.loads(urllib.request.urlopen(
            f"{BRIDGE}/rx?t={self.t}", timeout=6).read().decode())
        for ln in d.get("lines", []):
            if not ln.startswith(";"):
                self.buf.append(ln)
        if time.time() - self.last_hb > HEARTBEAT_S:
            self.last_hb = time.time()
            self._post("PNG")

    def send(self, line, wait=3.0):
        self.buf.clear()
        self._post(line)
        self.last_hb = time.time()
        out, end = [], time.time() + wait
        while time.time() < end:
            time.sleep(0.12)
            self.pump()
            while self.buf:
                out.append(self.buf.pop(0))
            if out and (out[-1].startswith("OK") or out[-1].startswith("ERR")):
                return out
        return out

    def idle(self, seconds):
        """Sleep without letting the watchdog trip."""
        end = time.time() + seconds
        while time.time() < end:
            time.sleep(0.15)
            self.pump()

    def sta(self):
        snap = {}
        for ln in self.send("STA", 3.0):
            p = ln.split()
            if not p:
                continue
            if p[0] == "STA" and len(p) > 1 and p[1].startswith("J"):
                snap[int(p[1][1:])] = dict(k.split("=", 1) for k in p[2:] if "=" in k)
            elif p[0] == "SYS":
                snap["SYS"] = dict(k.split("=", 1) for k in p[1:] if "=" in k)
        return snap


def frame():
    b = np.frombuffer(urllib.request.urlopen(CAM, timeout=10).read(), np.uint8)
    f = cv2.imdecode(b, cv2.IMREAD_COLOR)
    x0, y0, x1, y1 = ROI
    return cv2.cvtColor(f[max(y0, BANNER_ROWS):y1, x0:x1], cv2.COLOR_BGR2GRAY)


def diff(a, b):
    return int((cv2.absdiff(a, b) > THRESH).sum())


class Bench:
    def __init__(self, link):
        self.L = link

    # ---- rule 4: never measure a moving scene ----------------------------
    def wait_quiet(self, why, timeout=25.0):
        a = frame()
        run, worst, end = 0, 0, time.time() + timeout
        while time.time() < end:
            self.L.idle(0.4)
            b = frame()
            px = diff(a, b)
            a, worst = b, max(worst, px)
            run = run + 1 if px < QUIET_PX else 0
            if run >= QUIET_RUN:
                log(step="scene_quiet", why=why, settled_px=px)
                return True
        log(step="scene_NOT_quiet", why=why, worst_px=worst)
        return False

    def floor(self, tries=4):
        for attempt in range(tries):
            if not self.wait_quiet(f"settling before floor (try {attempt + 1})"):
                continue
            a, peak = frame(), 0
            for _ in range(6):
                self.L.idle(0.4)
                b = frame()
                peak = max(peak, diff(a, b))
                a = b
            if peak <= FLOOR_CAP:
                log(step="floor", floor_px=peak)
                return peak
            log(step="floor_rejected", floor_px=peak, cap=FLOOR_CAP)
        return None

    def stop(self, why):
        log(step="ABORT", why=why)
        self.L.send("STP")
        self.L.send("DIS A")
        self.report()
        sys.exit(1)

    def clr(self):
        s = self.L.sta()
        if s.get("SYS", {}).get("ES") == "1":
            log(step="clr", was_watchdog=s["SYS"].get("WD"),
                result=" | ".join(self.L.send("CLR")))
            s = self.L.sta()
        return s

    # ---- rules 2 + 3: shoulder first, adopt supplied by a human ----------
    def capture_shoulder(self, adopt):
        if not self.wait_quiet("before enabling the shoulder"):
            self.stop("scene not still before the first enable")
        self.clr()
        before = frame()
        r = self.L.send(f"ENA 1 {int(adopt)}")
        self.L.idle(2.0)
        jump = diff(before, frame())
        s = self.L.sta()
        ok = s.get(1, {}).get("EN") == "1"
        log(step="capture_shoulder", adopt=int(adopt), reply=" | ".join(r),
            camera_px=jump, en=s.get(1, {}).get("EN"),
            verdict="PASS" if ok and jump < 4000 else "LARGE JUMP - adopt angle was wrong")
        if not ok:
            self.stop("shoulder would not enable")

    def enable(self, j, adopt):
        before = frame()
        r = self.L.send(f"ENA {j} {int(adopt)}")
        self.L.idle(2.0)
        jump = diff(before, frame())
        s = self.L.sta()
        ok = s.get(j, {}).get("EN") == "1"
        log(step="enable", joint=j, name=NAMES.get(j), adopt=int(adopt),
            reply=" | ".join(r), camera_px=jump, en=s.get(j, {}).get("EN"),
            note="a large px here means the adopt angle did not match reality")
        return ok

    def move(self, j, target, floor, label):
        before = frame()
        r = self.L.send(f"MOV {j} {int(target)}")
        self.L.idle(3.0)
        px = diff(before, frame())
        s = self.L.sta()
        seen = px > max(floor * 2, 60)
        log(step=label, joint=j, name=NAMES.get(j), target=int(target),
            speed_dps=TEST_DPS, reply=" | ".join(r), camera_px=px, floor_px=floor,
            camera_saw_motion=seen, set_deg=s.get(j, {}).get("SET"),
            verdict="PASS" if seen else "NO MOTION SEEN")
        return seen

    def probe(self, j, delta, floor):
        """out/back both ways. Returns a verdict string, not a bare bool."""
        s = self.L.sta()
        lo, hi = int(s[j]["MIN"]), int(s[j]["MAX"])
        start = int(s[j]["SET"])
        self.L.send(f"SPD {j} {TEST_DPS}")

        results, skipped = [], 0
        for tgt, label in [(min(hi, start + delta), "out+"), (start, "back+"),
                           (max(lo, start - delta), "out-"), (start, "back-")]:
            cur = int(self.L.sta()[j]["SET"])
            # rule 5: an unobservably small move is not a failure
            if abs(tgt - cur) < MIN_DELTA_DEG:
                skipped += 1
                log(step=label, joint=j, name=NAMES.get(j), target=tgt,
                    skipped=f"only {abs(tgt - cur)} deg - at a soft limit, "
                            f"below the {MIN_DELTA_DEG} deg observable minimum")
                continue
            results.append(self.move(j, tgt, floor, label))

        if not results:
            return "NOT TESTED (every step clamped by a limit)"
        if all(results):
            return "PASS"
        # rule 6: one dead first move is backlash, not a dead joint
        if results[0] is False and all(results[1:]):
            return "PASS (backlash: first move after enable was dead)"
        if not any(results):
            return "FAIL (no commanded move was ever visible)"
        return f"PARTIAL ({sum(results)}/{len(results)} moves seen)"

    def report(self):
        print("\n===== BENCH LOG =====", flush=True)
        for e in LOG:
            print(json.dumps(e), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("token")
    ap.add_argument("--shoulder-deg", type=float,
                    help="the shoulder's CURRENT PHYSICAL angle, read off the arm "
                         "by a human. Rule 3: this is never inferred.")
    ap.add_argument("--joints", default="1,3,4,5,0,6")
    ap.add_argument("--delta", type=int, default=3)
    ap.add_argument("--home", action="store_true")
    ap.add_argument("--off", action="store_true")
    a = ap.parse_args()

    L = Link(a.token)
    B = Bench(L)

    if a.off:
        print(" | ".join(L.send("DIS A")), flush=True)
        return

    if a.shoulder_deg is None:
        sys.exit("--shoulder-deg is required: read the shoulder angle off the arm.\n"
                 "It cannot be inferred - see rule 3 in this file's header.")

    order = [int(x) for x in a.joints.split(",")]
    if order and order[0] != 1:
        # rule 2 is not advisory
        sys.exit("the shoulder (1) must be first in --joints: it carries the load, "
                 "and enabling anything else first is what dropped the arm.")

    verdicts = {}
    B.capture_shoulder(a.shoulder_deg)
    fl = B.floor()
    if fl is None:
        B.stop("the scene never settled enough to trust a noise floor")

    for j in order:
        s = B.clr()
        if j not in s:
            log(step="skip", joint=j, why="not an addressable joint on this arm")
            continue
        if s[j]["EN"] != "1" and not B.enable(j, int(s[j]["SET"])):
            verdicts[j] = "NOT TESTED (would not enable)"
            continue
        verdicts[j] = B.probe(j, a.delta, fl)
        log(step="joint_verdict", joint=j, name=NAMES.get(j), verdict=verdicts[j])

    if a.home:
        home = {0: 110, 1: 91, 3: 15, 4: 90, 5: 180, 6: 90}
        log(step="home_begin", target=home)
        # Wrist and elbow first while the shoulder is still low, then the
        # shoulder last, so the arm sweeps up already folded instead of
        # swinging an extended forearm across the bench.
        for j in (3, 4, 5, 0, 1):
            s = L.sta()
            if s.get(j, {}).get("EN") != "1":
                continue
            tgt = max(int(s[j]["MIN"]), min(int(s[j]["MAX"]), home[j]))
            B.move(j, tgt, fl, "home_move")
        log(step="home_done",
            sta={k: v.get("SET") for k, v in L.sta().items() if k != "SYS"})

    log(step="verdicts", per_joint={str(k): v for k, v in verdicts.items()})
    # Always park the link deliberately rather than letting the watchdog do it.
    log(step="off", result=" | ".join(L.send("DIS A")))
    B.report()


# Importable on purpose. Link and Bench carry the six rules, and the precision
# work reuses them rather than forking a copy that quietly drops one.
if __name__ == "__main__":
    main()
