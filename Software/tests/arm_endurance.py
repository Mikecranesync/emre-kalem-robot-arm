#!/usr/bin/env python3
"""Overnight endurance / burn-in run for the arm. RUNS ON THE PI, UNATTENDED.

    arm_endurance.py <token> <shoulder_deg> --hours 8 [--outdir DIR]
    arm_endurance.py <token> <shoulder_deg> --preflight-only

Industrial commissioning burn-in, not a demo. The objective is to make the arm
expose weaknesses a short bench test cannot: drift, degradation, intermittent
faults, weak directions, settling changes over hundreds of cycles.

=============================================================================
TWO KINDS OF TELEMETRY, NEVER CONFLATED
=============================================================================
CONTROLLER telemetry - SET, TGT, EN, ES, WD, firmware replies, round-trip time.
    This is what we COMMANDED and what the firmware believes. SET is a commanded
    angle. It is never evidence of where the arm physically is.
PHYSICAL telemetry - changed pixels, settle time, ROI centroid, brightness.
    This is what the camera actually saw.

Every record carries both, tagged. Nothing here converts pixels into degrees or
millimetres: this camera cannot support that claim, and Phase 2 already produced
one set of discarded numbers by trying.

=============================================================================
SAFE WORKSPACE - PROVEN, NOT ASSUMED
=============================================================================
Individually legal joint angles do NOT make a collision-free whole-arm pose, and
there is no self-collision model. So the tour is built only from bands this arm
has already physically traversed today, pulled back from every endpoint:

    J0 100..125   (drove 100-130)      J1  78..89   (drove 71-91)
    J3   5..25    (drove 0-30)         J4  85..105  (drove 82-110)
    J5 165..178   (drove 160-180)

J1 and J5 are deliberately kept off their MAX (91 and 180): a joint parked on its
own limit has nowhere to back off to, and it already cost Phase 2 its J1
repeatability measurement.

J6 (gripper) is EXCLUDED. Its one-way mechanical fault is known and thousands of
cycles would either mask it or make it worse.

=============================================================================
SAFETY - EVERY EXISTING MECHANISM KEPT
=============================================================================
- one heartbeated process, one command in flight (watchdog stays at its
  configured WDMS; it is never widened or disabled)
- shoulder enabled FIRST, at an operator-supplied adopt angle, never inferred
- soft limits are read from the board and every target is clamped to them; this
  code never widens a limit
- reserved J2 is never addressed; the mirror relationship is never touched
- acceleration-limited reversal stays in firmware and is not bypassed
- ALWAYS finishes with a controlled park and DIS A - on normal exit, on abort,
  on SIGTERM/SIGINT, and on an unhandled exception
- an enable is only ever re-attempted where measured sag proves the physical
  position is still known
"""
import argparse
import hashlib
import json
import os
import signal
import statistics
import sys
import time
import traceback

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, "/tmp")
from arm_bench_test import Link, Bench, frame, diff, NAMES  # noqa: E402

# Bands physically traversed today. See module docstring.
SAFE = {0: (100, 125), 1: (78, 89), 3: (5, 25), 4: (85, 105), 5: (165, 178)}
JOINTS = [1, 3, 4, 5, 0]          # shoulder first, always
REF_POSE = {0: 112, 1: 85, 3: 15, 4: 95, 5: 172}

# Conservative and well below anything visibly violent.
SPEEDS = [5, 10, 20]
ACCELS = {1: [120, 200], 3: [120, 200], 4: [40, 60], 5: [40, 60], 0: [120, 200]}

# Beyond this the adopt angle was badly wrong. RESCALED 2026-08-10 from 25000
# to 40000 - not to get past a gate that fired, but because the ROI it is
# measured through changed underneath it and the number stopped meaning what it
# meant. Same joint, same adopt angle, same physical sag-and-snap:
#     old ROI (212k px, clipped the arm's base): 15715, 17844, 13241  -> passed
#     new ROI (506k px, sees the whole arm):     25994, 25217, 25533  -> failed
# Ratio 1.63x, measured across six runs, with the operator confirming by eye
# that the elbow really was at 15 both nights. 25000 x 1.63 = 40625; 40000 keeps
# a little of the original margin. NOT scaled by ROI area (2.38x) - the arm's
# motion is a fixed physical thing, so a bigger box only adds the parts of the
# arm the old box was cutting off, which is 1.63x here and not 2.38x.
# If the ROI moves again, re-measure this the same way. It is an absolute pixel
# count against a specific box, and it silently lies the moment that box changes.
MAX_ENABLE_JUMP_PX = 40000
MAX_SAG_PX_FOR_REENABLE = 1500    # above this the physical position is NOT known
SETTLE_TIMEOUT_S = 20.0
NO_MOTION_STRIKES = 4             # consecutive dead commanded moves -> abort


class Runner:
    def __init__(self, token, shoulder, outdir, hours):
        self.L = Link(token)
        self.B = Bench(self.L)
        self.shoulder = shoulder
        self.outdir = outdir
        self.deadline = time.time() + hours * 3600.0
        os.makedirs(outdir, exist_ok=True)
        os.makedirs(os.path.join(outdir, "img"), exist_ok=True)
        self.run_id = time.strftime("%Y%m%d-%H%M%S")
        self.f = open(os.path.join(outdir, f"endurance-{self.run_id}.jsonl"), "a")
        self.cycle = 0
        self.moves = 0
        self.physical_moves = 0
        self.no_motion_streak = 0
        self.counts = {j: {"moves": 0, "dead": 0, "errors": 0} for j in JOINTS}
        self.events = []
        self.roi = {}
        self.floor = {}
        self.stopped = False
        self.abort_reason = None
        self.cam_stale_streak = 0
        self.cam_stale_total = 0

    # ---------------------------------------------------------------- log
    def rec(self, kind, **kw):
        e = {"ts": time.time(), "t": time.strftime("%H:%M:%S"), "run": self.run_id,
             "cycle": self.cycle, "kind": kind, **kw}
        self.f.write(json.dumps(e) + "\n")
        self.f.flush()
        return e

    def snap(self, tag):
        """Keep periodic and anomaly images only - not every frame all night."""
        try:
            import urllib.request
            b = urllib.request.urlopen("http://127.0.0.1:8781/snapshot", timeout=10).read()
            p = os.path.join(self.outdir, "img", f"{self.run_id}-{int(time.time())}-{tag}.jpg")
            with open(p, "wb") as fh:
                fh.write(b)
            return p
        except Exception as exc:
            self.rec("camera_fail", where="snap", err=str(exc))
            return None

    # ------------------------------------------------------------ physical
    def roi_frame(self, j):
        f = frame()
        r = self.roi.get(j)
        if not r:
            return f
        return f[r[1]:r[3], r[0]:r[2]]

    @staticmethod
    def sig(arr):
        """Fingerprint of one frame, to catch a camera handing back stale data."""
        return hashlib.md5(arr.tobytes()).hexdigest()[:12]

    def brightness(self):
        try:
            return round(float(frame().mean()), 1)
        except Exception:
            return None

    def measure_floor(self, j, n=5):
        for _ in range(3):
            if not self.B.wait_quiet(f"floor J{j}"):
                continue
            a, pk = self.roi_frame(j), 0
            for _ in range(n):
                self.L.idle(0.4)
                b = self.roi_frame(j)
                pk = max(pk, diff(a, b))
                a = b
            r = self.roi.get(j)
            area = (r[3] - r[1]) * (r[2] - r[0]) if r else 800 * 600
            if pk <= max(80, area * 0.02):
                self.floor[j] = pk
                return pk
        self.floor[j] = self.floor.get(j, 400)
        self.rec("floor_unsettled", joint=j, using=self.floor[j])
        return self.floor[j]

    # ------------------------------------------------------------- motion
    def clamp(self, j, v, s=None):
        s = s or self.L.sta()
        lo, hi = int(s[j]["MIN"]), int(s[j]["MAX"])
        band = SAFE.get(j)
        if band:                      # never leave the proven band either
            lo, hi = max(lo, band[0]), min(hi, band[1])
        return max(lo, min(hi, int(v)))

    def move(self, j, target, spd, acc, test, extra=None):
        """One camera-supervised move with full controller + physical telemetry."""
        s0 = self.L.sta()
        if s0.get(j, {}).get("EN") != "1":
            self.rec("skip", joint=j, why="not enabled", test=test)
            return None
        start = int(s0[j]["SET"])
        target = self.clamp(j, target, s0)
        if abs(target - start) < 1:
            return None

        self.L.send(f"SPD {j} {spd}")
        self.L.send(f"ACC {j} {acc}")
        floor = self.floor.get(j, 400)

        before = self.roi_frame(j)
        t_issue = time.time()
        reply = self.L.send(f"MOV {j} {int(target)}")
        rtt = time.time() - t_issue
        ok = any(x.startswith("OK") for x in reply)
        err = [x for x in reply if x.startswith("ERR")]

        # Physical: when motion first appears, and when the scene goes quiet.
        #
        # The first version required >2x floor BETWEEN CONSECUTIVE 250 ms samples
        # before it would even start looking for quiet, so a slow low-signature
        # move never armed the detector and was reported as "never settled". That
        # false-aborted a healthy run at cycle 9 on a J0 move that had completed
        # correctly: reply OK, SET reached, 902 px total against a 111 floor, but
        # no single sample cleared 222.
        #
        # Settling is now judged on its own: N consecutive quiet samples means
        # settled, whether or not motion was ever detected tick-by-tick. So
        # "never settled" now means what it should - STILL MOVING at timeout,
        # which is the genuinely dangerous case - and a weak signal just leaves
        # cam_first_motion_s null without killing the run.
        t_first = None
        prev = before
        t0 = time.time()
        settled_at = None
        quiet_run = 0
        QUIET_SAMPLES = 3
        sigs = {self.sig(before)}
        while time.time() - t0 < SETTLE_TIMEOUT_S:
            time.sleep(0.25)
            self.L.pump()
            cur = self.roi_frame(j)
            sigs.add(self.sig(cur))
            d = diff(prev, cur)
            prev = cur
            if d > max(floor, 60):
                quiet_run = 0
                if t_first is None:
                    t_first = time.time() - t0
            else:
                quiet_run += 1
                if quiet_run >= QUIET_SAMPLES:
                    settled_at = time.time() - t0
                    break
        after = self.roi_frame(j)
        moved_px = diff(before, after)

        s1 = self.L.sta()
        sysd = s1.get("SYS", {})
        physical = moved_px > max(floor * 2, 60)

        # A CAMERA THAT STOPPED UPDATING LOOKS EXACTLY LIKE AN ARM THAT STOPPED
        # MOVING, and the second overnight attempt aborted for precisely that
        # confusion: five joints reported 5, 6, 24, 34 and 255 changed pixels in
        # the same few seconds, having each given thousands moments earlier. Five
        # joints do not fail mechanically at once - the frames were identical.
        # If every frame across a whole measurement has the same fingerprint,
        # this is a camera failure and must never be charged to the joint.
        camera_stale = len(sigs) <= 1
        if camera_stale:
            self.cam_stale_streak += 1
            self.cam_stale_total += 1
        else:
            self.cam_stale_streak = 0
        self.moves += 1
        self.counts[j]["moves"] += 1
        if camera_stale:
            # Not the arm's fault: do not count it dead, do not advance the
            # no-motion streak. The camera-stale streak has its own abort.
            self.rec("camera_stale", joint=j, test=test, px=moved_px,
                     note="identical frames for the whole measurement")
        elif physical:
            self.physical_moves += 1
            self.no_motion_streak = 0
        else:
            self.counts[j]["dead"] += 1
            # Only a move BIG enough to be reliably visible counts toward the
            # abort streak. J0 and J4 have a measured dead band, so their small
            # commands are legitimately invisible - Phase 2 logged J0 at 134 px
            # for 3 deg and 7019 for the next one. Counting those would abort a
            # healthy run at 2am on a known characteristic.
            if abs(target - start) >= 4:
                self.no_motion_streak += 1
            else:
                self.rec("dead_small_move", joint=j, size_deg=abs(target - start),
                         px=moved_px, floor=floor,
                         note="inside the joint's known dead band - not counted "
                              "toward the abort streak")
        if err:
            self.counts[j]["errors"] += 1

        rec = self.rec(
            "move", test=test, joint=j, name=NAMES.get(j),
            # controller telemetry - COMMANDED, not measured
            cmd_start_deg=start, cmd_target_deg=target,
            direction=1 if target > start else -1,
            cmd_size_deg=abs(target - start), spd=spd, acc=acc,
            reply=" | ".join(reply), cmd_ok=ok, cmd_err=err, rtt_s=round(rtt, 3),
            set_deg=s1.get(j, {}).get("SET"), tgt_deg=s1.get(j, {}).get("TGT"),
            en=s1.get(j, {}).get("EN"), es=sysd.get("ES"), wd=sysd.get("WD"),
            limits=[s1.get(j, {}).get("MIN"), s1.get(j, {}).get("MAX")],
            # physical telemetry - what the CAMERA saw
            cam_first_motion_s=round(t_first, 2) if t_first is not None else None,
            cam_settled_s=round(settled_at, 2) if settled_at is not None else None,
            cam_never_settled=settled_at is None,
            roi_changed_px=moved_px, floor_px=floor, camera_stale=camera_stale,
            distinct_frames=len(sigs),
            brightness=self.brightness(), physical_move=physical and not camera_stale,
            **(extra or {}))

        self.check_aborts(s1, rec)
        return rec

    # -------------------------------------------------------------- aborts
    def check_aborts(self, s, rec):
        sysd = s.get("SYS", {})
        if sysd.get("ES") == "1":
            self.abort("unexpected e-stop / watchdog latch", rec)
        if self.cam_stale_streak >= 3:
            self.abort("camera stalled - identical frames across 3 consecutive "
                       "measurements; this is an instrument failure, not the arm", rec)
        if self.no_motion_streak >= NO_MOTION_STRIKES:
            self.abort(f"{NO_MOTION_STRIKES} consecutive commanded moves produced "
                       f"no physical motion", rec)
        if rec.get("cam_never_settled") and rec.get("physical_move"):
            self.abort("scene never settled after a move", rec)
        if rec.get("cmd_err"):
            self.abort(f"firmware error: {rec['cmd_err']}", rec)
        for j in JOINTS:
            if j in s and s[j].get("CAL") != "1":
                self.abort(f"J{j} calibration flag changed - limits are not trusted", rec)

    def abort(self, why, rec=None):
        if self.stopped:
            return
        self.abort_reason = why
        self.stopped = True
        self.rec("ABORT", why=why, at=rec)
        self.snap("abort")
        try:
            self.L.send("STP")
        except Exception:
            pass
        raise SystemExit(f"ABORT: {why}")

    # ------------------------------------------------------------- blocks
    def bring_up(self):
        if not self.B.wait_quiet("preflight: scene clear"):
            raise SystemExit("scene never went quiet before start")
        self.B.clr()
        self.B.capture_shoulder(self.shoulder)
        s = self.L.sta()
        if s.get(1, {}).get("EN") != "1":
            raise SystemExit("shoulder would not enable")
        for j in JOINTS:
            if j == 1:
                continue
            s = self.B.clr()
            # Rule 4, which this block used to skip. capture_shoulder() has just
            # lifted the whole arm - 33k px of it - and the arm is still settling
            # when the next joint is enabled. Without this wait the shoulder's
            # settle is measured as the NEXT joint's enable jump and charged to
            # it: on 2026-08-10 J3 was adopted at 15 with the elbow genuinely at
            # 15 - the operator read the angle off the arm and confirmed it - and
            # it still measured 25217 px, tripping MAX_ENABLE_JUMP_PX and
            # aborting with "adopt angle badly wrong". The angle was right. The
            # scene was moving. The gate was not the problem and is untouched.
            if not self.B.wait_quiet(f"before enabling J{j}"):
                self.abort(f"scene never settled before enabling J{j}")
            before = frame()
            self.L.send(f"ENA {j} {int(s[j]['SET'])}")
            self.L.idle(2.0)
            jump = diff(before, frame())
            en = self.L.sta().get(j, {}).get("EN")
            self.rec("enable", joint=j, name=NAMES.get(j), adopt=int(s[j]["SET"]),
                     enable_jump_px=jump, en=en)
            if jump > MAX_ENABLE_JUMP_PX:
                self.abort(f"J{j} enable jump {jump} px - adopt angle badly wrong")

    def load_rois(self, cal_path):
        try:
            with open(cal_path) as fh:
                self.roi = {int(k): v for k, v in json.load(fh).get("roi", {}).items()}
            self.rec("roi_loaded", roi={str(k): v for k, v in self.roi.items()})
        except Exception as exc:
            self.rec("roi_missing", err=str(exc), note="using whole-arm ROI")

    def goto_pose(self, pose, test, spd=10):
        """Move joints one at a time. Wrist and elbow before the shoulder, so the
        arm reconfigures folded rather than sweeping an extended forearm."""
        for j in (3, 4, 5, 0, 1):
            if j in pose:
                self.move(j, pose[j], spd, ACCELS[j][0], test)

    def block_single_joint(self):
        for j in JOINTS:
            lo, hi = SAFE[j]
            mid = (lo + hi) // 2
            for size in (2, 5, (hi - lo) // 2):
                for direction in (+1, -1):
                    self.move(j, mid + direction * size, 10, ACCELS[j][0],
                              "single_joint", extra={"move_class": size})
                    self.move(j, mid, 10, ACCELS[j][0], "single_joint_return")
            if self.stopped:
                return

    def block_reversal(self):
        for j in (4, 3, 1, 5, 0):
            lo, hi = SAFE[j]
            mid = (lo + hi) // 2
            span = max(2, (hi - lo) // 4)
            for _ in range(3):
                self.move(j, mid + span, 10, ACCELS[j][0], "reversal")
                self.move(j, mid - span, 10, ACCELS[j][0], "reversal")
            self.move(j, mid, 10, ACCELS[j][0], "reversal_return")
            if self.stopped:
                return

    def block_speed_char(self):
        for j in JOINTS:
            lo, hi = SAFE[j]
            mid = (lo + hi) // 2
            span = max(2, (hi - lo) // 3)
            for spd in SPEEDS:
                for acc in ACCELS[j]:
                    self.move(j, mid + span, spd, acc, "speed_char",
                              extra={"combo": f"spd{spd}_acc{acc}"})
                    self.move(j, mid, spd, acc, "speed_char_return",
                              extra={"combo": f"spd{spd}_acc{acc}"})
            if self.stopped:
                return

    def block_reference(self):
        """The most important block: does the arm still land where it used to."""
        self.goto_pose(REF_POSE, "reference_goto")
        self.B.wait_quiet("reference settle")
        img = self.snap(f"ref-c{self.cycle}")
        f = frame()
        px = None
        if getattr(self, "_ref_frame", None) is not None:
            px = diff(self._ref_frame, f)
        else:
            self._ref_frame = f
        self.rec("reference", vs_first_ref_px=px, image=img,
                 brightness=self.brightness(),
                 note="drift of this number over the night is the degradation signal")

    def block_hold(self, secs=180):
        self.goto_pose(REF_POSE, "hold_goto")
        if not self.B.wait_quiet("before hold"):
            self.rec("hold_warn", note="scene not quiet - hold result contaminated")
        j = 3
        fl = self.measure_floor(j)
        ref = a = self.roi_frame(j)
        pk_adj = pk_drift = 0
        pairs = set()
        t0 = time.time()
        while time.time() - t0 < secs and not self.stopped:
            self.L.idle(0.5)
            b = self.roi_frame(j)
            pk_adj = max(pk_adj, diff(a, b))
            pk_drift = max(pk_drift, diff(ref, b))
            a = b
            st = self.L.sta().get(j, {})
            pairs.add((st.get("SET"), st.get("TGT")))
        self.rec("hold", joint=j, seconds=secs, floor_px=fl,
                 peak_adjacent_px=pk_adj, peak_drift_px=pk_drift,
                 host_rewrote_target=len(pairs) > 1,
                 distinct_pairs=sorted(pairs), brightness=self.brightness(),
                 note="powered hold; compare against the sag control before "
                      "calling any of it hunting")

    def block_enable_cycle(self):
        """Only where sag proves the physical position is still known."""
        self.goto_pose(REF_POSE, "encycle_goto")
        self.B.wait_quiet("before enable-cycle")
        j = 3                                   # not load-bearing; shoulder holds the arm
        before = self.roi_frame(j)
        self.L.send(f"DIS {j}")
        self.L.idle(4.0)
        sag = diff(before, self.roi_frame(j))
        self.rec("encycle_sag", joint=j, sag_px=sag,
                 threshold=MAX_SAG_PX_FOR_REENABLE)
        s = self.L.sta()
        adopt = int(s[j]["SET"])
        if sag > MAX_SAG_PX_FOR_REENABLE:
            # The joint moved while detached, so its commanded angle is no longer
            # its physical angle. Re-enabling there would be inventing an adopt.
            self.rec("encycle_skip", joint=j, sag_px=sag,
                     why="sagged while detached - physical position no longer known")
            self.L.send(f"ENA {j} {adopt}")     # restore, but do not measure it
            self.L.idle(2.0)
            return
        b2 = self.roi_frame(j)
        self.L.send(f"ENA {j} {adopt}")
        self.L.idle(2.0)
        jump = diff(b2, self.roi_frame(j))
        first = self.move(j, adopt + 3, 10, ACCELS[j][0], "encycle_first_move")
        second = self.move(j, adopt, 10, ACCELS[j][0], "encycle_second_move")
        self.rec("encycle", joint=j, sag_px=sag, enable_jump_px=jump,
                 first_move_px=(first or {}).get("roi_changed_px"),
                 second_move_px=(second or {}).get("roi_changed_px"),
                 note="first vs second is the take-up signature over cycle count")

    def tour(self):
        """A -> B -> C -> D -> C -> B -> A over proven-safe whole-arm poses."""
        A = REF_POSE
        B = {0: 105, 1: 88, 3: 8, 4: 100, 5: 168}
        C = {0: 120, 1: 80, 3: 22, 4: 88, 5: 176}
        D = {0: 112, 1: 84, 3: 12, 4: 103, 5: 170}
        for p, nm in ((B, "B"), (C, "C"), (D, "D"), (C, "C"), (B, "B"), (A, "A")):
            if self.stopped:
                return
            self.goto_pose(p, f"tour_{nm}")

    # ---------------------------------------------------------------- park
    def park(self):
        try:
            self.rec("park_begin")
            for j in (3, 4, 5, 0):
                try:
                    self.move(j, REF_POSE[j], 5, ACCELS[j][0], "park")
                except Exception:
                    pass
            self.L.send("STP")
            self.L.send("DIS A")
            self.rec("park_done", sta=self.L.sta().get("SYS"))
        except Exception as exc:
            self.rec("park_failed", err=str(exc))
        finally:
            try:
                self.L.send("DIS A")
            except Exception:
                pass
            self.f.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("token")
    ap.add_argument("shoulder_deg", type=int)
    ap.add_argument("--hours", type=float, default=8.0)
    ap.add_argument("--outdir", default="/home/armnode/endurance")
    ap.add_argument("--cal", default="/tmp/arm_precision_cal.json")
    ap.add_argument("--preflight-only", action="store_true")
    a = ap.parse_args()

    R = Runner(a.token, a.shoulder_deg, a.outdir, a.hours)

    def bail(signum, _frame):
        R.rec("signal", signum=signum)
        R.stopped = True
        R.park()
        sys.exit(0)
    signal.signal(signal.SIGTERM, bail)
    signal.signal(signal.SIGINT, bail)

    try:
        ver = R.L.send("VER")
        sysd = R.L.sta().get("SYS", {})
        R.rec("preflight", ver=" | ".join(ver), sys=sysd,
              expect_fw="1.2.0", fw_ok=any("FW=1.2.0" in x for x in ver),
              es_ok=sysd.get("ES") == "0", mir=sysd.get("MIR"),
              wdms=sysd.get("WDMS"), brightness=R.brightness())
        if not any("FW=1.2.0" in x for x in ver):
            raise SystemExit("unexpected firmware version")
        if sysd.get("MIR") != "INV":
            raise SystemExit("shoulder mirror is not INV - refusing to run")

        R.load_rois(a.cal)
        R.bring_up()
        for j in JOINTS:
            R.measure_floor(j)
        R.snap("preflight")

        # Prove the camera can see this arm move before trusting it all night.
        #
        # PRIMED FIRST, per rule 6 of arm_bench_test.py: "THE FIRST MOVE AFTER AN
        # ENABLE MAY BE DEAD. Backlash." This probe used to BE that first move,
        # which made it a backlash test wearing a visibility test's clothes.
        #
        # It passed on 2026-08-09 only by accident: the adopt angles were wrong
        # that night, so a 13-17k px snap on enable took up the gear train before
        # the probe ran. On 2026-08-10, with the shoulder remounted and every
        # adopt landing within 19-36 px of reality, nothing pre-loaded the train
        # and the 4 deg probe was swallowed whole - 22 px against a floor of 24 -
        # while the same joint had moved 25700 px on a 15 deg command three
        # minutes earlier through arm_bench_test. The better the adopt, the more
        # reliably the old gate false-failed.
        #
        # The gate itself is UNCHANGED: a probe must still show physical_move, and
        # the probe is still 4 deg. What changed is that the backlash is taken up
        # first, so the probe measures what it claims to measure. A dead FIRST
        # move is expected and is not evidence of anything; a dead SECOND one is.
        R.move(3, REF_POSE[3] + 4, 10, ACCELS[3][0], "preflight_prime")
        R.move(3, REF_POSE[3], 10, ACCELS[3][0], "preflight_prime_return")
        probe = R.move(3, REF_POSE[3] + 4, 10, ACCELS[3][0], "preflight_probe")
        R.move(3, REF_POSE[3], 10, ACCELS[3][0], "preflight_probe_return")
        if not probe or not probe.get("physical_move"):
            raise SystemExit("camera did not see a known-good probe move")
        R.rec("preflight_ok", note="camera sees motion; safe to begin")

        if a.preflight_only:
            R.rec("preflight_only_done")
            return

        while time.time() < R.deadline and not R.stopped:
            R.cycle += 1
            R.tour()
            if R.cycle % 5 == 0:
                R.block_reference()
            if R.cycle % 7 == 0:
                R.block_single_joint()
            if R.cycle % 11 == 0:
                R.block_reversal()
            if R.cycle % 17 == 0:
                R.block_speed_char()
            if R.cycle % 23 == 0:
                R.block_enable_cycle()
            if R.cycle % 29 == 0:
                R.block_hold(180)
            if R.cycle % 13 == 0:
                # hobby servos should not be driven flat out all night
                R.rec("rest", seconds=60)
                R.L.idle(60)
            R.rec("cycle_done", moves=R.moves, physical=R.physical_moves,
                  counts=R.counts)
    except SystemExit as exc:
        R.rec("exit", why=str(exc))
    except Exception:
        R.rec("crash", trace=traceback.format_exc())
        R.snap("crash")
    finally:
        R.park()
        R.rec("run_end", moves=R.moves, physical_moves=R.physical_moves,
              cycles=R.cycle, abort=R.abort_reason, counts=R.counts,
              camera_stale_total=R.cam_stale_total)
        R.f.close()


if __name__ == "__main__":
    main()
