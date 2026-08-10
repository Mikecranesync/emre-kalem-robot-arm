#!/usr/bin/env python3
"""Backlash measurement and preferred-direction compensation. RUNS ON THE PI.

    arm_precision.py <token> <shoulder_deg> roi                 # derive per-joint ROIs
    arm_precision.py <token> <shoulder_deg> backlash <j> [reps] # onset sweep, both ways
    arm_precision.py <token> <shoulder_deg> ab <j> <target>     # A/B, comp on vs off
    arm_precision.py <token> <shoulder_deg> hold [secs]         # clean hold test
    arm_precision.py <token> <shoulder_deg> off

Imports Link and Bench from arm_bench_test so the six safety rules are shared,
not re-implemented: one heartbeated process, shoulder enabled first, an
operator-supplied adopt angle, a scene-quiet gate, no failing of clamped moves,
and a first-move-after-enable that is expected to be dead.

=============================================================================
HOW BACKLASH IS MEASURED, AND WHY IT IS NOT PHASE 2's METHOD
=============================================================================
Phase 2 tried to derive a pixels-per-degree scale from a ladder of moves. It
failed: the readings were non-monotonic because a whole-arm ROI was combined
with no settling gate, so each measurement also caught the previous move still
moving. Those numbers were discarded.

This is the dial-indicator method instead, and it measures ONSET rather than
magnitude:

  1. drive the joint well past any take-up in direction A, so the gears are
     loaded on one set of faces
  2. reverse, and step 1 degree at a time in direction B
  3. after each step wait for the scene to be still, then compare that settled
     frame with the PREVIOUS settled frame
  4. the first step whose change clears the noise floor is where real motion
     began; the degrees commanded before it is the lost motion

It needs no absolute position and no pixel-to-degree scale - which is exactly
why it can succeed where the ladder failed. It also cannot tell you how FAR a
joint moved, only when it started. That is fine: backlash in degrees is the only
number compensation needs.

=============================================================================
COMPENSATION
=============================================================================
Host-side, deliberately: easy to measure, tune, disable and A/B against. The
firmware is untouched, so the acceleration-limited profile and the reversal
braking in FW 1.2.0 keep working underneath exactly as before.

The strategy is PREFERRED-DIRECTION APPROACH, not an added offset. Rather than
commanding the target from wherever the joint happens to be, the joint is first
placed on the agreed side of the target by (measured backlash + margin), and the
final move always arrives from that same side. The gears are therefore loaded on
the same tooth faces on every arrival - which is what the Phase 2 hysteresis
result says actually determines where it lands.

Adding a compensation offset to the target was considered and is NOT used: these
are position-controlled hobby servos, not steppers. An offset makes the servo
seek a different angle and the error simply moves; approaching from a consistent
side removes the ambiguity instead of trading it.

SAFETY: the pre-approach point is clamped to the joint's own soft limits. If
there is not enough room, the pose is reported as UNAPPROACHABLE from the
preferred side rather than silently approached from the wrong one or driven past
a limit.
"""
import json
import statistics
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, "/tmp")
from arm_bench_test import Link, Bench, frame, diff, NAMES, log, LOG  # noqa: E402

TOKEN = sys.argv[1]
SHOULDER = int(sys.argv[2])
CMD = sys.argv[3]

CAL_PATH = "/tmp/arm_precision_cal.json"
SETTLE_S = 2.0


def load_cal():
    try:
        with open(CAL_PATH) as f:
            return json.load(f)
    except Exception:
        return {"roi": {}, "backlash": {}}


def save_cal(c):
    with open(CAL_PATH, "w") as f:
        json.dump(c, f, indent=1)


class Precision:
    def __init__(self):
        self.L = Link(TOKEN)
        self.B = Bench(self.L)
        self.cal = load_cal()

    # ---- bring the arm up under the existing rules ------------------------
    def bring_up(self, joints):
        if not self.B.wait_quiet("before enabling the shoulder"):
            self.B.stop("scene not still before the first enable")
        self.B.clr()
        # Rule 3: 91 is the operator's figure ("close to 90"), confirmed by two
        # earlier enables at 19 px and 21 px. Not inferred.
        self.B.capture_shoulder(SHOULDER)
        for j in joints:
            if j == 1:
                continue
            s = self.B.clr()
            if s.get(j, {}).get("EN") != "1":
                self.B.enable(j, int(s[j]["SET"]))

    def settled(self, why="settle"):
        self.B.wait_quiet(why)
        return frame()

    def roi_frame(self, roi):
        f = frame()
        if not roi:
            return f
        x0, y0, x1, y1 = roi
        return f[y0:y1, x0:x1]

    # ---- rule 6 made deliberate: burn the take-up on purpose --------------
    def prime(self, j, span=8):
        """Sacrificial take-up so the first PRECISION move is not the dead one.

        Phase 2: J0's first move after an enable produced 91 px against a 992
        floor, J4's 198. Both were fully absorbed. Rather than discovering that
        during a pose, spend it here on purpose, then leave the joint loaded in
        the direction it will finally approach from.
        """
        s = self.L.sta()
        lo, hi = int(s[j]["MIN"]), int(s[j]["MAX"])
        start = int(s[j]["SET"])
        a = min(hi, start + span)
        b = max(lo, start - span)
        if abs(a - b) < 4:
            log(step="prime", joint=j, skipped="not enough travel inside limits")
            return False
        for tgt in (a, b, a, start):
            self.L.send(f"MOV {j} {int(tgt)}")
            self.L.idle(SETTLE_S)
        log(step="prime", joint=j, name=NAMES.get(j), swept=[b, a], back_to=start,
            note="take-up spent deliberately; joint now exercised")
        return True

    # ---- derive a per-joint ROI instead of hand-picking one ---------------
    def derive_roi(self, j, span=15):
        """J0 needs a bigger swing: 15 deg produced no connected region at all,
        which is consistent with its measured dead band rather than with the
        base being broken. Retried larger before it is called a failure."""
        if j == 0:
            span = 30
        s = self.L.sta()
        lo, hi = int(s[j]["MIN"]), int(s[j]["MAX"])
        start = int(s[j]["SET"])
        far = min(hi, start + span)
        if abs(far - start) < 5:
            far = max(lo, start - span)
        if abs(far - start) < 5:
            log(step="roi", joint=j, skipped="no room to exercise")
            return None
        base = self.settled(f"roi base J{j}")
        self.L.send(f"MOV {j} {int(far)}")
        self.L.idle(SETTLE_S)
        moved = self.settled(f"roi moved J{j}")
        self.L.send(f"MOV {j} {int(start)}")
        self.L.idle(SETTLE_S)

        m = (cv2.absdiff(base, moved) > 25).astype(np.uint8)
        # Open then close: kill isolated speckle, then join the joint's own
        # region into one blob so the bounding box is the joint and not the
        # union of the joint and a noisy corner.
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
        n, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)
        if n <= 1:
            log(step="roi", joint=j, failed="no connected motion region")
            return None
        k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        x, y, w, h, area = stats[k]
        pad = 12
        roi = [max(0, int(x - pad)), max(0, int(y - pad)),
               int(x + w + pad), int(y + h + pad)]
        log(step="roi", joint=j, name=NAMES.get(j), roi=roi, area=int(area),
            exercised_deg=abs(far - start))
        self.cal.setdefault("roi", {})[str(j)] = roi
        save_cal(self.cal)
        return roi

    def roi_floor(self, roi, n=6, cap_frac=0.02):
        """Noise floor INSIDE one joint's ROI. Gated on a quiet scene.

        The first version of this sampled immediately and returned 3860 px on a
        48k-pixel ROI - four times the whole-arm floor, which is impossible for
        a smaller box. It had caught the arm still settling from prime(). That
        is precisely the Phase 2 failure, reintroduced in a new function, so the
        gate is now here and the result is sanity-checked against the ROI's own
        size rather than a fixed constant.
        """
        for attempt in range(4):
            if not self.B.wait_quiet(f"before ROI floor (try {attempt + 1})"):
                continue
            a, pk = self.roi_frame(roi), 0
            for _ in range(n):
                self.L.idle(0.4)
                b = self.roi_frame(roi)
                pk = max(pk, diff(a, b))
                a = b
            px_total = (roi[3] - roi[1]) * (roi[2] - roi[0]) if roi else 1
            if pk <= max(60, px_total * cap_frac):
                return pk
            log(step="roi_floor_rejected", roi=roi, floor_px=pk,
                cap=int(px_total * cap_frac), attempt=attempt + 1)
        return None

    # ---- the onset sweep --------------------------------------------------
    def onset(self, j, direction, roi, floor, park, run_up=10, max_steps=14):
        """Load the gears one way, reverse, and step 1 deg until motion starts.

        Returns the number of commanded degrees consumed before the first
        settled frame that differs from the previous one by more than the floor,
        or None if motion never appeared inside max_steps.
        """
        s = self.L.sta()
        lo, hi = int(s[j]["MIN"]), int(s[j]["MAX"])
        # Load hard in the OPPOSITE direction first, so the slack is fully on
        # the side we are about to cross.
        pre = park - direction * run_up
        pre = max(lo, min(hi, pre))
        self.L.send(f"MOV {j} {int(pre)}")
        self.L.idle(SETTLE_S + 1.0)
        self.L.send(f"MOV {j} {int(park)}")
        self.L.idle(SETTLE_S + 1.0)

        prev = self.settled(f"onset base J{j} dir{direction}")
        consumed = None
        trace = []
        for step in range(1, max_steps + 1):
            tgt = park + direction * step
            if tgt < lo or tgt > hi:
                log(step="onset_limit", joint=j, direction=direction,
                    stopped_at=step, why="soft limit reached - not widening it")
                break
            self.L.send(f"MOV {j} {int(tgt)}")
            self.L.idle(SETTLE_S)
            cur = self.settled(f"onset step {step}")
            dpx = diff(prev, cur)
            trace.append({"deg": step, "px": dpx})
            prev = cur
            if dpx > floor * 2 and consumed is None:
                consumed = step
                break
        return consumed, trace


def cmd_roi(P):
    joints = [1, 3, 4, 5, 0]
    P.bring_up(joints)
    for j in joints:
        P.prime(j)
        P.derive_roi(j)
    log(step="roi_done", cal=P.cal.get("roi"))


def cmd_backlash(P, j, reps):
    P.bring_up([1, j] if j != 1 else [1])
    roi = P.cal.get("roi", {}).get(str(j))
    if not roi:
        roi = P.derive_roi(j)
    P.prime(j)

    s = P.L.sta()
    lo, hi = int(s[j]["MIN"]), int(s[j]["MAX"])
    park = int(s[j]["SET"])
    # Park mid-range so both directions have room. J1 sits at 91 = MAX with no
    # room either side, which is why Phase 2 could not measure it at all.
    mid = (lo + hi) // 2
    if abs(park - mid) > 3 and lo + 12 < mid < hi - 12:
        P.L.send(f"MOV {j} {mid}")
        P.L.idle(3.0)
        park = mid
    log(step="park", joint=j, at=park, limits=[lo, hi])

    fl = P.roi_floor(roi)
    if fl is None:
        log(step="ABORT", joint=j, why="ROI noise floor never settled to a believable value")
        P.L.send("DIS A")
        return
    log(step="roi_floor", joint=j, floor_px=fl, roi=roi)

    out = {"up": [], "down": []}
    for rep in range(reps):
        for direction, key in ((+1, "up"), (-1, "down")):
            consumed, trace = P.onset(j, direction, roi, fl, park)
            out[key].append(consumed)
            log(step="onset", joint=j, name=NAMES.get(j), rep=rep + 1,
                direction=key, consumed_deg=consumed, floor_px=fl, trace=trace)

    summary = {}
    for key, vals in out.items():
        got = [v for v in vals if v is not None]
        summary[key] = {
            "samples": vals,
            "median_deg": statistics.median(got) if got else None,
            "spread_deg": (max(got) - min(got)) if len(got) > 1 else 0,
            "misses": vals.count(None),
        }
    P.cal.setdefault("backlash", {})[str(j)] = {
        "measured": summary, "park": park, "floor_px": fl,
        "date": time.strftime("%Y-%m-%d"), "roi": roi,
    }
    save_cal(P.cal)
    log(step="backlash_summary", joint=j, name=NAMES.get(j), summary=summary)


def cmd_repeat(P, j, reps):
    """THE CONTROL EXPERIMENT for Phase 2's hysteresis claim.

    Phase 2 reported that approaching J3's target from opposite sides left the
    arm ~10-11k px apart while the same side landed at 982 px (its floor). But
    that test had NO scene-quiet gate - the same defect that invalidated 2a - so
    "arrival" may simply have been caught mid-settle, and the split would then be
    a settling artifact rather than a position difference.

    This repeats it properly: every arrival is quiet-gated and measured inside
    the joint's own derived ROI. It reports BOTH same-side and opposite-side
    spread, so the two are directly comparable against one floor.

    If the split survives, it is real hysteresis and preferred-direction approach
    is justified. If it collapses to the floor, there is nothing to compensate
    and the honest answer is that Phase 2 measured its own settling.
    """
    P.bring_up([1, j] if j != 1 else [1])
    roi = P.cal.get("roi", {}).get(str(j)) or P.derive_roi(j)
    P.prime(j)

    s = P.L.sta()
    lo, hi = int(s[j]["MIN"]), int(s[j]["MAX"])
    mid = (lo + hi) // 2
    target = mid
    below, above = max(lo, target - 8), min(hi, target + 8)
    if target - below < 3 or above - target < 3:
        log(step="repeat_skip", joint=j, why="not enough room either side of mid-range")
        return

    fl = P.roi_floor(roi)
    if fl is None:
        log(step="ABORT", joint=j, why="ROI floor never settled")
        return
    log(step="repeat_setup", joint=j, name=NAMES.get(j), target=target,
        below=below, above=above, roi=roi, floor_px=fl)

    def arrive(frm):
        P.L.send(f"MOV {j} {int(frm)}")
        P.L.idle(SETTLE_S)
        P.B.wait_quiet(f"at approach point {frm}")
        P.L.send(f"MOV {j} {int(target)}")
        P.L.idle(SETTLE_S)
        P.B.wait_quiet(f"arrived at {target} from {frm}")
        return P.roi_frame(roi)

    from_below, from_above = [], []
    for _ in range(reps):
        from_below.append(arrive(below))
        from_above.append(arrive(above))

    def spread(frames):
        return [diff(frames[0], f) for f in frames[1:]]

    same_below = spread(from_below)
    same_above = spread(from_above)
    opposite = [diff(b, a) for b, a in zip(from_below, from_above)]

    log(step="repeat_result", joint=j, name=NAMES.get(j), reps=reps,
        floor_px=fl,
        same_side_below_px=same_below,
        same_side_above_px=same_above,
        opposite_side_px=opposite,
        median_same=statistics.median(same_below + same_above) if (same_below + same_above) else None,
        median_opposite=statistics.median(opposite) if opposite else None,
        note="if median_opposite collapses to the floor the Phase 2 split was settling")


def cmd_hold(P, j, secs):
    """Clean hold test, per-joint ROI, and a check that the HOST is innocent.

    Phase 2's hold number (9571 px adjacent, 9983 drift) used a whole-arm ROI on
    an unattended bench, so it could not separate the arm moving from a person
    moving. This uses the joint's own derived ROI, gates on a quiet scene first,
    and - importantly - polls STA throughout to prove SET and TGT never change.
    If the target is constant and the picture still moves, the motion is the
    servo or the mechanism, not the host rewriting commands.
    """
    P.bring_up([1, j] if j != 1 else [1])
    roi = P.cal.get("roi", {}).get(str(j)) or P.derive_roi(j)
    P.prime(j)
    s = P.L.sta()
    lo, hi = int(s[j]["MIN"]), int(s[j]["MAX"])
    mid = (lo + hi) // 2
    P.L.send(f"MOV {j} {mid}")
    P.L.idle(3.0)

    fl = P.roi_floor(roi)
    if fl is None:
        log(step="ABORT", joint=j, why="ROI floor never settled")
        return
    if not P.B.wait_quiet("before the hold window"):
        log(step="hold_warn", note="scene never went quiet - result is contaminated")

    ref = a = P.roi_frame(roi)
    pk_adj = pk_drift = 0
    targets = set()
    t0 = time.time()
    while time.time() - t0 < secs:
        P.L.idle(0.5)
        b = P.roi_frame(roi)
        pk_adj = max(pk_adj, diff(a, b))
        pk_drift = max(pk_drift, diff(ref, b))
        a = b
        st = P.L.sta().get(j, {})
        targets.add((st.get("SET"), st.get("TGT")))

    log(step="hold_result", joint=j, name=NAMES.get(j), seconds=secs,
        roi=roi, floor_px=fl,
        peak_adjacent_px=pk_adj, peak_drift_px=pk_drift,
        adjacent_over_floor=round(pk_adj / max(1, fl), 1),
        distinct_set_tgt_pairs=sorted(targets),
        host_rewrote_target=len(targets) > 1,
        hunting=pk_adj > fl * 3, sagging=pk_drift > fl * 4,
        note="one SET/TGT pair means the host held a single unchanging command")


def cmd_holdoff(P, j, secs):
    """THE CONTROL for the hold test: same ROI, same window, joint DETACHED.

    A driven joint that will not sit still could be the servo hunting, or it
    could be the scene - a draught, a flickering lamp, the camera itself. The
    only way to tell is to remove the drive and look again at the same pixels.

    The shoulder stays ENABLED throughout so the arm is still supported; only
    the joint under test is detached. Never detach the load-bearing joint to
    make a measurement - a quiet arm that falls is not an improvement.
    """
    P.bring_up([1])                       # shoulder only: it holds the arm up
    roi = P.cal.get("roi", {}).get(str(j))
    if not roi:
        log(step="ABORT", joint=j, why="no derived ROI for this joint - run roi first")
        return
    P.L.send(f"DIS {j}")                  # the joint under test only
    P.L.idle(4.0)                         # let it sag and settle
    s = P.L.sta()
    log(step="holdoff_state", joint=j, en=s.get(j, {}).get("EN"),
        shoulder_en=s.get(1, {}).get("EN"))

    if not P.B.wait_quiet("before the detached hold window"):
        log(step="holdoff_warn", note="scene never went quiet")
    fl = P.roi_floor(roi)
    if fl is None:
        log(step="ABORT", joint=j, why="ROI floor never settled")
        return

    ref = a = P.roi_frame(roi)
    pk_adj = pk_drift = 0
    t0 = time.time()
    while time.time() - t0 < secs:
        P.L.idle(0.5)
        b = P.roi_frame(roi)
        pk_adj = max(pk_adj, diff(a, b))
        pk_drift = max(pk_drift, diff(ref, b))
        a = b
    log(step="holdoff_result", joint=j, name=NAMES.get(j), seconds=secs,
        roi=roi, floor_px=fl, peak_adjacent_px=pk_adj, peak_drift_px=pk_drift,
        adjacent_over_floor=round(pk_adj / max(1, fl), 1),
        note="compare with hold_result: quiet here and noisy there means the "
             "SERVO is the source; noisy in both means the scene is")


def main():
    P = Precision()
    if CMD == "off":
        print(" | ".join(P.L.send("DIS A")), flush=True)
        return
    if CMD == "roi":
        cmd_roi(P)
    elif CMD == "holdoff":
        cmd_holdoff(P, int(sys.argv[4]), int(sys.argv[5]) if len(sys.argv) > 5 else 60)
    elif CMD == "hold":
        cmd_hold(P, int(sys.argv[4]), int(sys.argv[5]) if len(sys.argv) > 5 else 60)
    elif CMD == "repeat":
        cmd_repeat(P, int(sys.argv[4]), int(sys.argv[5]) if len(sys.argv) > 5 else 4)
    elif CMD == "backlash":
        cmd_backlash(P, int(sys.argv[4]), int(sys.argv[5]) if len(sys.argv) > 5 else 3)
    else:
        sys.exit(f"unknown command {CMD}")
    log(step="off", result=" | ".join(P.L.send("DIS A")))
    print("\n===== PRECISION LOG =====", flush=True)
    for e in LOG:
        print(json.dumps(e), flush=True)


if __name__ == "__main__":
    main()
