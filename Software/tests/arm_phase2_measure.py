#!/usr/bin/env python3
"""Phase 2 — measure the arm against a scale, not just "did it move".

    phase2.py <token> <shoulder_deg>

Phase 1 asked "does each joint move when commanded". This asks how MUCH, which
WAY, and how REPEATABLY. It adds a centroid to every measurement: the mean
position of the changed pixels, so a move has a direction on screen and not just
a magnitude.

Runs as ONE process holding the link with a heartbeat - the watchdog detaches
everything after ~4 s of host silence, which is what broke the first harness.

2a direction + magnitude   ladder per joint, px and centroid shift
2b backlash                +N/-N repeatedly; how dead is the first move
2c repeatability           same target from both directions, compare settled frames
2e hold quality            parked and enabled, hunting vs sag, two channels
2f gripper root cause      large command, does ANYTHING in the gripper region move

2d reversal braking is deliberately NOT attempted: the ramp is a sub-second
event and /snapshot gives ~2.5 fps. It cannot be sampled here, and a PASS
invented from 2 frames would be worse than an honest gap.
"""
import json
import sys
import time
import urllib.request

import cv2
import numpy as np

BRIDGE = "http://127.0.0.1:8770"
CAM = "http://127.0.0.1:8781/snapshot"
ROI = (800, 70, 1160, 660)
THRESH = 25
HB = 1.5
NAMES = {0: "Base", 1: "Shoulder", 3: "Elbow", 4: "WristPitch", 5: "WristRoll", 6: "Gripper"}
TOK = sys.argv[1]
SHOULDER = int(sys.argv[2])
OUT = []


def log(**k):
    k = {"t": time.strftime("%H:%M:%S"), **k}
    OUT.append(k)
    print(json.dumps(k), flush=True)


class L:
    buf, hb = [], 0.0

    @classmethod
    def post(cls, line):
        b = json.dumps({"data": line + "\n"}).encode()
        r = urllib.request.Request(f"{BRIDGE}/tx?t={TOK}", data=b,
                                   headers={"Content-Type": "application/json"})
        urllib.request.urlopen(r, timeout=6).read()
        cls.hb = time.time()

    @classmethod
    def pump(cls):
        d = json.loads(urllib.request.urlopen(f"{BRIDGE}/rx?t={TOK}", timeout=6).read().decode())
        cls.buf += [x for x in d.get("lines", []) if not x.startswith(";")]
        if time.time() - cls.hb > HB:
            cls.post("PNG")

    @classmethod
    def send(cls, line, w=3.0):
        cls.buf = []
        cls.post(line)
        out, end = [], time.time() + w
        while time.time() < end:
            time.sleep(0.12)
            cls.pump()
            while cls.buf:
                out.append(cls.buf.pop(0))
            if out and (out[-1].startswith("OK") or out[-1].startswith("ERR")):
                return out
        return out

    @classmethod
    def idle(cls, s):
        e = time.time() + s
        while time.time() < e:
            time.sleep(0.15)
            cls.pump()

    @classmethod
    def sta(cls):
        snap = {}
        for ln in cls.send("STA", 3.0):
            p = ln.split()
            if not p:
                continue
            if p[0] == "STA" and len(p) > 1 and p[1].startswith("J"):
                snap[int(p[1][1:])] = dict(k.split("=", 1) for k in p[2:] if "=" in k)
            elif p[0] == "SYS":
                snap["SYS"] = dict(k.split("=", 1) for k in p[1:] if "=" in k)
        return snap


def fr():
    b = np.frombuffer(urllib.request.urlopen(CAM, timeout=10).read(), np.uint8)
    f = cv2.imdecode(b, cv2.IMREAD_COLOR)
    x0, y0, x1, y1 = ROI
    return cv2.cvtColor(f[max(y0, 70):y1, x0:x1], cv2.COLOR_BGR2GRAY)


def delta(a, b):
    """Changed pixels AND where they are - the centroid gives a move a direction."""
    m = cv2.absdiff(a, b) > THRESH
    n = int(m.sum())
    if n < 30:
        return n, None
    ys, xs = np.nonzero(m)
    return n, (round(float(xs.mean()), 1), round(float(ys.mean()), 1))


def quiet(tries=40):
    a = fr()
    run = 0
    for _ in range(tries):
        L.idle(0.4)
        b = fr()
        n, _ = delta(a, b)
        a = b
        run = run + 1 if n < 900 else 0
        if run >= 3:
            return True
    return False


def floor():
    a, pk = fr(), 0
    for _ in range(6):
        L.idle(0.4)
        b = fr()
        n, _ = delta(a, b)
        pk = max(pk, n)
        a = b
    return pk


def mv(j, tgt, settle=3.0):
    before = fr()
    r = L.send(f"MOV {j} {int(tgt)}")
    L.idle(settle)
    after = fr()
    n, c = delta(before, after)
    ok = any(x.startswith("OK") for x in r)
    return n, c, ok, after


def ena(j, adopt):
    b = fr()
    r = L.send(f"ENA {j} {int(adopt)}")
    L.idle(2.0)
    n, _ = delta(b, fr())
    s = L.sta()
    return s.get(j, {}).get("EN") == "1", n


def clr():
    s = L.sta()
    if s.get("SYS", {}).get("ES") == "1":
        L.send("CLR")
        s = L.sta()
    return s


# ---------------------------------------------------------------- start up
quiet()
clr()
ok, jump = ena(1, SHOULDER)
log(test="setup", step="shoulder", adopt=SHOULDER, enabled=ok, camera_px=jump,
    verdict="PASS" if ok and jump < 4000 else "JUMP - adopt angle wrong")
if not ok or jump >= 4000:
    L.send("DIS A")
    sys.exit("shoulder did not adopt cleanly - stopping")

for j in (3, 4, 5, 0, 6):
    s = clr()
    e, jp = ena(j, int(s[j]["SET"]))
    log(test="setup", step="enable", joint=j, name=NAMES[j], enabled=e, camera_px=jp)
    L.send(f"SPD {j} 10")
L.send("SPD 1 10")

quiet()
FL = floor()
log(test="setup", step="floor", floor_px=FL)

# ------------------------------------------------- 2a direction + magnitude
log(test="2a", step="begin", note="ladder per joint; centroid gives direction")
for j in (1, 3, 4, 5, 0, 6):
    s = L.sta()
    if s.get(j, {}).get("EN") != "1":
        log(test="2a", joint=j, name=NAMES[j], skipped="not enabled")
        continue
    lo, hi = int(s[j]["MIN"]), int(s[j]["MAX"])
    home = int(s[j]["SET"])
    for step in (5, 10, 20):
        tgt = min(hi, home + step)
        if abs(tgt - home) < 2:
            tgt = max(lo, home - step)          # room is on the other side
        if abs(tgt - home) < 2:
            log(test="2a", joint=j, name=NAMES[j], step=step, skipped="no room inside limits")
            continue
        n, c, ok, _ = mv(j, tgt)
        log(test="2a", joint=j, name=NAMES[j], frm=home, to=tgt,
            deg=abs(tgt - home), camera_px=n, centroid=c, cmd_ok=ok,
            px_per_deg=round(n / max(1, abs(tgt - home)), 1))
        mv(j, home)                              # always come back

# ------------------------------------------------------------- 2b backlash
log(test="2b", step="begin", note="dead-band on the first move after a reversal")
for j in (0, 4, 3):
    s = L.sta()
    if s.get(j, {}).get("EN") != "1":
        continue
    lo, hi = int(s[j]["MIN"]), int(s[j]["MAX"])
    home = int(s[j]["SET"])
    a, b = min(hi, home + 10), max(lo, home - 10)
    if abs(a - b) < 5:
        log(test="2b", joint=j, name=NAMES[j], skipped="not enough travel")
        continue
    seq = []
    for i in range(3):
        n1, _, _, _ = mv(j, a)
        n2, _, _, _ = mv(j, b)
        seq.append((n1, n2))
    log(test="2b", joint=j, name=NAMES[j], swings=seq, floor_px=FL,
        note="a much smaller FIRST value in each pair is lost motion")
    mv(j, home)

# -------------------------------------------------------- 2c repeatability
log(test="2c", step="begin", note="same target approached from both sides")
for j in (3, 1):
    s = L.sta()
    if s.get(j, {}).get("EN") != "1":
        continue
    lo, hi = int(s[j]["MIN"]), int(s[j]["MAX"])
    home = int(s[j]["SET"])
    tgt = home
    below, above = max(lo, home - 8), min(hi, home + 8)
    if abs(above - tgt) < 3 or abs(tgt - below) < 3:
        log(test="2c", joint=j, name=NAMES[j], skipped="not enough room either side")
        continue
    ref = None
    spread = []
    for i in range(4):
        mv(j, below if i % 2 == 0 else above)
        _, _, _, settled = mv(j, tgt)
        if ref is None:
            ref = settled
        else:
            n, _ = delta(ref, settled)
            spread.append(n)
    log(test="2c", joint=j, name=NAMES[j], target=tgt, from_deg=[below, above],
        vs_first_arrival_px=spread, floor_px=FL,
        note="near the floor = lands in the same place; large = it does not")

# ------------------------------------------------------- 2e hold quality
log(test="2e", step="begin", note="parked and enabled: hunting vs sag")
quiet()
ref = a = fr()
pk_adj = pk_drift = 0
t0 = time.time()
while time.time() - t0 < 45:
    L.idle(0.5)
    b = fr()
    adj, _ = delta(a, b)
    dr, _ = delta(ref, b)
    pk_adj, pk_drift = max(pk_adj, adj), max(pk_drift, dr)
    a = b
log(test="2e", peak_adjacent_px=pk_adj, peak_drift_px=pk_drift, floor_px=FL,
    hunting=pk_adj > FL * 3, sagging=pk_drift > FL * 4,
    note="adjacent = oscillation; drift = creeping away from where it started")

# ------------------------------------------------------------ 2f gripper
log(test="2f", step="begin", note="large command - does ANYTHING move")
s = L.sta()
if s.get(6, {}).get("EN") == "1":
    lo, hi = int(s[6]["MIN"]), int(s[6]["MAX"])
    home = int(s[6]["SET"])
    for tgt in (min(hi, home + 40), home, max(lo, home - 40), home):
        n, c, ok, _ = mv(6, tgt)
        log(test="2f", joint=6, to=tgt, deg=abs(tgt - home), camera_px=n,
            centroid=c, cmd_ok=ok, moved=n > max(FL * 2, 60))

log(test="done", off=" | ".join(L.send("DIS A")))
print("\n===== PHASE 2 LOG =====", flush=True)
for e in OUT:
    print(json.dumps(e), flush=True)
