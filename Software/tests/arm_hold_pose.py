"""Enable the shoulder, drive it to a commanded angle, then HOLD it there.

Bench helper. The arm collapses whenever the joints detach, so a step that ends
with DIS A destroys the pose before the next step can use it. This keeps one
process, one link and a heartbeat alive so the arm stays where it was put.

    arm_hold_pose.py <token> <adopt_deg> <target_deg> [hold_seconds]

Parks with DIS A on timeout, on Ctrl-C, and on any exception. The arm WILL sag
when it parks - that is gravity, not a fault.
"""

import sys
import time

sys.path.insert(0, "/home/armnode/arm/tests")
from arm_bench_test import Link, Bench, frame, diff, log  # noqa: E402

token = sys.argv[1]
adopt = int(sys.argv[2])
target = int(sys.argv[3])
hold_s = float(sys.argv[4]) if len(sys.argv) > 4 else 900.0

L = Link(token)
B = Bench(L)
try:
    B.clr()
    B.wait_quiet("before adopt")
    B.capture_shoulder(adopt)
    if L.sta().get(1, {}).get("EN") != "1":
        raise SystemExit("shoulder would not enable")
    B.wait_quiet("after adopt")
    fl = B.floor() or 200

    before = frame()
    L.send("SPD 1 5")
    L.send(f"MOV 1 {target}")
    for _ in range(8):
        L.idle(1.0)
    px = diff(before, frame())
    s = L.sta().get(1, {})
    log(step="set_j1", adopt=adopt, target=target, changed_px=px, floor_px=fl,
        set_deg=s.get("SET"), en=s.get("EN"), jto=s.get("JTO"),
        moved=px > max(fl * 3, 300))

    log(step="holding", seconds=hold_s,
        note="shoulder powered and holding; heartbeat alive")
    end = time.time() + hold_s
    last = 0.0
    while time.time() < end:
        L.idle(1.0)
        if time.time() - last >= 30.0:
            s = L.sta().get(1, {})
            sysd = L.sta().get("SYS", {})
            log(step="hold_tick", remaining_s=round(end - time.time()),
                set_deg=s.get("SET"), en=s.get("EN"), es=sysd.get("ES"),
                wd=sysd.get("WD"))
            last = time.time()
finally:
    log(step="off", result=" | ".join(L.send("DIS A")))
