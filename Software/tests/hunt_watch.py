"""Watch a parked arm for HUNTING -- a joint that cannot hold still under load.

This is not motion verification. Nothing is commanded. The arm is parked and the
question is whether it STAYS parked. The ROI is the claw, deliberately: it is the
far end of the kinematic chain, so a degree of elbow hunt swings it further than
anything nearer the base. Idle floor measured 2026-08-08 on this pose: 0-5 px.

Reports every sample so a quiet run is evidence too, and flags excursions.
"""
import sys, time, urllib.request
import cv2, numpy as np

URL = "http://127.0.0.1:8781/snapshot"
ROI = (300, 590, 480, 719)
FLAG = 30           # px; 6x the measured idle ceiling of 5
TICK_S = 60.0       # how often to report cumulative drift regardless of flags
SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0

def grab():
    buf = np.frombuffer(urllib.request.urlopen(URL, timeout=10).read(), np.uint8)
    f = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    x0, y0, x1, y1 = ROI
    return cv2.cvtColor(f[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)

ref = prev = grab()
t0 = time.time()
n = flagged = 0
peak_adj = peak_ref = 0
last_tick = 0.0
print(f"watching {SECS:.0f}s   ROI {ROI}   flag at {FLAG} px", flush=True)
while time.time() - t0 < SECS:
    time.sleep(0.4)
    cur = grab()
    n += 1
    el = time.time() - t0
    adj = int((cv2.absdiff(prev, cur) > 25).sum())     # frame-to-frame = hunting
    ref_d = int((cv2.absdiff(ref, cur) > 25).sum())    # vs t=0 = drift
    peak_adj, peak_ref = max(peak_adj, adj), max(peak_ref, ref_d)
    if adj >= FLAG:
        flagged += 1
        print(f"  t={el:6.1f}s  HUNT? adjacent {adj:6d} px   vs-start {ref_d:6d} px", flush=True)
    # DRIFT IS REPORTED ON A TICK, NOT ONLY IN THE SUMMARY. The first version
    # printed vs-start only when the OSCILLATION flag fired, so a run that
    # oscillates not at all but creeps steadily -- which is exactly what this
    # arm does with J1 detached, 134 px over 900 s -- printed "no hunt" and
    # buried the one number that mattered. Two different faults, two channels.
    if el - last_tick >= TICK_S:
        last_tick = el
        print(f"  t={el:6.1f}s  drift vs-start {ref_d:6d} px   (adjacent {adj:4d})", flush=True)
    prev = cur
print(f"\n{n} samples over {time.time()-t0:.0f}s")
print(f"peak adjacent-frame {peak_adj} px   peak vs-start {peak_ref} px")
print(f"frames over {FLAG} px: {flagged}")
print("VERDICT:", "HUNT CAUGHT" if flagged else "no hunt in this window -- arm held still")
