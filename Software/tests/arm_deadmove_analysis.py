#!/usr/bin/env python3
"""Re-read past run logs and ask whether a dead move was the JOINT or the LASH.

    arm_deadmove_analysis.py <dir-or-glob-of-jsonl>

WHY THIS EXISTS
    "J0 has a 36% dead-move rate" is only a finding if the dead moves cannot be
    explained by something duller. Backlash is the duller explanation, and it has
    a signature: a joint with lash fails SMALL moves and passes LARGE ones,
    because a big enough command drives through the slop. A joint that is
    actually broken fails at every size.

    Offline. Reads JSONL that already exists. Touches no hardware, so it can run
    while the arm is in pieces on the bench.

WHAT IT FOUND ON 2026-08-10 (the reason it was written)

    dead-move rate by commanded size
                     1-4     5-9    10-14    15+
    all but base     17%      3%      1%      4%     <- textbook lash
    J0 base         100%     30%     50%     37%     <- flat; not lash

    Every joint except the base shows the lash signature and its rate collapses
    with move size. J0 does not: it stays bad at 15+ degrees, where any plausible
    backlash is long since taken up. So J0's dead-move rate SURVIVES the duller
    explanation and remains a real finding about that joint.

    The same pass showed the shoulder at 0-4% dead on 2026-08-09 - the best joint
    on the arm that night - which dates its collapse to 2026-08-10, the day a
    mounting screw worked loose.
"""

from __future__ import annotations

import collections
import glob
import json
import os
import statistics
import sys

NAMES = {0: "Base", 1: "Shoulder", 3: "Elbow", 4: "WristPitch", 5: "WristRoll", 6: "Gripper"}
BUCKETS = [("1-4", 0, 5), ("5-9", 5, 10), ("10-14", 10, 15), ("15+", 15, 10_000)]
MIN_N = 10        # below this a rate is noise wearing a percentage sign


def bucket(deg):
    for name, lo, hi in BUCKETS:
        if lo <= deg < hi:
            return name
    return BUCKETS[-1][0]


def load(pattern):
    files = []
    if os.path.isdir(pattern):
        files = sorted(glob.glob(os.path.join(pattern, "*.jsonl")))
    else:
        files = sorted(glob.glob(pattern))
    rows = []
    for f in files:
        with open(f) as fh:
            for ln in fh:
                try:
                    e = json.loads(ln)
                except ValueError:
                    continue
                if e.get("kind") == "move" and e.get("joint") is not None:
                    rows.append(e)
    return files, rows


def rate(rows):
    if not rows:
        return None
    dead = [e for e in rows if not e.get("physical_move")]
    return len(rows), len(dead), len(dead) / len(rows) * 100.0


def table(title, groups, keyfmt):
    print(f"\n{title}")
    print(f"{'':<14}" + "".join(f"{b[0]:>9}" for b in BUCKETS))
    print("-" * (14 + 9 * len(BUCKETS)))
    for key in sorted(groups, key=lambda k: str(k)):
        cells = []
        for b, _, _ in BUCKETS:
            r = rate(groups[key].get(b, []))
            cells.append("-" if not r else f"{r[2]:.0f}% ({r[0]})")
        print(f"{keyfmt(key):<14}" + "".join(f"{c:>9}" for c in cells))


def main():
    pattern = sys.argv[1] if len(sys.argv) > 1 else "/home/armnode/endurance"
    files, rows = load(pattern)
    if not rows:
        raise SystemExit(f"no move records found in {pattern}")

    print(f"files   {len(files)}")
    print(f"moves   {len(rows)}")

    by_joint = collections.defaultdict(lambda: collections.defaultdict(list))
    for e in rows:
        by_joint[e["joint"]][bucket(e.get("cmd_size_deg", 0))].append(e)
    table("DEAD-MOVE RATE BY JOINT AND COMMANDED SIZE   rate (n)",
          by_joint, lambda j: NAMES.get(j, f"J{j}"))

    print("\nBY JOINT AND DIRECTION")
    print(f"{'joint':<14}{'dir':>5}{'n':>6}{'dead':>6}{'rate':>8}{'median deg':>12}")
    print("-" * 51)
    by_dir = collections.defaultdict(list)
    for e in rows:
        by_dir[(e["joint"], e.get("direction"))].append(e)
    for k in sorted(by_dir, key=lambda x: (x[0], x[1] if x[1] is not None else 0)):
        v = by_dir[k]
        n, d, pct = rate(v)
        med = statistics.median([e.get("cmd_size_deg", 0) for e in v])
        print(f"{NAMES.get(k[0], k[0]):<14}{str(k[1]):>5}{n:>6}{d:>6}{pct:>7.0f}%{med:>12.0f}")

    print("\nVERDICT PER JOINT")
    for j in sorted(by_joint):
        small = rate(by_joint[j].get("1-4", []) + by_joint[j].get("5-9", []))
        large = rate(by_joint[j].get("10-14", []) + by_joint[j].get("15+", []))
        # MIN_N or the verdict is noise wearing a percentage sign. The first run
        # of this tool called WristRoll "NOT explained by backlash - still
        # failing at large moves" on the strength of one dead move out of two.
        if not small or not large or small[0] < MIN_N or large[0] < MIN_N:
            have = f"small n={small[0] if small else 0}, large n={large[0] if large else 0}"
            print(f"  {NAMES.get(j, j):<12} NO VERDICT - need {MIN_N} moves in each range ({have})")
            continue
        if small[2] > 10 and large[2] < small[2] / 3:
            v = "consistent with BACKLASH - small moves die, large ones live"
        elif large[2] > 15:
            v = "NOT explained by backlash - still failing at large moves"
        else:
            v = "healthy at every size"
        print(f"  {NAMES.get(j, j):<12} small {small[2]:>3.0f}%   large {large[2]:>3.0f}%   {v}")


if __name__ == "__main__":
    main()
