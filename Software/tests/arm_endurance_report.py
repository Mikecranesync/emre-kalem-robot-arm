#!/usr/bin/env python3
"""Morning report from an endurance run's raw JSONL.

    arm_endurance_report.py <endurance-*.jsonl> [more.jsonl ...]

Deliberately separate from the runner so the report can be regenerated from
preserved evidence without touching the arm, and so a bug here can never abort a
run in progress.

The question this answers is NOT "did it survive". It is "did it get WORSE".
So every per-joint metric is split into FIRST / MIDDLE / LAST thirds by cycle and
compared. A single number for the whole night would hide exactly the degradation
the run exists to find.

Verdicts are deliberately four-valued. "INSTRUMENT CANNOT DECIDE" is a real
outcome on this bench: the camera's noise floor has been measured drifting 3.6x
between runs minutes apart, so an effect of that size cannot be attributed to the
arm at all.
"""
import json
import statistics
import sys


def load(paths):
    rows = []
    for p in paths:
        with open(p) as f:
            for line in f:
                line = line.strip()
                if not line.startswith("{"):
                    continue
                try:
                    rows.append(json.loads(line))
                except Exception:
                    pass
    return rows


def thirds(items, key="cycle"):
    if not items:
        return [], [], []
    cs = sorted({r.get(key, 0) for r in items})
    if len(cs) < 3:
        return items, [], []
    a, b = cs[len(cs) // 3], cs[2 * len(cs) // 3]
    return ([r for r in items if r.get(key, 0) <= a],
            [r for r in items if a < r.get(key, 0) <= b],
            [r for r in items if r.get(key, 0) > b])


def med(vals):
    vals = [v for v in vals if isinstance(v, (int, float))]
    return round(statistics.median(vals), 2) if vals else None


def trend(early, late, higher_is_worse=True, tol=0.30):
    """Compare two medians and say plainly whether it got worse."""
    if early is None or late is None:
        return "no data"
    if early == 0:
        return "no baseline"
    change = (late - early) / abs(early)
    if abs(change) < tol:
        return f"stable ({change:+.0%})"
    worse = change > 0 if higher_is_worse else change < 0
    return f"{'WORSE' if worse else 'better'} ({change:+.0%})"


def main():
    rows = load(sys.argv[1:])
    if not rows:
        sys.exit("no records")
    moves = [r for r in rows if r.get("kind") == "move"]
    ends = [r for r in rows if r.get("kind") == "run_end"]
    aborts = [r for r in rows if r.get("kind") == "ABORT"]
    holds = [r for r in rows if r.get("kind") == "hold"]
    refs = [r for r in rows if r.get("kind") == "reference"]
    encyc = [r for r in rows if r.get("kind") == "encycle"]
    camfail = [r for r in rows if r.get("kind") == "camera_fail"]
    unsettled = [r for r in rows if r.get("kind") == "floor_unsettled"]
    crashes = [r for r in rows if r.get("kind") == "crash"]

    t0 = min(r["ts"] for r in rows)
    t1 = max(r["ts"] for r in rows)
    cycles = max((r.get("cycle", 0) for r in rows), default=0)
    phys = [m for m in moves if m.get("physical_move")]
    wd = [r for r in rows if r.get("wd") == "1" or (r.get("sys") or {}).get("WD") == "1"]

    P = print
    P("=" * 70)
    P("  ENDURANCE MORNING REPORT")
    P("=" * 70)
    P(f"runtime            {(t1 - t0) / 3600:.2f} h")
    P(f"cycles completed   {cycles}")
    P(f"commanded moves    {len(moves)}")
    P(f"physical moves     {len(phys)}  ({100 * len(phys) / max(1, len(moves)):.0f}% of commanded)")
    P(f"aborts             {len(aborts)}" + (f"  -> {aborts[0].get('why')}" if aborts else ""))
    P(f"crashes            {len(crashes)}")
    P(f"camera failures    {len(camfail)}")
    P(f"floor unsettled    {len(unsettled)}   (scene too busy to trust a floor)")
    P(f"watchdog events    {len(wd)}")
    for e in ends:
        P(f"final counts       {json.dumps(e.get('counts'))}")

    P("\n--- PER JOINT ---")
    P(f"{'joint':16s} {'moves':>6s} {'dead':>6s} {'err':>5s} "
      f"{'settle early':>13s} {'settle late':>12s} {'trend':>18s}")
    verdicts = {}
    for j in sorted({m.get("joint") for m in moves if m.get("joint") is not None}):
        jm = [m for m in moves if m.get("joint") == j]
        e, _m, l = thirds(jm)
        se, sl = med([x.get("cam_settled_s") for x in e]), med([x.get("cam_settled_s") for x in l])
        dead = sum(1 for x in jm if not x.get("physical_move"))
        errs = sum(1 for x in jm if x.get("cmd_err"))
        tr = trend(se, sl)
        P(f"J{j} {str(NAMES_LOCAL.get(j, '')):13s} {len(jm):6d} {dead:6d} {errs:5d} "
          f"{str(se):>13s} {str(sl):>12s} {tr:>18s}")

        # directional weakness: does one direction go dead more than the other?
        pos = [x for x in jm if x.get("direction") == 1]
        neg = [x for x in jm if x.get("direction") == -1]
        dpos = sum(1 for x in pos if not x.get("physical_move")) / max(1, len(pos))
        dneg = sum(1 for x in neg if not x.get("physical_move")) / max(1, len(neg))
        verdicts[j] = {
            "moves": len(jm), "dead": dead, "errors": errs,
            "settle_early": se, "settle_late": sl, "settle_trend": tr,
            "dead_rate_pos": round(dpos, 3), "dead_rate_neg": round(dneg, 3),
            "directional": "YES" if abs(dpos - dneg) > 0.25 else "no",
        }

    P("\n--- DIRECTIONAL WEAKNESS (dead-move rate by direction) ---")
    for j, v in verdicts.items():
        P(f"J{j}  +dir {v['dead_rate_pos']:.0%}   -dir {v['dead_rate_neg']:.0%}   "
          f"asymmetric: {v['directional']}")

    P("\n--- REFERENCE POSE DRIFT (does it still land in the same place?) ---")
    if refs:
        vals = [(r.get("cycle"), r.get("vs_first_ref_px")) for r in refs
                if r.get("vs_first_ref_px") is not None]
        if vals:
            e, _m, l = thirds([{"cycle": c, "v": v} for c, v in vals])
            P(f"  arrivals compared: {len(vals)}")
            P(f"  early median {med([x['v'] for x in e])}   late median {med([x['v'] for x in l])}")
            P(f"  trend: {trend(med([x['v'] for x in e]), med([x['v'] for x in l]))}")
        else:
            P("  only one reference arrival - no drift comparison possible")
    else:
        P("  none recorded")

    P("\n--- HOLD BLOCKS (powered) ---")
    for h in holds:
        P(f"  cycle {h.get('cycle')}: adjacent {h.get('peak_adjacent_px')} px, "
          f"drift {h.get('peak_drift_px')} px, floor {h.get('floor_px')}, "
          f"host rewrote target: {h.get('host_rewrote_target')}")
    if holds:
        P("  NOTE: a powered-hold number means nothing without the detached control.")
        P("        Phase 3 measured the DETACHED joint moving MORE than the driven one.")

    P("\n--- ENABLE CYCLES (take-up signature over time) ---")
    for c in encyc:
        P(f"  cycle {c.get('cycle')}: sag {c.get('sag_px')} px, jump {c.get('enable_jump_px')} px, "
          f"first move {c.get('first_move_px')} px, second {c.get('second_move_px')} px")

    P("\n--- SPEED / ACCELERATION CHARACTERISATION ---")
    sc = [m for m in moves if m.get("test", "").startswith("speed_char")]
    combos = {}
    for m in sc:
        k = (m.get("joint"), m.get("combo"))
        combos.setdefault(k, []).append(m)
    P(f"{'joint':6s} {'combo':16s} {'n':>3s} {'settle med':>11s} {'px med':>9s}")
    best = {}
    for (j, c), ms in sorted(combos.items(), key=lambda x: (x[0][0] or 0, str(x[0][1]))):
        s = med([x.get("cam_settled_s") for x in ms])
        px = med([x.get("roi_changed_px") for x in ms])
        P(f"J{j:<5} {str(c):16s} {len(ms):3d} {str(s):>11s} {str(px):>9s}")
        if s is not None and (j not in best or s < best[j][1]):
            best[j] = (c, s)
    if best:
        P("  smoothest (shortest settle) per joint:")
        for j, (c, s) in best.items():
            P(f"    J{j}: {c}  settle {s}s")

    P("\n" + "=" * 70)
    P("  CLASSIFICATION")
    P("=" * 70)
    healthy, suspicious, faults, cannot = [], [], [], []
    for j, v in verdicts.items():
        name = f"J{j} {NAMES_LOCAL.get(j, '')}"
        if v["errors"] > 0:
            faults.append(f"{name}: {v['errors']} firmware command errors")
        if v["moves"] and v["dead"] / v["moves"] > 0.5:
            faults.append(f"{name}: {v['dead']}/{v['moves']} commanded moves produced no motion")
        elif v["directional"] == "YES":
            suspicious.append(f"{name}: dead-move rate differs by direction "
                              f"(+{v['dead_rate_pos']:.0%} vs -{v['dead_rate_neg']:.0%})")
        if "WORSE" in (v["settle_trend"] or ""):
            suspicious.append(f"{name}: settling time {v['settle_trend']} across the night")
        if v["errors"] == 0 and v["moves"] and v["dead"] / max(1, v["moves"]) < 0.2 \
                and "WORSE" not in (v["settle_trend"] or ""):
            healthy.append(f"{name}: {v['moves']} moves, {v['dead']} dead, settling {v['settle_trend']}")
    if len(unsettled) > len(moves) * 0.1:
        cannot.append(f"scene stability: {len(unsettled)} floor measurements never settled - "
                      f"ambient motion is competing with the signal")
    if holds:
        cannot.append("hold/hunting: no detached control was run inside this loop, and "
                      "Phase 3 showed the detached joint moving more than the driven one")
    cannot.append("absolute position: changed-pixel counts cannot be converted to degrees; "
                  "a fiducial marker is required before any accuracy claim")

    for title, items in (("PROVEN HEALTHY", healthy), ("SUSPICIOUS - NEEDS RETEST", suspicious),
                         ("PROVEN FAULT", faults), ("INSTRUMENT CANNOT DECIDE", cannot)):
        P(f"\n{title}:")
        if not items:
            P("  (none)")
        for i in items:
            P(f"  - {i}")
    P("")


NAMES_LOCAL = {0: "Base", 1: "Shoulder", 3: "Elbow", 4: "WristPitch",
               5: "WristRoll", 6: "Gripper"}

if __name__ == "__main__":
    main()
