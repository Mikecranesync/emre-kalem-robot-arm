# 2026-08-08 ~20:55 — elbow "bounce": not caught, and the arm creeps

The operator reported that a servo bounces "sometimes", and on being asked, placed it
**while the joint is holding still**, on **the elbow**. That rules out a motion-profile
fix: a smoothing / easing profile shapes a joint while it TRAVELS and does nothing for
one that is parked. The symptom described is **hunting** — a servo whose internal loop
cannot hold position inside its deadband under load, so it corrects, overshoots,
corrects back, and limit-cycles.

## What was run

`Software/tests/hunt_watch.py` — nothing is commanded; the arm is parked and the question
is only whether it STAYS parked. ROI is the claw (`300,590,480,719`) deliberately: it is
the far end of the kinematic chain, so a degree of elbow hunt swings it further than any
ROI nearer the base. 900 s, sampled every 0.4 s.

Pose during the watch: J0 90, J3 (elbow) 15, J4 50, J5 90, J6 90 — **J1 detached**, held
by `hold_arm_pi.py`.

```
1600 samples over 900s
peak adjacent-frame 26 px   peak vs-start 134 px
frames over 30 px: 0
VERDICT: no hunt in this window -- arm held still
```

## Two findings

**1. No hunt, at this sensitivity, in this pose.** 1600 samples, nothing over the 30 px
flag, peak frame-to-frame 26 px. Combined with the earlier 2-minute idle test, that is
~17 minutes of this pose with no oscillation. The operator said "sometimes", so this is a
null in one pose and one window — NOT a refutation of what he saw.

**This also rules out one hypothesis that had been raised and should not be repeated:**
that the daemon's 5 s `STA` poll was jittering the AVR's Timer1 servo pulses. Sampling at
0.4 s for 900 s would have caught a 5-second periodic excursion about 180 times over. It
caught none. Sub-threshold jitter remains possible but there is no evidence for it, and
it should not be presented as a fix waiting to be confirmed.

**2. The arm creeps slowly while held — 134 px over 900 s, with J1 detached.** Not
oscillation (adjacent frames stayed under 26 px); slow, roughly monotonic drift. This is
a **measurement-validity note for every future long run**: a multi-minute waypoint
sequence accumulates drift that a short control pair will not reveal, because the control
pair is two frames 0.5 s apart. It is small — see the arithmetic in
`../2026-08-08_J6-retry/README.md`, where it is bounded at ~20–30 px across that run
against a 1,073–1,440 px effect — but it is not zero, and it grows with run length.

## Why the obvious next test was NOT run

The tempting move is to sweep the elbow across its 0–30 envelope and hold at each angle,
on the theory that the current near-vertical pose unloads it and an extended pose would
provoke the hunt. That was deliberately not done:

- The link geometry is not modelled here, so which elbow angle actually extends the
  forearm is a guess.
- **J1 is limp.** The pose that maximises elbow torque is also the pose most likely to
  back-drive the unpowered shoulder, and the failure mode of that is the arm falling.
- The operator can describe the pose he saw it in, in one sentence, which is cheaper and
  safer than searching a space we cannot model on the one joint whose failure mode is a
  drop.

## Open, and needing the operator rather than more instrumentation

1. **Was the bounce seen tonight, or is it from an earlier session?** If it predates the
   header rewiring or the old 6.62 V supply, it may already be fixed.
2. **What pose was the arm in?**

## If it is confirmed live, the ranked fixes — none of which is a smoothing program

1. **Supply voltage.** The elbow is an MG996R spec'd 4.8–7.2 V running at **5.0 V**, the
   bottom of its window; `joint-limits.csv` already predicts "roughly 10-15 percent less
   holding torque". Headroom is 5.0 → **6.0 V and no further**: the wrist and gripper
   MG90S servos top out at 6.0 V, and this bench has already been burned once by an
   over-spec 6.62 V supply.
2. **Unload the elbow mechanically** — counterbalance spring or equivalent. Attacks the
   cause; cheap and reversible.
3. **Get J1 carrying its share.** Blocked on measuring `mirror_offset_deg`, still 0 and
   still a placeholder.
