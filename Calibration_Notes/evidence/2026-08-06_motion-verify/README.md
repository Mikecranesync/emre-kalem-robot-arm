# Camera-verified motion, three joints — 2026-08-06 ~19:15

Run with `Software/tests/motion_verify.py` against the live holder daemon
(`hold_arm.py`, COM5), camera = the laptop's internal `Integrated Camera`, operator's
white paper backdrop in place, whole arm in frame.

**Pass bar was fixed before the runs, not after:** at every waypoint transition the
changed-pixel count inside the joint's ROI must beat that same waypoint's own
noise floor by ≥4×, where the floor is two frames captured 0.5 s apart at the
*same* command. Both frames of each pair are adjacent in time because the arm
settles downward while held, so frames minutes apart are not comparable.

| Joint | Waypoints | Result | Signal px | Noise floor px | Ratio | Global shift |
|---|---|---|---|---|---|---|
| **J5 wrist roll** | 104 → 70 → 140 → 104 | **3/3 MOVED** | 10221 / 13487 / 10533 | 6 / 7 / 0 | 1700–1930× | 25–60 px (expected: roll swings the whole gripper) |
| **J4 wrist pitch** | 90 → 75 → 90 → 105 → 90 | **4/4 MOVED** | 1852–2286 | 1–4 | 463–2286× | 0.3–0.56 px (articulation, not arm swing) |
| **J6 gripper** | 10 ↔ 70, two cycles | **NOT ARTICULATING** | 27–60 | 2–17 | 1.4–1.8× | <0.5 px |

`JTO=0` on every waypoint of every run — no joint timeout, no stall. Every `MOV`
returned `CL=0` and every `STA` confirmed `SET=<commanded> MOV=0`. The daemon did
not intervene (`re-ENA`/`LATCHED`) inside any measured window.

## J6 does not open and close any more, and this is not a measurement artifact

Five hours earlier (`calibration-log.csv`, J6 row, 14:53) the operator watched this
gripper open and close and the camera measured 1799/1657 changed px against a
272 px control. Tonight, on the same board and the same command channel:

1. **Frame differencing:** commanded 10 vs commanded 70 gives 27–60 changed px
   against noise floors of 2–17. Ratio 1.4–1.8×, nowhere near the 4× bar.
2. **Geometry, independent of illumination and of differencing:** the dark prong
   silhouette is *identical* at both commands — area 2626 vs 2618 px, bounding-box
   height 73 vs 73 px, median finger span 55 vs 55 px.
3. **The diff heatmap** (`dbg_heat.png`) shows an edge band around the *entire*
   gripper outline rather than change localised at the finger tips — the signature
   of a small rigid displacement of the whole assembly, not of fingers moving.
4. **Something does move during travel.** Filmed travel windows peak at 577–1529
   changed px, then the assembly returns to the same silhouette
   (`J6f_02_closed_10_filmstrip.png` shows the whole assembly translating while the
   finger gap holds constant). So the servo is doing *something* — it is just not
   ending with the fingers in different positions.
5. **J5 and J4 are the control.** Same harness, same camera, same ROI scale, same
   session: they resolve at 460–1930×. A harness that measures 13,487 px on a roll
   command and 27 px on a gripper command is not failing to see motion.

**Not diagnosed.** A slipped horn on the splined shaft, a loose gripper-body
mounting screw letting the servo's reaction torque rotate the body instead of the
fingers, a disconnected linkage, or a dead/de-powered servo would all produce
exactly this. Nothing in this system observes the shaft, so the camera cannot
separate them — the operator's eyes can. **Ask what he sees while it runs.**

## Harness defects found and fixed during these runs

- Two waypoints sharing a label overwrote each other's PNGs, leaving 3 evidence
  files for 5 waypoints. Names are now index-prefixed.
- Ratio alone was not a sufficient gate: a 70 px edge shimmer against a freakishly
  quiet 2 px floor scored 35× and printed **MOVED** on the first gripper run, with
  no articulation in the frames at all. There is now an absolute floor
  (`MIN_SIGNAL_PX`) that must pass *as well as* the ratio.
- Settle-point frames alone cannot tell "never moved" from "moved and came back".
  `--film` captures the travel window and reports its peak.
