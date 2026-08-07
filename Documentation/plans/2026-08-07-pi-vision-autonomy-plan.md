# Phased implementation plan — Raspberry Pi vision autonomy

**Source PRD:** `2026-08-07 Raspberry Pi Vision Autonomy PRD` (M0–M10, click-to-pick as the release gate)
**Written:** 2026-08-07, after a full bench session
**Status:** plan. Nothing below has been executed except where it says ALREADY DONE.

The PRD is well-aimed and this plan does not argue with its architecture. Perception decides where,
kinematics decides how, the Uno decides whether — that division is right and it matches what the
literature review the same day concluded independently
(`docs/research/2026-08-07-vla-training-feasibility.md`): the classical route is the one this arm can
actually walk.

What follows is the reconciliation the PRD asks for in §2 — what already exists, what moved today,
where the ordering can be cheaper, and what has to be decided by a human before code is written.

---

## 1. What changed today, after the PRD was written

Eighteen commits landed on `feat/telegram-voice-control` on 2026-08-07. Several PRD assumptions are
now stale in the project's favour.

| PRD assumption | Actual state as of tonight |
|---|---|
| "Webcam available/planned" | TWO cameras working and configured. `Software/arm-vision/cameras.csv` + `cameras.py` resolve them BY ROLE (`side`, `wrist`) via VID:PID capability probe, re-applying format/rotation/focus on every open because DirectShow forgets all of it |
| §10.9 stale status after serial loss is a real failure mode | Confirmed AGAIN today — the daemon was killed and the arm dropped while `arm_status.txt` still read `EN=1` on every joint. A freshness verdict already exists: `Software/arm-telegram/arm_link.py` returns LIVE / STALE / NO LINK and refuses motion on the latter two |
| Marker sizes rest on an assumed ~60° HFOV | Still true. `calibrate_intrinsics.py` is more built-out than the PRD implies — quality-gate criteria, coverage stats, per-view error, HFOV computation and a synthetic self-test |
| Gripper acks but does not close | Still true, and it is now the single highest-value blocker (see §4) |
| Base servo dead | Still true. Replacement ordered 2026-08-06, not fitted |

New tooling the PRD's `robot-brain/` should reuse rather than duplicate:

- `Software/tests/reply_cut.py` — the log-reply cut + three-state `clamped()`. Fixes a defect where a
  truncated reply read as CLEAN and a real clamp was invisible. Every tool that talks to the daemon
  now shares it.
- `Software/tests/goto_pose.py` — drives a named pose along its RECORDED entry path, phase by phase,
  stopping on clamp / stall / latch / unreadable reply. Proven both directions today at 5 °/s.
- `Software/arm-vision/dual_record.py` — synchronised two-camera episode recorder keyed to commanded
  joint state. Writes `metadata.json` + `transitions.jsonl` + `frames/`.
- `Software/arm-vision/correlate.py` — turns an episode into px-per-degree and image-deg-per-degree,
  with the phase-correlation response reported so a low-confidence number cannot pass as a
  measurement.
- `Software/arm-vision/wrist_map.py` — J4×J5 grid sweep + contact sheet, with a hard safe envelope
  narrower than `joint-limits.csv`.

---

## 2. Two places the PRD re-opens a question the operator already closed

Flagging once, then deferring to whatever the operator decides. Neither is re-litigated below.

**The shoulder mirror (PRD §10.2, M0).** The PRD says do not enable the opposed MG996R pair until the
neutral/mirror relationship is physically measured. `Documentation/RESUME-PROMPT.md` has this in its
🛑 SETTLED section as closed: the operator has run this joint on exactly this configuration, and
`joint-limits.csv` records `OK LIM J1 MIN=0 MAX=91 CAL=1` — *you cannot lock a range you have not
driven.* Today J1 was driven eight times across 1→88 with `MIR=INV OFF=0`, `JTO=0` throughout.
`mirror_offset_deg` is still 0 and still unmeasured; the operator's ruling is that this is worth
doing some day and is not a blocker. **Recommendation: keep it out of M0's exit criteria; treat
measuring the offset as an independent improvement.**

**The fuse (PRD §10.3, M0).** The PRD requires a fuse installed and verified before full-arm powered
work. The SETTLED section records the operator's ruling verbatim: *"Don't worry about the fuse. the
power is fine."* **Recommendation: strike it from M0's exit criteria.** It is the operator's call and
he has made it.

Two other M0 items are genuinely open and should stay: the supply voltage is still label data
(`_BENCH_SUPPLY_V = 5.0` is what is printed on the JCPOWER unit, not a meter reading), and J4's
locked 0–180 is the servo's whole electrical range with its mechanical ends never found.

---

## 3. Milestone-by-milestone: exists / partial / to build

| Milestone | State | What is actually left |
|---|---|---|
| **M0** bench safety | **PARTIAL** | Fit base servo + re-measure J0. Meter the supply. Find J4's real mechanical ends. Fix the gripper gear and lock its range. (Mirror + fuse: see §2) |
| **M1** Pi bring-up | **NOT STARTED** | Ubuntu 24.04 arm64, Python/OpenCV, serial discovery, service layout. ROS 2 — see §5 |
| **M2** Pi→Uno motion contract | **~80% EXISTS, wrong host** | The whole contract is built and proven on Windows: `SERIAL-PROTOCOL.md`, `hold_arm.py`, the file channel, `reply_cut`, `goto_pose`, the freshness verdict, `three_device_check`. M2 is largely a PORT, not a build |
| **M3** intrinsics + marker truth | **TOOLING EXISTS, never run on a real camera** | Print the ChArUco board, capture views, pass the gate, re-grade markers with measured intrinsics via `regrade_markers.py`, then generate the sheet. Blocked on nothing but doing it |
| **M4** world/robot frames | **NOT STARTED** | Needs a fixed camera looking at the work plane — see §4.2 |
| **M5** kinematics/URDF | **NOT STARTED, heaviest milestone** | No assembly CAD exists. Axis locations/directions/zeros must come from measurement + isolated-joint visual passes |
| **M6** click-to-move | **NOT STARTED** | See §5 for a cheaper ordering |
| **M7** click-to-pick | **BLOCKED on the gripper** | Everything else can proceed; this cannot |
| **M8** object perception | not started | Depends on M7 |
| **M9** language selection | not started | Depends on M8 |
| **M10** LeRobot | **UNPROVEN at both ends** | Plugin exists, never imported against a real LeRobot install, never run against a board |

---

## 4. The critical path, and what is genuinely blocking

### 4.1 The gripper is the whole product gate

M7 is the PRD's release gate and it needs a gripper that closes. J6 acks every command and the
fingers do not move; the operator's hand-check diagnosed a gear slipping on the shaft. **Nothing in
software fixes this and no amount of calibration routes around it.** It also blocks the alternative
autonomy routes in the research report (self-supervised grasping needs a grasp and a success signal).

Everything from M1 to M6 can proceed in parallel with the repair. M7 cannot start.

### 4.2 There is no camera looking at the work plane

The PRD's §5.3 work-plane approach — click a pixel, get robot X/Y — needs a **fixed camera viewing
the table**, ideally elevated and angled. What exists:

- **side** (laptop lid): fixed, but near-horizontal and low. A near-horizontal view of a table plane
  gives a badly conditioned homography — small pixel errors become large ground errors at range. It
  also points where the screen points, so aiming it at the table turns the display away from the
  operator.
- **wrist** (Arducam): on the gripper, so it MOVES. It is an eye-in-hand camera and cannot serve as
  the fixed workspace camera.

**This is a real gap in M4 that no existing asset closes.** Options: re-position the laptop to look
down at the table and accept losing the screen; or add one clamped USB camera above/beside the work
plane. `cameras.csv` already supports a third role — it costs a config row, not architecture. **This
is a decision for the operator, and it gates M4 and M6.**

### 4.3 M5 is the heaviest milestone and may be partly avoidable

See §5.

---

## 5. Two proposed re-orderings, with the reasoning

### 5.1 Uncalibrated visual servoing gets click-to-move without a URDF

The PRD sequences M5 (measured kinematics, URDF, FK, IK, round-trip validation) before M6
(click-to-move). M5 is the largest single body of work in the plan: there is no assembly CAD, so every
axis location, direction, zero and link transform has to be recovered by measurement.

There is a well-established route to click-to-move that needs none of it. **Uncalibrated visual
servoing** estimates the image Jacobian online from motion that already happened:

- Hosoda & Asada, IROS 1994 — recursive online estimator, no calibration.
- Jägersand, Fuentes & Nelson, ICRA 1997 — Broyden secant update, no dedicated calibration moves.
- Piepmeier, McMurray & Lipkin, ICRA 1999 / IEEE T-RO 2004 — quasi-Newton/RLS estimator.
- Sutanto, Sharma & Varma, RAS 1998 — injects deliberate small exploratory motions, separate from
  the task motion, purely to keep the Jacobian estimate well-conditioned.

That last paper describes the sweeps already recorded today. The loop is: click a target, measure the
error in image space, move one joint, observe which way the error moved, update the estimate, repeat.
It converges to the target without ever knowing a link length.

**Proposal: insert M5.5 — uncalibrated click-to-move — before M5.** It reaches the PRD's M6 exit
criterion (repeated clicks put the tool above intended points within a documented tolerance) using
`correlate.py` output that partly exists. It also de-risks M5: if uncalibrated servoing already
places the tool well enough for the training object, the URDF becomes an accuracy upgrade rather than
a prerequisite.

**What this does NOT replace.** A URDF is still wanted for MoveIt, for collision awareness, for
reasoning about reachability before moving, and for anything metric. This defers M5, it does not
delete it.

### 5.2 ROS 2 is not needed before M7

The PRD specifies ROS 2 Jazzy and tf2 in the M1 baseline, with MoveIt 2 explicitly deferred until
after model validation. For a 4-working-DOF arm on a Pi 4, driving click-to-pick, ROS 2 buys
tf2 frame bookkeeping and a node graph — real value, at the cost of a substantial install, a
build toolchain, and a second process model on top of the file-channel contract that already works.

**Proposal: treat ROS 2 as an M8+ decision, not an M1 requirement.** Everything through M7 can run as
plain Python with an explicit transform module. If and when MoveIt is wanted, ROS 2 arrives with it,
and the deterministic skill layer sits behind a node rather than being rewritten. This is a
recommendation, not a strong objection — if the intent is to learn ROS 2 as part of the project, doing
it at M1 is defensible and this plan does not fight it.

---

## 6. Recommended phase order

Phases, not milestones — several PRD milestones collapse or split.

**Phase A — unblock the bench (needs the operator's hands).** Gripper gear. Base servo + re-measure
J0. Meter the supply. Find J4's real ends. → satisfies the live parts of M0.

**Phase B — the fixed workspace camera (needs a decision, then 30 minutes).** Choose reposition vs.
third camera. Add the role to `cameras.csv`. Aim it at the work plane. → unblocks M4/M6.

**Phase C — intrinsics, and the marker unlock (no arm needed, can run now).** Print the ChArUco board,
capture views, pass the gate, publish the artifact, re-grade markers with real intrinsics, generate
the sheet. **This is the single most unblocking software task and it needs neither the arm nor the
Pi.** → M3.

**Phase D — Pi bring-up and the port (parallel with A–C).** Ubuntu 24.04, Python/OpenCV, serial
discovery, port the daemon + file channel + `reply_cut` + freshness verdict. Layer B tests from PRD
§12 with servo power off. → M1 + M2.

**Phase E — frames and the sign/scale table.** Camera→workspace→robot-base transforms with a
known-distance target. Isolated-joint visual passes for axis/sign — `dual_record.py --sweep` and
`correlate.py` already do this; run them for J1, J3, J4, J5 with a focused camera. → M4 + the
measurement half of M5.

**Phase F — uncalibrated click-to-move.** §5.1. → M6 by a cheaper path.

**Phase G — click-to-pick.** Approach clearance, open, slow descent, close, lift, fault handling.
Reuses `goto_pose.py`'s phase-and-verify structure. **Gated on Phase A's gripper repair.** → M7.

**Phase H — measured kinematics / URDF.** Now an accuracy and reachability upgrade rather than a
gate. → M5 proper.

**Phase I onward — object perception, language, LeRobot.** As the PRD sequences them. Note M10 needs
a real LeRobot install before the adapter can be called anything.

---

## 7. What must be decided by a human before code is written

1. **The fixed workspace camera** — reposition the laptop, or buy/clamp a third camera? Gates Phases
   B, E, F.
2. **ROS 2 at M1, or deferred to M8+?** §5.2. Changes the whole Pi bring-up scope.
3. **M0 exit criteria** — accept striking the mirror and fuse items per §2, or keep them?
4. **Where the wrist camera's focus should be pinned** — the value in `cameras.csv` is currently `-1`
   (unset) because 469 belonged to the old bench-edge mounting. Needs an object at the grasp distance
   and a re-sweep. Everything visual is provisional until then.

## 8. What this plan does not claim

Nothing here has been executed. The px-per-degree figures that Phase E builds on are provisional —
measured through an out-of-focus wrist camera with a phase-correlation response of 0.15, at the floor
of believability. No camera in this project has been calibrated, so no pixel figure converts to
millimetres. `MARKER-SYSTEM.md` §4 already reports that from a single camera at the standoff needed
to frame the arm, only the two BASE markers reach pose grade and everything past the turret is below
the detection floor — that is a real obstacle to the marker-based observation route in PRD §5.4, and
it is documented, not solved.

---

## 9. Outer context — the rover master plan

A second document arrived the same day: *Emre Kalem Autonomous Pickup Rover — Master Phase Plan*
(Phases A–E, ending at scheduled household cleanup missions). It does not compete with the Pi vision
PRD; it **contains** it. The rover plan's Phase A ("finish the stationary arm") is the Pi PRD's
M0–M7. Everything in this plan up to click-to-pick serves both.

Its two governing principles are the right ones and are adopted here without qualification:

> *"Prove manipulation before mobility. A rover that cannot reliably grasp objects is only a moving
> camera."*
> *"Do not buy a depth camera until the webcam-only floor-plane approach has been tested."*

### 9.1 It resolves the open camera question from §4.2

§4.2 flagged that no existing asset gives a fixed camera viewing the work plane, and left the choice
open. The rover plan settles it by architecture: the webcam's job is *"what is on the floor / where
is the object / where should the gripper go"*, and on the rover it is **chassis-mounted looking
forward and down at the pickup zone**. It is not a wrist camera, and the rover architecture never
asks for one.

**Recommendation: move the Arducam off the gripper onto a fixed mount looking down at the pickup
zone.** Reasons, in order:

1. It is the camera that can actually be calibrated well — MJPG, manual focus that holds, 4K-capable.
   The laptop lid camera refuses MJPG, caps at 720p, has no focus control at all, and points where
   the screen points.
2. A fixed downward view is the geometry both documents' pixel→floor-plane maths assumes. The lid
   camera's near-horizontal view gives a badly conditioned homography.
3. It costs nothing. No third camera, no purchase.
4. It matches the rover's final geometry, so the bench calibration procedure is the one that will be
   re-run after Phase D rather than a throwaway.

What is given up: the eye-in-hand view. That was a good experiment — the POV map and the J5
sign/scale measurement both came from it — but **neither PRD requires it**, and grasp confirmation
can come from the fixed camera watching the gripper close. Keep the wrist mount; it can go back on
later for grasp verification once the fixed pipeline works.

### 9.2 Its empirical reach map is a bigger simplification than §5.1

§5.1 proposed uncalibrated visual servoing to defer the URDF. The rover plan §5 goes further and is
better for this arm: **teach the software where the real gripper lands for a set of safe, observed
joint configurations**, and interpolate between them. It states the reason exactly right — that
automatically captures print tolerance, servo-horn offsets, actual assembly and linkage variation,
none of which loose STLs can supply.

That is a **taught reach map**, and it sidesteps M5 (measured kinematics, URDF, FK, IK) entirely for
v1 rather than merely deferring it. For a 4-working-DOF arm on a fixed table it is very likely
sufficient, and it is far less work than recovering a kinematic model by measurement.

**Recommendation: build the taught reach map first; keep uncalibrated visual servoing as the
refinement that corrects residual error near the target; treat the URDF as Phase H, wanted for
MoveIt and reachability reasoning rather than for picking things up.**

### 9.3 A direct conflict between the two documents

- Pi vision PRD §4: **Ubuntu 24.04 + ROS 2 Jazzy**.
- Rover master plan §22: **Ubuntu 22.04 + ROS 2 Humble**, on stated stability grounds.

Both cannot be the baseline. This plan's §5.2 position makes the conflict non-urgent: **nothing
through click-to-pick needs ROS 2 at all**, so the decision can be deferred to the point where Nav2
and SLAM Toolbox are actually being installed — which is rover Phase C, well after the bench arm is
finished. Deciding it then also means deciding it against whatever is current at that time rather
than now.

### 9.4 Reuse, not re-creation

Rover plan §27 proposes `Software/arm-autonomy/` containing, among others, `arm_link.py`. **That file
already exists** at `Software/arm-telegram/arm_link.py` — stdlib-only, with the log-offset primitive,
the reply cut and the LIVE/STALE/NO-LINK freshness verdict already in it. `Software/tests/reply_cut.py`
holds the shared cut + three-state `clamped()`. Import them; do not write a third copy. Three private
copies of that exact logic is what produced the defect fixed in `fd97760`.

`geometry.py` / `reach_map.py` / `calibrate_floor.py` / `teach_reach.py` are genuinely new and belong
in the proposed package.

### 9.5 The fuse, again

Rover plan §29 Phase A item 6 ("fit intended fuse") and §28 readiness gates both require it, as did
the Pi PRD. Same note as §2 of this plan: the operator's recorded ruling is *"Don't worry about the
fuse. the power is fine."* Three documents now ask for it and the operator has closed it once.
Recorded here so the next reader does not raise it a fourth time; it is his call and he has made it.
