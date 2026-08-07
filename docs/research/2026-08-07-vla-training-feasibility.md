# Can this arm be trained to act autonomously, and how?

**Date:** 2026-08-07
**Question asked:** we are driving the arm through poses and sweeps, recording the wrist and side
cameras against commanded joint angles. Is that on track to train a VLA, and what is the strategy
for giving this robot autonomy?

---

## The verdict, first

**The data we are collecting will not train a VLA or any imitation-learned policy, and the reason
is not that we collected it badly — it is the wrong KIND of data.** Imitation learning treats the
action as a *label*, and a label is only worth training on if it came from someone competent doing
something purposeful. A scripted joint sweep has no intent behind it, so there is nothing to
imitate.

**But the same data is exactly the right shape for three other things** — camera and hand-eye
calibration, a dynamics/world model, and the sign-and-scale table that image-based visual servoing
needs. Those are not consolation prizes. The third one is a genuine route to autonomous reaching on
this arm, and today's wrist grid is already most of the data it needs.

**The binding constraints are not algorithmic.** They are: a gripper that does not close, no
position feedback of any kind, and no way to record a human driving the arm. Every learned-policy
method below requires proprioception in its observation vector, and this arm structurally cannot
supply it. Fixing that is a hardware question, not a training question.

---

## 1. What we collected today, and what it is worth

Two datasets exist as of this writing:

| Run | Content | Honest description |
|---|---|---|
| J5 sweep, 104→134 in 6° steps | 6 steps, both cameras, commanded angle + board reply | image-space sign/scale for one joint |
| Wrist POV grid, J4×J5 | 15 cells (J4 78/90/102 × J5 50→150), both cameras, contact sheet | orientation→view map at one arm pose |

Both are **commanded-angle records**. Nothing in this system observes a shaft, so neither is a
record of where the wrist *was*. Both were driven from `storage`, which means the gripper camera was
looking at the room — whiteboard, garage door, floor — and never at the work surface.

**What they are good for:**

- **Hand-eye calibration.** Solving `AX=XB` — the fixed transform between camera and gripper — is
  only observable if the motion set contains **at least two rotations about non-parallel axes**
  (Enebuse et al. 2021; the requirement traces to Tsai & Lenz 1989). J4 pitch and J5 roll are
  perpendicular, so a J4×J5 grid satisfies that condition almost by construction. Parallel-axis-only
  motion is the degenerate case and is the usual reason a hand-eye solve returns garbage. **The grid
  we ran is close to the right shape already** — it needs calibrated intrinsics and a calibration
  target in view, not a different motion plan.
- **Visual servoing gains.** px-per-degree and image-degrees-per-degree per joint. The J5 sweep
  produced 14.56 px/deg on the wrist camera and 0.02 px/deg on the side camera — a wrist roll is
  nearly invisible side-on. Both figures are **provisional**: the wrist camera was out of focus and
  the phase-correlation response was 0.15, at the floor of believability.
- **A dynamics / world model.** See §3.

**What they are not good for:** training a policy. See §2.

---

## 2. Why scripted sweeps do not train a policy

This is settled in the literature and worth stating with the sources, because it is the whole crux.

**The mechanism.** Behaviour cloning is supervised learning where the demonstrator's action is the
target. Ross, Gordon & Bagnell's DAgger paper (arXiv:1011.0686) established the failure mode
formally: *"Sequential prediction problems such as imitation learning, where future observations
depend on previous predictions (actions), violate the common i.i.d. assumptions made in statistical
learning."* Errors compound, and the policy needs expert action labels covering the states it will
actually visit. A taskless sweep supplies neither expert actions nor task-relevant states.

**"Play data" is the closest thing, and it is still human-driven.** Lynch et al., *Learning Latent
Plans from Play* (arXiv:1903.01973), is the canonical unstructured-data paper, and it is explicit
that play is **human teleoperated**: *"we collect a robot play dataset by allowing a user to
teleoperate the robot in a playground environment."* On the scripted alternative it is blunt:
*"sampling random actions is very unlikely to traverse through more complex manipulations by
chance."* And on why the human is load-bearing: *"A human operator provides the necessary properties
of curiosity, boredom, and affordance priors to guide rich object play."*

The reason play works and sweeps do not is hindsight relabelling: play is sliced into
(state, later-state-as-goal, actions-between) windows, which only teaches anything if the actions in
between *accomplished something*. Scripted motion overwhelmingly does not.

**Data quality is formalised around exactly this.** Belkhale, Cui & Sadigh, *Data Quality in
Imitation Learning* (arXiv:2306.02437), defines dataset quality along **action divergence** (how far
recorded actions are from what a good policy would do) and **transition diversity**. Sweep data is
maximally divergent on the first axis.

**What the flagship models actually trained on:**

| Model | Action-training data | Non-demonstration data? |
|---|---|---|
| RT-1 (arXiv:2212.06817) | 130k demos, 13 robots, 17 months. *"Demonstrations are collected with direct line-of-sight between operator and robot using 2 virtual reality remotes."* | none |
| RT-2 (arXiv:2307.15818) | reuses RT-1's set; web/VQA data is 34–50% of the mixture but exists only to preserve the VLM's semantics | none for actions |
| DROID (arXiv:2403.12945) | 76k VR-teleoperated demos. The randomness is in *which task is assigned*, not in the motion | none |
| OpenVLA (arXiv:2406.09246) | 970k demos curated from Open X-Embodiment; dropped DROID entirely for the final third of training when action-token accuracy stayed low | inherited, then suppressed by filtering |
| π₀ (arXiv:2410.24164) | 903M timesteps, 68 tasks, ALOHA-style teleoperation. A full-text search for "random", "exploration", "play", "scripted" returns **zero matches** | none |

**One honest nuance.** Open X-Embodiment *does* contain non-teleoperated data — QT-Opt (autonomous
RL) and Task Agnostic Robot Play are constituent datasets. So the flat claim "no VLA touches
exploration data" is wrong. But they are a minority slice of a million-trajectory teleop-dominated
mixture, and OpenVLA's Octo-derived weighting actively down-weights small, low-diversity sets. The
conclusion survives the nuance; the nuance should not be hidden.

---

## 3. Where undirected data IS the right answer

The requirement inverts completely for world models, and this is the part worth internalising.

**PlaNet (arXiv:1811.04551) and Dreamer (arXiv:1912.01603) both SEED their replay buffer with purely
random rollouts.** From Dreamer's Algorithm 1: *"Initialize dataset D with S random seed episodes."*
Data collection thereafter is the actor's prediction plus `Normal(0, 0.3)` exploration noise — never
purely greedy.

The reason is structural. A world model learns `p(next state | state, action)`. The action is an
*input variable*, not a label, so **any** action-outcome pair is valid signal regardless of whether
the action was any good. What such a model needs is broad state-action coverage so it generalises
off the optimal manifold — which is precisely what expert-only data lacks.

- **Plan2Explore (arXiv:2005.05960)** makes the split explicit: a task-free exploration phase trains
  the world model, then a task specified *afterwards* is solved by planning inside it.
- **DayDreamer (arXiv:2206.14176)** ran Dreamer on real robots with **zero simulation and zero
  demonstrations** — a quadruped learned to walk in an hour, arms learned pick-and-place from sparse
  reward.
- **Self-supervised grasping needs no demonstrations either.** Levine et al. (arXiv:1603.02199)
  trained grasping from **800,000 autonomous attempts**; Pinto & Gupta (arXiv:1509.06825) from
  **50,000 tries over 700 robot hours**. Both self-supervised, no human demonstrator.

That last pair is the most hopeful result for this project — and it needs a gripper that closes and
a success signal, which the wrist camera could plausibly provide.

---

## 4. The three structural blockers

Ranked by how much they block, not by how hard they are.

### 4.1 No proprioception — the one that decides the architecture

Every method above conditions on robot state:

- **ACT/ALOHA** — 14-DoF joint positions in the observation; actions are **absolute joint positions**.
- **Diffusion Policy** — *"All experiments include proprioceptive end-effector information."*

This arm has none. `Servo.read()` returns the last commanded value. Feeding a policy the number you
just sent, and asking it to predict the number you just sent, is not learning. On 5 V hobby servos
under gravity load the gap between commanded and actual is real and unbounded — the arm demonstrably
sags while held.

Three honest options: accept commanded-as-state and know the model is learning a fiction; estimate
pose from the cameras via the fiducial markers already designed in `Software/vision/markers.csv`
(which needs a calibrated camera first); or change to servos that report position.

### 4.2 The gripper does not close

J6 acks every command and the fingers do not move — the operator's hand-check diagnosed a gear
slipping on the shaft. **No grasp means no pick-and-place**, which is the task family every one of
these methods is built and benchmarked on. There is no task to demonstrate and no success signal to
self-supervise against. This blocks both the imitation route and the self-supervised route.

### 4.3 No way to record a demonstration

Nothing in the repo captures a human driving the arm. This is the cheapest to fix and the fix is
already half-built: **the console has a working joystick**, and `JOG` is implemented in the
firmware. Recording joystick sessions into the episode format `dual_record.py` already writes would
turn it into a demonstration collector.

Also worth knowing: J0's servo is dead, so this is currently a 4-DoF arm (J1, J3, J4, J5) — a planar
shoulder/elbow plus a two-axis wrist. That is a small workspace to define tasks in.

---

## 5. How much data would actually be needed

Because the answer is smaller than people expect, and it is **not** the constraint here:

- **ACT/ALOHA (arXiv:2304.13705):** 50 demonstrations per task, 100 for the hardest of six. Episodes
  are 8–14 s. Four cameras at 480×640 — two static, two wrist-mounted.
- **Diffusion Policy (arXiv:2303.04137):** 136 demos for real Push-T; 50 each for 6-DoF Pour and
  Periodic Spread.
- **LeRobot official guidance:** *"We suggest recording at least 50 episodes, with 10 episodes per
  location."* `--dataset.num_episodes` defaults to 50. SmolVLA repeats the ~50 figure and reports
  that 25 was *"not enough leading to a bad performance."* A community SO-101 write-up needed 150.
- **Pretrained visual encoders** (R3M arXiv:2203.12601, VIP arXiv:2210.00030) cut the requirement to
  roughly 20 demos per task — **never to zero**. R3M's real-robot result still used 20; TCN's most
  demo-light mode still needs one.

50–150 episodes at ~10 s each is a weekend of teleoperation. **Data volume was never the blocker.**

---

## 6. Three routes to autonomy, ranked

### Route A — classical visual servoing on this arm. No learning at all.

Calibrate intrinsics (ChArUco board and `calibrate_intrinsics.py` already exist), pin the wrist
focus at the grasp distance, hand-eye calibrate using a J4×J5 grid like today's, then close a
proportional loop: drive the target to the centre of the wrist image, one joint at a time, using the
measured sign and scale.

- **Needs:** no demonstrations, no proprioception, no GPU. A working gripper only for the final grasp.
- **Gets you:** the arm autonomously reaching to a seen object. That is real autonomy by any
  reasonable definition.
- **This is what the arm is genuinely well suited to, and today's data is already most of the way
  there.**

### Route B — self-supervised grasping on this arm

Fix the gripper, then run autonomous grasp attempts with the wrist camera as the success signal, per
Levine et al. and Pinto & Gupta.

- **Needs:** working gripper, a success detector, and a great many trials. Pinto & Gupta needed 700
  robot hours; Levine needed 800k attempts across a fleet.
- **Honest risk:** MG90S and MG996R servos at 5 V, hundreds of hours of repeated grasping. The
  gripper has already failed once mechanically.

### Route C — buy the arm the tutorials are written for

ACT is robot-agnostic; it was demoed on a ~$20,000 bimanual rig but **LeRobot has ported it to the
SO-100/SO-101 at roughly $100–250**. Those use serial-bus servos with real position feedback and a
working gripper, and they are the reference platform every LeRobot dataset, tutorial and
episode-count recommendation targets.

- **Needs:** ~$150–250 and a weekend of teleoperation for 50 episodes.
- **Gets you:** a trained policy doing autonomous pick-and-place, on the well-trodden path.
- This is **not** an argument to abandon the Emre Kalem arm. The protocol work, the camera stack,
  the calibration and the safety discipline all transfer. It is an argument that if the goal is a
  *trained policy*, the cheapest path is hardware that already has what this arm structurally lacks.

**Recommendation: A now, C in parallel if a trained policy is the actual goal, B only after the
gripper has proven durable.**

---

## 7. Next concrete steps

1. **Fix the J6 gripper gear.** It blocks Routes A (grasp), B (entirely) and any task definition.
2. **Pin the wrist focus at the grasp distance** — put an object where it would be picked up, sweep,
   record the value *and* the measured distance. Everything visual is provisional until this is done;
   today's correlation had a phase-correlation response of 0.15.
3. **Calibrate intrinsics** with the ChArUco board. This also unblocks printing the marker set, which
   is currently forbidden because every marker size rests on an assumed ~60° field of view.
4. **Hand-eye calibrate** using a J4×J5 grid with a target in view — the motion plan already exists.
5. **Re-run the sign/scale sweeps** for J1, J3, J4, J5 with a focused camera, and build the full
   table.
6. **Record joystick sessions** into the episode format. Even if no policy is ever trained, that is
   the only path that produces demonstration data, and the joystick already works.

---

## What is not proven in this document

Nothing here was tested on this arm. The px-per-degree figures are provisional (out-of-focus camera,
correlation response 0.15). No camera in this project has been calibrated, so no pixel figure
converts to millimetres. The claim that markers can substitute for proprioception is untested here —
`MARKER-SYSTEM.md` §4 already reports that from a single camera at the standoff needed to frame the
arm, only the two BASE markers reach pose grade and everything past the turret is below the
detection floor. That is a real obstacle to the marker route and it is documented, not solved.

## Sources

Every URL below was fetched and verified during this research; none are recalled from memory.

DAgger https://arxiv.org/abs/1011.0686 · Data Quality in IL https://arxiv.org/abs/2306.02437 ·
RoboMimic https://arxiv.org/abs/2108.03298 · LMP https://arxiv.org/abs/1903.01973 ·
C-BeT https://arxiv.org/abs/2210.10047 · MimicPlay https://arxiv.org/abs/2302.12422 ·
PlaNet https://arxiv.org/abs/1811.04551 · Dreamer https://arxiv.org/abs/1912.01603 ·
DreamerV3 https://arxiv.org/abs/2301.04104 · Plan2Explore https://arxiv.org/abs/2005.05960 ·
DayDreamer https://arxiv.org/abs/2206.14176 · World Models https://arxiv.org/abs/1803.10122 ·
Learning to Poke https://arxiv.org/abs/1606.07419 · Visual Foresight https://arxiv.org/abs/1812.00568 ·
RoboNet https://arxiv.org/abs/1910.11215 · Levine grasping https://arxiv.org/abs/1603.02199 ·
Pinto & Gupta https://arxiv.org/abs/1509.06825 · RT-1 https://arxiv.org/abs/2212.06817 ·
RT-2 https://arxiv.org/abs/2307.15818 · OXE https://arxiv.org/abs/2310.08864 ·
DROID https://arxiv.org/abs/2403.12945 · OpenVLA https://arxiv.org/abs/2406.09246 ·
π₀ https://arxiv.org/abs/2410.24164 · ACT/ALOHA https://arxiv.org/abs/2304.13705 ·
Diffusion Policy https://arxiv.org/abs/2303.04137 · R3M https://arxiv.org/abs/2203.12601 ·
VIP https://arxiv.org/abs/2210.00030 · TCN https://arxiv.org/abs/1704.06888 ·
Zhang calibration https://www.microsoft.com/en-us/research/wp-content/uploads/2016/02/tr98-71.pdf ·
Tsai & Lenz https://kmlee.gatech.edu/me6406/handeye.pdf ·
Swevers excitation https://lirias.kuleuven.be/server/api/core/bitstreams/ead2fc07-2a23-4cf6-a97c-70310c363294/content ·
LeRobot IL docs https://huggingface.co/docs/lerobot/il_robots
