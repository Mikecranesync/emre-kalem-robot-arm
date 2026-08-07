# Can this arm's data collection plan train a VLA? No — and not because of one fixable gap

**Short answer: not with this hardware, and not with this data collection plan even on better
hardware.** Two independent problems stack, and either one alone kills it. First, the proposed data
— scripted joint sweeps with no task and no operator intent — is not the kind of data imitation
learning trains on, regardless of what robot collects it; every model surveyed below learns to
reproduce *demonstrated* actions, and a sweep demonstrates nothing. Second, even if real
demonstrations were collected, this arm has no position feedback on any joint, a base joint that is
dead, a gripper that does not close, two uncalibrated cameras, and — by this project's own spec — a
data-collection procedure its authors already concluded is "not safely automatable... until a marker
observer exists" (`docs/lerobot_emre_arm_spec.md` §5.7). None of that is a training-data problem.
It is a robot problem, and it has to be fixed before any training-data problem is worth discussing.

What follows is not "this will never work." Four of six logical joints move, camera hardware exists,
and the LeRobot adapter's observation contract (`Software/lerobot_robot_emre_arm/`) was deliberately
built honest about what it doesn't know. The realistic path is narrower and slower than "collect
sweeps, train a VLA" — it runs through calibration and classical control first. §6 ranks it.

---

## 1. What VLAs and learned manipulation policies actually require as training data

Every model below is trained the same way at its core, whatever the marketing copy says: a dataset
of `(observation, action)` pairs recorded from something that was actually trying to do a task, and a
loss that pulls the policy toward reproducing the recorded action given the recorded observation. The
differences are in scale, architecture, and how much they condition on robot state — not in that
basic contract.

### OpenVLA (Kim et al., 2024, arXiv:2406.09246)

- **Training data:** 970,000 real-world robot demonstrations (Open X-Embodiment), evaluated across 29
  tasks and multiple embodiments. Verbatim from the abstract: "trained on a diverse collection of
  970k real-world robot demonstrations."
- **Observation:** single RGB image at 224×224px, **no proprioceptive input in the default
  configuration** — confirmed by the paper's own text ("no proprioceptive information," §5.2) and by
  the model's `predict_action()` call signature, which takes only `input_ids` (language) and
  `pixel_values` (image) — nothing else. The paper's own Limitations section (§6) names this a gap,
  not a validated choice: *"Expanding OpenVLA to support multiple image and proprioceptive inputs...
  is an important avenue for future work."*
- **Action space:** each dimension discretized into 256 bins, predicted as relative (delta)
  end-effector control (§3.2, §5.2). Backbone: 7B parameters, Llama 2 + a fused DINOv2/SigLIP visual
  encoder.
- **Fine-tuning:** the paper states LoRA fine-tuning is practical "on consumer GPUs... without a hit
  to downstream success rate," but does not enumerate exact GPU-hours or demo counts in the abstract
  or the sections fetched for this report — **not independently verified in this session; do not
  treat a specific hour figure as sourced here.**
- A follow-up, **OpenVLA-OFT** (Kim/Finn/Liang 2025, arXiv:2502.19645), adds proprioception back via a
  small MLP — but only when fine-tuning to one specific robot, not in the shared pretraining stage.
  That split (proprioception-free generalist pretraining, proprioception-aware single-robot
  fine-tuning) is a recurring pattern, not a one-off.

### π0 / pi-zero (Physical Intelligence, 2024, arXiv:2410.24164; pi.website/blog/pi0)

- **Training data:** described by Physical Intelligence's own announcement as "the largest robot
  interaction dataset to date," collected across **8 distinct robot platforms** (UR5e, Bimanual UR5e,
  Franka, Bimanual Trossen, Bimanual Arx, Mobile Trossen, Mobile Fibocom). Exact hour-count or
  demonstration-count **not disclosed** in the sources checked for this report — the blog explicitly
  stops short of quantifying it.
- **Observation — and this is the opposite of OpenVLA:** π0 explicitly conditions on proprioceptive
  state. The paper's own formula (§IV): observation `o_t = [I_t¹, …, I_tⁿ, ℓ_t, q_t]` — multiple RGB
  images, language, **and joint-angle state `q_t`**, all as explicit inputs.
- **Architecture:** a ~3B-parameter PaliGemma VLM backbone plus a smaller flow-matching "action
  expert" (470M in the published "π0-small" variant; other reporting on the full model puts the
  action-expert head around 300M) sharing one transformer via blockwise causal attention — image and
  language route through the large block, state and noised-action tokens through the small one, and
  the two interact only through attention. Output: continuous 50-step action chunks, closed-loop up
  to 50 Hz.
- **Minimum demos for a new task:** not found stated as a number anywhere in this report's sources.

### RT-1 (Brohan et al., 2022, arXiv:2212.06817) and RT-2 (Brohan et al., 2023, arXiv:2307.15818)

- **RT-1 training data:** ~130,000 episodes, collected over 17 months with a fleet of 13 robots,
  covering 744 distinct task instructions across 12 skill types (§5.2, Table 1).
- **RT-1 observation:** a 6-image temporal history (6 timesteps, not 6 cameras) at 300×300px through
  EfficientNet-B3, plus a language instruction. **No proprioceptive input is described anywhere in the
  paper.** Camera count/model is not stated in the primary text.
- **RT-1 action space:** 11 total dimensions (7 arm + 3 mobile base + 1 mode-switch), each discretized
  into 256 bins (§5.1).
- **RT-1's own data-quantity ablation is the single most directly useful number in this literature for
  sizing a collection plan** (§6.5, Table 7, fetched directly and quoted verbatim below):

  | % tasks kept | % data kept | Seen-task success | Unseen-task success |
  |---|---|---|---|
  | 100 | 100 (full) | 97 | 76 |
  | 100 | 51 | 71 | 52 |
  | 100 | 37 | 55 | 57 |
  | 100 | 22 | 59 | 29 |
  | 75 | 97 | 86 | 67 |

  The last row is the paper's own point made concrete: cutting to 75% of the *tasks* while keeping
  97% of the *per-task data* (86%/67%) beats keeping all 100% of the tasks but cutting to 51% of the
  data (71%/52%) — fewer tasks with ample examples each generalizes better than more tasks with data
  spread thin. The paper's own conclusion: **"data diversity is more essential than data quantity."**
- **RT-2:** reuses RT-1's robot-action dataset (no new episode count reported), co-fine-tuned on a
  ~1B-example filtered subset of the WebLI web image-text corpus (Appendix B). Action space is
  described slightly differently (a 6-DoF delta-pose + gripper + terminate framing, §3.2) — not a
  verbatim restatement of RT-1's 11-dim scheme. No camera or proprioception details beyond RT-1's.
  **No specific "minimum demonstrations for a new skill" number is published anywhere checked.**

### ACT / ALOHA (Zhao et al., 2023, arXiv:2304.13705) — the LeRobot-family default

- **Demonstrations: 50 per task, 100 for the hardest one** ("Thread Velcro") — direct quote from §V-B:
  *"We record 50 demonstrations for each task, except for Thread Velcro which has 100."* Roughly 10
  minutes of teleoperation per task in the abstract's framing.
- **Observation:** RGB images plus **14-dimensional joint-position proprioception (`qpos`)**, fused as
  a single learned token that attends against every image patch (not concatenated into the image
  features) — confirmed at the code level (`detr/models/detr_vae.py`,
  `github.com/tonyzhaozh/act`). Joint *velocity* (`qvel`) is collected but **never reaches the
  policy** — dropped before the model's forward pass.
- **Action space:** action *chunks* — sequences of future joint-angle targets predicted jointly,
  the "action chunking" the name refers to — not single-step actions.
- **Hardware:** designed around low-cost teleoperated bimanual arms (ALOHA), the direct ancestor of
  LeRobot's SO-100/SO-101/Koch reference platforms.

### Diffusion Policy (Chi et al., 2023, arXiv:2303.04137)

- **Demonstrations, real-world tasks (Table III):** Push-T = 136; sauce-pouring/spreading tasks = 50
  collected (90% used for training); Mug Flip = 250. Simulation benchmarks reuse RoboMimic's
  Proficient-Human (200/task) and Multi-Human (300/task) sets.
- **Observation:** RGB plus low-dimensional robot state (`agent_pos` / `robot_eef_pose`) in **every
  shipped task config** — the architecture can technically run without a state input, but no released
  configuration does. Observation horizon is 2 timesteps by default.
- **Action:** a denoising-diffusion model over a chunk of future actions, conditioned on the
  observation window.

### RoboMimic (Mandlekar et al., 2021, arXiv:2108.03298) — the demo-count reference point

Not a policy architecture but the standard benchmark for "how many demos does behavior cloning need":
200 demonstrations per task from a single proficient teleoperator (or 300 from six teleoperators,
50 each). Per-task counts in the released data range 48–480. Simple tasks (Lift, Can) reach
75–100% success on as little as ~20% of the 200-demo set (~40 demos); harder tasks degrade sharply
below that. **No paper in this survey reports a hard floor below roughly 50 demonstrations for a
useful single-task policy, and several report success rate degrading gradually rather than falling
off a cliff** — which matters for planning, but doesn't change the fact that the floor is
demonstrations, not sweeps.

### The pattern across all six

| Model | Demos/task or dataset size | Proprioception input? | Cameras | Action rate |
|---|---|---|---|---|
| OpenVLA | 970k demos (pretrain) | **No** (default) | 1, 224×224 | n/a (offline) |
| π0 | undisclosed, 8 platforms | **Yes**, `q_t` in the observation | multiple | up to 50 Hz |
| RT-1 | ~130k episodes, 744 tasks | No | ~1 (unspecified) | n/a (offline) |
| RT-2 | reuses RT-1 data + web data | No | ~1 (unspecified) | n/a (offline) |
| ACT | 50–100/task | **Yes**, 14-dim `qpos` | 4 (ALOHA rig) | 50 Hz control |
| Diffusion Policy | 50–250/task | **Yes**, low-dim state | 1–2 | 10–30 Hz typical |

Every model that trains a single-embodiment task policy (ACT, Diffusion Policy, π0) conditions on
proprioceptive state. The two that don't (OpenVLA, RT-2) do so because they're cross-embodiment
generalists trained across robots whose state representations don't line up — and their own authors
call the omission a limitation, not a feature. **There is no model here that trains on scripted,
task-less motion.**

---

## 2. Does scripted sweep data work for imitation learning? No — and here's why, precisely

Behavior cloning's training objective is supervised regression (or, for OpenVLA/RT-2/π0, next-token
prediction / flow matching) toward the action recorded in the dataset at each observed state. The
objective has **no independent notion of task success** — it only ever learns "reproduce the action
that was recorded here." The *only* channel through which task-solving behavior enters a
behavior-cloned policy is that the recorded actions were chosen, by whoever or whatever generated
them, because they solved the task. A scripted joint sweep was chosen because it *covers a range*,
not because it accomplishes anything — so a policy trained on it would, at best, learn to reproduce
the sweep. That is the whole mechanism, and it's why "no task, no operator intent" in this project's
own proposed approach is disqualifying on its own, independent of anything about the arm's hardware.

This is a settled distinction in the literature, not a judgment call:

- **Play data is not scripted data**, and the paper that established "task-less" demonstrations as
  useful is explicit about the difference. Lynch et al., "Learning Latent Plans from Play"
  (arXiv:1903.01973) collect *human-teleoperated*, unsegmented interaction — the human is still
  semantically manipulating objects (opening drawers, picking things up) even without a labeled goal,
  so any sub-window still *locally accomplished something*, which is what their method (relabeling
  sub-trajectories as `(start, end, action-sequence)` triples) exploits. The paper explicitly
  distinguishes itself from prior **scripted, unattended** self-supervised grasping-data collection
  (Pinto & Gupta, arXiv:1509.06825; Levine et al., arXiv:1603.02199) as a different, narrower prior
  line of work — and even that scripted-grasping precedent had an implicit success/failure signal
  (did the gripper close on something), which a pure joint-angle sweep with no scene interaction does
  not have at all.
- **Self-supervised visual pretraining on unlabeled interaction reduces demo requirements, it doesn't
  eliminate them.** R3M (Nair et al., arXiv:2203.12601) pretrains a visual representation on Ego4D
  human video with no task labels — and still needed roughly 20 real task demonstrations per skill on
  the downstream robot. Undirected data made demonstrations more sample-efficient; it did not replace
  them.
- **Covariate shift is a known failure mode even with *real* demonstrations**, formalized in the
  DAgger literature (arXiv:2102.02872, arXiv:2309.02473 survey): a policy trained on the expert's
  state distribution compounds small errors once deployed on its own trajectory. A scripted sweep is
  strictly worse than an imperfect expert — it isn't a noisy demonstration signal, it's the absence of
  one.
- **No source found in this research proposes or reports training a behavior-cloned or VLA policy
  directly on task-less scripted motion and getting task-competent behavior out.** That silence is
  itself informative — it isn't a debated or marginal practice, it simply isn't how any of these
  systems are built.

### What scripted sweep data actually IS good for — and this project already uses it correctly once

Scripted, camera-logged joint sweeps are real, standard, useful data — for a different family of
problems than policy training:

- **Kinematic calibration / axis identification.** This project's own implementation plan already
  proposes exactly this, correctly scoped: `docs/implementation_plan.md` M6 commands "each joint alone
  across its calibrated range" and least-squares fits a rotation axis to the marker trajectory it
  produces — recovering link-to-link transforms that don't exist anywhere else in the repo (no
  assembly file, no URDF). That's a sweep used for **system identification**, not for training a task
  policy, and it's the right use of exactly this kind of data.
- **Camera-to-robot extrinsic calibration and hand-eye calibration** (Tsai–Lenz-style) — a scripted
  sweep with known commanded angles and observed marker poses is the standard input to this class of
  problem.
- **Dynamics models / world models.** Undirected interaction data is exactly what model-based RL
  methods (PlaNet, Dreamer, Plan2Explore; recent example: "PlayWorld," arXiv:2603.09030) train a
  *forward* model on — predict what the next frame/state looks like given an action — which is a
  different learning objective from "reproduce the recorded action." A trained forward model can later
  support planning or visual-servoing gain estimation, but it is not itself a task policy.
- **Visual-servoing gain / image-Jacobian estimation.** See §5 — this is arguably the single best fit
  for this arm's exact data shape (commanded angle → resulting pixel change), and it produces a
  usable control law without ever training a neural policy.

The honest framing: **the arm's proposed sweep data is good raw material for calibration and
kinematic recovery — a job this repo already correctly assigns it in M6 — and it is not usable, by the
mechanism of the loss function itself, for training an imitation-learning or VLA policy.**

---

## 3. The no-proprioception problem

Every single-embodiment policy surveyed above (ACT, Diffusion Policy, π0) conditions on
proprioceptive state as a first-class input, fused directly into the model. This arm has none. Every
servo is open-loop RC hardware (MG996R, MG90S) with an internal potentiometer wired only into the
servo's own analog control loop — never exposed to the host over any pin or protocol. `Servo.read()`
on the firmware returns the last **commanded** pulse width, not a shaft reading. This is not a
software gap; it's what the hardware physically is.

### What breaks

Two distinct risks, not one, and it matters which is which:

1. **Copycat / causal confusion** — a general imitation-learning failure mode (Wen et al.,
   arXiv:2010.14876; de Haan/Jayaraman/Levine, arXiv:1905.11979), where a policy conditioned on
   recent state learns to echo the last action instead of reasoning from the scene. This risk exists
   **even with real, accurate encoders** — it's a property of conditioning on state/action history at
   all, not specific to this hardware.
2. **Observability loss — the risk specific to this arm, and the one that actually matters here.**
   When the "state" channel is the commanded setpoint and the servo has zero feedback, gravity sag and
   mechanical backlash don't appear as *noise* in the state signal — they are **structurally absent**
   from it. The state channel cannot distinguish "arm is exactly on target" from "arm has drooped
   under a loaded gripper and is silently stuck 8° off," because both situations produce the identical
   commanded-angle reading. No amount of training data, noise augmentation, or robustness technique
   (e.g. DART-style noise injection, arXiv:1703.09327) fixes this — it isn't noisy information, it's
   information the channel never carried. And the error is not constant: the same commanded angle maps
   to a different true angle depending on payload and workspace pose, so the aliasing itself drifts
   across the dataset with nothing in the recorded data to reveal it.

   This project's own evidence already demonstrates the size of the problem it's trying to paper over.
   `Documentation/RESUME-PROMPT.md` records the arm **sagging while held**, gradually, over minutes —
   with joints still reporting their commanded values throughout. `joint-limits.csv` records every
   locked limit as an "ACCEPTED COMMANDED SOFT LIMIT," explicitly **not** a mechanical measurement,
   because "nothing in this system observes the output shaft." The gap this section is warning about
   is not hypothetical on this arm; it's already been directly observed and written down by the people
   running it.

### The honest options, ranked by what they actually cost

1. **Accept commanded-as-state and declare it — this project already does this correctly.** The
   LeRobot adapter's Option A observation contract (`observation.py`, §3 of the spec) is explicit:
   `.pos` always means commanded, never measured; every observed/residual/source field is NaN with a
   finite source code (`OBS_SOURCE_NONE`) rather than silently faking a value. This is the right
   *engineering* answer to a hardware limitation that cannot be fixed in software — but it does not
   make the resulting policy trainable on a task; it just prevents the dataset from lying about what
   it contains. This option is necessary regardless of which of the other two is also pursued.
2. **Add real feedback hardware.** LeRobot's own reference platforms don't have this problem: SO-100 /
   SO-101 use Feetech STS3215 servos with a 12-bit magnetic encoder (`Present_Position` is a genuine
   register readback, confirmed at the LeRobot source level — `sync_read("Present_Position", ...)`),
   and Koch v1.1 uses Dynamixel XL330/XL430, same class of onboard encoder. Swapping this arm's servos
   for smart servos with real position feedback is the closest thing to a direct fix, and it is the
   same fix LeRobot's own hardware already made — this arm diverges from the reference platform
   precisely on the one axis (feedback) that reference platform depends on.
3. **Estimate pose from the cameras via fiducial markers.** This project has already scoped this path
   (`Documentation/MARKER-SYSTEM.md`, `Software/vision/markers.csv`) and it is methodologically sound
   — a January 2026 paper, "Fiducial Exoskeletons: Image-Centric Robot State Estimation" (Smith, Van
   Hoorick, Guizilini, Wang; arXiv:2601.08034), demonstrates exactly this substitution on a low-cost
   arm, explicitly targeting "robust state estimation even on unplugged robots" — i.e., zero working
   proprioceptive sensing, this arm's exact situation. Marker-based pose accuracy is genuinely good at
   close range: AprilTag/ArUco reach sub-millimeter translation accuracy out to roughly 1 m
   (Kalaitzakis et al., *J. Intelligent & Robotic Systems* 101(4), 2021, DOI:10.1007/s10846-020-01307-9),
   degrading with range. **But this option is not available on this arm today** — neither camera has
   been calibrated (no intrinsics exist anywhere in the repo), so `cameras.csv`'s own header states
   every marker size still rests on an *assumed* ~60° field of view, and no marker stickers should be
   printed until that's fixed (M5 in the implementation plan, which needs no arm and can run today).

None of these three options is optional if the goal is a trainable state channel: option 1 is a
documentation/honesty floor everyone should already be doing, and one of options 2 or 3 is required to
get past "commanded angle" into something a policy can actually condition on without learning a lie.

---

## 4. Is 4 working DOF and a non-functional gripper enough to train anything worth having?

No, and the gripper is the more disqualifying half of that sentence.

Across the standard benchmarks this literature is built on, gripper-closure is not one task family
among several — it's structurally baked into the action space itself. The Open X-Embodiment / RT-1 /
RT-2 / OpenVLA standard action representation includes a gripper dimension on **every timestep of
every task**, regardless of whether that particular task needs it. RT-1's own 744-task dataset is
almost entirely prehensile (pick, place-into, place-upright — Table 1's task list is grasp-centric
top to bottom). ACT/ALOHA and RoboMimic's benchmark suites are effectively 100% grasping-based, with
no non-prehensile exceptions found in this research. The real presence of non-prehensile tasks
(pushing, sliding, wiping) concentrates almost entirely in Diffusion Policy's Push-T/BlockPush
benchmarks and parts of BridgeData V2 — a narrow, separate slice of the literature, not the mainstream
this arm's proposed approach would be measured against.

This arm's J6 gripper acknowledges every command the firmware sends and ramps its setpoint correctly
— the firmware side is fully exonerated (`RESUME-PROMPT.md`) — but the fingers do not move, because
the gear has slipped on the motor shaft. That's not "a gripper with reduced range" or "a gripper that
needs recalibration." It's the mechanical absence of the one degree of freedom that defines
pick-and-place, the task family every model surveyed in §1 is built and benchmarked around. Combined
with J0 (base yaw) being a dead servo — no lateral reach, no ability to address more than a fixed
wedge of workspace in front of the arm — the honest description isn't "4 of 6 DOF work, that's a bit
narrow." It's **no working prehensile task family, and no working base rotation to vary the workspace
a narrower task could be attempted in.** Four DOF is enough to move a wrist through space; it is not
enough, combined with a non-functional gripper, to demonstrate anything in the task vocabulary this
research area evaluates on.

---

## 5. Realistic alternatives, if a VLA is the wrong target — which it currently is

These aren't independent options so much as one continuum, from fully hand-specified control toward a
fully learned policy. Given this arm's constraints (no proprioception, uncalibrated cameras, narrow
working DOF), the right end of that continuum to start from is the classical-control end.

- **Classical visual servoing — the best-fitting option for the data this arm can actually produce
  today.** Standard image-based (IBVS) or position-based (PBVS) visual servoing (Chaumette &
  Hutchinson's reference treatment, *IEEE Robotics & Automation Magazine*, 2006/2007) still typically
  assumes at least approximate camera calibration. The better fit here is **uncalibrated / model-free
  visual servoing** (Jägersand, 1997; Piepmeier et al.), which estimates the image Jacobian —
  "commanding X degrees produces Y pixels of visual change" — empirically online, and is explicitly
  designed to be independent of robot type, camera type, and camera mounting. That description matches
  this arm's exact situation (uncalibrated cameras, unmeasured kinematics) far better than anything
  requiring intrinsics or a trained policy, and it can be built directly from scripted sweep data —
  the same data this project's proposed approach would collect, just used for the right purpose.
- **Learned forward/dynamics models as a stepping stone, not an end product.** The neural-network
  version of the same idea (Agrawal et al., 2016; Finn & Levine, 2017): learn "if I command X, the
  camera will show Y" without any notion of task success. This is a legitimate, buildable target for
  exactly the sweep data described in the proposed approach — it's §2's "what sweep data is actually
  good for," restated as a control-relevant deliverable rather than a calibration one.
- **Behavior cloning on a much narrower task, once real demonstrations exist.** Few-shot BC has been
  pushed further than the 50-demo RoboMimic/ACT floor in §1 — "One ACT Play" (arXiv:2309.10175)
  reports near-perfect performance on a 4-DOF push/pick-place task from a **single** real
  demonstration, heavily augmented to 25–400 synthetic variants. This is the closest literal match in
  the literature to "few working DOF, small demo budget" — but it still requires at least one genuine,
  successful, human-directed demonstration. A script cannot generate the seed demonstration this
  method depends on.
- **No learning at all, yet.** Given the arm cannot currently observe its own state, cannot close its
  gripper, and has one dead axis, a defensible position is that the correct next engineering step is
  classical: fixed lookup-table or PID-style control from marker-observed pose to commanded angle,
  with no learned component. This is not a lesser answer — it's what the repo's own spec effectively
  proposes for M6/M7 (axis identification → closed observation loop) before any policy work is
  mentioned at all.

---

## 6. The honest cheapest path to actual autonomy on this specific arm — ranked

Ranked by what's genuinely unblocked today, cheapest first, because everything after item 2 depends on
having it:

1. **Camera intrinsics calibration (M5) — unblocked today, needs no arm, cost: a printed ChArUco
   board and an afternoon.** Every marker size in `Software/vision/markers.csv` rests on an *assumed*
   ~60° field of view that has never been checked. This blocks marker printing, blocks accurate
   pose estimation, and blocks nothing else — it can run in parallel with everything below and should
   run first because everything below is more accurate once it's done.
2. **Marker-based pose observation (M6/M7) — needs the assembled arm + calibrated cameras from step
   1.** This is the single highest-leverage step on the list: it closes the no-proprioception gap
   (§3, option 3) using hardware already in the repo's plan, it recovers the link-to-link kinematic
   transforms that don't exist anywhere else (no URDF, no assembly file), and — as a direct side
   effect — it's the only mechanism this project has ever had for measuring the shoulder's unmeasured
   mirror offset (`mirror_offset_deg`, stuck at a placeholder 0 today). Cost: fiducial markers sized
   per the already-computed `markers.csv` table, a `solvePnP` pipeline, and the sweep-based axis-ID
   procedure the implementation plan already specifies (M6) — which is precisely the scripted-sweep
   data collection the original proposal describes, aimed at the job it's actually good for.
3. **Fix the gripper mechanically, and confirm J0 once the replacement servo is fitted.** Neither is
   a data or software problem. The gripper's gear-slip is a hand-repair or a mechanical-part
   replacement, and J0 already has a replacement servo on order per `RESUME-PROMPT.md` — it just needs
   fitting and re-measurement (a new horn seats on a different spline tooth, so 29–110 won't hold).
   Without this step, no version of the arm can demonstrate pick-and-place, so no version of this plan
   reaches even the narrowest BC target in §5.

Only after those three does "collect demonstrations" become a meaningful next step — and at that
point, the right target is a narrow single-task ACT or Diffusion Policy model on real, human-directed
demonstrations of a pick-and-place using the two already-recorded, camera-verified poses (`storage`,
`pick`) as start/end anchors — not a VLA, and not scripted sweeps. A VLA (OpenVLA, π0) is trained for
cross-embodiment generalization from datasets two to three orders of magnitude larger than anything a
single hobby arm can collect; it is the wrong target for this hardware at any stage, not just this one.
A narrow, single-task, single-embodiment imitation policy is the correct ambition once steps 1–3 are
done — and even that needs a human at the controls for the demonstrations, not a script.

---

## Sources

- Kim et al., "OpenVLA: An Open-Source Vision-Language-Action Model," arXiv:2406.09246 —
  https://arxiv.org/abs/2406.09246 (abstract + §3.2, §3.4, §5.2, §6 fetched directly for this report)
- Kim, Finn, Liang et al., "OpenVLA-OFT," arXiv:2502.19645 — https://arxiv.org/abs/2502.19645
- Black et al., "π0: A Vision-Language-Action Flow Model for General Robot Control," arXiv:2410.24164
  — https://arxiv.org/abs/2410.24164 ; announcement https://www.pi.website/blog/pi0
- Brohan et al., "RT-1: Robotics Transformer for Real-World Control at Scale," arXiv:2212.06817 —
  https://arxiv.org/abs/2212.06817
- Brohan et al., "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control,"
  arXiv:2307.15818 — https://arxiv.org/abs/2307.15818
- Zhao et al., "Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware" (ACT/ALOHA),
  arXiv:2304.13705 — https://arxiv.org/abs/2304.13705 ; https://tonyzhaozh.github.io/aloha/ ;
  https://github.com/tonyzhaozh/act
- Chi et al., "Diffusion Policy," arXiv:2303.04137 — https://arxiv.org/abs/2303.04137 ;
  https://diffusion-policy.cs.columbia.edu/ ; https://github.com/real-stanford/diffusion_policy
- Mandlekar et al., "What Matters in Learning from Offline Human Demonstrations for Robot
  Manipulation" (RoboMimic), arXiv:2108.03298 — https://arxiv.org/abs/2108.03298
- Lynch et al., "Learning Latent Plans from Play," arXiv:1903.01973 —
  https://arxiv.org/abs/1903.01973 ; https://learning-from-play.github.io/
- Pinto & Gupta, "Supersizing Self-Supervision," arXiv:1509.06825 —
  https://arxiv.org/abs/1509.06825
- Levine et al., "Learning Hand-Eye Coordination for Robotic Grasping," arXiv:1603.02199 —
  https://arxiv.org/abs/1603.02199
- Nair et al., "R3M: A Universal Visual Representation for Robot Manipulation," arXiv:2203.12601 —
  https://arxiv.org/abs/2203.12601
- Wen et al., "Fighting Copycat Agents in Behavioral Cloning from Observation Histories,"
  arXiv:2010.14876 — https://arxiv.org/abs/2010.14876
- de Haan, Jayaraman, Levine, "Causal Confusion in Imitation Learning," arXiv:1905.11979 —
  https://arxiv.org/abs/1905.11979
- Laskey et al., "DART: Noise Injection for Robust Imitation Learning," arXiv:1703.09327 —
  https://arxiv.org/pdf/1703.09327
- Survey: "A Survey of Imitation Learning," arXiv:2309.02473 — https://arxiv.org/pdf/2309.02473 ;
  "Feedback in Imitation Learning," arXiv:2102.02872 — https://arxiv.org/pdf/2102.02872
- Smith, Van Hoorick, Guizilini, Wang, "Fiducial Exoskeletons: Image-Centric Robot State Estimation,"
  arXiv:2601.08034 — https://arxiv.org/abs/2601.08034
- Kalaitzakis et al., "Fiducial Markers for Pose Estimation," *J. Intelligent & Robotic Systems*
  101(4), 2021, DOI:10.1007/s10846-020-01307-9 — https://link.springer.com/article/10.1007/s10846-020-01307-9
- "One ACT Play: Single Demonstration Behavior Cloning with Action Chunking Transformers,"
  arXiv:2309.10175 — https://arxiv.org/abs/2309.10175
- Chaumette & Hutchinson, "Visual Servo Control," *IEEE Robotics & Automation Magazine* — standard
  reference survey on IBVS/PBVS
- LeRobot hardware docs: SO-101 (Feetech STS3215) — https://huggingface.co/docs/lerobot/en/so101 ;
  Koch v1.1 (Dynamixel) — https://github.com/jess-moss/koch-v1-1 ;
  https://github.com/huggingface/lerobot (source-level confirmation of `Present_Position` register
  readback vs `Goal_Position` write-only channel)
- This repo: `Documentation/RESUME-PROMPT.md`, `Software/arm-console/joint-limits.csv`,
  `Software/arm-console/arm-poses.csv`, `docs/lerobot_emre_arm_spec.md`, `docs/implementation_plan.md`,
  `Software/arm-vision/cameras.csv`, `Documentation/MARKER-SYSTEM.md`,
  `Software/lerobot_robot_emre_arm/`

---

## Addendum — the classical route, in more detail than §6 gives it

Added after the main report, from a parallel research thread. It does not change the verdict; it
makes the recommended route materially cheaper than the body of this document implies.

### Visual servoing does not require calibration

§6 sequences calibration before classical control. That ordering is optional. There is a mature line
of work that estimates the **image Jacobian online, from motion that already happened**, with no
intrinsics and no hand-eye solve:

- Hosoda & Asada, *Versatile Visual Servoing without Knowledge of True Jacobian*, IROS 1994 —
  http://www.er.ams.eng.osaka-u.ac.jp/Paper/1994/Hosoda94b.pdf
- Jägersand, Fuentes & Nelson, ICRA 1997 — Broyden secant update from observed motion, so no
  dedicated calibration moves are needed at all.
- Piepmeier, McMurray & Lipkin, ICRA 1999 / IEEE T-RO 20(1):143–147, 2004 — quasi-Newton/RLS
  estimator; the authors state plainly that this *is* system identification applied to servoing.
  https://www.usna.edu/Users/weaprcon/piepmeie/_files/documents/H2002_282final.pdf

So **calibration is an optimisation of the classical route, not a gate on it.** An uncalibrated loop
can start from the sign-and-scale table measurable this week; intrinsics and hand-eye get added later
to make the result metric.

### The sweeps already collected are the right shape for this

Sutanto, Sharma & Varma, *The Role of Exploratory Movement in Visual Servoing Without Calibration*,
Robotics and Autonomous Systems 23(3):153–169, 1998 —
https://www.sciencedirect.com/science/article/abs/pii/S0921889097000523 — proposes injecting
deliberate small exploratory motions, **separate from the task motion**, purely to keep the online
Jacobian well-conditioned as it degrades during servoing. That is a description of the J5 sweep and
the J4xJ5 wrist grid already recorded on 2026-08-07.

And for the hand-eye solve when it is eventually wanted: `AX=XB` is only observable with **at least
two rotations about non-parallel axes** (Tsai & Lenz 1989; Enebuse et al. 2021). J4 pitch and J5 roll
are perpendicular, so a J4xJ5 grid satisfies that by construction — parallel-axis-only motion is the
degenerate case and the usual reason a hand-eye solve returns garbage. See also EasyHeC
(https://arxiv.org/abs/2305.01191), which automates the pose selection.

### World-model data volumes are nothing like imitation-learning volumes

Nagabandi, Kahn, Fearing & Levine, ICRA 2018 —
https://people.eecs.berkeley.edu/~ronf/PAPERS/anagabandi-icra18.pdf — trained a usable dynamics model
on a real millirobot from **17 minutes of purely random data**. Compare the 50–970,000 demonstrations
the imitation methods in §1 require. If a learned component is ever wanted on this arm, a dynamics
model is the one whose data budget it can actually meet.

### The name for what we are doing

In the literature, goal-free demonstrator-free self-generated motion is **"motor babbling"** — the
term is borrowed from infant motor development. Searching under that name finds the relevant work
(system identification, self-supervised dynamics, Jacobian estimation) far faster than searching for
"random exploration", which returns reinforcement-learning results that do not apply here.
