# Implementation plan — `lerobot_robot_emre_arm`

**Status:** M0 complete (package scaffolded, observation contract and calibration
loader written and executed). Nothing here has yet spoken to the board.

**Decision already taken, not reopened here:** keep this arm, standardise the
software on LeRobot, build an installable plugin package. The observation
contract is **Option A** — `.pos` is the *commanded* angle, and anything observed
lives in separate, differently-named fields with a residual, a source and an age.

---

## 1. The one-paragraph version

Build the adapter first, because it is the piece that is fully specified by
things that already exist in this repo: a documented wire protocol, a firmware
whose behaviour has been read line by line, five real calibration locks, and a
working reference implementation in the arm console. Build the marker observer
second, because every part of it depends on measurements nobody has taken yet.
Ship them in that order — **but do not flatten "second" into "optional"**, because
the observer is what makes unattended recording possible at all (§5).

---

## 2. What ships first, and why it is the right first thing

**M1 — the adapter talks to the board and pushes a verified envelope.**

The whole of M1 is testable **today, on USB power alone, with no servo supply
connected**, because none of it drives a pin:

- open the port, wait out the Optiboot reset, gate on `NAME=FACTORYLM-ARM`
- push `WDG`, then per-joint `LIM`/`SPD`, then `MIR`
- read `STA` back and assert every `MIN`/`MAX`/`CAL`/`DPS` matches the file

That last step is the point of the milestone. The firmware retains **nothing**
across the DTR reset that opening the port causes — limits revert to 70-110,
`CAL=0`, `MIR=UNKNOWN`, watchdog off. Joint 0's home of 64, joint 1's of 1 and
joint 3's of 33 all sit *outside* 70-110, so enabling before the push returns
`E5` on three of six joints. Proving the push lands is therefore the precondition
for every later milestone, and it costs nothing but a USB cable.

This matters for sequencing: **the outstanding 3–5 A supply blocks M2 onward, but
it does not block M1.** Do M1 while waiting for the hardware.

---

## 3. Milestones

Each milestone names what unblocks it, so nothing gets started before it can
actually be finished.

### M0 — package scaffold and the contracts ✅ *complete*

| Deliverable | State |
|---|---|
| `pyproject.toml`, `__init__.py` | written |
| `observation.py` — Option A schema | written, **executed**: 37 float observation features, 6 action features, no `j2` key |
| `calibration.py` — CSV + lock-artifact loader, validators H1–H7 | written, **executed against the real repo files** |
| `transport.py` — serial link, §9 handshake, heartbeat, idle park | written, **never run against hardware** |
| `config_emre_arm.py`, `emre_arm.py` | written, **never imported** (LeRobot absent — §7.2) |
| `markers.py` — observer Protocol + `NullMarkerObserver` | written; `ArucoMarkerObserver` raises with a list of what is missing |

The calibration loader already reproduces every value the design predicted: the
`(-44, 0)` legal mirror-offset window at joint 1's real envelope versus
`(-35, +35)` at the firmware's boot default, joint 6 as the sole uncalibrated
joint, and the superseded-lock counts (2 for joint 1, 1 for joint 3).

Validator **H5** was demonstrated to catch the one real calibration bug this
project has had: widening joint 0 back to 0-180 is rejected because the newest
lock artifact says 29-110. That regression actually happened on 2026-08-05 and
cost a morning of "why won't the base drive".

### M1 — link bring-up *(unblocked: USB only)*

**Exit criteria**

1. `SerialLink.open()` completes against the real board; `VER` returns
   `NAME=FACTORYLM-ARM` and a firmware version.
2. The full state push completes and the `STA` readback matches
   `joint-limits.csv` on all six joints.
3. `SYS WDMS` is non-zero after the push.
4. The heartbeat runs for five minutes with no watchdog event, then stopping it
   deliberately produces `EVT WDOG` — i.e. the dead-man is *proven*, not assumed.
5. `Software/tests/protocol_check.py` still passes with the console closed.

**Risks specific to M1**

- `exclusive=True` on POSIX is unverified. On Windows the OS makes COM opens
  exclusive anyway, so this only matters when moving to Bravo or Charlie.
- If the push returns `ERR E9 … STATE=enabled`, the board did **not** reset when
  the port opened. `_push_state()` refuses rather than pushing an envelope over a
  live joint. Do not "fix" this by ignoring it.

### M2 — one joint, bounded motion *(needs: a supply that can hold one servo)*

A ~700 mA supply can drive **one unloaded** servo — the single-servo bench sketch
already did. Enable one calibrated joint with a human-supplied adopt angle, issue
a small `MOV`, confirm `SET=`/`CL=` come back as expected, confirm `STP` holds and
`DIS A` detaches.

Start with **joint 5 (wrist roll, D9)** — it is the only joint whose servo type
is `DOC-CONFIRMED` rather than inferred, and it carries the least mechanical load.
Do **not** start with joint 0 (identity disputed) or joint 1 (mirrored pair,
unmeasured offset).

### M3 — six joints, teleoperation *(needs: 3–5 A supply + 1 A slow-blow fuse)*

`lerobot-teleoperate --robot.type=emre_arm`. This is the first milestone that
requires the assembled arm and therefore the outstanding hardware.

Send `MOV` **only when a target changes**. `MOV` is target-seeking: it sets the
goal and the firmware interpolator walks toward it at the joint's deg/s. Six
`MOV`s per control tick would be six sequential round trips against the
one-command-in-flight rule, and would make an achievable rate look impossible.

### M4 — recording *(needs: M3)*

`lerobot-record --robot.type=emre_arm --fps 20`.

**Set `fps` at or below 20.** A full `STA` reply is roughly 40 ms of wire time at
115200 — that is arithmetic from the reply's byte count, not a bench measurement,
and real throughput will be lower once the USB latency timer and Python overhead
are included. LeRobot's 30 fps default is not reachable through `STA`.

Every `observed_*` column will be NaN at this milestone. That is correct and
intended. A day-one training config should select `observation.state[:6]` plus
the camera keys and not consume the observation block.

**M4 is where the automation ceiling bites** — see §5.

### M5 — camera intrinsics *(unblocked: no arm needed, can run in parallel)*

Print a ChArUco board, calibrate the camera, store the intrinsics. Do this early
and out of order, because **every marker size and every detection-grade claim in
`Documentation/MARKER-SYSTEM.md` scales linearly off an assumed 60° horizontal
field of view that nothing in this repo has verified.** If the real figure differs,
the sticker sizes are wrong and any that have been printed must be reprinted.

### M6 — axis identification and datum *(needs: assembled arm + M3)*

Two passes, in order, both documented in `Documentation/MARKER-SYSTEM.md`:

1. **Axis identification** — command each joint alone across its calibrated range
   and least-squares fit a rotation axis to the child tag's trajectory relative to
   the parent tag. This *recovers* the joint axes and link-to-link transforms.
   They cannot be read off the STLs: those are 21 individual parts in print
   orientation with no assembly file, so the transforms are genuinely unknown.
2. **Datum capture** at commanded home, so `observed − commanded` has a reference.

**Pass 1 also settles the D3 identity dispute in one shot**: command joint 0 and
watch which link moves.

The datum is captured at a *commanded* home, and `home_deg` is editorial — several
rows were hand-moved off the locked value. So every observed angle carries a
constant bias. Say so; the residual still detects *change* (sag, slip, stall)
without that bias being zero.

### M7 — closed observation loop *(needs: M5 + M6)*

Replace `NullMarkerObserver` with a real one. `observed_deg`, `residual_deg` and
`obs_age_s` stop being NaN. The first genuinely new capability this unlocks is
`j1.pair_disagree_deg` — the only mechanism this project has ever had for
measuring the shoulder mirror offset.

---

## 4. Build order inside the adapter slice

```
observation.py    no dependencies          DONE, executed
calibration.py    -> observation.py        DONE, executed against real files
transport.py      pyserial only            DONE, unexercised
config_emre_arm.py -> calibration, markers  DONE, unimportable here
emre_arm.py       -> all of the above      DONE, unimportable here
markers.py        -> observation.py        interface DONE, detection NOT built
```

Bottom-up on purpose: the two modules with no third-party dependencies are the
two that can be executed and asserted on a machine with neither LeRobot nor a
board attached — which is this machine, and which is why they are the two that
are actually verified rather than merely written.

---

## 5. Where the marker observer really sits — the dependency not to flatten

In build order the observer is later than the adapter, and this plan says so
plainly. But "later" is not "optional", and the reason is easy to miss:

> `ENA <j> <adopt_deg>` takes a **human's by-eye estimate** of where a joint's
> shaft physically is. Nothing in this system can produce that number.
> After any detach — a watchdog trip, an e-stop, or this adapter's own idle park —
> a gravity-loaded arm **sags**, and the next adopt angle must be freshly
> estimated against an arm that moved while nothing was commanding it.

So no joint can be re-enabled automatically. A multi-episode `lerobot-record`
session that hits a single watchdog trip cannot recover without a person looking
at the arm. **Multi-episode recording is not safely automatable on this arm until
something can observe it** — and an observer is the only mechanism that could ever
supply a trustworthy adopt angle.

That is the structural link between M4 and M7: the adapter ships first and is
genuinely useful for supervised, single-session recording; the observer is what
removes the human from the loop. Anyone reading this plan as "adapter now,
cameras someday" has lost the actual dependency.

---

## 6. Hardware blockers

These are physical, and no amount of software closes them.

| # | Blocker | Consequence | Gates |
|---|---|---|---|
| H-1 | **No 3–5 A supply.** The bench adapter measures 6.62 V at ~700 mA and cannot hold an assembled arm. | One unloaded servo works. Six under load brown out. | M3, M4, M6, M7 |
| H-2 | **No 1 A slow-blow fuse.** | No current limit between the supply and seven servos. A stalled servo is a fire risk, and a joint commanded into a mechanical stop *does* stall — that is exactly what the widened joint-0 envelope caused. | M3 onward |
| H-3 | **D3 motor identity disputed.** `wiring-map.csv` says the Base is on D3; the 2026-08-01 calibration-log row says the servo physically wired to D3 was the **gripper**. | Limits follow the *pin*, so they hold either way — the dataset is safe because keys are id-keyed, never name-keyed. But `joint_name` may be wrong on joints 0 and 6, and so may the servo *type*, and therefore the voltage-headroom judgement. If D3 really carries the gripper's MG90S, joint 0 is running over spec. | Resolved by M6 pass 1, or by one person looking at the wiring |
| H-4 | **Shoulder mirror offset never measured.** `mirror_offset_deg` sits at the placeholder 0. | If the true axis is not 90°, the two MG996Rs fight each other by twice the error for as long as joint 1 is driven — they hold, run hot, and eventually strip a gear, with nothing on screen showing it. Worse, joint 1's mirrored image already touches 180 exactly, so **any positive offset is refused**: measuring one forces joint 1's max to narrow first. The documented procedure needs the linkage unbolted and both horns off, so it cannot be done on the assembled arm. | Gated behind `allow_unmeasured_mirror` |
| H-5 | **MG90S driven over voltage.** Joints 4, 5 and 6 are MG90S, rated 4.8–6.0 V, on a 6.62 V supply. | Already driven out of spec. Reduces service life; may already have caused damage. | Fix with H-1 |

---

## 7. Non-hardware blockers

These are not "software work remaining" — they are things that block a step and
cannot be resolved by writing more code.

### 7.1 The repository grants no licence

There is **no LICENSE file anywhere in this repo.** The stated rule is
Apache-2.0 or MIT only, and `pyproject.toml` therefore **deliberately omits the
`license` key** rather than asserting terms nobody has granted. This blocks
publishing the package to any index. It does not block `pip install -e .` locally.

Two dependency exceptions also need recording rather than passing silently:
**pyserial** and **numpy** are both BSD-3-Clause, which is not on the allow-list.
Both are unavoidable — pyserial is already a dependency of `arm-bridge.py` and
`protocol_check.py`, and numpy is required by both OpenCV and LeRobot. Both are
permissive and Apache-2.0 compatible in practice. OpenCV is Apache-2.0 and needs
no exception.

### 7.2 LeRobot is not installed on the development host

So the `Robot` ABC and `RobotConfig` have **never been read from source**.
`emre_arm.py` and `config_emre_arm.py` are written to the documented contract and
are **not importable on this machine** — `import lerobot_robot_emre_arm` raises
`ModuleNotFoundError: No module named 'lerobot'`, by design (§9).

Confirm on first install, in this order:

1. `connect(calibrate: bool = True)` is the real signature.
2. Something calls `configure()`. If not, the `ENA` step never runs and the first
   `send_action()` returns `E6`.
3. `RobotConfig` does not already define fields that collide with
   `EmreArmConfig`'s.
4. The dataset packer really does select features with `v is float` — this is why
   *every* Option A field is declared `float`, including the categorical source
   enum. If the predicate differs, a non-float feature would be silently dropped.
5. Whether `OpenCVCameraConfig` selects `cv2.CAP_DSHOW` on Windows. This machine
   requires it, and the built-in camera silently refuses 1080p (1280×720 works).
6. Pin the `lerobot` floor properly. `>=0.3` is a deliberate placeholder.

### 7.3 Camera field of view unverified

See M5. Every marker size in the design scales linearly off an assumed 60° HFOV.

### 7.4 Joint 6 is uncalibrated; joint 4's range is suspect

`is_calibrated` is **False today**, and that is the correct answer. Read it as
"recording is fine, joint 6 must not be driven" rather than as a blocker.

- **Joint 6** is `calibrated=no`, with 0–180 — the servo's full electrical range,
  not measured travel. A gripper hits its own linkage long before either end.
- **Joint 4** is `calibrated=yes` at exactly 0–180, which is simultaneously the
  placeholder width and the electrical range. A joint that locks at exactly that
  has most likely never been driven to a mechanical stop at either end. The
  `full_electrical_range` flag carries the suspicion, and it is what keeps
  `is_calibrated` False even if joint 6 were fixed tomorrow.

### 7.5 Install editable, not plain

`calibration.py` resolves its default paths relative to its own file position, so
it finds `joint-limits.csv` and `lock-artifacts/` by walking up to the repo root.
That works under `pip install -e` because of the src layout, and it is how the
loader was verified. It **breaks under a non-editable `pip install .`**: the
defaults would point into site-packages and `load_calibration()` would raise
"file not found".

Recoverable — `EmreArmConfig` exposes `limits_csv`, `lock_artifacts_dir` and
`wiring_map_csv` explicitly, so a non-editable install just has to set them. But
`pip install -e` is the supported path, and the calibration files deliberately
stay in the repository rather than being packaged, because they are operational
records that get edited, not shipped assets.

### 7.6 The console and the adapter are mutually exclusive

The adapter opens the serial port directly and is its sole owner. The wire
protocol correlates replies by **echoed verb with no sequence numbers**, so two
transmitting clients physically cannot tell their replies apart — serialising
writes would not be sufficient, because *attribution* is what breaks.

Close the arm console and its bridge before running anything here. `open()`
probes `127.0.0.1:8770` first and refuses with a plain-English instruction rather
than a `PermissionError` traceback.

---

## 8. Verification ladder

Each rung is runnable without the rung above it.

| Rung | Needs | Command |
|---|---|---|
| Schema and validators | nothing | import `observation` / `calibration` directly, without the package `__init__` |
| Package imports, discovery works | `pip install lerobot` | `python -c "import lerobot_robot_emre_arm"`, then `lerobot-record --robot.type=emre_arm --help` |
| Wire protocol | board on USB | `python Software/tests/protocol_check.py` (console closed) |
| Handshake and envelope | board on USB | M1 exit criteria |
| Motion | one-servo supply | M2 |
| Full arm | 3–5 A supply + fuse | M3 onward |

---

## 9. Things this plan deliberately does **not** do

- **No writable LeRobot-side calibration file.** `joint-limits.csv` and
  `Calibration_Notes/lock-artifacts/` are the sole source of truth, and
  `_save_calibration()` raises. A second writable copy of the limits is precisely
  how joint 0's measured envelope was lost once already.
- **No automatic `calibrate()`.** It raises and points at the console's LOCK
  procedure. Producing a calibration means driving a joint to find its travel,
  which needs an adopt angle nothing can know — and `connect(calibrate=True)` is
  the *default*, so a driving `calibrate()` would move a gravity-loaded arm on
  every connect with nobody's hand near the rocker switch.
- **No `try/except ImportError` around LeRobot.** The registration decorator only
  runs when the module executes, so a lazy import would silently break
  `--robot.type=emre_arm` while appearing to work. This is a LeRobot plugin; it
  requires LeRobot.
- **No host-side re-implementation of the shoulder mirror.** That arithmetic lives
  at the firmware's single point of write. A second copy would drift.
- **No `j2` anywhere.** The visible gap in `j0 j1 j3 j4 j5 j6` is the documentation.
- **No emergency stop, anywhere, under any name.** `STP` aborts motion and *holds*
  with the joints still driven. `EST`, the watchdog and the idle park all
  *detach*, and a gravity-loaded arm **sags**. The rocker switch and the inline
  fuse are the only real stop.

---

## 10. Cross-references

| Document | What it settles |
|---|---|
| `Documentation/SERIAL-PROTOCOL.md` | the wire contract; §9 is the mandatory connect handshake, §10 the CSV columns |
| `Documentation/MARKER-SYSTEM.md` | the 13-sticker design, the size chain, the two calibration passes |
| `Documentation/specs/2026-08-04-envelope-joystick-design.md` | §3 the vocabulary rule; §12 pre-authorises externally-observed values as a *separate* channel — Option A, written down before this package existed |
| `Software/arm-console/joint-limits.csv` | the calibration of record, with its own header explaining every column |
| `Calibration_Notes/lock-artifacts/` | the board's own receipts; validators H4/H5 check the CSV against these |
| `Software/tests/protocol_check.py` | the firmware's real-board test suite, and the shape `transport.py` reuses |
