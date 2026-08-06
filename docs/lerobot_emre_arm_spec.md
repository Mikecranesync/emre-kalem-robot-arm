# LeRobot adapter for the Emre Kalem arm — specification

The design of record for `lerobot_robot_emre_arm`, an installable LeRobot plugin package
that lets `lerobot-record` and `lerobot-teleoperate` drive this arm.

The house reference documents live in `Documentation/` — `SERIAL-PROTOCOL.md` is the wire
spec and this document defers to it everywhere. This file lives in `docs/` because it is
the adapter spec, not a bench reference.

---

## 0. What this is — and what it is not

This is a supervised hobby bench project, the same as everything else in this repo.

> **The real emergency stop is the KCD1 rocker switch and the inline fuse.**
> Nothing in this document is an emergency stop. `STP` aborts motion and **holds** with the
> joints still driven. `EST`, the `!` byte, and the serial watchdog **detach**, and a
> gravity-loaded arm **sags**. A Python process can die mid-move. The only stop you can
> rely on is removing servo power.

Four things stated up front because they shape every decision below.

**Nothing in this document has been run against the hardware.** The arm cannot currently be
assembled: the bench supply measures 6.62 V at about 700 mA, an assembled seven-servo arm
holding its own weight needs 3–5 A, and a 1 A slow-blow fuse is still outstanding
(`Documentation/SERIAL-PROTOCOL.md` §15). Every rate, latency and byte count here is
**calculated from the firmware source, never measured**. Treat them as budgets to verify,
not as results.

**These servos have no position feedback of any kind.** The firmware knows only what it
last *commanded*; `Servo.read()` returns the last commanded value, not a shaft angle. That
single fact is why this adapter exists in the shape it does, and it is the whole reason for
the Option A observation contract in §3.

**The vocabulary rule applies here as it does everywhere else.** *Commanded*, *target*,
*held*, *accepted* for anything the firmware knows. *Observed* — always paired with a
source and a validity field — for something a camera or a human really saw. The words
**position**, **actual**, **measured** and **feedback** are not used for shaft state.

There is exactly **one knowing violation**, and it is stated rather than hidden: LeRobot's
fixed ecosystem spelling for the command channel is `.pos`, and the stock tooling keys off
it. We keep `.pos` for drop-in compatibility and pay for it in documentation — **`.pos` in
this system means COMMANDED, always, and never a shaft angle.** `observation.py`'s module
docstring says the same thing in the same words, on purpose.

**Status of the pieces.** The Option A contract module is written, executed and committed
(`d2d776a`, as `schema.py`; since renamed to **`observation.py`** — the content is
unchanged and every line reference in §3 still holds). The marker set is committed
(`ae598cb`).

The rest of the package — `emre_arm.py`, `config_emre_arm.py`, `transport.py`,
`calibration.py`, `markers.py`, `pyproject.toml` — was being written **concurrently with
this document, in the same working tree**, and is uncommitted at the time of writing. This
file is a specification, not a description of that code. Where the two differ, the code is
newer; check `§2.1`'s layout against the tree before relying on a filename.

---

## 1. Repo inventory

What is already here, and what each file means for the adapter.

| File | What it is | Verdict | The constraint it imposes |
|---|---|---|---|
| `Software/factorylm_arm_controller/factorylm_arm_controller.ino` | The firmware. 115200 8N1, line-oriented ASCII, non-blocking `millis()` state machine, no `delay()` anywhere. | **The contract.** Do not modify it for the adapter. | 15 verbs (§7 corrects the count). Inbound line ≤ 48 chars including the terminator, else `ERR E8 LINE`. Every angle on the wire is an **integer degree** — centidegrees exist only inside the firmware (`degOf`, `:284`). |
| ↳ the single clamp | `clampToLimits` → `mirrorC` → `writeJoint`, and `writeJoint` (`:441`) is called **only** from the interpolator (`:1272`). | Reuse the invariant, not the code. | The interpolator skips joints where `!en` (`:1260`). No pulse is ever emitted for a disabled joint. This is load-bearing — see §3.6. |
| ↳ `ENA` / `DIS` | `enableJoint()` pre-loads the adopt pulse with `writeMicroseconds()` **before** `attach()` (`:501-537`). `disableJoint()` detaches then drives the pin LOW (`:470-495`). | Wire calls. | The adopt angle is a human's by-eye estimate of where the shaft is sitting **right now**. A wrong one snaps the joint. Nothing in software can produce it. |
| ↳ the watchdog | `WDG <ms>`, 0 = off, else 200–10000. Fed in exactly one place: `handleLine()` after `dispatch()` (`:1153`). | The adapter must generate its own heartbeat. | The raw `!` and `?` bytes bypass line assembly and **do not feed it** (`:1219-1220`). On trip: `EVT WDOG MS=…` then a full latched e-stop — every joint detached (`:1303-1309`). Recovery is `CLR` then a fresh `ENA` per joint. |
| ↳ boot defaults | `DEF_MIN_DEG=70`, `DEF_MAX_DEG=110`, `DEF_SET_C=9000`, `DEF_DPS=30` (`:170-174`), applied in `setup()`. Nothing survives the DTR reset. | — | **J0 home 64, J1 home 1 and J3 home 33 all sit outside 70–110.** An `ENA` at home before the `LIM` push returns `ERR E5`. The limits push is mandatory on every connect, not an optimisation. |
| `Software/arm-console/arm-bridge.py` | Localhost HTTP↔serial pipe, `127.0.0.1:8770` only, per-launch token, Origin/Host checked. | **Not in the adapter's path.** Reuse `list_ports`/`_rank`/`auto_pick` (`:213-251`) and `friendly_open_error` (`:257-272`) by shape only. | `GET /rx` calls `drain()`, which **clears** the deque (`:429-433`) — a second poller steals lines. `Link.tx()` writes outside its own lock (`:407-426`, write at `:418`) — defect C13, already documented in `protocol_check.py:23-26`. It deliberately never heartbeats (`:26-32`). See §4. |
| `Software/arm-console/arm-console.html` | Single-file GUI, vanilla JS, **no build step** — a deliberate virtue. `?selftest=1` runs a pure-maths suite. | **Reference implementation.** Clone the handshake; replace the code. | `pushState()` (`:1419-1443`) is the canonical 14-command state push. Constants that must match the firmware: `LIM_MIN_SPAN=5`, `JOG_BEAT_MS=250`. **Mutually exclusive with the adapter** — one transmitting client. |
| `Software/arm-console/joint-limits.csv` | **The calibration data of record.** 106-line comment header, six data rows. | **Reuse verbatim.** This is `calibrate()`'s input. | No commas in any field. `#` and blank lines skipped, header required, columns looked up **by name**. A single malformed row rejects the whole file. No row for joint 2, deliberately. |
| `Calibration_Notes/lock-artifacts/*.csv` | Eight raw LOCK artifacts, headerless, 11 columns, one per file. | **Data — and the schema to extend for markers.** | Three exist for J1 and two for J3; **newest wins**. The trailing disclaimer string is part of the schema, not decoration. |
| `Calibration_Notes/calibration-log.csv` | Narrative, append-only, seven rows. | Provenance. **Not machine-read.** | Row 2 (2026-08-01) records the servo physically wired to D3 as the **gripper**, not the base. Row 4 flags it still open. |
| `Software/wiring-map.csv` | `index,servo_name,uno_pin,servo_type,servo_type_source,…` | Data — but treat `servo_type` as **inferred**. | Six of seven types are `INFERRED` from the BOM. Only wrist roll (D10) is `DOC-CONFIRMED`. Nothing in the firmware varies by servo type and nothing should. |
| `Documentation/SERIAL-PROTOCOL.md` | The wire spec. | **Read it first.** | §9 (connect handshake) is mandatory on both transports. §15 states the no-feedback rule the whole Option A contract rests on. Three of its statements are now stale relative to the firmware — see §7. |
| `Documentation/specs/2026-08-04-envelope-joystick-design.md` | Design authority for the current firmware and console. | **Read §12 before any marker work.** | §3 is the source of the vocabulary rule. **§12 pre-authorises Option A** — it defers external camera + fiducial markers, states that "externally-observed is a different thing from servo-reported, and conflating them would be worse than the current ban", and leaves the calibration schema explicitly not frozen so observed values can sit beside commanded ones. §4 surveys and rejects ROS 2 + MoveIt; the LeRobot decision supersedes it and that needs saying in that file. |
| `Software/tests/protocol_check.py` | Real-board harness. Default mode enables nothing. | **Reuse the shape.** `Board` (`:67-111`), `find_port()` (`:161-183`) and `sta()` are the adapter's transport skeleton. | Three outcomes — PASS / **PEND** / FAIL — exit 0/1/2. `BOOT_WAIT_S = 2.0`. |
| `Software/tests/selftest.sh` | Headless-Chrome gate for the console's pure maths. | Not reusable for Python. **Copy the discipline**: it extracts `<pre id="selftest">` first and never gates on the browser's exit code. | — |
| `Software/emre_kalem_single_servo_bench_test/…ino` | The proven single-servo calibration tool. | **Leave alone.** It is where calibration actually happens. | It reads a bare `a` as *attach* and a bare digit as *select this pin*. This is exactly why the `VER` / `NAME=FACTORYLM-ARM` gate is mandatory. |
| `Software/vision/stl_face_survey.py`, `stl-face-survey.csv`, `markers.csv` | Binary-STL analyser + 59 measured planar-patch rows + the 13-marker set. Committed at `ae598cb`. | **Data for §6.** | Marker sizes come from largest-inscribed-square on a rasterised patch, not from bounding boxes. `Tabla_Alt` is 113.7 mm across and takes an 11.6 mm marker. |
| `Documentation/MARKER-SYSTEM.md` | The marker design. | **The authority for §6.** This document specifies only the software interface. | Its negative finding is load-bearing and is repeated in §6.1. |
| `Software/lerobot_robot_emre_arm/src/lerobot_robot_emre_arm/observation.py` | **The Option A contract, written and executed.** Committed at `d2d776a` as `schema.py`; renamed, content unchanged. | **Frozen.** §3 quotes it; it does not restate it. | Pure Python, no I/O, no third-party imports — so the feature dicts are callable while disconnected, which LeRobot requires. |
| `Software/conveyor-waypoints-template.csv` | Header only, no data. | **Do not use as a dataset schema.** | It has `shoulder_left_deg` **and** `shoulder_right_deg`. `SERIAL-PROTOCOL.md` §11 specifies six joint columns and no split, "because there is no way to command them separately". §7 records this. |
| `Backups/STL_parts/*.stl` | 21 binary STLs — individual printed parts in print orientation. | Data only. | **No assembly file. No URDF, xacro, STEP or f3d anywhere in the repo.** Link-to-link transforms are UNKNOWN and must be stated as such, not estimated. |

### What does not exist

- **No `LICENSE` file, anywhere in this repository.** The repo declares no licence at all.
  That blocks publishing `lerobot_robot_emre_arm` as a package and it is not a detail to
  fix later — see §8.
- **No kinematic model.** No assembly, no URDF, no joint axes. §6.3 recovers them by
  observation rather than by reading them off a part.
- **No camera or marker runtime before `ae598cb`.** No intrinsics, no calibration images,
  no `solvePnP` anywhere. The marker pipeline is greenfield.
- **LeRobot itself is not installed on this machine.** `import lerobot` →
  `ModuleNotFoundError`. The `Robot` base-class contract used throughout this document
  comes from the official documentation, not from source we have read. §7 lists what must
  be verified against the installed package before the class is written.

---

## 2. Adapter architecture

### 2.1 Package layout

```
Software/lerobot_robot_emre_arm/
├── pyproject.toml
└── src/
    └── lerobot_robot_emre_arm/
        ├── __init__.py           exposes EmreArm + EmreArmConfig eagerly
        ├── emre_arm.py           EmreArm(Robot)
        ├── config_emre_arm.py    EmreArmConfig + the register_subclass decorator
        ├── observation.py        ★ the Option A contract (committed d2d776a)
        ├── transport.py          pyserial link, §9 handshake, heartbeat
        ├── calibration.py        joint-limits.csv + lock-artifact loader, validators
        └── markers.py            MarkerObserver Protocol + NullMarkerObserver
```

The plugin auto-discovery convention is "device class in the same module as its config,
**or** a submodule named after it". `config_emre_arm.py` is the second reading and it is
the one the reference plugins use.

Two things that look like style and are not:

- **`__init__.py` must import `config_emre_arm` eagerly.** The
  `@RobotConfig.register_subclass("emre_arm")` decorator only takes effect when that module
  *executes*. A lazy import — the obvious way to dodge a hard `lerobot` dependency — would
  silently break `--robot.type=emre_arm` discovery with no error message.
- **`observation.py` imports nothing from the rest of the package.** It is the leaf. That is
  what lets the feature dicts be asserted with the board unplugged and `lerobot` uninstalled.

```
pip install -e Software/lerobot_robot_emre_arm

lerobot-record      --robot.type=emre_arm ...
lerobot-teleoperate --robot.type=emre_arm ...
```

Discovery needs no change to LeRobot's source: the package is named
`lerobot_robot_<name>`, the config is registered with
`@RobotConfig.register_subclass("emre_arm")`, and `__init__.py` exposes both classes.

### 2.2 What talks to what

```
        lerobot-record / lerobot-teleoperate / a policy
                          │
                          │  get_observation()  /  send_action()
                          ▼
   ┌──────────────────────────────────────────────────────────┐
   │  EmreArm(Robot)                    emre_arm.py           │
   │    · owns the enable set, the clamp flags, the config     │
   │    · merges frames + fixes + commanded into one dict      │
   └───┬───────────────┬────────────────────┬──────────────────┘
       │               │                    │
       │ build_        │ observe(frames,    │ cmd("MOV 3 110")
       │ observation() │       capture_mono)│ cmd("STA")
       ▼               ▼                    ▼
  observation.py  markers.py           transport.py
  (committed)     MarkerObserver        SerialLink
  pure floats     Protocol              pyserial, DIRECT
       │               │                    │
       │               ▼                    ▼
       │        cv2 + the frames      ══════════════════
       │        LeRobot already        USB · 115200 8N1
       │        grabbed                the Uno · firmware
       │
       └─── calibration.py feeds calibrate() and configure()
```

Three properties of that picture are the architecture:

1. **`transport.py` is the only thing that knows a serial port exists.** Nothing else
   imports pyserial. The marker observer must not be able to write to the wire even by
   accident.
2. **`observation.py` is pure.** It takes a commanded mapping, a clamp mapping and a fix
   mapping, and returns floats. It has no idea where any of them came from. That is what
   makes it assertable with the board unplugged.
3. **The adapter opens the port directly.** It does not speak HTTP to `arm-bridge.py`.
   §4 gives the three reasons.

### 2.3 The LeRobot `Robot` contract, mapped

| LeRobot member | This adapter |
|---|---|
| `config_class` | `EmreArmConfig` |
| `name` | `"emre_arm"` → `--robot.type=emre_arm` |
| `observation_features` | `observation.observation_features({name: (h, w, 3) …})`. Pure function of config — callable while disconnected. |
| `action_features` | `observation.action_features()`. Six keys. |
| `is_connected` | the link is open |
| `connect(calibrate=True)` | open → 2000 ms → flush → `VER` gate → `calibrate()` → `configure()`. **Leaves every joint disabled.** |
| `disconnect()` | `STP` → `DIS A` → close cameras → close port |
| `is_calibrated` | computed and connection-aware. **False today.** §5.4 |
| `calibrate()` | **loads and validates files. Never drives.** §5.1 |
| `configure()` | pushes `WDG`, `LIM`, `SPD`, `MIR`, reads back `STA`, and performs the `ENA` for any joint that has an adopt angle. §5.6 |
| `get_observation()` | one `STA` round trip + camera frames + optional marker fixes → `build_observation()` |
| `send_action(action)` | one `MOV` per **changed** target; returns the **clamped** value |

### 2.4 The rate budget — and why 30 fps is not available

One `STA` reply is eight lines. Counted from `doSta()` (`factorylm_arm_controller.ino:569-596`)
with the field widths of the worked reply in `SERIAL-PROTOCOL.md` §4 plus the `JTO=` field
the firmware actually emits:

```
6 joint lines × 67 B   = 402 B
1 SYS line             =  54 B
1 "OK STA N=6"         =  12 B
inbound "STA\n"        =   4 B
                        ──────
                         472 B
```

At 115200 8N1 that is 10 bits per byte, so **472 B ≈ 41 ms of blocking transmit** and the
ceiling is **about 24 `STA` per second at 100 % UART duty** — before the USB latency timer
(typically 16 ms on Windows) and before any Python overhead. The firmware's own comment
calls it a "~36 ms burst" (`:1252-1253`). The console polls at **4 Hz** and has done since
it shipped.

**Consequence: LeRobot's 30 fps default is not reachable through `STA`. Use `fps ≤ 20`.**

Two mitigations, both available because of Option A:

- Poll `STA` at 4 Hz for **health** — `EN`, `ES`, `WD`, `CAL`, `JTO`, `MOV` — and source
  `.pos` from the adapter's own command state, which the host already knows exactly. This
  is the whole reason Option A defines `.pos` as commanded rather than pretending it is a
  reading. §3.7 records the open question this raises.
- **`MOV` is target-seeking, not position-streaming.** `MOV` sets `tgtC` (`:821`) and the
  interpolator walks `setC` toward it at the joint's `dps` (`:1259-1274`). Teleop, replay
  and policy therefore send `MOV` only when a joint's **target changes**, not six `MOV`s
  every control tick. An author with Feetech or Dynamixel streaming habits will send six
  sequential round trips per tick, hit the wall at 10–30 Hz, and wrongly conclude the arm
  cannot go faster.

### 2.5 The `SET=` collision — read this before writing a parser

The same wire key carries two different quantities depending on which reply it is in.

```
MOV ack:   OK MOV J3 REQ=140 SET=110 CL=1      SET is the ACCEPTED TARGET   (tgtC)
STA line:  STA J3 EN=1 SET=94 TGT=110 …        SET is the CURRENT COMMANDED (setC)
```

`doMov` prints `SET=` from `applied`, which it has just assigned to `tgtC` (`:821`, `:831`).
`doSta` prints `SET=` from `degOf(j[i].setC)`, the interpolator's present output (`:576`).

A flat `key=value` parser applied uniformly to every reply line — the shape of the
console's `kv()` — conflates them and will silently record **targets as commanded angles**.
Parse per reply context. This is the single most likely quiet-corruption bug in the whole
adapter.

### 2.6 The config dataclass

```python
@RobotConfig.register_subclass("emre_arm")
@dataclass
class EmreArmConfig(RobotConfig):
    port: str | None = None            # None => auto-pick; REFUSE when ambiguous
    limits_csv: Path = Path("Software/arm-console/joint-limits.csv")
    lock_artifacts_dir: Path = Path("Calibration_Notes/lock-artifacts")

    joints: tuple[int, ...] = (0, 1, 3, 4, 5, 6)

    # ADOPT ANGLES -- the human step, made expressible and recorded.
    # ENA <j> <adopt_deg> takes a by-eye estimate of where the shaft is sitting
    # RIGHT NOW. Nothing in software can synthesise it. None => connect()
    # completes the handshake and ENABLES NOTHING.
    adopt_deg: dict[int, float] | None = None

    # SAFETY GATES. Both triggers are live today; neither is speculative.
    allow_unmeasured_mirror: bool = False     # refuse ENA 1 while the offset is a placeholder
    allow_uncalibrated_joints: bool = False   # refuse ENA 6 while calibrated=no

    watchdog_ms: int = 4000            # WDG. 0 would disable it -- never do that.
    heartbeat_s: float = 0.25          # PNG cadence
    idle_disable_s: float = 2.0        # no loop call for this long => DIS A
    max_fix_age_s: float = 0.25        # older => STALE, residual withheld

    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    marker_observer: MarkerObserver | None = None
```

Two of those knobs deserve their justification stated, because "config flag with no caller"
is normally a smell:

- **`allow_unmeasured_mirror`.** The firmware's own guard does **not** fire in the current
  configuration. `joint-limits.csv` sets `mirror_mode=inverted`, and `ERR E13` only fires
  on `MIR=UNKNOWN` (`:782-787`). So the firmware happily accepts `ENA 1` while
  `mirror_offset_deg` is still the unmeasured placeholder `0` — which is precisely the case
  where two MG996Rs can fight each other for the whole time joint 1 is driven. The guard has
  to live at this layer because the layer below is already satisfied.
- **`adopt_deg`.** Not a convenience. `ENA` cannot be issued without it and nothing can
  generate it, so a `connect()` that enables anything on its own is impossible by
  construction. §5.7 draws out what that means for unattended recording.

---

## 3. The Option A observation contract

### 3.1 Why Option A

`.pos` is what the firmware was **told** to hold. There is no second source. A schema with
one ambiguous "position" field would therefore be a lie in every row of every dataset —
and a policy trained on it would learn the lie.

Option A is: **`.pos` is commanded, and observed-from-markers lives in separate, differently
named fields, with a residual, a source, a validity mask and a staleness age.** Commanded
and observed are never fused.

This was pre-authorised in this repo before the LeRobot decision existed.
`Documentation/specs/2026-08-04-envelope-joystick-design.md` §12 defers external cameras
and fiducial markers, requires an explicit carve-out because "externally-observed is a
different thing from servo-reported, and conflating them would be worse than the current
ban", and leaves the calibration schema not frozen so observed values can sit beside
commanded ones. The adapter cites that; it does not reinvent it.

### 3.2 The committed contract

The text below is quoted from
`Software/lerobot_robot_emre_arm/src/lerobot_robot_emre_arm/observation.py` — committed at
`d2d776a` under its original name `schema.py`, renamed since, content unchanged.

**That file is the contract; this section is a pointer to it.** If the two ever disagree,
the file wins and this section is a defect. Every block below was checked line-for-line
against the file; two are marked *abridged*, meaning docstrings were elided and **no code
line was altered**.

Joints and the reserved id — `observation.py:100-114`:

```python
#: The addressable logical joints, ascending. Six of them; seven physical servos.
JOINT_IDS: tuple[int, ...] = (0, 1, 3, 4, 5, 6)

#: D5, the shoulder pair's second servo. Never addressable, never a dataset key.
RESERVED_JOINT_ID: int = 2

#: Display only -- for log lines and error messages. NEVER used to build a key.
JOINT_LABELS: dict[int, str] = {
    0: "Base (D3)",        # identity DISPUTED: calibration-log 2026-08-01 says gripper
    1: "Shoulder (D4+D5)",  # one logical joint, mirrored pair
    3: "Elbow (D6)",
    4: "Wrist pitch (D9)",
    5: "Wrist roll (D10)",
    6: "Gripper (D11)",     # UNCALIBRATED -- limits are the full electrical range
}
```

There is **no `j2` key, not even a NaN placeholder.** The visible gap in
`j0 j1 j3 j4 j5 j6` is the documentation. It matches the `STA` dump, which never emits a
`J2` line, and the waypoints CSV, which has one column per logical joint.

Keys are **id-keyed, never name-keyed**. `calibration-log.csv` row 2 records that the servo
physically wired to D3 was the gripper, not the base, and that dispute is still open. Limits
follow the **pin**, so an id-keyed dataset stays true whichever way it resolves; a
`base.pos` column would bake a possible mislabel permanently into every recorded episode.

The observation-source enum — `observation.py:126-138`:

```python
OBS_SOURCE_NONE: float = 0.0      # no observer configured, or never a fix for this joint
OBS_SOURCE_STALE: float = 1.0     # a fix exists but is older than max_fix_age_s
OBS_SOURCE_OCCLUDED: float = 2.0  # observer ran this frame and could not see the marker
OBS_SOURCE_MARKER: float = 3.0    # a camera observed it this frame
OBS_SOURCE_HUMAN: float = 4.0     # a human read it off the arm and typed it in
```

It is a **float** because that is the only dtype LeRobot packs into `observation.state`,
and it is **categorical despite the numeric type**: use it as a mask
(`src == OBS_SOURCE_MARKER`), never as a continuous regression input. If a policy genuinely
needs it, one-hot it at training time from `names`.

`OCCLUDED` and `NONE` are deliberately distinct. "I looked and could not see it" and "I
never looked" are different facts, and only the first tells you the marker is obscured, the
lighting failed, or the link moved out of frame.

The field layout — `observation.py:147-154`:

```python
OBSERVATION_FIELDS: tuple[str, ...] = (
    "pos",
    "observed_deg",
    "residual_deg",
    "obs_source",
    "obs_age_s",
    "clamped",
)
```

Ordering is **field-major** — all six joints' `pos`, then all six `observed_deg`, and so
on — so each block is contiguous and sliceable, and `state[:6]` is the commanded vector on
day one.

The tail singleton — `observation.py:170`:

```python
PAIR_DISAGREE_KEY: str = "j1.pair_disagree_deg"
```

Joint 1's two servos are commanded from one logical angle mirrored about
`90 + mirror_offset_deg`. **That offset has never been measured on this arm** and sits at
the placeholder `0`. If the true axis is not 90, the two MG996Rs fight each other by twice
the error for the whole time joint 1 is driven — they hold, run hot, and eventually strip a
gear, and nothing on screen shows it. An observer that can see both shoulder horns is the
only mechanism this project has ever had for measuring that offset, and this is where it
reports. It is NaN today and will stay NaN until a two-marker observer exists, but it is
declared **now** so the schema does not churn later. It sits after every six-wide block
rather than making one block ragged.

The two feature dicts — `observation.py:192-223`, *abridged*:

```python
def action_features() -> dict[str, type]:
    """What `send_action()` accepts: one commanded angle per logical joint.

    Six keys, not seven. There is no way to command the shoulder's two servos
    separately, so exposing them separately would let a policy express the single
    highest-consequence mechanical failure available on this arm.
    """
    return {joint_key(j, "pos"): float for j in JOINT_IDS}


def observation_features(
    cameras: Mapping[str, tuple[int, int, int]] | None = None,
) -> dict[str, type | tuple[int, int, int]]:
    features: dict[str, type | tuple[int, int, int]] = {}
    for field in OBSERVATION_FIELDS:          # field-major -- see module docstring
        for joint_id in JOINT_IDS:
            features[joint_key(joint_id, field)] = float
    features[PAIR_DISAGREE_KEY] = float
    for name, shape in (cameras or {}).items():
        features[name] = shape
    return features
```

The schema is **stable**: the observed / residual / source / age columns are declared
whether or not an observer is configured. Dropping them when no observer exists would
change the feature schema between runs, which breaks `lerobot-record --resume` and makes
two datasets impossible to concatenate. On day one they are simply all NaN with
`obs_source = OBS_SOURCE_NONE`.

The observer's half of the contract — `observation.py:241-271`, *abridged*:

```python
@dataclass(frozen=True)
class JointFix:
    angle_deg: float
    source: float
    captured_mono: float
    pair_disagree_deg: float | None = None
```

`captured_mono` is `time.monotonic()` at **frame capture**, not at the moment `observe()`
was called. LeRobot camera reads come from a background thread and return the most recent
frame, so call time can lag capture time by a whole frame interval — using it would
silently under-report staleness and inflate trust in an old fix.

And the assembler — `observation.py:307-344`:

```python
    out: dict[str, float] = {}

    for joint_id in JOINT_IDS:
        cmd = float(commanded[joint_id])
        fix = (fixes or {}).get(joint_id)

        observed = math.nan
        residual = math.nan
        age = math.nan
        source = OBS_SOURCE_NONE

        if fix is not None:
            age = now_mono - fix.captured_mono
            source = fix.source
            if fix.source in (OBS_SOURCE_MARKER, OBS_SOURCE_HUMAN):
                observed = float(fix.angle_deg)
                if age <= max_fix_age_s:
                    # Residual is only meaningful when the observation and the
                    # command are contemporaneous.
                    residual = observed - cmd
                else:
                    # Keep the value and the age -- they are informative -- but
                    # demote the source and refuse to publish a residual computed
                    # across a stale gap.
                    source = OBS_SOURCE_STALE

        out[joint_key(joint_id, "pos")] = cmd
        out[joint_key(joint_id, "observed_deg")] = observed
        out[joint_key(joint_id, "residual_deg")] = residual
        out[joint_key(joint_id, "obs_source")] = source
        out[joint_key(joint_id, "obs_age_s")] = age
        out[joint_key(joint_id, "clamped")] = 1.0 if clamped.get(joint_id) else 0.0

    shoulder_fix = (fixes or {}).get(1)
    disagree = shoulder_fix.pair_disagree_deg if shoulder_fix is not None else None
    out[PAIR_DISAGREE_KEY] = math.nan if disagree is None else float(disagree)

    return out
```

**Why NaN and not something friendlier.** With no observer, every `observed_deg`,
`residual_deg` and `obs_age_s` is NaN. The alternatives were both rejected:
`observed = commanded` asserts the arm is exactly where it was told to be, which is
precisely the fiction this schema exists to destroy; `observed = 0.0` claims the joint was
observed at zero degrees. Both are *silently* wrong. NaN is *loudly* wrong, and it is paired
with an always-finite `obs_source` so a consumer always has a clean mask.

**Known consequence, stated not hidden:** a column that is entirely NaN makes LeRobot's
dataset mean and standard deviation for that column NaN, and normalising against it yields
NaN activations. That is intended — it fails immediately rather than quietly. A day-one
training config should select `observation.state[:6]` plus the camera keys and simply not
consume the observation block until an observer exists.

### 3.3 The features, as executed

Run against the committed file, not hand-written:

```python
>>> action_features()
{'j0.pos': float, 'j1.pos': float, 'j3.pos': float,
 'j4.pos': float, 'j5.pos': float, 'j6.pos': float}

>>> observation_features({'overhead': (720, 1280, 3), 'wrist': (720, 1280, 3)})
{'j0.pos': float,               'j1.pos': float,               'j3.pos': float,
 'j4.pos': float,               'j5.pos': float,               'j6.pos': float,
 'j0.observed_deg': float,      'j1.observed_deg': float,      'j3.observed_deg': float,
 'j4.observed_deg': float,      'j5.observed_deg': float,      'j6.observed_deg': float,
 'j0.residual_deg': float,      'j1.residual_deg': float,      'j3.residual_deg': float,
 'j4.residual_deg': float,      'j5.residual_deg': float,      'j6.residual_deg': float,
 'j0.obs_source': float,        'j1.obs_source': float,        'j3.obs_source': float,
 'j4.obs_source': float,        'j5.obs_source': float,        'j6.obs_source': float,
 'j0.obs_age_s': float,         'j1.obs_age_s': float,         'j3.obs_age_s': float,
 'j4.obs_age_s': float,         'j5.obs_age_s': float,         'j6.obs_age_s': float,
 'j0.clamped': float,           'j1.clamped': float,           'j3.clamped': float,
 'j4.clamped': float,           'j5.clamped': float,           'j6.clamped': float,
 'j1.pair_disagree_deg': float,
 'overhead': (720, 1280, 3),    'wrist': (720, 1280, 3)}
```

**37 float features. 6 action features.** Verified by execution:

```
OBSERVATION_STATE_NAMES[ 0: 6]  == the six '.pos'          keys
OBSERVATION_STATE_NAMES[ 6:12]  == the six '.observed_deg' keys
OBSERVATION_STATE_NAMES[12:18]  == the six '.residual_deg' keys
OBSERVATION_STATE_NAMES[18:24]  == the six '.obs_source'   keys
OBSERVATION_STATE_NAMES[24:30]  == the six '.obs_age_s'    keys
OBSERVATION_STATE_NAMES[30:36]  == the six '.clamped'      keys
OBSERVATION_STATE_NAMES[36]     == 'j1.pair_disagree_deg'

'j2.pos' not in observation_features()
joint_key(2, 'pos') raises ValueError
```

### 3.4 Worked example — day one, no observer

Commanded values are the real `home_deg` column from `joint-limits.csv`. **This is what
every row looks like until a marker pipeline exists**, so it is the important example, not
the boring one.

```python
build_observation(
    commanded={0: 64.0, 1: 1.0, 3: 33.0, 4: 90.0, 5: 104.0, 6: 90.0},
    clamped={6: True},          # the gripper was asked past its range
    fixes=None,                 # NullMarkerObserver, or no observer at all
    now_mono=1000.0,
)
```

```python
{'j0.pos': 64.0,  'j1.pos': 1.0,   'j3.pos': 33.0,
 'j4.pos': 90.0,  'j5.pos': 104.0, 'j6.pos': 90.0,

 'j0.observed_deg': nan, 'j1.observed_deg': nan, 'j3.observed_deg': nan,
 'j4.observed_deg': nan, 'j5.observed_deg': nan, 'j6.observed_deg': nan,

 'j0.residual_deg': nan, 'j1.residual_deg': nan, 'j3.residual_deg': nan,
 'j4.residual_deg': nan, 'j5.residual_deg': nan, 'j6.residual_deg': nan,

 'j0.obs_source': 0.0,   'j1.obs_source': 0.0,   'j3.obs_source': 0.0,
 'j4.obs_source': 0.0,   'j5.obs_source': 0.0,   'j6.obs_source': 0.0,

 'j0.obs_age_s': nan,    'j1.obs_age_s': nan,    'j3.obs_age_s': nan,
 'j4.obs_age_s': nan,    'j5.obs_age_s': nan,    'j6.obs_age_s': nan,

 'j0.clamped': 0.0,      'j1.clamped': 0.0,      'j3.clamped': 0.0,
 'j4.clamped': 0.0,      'j5.clamped': 0.0,      'j6.clamped': 1.0,

 'j1.pair_disagree_deg': nan}
```

Not one field pretends to know something it does not. The six commanded values are real;
everything observational is NaN with a finite source of `0.0` saying exactly why.

### 3.5 Worked example — every source state exercised

`now_mono = 1000.0`, `max_fix_age_s = 0.25`:

| joint | `pos` | `observed_deg` | `residual_deg` | `obs_source` | `obs_age_s` | `clamped` |
|---|---|---|---|---|---|---|
| j0 | 64.0 | 61.4 | **-2.6** | 3.0 marker | 0.03 | 0.0 |
| j1 | 1.0 | 3.2 | **2.2** | 3.0 marker | 0.04 | 0.0 |
| j3 | 33.0 | nan | nan | 2.0 occluded | 0.03 | 0.0 |
| j4 | 90.0 | nan | nan | 0.0 none | nan | 0.0 |
| j5 | 104.0 | 99.0 | nan | 1.0 stale | 1.90 | 0.0 |
| j6 | 90.0 | 72.0 | **-18.0** | 4.0 human | 0.01 | 1.0 |

`j1.pair_disagree_deg = 4.8`

Row by row, because each one is a rule:

- **j0** — a live camera fix 31 ms old, so the residual is published: the link is 2.6°
  short of where it was told to be. This is the number the whole contract exists to
  produce.
- **j3** — the observer *ran* and could not see the marker. `observed_deg` and
  `residual_deg` are NaN, but `obs_age_s` is finite at 0.03, which is the proof it looked.
- **j4** — no fix entry at all. Everything NaN, source `none`. **Distinct from j3**, and
  that distinction is the point of having two codes.
- **j5** — the fix is 1.9 s old, past the 0.25 s window. The value and the age are **kept**
  because they are informative; the residual is **withheld** because a residual computed
  across a stale gap is a fiction; the source is demoted to `stale`.
- **j6** — a human read the gripper with a protractor: commanded 90, observed 72,
  residual −18°, **and** clamped. This single row is the uncalibrated gripper reporting its
  own problem, which is exactly the case `joint-limits.csv` warns about — 0–180 is the
  servo's electrical range and the linkage stops far sooner.
- **the tail** — 4.8° of shoulder-pair disagreement. The first number this project has ever
  had that bears on the unmeasured `mirror_offset_deg`.

### 3.6 How it lands in a LeRobot dataset

The surprising part first: **no Option A field gets its own dataset column.** LeRobot packs
every feature whose declared type is `float` into one flat array.

| Adapter feature | Declared | Dataset column | dtype | Shape |
|---|---|---|---|---|
| `j{0,1,3,4,5,6}.pos` (observation) | `float` | `observation.state[0:6]` | float32 | (37,) |
| `j*.observed_deg` | `float` | `observation.state[6:12]` | float32 | (37,) |
| `j*.residual_deg` | `float` | `observation.state[12:18]` | float32 | (37,) |
| `j*.obs_source` | `float` | `observation.state[18:24]` | float32 | (37,) |
| `j*.obs_age_s` | `float` | `observation.state[24:30]` | float32 | (37,) |
| `j*.clamped` | `float` | `observation.state[30:36]` | float32 | (37,) |
| `j1.pair_disagree_deg` | `float` | `observation.state[36]` | float32 | (37,) |
| `j{0,1,3,4,5,6}.pos` (action) | `float` | `action[0:6]` | float32 | (6,) |
| `overhead` / `wrist` | tuple | `observation.images.<name>` | video | (720,1280,3) |
| anything on `j2` | **absent** | — | — | — |

`observation.state.names` is `OBSERVATION_STATE_NAMES`; `action.names` is `ACTION_NAMES`.

**Consumer rule: derive indices from `names` at load time. Never hardcode them** — adding a
joint or a field silently shifts every downstream slice.

**Why everything is declared `float`.** The packer selects float features; a field typed
`str`, `bool` or `int` would be **silently dropped** from the dataset. Declaring every
Option A field as a float makes the schema survive whatever the exact predicate turns out
to be. (§7 lists this as a must-verify once LeRobot is installed. The design is correct
either way; only the explanation changes.)

**Why age in seconds and not a timestamp.** `observation.state` is float32, and a float32
quantises epoch time to **128.0 seconds** at 1.75×10⁹ — verified with numpy, the next
representable value after `1750000000.0` is `1750000128.0`. Every fix inside the same
two-minute window would collapse to one indistinguishable number. The same float32 holds an
age of 0.031 s to about 2×10⁻⁹ s. Staleness is also the non-redundant quantity, because
`LeRobotDataset` already stamps every frame with its own `timestamp` column.

### 3.7 What this contract cannot say — two gaps, stated

**Gap 1 — the contract cannot flag a `.pos` that no servo ever received.**

`writeJoint()` (`:441`) is reached only from the interpolator (`:1272`), which skips joints
where `!en` (`:1260`). `doLimSet`'s disabled branch mutates `setC` without writing a pulse
(`:682-688`). And `setup()` seeds every joint to `DEF_SET_C = 9000` (`:172`, `:1175-1186`).
Net effect: **a freshly booted board with nothing attached reports `SET=90` on all six
joints, and no servo has ever been driven.**

The committed six per-joint fields are `pos / observed_deg / residual_deg / obs_source /
obs_age_s / clamped`. **There is no field derived from `EN=`.** So the frozen contract
cannot express "this `.pos` is bookkeeping because the joint was never enabled". That is a
real limitation of the shipped schema, not an omission from this document.

Three things partly cover it and none of them closes it:

- `send_action()` returns NaN for a joint that is not enabled (§2.3), so the **action** side
  is honest.
- `configure()` refuses to enable a joint that fails a gate and logs which ones (§5.6).
- The adapter knows its own enable set and can refuse to record at all until every joint in
  `config.joints` is enabled.

If it is ever closed, the place is a seventh per-joint field — `j<N>.driven`, sourced from
`STA`'s `EN=` — appended after `clamped` and before the tail singleton. **Do not add it to
this document without adding it to `observation.py` in the same change**, or the repo carries two
contradictory Option A contracts.

**Gap 2 — `.pos` is not yet pinned to one quantity.**

`build_observation()` takes `commanded` from its caller and is deliberately agnostic. Three
candidates, all defensible:

1. **`tgtC`** (`STA … TGT=`) — what we asked for.
2. **`setC`** (`STA … SET=`) — what the board is pulsing right now, mid-slew.
3. **the host's model of `setC`** — the adapter knows `dps`, `TICK_MS = 20` (`:196`) and
   `MAX_STEP_C = 200` (`:207`), so it can compute the interpolator's output exactly between
   4 Hz polls.

`STA` gives (1) and (2) but only at 4 Hz, and (3) is a model, not the board's truth. **This
must be decided before `observation_features` is treated as frozen in a published dataset**,
because the choice changes what a policy learns during every slew. It is deliberately left
open here rather than settled by whichever line of code got written first.

Related and also unresolved: the wire is **integer degrees in both directions** (`degOf`,
`:284`; `MOV` takes an integer, `:813`). So `.pos` and every action carry up to ±0.5° of
quantisation, and `residual_deg` is measured against that before any marker error at all.
If the marker pipeline ever resolves better than about 1°, the residual is dominated by
rounding rather than by real mechanical error — and fixing that means a centidegree wire
verb, a protocol v1.1 change, out of scope for this adapter.

---

## 4. Serial ownership — the numbered contract

Twenty-five items. They are numbered because they get cited.

### A. Ownership

1. The serial port has exactly **one owner process** at a time. The owner is whichever
   process holds an open handle. Ownership is process-wide — not per-thread, not
   per-object. There is no shared-ownership mode and none will be added.
2. **The owner is the only transmitter.** No other process, thread, script, browser tab or
   tool may write a single byte — not a status poll, not a heartbeat, not a bare `!`.
3. Ownership is exclusive **by construction**, not by convention. On Windows the OS makes
   COM opens exclusive, so the operating system is the mutex. On POSIX, pass
   `exclusive=True` to `serial.Serial` explicitly, and verify that behaviour on that
   platform before relying on it.
4. **The reason is the wire protocol, not politeness.** Replies correlate to commands by
   echoed verb only; there are no sequence numbers, and the host keeps exactly one command
   outstanding (`SERIAL-PROTOCOL.md` §2). Two writers cannot attribute replies to their own
   commands. **Serialising the writes is not sufficient — attribution is what breaks.**
5. Exactly one of these runs at a time, and starting one requires closing the others:
   the bridge + console; the LeRobot adapter; `Software/tests/protocol_check.py`; the
   Arduino IDE Serial Monitor. Switching between them is a **mode switch**. It is not a
   limitation to engineer around.

### B. Heartbeat and watchdog

6. **The owner generates its own heartbeat.** It must originate in the process whose death
   should stop the arm. It is never relayed, proxied, or generated on a client's behalf —
   a proxy survives its client's crash and keeps the arm driven, which turns the watchdog
   into decoration. `arm-bridge.py:26-32` refuses to do this for the GUI for exactly this
   reason.
7. The heartbeat is **`PNG\n` every 250 ms on an independent timer**. It must be a complete,
   well-formed line: `lastRxMs` is fed in exactly one place, after a line arrived complete,
   held a legal three-letter verb, and reached a handler (`:1153`). The raw bytes `!` and
   `?` **do not** feed it (`:1219-1220`). Do not heartbeat with `?` because it is cheaper.
   (`STA` also feeds the watchdog, so a 4 Hz poll would technically suffice — keep the
   independent `PNG` timer anyway so the two failure modes stay separable.)
8. **Push `WDG` before any `ENA`.** The watchdog arms only when `wdgMs != 0` **and** nothing
   is latched **and** at least one joint is enabled (`:1303`). Enabling first opens a window
   in which a crash leaves the arm driven with no host and no timer. Send `WDG` at connect,
   require the `OK WDG` terminator, and confirm `WDMS=` on a `STA` before the first `ENA`.

### C. Acquiring the port

9. **Pre-flight before opening:** TCP-connect to `127.0.0.1:8770`. If anything is listening,
   the bridge is running — refuse to open and tell the operator to close the black
   FactoryLM Arm GUI window. Costs nothing, needs no access token, and turns a cryptic
   failure into an instruction.
10. **On open failure, name the likely holder in plain English.** Reuse the wording already
    proven in `arm-bridge.py:257-272`: a permission error almost always means the Arduino
    IDE Serial Monitor or the bridge is holding the port. Never surface a raw traceback to
    the operator.
11. **The lockfile is third and cosmetic.** Write it only *after* a successful open (pid,
    device, process name, start time), delete it in a `finally`, and treat a dead pid as
    absent. Its only job is improving the error sentence. If it starts growing stale-lock
    logic, delete the feature — items 3 and 9 already cover the real cases.
12. **Runtime cross-writer detection is verb-echo desync.** The owner knows which verb it
    sent; a terminator echoing a different verb proves another writer exists. Log it loudly
    and stop. This is a **smoke detector, not a mutex** — two concurrent `MOV`s both return
    `OK MOV`, so it catches cross-verb interleave only. Prevention is item 3.

### D. Observation without writing

13. **Observation-only components never touch the port.** They subscribe in-process to the
    owner. A component that cannot get what it needs without writing is misdesigned — say
    so rather than granting it write access.
14. The marker observer needs a **camera** and a time-aligned **commanded vector**, not
    serial access. In-process, the frame and the commanded vector are stamped on one clock
    and are aligned by construction. Out-of-process you would owe yourself clock
    synchronisation for no benefit.
15. **The observer bus broadcasts; it never consumes.** Every RX line is fanned out to every
    subscriber, and no subscriber's read removes a line from another subscriber's view.
    This is the direct lesson of the bridge's destructive `GET /rx` drain
    (`arm-bridge.py:429-433`), where a second poller silently steals lines from the first. A
    subscriber that falls behind drops its **own** oldest lines and reports the drop.
16. **Unsolicited lines go to every subscriber and are never treated as terminators.**
    `EVT` (e-stop fired, watchdog tripped), `RDY` (boot banner) and `;` (comments) are
    asynchronous. A host that treats `EVT` as a terminator hangs; a host that ignores `EVT`
    misses a stop it did not initiate. `EVT ESTOP` and `EVT WDOG` must reach the observation
    record and the log, not just a console.

### E. Issuing commands

17. **One command outstanding, always.** Send, then read lines until exactly one `OK …` or
    `ERR …` terminator, then send the next. That is the entire read-loop algorithm and it
    has no special cases.
18. **`MOV` is target-seeking, not position-streaming** — §2.4. Send `MOV` only when a
    joint's target changes.
19. **Beware the `SET=` collision** — §2.5. Map it deliberately; never assume.

### F. Shutdown and crash

20. **Clean shutdown is ordered:** `STP` (abort motion, joints still held) → `DIS A`
    (detach; **the arm sags**) → close the port. Do it while the operator is present and
    ready to support the arm. A supervised sag is better than letting the watchdog detach it
    a second after the process is gone with nobody watching.
21. **Crash behaviour is a sag, not a stop.** If the owner dies with joints enabled: the
    heartbeat stops; the firmware watchdog trips after `WDMS`; it emits `EVT WDOG`, detaches
    every channel and drives every pin LOW. A de-energised gravity-loaded arm falls. Between
    the crash and the trip the arm is **driven with no host**. Recovery is always `CLR` then
    `ENA <j> <fresh adopt angle>` per joint, because the staleness is *mechanical* — the arm
    moved while nothing was commanding it.
22. **Register a best-effort shutdown hook** (`finally` / `atexit` / SIGINT-SIGTERM) that
    sends `!` and closes. This is a **courtesy**, exactly like the console's unload beacon —
    not a guarantee. A killed process, a yanked USB cable or a hung interpreter runs no hook.
23. **Nothing in this contract is an emergency stop.** Not the watchdog, not `EST`, not `!`,
    not item 22. The rocker switch and the inline fuse are the only real stop.

### G. Scope and migration

24. The adapter talks to the port through a `Transport` interface — `open` / `close` /
    `command` / `subscribe` — so the `Robot` class never learns which transport it has.
    **Ship `DirectSerialTransport` only.**
25. **Do not build a bridge-backed transport today.** Routing the adapter through the bridge
    first requires three things: a sequence-numbered non-destructive read to replace the
    destructive drain; per-client reply attribution — a **broker**, not a writer lease, see
    item 4; and an observe-only mode in a 136 KB single-file GUI with intricate awaiter
    logic. Promote the bridge to sole owner only if a second **live** consumer is ever
    genuinely needed, and budget those three items when you do.

### The acquisition guard, concretely

```python
BRIDGE_PROBE = ("127.0.0.1", 8770)          # matches arm-bridge.py HOST/PORT

class PortBusy(RuntimeError):
    """Plain English only. The operator reads this, never a traceback."""

def _bridge_is_running() -> bool:           # contract item 9
    s = socket.socket(); s.settimeout(0.25)
    try:
        s.connect(BRIDGE_PROBE); return True
    except OSError:
        return False
    finally:
        s.close()

class DirectSerialTransport(Transport):
    def open(self) -> None:
        if _bridge_is_running():
            raise PortBusy(
                "The arm bridge is already running, which means the console owns "
                "the USB port. Close the black FactoryLM Arm GUI window, then start "
                "again. Only one program can talk to the board at a time.")
        try:
            # exclusive=True is the POSIX half of contract item 3. On Windows the
            # OS already makes this exclusive; VERIFY the POSIX behaviour before
            # relying on it.
            self._ser = serial.Serial(self._device, 115200, timeout=0.4, exclusive=True)
        except PermissionError as exc:
            raise PortBusy(
                "Could not open %s. Almost always this means the Arduino IDE Serial "
                "Monitor or the arm console is still holding the port. Close it and "
                "try again. (%s)" % (self._device, exc)) from exc

# Deliberately NOT implemented: class BridgeTransport. See contract item 25.
```

---

## 5. Calibration mapping

### 5.1 `calibrate()` never drives the arm

`calibrate()` is a **pure load, resolve and validate** of
`Software/arm-console/joint-limits.csv` plus `Calibration_Notes/lock-artifacts/*.csv`,
cross-checked against `Software/wiring-map.csv`. It sends nothing. Driving stays where it
already is — the console's operator-driven **LOCK THIS AXIS** flow — and that flow's
downloaded artifacts are `calibrate()`'s *input*.

Five reasons, weakest first:

1. To drive anything you must `ENA j <adopt_deg>`, and the adopt angle asserts where the
   shaft physically is. **Nothing in this system can produce that number.** `enableJoint()`
   pre-loads the adopt pulse before `attach()` precisely because a wrong one snaps the
   joint. A `calibrate()` that guessed would fire the exact landmine the firmware exists to
   prevent.
2. LeRobot's `connect(calibrate=True)` is the **default**, so a driving `calibrate()` would
   move a gravity-loaded arm on every connect, unattended, with nobody's hand on the rocker
   switch.
3. The existing LOCK flow refuses to run unless the joint is enabled **and** the operator
   has dragged the envelope handles (`arm-console.html:1381-1388`). Automating it would
   record "a range nobody chose", which is what that guard is for.
4. The bench supply is about 700 mA and cannot hold an assembled arm. Auto-enabling six
   joints on connect is a brownout.
5. **The counterfactual, and the strongest argument.** A load-and-validate `calibrate()`
   that cross-checks the CSV against the newest lock artifact per joint **would have caught
   the one real calibration bug this project has had**: the 2026-08-05 sweep that silently
   widened J0 from 29–110 back to 0–180 because the lock existed only in a Downloads
   folder. A driving `calibrate()` would have caught nothing.

**Declared divergence from LeRobot convention.** LeRobot expects `calibrate()` to persist to
a writable `calibration_fpath`. This adapter does not. `joint-limits.csv` plus
`lock-artifacts/` are the sole source of truth; a second writable copy would recreate
exactly the two-copies-disagreeing failure that widened J0. `calibration_fpath` points at
the CSV **read-only**, and `_save_calibration()` raises:

```python
raise NotImplementedError(
    "Calibration is recorded by the arm console's LOCK THIS AXIS flow, which "
    "requires an operator. See the joint-limits.csv header.")
```

> **Divergence from the concurrent implementation.** `emre_arm.py:140` makes **`calibrate()`
> itself** the raise, and does the load-resolve-validate elsewhere (`calibration.py`, called
> from construction / `connect()`). That is a placement difference, not a disagreement: the
> validation still runs, still never drives, and still catches the J0 case. Decide which
> name owns it before either is depended on — LeRobot calls `calibrate()` on the default
> `connect()` path, so a raise there must be unreachable in normal use or it will crash
> `lerobot-record`.


### 5.2 `joint-limits.csv` → dict → LeRobot → wire

| CSV column | Dict field | LeRobot surface | Wire | Notes |
|---|---|---|---|---|
| `joint_id` | `JointCalibration.joint_id` | key of `calibration.joints` (`"j0"` …) and the stem of every feature key | `<j>` arg of every verb | id 2 is rejected by the loader and by the firmware (`E4`) |
| `joint_name` | `.name` | display and logs only | — | **may be wrong on J0 and J6** — the D3 dispute |
| `uno_pins` | `.pins` | provenance only | — | the firmware owns the real pin map |
| `min_deg` | `.min_deg` | lower bound of that joint's action range; clamp in `send_action` | `LIM <j> <min> <max> <cal>` arg 2 | an **accepted commanded soft limit** — an angle the operator drove to and chose, **not** a mechanical extreme |
| `max_deg` | `.max_deg` | upper bound | `LIM` arg 3 | legal 0–180, `min < max`, span ≥ `LIM_MIN_SPAN_DEG` = 5 |
| `home_deg` | **`.suggested_adopt_deg`** | **not a home pose** — the default prefilled into the operator's `ENA` adopt box | **never sent by `configure()`** | GUI-only. The console's single use site is the adopt-dialog prefill. **Editorial** — hand-moved off the locked value on four of five rows |
| `max_deg_per_sec` | `.max_deg_per_sec` | rate cap advertised to the caller | `SPD <j> <dps>` | 1–90 |
| `calibrated` | `.calibrated` | feeds `is_calibrated`; gates membership of the enable set | `LIM` arg 4 → `CAL=0`/`CAL=1` | set by the LOCK flow and nothing else; the firmware sets it explicitly and never infers it |
| `mirror_mode` *(J1 row only)* | `ArmCalibration.mirror_mode` | gates J1 in the enable set | `MIR SAME` / **`MIR INV`** / `MIR UNKNOWN` | `inverted` → **`INV`**, not `INVERTED`. `unknown` ⇒ `ENA 1` returns `E13` and J1 can never be driven |
| `mirror_offset_deg` *(J1 row only)* | `.mirror_offset_deg` | validation only | trailing arg of `MIR INV <off>`; omitted when 0 | ±90, and envelope-checked — §5.6 |
| `notes` | `.notes` | provenance string, surfaced in warnings | — | free text, no comma |
| **(proposed, absent today)** `mirror_offset_source` | `.mirror_offset_source` | `mirror_offset_measured` | — | `measured` / `placeholder`; **absent ⇒ `placeholder`** — §5.8 |

Renaming `home_deg` to `suggested_adopt_deg` happens **in the dict only**. The CSV column
name must not change: it is inside `LIM_COLS.slice(0,9)`, the console loader's required-column
check, and renaming it breaks the shipped GUI. The dict rename exists so that nobody later
writes a `go_home()` that drives a loaded arm to a number no one ever observed.

### 5.3 Lock artifacts — headerless, 11 columns

The schema is fixed by `downloadCalibrationRow()` (`arm-console.html:1405-1417`). There is
no header row; positions are the contract.

| Col | Content | Dict field | Validator |
|---|---|---|---|
| 0 | `YYYY-MM-DD HH:MM` local | `.lock_local_time` | newest per joint wins; older ones counted in `.superseded_locks` |
| 1 | joint id | join key | must match the row |
| 2 | joint name | — | cross-check only |
| 3 | uno pins | — | cross-check only |
| 4 | min as locked | `.lock_min_deg` | **H5 — must equal `.min_deg` or raise** |
| 5 | max as locked | `.lock_max_deg` | **H5 — must equal `.max_deg` or raise** |
| 6 | home as locked | `.lock_home_deg` | **compared and reported, never raised on** |
| 7 | the literal `OK LIM …` reply | `.lock_reply` | the board's own acknowledgement — the only wire-level evidence that exists |
| 8 | firmware version | `.lock_fw` | `1.0.0` on all eight |
| 9 | console version | `.lock_console` | `1.2.0` on all eight |
| 10 | the constant disclaimer string | discarded | part of the schema, not decoration |

**Home must be excluded from H5.** It legitimately differs on four of five rows by
deliberate hand-edit — J1 0→1, J3 0→33, J4 180→90, J5 178→104; only J0 matches at 64. A
validator that raised on home divergence would be dead on arrival against today's file.

### 5.4 `is_calibrated` — and why it is False today

```python
@property
def is_calibrated(self) -> bool:
    c = self.calibration
    if c is None:
        return False
    for jid in self.config.joints:
        j = c.joints[f"j{jid}"]
        if not j.calibrated:
            return False
        if "full_electrical_range" in j.flags:
            return False                 # locked at exactly the placeholder width
        if jid == 1 and c.mirror_mode == "unknown":
            return False                 # ENA 1 would return E13
    if self.is_connected and not self._board_matches_file:
        return False                     # a DTR reset silently restores 70-110 / CAL=0
    return True
```

**Over the default six joints: False.** Two causes, both real:

- **J6 is `calibrated=no`.** The gripper has never been bench-tested and its 0–180 is the
  servo's full electrical range.
- **J4 is locked at exactly 0–180**, which is the placeholder width. A joint that locks at
  precisely the servo's whole electrical range has most likely not been driven to a real
  mechanical stop at either end — `joint-limits.csv` says so in its own notes column.

Over `joints=(0,1,3,5)` it is **True**, with warnings. That J4 has to be excluded to reach
True is the correct outcome, not a bug in the rule.

> **Divergence from the concurrent implementation.** `emre_arm.py:118` counts **J6 only** —
> "exactly one joint is outstanding". This section's rule is the superset and additionally
> fails J4 on `full_electrical_range`. Both return False today, so nothing observable
> differs yet; they diverge the moment somebody bench-tests the gripper and sets
> `calibrated=yes` without re-locking J4. Reconcile before that happens.

False is **not a blocker**. A caller should read it as *"recording is fine; J4 and J6 must
not be enabled."* It is also not permanently red: two joints are outstanding, each with a
documented procedure to clear it, so the flag stays meaningful rather than becoming noise an
operator learns to ignore — which is the failure `joint-limits.csv` explicitly warns about.

The feature dicts are **not** connection-aware; they are built in `__init__` from
`config.joints` and the CSV, because LeRobot requires them callable while disconnected.
`is_calibrated` **is** connection-aware, because a DTR reset silently restores 70–110 and
`CAL=0` and a stale `True` would be a lie.

### 5.5 The validators

**Hard — raise, refuse to connect.**

| # | Condition | Why |
|---|---|---|
| H1 | CSV missing, unparseable, or a required column absent | matches the console loader's own required set |
| H2 | a row with `joint_id == 2` | reserved, D5, never commandable |
| H3 | `min<0`, `max>180`, `min>=max`, `max-min < 5`, `home` outside `[min,max]`, `dps` outside 1–90 | mirrors the firmware — refuse host-side rather than eat an `E10`/`E12` mid-push |
| **H4** | **`calibrated=yes` and no lock artifact for that joint** | **the J0-regression detector.** On 2026-08-05 the CSV claimed a range no artifact backed |
| **H5** | **CSV `min`/`max` ≠ newest artifact `min`/`max`** | the other half of the same detector. `home` is compared and reported but never raises |
| H6 | J1 in the enable set and `mirror_mode == unknown` | `ENA 1` would return `E13` and J1 could never be driven |
| H7 | `mirror_mode == inverted` and J1's `[min,max]` mirror image escapes 0–180 | mirrors `doMir` (`:747-760`) and the console's `mirrorImageOk` |
| H8 | after the push, `STA` `MIN`/`MAX`/`CAL`/`DPS` ≠ file, or `MIR` word ≠ file, or `UNCAL` ≠ expected | a reset the host did not notice looks exactly like a working connection until a joint refuses to move |
| H9 | `configure()` called while any joint is enabled | stricter than the firmware, on purpose — see §7 |

H4 has known friction with no owner: a fresh lock lands in the browser's `Downloads/`
folder, so H4 fails until somebody copies it into `lock-artifacts/`. **That forcing function
is the point** — the gap it closes is exactly how J0 was lost — but it is manual and nobody
has been assigned it.

**Soft — warn once at connect, record on `flags`.**

`uncalibrated` (J6) · `full_electrical_range` (J4, J6) · `min_at_electrical_floor`
(J1, J3, J4, J6) · `max_at_electrical_ceiling` (J4, J5, J6) · `home_edited` (J1, J3, J4, J5)
· `superseded_lock` (J1 ×2, J3 ×1) · `type_inferred` (all but J5) · `over_spec_supply`
(J4, J5, J6 — MG90S on 6.62 V) · `identity_disputed` (J0, J6) ·
`over_spec_supply_unresolved` (J0) · `mirror_offset_unmeasured` (J1).

Most of those are **derived from a numeric rule** so they cannot go stale.
`full_electrical_range` is `min == 0 and max == 180`. `min_at_electrical_floor` is
`min <= 2`. `max_at_electrical_ceiling` is `max >= 178`. `home_edited` is
`suggested_adopt_deg != lock_home_deg`. Only `identity_disputed` and
`mirror_offset_unmeasured` come from a small hand-maintained table, because no column can
express either fact today.

### 5.6 `connect()` and `configure()` — the ordering that matters

Opening the port asserts DTR and **resets the board**. The firmware keeps nothing: every
joint returns to detached, `CAL=0`, 70–110, `MIR=UNKNOWN`, `WDG=0`. **The entire calibration
is re-pushed on every single connect.** This is not an optimisation to skip.

```
connect(calibrate=True):
  1. open(port)                    # DTR asserted -> board resets
  2. sleep 2000 ms                 # Optiboot ~1 s + banner
  3. flush RX                      # discard the ';' banner and RDY
  4. VER -> assert NAME=FACTORYLM-ARM, record FW
  5. calibrate()                   # PURE FILE WORK. Sends nothing. H1-H7.
  6. configure()                   # H9, then the push, then H8
  -> every joint that has no adopt angle stays DISABLED.

configure():
  H9. refuse if ANY joint is enabled
  a. WDG 4000
  b. for each joint in file order:  LIM <j> <min> <max> <cal>  then  SPD <j> <dps>
  c. MIR <SAME|INV|UNKNOWN> [off]
  d. STA -> H8 readback
  e. for each joint with an adopt angle, subject to the gates:  ENA <j> <adopt>

disconnect():
  STP ; DIS A ; close cameras ; close port.   Never EST -- it latches.

on desync:
  flush ; VER ; WDG ; STA.
  NEVER reopen the port (that resets the board and the arm sags) and NEVER
  re-push LIM/SPD/MIR (joints are usually enabled during a resync).
```

> **Divergence from the concurrent implementation.** `emre_arm.py` splits this into
> `_push_state()` (`:216`) and `_verify_pushed_state()` (`:273`), with `configure()`
> (`:310`) owning only the `ENA` step. Same sequence, same guarantees, finer methods — the
> pseudocode above is the contract, not the call graph.

**Why `WDG` first.** The board boots with the watchdog disabled, and the `LIM`/`SPD` loop is
about twelve round trips. The console sends `WDG` first for exactly this reason
(`arm-console.html:1419-1425`). §7 records that `SERIAL-PROTOCOL.md` §9 says the opposite,
and that the disagreement is benign.

**Why `MIR` must come after `LIM` for joint 1 — with numbers.** `doMir` validates an `INV`
offset against joint 1's envelope **as the firmware currently holds it** (`:747-760`). The
legal integer-offset window is therefore different before and after the limits push:

| J1 envelope | Legal `MIR INV` offsets |
|---|---|
| boot default 70–110 | **[−35, +35]** |
| the locked 0–91 | **[−44, 0]** |

So a future measured offset of −44 is legal against the real envelope and **refused** against
the boot envelope. This is why the ordering is load-bearing rather than stylistic.

Note also what that second row says: at the current locked range, **any positive offset is
refused.** The mirror image of 0–91 at offset 0 is 89–180, which touches the ceiling with
zero margin. Measuring a positive offset would force J1's range to narrow first. The CSV
header warns about this from the data side; this is the same fact from the firmware side.

**Deviation from the protocol, declared:** `SERIAL-PROTOCOL.md` §10 specifies a fallback to
70–110 with `CAL=0` when the limits file is missing. That fallback assumes a GUI with an
amber UNCALIBRATED badge. **A headless adapter has nowhere to show amber**, and a silent
70–110 would clamp every command into a 40° window and present as a hardware fault. So H1
**raises** instead.

### 5.7 What the adopt angle means for unattended recording

`ENA <j> <adopt_deg>` takes a human's by-eye estimate. Nothing can supply it. Two
consequences follow directly and they are the most important operational finding in this
document:

1. **`connect()` cannot enable anything on its own.** With `adopt_deg = None` the handshake
   completes, the state is pushed, and every joint stays detached. `send_action()` then
   raises, naming exactly which joints lack an adopt angle.
2. **After any detach — watchdog trip, e-stop, or the adapter's own idle park — the arm
   sags, and the next adopt angle must be freshly estimated by a human.**

Therefore: **multi-episode `lerobot-record` is not safely automatable on this arm until a
marker observer exists.** The observer is not a nice-to-have on top of a working adapter; it
is the only possible source of a trustworthy adopt angle after a detach. That is the
structural link between §5 and §6, and it is the reason §6 is part of this spec rather than
a follow-up.

The adopted angles are also the only record of what a human believed at episode start, so
they belong in the dataset metadata, not just a log line.

### 5.8 One proposed CSV column

The CSV physically cannot express *"this mirror offset was measured"*. Deriving it from
`offset == 0` is wrong, because a real measurement could legitimately come out 0.

Add one optional column, `mirror_offset_source`, values `measured` | `placeholder`, **absent
⇒ `placeholder`**. This is safe by construction: `SERIAL-PROTOCOL.md` §10 states that
unknown columns are ignored and column order does not matter, and the console loader
accesses fields by name.

Do **not** rename any existing column. §5.2 explains why `home_deg` in particular is
load-bearing in the shipped GUI.

### 5.9 The four named hazards, resolved

| Hazard | Resolution |
|---|---|
| **Uncalibrated gripper (J6)** | Loaded, observable, `calibrated=False`, **excluded from the enable set** — so it physically cannot move (`MOV` on a disabled joint is `E6`). `send_action` for j6 is refused unless `allow_uncalibrated_joints=True`. Not hidden: hiding it would erase the only warning that exists. |
| **J4's suspect 0–180** | `calibrated=True` is kept because it is *honest about what happened* — a lock really did occur and the board really did reply `CAL=1`. The derived `full_electrical_range` flag carries the suspicion, and it is what makes `is_calibrated` False. |
| **Unmeasured mirror offset (J1)** | `mirror_offset_source = "placeholder"`. Never inferred from `offset == 0`. J1 stays drivable at offset 0 only when `allow_unmeasured_mirror=True`, and the warning prints the legal window `[−44, 0]` so nobody measures +6 and is surprised by an `E11`. |
| **Disputed D3 identity** | Everything keys on **pin and id**, never on name — which the firmware already does — so the limits hold either way. `identity_disputed` is flagged on **J0 and J6**, and because the dispute leaves `servo_type` unresolved it also leaves voltage headroom unresolved: J0 gets `over_spec_supply_unresolved` rather than a confident absence. If D3 really carries the gripper's MG90S, J0 is being run over spec on 6.62 V. |

---

## 6. Marker-observer integration plan — the interface

This section specifies **the software interface only**. The marker design — dictionary,
sizes, placement, camera plan, the two calibration passes — is
`Documentation/MARKER-SYSTEM.md` and `Software/vision/markers.csv`, both committed at
`ae598cb`. This document does not restate them and must not contradict them.

### 6.1 The negative finding comes first

**A single 1280×720 camera at the standoff needed to frame the arm resolves only the two
BASE markers at pose grade and the two TURRET markers at detect grade. Shoulder, forearm,
wrist and gripper are below the detection floor — not imprecise, not seen.**

That is not a caveat, it is the operating reality. It means that **on the hardware that
exists today, `residual_deg` is NaN for J1, J3, J4, J5 and J6 even with an observer running.**
Any reading of this section as "markers will give you residuals" is wrong.

`MARKER-SYSTEM.md` §8 gives two honest ways past it — a second close camera, or three
printed 40 × 40 × 2 mm flat marker tabs — and §7 gives the camera plan that follows from
choosing the first. The adapter is neutral between them: it consumes whatever fixes the
observer produces and reports `OBS_SOURCE_NONE` or `OBS_SOURCE_OCCLUDED` for everything
else. Neither option is implemented.

### 6.2 The interface

```python
# lerobot_robot_emre_arm/markers.py
from typing import Any, Mapping, Protocol

from .observation import JointFix


class MarkerObserver(Protocol):
    """Turns camera frames into per-joint observations."""

    def observe(
        self,
        frames: Mapping[str, Any],
        capture_mono: Mapping[str, float],
    ) -> dict[int, JointFix]:
        """Return {joint_id: JointFix}. Omit a joint only if it was not looked
        at; use OBS_SOURCE_OCCLUDED if it was looked at and not found."""
        ...


class NullMarkerObserver:
    """The day-one implementation, and the honest default.

    Returns nothing at all, so every observed / residual / age column is NaN and
    every obs_source is OBS_SOURCE_NONE. This is not a stub to be replaced by
    something that guesses -- it is the correct behaviour when nothing observes
    the arm, which is the current state of this project.
    """

    def observe(
        self,
        frames: Mapping[str, Any],
        capture_mono: Mapping[str, float],
    ) -> dict[int, JointFix]:
        return {}
```

`JointFix` already exists in `observation.py` (`:241-271`) and is quoted in §3.2. The
observer side of the contract adds nothing to it.

**Frames are typed `Mapping[str, Any]`, not `dict[str, np.ndarray]`, deliberately.** It
keeps `markers.py` importable without numpy, so the interface can be asserted on a machine
with no vision stack — the same reasoning that keeps `observation.py` a leaf. The concrete
array type is the detection slice's business.

### 6.3 The nine hard constraints

An implementation that breaks any of these is wrong, not merely suboptimal.

1. **It consumes frames it is given.** It must not open its own `cv2.VideoCapture`. The
   camera is already open under LeRobot, a camera cannot be opened twice, and a second
   capture would not be time-aligned with the frame actually recorded into the dataset.
2. **It takes capture time, not call time.** `capture_mono` gives `time.monotonic()` at
   which each frame was *grabbed*. LeRobot camera reads come from a background thread and
   return the most recent frame, so call time can lag capture by a whole frame interval and
   would silently under-report staleness.
3. **Keyed by logical joint id**, never by part name or marker id. Mapping marker → link →
   joint is the observer's job. `Software/vision/markers.csv` is keyed by `link_name` and
   `target_part_file`; that translation lives on the observer's side of this boundary.
4. **It never returns joint id 2.** D5 is not an addressable joint. An observer that can see
   the second shoulder horn reports it as `JointFix.pair_disagree_deg` on joint 1 — see
   constraint 7.
5. **It distinguishes "could not see" from "did not look".** A joint the observer tried and
   failed to resolve returns `source=OBS_SOURCE_OCCLUDED`. It must **not** be omitted;
   omission means "never looked" and the mask has to be able to tell them apart.
6. **Angles are in the commanded-degree convention.** `JointFix.angle_deg` must be in the
   same degree space the firmware is commanded in, otherwise `residual_deg` measures the
   observer's calibration error rather than the arm's tracking error. **That mapping is
   unmeasured today; until it is calibrated an observer must not claim
   `OBS_SOURCE_MARKER`.**
7. **Joint 1 pair disagreement is optional and reported on the j1 fix.** An observer that can
   see both shoulder horns sets `pair_disagree_deg` to the signed difference between the two
   horns' implied angles. This is the only mechanism this project has ever had for measuring
   `mirror_offset_deg`.
8. **Time budget ≤ 10 ms.** A full `STA` poll already costs about 41 ms of wire time (§2.4).
   An observer that blocks longer sets the frame rate, not the arm.
9. **No serial access.** It must not know the port exists. One transmitting client at a time
   is absolute — §4 item 13.

### 6.4 Where the datum comes from

A marker table on its own does **not** satisfy Option A: `residual = observed − commanded`
has nothing to subtract from without a reference. `MARKER-SYSTEM.md` §6 specifies two
required passes, both prerequisites for any observer claiming `OBS_SOURCE_MARKER`:

- **Pass 1 — axis identification.** Command each joint alone across its calibrated range in
  ~5° steps; capture the child-link marker pose relative to the parent-link marker; fit a
  rotation axis by least squares. This recovers the joint axes **and** the parent→child
  transforms empirically — the correct answer to "the STLs give no link geometry" is that
  you do not read it off a part, you observe it with the marker system being designed. It
  also settles the D3 dispute in one shot: command J0 and watch which link moves.
- **Pass 2 — datum capture.** Drive every joint to its recorded `home_deg`, capture the
  relative marker geometry, and store it. Thereafter
  `observed_angle = home_deg + Δ(about the fitted axis)`.

**The datum is captured at a *commanded* home and is not a kinematic zero.** `home_deg` is
explicitly editorial — several values were hand-moved off a hard end of travel — so every
observed angle carries a constant bias. The residual still detects *change* (sag, slip,
stall) without that bias being zero, and that is what it is for.

Pass 1 requires driving an assembled arm, so it is **blocked on the 3–5 A supply** (§0).

### 6.5 What lands in the dataset when an observer exists

Nothing in the schema changes. That is the point of declaring the columns on day one. The
only difference is that `observed_deg`, `residual_deg` and `obs_age_s` stop being NaN for
whichever joints the observer can actually see, and `obs_source` moves off `0.0` for those
joints. A dataset recorded before the observer and one recorded after remain concatenable.

### 6.6 Explicitly not specified here

Marker detection, pose estimation, camera intrinsics, the marker→degree fit, the frame
pipeline, and the choice between `MARKER-SYSTEM.md` §8's two options. `Software/vision/` is
another workstream's; this document defines only the boundary it hands across.

---

## 7. Conflicts, corrections and open questions

### 7.1 Where the shipped docs disagree with the shipped code

These are real and they are in the repository right now. Flagged, not fixed — fixing them is
a change to `SERIAL-PROTOCOL.md` and the firmware comments, not to this spec.

| # | The doc says | The code does | Consequence |
|---|---|---|---|
| 1 | `SERIAL-PROTOCOL.md` §3: `LIM` — "Joint must be **disabled**." | `doLimSet` refuses on `jogActive` (`E9 STATE=jogging`, `:633`), and on an *enabled* joint **only when the new range would exclude that joint's own `setC`** (`:665-669`). Otherwise it is accepted and the pending target is clamped inward (`:674-681`). The firmware's own header comment at `:614` is stale — the comment at `:664` says so. | "Sometimes accepted" is worse than "refused" for a host that is not watching. **The adapter adopts the console's stricter host-side policy (H9): refuse to push while any joint is enabled.** |
| 2 | `SERIAL-PROTOCOL.md` §4's worked `STA` reply and field table show nine per-joint fields, ending at `MOV=`. | `doSta` also emits **`JTO=`** (`:586`) — latched, and the only way to learn that a jog ended because the host went quiet, since the timeout itself is silent on the wire. | Any parser written from §4 drops a field. It also makes the reply about 40 bytes longer than §4 implies, which is why §2.4's byte count is computed from the firmware and not from the doc. |
| 3 | `SERIAL-PROTOCOL.md` §9 step 6: `LIM`/`SPD` per joint, then `MIR`, then `WDG 1000`. | `pushState()` sends **`WDG` first**, then the per-joint loop, then `MIR` (`arm-console.html:1419-1443`). | Benign — the watchdog only arms when `anyEnabled()` is true and nothing is enabled during a push, which the console's own comment concedes. Flagged so nobody "fixes" one to match the other. **The ordering that genuinely matters is `MIR` after `LIM` for joint 1** — §5.6. |
| 4 | §8 recommends `WDG 1000`. | The console pushes `WATCHDOG_MS = 4000`, because of browser timer throttling in a background tab. | That rationale does not apply to a Python host. The adapter may legitimately use a tighter value; just do not assume 4000 is the spec. |
| 5 | §11 specifies a waypoints CSV with **six** joint columns and no shoulder split, "because there is no way to command them separately". | `Software/conveyor-waypoints-template.csv` has `shoulder_left_deg` **and** `shoulder_right_deg`. §11 itself acknowledges it as the older format. | A live trap for any dataset exporter that greps for a waypoint schema. Use §11's format. |
| 6 | §10's parser rules specify a fallback to 70–110 / `CAL=0` when the limits file is absent. | The adapter **raises** instead. | Declared deviation, not an oversight — §5.6. The fallback assumes an amber badge a headless process cannot show. |

### 7.2 Corrections to the task brief this document was written from

- **15 verbs, not 13.** The brief's list omits **`PNG`** and `HLP`. `PNG` is load-bearing —
  it is the documented watchdog heartbeat, and an adapter built from the 13-verb list has no
  heartbeat and will be e-stopped mid-episode.
- **"Angles are centidegrees internally (0..18000)"** is true internally but reads as if the
  wire carries 0.01°. It does not: **every value on the wire is a whole degree, in both
  directions**. That is a hard floor on `.pos` and on any marker residual — §3.7.
- **"On host silence every joint detaches"** understates it: the watchdog also **latches a
  full e-stop**. Recovery needs `CLR` and then a fresh `ENA` per joint, and the fresh adopt
  angle is a guess about an arm that has sagged.
- **"LIM rejects max>180"** is one of six rejection conditions — see H3.

### 7.3 Where two design passes disagreed, and which one this document adopts

- **`calibrate()`** — one pass proposed raising `NotImplementedError` and pointing at the
  bench procedure; another proposed the pure load-resolve-validate in §5.1. **§5.1 wins:**
  it never drives *and* it is the only version that catches the 2026-08-05 J0 widening. The
  raise moved to `_save_calibration()`, where it belongs.
- **`is_calibrated`** — one pass said False because of J6 alone; §5.4 says J6 **and** J4's
  `full_electrical_range`. §5.4 is the superset and is adopted.
- **`.pos` source** — one pass sourced it from `STA … SET=`, another from the adapter's own
  command state. Neither is adopted; **§3.7 gap 2 leaves it open on purpose**, because the
  committed `build_observation()` is deliberately agnostic and this is a decision that
  should be made once, deliberately, before a dataset is published.
- **Throughput** — one pass computed 473 B / 41 ms / ~24 Hz, another 428 B / 37 ms / 27 Hz.
  The discrepancy is conflict #2 above: the smaller figure was computed from
  `SERIAL-PROTOCOL.md` §4, which omits `JTO=`. §2.4 uses the firmware-derived figure. Either
  way it is **calculated wire time and has never been measured.**

### 7.4 Open questions

1. **`.pos` = `tgtC`, `setC`, or the host's model of `setC`?** §3.7 gap 2. Must be settled
   before any published dataset.
2. **Should the committed schema gain a `j<N>.driven` field?** §3.7 gap 1. It is the only way
   to flag a `.pos` that no servo received. Adding it changes `observation.py`, the slice
   indices, and every already-recorded dataset's schema — so decide before recording, not
   after.
3. **Is `configure()` actually invoked by `connect()` in the installed LeRobot version?** If
   not, the enable step never runs and the first `send_action` gets `ERR E6`. Verify against
   the real base class and move the `ENA` into `connect()` if needed.
4. **What does `connect(calibrate=True)` do when `is_calibrated` is False?** Call
   `calibrate()` once, loop, or raise? This document assumes: call once, then `configure()`,
   then warn — not raise. If the installed base class raises, the fallback is to default
   `config.joints` to `(0, 1, 3, 5)`, which makes `is_calibrated` True honestly.
5. **Does the dataset packer really filter on `v is float`?** Check with
   `inspect.getsource` on `lerobot.datasets.utils.hw_to_dataset_features`. A different
   predicate would silently drop non-float features — which is precisely why every Option A
   field is declared float. The design is right either way; only §3.6's explanation changes.
6. **Does `OpenCVCameraConfig` select `cv2.CAP_DSHOW` on Windows automatically?** This
   machine requires it, and the built-in camera silently refuses 1080p (1280×720 works). If
   LeRobot does not set the backend per platform, the camera wiring needs an explicit
   override.
7. **Does pyserial apply `TIOCEXCL` by default on POSIX?** Do not assume. §4 item 3 requires
   passing `exclusive=True` explicitly; verify it on any Mac target before demoting the
   lockfile to cosmetic there. On Windows it is moot — the OS gives exclusivity free.
8. **Who copies a fresh lock artifact from `Downloads/` into `Calibration_Notes/lock-artifacts/`?**
   H4 fails until somebody does. Either leave it as a forcing function and document it in the
   LOCK flow, or give the bridge a write path — which would be its first, and a real change
   to a token-guarded loopback server's security surface.
9. **Does the assembly get taken apart to measure `mirror_offset_deg` before any session
   enables J1?** The measurement procedure requires the shoulder linkage unbolted and both
   horns off, so it cannot be done on the assembled arm. The adapter can enforce either
   answer; it cannot choose one.
10. **`STA` never echoes the mirror offset.** `doSta` prints `MIR=<word>` only (`:591`); the
    offset appears solely in the `OK MIR … OFF=<n>` reply. So H8 can verify the mirror
    **mode** but must capture the **offset** from the push reply and trust it thereafter.
    Adding `MOFF=` to the `SYS` line would close it — a firmware change, out of scope here.
11. **`disconnect()`: detach or hold?** `STP` then `DIS A` sags the arm immediately while the
    operator is present; `STP` alone lets the watchdog detach it about a second after the
    process exits, with nobody watching. §5.6 specifies the former on the reasoning that a
    supervised sag beats an unsupervised one. This is a judgement about the physical arm and
    should be confirmed on the bench.
12. **Centroid noise is assumed at 0.2 px** in every marker angle-resolution figure. Not
    measured on this camera. A static N-frame capture would settle it.

---

## 8. Licences — flagged, not passed silently

The stated rule for this project is **Apache-2.0 or MIT only**.

| Dependency | Licence | Status |
|---|---|---|
| `lerobot` | stated Apache-2.0 | **Verify from the installed package metadata** before adding the dependency. Not confirmed here. |
| `opencv-python` | Apache-2.0 | Compliant. Installed, 4.13.0.90. Use `CAP_DSHOW` on this host. |
| `pyserial` | **BSD-3-Clause** | **Exception needed.** Already unavoidable — `arm-bridge.py` and `protocol_check.py` both import it, and the adapter needs it directly. |
| `numpy` | **BSD-3-Clause** | **Exception needed.** Already unavoidable — required for the marker maths and by `cv2`. Note numpy 2.x leaves the dist-info `License` field empty; the licence was read from the licence text itself. |
| **this repository** | **none declared** | **Blocker.** `ls | grep -i licen` returns nothing. There is no `LICENSE` file anywhere. |

Both BSD-3-Clause dependencies are permissive and Apache-2.0-compatible in practice, but
neither is on the allow-list and the exception should be **written down** — an ADR or a
`LICENSES` note — rather than passed silently. The missing repository licence is the harder
one: it blocks publishing `lerobot_robot_emre_arm` at all, and it blocks any claim about what
this code may be combined with.

No LangChain, and no framework that abstracts a model call. The adapter is plain pyserial
plus LeRobot's `Robot` ABC.

---

## Files that go with this one

| File | What it is |
|---|---|
| `Documentation/SERIAL-PROTOCOL.md` | the wire spec this adapter speaks — read it first |
| `Documentation/MARKER-SYSTEM.md` | the marker design §6 defers to |
| `Documentation/specs/2026-08-04-envelope-joystick-design.md` | §3 vocabulary, §12 the pre-authorisation for Option A |
| `Software/lerobot_robot_emre_arm/src/lerobot_robot_emre_arm/observation.py` | **the Option A contract, committed at `d2d776a`** |
| `Software/arm-console/joint-limits.csv` | the calibration data of record — `calibrate()`'s input |
| `Calibration_Notes/lock-artifacts/*.csv` | the eight raw LOCK artifacts H4 and H5 check against |
| `Calibration_Notes/calibration-log.csv` | the narrative log, including the D3 dispute |
| `Software/wiring-map.csv` | the pin/index map the joint ids come from |
| `Software/vision/markers.csv` | the 13-marker set, committed at `ae598cb` |
| `Software/tests/protocol_check.py` | the real-board harness whose `Board` shape `transport.py` reuses |
| `Software/factorylm_arm_controller/factorylm_arm_controller.ino` | the firmware every line number in this document refers to |
