"""The Option A observation contract for the Emre Kalem arm.

This module is the single source of truth for what `EmreArm.observation_features`,
`EmreArm.action_features` and `EmreArm.get_observation()` contain. It is pure
Python with no I/O and no third-party imports, so it can be imported and asserted
against while the board is unplugged -- which is exactly what LeRobot requires of
`observation_features` / `action_features`.

WHY "OPTION A" EXISTS AT ALL
    These servos have NO POSITION FEEDBACK OF ANY KIND. The firmware knows only
    what it last commanded; `Servo.read()` returns the last commanded value, not a
    shaft angle (Documentation/SERIAL-PROTOCOL.md section 15). A schema with a
    single ambiguous "position" field would therefore be a lie in every row.

    So every joint carries SIX fields, and the commanded channel is never fused
    with the observed one:

        j<N>.pos            what the firmware was told to hold      (always known)
        j<N>.observed_deg   what a camera or a human actually saw   (usually unknown)
        j<N>.residual_deg   observed - commanded                    (the useful signal)
        j<N>.obs_source     WHERE the observation came from         (always finite)
        j<N>.obs_age_s      how stale that observation is           (seconds)
        j<N>.clamped        1.0 if the firmware clamped the command

    `obs_source` is the mask. It is the only field guaranteed never to be NaN, so
    a consumer always has a finite answer to "may I trust this row?".

VOCABULARY -- ONE DELIBERATE EXCEPTION, STATED OUT LOUD
    The project vocabulary rule bans the words position / actual / measured /
    feedback for shaft state; the approved exception is a value a human or a
    camera really observed, which must be NAMED as observed. This module honours
    that -- `observed_deg` is the only field claiming observation, and it is NaN
    until something actually observes.

    `.pos` is the one violation, and it is knowing. `.pos` is LeRobot's fixed
    ecosystem spelling for the command channel (the SO-10x robots use it, and
    `lerobot-teleoperate` and stock policy configs key off it). Renaming it to
    `.cmd_deg` would buy vocabulary purity and cost drop-in compatibility with
    every downstream LeRobot tool. We keep `.pos` and pay for it in documentation:
    `.pos` here means COMMANDED, always, and never a shaft angle.

WHY KEYS ARE `j0`, NOT `base`
    Joint ids are the indices from Software/wiring-map.csv, the project's single
    numbering system (SERIAL-PROTOCOL.md section 1 keeps it that way on purpose).
    Names are worse than useless here: Calibration_Notes/calibration-log.csv
    (2026-08-01) records that the servo physically wired to D3 was the GRIPPER, not
    the Base, and that dispute is still open. Limits follow the PIN, so an id-keyed
    dataset stays true whichever way the dispute resolves; a `base.pos` column would
    bake a possible mislabel into every recorded episode permanently.

    Human-readable names live in JOINT_LABELS for logs and never appear in a key.

WHY THERE IS NO j2
    Joint id 2 is the second servo of the shoulder pair (D5). It is not addressable
    by construction: joint 1 drives D4+D5 as ONE logical joint, mirrored at the
    single point of write. There is no j2 key, not even a NaN placeholder -- the
    visible gap in `j0 j1 j3 j4 j5 j6` is the documentation, and it matches both the
    STA dump (which never emits a J2 line) and the waypoints CSV (one column per
    logical joint).

HOW THIS LANDS IN A LeRobot DATASET -- THE SURPRISING PART
    No field below gets its own dataset column. LeRobot packs every feature whose
    declared type is `float` into ONE flat array:

        observation.state  float32 (37,)  names = OBSERVATION_STATE_NAMES
        action             float32 (6,)   names = ACTION_NAMES
        observation.images.<cam>          dtype video/image, shape (h, w, c)

    Ordering is FIELD-MAJOR so the blocks are contiguous and sliceable:

        state[ 0: 6]  commanded          state[18:24]  obs_source
        state[ 6:12]  observed_deg       state[24:30]  obs_age_s
        state[12:18]  residual_deg       state[30:36]  clamped
        state[36]     j1.pair_disagree_deg

    Derive indices from `names` at load time. NEVER hardcode them -- adding a
    joint or a field silently shifts every downstream slice.

WHY AGE IN SECONDS AND NOT A TIMESTAMP
    `observation.state` is float32. An absolute epoch timestamp cannot survive it:
    float32 quantises to 128.0 SECONDS at 1.75e9 (verified with numpy -- the next
    representable value after 1750000000.0 is 1750000128.0). Every fix inside the
    same two-minute window would collapse to one indistinguishable number.

    The same float32 holds an AGE of 0.031 s to 1.9e-09 s. So we record staleness,
    not wall-clock -- and staleness is the non-redundant quantity anyway, because
    LeRobotDataset already stamps every frame with its own `timestamp` column.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping

# --------------------------------------------------------------------------
# Joints
# --------------------------------------------------------------------------

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

# --------------------------------------------------------------------------
# Observation source enum
#
# Encoded as float because that is the only dtype LeRobot packs into
# observation.state (see the module docstring). It is CATEGORICAL despite the
# numeric type: use it as a mask (`src == OBS_SOURCE_MARKER`), never as a
# continuous regression input. If a policy genuinely needs it as an input,
# one-hot it at training time from `names` -- do not feed the raw ordinal.
# --------------------------------------------------------------------------

OBS_SOURCE_NONE: float = 0.0      # no observer configured, or never a fix for this joint
OBS_SOURCE_STALE: float = 1.0     # a fix exists but is older than max_fix_age_s
OBS_SOURCE_OCCLUDED: float = 2.0  # observer ran this frame and could not see the marker
OBS_SOURCE_MARKER: float = 3.0    # a camera observed it this frame
OBS_SOURCE_HUMAN: float = 4.0     # a human read it off the arm and typed it in

OBS_SOURCE_LABELS: dict[float, str] = {
    OBS_SOURCE_NONE: "none",
    OBS_SOURCE_STALE: "stale",
    OBS_SOURCE_OCCLUDED: "occluded",
    OBS_SOURCE_MARKER: "marker",
    OBS_SOURCE_HUMAN: "human",
}

# --------------------------------------------------------------------------
# Field layout
# --------------------------------------------------------------------------

#: Per-joint observation fields, in dataset order. FIELD-MAJOR: all six joints'
#: `pos` come first, then all six `observed_deg`, and so on. That keeps each
#: block contiguous, so `state[:6]` is the commanded vector on day one.
OBSERVATION_FIELDS: tuple[str, ...] = (
    "pos",
    "observed_deg",
    "residual_deg",
    "obs_source",
    "obs_age_s",
    "clamped",
)

#: Tail singleton. Only joint 1 has it, so it sits AFTER every six-wide block
#: rather than making one block ragged.
#:
#: Joint 1's two servos are commanded from one logical angle mirrored about
#: `90 + mirror_offset_deg`. That offset has NEVER been measured on this arm and
#: sits at the placeholder 0 (Software/arm-console/joint-limits.csv). If the true
#: axis is not 90, the two MG996Rs fight each other by twice the error the whole
#: time joint 1 is driven -- they hold, run hot, and eventually strip a gear, and
#: nothing on screen shows it.
#:
#: An observer that can see BOTH shoulder horns is the only mechanism this project
#: has ever had for measuring that offset. This column is where it reports. It is
#: NaN today and will stay NaN until a two-marker observer exists -- but it is
#: declared now so the schema does not churn later.
PAIR_DISAGREE_KEY: str = "j1.pair_disagree_deg"


def joint_key(joint_id: int, field: str) -> str:
    """Build one dataset key. The ONLY place a key string is assembled."""
    if joint_id == RESERVED_JOINT_ID:
        raise ValueError(
            "joint id 2 is the shoulder pair's second servo (D5) and is not "
            "addressable by construction -- it has no dataset key"
        )
    if joint_id not in JOINT_IDS:
        raise ValueError("unknown joint id %r; addressable ids are %r" % (joint_id, JOINT_IDS))
    return "j%d.%s" % (joint_id, field)


# --------------------------------------------------------------------------
# The LeRobot feature dicts
#
# Both are pure functions of configuration, never of board state, so they are
# callable while disconnected -- which LeRobot requires.
# --------------------------------------------------------------------------

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
    """What `get_observation()` returns.

    `cameras` maps camera name -> (height, width, channels). Camera entries are
    declared as tuples, which is how LeRobot tells an image feature from a scalar.

    The schema is STABLE: the observed/residual/source/age columns are declared
    whether or not an observer is configured. Dropping them when no observer
    exists would change the feature schema between runs, which breaks
    `lerobot-record --resume` and makes two datasets impossible to concatenate.
    On day one they are simply all NaN with `obs_source = OBS_SOURCE_NONE`.
    """
    features: dict[str, type | tuple[int, int, int]] = {}
    for field in OBSERVATION_FIELDS:          # field-major -- see module docstring
        for joint_id in JOINT_IDS:
            features[joint_key(joint_id, field)] = float
    features[PAIR_DISAGREE_KEY] = float
    for name, shape in (cameras or {}).items():
        features[name] = shape
    return features


#: The exact `names` list LeRobot will attach to `observation.state`, in order.
#: Consumers should read this from the dataset metadata rather than importing it,
#: but it is exported so tests can assert the two agree.
OBSERVATION_STATE_NAMES: tuple[str, ...] = tuple(
    k for k, v in observation_features().items() if v is float
)

#: Likewise for `action`.
ACTION_NAMES: tuple[str, ...] = tuple(action_features())


# --------------------------------------------------------------------------
# The observer's half of the contract
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class JointFix:
    """One joint observed once, by a camera or a human.

    This is the ONLY thing a marker pipeline hands back. It deliberately knows
    nothing about serial ports, commanded angles, or LeRobot.

    angle_deg
        The observed joint angle, in the SAME degree convention the firmware is
        commanded in. Converting marker pose -> commanded-degree scale is the
        observer's job and is itself an unmeasured mapping today; until that
        mapping is calibrated an observer must NOT claim OBS_SOURCE_MARKER.
    source
        One of the OBS_SOURCE_* constants. An observer that ran and could not see
        the marker returns OBS_SOURCE_OCCLUDED -- it must not simply omit the
        joint, because "I looked and could not see it" and "I never looked" are
        different facts and the mask must be able to tell them apart.
    captured_mono
        `time.monotonic()` at FRAME CAPTURE -- not when `observe()` was called.
        LeRobot camera reads come from a background thread and return the most
        recent frame, so call time can lag capture time by a whole frame interval.
        Using call time would silently under-report staleness.
    pair_disagree_deg
        Joint 1 only, and only from an observer that can see both shoulder horns.
        None everywhere else.
    """

    angle_deg: float
    source: float
    captured_mono: float
    pair_disagree_deg: float | None = None


# --------------------------------------------------------------------------
# Assembling one observation
# --------------------------------------------------------------------------

def build_observation(
    commanded: Mapping[int, float],
    clamped: Mapping[int, bool],
    fixes: Mapping[int, JointFix] | None,
    now_mono: float,
    max_fix_age_s: float = 0.25,
) -> dict[str, float]:
    """Assemble the scalar half of one `get_observation()` return value.

    Camera frames are merged in by the caller; this function owns only the floats.

    THE UNOBSERVED CASE IS THE NORMAL CASE ON DAY ONE. With no observer
    configured, every `observed_deg` / `residual_deg` / `obs_age_s` is NaN and
    every `obs_source` is OBS_SOURCE_NONE.

    NaN is deliberate, and the alternatives were rejected:
      - `observed = commanded` asserts the arm is exactly where it was told to be,
        which is precisely the fiction this whole schema exists to destroy.
      - `observed = 0.0` claims the joint was observed at zero degrees.
    Both are silently wrong. NaN is loudly wrong, and it is paired with an
    always-finite `obs_source` so consumers have a clean mask.

    KNOWN CONSEQUENCE, STATED NOT HIDDEN: a column that is entirely NaN makes
    LeRobot's dataset mean/std for that column NaN, and normalising against it
    yields NaN activations. That is intended -- it fails loudly and immediately.
    A day-one training config should select `observation.state[:6]` plus the
    camera keys and simply not consume the observation block until an observer
    exists.
    """
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
