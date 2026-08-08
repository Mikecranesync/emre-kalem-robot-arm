"""`EmreArmConfig` -- the registered LeRobot config for `--robot.type=emre_arm`.

The registration decorator is what makes `lerobot-record --robot.type=emre_arm`
and `lerobot-teleoperate --robot.type=emre_arm` work with no change to LeRobot's
own source. It only takes effect when this module EXECUTES, which is why
`__init__.py` imports it eagerly rather than lazily. A lazy import to dodge the
`lerobot` dependency would silently break discovery.

    !!  THE lerobot PACKAGE IS NOT INSTALLED ON THE DEVELOPMENT HOST  !!
    So `RobotConfig`'s real field set has never been read, and neither has the
    `Robot` ABC that emre_arm.py subclasses. This module is written to the
    documented contract. Confirm it against the installed base class the first
    time `pip install lerobot` succeeds -- in particular whether `RobotConfig`
    already defines `id` / `calibration_dir` fields that would collide here.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig

from .calibration import DEFAULT_LIMITS_CSV, DEFAULT_LOCK_DIR, DEFAULT_WIRING_MAP
from .markers import MarkerObserver


@RobotConfig.register_subclass("emre_arm")
@dataclass
class EmreArmConfig(RobotConfig):
    """Configuration for the Emre Kalem 6-axis arm.

    THE ONE FIELD THAT MATTERS MOST IS `adopt_deg`, AND IT HAS NO DEFAULT.
    `ENA <j> <adopt_deg>` asserts where a joint's shaft physically is right now.
    Nothing in this system can produce that number -- there is no feedback of any
    kind -- so it is a human's by-eye estimate, and the firmware pre-loads that
    pulse BEFORE attaching the servo precisely because a wrong one snaps the
    joint. Leaving it None means `connect()` completes the handshake and pushes
    the envelope but ENABLES NOTHING, which is the safe default.

    `home_deg` from joint-limits.csv is deliberately NOT used as a fallback. It
    is editorial: several rows were hand-moved off the locked value because a
    home sitting on a hard end of travel leaves nowhere to back off to. It is a
    suggestion about where a joint might be, not an observation of where it is.
    """

    # --- link -------------------------------------------------------------
    #: None means auto-pick, and refuse to guess when two candidates tie.
    port: str | None = None

    # --- calibration sources (read-only; see calibration.py) --------------
    limits_csv: Path = DEFAULT_LIMITS_CSV
    lock_artifacts_dir: Path = DEFAULT_LOCK_DIR
    wiring_map_csv: Path = DEFAULT_WIRING_MAP

    # --- the human step ---------------------------------------------------
    #: {joint_id: by-eye estimate of where that joint is sitting RIGHT NOW}.
    #: Joints absent from this dict are never enabled and report NaN actions.
    adopt_deg: dict[int, float] | None = None

    # --- safety gates. Both triggers are live right now. -------------------
    #
    # Not speculative flexibility: joint-limits.csv sets mirror_mode=inverted, so
    # the firmware's own E13 guard does NOT fire on `ENA 1`, while
    # mirror_offset_deg is still the unmeasured placeholder 0. The firmware guard
    # is satisfied and the hazard -- two MG996Rs fighting each other -- is fully
    # present. And joint 6 is calibrated=no with the servo's whole electrical
    # range as its limits.
    allow_unmeasured_mirror: bool = False
    allow_uncalibrated_joints: bool = False

    # --- watchdog and timing ----------------------------------------------
    #
    # WDG 1000 is the protocol doc's recommendation. The arm console uses 4000
    # instead, for a reason that does NOT apply here: browsers throttle timers in
    # background tabs. A Python heartbeat on its own thread has no such problem,
    # so the tighter value is both safe and better -- 250 ms of heartbeat gives a
    # 4x margin. Never set this to 0; that disables the only protection against a
    # dead host.
    watchdog_ms: int = 1000
    heartbeat_s: float = 0.25

    #: No get_observation()/send_action() for this long -> `DIS A`. This catches
    #: a WEDGED control loop in a still-live process, which the firmware watchdog
    #: cannot see because the heartbeat thread would keep feeding it.
    #:
    #: 2.0 was WRONG and shipped briefly. lerobot-record's flow is connect() ->
    #: configure() -> a startup/reset phase -> only then the record loop, and the
    #: gaps between those routinely exceed two seconds. The watcher fired during
    #: ordinary startup and sent `DIS A`, which de-energises a gravity-loaded arm
    #: -- a self-inflicted sag dressed up as a safety feature. 15 s is longer than
    #: every known non-loop gap and still catches a genuinely wedged loop. The
    #: watcher also refuses to act unless a joint is actually enabled; see
    #: SerialLink._idle_loop.
    idle_disable_s: float = 15.0

    #: Fixes older than this keep their value and age but lose their residual and
    #: are demoted to OBS_SOURCE_STALE.
    max_fix_age_s: float = 0.25

    # --- cameras and observation ------------------------------------------
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    #: None means NullMarkerObserver: every observed_* column is NaN. That is the
    #: honest day-one state, not a placeholder to be filled with a guess.
    marker_observer: MarkerObserver | None = None
