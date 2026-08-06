"""`EmreArm` -- the LeRobot `Robot` implementation for the Emre Kalem arm.

    !!  UNVERIFIED AGAINST THE REAL BASE CLASS  !!
    `lerobot` is not installed on the development host, so the `Robot` ABC's
    actual signatures have never been read. Everything here is written to the
    documented contract. Two assumptions in particular must be confirmed the
    first time `pip install lerobot` succeeds, because both are load-bearing:

      A. `connect(calibrate: bool = True)` is the real signature.
      B. Something calls `configure()`. If nothing does, the `ENA` step never
         runs and the first `send_action()` gets ERR E6 (joint not enabled).
         Fix by calling `configure()` from `connect()`, which is what this file
         already does -- so A is the one to check hardest.

WHAT THIS ADAPTER REFUSES TO DO, AND WHY EACH REFUSAL IS DELIBERATE

    `calibrate()` raises.
        LeRobot's convention is that calibrate() produces a calibration. On this
        arm, producing one means driving a joint to find its travel, and driving
        requires an adopt angle that asserts where the shaft physically is --
        which nothing here can know. Worse, `connect(calibrate=True)` is the
        DEFAULT, so a driving calibrate() would move a gravity-loaded arm on
        every connect, unattended, with nobody's hand near the rocker switch.
        Calibration is an operator act at the arm console. See calibration.py.

    `_save_calibration()` raises.
        There is no writable LeRobot-side calibration file. A second writable
        copy of the limits is exactly how this project already lost joint 0's
        measured envelope once.

    `connect()` enables nothing.
        It opens the port, gates on the firmware name, pushes the envelope, and
        stops. Enabling needs `adopt_deg`.

    `connect(calibrate=...)` does not trigger `calibrate()`.
        `is_calibrated` is False today (joint 6), so honouring the flag would
        crash the stock `lerobot-record` path over a known, documented, entirely
        non-blocking condition.
"""

from __future__ import annotations

import logging
import math
import time
from functools import cached_property
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots import Robot

from .calibration import ArmCalibration, enable_blockers, load_calibration
from .config_emre_arm import EmreArmConfig
from .markers import NullMarkerObserver
from .observation import (
    JOINT_IDS,
    JOINT_LABELS,
    action_features,
    build_observation,
    joint_key,
    observation_features,
)
from .transport import CommandError, SerialLink, parse_sta

log = logging.getLogger("lerobot_robot_emre_arm")


class EmreArm(Robot):
    """Six logical joints, seven servos, and no position feedback anywhere."""

    config_class = EmreArmConfig
    name = "emre_arm"

    def __init__(self, config: EmreArmConfig) -> None:
        self.config = config

        # Pure file work, so it can run in __init__ -- which it MUST, because
        # LeRobot requires observation_features/action_features to be callable
        # while disconnected. This is only possible because calibrate() does not
        # drive; a driving calibrate() could not be resolved here.
        self.calibration: ArmCalibration = load_calibration(
            config.limits_csv, config.lock_artifacts_dir, config.wiring_map_csv,
        )

        self.cameras = make_cameras_from_configs(config.cameras)
        self.observer = config.marker_observer or NullMarkerObserver()

        self._link: SerialLink | None = None
        self._enabled: set[int] = set()
        self._clamped: dict[int, bool] = {j: False for j in JOINT_IDS}
        self._adopted: dict[int, float] = {}

    # ------------------------------------------------------------------
    # Feature dicts -- pure functions of config, callable while disconnected
    # ------------------------------------------------------------------

    @cached_property
    def _camera_shapes(self) -> dict[str, tuple[int, int, int]]:
        return {n: (c.height, c.width, 3) for n, c in self.config.cameras.items()}

    @property
    def observation_features(self) -> dict[str, Any]:
        return observation_features(self._camera_shapes)

    @property
    def action_features(self) -> dict[str, type]:
        return action_features()

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        return self._link is not None and self._link.is_open

    @property
    def is_calibrated(self) -> bool:
        """False today, and that is the correct answer -- joint 6.

        Read it as "recording is fine, but joint 6 must not be driven", not as a
        blocker. It is not permanently red either: exactly one joint is
        outstanding, with a documented procedure to clear it. Keeping it strict
        is what stops the flag decaying into noise an operator learns to ignore.

        A joint locked at exactly 0-180 also fails, because that is the servo's
        whole electrical range and the placeholder width at once -- a joint that
        locks at exactly that has most likely never met a mechanical stop.
        """
        for jid in JOINT_IDS:
            jc = self.calibration.joint(jid)
            if not jc.calibrated:
                return False
            if "full_electrical_range" in jc.flags:
                return False
        if self.calibration.mirror_mode == "unknown":
            return False
        return True

    def calibrate(self) -> None:
        raise NotImplementedError(
            "Calibration on this arm is a bench procedure performed by a human, "
            "not a routine this adapter may run.\n\n"
            "To create one you must drive a joint until it stops, which needs "
            "`ENA <j> <adopt_deg>` -- an assertion of where the shaft physically "
            "is. These servos have NO POSITION FEEDBACK, so nothing here can "
            "supply that number, and the firmware pre-loads the adopt pulse "
            "before attaching precisely because a wrong one snaps the joint. An "
            "automatic sweep would also stall servos against mechanical stops on "
            "a ~700 mA supply.\n\n"
            "Do this instead: open the arm console, enable the joint, drag the "
            "envelope handles to the ends you actually reach, press LOCK THIS "
            "AXIS, then copy the downloaded calibration-row-J<id>_*.csv into "
            "Calibration_Notes/lock-artifacts/ and update "
            "Software/arm-console/joint-limits.csv.\n\n"
            f"Currently uncalibrated: {list(self.calibration.uncalibrated_ids())}"
        )

    def _save_calibration(self, *args, **kwargs) -> None:
        raise NotImplementedError(
            "This adapter has no writable calibration file, on purpose. "
            "Software/arm-console/joint-limits.csv and "
            "Calibration_Notes/lock-artifacts/ are the sole source of truth. A "
            "second writable copy is exactly how joint 0's measured 29-110 "
            "envelope was silently widened back to 0-180 on 2026-08-05 -- the "
            "lock that proved it existed only in a Downloads folder, and the file "
            "being edited did not know about it."
        )

    # ------------------------------------------------------------------
    # Connect / configure / disconnect
    # ------------------------------------------------------------------

    def connect(self, calibrate: bool = True) -> None:
        """Open, identify, push the envelope. Enables nothing by itself.

        `calibrate` is accepted for signature compatibility and deliberately does
        NOT call calibrate() -- see the module docstring.

        The whole envelope is re-pushed on EVERY connect, and that is not an
        optimisation to skip: opening the port asserts DTR, which resets a
        genuine Uno, and the firmware keeps nothing across that reset. Limits go
        back to 70-110, CAL=0, MIR=UNKNOWN, watchdog off. Joint 0's home of 64,
        joint 1's of 1 and joint 3's of 33 all sit OUTSIDE 70-110, so enabling
        before the push would return E5 on three of six joints.
        """
        if self.is_connected:
            return

        if not self.is_calibrated:
            log.warning(
                "arm reports is_calibrated=False. Uncalibrated joints: %s. Their "
                "limits are the servo's electrical range, not measured travel.",
                [f"j{j} ({JOINT_LABELS[j]})" for j in self.calibration.uncalibrated_ids()],
            )
        for line in self.calibration.warnings:
            log.warning("calibration: %s", line)

        self._link = SerialLink(
            self.config.port,
            watchdog_ms=self.config.watchdog_ms,
            heartbeat_s=self.config.heartbeat_s,
            idle_disable_s=self.config.idle_disable_s,
        )
        self._link.open()                 # handshake steps 1-5, incl. the VER gate
        log.info("connected to %s, firmware %s", self._link.port, self._link.firmware_version)

        self._push_state()
        self._link.start_background()     # PNG heartbeat + idle park

        for cam in self.cameras.values():
            cam.connect()

        self.configure()

    def _push_state(self) -> None:
        """Handshake step 6, plus a readback. Refuses to run on a live joint.

        ORDER IS LOAD-BEARING IN TWO PLACES:

          WDG FIRST. The board boots with the watchdog disabled, and the LIM/SPD
          loop is a dozen round trips. Arming first means there is no window in
          which a crash leaves the arm driven with no host and no timer. (The
          protocol doc puts WDG last; the arm console puts it first, with that
          rationale. Harmless either way -- the watchdog only arms once a joint
          is enabled, and nothing is enabled during the push -- but first is
          better and it matches the code the board has actually been driven with.)

          MIR AFTER JOINT 1'S LIM. An INV offset is validated against joint 1's
          limits AS THE FIRMWARE CURRENTLY HOLDS THEM. At the boot default of
          70-110 the legal offset window is -35..+35; at the real locked 0-91 it
          is -44..0. Push MIR first and a legal -44 gets refused. Ascending joint
          order already does this -- do not "optimise" it.
        """
        assert self._link is not None
        cal = self.calibration

        # Stricter than the firmware, on purpose. `LIM` on an ENABLED joint is
        # only refused when the new range would exclude that joint's own
        # commanded value -- otherwise it is accepted and the target is clamped
        # inward. "Sometimes accepted" is worse than "refused" for a host that is
        # not watching, so we refuse outright, as the console's loader does.
        snap = parse_sta(self._link.command("STA"))
        live = [j for j in JOINT_IDS if snap.get(j, {}).get("EN") == "1"]
        if live:
            raise RuntimeError(
                f"joints {live} are still ENABLED after opening the port. The "
                "board did NOT reset -- some transports do not assert DTR the "
                "same way -- so they may be live from a previous session at "
                "limits nobody set. Send DIS A and reconnect rather than pushing "
                "an envelope over a driven joint."
            )

        self._link.command(f"WDG {self.config.watchdog_ms}")

        for jid in JOINT_IDS:              # ascending; joint 1's LIM precedes MIR
            jc = cal.joint(jid)
            self._link.command(
                f"LIM {jid} {jc.min_deg} {jc.max_deg} {1 if jc.calibrated else 0}"
            )
            self._link.command(f"SPD {jid} {jc.max_deg_per_sec}")

        # CSV vocabulary is not wire vocabulary: `inverted` -> `INV`, not
        # `INVERTED`. Translated here and nowhere else.
        mir_word = {"same": "SAME", "inverted": "INV"}.get(cal.mirror_mode, "UNKNOWN")
        mir = f"MIR {mir_word}"
        if mir_word == "INV" and cal.mirror_offset_deg:
            mir += f" {cal.mirror_offset_deg}"
        self._link.command(mir)

        self._verify_pushed_state()

    def _verify_pushed_state(self) -> None:
        """Readback. A reset the host did not notice looks exactly like a working
        connection right up until a joint refuses to move."""
        assert self._link is not None
        snap = parse_sta(self._link.command("STA"))

        for jid in JOINT_IDS:
            jc = self.calibration.joint(jid)
            got = snap.get(jid)
            if got is None:
                raise RuntimeError(f"STA returned no line for joint {jid} after the state push")
            want = {
                "MIN": str(jc.min_deg),
                "MAX": str(jc.max_deg),
                "CAL": "1" if jc.calibrated else "0",
                "DPS": str(jc.max_deg_per_sec),
            }
            bad = {k: (got.get(k), v) for k, v in want.items() if got.get(k) != v}
            if bad:
                raise RuntimeError(
                    f"joint {jid} did not take the pushed state: {bad} "
                    "(got, wanted). The board may have reset mid-push."
                )

        sys_row = snap.get("SYS", {})
        mir_word = {"same": "SAME", "inverted": "INV"}.get(self.calibration.mirror_mode, "UNKNOWN")
        if sys_row.get("MIR") != mir_word:
            raise RuntimeError(f"SYS MIR={sys_row.get('MIR')}, wanted {mir_word}")
        if sys_row.get("WDMS") in (None, "0"):
            raise RuntimeError(
                f"SYS WDMS={sys_row.get('WDMS')} -- the watchdog is not armed. "
                "Refusing to enable a joint with no dead-man."
            )
        # NOTE: STA reports the mirror MODE only. The offset appears solely in the
        # `OK MIR ... OFF=` reply, so it cannot be verified from status. Worth a
        # firmware follow-up (add MOFF= to the SYS line).

    def configure(self) -> None:
        """The step that actually drives pins. Separate from connect() because
        the handshake is safe and this is not."""
        if self.config.adopt_deg is None:
            log.warning(
                "no adopt_deg supplied -- NOTHING IS ENABLED. send_action() will "
                "refuse. `ENA <j> <adopt_deg>` needs a human's by-eye estimate of "
                "where each joint is sitting right now; it cannot be synthesised, "
                "and home_deg is editorial, not an observation."
            )
            return

        assert self._link is not None
        for jid in sorted(self.config.adopt_deg):
            if jid not in JOINT_IDS:
                raise ValueError(f"adopt_deg names joint {jid}, which is not addressable")

            blockers = enable_blockers(
                self.calibration, jid,
                allow_unmeasured_mirror=self.config.allow_unmeasured_mirror,
                allow_uncalibrated_joints=self.config.allow_uncalibrated_joints,
            )
            if blockers:
                for b in blockers:
                    log.error("j%d NOT enabled: %s", jid, b)
                continue

            adopt = int(round(self.config.adopt_deg[jid]))
            try:
                self._link.command(f"ENA {jid} {adopt}")
            except CommandError as exc:
                log.error("j%d ENA %d refused: %s", jid, adopt, exc.reply.terminator)
                continue
            self._enabled.add(jid)
            self._adopted[jid] = float(adopt)

        # These are the only record of what a human believed at episode start.
        # They belong in dataset metadata, not just a log line.
        log.info("enabled %s with adopt angles %s", sorted(self._enabled), self._adopted)

    def disconnect(self) -> None:
        for cam in self.cameras.values():
            cam.disconnect()
        if self._link is not None:
            self._link.park_and_close()   # STP, DIS A, close -- THE ARM SAGS
            self._link = None
        self._enabled.clear()

    # ------------------------------------------------------------------
    # The loop
    # ------------------------------------------------------------------

    def get_observation(self) -> dict[str, Any]:
        """One STA round trip, plus whatever the cameras and observer provide.

        `.pos` comes from STA's `SET=` -- the interpolator's CURRENT commanded
        output. Not `TGT=` (where it is heading) and not the MOV ack's `SET=`
        (the accepted target). Three different quantities, two of them sharing a
        key name.
        """
        if not self.is_connected:
            raise RuntimeError("not connected")
        assert self._link is not None

        snap = parse_sta(self._link.command("STA"))

        commanded: dict[int, float] = {}
        for jid in JOINT_IDS:
            row = snap.get(jid, {})
            # On a DISABLED joint, SET= is pure bookkeeping -- no servo ever
            # received it. The firmware seeds every joint to 90 at boot, so an
            # untouched board reports SET=90 on all six. Recording that as a
            # commanded position is the single easiest way to poison a dataset.
            if row.get("EN") == "1":
                commanded[jid] = float(row.get("SET", "nan"))
            else:
                commanded[jid] = math.nan

        frames: dict[str, Any] = {}
        capture_mono: dict[str, float] = {}
        for name, cam in self.cameras.items():
            frames[name] = cam.async_read()
            # BEST EFFORT, AND KNOWN TO BE SLIGHTLY WRONG. The observer contract
            # wants time at FRAME CAPTURE; LeRobot's async_read returns the most
            # recent frame from a background thread, so this over-states freshness
            # by up to one frame interval. Replace with a real capture timestamp
            # if the camera layer ever exposes one.
            capture_mono[name] = time.monotonic()

        fixes = self.observer.observe(frames, capture_mono)

        obs = build_observation(
            commanded=commanded,
            clamped=self._clamped,
            fixes=fixes,
            now_mono=time.monotonic(),
            max_fix_age_s=self.config.max_fix_age_s,
        )
        obs.update(frames)
        return obs

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Returns the CLAMPED value the board accepted, never the requested one.

        LeRobot records what this returns. Recording the request when the
        firmware clamped would teach a policy that an unreachable angle is
        reachable. The protocol clamps and flags `CL=1` rather than rejecting,
        which is why `clamped` is also its own observation column.

        MOV IS TARGET-SEEKING, NOT POSITION-STREAMING. It sets the joint's target
        and the firmware interpolator walks toward it at the joint's deg/s. Send
        one only when a target CHANGES -- six MOVs every control tick would be six
        sequential round trips against the one-command-in-flight rule, and would
        make a perfectly achievable rate look impossible.
        """
        if not self.is_connected:
            raise RuntimeError("not connected")
        if not self._enabled:
            raise RuntimeError(
                "no joint is enabled -- set EmreArmConfig.adopt_deg. Each joint "
                "needs a human's by-eye estimate of where it is sitting right "
                "now; there is no feedback to derive one from."
            )
        assert self._link is not None

        out: dict[str, float] = {}
        for jid in JOINT_IDS:
            key = joint_key(jid, "pos")
            if jid not in self._enabled:
                # NaN means "this joint was not driven this tick" -- NOT
                # "commanded zero". Stated consequence: a dataset with NaN
                # actions is not trainable, so enable all six before recording.
                out[key] = math.nan
                continue

            requested = action.get(key)
            if requested is None or (isinstance(requested, float) and math.isnan(requested)):
                out[key] = math.nan
                continue

            reply = self._link.command(f"MOV {jid} {int(round(float(requested)))}")
            fields = reply.fields()
            self._clamped[jid] = fields.get("CL") == "1"
            out[key] = float(fields.get("SET", "nan"))

        return out
