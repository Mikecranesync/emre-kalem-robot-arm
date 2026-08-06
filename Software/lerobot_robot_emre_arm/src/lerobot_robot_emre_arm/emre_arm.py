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
from dataclasses import dataclass
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


# ---------------------------------------------------------------------------
# The one-MOV-per-CHANGE decision, deliberately kept pure
#
# `lerobot` is not installed on the development host, so NOTHING in this module
# can be imported there -- this function included. Keeping the decision a plain
# function of plain values at least lets it be lifted out of the AST and executed
# standalone, which no method of EmreArm can be. That is the only reason it is a
# module-level function rather than a method.
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class _AcceptedTarget:
    """What the board said the last time it accepted a target for one joint.

    sent_deg
        The INTEGER that actually went on the wire. The wire is integer degrees;
        centidegrees exist only inside the firmware. So this -- not the incoming
        float -- is what a later tick must compare against: 64.4 and 64.5 both
        send `MOV <j> 64`, and comparing the floats would emit a second,
        redundant MOV for a target that never changed.
    accepted_deg
        That MOV ack's `SET=`, i.e. the target AFTER clamping. Note this is the
        MOV-ack `SET=`, not STA's `SET=` -- same key, two different quantities
        (transport.py says so at length). This is the number LeRobot records.

        ALWAYS FINITE. An entry is only ever cached from an ack that carried a
        readable `SET=`; send_action() refuses to cache NaN. The no-MOV path
        replays this value verbatim, so a NaN in here would not be one bad row,
        it would be every row until the target next changed.
    clamped
        That same ack's `CL=`.
    """

    sent_deg: int
    accepted_deg: float
    clamped: bool


def _plan_joint(
    requested: Any, held: _AcceptedTarget | None
) -> tuple[int | None, _AcceptedTarget | None]:
    """Decide what one ENABLED joint does this tick. No I/O, no board state.

    Returns `(send_deg, held)`:

        (None, None)     nothing was asked of this joint -- record NaN.
        (None, entry)    the target has NOT changed. Send no MOV, and record
                         `entry.accepted_deg` -- the value the board actually
                         took when that target was set. Never the request (the
                         firmware may have clamped it, and recording the request
                         teaches a policy that an unreachable angle is
                         reachable), and never NaN (NaN means "not driven this
                         tick", and a joint holding a target is very much still
                         being driven).
        (deg, None)      the target changed. Send `MOV <j> deg`; the recorded
                         value and the clamp flag come from the ack.

    A None/NaN request also produces no MOV, which is why the caller must not
    read a clamp flag off the no-MOV path at all: a readable MOV ack is the only
    thing that SETS `EmreArm._clamped[jid]`. The one other writer clears it —
    get_observation() puts a joint's flag back to False when it reconciles that
    joint away as detached, because a detached joint's clamp flag would assert
    something nobody observed. Nothing else may touch it.
    """
    if requested is None:
        return None, None
    value = float(requested)
    if math.isnan(value):
        return None, None
    sent = int(round(value))
    if held is not None and held.sent_deg == sent:
        return None, held
    return sent, None


class EmreArm(Robot):
    """Six logical joints, seven servos, and no position feedback anywhere."""

    config_class = EmreArmConfig
    name = "emre_arm"

    def __init__(self, config: EmreArmConfig) -> None:
        # Robot.__init__ is CONCRETE, not an empty ABC stub: it sets robot_type,
        # id, calibration_dir and calibration_fpath. Skipping it leaves those
        # unset, and lerobot-record touches them when it names the dataset and
        # resolves the calibration path -- so the omission surfaces as an
        # AttributeError deep inside LeRobot rather than here.
        super().__init__(config)
        self.config = config

        # Pure file work, so it can run in __init__ -- which it MUST, because
        # LeRobot requires observation_features/action_features to be callable
        # while disconnected. This is only possible because calibrate() does not
        # drive; a driving calibrate() could not be resolved here.
        #
        # NOT self.calibration: the base class owns that name for its own
        # MotorCalibration-shaped dict. This arm's calibration is a different
        # shape (soft limits + lock provenance + the J1 mirror rule), so it takes
        # its own attribute instead of silently overwriting the base contract.
        self.arm_calibration: ArmCalibration = load_calibration(
            config.limits_csv, config.lock_artifacts_dir, config.wiring_map_csv,
        )

        self.cameras = make_cameras_from_configs(config.cameras)
        self.observer = config.marker_observer or NullMarkerObserver()

        self._link: SerialLink | None = None
        self._enabled: set[int] = set()
        self._clamped: dict[int, bool] = {j: False for j in JOINT_IDS}
        self._adopted: dict[int, float] = {}
        # The last target this host got the board to accept, per joint. It is
        # what makes send_action() send a MOV only when a target CHANGES, and it
        # is also what send_action() records for a joint that got no MOV. It
        # describes FIRMWARE state, so it is void the moment the firmware is
        # reset or a joint is re-adopted -- cleared in configure(), disconnect()
        # and _check_latch(), and never anywhere else.
        self._targets: dict[int, _AcceptedTarget] = {}

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
            jc = self.arm_calibration.joint(jid)
            if not jc.calibrated:
                return False
            if "full_electrical_range" in jc.flags:
                return False
        if self.arm_calibration.mirror_mode == "unknown":
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
            f"Currently uncalibrated: {list(self.arm_calibration.uncalibrated_ids())}"
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
                [f"j{j} ({JOINT_LABELS[j]})" for j in self.arm_calibration.uncalibrated_ids()],
            )
        for line in self.arm_calibration.warnings:
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
        cal = self.arm_calibration

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
            jc = self.arm_calibration.joint(jid)
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
        mir_word = {"same": "SAME", "inverted": "INV"}.get(self.arm_calibration.mirror_mode, "UNKNOWN")
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
        # FIRST, and above the early return on purpose. This is the only ENA site
        # in the file and connect() always reaches it, so clearing here covers
        # every way a joint's accepted target can become a lie: the DTR reset that
        # opening the port causes (the firmware keeps nothing across it), the
        # re-adopt that ENA itself performs, and a reconnect onto a fresh
        # SerialLink after the port died with _enabled still populated. A joint
        # must never consult an entry left over from a board that has since reset.
        self._targets.clear()

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
                self.arm_calibration, jid,
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
            # A joint is now driven, so the wedged-loop watcher becomes
            # meaningful. It stays disarmed until this point.
            self._link.set_idle_watch(True)
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
        # park_and_close() already sent STP, which freezes every target at its
        # current commanded angle, and DIS A, which detaches. Both make the cached
        # accepted targets false, and the next open() resets the board anyway.
        self._targets.clear()
        # Nothing is driven any more; disarm so the watcher cannot park
        # an arm that is already detached.
        if self._link is not None:
            self._link.set_idle_watch(False)

    # ------------------------------------------------------------------
    # The loop
    # ------------------------------------------------------------------

    def _check_latch(self, snap: dict[Any, dict[str, str]]) -> None:
        """Notice a stop we did not initiate, on the tick it happens.

        `EVT ESTOP` / `EVT WDOG` are asynchronous, and the next command's input
        flush can discard them, so they are not a reliable channel. The LATCH is:
        `SYS ES=1` means the e-stop is latched and every subsequent command
        returns E7; `SYS WD=1` means the latch came from the watchdog rather than
        from an operator. Both are in the STA snapshot we already have.

        Without this check a mid-episode watchdog trip is silent: every joint
        reports EN=0, every commanded value becomes NaN, and the recorded episode
        gives no clue why.
        """
        sys_row = snap.get("SYS", {})
        if sys_row.get("ES") != "1":
            return
        self._enabled.clear()
        # Every joint is detached and recovery routes through CLR + a fresh
        # ENA per joint, so nothing the board previously accepted is still true.
        self._targets.clear()
        # Nothing is driven any more; disarm so the watcher cannot park
        # an arm that is already detached.
        if self._link is not None:
            self._link.set_idle_watch(False)
        raise RuntimeError(
            "the board is E-STOP LATCHED"
            + (" and the latch came from the WATCHDOG -- the host went quiet long "
               "enough that the firmware detached every joint"
               if sys_row.get("WD") == "1" else " (operator-initiated)")
            + ". EVERY JOINT IS DETACHED and a gravity-loaded arm has sagged. "
            "Recovery is CLR, then ENA <j> <FRESHLY ESTIMATED adopt angle> per "
            "joint -- the staleness is mechanical, so the previous adopt angles "
            "are no longer true."
        )

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

        # Tells the idle watcher the control loop is alive. Must come from here
        # and from send_action() only -- the heartbeat is wire traffic, not loop
        # traffic, and counting it would make the idle timeout unreachable.
        self._link.note_loop_activity()

        snap = parse_sta(self._link.command("STA"))
        self._check_latch(snap)

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

        # RECONCILE `_enabled` AGAINST THE BOARD. `EN=` is the truth; the set is
        # only what this object last asked for, and the two diverge SILENTLY.
        # transport.SerialLink._idle_loop sends `DIS A` from its own thread after
        # idle_disable_s of loop silence, detaching every joint without telling
        # this object anything -- try_command swallows the reply, and
        # note_loop_activity() clears only its own parked flag.
        #
        # Left unreconciled, the very next send_action() finds the joint still in
        # `_enabled`, finds an unchanged target in `_targets`, sends no MOV, and
        # writes down the angle the board accepted BEFORE it was detached: a
        # finite, confident number for a joint hanging loose on a sagged arm.
        # That is the dataset lying.
        #
        # Deduplicating MOVs is what made it silent. Before the dedup this same
        # tick sent a MOV at a detached joint and earned a loud ERR E6. Dropping
        # the joint here is what puts the loudness back -- afterwards the joint
        # takes the `jid not in self._enabled` branch and records NaN, which is
        # already defined as "not driven this tick" and is exactly true.
        #
        # `_targets` goes with it: re-enabling at a fresh adopt angle that happens
        # to equal the stale target would otherwise skip the first MOV. `_clamped`
        # goes back to False because a detached joint's clamp flag would assert
        # something nobody observed.
        #
        # The idle watcher's own arming state is deliberately NOT touched here.
        # set_idle_watch() is the explicit-DIS path's to call; the watcher already
        # stops repeating via its parked flag, and a third writer to that state
        # from a reconciliation path is how the 2.0 s watcher went wrong.
        detached = {j for j in self._enabled if snap.get(j, {}).get("EN") != "1"}
        if detached:
            log.error(
                "joints %s are DETACHED on the board but were still marked "
                "enabled here -- almost certainly the idle watcher's DIS A. THE "
                "ARM HAS PROBABLY SAGGED. Their actions now record NaN, and "
                "nothing is driven until each is re-enabled with a FRESHLY "
                "estimated adopt angle: the staleness is mechanical, not "
                "bookkeeping.",
                sorted(detached),
            )
            for jid in detached:
                self._enabled.discard(jid)
                self._targets.pop(jid, None)
                self._clamped[jid] = False

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

        WHAT A JOINT THAT GOT NO MOV RECORDS, AND WHY IT IS NOT NaN. Sending
        fewer commands must not change what LeRobot writes down. A joint whose
        target did not change is still being held at that target, so it records
        the value the board ACCEPTED when the target was set -- from `_targets`,
        which persists across ticks. Not NaN, which means "not driven this tick"
        and would be a lie about a held joint; not the request, which the
        firmware may have clamped. `_clamped[jid]` likewise keeps the flag from
        the accepting ack: no reply arrived this tick, so there is nothing to
        recompute it from, and it must not be.

        Returning the accepted target while the interpolator is still walking
        toward it is correct, not stale. This channel is the accepted target by
        contract; the interpolator's current commanded output is a different
        quantity and it is reported by get_observation() from STA's `SET=`.
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
        self._link.note_loop_activity()      # see get_observation()

        out: dict[str, float] = {}
        for jid in JOINT_IDS:
            key = joint_key(jid, "pos")
            if jid not in self._enabled:
                # NaN means "this joint was not driven this tick" -- NOT
                # "commanded zero". Stated consequence: a dataset with NaN
                # actions is not trainable, so enable all six before recording.
                out[key] = math.nan
                continue

            send_deg, held = _plan_joint(action.get(key), self._targets.get(jid))

            if send_deg is None:
                # No MOV this tick. Either nothing was asked of this joint
                # (held is None -> NaN, same meaning as the disabled case above),
                # or the target is unchanged and the honest record is what the
                # board accepted for it. self._clamped[jid] is deliberately NOT
                # touched on this path -- see the docstring.
                out[key] = math.nan if held is None else held.accepted_deg
                continue

            reply = self._link.command(f"MOV {jid} {send_deg}")
            fields = reply.fields()
            accepted = float(fields.get("SET", "nan"))
            was_clamped = fields.get("CL") == "1"
            if not math.isnan(accepted):
                # Cache and record come from the same ack, so the value a later
                # tick replays can never drift from the value this tick wrote
                # down. An ack with no readable `SET=` is NOT cached: caching NaN
                # would make the next unchanged-target tick replay it, and the
                # one after that, latching a held joint at NaN forever. Learning
                # nothing means the next tick re-sends and gets another chance --
                # which is how this behaved before MOVs were deduplicated.
                self._targets[jid] = _AcceptedTarget(send_deg, accepted, was_clamped)
                # INSIDE the same guard, for the same reason. `CL=` is read off
                # an ack whose `SET=` was unreadable, and an ack we could not
                # parse is not evidence about clamping either -- `fields.get("CL")`
                # simply returns None there and would silently record False,
                # CLEARING a real clamp flag. Flag and target come from one ack or
                # from neither, so they can never disagree about the same tick.
                self._clamped[jid] = was_clamped
            out[key] = accepted

        return out
