"""Load, resolve and validate this arm's calibration from the files that own it.

THE SOURCE OF TRUTH IS THE REPOSITORY, NOT A LeRobot CACHE
    Two files define the calibration and nothing else does:

        Software/arm-console/joint-limits.csv        the limits actually pushed
        Calibration_Notes/lock-artifacts/*.csv       the board's own receipts

    This module reads them. It never writes them. LeRobot's convention is that
    `calibrate()` persists into a writable `calibration_fpath`; we deliberately
    do NOT do that, because a second writable copy of the limits is exactly how
    this project already lost a calibration once. On 2026-08-05 a sweep widened
    joint 0 from its measured 29-110 back to the servo's full 0-180, because the
    lock that proved 29-110 existed only in a browser Downloads folder and the
    file being edited did not know about it. Symptom the next morning: J0 would
    not drive, because commanding into the reclaimed 29 degrees drives the joint
    against a mechanical stop and stalls it there.

    Validators H4 and H5 below are that incident turned into an assertion.

WHAT A "CALIBRATION" IS ON THIS ARM -- SAY IT PLAINLY
    Every number here is an ACCEPTED COMMANDED SOFT LIMIT: an angle a human drove
    the joint to and chose. It is NOT measured mechanical travel. Nothing in this
    system observes a shaft. `calibrated=yes` means "an operator ran LOCK THIS
    AXIS and the board acknowledged it", not "this is where the joint stops".

WHY THIS MODULE CANNOT DRIVE ANYTHING
    Producing a NEW calibration means driving a joint to find its travel, and
    driving requires `ENA <j> <adopt_deg>` -- an assertion of where the shaft
    physically is right now. Nothing in this system can produce that number.
    The firmware pre-loads the adopt pulse BEFORE `attach()` precisely because a
    wrong one snaps the joint. So calibration is an operator act performed at the
    arm console, and this module's job is to load what that act produced, check
    it, and refuse to launder a guess into something that looks measured.
"""

from __future__ import annotations

import csv
import hashlib
import re
from dataclasses import dataclass, field
from pathlib import Path

from .observation import JOINT_IDS, RESERVED_JOINT_ID

# ---------------------------------------------------------------------------
# Repository-relative default locations
#
# Resolved from this file's own position: src/lerobot_robot_emre_arm/ ->
# Software/lerobot_robot_emre_arm/ -> Software/ -> repo root.
# ---------------------------------------------------------------------------

_REPO_ROOT = Path(__file__).resolve().parents[4]

DEFAULT_LIMITS_CSV = _REPO_ROOT / "Software" / "arm-console" / "joint-limits.csv"
DEFAULT_LOCK_DIR = _REPO_ROOT / "Calibration_Notes" / "lock-artifacts"
DEFAULT_WIRING_MAP = _REPO_ROOT / "Software" / "wiring-map.csv"

# ---------------------------------------------------------------------------
# Firmware-mirrored constants. These MUST match the .ino or the adapter will
# push values the board rejects mid-handshake.
# ---------------------------------------------------------------------------

LIM_MIN_SPAN_DEG = 5     # factorylm_arm_controller.ino LIM_MIN_SPAN_DEG
DPS_MIN, DPS_MAX = 1, 90  # SPD argument range
MIR_OFF_MAX_DEG = 90     # MIR offset magnitude cap
ANGLE_MIN, ANGLE_MAX = 0, 180

#: The columns the console's loader requires. A file missing any of these is
#: rejected outright rather than partially loaded.
REQUIRED_COLUMNS = (
    "joint_id",
    "joint_name",
    "uno_pins",
    "min_deg",
    "max_deg",
    "home_deg",
    "max_deg_per_sec",
    "calibrated",
    "mirror_mode",
)

#: Lock artifacts are headerless. Position is the contract, fixed by the console's
#: downloadCalibrationRow(). Column 10 is a constant disclaimer string and is read
#: but discarded.
_LOCK_COLUMNS = 11
_LOCK_FILENAME_RE = re.compile(r"^calibration-row-J(\d+)_(\d{4}-\d{2}-\d{2})_(\d{2})-(\d{2})\.csv$")

# ---------------------------------------------------------------------------
# Flags that cannot be derived from a number, and so are maintained by hand.
#
# Everything else in `JointCalibration.flags` is DERIVED from the loaded values,
# so it cannot go stale. These two are narrative facts read out of
# Calibration_Notes/calibration-log.csv by a human, and they are listed here
# rather than parsed, because the log is prose and parsing prose to decide a
# safety flag would be worse than maintaining a four-line table.
# ---------------------------------------------------------------------------

#: calibration-log.csv 2026-08-01 said the servo physically wired to D3 was the
#: GRIPPER, not the Base, which put the joint NAME -- and therefore the servo
#: type, and therefore the voltage-headroom judgement -- in doubt on both rows.
#:
#: JOINT 6 IS RESOLVED (2026-08-06, OBSERVED). J6 was commanded through two full
#: 10-70 sweeps and the operator watched the gripper open and close. The joint
#: the firmware drives as J6 IS the gripper, so the wiring map is correct about
#: D11 and the dispute does not apply to this row.
#:
#: The 2026-08-01 row was never actually in conflict: it records the SINGLE-SERVO
#: BENCH SKETCH with the horn off and the servo unloaded, where "index 0" selects
#: pin D3 on a bench rig. That is not the assembled arm's wiring. Two different
#: configurations, not two claims about one.
#:
#: JOINT 0 STAYS DISPUTED. The same observation cannot be made for D3, because
#: the base servo is dead -- driving it proves nothing either way. Re-test after
#: the replacement lands and resolve this row the same way: drive it and watch.
_IDENTITY_DISPUTED = frozenset({0})

#: joint-limits.csv: mirror_offset_deg has never been measured on this arm and
#: sits at the placeholder 0. No column can currently express "this offset was
#: measured", which is why `mirror_offset_source` is proposed in the docs.
_MIRROR_OFFSET_UNMEASURED = frozenset({1})

#: THE CURRENT bench supply: a JCPOWER JC-25-5, +5 V 5 A, fitted 2026-08-06 and
#: replacing the unit that measured 6.62 V. At 5.0 V BOTH candidate servo types
#: are in spec -- MG996R 4.8-7.2, MG90S 4.8-6.0 -- so nothing is over spec today.
#: NOT METERED: this is the label value. Meter it. The old unit was nominally
#: 6 V and read 6.62, which is how the over-spec problem happened at all.
_BENCH_SUPPLY_V = 5.0
_MG90S_MAX_V = 6.0

#: Joints that WERE driven on the old 6.62 V supply while rated 4.8-6.0 V.
#:
#: This is a HISTORICAL FACT about those servos and it does not stop being true
#: when the supply is replaced. It is deliberately a separate set from the live
#: `_BENCH_SUPPLY_V` comparison, because conflating the two is what made J6
#: report "over spec" on 2026-08-06 for a supply it had never been connected to
#: -- its own row had forbidden testing on that adapter, so it never ran on it.
#: "This joint was stressed once" and "the supply is unsafe now" are different
#: claims and only one of them is still true.
_DRIVEN_OVER_SPEC_HISTORICALLY = frozenset({4, 5})


class CalibrationError(Exception):
    """A hard validator failed. The adapter must refuse to connect.

    Distinct from a soft warning: a soft warning describes a joint that is
    usable but not trustworthy, and is carried on `JointCalibration.flags`.
    """


# ---------------------------------------------------------------------------
# Dataclasses
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class JointCalibration:
    """One row of joint-limits.csv, joined to its newest lock artifact."""

    # --- identity ---------------------------------------------------------
    joint_id: int
    name: str                    # display only; may be wrong -- see _IDENTITY_DISPUTED
    pins: str                    # provenance only; the firmware owns the real pin map

    # --- what configure() actually pushes ---------------------------------
    min_deg: int                 # LIM arg 2
    max_deg: int                 # LIM arg 3
    max_deg_per_sec: int         # SPD arg 2
    calibrated: bool             # LIM arg 4 -> CAL=0/1

    # --- never pushed, never driven to ------------------------------------
    #
    # This was `home_deg` in the CSV. It is renamed here on purpose. The console
    # uses it at exactly one site: pre-filling the operator's ENA adopt box. It
    # is NOT a home pose, there is no HOM verb, and nothing in this system homes.
    # Calling it `home` in a LeRobot-facing dict would invite a future `go_home()`
    # that drives a gravity-loaded arm to a number nobody has ever observed.
    suggested_adopt_deg: int

    # --- hardware ---------------------------------------------------------
    servo_type: str
    servo_type_source: str       # "INFERRED" | "DOC-CONFIRMED"

    # --- provenance -------------------------------------------------------
    lock_source: str | None
    lock_local_time: str | None
    lock_reply: str | None       # the literal "OK LIM J3 MIN=0 MAX=66 CAL=1"
    lock_min_deg: int | None
    lock_max_deg: int | None
    lock_home_deg: int | None    # compared and reported; NEVER raised on
    lock_fw: str | None
    lock_console: str | None
    superseded_locks: int

    # --- honesty ----------------------------------------------------------
    flags: tuple[str, ...]
    notes: str

    @property
    def span_deg(self) -> int:
        return self.max_deg - self.min_deg

    def clamp(self, deg: float) -> tuple[int, bool]:
        """Clamp to this joint's envelope. Returns (value, was_clamped).

        The firmware clamps too and reports CL=1 rather than rejecting, so this
        is a courtesy that keeps the host's own bookkeeping honest -- it is NOT
        the authority. The board's CL= flag is.
        """
        v = int(round(deg))
        out = max(self.min_deg, min(self.max_deg, v))
        return out, out != v


@dataclass(frozen=True)
class ArmCalibration:
    """Everything loaded, joined and validated. Immutable once built."""

    source_csv: str
    source_csv_sha256: str
    lock_artifacts_dir: str
    wiring_map_csv: str

    #: NOT a kinematic frame. Values are what the firmware is commanded, in the
    #: servo's own degrees. The 21 STLs are individual parts in print orientation
    #: with no assembly file, so link-to-link transforms are UNKNOWN. No sign or
    #: offset knob is offered, because there is no frame to convert into.
    frame: str

    #: Read from the joint 1 row ONLY; ignored on every other row.
    mirror_mode: str             # "same" | "inverted" | "unknown"
    mirror_offset_deg: int
    mirror_offset_source: str    # "measured" | "placeholder"
    mirror_legal_offset_window: tuple[int, int]

    joints: dict[str, JointCalibration]
    warnings: tuple[str, ...] = field(default=())
    reserved_joint_id: int = RESERVED_JOINT_ID

    @property
    def mirror_offset_measured(self) -> bool:
        return self.mirror_offset_source == "measured"

    def joint(self, joint_id: int) -> JointCalibration:
        return self.joints[f"j{joint_id}"]

    def uncalibrated_ids(self) -> tuple[int, ...]:
        return tuple(j for j in JOINT_IDS if not self.joint(j).calibrated)


# ---------------------------------------------------------------------------
# The mirror arithmetic
#
# This is the THIRD copy of this formula in the project -- the firmware's
# mirrorC()/doMir() and the console's mirrorImageOk() are the other two, and they
# were verified identical including the inclusive boundaries. A third copy is a
# drift risk, so it is deliberately tiny, and it carries the console's own
# selftest vectors as a docstring example.
# ---------------------------------------------------------------------------

def mirror_image(min_deg: int, max_deg: int, mode: str, offset_deg: int) -> tuple[bool, int, int]:
    """Where joint 1's mirrored servo (D5) travels, and whether that is legal.

    >>> mirror_image(0, 180, "inverted", 0)[0]
    True
    >>> mirror_image(0, 180, "inverted", 5)[0]
    False
    """
    if mode != "inverted":
        return True, min_deg, max_deg
    axis = 180 + 2 * offset_deg
    lo = axis - max_deg
    hi = axis - min_deg
    return (lo >= ANGLE_MIN and hi <= ANGLE_MAX), lo, hi


def legal_offset_window(min_deg: int, max_deg: int) -> tuple[int, int]:
    """Integer mirror offsets that keep D5 inside 0-180 at THIS envelope.

    Load-bearing, and not obvious: the window depends on the limits the firmware
    is CURRENTLY holding, which is why `MIR` must be pushed after joint 1's `LIM`.

    At the firmware's boot default of 70-110 the window is (-35, 35).
    At joint 1's locked 0-91 it is (-44, 0) -- the mirrored image already touches
    180 exactly, with zero margin, so ANY positive offset is refused. Measuring a
    positive offset therefore FORCES joint 1's max to narrow first. "Just measure
    it later" is not available as a no-op.
    """
    ok = [o for o in range(-MIR_OFF_MAX_DEG, MIR_OFF_MAX_DEG + 1)
          if mirror_image(min_deg, max_deg, "inverted", o)[0]]
    return (min(ok), max(ok)) if ok else (0, 0)


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

def _read_csv_rows(path: Path, *, strict_comma_count: bool) -> list[list[str]]:
    """Read a CSV the way the console reads it.

    Rules from SERIAL-PROTOCOL.md section 10: `#` comment lines and blank lines
    are skipped, and a single malformed row rejects the WHOLE file -- never a
    partial load, because half an envelope is worse than none.

    `strict_comma_count` reproduces the console's split-on-comma strictness for
    joint-limits.csv (no field may contain a comma) while still letting csv
    handle the quoting used by the lock artifacts. Without it this loader would
    accept a quoted-comma file that the console rejects, and the two would then
    disagree about the same file.
    """
    if not path.is_file():
        raise CalibrationError(f"calibration file not found: {path}")

    rows: list[list[str]] = []
    raw = path.read_text(encoding="utf-8-sig").splitlines()
    expected: int | None = None

    for lineno, line in enumerate(raw, start=1):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        parsed = next(csv.reader([line]))
        if expected is None:
            expected = len(parsed)
        elif len(parsed) != expected:
            raise CalibrationError(
                f"{path.name} line {lineno}: {len(parsed)} fields, expected {expected}. "
                "One malformed row rejects the whole file -- a partially applied "
                "limits file is worse than no limits file."
            )
        if strict_comma_count and line.count(",") != expected - 1:
            raise CalibrationError(
                f"{path.name} line {lineno}: a field contains a comma. The arm "
                "console splits this file on commas, so it would read this row "
                "differently. No field may contain a comma."
            )
        rows.append(parsed)

    if not rows:
        raise CalibrationError(f"{path.name} contains no data rows")
    return rows


def _load_wiring_map(path: Path) -> dict[int, tuple[str, str]]:
    """index -> (servo_type, servo_type_source). Missing file is NOT fatal."""
    if not path.is_file():
        return {}
    rows = _read_csv_rows(path, strict_comma_count=False)
    header = [c.strip() for c in rows[0]]
    out: dict[int, tuple[str, str]] = {}
    for row in rows[1:]:
        rec = dict(zip(header, row))
        try:
            idx = int(rec["index"])
        except (KeyError, ValueError):
            continue
        out[idx] = (rec.get("servo_type", "unknown").strip(),
                    rec.get("servo_type_source", "unknown").strip())
    return out


@dataclass(frozen=True)
class _LockRow:
    joint_id: int
    local_time: str
    min_deg: int
    max_deg: int
    home_deg: int
    reply: str
    fw: str
    console: str
    source: str


def _load_lock_artifacts(directory: Path) -> dict[int, list[_LockRow]]:
    """All lock receipts, grouped by joint, newest first."""
    by_joint: dict[int, list[_LockRow]] = {}
    if not directory.is_dir():
        return by_joint

    for path in sorted(directory.glob("calibration-row-J*.csv")):
        m = _LOCK_FILENAME_RE.match(path.name)
        if m is None:
            raise CalibrationError(
                f"{path.name} is in lock-artifacts/ but does not match the "
                "calibration-row-J<id>_<YYYY-MM-DD>_<HH-MM>.csv name the console "
                "produces. Refusing to guess what it is."
            )
        rows = _read_csv_rows(path, strict_comma_count=False)
        if len(rows) != 1 or len(rows[0]) != _LOCK_COLUMNS:
            raise CalibrationError(
                f"{path.name}: expected exactly one row of {_LOCK_COLUMNS} columns, "
                f"got {len(rows)} row(s). The lock artifact schema is positional."
            )
        r = rows[0]
        jid = int(r[1])
        if jid != int(m.group(1)):
            raise CalibrationError(
                f"{path.name}: filename says joint {m.group(1)} but the row says "
                f"joint {jid}. One of them is wrong; resolve it by hand."
            )
        by_joint.setdefault(jid, []).append(_LockRow(
            joint_id=jid,
            local_time=r[0].strip(),
            min_deg=int(r[4]),
            max_deg=int(r[5]),
            home_deg=int(r[6]),
            reply=r[7].strip(),
            fw=r[8].strip(),
            console=r[9].strip(),
            source=path.name,
        ))

    # Newest wins. Three locks exist for joint 1 and two for joint 3; the
    # timestamp is the tie-break and the filename sorts the same way.
    for jid in by_joint:
        by_joint[jid].sort(key=lambda row: row.local_time, reverse=True)
    return by_joint


# ---------------------------------------------------------------------------
# Flag derivation
# ---------------------------------------------------------------------------

def _derive_flags(
    joint_id: int,
    min_deg: int,
    max_deg: int,
    calibrated: bool,
    suggested_adopt_deg: int,
    servo_type: str,
    servo_type_source: str,
    lock: _LockRow | None,
    superseded: int,
) -> tuple[str, ...]:
    flags: list[str] = []

    if not calibrated:
        flags.append("uncalibrated")
    if lock is None:
        flags.append("no_lock_artifact")

    # 0-180 is the placeholder width AND the servo's whole electrical range. A
    # joint that locks at exactly that has most likely never been driven to a
    # mechanical stop at either end.
    if min_deg == ANGLE_MIN and max_deg == ANGLE_MAX:
        flags.append("full_electrical_range")
    if min_deg <= 2:
        flags.append("min_at_electrical_floor")
    if max_deg >= 178:
        flags.append("max_at_electrical_ceiling")

    if lock is not None and suggested_adopt_deg != lock.home_deg:
        # Expected, not alarming: home is editorial and was hand-moved off a hard
        # end of travel on four of five rows.
        flags.append("home_edited")
    if superseded:
        flags.append("superseded_lock")

    if servo_type_source.upper() == "INFERRED":
        flags.append("type_inferred")

    is_mg90s = "MG90S" in servo_type.upper()
    if joint_id in _IDENTITY_DISPUTED:
        flags.append("identity_disputed")

    # Voltage headroom against the CURRENT supply, tested at the STRICTEST rating
    # this joint could plausibly carry -- so an unresolved identity is handled by
    # assuming the tighter MG90S window rather than by reporting "unknown".
    #
    # This used to be nested inside the identity branch, on the reasoning that an
    # unresolved TYPE left the headroom unresolved too. That reasoning expired
    # with the supply: at 5.0 V both candidate types are comfortably in spec, so
    # the answer is the same whichever the servo turns out to be. There is
    # nothing left to be unresolved about, and `over_spec_supply_unresolved` is
    # therefore no longer emitted.
    if (is_mg90s or joint_id in _IDENTITY_DISPUTED) and _BENCH_SUPPLY_V > _MG90S_MAX_V:
        flags.append("over_spec_supply")

    # Separate, and permanent: what this joint was PUT THROUGH, not what it sits
    # on now. Replacing the supply fixes the condition, not the history.
    if joint_id in _DRIVEN_OVER_SPEC_HISTORICALLY:
        flags.append("driven_over_spec_historically")

    if joint_id in _MIRROR_OFFSET_UNMEASURED:
        flags.append("mirror_offset_unmeasured")

    return tuple(flags)


# ---------------------------------------------------------------------------
# The loader -- hard validators H1..H7 live here
# ---------------------------------------------------------------------------

def load_calibration(
    limits_csv: Path | str = DEFAULT_LIMITS_CSV,
    lock_dir: Path | str = DEFAULT_LOCK_DIR,
    wiring_map: Path | str = DEFAULT_WIRING_MAP,
) -> ArmCalibration:
    """Load, join and validate. Sends nothing. Drives nothing. Writes nothing.

    Raises CalibrationError on H1-H7. Soft findings become `flags` on the joint
    and a line in `ArmCalibration.warnings`.

    A DELIBERATE DEVIATION FROM THE PROTOCOL DOC: SERIAL-PROTOCOL.md section 10
    says a missing limits file falls back to 70-110 with CAL=0 and an amber
    UNCALIBRATED badge in the GUI. A headless adapter has nowhere to show amber,
    and a silent 70-110 would clamp every command into a 40-degree window and
    present as a hardware fault. So we raise instead.
    """
    limits_path = Path(limits_csv)
    lock_path = Path(lock_dir)
    wiring_path = Path(wiring_map)

    # H1 -- file present, parseable, required columns present.
    rows = _read_csv_rows(limits_path, strict_comma_count=True)
    header = [c.strip() for c in rows[0]]
    missing = [c for c in REQUIRED_COLUMNS if c not in header]
    if missing:
        raise CalibrationError(
            f"H1: {limits_path.name} is missing required column(s): {missing}. "
            f"Required: {list(REQUIRED_COLUMNS)}"
        )

    wiring = _load_wiring_map(wiring_path)
    locks = _load_lock_artifacts(lock_path)
    warnings: list[str] = []
    joints: dict[str, JointCalibration] = {}

    mirror_mode = "unknown"
    mirror_offset_deg = 0
    mirror_offset_source = "placeholder"

    seen: set[int] = set()
    for row in rows[1:]:
        rec = {k: v.strip() for k, v in zip(header, row)}
        try:
            jid = int(rec["joint_id"])
        except ValueError as exc:
            raise CalibrationError(f"H1: joint_id {rec['joint_id']!r} is not an integer") from exc

        # H2 -- joint 2 is the shoulder pair's second servo and is unaddressable
        # by construction. A row for it means the file was written against a
        # different mental model of this arm.
        if jid == RESERVED_JOINT_ID:
            raise CalibrationError(
                f"H2: {limits_path.name} has a row for joint {RESERVED_JOINT_ID}. "
                "That is D5, the shoulder pair's second servo. It is not "
                "addressable -- joint 1 drives D4+D5 as one logical joint."
            )
        if jid not in JOINT_IDS:
            raise CalibrationError(f"H2: unknown joint_id {jid}; addressable ids are {list(JOINT_IDS)}")
        if jid in seen:
            raise CalibrationError(f"H2: duplicate row for joint {jid}")
        seen.add(jid)

        try:
            mn = int(rec["min_deg"])
            mx = int(rec["max_deg"])
            home = int(rec["home_deg"])
            dps = int(rec["max_deg_per_sec"])
        except ValueError as exc:
            raise CalibrationError(f"H1: joint {jid} has a non-integer numeric field") from exc

        # H3 -- everything the firmware would reject, rejected here first, so the
        # failure names the file rather than arriving as an E10 mid-handshake.
        if not (ANGLE_MIN <= mn < mx <= ANGLE_MAX):
            raise CalibrationError(f"H3: joint {jid} range {mn}-{mx} is outside 0-180 or not min<max")
        if mx - mn < LIM_MIN_SPAN_DEG:
            raise CalibrationError(
                f"H3: joint {jid} span {mx - mn} is below the firmware's "
                f"LIM_MIN_SPAN_DEG of {LIM_MIN_SPAN_DEG}"
            )
        if not (mn <= home <= mx):
            raise CalibrationError(f"H3: joint {jid} home {home} is outside its own range {mn}-{mx}")
        if not (DPS_MIN <= dps <= DPS_MAX):
            raise CalibrationError(f"H3: joint {jid} max_deg_per_sec {dps} is outside {DPS_MIN}-{DPS_MAX}")

        cal_raw = rec["calibrated"].lower()
        if cal_raw not in ("yes", "no"):
            raise CalibrationError(f"H3: joint {jid} calibrated={cal_raw!r}; expected yes or no")
        calibrated = cal_raw == "yes"

        joint_locks = locks.get(jid, [])
        newest = joint_locks[0] if joint_locks else None

        # H4 -- claims a calibration with no receipt to back it.
        #
        # THIS IS THE J0-REGRESSION DETECTOR. Known friction, and the friction is
        # the point: a fresh lock lands in the browser's Downloads folder, so this
        # fails until someone copies it into Calibration_Notes/lock-artifacts/.
        if calibrated and newest is None:
            raise CalibrationError(
                f"H4: joint {jid} is marked calibrated=yes but no lock artifact "
                f"exists in {lock_path}. A LOCK downloads to the browser's "
                "Downloads folder; copy calibration-row-J"
                f"{jid}_<date>_<HH-MM>.csv into that directory. Until then this "
                "row asserts a measurement with nothing behind it -- which is "
                "exactly how joint 0's measured 29-110 was silently widened to "
                "0-180 on 2026-08-05."
            )

        # H5 -- the file and the board's own receipt disagree.
        #
        # home is compared and REPORTED but never raised on: it legitimately
        # differs on four of five rows because a home sitting on a hard end of
        # travel leaves nowhere to back off to, so operators moved it by hand.
        if newest is not None and (mn, mx) != (newest.min_deg, newest.max_deg):
            raise CalibrationError(
                f"H5: joint {jid} says {mn}-{mx} but its newest lock artifact "
                f"({newest.source}) says {newest.min_deg}-{newest.max_deg}. "
                "One of them has been edited without a new measurement. Resolve "
                "by hand -- do not assume the wider one is right."
            )

        if jid == 1:
            mirror_mode = (rec.get("mirror_mode") or "unknown").lower() or "unknown"
            if mirror_mode not in ("same", "inverted", "unknown"):
                raise CalibrationError(
                    f"H3: joint 1 mirror_mode={mirror_mode!r}; expected same, inverted or unknown"
                )
            off_raw = (rec.get("mirror_offset_deg") or "0").strip() or "0"
            try:
                mirror_offset_deg = int(off_raw)
            except ValueError as exc:
                raise CalibrationError(f"H3: joint 1 mirror_offset_deg={off_raw!r} is not an integer") from exc
            if abs(mirror_offset_deg) > MIR_OFF_MAX_DEG:
                raise CalibrationError(
                    f"H3: joint 1 mirror_offset_deg {mirror_offset_deg} exceeds "
                    f"the firmware's +/-{MIR_OFF_MAX_DEG}"
                )
            # PROPOSED COLUMN, absent today. Deriving measured-ness from
            # `offset == 0` would be wrong: a real measurement could legitimately
            # come out 0. Absent column means placeholder, always.
            mirror_offset_source = (rec.get("mirror_offset_source") or "placeholder").lower() or "placeholder"
            if mirror_offset_source not in ("measured", "placeholder"):
                raise CalibrationError(
                    f"H3: joint 1 mirror_offset_source={mirror_offset_source!r}; "
                    "expected measured or placeholder"
                )

        servo_type, servo_type_source = wiring.get(jid, ("unknown", "unknown"))
        superseded = max(0, len(joint_locks) - 1)
        flags = _derive_flags(
            jid, mn, mx, calibrated, home, servo_type, servo_type_source, newest, superseded,
        )

        joints[f"j{jid}"] = JointCalibration(
            joint_id=jid,
            name=rec["joint_name"],
            pins=rec["uno_pins"],
            min_deg=mn,
            max_deg=mx,
            max_deg_per_sec=dps,
            calibrated=calibrated,
            suggested_adopt_deg=home,
            servo_type=servo_type,
            servo_type_source=servo_type_source,
            lock_source=newest.source if newest else None,
            lock_local_time=newest.local_time if newest else None,
            lock_reply=newest.reply if newest else None,
            lock_min_deg=newest.min_deg if newest else None,
            lock_max_deg=newest.max_deg if newest else None,
            lock_home_deg=newest.home_deg if newest else None,
            lock_fw=newest.fw if newest else None,
            lock_console=newest.console if newest else None,
            superseded_locks=superseded,
            flags=flags,
            notes=rec.get("notes", ""),
        )

    absent = [j for j in JOINT_IDS if f"j{j}" not in joints]
    if absent:
        raise CalibrationError(f"H1: {limits_path.name} has no row for joint(s) {absent}")

    # H7 -- joint 1's mirrored image must stay inside 0-180 at the CURRENT
    # envelope. The firmware checks this too (E11 MIRROR=out_of_travel); checking
    # here means the failure names the file instead of arriving mid-handshake.
    j1 = joints["j1"]
    ok, lo, hi = mirror_image(j1.min_deg, j1.max_deg, mirror_mode, mirror_offset_deg)
    if not ok:
        raise CalibrationError(
            f"H7: joint 1 at {j1.min_deg}-{j1.max_deg} with mirror_mode="
            f"{mirror_mode} offset={mirror_offset_deg} mirrors D5 to {lo}-{hi}, "
            "which escapes 0-180. The firmware would refuse this with E11."
        )
    window = legal_offset_window(j1.min_deg, j1.max_deg) if mirror_mode == "inverted" else (0, 0)

    # ---- soft warnings ---------------------------------------------------
    for jid in JOINT_IDS:
        jc = joints[f"j{jid}"]
        if jc.flags:
            warnings.append(f"j{jid} ({jc.name}): {', '.join(jc.flags)}")
    if mirror_mode == "inverted" and mirror_offset_source != "measured":
        warnings.append(
            f"joint 1 mirror_offset_deg={mirror_offset_deg} is a PLACEHOLDER, not a "
            f"measurement. Legal offsets at the current {j1.min_deg}-{j1.max_deg} "
            f"envelope are {window[0]}..{window[1]} -- note that any POSITIVE offset "
            "is refused, so measuring one forces joint 1's max to narrow first. "
            "Until it is measured, two MG996Rs may be fighting each other the whole "
            "time joint 1 is driven, with nothing on screen showing it."
        )

    return ArmCalibration(
        source_csv=str(limits_path),
        source_csv_sha256=hashlib.sha256(limits_path.read_bytes()).hexdigest(),
        lock_artifacts_dir=str(lock_path),
        wiring_map_csv=str(wiring_path),
        frame="servo_command_deg",
        mirror_mode=mirror_mode,
        mirror_offset_deg=mirror_offset_deg,
        mirror_offset_source=mirror_offset_source,
        mirror_legal_offset_window=window,
        joints=joints,
        warnings=tuple(warnings),
    )


# ---------------------------------------------------------------------------
# Enable-time gate (H6) -- checked before ENA, not at load time
# ---------------------------------------------------------------------------

def enable_blockers(
    cal: ArmCalibration,
    joint_id: int,
    *,
    allow_unmeasured_mirror: bool = False,
    allow_uncalibrated_joints: bool = False,
) -> list[str]:
    """Reasons this joint must NOT be enabled. Empty list means go ahead.

    H6 (joint 1 with an unknown mirror relation) is the firmware's own guard --
    `ENA 1` returns E13 while MIR is UNKNOWN. The other two guards exist because
    the firmware canNOT see the hazard:

      * joint-limits.csv sets mirror_mode=inverted, so E13 does NOT fire, yet
        mirror_offset_deg is still the unmeasured placeholder. The firmware guard
        is satisfied while the actual hazard is fully present.
      * an uncalibrated joint's limits are the servo's electrical range, not its
        travel. A gripper hits its own linkage long before 0 or 180.
    """
    jc = cal.joint(joint_id)
    blockers: list[str] = []

    if joint_id == 1 and cal.mirror_mode == "unknown":
        blockers.append(
            "H6: joint 1 cannot be enabled while the mirror relation is UNKNOWN "
            "-- the firmware refuses ENA 1 with E13."
        )
    if joint_id == 1 and not cal.mirror_offset_measured and not allow_unmeasured_mirror:
        blockers.append(
            "joint 1's mirror_offset_deg has never been measured. If the true "
            "axis is not 90, the two MG996Rs fight each other by twice the error "
            "for as long as the joint is driven -- they hold, run hot, and strip "
            "a gear. Set allow_unmeasured_mirror=True to accept that risk."
        )
    if not jc.calibrated and not allow_uncalibrated_joints:
        blockers.append(
            f"joint {joint_id} is UNCALIBRATED: {jc.min_deg}-{jc.max_deg} is the "
            "servo's electrical range, not measured travel. Set "
            "allow_uncalibrated_joints=True to accept that risk."
        )
    return blockers
