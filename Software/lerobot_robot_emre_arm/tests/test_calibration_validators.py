"""The calibration loader and its hard validators, asserted against the real files.

The happy-path tests run `load_calibration()` over the files actually committed to
this repository -- `Software/arm-console/joint-limits.csv`,
`Calibration_Notes/lock-artifacts/` and `Software/wiring-map.csv`. That is
deliberate. H4 and H5 exist to keep those three in agreement, so a fixture copy
would test the fixture instead of the thing the validators guard.

The failure-path tests start from the same real file, change exactly ONE thing,
write the result to a tmp_path, and assert the loader refuses it. The committed
files are only ever read.

THE INCIDENT THESE TESTS ENCODE
    On 2026-08-05 a sweep widened joint 0 from its locked 29-110 back to the
    servo's full 0-180, because the receipt proving 29-110 lived only in a browser
    Downloads folder. Commanding into the reclaimed 29 degrees drives the joint
    against a mechanical stop and stalls it there. H5 is that incident turned into
    an assertion, and `test_widening_an_envelope_against_its_own_receipt_is_refused`
    is the test that would have caught it.

VOCABULARY
    Every number in these files is an ACCEPTED COMMANDED SOFT LIMIT -- an angle an
    operator drove to and chose, acknowledged by the controller. Nothing in this
    system observes a shaft, so a lock is a record of a decision, not of travel.
    "measurement" below always means the operator act, never a shaft reading.
"""

from __future__ import annotations

import hashlib
import inspect

import pytest

from _repo_files import (
    REAL_LIMITS_CSV,
    REAL_LOCK_DIR,
    REAL_WIRING_MAP,
    write_lock_artifact,
)


# ==========================================================================
# The committed files, as they stand
# ==========================================================================


def test_the_committed_calibration_loads(real_cal):
    assert sorted(real_cal.joints) == ["j0", "j1", "j3", "j4", "j5", "j6"]


def test_there_is_no_entry_for_the_reserved_joint(real_cal, observation):
    assert "j2" not in real_cal.joints
    assert real_cal.reserved_joint_id == observation.RESERVED_JOINT_ID == 2


def test_joint_0_holds_its_restored_envelope(real_cal):
    # THE REGRESSION PIN. If this ever reads 0-180 again, the incident has
    # repeated. 29-110 was driven to and chosen; 0-180 is the servo's electrical
    # range and 29 degrees of it are known to be unreachable.
    j0 = real_cal.joint(0)
    assert (j0.min_deg, j0.max_deg) == (29, 110)
    assert j0.span_deg == 81
    assert j0.calibrated is True


def test_joint_0_agrees_with_its_own_receipt(real_cal):
    j0 = real_cal.joint(0)
    assert (j0.lock_min_deg, j0.lock_max_deg) == (29, 110)
    assert j0.lock_source == "calibration-row-J0_2026-08-05_15-36.csv"
    assert j0.lock_reply == "OK LIM J0 MIN=29 MAX=110 CAL=1"
    assert j0.lock_fw == "1.0.0"
    assert j0.lock_console == "1.2.0"


def test_every_joint_on_this_arm_now_has_a_lock(real_cal):
    """J6 was the last hold-out. It was locked 2026-08-06 14:53 at 10-70.

    This test used to assert the opposite -- that the gripper was the one joint
    with no lock at all -- and it failed the moment the operator locked it. That
    is the test doing its job: it encodes a fact about THIS arm, so it is
    supposed to break when the arm changes.
    """
    j6 = real_cal.joint(6)
    assert j6.calibrated is True
    assert j6.lock_source == "calibration-row-J6_2026-08-06_14-53.csv"
    assert (j6.lock_min_deg, j6.lock_max_deg) == (10, 70)
    assert real_cal.uncalibrated_ids() == ()


def test_newest_lock_wins_and_the_older_ones_are_counted(real_cal):
    # Three receipts exist for joint 1 (18:05, 18:47, 19:04) and two for joint 3.
    j1 = real_cal.joint(1)
    assert j1.lock_source == "calibration-row-J1_2026-08-05_19-04.csv"
    assert (j1.lock_min_deg, j1.lock_max_deg) == (0, 91)
    assert j1.superseded_locks == 2
    assert "superseded_lock" in j1.flags

    j3 = real_cal.joint(3)
    assert j3.lock_source == "calibration-row-J3_2026-08-06_04-43.csv"
    assert j3.superseded_locks == 1


def test_the_adopt_angle_is_editorial_and_is_flagged_when_it_was_moved(real_cal):
    # `home_deg` is renamed `suggested_adopt_deg` here because it is only ever used
    # to pre-fill the operator's ENA box. Nothing homes; there is no HOM verb.
    j1 = real_cal.joint(1)
    assert j1.lock_home_deg == 0          # the operator was at the bottom of travel
    assert j1.suggested_adopt_deg == 1    # moved off the hard end by hand
    assert "home_edited" in j1.flags


def test_an_unedited_adopt_angle_is_not_flagged(real_cal):
    # The flag is derived, not decorative: joint 0's adopt angle matches its
    # receipt, so it carries no `home_edited`.
    j0 = real_cal.joint(0)
    assert j0.suggested_adopt_deg == j0.lock_home_deg == 64
    assert "home_edited" not in j0.flags


# ==========================================================================
# Derived flags -- the soft findings that do NOT raise
# ==========================================================================


def test_a_locked_joint_sitting_at_the_full_electrical_range_is_still_flagged(real_cal):
    # Joint 4 IS locked, so calibrated=yes is honest about what happened. But
    # 0-180 is the servo's whole electrical range, and a lock is not evidence of
    # travel. The flag is the only thing that says so.
    j4 = real_cal.joint(4)
    assert j4.calibrated is True
    assert (j4.min_deg, j4.max_deg) == (0, 180)
    assert "full_electrical_range" in j4.flags
    assert "min_at_electrical_floor" in j4.flags
    assert "max_at_electrical_ceiling" in j4.flags


def test_joint_0_identity_is_still_disputed_but_its_voltage_no_longer_is(real_cal):
    """The dispute outlived the voltage question it used to imply.

    calibration-log 2026-08-01 says the servo on D3 was the GRIPPER, not the
    Base, so J0's servo TYPE is unresolved -- and it stays unresolved, because
    the base servo is dead and driving D3 proves nothing either way. Re-test
    after the replacement.

    What DID resolve is the headroom. That used to be reported as unknown
    because the type was unknown; on the +5 V JC-25-5 both candidate types are
    in spec (MG996R 4.8-7.2, MG90S 4.8-6.0), so the answer is the same whichever
    it turns out to be and there is nothing left to be unresolved about.
    """
    j0 = real_cal.joint(0)
    assert "identity_disputed" in j0.flags
    assert "over_spec_supply_unresolved" not in j0.flags
    assert "over_spec_supply" not in j0.flags


def test_joint_6_identity_was_resolved_by_watching_it_move(real_cal):
    """J6 was commanded 10-70 twice and the operator saw the gripper open/close.

    That is the approved exception to the vocabulary rule: a value a human
    actually OBSERVED. The joint the firmware drives as J6 is the gripper, so
    the wiring map is right about D11 and the 2026-08-01 row -- which records a
    single-servo BENCH rig, horn off, where index 0 meant pin D3 -- was never
    describing the assembled arm at all.
    """
    assert "identity_disputed" not in real_cal.joint(6).flags


def test_a_joint_driven_over_spec_keeps_that_history_after_the_supply_is_fixed(real_cal):
    """Replacing the supply fixes the condition, not the history.

    J4 and J5 are MG90S rated 4.8-6.0 V and were both driven on the old 6.62 V
    unit. Swapping to +5 V makes them safe FROM NOW ON; it does not un-stress
    them. The two claims are now separate flags, because conflating them made J6
    report "over spec" for a supply its own row had forbidden it from ever
    touching.
    """
    j4 = real_cal.joint(4)
    assert j4.servo_type == "MG90S"
    assert "driven_over_spec_historically" in j4.flags
    assert "driven_over_spec_historically" in real_cal.joint(5).flags
    # ...and nothing is over spec on the CURRENT supply.
    assert "over_spec_supply" not in j4.flags
    assert not any(
        "over_spec_supply" in real_cal.joint(j).flags for j in (0, 1, 3, 4, 5, 6)
    )
    # J6 is MG90S too but was never connected to the old supply.
    assert "driven_over_spec_historically" not in real_cal.joint(6).flags


def test_an_inferred_servo_type_is_flagged_and_a_confirmed_one_is_not(real_cal):
    assert real_cal.joint(4).servo_type_source == "INFERRED"
    assert "type_inferred" in real_cal.joint(4).flags
    assert real_cal.joint(5).servo_type_source == "DOC-CONFIRMED"
    assert "type_inferred" not in real_cal.joint(5).flags


def test_the_placeholder_width_flags_now_live_on_the_suspect_wrist(real_cal):
    """J4 is the remaining 0-180 row, so it carries the placeholder-width flags.

    These used to be asserted against J6. J6 was locked to a real 10-70 on
    2026-08-06 and dropped them; J4 still sits at exactly the servo's whole
    electrical range and is flagged SUSPECT in joint-limits.csv for that reason.
    The flags did not disappear -- they moved to the joint that still earns them.
    """
    flags = real_cal.joint(4).flags
    for expected in (
        "full_electrical_range",
        "min_at_electrical_floor",
        "max_at_electrical_ceiling",
    ):
        assert expected in flags, expected


def test_the_locked_gripper_no_longer_claims_placeholder_width(real_cal):
    flags = real_cal.joint(6).flags
    for gone in (
        "uncalibrated",
        "no_lock_artifact",
        "full_electrical_range",
        "min_at_electrical_floor",
        "max_at_electrical_ceiling",
    ):
        assert gone not in flags, gone
    # What it DOES still carry: home was moved off the locked end. The D3/D11
    # identity dispute that used to sit here was resolved on 2026-08-06 by
    # driving J6 and watching the gripper move -- see
    # test_joint_6_identity_was_resolved_by_watching_it_move.
    assert "home_edited" in flags
    assert "identity_disputed" not in flags


def test_every_flagged_joint_appears_in_the_warnings(real_cal, observation):
    joined = "\n".join(real_cal.warnings)
    for jid in observation.JOINT_IDS:
        jc = real_cal.joint(jid)
        if jc.flags:
            assert f"j{jid}" in joined


def test_soft_findings_do_not_raise(real_cal):
    # Every joint on this arm has at least one flag, and the load still succeeds.
    # A soft finding describes a joint that is usable but not trustworthy.
    assert real_cal.warnings
    assert all(real_cal.joint(j).flags for j in (0, 1, 3, 4, 5, 6))


# ==========================================================================
# The mirror -- joint 1's unmeasured offset
# ==========================================================================


def test_the_mirror_offset_is_a_placeholder_and_says_so(real_cal):
    assert real_cal.mirror_mode == "inverted"
    assert real_cal.mirror_offset_deg == 0
    assert real_cal.mirror_offset_source == "placeholder"
    assert real_cal.mirror_offset_measured is False
    assert "mirror_offset_unmeasured" in real_cal.joint(1).flags


def test_an_absent_source_column_means_placeholder_not_measured(real_cal):
    # A real reading could legitimately come out 0, so measured-ness can never be
    # inferred from `offset == 0`. The column is absent from the committed file,
    # and absence means placeholder.
    header = REAL_LIMITS_CSV.read_text(encoding="utf-8-sig")
    assert "mirror_offset_source" not in header
    assert real_cal.mirror_offset_source == "placeholder"


def test_the_legal_offset_window_at_the_locked_shoulder_range_forbids_positives(real_cal):
    # At joint 1's locked 0-91 the mirrored image already touches 180 exactly,
    # with zero margin. Measuring any positive offset therefore FORCES joint 1's
    # max to narrow first -- "just measure it later" is not a no-op.
    assert real_cal.mirror_legal_offset_window == (-44, 0)
    assert real_cal.mirror_legal_offset_window[1] == 0


def test_the_placeholder_offset_is_called_out_in_the_warnings(real_cal):
    hits = [w for w in real_cal.warnings if "PLACEHOLDER" in w]
    assert len(hits) == 1
    assert "-44..0" in hits[0]


def test_mirror_image_matches_its_own_docstring_vectors(calibration):
    # The docstring's two examples, hand-written rather than run through
    # --doctest-modules (which would use normal import machinery and hit the
    # eager package __init__).
    assert calibration.mirror_image(0, 180, "inverted", 0)[0] is True
    assert calibration.mirror_image(0, 180, "inverted", 5)[0] is False


def test_mirror_image_is_a_no_op_when_the_relation_is_not_inverted(calibration):
    for mode in ("same", "unknown"):
        assert calibration.mirror_image(0, 91, mode, 40) == (True, 0, 91)


def test_the_mirrored_image_of_the_locked_shoulder_range_touches_180_exactly(calibration):
    ok, lo, hi = calibration.mirror_image(0, 91, "inverted", 0)
    assert (ok, lo, hi) == (True, 89, 180)


def test_the_legal_offset_window_depends_on_the_current_envelope(calibration):
    # Which is why MIR must be pushed after joint 1's LIM. At the firmware's boot
    # default of 70-110 the window is symmetric; at the locked 0-91 it is not.
    assert calibration.legal_offset_window(70, 110) == (-35, 35)
    assert calibration.legal_offset_window(0, 91) == (-44, 0)


# ==========================================================================
# Provenance and immutability
# ==========================================================================


def test_the_loaded_calibration_records_where_it_came_from(real_cal):
    assert real_cal.source_csv == str(REAL_LIMITS_CSV)
    assert real_cal.lock_artifacts_dir == str(REAL_LOCK_DIR)
    assert real_cal.wiring_map_csv == str(REAL_WIRING_MAP)
    assert real_cal.source_csv_sha256 == hashlib.sha256(
        REAL_LIMITS_CSV.read_bytes()
    ).hexdigest()


def test_the_frame_is_named_as_servo_command_degrees(real_cal):
    # NOT a kinematic frame. The 21 STLs are individual parts in print orientation
    # with no assembly file, so link-to-link transforms are unknown and no sign or
    # offset knob is offered.
    assert real_cal.frame == "servo_command_deg"


def test_loading_writes_nothing(calibration):
    before = hashlib.sha256(REAL_LIMITS_CSV.read_bytes()).hexdigest()
    locks_before = sorted(p.name for p in REAL_LOCK_DIR.iterdir())
    calibration.load_calibration(REAL_LIMITS_CSV, REAL_LOCK_DIR, REAL_WIRING_MAP)
    assert hashlib.sha256(REAL_LIMITS_CSV.read_bytes()).hexdigest() == before
    assert sorted(p.name for p in REAL_LOCK_DIR.iterdir()) == locks_before


def test_the_loaded_calibration_is_frozen(real_cal):
    with pytest.raises(Exception):
        real_cal.mirror_offset_deg = 6
    with pytest.raises(Exception):
        real_cal.joint(0).min_deg = 0


# ==========================================================================
# clamp()
# ==========================================================================


def test_clamp_holds_a_command_inside_the_accepted_envelope(real_cal):
    j0 = real_cal.joint(0)
    assert j0.clamp(64) == (64, False)
    assert j0.clamp(0) == (29, True)
    assert j0.clamp(200) == (110, True)
    assert j0.clamp(29) == (29, False)
    assert j0.clamp(110) == (110, False)


def test_clamp_rounds_before_it_compares(real_cal):
    j0 = real_cal.joint(0)
    assert j0.clamp(63.6) == (64, False)
    assert j0.clamp(28.4) == (29, True)


# ==========================================================================
# CalibrationError
# ==========================================================================


def test_calibration_error_is_its_own_exception_type(calibration):
    assert issubclass(calibration.CalibrationError, Exception)
    assert not issubclass(calibration.CalibrationError, (ValueError, OSError, KeyError))


# ==========================================================================
# H1 -- the file itself
# ==========================================================================


def test_a_missing_limits_file_is_refused_rather_than_defaulted(calibration, tmp_path):
    # SERIAL-PROTOCOL.md section 10 says a missing file falls back to 70-110 with
    # an amber badge. A headless adapter has nowhere to show amber, and a silent
    # 70-110 would clamp every command into a 40-degree window and present as a
    # hardware fault. So it raises instead.
    with pytest.raises(calibration.CalibrationError, match="not found"):
        calibration.load_calibration(tmp_path / "absent.csv", REAL_LOCK_DIR, REAL_WIRING_MAP)


def test_a_missing_required_column_is_refused(calibration, limits, tmp_limits_path):
    path = limits().rename_column("calibrated", "cal_status").write(tmp_limits_path)
    with pytest.raises(calibration.CalibrationError, match="H1") as excinfo:
        calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)
    assert "calibrated" in str(excinfo.value)


def test_a_file_with_no_data_rows_is_refused(calibration, tmp_path):
    path = tmp_path / "joint-limits.csv"
    path.write_text("# only comments\n\n#and blanks\n", encoding="utf-8")
    with pytest.raises(calibration.CalibrationError, match="no data rows"):
        calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)


def test_a_non_integer_joint_id_is_refused(calibration, limits, tmp_limits_path):
    path = limits().set(0, "joint_id", "zero").write(tmp_limits_path)
    with pytest.raises(calibration.CalibrationError, match="H1"):
        calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)


def test_a_non_integer_numeric_field_is_refused(calibration, limits, tmp_limits_path):
    path = limits().set(3, "min_deg", "low").write(tmp_limits_path)
    with pytest.raises(calibration.CalibrationError, match="non-integer"):
        calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)


def test_a_missing_joint_row_is_refused(calibration, limits, tmp_limits_path):
    path = limits().drop_joint(5).write(tmp_limits_path)
    with pytest.raises(calibration.CalibrationError, match="no row for joint"):
        calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)


def test_one_malformed_row_rejects_the_whole_file(calibration, limits, tmp_limits_path):
    # Never a partial load: half an envelope is worse than none.
    path = limits().append_raw_line("0,Base,D3").write(tmp_limits_path)
    with pytest.raises(calibration.CalibrationError, match="expected 11"):
        calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)


def test_a_field_containing_a_comma_is_refused(calibration, limits, tmp_limits_path):
    # The arm console splits this file on commas, so a quoted comma would make the
    # two readers disagree about the same file.
    edited = limits().set(0, "notes", "locked 2026-08-05, restored 2026-08-06")
    path = edited.write(tmp_limits_path, allow_comma_in_field=True)
    with pytest.raises(calibration.CalibrationError, match="contains a comma"):
        calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)


def test_comment_and_blank_lines_are_skipped(real_cal):
    # The committed file is mostly a `#` preamble; loading it at all proves this,
    # and every failure-path test above carries that preamble through verbatim.
    assert REAL_LIMITS_CSV.read_text(encoding="utf-8-sig").startswith("#")
    assert len(real_cal.joints) == 6


# ==========================================================================
# H2 -- the joint id set
# ==========================================================================


def test_a_row_for_the_reserved_joint_is_refused(calibration, limits, tmp_limits_path):
    # D5 is the shoulder pair's second servo. A row for it means the file was
    # written against a different mental model of this arm.
    path = limits().copy_joint_row(3, 2).write(tmp_limits_path)
    with pytest.raises(calibration.CalibrationError, match="H2") as excinfo:
        calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)
    assert "not addressable" in str(excinfo.value)
    assert "D5" in str(excinfo.value)


def test_an_unknown_joint_id_is_refused(calibration, limits, tmp_limits_path):
    path = limits().copy_joint_row(3, 7).write(tmp_limits_path)
    with pytest.raises(calibration.CalibrationError, match="unknown joint_id 7"):
        calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)


def test_a_duplicate_joint_row_is_refused(calibration, limits, tmp_limits_path):
    path = limits().duplicate_joint_row(3).write(tmp_limits_path)
    with pytest.raises(calibration.CalibrationError, match="duplicate row for joint 3"):
        calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)


# ==========================================================================
# H3 -- everything the firmware would reject, rejected here first
# ==========================================================================


@pytest.mark.parametrize(
    "column, value, fragment",
    [
        ("max_deg", "181", "outside 0-180"),
        ("min_deg", "66", "not min<max"),          # joint 3 max is 66
        ("max_deg", "0", "not min<max"),           # joint 3 min is 0
        ("home_deg", "99", "outside its own range"),
        ("max_deg_per_sec", "0", "max_deg_per_sec"),
        ("max_deg_per_sec", "91", "max_deg_per_sec"),
        ("calibrated", "maybe", "expected yes or no"),
    ],
)
def test_values_the_firmware_would_reject_are_refused_first(
    calibration, limits, tmp_limits_path, column, value, fragment
):
    path = limits().set(3, column, value).write(tmp_limits_path)
    with pytest.raises(calibration.CalibrationError, match="H3") as excinfo:
        calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)
    assert fragment in str(excinfo.value)


def test_an_inverted_range_is_refused(calibration, limits, tmp_limits_path):
    # Joint 5 is 31-178, so it is the one row where max can be dropped BELOW min
    # without first hitting the 0-180 bound. `min<max` is checked before the span
    # floor, so this reports the inversion rather than a small span.
    path = limits().set(5, "max_deg", "20").set(5, "home_deg", "20").write(tmp_limits_path)
    with pytest.raises(calibration.CalibrationError, match="H3") as excinfo:
        calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)
    assert "range 31-20" in str(excinfo.value)
    assert "not min<max" in str(excinfo.value)


def test_a_span_below_the_firmwares_floor_is_refused(calibration, limits, tmp_limits_path):
    path = limits().set(3, "min_deg", "60").set(3, "max_deg", "63").set(
        3, "home_deg", "61"
    ).write(tmp_limits_path)
    with pytest.raises(calibration.CalibrationError, match="LIM_MIN_SPAN_DEG"):
        calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)


def test_the_firmware_mirrored_constants_match_the_ino(calibration):
    assert calibration.LIM_MIN_SPAN_DEG == 5
    assert (calibration.DPS_MIN, calibration.DPS_MAX) == (1, 90)
    assert calibration.MIR_OFF_MAX_DEG == 90
    assert (calibration.ANGLE_MIN, calibration.ANGLE_MAX) == (0, 180)


@pytest.mark.parametrize("value", ["mirrored", "flipped", "yes"])
def test_an_unknown_mirror_mode_is_refused(calibration, limits, tmp_limits_path, value):
    path = limits().set(1, "mirror_mode", value).write(tmp_limits_path)
    with pytest.raises(calibration.CalibrationError, match="mirror_mode"):
        calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)


def test_a_mirror_offset_beyond_the_firmware_cap_is_refused(
    calibration, limits, tmp_limits_path
):
    path = limits().set(1, "mirror_offset_deg", "91").write(tmp_limits_path)
    with pytest.raises(calibration.CalibrationError, match="exceeds"):
        calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)


def test_a_non_integer_mirror_offset_is_refused(calibration, limits, tmp_limits_path):
    path = limits().set(1, "mirror_offset_deg", "six").write(tmp_limits_path)
    with pytest.raises(calibration.CalibrationError, match="not an integer"):
        calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)


def test_an_unknown_mirror_offset_source_is_refused(calibration, limits, tmp_limits_path):
    edited = limits().add_column("mirror_offset_source", "placeholder")
    path = edited.set(1, "mirror_offset_source", "guessed").write(tmp_limits_path)
    with pytest.raises(calibration.CalibrationError, match="mirror_offset_source"):
        calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)


# ==========================================================================
# H4 -- a calibration claimed with no receipt behind it
# ==========================================================================


def test_a_calibrated_joint_with_no_lock_artifact_is_refused(
    calibration, empty_lock_dir
):
    # THE J0-REGRESSION DETECTOR. Known friction, and the friction is the point:
    # a fresh lock lands in the browser's Downloads folder, so this fails until
    # someone copies it into Calibration_Notes/lock-artifacts/.
    with pytest.raises(calibration.CalibrationError, match="H4") as excinfo:
        calibration.load_calibration(REAL_LIMITS_CSV, empty_lock_dir, REAL_WIRING_MAP)
    message = str(excinfo.value)
    assert "joint 0" in message
    assert "Downloads" in message


def test_h4_names_the_joint_that_claims_the_calibration(
    calibration, limits, tmp_limits_path, empty_lock_dir
):
    edited = limits()
    for jid in (0, 1, 3, 4):
        edited.set(jid, "calibrated", "no")
    path = edited.write(tmp_limits_path)          # only joint 5 still claims one
    with pytest.raises(calibration.CalibrationError, match="H4") as excinfo:
        calibration.load_calibration(path, empty_lock_dir, REAL_WIRING_MAP)
    assert "joint 5" in str(excinfo.value)


def test_an_uncalibrated_joint_needs_no_receipt(
    calibration, limits, tmp_limits_path, empty_lock_dir
):
    # Every joint, explicitly. This used to set only 0/1/3/4/5 and rely on J6
    # already being calibrated=no in the real file -- so it broke the moment the
    # operator locked the gripper. A validator test must not depend on the arm's
    # live state; construct the state you are testing.
    edited = limits()
    for jid in (0, 1, 3, 4, 5, 6):
        edited.set(jid, "calibrated", "no")
    path = edited.write(tmp_limits_path)
    cal = calibration.load_calibration(path, empty_lock_dir, REAL_WIRING_MAP)
    assert cal.uncalibrated_ids() == (0, 1, 3, 4, 5, 6)
    assert all("no_lock_artifact" in cal.joint(j).flags for j in cal.uncalibrated_ids())


# ==========================================================================
# H5 -- an envelope edit cannot create travel
# ==========================================================================


def test_widening_an_envelope_against_its_own_receipt_is_refused(
    calibration, limits, tmp_limits_path
):
    # This is the 2026-08-05 incident, replayed exactly: joint 0 widened from its
    # locked 29-110 back to the servo's full electrical range.
    path = limits().set(0, "min_deg", "0").set(0, "max_deg", "180").write(tmp_limits_path)
    with pytest.raises(calibration.CalibrationError, match="H5") as excinfo:
        calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)
    message = str(excinfo.value)
    assert "29-110" in message
    assert "calibration-row-J0_2026-08-05_15-36.csv" in message
    assert "do not assume the wider one is right" in message


def test_narrowing_an_envelope_against_its_own_receipt_is_also_refused(
    calibration, limits, tmp_limits_path
):
    # The rule is not "no widening" but "no edit without a new receipt". A quiet
    # narrowing is the same class of unbacked claim.
    path = limits().set(0, "min_deg", "40").set(0, "max_deg", "100").write(tmp_limits_path)
    with pytest.raises(calibration.CalibrationError, match="H5"):
        calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)


def test_h5_cannot_be_dodged_by_dropping_the_calibrated_claim(
    calibration, limits, tmp_limits_path
):
    # Flipping calibrated to no silences H4 but NOT H5 -- the receipt still exists
    # and still disagrees.
    path = (
        limits()
        .set(0, "min_deg", "0")
        .set(0, "max_deg", "180")
        .set(0, "calibrated", "no")
        .write(tmp_limits_path)
    )
    with pytest.raises(calibration.CalibrationError, match="H5"):
        calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)


def test_the_loader_offers_no_switch_that_accepts_a_disagreement(calibration):
    # There is no allow=, force= or override= on the loader, so no caller can opt
    # out of H4/H5. Widening is not a parameter; it is a new receipt.
    params = list(inspect.signature(calibration.load_calibration).parameters)
    assert params == ["limits_csv", "lock_dir", "wiring_map"]
    for name in params:
        assert not any(w in name for w in ("allow", "force", "override", "skip"))


def test_a_new_receipt_is_the_only_thing_that_widens_an_envelope(
    calibration, limits, tmp_limits_path, copied_lock_dir
):
    # The positive half of H5: the same widening that was refused above is
    # accepted once -- and only once -- a newer lock artifact backs it.
    write_lock_artifact(
        copied_lock_dir,
        0,
        date="2026-08-07",
        hhmm="09-00",
        min_deg=0,
        max_deg=180,
        home_deg=64,
    )
    path = limits().set(0, "min_deg", "0").set(0, "max_deg", "180").write(tmp_limits_path)
    cal = calibration.load_calibration(path, copied_lock_dir, REAL_WIRING_MAP)
    j0 = cal.joint(0)
    assert (j0.min_deg, j0.max_deg) == (0, 180)
    assert j0.lock_source == "calibration-row-J0_2026-08-07_09-00.csv"
    assert j0.superseded_locks == 1
    # And the widened row immediately announces what it now is.
    assert "full_electrical_range" in j0.flags


def test_deleting_a_receipt_and_dropping_the_claim_downgrades_rather_than_hides(
    calibration, limits, tmp_limits_path, copied_lock_dir, tmp_path
):
    # The honest boundary of the guarantee, pinned so nobody overstates it: H5
    # only fires while a receipt exists. Delete the receipt AND drop the
    # calibrated claim and the wider envelope does load -- but it loads as
    # explicitly UNCALIBRATED, with no_lock_artifact and full_electrical_range
    # flags, in uncalibrated_ids(), and blocked from being enabled by default.
    (copied_lock_dir / "calibration-row-J0_2026-08-05_15-36.csv").unlink()
    path = (
        limits()
        .set(0, "min_deg", "0")
        .set(0, "max_deg", "180")
        .set(0, "calibrated", "no")
        .write(tmp_limits_path)
    )
    cal = calibration.load_calibration(path, copied_lock_dir, REAL_WIRING_MAP)
    j0 = cal.joint(0)
    assert (j0.min_deg, j0.max_deg) == (0, 180)
    assert j0.calibrated is False
    assert 0 in cal.uncalibrated_ids()
    for expected in ("uncalibrated", "no_lock_artifact", "full_electrical_range"):
        assert expected in j0.flags
    assert calibration.enable_blockers(cal, 0) != []


# ==========================================================================
# Lock artifacts -- the receipts themselves
# ==========================================================================


def test_an_unrecognised_file_in_the_lock_directory_is_refused(
    calibration, tmp_path
):
    lock_dir = tmp_path / "locks"
    lock_dir.mkdir()
    (lock_dir / "calibration-row-J9.csv").write_text("whatever\n", encoding="utf-8")
    with pytest.raises(calibration.CalibrationError, match="Refusing to guess"):
        calibration.load_calibration(REAL_LIMITS_CSV, lock_dir, REAL_WIRING_MAP)


def test_a_receipt_whose_filename_disagrees_with_its_row_is_refused(
    calibration, tmp_path
):
    lock_dir = tmp_path / "locks"
    lock_dir.mkdir()
    write_lock_artifact(
        lock_dir, 0, date="2026-08-07", hhmm="09-00",
        min_deg=29, max_deg=110, home_deg=64, row_joint_id=3,
    )
    with pytest.raises(calibration.CalibrationError, match="filename says joint 0"):
        calibration.load_calibration(REAL_LIMITS_CSV, lock_dir, REAL_WIRING_MAP)


def test_a_receipt_with_the_wrong_column_count_is_refused(calibration, tmp_path):
    lock_dir = tmp_path / "locks"
    lock_dir.mkdir()
    write_lock_artifact(
        lock_dir, 0, date="2026-08-07", hhmm="09-00",
        min_deg=29, max_deg=110, home_deg=64, columns=10,
    )
    with pytest.raises(calibration.CalibrationError, match="11 columns"):
        calibration.load_calibration(REAL_LIMITS_CSV, lock_dir, REAL_WIRING_MAP)


def test_a_receipt_with_more_than_one_row_is_refused(calibration, tmp_path):
    lock_dir = tmp_path / "locks"
    lock_dir.mkdir()
    write_lock_artifact(
        lock_dir, 0, date="2026-08-07", hhmm="09-00",
        min_deg=29, max_deg=110, home_deg=64, extra_rows=1,
    )
    with pytest.raises(calibration.CalibrationError, match="got 2 row"):
        calibration.load_calibration(REAL_LIMITS_CSV, lock_dir, REAL_WIRING_MAP)


def test_a_missing_lock_directory_is_treated_as_no_receipts(
    calibration, limits, tmp_limits_path, tmp_path
):
    edited = limits()
    for jid in (0, 1, 3, 4, 5, 6):
        edited.set(jid, "calibrated", "no")
    path = edited.write(tmp_limits_path)
    cal = calibration.load_calibration(path, tmp_path / "nope", REAL_WIRING_MAP)
    assert all(cal.joint(j).lock_source is None for j in (0, 1, 3, 4, 5, 6))


# ==========================================================================
# H7 -- the mirrored image must stay inside 0-180
# ==========================================================================


def test_any_positive_offset_at_the_locked_shoulder_range_is_refused(
    calibration, limits, tmp_limits_path
):
    # 0-91 mirrors to 89-180 at offset 0 -- legal, with zero margin at the top.
    # One positive degree pushes it to 182 and the firmware would answer E11.
    path = limits().set(1, "mirror_offset_deg", "1").write(tmp_limits_path)
    with pytest.raises(calibration.CalibrationError, match="H7") as excinfo:
        calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)
    assert "escapes 0-180" in str(excinfo.value)


def test_the_most_negative_legal_offset_is_accepted(
    calibration, limits, tmp_limits_path
):
    path = limits().set(1, "mirror_offset_deg", "-44").write(tmp_limits_path)
    cal = calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)
    assert cal.mirror_offset_deg == -44


def test_one_degree_past_the_negative_edge_is_refused(
    calibration, limits, tmp_limits_path
):
    path = limits().set(1, "mirror_offset_deg", "-45").write(tmp_limits_path)
    with pytest.raises(calibration.CalibrationError, match="H7"):
        calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)


def test_an_offset_is_ignored_when_the_relation_is_not_inverted(
    calibration, limits, tmp_limits_path
):
    path = limits().set(1, "mirror_mode", "same").set(
        1, "mirror_offset_deg", "40"
    ).write(tmp_limits_path)
    cal = calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)
    assert cal.mirror_mode == "same"
    assert cal.mirror_legal_offset_window == (0, 0)


# ==========================================================================
# enable_blockers -- the gate checked before ENA, not at load time
# ==========================================================================


def test_an_unmeasured_mirror_offset_blocks_the_shoulder(calibration, real_cal):
    blockers = calibration.enable_blockers(real_cal, 1)
    assert len(blockers) == 1
    assert "never been measured" in blockers[0]
    assert "allow_unmeasured_mirror=True" in blockers[0]


def test_the_shoulder_can_be_enabled_only_by_accepting_that_risk_explicitly(
    calibration, real_cal
):
    assert calibration.enable_blockers(real_cal, 1, allow_unmeasured_mirror=True) == []


def test_a_measured_mirror_offset_removes_the_blocker(
    calibration, limits, tmp_limits_path
):
    edited = limits().add_column("mirror_offset_source", "placeholder")
    path = edited.set(1, "mirror_offset_source", "measured").write(tmp_limits_path)
    cal = calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)
    assert cal.mirror_offset_measured is True
    assert calibration.enable_blockers(cal, 1) == []
    assert not any("PLACEHOLDER" in w for w in cal.warnings)


def test_an_uncalibrated_joint_is_blocked(
    calibration, limits, tmp_limits_path, copied_lock_dir
):
    """Built, not borrowed.

    This used to run against the real J6, which was the arm's only uncalibrated
    joint. The operator locked it on 2026-08-06 and `uncalibrated_ids()` is now
    empty, so there is no longer a real joint to point at -- and the guard still
    has to be tested. Construct one.
    """
    path = (
        limits()
        .set(6, "calibrated", "no")
        .set(6, "min_deg", "0")
        .set(6, "max_deg", "180")
        .write(tmp_limits_path)
    )
    # J6's receipt is REMOVED rather than the whole directory emptied. Two guards
    # already caught cheaper versions of this fixture and both were right:
    #   - REAL_LOCK_DIR + a 0-180 J6 row tripped H5, because J6's genuine 10-70
    #     receipt contradicts it. That is precisely the 2026-08-05 widening.
    #   - an EMPTY dir tripped H4 on joint 0, because 0/1/3/4/5 really are
    #     calibrated=yes and deleting their receipts makes them lie.
    # An uncalibrated joint on an otherwise-honest arm is the state under test.
    (copied_lock_dir / "calibration-row-J6_2026-08-06_14-53.csv").unlink()
    cal = calibration.load_calibration(path, copied_lock_dir, REAL_WIRING_MAP)
    blockers = calibration.enable_blockers(cal, 6)
    assert len(blockers) == 1
    assert "UNCALIBRATED" in blockers[0]
    assert "electrical range, not measured travel" in blockers[0]


def test_an_uncalibrated_joint_can_be_enabled_only_by_accepting_that_risk(
    calibration, limits, tmp_limits_path, copied_lock_dir
):
    path = (
        limits()
        .set(6, "calibrated", "no")
        .set(6, "min_deg", "0")
        .set(6, "max_deg", "180")
        .write(tmp_limits_path)
    )
    (copied_lock_dir / "calibration-row-J6_2026-08-06_14-53.csv").unlink()
    cal = calibration.load_calibration(path, copied_lock_dir, REAL_WIRING_MAP)
    assert calibration.enable_blockers(cal, 6) != []
    assert calibration.enable_blockers(cal, 6, allow_uncalibrated_joints=True) == []


@pytest.mark.parametrize("joint_id", [0, 3, 4, 5, 6])
def test_a_calibrated_non_shoulder_joint_has_no_blockers(calibration, real_cal, joint_id):
    assert calibration.enable_blockers(real_cal, joint_id) == []


def test_an_unknown_mirror_relation_blocks_the_shoulder_with_no_way_out(
    calibration, limits, tmp_limits_path
):
    # H6 is the firmware's own guard -- ENA 1 answers E13 while MIR is UNKNOWN --
    # and unlike the other two guards it has NO opt-out here, because the board
    # will refuse the command regardless of what the host thinks.
    path = limits().set(1, "mirror_mode", "").write(tmp_limits_path)
    cal = calibration.load_calibration(path, REAL_LOCK_DIR, REAL_WIRING_MAP)
    assert cal.mirror_mode == "unknown"

    blockers = calibration.enable_blockers(cal, 1)
    assert any("H6" in b and "E13" in b for b in blockers)

    forced = calibration.enable_blockers(
        cal, 1, allow_unmeasured_mirror=True, allow_uncalibrated_joints=True
    )
    assert any("H6" in b for b in forced)


def test_the_enable_gate_risk_switches_are_keyword_only(calibration):
    params = inspect.signature(calibration.enable_blockers).parameters
    assert params["allow_unmeasured_mirror"].kind is inspect.Parameter.KEYWORD_ONLY
    assert params["allow_uncalibrated_joints"].kind is inspect.Parameter.KEYWORD_ONLY
    assert params["allow_unmeasured_mirror"].default is False
    assert params["allow_uncalibrated_joints"].default is False


# ==========================================================================
# The wiring map is provenance, not a dependency
# ==========================================================================


def test_a_missing_wiring_map_is_not_fatal(calibration, tmp_path):
    cal = calibration.load_calibration(
        REAL_LIMITS_CSV, REAL_LOCK_DIR, tmp_path / "no-wiring-map.csv"
    )
    assert cal.joint(0).servo_type == "unknown"
    assert cal.joint(0).servo_type_source == "unknown"


def test_the_wiring_map_supplies_the_servo_type(real_cal):
    assert real_cal.joint(0).servo_type == "MG995/MG996R"
    assert real_cal.joint(5).servo_type == "MG90S"


def test_the_reserved_servos_wiring_row_is_never_consumed(calibration, real_cal, tmp_path):
    # wiring-map.csv has SEVEN servo rows (index 0..6) while the loader has six
    # logical joints -- index 2 is the shoulder pair's second servo, D5. Nothing
    # ever reads it, so deleting that row changes no joint's servo type.
    data = [
        line for line in REAL_WIRING_MAP.read_text(encoding="utf-8-sig").splitlines()
        if line.strip()
    ]
    assert len(data) == 8                       # one header + seven servos
    assert data[3].startswith("2,")             # the D5 row
    trimmed = tmp_path / "wiring-map.csv"
    trimmed.write_text(
        "\n".join([data[0]] + [line for line in data[1:] if not line.startswith("2,")])
        + "\n",
        encoding="utf-8",
    )
    cal = calibration.load_calibration(REAL_LIMITS_CSV, REAL_LOCK_DIR, trimmed)
    for jid in (0, 1, 3, 4, 5, 6):
        assert cal.joint(jid).servo_type == real_cal.joint(jid).servo_type
        assert cal.joint(jid).servo_type != "unknown"
