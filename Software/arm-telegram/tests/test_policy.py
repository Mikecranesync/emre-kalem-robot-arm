"""The refusal table, and the assertions that make it THE table rather than A table.

WHAT A GREEN RUN HERE MEANS. The decision logic is modelled as written. Nothing
in this file has touched a serial port, a daemon, a clock that ticks by itself,
or an arm. It says nothing whatever about a motor turning.

NO REAL SLEEPS. Every clock value is a number passed into `decide()`. A suite
that waits out a 300 s TTL in wall-clock seconds is a suite nobody runs
(`arm-sim/README.md` section 9).

THE PATHS ARE INSERTED HERE, ON PURPOSE. `tests/conftest.py` belongs to another
phase and puts `arm-sim` and `arm-console` on `sys.path`, but this file must pass
on its own -- and pytest's default import mode puts `tests/` on the path, not the
package directory above it, so `import policy` needs the insert below regardless.
"""

from __future__ import annotations

import ast
import os
import re
import sys
from dataclasses import dataclass, replace

import pytest

_TESTS = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_TESTS)                      # Software/arm-telegram
_SOFTWARE = os.path.dirname(_PKG)                   # Software
_BENCH_TESTS = os.path.join(_SOFTWARE, "tests")     # cycle_poses.py lives here
_CONSOLE = os.path.join(_SOFTWARE, "arm-console")   # hold_arm.py lives here

for _p in (_PKG, _BENCH_TESTS, _CONSOLE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import policy  # noqa: E402
from policy import (ALLOWED_VERBS, DEFAULT_TTL_S, FORBIDDEN_VERBS,  # noqa: E402
                    MAX_LINE_CHARS, OUTCOME_REASONS, POSES, REASONS, ForbiddenLine,
                    State, decide, finish_motion)


# --------------------------------------------------------------------------
# The link stand-in.
#
# Duck-typed on the three attributes policy reads. It is NOT imported from
# Phase 1's `arm_link.py`: that file is being written in a parallel session, and
# a test that cannot pass until somebody else's file lands is a test that blocks
# on a thing it does not own. The two halves meet in the bot (Phase 3).
# --------------------------------------------------------------------------
@dataclass(frozen=True)
class Link:
    verdict: str = "LIVE"
    restarted: bool = False
    log_window: str = ""


LIVE = Link()
STALE = Link(verdict="STALE")
GONE = Link(verdict="NO LINK")
RESTARTED = Link(restarted=True)

#: Shaped like real `hold_arm.py` output: `log()` prefixes HH:MM:SS, the latch
#: line comes from line 168 and the re-ENA line from line 135.
LATCH_LOG = ("21:03:11 LATCHED (ES/WD) -- the arm has dropped. Clearing and re-enabling.\n"
             "21:03:11 CLR -> OK CLR\n"
             "21:03:13   re-ENA 1 1 -> OK ENA J1 SET=1\n")
LATCHED = Link(log_window=LATCH_LOG)

T_ARM = 1000.0                       # when the window was opened
T_END = T_ARM + DEFAULT_TTL_S        # when it expires
T_MID = T_ARM + 10.0                 # comfortably inside it

ARMED_STORAGE = State(armed_until=T_END, declared_pose="storage")
ARMED_PICK = State(armed_until=T_END, declared_pose="pick")
ARMED_NO_POSE = State(armed_until=T_END, declared_pose=None)
MOVING = State(armed_until=T_END, declared_pose="storage", in_flight="pick")


# --------------------------------------------------------------------------
# THE REFUSAL TABLE
# --------------------------------------------------------------------------
CASES = [
    # id,                     state,           message,          now,    link,       accept, reason
    ("empty",                 State(),         "",               T_MID, LIVE,  False, "empty_message"),
    ("whitespace only",       State(),         "   \n ",         T_MID, LIVE,  False, "empty_message"),
    ("gripper verb",          ARMED_STORAGE,   "/grip open",     T_MID, LIVE,  False, "gripper_refused"),
    ("claw named",            ARMED_STORAGE,   "close the claw", T_MID, LIVE,  False, "gripper_refused"),
    ("j6 beats the number rule", ARMED_STORAGE, "j6 open",       T_MID, LIVE,  False, "gripper_refused"),
    ("j0 named",              ARMED_STORAGE,   "j0 home",        T_MID, LIVE,  False, "j0_refused"),
    ("base named",            ARMED_STORAGE,   "/go base",       T_MID, LIVE,  False, "j0_refused"),
    ("free-form angle",       ARMED_STORAGE,   "elbow to 50",    T_MID, LIVE,  False, "number_refused"),
    ("go with a number",      ARMED_STORAGE,   "/go 45",         T_MID, LIVE,  False, "number_refused"),
    ("per-joint stop",        ARMED_STORAGE,   "/stop 3",        T_MID, LIVE,  False, "number_refused"),
    ("unknown verb",          ARMED_STORAGE,   "/dance",         T_MID, LIVE,  False, "unknown_command"),
    ("bare pose name",        ARMED_STORAGE,   "storage",        T_MID, LIVE,  False, "unknown_command"),
    ("unknown pose on go",    ARMED_STORAGE,   "/go home",       T_MID, LIVE,  False, "unknown_pose"),
    ("unknown pose on arm",   State(),         "/arm home",      T_MID, LIVE,  False, "unknown_pose"),
    ("arm with no pose",      State(),         "/arm",           T_MID, LIVE,  False, "unknown_pose"),
    ("motion while unarmed",  State(),         "/go pick",       T_MID, LIVE,  False, "not_armed"),
    ("motion after expiry",   ARMED_STORAGE,   "/go pick",       T_END, LIVE,  False, "window_expired"),
    ("motion on stale link",  ARMED_STORAGE,   "/go pick",       T_MID, STALE, False, "link_stale"),
    ("motion on no link",     ARMED_STORAGE,   "/go pick",       T_MID, GONE,  False, "link_absent"),
    ("motion after restart",  ARMED_STORAGE,   "/go pick",       T_MID, RESTARTED, False, "daemon_restarted"),
    ("motion after latch",    ARMED_STORAGE,   "/go pick",       T_MID, LATCHED,   False, "latched"),
    ("second move in flight", MOVING,          "/go pick",       T_MID, LIVE,  False, "move_in_flight"),
    ("re-arm mid transition", MOVING,          "/arm storage",   T_MID, LIVE,  False, "move_in_flight"),
    ("armed but nothing declared", ARMED_NO_POSE, "/go pick",    T_MID, LIVE,  False, "no_declared_pose"),
    ("start pose is not the path's", ARMED_PICK, "/go pick",     T_MID, LIVE,  False, "wrong_declared_pose"),
    ("arm on stale link",     State(),         "/arm storage",   T_MID, STALE, False, "link_stale"),
    ("arm on no link",        State(),         "/arm storage",   T_MID, GONE,  False, "link_absent"),
    ("arm",                   State(),         "/arm storage",   T_MID, LIVE,  True,  "armed"),
    ("arm in a group chat",   State(),         "/arm@armbot pick", T_MID, LIVE, True,  "armed"),
    ("disarm",                ARMED_STORAGE,   "/disarm",        T_MID, LIVE,  True,  "disarmed"),
    ("status live",           State(),         "/status",        T_MID, LIVE,  True,  "status"),
    ("status stale",          State(),         "/status",        T_MID, STALE, True,  "status"),
    ("status no link",        State(),         "/status",        T_MID, GONE,  True,  "status"),
    ("motion",                ARMED_STORAGE,   "/go pick",       T_MID, LIVE,  True,  "motion_accepted"),
    ("stop while unarmed",    State(),         "/stop",          T_MID, LIVE,  True,  "stopped"),
    ("stop while armed",      ARMED_STORAGE,   "/stop",          T_MID, LIVE,  True,  "stopped"),
    ("stop on stale link",    ARMED_STORAGE,   "/stop",          T_MID, STALE, True,  "stopped"),
    ("stop on no link",       State(),         "/stop",          T_MID, GONE,  True,  "stopped"),
]

IDS = [c[0] for c in CASES]


def _run(case):
    _id, state, message, now, link, _accept, _reason = case
    return decide(state, message, now, link)


@pytest.mark.parametrize("case", CASES, ids=IDS)
def test_refusal_table(case):
    _id, _state, _message, _now, _link, accept, reason = case
    d = _run(case)
    assert d.accept is accept
    assert d.reason == reason
    if not accept:
        # A refusal sends NOTHING. Not a queued line, not a "safe" one.
        assert d.lines == (), f"{_id}: a refusal emitted {d.lines}"
        assert d.phases == (), f"{_id}: a refusal emitted phases {d.phases}"
        # A refusal never forgets that a transition is still out.
        assert d.new_state.in_flight == _state.in_flight, f"{_id}: a refusal moved in_flight"


def test_the_table_covers_every_reason_decide_can_return():
    """Otherwise a reason added later lands untested and nobody notices."""
    assert {c[6] for c in CASES} == set(REASONS)


# --------------------------------------------------------------------------
# The central inversion: expiry sends nothing
# --------------------------------------------------------------------------
def test_expiry_sends_nothing_to_the_arm():
    """EST/DIS would 'fail safe' by DETACHING, and a gravity-loaded arm falls."""
    d = decide(ARMED_STORAGE, "/go pick", T_END, LIVE)
    assert d.reason == "window_expired"
    assert d.lines == ()
    assert d.phases == ()
    assert d.new_state.armed_until is None


def test_expiry_clears_permission_but_not_the_declaration():
    """Nothing moved when the clock ran out, so the operator's estimate still stands.

    A latch is the opposite case -- see the next test. Reusing one _disarm() for
    both is the instinctive mistake.
    """
    d = decide(ARMED_STORAGE, "/go pick", T_END, LIVE)
    assert d.new_state.declared_pose == "storage"


def test_a_latch_voids_the_declaration_as_well_as_the_window():
    d = decide(ARMED_STORAGE, "/go pick", T_MID, LATCHED)
    assert d.reason == "latched"
    assert d.lines == ()
    assert d.new_state.armed_until is None
    assert d.new_state.declared_pose is None
    # The log lines go back verbatim -- never paraphrased into "the arm moved".
    assert "LATCHED (ES/WD)" in d.reply
    assert "re-ENA 1 1" in d.reply


def test_a_latch_voids_the_declaration_even_when_nothing_is_armed():
    d = decide(State(declared_pose="storage"), "/status", T_MID, LATCHED)
    assert d.new_state.declared_pose is None
    assert "latched" in d.events


def test_a_daemon_restart_disarms_and_voids_the_declaration():
    """hold_arm.py:96 truncates the log at startup and re-runs ENA on every joint."""
    d = decide(ARMED_STORAGE, "/go pick", T_MID, RESTARTED)
    assert d.reason == "daemon_restarted"
    assert d.lines == ()
    assert d.new_state.armed_until is None
    assert d.new_state.declared_pose is None


@pytest.mark.parametrize("link,reason", [(STALE, "link_stale"), (GONE, "link_absent")])
def test_an_unhealthy_link_refuses_motion_rather_than_annotating_it(link, reason):
    d = decide(ARMED_STORAGE, "/go pick", T_MID, link)
    assert d.accept is False
    assert d.reason == reason
    assert d.lines == ()
    assert d.new_state.armed_until is None


def test_a_malformed_link_object_fails_closed():
    """A producer that hands over the wrong shape must refuse, not crash or allow."""

    class Broken:
        pass

    d = decide(ARMED_STORAGE, "/go pick", T_MID, Broken())
    assert d.accept is False
    assert d.reason == "link_absent"
    assert d.lines == ()


def test_a_none_log_window_does_not_crash():
    d = decide(ARMED_STORAGE, "/go pick", T_MID, Link(log_window=None))
    assert d.accept is True


# --------------------------------------------------------------------------
# The armed window
# --------------------------------------------------------------------------
def test_arming_opens_the_window_and_sends_nothing():
    d = decide(State(), "/arm storage", T_ARM, LIVE)
    assert d.accept is True
    assert d.lines == ()
    assert d.new_state.armed_until == T_ARM + DEFAULT_TTL_S
    assert d.new_state.declared_pose == "storage"
    # finding 6: the bot cannot enforce sole-writer on arm_cmd.txt, and says so.
    assert "one driver at a time" in d.reply.lower()


def test_the_ttl_is_a_parameter_not_a_clock_read():
    d = decide(State(), "/arm pick", 5.0, LIVE, ttl_s=120.0)
    assert d.new_state.armed_until == 125.0


def test_the_window_closes_at_the_boundary_not_after_it():
    one_tick_early = decide(ARMED_STORAGE, "/go pick", T_END - 0.001, LIVE)
    assert one_tick_early.reason == "motion_accepted"
    exactly_at = decide(ARMED_STORAGE, "/go pick", T_END, LIVE)
    assert exactly_at.reason == "window_expired"


def test_activity_does_not_extend_the_window():
    opened = decide(State(), "/arm storage", T_ARM, LIVE).new_state
    for msg, now in (("/status", T_ARM + 10), ("/go pick", T_ARM + 20), ("/dance", T_ARM + 30)):
        after = decide(opened, msg, now, LIVE).new_state
        assert after.armed_until == T_END, f"{msg} moved the deadline"


def test_a_completed_move_does_not_extend_the_window():
    accepted = decide(ARMED_STORAGE, "/go pick", T_MID, LIVE)
    done = finish_motion(accepted.new_state, _clean_replies())
    assert done.new_state.armed_until == T_END
    assert done.new_state.declared_pose == "pick"


def test_disarm_sends_nothing():
    d = decide(ARMED_STORAGE, "/disarm", T_MID, LIVE)
    assert d.lines == ()
    assert d.new_state.armed_until is None


def test_disarm_does_not_pretend_to_recall_a_dispatched_transition():
    d = decide(MOVING, "/disarm", T_MID, LIVE)
    assert d.new_state.in_flight == "pick"
    assert "not recalled" in d.reply


def test_arming_cannot_smuggle_a_second_transition_past_the_in_flight_guard():
    """The hole this closes: /arm built a fresh State, which reset in_flight.

    The bot must poll for updates during a 5-15 s transition, or /stop mid-move
    is decorative. So this sequence is live, not theoretical: /go pick accepted,
    five lines out, then /arm storage, then /go pick again -- a second set of
    MOVs written while the first path is still running.
    """
    rearm = decide(MOVING, "/arm storage", T_MID, LIVE)
    assert rearm.accept is False
    assert rearm.reason == "move_in_flight"
    assert rearm.new_state.in_flight == "pick"
    second = decide(rearm.new_state, "/go pick", T_MID + 1, LIVE)
    assert second.lines == ()
    assert second.reason == "move_in_flight"


def test_stop_is_the_way_out_of_a_transition_and_then_arming_works():
    """Refusing /arm mid-transition strands nobody: /stop clears in_flight."""
    stopped = decide(MOVING, "/stop", T_MID, LIVE)
    assert stopped.new_state.in_flight is None
    rearm = decide(stopped.new_state, "/arm storage", T_MID + 1, LIVE)
    assert rearm.accept is True
    assert rearm.new_state.declared_pose == "storage"


def test_an_unlisted_reason_code_cannot_be_returned():
    with pytest.raises(ForbiddenLine):
        policy._decision(False, "sounds_plausible", "copy", State())


def test_arming_is_the_recovery_path_after_a_latch():
    """The remedy for not knowing where the arm is, is a human looking at it."""
    d = decide(ARMED_STORAGE, "/arm storage", T_MID, LATCHED)
    assert d.accept is True
    assert d.new_state.declared_pose == "storage"
    assert d.new_state.armed_until == T_MID + DEFAULT_TTL_S
    assert "LATCHED (ES/WD)" in d.reply    # reported, not swallowed


# --------------------------------------------------------------------------
# Refuse, never queue
# --------------------------------------------------------------------------
def test_a_second_request_mid_transition_is_refused_and_nothing_is_queued():
    first = decide(ARMED_STORAGE, "/go pick", T_MID, LIVE)
    assert first.accept is True
    second = decide(first.new_state, "/go pick", T_MID + 1, LIVE)
    assert second.accept is False
    assert second.reason == "move_in_flight"
    # The assertion that matters is emptiness, not ordering: a queued motion
    # command asserts consent that has already expired.
    assert second.lines == ()
    assert second.phases == ()


# --------------------------------------------------------------------------
# /stop
# --------------------------------------------------------------------------
@pytest.mark.parametrize("state", [State(), ARMED_STORAGE, MOVING], ids=["unarmed", "armed", "moving"])
@pytest.mark.parametrize("link", [LIVE, STALE], ids=["live", "stale"])
def test_stop_is_accepted_everywhere_and_sends_exactly_stp(state, link):
    d = decide(state, "/stop", T_MID, link)
    assert d.accept is True
    assert d.reason == "stopped"
    assert d.lines == ("STP",)
    assert d.new_state.armed_until is None
    assert d.new_state.declared_pose is None


def test_stop_on_no_link_sends_nothing_rather_than_queueing_a_command():
    """hold_arm.py:177 consumes arm_cmd.txt whenever it is non-empty.

    With no daemon present, a line written now is a command waiting for the next
    daemon to start -- a queued command wearing a different hat.
    """
    d = decide(ARMED_STORAGE, "/stop", T_MID, GONE)
    assert d.accept is True
    assert d.lines == ()
    assert "rocker" in d.reply


def test_stop_copy_does_not_claim_to_be_an_emergency_stop():
    d = decide(ARMED_STORAGE, "/stop", T_MID, LIVE)
    assert "did not remove power" in d.reply
    assert "rocker" in d.reply


# --------------------------------------------------------------------------
# What may go on the wire
# --------------------------------------------------------------------------
def _every_emitted_line():
    lines = []
    for case in CASES:
        lines.extend(_run(case).lines)
    first = decide(ARMED_STORAGE, "/go pick", T_MID, LIVE)
    lines.extend(first.lines)
    lines.extend(decide(ARMED_PICK, "/go storage", T_MID, LIVE).lines)
    return lines


def test_no_forbidden_verb_is_ever_emitted_on_any_path():
    for ln in _every_emitted_line():
        assert ln.split()[0].upper() not in FORBIDDEN_VERBS, ln


def test_only_mov_sta_and_stp_are_ever_emitted():
    for ln in _every_emitted_line():
        assert ln.split()[0].upper() in ALLOWED_VERBS, ln


def test_no_speed_is_ever_commanded():
    """Register finding 5: the daemon holds J1 at 15 and J3/J4/J5 at 20 deg/s.

    Every proposed cap was ABOVE those, so a phone would have been speeding the
    arm up. The bot inherits the daemon's speeds and mutates nothing.
    """
    for ln in _every_emitted_line():
        assert not ln.upper().startswith("SPD"), ln


def test_every_emitted_line_fits_the_firmware_line_limit():
    for ln in _every_emitted_line():
        assert len(ln) <= MAX_LINE_CHARS, f"{ln!r} would be discarded with ERR E8 LINE"


@pytest.mark.parametrize("bad", ["EST", "DIS 1", "DIS A", "CLR"])
def test_the_guard_raises_on_a_forbidden_verb(bad):
    with pytest.raises(ForbiddenLine):
        policy._guard([bad])


def test_the_guard_raises_on_an_unlisted_verb():
    with pytest.raises(ForbiddenLine):
        policy._guard(["JOG 1 5"])


@pytest.mark.parametrize("bad", ["MOV 6 70", "MOV 6 10", "MOV 0 64", "MOV 2 90"])
def test_the_guard_raises_on_a_joint_this_surface_may_not_command(bad):
    """The wire half of "no gripper, no J0" -- HARDENING, not a bug that was found.

    Nothing in this package constructs such a line today: both recorded
    transitions move only J1/J3/J4/J5, and
    `test_no_waypoint_touches_j0_or_the_gripper` pins the DATA. But the data was
    the only guard there was -- `_guard` checked the verb and the line length and
    nothing else -- so a hand-built `Phase` or a later code path could have put
    `MOV 6 70` on the wire past every allowlist in the package.

    J6 acks and ramps correctly and its fingers do not articulate; the operator's
    hand-check diagnosis is that the gear is slipping on the motor shaft. A
    gripper command must never go out with fingers in the claw, and a phone can
    never rule that out. J0's servo is dead.
    """
    with pytest.raises(ForbiddenLine):
        policy._guard([bad])


@pytest.mark.parametrize("bad", ["MOV", "MOV x 40", "MOV -1 40"])
def test_the_guard_raises_on_a_mov_it_cannot_read_a_joint_out_of(bad):
    """Fail closed. A line whose joint cannot be parsed is not a line to send."""
    with pytest.raises(ForbiddenLine):
        policy._guard([bad])


def test_guard_line_is_the_same_allowlist_the_bot_choke_point_uses():
    """`bot._send()` calls this, and it must not be a second, drifting copy."""
    assert policy.guard_line("MOV 1 40") == "MOV 1 40"
    assert policy.guard_line("STA") == "STA"
    assert policy.guard_line("STP") == "STP"
    for bad in ("EST", "DIS 1", "CLR", "JOG 1 5", "MOV 6 70", "MOV 0 64"):
        with pytest.raises(ForbiddenLine):
            policy.guard_line(bad)


def test_the_guard_raises_on_an_over_long_line():
    with pytest.raises(ForbiddenLine):
        policy._guard(["MOV 1 40 " + "x" * MAX_LINE_CHARS])


def test_lines_is_exactly_the_flattened_phases():
    """Otherwise somebody adds a waypoint to phases and the `lines == ()` safety
    assertions quietly stop covering it."""
    d = decide(ARMED_STORAGE, "/go pick", T_MID, LIVE)
    assert tuple(ln for ph in d.phases for ln in ph.lines) == d.lines
    assert d.lines == ("MOV 1 40", "MOV 3 36", "MOV 4 140", "MOV 5 165", "MOV 1 8")


def test_the_transition_keeps_its_phase_boundaries():
    d = decide(ARMED_PICK, "/go storage", T_MID, LIVE)
    assert [ph.label for ph in d.phases] == ["lift", "neutralise", "fold"]
    assert d.phases[0].lines == ("MOV 1 40",)          # claw off the mat FIRST
    assert d.phases[-1].lines == ("MOV 1 88",)         # fold onto the base LAST


# --------------------------------------------------------------------------
# Drift guards against the proven bench tools
# --------------------------------------------------------------------------
def test_pose_constants_match_cycle_poses():
    """The bot may not import cycle_poses (it imports cv2 and numpy at top level),
    so the constants are lifted -- and a lifted constant drifts unless something
    compares them. Behavioural comparison, not textual: whitespace would false-fail.
    """
    import cycle_poses

    assert policy.PICK == cycle_poses.PICK
    assert policy.STORAGE == cycle_poses.STORAGE
    assert policy.TRANSIT_J1 == cycle_poses.TRANSIT_J1
    assert list(policy.TO_STORAGE) == cycle_poses.TO_STORAGE
    assert list(policy.TO_PICK) == cycle_poses.TO_PICK


def test_built_phases_match_the_lifted_spec():
    import cycle_poses

    built = policy._build_phases(policy.TO_PICK)
    assert [(ph.label, dict(ph.targets)) for ph in built] == cycle_poses.TO_PICK


def _all_waypoints():
    for spec in (policy.TO_STORAGE, policy.TO_PICK):
        for _label, targets in spec:
            for j, v in targets.items():
                yield j, v
    for pose in (policy.PICK, policy.STORAGE):
        for j, v in pose.items():
            yield j, v


def test_every_waypoint_sits_inside_the_daemons_own_limits():
    """Two lines here beats finding it with a motor.

    A later pose edit that pushes past one of these produces CL=1 at the bench,
    which is exactly the failure finish_motion() has to report.
    """
    import hold_arm

    for j, v in _all_waypoints():
        lo, hi = hold_arm.HOLD[j][0], hold_arm.HOLD[j][1]
        assert lo <= v <= hi, f"J{j}={v} is outside the daemon's {lo}-{hi}"


def test_no_waypoint_touches_j0_or_the_gripper():
    joints = {j for j, _v in _all_waypoints()}
    assert 0 not in joints, "J0's servo is dead and appears in no pose"
    assert 6 not in joints, "J6 acks and does not articulate; a phone cannot see the claw"


def test_the_data_guard_and_the_wire_guard_agree_on_the_joints():
    """Two halves of one rule, tied together so neither can drift alone.

    The data half is the table above. The wire half is `MOV_JOINTS`, enforced in
    `_guard`. Widening one without the other is how a rule ends up enforced in a
    place nothing reaches.
    """
    assert {j for j, _v in _all_waypoints()} <= set(policy.MOV_JOINTS)
    assert 0 not in policy.MOV_JOINTS and 6 not in policy.MOV_JOINTS


# --------------------------------------------------------------------------
# Reading the outcome from the reply, never from arm_status.txt
# --------------------------------------------------------------------------
def _clean_replies():
    return ["OK MOV J1 REQ=40 SET=40 CL=0",
            "OK MOV J3 REQ=36 SET=36 CL=0",
            "OK MOV J4 REQ=140 SET=140 CL=0",
            "OK MOV J5 REQ=165 SET=165 CL=0",
            "OK MOV J1 REQ=8 SET=8 CL=0"]


def test_a_clean_transition_moves_the_declared_pose_to_the_destination():
    moving = decide(ARMED_STORAGE, "/go pick", T_MID, LIVE).new_state
    out = finish_motion(moving, _clean_replies())
    assert out.accept is True
    assert out.reason == "motion_complete"
    assert out.lines == ()
    assert out.new_state.declared_pose == "pick"
    assert out.new_state.in_flight is None


def test_a_clamped_waypoint_is_a_failure_and_voids_the_declared_pose():
    """CL=1 means the firmware clamped it and the joint is NOT where you asked."""
    replies = _clean_replies()
    replies[2] = "OK MOV J4 REQ=140 SET=91 CL=1"
    out = finish_motion(MOVING, replies, log_window="")
    assert out.accept is False
    assert out.reason == "clamped"
    assert out.lines == ()
    assert out.new_state.declared_pose is None
    assert out.new_state.armed_until is None
    assert "SET=91 CL=1" in out.reply          # the board's own line, verbatim


def test_a_board_error_is_a_failure():
    replies = _clean_replies()
    replies[1] = "ERR E6 MOV JOINT=3"
    out = finish_motion(MOVING, replies)
    assert out.accept is False
    assert out.reason == "board_error"
    assert out.new_state.declared_pose is None
    assert "ERR E6 MOV JOINT=3" in out.reply


def test_a_latch_during_the_move_is_a_failure_however_clean_the_replies_look():
    out = finish_motion(MOVING, _clean_replies(), log_window=LATCH_LOG)
    assert out.accept is False
    assert out.reason == "latched_during_move"
    assert out.new_state.declared_pose is None
    assert "re-ENA 1 1" in out.reply


def test_finish_motion_never_emits_a_line():
    for replies, window in ((_clean_replies(), ""),
                            (["OK MOV J1 REQ=40 SET=40 CL=1"], ""),
                            (["ERR E7 MOV"], ""),
                            (_clean_replies(), LATCH_LOG)):
        assert finish_motion(MOVING, replies, window).lines == ()
    assert finish_motion(MOVING, _clean_replies(), "", True).lines == ()


def test_finish_motion_refuses_to_judge_a_move_that_was_never_started():
    with pytest.raises(ValueError):
        finish_motion(ARMED_STORAGE, _clean_replies())


def test_a_restart_during_the_move_is_a_failure_however_clean_the_replies_look():
    """The hole this closes, and NONE of the other checks could have caught it.

    A daemon that restarts mid-transition truncates its log and re-runs
    `ENA <j> <adopt>` on every joint. So: every reply collected is `CL=0`, no
    reply is an `ERR`, and the log window is a BRAND NEW log that carries no
    `LATCHED` and no `re-ENA` -- `hold_arm.main()` logs plain `ENA` on the way
    up, and only `enable_all()` logs `re-ENA`. Every existing check passes and
    the answer was `motion_complete`.

    Note the arguments below: clean replies, empty window. That is the whole
    point -- the flag is the only thing that can say so.
    """
    out = finish_motion(MOVING, _clean_replies(), log_window="", restarted=True)
    assert out.accept is False
    assert out.reason == "restarted_during_move"
    assert out.lines == ()
    assert out.new_state.declared_pose is None
    assert out.new_state.armed_until is None
    assert out.new_state.in_flight is None
    assert "adopt angles" in out.reply


def test_a_restart_beats_a_clean_log_window_and_is_judged_first():
    """Ordering, pinned. A restart is the most destructive of the four facts:
    the log carries no marker to fall back on, so anything checked before it
    would let a clean-looking transition through."""
    out = finish_motion(MOVING, _clean_replies(),
                        log_window="12:00:00 opening SIM\n12:00:02 ENA 1 1 -> OK ENA J1 SET=1\n",
                        restarted=True)
    assert out.reason == "restarted_during_move"


def test_every_outcome_reason_is_exercised():
    seen = {
        finish_motion(MOVING, _clean_replies()).reason,
        finish_motion(MOVING, ["OK MOV J1 REQ=40 SET=91 CL=1"]).reason,
        finish_motion(MOVING, ["ERR E6 MOV JOINT=3"]).reason,
        finish_motion(MOVING, _clean_replies(), LATCH_LOG).reason,
        finish_motion(MOVING, _clean_replies(), "", True).reason,
    }
    assert seen == set(OUTCOME_REASONS)


# --------------------------------------------------------------------------
# Copy discipline, and the purity of the layer
# --------------------------------------------------------------------------
#: PRD section 3. Banned harder here than anywhere else in the project, because
#: the reader cannot glance at the bench and catch the lie.
BANNED_PHRASES = ("the arm is at", "the arm is held", "the move completed",
                  "stopped for safety", "emergency stop")
BANNED_WORDS = ("actual", "measured", "feedback", "position")


def _every_reply():
    replies = [_run(case).reply for case in CASES]
    replies.append(decide(ARMED_PICK, "/go storage", T_MID, LIVE).reply)
    replies.append(decide(ARMED_STORAGE, "/arm storage", T_MID, LATCHED).reply)
    replies.append(decide(ARMED_STORAGE, "/arm pick", T_MID, RESTARTED).reply)
    replies.append(decide(MOVING, "/disarm", T_MID, LIVE).reply)
    replies.append(finish_motion(MOVING, _clean_replies()).reply)
    replies.append(finish_motion(MOVING, ["OK MOV J1 REQ=40 SET=91 CL=1"]).reply)
    replies.append(finish_motion(MOVING, ["ERR E6 MOV JOINT=3"]).reply)
    replies.append(finish_motion(MOVING, _clean_replies(), LATCH_LOG).reply)
    replies.append(finish_motion(MOVING, _clean_replies(), "", True).reply)
    return replies


def test_no_reply_claims_something_nothing_observed():
    for reply in _every_reply():
        low = reply.lower()
        for phrase in BANNED_PHRASES:
            assert phrase not in low, f"{phrase!r} in {reply!r}"
        for word in BANNED_WORDS:
            assert not re.search(rf"\b{word}\b", low), f"{word!r} in {reply!r}"


def test_every_reply_says_something():
    for reply in _every_reply():
        assert reply.strip(), "a decision with no copy is a decision the operator cannot read"


def test_policy_imports_nothing_that_could_do_io():
    """The claim 'pure, no I/O at all' is worth a deterministic check, not a promise.

    ast, not a runtime probe: an import that only fires on one branch would still
    be in the tree, and a sys.modules check is order-dependent inside a pytest
    session and can false-pass.
    """
    src = open(os.path.join(_PKG, "policy.py"), "r", encoding="utf-8").read()
    roots = set()
    for node in ast.walk(ast.parse(src)):
        if isinstance(node, ast.Import):
            roots.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            roots.add((node.module or "").split(".")[0])
    assert roots <= {"__future__", "dataclasses", "typing"}, roots


def test_policy_names_no_serial_port():
    src = open(os.path.join(_PKG, "policy.py"), "r", encoding="utf-8").read()
    assert "COM5" not in src


def test_decide_is_pure_in_the_only_way_that_matters():
    """Same arguments, same Decision -- and the state passed in is never mutated."""
    before = replace(ARMED_STORAGE)
    first = decide(ARMED_STORAGE, "/go pick", T_MID, LIVE)
    second = decide(ARMED_STORAGE, "/go pick", T_MID, LIVE)
    assert first == second
    assert ARMED_STORAGE == before


def test_the_pose_vocabulary_is_exactly_two_names():
    assert set(POSES) == {"pick", "storage"}
