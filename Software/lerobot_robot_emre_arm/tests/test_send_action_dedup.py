"""The MOV dedup decision, and the state rule that keeps it honest.

`send_action()` used to emit six MOVs per control tick against its own
docstring. It now sends one only when a target CHANGES. That is a rate fix with
a dataset-integrity contract attached, and this file is the contract:

    for a joint whose target did NOT change, send_action records the value the
    BOARD ACCEPTED when that target was set -- never the request (the firmware
    may have clamped it, and recording the request teaches a policy that an
    unreachable angle is reachable), and never NaN (NaN means "not driven this
    tick", and a joint holding a target is very much still being driven).

WHY THE SOURCE IS EXTRACTED RATHER THAN IMPORTED. emre_arm.py imports lerobot,
which is not installed on this host, so the module cannot be loaded at all --
which is exactly why the rest of the suite could not reach this logic.
`_plan_joint` and `_AcceptedTarget` are pure: no I/O, no board state, no lerobot.
So they are lifted out of the real file by ast and executed here. This tests the
SHIPPED source, not a copy; editing or deleting either symbol fails this file.

The half that cannot be extracted -- send_action's loop and get_observation's
reconciliation -- is asserted structurally at the bottom instead, and that
limitation is stated rather than papered over.
"""

from __future__ import annotations

import ast
import math
from dataclasses import dataclass
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[1] / "src" / "lerobot_robot_emre_arm" / "emre_arm.py"

WANTED = ("_AcceptedTarget", "_plan_joint")


def _extract():
    """Exec just the two pure symbols out of emre_arm.py."""
    text = SRC.read_text(encoding="utf-8")
    tree = ast.parse(text)
    picked = [
        n
        for n in tree.body
        if isinstance(n, (ast.ClassDef, ast.FunctionDef)) and n.name in WANTED
    ]
    missing = set(WANTED) - {n.name for n in picked}
    if missing:
        raise AssertionError(
            f"{sorted(missing)} is gone from emre_arm.py. The MOV dedup contract "
            "cannot be checked, which means it is not being kept."
        )
    # `from __future__ import annotations` is prepended so annotations stay
    # strings, exactly as they do in the real module -- otherwise `Any` and the
    # `X | None` forms would have to be resolved at exec time and this harness
    # would start needing imports the tested code does not.
    future = ast.ImportFrom(
        module="__future__", names=[ast.alias(name="annotations", asname=None)], level=0
    )
    mod = ast.fix_missing_locations(ast.Module(body=[future, *picked], type_ignores=[]))
    ns: dict = {"math": math, "dataclass": dataclass}
    exec(compile(mod, str(SRC), "exec"), ns)
    return ns["_AcceptedTarget"], ns["_plan_joint"]


AcceptedTarget, plan_joint = _extract()


def held(sent_deg: int, accepted_deg: float, clamped: bool = False):
    return AcceptedTarget(sent_deg, accepted_deg, clamped)


# --- the four rows of the decision table ----------------------------------

def test_nothing_asked_of_this_joint_sends_nothing_and_records_nothing():
    assert plan_joint(None, None) == (None, None)
    assert plan_joint(None, held(64, 64.0)) == (None, None)


def test_a_nan_request_sends_nothing_even_when_a_target_is_held():
    assert plan_joint(float("nan"), None) == (None, None)
    assert plan_joint(float("nan"), held(64, 64.0)) == (None, None)


def test_a_changed_target_sends_a_mov_and_caches_nothing_yet():
    send, keep = plan_joint(95.0, held(64, 64.0))
    assert send == 95
    assert keep is None, "the ack is the only thing allowed to fill the cache"


def test_the_first_command_to_a_joint_always_sends():
    send, keep = plan_joint(64.0, None)
    assert send == 64 and keep is None


# --- the row that carries the whole contract -------------------------------

def test_an_unchanged_target_sends_no_mov_and_replays_what_the_board_accepted():
    entry = held(sent_deg=95, accepted_deg=91.0, clamped=True)
    send, keep = plan_joint(95.0, entry)

    assert send is None, "an unchanged target must not put a MOV on the wire"
    assert keep is entry
    assert keep.accepted_deg == 91.0, "records what the board took"
    assert keep.accepted_deg != 95.0, "never the request -- the firmware clamped it"
    assert not math.isnan(keep.accepted_deg), "never NaN -- the joint is still held"
    assert keep.clamped is True, "the clamp flag survives the tick that sent nothing"


def test_the_clamped_flag_persists_across_many_silent_ticks():
    entry = held(sent_deg=95, accepted_deg=91.0, clamped=True)
    for _ in range(50):
        send, entry = plan_joint(95.0, entry)
        assert send is None
        assert entry.accepted_deg == 91.0 and entry.clamped is True


# --- integer degrees are the wire, and the comparison must live there ------

def test_the_comparison_is_on_the_rounded_integer_actually_sent():
    """The wire is integer degrees. Centidegrees exist only inside the firmware."""
    entry = held(64, 64.0)
    assert plan_joint(64.4, entry)[0] is None, "64.4 rounds to the 64 already sent"
    # Python rounds halves to even, so BOTH neighbouring halves land on 64 and
    # neither is a new target. Pinned deliberately: a future switch to
    # round-half-up would make 64.5 send a MOV, and the wire would disagree with
    # this cache about what 64.5 means.
    assert plan_joint(63.5, entry)[0] is None, "63.5 -> 64, already sent"
    assert plan_joint(64.5, entry)[0] is None, "64.5 -> 64, already sent"
    assert plan_joint(65.4, entry)[0] == 65, "a genuinely different degree sends"
    assert plan_joint(65.5, entry)[0] == 66, "65.5 -> 66, half to even upward"


def test_a_float_that_rounds_to_the_held_degree_is_not_a_new_target():
    entry = held(90, 90.0)
    for near in (89.5001, 89.9, 90.0, 90.4999):
        assert plan_joint(near, entry)[0] is None, f"{near} is still 90 on the wire"


def test_the_sent_value_is_an_int_so_it_can_be_formatted_onto_the_wire():
    send, _ = plan_joint(64.7, None)
    assert isinstance(send, int) and send == 65


# --- structural: the cache must never outlive the enablement it describes ---

def _emre_source() -> str:
    return SRC.read_text(encoding="utf-8")


@pytest.mark.parametrize("site", ["disconnect", "_check_latch", "get_observation"])
def test_every_site_that_drops_a_joint_also_drops_its_cached_target(site: str):
    """A stale `_targets` entry survives re-enablement and skips the first MOV.

    After a joint is dropped, it is re-enabled with a FRESHLY estimated adopt
    angle -- the shaft is somewhere new. If a cached target happens to equal the
    next request, `_plan_joint` returns (None, held) and the joint is never
    actually commanded, while the action column reports a confident angle. So
    `_enabled` and `_targets` must always be dropped together.
    """
    tree = ast.parse(_emre_source())
    fn = next(
        (n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == site),
        None,
    )
    assert fn is not None, f"{site}() is gone from emre_arm.py"
    body = ast.unparse(fn)
    drops_enabled = "_enabled.clear()" in body or "_enabled.discard" in body
    assert drops_enabled, f"{site}() no longer drops joints -- retarget this test"
    assert "_targets.clear()" in body or "_targets.pop" in body, (
        f"{site}() drops a joint from _enabled without dropping its cached "
        "target. A re-enable at a fresh adopt angle can then skip its first MOV."
    )


def test_get_observation_reconciles_enabled_against_the_boards_EN_field():
    """The idle watcher detaches every joint without telling EmreArm.

    transport.SerialLink._idle_loop sends `DIS A` from its own thread after
    idle_disable_s of loop silence. Nothing raises -- try_command swallows the
    reply. Unless get_observation reconciles against `EN=`, the next
    send_action finds the joint still enabled with an unchanged target, sends no
    MOV, and writes down the angle accepted BEFORE the joint was detached: a
    finite, confident number for a joint hanging loose on a sagged arm.

    Deduplicating MOVs is what made that silent. Before the dedup the same tick
    sent a MOV and earned a loud ERR E6.
    """
    tree = ast.parse(_emre_source())
    fn = next(
        n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "get_observation"
    )
    body = ast.unparse(fn)
    assert 'EN' in body and "_enabled" in body, (
        "get_observation no longer compares the board's EN= against _enabled. "
        "An idle park will silently fabricate action values again."
    )
    assert "_enabled.discard" in body, "detached joints are not dropped from _enabled"
    assert "_targets.pop" in body, "detached joints keep their cached target"
    assert "_clamped" in body, (
        "a detached joint keeps its stale clamp flag, which asserts something "
        "nobody observed"
    )


def test_the_clamp_flag_is_only_written_from_an_ack_that_parsed():
    """`CL=` read off an ack whose `SET=` was unreadable is not evidence.

    `fields.get("CL")` returns None on an unparseable ack, which would silently
    record False and CLEAR a real clamp. Flag and target come from one ack or
    from neither.
    """
    tree = ast.parse(_emre_source())
    fn = next(
        n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "send_action"
    )
    guard = next(
        n
        for n in ast.walk(fn)
        if isinstance(n, ast.If) and "isnan(accepted)" in ast.unparse(n.test)
    )
    inside = ast.unparse(ast.Module(body=guard.body, type_ignores=[]))
    assert "_clamped[jid] = was_clamped" in inside, (
        "the clamp flag is written outside the readable-ack guard, so an "
        "unparseable ack can clear a real clamp"
    )
