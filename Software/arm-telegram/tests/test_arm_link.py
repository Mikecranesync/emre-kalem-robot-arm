"""The link client: the offset primitive, the reply cut, freshness, restart.

WHAT IS PROVEN HERE
    That the offset primitive returns THIS command's reply after prior traffic
    is seeded ahead of it; that a reply is cut at the next daemon log line and
    not at the next newline; that the verdict flips at the window boundary on
    an injected clock; that a shrunken log is caught; and that the lifted copy
    still behaves like `motion_verify.ArmLink`.

WHAT IS NOT
    Anything about a real daemon under real load, and anything whatsoever about
    the arm. The daemon here is a model of `hold_arm.py`. The board underneath
    it is a model of the firmware. No serial port is opened and no wall-clock
    second passes.

NO TEST SLEEPS. Two mechanisms, and both are deliberate:
  - the freshness clock is an ARGUMENT (`verdict(now)`), so a STALE case is a
    subtraction rather than fifteen real seconds;
  - `timeout=0.0` exercises the give-up path with the loop body never entered.
"""

from __future__ import annotations

import os

import motion_verify
import pytest

from arm_link import (
    FRESHNESS_WINDOW_S,
    LIVE,
    NO_LINK,
    STALE,
    ArmLink,
    LinkState,
    _cut_reply,
)


@pytest.fixture
def link(daemon):
    """An `ArmLink` pointed at a started, polled fake daemon."""
    return ArmLink(daemon.dir)


def pumping(daemon):
    """An `on_poll` hook. The fake daemon is synchronous, so without this
    nothing can append the reply while the test is blocked inside `send()`."""
    return daemon.pump


# ===========================================================================
# The offset primitive -- lifted, and the reason it was lifted
# ===========================================================================

def test_the_constructor_refuses_when_there_is_no_daemon_log(link_dir):
    """Lifted verbatim, `SystemExit` and all.

    The wart comes with the copy on purpose: swapping in a custom exception
    would diverge the constructor from `motion_verify.ArmLink` and weaken the
    drift guard. The bot has no fallback and no degraded mode -- if the daemon
    is not there it refuses, and this is where that starts.
    """
    with pytest.raises(SystemExit) as exc:
        ArmLink(str(link_dir))
    assert "is hold_arm.py running?" in str(exc.value)


def test_send_returns_this_commands_reply_and_not_an_identical_earlier_one(daemon, link):
    """The whole reason for the offset primitive.

    A `CMD VER -> ` line already sits in the log. Splitting the WHOLE log on the
    marker finds that one; recording the byte length first and tailing from
    there finds this call's own. The seeded line carries a payload the firmware
    would never emit, so there is no way to pass this by accident.
    """
    with open(link.log, "a", encoding="utf-8") as fh:
        fh.write("08:00:00 CMD VER -> OK VER STALE-EARLIER-TRAFFIC\n")

    reply = link.send("VER", on_poll=pumping(daemon))

    assert "STALE-EARLIER-TRAFFIC" not in reply
    assert reply.startswith("OK VER NAME=FACTORYLM-ARM")


def test_offset_and_tail_read_only_what_arrived_after_the_mark(daemon, link):
    off = link.offset()
    assert off == os.path.getsize(link.log)
    assert link.tail(off) == ""

    daemon.write_cmd("VER")
    daemon.pump()

    assert link.offset() > off
    assert "CMD VER -> " in link.tail(off)


def test_field_reads_a_board_line_and_guesses_at_nothing(daemon, link):
    reply = link.send("STA", on_poll=pumping(daemon))
    row = [ln for ln in reply.splitlines() if ln.startswith("STA J3 ")][0]

    assert link.field(row, "EN") == "1"
    assert link.field(row, "SET") == "33"
    assert link.field(row, "CL") == "", "an absent key is empty, never a guess"


def test_sta_row_keeps_the_originals_signature_and_needs_no_hook(daemon, link, monkeypatch):
    """`sta_row()` calls `send()` internally with `on_poll` left at its default.

    That is deliberate -- adding a hook parameter would diverge it from
    `motion_verify.ArmLink.sta_row` for no gain, since the REAL daemon is
    asynchronous and appends the reply on its own. Only the synchronous fake
    needs a nudge, and a test gives it one the same way the drift guard does.
    """
    import time as _time

    monkeypatch.setattr(_time, "sleep", lambda _s: daemon.pump())

    row = link.sta_row(3)
    assert row.startswith("STA J3 EN=1 SET=33 ")
    assert link.sta_row(2) == "", "J2 is reserved -- there is no row to find"


def test_a_reply_that_never_lands_raises_rather_than_returning_empty(link):
    """`timeout=0.0` -- the loop body is never entered, so this costs no time.

    A silent empty return here would be reported to the operator as a
    successful move that produced no board line.
    """
    with pytest.raises(SystemExit) as exc:
        link.send("VER", timeout=0.0)
    assert "never logged a reply" in str(exc.value)


def test_the_command_file_carries_exactly_one_line(daemon, link):
    """48-character inbound cap, one command outstanding at a time. The bot
    writes one line and the daemon truncates the file when it consumes it."""
    link.send("VER", on_poll=pumping(daemon))
    assert os.path.getsize(daemon.cmd) == 0


# ===========================================================================
# The reply cut -- plan finding 4
# ===========================================================================

def test_cut_reply_stops_at_the_next_log_line_not_the_next_newline():
    """The unit case, both halves in one assertion pair."""
    single = "OK MOV J3 REQ=64 SET=64 CL=0\n08:00:05 LATCHED (ES/WD) -- the arm has dropped.\n"
    assert _cut_reply(single) == "OK MOV J3 REQ=64 SET=64 CL=0"

    multi = "STA J0 EN=0\nSTA J1 EN=1\nSYS ES=0 WD=0\nOK STA N=6\n08:00:05 CLR -> OK CLR\n"
    assert _cut_reply(multi) == "STA J0 EN=0\nSTA J1 EN=1\nSYS ES=0 WD=0\nOK STA N=6"


def test_a_multi_line_STA_survives_the_cut_intact(daemon, link):
    """`log()` timestamps only the FIRST line of a multi-line message.

    A naive cut at the next newline would return the J0 row alone and would
    look exactly like it worked -- until somebody asked the bot about J5.
    """
    reply = link.send("STA", on_poll=pumping(daemon))

    rows = [ln for ln in reply.splitlines() if ln.startswith("STA J")]
    assert len(rows) == 6, "one row per joint"
    assert any(ln.startswith("SYS ") for ln in reply.splitlines())
    assert reply.splitlines()[-1].startswith("OK STA N=6")


def test_a_daemon_line_landing_after_the_reply_is_not_quoted_back(daemon, link):
    """The hazard finding 4 names: an unrelated daemon line inside the window.

    Here it is the real thing rather than a synthetic append -- a latch fires
    between the reply landing and the tail being read, and `enable_all()` writes
    `LATCHED` and five `re-ENA` lines straight into the window. Quoted verbatim
    into Telegram, that reads as though the board answered the move with them.

    Those lines are not lost: Phase 2 reads them from `observe().log_window`
    and disarms. They are simply not part of the board's answer to THIS command.
    """
    fired = []

    def on_poll():
        daemon.pump()
        if not fired:
            fired.append(True)
            daemon.sim.feed(b"!")  # realtime e-stop -- detaches everything
            daemon.poll()          # the health check notices and re-enables

    reply = link.send("MOV 3 64", on_poll=on_poll)

    assert reply == "OK MOV J3 REQ=64 SET=64 CL=0"
    assert "LATCHED" not in reply
    assert "re-ENA" not in reply
    assert "LATCHED" in daemon.read_log(), "the line was written, just not quoted"


# ===========================================================================
# The freshness verdict -- plan finding 2, no PID anywhere
# ===========================================================================

def test_verdict_is_LIVE_on_a_freshly_polled_status_file(daemon, link):
    assert link.verdict(daemon.now) == LIVE


@pytest.mark.parametrize("age,expected", [
    (0.0, LIVE),
    (4.9, LIVE),
    (FRESHNESS_WINDOW_S - 0.01, LIVE),
    (FRESHNESS_WINDOW_S, LIVE),
    (FRESHNESS_WINDOW_S + 0.01, STALE),
    (20.0, STALE),
    (600.0, STALE),
])
def test_the_verdict_flips_at_the_window_boundary(daemon, link, age, expected):
    """The clock is an argument, so a ten-minute-stale case costs no time.

    15 s is a DESK CHOICE, never observed under load. It is three of the
    daemon's 5 s poll periods, so two missed polls are tolerated. If a real run
    shows the file routinely older than that, this number moves -- the test is
    parametrised so moving it is one edit.
    """
    assert link.verdict(daemon.now + age) == expected


def test_verdict_reads_no_clock_of_its_own(daemon, link):
    """Two answers from one unchanged filesystem: the argument is the clock."""
    assert link.verdict(daemon.now) == LIVE
    assert link.verdict(daemon.now + 999) == STALE


def test_verdict_is_NO_LINK_before_the_daemons_first_poll(idle_daemon):
    """A bot started in the same second as the daemon refuses, and should.

    `hold_arm.py` writes `arm_status.txt` only on its 5 s poll and only
    `if sta:`. Falling back to the log's mtime here would look tempting and
    would be wrong: the daemon TRUNCATES the log on the way up, so the log is
    freshest at exactly the moment the board is least ready.
    """
    idle_daemon.start()
    link = ArmLink(idle_daemon.dir)

    assert not os.path.exists(link.status)
    assert link.verdict(idle_daemon.now) == NO_LINK

    idle_daemon.poll()
    assert link.verdict(idle_daemon.now) == LIVE


def test_verdict_is_NO_LINK_when_the_link_directory_is_wiped(daemon, link):
    """The daemon's link dir has been session-temporary before now."""
    os.remove(link.log)
    os.remove(link.status)
    assert link.verdict(daemon.now) == NO_LINK


def test_a_stale_status_file_reads_STALE_while_every_joint_has_detached(daemon, link):
    """The lie this verdict exists to catch, built rather than asserted.

    `wedge()` and not `advance()`: the loop has STOPPED, so nothing feeds the
    4000 ms watchdog either. The board latches and detaches all five joints,
    while `arm_status.txt` -- written at the last poll -- still reports `EN=1`
    on every one of them. That is the 2026-08-06 evening failure: the bench was
    powered down, the daemon died, and the status file it left behind read
    exactly like a healthy held arm.

    An earlier version of this test called `advance()` under a docstring that
    said "the loop stops". It passed -- `verdict()` only reads the mtime -- but
    the model was keeping the board fed the whole time, so the test proved the
    verdict was a function of a timestamp and nothing more, while claiming to
    prove it caught a dead daemon.
    """
    assert "EN=1" in open(link.status, encoding="utf-8").read()

    daemon.wedge(30_000)

    assert link.verdict(daemon.now) == STALE

    # What the status file says, versus what the board would say if asked.
    on_disk = open(link.status, encoding="utf-8").read()
    assert "EN=1" in on_disk and "ES=0" in on_disk
    daemon.sim.discard_out()
    daemon.sim.feed(b"STA\n")
    truth = daemon.sim.read_out().decode("ascii", "replace")
    assert "ES=1" in truth and "WD=1" in truth, "the watchdog latched"
    assert " EN=1" not in truth, "every joint detached"


# ===========================================================================
# Restart detection -- plan finding 3
# ===========================================================================

def test_a_shrunken_log_means_the_daemon_restarted(daemon, link):
    daemon.write_cmd("VER")
    daemon.pump()
    link.observe(daemon.now)
    assert link.restarted() is False

    daemon.restart()

    assert link.restarted() is True


def test_a_restart_is_caught_even_when_the_log_did_not_shrink(daemon, link):
    """THE CASE THE SHRINK TEST ALONE MISSES. Found by running it, not reading it.

    Nothing has been sent yet, so the log holds only the startup handshake --
    and a restart writes that same handshake again. Measured here: 1517 bytes
    before, 1521 after. `current_size < last_offset` is False, and a bot relying
    on it alone would keep a declared pose across a restart that had just
    snapped five joints to their adopt angles.

    This is the highest-risk window for exactly the wrong reason: it is the
    moment right after the operator armed, when he is most likely to send a
    move and least likely to have generated any traffic that would have grown
    the log.
    """
    link.observe(daemon.now)
    before = os.path.getsize(link.log)

    daemon.advance(60_000)
    daemon.restart()

    assert os.path.getsize(link.log) >= before, "this is the no-shrink case"
    assert link.restarted() is True


def test_restarted_is_a_pure_query_and_answers_the_same_twice(daemon, link):
    """A predicate that consumed the edge would tell the second caller `False`
    -- failing open in the exact path that catches the arm snapping to five
    adopt angles with nobody having asked it to."""
    daemon.write_cmd("VER")
    daemon.pump()
    link.observe(daemon.now)
    daemon.restart()

    assert link.restarted() is True
    assert link.restarted() is True
    assert link.restarted() is True


def test_observe_is_the_only_thing_that_rebaselines(daemon, link):
    daemon.write_cmd("VER")
    daemon.pump()
    link.observe(daemon.now)
    daemon.restart()

    assert link.observe(daemon.now).restarted is True
    assert link.observe(daemon.now).restarted is False, "reported once, then re-based"


def test_a_growing_log_is_not_a_restart(daemon, link):
    """No false positives while one daemon runs -- it only ever appends."""
    for _ in range(3):
        daemon.write_cmd("VER")
        daemon.pump()
        daemon.advance(6_000)
        daemon.poll()
        state = link.observe(daemon.now)
        assert state.restarted is False


def test_a_latch_recovery_is_not_mistaken_for_a_restart(daemon, link):
    """`enable_all()` re-drives every joint and writes `re-ENA` lines, which
    reads like a restart and is not one. It disarms for its own reason (Phase 2
    Step 6) and the operator is told which of the two happened."""
    link.observe(daemon.now)
    daemon.sim.feed(b"!")
    daemon.poll()

    state = link.observe(daemon.now)

    assert state.restarted is False
    assert "re-ENA" in state.log_window


def test_the_restart_window_carries_the_ENA_lines_that_moved_the_arm(daemon, link):
    """Not an empty string. After a truncation "everything since I last looked"
    IS the whole file, and those `ENA <j> <adopt>` lines are the report the
    operator needs: every joint was just re-driven to its adopt angle."""
    link.observe(daemon.now)
    daemon.restart()

    state = link.observe(daemon.now)

    assert state.restarted is True
    assert "ENA 1 1 -> " in state.log_window
    assert "ENA 6 40 -> " in state.log_window


def test_a_wiped_link_reports_NO_LINK_and_not_a_restart(daemon, link):
    """A missing log is a dead link, not a shrunken one. Both refuse, but the
    operator is told different things, and only one of them is true."""
    link.observe(daemon.now)
    os.remove(link.log)

    state = link.observe(daemon.now)

    assert state.verdict == NO_LINK
    assert state.restarted is False
    assert state.log_window == ""


# ===========================================================================
# LinkState -- the only channel into the pure policy layer
# ===========================================================================

def test_observe_hands_the_policy_layer_everything_it_cannot_re_derive(daemon, link):
    """A bare verdict string is not enough: a restart and a `LATCHED` /
    `re-ENA` line both disarm, and neither is recoverable from three words."""
    link.observe(daemon.now)
    daemon.sim.feed(b"!")
    daemon.poll()

    state = link.observe(daemon.now)

    assert isinstance(state, LinkState)
    assert state.verdict == LIVE
    assert state.restarted is False
    assert "LATCHED" in state.log_window
    assert "re-ENA" in state.log_window


def test_linkstate_has_no_fail_open_default():
    """`verdict` is required. A default of LIVE would let a half-built call
    site command the arm."""
    assert LinkState("STALE") == LinkState("STALE", False, "")
    with pytest.raises(TypeError):
        LinkState()


# ===========================================================================
# The drift guard against motion_verify.ArmLink
# ===========================================================================
#
# WHY THIS IS NOT THE SHAPE THE PLAN SPECIFIED, AND THE PLAN IS WRONG HERE.
#
# Phase 1 Step 5 says to call both copies with `on_poll` unset "against a
# pre-seeded log". That cannot close, and it was verified rather than reasoned
# about: `send()` captures `off = self.offset()` at EOF INSIDE the call and
# tails from there, so a reply seeded BEFORE the call sits behind the offset and
# is never seen. Both copies spin the full timeout and raise `SystemExit`.
#
# The intent of the step is sound -- both copies must take the SAME path, with
# `on_poll` unset on both, or the guard proves nothing. So the reply is made to
# arrive during the call by substituting `time.sleep`, which is the recipe
# `arm-sim/README.md` §7 already uses for exactly this reason. `on_poll` stays
# `None` on both copies, both enter the identical loop, and the marker lands on
# iteration two in zero real time.
# ===========================================================================

@pytest.fixture
def both(daemon, monkeypatch):
    """Two `ArmLink`s on one link directory, with `time.sleep` pumping.

    `motion_verify` and `arm_link` both do `import time`, so patching the
    module attribute covers both copies at once.
    """
    import time as _time

    monkeypatch.setattr(_time, "sleep", lambda _s: daemon.pump())
    return ArmLink(daemon.dir), motion_verify.ArmLink(daemon.dir)


def test_drift_pure_reads_agree(daemon, both):
    """`offset` / `tail` / `field` need no substitution -- they only read."""
    new, old = both
    daemon.write_cmd("VER")
    daemon.pump()

    assert new.offset() == old.offset()
    assert new.tail(0) == old.tail(0)
    assert new.tail(new.offset()) == old.tail(old.offset())

    row = "STA J3 EN=1 SET=33 TGT=33 MIN=0 MAX=66 CAL=1 DPS=20 MOV=0 JTO=0"
    for key in ("EN", "SET", "MIN", "MAX", "DPS", "MOV", "JTO", "CL", "NOPE"):
        assert new.field(row, key) == old.field(row, key)


def test_drift_send_returns_the_same_line_on_the_same_path(daemon, both):
    """`on_poll` unset on BOTH -- this is a same-path comparison."""
    new, old = both

    assert new.send("VER") == old.send("VER")
    assert new.send("MOV 3 64") == old.send("MOV 3 64") == "OK MOV J3 REQ=64 SET=64 CL=0"


def test_drift_sta_row_agrees(daemon, both):
    new, old = both
    for jid in (1, 3, 4, 5, 6):
        assert new.sta_row(jid) == old.sta_row(jid)
    assert new.sta_row(2) == old.sta_row(2) == "", "J2 is reserved -- no row"


def test_drift_the_give_up_message_is_identical(daemon, both):
    """Same failure, same words. `timeout=0.0` costs no time on either."""
    new, old = both
    with pytest.raises(SystemExit) as a:
        new.send("VER", timeout=0.0)
    with pytest.raises(SystemExit) as b:
        old.send("VER", timeout=0.0)
    assert str(a.value) == str(b.value)


def test_both_copies_now_cut_the_reply_the_same_way(daemon, both, monkeypatch):
    """The reply cut, now present in BOTH copies. Was a deliberate divergence.

    THIS TEST USED TO ASSERT THE OPPOSITE, and the change is the point. It
    pinned motion_verify.ArmLink as quoting the daemon's `LATCHED` line back as
    though the board had answered the move with it, because only the lifted copy
    cut at the log boundary. On 2026-08-07 that defect surfaced on the bench in a
    third tool -- a live pick->storage run returned `OK MOV J1 REQ=88 SET=88 C`,
    truncated before the clamp flag, on which `"CL=1" in reply` reads CLEAN --
    so the cut was pushed back into motion_verify.py and cycle_poses.py via the
    shared Software/tests/reply_cut.py. The divergence is closed.

    The assertion is now the stronger one: identical input to both copies, a
    latch firing between the reply landing and the tail being read, and NEITHER
    quotes the daemon's line. It still fails if either side regresses.
    """
    import time as _time

    new, old = both
    armed = []

    def latch_once_after_the_reply(_s):
        daemon.pump()
        if armed:
            armed.pop()
            daemon.sim.feed(b"!")  # detaches everything
            daemon.poll()          # the health check writes LATCHED + re-ENA

    monkeypatch.setattr(_time, "sleep", latch_once_after_the_reply)

    armed.append(True)
    old_reply = old.send("MOV 3 64")
    assert old_reply == "OK MOV J3 REQ=64 SET=64 CL=0"
    assert "LATCHED" not in old_reply, "motion_verify must cut at the boundary now"

    armed.append(True)
    new_reply = new.send("MOV 3 64")
    assert new_reply == "OK MOV J3 REQ=64 SET=64 CL=0"
    assert "LATCHED" not in new_reply

    assert old_reply == new_reply, "the two copies must not drift apart again"


def test_the_lifted_methods_have_the_same_signature_shape():
    """`send` is the only one that gained a parameter, and it has a default --
    so every call site written against the original still type-checks."""
    import inspect

    for name in ("offset", "tail", "sta_row", "field"):
        assert (inspect.signature(getattr(ArmLink, name))
                == inspect.signature(getattr(motion_verify.ArmLink, name))), name

    new_sig = inspect.signature(ArmLink.send)
    old_sig = inspect.signature(motion_verify.ArmLink.send)
    assert list(new_sig.parameters)[:3] == list(old_sig.parameters)
    assert new_sig.parameters["on_poll"].default is None


def test_the_bot_copy_does_not_reach_for_the_camera_harness():
    """`motion_verify.py` imports cv2 and numpy at module top level.

    The bot's strongest true claim is that it adds no dependency. This is a
    cheap statement of that at the boundary; the deterministic `ast` guard over
    the whole package is Phase 3 Step 6.
    """
    import arm_link

    src = open(arm_link.__file__, encoding="utf-8").read()
    tree = __import__("ast").parse(src)
    imported = set()
    for node in __import__("ast").walk(tree):
        if isinstance(node, __import__("ast").Import):
            imported.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, __import__("ast").ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])

    assert imported <= {"os", "re", "time", "typing", "__future__"}, imported
    assert "serial" not in imported and "cv2" not in imported
