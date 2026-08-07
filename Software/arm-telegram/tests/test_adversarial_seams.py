"""Seam defects found by attacking the stack, not by reading it.

Each test here reproduces a failure that the rest of the suite was green
through. They are in their own file rather than in `test_bot.py` on purpose:
they are adversarial probes with a different job from that file's contract
tests, and each one carries the reasoning for a rule that was learned the
expensive way -- here, by driving the thing until it broke.

NOTHING IN THIS FILE HAS RUN AGAINST HARDWARE. It is a model of a bot talking
to a model of a daemon talking to a model of the firmware. A green run says the
stranded-command hole and the wedge are closed IN THE MODEL. It says nothing
about a motor, a gear or an arm.

NO TEST HERE SLEEPS ON THE WALL CLOCK. The clock is `test_bot.Clock`, injected.
"""

from __future__ import annotations

import os
import sys

import pytest

_TESTS = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_TESTS)
for _p in (_PKG, _TESTS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import bot as botmod  # noqa: E402
import policy  # noqa: E402
from test_bot import TOKEN, Rig, update  # noqa: E402


@pytest.fixture
def rig(tmp_path):
    """A started fake daemon and a bot wired to it, rooted in `tmp_path`.

    Declared here rather than imported: `test_bot.rig` is a fixture in another
    module and pytest does not share those across files. Same `Rig`, so there is
    only one harness -- only the fixture name is local.
    """
    return Rig(tmp_path)


# ==========================================================================
# 1. A command whose reply never landed must not be left in `arm_cmd.txt`
# ==========================================================================
def _daemon_dies_before(rig, dead_line: str):
    """Wire the link so the daemon stops consuming from `dead_line` onward.

    `timeout=0.0` ON THE DEAD CALLS, AND IT IS LOAD-BEARING RATHER THAN A
    SHORTCUT. The real `ArmLink.send()` writes the command file BEFORE it starts
    waiting, so with `timeout=0.0` the loop body is never entered and the give-up
    path is reached having left exactly the same stranded line -- which is the
    thing under test. Letting it run the real 6 s timeout would cost six
    wall-clock seconds per call and prove nothing extra; a suite that waits out
    timeouts in real seconds is a suite nobody runs (`arm-sim/README.md` §9).
    Same technique, same reason, as `test_arm_link.py`'s give-up tests.
    """
    real = rig.link.send
    dead: list[bool] = []

    def send(line, timeout=6.0, on_poll=None):
        if line == dead_line:
            dead.append(True)
        if dead:                       # nothing consumes arm_cmd.txt any more
            return real(line, 0.0, on_poll=lambda: None)
        return real(line, timeout, on_poll)

    rig.link.send = send


def test_a_stranded_MOV_is_not_left_queued_for_the_next_daemon(rig):
    """THE HAZARD THIS CLOSES, AND WHY IT IS THE WORST ONE IN THE PACKAGE.

    `ArmLink.send()` writes the command file and then waits for the reply. When
    the reply never lands it raises -- and the line is still in `arm_cmd.txt`.
    `hold_arm.main()` truncates the LOG on the way up (line 96) and NEVER
    touches `CMD`; the loop's first pass consumes whatever is sitting there
    (line 177) and sends it to the board.

    So: the daemon dies at a phase boundary mid-transition, the bot correctly
    gives up, voids everything and tells the operator. `arm_cmd.txt` is left
    holding `MOV 3 36`. Whenever the daemon is next started -- that evening, the
    next morning -- its first act is to drive the elbow. No armed window, no
    declared pose, nobody in the room, and no message anywhere saying it would
    happen.

    That is exactly the hazard `policy.decide()` refuses to create for `/stop`
    on `NO LINK`, in those words: "a line written now would sit in arm_cmd.txt
    until the next daemon starts and then run". The transition write path
    created it anyway, because the guard was on the DECISION and the hole is in
    the WRITE.

    Measured before the fix: `arm_cmd.txt` held `'MOV 3 36\\n'`, and a restarted
    fake daemon logged `CMD MOV 3 36 -> OK MOV J3 REQ=36 SET=36 CL=0`.
    """
    rig.say("/arm storage")
    _daemon_dies_before(rig, "MOV 3 36")
    rig.say("/go pick")

    assert rig.reasons()[-1] == "internal_error"
    assert rig.bot.state == policy.State(), "a dispatch that threw is not in flight"

    pending = open(rig.link.cmd, encoding="utf-8").read().strip()
    assert pending == "", (
        f"arm_cmd.txt was left holding {pending!r} -- the next daemon to start "
        "consumes it on its first loop pass and drives the joint, with no armed "
        "window and nobody present")

    # And prove it end to end: a daemon starting now must find nothing to run.
    rig.daemon.restart()
    rig.daemon.pump()
    ran = [ln for ln in rig.daemon.read_log().splitlines() if "CMD MOV" in ln]
    assert ran == [], f"the next daemon executed a stranded command: {ran}"


def test_the_retraction_does_not_clobber_a_line_it_does_not_recognise(rig):
    """Never a blind truncate. The bot cannot enforce sole-writer on
    `arm_cmd.txt` (finding 6) -- both it and the daemon open the file `"w"` --
    so a retraction that cleared the file regardless of contents would make this
    package the second writer it warns everyone else about."""
    rig.say("/arm storage")

    def never_answers(line, timeout=6.0, on_poll=None):
        with open(rig.link.cmd, "w", encoding="utf-8") as fh:
            fh.write("MOV 5 165\n")          # somebody else's line, not ours
        raise SystemExit(f"daemon never logged a reply for {line!r} in {timeout}s")

    rig.link.send = never_answers
    rig.say("/status")

    assert open(rig.link.cmd, encoding="utf-8").read().strip() == "MOV 5 165", (
        "the retraction cleared a line this bot did not write")


def test_a_command_the_daemon_answered_is_not_retracted(rig):
    """The happy path must be untouched: the daemon truncates the file itself
    when it consumes the line, so there is nothing to retract and no second
    write goes anywhere near it."""
    rig.say("/status")
    assert rig.reasons() == ["status"]
    assert rig.cmd_lines() == ["STA"]
    assert open(rig.link.cmd, encoding="utf-8").read().strip() == ""


# ==========================================================================
# 2. One malformed update must not wedge the long-poll loop forever
# ==========================================================================
def test_a_malformed_update_does_not_wedge_the_bot_forever(rig):
    """`poll_once` read `update["update_id"]` OUTSIDE `handle_update`'s try.

    A `KeyError` there propagates to `run()`, which catches it, logs, sleeps 3 s
    and polls again -- with `self.offset` NEVER ADVANCED. Telegram re-delivers
    the same update, and the bot loops on it forever: no reply to anything, no
    audit row, nothing on the wire. It is not a crash the operator can see. It
    is a phone that has silently stopped answering, which is the failure shape
    this project keeps re-learning.

    Worse, the good message QUEUED BEHIND IT is never handled either -- so a
    `/stop` typed after a mangled update would never be read.

    Measured before the fix: three consecutive `poll_once()` calls each raised
    `KeyError: 'update_id'`, `offset` stayed 100, and no audit row was written.
    """
    rig.bot.offset = 100
    rig.transport.pending = [{"no_update_id": 1},
                             update("/status", update_id=101)]

    rig.bot.poll_once()          # must not raise

    assert rig.bot.offset == 102, "the offset must advance past the bad update"
    assert "status" in rig.reasons(), (
        "the good message queued behind the malformed one was never handled")
    assert "unsupported_update" in rig.reasons(), (
        "the malformed update must leave a row -- it is the only trace it exists")


def test_a_malformed_startup_backlog_does_not_stop_the_bot_starting(rig):
    """`drain_backlog()` is called from `run()` OUTSIDE its `try`, so an
    unreadable update there kills the process before the loop begins. It fails
    closed -- nothing is armed, nothing moves -- but the operator gets a
    traceback instead of a bot, and the daemon goes on holding with no way to
    ask it anything."""
    rig.transport.pending = [{"garbage": True}]
    assert rig.bot.drain_backlog() == 0     # must not raise
    assert not rig.cmd_lines()


# ==========================================================================
# 3. The armed window is the whole safety contract -- it may not be disabled
#    by an unvalidated environment variable
# ==========================================================================
def _env(**over):
    base = {"ARM_TELEGRAM_BOT_TOKEN": TOKEN,
            "ARM_TELEGRAM_ALLOWED_CHAT_IDS": "4242",
            "ARM_TELEGRAM_LINK_DIR": "/tmp/link"}
    base.update(over)
    return base


@pytest.mark.parametrize("raw", ["0", "-5", "abc", "1e9", "nan"])
def test_a_nonsensical_ttl_refuses_to_start_rather_than_crashing(raw):
    """`float(env.get(...))` accepted `0` and `-5` -- a window that is expired
    before it opens, so every `/go` refuses `window_expired` and the operator
    debugs the arm before he debugs the variable. `abc` raised a bare
    `ValueError` traceback instead of the readable `SystemExit` the other three
    variables get. Same class of mistake as a silently-dropped allowlist entry.
    """
    with pytest.raises(SystemExit) as exc:
        botmod.Config.from_env(_env(ARM_TELEGRAM_TTL_S=raw))
    assert "ARM_TELEGRAM_TTL_S" in str(exc.value)


@pytest.mark.parametrize("raw", ["", "   "])
def test_an_empty_ttl_variable_falls_back_to_the_shipped_default(raw):
    """SPLIT OUT FROM THE REFUSAL CASES DELIBERATELY, AND THE SPLIT IS THE POINT.

    `ARM_TELEGRAM_TTL_S` is OPTIONAL -- unlike the token, the allowlist and the
    link dir, which are required and where empty correctly means missing and
    refuses. An optional variable that is set but empty falling back to its
    documented default is the same shape as `ARM_TELEGRAM_AUDIT` right beside
    it, and 300 s is the value the plan ships.

    This is not a softer version of the test above. It has its own teeth: it
    fails if somebody ever makes empty mean 0 (a window expired before it
    opens), or unbounded (no window at all).
    """
    assert botmod.Config.from_env(
        _env(ARM_TELEGRAM_TTL_S=raw)).ttl_s == policy.DEFAULT_TTL_S


def test_an_unbounded_ttl_cannot_silently_disable_the_arming_contract():
    """A window measured in years is not a window.

    Arming exists to bound how long a stale intention stays valid -- it is the
    only thing standing between "Mike typed /arm at the bench" and "a phone in a
    pocket runs a transition tomorrow". A typo of an extra digit turns that into
    a permanent grant, and nothing would have said so.

    THE CEILING IS A DESK CHOICE, NOT A MEASUREMENT, exactly like
    `FRESHNESS_WINDOW_S`. The real TTL is Mike's number and PRD Q17-Q3 is still
    unanswered; this only refuses the values that cannot be what he meant.
    """
    with pytest.raises(SystemExit) as exc:
        botmod.Config.from_env(_env(ARM_TELEGRAM_TTL_S="99999999"))
    assert "ARM_TELEGRAM_TTL_S" in str(exc.value)

    ok = botmod.Config.from_env(_env(ARM_TELEGRAM_TTL_S="120"))
    assert ok.ttl_s == 120.0


def test_the_default_ttl_is_still_the_one_the_plan_ships():
    assert botmod.Config.from_env(_env()).ttl_s == policy.DEFAULT_TTL_S


# ==========================================================================
# 4. Attacked and HELD -- kept so a later change cannot quietly break them
# ==========================================================================
def test_a_daemon_poll_landing_inside_the_reply_window_is_not_quoted_back(rig):
    """Item 4 of the attack, with prior traffic carrying the SAME marker.

    A `CMD MOV 3 64 -> ` line is seeded ahead of the call, and a 5 s health poll
    is fired INSIDE the reply window. The reply must be this call's own, cut at
    the log-timestamp boundary: neither the earlier identical command's payload
    nor the daemon's later line.
    """
    with open(rig.link.log, "a", encoding="utf-8") as fh:
        fh.write("07:59:59 CMD MOV 3 64 -> OK MOV J3 REQ=64 SET=64 CL=0 EARLIER\n")

    fired: list[bool] = []

    def poll_inside_the_window():
        rig.daemon.pump()
        if not fired:
            fired.append(True)
            rig.daemon.poll()

    reply = rig.link.send("MOV 3 64", on_poll=poll_inside_the_window)

    assert reply == "OK MOV J3 REQ=64 SET=64 CL=0"
    assert "EARLIER" not in reply, "the offset primitive read prior traffic"
    assert "CMD STA" not in reply, "a daemon poll line was carried into the reply"


def test_a_bare_log_truncation_disarms_rather_than_hanging(rig):
    """Item 5: `hold_arm.py:96` is `open(LOG, "w").close()`. The bot must notice
    the log went backwards and disarm -- not seek past EOF and wait out every
    later command's timeout while the operator watches nothing happen."""
    rig.say("/arm storage")
    assert rig.bot.state.armed_until is not None

    open(rig.link.log, "w").close()          # line 96, verbatim
    rig.say("/status")

    assert rig.bot.state.armed_until is None, "a truncated log must close the window"
    assert rig.bot.state.declared_pose is None
    assert rig.reasons()[-1] == "status", "it answered rather than hanging"
