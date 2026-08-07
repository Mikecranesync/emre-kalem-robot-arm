"""The whole stack, end to end, against the fake daemon. No network, no port, no real time.

WHAT "END TO END" MEANS HERE AND WHAT IT DOES NOT
    A Telegram update dict goes in at `Bot.handle_update`. What comes out is a
    real protocol line in a real `arm_cmd.txt`, consumed by the Phase 0 fake
    daemon, answered by a real `ArmSim` in the firmware's own wire format, logged
    in the daemon's own log format, and read back through the byte-offset
    primitive. Every layer of this package is in the path.

    IT IS STILL A MODEL OF A PROGRAM TALKING TO A MODEL OF A FIRMWARE. Nothing
    here observes a shaft. A green run is evidence that the bot's logic is
    modelled as written -- that a refusal refuses, that an accepted transition
    puts the right lines on the wire in the right order, that the token does not
    leak. It is evidence about no motor, no gear and no arm.

THE THREE INJECTED SEAMS, AND WHY EACH ONE HAS TO BE INJECTED
    * `FakeTransport` -- no test may touch a network. It also records every
      outgoing message, which is how "a stranger gets silence" is asserted as a
      fact rather than assumed from a code path.
    * `Clock` -- a suite that waits out a 300 s TTL, or a 25 s settle timeout, in
      wall-clock seconds is a suite nobody runs (`arm-sim/README.md` section 9).
    * `PumpingLink` -- the fake daemon is synchronous by design, so something has
      to consume `arm_cmd.txt` while `ArmLink.send()` is blocked waiting for the
      reply. The hook lives in the TEST's link subclass, not in `bot.py`: the
      shipped bot never passes `on_poll`, which is what keeps the Phase 1 drift
      guard against `motion_verify.ArmLink` honest.
"""

from __future__ import annotations

import ast
import json
import os
import re
import sys

import pytest

_TESTS = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_TESTS)
for _p in (_PKG, _TESTS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import audit  # noqa: E402
import bot as botmod  # noqa: E402
import policy  # noqa: E402
import voice  # noqa: E402
from arm_link import ArmLink  # noqa: E402
from fake_daemon import FakeDaemon  # noqa: E402

#: Not a real token and never was. It is shaped like one so a substring search
#: for it is a meaningful assertion.
TOKEN = "1234567890:AAFAKE-not-a-real-bot-token-do-not-use"
MIKE = 4242
STRANGER = -99


# --------------------------------------------------------------------------
# The three seams
# --------------------------------------------------------------------------
class PumpingLink(ArmLink):
    """An `ArmLink` that lets the synchronous fake daemon answer.

    `send()` writes the command file and then loops until the reply appears in
    the log. Nothing else runs during that loop, so with a fake daemon that only
    acts when a test calls it, the reply would never land -- `send()` would spin
    the full 6 s timeout and raise `SystemExit`. `on_poll` is called once per
    iteration and is wired to `pump()` here.

    IT IS OVERRIDDEN IN THE TEST, NOT PASSED BY THE BOT, ON PURPOSE. `bot.py`
    calls `link.send(line)` with no hook, exactly as the shipped bot will against
    a real daemon that answers on its own.
    """

    def __init__(self, link_dir: str, daemon: FakeDaemon):
        super().__init__(str(link_dir))
        self.daemon = daemon
        #: Every line this bot has ever put on the wire, in order.
        self.wire: list[str] = []

    def send(self, line: str, timeout: float = 6.0, on_poll=None) -> str:
        self.wire.append(line)
        return super().send(line, timeout, on_poll=on_poll or self.daemon.pump)


class Clock:
    """One injected clock driving virtual time, the daemon, and the 5 s poll.

    `mono()` is the TTL clock and `wall()` is what `arm_status.txt`'s mtime is
    compared against -- two clocks in the bot, one source of truth here.

    THE POLL IS MODELLED HERE AND IT MATTERS. The fake daemon deliberately does
    not poll on its own, but the real one polls every 5 s regardless of what the
    host is doing. A transition burns 5-15 s of virtual time inside the settle
    loop; without a poll, `arm_status.txt`'s mtime ages past the 15 s freshness
    window and THE NEXT MESSAGE READS STALE -- so a second `/go` would refuse for
    a reason the test never meant to exercise. Firing `poll()` every 5000 ms of
    accumulated sleep makes the harness more faithful, not less.
    """

    POLL_MS = 5000.0

    def __init__(self, daemon: FakeDaemon, start: float = 10_000.0):
        self.daemon = daemon
        self.t = start
        self._since_poll = 0.0

    def mono(self) -> float:
        return self.t

    def wall(self) -> float:
        return self.daemon.now

    def sleep(self, seconds: float) -> None:
        self.t += seconds
        self.daemon.advance(seconds * 1000.0)
        self._since_poll += seconds * 1000.0
        while self._since_poll >= self.POLL_MS:
            self._since_poll -= self.POLL_MS
            self.daemon.poll()


class FakeTransport:
    """The Bot API, minus the API. Records everything and reaches no network."""

    def __init__(self, token: str = TOKEN):
        self.token = token
        self.sent: list[tuple[int, str]] = []
        self.pending: list[dict] = []
        self.downloads: list[str] = []
        #: Set to an exception to make the NEXT `send_message` raise it once.
        self.raise_once: Exception | None = None
        self.download_raises: Exception | None = None

    def get_updates(self, offset, timeout):
        if offset == -1:                     # the startup backlog probe
            return self.pending[-1:]
        taken, self.pending = self.pending, []
        return taken

    def send_message(self, chat_id, text):
        if self.raise_once is not None:
            exc, self.raise_once = self.raise_once, None
            raise exc
        self.sent.append((chat_id, text))
        return {"message_id": len(self.sent)}

    def download(self, file_id):
        if self.download_raises is not None:
            raise self.download_raises
        self.downloads.append(file_id)
        return b"OggS\x00fake-opus-bytes"

    # -- assertions read through these --------------------------------------

    def texts(self) -> list[str]:
        return [t for _, t in self.sent]

    def to(self, chat_id: int) -> list[str]:
        return [t for c, t in self.sent if c == chat_id]


class Rig:
    """A started daemon, a bot wired to it, and the handles a test asserts on."""

    def __init__(self, tmp_path, transcribe=voice.transcribe, ttl_s=300.0):
        self.dir = tmp_path / "link"
        self.dir.mkdir()
        self.daemon = FakeDaemon(str(self.dir))
        self.daemon.start()
        self.daemon.poll()
        self.clock = Clock(self.daemon)
        self.link = PumpingLink(str(self.dir), self.daemon)
        self.transport = FakeTransport()
        self.audit_path = str(tmp_path / "audit.jsonl")
        self.config = botmod.Config(
            token=TOKEN, allowed_chat_ids=frozenset({MIKE}),
            link_dir=str(self.dir), ttl_s=ttl_s, audit_path=self.audit_path)
        self.log_lines: list[str] = []
        self.bot = botmod.Bot(
            self.config, self.link, self.transport,
            audit.AuditLog(self.audit_path,
                           scrub=lambda s: botmod.scrub(s, TOKEN)),
            mono=self.clock.mono, wall=self.clock.wall, sleep=self.clock.sleep,
            transcribe=transcribe, logger=self.log_lines.append)

    # -- driving -------------------------------------------------------------

    def say(self, text, chat_id=MIKE, username="mike", update_id=1):
        self.bot.handle_update(update(text, chat_id, username, update_id))

    def speak(self, chat_id=MIKE, duration=2, update_id=1):
        self.bot.handle_update(voice_update(chat_id, duration, update_id))

    # -- observing -----------------------------------------------------------

    def rows(self) -> list[dict]:
        return audit.read(self.audit_path)

    def reasons(self) -> list[str]:
        return [r["reason"] for r in self.rows()]

    def cmd_lines(self) -> list[str]:
        """Every `CMD <line> -> ` the daemon logged. The wire, from its side.

        Read from the LOG rather than from `arm_cmd.txt`: the daemon truncates
        the command file as it consumes it, so an empty file proves nothing.
        """
        return re.findall(r"CMD (.+?) -> ", self.daemon.read_log())

    def moves(self) -> list[str]:
        return [c for c in self.cmd_lines() if c.startswith("MOV")]


def update(text, chat_id=MIKE, username="mike", update_id=1) -> dict:
    return {"update_id": update_id,
            "message": {"chat": {"id": chat_id},
                        "from": {"username": username},
                        "text": text}}


def voice_update(chat_id=MIKE, duration=2, update_id=1) -> dict:
    return {"update_id": update_id,
            "message": {"chat": {"id": chat_id},
                        "from": {"username": "mike"},
                        "voice": {"file_id": "AwACfake", "duration": duration,
                                  "mime_type": "audio/ogg"}}}


@pytest.fixture
def rig(tmp_path):
    return Rig(tmp_path)


# --------------------------------------------------------------------------
# The happy path -- and it is the only test here that moves a simulated joint
# --------------------------------------------------------------------------
def test_arm_then_go_runs_the_recorded_path_phase_by_phase(rig):
    """`/arm storage` then `/go pick`: the whole contract, in order.

    Asserts the ORDERING, not just the arrival. `storage -> pick` is swing_out
    (J1 to the transit height), aim (the wrist and elbow, in free space), then
    descend (J1 onto the mat). Reversing that descends a claw that is still
    pointing the wrong way onto the table.
    """
    rig.say("/arm storage")
    assert not rig.cmd_lines(), "arming must send nothing to the arm"
    assert "Armed for 300 s" in rig.transport.to(MIKE)[0]

    rig.say("/go pick")

    assert rig.moves() == ["MOV 1 40", "MOV 3 36", "MOV 4 140", "MOV 5 165", "MOV 1 8"]
    assert rig.reasons() == ["armed", "motion_accepted", "motion_complete"]
    assert rig.bot.state.declared_pose == "pick"
    assert rig.bot.state.in_flight is None
    # The outcome quotes the board and nothing more.
    assert "OK MOV J1 REQ=8 SET=8 CL=0" in rig.transport.to(MIKE)[-1]


def test_a_completed_move_does_not_extend_the_window(rig):
    """The window is extended by nothing -- not by activity, not by a finished move."""
    rig.say("/arm storage")
    armed_until = rig.bot.state.armed_until
    rig.say("/go pick")
    assert rig.bot.state.armed_until == armed_until


def test_two_audit_rows_per_transition_one_accepted_one_outcome(rig):
    """Deliberate: the accept and the outcome are separate facts.

    The accepted row is written BEFORE anything is on the wire and therefore
    carries `lines == []`. Recording the planned waypoints there would claim
    lines were sent that a mid-transition abort never sends -- the exact shape of
    lie this surface exists to avoid.
    """
    rig.say("/arm storage")
    rig.say("/go pick")
    accepted, outcome = rig.rows()[1], rig.rows()[2]
    assert accepted["reason"] == "motion_accepted"
    assert accepted["lines"] == []
    assert "phases: swing_out, aim, descend" in accepted["note"]
    assert outcome["reason"] == "motion_complete"
    assert outcome["lines"] == ["MOV 1 40", "MOV 3 36", "MOV 4 140", "MOV 5 165", "MOV 1 8"]
    assert all("CL=0" in r for r in outcome["board_replies"])


# --------------------------------------------------------------------------
# Gate 1: the allowlist, outermost
# --------------------------------------------------------------------------
def test_a_stranger_gets_silence_and_is_still_audited(rig):
    """No reply, no error, no hint a bot exists -- and a row, because that row is
    the only trace the contact leaves anywhere."""
    rig.say("/status", chat_id=STRANGER, username="whoever")
    assert rig.transport.sent == [], "a stranger must get no reply of any kind"
    row = rig.rows()[0]
    assert row["reason"] == "not_allowlisted"
    assert row["chat_id"] == STRANGER and row["username"] == "whoever"
    assert row["text"] == "/status", "the raw text is what reveals probing"
    assert not rig.cmd_lines()


def test_a_stranger_cannot_move_the_arm_even_while_mike_has_a_window_open(rig):
    """The allowlist is checked BEFORE arming state, so an open window is not a hole."""
    rig.say("/arm storage")
    rig.say("/go pick", chat_id=STRANGER)
    assert rig.transport.to(STRANGER) == []
    assert not rig.moves()
    assert rig.bot.state.declared_pose == "storage", "the stranger changed nothing"


def test_a_stranger_sending_a_voice_note_is_never_even_downloaded(rig):
    """Gate 1 is before the transcriber, so an unknown chat cannot make the bot
    fetch a file -- which would be an outbound request a stranger controls."""
    rig.speak(chat_id=STRANGER)
    assert rig.transport.downloads == []
    assert rig.transport.sent == []
    assert rig.reasons() == ["not_allowlisted"]


# --------------------------------------------------------------------------
# Refusals must reach the wire with NOTHING on it
# --------------------------------------------------------------------------
@pytest.mark.parametrize("script,reason", [
    (["/go pick"], "not_armed"),
    (["/arm storage", "/go storage"], "wrong_declared_pose"),
    (["/arm storage", "/go somewhere"], "unknown_pose"),
    (["/arm storage", "/go pick 40"], "number_refused"),
    (["/arm storage", "/open the gripper"], "gripper_refused"),
    (["/arm storage", "/dance"], "unknown_command"),
])
def test_every_refusal_leaves_the_command_channel_untouched(rig, script, reason):
    """Not "a refusal came back" -- NOTHING WAS SENT. This is the layer that would
    actually write `arm_cmd.txt`, so the assertion means more here than upstream."""
    for line in script:
        rig.say(line)
    assert rig.reasons()[-1] == reason
    assert not rig.moves(), f"{reason} put a MOV on the wire"


def test_an_expired_window_sends_nothing_at_all(rig):
    """THE CENTRAL INVERSION. The instinctive "window expired, make it safe" is
    `EST` or `DIS`, and on this arm those DETACH and it falls. Expiry is a state
    change in the bot; the daemon goes on holding exactly as before."""
    rig.say("/arm storage")
    rig.clock.sleep(301.0)                  # past the TTL, daemon healthy throughout
    rig.say("/go pick")
    assert rig.reasons()[-1] == "window_expired"
    assert not rig.cmd_lines(), "expiry must be silent on the wire"
    # AND the command file itself, which is what the PRD's acceptance criterion
    # asks for in those words: "the stub records that no line was written to
    # arm_cmd.txt". Both assertions, not either -- the log proves nothing was
    # CONSUMED, this proves nothing was WRITTEN. A line written but never pumped
    # would satisfy the first and still be sitting there for the next daemon.
    assert _cmd_file(rig) == "", "expiry left a line in arm_cmd.txt"
    assert "NOTHING WAS SENT TO THE ARM" in rig.transport.to(MIKE)[-1]


def _cmd_file(rig) -> str:
    """`arm_cmd.txt`'s literal contents, or "" when it is absent."""
    path = os.path.join(str(rig.dir), "arm_cmd.txt")
    if not os.path.exists(path):
        return ""
    return open(path, "r", encoding="utf-8").read().strip()


@pytest.mark.parametrize("script,reason", [
    (["/go pick"], "not_armed"),
    (["/arm storage", "/go storage"], "wrong_declared_pose"),
    (["/arm storage", "/go pick 40"], "number_refused"),
    (["/arm storage", "/dance"], "unknown_command"),
    (["/arm storage", "/disarm", "/go pick"], "not_armed"),
])
def test_no_refusal_ever_leaves_a_line_in_the_command_file(rig, script, reason):
    """The stronger form of "a refusal sends nothing".

    `hold_arm.py:177` consumes `arm_cmd.txt` whenever it is non-empty, so a line
    written and not yet consumed is a command QUEUED for whichever daemon reads
    it next -- possibly one that starts tomorrow. Asserting on the daemon's log
    alone cannot see that.
    """
    for line in script:
        rig.say(line)
    assert rig.reasons()[-1] == reason
    assert _cmd_file(rig) == "", f"{reason} left {_cmd_file(rig)!r} in arm_cmd.txt"
    assert not rig.moves()


def test_a_second_request_mid_transition_is_refused_and_not_queued(rig):
    """A queued motion command asserts consent that has already expired.

    The bot is single-threaded, so a genuine mid-flight message cannot be
    delivered here -- the state is set up directly instead, which is what a
    second update would find if one arrived.
    """
    rig.say("/arm storage")
    rig.bot.state = policy.replace(rig.bot.state, in_flight="pick")
    rig.say("/go pick")
    assert rig.reasons()[-1] == "move_in_flight"
    assert not rig.moves()


def test_stop_CANNOT_interrupt_a_transition_and_this_is_a_limitation(rig):
    """A LIMITATION, PINNED SO NOBODY MEETS IT AT THE BENCH. Not a feature.

    The bot is a single long-poll loop. While a transition runs it is inside the
    settle loop, not reading updates -- so a `/stop` typed mid-move sits in
    Telegram's queue and is not read until the transition has already finished.
    The `STP` then goes out AFTER `motion_complete`, which is to say after the
    move it was meant to stop.

    This contradicts plan Phase 4 Step 5, which expects to send `/stop`
    mid-transition and watch it take effect. **That step cannot pass as written.**

    It is reported rather than fixed. Fixing it needs a second thread reading
    updates while the first dispatches -- which puts TWO WRITERS on `arm_cmd.txt`
    inside this package, the exact hazard plan finding 6 names as unfixable from
    here. That is a design change well outside this phase.

    At the bench, the stop mid-move is the rocker switch. It always was.
    """
    rig.say("/arm storage")
    rig.transport.pending = [update("/go pick", update_id=1),
                             update("/stop", update_id=2)]
    rig.bot.poll_once()

    assert rig.reasons() == ["armed", "motion_accepted", "motion_complete", "stopped"]
    assert rig.cmd_lines()[-1] == "STP", "the STP arrives, but only after the move ran"
    assert rig.moves() == ["MOV 1 40", "MOV 3 36", "MOV 4 140", "MOV 5 165", "MOV 1 8"], (
        "every waypoint went out before the /stop was even read")


def test_a_daemon_restart_BETWEEN_UPDATES_disarms_and_refuses(rig):
    """Plan finding 3, through the whole stack -- and this is the EASY half.

    A restart truncates `arm_hold.log` to zero bytes AND re-runs `ENA <j> <adopt>`
    on every joint -- so five joints snapped somewhere while nobody asked. The
    declared pose is void; re-arming needs a human looking at the arm.

    RENAMED. It was `..._mid_window_...`, which reads as though it covered
    restarts generally. It does not, and that name is part of why the
    mid-TRANSITION case below went unnoticed: `decide()` only ever sees a restart
    between messages, because `observe()` runs once per update.
    """
    rig.say("/arm storage")
    rig.daemon.restart()
    rig.daemon.poll()
    rig.say("/go pick")
    assert rig.reasons()[-1] == "daemon_restarted"
    assert not rig.moves()
    assert rig.bot.state.declared_pose is None and rig.bot.state.armed_until is None


def test_a_daemon_restart_at_a_phase_boundary_stops_the_transition(rig):
    """THE HOLE THIS FILE MISSED, and it moved a simulated arm along a bad path.

    Measured against the fake daemon BEFORE the fix: `/arm storage`, `/go pick`,
    restart the daemon after the first phase settled. The bot sent
    `MOV 3 36`, `MOV 4 140`, `MOV 5 165` and then `MOV 1 8` into an arm whose
    shoulder had just snapped from 40 back to its adopt angle of 1, replied
    "the declared pose is now pick", and LEFT THE WINDOW OPEN. The `aim` phase
    exists to swing the wrist and elbow while the claw is clear of the mat --
    it ran with the shoulder folded back down.

    Every existing check passed and none of them could have caught it:
      * every reply was `CL=0` and none was an `ERR`;
      * the latch scan tailed from an offset the truncation had put past EOF,
        so it read an empty string -- plan finding 3, one layer over;
      * tailing from 0 would have found nothing either, because
        `hold_arm.main()` logs plain `ENA` on the way up. Only `enable_all()`
        logs `re-ENA`, so a restart handshake carries NO `LATCH_MARKERS`.

    The check is `link.restarted()` at each phase boundary -- a pure query
    against the baseline `observe()` set at the top of the update.
    """
    rig.say("/arm storage")
    real_settle = rig.bot._settle
    fired = {"n": 0}

    def settle_then_restart(phase):
        ok = real_settle(phase)
        fired["n"] += 1
        if fired["n"] == 1:          # the first phase has landed and settled
            rig.daemon.restart()     # truncates the log, re-ENAs all five joints
            rig.daemon.poll()
        return ok

    rig.bot._settle = settle_then_restart
    rig.say("/go pick")

    # The remaining phases must never reach the wire. `moves()` reads the NEW
    # log, so this is exactly "what did the restarted daemon receive".
    assert rig.moves() == [], "a phase was sent after the daemon re-ENA'd every joint"
    assert rig.reasons()[-1] == "restarted_during_move"
    assert rig.bot.state == policy.State(), "the declared pose and window are void"
    reply = rig.transport.to(MIKE)[-1]
    assert "adopt angles" in reply
    assert "NOT sent" in reply
    assert rig.rows()[-1]["events"] == ["restarted"]


def test_a_latch_at_a_phase_boundary_stops_the_transition(rig):
    """PRD section 5c, driven through the whole stack -- it was unit-level only.

    `test_policy.py` hands `finish_motion()` a canned `LATCH_LOG` string, which
    proves the judgement but not the PLUMBING: nothing asserted that
    `_run_transition` actually scans the log between phases, or that the window
    it scans is the right one. That matters more after the restart fix, because
    that fix changed the very line which computes that window.

    A real latch here, not a canned string: `enable_all()` is what the daemon
    runs on latch recovery, and it writes the `  re-ENA` lines verbatim. The arm
    has just snapped to five adopt angles, so the remaining phases assume a start
    it no longer has -- same consequence as a restart, different cause.
    """
    rig.say("/arm storage")
    real_settle = rig.bot._settle
    fired = {"n": 0}

    def settle_then_latch(phase):
        ok = real_settle(phase)
        fired["n"] += 1
        if fired["n"] == 1:
            rig.daemon.log("LATCHED (ES/WD) -- the arm has dropped. Clearing and re-enabling.")
            rig.daemon.enable_all()          # writes the real `  re-ENA` lines
        return ok

    rig.bot._settle = settle_then_latch
    rig.say("/go pick")

    assert rig.moves() == ["MOV 1 40"], "a phase was sent after the daemon re-ENA'd"
    assert rig.reasons()[-1] == "latched_during_move"
    assert rig.bot.state == policy.State(), "the declared pose and window are void"
    assert "re-ENA" in rig.transport.to(MIKE)[-1], "the log lines go back verbatim"


def test_the_restart_check_does_not_fire_on_a_healthy_transition(rig):
    """The canary for the check above. A log that merely GROWS is not a restart.

    `link.restarted()` is size-shrink OR first-line-changed, and a normal
    transition appends several kilobytes without touching either. If this goes
    red, the new check is firing wrongly and every transition would abort.
    """
    rig.say("/arm storage")
    rig.say("/go pick")
    assert rig.reasons()[-1] == "motion_complete"
    assert rig.moves() == ["MOV 1 40", "MOV 3 36", "MOV 4 140", "MOV 5 165", "MOV 1 8"]
    assert rig.rows()[-1]["events"] == []


def test_which_half_of_the_restart_check_actually_fires(rig):
    """Named because the two halves have different reliability, and it matters.

    `ArmLink.restarted()` is `size < last_size` OR `first_line != last_first_line`.
    The SIZE half is documented-unreliable -- the docstring's own measurement is
    1517 bytes before a restart and 1521 after, so it can miss. The FIRST-LINE
    half depends on the two daemon starts landing in different `HH:MM:SS`, which
    the ~8.2 s startup handshake makes near-certain but does not guarantee.

    This pins WHICH one carries the check in the rig, so a future change that
    silently leans on the weaker half is visible rather than assumed.
    """
    rig.say("/arm storage")
    before_size = rig.link._last_size
    before_first = rig.link._last_first_line
    rig.daemon.restart()
    rig.daemon.poll()
    after_size = os.path.getsize(rig.link.log)
    after_first = open(rig.link.log, encoding="utf-8").readline()

    assert rig.link.restarted() is True
    assert after_first != before_first, "the first-line half must catch this"
    # Reported, not asserted as a requirement: the size half is the weak one.
    print(f"\n  restart detection: size {before_size} -> {after_size} "
          f"(shrunk? {after_size < before_size}); first line changed? "
          f"{after_first != before_first}")


def test_a_stale_link_refuses_motion_and_closes_the_window(rig):
    """The 2026-08-06 evening failure: the loop stopped, the watchdog tripped, and
    `arm_status.txt` sat on disk still reporting `EN=1` on every joint. A stale
    status file reads exactly like a healthy held arm, and Telegram is the surface
    where that lie does the most damage."""
    rig.say("/arm storage")
    rig.daemon.wedge(20_000)                # the loop stops; nothing rewrites the status file
    rig.say("/go pick")
    assert rig.reasons()[-1] == "link_stale"
    assert not rig.moves()
    assert rig.bot.state.armed_until is None


# --------------------------------------------------------------------------
# The token
# --------------------------------------------------------------------------
def test_the_token_never_reaches_a_reply_or_the_audit_log_on_an_exception_path(rig):
    """THE TOKEN IS IN THE URL, so a Bot API failure carries it.

    A `urllib` error's text quotes what it was doing, and what it was doing was
    `https://api.telegram.org/bot<token>/sendMessage`. This raises exactly that
    shape of error from inside the transport, on the first reply, and then checks
    every surface the secret could have reached.
    """
    rig.transport.raise_once = RuntimeError(
        f"HTTP Error 502: Bad Gateway for "
        f"https://api.telegram.org/bot{TOKEN}/sendMessage")
    rig.say("/status")

    on_disk = open(rig.audit_path, "r", encoding="utf-8").read()
    assert TOKEN not in on_disk
    assert TOKEN not in "".join(rig.transport.texts())
    assert TOKEN not in "".join(rig.log_lines)
    assert "<redacted>" in "".join(rig.log_lines), "the failure must still be reported"


def test_the_token_is_scrubbed_out_of_an_error_reply_the_operator_receives(rig):
    """PRD Q17-Q5: paste the exact reply, because every debugging session in this
    project has turned on somebody having the literal line. That is only safe if
    the secret is removed from it first."""
    boom = RuntimeError(f"getFile failed for https://api.telegram.org/bot{TOKEN}/getFile")

    def explode(*_a, **_k):
        raise boom

    rig.bot.link = _Exploding(explode)
    rig.say("/status")
    reply = rig.transport.to(MIKE)[-1]
    assert TOKEN not in reply
    assert "<redacted>" in reply
    assert TOKEN not in open(rig.audit_path, "r", encoding="utf-8").read()
    assert rig.reasons()[-1] == "internal_error"


def test_a_link_failure_mid_transition_neither_kills_the_bot_nor_wedges_it(rig):
    """`ArmLink.send()` raises `SystemExit`, which is NOT an `Exception`.

    Found by probing, not by reading. `ArmLink` is lifted verbatim from
    `motion_verify.py`, which is a command-line script -- and in a script,
    `SystemExit` on an unanswered command is the right thing. In a long-running
    bot it is two bugs at once:

      * `SystemExit` inherits from `BaseException`, so a plain `except Exception`
        does not catch it. THE PROCESS EXITS -- silently, mid-transition, with no
        reply and no audit row. The arm stays held by the daemon, so nothing
        falls, but the operator is left with a phone that has simply stopped
        answering, which is the failure shape this project keeps re-learning.
      * `in_flight` is left set, so `/arm` and `/go` both refuse `move_in_flight`
        forever after. Only `/stop` clears it, and nothing said so.

    The fix belongs HERE and not in `arm_link.py`: that copy must keep behaving
    exactly like `motion_verify.ArmLink` or the Phase 1 drift guard stops meaning
    anything. The bot is what has to be robust to it.
    """
    rig.say("/arm storage")
    real = rig.link.send

    def flaky(line, timeout=6.0, on_poll=None):
        if line == "MOV 3 36":                    # the second phase, mid-transition
            raise SystemExit("daemon never logged a reply for 'MOV 3 36' in 6.0s")
        return real(line, timeout, on_poll)

    rig.link.send = flaky
    rig.say("/go pick")                            # must not raise out of here

    assert rig.reasons()[-1] == "internal_error"
    assert "never logged a reply" in rig.transport.to(MIKE)[-1]
    assert rig.bot.state == policy.State(), "a dispatch that threw is not in flight"

    rig.link.send = real
    rig.say("/arm pick")
    assert rig.reasons()[-1] == "armed", "the bot must be recoverable without /stop"


class _Exploding:
    """A link whose every call raises. Models a link layer failing mid-update."""

    def __init__(self, boom):
        self.boom = boom

    def __getattr__(self, _name):
        return self.boom


def test_the_reply_path_censors_the_token_even_when_the_caller_did_not(rig):
    """The LAST line of defence, tested as one -- and it was not, until a mutation
    said so.

    Both token tests above feed `_reply` text that some earlier layer had already
    scrubbed, so deleting the scrub inside `_reply` left the whole suite green.
    That made "two layers on the one secret" a claim in a docstring rather than a
    fact. This asserts the second layer on its own: whatever a future call site
    hands `_reply`, the secret does not leave the process.
    """
    rig.bot._reply(MIKE, f"the board said {TOKEN} which it never would")
    body = rig.transport.to(MIKE)[-1]
    assert TOKEN not in body
    assert "<redacted>" in body


def test_the_audit_layer_censors_the_token_even_when_the_caller_did_not(rig):
    """Same argument, other layer. `AuditLog.write` scrubs every string field of
    every row, so a call site added later cannot leak by forgetting -- and the
    only way to know that is to hand it a row that forgot."""
    rig.bot._row(chat_id=MIKE, username="mike", source="text", text="/status",
                 decision="refused", reason="internal_error",
                 note=f"raw failure quoting https://api.telegram.org/bot{TOKEN}/x",
                 board_replies=(f"nested {TOKEN}",))
    on_disk = open(rig.audit_path, "r", encoding="utf-8").read()
    assert TOKEN not in on_disk
    assert on_disk.count("<redacted>") == 2, "nested strings are scrubbed too"


def test_scrub_leaves_text_alone_when_the_secret_is_empty():
    """A scrubber built with `""` would otherwise splice the marker between every
    character of every message."""
    assert botmod.scrub("nothing to hide", "") == "nothing to hide"
    assert botmod.scrub("abc", None or "") == "abc"


# --------------------------------------------------------------------------
# Voice -- the same road, no privileges of its own
# --------------------------------------------------------------------------
def test_a_voice_note_refuses_and_sends_nothing(rig):
    rig.say("/arm storage")
    rig.speak(duration=3)
    assert rig.reasons()[-1] == "voice_not_enabled"
    assert not rig.cmd_lines()
    assert "Voice is not enabled" in rig.transport.to(MIKE)[-1]
    assert rig.rows()[-1]["voice_seconds"] == 3
    assert rig.rows()[-1]["text"] == "", "no transcription means nothing to record"


def test_the_audit_row_for_a_voice_note_carries_duration_and_never_audio(rig):
    rig.speak(duration=7)
    row = rig.rows()[-1]
    assert row["source"] == "voice" and row["voice_seconds"] == 7
    assert "Ogg" not in json.dumps(row) and "opus" not in json.dumps(row)


def test_a_transcription_goes_through_the_identical_gates(tmp_path):
    """Voice is a text source and nothing more. Unarmed, the transcribed `/go
    pick` refuses for exactly the reason the typed one does."""
    rig = Rig(tmp_path, transcribe=lambda data, mime: "/go pick")
    rig.speak()
    assert rig.reasons()[-1] == "not_armed"
    assert not rig.moves()

    rig.say("/arm storage")
    rig.speak(update_id=2)
    assert rig.reasons()[-1] == "motion_complete"
    assert rig.moves() == ["MOV 1 40", "MOV 3 36", "MOV 4 140", "MOV 5 165", "MOV 1 8"]


@pytest.mark.parametrize("heard", ["quick", "stick", "go quick", "pick", "storage",
                                   "go to pick please", "/go pik"])
def test_a_near_miss_transcription_is_refused_and_never_snapped_to_a_pose(tmp_path, heard):
    """"pick", "quick" and "stick" are one bad microphone apart.

    A near-miss that MOVES THE ARM is the failure a phone-shaped interface
    invites. Note that bare `pick` and bare `storage` refuse too: they are pose
    names, not commands, and inferring `/go` from a noun is the same guess
    wearing a different hat.
    """
    rig = Rig(tmp_path, transcribe=lambda data, mime: heard)
    rig.say("/arm storage")
    rig.speak(update_id=2)
    assert rig.rows()[-1]["decision"] == "refused"
    assert not rig.moves()


def test_a_failed_voice_download_refuses_rather_than_crashing(rig):
    """Fail closed: not knowing what was said is the same as not being told."""
    rig.transport.download_raises = RuntimeError("connection reset")
    rig.speak()
    assert rig.reasons()[-1] == "voice_not_enabled"
    assert not rig.cmd_lines()


def test_voice_module_imports_no_similarity_matcher():
    """The fail-closed rule is only real while nothing fuzzy sits in the path.

    `difflib` is standard library, so the stdlib-only guard in
    `test_no_serial.py` would NOT catch `difflib.get_close_matches` being added
    here -- and that one function is exactly how "quick" becomes "pick".
    """
    src = open(os.path.join(_PKG, "voice.py"), "r", encoding="utf-8").read()
    roots = {n.name.split(".")[0] for node in ast.walk(ast.parse(src))
             if isinstance(node, ast.Import) for n in node.names}
    roots |= {(node.module or "").split(".")[0] for node in ast.walk(ast.parse(src))
              if isinstance(node, ast.ImportFrom)}
    assert roots <= {"__future__"}, f"voice.py imports {sorted(roots)}"
    assert voice.transcribe(b"anything at all", "audio/ogg") is None


# --------------------------------------------------------------------------
# The wire: what may go on it, and what may not
# --------------------------------------------------------------------------
def test_no_forbidden_verb_ever_reaches_the_wire_across_the_whole_table(rig):
    """Runtime evidence, not a grep over the source.

    `EST` and the watchdog DETACH every joint and a gravity-loaded arm falls;
    `DIS` detaches one; `CLR` clears a latch, which is the daemon's job. This
    drives every command the bot accepts, including `/stop`, and asserts on the
    lines the daemon actually logged.
    """
    for line in ("/status", "/arm storage", "/go pick", "/stop", "/disarm",
                 "/arm pick", "/go storage", "/status"):
        rig.say(line)
    assert rig.cmd_lines(), "the script must actually have reached the wire"
    for sent in rig.cmd_lines():
        verb = sent.split()[0].upper()
        assert verb not in policy.FORBIDDEN_VERBS, f"{sent!r} reached the daemon"
        assert verb in policy.ALLOWED_VERBS, f"{sent!r} is outside ALLOWED_VERBS"


def test_send_refuses_a_forbidden_verb_at_the_choke_point(rig):
    """One choke point, and the settle poll's own `STA` goes through it too."""
    for line in ("EST", "DIS 1", "CLR", "JOG 1 5"):
        with pytest.raises(policy.ForbiddenLine):
            rig.bot._send(line)
    assert not rig.cmd_lines()
    assert _cmd_file(rig) == "", "a refused verb still reached arm_cmd.txt"


@pytest.mark.parametrize("line", ["MOV 6 70", "MOV 6 10", "MOV 0 64", "MOV 2 90"])
def test_send_refuses_a_joint_this_surface_may_not_command(rig, line):
    """The joint half of the allowlist, at the layer that actually writes the file.

    HARDENING. Nothing in this package builds such a line today -- but the only
    thing that had been stopping one was the pose DATA, and `_run_transition`
    sends whatever lines a `Phase` carries. A hand-built `Phase` (there is one in
    this very file, for the clamp test) went straight past every allowlist.

    J6 acks and does not articulate and a phone can never rule out fingers in the
    claw. J0's servo is dead.
    """
    with pytest.raises(policy.ForbiddenLine):
        rig.bot._send(line)
    assert not rig.cmd_lines()
    assert _cmd_file(rig) == ""


def test_stop_works_unarmed_and_sends_exactly_STP(rig):
    """The sole unarmed exception. A transition takes 5-15 s and can outlive a TTL
    that expired while it ran, so a `/stop` refused for being unarmed would be
    refused at exactly the moment it was wanted."""
    rig.say("/stop")
    assert rig.cmd_lines() == ["STP"]
    assert rig.reasons() == ["stopped"]
    reply = rig.transport.to(MIKE)[-1]
    assert "did not remove power" in reply
    assert "rocker switch" in reply


def test_status_reports_the_verdict_and_quotes_the_board(rig):
    rig.say("/status")
    reply = rig.transport.to(MIKE)[-1]
    assert "LIVE" in reply
    assert "STA J1 EN=1" in reply, "the board's own line, verbatim"
    assert rig.rows()[-1]["board_replies"][0].startswith("STA J0")


# --------------------------------------------------------------------------
# A transition that goes wrong
# --------------------------------------------------------------------------
def test_a_clamped_waypoint_stops_the_transition_and_voids_the_pose(rig):
    """`CL=1` is a FAILED command, not a warning -- the joint is not where it was
    asked, so the next phase would start from somewhere nobody knows.

    The Decision is hand-built because no recorded pose can clamp today:
    `test_policy.py` asserts every waypoint in both transitions sits inside
    `hold_arm.HOLD`'s limits. This is the failure a LATER POSE EDIT would produce,
    which is exactly why it is asserted before anyone makes one.
    """
    over = policy.Phase(label="too_far", targets={3: 200}, lines=("MOV 3 200",))
    after = policy.Phase(label="never_run", targets={1: 40}, lines=("MOV 1 40",))
    rig.bot.state = policy.State(armed_until=rig.clock.mono() + 300,
                                 declared_pose="storage", in_flight="pick")
    decision = policy.Decision(accept=True, reason="motion_accepted", lines=(),
                               reply="", new_state=rig.bot.state,
                               phases=(over, after))
    rig.bot._run_transition(MIKE, "mike", "text", "/go pick", decision, "LIVE", None)

    assert rig.moves() == ["MOV 3 200"], "the second phase must never be sent"
    assert rig.reasons()[-1] == "clamped"
    assert rig.bot.state == policy.State(), "the declared pose is void"
    assert "SET=66 CL=1" in rig.transport.to(MIKE)[-1]


def test_a_phase_that_never_settles_stops_and_sends_nothing_to_make_it_safe(rig):
    """THE CHOICE, NAMED: no automatic `STP`.

    There is no command that makes this arm safe -- `STP` holds with the joints
    driven, `EST` detaches and drops it. So a settle timeout does what expiry
    does: the bot goes quiet, the daemon goes on holding, and the operator
    decides with his eyes on the arm.

    `_sta_rows` is stubbed empty, which is what a joint that never reports itself
    parked looks like from here.
    """
    rig.bot._sta_rows = lambda: {}
    rig.say("/arm storage")
    rig.say("/go pick")

    assert rig.moves() == ["MOV 1 40"], "only the first phase was sent"
    assert rig.reasons()[-1] == "did_not_settle"
    assert rig.bot.state == policy.State()
    reply = rig.transport.to(MIKE)[-1]
    assert "part-way along the path" in reply
    assert "STP" in reply, "the operator is told how to abort interpolation"


def test_a_move_outcome_is_read_off_the_wire_and_never_out_of_the_status_file(rig):
    """PRD section 7a, pinned as behaviour rather than as a docstring promise.

    `arm_status.txt` is rewritten only on the daemon's 5 s poll, so immediately
    after a `MOV` it still holds PRE-MOVE state -- and says `MOV=0`. A settle
    check that read it would return instantly and call a move done before it had
    started: a lie shaped exactly like success. That is the single most dangerous
    shortcut available in this package, and it is one line of code away.

    The evidence that it was not taken: a fresh `STA` goes out on the COMMAND
    CHANNEL between the waypoints, so the daemon's own log records it. Swap
    `_sta_rows()` for a status-file read and these `STA` lines vanish.

    WHAT THIS DOES NOT PROVE, and I asserted it wrongly first. I tried to assert
    that `arm_status.txt` was untouched across the transition; it is not, because
    `Clock` fires the daemon's 5 s poll and the real daemon does too. So the file
    is a LIVE alternative source in this rig, not an obviously-dead one -- which
    is precisely why the wire evidence is the assertion and the file's contents
    are not.
    """
    rig.say("/arm storage")
    rig.say("/go pick")

    wire = rig.cmd_lines()
    assert rig.reasons()[-1] == "motion_complete"
    assert "STA" in wire, "the settle predicate never reached the command channel"
    first_mov = wire.index("MOV 1 40")
    last_mov = len(wire) - 1 - wire[::-1].index("MOV 1 8")
    assert "STA" in wire[first_mov:last_mov], (
        "no STA between the first and last waypoint -- the settle check is not "
        "reading a fresh reply off the wire")


def test_the_settle_poll_is_never_tighter_than_the_proven_interval():
    """0.12 s is `cycle_poses.py`'s settle interval and the only polling rate this
    project has evidence for: 16 clean cycles, zero clamps, zero watchdog latches.
    The daemon's `send()` blocks up to 1.2 s during which it is not writing the
    `PNG` heartbeat, against a 4000 ms watchdog. Do not invent a tighter number to
    make a test faster."""
    assert botmod.SETTLE_POLL_S >= 0.12


# --------------------------------------------------------------------------
# Startup: the environment, and Telegram's own queue
# --------------------------------------------------------------------------
@pytest.mark.parametrize("missing", ["ARM_TELEGRAM_BOT_TOKEN",
                                     "ARM_TELEGRAM_ALLOWED_CHAT_IDS",
                                     "ARM_TELEGRAM_LINK_DIR"])
def test_the_bot_refuses_to_start_without_any_one_of_the_three(missing):
    env = {"ARM_TELEGRAM_BOT_TOKEN": TOKEN,
           "ARM_TELEGRAM_ALLOWED_CHAT_IDS": "4242",
           "ARM_TELEGRAM_LINK_DIR": "/tmp/link"}
    env.pop(missing)
    with pytest.raises(SystemExit) as exc:
        botmod.Config.from_env(env)
    assert missing in str(exc.value)


def test_an_empty_or_malformed_allowlist_refuses_to_start():
    """An empty allowlist is not "allow everyone" -- but neither is it something to
    shrug at. Far more likely, it is a typo, and a bot that silently drops one id
    is one the operator debugs by looking at the arm."""
    base = {"ARM_TELEGRAM_BOT_TOKEN": TOKEN, "ARM_TELEGRAM_LINK_DIR": "/tmp/link"}
    with pytest.raises(SystemExit):
        botmod.Config.from_env({**base, "ARM_TELEGRAM_ALLOWED_CHAT_IDS": " , "})
    with pytest.raises(SystemExit) as exc:
        botmod.Config.from_env({**base, "ARM_TELEGRAM_ALLOWED_CHAT_IDS": "4242,mike"})
    assert "mike" in str(exc.value)


def test_the_env_never_leaks_the_token_into_the_refusal_message():
    with pytest.raises(SystemExit) as exc:
        botmod.Config.from_env({"ARM_TELEGRAM_BOT_TOKEN": TOKEN,
                                "ARM_TELEGRAM_ALLOWED_CHAT_IDS": "",
                                "ARM_TELEGRAM_LINK_DIR": "/tmp/link"})
    assert TOKEN not in str(exc.value)


def test_a_startup_backlog_is_discarded_and_never_replayed(rig):
    """Telegram holds undelivered updates for about 24 hours and replays them on
    the first `getUpdates`. So a `/arm storage` and a `/go pick` typed at the
    bench yesterday would arrive together today, arm against TODAY's clock, and
    run the transition with nobody in the room.

    That is PRD section 6's own prohibition -- "a queued motion command asserts
    consent that has already expired" -- arriving through a queue the bot did not
    build. Refusing to queue internally does nothing about it.
    """
    rig.transport.pending = [update("/arm storage", update_id=10),
                             update("/go pick", update_id=11)]
    rig.bot.drain_backlog()

    assert rig.bot.offset == 12, "the backlog is skipped past, not read"
    assert not rig.cmd_lines()
    assert rig.bot.state == policy.State(), "nothing in the backlog armed anything"
    assert rig.reasons() == ["backlog_discarded"]
    assert "update_id 11" in rig.rows()[0]["note"]

    # And the next real message is handled normally.
    rig.transport.pending = [update("/status", update_id=12)]
    assert rig.bot.poll_once() == 1
    assert rig.reasons()[-1] == "status"


def test_poll_once_advances_the_offset_so_an_update_is_never_handled_twice(rig):
    rig.transport.pending = [update("/status", update_id=77)]
    rig.bot.poll_once()
    assert rig.bot.offset == 78
    assert rig.bot.poll_once() == 0
    assert rig.reasons() == ["status"]


def test_an_update_that_is_not_a_message_is_audited_and_ignored(rig):
    rig.bot.handle_update({"update_id": 5, "channel_post": {"text": "/go pick"}})
    assert rig.reasons() == ["unsupported_update"]
    assert rig.transport.sent == []
    assert not rig.cmd_lines()


# --------------------------------------------------------------------------
# The copy, and the audit schema
# --------------------------------------------------------------------------
#: `SERIAL-PROTOCOL.md` section 0 bans these because these servos have no
#: feedback of any kind: the firmware knows only what it last COMMANDED. The ban
#: bites hardest on this surface, because the reader cannot glance at the bench
#: and catch the lie.
BANNED_WORDS = ("actual", "measured", "feedback", "position")


def test_no_reply_this_module_can_emit_uses_a_banned_word():
    for reply in list(botmod.REPLIES.values()) + [voice.VOICE_DISABLED_REPLY]:
        low = reply.lower()
        for word in BANNED_WORDS:
            assert not re.search(rf"\b{word}\b", low), f"{word!r} in {reply!r}"


#: The SAME five `test_policy.py` uses. They were three here, which meant a
#: bot-layer reply could have said "the arm is held" or "stopped for safety" and
#: only the policy layer would have failed. Two copies of a ban list are two
#: chances to enforce a different ban.
BANNED_PHRASES = ("the arm is at", "the arm is held", "the move completed",
                  "stopped for safety", "emergency stop")


def test_no_reply_claims_the_arm_is_anywhere_or_that_a_move_happened():
    """A board ack proves the firmware accepted a command. It proves nothing about
    a shaft -- that cost a whole afternoon on D3."""
    for reply in list(botmod.REPLIES.values()) + [voice.VOICE_DISABLED_REPLY]:
        low = reply.lower()
        for phrase in BANNED_PHRASES:
            assert phrase not in low, f"{phrase!r} in {reply!r}"


def test_the_two_ban_lists_have_not_drifted_apart():
    """One rule, enforced in two files. Pin them equal so neither loosens alone."""
    import test_policy

    assert set(BANNED_PHRASES) == set(test_policy.BANNED_PHRASES)
    assert set(BANNED_WORDS) == set(test_policy.BANNED_WORDS)


def test_every_reply_says_what_to_do_next():
    for key, reply in botmod.REPLIES.items():
        assert reply.strip(), f"{key} has no copy"


def test_the_audit_schema_covers_every_field_the_prd_asks_for():
    """PRD section 13's list, pinned. A field dropped in a refactor is a field the
    operator does not have on the night he needs it."""
    required = {"ts", "chat_id", "username", "text", "decision", "reason",
                "lines", "board_replies", "verdict", "armed", "remaining_s"}
    assert required <= set(audit.FIELD_NAMES), required - set(audit.FIELD_NAMES)


def test_every_reason_written_to_the_audit_log_is_a_declared_one(rig):
    """An undeclared reason is a branch nothing exercises and nobody can grep for."""
    known = set(policy.REASONS) | set(policy.OUTCOME_REASONS) | set(botmod.BOT_REASONS)
    for line in ("/status", "/arm storage", "/go pick", "/nonsense", "/stop"):
        rig.say(line)
    rig.say("/go pick", chat_id=STRANGER)
    rig.speak(update_id=9)
    for reason in rig.reasons():
        assert reason in known, reason


def test_the_audit_log_is_append_only_across_a_reopen(rig, tmp_path):
    """Nothing in `audit.py` updates, rewrites or truncates. A second AuditLog on
    the same path appends to what is already there."""
    rig.say("/status")
    second = audit.AuditLog(rig.audit_path)
    second.write(audit.Row(ts="later", chat_id=MIKE, username="mike", source="text",
                           text="/status", decision="refused", reason="not_armed"))
    rows = rig.rows()
    assert len(rows) == 2 and rows[0]["reason"] == "status" and rows[1]["ts"] == "later"


def test_a_reply_longer_than_telegram_accepts_is_truncated_visibly(rig):
    """Telegram rejects a `sendMessage` over 4096 characters with a 400, and a
    rejected reply is a SILENT one -- the operator sees nothing and assumes the
    bot is dead."""
    rig.bot._reply(MIKE, "x" * 9000)
    body = rig.transport.to(MIKE)[-1]
    assert len(body) < 4096
    assert body.endswith("[...truncated -- see the audit log]")
