"""Does `fake_daemon.py` still resemble `hold_arm.py`?

WHAT THIS FILE IS FOR
    `fake_daemon.py` is a MODEL of the holder daemon, and every phase above it
    is measured against that model. If the model drifts from the real daemon,
    a green suite proves a fiction. So this file pins the model to the daemon's
    ACTUAL SOURCE -- the three filenames, the log format, the poll interval, the
    startup truncation, the joint table -- and asserts that replies come from a
    real `ArmSim` rather than from a canned string.

    A drift guard in the shape this repo already uses: the CSV field-count
    validator that caught a malformed row reading `pass=yes`. Cheap, and it
    fails loud.

WHY IMPORTING `hold_arm` IS SAFE
    `main()` is guarded behind `if __name__ == "__main__"`, so importing the
    module opens no port, resets no board and enables no joint. The import does
    pull in pyserial, which is why the Phase 3 `ast` guard excludes `tests/`.

    Nothing here writes into `Software/arm-console/`. `hold_arm.CMD`/`.LOG`/
    `.STATUS` are absolute paths into the real link directory, and a stray
    `arm_cmd.txt` there is a protocol line QUEUED for the next real daemon run.
    Only their basenames are read.

NOTHING HERE IS EVIDENCE ABOUT THE ARM. It is evidence that one Python file
resembles another Python file.
"""

from __future__ import annotations

import inspect
import os

import hold_arm
import pytest

import fake_daemon
from fake_daemon import HEARTBEAT_MS, FakeDaemon

LOG_SRC = inspect.getsource(hold_arm.log)
MAIN_SRC = inspect.getsource(hold_arm.main)


# ===========================================================================
# The three files, and the strings the bot navigates by
# ===========================================================================

def test_the_model_uses_the_daemons_own_three_filenames(link_dir):
    """Read from `hold_arm`'s constants, not from a name I remembered."""
    d = FakeDaemon(str(link_dir))
    assert os.path.basename(d.cmd) == os.path.basename(hold_arm.CMD) == "arm_cmd.txt"
    assert os.path.basename(d.log_path) == os.path.basename(hold_arm.LOG) == "arm_hold.log"
    assert os.path.basename(d.status) == os.path.basename(hold_arm.STATUS) == "arm_status.txt"


def test_the_daemon_owns_com5_and_the_model_owns_nothing(link_dir):
    """The port is the daemon's, with no fallback and no degraded mode.

    `fake_daemon` names its port `SIM` deliberately -- nothing in this suite
    should WRITE a string that reads like an instruction to open the real port.
    The assertion is on the model's runtime port name, not on its prose: an
    earlier version scanned the source for the literal and tripped over its own
    explanatory comment. The literal check belongs to the Phase 3 `ast` guard,
    which parses the bot package and excludes `tests/` for exactly this reason.
    """
    assert hold_arm.PORT == "COM5"
    assert "serial" not in dir(fake_daemon)

    d = FakeDaemon(str(link_dir))
    assert d.port_name != hold_arm.PORT
    d.start()
    assert hold_arm.PORT not in d.read_log()


def test_log_lines_carry_the_HHMMSS_prefix_the_reply_cut_depends_on(daemon):
    """`ArmLink` cuts a reply at the next `^\\d\\d:\\d\\d:\\d\\d ` boundary.

    That only works if the daemon writes one. Pin the format to its source.
    """
    assert "time.strftime('%H:%M:%S')" in LOG_SRC
    assert 'fh.write(line + "\\n")' in LOG_SRC

    import re
    lines = daemon.read_log().splitlines()
    assert lines, "a started daemon logged nothing"
    assert re.match(r"^\d\d:\d\d:\d\d ", lines[0])


def test_the_command_marker_is_the_daemons_own(daemon):
    """`CMD <line> -> ` is how `ArmLink.send()` finds its own reply."""
    assert 'log(f"CMD {ln} -> " + send(ser, ln))' in MAIN_SRC

    daemon.write_cmd("VER")
    off = len(daemon.read_log())
    daemon.pump()
    assert "CMD VER -> " in daemon.read_log()[off:]


def test_the_command_file_is_truncated_after_it_is_consumed(daemon):
    """One line per write. A command left in the file would be re-sent."""
    assert 'open(CMD, "w").close()' in MAIN_SRC

    daemon.write_cmd("VER")
    assert os.path.getsize(daemon.cmd) > 0
    daemon.pump()
    assert os.path.getsize(daemon.cmd) == 0


def test_pump_on_an_absent_or_empty_command_file_is_a_no_op(daemon):
    before = daemon.read_log()
    daemon.pump()  # file does not exist yet
    daemon.write_cmd("")
    daemon.pump()  # file exists but holds nothing
    assert daemon.read_log() == before


# ===========================================================================
# The startup sequence, and the truncation that finding 3 turns on
# ===========================================================================

def test_startup_truncates_the_log_which_is_how_a_restart_is_detectable():
    """`hold_arm.py:96`. The single fact plan finding 3 is built on."""
    assert 'open(LOG, "w").close()' in MAIN_SRC


def test_a_restart_shrinks_the_log_and_re_drives_every_joint(daemon):
    """The observable a bot must catch, and the reason it must.

    A restart re-runs `ENA <j> <adopt>`, so every joint snapped to its adopt
    angle -- the same spontaneous-looking motion the arming contract exists to
    disarm on. It is not merely a bookkeeping problem for the byte offset.
    """
    daemon.write_cmd("VER")
    daemon.pump()
    grown = len(daemon.read_log())
    assert grown > 0

    daemon.restart()

    assert len(daemon.read_log()) < grown, "a restart must shrink the log"
    for jid, (_lo, _hi, adopt, _dps) in fake_daemon.HOLD.items():
        assert f"ENA {jid} {adopt} -> " in daemon.read_log()


def test_a_restart_leaves_the_stale_status_file_exactly_where_it_was(daemon):
    """The daemon never truncates or deletes `arm_status.txt`.

    So the pair a bot sees just after a restart is a SHRUNKEN log beside an
    UNCHANGED, now-stale status file. Both halves matter: the log says the
    daemon restarted, the status file still describes a board that no longer
    exists.
    """
    startup_block = MAIN_SRC.split("last_beat = last_poll")[0]
    assert 'open(LOG, "w").close()' in startup_block
    assert "STATUS" not in startup_block, "startup must not touch the status file"

    before_mtime = os.path.getmtime(daemon.status)
    before_text = open(daemon.status, encoding="utf-8").read()

    daemon.advance(30_000)
    daemon.restart()

    assert os.path.getmtime(daemon.status) == before_mtime
    assert open(daemon.status, encoding="utf-8").read() == before_text


def test_no_startup_line_comes_back_ERR(daemon):
    """If this ever fails it is a FINDING, not something to work around.

    `MIR INV 0` is the interesting one: `arm-sim/README.md` §8 records that the
    documented +/-90 bound is not the binding check, and the envelope check
    bites much earlier. At joint 1's real locked range of 0-91 it is accepted.
    """
    assert "ERR" not in daemon.read_log()


# ===========================================================================
# The joint table and the watchdog window, pinned to the daemon's own values
# ===========================================================================

def test_the_hold_table_matches_the_daemons():
    """`fake_daemon` COPIES this table rather than importing it, so that the
    fake daemon stays stdlib-only. This is the assertion that pays for that."""
    assert fake_daemon.HOLD == hold_arm.HOLD
    assert fake_daemon.WDG_MS == hold_arm.WDG_MS
    assert fake_daemon.MIRROR == hold_arm.MIRROR


def test_the_health_poll_interval_is_five_seconds():
    """The number the 15 s freshness window in `arm_link.py` is derived from."""
    assert "now - last_poll > 5.0" in MAIN_SRC


def test_the_heartbeat_is_fire_and_forget_in_both_files():
    """`hold_arm.py`'s bug (a), and the reason `advance()` models the beat.

    The first version of the daemon fed the watchdog with a `send()` that
    blocks up to 1.2 s. Two of those per cycle exceeds the 4000 ms watchdog,
    the joints detached, and the arm sat on the bench while the loop looked
    healthy.
    """
    assert 'ser.write(b"PNG\\n")' in MAIN_SRC
    assert HEARTBEAT_MS / 1000.0 <= 0.5


def test_a_wedged_loop_latches_the_board_and_leaves_the_status_file_lying(daemon):
    """`wedge()` is `advance()` with the heartbeat removed, and it must bite.

    A daemon whose loop has stopped stops feeding the watchdog as well as
    stopping its poll. The pair this builds -- a detached board beside a status
    file that still says `EN=1` -- is the state `ArmLink.verdict()` exists to
    refuse on, so the model has to be able to construct it.
    """
    before = open(daemon.status, encoding="utf-8").read()
    assert " EN=1" in before and "ES=0" in before

    daemon.wedge(30_000)

    assert open(daemon.status, encoding="utf-8").read() == before, "nothing rewrote it"
    daemon.sim.discard_out()
    daemon.sim.feed(b"STA\n")
    truth = daemon.sim.read_out().decode("ascii", "replace")
    assert "ES=1" in truth and "WD=1" in truth
    assert " EN=1" not in truth


def test_a_long_quiet_advance_does_not_latch_the_board(daemon):
    """Without the modelled heartbeat this fails, and the fiction is expensive.

    A tripped watchdog writes `LATCHED` into the log and re-drives every joint.
    Phase 2 disarms on exactly those strings. A model that manufactured them
    out of nothing but elapsed time would teach the bot to disarm at random.
    """
    daemon.advance(20_000)
    daemon.poll()

    status = open(daemon.status, encoding="utf-8").read()
    assert "ES=0" in status and "WD=0" in status
    assert "LATCHED" not in daemon.read_log()
    assert "re-ENA" not in daemon.read_log()


# ===========================================================================
# The health poll
# ===========================================================================

def test_the_status_file_is_rewritten_on_a_poll_and_only_on_a_poll(daemon):
    """This model does NOT poll on its own -- see `fake_daemon`'s docstring.

    That divergence is what makes a STALE status file reachable offline, and a
    stopped loop IS a daemon that has stopped polling.
    """
    assert "if sta:" in MAIN_SRC
    first = os.path.getmtime(daemon.status)

    daemon.advance(20_000)
    assert os.path.getmtime(daemon.status) == first, "advance() must not poll"

    daemon.poll()
    assert os.path.getmtime(daemon.status) == pytest.approx(daemon.now)
    assert os.path.getmtime(daemon.status) > first


def test_there_is_no_status_file_until_the_first_poll(idle_daemon):
    """A bot started in the same second as the daemon reads `NO LINK`.

    Correct, and the safe direction -- it refuses. Written down so nobody
    later "fixes" it by falling back to the log's mtime, which would report a
    daemon healthy on the strength of a file it truncated on the way up.
    """
    idle_daemon.start()
    assert os.path.exists(idle_daemon.log_path)
    assert not os.path.exists(idle_daemon.status)

    idle_daemon.poll()
    assert os.path.exists(idle_daemon.status)


def test_a_latch_produces_the_two_strings_phase_2_disarms_on(daemon):
    """`enable_all()` snapping joints to adopt angles reads exactly like the
    arm moving on its own. `motion_verify.py` marks any window containing these
    INVALID; the bot must void the declared pose on them."""
    assert 'log("LATCHED (ES/WD) -- the arm has dropped.' in MAIN_SRC
    assert 'log(f"  re-ENA {j} {adopt} -> " + send(ser, f"ENA {j} {adopt}"))' in MAIN_SRC

    daemon.sim.feed(b"!")  # the realtime e-stop byte -- detaches everything
    off = len(daemon.read_log())
    daemon.poll()

    window = daemon.read_log()[off:]
    assert "LATCHED (ES/WD)" in window
    assert "  re-ENA 1 1 -> " in window
    assert "  re-ENA 6 40 -> " in window


# ===========================================================================
# The replies are the firmware's, not mine
# ===========================================================================

def test_replies_are_generated_by_a_real_ArmSim(daemon):
    """Plan Phase 0 Step 4. The wire format must be the firmware's own.

    `OK MOV J3 REQ=64 SET=64 CL=0` is `arm_sim.doMov`'s output, field for
    field. A canned string would pass every test above this one.
    """
    daemon.write_cmd("MOV 3 64")
    off = len(daemon.read_log())
    daemon.pump()

    assert "CMD MOV 3 64 -> OK MOV J3 REQ=64 SET=64 CL=0" in daemon.read_log()[off:]


def test_a_clamped_move_reports_CL_1_rather_than_refusing(daemon):
    """J3's locked range is 0-66. `CL=1` is a FAILED command, not a warning --
    the joint is not where you asked. The bot must read this field."""
    daemon.write_cmd("MOV 3 200")
    daemon.pump()
    assert "OK MOV J3 REQ=200 SET=66 CL=1" in daemon.read_log()


def test_a_multi_line_reply_carries_exactly_one_timestamp(daemon):
    """The fact plan finding 4's reply cut turns on, asserted rather than assumed.

    `log()` prefixes only the FIRST line of a multi-line message, so a `CMD STA`
    block has seven untimestamped continuation lines. A naive cut at the next
    NEWLINE would truncate every `STA` to its J0 row and look like it worked.
    """
    import re

    daemon.write_cmd("STA")
    off = len(daemon.read_log())
    daemon.pump()

    window = daemon.read_log()[off:]
    stamped = [ln for ln in window.splitlines() if re.match(r"^\d\d:\d\d:\d\d ", ln)]
    assert len(stamped) == 1, "a multi-line reply must be one log entry"
    assert len(window.splitlines()) > 5, "STA returns a row per joint plus SYS plus OK"
    assert window.splitlines()[-1].startswith("OK STA")
