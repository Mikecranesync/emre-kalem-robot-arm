"""The command channel to the holder daemon. It NEVER opens the serial port.

WHERE THIS CAME FROM, AND THE RULE THAT COMES WITH IT
    `ArmLink` is lifted from `Software/tests/motion_verify.py`. THE TWO COPIES
    MUST NOT DRIFT, and `tests/test_arm_link.py` holds a behavioural guard that
    points both at one seeded link directory and asserts identical returns.

    It is a copy rather than an import because `motion_verify.py` imports `cv2`
    and `numpy` at module top level. Importing `ArmLink` from it -- which is
    what `arm-serial-control` §4 tells the next person to do, in a literal code
    snippet -- would drag OpenCV into a Telegram bot whose strongest true claim
    is that it adds no dependency at all. That skill file needs correcting;
    correcting a skill is the operator's call, so it is reported, not done here.

    The guard is BEHAVIOURAL, not textual: a diff would false-fail on
    whitespace, and would also fire on the three deliberate differences below.

THE THREE DELIBERATE DIFFERENCES FROM `motion_verify.ArmLink`
    1. `send()` CUTS the reply at the next log-timestamp boundary. The original
       returns everything to end-of-tail. See `_cut_reply`.
    2. `send()` takes an `on_poll` hook, defaulting to `None`. With no hook the
       behaviour is identical to the original, which is what keeps the drift
       guard honest. It exists because the fake daemon is synchronous.
    3. `self.status`, plus `verdict()` / `restarted()` / `observe()` on top.
       The original has no concept of freshness; it assumes a daemon is running
       because a human started one thirty seconds ago and is watching the arm.
       A phone is not a human standing at the bench.

WHAT THIS LAYER CANNOT DO, STATED PLAINLY
    It cannot tell a crashed daemon from a wedged one from a laptop that went
    to sleep. All three read `STALE`, all three refuse, and the reply says which
    file was how old rather than diagnosing why.

    It cannot stop a second writer. Both this and the daemon open `arm_cmd.txt`
    with `"w"`, so two writers clobber each other silently and with no error
    anywhere. If somebody runs `cycle_poses.py` while the bot is armed,
    commands are lost. ONE DRIVER AT A TIME -- this is named, not fixed.

    It knows nothing about the arm. A board ack proves the firmware accepted a
    command; it proves nothing about a shaft.
"""

from __future__ import annotations

import os
import re
import time
from typing import Callable, NamedTuple

#: The three verdicts. `NO LINK` and not the PRD §7 spelling `NO DAEMON`:
#: `hold_arm.py` writes no PID anywhere, so "the daemon process exists" is not
#: an observable this client has. See the module note on `verdict()`.
LIVE = "LIVE"
STALE = "STALE"
NO_LINK = "NO LINK"

#: How old `arm_status.txt` may be and still count as LIVE.
#:
#: A DESK CHOICE, NEVER OBSERVED UNDER LOAD. The daemon rewrites the status file
#: on a 5 s poll, so it is stale by design even when everything is healthy and
#: anything tighter than 5 s false-alarms constantly. 15 s is three poll periods
#: -- two missed polls tolerated, a stopped loop caught inside a quarter of a
#: minute. Tighten or loosen it against a real run; do not treat it as measured.
FRESHNESS_WINDOW_S = 15.0

#: `hold_arm.log()` writes `f"{time.strftime('%H:%M:%S')} {msg}"`.
_LOG_TIMESTAMP = re.compile(r"^\d\d:\d\d:\d\d ")


class LinkState(NamedTuple):
    """Everything the pure policy layer is allowed to know about the link.

    `verdict` alone is not enough. A daemon restart and a `LATCHED` / `re-ENA`
    line are link-layer OBSERVATIONS that must disarm, and neither can be
    re-derived from a three-valued string. They are passed in, not looked up --
    that is what keeps `policy.decide()` free of I/O.
    """

    verdict: str
    restarted: bool = False
    log_window: str = ""


def _cut_reply(after_marker: str) -> str:
    """Trim a reply at the next daemon log line, not at the next newline.

    `send()` finds its reply by splitting on `CMD <line> -> ` and the remainder
    runs to the END OF THE TAIL, so the daemon's 5 s health poll can append an
    unrelated line into the window -- and that line then gets quoted verbatim
    into a Telegram message as if the board had said it.

    SO WHY NOT JUST CUT AT THE FIRST NEWLINE. Because `hold_arm.log()` stamps
    only the FIRST line of a multi-line message. A `CMD STA -> ` entry is one
    log record with seven untimestamped continuation lines -- six joint rows,
    a SYS row and `OK STA N=6`. A newline cut would truncate every `STA` to its
    J0 row and would look like it worked.

    Somebody will try to simplify this. That is what the comment is for, and
    `test_a_multi_line_reply_carries_exactly_one_timestamp` is the assertion.

    Line 0 is skipped deliberately: by construction it is the tail of the log
    line that carried the marker, so it starts mid-line and can never match.
    """
    lines = after_marker.split("\n")
    kept = [lines[0]]
    for line in lines[1:]:
        if _LOG_TIMESTAMP.match(line):
            break
        kept.append(line)
    return "\n".join(kept).strip()


class ArmLink:
    """Command channel to the holder daemon. Never opens the serial port."""

    def __init__(self, link_dir: str):
        self.cmd = os.path.join(link_dir, "arm_cmd.txt")
        self.log = os.path.join(link_dir, "arm_hold.log")
        #: Not in the original. The freshness layer needs it; nothing lifted
        #: from `motion_verify` reads it.
        self.status = os.path.join(link_dir, "arm_status.txt")
        if not os.path.exists(self.log):
            raise SystemExit(f"no daemon log at {self.log} -- is hold_arm.py running?")
        #: The byte length and first line last OBSERVED. `observe()` is the
        #: only thing that moves either -- see `restarted()`.
        self._last_size = self.offset()
        self._last_first_line = self._first_line()

    # -- lifted verbatim ----------------------------------------------------

    def offset(self) -> int:
        return os.path.getsize(self.log)

    def tail(self, offset: int) -> str:
        with open(self.log, "r", encoding="utf-8", errors="replace") as fh:
            fh.seek(offset)
            return fh.read()

    def send(self, line: str, timeout: float = 6.0,
             on_poll: Callable[[], None] | None = None) -> str:
        """Write one protocol line, return the daemon-logged reply for it.

        The offset primitive, and it is not optional: `arm_status.txt` is only
        rewritten on the daemon's 5 s poll, so immediately after a `MOV` it
        still holds PRE-MOVE state -- and says `MOV=0`. Reading an outcome
        there returns a lie shaped exactly like success. Record the log's byte
        length BEFORE writing the command, then tail from that offset.

        `on_poll` is called once per loop iteration, BEFORE the tail, and is
        for tests only -- the shipped bot leaves it unset. The Phase 0 fake
        daemon is synchronous by design, so with no hook nothing can append the
        reply while a test is blocked in here: it would spin the full timeout
        and raise. Defaulting to `None` is what keeps this identical to
        `motion_verify.ArmLink.send()` when nobody passes one.
        """
        off = self.offset()
        with open(self.cmd, "w", encoding="utf-8") as fh:
            fh.write(line + "\n")
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if on_poll is not None:
                on_poll()
            new = self.tail(off)
            marker = f"CMD {line} -> "
            if marker in new:
                return _cut_reply(new.split(marker, 1)[1])
            time.sleep(0.08)
        raise SystemExit(f"daemon never logged a reply for {line!r} in {timeout}s")

    def sta_row(self, jid: int) -> str:
        for ln in self.send("STA").splitlines():
            if ln.startswith(f"STA J{jid} "):
                return ln.strip()
        return ""

    @staticmethod
    def field(row: str, key: str) -> str:
        for tok in row.split():
            if tok.startswith(key + "="):
                return tok.split("=", 1)[1]
        return ""

    # -- freshness, restart, and the one object the policy layer consumes ----

    def _size(self) -> int | None:
        """The log's byte length, or None if it is not there at all."""
        try:
            return os.path.getsize(self.log)
        except OSError:
            return None

    def _first_line(self) -> str | None:
        """The log's opening line. It never changes while a daemon runs.

        `hold_arm.py` only ever APPENDS after `main()`'s truncation, so this
        line is fixed for the life of one daemon process and changes only when
        the log has been replaced. That makes it the second half of restart
        detection -- see `restarted()`.
        """
        try:
            with open(self.log, "r", encoding="utf-8", errors="replace") as fh:
                return fh.readline()
        except OSError:
            return None

    def verdict(self, now: float) -> str:
        """`LIVE` / `STALE` / `NO LINK`, from whether the status file is fresh.

        THERE IS NO PID CHECK AND THERE CANNOT BE ONE. `hold_arm.py` writes no
        PID -- not to a file, not to the log -- so the bot cannot know whether a
        process is alive and must never claim it. PRD §7's `daemon PID 8123
        alive` is unimplementable without changing the daemon, which is
        forbidden.

        That is a better signal anyway, and this is the part worth keeping.
        "Alive" and "working" collapse into one observable -- IS THE STATUS FILE
        BEING REWRITTEN -- and a PID can be alive while the loop is wedged. A
        wedged loop is exactly what stops feeding the 4000 ms watchdog. Sharper
        still: `hold_arm.py:161` writes the status file only `if sta:`, so a
        live process whose `STA` comes back empty leaves a stale file behind
        with a perfectly healthy PID.

        `STALE` and `NO LINK` both refuse motion, so the BEHAVIOUR is what the
        PRD asked for. Only the copy changes.

        A freshly started daemon has no status file for up to five seconds and
        reads `NO LINK`. That is correct and it refuses -- do not "fix" it by
        falling back to the log's mtime, which the daemon truncates on the way
        up and would therefore report healthy at the one moment it is not.
        """
        if not os.path.exists(self.log):
            return NO_LINK
        try:
            age = now - os.path.getmtime(self.status)
        except OSError:
            return NO_LINK
        return LIVE if age <= FRESHNESS_WINDOW_S else STALE

    def restarted(self) -> bool:
        """Has the log SHRUNK since the last `observe()`? Then the daemon restarted.

        `hold_arm.py:96` is `open(LOG, "w").close()` -- a restarted daemon
        resets `arm_hold.log` to zero bytes. A bot holding a byte offset across
        that calls `tail(offset)`, seeks past EOF, gets an empty string forever,
        and EVERY COMMAND HANGS TO TIMEOUT while the operator watches nothing
        happen.

        It is worse than a hang. A restart re-ran `ENA <j> <adopt>`, so every
        joint SNAPPED TO ITS ADOPT ANGLE -- the same spontaneous-looking motion
        the arming contract exists to disarm on. Re-arming must need a fresh
        `/arm <pose>`, which means a human looking at the arm.

        PURE QUERY. It does not consume the edge, because a predicate that
        re-baselines itself gives the second caller a different answer -- and
        that answer would be "no restart", failing open in the exact path that
        exists to catch the arm moving on its own. `observe()` is the only
        thing that moves the baseline.

        WHY THE SHRINK TEST IS NOT ENOUGH ON ITS OWN -- FOUND BY TESTING IT,
        NOT BY READING IT. The plan specifies `current_size < last_offset` and
        that is a real signal, but it is blind to a restart that happens before
        the log has grown past the startup handshake: the daemon writes the
        SAME handshake every time, so the replacement log can be the same
        length as the one it replaced, or longer. Measured against the fake
        daemon: 1517 bytes before, 1521 after, `shrunk? False`. The bot would
        have gone on believing a declared pose across a restart that snapped
        five joints to their adopt angles.

        So the first line is checked too. It is written once per daemon process
        and only appended after, so a CHANGED first line means the log was
        replaced. False positives are structurally impossible while one daemon
        runs; the one residual blind spot is two daemon runs whose handshakes
        begin in the same `HH:MM:SS` of the clock, which needs a restart landing
        on the same second of the same minute of the same hour as the previous
        start. Both halves fail SAFE -- they over-report, and over-reporting
        costs a re-arm, which is a human looking at the arm.
        """
        size = self._size()
        if size is None:
            return False  # the log is GONE, not shrunk. That is NO LINK.
        if size < self._last_size:
            return True
        return self._first_line() != self._last_first_line

    def observe(self, now: float) -> LinkState:
        """One look at the link, and the only channel into the policy layer.

        Takes `now` because clocks are injected everywhere in this project --
        a suite that waits out a 15 s freshness window in wall-clock seconds is
        a suite nobody runs.

        On a restart `log_window` is the WHOLE new log rather than an empty
        string, because after a truncation "everything since I last looked"
        genuinely is the whole file -- and the restart handshake's
        `ENA <j> <adopt>` lines are precisely what the operator needs to see.
        Tailing from the old offset would seek past EOF and hand back nothing
        at the one moment there is most to report.
        """
        verdict = self.verdict(now)
        size = self._size()
        if size is None:
            self._last_size = 0
            self._last_first_line = None
            return LinkState(verdict, False, "")
        restarted = self.restarted()
        window = self.tail(0) if restarted else self.tail(self._last_size)
        self._last_size = size
        self._last_first_line = self._first_line()
        return LinkState(verdict, restarted, window)
