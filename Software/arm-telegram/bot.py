"""The Telegram adapter. It owns the token, the allowlist, and the wire.

    Telegram --HTTPS long poll--> bot.py --writes-------> arm_cmd.txt   \\
                                    |  <--byte-offset--- arm_hold.log    >  hold_arm.py --> the port
                                    |  <--mtime--------- arm_status.txt /     (sole owner)

STANDARD LIBRARY ONLY. `urllib` long-polls `getUpdates`; there is no client
library. `python-telegram-bot` is LGPL-3 and this project allows Apache-2.0 or
MIT, but the licence is not the interesting part -- ZERO NEW DEPENDENCIES is the
strongest true statement this package has, and the Bot API is plain HTTPS with
JSON, so keeping it is nearly free. Do not trade it for convenience.

THE BOT IS A CLIENT AND NEVER A PORT OWNER. It writes one protocol line into
`arm_cmd.txt` and reads that line's own reply out of `arm_hold.log` by byte
offset. It never opens the serial port, not as a fallback and not "just to
check": opening it DTR-resets the board, which drops every limit back to the
firmware default 70-110 with `MIR=UNKNOWN` and `CAL=0`. J6's real 10-70 lies
almost entirely BELOW that default, so everything under 70 would silently clamp.
`tests/test_no_serial.py` is the deterministic guard on that.

THE GATES, OUTERMOST FIRST -- the order is the design:

  1. ALLOWLIST. Before arming state, before parsing, before anything. A chat id
     that is not on the list gets NO REPLY AT ALL: no error, no hint a bot
     exists. A token can leak; a bot that answers strangers tells them what it
     controls. Silent to the sender, LOUD in the audit log.
  2. Voice, if it is a voice note -- and voice is a text source with no
     privileges of its own. `voice.transcribe()` returns `None` today, so it
     refuses here and never reaches the arm.
  3. `link.observe()`. One look, and the only channel by which a link-layer fact
     (freshness, a restart, a latch in the log window) reaches the pure layer.
  4. `policy.decide()`. Pure. It is handed the clock and the link state; it reads
     no file and no environment.
  5. Dispatch -- and ONLY here does anything reach `arm_cmd.txt`.

TWO CLOCKS, AND MIXING THEM IS A REAL BUG
    `link.observe(now)` compares `now` against `arm_status.txt`'s MTIME, so it
    needs WALL time (`time.time`). `policy.decide(now)` measures a TTL, so it
    needs a MONOTONIC clock that a laptop waking from sleep or an NTP step cannot
    move backwards. Passing `time.monotonic()` to `observe()` reports a status
    file aged by the whole uptime of the machine and refuses everything; passing
    `time.time()` to `decide()` hands the arming window to the system clock. Both
    are injected, which is also what makes the TTL testable with a number instead
    of a five-minute sleep.

WHAT THIS LAYER ADDS THAT `policy` CANNOT, and why the reasons live here
    `policy.decide()` is pure and has no chat id, no socket and no file. Four
    refusals therefore cannot exist in its table and are decided here instead --
    `BOT_REASONS` below. They are gate-layer and I/O-layer facts, not decisions
    about arming.

WHAT IT CANNOT DO -- stated here so nobody has to infer it
    It cannot prevent a second writer to `arm_cmd.txt`. Both this and the daemon
    open it `"w"`, so two writers clobber each other silently, with no error
    anywhere. Run `cycle_poses.py` while a window is open and commands are lost.
    ONE DRIVER AT A TIME. Named, not fixed.

    It cannot know where the arm is, cannot prove a human is at the bench, and
    cannot stop anything. `/stop` sends `STP`, which HOLDS with the joints still
    driven. `EST`, `DIS` and `CLR` are forbidden on this surface -- `EST` and the
    watchdog DETACH, and a gravity-loaded arm falls. The rocker switch and the
    fuse are the only emergency stop, and no copy here calls anything else one.

NOTHING IN THIS FILE HAS RUN AGAINST HARDWARE.
"""

from __future__ import annotations

import json
import os
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from typing import Any, Callable, Mapping, Sequence

import audit
import policy
import voice
from arm_link import ArmLink

# --------------------------------------------------------------------------
# Reasons this layer owns. They are NOT in `policy.REASONS` and must not be
# added there: policy is pure, and every one of these is a fact about a socket,
# a chat id, or a file -- things a pure function cannot see.
# --------------------------------------------------------------------------
BOT_REASONS = (
    "not_allowlisted",      # gate 1. The sender gets silence; this row is the only trace.
    "unsupported_update",   # not a message at all (channel post, callback, edit).
    "voice_not_enabled",    # transcribe() returned None, which is every time today.
    "did_not_settle",       # a phase never reached its commanded angles. See _settle.
    "backlog_discarded",    # updates that arrived while the bot was not running.
    "internal_error",       # anything unhandled, scrubbed before it is written.
)

#: Every string this module can send to Telegram. Kept in one dict so the
#: banned-word test asserts on the real emitted copy rather than on an ast walk
#: that would also trip over docstrings explaining WHY the words are banned.
#: `SERIAL-PROTOCOL.md` section 0 bans "actual", "measured", "feedback" and
#: "position" because these servos have no feedback of any kind -- the firmware
#: knows only what it last COMMANDED -- and the ban bites hardest here, because
#: the reader cannot glance at the bench and catch the lie.
REPLIES = {
    "internal_error": (
        "Something in the bot failed while handling that. Nothing further was sent "
        "to the arm, and nothing was sent to make it safe -- the verbs that sound "
        "safe (EST, DIS) detach every joint and a loaded arm falls. The holder "
        "daemon goes on holding exactly as before.\n"
        "If this happened during a transition the arm may be part-way along the "
        "path. The window is closed and the declared pose is void: look at the "
        "arm, then /arm <pose> again. The error follows, with the API token removed:"
    ),
    "did_not_settle": (
        "STOPPED -- a phase did not reach its commanded angles inside {timeout:.0f} s, "
        "so the rest of the transition was NOT sent.\n"
        "The arm may be part-way along the path. Nothing was sent to make it safe, "
        "because the verbs that sound safe (EST, DIS) detach every joint and a "
        "loaded arm falls. The holder daemon goes on holding exactly as before.\n"
        "The window is closed and the declared pose is void. Look at the arm, then "
        "/arm <pose> again. /stop sends STP if you want interpolation aborted.\n"
        "Last board lines, verbatim:"
    ),
    "voice_download_failed": (
        "That voice note could not be fetched from Telegram, so nothing was "
        "transcribed and nothing was sent to the arm. Send a text command."
    ),
    "backlog_discarded": (
        "Backlog discarded on startup: {count} update(s) that arrived while the bot "
        "was not running were dropped without being read as commands.\n"
        "Telegram holds undelivered updates for about 24 hours. A /go from an hour "
        "ago asserts consent that has already expired, so it is never replayed."
    ),
}

#: How long one phase of a transition may take before the bot gives up on it.
#:
#: A FIXED CEILING, because the bot cannot compute a travel time: it commands no
#: `SPD` at all and inherits whatever the daemon set (`hold_arm.py`'s HOLD table
#: is J1=15, J3=20, J4=20, J5=20 deg/s), so it does not know the rate and must
#: not assume one. The arithmetic behind the number: the longest single move in
#: either recorded transition is J1 across its whole 0-91 range, ~6.1 s at
#: 15 deg/s; the widest phase moves J3/J4/J5 together and is bounded by J4 at
#: ~9 s worst case; the daemon's own 5 s health poll can delay a reply by up to
#: ~1.5 s. 25 s is generous against all of that, which is the point -- a timeout
#: here should mean something is genuinely wrong, not that a joint was slow.
SETTLE_TIMEOUT_S = 25.0

#: NEVER TIGHTER THAN THIS. 0.12 s is `cycle_poses.py`'s settle interval and the
#: only polling rate this project has evidence for: 16 clean cycles, zero clamps,
#: zero joint timeouts, zero watchdog latches. The daemon's `send()` blocks up to
#: 1.2 s during which it is NOT writing the `PNG` heartbeat, against a 4000 ms
#: watchdog. Do not invent a tighter number to make a test faster.
SETTLE_POLL_S = 0.12

#: Telegram rejects a `sendMessage` over 4096 characters with a 400. A rejected
#: reply is a SILENT one -- the operator sees nothing and assumes the bot is
#: dead -- so long copy is truncated visibly instead.
MAX_REPLY_CHARS = 3500

#: Long-poll seconds asked of `getUpdates`, plus the slack added to the socket
#: timeout so the HTTP read outlives the server's own wait.
POLL_TIMEOUT_S = 25
SOCKET_SLACK_S = 10


def scrub(text: Any, *secrets: str) -> str:
    """Remove every secret from a string. The one place the token is censored.

    THE TOKEN IS IN THE URL. `https://api.telegram.org/bot<token>/sendMessage` --
    so a `urllib` failure, an `HTTPError`, a traceback, a retry message, anything
    that quotes what it was doing, carries the secret. Scrub before logging,
    before replying, and before writing an audit row.

    A falsy secret is skipped: a scrubber built with `""` would otherwise splice
    the redaction marker between every character of every message.
    """
    out = text if isinstance(text, str) else str(text)
    for secret in secrets:
        if secret:
            out = out.replace(secret, "<redacted>")
    return out


@dataclass(frozen=True)
class Config:
    """Everything the bot needs from outside itself. No defaults for the secrets."""

    token: str
    allowed_chat_ids: frozenset[int]
    link_dir: str
    ttl_s: float = policy.DEFAULT_TTL_S
    audit_path: str = ""

    @staticmethod
    def from_env(env: Mapping[str, str] | None = None) -> "Config":
        """Read the environment, or refuse to start. There is no partial start.

        The token and the allowlist are NEVER committed, never defaulted, never
        written into a test fixture. A bot that starts without an allowlist would
        answer everyone, and a default that "works" is the one somebody ships.

        A malformed allowlist raises rather than dropping the bad entry: an
        allowlist quietly one id short is a bot that ignores the operator, and he
        would debug the arm before he debugged the comma.
        """
        env = os.environ if env is None else env
        missing = [k for k in ("ARM_TELEGRAM_BOT_TOKEN",
                               "ARM_TELEGRAM_ALLOWED_CHAT_IDS",
                               "ARM_TELEGRAM_LINK_DIR") if not env.get(k)]
        if missing:
            raise SystemExit(
                "refusing to start -- missing " + ", ".join(missing) + ".\n"
                "ARM_TELEGRAM_BOT_TOKEN         the Bot API token, from @BotFather\n"
                "ARM_TELEGRAM_ALLOWED_CHAT_IDS  comma-separated chat ids that may command the arm\n"
                "ARM_TELEGRAM_LINK_DIR          the directory hold_arm.py writes its three files into")

        raw = env["ARM_TELEGRAM_ALLOWED_CHAT_IDS"]
        ids = []
        for token_ in raw.split(","):
            token_ = token_.strip()
            if not token_:
                continue
            try:
                ids.append(int(token_))
            except ValueError:
                raise SystemExit(
                    f"refusing to start -- {token_!r} in ARM_TELEGRAM_ALLOWED_CHAT_IDS "
                    "is not a chat id. Chat ids are integers and may be negative "
                    "(a group). Nothing is guessed here.")
        if not ids:
            raise SystemExit(
                "refusing to start -- ARM_TELEGRAM_ALLOWED_CHAT_IDS is empty. "
                "An empty allowlist is not 'allow everyone'; it is a bot nobody "
                "can command, and it is far more likely to be a typo.")

        link_dir = env["ARM_TELEGRAM_LINK_DIR"]
        return Config(
            token=env["ARM_TELEGRAM_BOT_TOKEN"],
            allowed_chat_ids=frozenset(ids),
            link_dir=link_dir,
            ttl_s=_ttl_from_env(env),
            audit_path=env.get("ARM_TELEGRAM_AUDIT")
            or os.path.join(link_dir, "arm_telegram_audit.jsonl"),
        )


#: The widest armed window this will accept from the environment.
#:
#: A DESK CHOICE, NOT A MEASUREMENT -- the same status as `FRESHNESS_WINDOW_S`.
#: The real TTL is Mike's number and PRD Q17-Q3 is still unanswered; this does
#: not pick it, it only refuses the values that cannot be what he meant.
#:
#: WHY A CEILING EXISTS AT ALL. Arming is the whole safety contract on this
#: surface: it bounds how long a stale intention stays valid, and it is the only
#: thing between "Mike typed /arm at the bench" and "a phone in a pocket runs a
#: transition tomorrow". `float()` accepted anything, so one extra digit turned
#: that bound into a permanent grant and NOTHING WOULD HAVE SAID SO. An hour is
#: already far longer than any bench session this plan describes.
MAX_TTL_S = 3600.0


def _ttl_from_env(env: Mapping[str, str]) -> float:
    """`ARM_TELEGRAM_TTL_S`, or refuse to start. Never a silent fallback.

    `0` and a negative both parsed happily and produced a window that is expired
    before it opens -- so every `/go` refuses `window_expired` and the operator
    debugs the arm before he debugs the variable. A non-numeric value raised a
    bare `ValueError` traceback instead of the readable `SystemExit` the other
    three variables get. Same class of mistake as an allowlist quietly one id
    short, and the same answer: refuse loudly, guess nothing.
    """
    raw = env.get("ARM_TELEGRAM_TTL_S")
    if raw is None or not str(raw).strip():
        return policy.DEFAULT_TTL_S
    try:
        ttl = float(str(raw).strip())
    except ValueError:
        raise SystemExit(
            f"refusing to start -- ARM_TELEGRAM_TTL_S is {raw!r}, which is not a "
            "number of seconds. Nothing is guessed here.")
    # Chained, not `ttl <= 0 or ttl > MAX_TTL_S`: that form lets `nan`
    # straight through, because every comparison against `nan` is False.
    # `float('nan')` is a perfectly ordinary thing for a shell to produce.
    if not (0 < ttl <= MAX_TTL_S):
        raise SystemExit(
            f"refusing to start -- ARM_TELEGRAM_TTL_S is {ttl:g} s, outside "
            f"0 < ttl <= {MAX_TTL_S:g}. A window that is expired before it opens "
            "refuses every /go for a reason nobody can see, and a window measured "
            "in years is not a window at all -- arming is the only thing bounding "
            "how long a stale intention stays valid.")
    return ttl


class TransportError(RuntimeError):
    """A Bot API call failed. The message is ALREADY SCRUBBED when this is raised."""


class TelegramTransport:
    """`urllib` against the Bot API. Injectable, so no test ever reaches a network.

    Parameters go in the POST body rather than the query string. The token is
    still in the path -- the Bot API gives no choice about that -- but a chat id
    and the reply text stay out of any URL that might be logged by something
    else.
    """

    def __init__(self, token: str, base: str = "https://api.telegram.org",
                 opener: Callable[..., Any] = urllib.request.urlopen):
        self._token = token
        self._base = base.rstrip("/")
        self._opener = opener

    def _scrub(self, text: Any) -> str:
        return scrub(text, self._token)

    def call(self, method: str, params: Mapping[str, Any],
             timeout: float = 30.0) -> Any:
        """One Bot API method. Returns `result`; raises `TransportError`, scrubbed."""
        url = f"{self._base}/bot{self._token}/{method}"
        data = urllib.parse.urlencode(
            {k: v for k, v in params.items() if v is not None}).encode("utf-8")
        request = urllib.request.Request(url, data=data)
        try:
            with self._opener(request, timeout=timeout) as response:
                payload = json.loads(response.read().decode("utf-8"))
        except Exception as exc:                      # noqa: BLE001 -- see below
            # Deliberately broad. urllib raises HTTPError, URLError, socket
            # timeouts, and json raises its own -- and EVERY one of them may
            # carry the URL, which carries the token. The catch-all exists so
            # there is exactly one place that scrubbing has to be right.
            raise TransportError(f"{method} failed: {self._scrub(exc)}") from None
        if not payload.get("ok"):
            raise TransportError(f"{method} refused: {self._scrub(payload)}")
        return payload.get("result")

    def get_updates(self, offset: int | None, timeout: int) -> list[dict[str, Any]]:
        return self.call("getUpdates", {
            "offset": offset,
            "timeout": timeout,
            # Only plain messages. An EDITED message is a command whose text
            # changed after it was sent, and there is no honest way to act on
            # that -- so the API is told not to send them at all.
            "allowed_updates": json.dumps(["message"]),
        }, timeout=timeout + SOCKET_SLACK_S) or []

    def send_message(self, chat_id: int, text: str) -> Any:
        return self.call("sendMessage", {"chat_id": chat_id, "text": text})

    def download(self, file_id: str) -> bytes:
        """Fetch a file's bytes. Two calls: `getFile`, then the file URL."""
        info = self.call("getFile", {"file_id": file_id})
        path = (info or {}).get("file_path")
        if not path:
            raise TransportError("getFile returned no file_path")
        url = f"{self._base}/file/bot{self._token}/{path}"
        try:
            with self._opener(url, timeout=60) as response:
                return response.read()
        except Exception as exc:                      # noqa: BLE001 -- token in the URL
            raise TransportError(f"download failed: {self._scrub(exc)}") from None


class Bot:
    """One long-poll loop, one arming window, one command outstanding at a time.

    Everything with a clock, a socket or a file underneath it is injected, which
    is what lets `tests/test_bot.py` drive the whole stack against the fake
    daemon without touching a network, a port or a real second of time.
    """

    def __init__(self, config: Config, link: ArmLink, transport: TelegramTransport,
                 audit_log: audit.AuditLog,
                 mono: Callable[[], float] = time.monotonic,
                 wall: Callable[[], float] = time.time,
                 sleep: Callable[[float], None] = time.sleep,
                 transcribe: Callable[[bytes, str], str | None] = voice.transcribe,
                 logger: Callable[[str], None] | None = None):
        self.config = config
        self.link = link
        self.transport = transport
        self.audit = audit_log
        self.mono = mono
        self.wall = wall
        self.sleep = sleep
        self.transcribe = transcribe
        self.logger = logger or (lambda line: print(line, file=sys.stderr, flush=True))
        self.state = policy.State()
        self.offset: int | None = None

    # -- the one secret, censored in one place ------------------------------

    def _scrub(self, text: Any) -> str:
        return scrub(text, self.config.token)

    # -- the wire ------------------------------------------------------------

    def _send(self, line: str) -> str:
        """THE ONLY PLACE THIS PACKAGE WRITES TO `arm_cmd.txt`. One choke point.

        Every protocol line -- a transition waypoint, `STP`, and the `STA` this
        module composes itself while settling -- passes the same allowlist of
        verbs. `policy._guard` cannot cover the `STA`, because policy never sees
        it: the settle poll is invented at this layer, and a verb allowlist with
        a hole in it for the one line the pure layer does not vet is not an
        allowlist.

        `EST`, `DIS` and `CLR` are what this is really for. `EST` and the
        watchdog DETACH every joint and the arm falls.

        It calls `policy.guard_line()` rather than re-checking the verb here,
        because a second copy of the allowlist is a copy that drifts -- and the
        one it drifts away from is the one enforcing "no gripper, no J0".
        A LINE THAT NEVER GOT A REPLY IS RETRACTED, AND THAT HOLE WAS REAL.
        `ArmLink.send()` writes the command file and then waits; when the reply
        never lands it raises -- and the line IS STILL SITTING IN `arm_cmd.txt`.
        `hold_arm.main()` truncates the LOG on the way up (line 96) and NEVER
        touches `CMD`, and the loop's very first pass consumes whatever is there
        (line 177) and sends it to the board.

        So a `MOV` stranded by a daemon that died at a phase boundary is a
        motion command QUEUED FOR THE NEXT DAEMON. It runs whenever the daemon
        is next started -- that evening, the next morning -- with no armed
        window, no declared pose, nobody in the room, and nothing anywhere
        saying it would happen.

        That is precisely the hazard `policy.decide()` already refuses to create
        for `/stop` on `NO LINK`, in those words. This path created it anyway,
        because the guard was on the DECISION and the hole was in the WRITE.
        Found by probing, not by reading; the reproduction is
        `test_a_stranded_MOV_is_not_left_queued_for_the_next_daemon`.

        WHAT THE RETRACTION CANNOT DO, SAID PLAINLY: it cannot un-send a line a
        RUNNING daemon has already read out of the file but not yet answered
        within the timeout. It closes the "sits there for the next daemon" hole,
        which is the one that fires with nobody present. IT IS NOT AN ABORT, and
        no copy anywhere may call it one.
        """
        policy.guard_line(line)
        try:
            return self.link.send(line)
        except BaseException:
            # BaseException deliberately: `ArmLink.send()` raises `SystemExit`,
            # and a Ctrl-C at the terminal lands here too -- both leave exactly
            # the same stranded line. Nothing is swallowed; it is re-raised
            # untouched on the next line.
            self._retract(line)
            raise

    def _retract(self, line: str) -> None:
        """Clear `arm_cmd.txt`, but ONLY if it still holds exactly our line.

        NEVER A BLIND TRUNCATE. Both this and the daemon open that file `"w"`
        and the bot cannot enforce sole-writer (finding 6), so clearing a file
        whose contents we do not recognise would make this package the second
        writer it warns everybody else about. If the line is gone, the daemon
        consumed it and there is nothing to retract. If it is something else,
        somebody else owns the file and it is not ours to clear.
        """
        try:
            with open(self.link.cmd, "r", encoding="utf-8") as fh:
                pending = fh.read()
        except OSError:
            return
        if pending.strip() != line.strip():
            return
        try:
            with open(self.link.cmd, "w", encoding="utf-8") as fh:
                fh.write("")
        except OSError:
            # Best effort. A failed retraction must not mask the original
            # failure, which is the thing the operator actually needs to read.
            self.logger(self._scrub(
                f"could not retract {line!r} from the command file -- the next "
                "daemon to start may run it"))

    def _reply(self, chat_id: int, text: str) -> None:
        """Send one message, scrubbed and length-capped. Never raises."""
        body = self._scrub(text)
        if len(body) > MAX_REPLY_CHARS:
            body = body[:MAX_REPLY_CHARS] + "\n[...truncated -- see the audit log]"
        try:
            self.transport.send_message(chat_id, body)
        except Exception as exc:                      # noqa: BLE001
            # A failed reply must not take down the update it belongs to, and
            # must not re-raise into the error handler that called it.
            self.logger(self._scrub(f"reply to {chat_id} failed: {exc}"))

    def _row(self, **kw: Any) -> None:
        ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(self.wall()))
        self.audit.write(audit.Row(ts=ts, **kw))

    # -- the loop ------------------------------------------------------------

    def drain_backlog(self) -> int:
        """Discard everything Telegram queued while the bot was not running.

        THE FAILURE THIS PREVENTS IS THE ONE THE WHOLE DESIGN EXISTS TO PREVENT,
        and neither the PRD nor the plan noticed it. Telegram holds undelivered
        updates for about 24 hours and replays them on the first `getUpdates`.
        So: the operator sends `/arm storage` and `/go pick` at the bench, the
        bot is not running, he walks away. Tomorrow the bot starts, arms with
        `now` = startup -- the TTL is measured from delivery, not from when the
        message was sent -- and immediately runs the transition behind it. The
        arm moves for consent that expired yesterday, with nobody in the room.

        That is PRD section 6 exactly ("a queued motion command asserts consent
        that has already expired"), arriving through Telegram's queue instead of
        through one the bot built. Refusing to queue internally does nothing
        about it.

        `getUpdates(offset=-1)` returns only the most recent update, which is
        enough to learn the id to skip past. The count is therefore "at least
        one", not an exact number, and the audit row says so rather than
        inventing a total.
        """
        try:
            latest = self.transport.get_updates(offset=-1, timeout=0)
        except TransportError as exc:
            self.logger(self._scrub(f"backlog drain failed: {exc}"))
            return 0
        if not latest:
            return 0
        # Defensive for the same reason `poll_once` is: `drain_backlog()` runs
        # from `run()` OUTSIDE its `try`, so an unreadable update here killed
        # the process before the loop ever started. It failed closed -- nothing
        # armed, nothing moved -- but the operator got a traceback instead of a
        # bot, while the daemon went on holding with no way to ask it anything.
        ids = [i for i in (self._update_id(u) for u in latest) if i is not None]
        if not ids:
            self.logger("backlog probe returned nothing with a readable "
                        "update_id -- not skipping past anything")
            return 0
        last_id = max(ids)
        self.offset = last_id + 1
        self._row(chat_id=None, username="", source="startup", text="",
                  decision="refused", reason="backlog_discarded",
                  note=f"discarded every update through update_id {last_id}")
        self.logger(REPLIES["backlog_discarded"].format(count=len(latest)))
        return len(latest)

    @staticmethod
    def _update_id(item: Any) -> int | None:
        """The update's id, or `None` if it cannot be read as one.

        Telegram always sends an integer `update_id`. A proxy, a mangled body or
        a partial read need not, and the difference between "cannot parse" and
        "crash" is the difference between skipping one message and the bot going
        silent forever -- see `poll_once`.
        """
        try:
            return int(item["update_id"])
        except (KeyError, TypeError, ValueError, IndexError):
            return None

    def poll_once(self) -> int:
        """One `getUpdates`, then handle what came back. Returns the count.

        THE OFFSET MUST ADVANCE PAST AN UPDATE THAT CANNOT BE PARSED, AND IT DID
        NOT. `int(update["update_id"])` sat outside `handle_update`'s try, so a
        `KeyError` propagated to `run()`, which caught it, logged, slept 3 s and
        polled again -- WITH THE OFFSET UNCHANGED. Telegram re-delivers an
        update until it is acknowledged by offset, so the bot looped on the same
        bad update forever: no reply to anything, nothing on the wire, not one
        audit row. Not a crash the operator can see -- a phone that silently
        stopped answering, which is the failure shape this project keeps
        re-learning.

        The message QUEUED BEHIND IT never ran either, so a `/stop` typed after
        a mangled update would have been read by nobody.

        An unreadable update is therefore audited and SKIPPED, and the loop goes
        on to the next one. Skipping is safe in the direction that matters: an
        update that cannot be parsed cannot be a command, so nothing is lost
        except a row saying it arrived.
        """
        updates = self.transport.get_updates(self.offset, POLL_TIMEOUT_S)
        for item in updates:
            update_id = self._update_id(item)
            if update_id is None:
                self._row(chat_id=None, username="", source="text", text="",
                          decision="refused", reason="unsupported_update",
                          note="update carried no readable update_id: "
                               f"{type(item).__name__}")
                continue
            self.offset = update_id + 1
            self.handle_update(item)
        return len(updates)

    def run(self) -> None:
        """Long-poll forever. One bad update never takes the loop down."""
        self.drain_backlog()
        self.logger(f"listening. allowlist: {sorted(self.config.allowed_chat_ids)}; "
                    f"link dir: {self.config.link_dir}; TTL {self.config.ttl_s:.0f} s")
        while True:
            try:
                self.poll_once()
            except Exception as exc:                  # noqa: BLE001
                # Scrubbed: a getUpdates failure carries the URL, and the token
                # is in the URL.
                self.logger(self._scrub(f"poll failed: {exc}"))
                self.sleep(3.0)

    # -- one update ----------------------------------------------------------

    def handle_update(self, update: Mapping[str, Any]) -> None:
        """GATE 1 IS HERE AND IT IS FIRST. Everything else is inside it."""
        message = update.get("message") or {}
        chat_id = (message.get("chat") or {}).get("id")
        user = message.get("from") or {}
        username = user.get("username") or user.get("first_name") or ""

        if not message:
            self._row(chat_id=chat_id, username=username, source="text", text="",
                      decision="refused", reason="unsupported_update",
                      note=f"update carried no message: keys {sorted(update)}")
            return

        if chat_id is None or int(chat_id) not in self.config.allowed_chat_ids:
            # SILENCE. No reply, no error, no hint a bot exists -- and a row,
            # because this log is the only place the contact is recorded at all.
            self._row(chat_id=chat_id, username=username,
                      source="voice" if "voice" in message else "text",
                      text=str(message.get("text", "")),
                      decision="refused", reason="not_allowlisted",
                      note="no reply sent -- the sender gets silence")
            return

        try:
            self._handle_allowed(int(chat_id), username, message)
        except (Exception, SystemExit) as exc:        # noqa: BLE001
            # SYSTEMEXIT IS DELIBERATE AND IT IS NOT PARANOIA. `ArmLink.send()`
            # raises `SystemExit` when the daemon never logs a reply for a
            # command -- it is lifted verbatim from `motion_verify.py`, a
            # command-line script, where exiting IS the right thing. But
            # `SystemExit` inherits from `BaseException`, so a plain
            # `except Exception` lets it straight through and THE BOT PROCESS
            # EXITS: silently, possibly mid-transition, with no reply and no
            # audit row. The arm stays held by the daemon so nothing falls, but
            # the operator is left with a phone that simply stopped answering.
            #
            # Caught HERE rather than fixed in `arm_link.py` on purpose: that
            # copy has to keep behaving exactly like `motion_verify.ArmLink` or
            # the Phase 1 drift guard stops meaning anything. The bot is what has
            # to be robust to it.
            #
            # NOT `BaseException` -- that would swallow KeyboardInterrupt and
            # leave no way to stop the bot from the terminal running it.
            detail = self._scrub(f"{type(exc).__name__}: {exc}")
            # VOID EVERYTHING. An update that threw part-way through dispatch may
            # have left a transition half-sent, so nothing here knows where the
            # arm is. Worse, `in_flight` left set refuses every later `/go` AND
            # every `/arm` with `move_in_flight` -- the bot wedges until somebody
            # guesses that `/stop` is the one command that clears it. The remedy
            # this project prescribes for not knowing where the arm is, is a
            # human looking at it and declaring a pose.
            self.state = policy.State()
            self._row(chat_id=int(chat_id), username=username, source="text",
                      text=str(message.get("text", "")), decision="refused",
                      reason="internal_error", note=detail)
            self._reply(int(chat_id), REPLIES["internal_error"] + "\n" + detail)

    def _handle_allowed(self, chat_id: int, username: str,
                        message: Mapping[str, Any]) -> None:
        source = "text"
        voice_seconds: float | None = None
        text = str(message.get("text") or "")

        if "voice" in message:
            source = "voice"
            note = message.get("voice") or {}
            voice_seconds = note.get("duration")
            spoken = self._transcribe_voice(note)
            if spoken is None:
                self._reply(chat_id, voice.VOICE_DISABLED_REPLY)
                self._row(chat_id=chat_id, username=username, source=source, text="",
                          decision="refused", reason="voice_not_enabled",
                          voice_seconds=voice_seconds,
                          note="transcribe() returned None -- nothing was sent")
                return
            # FROM HERE THE ROAD IS IDENTICAL. A transcription is text, with no
            # privileges of its own: same link check, same policy, same refusals.
            text = spoken

        link_state = self.link.observe(self.wall())
        now = self.mono()
        decision = policy.decide(self.state, text, now, link_state,
                                 ttl_s=self.config.ttl_s)
        # BEFORE dispatch, deliberately: `finish_motion` raises if `in_flight` is
        # not set, and `in_flight` lives on the state this decision just built.
        self.state = decision.new_state

        common = dict(chat_id=chat_id, username=username, source=source, text=text,
                      verdict=link_state.verdict, events=tuple(decision.events),
                      voice_seconds=voice_seconds)

        if decision.phases:
            # Reply BEFORE the transition runs -- it takes 5-15 s and a phone
            # showing nothing for that long reads as a bot that died.
            self._reply(chat_id, decision.reply)
            self._row(decision="accepted", reason=decision.reason, lines=(),
                      note="phases: " + ", ".join(ph.label for ph in decision.phases),
                      armed=self.state.armed(now),
                      remaining_s=round(self.state.remaining_s(now), 1), **common)
            self._run_transition(chat_id, username, source, text, decision,
                                 link_state.verdict, voice_seconds)
            return

        board = tuple(self._send(line) for line in decision.lines)
        body = decision.reply + ("\n\n" + "\n".join(board) if board else "")
        self._reply(chat_id, body)
        self._row(decision="accepted" if decision.accept else "refused",
                  reason=decision.reason, lines=tuple(decision.lines),
                  board_replies=board, armed=self.state.armed(now),
                  remaining_s=round(self.state.remaining_s(now), 1), **common)

    def _transcribe_voice(self, note: Mapping[str, Any]) -> str | None:
        """Fetch the audio and hand it to the seam. Any failure returns `None`.

        FAIL CLOSED. A download that half worked, a mime type that is not what
        was expected, a transport error -- all of them mean the bot does not know
        what was said, and not knowing what was said refuses.
        """
        file_id = note.get("file_id")
        if not file_id:
            return None
        try:
            data = self.transport.download(str(file_id))
        except Exception as exc:                      # noqa: BLE001
            self.logger(self._scrub(f"voice download failed: {exc}"))
            return None
        return self.transcribe(data, str(note.get("mime_type") or "audio/ogg"))

    # -- a transition, one phase at a time -----------------------------------

    def _run_transition(self, chat_id: int, username: str, source: str, text: str,
                        decision: policy.Decision, verdict: str,
                        voice_seconds: float | None) -> None:
        """Drive the phases in order, settling each one before starting the next.

        PHASE ORDERING IS THE SAFETY PROPERTY, NOT THE ENDPOINTS. Two poses can
        both be safe while the straight line between them drives the claw through
        the bench, the wire loom or the base housing -- which is why
        `arm-poses.csv` records an entry_path in capitals. Firing every line back
        to back moves joints across a phase boundary and IS that straight line.

        Four ways this stops early, all of them sending nothing further:
          * a board line came back `CL=1` or `ERR` -- the joint is not where it
            was asked, so the next phase would start from somewhere unknown;
          * the daemon RESTARTED -- see below;
          * the log window gained a `LATCHED` or `re-ENA` -- the daemon cleared a
            latch and re-enabled, snapping joints to their adopt angles;
          * a phase did not settle inside `SETTLE_TIMEOUT_S`.
        The first three are handed to `policy.finish_motion`, which owns the copy
        for them. The fourth is this layer's, because a settle poll is this
        layer's invention.

        THE RESTART CHECK WAS MISSING AND THE HOLE WAS REAL. `decide()` refuses a
        `/go` when `observe()` saw a restart, so a restart BETWEEN messages was
        always caught. A restart AT A PHASE BOUNDARY was not, and nothing here
        could have seen it:

          * the latch scan tails from `start`, and a restart TRUNCATES the log --
            so `tail(start)` seeks past EOF and returns an empty string, exactly
            the failure plan finding 3 describes, one layer over;
          * even tailing from 0 would find nothing, because `hold_arm.main()`
            logs plain `ENA` on the way up. Only `enable_all()` logs `re-ENA`, so
            the restart handshake carries NO `LATCH_MARKERS` at all;
          * every reply already collected is `CL=0`, so no other check fires.

        Measured against the fake daemon before the fix: `/arm storage`, `/go
        pick`, restart after the first phase settled -- the bot sent `MOV 3 36`,
        `MOV 4 140`, `MOV 5 165` and `MOV 1 8` into an arm whose shoulder had just
        snapped from 40 back to its adopt angle of 1, then replied
        "the declared pose is now pick" and left the window open. That is the
        wrist-and-elbow phase running with the claw back down on the mat -- the
        straight line between two safe poses that the phase ordering exists to
        prevent.

        `link.restarted()` is a PURE QUERY against the baseline `observe()` set
        at the top of this update, so it is safe to call once per phase and does
        not consume the edge. It reads False while a log merely grows.

        AND IT IS CHECKED ALONGSIDE A SECOND, INDEPENDENT DETECTOR, because on
        its own it rests on one thin thread. `restarted()` is `size < last_size`
        OR `first_line changed`. Measured in this rig: 1595 bytes before the
        restart and 1596 after, so THE SIZE HALF DOES NOT FIRE and the whole
        detection rests on two daemon starts landing in different `HH:MM:SS`.
        That is near-certain -- a restart cannot complete in under about eight
        seconds -- but it is one timestamp collision away from failing open, and
        failing open here runs the rest of a recorded path against an arm that
        just moved on its own.

        The second detector is exact and needs neither a size nor a clock: THE
        LOG MUST STILL CONTAIN THE COMMAND WE JUST WATCHED IT LOG. Every phase's
        reply was found by locating `CMD <line> -> ` in the log, so that marker
        is there by construction. A truncation removes it -- `tail(start)` either
        seeks past EOF and returns nothing, or returns a fresh handshake that was
        never asked to move a joint. Its absence is proof the log was replaced.

        A high-water mark on the size was tried first and REJECTED: `high` at the
        first phase boundary is still the pre-transition size, and a fresh
        handshake is the same length as the one it replaced, so it read 1596 <
        1595 = False and let the transition run. Measured, not reasoned about.
        """
        start = self.link.offset()
        replies: list[str] = []
        stop_early = False
        restarted = False

        for phase in decision.phases:
            for line in phase.lines:
                replies.append(self._send(line))
            if any("CL=1" in r or r.strip().startswith("ERR") for r in replies):
                stop_early = True
                break
            if not self._settle(phase):
                self._abort_unsettled(chat_id, username, source, text, phase,
                                      replies, verdict, voice_seconds)
                return
            seen = self.link.tail(start)
            vanished = bool(phase.lines) and f"CMD {phase.lines[-1]} -> " not in seen
            if self.link.restarted() or vanished:
                restarted = stop_early = True
                break
            if any(m in seen for m in policy.LATCH_MARKERS):
                stop_early = True
                break

        window = self._transition_window(start, restarted)
        outcome = policy.finish_motion(self.state, replies, window, restarted=restarted)
        self.state = outcome.new_state
        now = self.mono()
        self._reply(chat_id, outcome.reply)
        self._row(chat_id=chat_id, username=username, source=source, text=text,
                  decision="accepted" if outcome.accept else "refused",
                  reason=outcome.reason,
                  # Sliced to the replies actually collected: a transition that
                  # stopped early must record only the lines that really went out.
                  lines=tuple(line for ph in decision.phases
                              for line in ph.lines)[:len(replies)],
                  board_replies=tuple(replies), verdict=verdict,
                  armed=self.state.armed(now),
                  remaining_s=round(self.state.remaining_s(now), 1),
                  events=("restarted",) if restarted
                  else ("stopped_early",) if stop_early else (),
                  voice_seconds=voice_seconds)

    def _transition_window(self, start: int, restarted: bool) -> str:
        """The log written since the transition began -- or the WHOLE new log.

        After a restart, "everything since I last looked" genuinely IS the whole
        file: `hold_arm.py:96` truncated it, so `tail(start)` seeks past EOF and
        hands back nothing at the one moment there is most to report. The
        handshake's `ENA <j> <adopt>` lines are precisely what the operator needs
        to see. `ArmLink.observe()` already resolves this the same way; this
        follows that precedent rather than inventing a second one.
        """
        return self.link.tail(0) if restarted else self.link.tail(start)

    def _settle(self, phase: policy.Phase) -> bool:
        """Wait until every joint in this phase is parked where it was told.

        The predicate is `cycle_poses.settle()`'s, unchanged: `MOV == "0"` AND
        `SET == <target>` for every joint in the phase. It is the one that ran 16
        clean cycles at the bench, so it is not re-derived here.

        `SET` and `MOV` are read from a FRESH `STA` pushed through the command
        channel -- never from `arm_status.txt`, which is only rewritten on the
        daemon's 5 s poll and therefore still holds PRE-MOVE state right after a
        `MOV`, saying `MOV=0`. A "done" read from that file is a lie shaped
        exactly like success.

        `ArmLink.sta_row()` is deliberately not used: it issues its own `STA` per
        joint, so a three-joint phase would put three commands on the wire per
        poll. One `STA`, then scan its rows.
        """
        deadline = self.mono() + SETTLE_TIMEOUT_S
        targets = dict(phase.targets)
        while self.mono() < deadline:
            rows = self._sta_rows()
            if all(ArmLink.field(rows.get(j, ""), "MOV") == "0"
                   and ArmLink.field(rows.get(j, ""), "SET") == str(v)
                   for j, v in targets.items()):
                return True
            self.sleep(SETTLE_POLL_S)
        return False

    def _sta_rows(self) -> dict[int, str]:
        """One `STA`, split into `{joint id: row}`. Unparsable rows are dropped."""
        rows: dict[int, str] = {}
        for line in self._send("STA").splitlines():
            line = line.strip()
            if not line.startswith("STA J"):
                continue
            try:
                rows[int(line.split()[1][1:])] = line
            except (IndexError, ValueError):
                continue
        return rows

    def _abort_unsettled(self, chat_id: int, username: str, source: str, text: str,
                         phase: policy.Phase, replies: Sequence[str], verdict: str,
                         voice_seconds: float | None) -> None:
        """A phase never arrived. Send NOTHING, void everything, say so.

        THE CHOICE, NAMED AS ONE: no `STP` is sent automatically. The instinct is
        to "make it safe", and on this arm there is no such command -- `STP` holds
        with the joints still driven and `EST` detaches and drops it. Sending
        anything here would also be this layer deciding, on its own, to command a
        physical arm that is somewhere nobody knows. The same doctrine as
        expiry-sends-nothing: the bot goes quiet, the daemon goes on holding, and
        the operator decides with his eyes on the arm. `/stop` is one message
        away and works unarmed.
        """
        self.state = policy.State()
        body = REPLIES["did_not_settle"].format(timeout=SETTLE_TIMEOUT_S)
        tail = "\n".join(replies[-len(phase.lines):]) if replies else "(none)"
        self._reply(chat_id, body + "\n" + tail)
        self._row(chat_id=chat_id, username=username, source=source, text=text,
                  decision="refused", reason="did_not_settle",
                  lines=tuple(phase.lines), board_replies=tuple(replies),
                  verdict=verdict, armed=False, remaining_s=0.0,
                  events=("did_not_settle",), voice_seconds=voice_seconds,
                  note=f"phase {phase.label!r} did not reach {dict(phase.targets)} "
                       f"in {SETTLE_TIMEOUT_S:.0f}s -- nothing further was sent")


def main() -> None:
    """Start the bot, or refuse to. Requires `hold_arm.py` to be running already.

    `ArmLink.__init__` raises `SystemExit` when `arm_hold.log` is absent, so a bot
    started before the daemon stops here with a readable message rather than
    idling and refusing every command later for a reason nobody can see.
    """
    config = Config.from_env()
    link = ArmLink(config.link_dir)
    transport = TelegramTransport(config.token)
    log = audit.AuditLog(config.audit_path, scrub=lambda s: scrub(s, config.token))
    Bot(config, link, transport, log).run()


if __name__ == "__main__":
    main()
