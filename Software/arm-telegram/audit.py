"""Append-only record of every decision the bot made. Accepts AND refusals.

WHY REFUSALS ARE IN HERE, WHICH IS THE PART PEOPLE TRIM
    A refusal history is the only thing that reveals probing. A stranger who
    messages the bot gets SILENCE -- no reply, no error, no hint a bot exists
    (PRD section 8) -- so the audit log is the ONLY place that contact is
    recorded anywhere. Drop refusals to save space and the surface goes quiet in
    both directions at once, which is indistinguishable from nothing happening.

    Same reasoning inside the allowlist: a run of `/go` refusals is what a
    forgotten phone in a pocket looks like, and a run of `window_expired` rows is
    what a TTL set too short looks like. Neither is visible from the accepts.

WHY JSONL AND NOT A NICE COLUMNAR TEXT FILE
    Because the daemon's replies are multi-line. One `STA` is eight lines -- six
    joint rows, a SYS row and `OK STA N=6` -- and the whole point of this file is
    that a reply is stored VERBATIM. A line-oriented format breaks its own
    one-row-per-decision promise on the very first `/status`, and the person
    reading it later is the person debugging, who needs the literal line. JSON
    escapes the newlines and `json` is standard library, so it costs nothing.

    Append-only means append-only: nothing in this module updates, rewrites,
    truncates or deletes a row. There is no rotation. If it grows, move it.

THE TOKEN NEVER REACHES DISK, AND THE GUARD IS HERE RATHER THAN AT THE CALLER
    A Bot API failure carries the URL and THE TOKEN IS IN THE URL, so any error
    text is a leak unless something scrubs it. `AuditLog` takes a `scrub`
    callable and applies it to EVERY string field of EVERY row, including strings
    nested in the tuple fields -- not because the caller is careless, but because
    a call site added six months from now will not remember. The caller scrubs
    too. Two layers, on the one secret in this package.

    `tests/test_bot.py` asserts the token appears nowhere in this file after an
    exception path that carries it.

NOTHING HERE IS EVIDENCE ABOUT AN ARM. A row saying `motion_complete` means every
board line came back `CL=0`. A board ack proves the firmware accepted a command;
it proves nothing about a shaft.
"""

from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass, fields
from typing import Any, Callable

#: The two ways a message can end. `refused` covers everything the bot declined
#: to act on, including a stranger's message and a startup backlog -- those were
#: refused too, just before anyone could see them.
DECISIONS = ("accepted", "refused")


@dataclass(frozen=True)
class Row:
    """One decision. The field list is PRD section 13's list, and a test pins it.

    `lines` is what was ACTUALLY PUT ON THE WIRE, never what was planned. A
    transition that aborts half way through must not leave a row claiming the
    later waypoints were sent -- that is the exact shape of lie this surface
    exists to avoid. The accepted-motion row therefore carries `lines=()` and
    names its phases in `note`; the outcome row carries everything that really
    went out, with the board's replies beside them.
    """

    ts: str                       #: local wall-clock time, `YYYY-MM-DD HH:MM:SS`
    chat_id: int | None           #: None when the update carried no chat at all
    username: str                 #: "" when Telegram sent none
    source: str                   #: "text" | "voice" | "startup"
    text: str                     #: the raw inbound text, or a transcription
    decision: str                 #: one of DECISIONS
    reason: str                   #: policy.REASONS, policy.OUTCOME_REASONS, or bot.BOT_REASONS
    lines: tuple[str, ...] = ()   #: protocol lines actually sent, in order
    board_replies: tuple[str, ...] = ()   #: the daemon's replies, verbatim, in order
    verdict: str = ""             #: LIVE | STALE | NO LINK at the moment of decision
    armed: bool = False           #: armed state AFTER the decision
    remaining_s: float = 0.0      #: TTL left after the decision, seconds
    events: tuple[str, ...] = ()  #: what the housekeeping pass fired: expired, restarted, latched...
    voice_seconds: float | None = None    #: duration only. NEVER the audio.
    note: str = ""                #: scrubbed error text, or a bot-layer note

    def __post_init__(self) -> None:
        if self.decision not in DECISIONS:
            raise ValueError(f"decision must be one of {DECISIONS}, got {self.decision!r}")


#: Every field name, for the schema drift guard in the tests.
FIELD_NAMES = tuple(f.name for f in fields(Row))


def _scrub_value(value: Any, scrub: Callable[[str], str]) -> Any:
    """Apply the scrubber to every string anywhere in a row's value."""
    if isinstance(value, str):
        return scrub(value)
    if isinstance(value, (list, tuple)):
        return [_scrub_value(v, scrub) for v in value]
    return value


class AuditLog:
    """Append-only JSONL. One row per decision, one line per row.

    Not thread-safe and does not need to be: the bot is a single long-poll loop
    handling one update at a time. If that ever stops being true, this needs a
    lock before it needs anything else.
    """

    def __init__(self, path: str, scrub: Callable[[str], str] | None = None):
        self.path = path
        #: Identity by default so this module is usable with no secret at all.
        #: `bot.py` always passes a real one.
        self.scrub = scrub or (lambda s: s)
        parent = os.path.dirname(os.path.abspath(path))
        if parent:
            os.makedirs(parent, exist_ok=True)

    def write(self, row: Row) -> dict[str, Any]:
        """Append one row. Returns the scrubbed dict that was written, for tests.

        Flushed on every write, deliberately. A buffered audit log loses exactly
        the rows written just before the thing that made you want to read it.
        """
        payload = {k: _scrub_value(v, self.scrub) for k, v in asdict(row).items()}
        with open(self.path, "a", encoding="utf-8") as fh:
            fh.write(json.dumps(payload, ensure_ascii=False) + "\n")
            fh.flush()
        return payload


def read(path: str) -> list[dict[str, Any]]:
    """Every row, in order. For tests and for reading the thing back by hand.

    A malformed line raises rather than being skipped: a log you cannot parse is
    a log you cannot trust, and silently dropping the unreadable row hides
    precisely the write that went wrong.
    """
    if not os.path.exists(path):
        return []
    with open(path, "r", encoding="utf-8") as fh:
        return [json.loads(ln) for ln in fh if ln.strip()]
