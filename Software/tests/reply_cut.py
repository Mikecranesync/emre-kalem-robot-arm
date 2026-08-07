"""Cut a daemon-log reply at its own boundary, and read a clamp flag honestly.

WHY THIS IS ONE MODULE AND NOT THREE COPIES. Three tools read replies out of
hold_arm.py's log -- cycle_poses.py, motion_verify.py and goto_pose.py -- and all
three had the same defect because all three had their own copy of the same six
lines. A rule that lives in three places is a rule that goes stale in two of
them. (arm-telegram/arm_link.py keeps its own implementation on purpose: the bot
must stay standard-library-only and self-contained. test_reply_cut.py pins the
two against each other so they cannot diverge silently.)

THE DEFECT, OBSERVED ON A LIVE RUN 2026-08-07, and it is the safety-relevant kind.
`send()` wrote a command and returned everything after its marker to the END of
the log. But the daemon writes into that same log continuously -- a PNG heartbeat
and a STA poll every 5 s -- and a reply can be read before the rest of its own
line has flushed. A real pick->storage run produced:

    OK PNG UP=2480971 \\n\\n 08:31:02 OK MOV J1 REQ=88 SET=88 C

The command's own reply is truncated immediately before the clamp flag. Every
guard in every one of those tools tested `"CL=1" in reply`. On that string a
genuine clamp is INVISIBLE: the test returns False, the caller reads "clean", and
the run drives on past a joint that is not where it was told to be.

TWO HALVES, AND ONLY BOTH TOGETHER ARE A FIX.

  cut_reply()  ends the reply at the next line beginning HH:MM:SS. NOT at the
               next newline -- hold_arm.log() timestamps only the FIRST line of a
               multi-line message, so a naive newline cut truncates every STA to
               its J0 row and looks like it worked.

  clamped()    returns True / False / None. None means the reply carried no CL=
               field at all, and None must STOP the caller. An unreadable reply
               is not a pass. Fixing only the cutting would still leave a
               truncated reply reading as clean.
"""

from __future__ import annotations

import re

# hold_arm.log() prefixes each record with "%H:%M:%S ". Anything matching this at
# the start of a line inside a reply is the DAEMON talking, not the board.
_STAMP = re.compile(r"^\d\d:\d\d:\d\d ", re.M)


def cut_reply(raw: str) -> str:
    """Trim a tailed reply at the daemon's next timestamped line."""
    m = _STAMP.search(raw)
    return (raw[:m.start()] if m else raw).strip()


def clamped(reply: str) -> bool | None:
    """True = firmware clamped, False = clean, None = unreadable.

    None is NOT a pass. Callers must stop on it.
    """
    if "CL=" not in reply:
        return None
    return "CL=1" in reply
