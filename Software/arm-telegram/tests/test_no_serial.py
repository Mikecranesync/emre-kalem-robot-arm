"""The bot must never be able to open the serial port. Deterministic, not a promise.

WHY THIS IS A TEST AND NOT A CODE REVIEW NOTE
    The daemon is the SOLE owner of the port, with no fallback and no degraded
    mode, and the cost of breaking that is not abstract:

      * Opening the port DTR-RESETS THE BOARD. The firmware keeps nothing across
        a reset -- every limit reverts to the default 70-110, `MIR=UNKNOWN`,
        `CAL=0`.
      * J6's real range is 10-70, which lies ALMOST ENTIRELY BELOW that default.
        After a reset, everything under 70 silently clamps until `LIM 6 10 70 1`
        goes out again.
      * `GET /rx` is destructive, so a second reader steals the daemon's replies
        and the daemon starts missing its own acks.

    A reviewer reading a diff can miss `import serial`. This cannot.

WHY `ast` AND NOT `sys.modules`
    A `sys.modules` check is order-dependent inside a pytest session: another
    test imports `hold_arm` (which imports `serial`) and every later check
    false-passes. The syntax tree is what the file says, whatever ran before it,
    and it catches an import that only fires on one branch.

WHY THE `tests/` DIRECTORY IS EXCLUDED, WHICH IS LOAD-BEARING AND NOT LAZINESS
    The Phase 0 conformance test imports `hold_arm` DELIBERATELY -- it asserts
    the fake daemon matches the real daemon's file format -- and `hold_arm`
    imports `serial` and contains the literal port name. `tests/fake_daemon.py`
    names the real port in a comment explaining why it uses `SIM` instead.
    Widening this guard to `tests/` trips it on the very tests that exist to
    prove the model is faithful. The glob below is non-recursive for that reason.
"""

from __future__ import annotations

import ast
import glob
import os
import sys

_PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#: Every module that ships. Asserted as a subset of what the glob found, so a
#: rename or a moved file turns this suite RED instead of quietly scanning
#: nothing and passing. A guard that can pass by finding no files is not a guard.
SHIPPED = {"arm_link.py", "audit.py", "bot.py", "policy.py", "voice.py"}

#: The package's own modules. Everything else an import resolves to must be
#: standard library -- that is the "zero new dependencies" claim, checkable.
OWN_MODULES = {"arm_link", "audit", "bot", "policy", "voice"}


def _package_sources() -> dict[str, str]:
    """`{filename: source}` for the shipped modules only. Never `tests/`."""
    found = {os.path.basename(p): open(p, "r", encoding="utf-8").read()
             for p in glob.glob(os.path.join(_PKG, "*.py"))}
    assert SHIPPED <= set(found), (
        f"expected to scan {sorted(SHIPPED)}, found {sorted(found)} -- "
        "a renamed or moved module must fail this test, not skip it")
    return found


def _import_roots(src: str) -> set[str]:
    roots: set[str] = set()
    for node in ast.walk(ast.parse(src)):
        if isinstance(node, ast.Import):
            roots.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            roots.add((node.module or "").split(".")[0])
    return roots


def test_no_shipped_module_imports_serial():
    """`import serial` anywhere in the package is the bug this whole file exists for."""
    for name, src in _package_sources().items():
        assert "serial" not in _import_roots(src), (
            f"{name} imports serial. The daemon is the sole owner of the port; "
            "opening it DTR-resets the board and J6's range falls below the "
            "firmware default, so everything under 70 would clamp.")


def test_no_shipped_module_names_the_serial_port():
    """The port name appears nowhere -- not in code, not in a comment.

    Stricter than an `ast` literal scan on purpose. A port name in a comment is
    one copy-paste away from being a port name in a `serial.Serial(...)` call,
    and the cost of the rule is one word: shipped prose says "the serial port".
    """
    for name, src in _package_sources().items():
        assert "COM5" not in src, (
            f"{name} names the serial port. Say 'the serial port' instead -- "
            "nothing in this package may know which one it is.")


def test_the_package_imports_only_the_standard_library():
    """Zero new dependencies is the PRD's strongest true statement. Pin it.

    `motion_verify.py` and `cycle_poses.py` import cv2 and numpy at module top
    level, and `arm-serial-control` section 4 tells the next person to
    `from motion_verify import ArmLink` in a literal code snippet. Following that
    instruction would drag OpenCV into a Telegram bot that writes one line to a
    text file. `arm_link.py` and `policy.py` exist as lifted copies precisely so
    nobody has to -- and this is what stops the instruction being followed later.

    `sys.stdlib_module_names` rather than a hand-maintained allow-list: a list
    somebody has to update is a list that grows a wrong entry.
    """
    for name, src in _package_sources().items():
        outside = _import_roots(src) - sys.stdlib_module_names - OWN_MODULES
        assert not outside, (
            f"{name} imports {sorted(outside)}, which is outside the standard "
            "library. A new dependency needs a written licence note first "
            "(Apache-2.0 or MIT only) -- it is a stop-and-ask, not a decision "
            "made on the way past.")


# THE FORBIDDEN VERBS ARE NOT CHECKED HERE, AND THAT WAS DECIDED BY WRITING IT.
#
# The obvious guard is an `ast` scan for a string literal beginning `EST`, `DIS`
# or `CLR`. It was written, and it failed immediately -- on `policy.py`'s own
# `FORBIDDEN_VERBS = ("EST", "DIS", "CLR")`, which is the constant that ENFORCES
# the rule. A text scan cannot tell a denylist entry from a command, so it would
# have had to carry an exemption for the one file most worth checking, and an
# exempted guard is a guard nobody can trust.
#
# The real enforcement is runtime and it is stronger: `bot._send()` refuses any
# verb outside `policy.ALLOWED_VERBS` before it can reach `arm_cmd.txt`, and
# `policy._guard()` refuses one at construction. `test_bot.py` records EVERY line
# the bot writes across the whole refusal table and asserts on what actually
# reached the wire -- evidence rather than a grep.
