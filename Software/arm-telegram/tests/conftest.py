"""Test wiring for the arm-telegram package. Paths first, fixtures second.

WHY THE PATH BLOCK EXISTS AT ALL
    The suite must not depend on how pytest happened to be invoked. Running
    `python -m pytest` from `Software/arm-telegram` puts the CWD on `sys.path`
    and a bare `import arm_link` would work by accident; a plain `pytest` from
    the repo root does not, and the suite would fail to COLLECT for a reason
    that has nothing to do with the bot. Same motivation as the header of
    `Software/arm-sim/tests/test_protocol_conformance.py`.

    The imports are done at module scope, immediately after the path block, so
    a broken path fails at collection with a clear name rather than inside some
    fixture three tests later.

WHAT GOES ON THE PATH, AND THE ONE RULE THAT MATTERS
    | dir                     | what it supplies              | imports |
    |-------------------------|-------------------------------|---------|
    | `arm-telegram/`         | `arm_link.py`, `policy.py`    | stdlib  |
    | `arm-telegram/tests/`   | `fake_daemon.py`              | stdlib  |
    | `Software/arm-sim/`     | `arm_sim.py`, `fake_serial.py`| stdlib  |
    | `Software/arm-console/` | `hold_arm.py`                 | pyserial|
    | `Software/tests/`       | `motion_verify.py`, `cycle_poses.py` | cv2 + numpy |

    THE TESTS MAY IMPORT THE HEAVY MODULES. THE BOT MAY NOT. `hold_arm.py`
    imports `serial`; `motion_verify.py` and `cycle_poses.py` import `cv2` and
    `numpy` at module top level. The conformance test imports `hold_arm`
    deliberately (it asserts the model matches the real daemon's format) and
    the Phase 1 drift guard imports `motion_verify.ArmLink` deliberately. The
    shipped bot modules import none of them -- that is the whole point of
    lifting `ArmLink` into `arm_link.py` (plan finding 1), and Phase 3's `ast`
    guard is what enforces it. That guard parses `arm-telegram/*.py` only and
    EXCLUDES `tests/`, precisely because of the table above.

NOTHING HERE TOUCHES A SERIAL PORT, A CAMERA, OR THE REAL LINK DIRECTORY.
    `hold_arm.CMD` / `.LOG` / `.STATUS` are absolute paths into
    `Software/arm-console/`. Every fixture below is rooted in `tmp_path`. A
    stray `arm_cmd.txt` written into the real link directory is a protocol line
    QUEUED FOR THE NEXT REAL DAEMON RUN -- the bench being disconnected tonight
    does not make that harmless.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

_TESTS_DIR = Path(__file__).resolve().parent
_PKG_DIR = _TESTS_DIR.parent  # Software/arm-telegram
_SOFTWARE = _PKG_DIR.parent  # Software

#: Front-inserted in reverse so the package dir ends up first.
_PATH_ENTRIES = (
    _SOFTWARE / "tests",
    _SOFTWARE / "arm-console",
    _SOFTWARE / "arm-sim",
    _TESTS_DIR,
    _PKG_DIR,
)

for _entry in _PATH_ENTRIES:
    _s = str(_entry)
    if _s not in sys.path:
        sys.path.insert(0, _s)

from fake_daemon import FakeDaemon  # noqa: E402  (path block must run first)


@pytest.fixture
def link_dir(tmp_path: Path) -> Path:
    """An empty daemon link directory, under tmp_path. Never the real one."""
    d = tmp_path / "link"
    d.mkdir()
    return d


@pytest.fixture
def idle_daemon(link_dir: Path) -> FakeDaemon:
    """A fake daemon that has NOT started -- no log, no status file yet.

    `ArmLink.__init__` raises `SystemExit` against this, exactly as the lifted
    original does when `hold_arm.py` is not running.
    """
    return FakeDaemon(str(link_dir))


@pytest.fixture
def daemon(idle_daemon: FakeDaemon) -> FakeDaemon:
    """A started fake daemon: handshake logged, joints enabled, one poll done.

    The poll is not tidiness. `hold_arm.py` writes `arm_status.txt` only on its
    5 s poll and only `if sta:` (line 161), so a freshly started daemon has NO
    status file for up to five seconds and a bot reading it gets `NO LINK`.
    That is correct and refusing is the safe direction -- but almost every test
    below wants a daemon that is past that window, so this fixture polls once.
    `idle_daemon` is there for the tests that want the gap itself.
    """
    idle_daemon.start()
    idle_daemon.poll()
    return idle_daemon
