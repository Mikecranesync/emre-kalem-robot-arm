"""Fixtures for the schema and calibration-validator tests.

The mechanics -- and the reason `import lerobot_robot_emre_arm.observation` cannot
be used here -- live in `_repo_files.py`. Read that first.

Nothing in this directory opens a serial port, contacts the arm bridge, or drives
a joint. Nothing here writes to a file the repository tracks.
"""

from __future__ import annotations

import shutil
from pathlib import Path
from types import ModuleType

import pytest

from _repo_files import (
    REAL_LIMITS_CSV,
    REAL_LOCK_DIR,
    REAL_WIRING_MAP,
    LimitsFile,
    load_module,
)


@pytest.fixture(scope="session")
def observation() -> ModuleType:
    """`observation.py`, loaded by path. Pure stdlib; no lerobot, no board."""
    return load_module("observation")


@pytest.fixture(scope="session")
def calibration() -> ModuleType:
    """`calibration.py`, loaded by path. Pulls in `observation` as its sibling."""
    load_module("observation")
    return load_module("calibration")


@pytest.fixture(scope="session")
def real_cal(calibration):
    """`load_calibration()` over the files committed to this repository.

    Session-scoped and safe to share: `ArmCalibration` is frozen and the loader
    writes nothing.
    """
    return calibration.load_calibration(REAL_LIMITS_CSV, REAL_LOCK_DIR, REAL_WIRING_MAP)


@pytest.fixture
def limits():
    """Factory: `limits()` -> an editable copy of the real joint-limits.csv.

    `edited.write(tmp_limits_path)` returns a path to hand to `load_calibration()`.
    The committed file is only ever read.
    """
    text = REAL_LIMITS_CSV.read_text(encoding="utf-8-sig")

    def _make() -> LimitsFile:
        return LimitsFile(text)

    return _make


@pytest.fixture
def tmp_limits_path(tmp_path) -> Path:
    return tmp_path / "joint-limits.csv"


@pytest.fixture
def empty_lock_dir(tmp_path) -> Path:
    d = tmp_path / "no-locks"
    d.mkdir()
    return d


@pytest.fixture
def copied_lock_dir(tmp_path) -> Path:
    """A writable copy of the real lock artifacts, for tests that add or remove one."""
    d = tmp_path / "lock-artifacts"
    shutil.copytree(REAL_LOCK_DIR, d)
    return d
