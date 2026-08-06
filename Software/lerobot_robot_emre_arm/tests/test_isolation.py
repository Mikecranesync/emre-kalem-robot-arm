"""Guards on HOW the two pure modules get loaded.

These are not tests of the adapter. They are tests of the test harness, and they
exist so that nobody later "simplifies" `_repo_files.load_module()` into a plain
`import lerobot_robot_emre_arm.observation` -- which would fail on any host
without lerobot -- and then "fixes" that by making the package's eager imports
lazy, which would silently break `--robot.type=emre_arm` discovery.

If one of these fails, read `src/lerobot_robot_emre_arm/__init__.py` before
changing anything.
"""

from __future__ import annotations

import sys

from _repo_files import SRC_DIR, SYNTHETIC_PKG


def test_the_modules_are_loaded_under_the_synthetic_package(observation, calibration):
    assert SYNTHETIC_PKG in sys.modules
    assert observation.__name__ == f"{SYNTHETIC_PKG}.observation"
    assert calibration.__name__ == f"{SYNTHETIC_PKG}.calibration"


def test_the_loaded_modules_are_the_real_source_files(observation, calibration):
    # A synthetic package name must not mean a synthetic module: these are the
    # exact files that ship in the distribution.
    assert observation.__file__ == str(SRC_DIR / "observation.py")
    assert calibration.__file__ == str(SRC_DIR / "calibration.py")


def test_the_packages_eager_init_was_never_executed():
    # The real package name stays unbound. Whether or not the distribution is
    # pip-installed on this host is therefore irrelevant to this suite.
    assert "lerobot_robot_emre_arm" not in sys.modules


def test_loading_the_pure_modules_pulls_in_no_heavy_dependency(observation, calibration):
    # `observation.py` and `calibration.py` claim to depend on nothing outside the
    # standard library. If that ever stops being true, board-free testing stops
    # being possible and this is where it shows up.
    for heavy in ("lerobot", "cv2", "serial", "numpy"):
        assert heavy not in sys.modules, heavy


def test_the_lerobot_dependent_modules_are_not_loaded():
    for name in ("emre_arm", "config_emre_arm", "markers", "transport"):
        assert f"{SYNTHETIC_PKG}.{name}" not in sys.modules, name


def test_the_package_init_still_imports_eagerly():
    # The trap this harness exists to work around must still be a trap. If these
    # imports ever become lazy or conditional, LeRobot plugin discovery is broken
    # and this suite's whole approach is unnecessary -- both are worth noticing.
    source = (SRC_DIR / "__init__.py").read_text(encoding="utf-8")
    for line in ("from .config_emre_arm import", "from .emre_arm import",
                 "from .markers import"):
        assert f"\n{line}" in source, line


def test_nothing_here_reaches_the_board_or_the_bridge():
    # Belt and braces on the one rule that is not about software: this adapter
    # owns the serial port outright, and a test run must never contend for it.
    for name in ("serial", "serial.tools.list_ports"):
        assert name not in sys.modules, name
    assert f"{SYNTHETIC_PKG}.transport" not in sys.modules
