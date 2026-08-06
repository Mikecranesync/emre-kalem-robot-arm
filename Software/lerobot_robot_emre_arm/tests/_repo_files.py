"""Loading the two pure modules, and editing a copy of joint-limits.csv.

Shared by conftest.py and the test modules. Split out so the test modules import
a plain module rather than reaching into `conftest`.

WHY THE MODULES ARE LOADED BY FILE PATH INSTEAD OF `import`
    `lerobot_robot_emre_arm/__init__.py` imports `config_emre_arm`, `emre_arm` and
    `markers` EAGERLY AND ON PURPOSE -- `@RobotConfig.register_subclass("emre_arm")`
    only registers when `config_emre_arm.py` actually executes, so a lazy import
    would silently break `--robot.type=emre_arm` discovery while appearing to work.
    Those three modules import `lerobot`.

    Python executes a package's `__init__.py` before any of its submodules, so

        import lerobot_robot_emre_arm.observation

    raises ModuleNotFoundError on any host without lerobot -- which is every host
    these tests are meant to run on. Making the package's imports lazy to make the
    tests easier would trade a working plugin for a convenient test. So the tests
    load `observation.py` and `calibration.py` STRAIGHT FROM THEIR FILE PATHS,
    under a synthetic parent package registered in `sys.modules` first.

    The synthetic name is deliberately NOT `lerobot_robot_emre_arm`: whether or not
    the real distribution happens to be pip-installed on this host is then
    irrelevant, and the real `__init__.py` can never be reached.

WHAT IS NOT COVERED, SAID PLAINLY
    `emre_arm.py`, `config_emre_arm.py` and `markers.py` import lerobot and are NOT
    imported, executed or covered by anything here. Nothing in this directory opens
    a serial port, touches the arm bridge, or drives a joint.
"""

from __future__ import annotations

import csv
import importlib.machinery
import importlib.util
import sys
from pathlib import Path
from types import ModuleType

#: Deliberately not the real distribution name -- see the module docstring.
SYNTHETIC_PKG = "_emre_pkg_under_test"

TESTS_DIR = Path(__file__).resolve().parent
PKG_ROOT = TESTS_DIR.parent                        # Software/lerobot_robot_emre_arm
SRC_DIR = PKG_ROOT / "src" / "lerobot_robot_emre_arm"
REPO_ROOT = PKG_ROOT.parents[1]                    # repo root

#: The files calibration.py reads by default. The tests assert against these exact
#: committed files -- not against a fixture copy -- because the whole point of
#: validators H4 and H5 is that the three files agree with each other.
REAL_LIMITS_CSV = REPO_ROOT / "Software" / "arm-console" / "joint-limits.csv"
REAL_LOCK_DIR = REPO_ROOT / "Calibration_Notes" / "lock-artifacts"
REAL_WIRING_MAP = REPO_ROOT / "Software" / "wiring-map.csv"


def load_module(name: str) -> ModuleType:
    """Load `<SRC_DIR>/<name>.py` as `SYNTHETIC_PKG.<name>`. Idempotent."""
    full_name = f"{SYNTHETIC_PKG}.{name}"
    if full_name in sys.modules:
        return sys.modules[full_name]

    if SYNTHETIC_PKG not in sys.modules:
        pkg_spec = importlib.machinery.ModuleSpec(SYNTHETIC_PKG, None, is_package=True)
        pkg = importlib.util.module_from_spec(pkg_spec)
        # Gives the relative-import machinery somewhere to look, so
        # `from .observation import ...` inside calibration.py resolves.
        pkg.__path__ = [str(SRC_DIR)]
        sys.modules[SYNTHETIC_PKG] = pkg

    path = SRC_DIR / f"{name}.py"
    spec = importlib.util.spec_from_file_location(full_name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not build an import spec for {path}")
    module = importlib.util.module_from_spec(spec)
    # Registered BEFORE exec so a sibling relative import finds this module
    # rather than executing it a second time.
    sys.modules[full_name] = module
    spec.loader.exec_module(module)
    setattr(sys.modules[SYNTHETIC_PKG], name, module)
    return module


# ---------------------------------------------------------------------------
# Editing joint-limits.csv WITHOUT touching the committed one
# ---------------------------------------------------------------------------

class LimitsFile:
    """An in-memory, editable copy of joint-limits.csv.

    Every hard-validator test starts from the real file and changes ONE thing, so
    a failure names the change rather than a hand-typed fixture that drifted. The
    long `#` preamble is carried through verbatim, which means every one of those
    tests also exercises the loader's comment/blank-line skipping.
    """

    def __init__(self, text: str) -> None:
        self.preamble: list[str] = []
        self.header: list[str] = []
        self.rows: list[list[str]] = []
        self.trailing_raw: list[str] = []

        for line in text.splitlines():
            if not line.strip() or line.lstrip().startswith("#"):
                self.preamble.append(line)
                continue
            fields = next(csv.reader([line]))
            if not self.header:
                self.header = fields
            else:
                self.rows.append(fields)

    # -- lookups ----------------------------------------------------------
    def _col(self, column: str) -> int:
        return self.header.index(column)

    def _row(self, joint_id: int) -> list[str]:
        idx = self._col("joint_id")
        for row in self.rows:
            if row[idx].strip() == str(joint_id):
                return row
        raise KeyError(f"no row for joint {joint_id}")

    def get(self, joint_id: int, column: str) -> str:
        return self._row(joint_id)[self._col(column)]

    # -- edits ------------------------------------------------------------
    def set(self, joint_id: int, column: str, value: object) -> "LimitsFile":
        self._row(joint_id)[self._col(column)] = str(value)
        return self

    def rename_column(self, old: str, new: str) -> "LimitsFile":
        self.header[self._col(old)] = new
        return self

    def add_column(self, name: str, default: str = "") -> "LimitsFile":
        self.header.append(name)
        for row in self.rows:
            row.append(default)
        return self

    def drop_joint(self, joint_id: int) -> "LimitsFile":
        self.rows.remove(self._row(joint_id))
        return self

    def copy_joint_row(self, src_joint_id: int, new_joint_id: int) -> "LimitsFile":
        row = list(self._row(src_joint_id))
        row[self._col("joint_id")] = str(new_joint_id)
        self.rows.append(row)
        return self

    def duplicate_joint_row(self, joint_id: int) -> "LimitsFile":
        self.rows.append(list(self._row(joint_id)))
        return self

    def append_raw_line(self, line: str) -> "LimitsFile":
        self.trailing_raw.append(line)
        return self

    # -- output -----------------------------------------------------------
    def write(self, path: Path, *, allow_comma_in_field: bool = False) -> Path:
        def render(fields: list[str]) -> str:
            rendered = []
            for value in fields:
                if "," in value:
                    if not allow_comma_in_field:
                        raise AssertionError(
                            f"field {value!r} contains a comma; joint-limits.csv "
                            "forbids that and this helper will not silently emit it"
                        )
                    rendered.append('"%s"' % value)
                else:
                    rendered.append(value)
            return ",".join(rendered)

        out: list[str] = list(self.preamble)
        out.append(render(self.header))
        out.extend(render(row) for row in self.rows)
        out.extend(self.trailing_raw)
        path.write_text("\n".join(out) + "\n", encoding="utf-8")
        return path


def write_lock_artifact(
    directory: Path,
    joint_id: int,
    *,
    date: str,
    hhmm: str,
    min_deg: int,
    max_deg: int,
    home_deg: int,
    row_joint_id: int | None = None,
    name: str = "Base",
    pins: str = "D3",
    fw: str = "1.0.0",
    console: str = "1.2.0",
    columns: int | None = None,
    extra_rows: int = 0,
) -> Path:
    """Write one lock receipt in the console's positional, headerless format.

    `row_joint_id`, `columns` and `extra_rows` exist so tests can produce the
    malformed artifacts the loader is supposed to refuse.
    """
    jid_in_row = joint_id if row_joint_id is None else row_joint_id
    reply = f"OK LIM J{jid_in_row} MIN={min_deg} MAX={max_deg} CAL=1"
    fields = [
        f"{date} {hhmm.replace('-', ':')}",
        str(jid_in_row),
        name,
        pins,
        str(min_deg),
        str(max_deg),
        str(home_deg),
        '"%s"' % reply,
        fw,
        console,
        '"accepted commanded soft limits - not mechanical extremes"',
    ]
    if columns is not None:
        fields = fields[:columns]
    line = ",".join(fields)
    path = directory / f"calibration-row-J{joint_id}_{date}_{hhmm}.csv"
    path.write_text("\n".join([line] * (1 + extra_rows)) + "\n", encoding="utf-8")
    return path
