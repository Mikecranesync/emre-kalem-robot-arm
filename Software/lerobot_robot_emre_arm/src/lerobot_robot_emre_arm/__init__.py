"""LeRobot plugin for the Emre Kalem 6-axis arm.

    pip install -e Software/lerobot_robot_emre_arm
    lerobot-record      --robot.type=emre_arm ...
    lerobot-teleoperate --robot.type=emre_arm ...

Discovery needs no change to LeRobot's source. It works because of four
conventions, all of which this package follows:

  1. the distribution is named `lerobot_robot_<name>`
  2. `EmreArm` is paired with `EmreArmConfig`
  3. the config lives in `config_emre_arm.py` beside `emre_arm.py`
  4. this file exposes both

IMPORTS ARE EAGER, ON PURPOSE. `@RobotConfig.register_subclass("emre_arm")` only
registers when config_emre_arm.py actually executes, so a lazy import to dodge a
missing `lerobot` would silently break `--robot.type=emre_arm` while appearing to
work. This package genuinely requires LeRobot; importing it without LeRobot
installed is expected to fail.

`observation.py` and `calibration.py` depend on nothing outside the standard
library, so they can be exercised while both LeRobot and the board are absent.
NOTE THAT `import lerobot_robot_emre_arm.observation` DOES NOT DO THAT: Python
executes this file before any submodule, and this file imports lerobot eagerly
and on purpose (above). The tests therefore load those two modules FROM THEIR
FILE PATHS with importlib, under a synthetic parent package, bypassing this
`__init__` entirely -- `tests/_repo_files.py` does the loading, and
`tests/test_observation_schema.py` and `tests/test_calibration_validators.py`
are where the schema and the calibration validators are tested. Run them with
`cd Software/lerobot_robot_emre_arm && python -m pytest`.

THE ONE RULE THAT IS NOT ABOUT SOFTWARE: this adapter opens the serial port
directly and is its sole owner. Close the arm console (and its bridge) before
running anything here. The wire protocol correlates replies by echoed verb with
no sequence numbers, so two transmitting clients cannot tell their replies apart.
"""

from .calibration import (
    ArmCalibration,
    CalibrationError,
    JointCalibration,
    load_calibration,
)
from .config_emre_arm import EmreArmConfig
from .emre_arm import EmreArm
from .markers import ArucoMarkerObserver, MarkerObserver, NullMarkerObserver
from .observation import (
    ACTION_NAMES,
    JOINT_IDS,
    JOINT_LABELS,
    OBS_SOURCE_HUMAN,
    OBS_SOURCE_MARKER,
    OBS_SOURCE_NONE,
    OBS_SOURCE_OCCLUDED,
    OBS_SOURCE_STALE,
    OBSERVATION_STATE_NAMES,
    RESERVED_JOINT_ID,
    JointFix,
    action_features,
    build_observation,
    observation_features,
)
from .transport import PortBusy, SerialLink

__version__ = "0.1.0"

__all__ = [
    # the pair LeRobot discovers
    "EmreArm",
    "EmreArmConfig",
    # observation contract
    "JointFix",
    "JOINT_IDS",
    "JOINT_LABELS",
    "RESERVED_JOINT_ID",
    "OBS_SOURCE_NONE",
    "OBS_SOURCE_STALE",
    "OBS_SOURCE_OCCLUDED",
    "OBS_SOURCE_MARKER",
    "OBS_SOURCE_HUMAN",
    "OBSERVATION_STATE_NAMES",
    "ACTION_NAMES",
    "action_features",
    "observation_features",
    "build_observation",
    # calibration
    "ArmCalibration",
    "JointCalibration",
    "CalibrationError",
    "load_calibration",
    # observers
    "MarkerObserver",
    "NullMarkerObserver",
    "ArucoMarkerObserver",
    # transport
    "SerialLink",
    "PortBusy",
]
