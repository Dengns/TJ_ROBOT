"""Configuration for Marvin SpaceMouse teleoperation."""

from __future__ import annotations

import ctypes.util
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# Robot connection and arm selection.
ROBOT_IP = os.environ.get("MARVIN_ROBOT_IP", "192.168.1.190")
ARM = os.environ.get("MARVIN_ARM", "A").upper()
ARM_TYPE = 0 if ARM == "A" else 1
CONFIG_PATH = os.environ.get("MARVIN_KINE_CONFIG", os.path.join(HERE, "ccs_m6_40.MvKDCfg"))

# Position mode safety defaults.
CONTROL_RATE_HZ = 50
VEL_RATIO = 10
ACC_RATIO = 10

# libspnav loading. Prefer a demo-local shared object if one is later copied in,
# otherwise use the system library name found by ldconfig.
_LOCAL_LIBSPNAV = os.path.join(HERE, "libspnav.so.0.4")
LIBSPNAV_PATH = (
    os.environ.get("LIBSPNAV_PATH")
    or (_LOCAL_LIBSPNAV if os.path.exists(_LOCAL_LIBSPNAV) else None)
    or ctypes.util.find_library("spnav")
    or "libspnav.so.0.4"
)

# SpaceMouse raw input.
DEADZONE = 40
SM_VEC3_TRANSLATION_SCALE = 1.0 / 350.0
SM_QUAT_ROTATION_RAD_PER_UNIT = math.pi / 350.0
SM_QUAT_ROTATION_MODE = "axis_angle"
SM_ROT_SO3_SOURCE = "measurement_log"

# Axis mapping: SpaceMouse [x, y, z, rx, ry, rz] -> robot base channels.
AXIS_MAP = [2, 0, 1, 5, 3, 4]
AXIS_SIGNS = [1, -1, 1, -1, 1, 1]
AXIS_ENABLE = [1, 1, 1, 0, 0, 0]

# Increment scaling and clamps. Translation is meters per cycle, rotation is
# radians per cycle after so(3) mapping.
TRANSLATION_SCALE = 0.0004 / 350.0
ROTATION_SCALE = 0.002 / 350.0
MAX_TRANSLATION_PER_CYCLE = 0.001
MAX_ROTATION_PER_CYCLE = 0.01
EMA_ALPHA = 1.0

# Workspace limits in robot base frame, meters. The startup FK pose is clamped
# only after applying user increments, so out-of-range IK is skipped safely.
WORKSPACE_MIN = [-0.5, -0.5, 0.0]
WORKSPACE_MAX = [0.5, 0.5, 0.7]

# Status logging.
TELEMETRY_INTERVAL_S = 0.5
