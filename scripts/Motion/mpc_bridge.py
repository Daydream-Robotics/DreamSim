"""Python access to robot-core's MPC path following (build/mpc_bridge.dll).

The DLL is built from native/tools/mpc_bridge.cpp by scripts/build_native.bat.
Frames and units are robot-core's: JerryIO field frame (field-centre origin,
+x forward, +y left), inches, theta 0 along +y and pi/2 along +x (clockwise
positive), wheel speeds in rad/s at the wheel.
"""

import ctypes
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
LIBRARY_NAME = "mpc_bridge.dll" if sys.platform == "win32" else "libmpc_bridge.so"
LIBRARY_PATH = REPO_ROOT / "build" / LIBRARY_NAME

RUNNING = 0
FINISHED = 1
ERROR = -1

_double_p = ctypes.POINTER(ctypes.c_double)


def _load_library():
    if not LIBRARY_PATH.is_file():
        raise RuntimeError(
            f"{LIBRARY_PATH} not found. Build it with: scripts\\build_native.bat"
        )
    library = ctypes.CDLL(str(LIBRARY_PATH))

    library.mpc_bridge_create.argtypes = [ctypes.c_char_p]
    library.mpc_bridge_create.restype = ctypes.c_void_p
    library.mpc_bridge_destroy.argtypes = [ctypes.c_void_p]
    library.mpc_bridge_destroy.restype = None
    library.mpc_bridge_path_count.argtypes = [ctypes.c_void_p]
    library.mpc_bridge_path_count.restype = ctypes.c_int
    library.mpc_bridge_start_pose.argtypes = [ctypes.c_void_p] + [_double_p] * 3
    library.mpc_bridge_start_pose.restype = None
    library.mpc_bridge_set_path.argtypes = [ctypes.c_void_p, ctypes.c_int]
    library.mpc_bridge_set_path.restype = ctypes.c_int
    library.mpc_bridge_step.argtypes = (
        [ctypes.c_void_p] + [ctypes.c_double] * 7 + [_double_p] * 3
    )
    library.mpc_bridge_step.restype = ctypes.c_int
    return library


class MpcBridge:
    """One loaded JerryIO file and the MPC following its paths."""

    def __init__(self, path_file):
        self._library = _load_library()
        self._handle = self._library.mpc_bridge_create(
            str(path_file).encode("utf-8")
        )
        if not self._handle:
            raise RuntimeError(f"No paths loaded from {path_file}")

    def close(self):
        if self._handle:
            self._library.mpc_bridge_destroy(self._handle)
            self._handle = None

    def __del__(self):
        self.close()

    @property
    def path_count(self):
        return self._library.mpc_bridge_path_count(self._handle)

    def start_pose(self):
        """The file's start pose as (x in, y in, theta rad)."""
        x, y, theta = ctypes.c_double(), ctypes.c_double(), ctypes.c_double()
        self._library.mpc_bridge_start_pose(
            self._handle, ctypes.byref(x), ctypes.byref(y), ctypes.byref(theta)
        )
        return x.value, y.value, theta.value

    def set_path(self, index):
        """Start following path index (0-based)."""
        if self._library.mpc_bridge_set_path(self._handle, index) != 0:
            raise IndexError(f"No path {index} (have {self.path_count})")

    def step(self, x, y, theta, omega_left, omega_right, battery_volts=12.0,
             age=0.0):
        """Run one control step.

        Returns (status, v_left, v_right, distance_left) with voltages in V
        and distance in inches. Apply the voltages robot-core's
        MPC_COMMAND_DELAY after this call: the MPC plans for that.
        """
        v_left, v_right = ctypes.c_double(), ctypes.c_double()
        distance_left = ctypes.c_double()
        status = self._library.mpc_bridge_step(
            self._handle, x, y, theta, omega_left, omega_right,
            battery_volts, age,
            ctypes.byref(v_left), ctypes.byref(v_right),
            ctypes.byref(distance_left),
        )
        return status, v_left.value, v_right.value, distance_left.value
