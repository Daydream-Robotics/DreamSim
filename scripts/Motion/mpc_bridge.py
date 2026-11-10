"""Python access to robot-core's MPC path following (build/mpc_bridge.dll).

The DLL is built from native/tools/mpc_bridge.cpp by scripts/build_native.bat.
Frames and units are robot-core's: JerryIO field frame (field-centre origin,
+x forward, +y left), inches, theta 0 along +y and pi/2 along +x (clockwise
positive), wheel speeds in rad/s at the wheel.
"""

import ctypes
import shutil
import sys
import tempfile
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
LIBRARY_NAME = (
    "mpc_bridge.dll" if sys.platform == "win32" else "libmpc_bridge.so"
)
LIBRARY_PATH = REPO_ROOT / "build" / LIBRARY_NAME

RUNNING = 0
FINISHED = 1
# Still following, but the solve failed so the voltages are 0 V.
SOLVE_FAILED = 2
ERROR = -1

_double_p = ctypes.POINTER(ctypes.c_double)


def _shadow_copy():
    """Copy the built library to a temp file and return the copy's path.

    A loaded DLL can't be overwritten on Windows, and Isaac Sim keeps it
    loaded until it exits, so loading build/ directly would block rebuilds.
    Each build gets its own copy (named by its modification time), so
    rebuilding while Isaac Sim is open works and the next load picks it up.
    """
    stamp = LIBRARY_PATH.stat().st_mtime_ns
    folder = Path(tempfile.gettempdir()) / "dreamsim_mpc_bridge"
    folder.mkdir(exist_ok=True)
    copy = folder / f"{stamp}_{LIBRARY_NAME}"
    if not copy.is_file():
        shutil.copy2(LIBRARY_PATH, copy)
        # Clear out older copies; ones still loaded are locked, so skip them.
        for old in folder.glob(f"*_{LIBRARY_NAME}"):
            if old != copy:
                try:
                    old.unlink()
                except OSError:
                    pass
    return copy


def _load_library():
    if not LIBRARY_PATH.is_file():
        raise RuntimeError(
            f"{LIBRARY_PATH} not found. Build it with: "
            "scripts\\build_native.bat"
        )
    library = ctypes.CDLL(str(_shadow_copy()))

    library.mpc_bridge_create.argtypes = [ctypes.c_char_p]
    library.mpc_bridge_create.restype = ctypes.c_void_p
    library.mpc_bridge_destroy.argtypes = [ctypes.c_void_p]
    library.mpc_bridge_destroy.restype = None
    library.mpc_bridge_path_count.argtypes = [ctypes.c_void_p]
    library.mpc_bridge_path_count.restype = ctypes.c_int
    library.mpc_bridge_start_pose.argtypes = (
        [ctypes.c_void_p] + [_double_p] * 3
    )
    library.mpc_bridge_start_pose.restype = None
    library.mpc_bridge_sample_count.argtypes = [ctypes.c_void_p, ctypes.c_int]
    library.mpc_bridge_sample_count.restype = ctypes.c_int
    library.mpc_bridge_samples.argtypes = (
        [ctypes.c_void_p, ctypes.c_int] + [_double_p] * 3 + [ctypes.c_int]
    )
    library.mpc_bridge_samples.restype = ctypes.c_int
    library.mpc_bridge_reference_speeds.argtypes = (
        [ctypes.c_void_p, ctypes.c_int, _double_p, ctypes.c_int]
    )
    library.mpc_bridge_reference_speeds.restype = ctypes.c_int
    library.mpc_bridge_closest_point.argtypes = (
        [ctypes.c_void_p] + [_double_p] * 2
    )
    library.mpc_bridge_closest_point.restype = ctypes.c_int
    library.mpc_bridge_set_path.argtypes = [ctypes.c_void_p, ctypes.c_int]
    library.mpc_bridge_set_path.restype = ctypes.c_int
    library.mpc_bridge_step.argtypes = (
        [ctypes.c_void_p] + [ctypes.c_double] * 7 + [_double_p] * 4
    )
    library.mpc_bridge_step.restype = ctypes.c_int
    return library


class MpcBridge:
    """One loaded JerryIO file and the MPC following its paths."""

    def __init__(self, path_file):
        # Set first so close() works even if loading fails below.
        self._handle = None
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

    def samples(self, index):
        """A path's spline points as [(x in, y in, v)], exactly what the
        MPC follows. v is in the path file's speed units.
        """
        count = self._library.mpc_bridge_sample_count(self._handle, index)
        if count < 0:
            raise IndexError(f"No path {index} (have {self.path_count})")
        array_type = ctypes.c_double * count
        xs, ys, vs = array_type(), array_type(), array_type()
        count = self._library.mpc_bridge_samples(
            self._handle, index, xs, ys, vs, count
        )
        return list(zip(xs[:count], ys[:count], vs[:count]))

    def reference_speeds(self, index):
        """The speed (in/s) robot-core's MPC reference targets at each of
        a path's samples, as if the robot tracked it perfectly from rest.
        Same order as samples(index).
        """
        count = self._library.mpc_bridge_sample_count(self._handle, index)
        if count < 0:
            raise IndexError(f"No path {index} (have {self.path_count})")
        speeds = (ctypes.c_double * count)()
        count = self._library.mpc_bridge_reference_speeds(
            self._handle, index, speeds, count
        )
        return list(speeds[:count])

    def closest_point(self):
        """The path point the follower last matched the robot to, as
        (x in, y in), or None if no path is set. Matched on the
        delay-compensated pose, so it can sit slightly ahead of the robot.
        """
        x, y = ctypes.c_double(), ctypes.c_double()
        status = self._library.mpc_bridge_closest_point(
            self._handle, ctypes.byref(x), ctypes.byref(y)
        )
        return (x.value, y.value) if status == 0 else None

    def set_path(self, index):
        """Start following path index (0-based)."""
        if self._library.mpc_bridge_set_path(self._handle, index) != 0:
            raise IndexError(f"No path {index} (have {self.path_count})")

    def step(self, x, y, theta, omega_left, omega_right, battery_volts=12.0,
             age=0.0):
        """Run one control step.

        Returns (status, v_left, v_right, distance_left, off_path): voltages
        in V; distance left along the path and distance from the closest
        path point in inches. Apply the voltages robot-core's
        MPC_COMMAND_DELAY after this call: the MPC plans for that.
        """
        v_left, v_right = ctypes.c_double(), ctypes.c_double()
        distance_left, off_path = ctypes.c_double(), ctypes.c_double()
        status = self._library.mpc_bridge_step(
            self._handle, x, y, theta, omega_left, omega_right,
            battery_volts, age,
            ctypes.byref(v_left), ctypes.byref(v_right),
            ctypes.byref(distance_left), ctypes.byref(off_path),
        )
        return (
            status, v_left.value, v_right.value, distance_left.value,
            off_path.value,
        )
