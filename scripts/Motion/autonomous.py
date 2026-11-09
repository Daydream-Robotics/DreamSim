"""Runs a JerryIO file's paths with robot-core's MPC, a physics step at a time.

Mirrors robot-core's Autonomous::run (jetson-nano/src/autonomous): paths run
back to back, the MPC solves every control period, each command is applied
MPC_COMMAND_DELAY after its solve, and the drive stops when a path finishes.
Shared by drive_controller.py (Isaac Sim) and test_mpc_bridge.py (offline),
so both behave the same. No Isaac Sim imports.

Poses here are robot-core's: field frame, inches, heading 0 along +y and
pi/2 along +x (clockwise).
"""

import drivetrain_config as config
import mpc_bridge


class PathResult:
    """How one path went."""

    def __init__(self, index):
        self.index = index
        self.steps = 0
        self.worst_off_path = 0.0
        self.final_off_path = 0.0
        self.failed_solves = 0
        self.finished = False

    @property
    def time(self):
        return self.steps / config.PHYSICS_STEPS_PER_SECOND

    def summary(self):
        outcome = "finished" if self.finished else "TIMED OUT"
        return (
            f"Path {self.index} {outcome} in {self.time:.2f} s: worst "
            f"{self.worst_off_path:.2f} in off the path, "
            f"{self.final_off_path:.2f} in off at the end, "
            f"{self.failed_solves} failed solves"
        )


class Autonomous:
    """Follows every path in a JerryIO file in order."""

    def __init__(self, path_file, log=print):
        self.log = log
        self.bridge = mpc_bridge.MpcBridge(path_file)
        self.results = []
        self.done = False
        self._step = 0
        # (physics step to apply at, v_left, v_right, brake)
        self._pending = []
        self._applied = (0.0, 0.0, True)
        self._start_path(0)

    def start_pose(self):
        """Where the robot must start: (x in, y in, heading rad)."""
        return self.bridge.start_pose()

    def close(self):
        self.bridge.close()

    def physics_step(self, x, y, heading, omega_left, omega_right):
        """Advance one physics step from the robot's current state.

        Returns (v_left, v_right, brake) to apply this step.
        """
        if not self.done and self._step % config.STEPS_PER_CONTROL_PERIOD == 0:
            self._control(x, y, heading, omega_left, omega_right)

        while self._pending and self._pending[0][0] <= self._step:
            self._applied = self._pending.pop(0)[1:]

        if not self.done:
            self.results[-1].steps += 1
            limit = config.MPC_PATH_TIMEOUT * config.PHYSICS_STEPS_PER_SECOND
            if self.results[-1].steps > limit:
                self.log(self.results[-1].summary())
                self.log("Autonomous stopped: path timed out.")
                self.done = True
                self._send(0.0, 0.0, brake=True)
        self._step += 1
        return self._applied

    def _start_path(self, index):
        self.bridge.set_path(index)
        self.results.append(PathResult(index))

    def _send(self, v_left, v_right, brake=False):
        apply_at = self._step + config.STEPS_OF_COMMAND_DELAY
        self._pending.append((apply_at, v_left, v_right, brake))

    def _control(self, x, y, heading, omega_left, omega_right):
        # Like Autonomous::run, the next path starts in the same control
        # step the previous one finishes.
        while True:
            status, v_left, v_right, _, off_path = self.bridge.step(
                x, y, heading, omega_left, omega_right,
                config.MPC_BATTERY_VOLTS,
            )
            result = self.results[-1]
            result.worst_off_path = max(result.worst_off_path, off_path)
            result.final_off_path = off_path

            if status == mpc_bridge.FINISHED:
                result.finished = True
                self.log(result.summary())
                self._send(0.0, 0.0, brake=True)
                if result.index + 1 < self.bridge.path_count:
                    self._start_path(result.index + 1)
                    continue
                self.done = True
                self.log("Autonomous complete.")
                return

            if status == mpc_bridge.ERROR:
                self.log("Autonomous stopped: MPC bridge error.")
                self.done = True
                self._send(0.0, 0.0, brake=True)
                return

            if status == mpc_bridge.SOLVE_FAILED:
                result.failed_solves += 1
            self._send(v_left, v_right)
            return
