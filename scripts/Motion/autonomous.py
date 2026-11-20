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
import mpc_constraints
import tunable_constants


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


FINISHED = "finished"
TIMED_OUT = "timed out"
BRIDGE_ERROR = "bridge error"


class TrialResult:
    """How one trial (an autonomous run started by run_trial) ended.

    status is FINISHED, TIMED_OUT or BRIDGE_ERROR from the run, or
    "tipped", "cancelled" or "error" when it was stopped from outside;
    reason says why for the last three. params are the bridge values it ran
    with (robot-core's plus overrides), results its PathResults.
    """

    def __init__(self, status, path_file, params=None, results=(),
                 reason=""):
        self.status = status
        self.reason = reason
        self.path_file = path_file
        self.params = dict(params or {})
        self.results = list(results)

    @property
    def finished(self):
        return self.status == FINISHED

    @property
    def time(self):
        return sum(result.time for result in self.results)

    def summary(self):
        text = f"Trial {self.status}"
        if self.reason:
            text += f" ({self.reason})"
        if self.results:
            worst = max(result.worst_off_path for result in self.results)
            text += (
                f" in {self.time:.2f} s: worst {worst:.2f} in off the path, "
                f"{self.results[-1].final_off_path:.2f} in off at the end"
            )
        return text


class Autonomous:
    """Follows every path in a JerryIO file in order.

    Once done, outcome is FINISHED, TIMED_OUT or BRIDGE_ERROR.
    """

    def __init__(self, path_file, log=print, overrides=None, timeout=None):
        """overrides: MPC parameters instead of robot-core's (a dict by
        mpc_bridge.PARAM_NAMES); None uses
        tunable_constants.runtime_overrides(). timeout: seconds per path
        before giving up; None uses config.MPC_PATH_TIMEOUT.
        """
        self.log = log
        self.path_file = path_file
        self.timeout = (
            config.MPC_PATH_TIMEOUT if timeout is None else timeout
        )
        if overrides is None:
            overrides = tunable_constants.runtime_overrides()
        self.bridge = mpc_bridge.MpcBridge(path_file, overrides)
        if overrides:
            defaults = mpc_bridge.default_params()
            changes = ", ".join(
                f"{name} {defaults[name]:g} -> {value:g}"
                for name, value in overrides.items()
            )
            self.log(f"MPC tuning (sim only): {changes}")
        self.constraint_log = mpc_constraints.open_log(self.bridge)
        self.params = self.bridge.params
        self.results = []
        self.done = False
        self.outcome = None
        self._step = 0
        # (physics step to apply at, v_left, v_right, brake)
        self._pending = []
        self._applied = (0.0, 0.0, True)
        self._start_path(0)

    def start_pose(self):
        """Where the robot must start: (x in, y in, heading rad)."""
        return self.bridge.start_pose()

    def close(self):
        if self.constraint_log is not None:
            self.constraint_log.close()
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
            limit = self.timeout * config.PHYSICS_STEPS_PER_SECOND
            if self.results[-1].steps > limit:
                self._log_result(self.results[-1])
                self.log("Autonomous stopped: path timed out.")
                self.done = True
                self.outcome = TIMED_OUT
                self._send(0.0, 0.0, brake=True)
        self._step += 1
        return self._applied

    def _start_path(self, index):
        self.bridge.set_path(index)
        self.results.append(PathResult(index))
        if self.constraint_log is not None:
            self.constraint_log.start_path()

    def _log_result(self, result):
        self.log(result.summary())
        if self.constraint_log is not None:
            self.log(self.constraint_log.tally.summary())

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
                self._log_result(result)
                self._send(0.0, 0.0, brake=True)
                if result.index + 1 < self.bridge.path_count:
                    self._start_path(result.index + 1)
                    continue
                self.done = True
                self.outcome = FINISHED
                self.log("Autonomous complete.")
                return

            if status == mpc_bridge.ERROR:
                self.log("Autonomous stopped: MPC bridge error.")
                self.done = True
                self.outcome = BRIDGE_ERROR
                self._send(0.0, 0.0, brake=True)
                return

            if status == mpc_bridge.SOLVE_FAILED:
                result.failed_solves += 1
            elif self.constraint_log is not None:
                self.constraint_log.record(
                    self._step / config.PHYSICS_STEPS_PER_SECOND,
                    result.index, x, y, heading, off_path,
                )
            self._send(v_left, v_right)
            return
