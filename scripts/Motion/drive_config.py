"""Settings for drive_controller.py.

Robot values (wheel size, track width, gearing, motor count, cartridge,
identified motor model) are read from robot-core's headers each time the
controller loads, so the sim follows the robot when robot-core changes.
Everything else here is sim-only.

robot-core uses inches; values here are converted to SI units.
"""

import ast
import math
import operator
import re
from pathlib import Path


INCH = 0.0254

REPO_ROOT = Path(__file__).resolve().parents[2]
ROBOT_CORE = (
    REPO_ROOT / "native" / "robotcore_mpc" / "third_party" / "robotcore"
)
MPC_CONFIG_HEADER = ROBOT_CORE / "shared" / "include" / "mpc" / "mpcConfig.hpp"
BRAIN_CONFIG_DIR = ROBOT_CORE / "v5-brain" / "include" / "config"
BRAIN_CONSTANTS_HEADER = BRAIN_CONFIG_DIR / "constants.h"
BRAIN_SUBSYSTEMS_HEADER = BRAIN_CONFIG_DIR / "subsystems.hpp"

CARTRIDGE_RPMS = {"red": 100.0, "green": 200.0, "blue": 600.0}

_OPERATORS = {
    ast.Add: operator.add,
    ast.Sub: operator.sub,
    ast.Mult: operator.mul,
    ast.Div: operator.truediv,
    ast.USub: operator.neg,
}


def _read_header(path):
    if not path.is_file():
        raise RuntimeError(
            f"{path} not found. Pull robot-core with: "
            "git submodule update --init --recursive"
        )
    return path.read_text(encoding="utf-8")


def _evaluate(expression):
    """Evaluate simple C++ arithmetic such as 450.0/600.0."""
    def visit(node):
        if isinstance(node, ast.Constant) and isinstance(
            node.value, (int, float)
        ):
            return node.value
        if isinstance(node, ast.BinOp) and type(node.op) in _OPERATORS:
            left, right = visit(node.left), visit(node.right)
            return _OPERATORS[type(node.op)](left, right)
        if isinstance(node, ast.UnaryOp) and type(node.op) in _OPERATORS:
            return _OPERATORS[type(node.op)](visit(node.operand))
        raise ValueError(expression)

    return visit(ast.parse(expression, mode="eval").body)


def read_constants(path):
    """Return the numeric constexpr values in a header, by name."""
    constants = {}
    pattern = r"constexpr\s+[\w:]+\s+(\w+)\s*=\s*([^;]+);"
    for name, expression in re.findall(pattern, _read_header(path)):
        try:
            constants[name] = float(_evaluate(expression))
        except (SyntaxError, ValueError):
            pass
    return constants


def read_define_list(path, name):
    """Return the numbers in a '#define NAME {a, b, c}' line."""
    pattern = rf"#define\s+{name}\s+\{{([^}}]*)\}}"
    match = re.search(pattern, _read_header(path))
    if match is None:
        raise RuntimeError(f"{name} not found in {path}")
    return [int(value) for value in match.group(1).split(",")]


def read_cartridge_rpm(path, motor_group):
    """Return the cartridge rpm of a pros::MotorGroup in a header."""
    match = re.search(
        rf"{motor_group}\([^)]*MotorGears::(\w+)", _read_header(path)
    )
    if match is None or match.group(1) not in CARTRIDGE_RPMS:
        raise RuntimeError(f"{motor_group} cartridge not found in {path}")
    return CARTRIDGE_RPMS[match.group(1)]


# Robot values from robot-core.
_mpc = read_constants(MPC_CONFIG_HEADER)
_brain = read_constants(BRAIN_CONSTANTS_HEADER)

WHEEL_RADIUS = _mpc["WHEEL_RADIUS"] * INCH
CONTROL_PERIOD = _mpc["H"]  # s
MOTOR_A = _mpc["MOTOR_A"]  # 1/s
MOTOR_B = _mpc["MOTOR_B"]  # rad/s^2 per V, wheel side

DRIVE_GEAR_RATIO = _brain["DRIVE_GEAR_RATIO"]  # wheel speed / motor speed
TRACK_WIDTH = _brain["TRACK_WIDTH"] * INCH

_left_ports = read_define_list(
    BRAIN_CONSTANTS_HEADER, "LEFT_DRIVE_WHEEL_PORTS"
)
_right_ports = read_define_list(
    BRAIN_CONSTANTS_HEADER, "RIGHT_DRIVE_WHEEL_PORTS"
)
if len(_left_ports) != len(_right_ports):
    raise RuntimeError("Left and right drive sides differ in motor count.")
MOTORS_PER_SIDE = len(_left_ports)

CARTRIDGE_RPM = read_cartridge_rpm(BRAIN_SUBSYSTEMS_HEADER, "leftMotors")

# V5 11W motor model.
# TODO: confirm. These are community estimates fitted to VEX's published
# curve (100 rpm cartridge, 12.8 V), not official VEX numbers. Check them
# against MOTOR_A/MOTOR_B, which come from the real robot.
NOMINAL_VOLTAGE = 12.8
STALL_TORQUE_100RPM = 3.99  # N*m, without the current limit
STALL_CURRENT = 4.69  # A
FREE_SPEED_100RPM = 121.0  # rpm
CURRENT_LIMIT = 2.5  # A, firmware limit

CARTRIDGE_RATIO = CARTRIDGE_RPM / 100.0
MOTOR_KT = STALL_TORQUE_100RPM / STALL_CURRENT / CARTRIDGE_RATIO  # N*m/A
MOTOR_RESISTANCE = NOMINAL_VOLTAGE / STALL_CURRENT  # ohm
MOTOR_KV = (
    FREE_SPEED_100RPM * CARTRIDGE_RATIO * 2.0 * math.pi / 60.0
    / NOMINAL_VOLTAGE
)  # rad/s per V

# Sim-only settings.
CONTROLLABLE_ATTR = "dreamsim:controllable"
MAX_VOLTAGE = 12.0

# Kept low: the motor's back-EMF already slows the robot.
LINEAR_DAMPING = 0.0
ANGULAR_DAMPING = 0.05
DEFAULT_MASS = 5.0  # kg, used when the prim has no mass authored

# TODO(real robot): when the robot model replaces the cube, set its real
# mass and switch drive_controller.py's on_physics_step to apply each
# side's force at its wheel contact point (see the TODO there).

# Frames to wait for the timeline to report that it has stopped.
STOP_TIMEOUT_FRAMES = 60
