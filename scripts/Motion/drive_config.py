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

# What a motor does at exactly 0 V. "coast" lets it freewheel; "brake"
# shorts it so back-EMF fights the rotation (up to the current limit).
# robot-core's path follower sets the drive to coast (pathFollower.cpp),
# but with coast, a keyboard turn at speed (inside side at 0 V) barely
# turns: nothing slows the inside wheels. So the sim brakes for now.
# TODO(real robot): confirm how V5 motors behave at move_voltage(0) in
# each brake mode.
DRIVE_BRAKE_MODE = "brake"
if DRIVE_BRAKE_MODE not in ("coast", "brake"):
    raise RuntimeError("DRIVE_BRAKE_MODE must be 'coast' or 'brake'.")

# Keyboard driving only: brake when every key is released, so letting go
# stops the robot instead of coasting. Doesn't affect MPC driving.
KEYBOARD_BRAKE_ON_RELEASE = True

# Sim-only settings.
CONTROLLABLE_ATTR = "dreamsim:controllable"
MAX_VOLTAGE = 12.0

# Physics rate. Must fit a whole number of steps into robot-core's control
# period so each MPC voltage is held for the same length of time.
PHYSICS_STEPS_PER_SECOND = 100
STEPS_PER_CONTROL_PERIOD = round(CONTROL_PERIOD * PHYSICS_STEPS_PER_SECOND)
if not math.isclose(
    STEPS_PER_CONTROL_PERIOD, CONTROL_PERIOD * PHYSICS_STEPS_PER_SECOND
):
    raise RuntimeError(
        f"PHYSICS_STEPS_PER_SECOND ({PHYSICS_STEPS_PER_SECOND}) must fit a "
        f"whole number of steps into robot-core's {CONTROL_PERIOD} s "
        "control period."
    )

# Kept low: the motor's back-EMF already slows the robot.
LINEAR_DAMPING = 0.0
ANGULAR_DAMPING = 0.05

GRAVITY = 9.81  # m/s^2

# Robot body. Mass and inertia are applied on every load and override
# anything saved in the scene. Inertia is a solid box of this size, so the
# robot turns like a robot even while the placeholder cube is smaller.
# TODO(real robot): confirm by weighing and measuring the robot.
ROBOT_MASS = 10.0  # kg
ROBOT_LENGTH = 18.0 * INCH
ROBOT_WIDTH = 18.0 * INCH
ROBOT_HEIGHT = 12.0 * INCH

# Scales the solid-box inertia to account for where the weight sits: below
# 1 means mass concentrated toward the centre, so quicker to spin up and to
# stop spinning.
# TODO(real robot): approximation tuned by feel. Get the real value from
# CAD mass properties, a bifilar pendulum test, or by matching the real
# robot's spin-up time at +/-12 V.
ROBOT_INERTIA_SCALE = 0.3
ROBOT_INERTIA = tuple(
    ROBOT_INERTIA_SCALE * ROBOT_MASS * (a**2 + b**2) / 12.0
    for a, b in (
        (ROBOT_WIDTH, ROBOT_HEIGHT),
        (ROBOT_LENGTH, ROBOT_HEIGHT),
        (ROBOT_LENGTH, ROBOT_WIDTH),
    )
)  # kg*m^2 about x, y, z

# Wheels: evenly spaced along each side, the end wheels WHEEL_EDGE_INSET in
# from the front and back edges. Sides sit at +/-TRACK_WIDTH/2.
# TODO(real robot): confirm count and positions.
WHEELS_PER_SIDE = 3
WHEEL_EDGE_INSET = 1.0 * INCH
_end_wheel_x = ROBOT_LENGTH / 2.0 - WHEEL_EDGE_INSET
if WHEELS_PER_SIDE == 1:
    WHEEL_X_POSITIONS = [0.0]
else:
    _spacing = 2.0 * _end_wheel_x / (WHEELS_PER_SIDE - 1)
    WHEEL_X_POSITIONS = [
        -_end_wheel_x + _spacing * i for i in range(WHEELS_PER_SIDE)
    ]  # m, forward of centre

# All wheels are omni.
# TODO(real robot): measure rolling with the tile tilt test. It's the only
# published number (one team's 4" omni on foam tile, direction not stated).
WHEEL_ROLLING_FRICTION = 0.93

# TODO(real robot): approximation tuned by feel. No published value exists
# for omni wheels sideways. With all-omni wheels this one value trades
# sliding against turning: 0.125 (an estimate from roller rolling
# resistance) let the robot hold its slide angle after turns, while 0.3
# made the end wheels scrub so much that gentle arcs (12 V / 6 V) barely
# turned, and 0.2 lost too much forward speed when turning. 0.14 is a
# compromise. Calibrate against the real robot: time a +/-12 V spin in
# place, measure a 12 V / 6 V arc, and measure the drift after a turn at
# full forward, then adjust until the sim matches.
WHEEL_SIDEWAYS_FRICTION = 0.125

# Rolling resistance and drivetrain drag, as a fraction of each wheel's
# load. Without it the motors' braking fades as the robot slows, leaving a
# long slow glide; this brings it to a full stop.
# TODO(real robot): approximation. Calibrate with a coast-down test: drive
# at full speed, cut the voltage, and match the stopping distance. The
# real robot's brake mode matters too: robot-core's path follower sets
# the drive to coast.
WHEEL_ROLLING_RESISTANCE = 0.03

# Fraction of each wheel's sideways slip removed per physics step, below
# the friction limit. Higher grips harder; near 1 or above jitters.
GRIP_RESPONSE = 0.5

# Beyond this tilt from upright the wheels are treated as off the ground
# and apply no force, e.g. when the robot is tipped over or on its side.
MAX_WHEEL_TILT = math.radians(20.0)

# The robot's own collider gets this friction so it glides on the field
# and all grip comes from the wheel forces.
ROBOT_COLLIDER_FRICTION = 0.0

# TODO(real robot): wheel forces are summed about the body origin, which
# assumes the centre of mass is there, and they ignore force height. When
# the real robot model arrives, sum them about its actual centre of mass
# (see the TODO in drive_controller.py's on_physics_step).

# Seconds of sim time between status lines in the Console while the robot
# is moving. A line is also printed on the step where the voltages change.
# 0 turns them off.
STATUS_INTERVAL = 0.05

# Frames to wait for the timeline to report that it has stopped.
STOP_TIMEOUT_FRAMES = 60
