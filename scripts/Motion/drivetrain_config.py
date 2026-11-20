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
SHARED_CONFIG_DIR = ROBOT_CORE / "shared" / "include" / "config"
CONSTANTS_HEADER = SHARED_CONFIG_DIR / "constants.h"
LINK_CONSTANTS_HEADER = SHARED_CONFIG_DIR / "linkConstants.hpp"
BRAIN_SUBSYSTEMS_HEADER = (
    ROBOT_CORE / "v5-brain" / "include" / "config" / "subsystems.hpp"
)

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
_robot = read_constants(CONSTANTS_HEADER)

WHEEL_RADIUS = _robot["DRIVE_WHEEL_DIAMETER_INCHES"] / 2.0 * INCH
DRIVE_GEAR_RATIO = _robot["DRIVE_GEAR_RATIO"]  # wheel speed / motor speed
TRACK_WIDTH = _robot["TRACK_WIDTH"] * INCH

CONTROL_PERIOD = _robot["MPC_SAMPLE_PERIOD"]  # s
# Time from the MPC solving to the brain applying its command. The MPC
# predicts the robot forward over it, so the sim applies each command this
# much later to match.
COMMAND_DELAY = _robot["MPC_COMMAND_DELAY"]  # s
# The MPC's motor model (MPC_MOTOR_A/B) isn't read here: it's compiled into
# the bridge. mpc_bridge.motor_model() reports the values it uses.

_left_ports = read_define_list(
    LINK_CONSTANTS_HEADER, "LEFT_DRIVE_WHEEL_PORTS"
)
_right_ports = read_define_list(
    LINK_CONSTANTS_HEADER, "RIGHT_DRIVE_WHEEL_PORTS"
)
if len(_left_ports) != len(_right_ports):
    raise RuntimeError("Left and right drive sides differ in motor count.")
MOTORS_PER_SIDE = len(_left_ports)

CARTRIDGE_RPM = read_cartridge_rpm(BRAIN_SUBSYSTEMS_HEADER, "leftMotors")

# V5 11W motor model.
# TODO: confirm. These are community estimates fitted to VEX's published
# curve (100 rpm cartridge, 12.8 V), not official VEX numbers. Check them
# against the MPC's motor model (printed when the controller loads), which
# comes from the real robot.
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
STEPS_OF_COMMAND_DELAY = round(COMMAND_DELAY * PHYSICS_STEPS_PER_SECOND)
if not math.isclose(
    STEPS_OF_COMMAND_DELAY, COMMAND_DELAY * PHYSICS_STEPS_PER_SECOND
):
    raise RuntimeError(
        f"PHYSICS_STEPS_PER_SECOND ({PHYSICS_STEPS_PER_SECOND}) must fit a "
        f"whole number of steps into robot-core's {COMMAND_DELAY} s "
        "command delay."
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
WHEELS_PER_SIDE = 4
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
# Tile tilt test, wheels locked, on the test robot (not the final one):
# slid at a 370 mm rise on a 595 mm tile, 370 / sqrt(595^2 - 370^2).
# TODO(real robot): re-measure on the final robot.
WHEEL_ROLLING_FRICTION = 0.79

# TODO(real robot): approximation tuned by feel. No published value exists
# for omni wheels sideways. With all-omni wheels this one value trades
# sliding against turning: 0.125 (an estimate from roller rolling
# resistance) let the robot hold its slide angle after turns, while 0.3
# made the end wheels scrub so much that gentle arcs (12 V / 6 V) barely
# turned, and 0.2 lost too much forward speed when turning. 0.14 is a
# compromise. Calibrate against the real robot: time a +/-12 V spin in
# place, measure a 12 V / 6 V arc, and measure the drift after a turn at
# full forward, then adjust until the sim matches.
WHEEL_SIDEWAYS_FRICTION = 0.3

# Rolling resistance and drivetrain drag, as a fraction of each wheel's
# load. Without it the motors' braking fades as the robot slows, leaving a
# long slow glide; this brings it to a full stop.
# Fitted to the test robot (not the final one; ~10 kg, 600 rpm at the
# wheel) with sideways friction 0.3: 0.18 matches its +/-12 V spin in place
# (600 deg/s; sim 601 deg/s). Its brake stop from full speed doesn't match
# (10-12 in; sim 18.7 in): matching that needs about 0.30, where the robot
# can't spin at all. Check the real top speed before blaming this value;
# the sim's motor braking may be too weak instead. Above about 0.20 the
# spin collapses. Mostly the unpowered motors' gearboxes being
# back-driven. The tile tilt test with wheels free gave 0.36 (200 mm rise
# on a 595 mm tile), but that's breakaway: the drag once rolling is lower.
# TODO(real robot): re-measure on the final robot.
WHEEL_ROLLING_RESISTANCE = 0.18

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

# Autonomous (P key in drive_controller.py, and test_mpc_bridge.py): the
# JerryIO file whose paths robot-core's MPC follows, in order.
MPC_PATH_FILE = (
    REPO_ROOT / "paths" / "twin_test_path.txt"
)
# Battery voltage reported to the MPC. The motor model has no sag, so this
# matches the 12 V it drives with.
MPC_BATTERY_VOLTS = 12.0
# Give up on a path that hasn't finished after this long (s). robot-core has
# no limit, but a robot stuck in the sim would otherwise run forever.
MPC_PATH_TIMEOUT = 15.0

# MPC tuning, sim only: parameters to use instead of robot-core's
# constants.h, without rebuilding. Empty runs robot-core's values. Keys are
# the constants' names in lower case without MPC_ (q_u is MPC_Q_U):
#   motor_a, motor_b     the MPC's motor model (1/s, rad/s^2 per V)
#   p_x, p_y, p_theta    weight on position and heading error; raise p_y
#                        if it drifts off straight lines, p_theta if it
#                        cuts corners
#   p_final_mult         extra weight on the error at the horizon's end
#   q_u                  penalty on voltage changes; raise it if the
#                        voltages chatter, lower it if it turns wide or
#                        leaves small errors uncorrected
#   q_zero_mult          extra penalty on the first change (from the last
#                        voltage sent)
#   q_final_mult         extra penalty on the last change
#   delta_u_max          largest voltage rise per 20 ms step (V)
#   delta_u_min          largest voltage drop per step (V, negative)
#   v_max, x_field_max, y_field_max, omega_wheel_max   limits
# robot-core's docs/MPC_DOCS.md has its tuning notes. The horizons and the
# reference's acceleration limit are compiled in and can't change here.
# Copy values that work into constants.h for the real robot.
# Example: MPC_TUNING = {"q_u": 0.5, "delta_u_max": 4.0, "delta_u_min": -4.0}
MPC_TUNING = {}

# Debug: log which MPC constraints each solve presses against, one CSV line
# per control step: the first move's limits per side (V+/V- voltage,
# slew+/slew- slew rate, speed wheel speed, field) and how many rows of
# each limit are active over the horizon. Each path's summary also says how
# often its first move was held at each limit. Overwritten each run;
# gitignored.
MPC_CONSTRAINT_LOG = True
MPC_CONSTRAINT_FILE = REPO_ROOT / "mpc_constraints_latest.csv"
# Also write every constraint row of every solve (group, bound, stage, side,
# G*U, b, slack, dual): about 300 lines per step, for a close look.
MPC_CONSTRAINT_ROWS = False
MPC_CONSTRAINT_ROWS_FILE = REPO_ROOT / "mpc_constraint_rows_latest.csv"

# Draw MPC_PATH_FILE's paths on the field when autonomous starts: ribbons
# coloured by path speed (red slow, green fast), drawn in the stage's
# session layer so saving the scene doesn't keep them.
SHOW_MPC_PATHS = True
PATH_DISPLAY_HEIGHT = 0.02  # m above the floor
PATH_DISPLAY_WIDTH = 0.02  # m
# What the colour shows: "reference" is the speed robot-core's MPC reference
# targets (curvature and acceleration limited, from rest, as if tracked
# perfectly); "file" is the speed column in the JerryIO file.
PATH_DISPLAY_SPEED = "reference"

# Debug: a dot on the path point the follower has matched the robot to,
# updated every physics step during autonomous. The match uses the MPC's
# delay-compensated pose, so the dot can sit slightly ahead of the robot.
SHOW_CLOSEST_POINT = True
CLOSEST_POINT_COLOR = (0.1, 0.5, 1.0, 1.0)  # RGBA, blue
CLOSEST_POINT_SIZE = 15  # pixels

# Robot position and path output (status lines, autonomous placement and
# per-path results). Overwritten each time the controller loads;
# gitignored.
OUTPUT_FILE = REPO_ROOT / "output_latest.txt"

# Frames to wait for the timeline to report that it has stopped.
STOP_TIMEOUT_FRAMES = 60
