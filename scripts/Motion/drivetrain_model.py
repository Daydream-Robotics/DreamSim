"""Drivetrain physics for drive_controller.py, with no Isaac Sim imports.

Kept separate so test_mpc_bridge.py can run the exact same model with plain
Python. Settings come from drivetrain_config.py.
"""

import math

import drivetrain_config as config


def clamp(value, limit):
    """Limit value to the range [-limit, limit]."""
    return max(-limit, min(limit, value))


def arcade_to_tank(drive, steer):
    """Mix drive and steer in [-1, 1] into (left, right) side voltages."""
    left = drive - steer
    right = drive + steer
    # Scale both sides down together so turning still works at full drive.
    scale = max(1.0, abs(left), abs(right))
    max_voltage = config.MAX_VOLTAGE
    return max_voltage * left / scale, max_voltage * right / scale


def motor_torque(voltage, motor_speed, brake=False):
    """Output torque of one motor from its voltage and speed (rad/s).

    At exactly 0 V the motor coasts (no torque) unless brake is set or
    config.DRIVE_BRAKE_MODE is "brake".
    """
    if voltage == 0.0 and not brake and config.DRIVE_BRAKE_MODE == "coast":
        return 0.0
    back_emf = motor_speed / config.MOTOR_KV
    current = (voltage - back_emf) / config.MOTOR_RESISTANCE
    current = clamp(current, config.CURRENT_LIMIT)
    return config.MOTOR_KT * current


def side_force(voltage, ground_speed, brake=False):
    """Force one drive side puts on the ground, from its ground speed."""
    wheel_speed = ground_speed / config.WHEEL_RADIUS
    motor_speed = wheel_speed / config.DRIVE_GEAR_RATIO
    torque = motor_torque(voltage, motor_speed, brake)
    wheel_torque = config.MOTORS_PER_SIDE * torque / config.DRIVE_GEAR_RATIO
    return wheel_torque / config.WHEEL_RADIUS


def drive_wrench(v_left, v_right, forward_speed, sideways_speed, yaw_rate,
                 dt, brake=False):
    """Net body-frame force (x, y) and yaw torque from all the wheels.

    brake makes sides at 0 V brake even when the drive is set to coast.

    Speeds are in the robot's frame: x forward, y left, z up.
    """
    wheel_count = 2 * config.WHEELS_PER_SIDE
    normal_force = config.ROBOT_MASS * config.GRAVITY / wheel_count
    rolling_limit = config.WHEEL_ROLLING_FRICTION * normal_force
    sideways_limit = config.WHEEL_SIDEWAYS_FRICTION * normal_force
    resistance_limit = config.WHEEL_ROLLING_RESISTANCE * normal_force
    # Force per m/s of slip that removes GRIP_RESPONSE of a wheel's slip in
    # one step. Used for sideways grip and rolling resistance.
    grip_gain = config.GRIP_RESPONSE * config.ROBOT_MASS / wheel_count / dt

    force_x = 0.0
    force_y = 0.0
    torque_z = 0.0
    half_track = config.TRACK_WIDTH / 2.0
    for side_y, voltage in ((half_track, v_left), (-half_track, v_right)):
        side_speed = forward_speed - yaw_rate * side_y
        side = side_force(voltage, side_speed, brake)
        per_wheel = side / config.WHEELS_PER_SIDE
        # Beyond the friction limit the wheel spins instead of pushing.
        drive = clamp(per_wheel, rolling_limit)
        # Resistance opposes rolling and holds the robot once stopped.
        resistance = clamp(-grip_gain * side_speed, resistance_limit)
        push = drive + resistance

        for wheel_x in config.WHEEL_X_POSITIONS:
            slip = sideways_speed + yaw_rate * wheel_x
            grip = clamp(-grip_gain * slip, sideways_limit)

            force_x += push
            force_y += grip
            torque_z += wheel_x * grip - side_y * push

    return force_x, force_y, torque_z


def wheel_speeds(forward_speed, yaw_rate):
    """Left and right wheel speeds (rad/s) from body motion (m/s, rad/s).

    yaw_rate is anticlockwise-positive, as in Isaac Sim.
    """
    half_track = config.TRACK_WIDTH / 2.0
    left = (forward_speed - yaw_rate * half_track) / config.WHEEL_RADIUS
    right = (forward_speed + yaw_rate * half_track) / config.WHEEL_RADIUS
    return left, right


def wrap_angle(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def heading_from_yaw(yaw):
    """robot-core heading from an Isaac Sim yaw.

    Isaac Sim yaw is 0 along +x, anticlockwise. robot-core (and JerryIO)
    heading is 0 along +y, pi/2 along +x, clockwise.
    """
    return wrap_angle(math.pi / 2.0 - yaw)


def yaw_from_heading(heading):
    """Isaac Sim yaw from a robot-core heading (see heading_from_yaw)."""
    return wrap_angle(math.pi / 2.0 - heading)
