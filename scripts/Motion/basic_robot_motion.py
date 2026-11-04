import asyncio
import math
import numpy as np

import isaacsim.core.experimental.utils.app as app_utils

from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.core.simulation_manager import (
    SimulationManager,
    SimulationEvent,
)


# ---------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------

ROBOT_PATH = "/World/body"

# Starting test values. Tune these for your robot's mass.
FORWARD_FORCE_N = 10.0
TURN_TORQUE_NM = 1.2

# Number of physics steps between console messages.
PRINT_EVERY_N_STEPS = 15


# ---------------------------------------------------------
# GLOBAL STATE
# ---------------------------------------------------------

robot = None

forward_command = 0.0   # -1.0 backward, 0.0 stop, +1.0 forward
turn_command = 0.0      # -1.0 right, 0.0 straight, +1.0 left

simulation_time = 0.0
step_count = 0

pre_step_callback_id = None
post_step_callback_id = None


# ---------------------------------------------------------
# UTILITY FUNCTIONS
# ---------------------------------------------------------

def to_numpy(value):
    """Convert Warp/Isaac arrays to NumPy arrays."""
    if hasattr(value, "numpy"):
        return value.numpy()

    return np.asarray(value)


def quaternion_to_yaw(quaternion):
    """
    Convert Isaac Sim quaternion [w, x, y, z] to yaw in radians.

    Assumes Z is the vertical axis.
    """
    w, x, y, z = quaternion

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(sin_yaw, cos_yaw)


def set_motion_command(forward, turn):
    """
    Update the current movement command.

    forward:
        -1.0 = backward
         0.0 = no translation
        +1.0 = forward

    turn:
        -1.0 = turn right
         0.0 = no rotation
        +1.0 = turn left
    """
    global forward_command, turn_command

    forward_command = float(np.clip(forward, -1.0, 1.0))
    turn_command = float(np.clip(turn, -1.0, 1.0))


# ---------------------------------------------------------
# TEST COMMAND SEQUENCE
# ---------------------------------------------------------

def update_test_commands(current_time):
    if current_time < 2.0:
        set_motion_command(forward=1.0, turn=0.0)
    else:
        set_motion_command(forward=1.5, turn=1.0)


# ---------------------------------------------------------
# PHYSICS CALLBACKS
# ---------------------------------------------------------

def apply_robot_commands(dt, context):
    """
    Called before every physics step.

    Applies a force and torque to the robot's rigid body.
    """
    global simulation_time

    simulation_time += dt
    update_test_commands(simulation_time)

    force = np.array(
        [[FORWARD_FORCE_N * forward_command, 0.0, 0.0]],
        dtype=np.float32,
    )

    torque = np.array(
        [[0.0, 0.0, TURN_TORQUE_NM * turn_command]],
        dtype=np.float32,
    )

    robot.apply_forces_and_torques_at_pos(
        forces=force,
        torques=torque,
        local_frame=False,
    )


def read_robot_state(dt, context):
    """
    Called after every physics step.

    Reads position, orientation, linear velocity, and angular velocity.
    """
    global step_count

    step_count += 1

    positions, orientations = robot.get_world_poses()
    linear_velocities, angular_velocities = robot.get_velocities()

    position = to_numpy(positions)[0]
    orientation = to_numpy(orientations)[0]
    linear_velocity = to_numpy(linear_velocities)[0]
    angular_velocity = to_numpy(angular_velocities)[0]

    yaw_radians = quaternion_to_yaw(orientation)
    yaw_degrees = math.degrees(yaw_radians)

    if step_count % PRINT_EVERY_N_STEPS == 0:
        print(
            "\n"
            f"Time: {simulation_time:.2f} s\n"
            f"Command: forward={forward_command:.2f}, "
            f"turn={turn_command:.2f}\n"
            f"Position: x={position[0]:.3f}, "
            f"y={position[1]:.3f}, "
            f"z={position[2]:.3f}\n"
            f"Orientation: yaw={yaw_degrees:.2f} degrees\n"
            f"Linear velocity: "
            f"vx={linear_velocity[0]:.3f}, "
            f"vy={linear_velocity[1]:.3f}, "
            f"vz={linear_velocity[2]:.3f}\n"
            f"Angular velocity: "
            f"wx={angular_velocity[0]:.3f}, "
            f"wy={angular_velocity[1]:.3f}, "
            f"wz={angular_velocity[2]:.3f}"
        )


# ---------------------------------------------------------
# CALLBACK CLEANUP
# ---------------------------------------------------------

def remove_callbacks():
    """
    Remove old callbacks before installing new ones.

    This prevents duplicate forces if the script is run twice.
    """
    global pre_step_callback_id, post_step_callback_id

    if pre_step_callback_id is not None:
        SimulationManager.deregister_callback(pre_step_callback_id)
        pre_step_callback_id = None

    if post_step_callback_id is not None:
        SimulationManager.deregister_callback(post_step_callback_id)
        post_step_callback_id = None


# ---------------------------------------------------------
# SETUP
# ---------------------------------------------------------

async def setup_robot_controller():
    global robot
    global simulation_time
    global step_count
    global pre_step_callback_id
    global post_step_callback_id

    remove_callbacks()

    simulation_time = 0.0
    step_count = 0

    # Wrap the existing rigid-body robot prim.
    robot = RigidPrim(ROBOT_PATH)

    # Tensor-backed physics APIs become valid after simulation starts.
    app_utils.play()
    await app_utils.update_app_async()

    if not robot.is_physics_tensor_entity_valid():
        print(
            f"ERROR: Could not initialize rigid body at {ROBOT_PATH}.\n"
            "Check that the path is correct and that the prim has "
            "Rigid Body and Collider physics applied."
        )
        return

    pre_step_callback_id = SimulationManager.register_callback(
        apply_robot_commands,
        event=SimulationEvent.PHYSICS_PRE_STEP,
    )

    post_step_callback_id = SimulationManager.register_callback(
        read_robot_state,
        event=SimulationEvent.PHYSICS_POST_STEP,
    )

    print("Robot motion controller started.")
    print(f"Controlling rigid body: {ROBOT_PATH}")
    print("Test sequence:")
    print("  0-2 s: forward")
    print("  2-4 s: rotate")
    print("  4-6 s: forward + rotate")
    print("  6+ s: no applied force or torque")


asyncio.ensure_future(setup_robot_controller())
