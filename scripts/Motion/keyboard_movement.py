import asyncio
import math
import numpy as np
import carb.input
import omni.appwindow

import isaacsim.core.experimental.utils.app as app_utils

from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.core.simulation_manager import (
    SimulationManager,
    SimulationEvent,
)


ROBOT_PATH = "/World/body"
FORWARD_FORCE_N = 10.0
TURN_TORQUE_NM = 1.2
PRINT_EVERY_N_STEPS = 15


# ---------------------------------------------------------
# GLOBAL STATE
# ---------------------------------------------------------

robot = None

forward_command = 0.0
turn_command = 0.0

simulation_time = 0.0
step_count = 0

pre_step_callback_id = None
post_step_callback_id = None

input_interface = None
keyboard = None
keyboard_subscription_id = None

# Tracks keys currently being held.
pressed_keys = set()


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
    Convert Isaac Sim quaternion [w, x, y, z]
    into yaw around the Z-axis.
    """
    w, x, y, z = quaternion

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(sin_yaw, cos_yaw)


def update_commands_from_keys():
    """
    Convert currently held WASD keys into movement commands.

    W = forward
    S = backward
    A = turn left
    D = turn right
    """
    global forward_command, turn_command

    forward_command = 0.0
    turn_command = 0.0

    if carb.input.KeyboardInput.UP in pressed_keys:
        forward_command += 1.0

    if carb.input.KeyboardInput.DOWN in pressed_keys:
        forward_command -= 1.0

    if carb.input.KeyboardInput.LEFT in pressed_keys:
        turn_command += 1.0

    if carb.input.KeyboardInput.RIGHT in pressed_keys:
        turn_command -= 1.0

    forward_command = float(
        np.clip(forward_command, -1.0, 1.0)
    )

    turn_command = float(
        np.clip(turn_command, -1.0, 1.0)
    )


# ---------------------------------------------------------
# KEYBOARD INPUT
# ---------------------------------------------------------

def on_keyboard_event(event, *args, **kwargs):
    """
    Record keyboard press and release events.
    """
    movement_keys = {
        carb.input.KeyboardInput.UP,
        carb.input.KeyboardInput.DOWN,
        carb.input.KeyboardInput.LEFT,
        carb.input.KeyboardInput.RIGHT,
    }

    if event.input not in movement_keys:
        return True

    if event.type in (
        carb.input.KeyboardEventType.KEY_PRESS,
        carb.input.KeyboardEventType.KEY_REPEAT,
    ):
        pressed_keys.add(event.input)

    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        pressed_keys.discard(event.input)

    update_commands_from_keys()

    return True


# ---------------------------------------------------------
# PHYSICS CALLBACKS
# ---------------------------------------------------------

def apply_robot_commands(dt, context):
    """
    Called before every physics step.

    Applies horizontal force in the direction the robot
    is facing and yaw torque around the world Z-axis.
    """
    global simulation_time

    simulation_time += dt

    # Read the current robot orientation.
    positions, orientations = robot.get_world_poses()
    orientation = to_numpy(orientations)[0]

    # Only use yaw, ignoring pitch and roll.
    yaw = quaternion_to_yaw(orientation)

    # Horizontal forward vector based on yaw.
    forward_x = math.cos(yaw)
    forward_y = math.sin(yaw)

    force = np.array(
        [[
            FORWARD_FORCE_N * forward_command * forward_x,
            FORWARD_FORCE_N * forward_command * forward_y,
            0.0,
        ]],
        dtype=np.float32,
    )

    # Rotation around the vertical world Z-axis.
    torque = np.array(
        [[
            0.0,
            0.0,
            TURN_TORQUE_NM * turn_command,
        ]],
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

    Reads and prints position, orientation,
    linear velocity, and angular velocity.
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
            f"Command: forward={forward_command:.1f}, "
            f"turn={turn_command:.1f}\n"
            f"Position: "
            f"x={position[0]:.3f}, "
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
# CLEANUP
# ---------------------------------------------------------

def remove_callbacks():
    """
    Remove previously registered physics and keyboard callbacks.
    """
    global pre_step_callback_id
    global post_step_callback_id
    global keyboard_subscription_id
    global input_interface
    global keyboard

    if pre_step_callback_id is not None:
        SimulationManager.deregister_callback(
            pre_step_callback_id
        )
        pre_step_callback_id = None

    if post_step_callback_id is not None:
        SimulationManager.deregister_callback(
            post_step_callback_id
        )
        post_step_callback_id = None

    if (
        keyboard_subscription_id is not None
        and input_interface is not None
        and keyboard is not None
    ):
        input_interface.unsubscribe_to_keyboard_events(
            keyboard,
            keyboard_subscription_id,
        )
        keyboard_subscription_id = None


# ---------------------------------------------------------
# SETUP
# ---------------------------------------------------------

async def setup_robot_controller():
    global robot
    global simulation_time
    global step_count
    global pre_step_callback_id
    global post_step_callback_id
    global input_interface
    global keyboard
    global keyboard_subscription_id

    remove_callbacks()

    simulation_time = 0.0
    step_count = 0
    pressed_keys.clear()

    robot = RigidPrim(ROBOT_PATH)

    # Start simulation so physics tensor APIs become available.
    app_utils.play()
    await app_utils.update_app_async()

    if not robot.is_physics_tensor_entity_valid():
        print(
            f"ERROR: Could not initialize rigid body at "
            f"{ROBOT_PATH}.\n"
            "Confirm that 'body' has Rigid Body physics and "
            "that its child Cube has a Collider."
        )
        return

    # Register physics callbacks.
    pre_step_callback_id = SimulationManager.register_callback(
        apply_robot_commands,
        event=SimulationEvent.PHYSICS_PRE_STEP,
    )

    post_step_callback_id = SimulationManager.register_callback(
        read_robot_state,
        event=SimulationEvent.PHYSICS_POST_STEP,
    )

    # Register keyboard callback.
    app_window = omni.appwindow.get_default_app_window()
    keyboard = app_window.get_keyboard()

    input_interface = carb.input.acquire_input_interface()

    keyboard_subscription_id = (
        input_interface.subscribe_to_keyboard_events(
            keyboard,
            on_keyboard_event,
        )
    )

    print("Robot WASD controller started.")
    print(f"Controlling: {ROBOT_PATH}")
    print("Controls:")
    print("  W: move forward")
    print("  S: move backward")
    print("  A: turn left")
    print("  D: turn right")
    print("  W + A/D: move in an arc")
    print("Click inside the viewport before using WASD.")


asyncio.ensure_future(setup_robot_controller())
