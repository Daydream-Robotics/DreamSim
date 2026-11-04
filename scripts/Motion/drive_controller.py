"""Tank-drive WASD controls for robots in Isaac Sim.

Paste into the Script Editor and run. The script stops the timeline,
installs the controller, then starts the simulation again.

Every prim on the stage whose config.CONTROLLABLE_ATTR is true is driven by
the controls. To tag a prim, add a uniform Bool attribute with that name in
the Property panel, tick it, then save the stage.

Controls: W/S drive, A/D turn, Space clears all input. The keys are mixed
into left/right side voltages, the same inputs robot-core's MPC produces.
Each physics step, every side's voltage and wheel speed go through a V5
motor model to get that side's wheel force.

Settings live in drive_config.py next to this file, which reads the
robot's values from robot-core. Edits to either take effect on re-run.
"""

import asyncio
import importlib
import math
import sys
from pathlib import Path

import carb
import numpy as np
import omni.appwindow
import omni.kit.app
import omni.timeline
import omni.usd
from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
from pxr import PhysxSchema, UsdGeom, UsdPhysics


def find_motion_dir():
    """Find scripts/Motion from this file or from the open scene's path.

    Code pasted into the Script Editor has no __file__, so fall back to
    searching upward from the scene, which lives in the same repo.
    """
    candidates = []
    if "__file__" in globals():
        candidates.append(Path(__file__).resolve().parent)
    stage = omni.usd.get_context().get_stage()
    scene_path = stage.GetRootLayer().realPath if stage else ""
    if scene_path:
        for parent in Path(scene_path).resolve().parents:
            candidates.append(parent / "scripts" / "Motion")

    for candidate in candidates:
        if (candidate / "drive_config.py").is_file():
            return candidate
    raise RuntimeError(
        "Couldn't find scripts/Motion/drive_config.py. Open a scene from "
        "the DreamSim repo with File > Open, then run this again."
    )


_motion_dir = str(find_motion_dir())
if _motion_dir not in sys.path:
    sys.path.insert(0, _motion_dir)

import drive_config as config  # noqa: E402

# Reload so edits to drive_config.py or robot-core apply without a restart.
config = importlib.reload(config)


def find_robot_prims(stage):
    """Return every prim on the stage tagged as controllable."""
    robot_prims = []
    for prim in stage.Traverse():
        attr = prim.GetAttribute(config.CONTROLLABLE_ATTR)
        if attr.IsValid() and attr.Get():
            robot_prims.append(prim)
    return robot_prims


def setup_rigid_body(prim):
    """Make the prim a dynamic rigid body."""
    rigid_body = UsdPhysics.RigidBodyAPI.Apply(prim)
    rigid_body.CreateRigidBodyEnabledAttr().Set(True)
    rigid_body.CreateKinematicEnabledAttr().Set(False)

    UsdPhysics.CollisionAPI.Apply(prim)

    # Dynamic bodies can't use triangle-mesh collision, so use a convex hull.
    if prim.IsA(UsdGeom.Mesh):
        mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
        mesh_collision.CreateApproximationAttr().Set(
            UsdPhysics.Tokens.convexHull
        )

    mass_api = UsdPhysics.MassAPI.Apply(prim)
    if not mass_api.GetMassAttr().HasAuthoredValueOpinion():
        mass_api.CreateMassAttr().Set(config.DEFAULT_MASS)

    physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    physx_body.CreateLinearDampingAttr().Set(config.LINEAR_DAMPING)
    physx_body.CreateAngularDampingAttr().Set(config.ANGULAR_DAMPING)


def arcade_to_tank(drive, steer):
    """Mix drive and steer in [-1, 1] into (left, right) side voltages."""
    left = drive - steer
    right = drive + steer
    # Scale both sides down together so turning still works at full drive.
    scale = max(1.0, abs(left), abs(right))
    max_voltage = config.MAX_VOLTAGE
    return max_voltage * left / scale, max_voltage * right / scale


def motor_torque(voltage, motor_speed):
    """Output torque of one motor from its voltage and speed (rad/s)."""
    back_emf = motor_speed / config.MOTOR_KV
    current = (voltage - back_emf) / config.MOTOR_RESISTANCE
    current = max(-config.CURRENT_LIMIT, min(config.CURRENT_LIMIT, current))
    return config.MOTOR_KT * current


def side_force(voltage, ground_speed):
    """Force one drive side puts on the ground, from its ground speed."""
    wheel_speed = ground_speed / config.WHEEL_RADIUS
    motor_speed = wheel_speed / config.DRIVE_GEAR_RATIO
    torque = motor_torque(voltage, motor_speed)
    wheel_torque = config.MOTORS_PER_SIDE * torque / config.DRIVE_GEAR_RATIO
    return wheel_torque / config.WHEEL_RADIUS


def quaternion_to_yaw(quaternion):
    """Yaw about Z from an Isaac Sim [w, x, y, z] quaternion."""
    w, x, y, z = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def to_numpy(value):
    """Convert Warp/Isaac arrays to NumPy arrays."""
    if hasattr(value, "numpy"):
        return value.numpy()
    return np.asarray(value)


class DriveController:
    def __init__(self, prims):
        self.keys = set()
        self.keyboard_subscription = None
        self.timeline_subscription = None
        self.physics_callback = None
        self.prims = prims
        self.v_left = 0.0
        self.v_right = 0.0

        self.timeline = omni.timeline.get_timeline_interface()

        if not self.timeline.is_stopped():
            raise RuntimeError("Timeline must be stopped; run again.")

        for prim in self.prims:
            setup_rigid_body(prim)
        self.robots = RigidPrim([str(prim.GetPath()) for prim in self.prims])

        self.physics_callback = SimulationManager.register_callback(
            self.on_physics_step, event=SimulationEvent.PHYSICS_PRE_STEP
        )

        self.input = carb.input.acquire_input_interface()
        self.keyboard = omni.appwindow.get_default_app_window().get_keyboard()

        self.keyboard_subscription = self.input.subscribe_to_keyboard_events(
            self.keyboard, self.on_keyboard
        )

        event_stream = self.timeline.get_timeline_event_stream()
        self.timeline_subscription = event_stream.create_subscription_to_pop(
            self.on_timeline_event
        )

        paths = ", ".join(str(prim.GetPath()) for prim in self.prims)
        print(f"Tank controller loaded for: {paths}")

        # Compare the motor model with the robot's identified model.
        model_speed = (
            config.MAX_VOLTAGE * config.MOTOR_KV * config.DRIVE_GEAR_RATIO
        )
        robot_speed = config.MAX_VOLTAGE * config.MOTOR_B / config.MOTOR_A
        print(
            f"Wheel top speed at {config.MAX_VOLTAGE:.0f} V: model "
            f"{model_speed:.1f} rad/s, robot-core {robot_speed:.1f} rad/s"
        )

    def update_voltages(self):
        key = carb.input.KeyboardInput

        drive = int(key.W in self.keys) - int(key.S in self.keys)
        steer = int(key.A in self.keys) - int(key.D in self.keys)

        self.set_voltages(*arcade_to_tank(drive, steer))

    def set_voltages(self, v_left, v_right):
        """Set the side voltages, clamped to +/-config.MAX_VOLTAGE."""
        max_voltage = config.MAX_VOLTAGE
        self.v_left = max(-max_voltage, min(max_voltage, v_left))
        self.v_right = max(-max_voltage, min(max_voltage, v_right))

    def on_physics_step(self, dt, context):
        if not self.robots.is_physics_tensor_entity_valid():
            return

        _, orientations = self.robots.get_world_poses()
        linear_velocities, angular_velocities = self.robots.get_velocities()
        orientations = to_numpy(orientations)
        linear_velocities = to_numpy(linear_velocities)
        angular_velocities = to_numpy(angular_velocities)

        forces = np.zeros((len(self.prims), 3), dtype=np.float32)
        torques = np.zeros((len(self.prims), 3), dtype=np.float32)
        for i in range(len(self.prims)):
            yaw = quaternion_to_yaw(orientations[i])
            forward_speed = (
                linear_velocities[i][0] * math.cos(yaw)
                + linear_velocities[i][1] * math.sin(yaw)
            )
            yaw_rate = angular_velocities[i][2]

            half_track = config.TRACK_WIDTH / 2.0
            left_speed = forward_speed - yaw_rate * half_track
            right_speed = forward_speed + yaw_rate * half_track
            left_force = side_force(self.v_left, left_speed)
            right_force = side_force(self.v_right, right_speed)

            # Two side forces equal one net force plus a yaw torque.
            # TODO(real robot): this assumes the centre of mass sits midway
            # between the wheels and ignores force height. When the real
            # robot model replaces the cube, apply each side's force at its
            # wheel contact point (apply_forces_and_torques_at_pos's
            # positions) so an off-centre mass and pitching show up.
            forces[i][0] = left_force + right_force
            torques[i][2] = (right_force - left_force) * half_track

        self.robots.apply_forces_and_torques_at_pos(
            forces=forces, torques=torques, local_frame=True
        )

    def on_keyboard(self, event, *args):
        key = carb.input.KeyboardInput
        controlled_keys = {key.W, key.A, key.S, key.D, key.SPACE}

        if event.input not in controlled_keys:
            return True

        if event.type in (
            carb.input.KeyboardEventType.KEY_PRESS,
            carb.input.KeyboardEventType.KEY_REPEAT,
        ):
            if event.input == key.SPACE:
                self.clear_input()
                return True

            # Ignore new driving input while paused or stopped.
            if self.timeline.is_playing():
                self.keys.add(event.input)

        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self.keys.discard(event.input)

        self.update_voltages()
        return True

    def on_timeline_event(self, event):
        if event.type in (
            int(omni.timeline.TimelineEventType.STOP),
            int(omni.timeline.TimelineEventType.PAUSE),
        ):
            self.clear_input()

    def clear_input(self):
        self.keys.clear()
        self.set_voltages(0.0, 0.0)

    def shutdown(self):
        self.clear_input()

        if self.physics_callback is not None:
            SimulationManager.deregister_callback(self.physics_callback)
            self.physics_callback = None

        if self.keyboard_subscription is not None:
            self.input.unsubscribe_to_keyboard_events(
                self.keyboard, self.keyboard_subscription
            )
            self.keyboard_subscription = None

        self.timeline_subscription = None


async def load_tank_controls():
    global _tank_controller

    app = omni.kit.app.get_app()
    timeline = omni.timeline.get_timeline_interface()

    # Tear down the controller from a previous run in this Script Editor.
    previous_controller = globals().get("_tank_controller")
    if previous_controller is not None:
        previous_controller.shutdown()
    _tank_controller = None

    # Check before touching the timeline so nothing changes when no prim is
    # tagged.
    stage = omni.usd.get_context().get_stage()
    prims = find_robot_prims(stage)
    if not prims:
        carb.log_error(
            f"Drive controller not loaded: no prims with "
            f"'{config.CONTROLLABLE_ATTR}' set to true on the stage."
        )
        return

    # Request Stop and let Isaac Sim process the state change.
    timeline.stop()
    await app.next_update_async()

    # Wait until the timeline actually reports that it has stopped.
    for _ in range(config.STOP_TIMEOUT_FRAMES):
        if timeline.is_stopped():
            break
        await app.next_update_async()
    else:
        carb.log_error("Isaac Sim did not finish stopping the timeline.")
        return

    # Exceptions raised inside this task are not shown in the Script
    # Editor, so log them explicitly.
    try:
        _tank_controller = DriveController(prims)
    except Exception as error:
        carb.log_error(f"Drive controller failed to load: {error}")
        return

    # Start simulation after the controller has been installed.
    timeline.play()


_tank_loader_task = asyncio.ensure_future(load_tank_controls())
