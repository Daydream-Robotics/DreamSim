"""Tank-drive WASD controls for robots in Isaac Sim.

Paste into the Script Editor and run. The script stops the timeline,
installs the controller, then starts the simulation again.

Every prim on the stage whose config.CONTROLLABLE_ATTR is true is driven by
the controls. To tag a prim, add a uniform Bool attribute with that name in
the Property panel, tick it, then save the stage.

Controls: W/S drive, A/D turn, Space clears all input. The keys are mixed
into left/right side voltages, the same inputs robot-core's MPC produces.
Each physics step, every side's voltage and wheel speed go through a V5
motor model to get that side's drive force, shared across its wheels.
Each wheel also resists sliding sideways, up to its friction limit. The
robot's own collider has almost no friction, so all grip comes from the
wheels.

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
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade


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
    mass_api.CreateMassAttr().Set(config.ROBOT_MASS)
    mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*config.ROBOT_INERTIA))

    physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    physx_body.CreateLinearDampingAttr().Set(config.LINEAR_DAMPING)
    physx_body.CreateAngularDampingAttr().Set(config.ANGULAR_DAMPING)
    # Never sleep. PhysX puts slow bodies to sleep (default threshold 5e-5,
    # which a robot creeping at a few deg/s is under). A sleeping body stops
    # being simulated, and its velocity reads back stale: the robot showed a
    # constant turn rate at 0 V that the wheel model could never damp out.
    physx_body.CreateSleepThresholdAttr().Set(0.0)


def setup_robot_material(stage, prims):
    """Bind a low-friction physics material to the robots' colliders."""
    root = stage.GetDefaultPrim()
    root_path = root.GetPath() if root else Sdf.Path.absoluteRootPath
    material = UsdShade.Material.Define(
        stage, root_path.AppendChild("RobotPhysicsMaterial")
    )

    material_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    material_api.CreateStaticFrictionAttr().Set(config.ROBOT_COLLIDER_FRICTION)
    material_api.CreateDynamicFrictionAttr().Set(
        config.ROBOT_COLLIDER_FRICTION
    )
    material_api.CreateRestitutionAttr().Set(0.0)

    # PhysX averages two surfaces' friction by default, which would bring
    # the field's friction back in. "min" wins over "average".
    physx_material = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx_material.CreateFrictionCombineModeAttr().Set("min")

    for prim in prims:
        binding = UsdShade.MaterialBindingAPI.Apply(prim)
        binding.Bind(
            material, UsdShade.Tokens.weakerThanDescendants, "physics"
        )


def check_stage_units(stage):
    """Return an error message if the stage isn't metres, kg and Z-up."""
    # The motor model works in newtons and the yaw maths assumes Z-up.
    # Changing a stage's units would rescale everything else in it, so
    # refuse to run instead.
    meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
    kilograms_per_unit = UsdPhysics.GetStageKilogramsPerUnit(stage)
    up_axis = UsdGeom.GetStageUpAxis(stage)
    if meters_per_unit != 1.0 or kilograms_per_unit != 1.0:
        return (
            f"stage units are {meters_per_unit} m and {kilograms_per_unit} "
            "kg per unit; the drive model needs 1.0 for both."
        )
    if up_axis != UsdGeom.Tokens.z:
        return f"stage up axis is {up_axis}; the drive model needs Z."
    return None


def setup_physics_scenes(stage):
    """Set every physics scene's rate to config.PHYSICS_STEPS_PER_SECOND."""
    scenes = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
    if not scenes:
        scenes = [UsdPhysics.Scene.Define(stage, "/PhysicsScene").GetPrim()]
    for scene in scenes:
        physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene)
        physx_scene.CreateTimeStepsPerSecondAttr().Set(
            config.PHYSICS_STEPS_PER_SECOND
        )


def arcade_to_tank(drive, steer):
    """Mix drive and steer in [-1, 1] into (left, right) side voltages."""
    left = drive - steer
    right = drive + steer
    # Scale both sides down together so turning still works at full drive.
    scale = max(1.0, abs(left), abs(right))
    max_voltage = config.MAX_VOLTAGE
    return max_voltage * left / scale, max_voltage * right / scale


def clamp(value, limit):
    """Limit value to the range [-limit, limit]."""
    return max(-limit, min(limit, value))


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


def quaternion_to_matrix(quaternion):
    """Body-to-world rotation matrix from an Isaac Sim [w, x, y, z]
    quaternion. Its columns are the body's x, y and z axes in world space.
    """
    w, x, y, z = quaternion / np.linalg.norm(quaternion)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


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
        self.brake = config.KEYBOARD_BRAKE_ON_RELEASE
        self.status_time = 0.0
        self.voltages_changed = False
        self.rate_checked = False

        self.timeline = omni.timeline.get_timeline_interface()

        if not self.timeline.is_stopped():
            raise RuntimeError("Timeline must be stopped; run again.")

        for prim in self.prims:
            setup_rigid_body(prim)
        setup_robot_material(omni.usd.get_context().get_stage(), self.prims)
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
        print(f"Config: {config.__file__}")
        print(
            f"Physics: {config.PHYSICS_STEPS_PER_SECOND} steps/s, "
            f"{config.STEPS_PER_CONTROL_PERIOD} per "
            f"{config.CONTROL_PERIOD * 1000:.0f} ms control period"
        )

    def update_voltages(self):
        key = carb.input.KeyboardInput

        drive = int(key.W in self.keys) - int(key.S in self.keys)
        steer = int(key.A in self.keys) - int(key.D in self.keys)

        # Letting go of every key brakes, if the config asks for it.
        released = drive == 0 and steer == 0
        brake = released and config.KEYBOARD_BRAKE_ON_RELEASE
        self.set_voltages(*arcade_to_tank(drive, steer), brake=brake)

    def set_voltages(self, v_left, v_right, brake=False):
        """Set the side voltages, clamped to +/-config.MAX_VOLTAGE.

        brake makes sides at 0 V brake even when the drive is set to coast.
        """
        v_left = clamp(v_left, config.MAX_VOLTAGE)
        v_right = clamp(v_right, config.MAX_VOLTAGE)
        if (v_left, v_right, brake) != (self.v_left, self.v_right, self.brake):
            self.voltages_changed = True
        self.v_left = v_left
        self.v_right = v_right
        self.brake = brake

    def on_physics_step(self, dt, context):
        # drive_wrench divides by dt, so skip zero-length steps.
        if dt <= 0.0 or not self.robots.is_physics_tensor_entity_valid():
            return

        _, orientations = self.robots.get_world_poses()
        linear_velocities, angular_velocities = self.robots.get_velocities()
        orientations = to_numpy(orientations)
        linear_velocities = to_numpy(linear_velocities)
        angular_velocities = to_numpy(angular_velocities)

        if not self.rate_checked:
            self.rate_checked = True
            self.check_physics_rate(dt)

        # Print on a fixed interval, and on the step a key press or release
        # changes the voltages so the instant of the change is captured.
        self.status_time += dt
        print_status = config.STATUS_INTERVAL > 0.0 and (
            self.status_time >= config.STATUS_INTERVAL
            or self.voltages_changed
        )
        if print_status:
            self.status_time = 0.0
            self.voltages_changed = False

        forces = np.zeros((len(self.prims), 3), dtype=np.float32)
        torques = np.zeros((len(self.prims), 3), dtype=np.float32)
        for i in range(len(self.prims)):
            # Measure motion in the robot's own frame, the same frame the
            # forces are applied in (local_frame=True below).
            rotation = quaternion_to_matrix(orientations[i])
            body_velocity = rotation.T @ linear_velocities[i]
            body_angular_velocity = rotation.T @ angular_velocities[i]
            forward_speed, sideways_speed = body_velocity[:2]
            yaw_rate = body_angular_velocity[2]

            # Tipped over: the wheels aren't on the ground.
            tipped = rotation[2][2] < math.cos(config.MAX_WHEEL_TILT)
            if tipped:
                if print_status:
                    self.print_status(
                        i, forward_speed, sideways_speed, yaw_rate,
                        tipped=True,
                    )
                continue

            # TODO(real robot): the wheel forces are summed about the body
            # origin, which assumes the centre of mass is there, and they
            # ignore force height. When the real robot model replaces the
            # cube, sum them about its actual centre of mass (or apply each
            # at its contact point with apply_forces_and_torques_at_pos's
            # positions) so an off-centre mass and pitching show up.
            force_x, force_y, torque_z = drive_wrench(
                self.v_left, self.v_right,
                forward_speed, sideways_speed, yaw_rate, dt, self.brake,
            )
            forces[i][0] = force_x
            forces[i][1] = force_y
            torques[i][2] = torque_z

            if print_status:
                self.print_status(
                    i, forward_speed, sideways_speed, yaw_rate
                )

        self.robots.apply_forces_and_torques_at_pos(
            forces=forces, torques=torques, local_frame=True
        )

    def check_physics_rate(self, dt):
        """Report the rate PhysX actually runs at, from the step length."""
        actual_rate = 1.0 / dt
        print(f"Physics running at {actual_rate:.0f} steps/s")
        if not math.isclose(
            actual_rate, config.PHYSICS_STEPS_PER_SECOND, rel_tol=0.01
        ):
            carb.log_warn(
                f"Physics is running at {actual_rate:.0f} steps/s, not the "
                f"configured {config.PHYSICS_STEPS_PER_SECOND}. Something "
                "else is setting the physics rate."
            )

    def print_status(self, index, forward_speed, sideways_speed, yaw_rate,
                     tipped=False):
        """Print one robot's motion, skipping robots sitting still."""
        moving = (
            abs(forward_speed) > 0.01
            or abs(sideways_speed) > 0.01
            or abs(yaw_rate) > 0.05
        )
        if not moving and self.v_left == 0.0 and self.v_right == 0.0:
            return
        print(
            f"{self.prims[index].GetPath()}: "
            f"V {self.v_left:+5.1f}/{self.v_right:+5.1f}  "
            f"forward {forward_speed / config.INCH:+6.1f} in/s  "
            f"sideways {sideways_speed / config.INCH:+6.1f} in/s  "
            f"turn {math.degrees(yaw_rate):+6.0f} deg/s"
            + ("  TIPPED, wheels off" if tipped else "")
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
        self.set_voltages(
            0.0, 0.0, brake=config.KEYBOARD_BRAKE_ON_RELEASE
        )

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

    units_error = check_stage_units(stage)
    if units_error is not None:
        carb.log_error(f"Drive controller not loaded: {units_error}")
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
        # The physics rate can only change while stopped.
        setup_physics_scenes(stage)
        _tank_controller = DriveController(prims)
    except Exception as error:
        carb.log_error(f"Drive controller failed to load: {error}")
        return

    # Start simulation after the controller has been installed.
    timeline.play()


_tank_loader_task = asyncio.ensure_future(load_tank_controls())
