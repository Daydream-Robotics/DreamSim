"""Tank-drive WASD controls for robots in Isaac Sim.

Paste into the Script Editor and run. The script stops the timeline,
installs the controller, then starts the simulation again.

Every prim on the stage whose config.CONTROLLABLE_ATTR is true is driven by
the controls. To tag a prim, add a uniform Bool attribute with that name in
the Property panel, tick it, then save the stage.

Controls: W/S drive, A/D turn, Space clears all input. The keys are mixed
into left/right side voltages, the same inputs robot-core's MPC produces.

P runs autonomous: the first robot is placed at the start of
config.MPC_PATH_FILE and robot-core's MPC drives its paths in order (see
autonomous.py), printing a summary per path. Any drive key or Space takes
back control. Isaac Sim's world axes are taken as JerryIO's field frame
(field centred on the origin, +x away from the red driver station).
Each physics step, every side's voltage and wheel speed go through a V5
motor model to get that side's drive force, shared across its wheels.
Each wheel also resists sliding sideways, up to its friction limit. The
robot's own collider has almost no friction, so all grip comes from the
wheels.

Settings live in drivetrain_config.py next to this file, which reads the
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
        if (candidate / "drivetrain_config.py").is_file():
            return candidate
    raise RuntimeError(
        "Couldn't find scripts/Motion/drivetrain_config.py. Open a scene from "
        "the DreamSim repo with File > Open, then run this again."
    )


_motion_dir = str(find_motion_dir())
if _motion_dir not in sys.path:
    sys.path.insert(0, _motion_dir)

import autonomous  # noqa: E402
import drivetrain_config as config  # noqa: E402
import drivetrain_model as model  # noqa: E402
import mpc_bridge  # noqa: E402
import path_display  # noqa: E402

# Reload so edits to these files or robot-core apply without a restart.
# Config first: the others read it when they load.
config = importlib.reload(config)
mpc_bridge = importlib.reload(mpc_bridge)
model = importlib.reload(model)
autonomous = importlib.reload(autonomous)
path_display = importlib.reload(path_display)


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
    # Likewise no stabilization, which also treats slow bodies as resting.
    # With it on, the robot sat still while PhysX reported a constant
    # -4 deg/s turn (the heading didn't change over 5 s).
    physx_body.CreateStabilizationThresholdAttr().Set(0.0)


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
        self.output_file = None
        self.prims = prims
        self.v_left = 0.0
        self.v_right = 0.0
        self.brake = config.KEYBOARD_BRAKE_ON_RELEASE
        self.status_time = 0.0
        self.voltages_changed = False
        self.rate_checked = False
        self.autonomous = None
        self.pending_autonomous = None

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

        # Position and path output goes to a file, overwritten each run.
        self.output_file = open(
            config.OUTPUT_FILE, "w", encoding="utf-8", buffering=1
        )

        paths = ", ".join(str(prim.GetPath()) for prim in self.prims)
        print(f"Tank controller loaded for: {paths}")
        print(f"Position and path output: {config.OUTPUT_FILE}")

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
        self.set_voltages(*model.arcade_to_tank(drive, steer), brake=brake)

    def set_voltages(self, v_left, v_right, brake=False):
        """Set the side voltages, clamped to +/-config.MAX_VOLTAGE.

        brake makes sides at 0 V brake even when the drive is set to coast.
        """
        v_left = model.clamp(v_left, config.MAX_VOLTAGE)
        v_right = model.clamp(v_right, config.MAX_VOLTAGE)
        if (v_left, v_right, brake) != (self.v_left, self.v_right, self.brake):
            self.voltages_changed = True
        self.v_left = v_left
        self.v_right = v_right
        self.brake = brake

    def on_physics_step(self, dt, context):
        # drive_wrench divides by dt, so skip zero-length steps.
        if dt <= 0.0 or not self.robots.is_physics_tensor_entity_valid():
            return

        positions, orientations = self.robots.get_world_poses()
        linear_velocities, angular_velocities = self.robots.get_velocities()
        positions = to_numpy(positions)
        orientations = to_numpy(orientations)
        linear_velocities = to_numpy(linear_velocities)
        angular_velocities = to_numpy(angular_velocities)

        if not self.rate_checked:
            self.rate_checked = True
            self.check_physics_rate(dt)

        # The robot is moved to the path start here, between physics steps,
        # and the MPC takes over from the next step.
        if self.pending_autonomous is not None:
            run = self.pending_autonomous
            self.pending_autonomous = None
            self.start_autonomous(run, positions[0])
            return

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
            if tipped and i == 0 and self.autonomous is not None:
                self.cancel_autonomous("the robot tipped over")
            if tipped:
                if print_status:
                    self.print_status(
                        i, positions[i], rotation, forward_speed,
                        sideways_speed, yaw_rate, tipped=True,
                    )
                continue

            # Autonomous drives from the first robot's state. Its voltages
            # are set before any wrench is computed this step.
            if i == 0 and self.autonomous is not None:
                self.step_autonomous(
                    positions[0], rotation, forward_speed, yaw_rate
                )

            # TODO(real robot): the wheel forces are summed about the body
            # origin, which assumes the centre of mass is there, and they
            # ignore force height. When the real robot model replaces the
            # cube, sum them about its actual centre of mass (or apply each
            # at its contact point with apply_forces_and_torques_at_pos's
            # positions) so an off-centre mass and pitching show up.
            force_x, force_y, torque_z = model.drive_wrench(
                self.v_left, self.v_right,
                forward_speed, sideways_speed, yaw_rate, dt, self.brake,
            )
            forces[i][0] = force_x
            forces[i][1] = force_y
            torques[i][2] = torque_z

            if print_status:
                self.print_status(
                    i, positions[i], rotation, forward_speed,
                    sideways_speed, yaw_rate,
                )

        self.robots.apply_forces_and_torques_at_pos(
            forces=forces, torques=torques, local_frame=True
        )

    def prepare_autonomous(self):
        """Load the path file and draw its paths (P key).

        Runs from the keyboard handler, outside the physics step, so the
        drawing doesn't change the stage mid-step. The next physics step
        places the robot and starts driving.
        """
        self.cancel_autonomous()
        self.discard_pending_autonomous()
        try:
            run = autonomous.Autonomous(config.MPC_PATH_FILE, log=self.output)
        except Exception as error:
            carb.log_error(f"Autonomous not started: {error}")
            return

        if config.SHOW_MPC_PATHS:
            try:
                self.draw_paths(run.bridge)
            except Exception as error:
                carb.log_warn(f"Couldn't draw the path: {error}")

        self.pending_autonomous = run

    def draw_paths(self, bridge):
        """Draw the bridge's paths, coloured per config.PATH_DISPLAY_SPEED."""
        paths = []
        for i in range(bridge.path_count):
            samples = bridge.samples(i)
            if config.PATH_DISPLAY_SPEED == "reference":
                speeds = bridge.reference_speeds(i)
                samples = [
                    (x, y, speed) for (x, y, _), speed in zip(samples, speeds)
                ]
            paths.append(samples)

        v_min, v_max = path_display.draw_paths(
            omni.usd.get_context().get_stage(), paths,
            height=config.PATH_DISPLAY_HEIGHT,
            width=config.PATH_DISPLAY_WIDTH,
            session=True,
        )
        if config.PATH_DISPLAY_SPEED == "reference":
            self.output(
                f"Path coloured by MPC reference speed: {v_min:.1f} in/s "
                f"(red) to {v_max:.1f} in/s (green)"
            )
        else:
            self.output(
                f"Path coloured by the file's speeds: {v_min:.1f} (red) to "
                f"{v_max:.1f} (green)"
            )

    def discard_pending_autonomous(self):
        if self.pending_autonomous is not None:
            self.pending_autonomous.close()
            self.pending_autonomous = None

    def start_autonomous(self, run, current_position):
        """Move the first robot to the run's start pose and start it."""
        self.autonomous = run

        x, y, heading = self.autonomous.start_pose()
        yaw = model.yaw_from_heading(heading)
        position = [x * config.INCH, y * config.INCH, current_position[2]]
        orientation = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]
        self.robots.set_world_poses(
            positions=np.array([position], dtype=np.float32),
            orientations=np.array([orientation], dtype=np.float32),
            indices=[0],
        )
        self.robots.set_velocities(
            np.zeros((1, 3), dtype=np.float32),
            np.zeros((1, 3), dtype=np.float32),
            indices=[0],
        )
        self.keys.clear()
        self.set_voltages(0.0, 0.0, brake=True)

        self.output(
            f"Autonomous: {self.autonomous.bridge.path_count} paths from "
            f"{config.MPC_PATH_FILE}"
        )
        self.output(
            f"Robot placed at ({x:.1f}, {y:.1f}) in, heading "
            f"{math.degrees(heading):.0f} deg. Drive keys or Space take "
            "back control."
        )
        if len(self.prims) > 1:
            self.output(
                "Autonomous follows the first robot; the others get the "
                "same voltages."
            )

    def step_autonomous(self, position, rotation, forward_speed, yaw_rate):
        """Run one physics step of autonomous from the first robot's state."""
        yaw = math.atan2(rotation[1][0], rotation[0][0])
        omega_left, omega_right = model.wheel_speeds(forward_speed, yaw_rate)
        v_left, v_right, brake = self.autonomous.physics_step(
            position[0] / config.INCH,
            position[1] / config.INCH,
            model.heading_from_yaw(yaw),
            omega_left,
            omega_right,
        )
        self.set_voltages(v_left, v_right, brake=brake)
        if self.autonomous.done and v_left == 0.0 and v_right == 0.0:
            self.autonomous.close()
            self.autonomous = None

    def cancel_autonomous(self, reason=None):
        """Stop autonomous, if running, and brake."""
        if self.autonomous is None:
            return
        self.autonomous.close()
        self.autonomous = None
        self.set_voltages(0.0, 0.0, brake=True)
        if reason:
            self.output(f"Autonomous stopped: {reason}.")

    def output(self, line):
        """Write a position or path line to config.OUTPUT_FILE."""
        if self.output_file is not None:
            self.output_file.write(line + "\n")

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

    def print_status(self, index, position, rotation, forward_speed,
                     sideways_speed, yaw_rate, tipped=False):
        """Write one robot's pose and motion to the output file, skipping
        robots that aren't moving.

        Position and heading are in robot-core's field frame (inches,
        heading 0 along +y, clockwise). The heading comes from the pose,
        not the velocity, so a turn rate that doesn't match a changing
        heading points at a bad reading.
        """
        moving = (
            abs(forward_speed) > 0.01
            or abs(sideways_speed) > 0.01
            or abs(yaw_rate) > 0.05
        )
        if not moving:
            return
        yaw = math.atan2(rotation[1][0], rotation[0][0])
        heading = math.degrees(model.heading_from_yaw(yaw))
        self.output(
            f"{self.prims[index].GetPath()}: "
            f"x {position[0] / config.INCH:+7.2f} in  "
            f"y {position[1] / config.INCH:+7.2f} in  "
            f"V {self.v_left:+5.1f}/{self.v_right:+5.1f}  "
            f"forward {forward_speed / config.INCH:+6.1f} in/s  "
            f"sideways {sideways_speed / config.INCH:+6.1f} in/s  "
            f"turn {math.degrees(yaw_rate):+6.0f} deg/s  "
            f"heading {heading:+7.2f} deg"
            + ("  TIPPED, wheels off" if tipped else "")
        )

    def on_keyboard(self, event, *args):
        key = carb.input.KeyboardInput
        controlled_keys = {key.W, key.A, key.S, key.D, key.SPACE}
        pressed = event.type in (
            carb.input.KeyboardEventType.KEY_PRESS,
            carb.input.KeyboardEventType.KEY_REPEAT,
        )

        # P starts (or restarts) autonomous: the path is drawn now, the
        # robot moves on the next physics step.
        if event.input == key.P:
            if (
                event.type == carb.input.KeyboardEventType.KEY_PRESS
                and self.timeline.is_playing()
            ):
                self.prepare_autonomous()
            return True

        if event.input not in controlled_keys:
            return True

        if pressed and self.autonomous is not None:
            self.cancel_autonomous("taken over by the keyboard")

        if pressed:
            if event.input == key.SPACE:
                self.clear_input()
                return True

            # Ignore new driving input while paused or stopped.
            if self.timeline.is_playing():
                self.keys.add(event.input)

        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self.keys.discard(event.input)
            # Releasing a key held from before doesn't override autonomous.
            if self.autonomous is not None:
                return True

        self.update_voltages()
        return True

    def on_timeline_event(self, event):
        if event.type in (
            int(omni.timeline.TimelineEventType.STOP),
            int(omni.timeline.TimelineEventType.PAUSE),
        ):
            self.discard_pending_autonomous()
            self.cancel_autonomous("the simulation stopped")
            self.clear_input()

    def clear_input(self):
        self.keys.clear()
        self.set_voltages(
            0.0, 0.0, brake=config.KEYBOARD_BRAKE_ON_RELEASE
        )

    def shutdown(self):
        self.discard_pending_autonomous()
        self.cancel_autonomous()
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

        if self.output_file is not None:
            self.output_file.close()
            self.output_file = None


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
