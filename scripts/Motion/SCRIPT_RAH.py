"""Tank-style WASD controls for a physics cube in Isaac Sim.

Paste into the Script Editor and run. The script stops the timeline,
installs the controller, then starts the simulation again.

Controls: W/S drive, A/D turn, Space clears all input.
"""

import asyncio

import carb
import omni.appwindow
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, PhysxSchema, UsdPhysics


# Edit these settings; they're all really bad right now.
CUBE_PATH = "/World/FIELD_WITH_PHYSICS_MOVEMENT/Cube"

DRIVE_FORCE = 40.0
TURN_TORQUE = 0.5

LINEAR_DAMPING = 1.0
ANGULAR_DAMPING = 3.0
DEFAULT_MASS = 5.0

# Frames to wait for the timeline to report that it has stopped.
STOP_TIMEOUT_FRAMES = 60


class TankCubeController:
    def __init__(self):
        self.keys = set()
        self.keyboard_subscription = None
        self.timeline_subscription = None

        self.stage = omni.usd.get_context().get_stage()
        self.prim = self.stage.GetPrimAtPath(CUBE_PATH)

        if not self.prim.IsValid():
            raise RuntimeError(f"No cube found at {CUBE_PATH}.")

        self.timeline = omni.timeline.get_timeline_interface()

        if not self.timeline.is_stopped():
            raise RuntimeError("Timeline must be stopped; run again.")

        rigid_body = UsdPhysics.RigidBodyAPI.Apply(self.prim)
        rigid_body.CreateRigidBodyEnabledAttr().Set(True)
        rigid_body.CreateKinematicEnabledAttr().Set(False)

        UsdPhysics.CollisionAPI.Apply(self.prim)

        mass_api = UsdPhysics.MassAPI.Apply(self.prim)
        mass_attr = mass_api.GetMassAttr()
        if not mass_attr.HasAuthoredValueOpinion():
            mass_api.CreateMassAttr().Set(DEFAULT_MASS)

        physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(self.prim)
        physx_body.CreateLinearDampingAttr().Set(LINEAR_DAMPING)
        physx_body.CreateAngularDampingAttr().Set(ANGULAR_DAMPING)

        self.force_api = PhysxSchema.PhysxForceAPI.Apply(self.prim)
        self.force_api.CreateModeAttr().Set("force")
        self.force_api.CreateWorldFrameEnabledAttr().Set(False)
        self.force_api.CreateForceEnabledAttr().Set(True)

        self.force_attr = self.force_api.CreateForceAttr()
        self.torque_attr = self.force_api.CreateTorqueAttr()
        self.clear_input()

        self.input = carb.input.acquire_input_interface()
        self.keyboard = omni.appwindow.get_default_app_window().get_keyboard()

        self.keyboard_subscription = self.input.subscribe_to_keyboard_events(
            self.keyboard, self.on_keyboard
        )

        event_stream = self.timeline.get_timeline_event_stream()
        self.timeline_subscription = event_stream.create_subscription_to_pop(
            self.on_timeline_event
        )

        print("Tank cube controller loaded.")

    def update_forces(self):
        key = carb.input.KeyboardInput

        drive = int(key.W in self.keys) - int(key.S in self.keys)
        steer = int(key.A in self.keys) - int(key.D in self.keys)

        self.force_attr.Set(Gf.Vec3f(drive * DRIVE_FORCE, 0.0, 0.0))
        self.torque_attr.Set(Gf.Vec3f(0.0, 0.0, steer * TURN_TORQUE))

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

        self.update_forces()
        return True

    def on_timeline_event(self, event):
        if event.type in (
            int(omni.timeline.TimelineEventType.STOP),
            int(omni.timeline.TimelineEventType.PAUSE),
        ):
            self.clear_input()

    def clear_input(self):
        self.keys.clear()
        if self.prim.IsValid():
            self.force_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))
            self.torque_attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))

    def shutdown(self):
        self.clear_input()

        if self.keyboard_subscription is not None:
            self.input.unsubscribe_to_keyboard_events(
                self.keyboard, self.keyboard_subscription
            )
            self.keyboard_subscription = None

        self.timeline_subscription = None


async def load_tank_controls():
    global _tank_cube_controller

    app = omni.kit.app.get_app()
    timeline = omni.timeline.get_timeline_interface()

    # Tear down the controller from a previous run in this Script Editor.
    previous_controller = globals().get("_tank_cube_controller")
    if previous_controller is not None:
        previous_controller.shutdown()
    _tank_cube_controller = None

    # Request Stop and let Isaac Sim process the state change.
    timeline.stop()
    await app.next_update_async()

    # Wait until the timeline actually reports that it has stopped.
    for _ in range(STOP_TIMEOUT_FRAMES):
        if timeline.is_stopped():
            break
        await app.next_update_async()
    else:
        raise RuntimeError("Isaac Sim did not finish stopping the timeline.")

    _tank_cube_controller = TankCubeController()

    # Start simulation after the controller has been installed.
    timeline.play()


_tank_loader_task = asyncio.ensure_future(load_tank_controls())
