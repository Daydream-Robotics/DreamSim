import carb
import omni.appwindow
import omni.timeline
import omni.usd

from pxr import Gf, PhysxSchema, UsdPhysics


# EDIT THESE SETTINGS they're all rlly bad rn 

CUBE_PATH = "/World/Cube"

DRIVE_FORCE = 40.0       
TURN_TORQUE = 0.5     

LINEAR_DAMPING = 1.0   
ANGULAR_DAMPING = 3.0   
DEFAULT_MASS = 5.0     



class TankCubeController:
    def __init__(self):
        self.keys = set()
        self.keyboard_subscription = None
        self.timeline_subscription = None

        self.stage = omni.usd.get_context().get_stage()
        self.prim = self.stage.GetPrimAtPath(CUBE_PATH)

        if not self.prim.IsValid():
            raise RuntimeError(
                f"no cube found"
            )

        self.timeline = omni.timeline.get_timeline_interface()

       
        if not self.timeline.is_stopped():
            raise RuntimeError(
                "run again"
            )

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
        self.keyboard = (
            omni.appwindow.get_default_app_window().get_keyboard()
        )

        self.keyboard_subscription = (
            self.input.subscribe_to_keyboard_events(
                self.keyboard, self.on_keyboard
            )
        )

        self.timeline_subscription = (
            self.timeline.get_timeline_event_stream().create_subscription_to_pop(
                self.on_timeline_event
            )
        )

        print(
            f"the thing loaded"
        )

    def update_forces(self):
        key = carb.input.KeyboardInput

        drive = int(key.W in self.keys) - int(key.S in self.keys)
        steer = int(key.A in self.keys) - int(key.D in self.keys)

        self.force_attr.Set(
            Gf.Vec3f(drive * DRIVE_FORCE, 0.0, 0.0)
        )
        self.torque_attr.Set(
            Gf.Vec3f(0.0, 0.0, steer * TURN_TORQUE)
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



_previous_controller = globals().get("_tank_cube_controller")
if _previous_controller is not None:
    _previous_controller.shutdown()

_tank_cube_controller = None
_tank_cube_controller = TankCubeController()