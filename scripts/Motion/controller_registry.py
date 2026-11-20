"""Where the loaded drive controller registers itself, so other tools (the
tuning window in plugin/tuning) can find it and start trials.

drive_controller.py registers its DriveController when it loads and
unregisters it on shutdown. This module holds that state across Script
Editor runs, so it must never be reloaded.

No Isaac Sim imports.
"""

_controller = None


def register(controller):
    global _controller
    _controller = controller


def unregister(controller):
    """Forget controller, if it's the registered one."""
    global _controller
    if _controller is controller:
        _controller = None


def get():
    """The loaded DriveController, or None if none is loaded."""
    return _controller
