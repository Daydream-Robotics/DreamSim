"""The tuning suite's Isaac Sim extension: menu entry and window.

Opens from Window > DreamSim > MPC Tuning. The window is the home for the
tuning tools; for now it shows where the suite found the repo and lists
robot-core's constants.h by category (read-only).
"""

import weakref
from pathlib import Path

import carb
import omni.ext
import omni.ui as ui
from omni.kit.menu.utils import (
    MenuItemDescription,
    add_menu_items,
    remove_menu_items,
)

from . import constants_file

WINDOW_TITLE = "MPC Tuning"
# Categories shown expanded when the window opens.
EXPANDED_CATEGORIES = {"MPC", "Motion Controllers"}
MUTED = 0xFFA0A0A0
ERROR = 0xFF4040FF
MENU_GROUP = "DreamSim"

# plugin/tuning/dreamsim/tuning/extension.py -> the repo root.
REPO_ROOT = Path(__file__).resolve().parents[4]
ROBOT_CORE = (
    REPO_ROOT / "native" / "robotcore_mpc" / "third_party" / "robotcore"
)
CONSTANTS_HEADER = ROBOT_CORE / "shared" / "include" / "config" / "constants.h"


class TuningExtension(omni.ext.IExt):
    """Adds the MPC Tuning window and its menu entry."""

    def on_startup(self, ext_id):
        self._ext_id = ext_id
        self._window = None
        # A weak proxy so the menu doesn't keep the extension alive after
        # it's disabled or hot-reloaded.
        proxy = weakref.proxy(self)
        self._menu_items = [
            MenuItemDescription(
                name=MENU_GROUP,
                sub_menu=[
                    MenuItemDescription(
                        name=WINDOW_TITLE,
                        onclick_fn=lambda: proxy.toggle_window(),
                    )
                ],
            )
        ]
        add_menu_items(self._menu_items, "Window")
        carb.log_info(f"[{ext_id}] DreamSim MPC Tuning loaded")

    def on_shutdown(self):
        remove_menu_items(self._menu_items, "Window")
        self._menu_items = []
        if self._window is not None:
            self._window.destroy()
            self._window = None

    def toggle_window(self):
        """Show the window, building it on first use, or hide it."""
        if self._window is None:
            self._build_window()
            return
        self._window.visible = not self._window.visible

    def _build_window(self):
        self._window = ui.Window(
            WINDOW_TITLE,
            width=560,
            height=600,
            dockPreference=ui.DockPreference.RIGHT_TOP,
        )
        with self._window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=6, height=0):
                    ui.Label(
                        "DreamSim MPC tuning",
                        style={"font_size": 18},
                        height=24,
                    )
                    ui.Label(
                        "Tunes robot-core's MPC constants against the sim. "
                        "Physics constants (drivetrain_config.py) stay "
                        "fixed.",
                        word_wrap=True,
                    )
                    ui.Spacer(height=4)
                    with ui.CollapsableFrame("Setup", height=0):
                        with ui.VStack(spacing=4, height=0):
                            self._path_row("Repo", REPO_ROOT)
                            self._path_row("constants.h", CONSTANTS_HEADER)
                    with ui.HStack(height=24, spacing=6):
                        ui.Label("constants.h", style={"font_size": 16})
                        ui.Button(
                            "Reload", width=80,
                            clicked_fn=self._reload_constants,
                            tooltip="Re-read constants.h from disk",
                        )
                    # Rebuilt on Reload.
                    self._constants_frame = ui.Frame(
                        height=0, build_fn=self._build_constants
                    )

    def _reload_constants(self):
        self._constants_frame.rebuild()

    def _build_constants(self):
        """List constants.h by category, read fresh from disk."""
        try:
            categories = constants_file.parse(CONSTANTS_HEADER)
        except OSError as error:
            ui.Label(
                f"Couldn't read constants.h: {error}",
                style={"color": ERROR}, word_wrap=True,
            )
            return
        with ui.VStack(spacing=4, height=0):
            for category in categories:
                title = f"{category.name}  ({len(category.constants)})"
                with ui.CollapsableFrame(
                    title, height=0,
                    collapsed=category.name not in EXPANDED_CATEGORIES,
                ):
                    with ui.VStack(spacing=2, height=0):
                        if category.description:
                            ui.Label(
                                category.description, word_wrap=True,
                                style={"color": MUTED},
                            )
                        for group in category.groups:
                            self._build_group(group)

    def _build_group(self, group):
        if group.name:
            ui.Spacer(height=2)
            ui.Label(
                group.name, word_wrap=True,
                style={"color": MUTED, "font_size": 13},
            )
        for constant in group.constants:
            with ui.HStack(height=20, spacing=8):
                ui.Label(
                    constant.name, width=240,
                    tooltip=f"{constant.ctype} {constant.name}, "
                            f"line {constant.line}",
                )
                ui.Label(
                    _format_value(constant), width=110,
                    elided_text=True, tooltip=constant.raw,
                )
                ui.Label(
                    constant.comment, elided_text=True,
                    style={"color": MUTED}, tooltip=constant.comment,
                )

    def _path_row(self, label, path):
        found = path.exists()
        with ui.HStack(height=20, spacing=6):
            ui.Label(label, width=90)
            ui.Label(
                str(path) if found else f"not found: {path}",
                style={"color": MUTED if found else ERROR},
                elided_text=True,
                tooltip=str(path),
            )


def _format_value(constant):
    """The value as shown: the evaluated number, with the expression when
    it isn't a plain literal (e.g. "1 (600.0/600.0)").
    """
    value = constant.value
    if value is None:
        return constant.raw
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str):
        return f'"{value}"'
    shown = f"{value:g}"
    plain = constant.raw.replace("f", "")
    try:
        is_literal = float(plain) == value
    except ValueError:
        is_literal = False
    return shown if is_literal else f"{shown} ({constant.raw})"
