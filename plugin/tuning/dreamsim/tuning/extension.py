"""The tuning suite's Isaac Sim extension: menu entry and window.

Opens from Window > DreamSim > MPC Tuning. Lists robot-core's constants.h
by category. Constants the sim can apply at runtime (tunable_constants.py)
get an override: a toggle and a value, saved to the overrides file and used
from the next autonomous run (P). The rest show why they're locked.
"""

import importlib
import sys
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
MENU_GROUP = "DreamSim"
# Categories shown expanded when the window opens.
EXPANDED_CATEGORIES = {"MPC", "Motion Controllers"}
MUTED = 0xFFA0A0A0
ERROR = 0xFF4040FF
GOOD = 0xFF60C060
OVERRIDDEN = 0xFF40B0FF
KIND_COLORS = {
    "physical": 0xFF80A0D0,
    "compile-time": 0xFFC0A060,
    "not simulated": 0xFF707070,
}

# plugin/tuning/dreamsim/tuning/extension.py -> the repo root.
REPO_ROOT = Path(__file__).resolve().parents[4]
MOTION_DIR = REPO_ROOT / "scripts" / "Motion"
ROBOT_CORE = (
    REPO_ROOT / "native" / "robotcore_mpc" / "third_party" / "robotcore"
)
CONSTANTS_HEADER = ROBOT_CORE / "shared" / "include" / "config" / "constants.h"
# Same file as drivetrain_config.TUNING_OVERRIDES_FILE, which runs read.
OVERRIDES_FILE = REPO_ROOT / "tuning_overrides.json"


def _motion_modules():
    """tunable_constants and mpc_bridge from scripts/Motion, freshly
    reloaded so edits there apply.
    """
    if str(MOTION_DIR) not in sys.path:
        sys.path.insert(0, str(MOTION_DIR))
    importlib.invalidate_caches()
    import mpc_bridge
    import tunable_constants

    return importlib.reload(tunable_constants), importlib.reload(mpc_bridge)


class TuningExtension(omni.ext.IExt):
    """Adds the MPC Tuning window and its menu entry."""

    def on_startup(self, ext_id):
        self._ext_id = ext_id
        self._window = None
        self._overrides = {}
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
            width=620,
            height=640,
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
                        "fixed. Overrides apply from the next autonomous "
                        "run (P) and never change constants.h.",
                        word_wrap=True,
                    )
                    ui.Spacer(height=4)
                    with ui.CollapsableFrame("Setup", height=0):
                        with ui.VStack(spacing=4, height=0):
                            self._path_row("Repo", REPO_ROOT)
                            self._path_row("constants.h", CONSTANTS_HEADER)
                            self._path_row(
                                "Overrides", OVERRIDES_FILE,
                                missing="none yet (created by the first "
                                        "override)",
                            )
                    with ui.HStack(height=24, spacing=6):
                        ui.Label("constants.h", style={"font_size": 16})
                        ui.Button(
                            "Reload", width=80,
                            clicked_fn=self._rebuild,
                            tooltip="Re-read constants.h, the overrides and "
                                    "the bridge build",
                        )
                        ui.Button(
                            "Disable all", width=90,
                            clicked_fn=self._disable_all,
                            tooltip="Turn every override off (values are "
                                    "kept)",
                        )
                    # Rebuilt on Reload.
                    self._constants_frame = ui.Frame(
                        height=0, build_fn=self._build_constants
                    )

    def _rebuild(self):
        self._constants_frame.rebuild()

    def _disable_all(self):
        for entry in self._overrides.values():
            entry["enabled"] = False
        self._save()
        self._rebuild()

    def _save(self):
        try:
            self._tunable.save_overrides(OVERRIDES_FILE, self._overrides)
        except OSError as error:
            carb.log_error(f"Couldn't save overrides: {error}")

    def _build_constants(self):
        """List constants.h by category, with overrides, read fresh."""
        try:
            categories = constants_file.parse(CONSTANTS_HEADER)
        except OSError as error:
            ui.Label(
                f"Couldn't read constants.h: {error}",
                style={"color": ERROR}, word_wrap=True,
            )
            return
        try:
            self._tunable, bridge = _motion_modules()
        except Exception as error:  # noqa: BLE001 - shown in the window
            ui.Label(
                f"Couldn't load scripts/Motion: {error}",
                style={"color": ERROR}, word_wrap=True,
            )
            return
        self._overrides = self._tunable.load_overrides(OVERRIDES_FILE)

        with ui.VStack(spacing=4, height=0):
            self._build_build_check(categories, bridge)
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

    def _build_build_check(self, categories, bridge):
        """Say whether the built bridge matches constants.h."""
        try:
            built = bridge.default_params()
        except Exception as error:  # noqa: BLE001 - shown in the window
            ui.Label(
                f"MPC bridge unavailable: {error}",
                style={"color": ERROR}, word_wrap=True,
            )
            return
        stale = []
        for category in categories:
            for constant in category.constants:
                param = self._tunable.binding(constant.name).param
                if param and constant.is_number and abs(
                    built[param] - constant.value
                ) > 1e-9:
                    stale.append(
                        f"{constant.name} {constant.value:g} (built "
                        f"{built[param]:g})"
                    )
        if stale:
            ui.Label(
                "constants.h changed since the bridge was built; run "
                "scripts\\build_native.bat. Differs: " + ", ".join(stale),
                style={"color": ERROR}, word_wrap=True,
            )
        else:
            ui.Label(
                "Bridge build matches constants.h.",
                style={"color": GOOD}, height=18,
            )

    def _build_group(self, group):
        if group.name:
            ui.Spacer(height=2)
            ui.Label(
                group.name, word_wrap=True,
                style={"color": MUTED, "font_size": 13},
            )
        for constant in group.constants:
            binding = self._tunable.binding(constant.name)
            if binding.kind == self._tunable.TUNABLE and constant.is_number:
                self._tunable_row(constant)
            else:
                self._locked_row(constant, binding)

    def _tunable_row(self, constant):
        """Toggle, override value and constants.h's value."""
        entry = self._overrides.get(constant.name)
        enabled = ui.SimpleBoolModel(bool(entry and entry["enabled"]))
        value = ui.SimpleFloatModel(
            entry["value"] if entry else float(constant.value)
        )
        name = constant.name
        proxy = weakref.proxy(self)

        def changed(*_):
            proxy._set_override(name, enabled.as_bool, value.as_float)

        enabled.add_value_changed_fn(changed)
        value.add_value_changed_fn(changed)
        with ui.HStack(height=20, spacing=8):
            ui.CheckBox(
                model=enabled, width=18,
                tooltip="Override constants.h's value for sim runs",
            )
            ui.Label(
                constant.name, width=222,
                style={"color": OVERRIDDEN} if enabled.as_bool else {},
                tooltip=f"{constant.ctype} {constant.name}, "
                        f"line {constant.line}",
            )
            ui.FloatField(model=value, width=80, precision=4)
            ui.Label(
                f"file {_format_value(constant)}", width=90,
                elided_text=True, style={"color": MUTED},
                tooltip=f"constants.h: {constant.raw}",
            )
            ui.Label(
                constant.comment, elided_text=True,
                style={"color": MUTED}, tooltip=constant.comment,
            )

    def _set_override(self, name, enabled, value):
        self._overrides[name] = {"value": value, "enabled": enabled}
        self._save()

    def _locked_row(self, constant, binding):
        """Name, value and why it can't be overridden."""
        reason = binding.kind + (f": {binding.note}" if binding.note else "")
        with ui.HStack(height=20, spacing=8):
            ui.Spacer(width=18)
            ui.Label(
                constant.name, width=222,
                tooltip=f"{constant.ctype} {constant.name}, "
                        f"line {constant.line}",
            )
            ui.Label(
                _format_value(constant), width=80,
                elided_text=True, tooltip=constant.raw,
            )
            ui.Label(
                binding.kind, width=90,
                style={"color": KIND_COLORS.get(binding.kind, MUTED)},
                tooltip=reason,
            )
            ui.Label(
                constant.comment, elided_text=True,
                style={"color": MUTED}, tooltip=constant.comment,
            )

    def _path_row(self, label, path, missing=None):
        """A file the suite uses. missing: text to show instead of an error
        when the file not existing yet is fine.
        """
        found = path.exists()
        if found:
            text, color = str(path), MUTED
        elif missing:
            text, color = missing, MUTED
        else:
            text, color = f"not found: {path}", ERROR
        with ui.HStack(height=20, spacing=6):
            ui.Label(label, width=90)
            ui.Label(
                text, style={"color": color}, elided_text=True,
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
