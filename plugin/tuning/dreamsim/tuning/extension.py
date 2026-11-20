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


def _add_motion_path():
    if str(MOTION_DIR) not in sys.path:
        sys.path.insert(0, str(MOTION_DIR))
    importlib.invalidate_caches()


def _motion_modules():
    """tunable_constants and mpc_bridge from scripts/Motion, freshly
    reloaded so edits there apply.
    """
    _add_motion_path()
    import mpc_bridge
    import tunable_constants

    return importlib.reload(tunable_constants), importlib.reload(mpc_bridge)


def _controller():
    """The drive controller loaded in the Script Editor, or None."""
    _add_motion_path()
    # Never reloaded: it holds the registration.
    import controller_registry

    return controller_registry.get()


class TuningExtension(omni.ext.IExt):
    """Adds the MPC Tuning window and its menu entry."""

    def on_startup(self, ext_id):
        self._ext_id = ext_id
        self._window = None
        self._overrides = {}
        self._trial_running = False
        self._last_trial = None
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
                    with ui.CollapsableFrame("Trial", height=0):
                        self._build_trial()
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

    def _build_trial(self):
        """Run one autonomous run with the current overrides and show how
        it went.
        """
        self._trial_path = ui.SimpleStringModel("")
        self._trial_timeout = ui.SimpleFloatModel(0.0)
        with ui.VStack(spacing=4, height=0):
            self._controller_label = ui.Label("", word_wrap=True)
            with ui.HStack(height=20, spacing=6):
                ui.Label("Path file", width=90)
                ui.StringField(
                    model=self._trial_path,
                    tooltip="JerryIO file to run; blank uses "
                            "drivetrain_config.MPC_PATH_FILE",
                )
            with ui.HStack(height=20, spacing=6):
                ui.Label("Timeout (s)", width=90)
                ui.FloatField(
                    model=self._trial_timeout, width=80, precision=1,
                    tooltip="Per path; 0 uses "
                            "drivetrain_config.MPC_PATH_TIMEOUT",
                )
            with ui.HStack(height=24, spacing=6):
                ui.Button(
                    "Run trial", width=100, clicked_fn=self._run_trial,
                    tooltip="Run autonomous once with the overrides below",
                )
                ui.Button(
                    "Stop", width=80, clicked_fn=self._stop_trial,
                    tooltip="Stop the running trial",
                )
                ui.Button(
                    "Check", width=80,
                    clicked_fn=self._update_controller_label,
                    tooltip="Look for the drive controller again",
                )
            self._trial_label = ui.Label(
                "No trial run yet.", word_wrap=True, style={"color": MUTED},
            )
        self._update_controller_label()

    def _update_controller_label(self):
        controller = _controller()
        if controller is None:
            self._controller_label.text = (
                "Drive controller not loaded: run "
                "scripts/Motion/drive_controller.py in the Script Editor."
            )
            self._controller_label.style = {"color": ERROR}
            return controller
        robots = len(controller.prims)
        self._controller_label.text = (
            f"Drive controller loaded ({robots} robot"
            f"{'' if robots == 1 else 's'})."
        )
        self._controller_label.style = {"color": GOOD}
        return controller

    def _run_trial(self):
        controller = self._update_controller_label()
        if controller is None:
            return
        path = self._trial_path.as_string.strip() or None
        if path is not None and not Path(path).is_absolute():
            path = str(REPO_ROOT / path)
        timeout = self._trial_timeout.as_float or None
        proxy = weakref.proxy(self)
        started = controller.run_trial(
            lambda result: proxy._trial_done(result),
            path_file=path, timeout=timeout,
        )
        if not started:
            self._show_trial("Press Play first: the simulation isn't "
                             "running.", ERROR)
            return
        self._trial_running = True
        self._show_trial("Running...", MUTED)

    def _stop_trial(self):
        controller = _controller()
        if controller is not None and self._trial_running:
            controller.discard_pending_autonomous(
                "stopped from the tuning window"
            )
            controller.cancel_autonomous("stopped from the tuning window")

    def _trial_done(self, result):
        self._trial_running = False
        self._last_trial = result
        lines = [result.summary()]
        for path in result.results:
            lines.append("  " + path.summary())
        try:
            defaults = _motion_modules()[1].default_params()
            changed = [
                f"{name} {defaults[name]:g} -> {value:g}"
                for name, value in result.params.items()
                if abs(value - defaults[name]) > 1e-12
            ]
        except Exception:  # noqa: BLE001 - informational only
            changed = []
        lines.append(
            "Ran with: " + (", ".join(changed) if changed
                            else "robot-core's values")
        )
        self._show_trial(
            "\n".join(lines), GOOD if result.finished else ERROR
        )

    def _show_trial(self, text, color):
        self._trial_label.text = text
        self._trial_label.style = {"color": color}

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
