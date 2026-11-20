"""Which constants.h values the sim can apply at runtime, and the overrides.

Every constant in robot-core's constants.h falls into one kind:

    TUNABLE       applied by the bridge at runtime; can be overridden
    PHYSICAL      applied at runtime, but describes the robot or field and
                  must match the sim's physics: locked
    COMPILE_TIME  sizes the MPC's matrices; needs a rebuild to change
    NOT_SIMULATED nothing in the sim reads it (ports, PIDs, odometry, ...)

REGISTRY lists the first three; anything else is NOT_SIMULATED. Adding a
constant to the tuning suite (e.g. PIDs once they're simulated) means adding
it here with the bridge parameter that applies it.

Overrides live in config.TUNING_OVERRIDES_FILE (written by the tuning
window), as {constant: {"value": v, "enabled": bool}}. runtime_overrides()
combines them with config.MPC_TUNING into the bridge parameters each run
uses. The file is read at the start of every run, so changes apply on the
next P press.

No Isaac Sim imports.
"""

import json
from dataclasses import dataclass
from pathlib import Path

TUNABLE = "tunable"
PHYSICAL = "physical"
COMPILE_TIME = "compile-time"
NOT_SIMULATED = "not simulated"


@dataclass(frozen=True)
class Binding:
    kind: str
    param: str = None  # mpc_bridge.PARAM_NAMES entry that applies it
    note: str = ""


REGISTRY = {
    # cost weights
    "MPC_P_X": Binding(TUNABLE, "p_x"),
    "MPC_P_Y": Binding(TUNABLE, "p_y"),
    "MPC_P_THETA": Binding(TUNABLE, "p_theta"),
    "MPC_Q_U": Binding(TUNABLE, "q_u"),
    "MPC_Q_ZERO_MULT": Binding(TUNABLE, "q_zero_mult"),
    "MPC_Q_FINAL_MULT": Binding(TUNABLE, "q_final_mult"),
    "MPC_P_FINAL_MULT": Binding(TUNABLE, "p_final_mult"),
    # slew limits
    "MPC_DELTA_U_MAX": Binding(TUNABLE, "delta_u_max"),
    "MPC_DELTA_U_MIN": Binding(TUNABLE, "delta_u_min"),
    # reference generation
    "MPC_A_MAX": Binding(TUNABLE, "a_max"),
    "MPC_A_LAT_MAX": Binding(TUNABLE, "a_lat_max"),
    "MPC_V_REF_MIN": Binding(TUNABLE, "v_ref_min"),
    # path follower
    "PATH_SEARCH_AHEAD": Binding(TUNABLE, "path_search_ahead"),
    "END_TOLERANCE": Binding(TUNABLE, "end_tolerance"),
    # the robot, field and hardware: shared with the sim or measured
    "MPC_MOTOR_A": Binding(
        PHYSICAL, "motor_a", "identified from the robot"
    ),
    "MPC_MOTOR_B": Binding(
        PHYSICAL, "motor_b", "identified from the robot"
    ),
    "MPC_V_MAX": Binding(PHYSICAL, "v_max", "battery voltage"),
    "MPC_X_FIELD_MAX": Binding(PHYSICAL, "x_field_max", "field size"),
    "MPC_Y_FIELD_MAX": Binding(PHYSICAL, "y_field_max", "field size"),
    "MPC_OMEGA_WHEEL_MAX": Binding(
        PHYSICAL, "omega_wheel_max", "motor limit"
    ),
    "MPC_SAMPLE_PERIOD": Binding(
        PHYSICAL, note="the Nano's control rate; the sim steps with it"
    ),
    "MPC_COMMAND_DELAY": Binding(PHYSICAL, note="measured link latency"),
    "DRIVE_WHEEL_DIAMETER_INCHES": Binding(
        PHYSICAL, note="read by the sim's physics"
    ),
    "DRIVE_GEAR_RATIO": Binding(PHYSICAL, note="read by the sim's physics"),
    "TRACK_WIDTH": Binding(PHYSICAL, note="read by the sim's physics"),
    # matrix sizes
    "MPC_PREDICTION_HORIZON": Binding(
        COMPILE_TIME, note="sizes the MPC's matrices"
    ),
    "MPC_CONTROL_HORIZON": Binding(
        COMPILE_TIME, note="sizes the MPC's matrices"
    ),
}


def binding(name):
    """The Binding for a constants.h name; NOT_SIMULATED if unlisted."""
    return REGISTRY.get(name, Binding(NOT_SIMULATED))


def tunable_names():
    return [name for name, b in REGISTRY.items() if b.kind == TUNABLE]


def load_overrides(path):
    """Read the overrides file: {constant: {"value", "enabled"}}. A missing
    or unreadable file is no overrides.
    """
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    overrides = {}
    for name, entry in data.items():
        if not isinstance(entry, dict) or "value" not in entry:
            continue
        try:
            value = float(entry["value"])
        except (TypeError, ValueError):
            continue
        overrides[name] = {
            "value": value, "enabled": bool(entry.get("enabled", False))
        }
    return overrides


def save_overrides(path, overrides):
    """Write the overrides file, sorted by name."""
    Path(path).write_text(
        json.dumps(dict(sorted(overrides.items())), indent=2) + "\n",
        encoding="utf-8",
    )


def enabled_overrides(overrides):
    """The enabled, tunable overrides as {constant: value}. Entries for
    constants that aren't tunable are skipped.
    """
    return {
        name: entry["value"]
        for name, entry in overrides.items()
        if entry["enabled"] and binding(name).kind == TUNABLE
    }


def to_params(constants):
    """{constant: value} to {bridge parameter: value}."""
    return {binding(name).param: value for name, value in constants.items()}


def runtime_overrides():
    """The bridge parameters a run should override: config.MPC_TUNING, then
    the enabled overrides from config.TUNING_OVERRIDES_FILE on top.
    """
    import drivetrain_config as config

    params = dict(config.MPC_TUNING)
    params.update(
        to_params(enabled_overrides(
            load_overrides(config.TUNING_OVERRIDES_FILE)
        ))
    )
    return params
